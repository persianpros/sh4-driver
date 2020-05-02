/****************************************************************************************
 *
 * adb_5800_fp_file.c
 *
 * (c) 20?? Freebox
 * (c) 2019 Audioniek
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *
 * Front panel driver for ADB ITI-5800S(X), BSKA, BSLA, BXZB and BZZB models;
 * Button, LEDs, LED display and VFD driver, file part
 *
 * Devices:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 20?????? freebox         Initial version       
 * 20190730 Audioniek       Beginning of rewrite.
 * 20190803 Audioniek       Add PT6302-001 UTF-8 support.
 * 20190804 Audioniek       Add VFDSETLED and VFDLEDBRIGHTNESS.
 * 20190804 Audioniek       Add IOCTL compatibility mode (handy for fp_control).
 * 20190805 Audioniek       procfs added.
 * 20190806 Audioniek       Scrolling on /dev/vfd added.
 * 20190807 Audioniek       Procfs adb_variant added; replaces boxtype.
 * 20190807 Audioniek       Automatic selection of key lay out & display type
 *                          when no module parameters are specified -> one driver for
 *                          all variants.
 * 20190809 Audioniek       Split in nuvoton style between main, file and procfs parts.
 * 20190810 Audioniek       Write PT6302 DCRAM and write VFD text split up as separate
 *                          routines; needed for icon display.
 * 20190812 Audioniek       Icon support on VFD added. Icons are displayed in character
 *                          position 16 (rightmost). If one icon is on, the display is
 *                          steady, not involving any extra processing until the icon is
 *                          switched off. If more than one icon is on, they are displayed
 *                          in a thread, displaying up to eight icons in circular fashion.
 *                          The maximum of eight is a compromise between a limitation of
 *                          the PT6302 display IC and code complexity. The thread is
 *                          stopped as soon as one icon is on again for performance
 *                          reasons.
 *                          As a consequence, text display is now limited to 15
 *                          characters maximum.
 * 20190813 Audioniek       Spinner added.
 * 20190813 Audioniek       VFDSETFAN IOCTL added.
 * 20190814 Audioniek       Display all icons on/off added.
 * 20190902 Audioniek       Text display through /dev/vfd handled in a thread, including
 *                          scrolling.
 * 20190903 Audioniek       Separate text display threads for VFD and LED.
 * 20190904 Audioniek       Stop icon changed, VFD issues fixed.
 * 20191028 Audioniek       Fixed: texts displayed through VFDDISPLAYCHARS were backwards
 *                          on VFD.
 * 20191028 Audioniek       Icon display in mode 0 did not work.
 * 20191028 Audioniek       VFDSETLED in mode 0 added.
 * 20191103 Audioniek       Debug icon handling and reverse text in VFDSETLED in
 *                          VFDDISPLAYCHARS.
 * 20200425 Audioniek       Fix PT6302 UTF8 support.
 *
 ****************************************************************************************/
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/input.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/stm/pio.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/i2c.h>
#include <linux/kthread.h>

#include "pt6958_utf.h"
#include "pt6302_utf.h"
#include "adb_5800_fp.h"

// Global variables
int box_variant = 0;     // 1=BSKA, 2=BSLA, 3=BXZB, 4=BZZB, 5=cable, 0=unknown
int display_type;        // active display: 0=LED, 1=VFD, 2=both
int led_brightness = 5;  // default LED brightness
// LED states
char led1 = 0;  // power LED
char led2 = 0;  // timer / record
char led3 = 0;  // at
char led4 = 0;  // alert

spinlock_t mr_lock = SPIN_LOCK_UNLOCKED;

static struct fp_driver fp;

struct saved_data_s lastdata;

struct stpio_pin *pt6xxx_din;  // note: PT6302 & PT6958 Din and PT6958 Dout pins are parallel connected
struct stpio_pin *pt6xxx_clk;  // note: PT6302 & PT6958 CLK pins are parallel connected
struct stpio_pin *pt6958_stb;  // note: PT6958 only, acts as chip select
struct stpio_pin *pt6302_cs;   // note: PT6302 only

typedef union
{
	struct
	{
		uint8_t addr: 4, cmd: 4;  // bits 0-3 = addr, bits 4-7 = cmd
	} dcram;  // PT6302_COMMAND_DCRAM_WRITE command

	struct
	{
		uint8_t addr: 3, reserved: 1, cmd: 4;  // bits 0-2 = addr, bit 3 = *, bits 4-7 = cmd
	} cgram;  // PT6302_COMMAND_CGRAM_WRITE command

	struct
	{
		uint8_t addr: 4, cmd: 4;  // bits 0-3 = addr, bits 4-7 = cmd
	} adram;  // PT6302_COMMAND_ADRAM_WRITE command

	struct
	{
		uint8_t port1: 1, port2: 1, reserved: 2, cmd: 4;  // bits 0 = port 1, bit 1 = port 2, bit 2-3 = *, bits 4-7 = cmd
	} port;  // PT6302_COMMAND_SET_PORTS command

	struct
	{
		uint8_t duty: 3, reserved: 1, cmd: 4;
	} duty;  // PT6302_COMMAND_SET_DUTY command

	struct
	{
		uint8_t digits: 3, reserved: 1, cmd: 4;
	} digits;  // PT6302_COMMAND_SET_DIGITS command

	struct
	{
		uint8_t onoff: 2, reserved: 2, cmd: 4;
	} light;  // PT6302_COMMAND_SET_LIGHT command

	uint8_t all;
} pt6302_command_t;

struct semaphore led_text_thread_sem;
struct semaphore vfd_text_thread_sem;
struct task_struct *led_text_task = 0;
struct task_struct *vfd_text_task = 0;
int led_text_thread_status = THREAD_STATUS_STOPPED;
int vfd_text_thread_status = THREAD_STATUS_STOPPED;
static struct vfd_ioctl_data last_led_draw_data;
static struct vfd_ioctl_data last_vfd_draw_data;

extern int display_led;  // module parameter
extern int display_vfd;  // module parameter
extern int rec_key;      // module parameter: front panel keyboard layout
extern int key_layout;   // active front panel key layout: 0=MENU/EPG/RES, 1=EPG/REC/RES
extern unsigned char key_group1, key_group2;
extern unsigned long fan_registers;
extern struct stpio_pin* fan_pin;
extern tIconState spinner_state;
extern tIconState icon_state;

/***************************************************************************
 *
 * Character & icon definitions.
 *
 ***************************************************************************/

/*********************************************
 *
 * Character table for PT6958 7 segment
 * LED display
 *
 * (Note the PT6302 VFD driver has a
 * character generator built in, so there is
 * no corresponding table for the VFD).
 *
 * ----a----
 * |       |
 * f       b
 * |       |
 * ----g----
 * |       |
 * e       c
 * |       |
 * ----d---- h
 *
 * segment a is bit 0 -> 00000001 (0x01)
 * segment b is bit 0 -> 00000010 (0x02)
 * segment c is bit 0 -> 00000100 (0x04)
 * segment d is bit 0 -> 00001000 (0x08)
 * segment e is bit 0 -> 00010000 (0x10)
 * segment f is bit 0 -> 00100000 (0x20)
 * segment g is bit 0 -> 01000000 (0x40)
 * segment h is bit 7 -> 10000000 (0x80)
 */
static const unsigned char seven_seg[128] =
{
	0x00,  //0x00,
	0x00,  //0x01,
	0x00,  //0x02,
	0x00,  //0x03,
	0x00,  //0x04,
	0x00,  //0x05,
	0x00,  //0x06,
	0x00,  //0x07,
	0x00,  //0x08,
	0x00,  //0x09,
	0x00,  //0x0a,
	0x00,  //0x0b,
	0x00,  //0x0c,
	0x00,  //0x0d,
	0x00,  //0x0e,
	0x00,  //0x0f,

	0x00,  //0x10,
	0x00,  //0x11,
	0x00,  //0x12,
	0x00,  //0x13,
	0x00,  //0x14,
	0x00,  //0x15,
	0x00,  //0x16,
	0x00,  //0x17,
	0x00,  //0x18,
	0x00,  //0x19,
	0x00,  //0x1a,
	0x00,  //0x1b,
	0x00,  //0x1c,
	0x00,  //0x1d,
	0x00,  //0x1e,
	0x00,  //0x1f,

	0x00,  //0x20, <space>
	0x86,  //0x21, !
	0x22,  //0x22, "
	0x76,  //0x23, #
	0x6d,  //0x24, $
	0x52,  //0x25, %
	0x7d,  //0x26, &
	0x02,  //0x27, '
	0x39,  //0x28, (
	0x0f,  //0x29, )
	0x76,  //0x2a, *
	0x46,  //0x2b, +
	0x84,  //0x2c, ,
	0x40,  //0x2d, -
	0x80,  //0x2e, .
	0x52,  //0x2f, /

	0x3f,  //0x30, 0
	0x06,  //0x31, 1
	0x5b,  //0x32, 2
	0x4f,  //0x33, 3
	0x66,  //0x34, 4
	0x6d,  //0x35, 5
	0x7d,  //0x36, 6
	0x07,  //0x37, 7
	0x7f,  //0x38, 8
	0x6f,  //0x39, 9
	0x06,  //0x3a, :
	0x86,  //0x3b, ;
	0x58,  //0x3c, <
	0x48,  //0x3d, =
	0x4c,  //0x3e, >
	0xd3,  //0x3f, ?

	0x7b,  //0x40, @
	0x77,  //0x41, A
	0x7c,  //0x42, B
	0x39,  //0x43, C
	0x5e,  //0x44, D
	0x79,  //0x45, E
	0x71,  //0x46, F
	0x3d,  //0x47, G
	0x76,  //0x48, H
	0x06,  //0x49, I
	0x0e,  //0x4a, J
	0x74,  //0x4b, K
	0x38,  //0x4c, L
	0x37,  //0x4d, M
	0x37,  //0x4e, N
	0x3f,  //0x4f, O

	0x73,  //0x50, P
	0xbf,  //0x51, Q
	0x50,  //0x52, R
	0x6d,  //0x53, S
	0x78,  //0x54, T
	0x3e,  //0x55, U
	0x3e,  //0x56, V
	0x3e,  //0x57, W
	0x76,  //0x58, X
	0x6e,  //0x59, Y
	0x5b,  //0x5a, Z
	0x39,  //0x5b  [
	0x30,  //0x5c, |
	0x0f,  //0x5d, ]
	0x23,  //0x5e, ^
	0x08,  //0x5f, _

	0x20,  //0x60, `
	0x5f,  //0x61, a
	0x7c,  //0x62, b
	0x58,  //0x63, c
	0x5e,  //0x64, d
	0x7b,  //0x65, e
	0x71,  //0x66, f
	0x6f,  //0x67, g
	0x74,  //0x68, h
	0x04,  //0x69, i
	0x0e,  //0x6a, j
	0x74,  //0x6b, k
	0x06,  //0x6c, l
	0x54,  //0x6d, m
	0x54,  //0x6e, n
	0x5c,  //0x6f, o

	0x73,  //0x70, p
	0x67,  //0x71, q
	0x50,  //0x72, r
	0x6d,  //0x73, s
	0x78,  //0x74, t
	0x1c,  //0x75, u
	0x1c,  //0x76, v
	0x1c,  //0x77, w
	0x76,  //0x78, x
	0x6e,  //0x79, y
	0x5b,  //0x7a, z
	0x39,  //0x7b, {
	0x64,  //0x7c, backslash
	0x0f,  //0x7d, }
	0x40,  //0x7e, ~
	0x7f,  //0x7f, <DEL>--> all segments on

//	0x80 onwards not used; taken care of by UTF8 support
};

/***************************************************************
 *
 * Icons for PT6302 VFD
 *
 * These are displayed in the rightmost character position.
 * Basically only one icon can be displayed at a time; if more
 * are on they are displayed successively in circular fashion
 * in a thread.
 *
 * A character on the VFD display is simply a pixel matrix
 * consisting of five columns of seven pixels high.
 * Column 1 is the leftmost.
 *
 * Format of pixel data:
 *
 * pixeldata1: column 1
 *  ...
 * pixeldata5: column 5
 *
 * Each bit in the pixel data represents one pixel in the column,
 * with the LSbit being the top pixel and bit 6 the bottom one.
 * The pixel will be on when the corresponding bit is one.
 * The resulting map is:
 *
 *  Register    byte0 byte1 byte2 byte3 byte4
 *              -----------------------------
 *  Bitmask     0x01  0x01  0x01  0x01  0x01
 *              0x02  0x02  0x02  0x02  0x02
 *              0x04  0x04  0x04  0x04  0x04
 *              0x08  0x08  0x08  0x08  0x08
 *              0x10  0x10  0x10  0x10  0x10
 *              0x20  0x20  0x20  0x20  0x20
 *              0x40  0x40  0x40  0x40  0x40
 *
 * To display an icon, its bit pattern is written in the first
 * free location in CGRAM. Then the corresponding character at
 * that position is set to the number of the bit pattern
 * position in CGRAM. The icon will then show on the character
 * position.
 * Switching an icon off can be achieved in two ways:
 * - rewriting its bit pattern in CGRAM to all zeroes;
 * - setting the character in DCRAM on the display position to
 *   a space.
 * The code in this driver employs both methods, to minimize
 * the chance of unwanted artifacts appearing in the display.
 *
 */
struct iconToInternal vfdIcons[] =
{
	/*- Name ---------- icon# ---------- data0 data1 data2 data3 data4-----*/
	{ "ICON_MIN",       ICON_MIN,       {0x00, 0x00, 0x00, 0x00, 0x00}},  // 00 (also used for clearing)
	{ "ICON_REC",       ICON_REC,       {0x1c, 0x3e, 0x3e, 0x3e, 0x1c}},  // 01
	{ "ICON_TIMESHIFT", ICON_TIMESHIFT, {0x01, 0x3f, 0x01, 0x2c, 0x34}},  // 02
	{ "ICON_TIMER",     ICON_TIMER,     {0x3e, 0x49, 0x4f, 0x41, 0x3e}},  // 03
	{ "ICON_HD",        ICON_HD,        {0x3e, 0x08, 0x3e, 0x22, 0x1c}},  // 04
	{ "ICON_USB",       ICON_USB,       {0x08, 0x0c, 0x1a, 0x2a, 0x28}},  // 05
	{ "ICON_SCRAMBLED", ICON_SCRAMBLED, {0x3c, 0x26, 0x25, 0x26, 0x3c}},  // 06
	{ "ICON_DOLBY",     ICON_DOLBY,     {0x3e, 0x22, 0x1c, 0x22, 0x3e}},  // 07
	{ "ICON_MUTE",      ICON_MUTE,      {0x1e, 0x12, 0x1e, 0x21, 0x3f}},  // 08
	{ "ICON_TUNER1",    ICON_TUNER1,    {0x01, 0x3f, 0x01, 0x04, 0x3e}},  // 09
	{ "ICON_TUNER2",    ICON_TUNER2,    {0x01, 0x3f, 0x01, 0x34, 0x2c}},  // 10
	{ "ICON_MP3",       ICON_MP3,       {0x77, 0x37, 0x00, 0x49, 0x77}},  // 11
	{ "ICON_REPEAT",    ICON_REPEAT,    {0x14, 0x34, 0x14, 0x16, 0x14}},  // 12
	{ "ICON_PLAY",      ICON_PLAY,      {0x00, 0x7f, 0x3e, 0x1c, 0x08}},  // 13
	{ "ICON_STOP",      ICON_STOP,      {0x3e, 0x3e, 0x3e, 0x3e, 0x3e}},  // 14
	{ "ICON_PAUSE",     ICON_PAUSE,     {0x3e, 0x3e, 0x00, 0x3e, 0x3e}},  // 15
	{ "ICON_REWIND",    ICON_REWIND,    {0x08, 0x1c, 0x08, 0x1c, 0x00}},  // 16
	{ "ICON_FF",        ICON_FF,        {0x00, 0x1c, 0x08, 0x1c, 0x08}},  // 17
	{ "ICON_STEP_BACK", ICON_STEP_BACK, {0x08, 0x1c, 0x3e, 0x00, 0x3e}},  // 18
	{ "ICON_STEP_FWD",  ICON_STEP_FWD,  {0x3e, 0x00, 0x3e, 0x1c, 0x08}},  // 19 
	{ "ICON_TV",        ICON_TV,        {0x01, 0x3f, 0x1d, 0x20, 0x1c}},  // 20
	{ "ICON_RADIO",     ICON_RADIO,     {0x78, 0x4a, 0x4c, 0x4a, 0x79}},  // 21
	{ "ICON_MAX",       ICON_MAX,       {0x00, 0x00, 0x00, 0x00, 0x00}},  // 22
	{ "ICON_SPINNER",   ICON_SPINNER,   {0x00, 0x00, 0x00, 0x00, 0x00}}   // 23
};
// End of character and icon definitions


/******************************************************
 *
 * Routines to communicate with the PT6958 and PT6302
 *
 */

/******************************************************
 *
 * Free the PT6958 and PT6302 PIO pins
 *
 */
void pt6xxx_free_pio_pins(void)
{
	dprintk(150, "%s >\n", __func__);
	if (pt6302_cs)
	{
		stpio_set_pin(pt6302_cs, 1);  // deselect PT6302
	}
	if (pt6958_stb)
	{
		stpio_set_pin(pt6958_stb, 1);  // deselect PT6958
	}
	if (pt6xxx_din)
	{
		stpio_free_pin(pt6xxx_din);
	}
	if (pt6xxx_clk)
	{
		stpio_free_pin(pt6xxx_clk);
	}
	if (pt6958_stb)
	{
		stpio_free_pin(pt6958_stb);
	}
	if (pt6302_cs)
	{
		stpio_free_pin(pt6302_cs);
	}
	dprintk(150, "%s < PT6958 & PT6302 PIO pins freed\n", __func__);
};

/******************************************************
 *
 * Initialize the PT6958 and PT6302 PIO pins
 *
 */
static unsigned char pt6xxx_init_pio_pins(void)
{
	dprintk(150, "Initialize PT6302 & PT6958 PIO pins\n");

// PT6302 Chip select
	dprintk(50, "Request STPIO %d,%d for PT6302 CS\n", PORT_CS, PIN_CS);
	pt6302_cs = stpio_request_pin(PORT_CS, PIN_CS, "PT6302_CS", STPIO_OUT);
	if (pt6302_cs == NULL)
	{
		dprintk(1, "Request for STPIO cs failed; abort\n");
		goto pt_init_fail;
	}
// PT6958 Strobe
	dprintk(50, "Request STPIO %d,%d for PT6958 STB\n", PORT_STB, PIN_STB);
	pt6958_stb = stpio_request_pin(PORT_STB, PIN_STB, "PT6958_STB", STPIO_OUT);
	if (pt6958_stb == NULL)
	{
		dprintk(1, "Request for STB STPIO failed; abort\n");
		goto pt_init_fail;
	}
// PT6302 & PT6958 Clock
	dprintk(50, "Request STPIO %d,%d for PT6302/PT6958 CLK\n", PORT_CLK, PIN_CLK);
	pt6xxx_clk = stpio_request_pin(PORT_CLK, PIN_CLK, "PT6958_CLK", STPIO_OUT);
	if (pt6xxx_clk == NULL)
	{
		dprintk(1, "Request for CLK STPIO failed; abort\n");
		goto pt_init_fail;
	}
// PT6302 & PT6958 Data in
	dprintk(50, "Request STPIO %d,%d for PT6302/PT6958 Din\n", PORT_DIN, PIN_DIN);
	pt6xxx_din = stpio_request_pin(PORT_DIN, PIN_DIN, "PT6958_DIN", STPIO_BIDIR);
	if (pt6xxx_din == NULL)
	{
		dprintk(1, "Request for DIN STPIO failed; abort\n");
		goto pt_init_fail;
	}
	stpio_set_pin(pt6302_cs, 1);  // set all involved PIO pins high
	stpio_set_pin(pt6958_stb, 1);
	stpio_set_pin(pt6xxx_clk, 1);
	stpio_set_pin(pt6xxx_din, 1);
	dprintk(150, "%s <\n", __func__);
	return 1;

pt_init_fail:
	pt6xxx_free_pio_pins();
	dprintk(1, "%s < Error initializing PT6302 & PT6958 PIO pins\n", __func__);
	return 0;
};

/****************************************************
 *
 * Routines for the PT6958 (LED display & buttons)
 *
 */

/****************************************************
 *
 * Read one byte from the PT6958 (button data)
 *
 */
static unsigned char pt6958_read_byte(void)
{
	unsigned char i;
	unsigned char data_in = 0;

	dprintk(150, "%s >\n", __func__);
	for (i = 0; i < 8; i++)  // 8 bits in a byte, LSB first
	{
		stpio_set_pin(pt6xxx_din, 1);  // data = 1 (key will pull down the pin if pressed)
		stpio_set_pin(pt6xxx_clk, 0);  // toggle
		udelay(LED_Delay);
		stpio_set_pin(pt6xxx_clk, 1);  // clock pin
		udelay(LED_Delay);
		data_in = (data_in >> 1) | (stpio_get_pin(pt6xxx_din) > 0 ? 0x80 : 0);
	}
	dprintk(150, "%s <\n", __func__);
	return data_in;
}

/****************************************************
 *
 * Write one byte to the PT6958 or PT6302
 *
 * Note: which of the two chips depend on the CS
 *       previously set.
 */
static void pt6xxx_send_byte(unsigned char Value)
{
	unsigned char i;

//	dprintk(150, "%s >\n", __func__);
	for (i = 0; i < 8; i++)  // 8 bits in a byte, LSB first
	{
		stpio_set_pin(pt6xxx_din, Value & 0x01);  // write bit
		stpio_set_pin(pt6xxx_clk, 0);  // toggle
		udelay(LED_Delay);
		stpio_set_pin(pt6xxx_clk, 1);  // clock pin
		udelay(VFD_Delay);
		Value >>= 1;  // get next bit
	}
//	dprintk(150, "%s <\n", __func__);
}

/****************************************************
 *
 * Write one byte to the PT6302
 *
 */
static void pt6302_send_byte(unsigned char Value)
{
	stpio_set_pin(pt6302_cs, 0);  // set chip select
	pt6xxx_send_byte(Value);
	stpio_set_pin(pt6302_cs, 1);  // deselect
	udelay(VFD_Delay);
}

/****************************************************
 *
 * Write one byte to the PT6958
 *
 */
static void pt6958_send_byte(unsigned char byte)
{
	stpio_set_pin(pt6958_stb, 0);  // select PT6958
	pt6xxx_send_byte(byte);  // send byte
	stpio_set_pin(pt6958_stb, 1);  // and deselect PT6958
	udelay(LED_Delay);
}

/****************************************************
 *
 * Write len bytes to the PT6958
 *
 */
static void pt6958_WriteData(unsigned char *data, unsigned int len)
{
	unsigned char i;

//	dprintk(150, "%s >\n", __func__);
	stpio_set_pin(pt6958_stb, 0);  // select PT6958
	for (i = 0; i < len; i++)
	{
		pt6xxx_send_byte(data[i]);
	}
	stpio_set_pin(pt6958_stb, 1);  // and deselect PT6958
	udelay(LED_Delay);
//	dprintk(150, "%s <\n", __func__);
}

/****************************************************
 *
 * Write len bytes to the PT6302
 *
 */
static void pt6302_WriteData(unsigned char *data, unsigned int len)
{
	unsigned char i;

//	dprintk(150, "%s >\n", __func__);
	stpio_set_pin(pt6302_cs, 0);  // select PT6302

	for (i = 0; i < len; i++)
	{
		pt6xxx_send_byte(data[i]);
	}
	stpio_set_pin(pt6302_cs, 1);  // and deselect PT6302
	udelay(VFD_Delay);
//	dprintk(150, "%s <\n", __func__);
}


/******************************************************
 *
 * PT6958 LED display functions
 *
 */

/****************************************************
 *
 * Read front panel button data from the PT6958
 *
 */
void ReadKey(void)
{
	dprintk(150, "%s >\n", __func__);
	spin_lock(&mr_lock);

	stpio_set_pin(pt6958_stb, 0);  // set strobe low (selects PT6958)
	pt6xxx_send_byte(DATA_SETCMD + READ_KEYD);  // send command 01000010b -> Read key data
	key_group1 = pt6958_read_byte();  // Get SG1/KS1 (b0..3), SG2/KS2 (b4..7)
	key_group2 = pt6958_read_byte();  // Get SG3/KS3 (b0..3), SG4/KS4 (b4..7)
//	key_group3 = pt6958_read_byte();  // Get SG5/KS5, SG6/KS6  // not needed
	stpio_set_pin(pt6958_stb, 1);  // set strobe high
	udelay(LED_Delay);

	spin_unlock(&mr_lock);
	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Set text and dots on LED display
 *
 * Directly sets display segments and LEDs: DIG1 - DIG4
 * are segment data, not display characters.
 * 
 * Note: also updates the LEDs (led1 through led4)
 *
 */
static void PT6958_Show(unsigned char DIG1, unsigned char DIG2, unsigned char DIG3, unsigned char DIG4,
                        unsigned char DOT1, unsigned char DOT2, unsigned char DOT3, unsigned char DOT4)
{
	dprintk(150, "%s >\n", __func__);
	spin_lock(&mr_lock);

	pt6958_send_byte(DATA_SETCMD + 0);  // Set test mode off, increment address and write data display mode (CMD = 01xx0000b)
	udelay(LED_Delay);

	stpio_set_pin(pt6958_stb, 0);  // drop strobe (latches command)

	pt6xxx_send_byte(ADDR_SETCMD + 0);  // Command 2 address set, (start from 0)   11xx0000b
	// handle decimal point
	DIG1 += (DOT1 == 1 ? 0x80 : 0);  // add to digit data
	pt6xxx_send_byte(DIG1);
	pt6xxx_send_byte(led1);

	DIG2 += (DOT2 == 1 ? 0x80 : 0);  // handle decimal point
	pt6xxx_send_byte(DIG2);
	pt6xxx_send_byte(led2);

	DIG3 += (DOT3 == 1 ? 0x80 : 0);  // handle decimal point
	pt6xxx_send_byte(DIG3);
	pt6xxx_send_byte(led3);

	DIG4 += (DOT4 == 1 ? 0x80 : 0);  // handle decimal point
	pt6xxx_send_byte(DIG4);
	pt6xxx_send_byte(led4);

	stpio_set_pin(pt6958_stb, 1);
	udelay(LED_Delay);

	spin_unlock(&mr_lock);
	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Convert UTF8 formatted text to display format for
 * PT6958.
 *
 * Returns corrected length.
 *
 * Note: return format is led segment data, not plain
 *       text.
 *
 */
int pt6958_utf8conv(unsigned char *text, unsigned char len)
{
	int i;
	int wlen;
	unsigned char kbuf[8];
	unsigned char *UTF_Char_Table = NULL;

	dprintk(150, "%s > len = %d\n", __func__, len);

	i = 0;  // input index
	wlen = 0;  // output index
	memset(kbuf, 0x00, sizeof(kbuf));  // clear output buffer
	text[len] = 0x00;  // terminate text

	while ((i < len) && (wlen < 8))
	{
		if (text[i] == 0)
		{
			break;  // stop processing
		}
		else if (text[i] < 0x20)
		{
			i++;  // skip
		}
		else if (text[i] < 0x80)
		{
			kbuf[wlen] = seven_seg[text[i]];  // get segment pattern
			wlen++;
		}
		else if (text[i] < 0xe0)
		{
			switch (text[i])
			{
				case 0xc2:
				{
					if (text[i + 1] < 0xa0)  // if character is not printable
					{
						i++;  // skip character
					}
					else
					{
						UTF_Char_Table = PT6958_UTF_C2;  // select table
					}
					break;
				}
				case 0xc3:
				{
					UTF_Char_Table = PT6958_UTF_C3;
					break;
				}
				case 0xc4:
				{
					UTF_Char_Table = PT6958_UTF_C4;
					break;
				}
				case 0xc5:
				{
					UTF_Char_Table = PT6958_UTF_C5;
					break;
				}
				case 0xd0:
				{
					UTF_Char_Table = PT6958_UTF_D0;  // cyrillic, currently not supported
					break;
				}
				case 0xd1:
				{
					UTF_Char_Table = PT6958_UTF_D1;  // cyrillic, currently not supported
					break;
				}
				default:
				{
					UTF_Char_Table = NULL;
				}
			}
			i++;  // skip UTF lead in

			if (UTF_Char_Table)
			{
				kbuf[wlen] = UTF_Char_Table[text[i] & 0x3f];  // get segment data
				wlen++;
			}
		}
		else
		{
			if (text[i] < 0xF0)
			{
				i += 2;
			}
			else if (text[i] < 0xF8)
			{
				i += 3;
			}
			else if (text[i] < 0xFC)
			{
				i += 4;
			}
			else
			{
				i += 5;
			}
		}
		i++;
	}
	if (wlen > 64)
	{
		wlen = 64;
	}
	memset(text, 0x00, sizeof(kbuf) + 1);
	for (i = 0; i < wlen; i++)
	{
		text[i] = kbuf[i];
	}
	dprintk(150, "%s < wlen = %d\n", __func__, wlen);
	return wlen;
}

/******************************************************
 *
 * Set text string on LED display
 *
 */
int pt6958_ShowBuf(unsigned char *data, unsigned char len, int utf8_flag)
{
	unsigned char z1, z2, z3, z4, k1, k2, k3, k4;
	unsigned char text[9];
	unsigned char kbuf[8];
	int i;
	int wlen;

	dprintk(150, "%s >\n", __func__);
	len = (len > 8 ? 8 : len);

	if (len == 0)
	{
		return 0;
	}
	memset(text, 0x20, sizeof(text));
	memcpy(text, data, len);

	/* Strip possible trailing LF */
	if (text[len - 1] == '\n')
	{
		len--;
	}
	text[len] = 0x00;  // terminate string
	memset(kbuf, 0x00, sizeof(kbuf));  // clear segment data buffer

	/* handle UTF-8 characters */
	if (utf8_flag)
	{
		wlen = pt6958_utf8conv(text, len);
		wlen = (len > LED_DISP_SIZE ? LED_DISP_SIZE : len);
		memcpy(kbuf, text, wlen);
	}
	else
	{
		wlen = (len > LED_DISP_SIZE ? LED_DISP_SIZE : len);
		for (i = 0; i < wlen; i++)
		{
			kbuf[i] = seven_seg[text[i]];
		}
	}

	k1 = k2 = k3 = k4 = 0;  // clear decimal point flags

	/* save last string written to fp */
	memcpy(&lastdata.leddata, text, wlen);
	lastdata.length = wlen;

//#define ALIGN_RIGHT 1
#if defined ALIGN_RIGHT
	// right align display
	z1 = z2 = z3 = z4 = 0x00;  // set non used positions to blank
	k1 = k2 = k3 = k4 = 0x00;  // clear decimal point flags
	switch (len)
	{
		case 1:
		{
			z4 = kbuf[0];
			break;
		}
		case 2:
		{
			z3 = kbuf[0];
			z4 = kbuf[1];
			break;
		}
		case 3:
		{
			z2 = kbuf[0];
			z3 = kbuf[1];
			z4 = kbuf[2];
			break;
		}
		case 4:
		{
			z1 = kbuf[0];
			z2 = kbuf[1];
			z3 = kbuf[2];
			z4 = kbuf[3];
		}
	}
	PT6958_Show(z1, z2, z3, z4, k1, k2, k3, k4); // display text & decimal points
#else
	k1 = k2 = k3 = k4 = 0x00;  // clear decimal point flags
//	z1 = z2 = z3 = z4 = 0x00;  // set non used positions to space (blank)
// determine the decimal points: if character is period, single quote or colon, replace it by a decimal point
//	if ((wlen >= 2) && (text[1] == '.' || text[1] == 0x27 || text[1] == ':'))
//	{
//		k1 = 1;  // flag period at position 1
//	}
//	if ((wlen >= 3) && (text[2] == '.' || text[2] == 0x27 || text[2] == ':'))
//	{
//		k2 = 1;  // flag period at position 2
//	}
//	if ((wlen >= 4) && (text[3] == '.' || text[3] == 0x27 || text[3] == ':'))
//	{
//		k3 = 1;  // flag period at position 3
//	}
//	if ((wlen >= 5) && (text[4] == '.' || text[4] == 0x27 || text[4] == ':'))
//	{
//		k4 = 1;  // flag period at position 4
//	}
// assigning segment data
//	if (wlen >= 1)
//	{
//		z1 = seven_seg[kbuf[0]];
//	}
//	if (wlen >= 2)
//	{
//		z2 = seven_seg[kbuf[1 + k1]];
//	}
//	if (wlen >= 3)
//	{
//		z3 = seven_seg[kbuf[2 + k1 + k2]];
//	}
//	if (wlen >= 4)
//	{
//		z4 = seven_seg[kbuf[3 + k1 + k2 + k3]];
//	}
//	PT6958_Show(z1, z2, z3, z4, k1, k2, k3, k4); // display text & decimal points
	PT6958_Show(kbuf[0], kbuf[1], kbuf[2], kbuf[3], k1, k2, k3, k4); // display text & decimal points
#endif
	dprintk(150, "%s <\n", __func__);
	return 0;
}

/******************************************************
 *
 * Set LEDs (deprecated)
 *
 */
static void pt6958_set_icon(unsigned char *kbuf, unsigned char len)
{
	unsigned char pos = 0, on = 0;

//	dprintk(150, "%s >\n", __func__);
	spin_lock(&mr_lock);

	if (len == 5)
	{
		pos = kbuf[0];  // 1st character is position 1 = power LED, 
		on  = kbuf[4];  // last character is state
		on  = (on == 0 ? 0 : 1);  // normalize on value

		switch (pos)
		{
			case 1:  // led power, red
			{
				led1 = on;
//				pos = 1;
				break;
			}
			case 2:  // led power, green
			{
				if (on == 1)
				{
					led1 = 2;  // select green
					on = 2;
				}
				else
				{
					led1 = 0;
//					on = 0;
				}
				pos = 1;
				break;
			}
			case 3:  // timer LED
			{
				led2 = on;
//				pos = 3;
				break;
			}
			case 4:  // @ LED
			{
				led3 = on;
				pos = 5;
				break;
			}
			case 5:  // alert LED
			{
				led4 = on;  // save state
				pos = 7;  // get address command offset
				break;
			}
		}
		pt6958_send_byte(DATA_SETCMD + ADDR_FIX);  // Set command, normal mode, fixed address, write date to display mode 01xx0100b
		udelay(LED_Delay);

		stpio_set_pin(pt6958_stb, 0);  // select PT6958
		pt6xxx_send_byte(ADDR_SETCMD + pos);  // set position (11xx????b)
		pt6xxx_send_byte(on);  // set state (on or off)
		stpio_set_pin(pt6958_stb, 1);  // and deselect PT6958
		udelay(LED_Delay);
	}
	spin_unlock(&mr_lock);
//	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Set LED
 *
 */
void pt6958_set_led(int led_nr, int level)
{
	dprintk(150, "%s >\n", __func__);

	switch (led_nr)
	{
		case 1:  // power LED, accepts values 0 (off), 1 (red), 2 (green) and 3 (orange)
		{
				if (level < 0 || level > 3)
				{
					dprintk(1, "Illegal LED state value; must be 0..3, default to off\n");
					level = 0;
				}
				led1 = level;
				break;
		}
		case 2:  // timer/rec LED
		case 3:  // at LED
		case 4:  // alert LED
		{
			level = (level == 0 ? 0 : 1);
			switch (led_nr)  // update icon states
			{
				case 2:  // timer/rec LED
				{
					led2 = level;
					break;
				}
				case 3:  // at LED
				{
					led3 = level;
					break;
				}
				case 4:  // alert LED
				{
					led4 = level;
					break;
				}
			}
			break;
		}
	}
	udelay(LED_Delay);
	stpio_set_pin(pt6958_stb, 0);  // select PT6958
	pt6xxx_send_byte(ADDR_SETCMD + (led_nr * 2) - 1);  // set position (command 11xx????b)
	pt6xxx_send_byte(level);  // set state
	stpio_set_pin(pt6958_stb, 1);  // and deselect PT6958
	udelay(LED_Delay);
		
	dprintk(150, "%s <\n", __func__);
}

/****************************************************
 *
 * Initialize the PT6958
 *
 */
static void pt6958_setup(void)
{
	unsigned char i;

//	dprintk(150, "%s >\n", __func__);
	pt6958_send_byte(DATA_SETCMD);  // Command 1, increment address, normal mode
	udelay(LED_Delay);

	stpio_set_pin(pt6958_stb, 0);  // select PT6958
	pt6xxx_send_byte(ADDR_SETCMD);  // Command 2, RAM address = 0
	for (i = 0; i < 10; i++)  // 10 bytes
	{
		pt6xxx_send_byte(0);  // clear display RAM (all segments off)
	}
	stpio_set_pin(pt6958_stb, 1);  // and deselect PT6958
	udelay(LED_Delay);
//	dprintk(10, "Switch LED display on\n");
	pt6958_send_byte(DISP_CTLCMD + DISPLAY_ON + led_brightness);  // Command 3, display control, (Display ON), brightness: 10xx1BBBb

	led1 = 2;  // power LED green
	led2 = 0;  // timer/rec LED off
	led3 = 0;  // @ LED off
	led4 = 0;  // alert LED off
	PT6958_Show(0x40, 0x40, 0x40, 0x40, 0, 0, 0, 0);  // display ----, periods off, power LED green
//	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Set brightness of LED display and LEDs
 *
 */
void pt6958_set_brightness(int level)
{
	dprintk(150, "%s >\n", __func__);
	spin_lock(&mr_lock);

	if (level < 0)
	{
		level = 0;
	}
	if (level > 7)
	{
		level = 7;
	}
	pt6958_send_byte(DISP_CTLCMD + DISPLAY_ON + level);  // Command 3, display control, Display ON, brightness level; 10xx1???b
	led_brightness = level;
	spin_unlock(&mr_lock);
	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Switch LED display on or off
 *
 */
static void pt6958_set_light(int onoff)
{
	dprintk(150, "%s >\n", __func__);
	spin_lock(&mr_lock);

//	dprintk(10, "Switch LED display %s\n", onoff == 0 ? "off" : "on");
	pt6958_send_byte(DISP_CTLCMD + (onoff ? DISPLAY_ON + led_brightness : 0));
	spin_unlock(&mr_lock);
	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * PT6302 VFD functions
 *
 */

/******************************************************
 *
 * Write PT6302 DCRAM (display text)
 *
 * The rightmost character on the VFD display is
 * digit position or PT6302 character address zero.
 *
 * addr is used as the direct PT6302 character address;
 * data[0] is written at address addr.
 *
 */
int pt6302_write_dcram(unsigned char addr, unsigned char *data, unsigned char len)
{
	int              i;
	uint8_t          wdata[17];
	pt6302_command_t cmd;

	dprintk(150, "%s >\n", __func__);
	if (len == 0)  // if length is zero,
	{
//		dprintk(1, "%s < (len =  0)\n", __func__);
		return 0; // do nothing
	}
	if (addr > 15)
	{
		dprintk(1, "%s Error: addr too large (max. 15)\n", __func__);
		return 0;  // do nothing
	}
	if (len + addr > VFD_DISP_SIZE + ICON_WIDTH)
	{
		dprintk(1, "%s len (%d) too large, adjusting to %d\n", len, VFD_DISP_SIZE + ICON_WIDTH - addr);
		len = VFD_DISP_SIZE + ICON_WIDTH - addr;
	}
	spin_lock(&mr_lock);

	// assemble command
	cmd.dcram.cmd  = PT6302_COMMAND_DCRAM_WRITE;
	cmd.dcram.addr = (addr & 0x0f);  // get address (= character position, 0 = rightmost!)

	wdata[0] = cmd.all;
//	dprintk(1, "wdata[0] = 0x%02x\n", wdata[0]);

	for (i = 0; i < len; i++)
	{
		wdata[i + 1] = data[i];
	}
	pt6302_WriteData(wdata, 1 + len);

	spin_unlock(&mr_lock);
//	dprintk(150, "%s <\n", __func__);
	return 0;
}

/****************************************************
 *
 * Write PT6302 ADRAM (set underline data)
 *
 * In the display, AD2 (bit 1) is connected to the
 * underline segments.
 * addr = starting digit number (0 = rightmost)
 *
 * NOTE: Display tube does have underline segments,
 * but they do not seem to be connected to
 * the PT6302, although the PCB has a trace.
 *
 */
int pt6302_write_adram(unsigned char addr, unsigned char *data, unsigned char len)
{
	pt6302_command_t cmd;
	uint8_t          wdata[20] = {0x0};
	int              i = 0;

//	dprintk(150, "%s >\n", __func__);

	spin_lock(&mr_lock);

	cmd.adram.cmd  = PT6302_COMMAND_ADRAM_WRITE;
	cmd.adram.addr = (addr & 0xf);

	wdata[0] = cmd.all;
//	dprintk(1, "wdata[0] = 0x%02x\n", wdata[0]);
	if (len > VFD_DISP_SIZE - addr)
	{
		len = VFD_DISP_SIZE - addr;
	}
	for (i = 0; i < len; i++)
	{
		wdata[i + 1] = data[i] & 0x03;
	}
	pt6302_WriteData(wdata, len + 1);
	spin_unlock(&mr_lock);
//	dprintk(150, "%s <\n", __func__);
	return 0;
}

/****************************************************
 *
 * Write PT6302 CGRAM (set symbol pixel data)
 *
 * addr = symbol number (0..7)
 * data = symbol data (5 bytes for each column,
 *        column 0 (leftmost) first, MSbit ignored)
 * len  = number of symbols to define pixels
 *        of (8 - addr max.)
 *
 */
int pt6302_write_cgram(unsigned char addr, unsigned char *data, unsigned char len)
{
	pt6302_command_t cmd;
	uint8_t          wdata[41];
	uint8_t          i = 0, j = 0;

	dprintk(150, "%s >\n", __func__);

	if (len == 0)
	{
		return 0;
	}
	spin_lock(&mr_lock);

	cmd.cgram.cmd  = PT6302_COMMAND_CGRAM_WRITE;
	cmd.cgram.addr = (addr & 0x7);
	wdata[0] = cmd.all;

	if (len > 8 - addr)
	{
		len = 8 - addr;
	}
	for (i = 0; i < len; i++)
	{
		for (j = 0; j < 5; j++)
		{
			wdata[i * 5 + j + 1] = data[i * 5 + j];
		}
	}
	pt6302_WriteData(wdata, len * 5 + 1);
	spin_unlock(&mr_lock);
//	dprintk(150, "%s <\n", __func__);
	return 0;
}

/******************************************************
 *
 * Convert UTF8 formatted text to display text for
 * PT6302.
 *
 * Returns corrected length.
 *
 */
int pt6302_utf8conv(unsigned char *text, unsigned char len)
{
	int i;
	int wlen;
	unsigned char kbuf[64];
	unsigned char *UTF_Char_Table = NULL;

//	dprintk(120, "%s >\n", __func__);

	wlen = 0;  // input index
	i = 0;  // output index
	memset(kbuf, 0x20, sizeof(kbuf));  // fill output buffer with spaces
	text[len] = 0x00;  // terminate text

	while ((i < len) && (wlen < VFD_DISP_SIZE))
	{
		if (text[i] == 0x00)
		{
			break;  // stop processing
		}
		else if (text[i] >= 0x20 && text[i] < 0x80)  // if normal ASCII, but not a control character
		{
			kbuf[wlen] = text[i];
			wlen++;
		}
		else if (text[i] < 0xe0)
		{
			switch (text[i])
			{
				case 0xc2:
				{
					if (text[i + 1] < 0xa0)
					{
						i++; // skip non-printing character
					}
					else
					{
						UTF_Char_Table = PT6302_UTF_C2;
					}
					break;
				}
				case 0xc3:
				{
					UTF_Char_Table = PT6302_UTF_C3;
					break;
				}
				case 0xc4:
				{
					UTF_Char_Table = PT6302_UTF_C4;
					break;
				}
				case 0xc5:
				{
					UTF_Char_Table = PT6302_UTF_C5;
					break;
				}
#if 0  // Cyrillic currently not supported
				case 0xd0:
				{
					UTF_Char_Table = PT6302_UTF_D0;
					break;
				}
				case 0xd1:
				{
					UTF_Char_Table = PT6302_UTF_D1;
					break;
				}
#endif
				default:
				{
					UTF_Char_Table = NULL;
				}
			}
			i++;  // skip UTF-8 lead in
			if (UTF_Char_Table)
			{
				kbuf[wlen] = UTF_Char_Table[text[i] & 0x3f];
				wlen++;
			}
		}
		else
		{
			if (text[i] < 0xF0)
			{
				i += 2;  // skip 2 bytes
			}
			else if (text[i] < 0xF8)
			{
				i += 3;  // skip 3 bytes
			}
			else if (text[i] < 0xFC)
			{
				i += 4;  // skip 4 bytes
			}
			else
			{
				i += 5;  // skip 5 bytes
			}
		}
		i++;
	}
	memset(text, 0x00, sizeof(kbuf) + 1);
	for (i = 0; i < wlen; i++)
	{
		text[i] = kbuf[i];
	}
//	dprintk(120, "%s <\n", __func__);
	return wlen;
}

/******************************************************
 *
 * Write text on VFD display
 *
 * NOTE: in order not to inadvertently display icons,
 * the full display width of 15 is always written,
 * padded with spaces if needed.
 *
 */
int pt6302_write_text(unsigned char offset, unsigned char *text, unsigned char len, int utf8_flag)
{
	int ret = -1;
	int i;
	int wlen;
	unsigned char kbuf[16];
	unsigned char wdata[17];

	dprintk(150, "%s > len = %d\n", __func__, len);
	if (len == 0)  // if length is zero,
	{
		dprintk(1, "%s < (len =  0)\n", __func__);
		return 0;  // do nothing
	}

	// remove possible trailing LF
	if (text[len -1 ] == 0x0a)
	{
		len--;
	}
	text[len] = 0x00;  // terminate text

	/* handle UTF-8 chars */
	if (utf8_flag)
	{
		len = pt6302_utf8conv(text, len);
	}
	memset(kbuf, 0x20, sizeof(kbuf));  // fill output buffer with spaces
	wlen = (len > VFD_DISP_SIZE ? VFD_DISP_SIZE : len);
	memcpy(kbuf, text, wlen);  // copy text

	/* save last string written to fp */
	memcpy(&lastdata.vfddata, kbuf, wlen);
	lastdata.length = wlen;

#if 0  // set to 1 if you want the display text centered (carried over, not tested yet)
	dprintk(10, "%s Center text\n", __func__);
	/* Center text */
	j = 0;

	if (wlen < VFD_DISP_SIZE - 1)  // handle leading spaces
	{
		for (i = 0; i < ((VFD_DISP_SIZE - wlen) / 2); i++)
		{
			wdata[i + 1] = 0x20;
			j++;
		}
	}
	for (i = 0; i < wlen; i++)  // handle text
	{
		wdata[i + 1 + j] = kbuf[(wlen - 1) - i];
	}

	if (wlen < VFD_DISP_SIZE)  // handle trailing spaces
	{
		for (i = j + wlen; i < VFD_DISP_SIZE; i++)
		{
			wdata[i + 1] = 0x20;
		}
	}
#else
	for (i = 0; i < VFD_DISP_SIZE; i++)
	{
		wdata[i] = kbuf[VFD_DISP_SIZE - 1 - i];
//		dprintk(1, "wdata[%02d] = 0x%02x\n", i, wdata[i]);
	}
#endif
	ret = pt6302_write_dcram(ICON_WIDTH, wdata, VFD_DISP_SIZE);

	dprintk(150, "%s <\n", __func__);
	return ret;
}

/******************************************************
 *
 * Set brightness of VFD display
 *
 */
void pt6302_set_brightness(int level)
{
	pt6302_command_t cmd;

//	dprintk(150, "%s >\n", __func__);

	spin_lock(&mr_lock);

	if (level < PT6302_DUTY_MIN)
	{
		level = PT6302_DUTY_MIN;
	}
	if (level > PT6302_DUTY_MAX)
	{
		level = PT6302_DUTY_MAX;
	}
	cmd.duty.cmd  = PT6302_COMMAND_SET_DUTY;
	cmd.duty.duty = level;

	pt6302_send_byte(cmd.all);
	spin_unlock(&mr_lock);
//	dprintk(150,"%s <\n", __func__);
}

/*********************************************************
 *
 * Set number of characters on VFD display
 *
 * Note: sets num + 8 as width, but num = 0 sets width 16
 * 
 */
static void pt6302_set_digits(int num)
{
	pt6302_command_t cmd;

//	dprintk(150, "%s >\n", __func__);

	spin_lock(&mr_lock);

	if (num < PT6302_DIGITS_MIN)
	{
		num = PT6302_DIGITS_MIN;
	}
	if (num > PT6302_DIGITS_MAX)
	{
		num = PT6302_DIGITS_MAX;
	}
	num = (num == PT6302_DIGITS_MAX) ? 0 : (num - PT6302_DIGITS_OFFSET);

	cmd.digits.cmd    = PT6302_COMMAND_SET_DIGITS;
	cmd.digits.digits = num;

	pt6302_send_byte(cmd.all);
	spin_unlock(&mr_lock);
//	dprintk(150,"%s <\n", __func__);
}

/******************************************************
 *
 * Set VFD display on, off or all segments lit
 *
 */
static void pt6302_set_light(int onoff)
{
	pt6302_command_t cmd;

//	dprintk(150, "%s >\n", __func__);

	spin_lock(&mr_lock);

	if (onoff < PT6302_LIGHT_NORMAL || onoff > PT6302_LIGHT_ON)
	{
		onoff = PT6302_LIGHT_NORMAL;
	}
//	dprintk(10, "Switch VFD display %s\n", onoff == PT6302_LIGHT_NORMAL ? "on" : "off");
	cmd.light.cmd   = PT6302_COMMAND_SET_LIGHT;
	cmd.light.onoff = onoff;
	pt6302_send_byte(cmd.all);
	spin_unlock(&mr_lock);
//	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Set binary output pins of PT6302
 *
 */
static void pt6302_set_port(int port1, int port2)
{
	pt6302_command_t cmd;

//	dprintk(150, "%s >\n", __func__);

	spin_lock(&mr_lock);

	cmd.port.cmd   = PT6302_COMMAND_SET_PORTS;
	cmd.port.port1 = (port1) ? 1 : 0;
	cmd.port.port2 = (port2) ? 1 : 0;

	pt6302_send_byte(cmd.all);
	spin_unlock(&mr_lock);
//	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Setup VFD display
 * 16 characters, normal display, maximum brightness
 *
 */
void pt6302_setup(void)
{
	unsigned char buf[16] = { 0x00 };  // init for ADRAM & CGRAM
	int ret;

	dprintk(150, "%s >\n", __func__);

	pt6302_set_port(1, 0);
	pt6302_set_digits(PT6302_DIGITS_MAX);
	pt6302_set_brightness(PT6302_DUTY_MAX);
	pt6302_set_light(PT6302_LIGHT_NORMAL);
	ret = pt6302_write_dcram(0x01, "               ", VFD_DISP_SIZE);  // clear display (1st 15 characters)
	ret = pt6302_write_dcram(0x00, buf, 1);  // initialize icon position
	pt6302_write_adram(0x00, buf, 16);  // clear ADRAM
	dprintk(10, "Clearing VFD icons\n");
	ret = pt6302_write_cgram(0x00, buf, 8);  // clear CGRAM
//	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Icon handling (VFD only)
 *
 */

/****************************************
 *
 * Check if icon icon_nr is displayed
 *
 * Returns -1 if not, else list index.
 *
 */
int icon_in_list(int icon_nr)
{
	int i;

	i = 0;
	while (lastdata.icon_list[i] != icon_nr && i < lastdata.icon_count)
	{
		i++;
	}
	return ((i != lastdata.icon_count) ? i : -1);
}

/****************************************
 *
 * (Re)set one icon on VFD
 *
 */
int pt6302_set_icon(int icon_nr, int on)
{
	int i, j;
	int ret = 0;
	unsigned char char_zero[1] = {0x00};

	dprintk(150, "%s >\n", __func__);

	if (on)
	{
		if (lastdata.icon_count)
		{
			// check if icon to set is already displayed
			i = icon_in_list(icon_nr);
			if (i > -1)
			{
				goto exit_set_icon;
			}
		}
		if (lastdata.icon_count == 8)
		{
			// already 8 icons displayed, remove oldest icon in list
			lastdata.icon_state[lastdata.icon_list[0]] = 0;
			
			for (i = 1; i < 8; i++)
			{
				lastdata.icon_list[i - 1] = lastdata.icon_list[i];
			}
			lastdata.icon_list[7] = icon_nr;
		}
		else
		{
			lastdata.icon_list[lastdata.icon_count] = icon_nr;
			lastdata.icon_count++;
			if (lastdata.icon_count == 1 && lastdata.icon_state[ICON_SPINNER] == 0)
			{
				ret |= pt6302_write_dcram(0, char_zero, 1);  // set icon position to zero
			}
		}
	}
	else  // switch icon to off
	{
		if (lastdata.icon_count == 0)
		{
			// no icons displayed
			goto exit_set_icon;
		}
		// find icon_nr in icon_list and remove it
		i = icon_in_list(icon_nr);
		if (i > -1)
		{
			// icon is in list, remove it
			for (j = i; j < lastdata.icon_count; j++)
			{
				lastdata.icon_list[j] = lastdata.icon_list[j + 1];
			}
			lastdata.icon_count--;
			// clear remaining icon bit patterns in PT6302
			if (lastdata.icon_state[ICON_SPINNER] == 0)
			{
				for (j = i; j < 8; j++)
				{
					ret |= pt6302_write_cgram(j, vfdIcons[ICON_MIN].pixeldata, 1);
				}
			}
			if (lastdata.icon_count == 0)  // if last icon off
			{
				i = 0;
				lastdata.icon_list[0] = 0;
				ret |= pt6302_write_dcram(0, " ", 1);  // set icon position to space
				
			}
		}
		else
		{
//			dprintk(50, "Icon %d not displayed, do nothing\n", icon_nr);
			goto exit_set_icon;
		}
	}
	// reload PT6302 icon patterns
	if (lastdata.icon_state[ICON_SPINNER] == 0)
	{
		for (i = 0; i < lastdata.icon_count; i++)
		{
			ret |= pt6302_write_cgram(i, vfdIcons[lastdata.icon_list[i]].pixeldata, 1);
		}
	}
	lastdata.icon_state[icon_nr] = on;

exit_set_icon:
 	dprintk(150, "%s < (%d)\n", __func__, ret);
	return ret;
}

/****************************************
 *
 * Determine box variant code
 *
 * Code largely taken from boxtype.c
 *
 */
int stv6412_box_variant(void)
{
	int ret;
	unsigned char buf;
	struct i2c_msg msg = { .addr = 0x4b, I2C_M_RD, .buf = &buf, .len = 1 };
	struct i2c_adapter *i2c_adap = i2c_get_adapter(I2C_BUS);

	ret = i2c_transfer(i2c_adap, &msg, 1);
	return ret;
}

int _read_reg_box_variant(unsigned int reg, unsigned char nr)
{
	int ret;
	u8 b0[] = { reg >> 8, reg & 0xff };
	u8 buf;

	struct i2c_msg msg[] =
	{
		{ .addr  = nr, .flags = 0,        .buf   = b0,   .len   = 2 },
		{ .addr  = nr, .flags = I2C_M_RD, .buf   = &buf, .len   = 1 }
	};

	struct i2c_adapter *i2c_adap = i2c_get_adapter(I2C_BUS);

	ret = i2c_transfer(i2c_adap, msg, 2);
	if (ret != 2)
	{
		return -1;
	}
	return (int)buf;
}

int stb0899_read_reg_box_variant(unsigned int reg, unsigned char nr)
{
	int result;

	result = _read_reg_box_variant(reg, nr);
	/*
	 * Bug ID 9:
	 * access to 0xf2xx/0xf6xx
	 * must be followed by read from 0xf2ff/0xf6ff.
	 */
	if ((reg != 0xf2ff) 
	&&  (reg != 0xf6ff)
	&&  (((reg & 0xff00) == 0xf200) || ((reg & 0xff00) == 0xf600)))
	{
		_read_reg_box_variant((reg | 0x00ff), nr);
	}
	return result;
}

/****************************************
 *
 * Determine box variant
 *
 * Code largely taken from boxtype.c
 *
 * Note: although not watertight as the
 *       tuners are pluggable, relies
 *       on the correct tuner board to be
 *       installed, as this is about the
 *       only distinguishing feature
 *       between models, due to the "A20
 *       tied to logic 1" patch on the
 *       mainboard.
 */
void get_box_variant(void)
{
	int ret;

	dprintk(150, "%s >\n", __func__);

//	first, check AVS for a STV6412
	ret = stv6412_box_variant();
	if (ret != 1)
	{
		box_variant = 3;  // no STV6412 --> BXZB model
		dprintk(50, "BXZB model detected\n");
	}
	else
	{	// if STV6412 found, check for 2nd demodulator (two tuners)
		// if the register from the second demodulator can be read, then this is a BSLA
		ret = stb0899_read_reg_box_variant(STB0899_NCOARSE, I2C_ADDR_STB0899_2);  // read DIV from demodulator2
		if (ret != -1)  // if read OK --> 2nd STV0899 demod present --> must be a BSLA
		{
			box_variant = 2;  // BSLA model
			dprintk(50, "BSLA model detected\n");
		}
		else  // check demodulator type
		{
			// check if demodulator is stb0899 or stv0900
			// read chip ID and revision
			ret = stb0899_read_reg_box_variant(STB0899_DEV_ID, I2C_ADDR_STB0899_1);
			if (ret > 0x30)  // if response greater than 0x30 --> STB0899 demodulator
			{
				box_variant = 1; // STB0899 demodulator --> BSKA model
				dprintk(50, "BSKA model detected\n");
			}
			else if (ret > 0)
			{
				box_variant = 4;  // STV0900 demodulator --> BZZB model
				dprintk(50, "BZZB model detected\n");
			}
			else
			{  // TODO
//				ret = read cable_demod
//				if (ret...)
//				{
//					boxvariant = 5;  //
//					dprintk(50, "Cable model detected\n");
//				}
//				else
//				{
					box_variant = 0;
					dprintk(1, "CAUTION: ITI-5800 variant cannot be determined; is tuner installed?\n");
//				}
			}
		}
	}
	dprintk(150, "%s <\n", __func__);
	return;	
}

/***************************************************
 *
 * Initialize the driver
 *
 */
int adb_5800_fp_init_func(void)
{
	int i;
	unsigned char *dsp_string;

	dprintk(150, "%s >\n", __func__);
	dprintk(10, "ADB ITI-5800S(X) front panel driver initializing\n");

	dprintk(50, "Probe PT6958 + PT6302\n");
	if (pt6xxx_init_pio_pins() == 0)
	{
		dprintk(1, "Unable to initialize PIO pins; abort\n");
		goto fp_init_fail;
	}
	pt6958_setup();
	pt6302_setup();

	sema_init(&(fp.sem), 1);
	fp.opencount = 0;

	get_box_variant();  // determine box variant

	// normalize module parameter values
	display_vfd = (display_vfd > 0 ? 1 : (display_vfd < 0 ? -1 : 0));
	display_led = (display_led > 0 ? 1 : (display_led < 0 ? -1 : 0));
	rec_key = (rec_key > 0 ? 1 : (rec_key < 0 ? -1 : 0));

	// process module parameters and apply defaults
	switch (box_variant)
	{
		case 1:  // BSKA
		case 3:  // BXZB
		{
			switch (display_led)
			{
				case -1: // led not specified (LED default on)
				{
					if (display_vfd == 1)
					{
						display_type = 2;  // both
					}
					else
					{
						display_type = 0;  // LED only
					}
					break;
				}
				case 0: // led specifically off
				{
					if (display_vfd == 1)
					{
						display_type = 1;  // VFD only
					}
					else
					{
						dprintk(1, "Defaulting to LED only\n");
						display_type = 0;
					}
					break;
				}
				default: // LED specifically on
				{
					if (display_vfd == 1)
					{
						display_type = 2;  // Both
					}
					else
					{
						display_type = 0;  // LED only
					}
					break;
				}
			}
			if (rec_key == 1)
			{
				dprintk(1, "CAUTION: non-standard key layout: EPG/REC/RES!\n");
				key_layout = 1;  // set EPG/REC/RES
			}
			else
			{
				key_layout = 0;  // default to MENU/EPG/RES
			}
			break;
		}
		case 2:  // BSLA
		case 4:  // BZZB
		{
			switch (display_vfd)
			{
				case -1: // vfd not specified (VFD default on)
				{
					if (display_led == 1)
					{
						display_type = 2;  // both
					}
					else
					{
						display_type = 1;  // VFD only
					}
					break;
				}
				case 0: // vfd specifically off
				{
					if (display_led == 1)
					{
						display_type = 0;  // LED only
					}
					else
					{
						dprintk(1, "Defaulting to VFD only\n");
						display_type = 1;
					}
					break;
				}
				default: // vfd specifically on
				{
					if (display_led == 1)
					{
						display_type = 2;  // Both
					}
					else
					{
						display_type = 1;  // VFD only
					}
					break;
				}
			}
			if (rec_key == 0)
			{
				dprintk(1, "CAUTION: non-standard key layout: MENU/EPG/RES!\n");
				key_layout = 0;  // set MENU/EPG/RES
			}
			else
			{
				key_layout = 1;  // default to EPG/REC/RES
			}
			break;
		}
		default:
		{
			// Receiver variant unknown, enable both displays & EPG/REC/RES key layout
			display_type = 2;
			key_layout = 1;
			break;
		}
	}

	switch (display_type)
	{
		case 0:
		{
			dsp_string = "LED";
			break;
		}
		case 1:
		{
			dsp_string = "VFD";
			break;
		}
		case 2:
		{
			dsp_string = "Both";
			break;
		}
	}
	dprintk(50, "Text display is %s\n", dsp_string);
	dprintk(50, "Front panel key layout: %s\n", (key_layout == 0 ? "MENU/EPG/RES" : "EPG/REC/RES"));

	// clear displayed icon list
	if (display_type > 0)
	{
		for (i = 0; i < 8; i++)
		{
			lastdata.icon_list[i] = 0;
		}
	}
	lastdata.icon_count = 0;

	dprintk(50, "ADB ITI-5800S(X) front panel driver initialization successful\n");
	dprintk(150, "%s <\n", __func__);
	return 0;

fp_init_fail:
	pt6xxx_free_pio_pins();
	dprintk(1, "ADB ITI-5800S(X) front panel driver initialization failed\n");
	dprintk(150, "%s <\n", __func__);
	return -1;
}

/******************************************************
 *
 * Text display thread code.
 *
 * All text display via /dev/vfd is done in this
 * thread that also handles scrolling.
 * 
 * Code largely taken from aotom_spark (thnx Martii)
 *
 */
void clear_display(void)
{
	char bBuf[16];
	int ret = 0;

	dprintk(100, "%s >\n", __func__);

	memset(bBuf, ' ', sizeof(bBuf));
	if (display_type > 0)
	{
		ret |= pt6302_write_text(0, bBuf, VFD_DISP_SIZE, 0);
	}
	if (display_type != 1)
	{
		ret |= pt6958_ShowBuf(bBuf, LED_DISP_SIZE, 0);
	}
	dprintk(100, "%s <\n", __func__);
}

static int led_text_thread(void *arg)
{
	struct vfd_ioctl_data *data = (struct vfd_ioctl_data *)arg;
	unsigned char buf[sizeof(data->data) + 2 * LED_DISP_SIZE];
	int len = data->length;
	int off = 0;
	int ret = 0;

	dprintk(150, "%s >\n", __func__);

	if (len > LED_DISP_SIZE)
	{
		memset(buf, ' ', sizeof(buf));
		off = 1;
		memcpy(buf + off, data->data, len);
		len += off;
		buf[len+ LED_DISP_SIZE] = 0;
	}
	else
	{
		memcpy(buf, data->data, len);
		buf[len] = 0;
	}
	led_text_thread_status = THREAD_STATUS_RUNNING;

	if (len > LED_DISP_SIZE + 1)
	{
		unsigned char *b = buf;
		int pos;

		for (pos = 0; pos < len; pos++)
		{
			int i;

			if (kthread_should_stop())
			{
				led_text_thread_status = THREAD_STATUS_STOPPED;
				return 0;
			}

			ret |= pt6958_ShowBuf(b, LED_DISP_SIZE, 1);

			// check for stop request
			for (i = 0; i < 10; i++)  // note: this loop also constitutes the delay
			{
				if (kthread_should_stop())
				{
					led_text_thread_status = THREAD_STATUS_STOPPED;
					return 0;
				}
				msleep(50);
			}
			// advance to next character
			b++;
		}
	}
	if (len > 0)  // final display, also no scroll
	{
		ret |= pt6958_ShowBuf(buf + off, LED_DISP_SIZE, 1);
	}
	else
	{
		clear_display();
	}
	led_text_thread_status = THREAD_STATUS_STOPPED;
	dprintk(150, "%s <\n", __func__);
	return 0;
}

static int vfd_text_thread(void *arg)
{
	struct vfd_ioctl_data *data = (struct vfd_ioctl_data *)arg;
	unsigned char buf[sizeof(data->data) + 2 * VFD_DISP_SIZE];
	int disp_size;
	int len = data->length;
	int off = 0;
	int ret = 0;
	int i;

//	dprintk(150, "%s >\n", __func__);

	data->data[len] = 0;  // terminate string
	len = pt6302_utf8conv(data->data, data->length);
	memset(buf, 0x20, sizeof(buf));

	if (len > VFD_DISP_SIZE)
	{
		off = 3;
		memcpy(buf + off, data->data, len);
		len += off;
		buf[len + VFD_DISP_SIZE] = 0;
	}
	else
	{
		memcpy(buf, data->data, len);
		buf[len + VFD_DISP_SIZE] = 0;
	}
	vfd_text_thread_status = THREAD_STATUS_RUNNING;

	if (len > VFD_DISP_SIZE + 1)
	{
		unsigned char *b = buf;
		int pos;

		for (pos = 0; pos < len; pos++)
		{
			int i;

			if (kthread_should_stop())
			{
				vfd_text_thread_status = THREAD_STATUS_STOPPED;
				return 0;
			}

			ret |= pt6302_write_text(0, b, VFD_DISP_SIZE, 0);

			// check for stop request
			for (i = 0; i < 6; i++)  // note: this loop also constitutes the delay
			{
				if (kthread_should_stop())
				{
					vfd_text_thread_status = THREAD_STATUS_STOPPED;
					return 0;
				}
				msleep(25);
			}
			// advance to next character
			b++;
		}
	}
	if (len > 0)  // final display, also no scroll
	{
		ret |= pt6302_write_text(0, buf + off, VFD_DISP_SIZE, 0);
	}
	else
	{
		clear_display();
	}
	vfd_text_thread_status = THREAD_STATUS_STOPPED;
//	dprintk(150, "%s <\n", __func__);
	return 0;
}

static int run_text_thread(struct vfd_ioctl_data *draw_data)
{
//	dprintk(150, "%s >\n", __func__);

	if (display_type != 1)
	{
		if (down_interruptible(&led_text_thread_sem))
		{
			return -ERESTARTSYS;
		}

		// return if there is already a draw task running for the same LED text
		if ((led_text_thread_status != THREAD_STATUS_STOPPED)
		&& led_text_task
		&& (last_led_draw_data.length == draw_data->length)
		&& !memcmp(&last_led_draw_data.data, draw_data->data, draw_data->length))
		{
			up(&led_text_thread_sem);
			return 0;
		}
	
		memcpy(&last_led_draw_data, draw_data, sizeof(struct vfd_ioctl_data));
	
		// stop existing thread, if any
		if ((led_text_thread_status != THREAD_STATUS_STOPPED) && led_text_task)
		{
			kthread_stop(led_text_task);
			while ((led_text_thread_status != THREAD_STATUS_STOPPED))
			{
				msleep(1);
			}
		}
		led_text_thread_status = THREAD_STATUS_INIT;
		led_text_task = kthread_run(led_text_thread, draw_data, "led_text_thread");

		// wait until thread has copied the argument
		while (led_text_thread_status == THREAD_STATUS_INIT)
		{
			msleep(1);
		}
		up(&led_text_thread_sem);
	}

	if (display_type > 0)
	{
		if (down_interruptible(&vfd_text_thread_sem))
		{
			return -ERESTARTSYS;
		}

		// return if there is already a draw task running for the same VFD text
		if ((vfd_text_thread_status != THREAD_STATUS_STOPPED)
		&& vfd_text_task
		&& (last_vfd_draw_data.length == draw_data->length)
		&& !memcmp(&last_vfd_draw_data.data, draw_data->data, draw_data->length))
		{
			up(&vfd_text_thread_sem);
			return 0;
		}
		memcpy(&last_vfd_draw_data, draw_data, sizeof(struct vfd_ioctl_data));
	
		// stop existing thread, if any
		if ((vfd_text_thread_status != THREAD_STATUS_STOPPED) && vfd_text_task)
		{
			kthread_stop(vfd_text_task);
			while ((vfd_text_thread_status != THREAD_STATUS_STOPPED))
			{
				msleep(1);
			}
		}
		vfd_text_thread_status = THREAD_STATUS_INIT;
		vfd_text_task = kthread_run(vfd_text_thread, draw_data, "vfd_text_thread");

		// wait until thread has copied the argument
		while (vfd_text_thread_status == THREAD_STATUS_INIT)
		{
			msleep(1);
		}
		up(&vfd_text_thread_sem);
	}
//	dprintk(150, "%s <\n", __func__);
	return 0;
}

int fp5800_WriteText(char *buf, size_t len)
{
	int res = 0;
	struct vfd_ioctl_data data;

//	dprintk(150, "%s > (len %d)\n", __func__, len);
	if (len > sizeof(data.data))  // do not display more than 64 characters
	{
		data.length = sizeof(data.data);
	}
	else
	{
		data.length = len;
	}

	while ((data.length > 0) && (buf[data.length - 1 ] == '\n'))
	{
		data.length--;
	}

	if (data.length > sizeof(data.data))
	{
		len = data.length = sizeof(data.data);
	}
	memcpy(data.data, buf, data.length);
	res = run_text_thread(&data);

//	dprintk(150, "%s < res = %d\n", __func__, res);
	return res;
}

/***************************************************
 *
 * Code for writing to /dev/vfd
 *
 * If text to display is longer than the display
 * width, it is scolled once. Maximum text length
 * is 64 characters.
 *
 */
static ssize_t vfd_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char *kernel_buf;
	int res = 0;

	struct vfd_ioctl_data data;

	dprintk(150, "%s > (len %d, offs %d)\n", __func__, len, (int) *off);

	kernel_buf = kmalloc(len + 1, GFP_KERNEL);

	memset(kernel_buf, 0, len + 1);
	memset(&data, 0, sizeof(struct vfd_ioctl_data));

	if (kernel_buf == NULL)
	{
		printk("%s returns no memory <\n", __func__);
		return -ENOMEM;
	}
	copy_from_user(kernel_buf, buff, len);

	fp5800_WriteText(kernel_buf, len);

	kfree(kernel_buf);

	dprintk(150, "%s < res %d len %d\n", __func__, res, len);

	if (res < 0)
	{
		return res;
	}
	return len;
}

/*******************************************
 *
 * Code for reading from /dev/vfd
 *
 */
static ssize_t vfd_read(struct file *filp, char *buf, size_t len, loff_t *off)
{
	dprintk(150, "%s >\n", __func__);

	if (len > lastdata.length)
	{
		len = lastdata.length;
	}

	if (display_type = 0)
	{
		if (len > 8)
		{
			len = 8;
		}
		copy_to_user(buf, lastdata.leddata, len);
	}
	else
	{
		if (len > VFD_DISP_SIZE)
		{
			len = VFD_DISP_SIZE;
		}
		copy_to_user(buf, lastdata.vfddata, len);
	}
	dprintk(150, "%s <\n", __func__);
	return len;
}

static int vfd_open(struct inode *inode, struct file *file)
{
//	dprintk(150, "%s >\n", __func__);

	if (down_interruptible(&(fp.sem)))
	{
		dprintk(1, "Interrupted while waiting for semaphore\n");
		return -ERESTARTSYS;
	}
	if (fp.opencount > 0)
	{
		dprintk(1, "Device already opened\n");
		up(&(fp.sem));
		return -EUSERS;
	}
	fp.opencount++;
	up(&(fp.sem));
//	dprintk(150,"%s <\n", __func__);
	return 0;
}

static int vfd_close(struct inode *inode, struct file *file)
{
//	dprintk(150, "%s >\n", __func__);
	fp.opencount = 0;
//	dprintk(150, "%s <\n", __func__);
	return 0;
}

/****************************************
 *
 * IOCTL handling.
 *
 */
static int vfd_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	static int mode = 0;
	struct adb_box_fp_ioctl_data *adb_box_fp = (struct adb_box_fp_ioctl_data *)arg;
	struct vfd_ioctl_data vfddata;
	int i;
	int level;
	int ret = 0;
	int icon_nr;
	int on;
	unsigned char clear[1] = {0x00};

//	dprintk(150, "%s >\n", __func__);

	switch (cmd)
	{
		case VFDDISPLAYCHARS:
		case VFDBRIGHTNESS:
		case VFDDISPLAYWRITEONOFF:
		case VFDICONDISPLAYONOFF:
		case VFDSETLED:
		case VFDLEDBRIGHTNESS:
		case VFDSETFAN:
		{
			copy_from_user(&vfddata, (void *)arg, sizeof(struct vfd_ioctl_data));
		}
	}

	switch (cmd)
	{
		case VFDSETMODE:
		{
			mode = adb_box_fp->u.mode.compat;
			break;
		}
		case VFDSETLED:
		{
			pt6958_set_led((mode == 0 ? vfddata.address : adb_box_fp->u.led.led_nr), (mode == 0 ? vfddata.data[3] : adb_box_fp->u.led.level));
			mode = 0;
			break;
		}
		case VFDBRIGHTNESS:
		{
			level = (mode == 0 ? vfddata.address : adb_box_fp->u.brightness.level);
			if (level < 0)
			{
				level = 0;
			}
			else if (level > 7)
			{
				level = 7;
			}
			if (display_type > 0)  // set VFD brightness
			{
				pt6302_set_brightness(level);
			}
			if (display_type == 0)  // set LED brightness only if LED text display only
			{
				pt6958_set_brightness(level);
			}
			mode = 0;
			break;
		}
		case VFDLEDBRIGHTNESS:
		{
			level = (mode == 0 ? vfddata.address : adb_box_fp->u.brightness.level);
			if (level < 0)
			{
				level = 0;
			}
			else if (level > 7)
			{
				level = 7;
			}
			if (display_type > 0)
			{
				pt6958_set_brightness(level);
				// do not set VFD text display
			}
			else
			{
				dprintk(1, "Display is set to LED; use VFDBRIGHTNESS IOCTL\n");
			}
			mode = 0;
			break;
		}
		case VFDDRIVERINIT:
		{
			ret = adb_5800_fp_init_func();
			mode = 0;
			break;
		}
		case VFDICONDISPLAYONOFF:
		{
			int locked = 0;

			icon_nr = (mode == 0 ? vfddata.data[0] : adb_box_fp->u.icon.icon_nr);
			on = (mode == 0 ? vfddata.data[4] : adb_box_fp->u.icon.on);

			if (icon_nr < ICON_MIN + 1 || icon_nr > ICON_SPINNER)
			{
				dprintk(1, "Error: illegal icon number %d (mode %d)\n", icon_nr, mode);
				goto icon_exit;
			}
//			dprintk(10, "%s Set icon %s (%d) to %s (mode %d)\n", __func__, vfdIcons[icon_nr].name, icon_nr, (on ? "on" : "off"), mode);

			if (locked)
			{
				dprintk(1, "Locked!\n");
				goto icon_exit;
			}
			locked = 1;

			// Part one: translate E2 icon numbers to own icon numbers (vfd mode only)
			if (mode == 0)  // vfd mode
			{
				switch (icon_nr)
				{
					case 0x13:  // crypted
					{
						icon_nr = ICON_SCRAMBLED;
						break;
					}
					case 0x17:  // dolby
					{
						icon_nr = ICON_DOLBY;
						break;
					}
					case 0x15:  // MP3
					{
						icon_nr = ICON_MP3;
						break;
					}
					case 0x11:  // HD
					{
						icon_nr = ICON_HD;
						break;
					}
					case 0x1e:  // record
					{
						icon_nr = ICON_REC;
						break;
					}
					case 0x1a:  // seekable (play)
					{
						icon_nr = ICON_PLAY;
						break;
					}
					default:
					{
						break;
					}
				}
			}
			// Part two: decide wether one icon, all or spinner
			switch (icon_nr)
			{
				case ICON_SPINNER:
				{
					ret = 0;
					if (on)
					{
						lastdata.icon_state[ICON_SPINNER] = 1;
						spinner_state.state = 1;
						if (icon_state.state == 1)
						{
//							dprintk(50, "%s Stop icon thread\n", __func__);
							icon_state.state = 0;
							i = 0;
							do
							{
								msleep(250);
								i++;
							}
							while (icon_state.status != THREAD_STATUS_HALTED && i < 5);
							if (i == 5)
							{
								dprintk(1, "%s Time out stopping icon thread!\n", __func__);
							}
//							dprintk(1, "%s Icon thread stopped\n", __func__);
						}
						if (on == 1)  // handle default
						{
							on = 25;  // full cycle takes 16 * 250 ms = 4 sec
						}
						spinner_state.period = on * 10;
						up(&spinner_state.sem);
					}
					else
					{
						if (spinner_state.state == 1)
						{
							spinner_state.state = 0;
//							dprintk(50, "%s Stop spinner\n", __func__);
							spinner_state.state = 0;
							i = 0;
							do
							{
								msleep(250);
								i++;
							}
							while (spinner_state.status != THREAD_STATUS_HALTED && i < 20);
							if (i == 20)
							{
								dprintk(1, "%s Time out stopping spinner thread!\n", __func__);
							}
//							dprintk(50, "%s Spinner stopped (%d icons to recover)\n", __func__, lastdata.icon_count);
						}
						lastdata.icon_state[ICON_SPINNER] = 0;  // flag spinner off
						if (lastdata.icon_count > 1)
						{
							// reload PT6302 icon patterns
							for (i = 0; i < lastdata.icon_count; i++)
							{
//								dprintk(50, "%s Reload pixel data for icon %d\n", __func__, lastdata.icon_list[i]);
								ret |= pt6302_write_cgram(i, vfdIcons[lastdata.icon_list[i]].pixeldata, 1);
							}
//							dprintk(50, "%s Restart icon thread\n", __func__);
							icon_state.state = 1;
							up(&icon_state.sem);
						}
						else if (lastdata.icon_count == 1)
						{
							// get icon number and set icon back on
							i = 0;
							while (lastdata.icon_state[i] == 0 && i < ICON_MAX)
							{
								i++;
							}
							lastdata.icon_count--;  // pt6302_set_icon will increment it back to 1
//							dprintk(50, "%s Restore icon %d\n", __func__, i);
							ret |= pt6302_set_icon(i, 1);
						}
					}
					break;
				}
				case ICON_MAX:
				{
					if (on > 1 || on < 0)
					{
						dprintk(1, "Illegal state for icon %d\n", ICON_MAX);
					}
					if (on)
					{
						lastdata.icon_state[ICON_MAX] = 1;
						if (spinner_state.state == 1)  // switch spinner off if on
						{
//							dprintk(50, "%s Stop spinner\n", __func__);
							spinner_state.state = 0;
							do
							{
								msleep(250);
							}
							while (spinner_state.status != THREAD_STATUS_HALTED);
//							dprintk(50, "%s Spinner stopped\n", __func__);
						}
						if (icon_state.state != 1)
						{
//							dprintk(50, "%s Start icon thread\n", __func__);
							icon_state.state = 1;
							up(&icon_state.sem);
						}

					}
					else  // all icons off
					{
						{  // stop thread and clear all icons
//							dprintk(50, "%s Stop icon thread\n", __func__);
							icon_state.state = 0;
							i = 0;
							do
							{
								msleep(250);
								i++;
							}
							while (icon_state.status != THREAD_STATUS_HALTED && i < 128);
							if (i == 128)
							{
								dprintk(1, "%s Time out stopping icon thread!\n", __func__);
							}
//							dprintk(50, "%s Icon thread stopped\n", __func__);
							
							lastdata.icon_count = 0;
							for (i = ICON_MIN + 1; i < ICON_SPINNER; i++) // include spinner
							{
								lastdata.icon_state[i] = 0;
							}
							// clear PT6302 CGRAM and icon list
							for (i = 0; i < 8; i++)
							{
								ret |= pt6302_write_cgram(i, vfdIcons[ICON_MIN].pixeldata, 1);
								lastdata.icon_list[i] = 0;
							}
						}
					}
					break;
				}
				default:  // (re)set a single icon
				{
					if (on > 1 || on < 0)
					{
						dprintk(1, "Illegal state for icon %d\n", icon_nr);
					}
					ret |= pt6302_set_icon(icon_nr, on);
					if (((icon_state.state == 0) && on) && lastdata.icon_count > 1)  // if no icon thread yet but > 1 icons on
					{
//						dprintk(50, "%s Start icon thread\n", __func__);
						icon_state.state = 1;
						up(&icon_state.sem);
					}
					if (lastdata.icon_count < 2 && icon_state.state == 1)
					{
						icon_state.state = 0;  // stop icon thread
//						dprintk(50, "%s Stopping icon thread\n", __func__);
						i = 0;
						do
						{
							msleep(250);
							i++;
						}
						while (icon_state.status != THREAD_STATUS_HALTED && i < 120);
						if (i == 120)
						{
							dprintk(1, "%s Time out stopping icon thread!\n", __func__);
						}
//						dprintk(50, "%s Icon thread stopped\n", __func__);
					}
					break;
				}
			}
			locked = 0;
icon_exit:
			mode = 0;
			break;
		}
#if 0
		case VFDSTANDBY:
		{
			clear_display();
			dprintk(10, "Set deep standby mode, wake up time: (MJD= %d) - %02d:%02d:%02d (local)\n", (adb_box_fp->u.standby.time[0] & 0xff) * 256 + (adb_box_fp->u.standby.time[1] & 0xff),
				adb_box_fp->u.standby.time[2], adb_box_fp->u.standby.time[3], adb_box_fp->u.standby.time[4]);
			res = adb_box_fp_SetStandby(adb_box_fp->u.standby.time);
			break;
		}
#endif
		case VFDDISPLAYCHARS:
		{
			if (mode == 0)
			{
				if (display_type > 0)
				{
					unsigned char out[VFD_DISP_SIZE + 1];
					int wlen;

					// code to correct reversed position numbering (0 = rightmost!)
					memset(out, 0x20, sizeof(out));  // fill output buffer with spaces
					wlen = (vfddata.length > VFD_DISP_SIZE ? VFD_DISP_SIZE : vfddata.length);
					for (i = 0; i < wlen - 1; i++)
					{
						out[VFD_DISP_SIZE - 1 - i] = vfddata.data[i];
					}
					ret |= pt6302_write_dcram(ICON_WIDTH, out, VFD_DISP_SIZE);
				}
				if (display_type != 1)
				{
					ret |= pt6958_ShowBuf(vfddata.data, vfddata.length, 0);
				}
			}
			else
			{
				mode = 0;
			}
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			if (display_type > 0)
			{
				pt6302_set_light(adb_box_fp->u.light.onoff == 0 ? PT6302_LIGHT_OFF : PT6302_LIGHT_NORMAL);
			}
			pt6958_set_light(adb_box_fp->u.light.onoff);
			mode = 0;
			break;
		}
		case VFDSETFAN:
		{
			if (adb_box_fp->u.fan.speed > 255)
			{
				adb_box_fp->u.fan.speed = 255;
			}
//			dprintk(10, "Set fan speed to: %d\n", adb_box_fp->u.fan.speed);
			ctrl_outl(adb_box_fp->u.fan.speed, fan_registers + 0x04);
			mode = 0;
			break;
		}
		case 0x5305:
		case 0x5401:
		{
			mode = 0;
			break;
		}
		default:
		{
			dprintk(1, "Unknown IOCTL %08x\n", cmd);
			mode = 0;
			break;
		}
	}
//	dprintk(150, "%s <\n", __func__);
	return ret;
}

struct file_operations vfd_fops =
{
	.owner   = THIS_MODULE,
	.ioctl   = vfd_ioctl,
	.write   = vfd_write,
	.read    = vfd_read,
	.open    = vfd_open,
	.release = vfd_close
};
// vim:ts=4
