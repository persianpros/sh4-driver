/*
 * front_bsla.c
 *
 * (c) 20?? ??
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
 * Front panel driver for ADB ITI-5800SX, BSLA and BZZB models;
 * Button and display driver.
 *
 * Devices:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *  - /dev/rc  (reading of key events)
 *
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 201????? freebox         Initial version.
 * 20190730 Audioniek       Beginning of rewrite
 * 20200424 Audioniek       Scrolling added for /dev/vfd.
 * 2020???? Audioniek       procfs added.
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
#include "pt6302_utf.h"
#include "front_bsla.h"

// Global variables
static int buttoninterval = 350 /*ms*/;
static struct timer_list button_timer;

static char *button_driver_name = "nBox frontpanel buttons";
static struct input_dev *button_dev;

struct fp_driver
{
	struct semaphore sem;
	int              opencount;
};

spinlock_t mr_lock = SPIN_LOCK_UNLOCKED;

static Global_Status_t status;

static struct fp_driver vfd;
//#if defined(button_polling)
static int bad_polling = 1;
//#endif

// Default module parameters values
short paramDebug = 0;
short rec_key    = 1;  // key layout = BSLA/BZZB

// LED states
static char led1 = 0;  // power LED
static char led2 = 0;  // record
static char led3 = 0;  // at
static char led4 = 0;  // alert
int led_brightness = 5;  // default LED brightness

static char button_reset = 0;

static struct workqueue_struct *wq;

//#define IO_Delay    5

static unsigned char key_group1, key_group2;
static unsigned int  key_front = 0;

struct stpio_pin *key_int;
struct stpio_pin *pt6xxx_din;  // note: PT6302 & PT6958 Din pins are parallel connected
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
static void pt6xxx_free_pio_pins(void)
{
//	dprintk(150, "%s >\n", __func__);
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
//	dprintk(150, "%s < PT6302/6958 PIO pins freed\n", __func__);
};

/******************************************************
 *
 * Initialize the PT6958 and PT6302 PIO pins
 *
 */
static unsigned char pt6xxx_init_pio_pins(void)
{
//	dprintk(150, "%s > Initialize PT6302 & PT6958 PIO pins\n", __func__);

// PT6302 Chip select
//	dprintk(50, "Request STPIO %d,%d for PT6302 CS\n", PORT_CS, PIN_CS);
	pt6302_cs = stpio_request_pin(PORT_CS, PIN_CS, "PT6302_CS", STPIO_OUT);
	if (pt6302_cs == NULL)
	{
		dprintk(1, "Request for CS STPIO failed; abort\n");
		goto pt_init_fail;
	}
// PT6958 Strobe
//	dprintk(50, "Request STPIO %d,%d for PT6958 STB\n", PORT_STB, PIN_STB);
	pt6958_stb = stpio_request_pin(PORT_STB, PIN_STB, "PT6958_STB", STPIO_OUT);
	if (pt6958_stb == NULL)
	{
		dprintk(1, "Request for STB STPIO failed; abort\n");
		goto pt_init_fail;
	}
// PT6302 & PT6958 Clock
//	dprintk(50, "Request STPIO %d,%d for PT6302/6958 CLK\n", PORT_CLK, PIN_CLK);
	pt6xxx_clk = stpio_request_pin(PORT_CLK, PIN_CLK, "PT6958_CLK", STPIO_OUT);
	if (pt6xxx_clk == NULL)
	{
		dprintk(1, "Request for CLK STPIO failed; abort\n");
		goto pt_init_fail;
	}
// Data (PT6302 & PT6958 Din)
//	dprintk(50, "Request STPIO %d,%d for PT6302/PT6958 Din\n", PORT_DIN, PIN_DIN);
	pt6xxx_din = stpio_request_pin(PORT_DIN, PIN_DIN, "PT6958_DIN", STPIO_BIDIR);
	if (pt6xxx_din == NULL)
	{
		dprintk(1, "Request DIN STPIO failed; abort\n");
		goto pt_init_fail;
	}
	stpio_set_pin(pt6302_cs, 1);  // set all involved PIO pins high
	stpio_set_pin(pt6958_stb, 1);
	stpio_set_pin(pt6xxx_clk, 1);
	stpio_set_pin(pt6xxx_din, 1);
	dprintk(10, "PT6302/6958 PIO pins allocated\n");
//	dprintk(150, "%s <\n", __func__);
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

//	dprintk(150, "%s >\n", __func__);
	for (i = 0; i < 8; i++)  // 8 bits in a byte, LSB first
	{
		stpio_set_pin(pt6xxx_din, 1);  // data = 1 (key will pull down the pin if pressed)
		stpio_set_pin(pt6xxx_clk, 0);  // toggle
		udelay(IO_Delay);
		stpio_set_pin(pt6xxx_clk, 1);  // clock pin
		udelay(IO_Delay);
		data_in = (data_in >> 1) | (stpio_get_pin(pt6xxx_din) > 0 ? 0x80 : 0);
	}
//	dprintk(150, "%s <\n", __func__);
	return data_in;
}

/****************************************************
 *
 * Write one byte to the PT6958 or PT6302
 *
 * Note: which of the two chips depend on the CS
 *       previously set.
 */
static void pt6xxx_send_byte(unsigned char byte)
{
	unsigned char i;

//	dprintk(150, "%s >\n", __func__);
	for (i = 0; i < 8; i++)  // 8 bits in a byte, LSB first
	{
		stpio_set_pin(pt6xxx_din, byte & 0x01);  // write bit
		stpio_set_pin(pt6xxx_clk, 0);  // toggle
		udelay(IO_Delay);
		stpio_set_pin(pt6xxx_clk, 1);  // clock pin
		udelay(IO_Delay);
		byte >>= 1;  // get next bit
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
	udelay(IO_Delay);
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
	udelay(IO_Delay);
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
	udelay(IO_Delay);
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
	udelay(IO_Delay);
//	dprintk(150, "%s <\n", __func__);
}


/******************************************************
 *
 * PT6958 LED functions
 *
 */

/****************************************************
 *
 * Read button data from the PT6958
 *
 */
static void ReadKey(void)
{
	dprintk(150, "%s >\n", __func__);
	spin_lock(&mr_lock);

	stpio_set_pin(pt6958_stb, 0);  // select PT6958
	pt6xxx_send_byte(DATA_SETCMD + READ_KEYD);  // send command 01000010b -> Read key data
	key_group1 = pt6958_read_byte();  // Get SG1/KS1 (b0..3), SG2/KS2 (b4..7)
	key_group2 = pt6958_read_byte();  // Get SG3/KS3 (b0..3), SG4/KS4 (b4..7)
//	key_group3 = pt6958_read_byte();  // Get SG5/KS5, SG6/KS6  // not needed
	stpio_set_pin(pt6958_stb, 1);  // deselect PT6958
	udelay(IO_Delay);

	spin_unlock(&mr_lock);
	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Set text and dots on LED display, also set LEDs
 *
 */
#if 0
static void PT6958_Show(unsigned char DIG1, unsigned char DIG2, unsigned char DIG3, unsigned char DIG4, unsigned char DOT1, unsigned char DOT2, unsigned char DOT3, unsigned char DOT4)
{
	spin_lock(&mr_lock);

	pt6958_send_byte(DATA_SETCMD + 0); // Set test mode off, increment address and write data display mode  01xx0000b
	udelay(IO_Delay);

	stpio_set_pin(pt6958_stb, 0);  // set strobe (latches command)

	pt6xxx_send_byte(ADDR_SETCMD + 0); // Command 2 address set, (start from 0)   11xx0000b
	DIG1 += (DOT1 == 1 ? 0x80 : 0);  // if decimal point set, add to digit data
	pt6xxx_send_byte(DIG1);
	pt6xxx_send_byte(led1);  // power led: 00-off, 01-red, 02-green, 03-orange

	DIG2 += (DOT2 == 1 ? 0x80 : 0);  // if decimal point set, add to digit data
	pt6xxx_send_byte(DIG2);
	pt6xxx_send_byte(led2);  // rec led

	DIG3 += (DOT3 == 1 ? 0x80 : 0);  // if decimal point set, add to digit data
	pt6xxx_send_byte(DIG3);
	pt6xxx_send_byte(led3);  // mail led (@)

	DIG4 += (DOT4 == 1 ? 0x80 : 0);  // if decimal point set, add to digit data
	pt6xxx_send_byte(DIG4);
	pt6xxx_send_byte(led4);  // alert led

	stpio_set_pin(pt6958_stb, 1);
	udelay(IO_Delay);

	spin_unlock(&mr_lock);
}
#else
/******************************************************
 *
 * Set text and dots on LED display
 *
 * Note: does not scroll, only displays the first 4
 * characters.
 */
static void PT6958_Show(unsigned char digit1, unsigned char digit2, unsigned char digit3, unsigned char digit4)
{
	spin_lock(&mr_lock);

	pt6958_send_byte(DATA_SETCMD + 0); // Set test mode off, increment address and write data display mode  01xx0000b
	udelay(IO_Delay);

	stpio_set_pin(pt6958_stb, 0);  // set strobe (latches command)

	pt6xxx_send_byte(ADDR_SETCMD + 0); // Command 2 address set, (start from 0)   11xx0000b

	pt6xxx_send_byte(digit1);
	pt6xxx_send_byte(led1);  // power led: 00-off, 01-red, 02-green, 03-orange

	pt6xxx_send_byte(digit2);
	pt6xxx_send_byte(led2);  // rec led

	pt6xxx_send_byte(digit3);
	pt6xxx_send_byte(led3);  // mail led (@)

	pt6xxx_send_byte(digit4);
	pt6xxx_send_byte(led4);  // alert led

	stpio_set_pin(pt6958_stb, 1);
	udelay(IO_Delay);

	spin_unlock(&mr_lock);
}
#endif

/******************************************************
 *
 * Set string on LED display, also set LEDs
 *
 * Accepts string lengths of 1, 2, 3, 4 & 8
 * First four bytes are text characters,
 * in case of length 8, last four are LED states (1=on,
 * off otherwise) for power, timer mail & alert
 */
static int PT6958_write_text(unsigned char *data, unsigned char len)
{
	unsigned char z1, z2, z3, z4;
	unsigned char kbuf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	int i = 0;
	int wlen = 0;
	unsigned char *UTF_Char_Table = NULL;

	/* Strip possible trailing LF */
	if (data[len - 1] == '\n')
	{
		data[len - 1] = '\0';
		len--;
	}
//	dprintk(10, "%s > LED text: [%s] (len = %d)\n", __func__, data, len);

	/* save last string written to fp */
//	memcpy(&lastdata.data, data, len);
#if 0 // no UTF8 handling
	while ((i < len) && (wlen < 4))
	{
		if (data[i] == '\n' || data[i] == 0)
		{
//			dprintk(10, "[%s] BREAK CHAR detected (0x%X)\n", __func__, data[i]);
//			ignore line feeds and nulls
			break;
		}
		else if (data[i] < 0x20)
		{
//			dprintk(10, "[%s] NON_PRINTABLE_CHAR '0x%X'\n", __func__, data[i]);
//			skip non printable control characters
			i++;
		}
		else if (data[i] < 0x80)  // normal ASCII
		{
			kbuf[wlen] = data[i];
//			dprintk(10, "[%s] ANSI_Char_Table '0x%X'\n", __func__, data[i]);
			wlen++;
		}
		/* handle UTF-8 characters */
		else if (data[i] < 0xE0)
		{
//			dprintk(10, "[%s] UTF_Char_Table= ", __func__);
			switch (data[i])
			{
				case 0xc2:
				{
					UTF_Char_Table = PT6958_UTF_C2;
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
#if 0 //cyrillic currently not supported
				case 0xd0:
				{
					UTF_Char_Table = PT6958_UTF_D0;
					break;
				}
				case 0xd1:
				{
					UTF_Char_Table = PT6958_UTF_D1;
					break;
				}
#endif
				default:
				{
					UTF_Char_Table = NULL;
				}
			}
			i++;  // skip UTF lead in
			if (UTF_Char_Table)
			{
//				dprintk(10, "[%s] UTF_Char = 0x%X, index = %i\n", __func__, UTF_Char_Table[data[i] & 0x3f], i);
				kbuf[wlen] = UTF_Char_Table[data[i] & 0x3f];
				wlen++;
			}
		}
		else
		{
			if (data[i] < 0xf0)
			{
				i += 2;
			}
			else if (data[i] < 0xf8)
			{
				i += 3;
			}
			else if (data[i] < 0xfC)
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
	/* end */
#endif
	z1 = z2 = z3 = z4 = 0x20;  // set non used positions to space (blank)
#if 0
	if (len == 1)
	{
		z4 = kbuf[0];
	}
	if (len == 2)
	{
		z3 = kbuf[0];
		z4 = kbuf[1];
	}
	if (len == 3)
	{
		z2 = kbuf[0];
		z3 = kbuf[1];
		z4 = kbuf[2];
	}
	if (len == 4)
	{
		z1 = ROM[data[0]];
	}
	if (wlen >= 2)
	{
		z2 = ROM[data[1 + k1]];
	}
	if (wlen >= 3)
	{
		z3 = ROM[data[2 + k1 + k2]];
	}
	if (wlen >= 4)
	{
		z4 = ROM[data[3 + k1 + k2 + k3]];
	}
	PT6958_Show(z1, z2, z3, z4, k1, k2, k3, k4); // display text & decimal points
#else
	if (wlen >= 1)
	{
		z1 = ROM[data[0]];
	}
	if (wlen >= 2)
	{
		z2 = ROM[data[1]];
	}
	if (wlen >= 3)
	{
		z3 = ROM[data[2]];
	}
	if (wlen >= 4)
	{
		z4 = ROM[data[3]];
	}
	PT6958_Show(z1, z2, z3, z4); // display text
#endif
//	dprintk(150, "%s <\n", __func__);
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
				break;
			}
			case 2:  // led power, green
			{
				led1 = (on == 1 ? 2 : 0);
				on = led1;
				pos = 1;
				break;
			}
			case 3:  // timer LED
			{
				led2 = on;
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
		udelay(IO_Delay);

		stpio_set_pin(pt6958_stb, 0);  // select PT6958
		pt6xxx_send_byte(ADDR_SETCMD + pos);  // set position (11xx????b)
		pt6xxx_send_byte(on);  // set state (on or off)
		stpio_set_pin(pt6958_stb, 1);  // and deselect PT6958
		udelay(IO_Delay);
	}
	spin_unlock(&mr_lock);
//	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Set LED
 *
 */
void pt6958_set_led(unsigned char *kbuf, unsigned char len)
{
	int led_nr = 0, on = 0;

//	dprintk(150, "%s >\n", __func__);

	led_nr = (int)kbuf[0];  // get position
	on = (int)kbuf[4];  // get on value

	switch (led_nr)
	{
		case 1:  // power LED, accepts values 0 (off), 1 (red), 2 (green) and 3 (orange)
		{
				if (on < 0 || on > 3)
				{
					dprintk(1, "Illegal LED on value %d; must be 0..3, default to off\n", on);
					on = 0;
				}
				led1 = on;
				break;
		}
		case 2:  // timer/rec LED
		case 3:  // at LED
		case 4:  // alert LED
		{
			on = (on == 0 ? 0 : 1);
			switch (led_nr)  // update led states
			{
				case 2:  // timer/rec LED
				{
					led2 = on;
					break;
				}
				case 3:  // at LED
				{
					led3 = on;
					break;
				}
				case 4:  // alert LED
				{
					led4 = on;
					break;
				}
			}
			break;
		}
	}
	udelay(IO_Delay);
	stpio_set_pin(pt6958_stb, 0);  // select PT6958
	pt6xxx_send_byte(ADDR_SETCMD + (led_nr * 2) - 1);  // set position (command 11xx????b)
	pt6xxx_send_byte(on);  // set state
	stpio_set_pin(pt6958_stb, 1);  // and deselect PT6958
	udelay(IO_Delay);
		
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
	udelay(IO_Delay);

	stpio_set_pin(pt6958_stb, 0);  // select PT6958
	pt6xxx_send_byte(ADDR_SETCMD);  // Command 2, RAM address = 0
	for (i = 0; i < 10; i++)  // 10 bytes
	{
		pt6xxx_send_byte(0);  // clear display RAM (all segments off)
	}
	stpio_set_pin(pt6958_stb, 1);  // and deselect PT6958
	udelay(IO_Delay);
//	dprintk(10, "Switch LED display on\n");
	pt6958_send_byte(DISP_CTLCMD + DISPLAY_ON + led_brightness);  // Command 3, display control, (Display ON), brightness: 10xx1BBBb

	led1 = 2;  // power LED green
	led2 = 0;  // timer/rec LED off
	led3 = 0;  // @ LED off
	led4 = 0;  // alert LED off
	PT6958_Show(0x40, 0x40, 0x40, 0x40);  // display ----, power LED green
//	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Set brightness of LED display and LEDs
 *
 */
static void pt6958_set_brightness(int level)
{
//	dprintk(150, "%s >\n", __func__);
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
//	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Switch LED display on or off
 *
 */
static void pt6958_set_light(int onoff)
{
//	dprintk(150, "%s >\n", __func__);
	spin_lock(&mr_lock);

//	dprintk(10, "Switch LED display %s\n", onoff == 0 ? "off" : "on");
	pt6958_send_byte(DISP_CTLCMD + (onoff ? DISPLAY_ON + led_brightness : 0));
	spin_unlock(&mr_lock);
//	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * PT6302 VFD functions
 *
 */

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

	dprintk(150, "%s >\n", __func__);

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
		else if (text[i] >= 0x20 || text[i] < 0x80)  // if normal ASCII, but not a control character
		{
			kbuf[wlen] = text[i];
			wlen++;
		}
		else if (text[i] < 0xE0)
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
	dprintk(150, "%s <\n", __func__);
	return wlen;
}

/******************************************************
 *
 * Write PT6302 DCRAM (display text)
 *
 */
static int pt6302_write_text(unsigned char addr, unsigned char *text, unsigned char len)
{
	unsigned char    kbuf[VFD_DISP_SIZE];
	int              i = 0;
	int              j = 0;
	pt6302_command_t cmd;

	uint8_t          wdata[20];
	int              wlen = 0;

	if (len == 0)  // if length is zero,
	{
		return 0; // do nothing
	}
	// remove possible trailing LF
	if (text[len - 1] == 0x0a)
	{
		len--;
	} 
	text[len - 1] = 0x00;  // terminate text

	/* handle UTF-8 chars */
//	len = pt6302_utf8conv(text, len);

	memset(kbuf, 0x20, sizeof(kbuf));  // fill output buffer with spaces
	wlen = (len > VFD_DISP_SIZE ? VFD_DISP_SIZE : len);
	memcpy(kbuf, text, wlen);  // copy text

	/* save last string written to fp */
//	memcpy(&lastdata.vfddata, kbuf, wlen);
//	lastdata.length = wlen;

	spin_lock(&mr_lock);

	// assemble command
	cmd.dcram.cmd  = PT6302_COMMAND_DCRAM_WRITE;
	cmd.dcram.addr = (addr & 0x0f);  // get address (= character position, 0 = rightmost!)

	wdata[0] = cmd.all;

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
		wdata[i + 1 + j] = kbuf[(wlen - 1) - i]];
	}

	if (wlen < VFD_DISP_SIZE)  // handle trailing spaces
	{
		for (i = j + wlen; i < VFD_DISP_SIZE; i++)
		{
			wdata[i + 1] = 0x20;
		}
	}
#else
	for (i = 0; i < VFD_DISP_SIZE; i++)  // move display text in reverse order to data to write
	{
		wdata[1 + (VFD_DISP_SIZE - 1 - i)] = kbuf[i];
	}
//	memcpy(wdata + 1, kbuf, VFD_DISP_SIZE);
#endif
	pt6302_WriteData(wdata, 1 + VFD_DISP_SIZE);  // j00zek: cmd + kbuf size
	spin_unlock(&mr_lock);
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
static int pt6302_write_adram(unsigned char addr, unsigned char *data, unsigned char len)
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
static int pt6302_write_cgram(unsigned char addr, unsigned char *data, unsigned char len)
{
	pt6302_command_t cmd;
	unsigned char    wdata[20];
	int              i = 0, j = 0;

//	dprintk(150, "%s >\n", __func__);

	if (len == 0)
	{
		return 0;
	}
	spin_lock(&mr_lock);

	cmd.cgram.cmd  = PT6302_COMMAND_CGRAM_WRITE;
	cmd.cgram.addr = (addr & 0x7);
	wdata[0] = cmd.all;

//	dprintk(1, "wdata[0] = 0x%02x\n", wdata[0]);
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
 * Set brightness of VFD display
 *
 */
static void pt6302_set_brightness(int level)
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
static void pt6302_setup(void)
{
	unsigned char buf[16] = { 0x00 };  // init for ADRAM & CGRAM
	int ret;

//	dprintk(150, "%s >\n", __func__);

	pt6302_set_port(1, 0);
	pt6302_set_digits(PT6302_DIGITS_MAX);
	pt6302_set_brightness(PT6302_DUTY_MAX);
	pt6302_set_light(PT6302_LIGHT_NORMAL);
	ret = pt6302_write_text(0x00, "                ", VFD_DISP_SIZE);  // clear display
	pt6302_write_adram(0x00, buf, 16);  // clear ADRAM
	ret = pt6302_write_cgram(0x00, buf, 8);  // clear CGRAM
//	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Button driver (interrupt driven)
 *
 */
//#define button_polling
//#define button_interrupt
#define	button_interrupt2

#if defined(button_interrupt2)
//#warning Compiling with button_interrupt2

/******************************************************
 *
 * Button driver (interrupt driven)
 *
 */
static void button_delay(unsigned long dummy)
{
	if (button_reset == 100)
	{
		kernel_restart(NULL);
	}
	else
	{
		if (key_front > 0)
		{
			input_report_key(button_dev, key_front, 0);
			input_sync(button_dev);
			key_front = 0;
		}
		enable_irq(FPANEL_PORT2_IRQ);
	}
}

/******************************************************
 *
 * Button IRQ handler
 *
 */
irqreturn_t fpanel_irq_handler(void *dev1, void *dev2)
{
	disable_irq(FPANEL_PORT2_IRQ);

	if ((stpio_get_pin(key_int) == 1) && (key_front == 0))
	{
		ReadKey();
		key_front = 0;
/* keygroup1: columns KS1 and KS2, bits 0 & 4 = row K1, bits 1 & 5 = row K2, bits 2 & 6 = row K3, bit s3 & 7 = row K4
 * keygroup2: columns KS3 and KS4, bits 0 & 4 = row K1, bits 1 & 5 = row K2, bits 2 & 6 = row K3, bits 3 & 7 = row K4 */
#if 0
		if (key_group2 == 0) // no key on KS3 / KS4
		{
			switch (key_group1) // get KS1 / KS2
			{
				case 64:  //  KS2 row K3
				{
					key_front = KEY_RIGHT;  // right
					break;
				}
				case 32:  // KS2 row K2
				{
					key_front = KEY_MENU;
//					key_front = KEY_EPG;  // epg
					break;
				}
				case 16: // KS2 row K1
				{
					key_front = KEY_UP;  // up
					break;
				}
				case 4:  // KS1 row K3
				{
					key_front = KEY_HOME;  // res
					break;
				}
				case 2:  // KS1 row K2
				{
					key_front = KEY_OK;  // ok
					break;
				}
				case 1: // KS1 row K1
				{
					key_front = KEY_POWER;  // pwr
					break;
				}
			}
			else
			{
			switch (key_group2)
			{
				case 4:  // KS3 row K3
				{
					key_front = KEY_EPG;  // epg
//					key_front = KEY_RECORD;  // record
					break;
				}
				case 2:  // KS3 row K2
				{
					key_front = KEY_LEFT;  // left
					break;
				}
				case 1:  // KS3 row K1
				{
					key_front = KEY_DOWN;  // down
					break;
				}
			}
		}
#else
		if ((key_group1 == 32) && (key_group2 == 0))
		{
			if (rec_key)  // if BSLA/BZZB lay out
			{
				key_front = KEY_EPG;
			}
			else
			{
				key_front = KEY_MENU;
			}
		}
		if ((key_group1 == 00) && (key_group2 == 4))
		{
			if (rec_key)  // if BSLA/BZZB lay out
			{
				key_front = KEY_RECORD;
			}
			else
			{
				key_front = KEY_EPG;
			}
		}
		if ((key_group1 == 04) && (key_group2 == 0))
		{
			key_front = KEY_HOME;  // RES
		}
		if ((key_group1 == 16) && (key_group2 == 0))
		{
			key_front = KEY_UP;  // up
		}
		if ((key_group1 == 00) && (key_group2 == 1))
		{
			key_front = KEY_DOWN;  // down
		}
		if ((key_group1 == 64) && (key_group2 == 0))
		{
			key_front = KEY_RIGHT;  // right
		}
		if ((key_group1 == 00) && (key_group2 == 2))
		{
			key_front = KEY_LEFT;  // left
		}
		if ((key_group1 == 02) && (key_group2 == 0))
		{
			key_front = KEY_OK;  // OK
		}
		if ((key_group1 == 01) && (key_group2 == 0))
		{
			key_front = KEY_POWER;  // power
		}
#endif
		if (key_front > 0)
		{
			if (key_front == KEY_HOME)  // emergency reboot: press RES 5 times
			{
				button_reset++;
				if (button_reset > 4)
				{
					dprintk(10, "!!! Restart system !!!\n");
					button_reset = 100;
				}
			}
			else
			{
				button_reset = 0;
			}
			input_report_key(button_dev, key_front, 1);
			input_sync(button_dev);
			dprintk(10, "%s Key: %d (%s)\n", __func__, key_front);
			init_timer(&button_timer);
			button_timer.function = button_delay;
			mod_timer(&button_timer, jiffies + buttoninterval);
		}
		else
		{
			enable_irq(FPANEL_PORT2_IRQ);
		}
	}
	else
	{
		enable_irq(FPANEL_PORT2_IRQ);
	}
	dprintk(150, "%s <\n", __func__);
	return IRQ_HANDLED;
}
#endif  // button_interrupt2

//----------------------------------------------------------------------------------
// handle queue

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
void button_bad_polling(void)
#else
void button_bad_polling(struct work_struct *ignored)
#endif
{
//	dprintk(150, "%s >\n", __func__);

	while (bad_polling == 1)
	{
		msleep(10);
		if (stpio_get_pin(key_int) == 1)
		{
			ReadKey();
			key_front = 0;
			if ((key_group1 == 32) && (key_group2 == 0))
			{
				if (rec_key)  // if BSLA/BZZB lay out
				{
					key_front = KEY_EPG;  // epg (bsla/bzzb)
				}
				else
				{
					key_front = KEY_MENU;  // MENU (bska/bxzb)
				}
			}
			if ((key_group1 == 00) && (key_group2 == 4))
			{
				if (rec_key)  // if BSLA/BZZB lay out
				{
					key_front = KEY_RECORD;  // REC (bsla/bzzb)
				}
				else
				{
					key_front = KEY_EPG;  // EPG (bska/bxzb)
				}
			}
			if ((key_group1 == 04) && (key_group2 == 0))
			{
				key_front = KEY_HOME;  // RES
			}
			if ((key_group1 == 16) && (key_group2 == 0))
			{
				key_front = KEY_UP;  // up
			}
			if ((key_group1 == 00) && (key_group2 == 1))
			{
				key_front = KEY_DOWN;  // down
			}
			if ((key_group1 == 64) && (key_group2 == 0))
			{
				key_front = KEY_RIGHT;  // right
			}
			if ((key_group1 == 00) && (key_group2 == 2))
			{
				key_front = KEY_LEFT;  // left
			}
			if ((key_group1 == 02) && (key_group2 == 0))
			{
				key_front = KEY_OK;  // ok
			}
			if ((key_group1 == 01) && (key_group2 == 0))
			{
				key_front = KEY_POWER;  // pwr
			}
			if (key_front > 0)
			{
				if (key_front == KEY_HOME)
				{
					button_reset++;
					if (button_reset > 4)  // emergency reboot: press res 5 times
					{
						dprintk(10, "!!! Restart system !!!\n");
						button_reset = 0;
						kernel_restart(NULL);
					}
				}
				else
				{
					button_reset = 0;
				}
				input_report_key(button_dev, key_front, 1);
				input_sync(button_dev);
				dprintk(10, "Key: %d (%s)\n", key_front, __func__);
				msleep(250);
				input_report_key(button_dev, key_front, 0);
				input_sync(button_dev);
			}
		}
	}  // while
//	dprintk(150, "%s <\n", __func__);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static DECLARE_WORK(button_obj, button_bad_polling, NULL);
#else
static DECLARE_WORK(button_obj, button_bad_polling);
#endif

static int button_input_open(struct input_dev *dev)
{
	dprintk(150, "%s >\n", __func__);
	dprintk(150, "%s <\n", __func__);
	return 0;
}

static void button_input_close(struct input_dev *dev)
{
	dprintk(150, "%s >\n", __func__);
	dprintk(150, "%s <\n", __func__);
}


/****************************************
 *
 * IOCTL handling.
 *
 */
static int vfd_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct vfd_ioctl_data vfddata;

	switch (cmd)
	{
		case VFDDISPLAYCHARS:
		case VFDBRIGHTNESS:
		case VFDDISPLAYWRITEONOFF:
		case VFDICONDISPLAYONOFF:
		case VFDSETLED:
		{
			copy_from_user(&vfddata, (void *)arg, sizeof(struct vfd_ioctl_data));
		}
	}
	switch (cmd)
	{
		case VFDDISPLAYCHARS:
		{
			pt6302_write_text(vfddata.address, vfddata.data, vfddata.length);
			return PT6958_write_text(vfddata.data, vfddata.length);
		}
		case VFDBRIGHTNESS:
		{
			pt6302_set_brightness(vfddata.address);
			pt6958_set_brightness(vfddata.address);
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			pt6302_set_light(vfddata.address);
			pt6958_set_light(vfddata.address);
			break;
		}
		case VFDICONDISPLAYONOFF:
		case VFDSETLED:
		{
			pt6958_set_led(vfddata.data, vfddata.length);
			break;
		}
		case VFDDRIVERINIT:
		{
			pt6958_setup();
			pt6302_setup();
			break;
		}
		default:
		{
			dprintk(1, "Unknown IOCTL %08x\n", cmd);
			break;
		}
	}
//	dprintk(150, "%s <\n", __func__);
	return 0;
}

/******************************************************
 *
 * Write string on VFD display (/dev/vfd)
 *
 * Texts longer than the display size are scrolled
 * once, up to a maximum of 64 characters.
 *
 */
int clear_display(void)
{
	int ret = 0;

	ret |= PT6958_write_text("                ", VFD_DISP_SIZE);
	return ret;
}

static ssize_t vfd_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
	int ret = 0;
	int i;
	int slen;
	unsigned char scroll_buf[65 + VFD_DISP_SIZE];

	if (len == 0)
	{
		ret |= clear_display();
	}
	else
	{
		slen = (len > 64 ? 64 : len);  // display 64 characters max.
		if (buf[slen - 1] == '\n')
		{
			slen--;
		}
		if (slen > VFD_DISP_SIZE)
		{
			memset(scroll_buf, 0x20, sizeof(scroll_buf));  // clear scroll buffer
			memcpy(scroll_buf, buf, slen);  // copy text to show in scroll buffer
			for (i = 0; i < slen; i++)
			{
				ret |= pt6302_write_text(0, scroll_buf + i, ((slen - i) < VFD_DISP_SIZE ? VFD_DISP_SIZE : (slen - i)));
				msleep(300);
			}
		}
		ret |= pt6302_write_text(0, (unsigned char *)buf, len);
	}
	if (ret < 0)
	{
		return ret;
	}
	else
	{
		return len;
	}
}

/******************************************************
 *
 * Read string from VFD display (/dev/vfd)
 *
 */
static ssize_t vfd_read(struct file *filp, char *buf, size_t len, loff_t *off)
#if 1
{
	// TODO: return current display string
	return len;
}
#else
{
	int minor, vLoop;

//	dprintk(100, "%s > (len %d, offs %d)\n", __func__, len, (int)*off);

	if (len == 0)
	{
		return len;
	}

	minor = -1;
	for (vLoop = 0; vLoop < LASTMINOR; vLoop++)
	{
		if (FrontPanelOpen[vLoop].fp == filp)
		{
			minor = vLoop;
		}
	}

	if (minor == -1)
	{
		dprintk(1, "Error Bad Minor\n");
		return -EUSERS;
	}

//	dprintk(100, "minor = %d\n", minor);
#if 0
	if (minor == FRONTPANEL_MINOR_RC)
	{
		int size = 0;
		unsigned char data[20];

		memset(data, 0, sizeof(data));

		getRCData(data, &size);

		if (size > 0)
		{
			if (down_interruptible(&FrontPanelOpen[minor].sem))
			{
				return -ERESTARTSYS;
			}

			copy_to_user(buf, data, size);
			up(&FrontPanelOpen[minor].sem);

			dprintk(100, "%s < %d\n", __func__, size);
			return size;
		}
		dumpValues();
		return 0;
	}
#endif

	/* copy the current display string to the user */
	if (down_interruptible(&FrontPanelOpen[minor].sem))
	{
		dprintk(1, "%s return erestartsys<\n", __func__);
		return -ERESTARTSYS;
	}

	if (FrontPanelOpen[minor].read == lastdata.length)
	{
		FrontPanelOpen[minor].read = 0;
		up(&FrontPanelOpen[minor].sem);
		dprintk(1, "%s < return 0\n", __func__);
		return 0;
	}

	if (len > lastdata.length)
	{
		len = lastdata.length;
	}

	/* fixme: needs revision because of utf8! */
	if (len > 64)
	{
		len = 64;
	}

	FrontPanelOpen[minor].read = len;
	copy_to_user(buf, lastdata.data, len);
	up(&FrontPanelOpen[minor].sem);
	dprintk(100, "%s < (len %d)\n", __func__, len);
	return len;
}
#endif

static int vfd_open(struct inode *inode, struct file *file)
{
//	dprintk(150, "%s >\n", __func__);

	if (down_interruptible(&(vfd.sem)))
	{
		dprintk(1, "Interrupted while waiting for semaphore\n");
		return -ERESTARTSYS;
	}
	if (vfd.opencount > 0)
	{
		dprintk(1, "Device already opened\n");
		up(&(vfd.sem));
		return -EUSERS;
	}
	vfd.opencount++;
	up(&(vfd.sem));
//	dprintk(150, "%s <\n", __func__);
	return 0;
}

static int vfd_close(struct inode *inode, struct file *file)
{
//	dprintk(150, "%s >\n", __func__);
	vfd.opencount = 0;
//	dprintk(150, "%s <\n", __func__);
	return 0;
}

static struct file_operations vfd_fops =
{
	.owner   = THIS_MODULE,
	.ioctl   = vfd_ioctl,
	.write   = vfd_write,
	.read    = vfd_read,
	.open    = vfd_open,
	.release = vfd_close
};

//static void __exit vfd_module_exit(void)
static void vfd_module_exit(void)
{
//	dprintk(150, "%s >\n", __func__);
	unregister_chrdev(VFD_MAJOR, "vfd");
	stpio_free_irq(key_int);
	pt6xxx_free_pio_pins();
	input_unregister_device(button_dev);
//	dprintk(150, "%s <\n", __func__);
}

static int __init vfd_module_init(void)
{
	int error;
	int res;

	dprintk(510, "Initialize ADB ITI-5800SX front panel\n");

	dprintk(20, "Probe PT6958 + PT6302\n");
	if (pt6xxx_init_pio_pins() == 0)
	{
		dprintk(1, "Unable to initinitialize driver; abort\n");
		goto vfd_init_fail;
	}
	dprintk(10, "Register character device %d\n", VFD_MAJOR);
	if (register_chrdev(VFD_MAJOR, "vfd", &vfd_fops))
	{
		dprintk(1, "Register major %d failed\n", VFD_MAJOR);
		goto vfd_init_fail;
	}
	sema_init(&(vfd.sem), 1);
	vfd.opencount = 0;

	pt6958_setup();
	pt6302_setup();

	dprintk(10, "Request STPIO %d,%d for key interrupt\n", PORT_KEY_INT, PIN_KEY_INT);
	key_int = stpio_request_pin(PORT_KEY_INT, PIN_KEY_INT, "Key_int_front", STPIO_IN);

	if (key_int == NULL)
	{
		dprintk(1, "Request STPIO for key_int failed; abort\n");
		goto vfd_init_fail;
	}
	stpio_set_pin(key_int, 1);

#if defined(button_interrupt2)
	dprintk(150, "stpio_flagged_request_irq IRQ_TYPE_LEVEL_LOW\n");
//	dprintk(10, "> stpio_flagged_request_irq IRQ_TYPE_LEVEL_LOW\n");
	stpio_flagged_request_irq(key_int, 0, (void *)fpanel_irq_handler, NULL, (long unsigned int)NULL);
//	dprintk(10, "< stpio_flagged_request_irq IRQ_TYPE_LEVEL_LOW\n");
#endif

	button_dev = input_allocate_device();
	if (!button_dev)
	{
		goto vfd_init_fail;
	}
	button_dev->name = button_driver_name;
	button_dev->open = button_input_open;
	button_dev->close = button_input_close;

	set_bit(EV_KEY, button_dev->evbit);
	if (rec_key == 0)
	{
		set_bit(KEY_MENU, button_dev->keybit);  // (bska/bxzb)
	}
	set_bit(KEY_EPG, button_dev->keybit);
	if (rec_key != 0)
	{
		set_bit(KEY_RECORD, button_dev->keybit);  // (bsla/bzzb)
	}
	set_bit(KEY_HOME, button_dev->keybit);
	set_bit(KEY_UP, button_dev->keybit);
	set_bit(KEY_DOWN, button_dev->keybit);
	set_bit(KEY_RIGHT, button_dev->keybit);
	set_bit(KEY_LEFT, button_dev->keybit);
	set_bit(KEY_OK, button_dev->keybit);
	set_bit(KEY_POWER, button_dev->keybit);

	error = input_register_device(button_dev);
	if (error)
	{
		input_free_device(button_dev);
		dprintk(1, "Request input_register_device failed; abort\n");
		goto vfd_init_fail;
	}

#if defined(button_interrupt)
	status.pio_addr = 0;
	status.pio_addr = (unsigned long)ioremap(PIO2_BASE_ADDRESS, (unsigned long)PIO2_IO_SIZE);
	if (!status.pio_addr)
	{
		dprintk(10, "Button PIO map: FATAL: Memory allocation error\n");
		goto vfd_init_fail;
	}
	status.button = BUTTON_RELEASED;

	ctrl_outl(BUTTON_PIN, status.pio_addr + PIO_CLR_PnC0);
	ctrl_outl(BUTTON_PIN, status.pio_addr + PIO_CLR_PnC1);
	ctrl_outl(BUTTON_PIN, status.pio_addr + PIO_CLR_PnC2);

	ctrl_outl(BUTTON_RELEASED, status.pio_addr + PIO_PnCOMP);
	ctrl_outl(BUTTON_PIN, status.pio_addr + PIO_PnMASK);

	res = request_irq(FPANEL_PORT2_IRQ, fpanel_irq_handler, IRQF_DISABLED, "button_irq" , NULL);
	if (res < 0)
	{
		dprintk(10, "Button IRQ: FATAL: Could not request interrupt\n");
		goto vfd_init_fail;
	}
#endif
	dprintk(50, "Front panel nBox BSLA/BZZB initialization successful\n");
	return 0;

vfd_init_fail:
	vfd_module_exit();
	return -EIO;
}

module_init(vfd_module_init);
module_exit(vfd_module_exit);

module_param(rec_key, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(rec_key, "Key lay out 0=BSKA/BXZB, >1=BSLA/BZZB (default)");

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled, >0=enabled (level)");

MODULE_DESCRIPTION("ADB ITI-5800SX PT6958 + PT6302 frontpanel driver");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
