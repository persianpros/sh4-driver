/*************************************************************************
 *
 * ufs910_fp.h
 *
 * 11. Nov 2007 - captaintrip
 *
 * Kathrein UFS910 VFD Kernelmodul
 * portiert aus den MARUSYS uboot sourcen

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
 *************************************************************************
 *
 * Frontpanel driver for Kathrein UFS910
 */
#ifndef _ufs910_fp_h_
#define _ufs910_fp_h_

extern short paramDebug;
#define TAGDEBUG "[ufs910_fp] "

#ifndef dprintk
#define dprintk(level, x...) do \
{ \
	if (((paramDebug) && (paramDebug >= level)) || level == 0) \
	{ \
		printk(TAGDEBUG x); \
	} \
} while (0)
#endif

#define VFD_MAJOR               147
#define VFD_LENGTH              16

/****************************************************************************
 *
 * The built-in display controller seems to have the following command set:
 *
 * Command                                         Data
 * -------------------------------------------------------------------------
 * DCRAMDATAWRITE    0x20 + DCRAM address (0-15)  1 byte (character data)
 * CGRAMDATAWRITE    0x40 + CGRAM address (0-7)   5 bytes for dot pattern
 * ADRAMDATAWRITE    0x60 + ADRAM address (0-23)  1 byte: #digits - 1
 * URAMDATAWRITE     0x80 + URAM address (0-7)    2 bytes
 * DIGITSET          0xe0                         1 byte:
 * DIMMINGSET        0xe4                         1 byte: brightness (0-255)
 * GRAYLEVELSET      0xa0 + register address (0)  1 byte: grey level
 * GRAYLEVELONOFF    0xc0 + digit number (0-23)   1 byte: on/off (bit 0)
 * DISPLAYLIGHTONOFF 0xe8 + LS/HS                 - 
 * STANDBYMODESET    0xec + ST                    -
 *
 * The commands 0x20, 0x40, 0x60 and 0x80 feature address auto increment
 *
 * LS HS (LS = 0x02, HS = 0x01)
 * -------------------------------
 *  0  0 Normal operation
 *  1  0 All segments off
 *  x  1 All segments on
 *
 * ST (ST = 0x01) 
 * -------------------------------
 *  0 Normal operation
 *  1 Standby mode
 *
 * CH (CH=0x02)
 * -------------------------------
 *  0 Charge pump output off (CP pin is not used on UFS910)
 *  1 Charge pump output on
 *
 */
#define DCRAM_COMMAND           (0x20 & 0xf0)
#define CGRAM_COMMAND           (0x40 & 0xf0)
#define ADRAM_COMMAND           (0x60 & 0xf0)  // controls upper row (icons)
#define NUM_DIGIT_COMMAND       (0xe0 & 0xf0)
// definitions for NUM_DIGIT_COMMAND argument
#define NUM_DIGIT_UV            0x80
#define NUM_DIGIT_01            0x00
#define NUM_DIGIT_02            0x01
#define NUM_DIGIT_03            0x02
#define NUM_DIGIT_04            0x03
#define NUM_DIGIT_05            0x04
#define NUM_DIGIT_06            0x05
#define NUM_DIGIT_07            0x06
#define NUM_DIGIT_08            0x07
#define NUM_DIGIT_09            0x08
#define NUM_DIGIT_10            0x09
#define NUM_DIGIT_11            0x0a
#define NUM_DIGIT_12            0x0b
#define NUM_DIGIT_13            0x0c
#define NUM_DIGIT_14            0x0d
#define NUM_DIGIT_15            0x0e
#define NUM_DIGIT_16            0x0f
#define NUM_DIGIT_17            (0x00 << 4) + NUM_DIGIT_UV
#define NUM_DIGIT_18            (0x01 << 4) + NUM_DIGIT_UV
#define NUM_DIGIT_19            (0x02 << 4) + NUM_DIGIT_UV
#define NUM_DIGIT_20            (0x03 << 4) + NUM_DIGIT_UV
#define NUM_DIGIT_21            (0x04 << 4) + NUM_DIGIT_UV
#define NUM_DIGIT_22            (0x05 << 4) + NUM_DIGIT_UV
#define NUM_DIGIT_23            (0x06 << 4) + NUM_DIGIT_UV
#define NUM_DIGIT_24            (0x07 << 4) + NUM_DIGIT_UV

#define LIGHT_ON_COMMAND        (0xe8 & 0xfc)
// definitions for LIGHT_ON_COMMAND argument
#define LIGHT_ON_LS             0x02
#define LIGHT_ON_HS             0x01

#define DIMMING_COMMAND         (0xe4 & 0xfc)
#define GRAY_LEVEL_DATA_COMMAND (0xa0 & 0xf8)
#define GRAY_LEVEL_ON_COMMAND   (0xc0 & 0xe0)

#define PIO_PORT_SIZE           0x1000
#define PIO_BASE                0xb8020000
#define STPIO_SET_OFFSET        0x04
#define STPIO_CLEAR_OFFSET      0x08
#define STPIO_POUT_OFFSET       0x00
#define STPIO_PIN_OFFSET	    0x10

//#define STPIO_SET_PIN(PIO_ADDR, PIN, V) writel(1 << PIN, PIO_ADDR + STPIO_POUT_OFFSET + ((V) ? STPIO_SET_OFFSET : STPIO_CLEAR_OFFSET))
#define STPIO_GET_PIN(PIO_ADDR, PIN)    ((readl(PIO_ADDR + STPIO_PIN_OFFSET) & (1<< PIN))? 1 : 0)
#define PIO_PORT(n)                     (((n)*PIO_PORT_SIZE) + PIO_BASE)


/****************************************************************************
 *
 * IOCTL definitions
 *
 */
#define VFDDISPLAYCHARS      0xc0425a00
#define VFDCGRAMWRITE1       0xc0425a01  // unique for Kathrein
// do not know why but vfdctl wants this address
#define VFDCGRAMWRITE2       0x40425a01
#define VFDBRIGHTNESS        0xc0425a03
#define VFDDISPLAYWRITEONOFF 0xc0425a05
#define VFDDRIVERINIT        0xc0425a08
#define VFDICONDISPLAYONOFF  0xc0425a0a
#define VFDSETLED            0xc0425afe
#define VFDSETMODE           0xc0425aff

//noch unbekannte ioctls
//0xc0425a04 = 0x10=Byte1
//0x40425a01 = 0x00=Byte1

#define SCP_TXD_BIT             6
#define SCP_SCK_BIT             8
#define SCP_ENABLE_BIT          5
#define VFD_PORT                0
#define SCP_DATA                3
#define SCP_CLK                 4
#define SCP_CS                  5

struct __vfd_scp
{
	__u8 tr_rp_ctrl;
	__u8 rp_data;
	__u8 tr_data;
	__u8 start_tr;
	__u8 status;
	__u8 reserved;
	__u8 clk_div;
};

struct set_brightness_s
{
	int level;
};

struct set_icon_s
{
	int icon_nr;
	int on;
};

struct set_led_s
{
	int led_nr;
	int on;
};

struct set_light_s
{
	int onoff;
};

/* This will set the mode temporarily (for one ioctl)
 * to the desired mode. Currently the "normal" mode
 * is the compatible vfd mode
 */
struct set_mode_s
{
	int compat; /* 0 = compatibility mode to vfd driver; 1 = micom mode */
};

struct ufs910_fp_ioctl_data
{
	union
	{
		struct set_icon_s icon;
		struct set_led_s led;
		struct set_brightness_s brightness;
		struct set_light_s light;
		struct set_mode_s mode;
	} u;
};

struct vfd_ioctl_data
{
	unsigned char start_address;
	unsigned char data[64];
	unsigned char length;
};

/* konfetti: quick and dirty open handling */
typedef struct
{
	struct file      *fp;
	int              read;
	struct semaphore sem;
} tVFDOpen;

struct saved_data_s
{
	int length;
	char data[128];
	int brightness;
	int display_on;
};
extern struct saved_data_s lastdata;

extern struct file_operations vfd_fops;

extern int boxtype;  // ufs910 14W: 1 or 3, ufs910 1W: 0

#define SCP_TXD_CTRL  (vfd_scp_ctrl->tr_rp_ctrl)
#define SCP_TXD_DATA  (vfd_scp_ctrl->tr_data)
#define SCP_TXD_START (vfd_scp_ctrl->start_tr)
#define SCP_STATUS    (vfd_scp_ctrl->status)

/***************************************************************************
 *
 * Front panel key definitions (14W model only).
 *
 ***************************************************************************/
#define FP_VFORMAT_KEY KEY_SWITCHVIDEOMODE
#define FP_MENU_KEY    KEY_MENU
#define FP_OPTION_KEY  KEY_AUX
#define FP_EXIT_KEY    KEY_EXIT

/***************************************************************************
 *
 * LED definitions.
 *
 ***************************************************************************/
enum
{
	LED_RED = 1,
	LED_GREEN,
	LED_YELLOW
};

/***************************************************************************
 *
 * Icon definitions.
 *
 ***************************************************************************/
enum  // UFS910 icon numbers and their names
{
	ICON_MIN,        // 00
	ICON_USB,        // 01
	ICON_HD,         // 02
	ICON_HDD,        // 03
	ICON_SCRAMBLED,  // 04
	ICON_BLUETOOTH,  // 05
	ICON_MP3,        // 06
	ICON_RADIO,      // 07
	ICON_DOLBY,      // 08
	ICON_MAIL,       // 09
	ICON_MUTE,       // 10
	ICON_PLAY,       // 11
	ICON_PAUSE,      // 12
	ICON_FF,         // 13
	ICON_REWIND,     // 14
	ICON_RECORD,     // 15
	ICON_TIMER,      // 16
	ICON_MAX         // 17
};

extern int vfd_init_func(void);

#endif // _ufs910_fp_h_
// vim:ts=4
