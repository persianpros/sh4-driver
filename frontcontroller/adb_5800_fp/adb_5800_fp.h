/****************************************************************************************
 *
 * adb_5800_fp.h
 *
 * (c) 20?? Freebox
 * (c) 2019-2020 Audioniek
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
 * Button, LEDs, LED display and VFD driver, header file.
 *
 * Devices:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *
 ****************************************************************************************/
#if !defined ADB_5800_FP_H
#define ADB_5800_FP_H

extern short paramDebug;
#define TAGDEBUG "[adb_5800_fp] "

#ifndef dprintk
#define dprintk(level, x...) do \
{ \
	if ((paramDebug) && (paramDebug >= level)) printk(TAGDEBUG x); \
} while (0)
#endif

#define FPANEL_PORT2_IRQ     88

#define BUTTON_PRESSED       0x04
#define BUTTON_RELEASED      0x00

/*********************/
/* For button driver */
/*********************/
#define PIO2_BASE_ADDRESS    0x18022000
#define PIO2_IO_SIZE         0x1000
#define PIO_CLR_PnC0         0x28
#define PIO_CLR_PnC1         0x38
#define PIO_CLR_PnC2         0x48
#define PIO_SET_PnC0         0x24
#define PIO_SET_PnC1         0x34
#define PIO_SET_PnC2         0x44
#define PIO_PnMASK           0x60
#define PIO_PnCOMP           0x50

#define PIO_SET_PnCOMP       0x54
#define PIO_CLR_PnCOMP       0x58

#define PIO_PnIN             0x10

// PT6302/6958 PIO pin definitions
#define PORT_STB             1  // PT6958 only
#define PIN_STB              6

#define PORT_CLK             4  // shared between PT6958/6302 in case of BSLA/BZZB
#define PIN_CLK              0

#define PORT_DIN             4  // shared between PT6958/6302 in case of BSLA/BZZB
#define PIN_DIN              1  // also connected to PT6958 Dout

// PT6302 PIO pin definitions
#define PORT_CS              1  // PT6302 only
#define PIN_CS               2

#define PORT_KEY_INT         2
#define PIN_KEY_INT          2

#define PIO_PORT_SIZE        0x1000
#define PIO_BASE             0xb8020000
#define STPIO_SET_OFFSET     0x04
#define STPIO_CLEAR_OFFSET   0x08
#define STPIO_POUT_OFFSET    0x00

#define STPIO_SET_PIN(PIO_ADDR, PIN, V) \
	writel(1 << PIN, PIO_ADDR + STPIO_POUT_OFFSET + ((V) ? STPIO_SET_OFFSET : STPIO_CLEAR_OFFSET))
#define PIO_PORT(n) \
	(((n) * PIO_PORT_SIZE) + PIO_BASE)

#define VFD_MAJOR            147
#define LED_DISP_SIZE        4
#define ICON_WIDTH           1
#define VFD_DISP_SIZE        16 - ICON_WIDTH  // position 16 is used for icon display

// Commands to the PT6958
#define DATA_SETCMD          0x40
// Options for DATA_SETCMD
#define TEST_MODE            0x08
#define ADDR_FIX             0x04
#define READ_KEYD            0x02 

#define ADDR_SETCMD          0xc0

// Commands to the PT6302 & PT6958
#define DISP_CTLCMD          0x80
// Options for DATA_CTLCMD
#define DISPLAY_ON           0x08

// Commands to the PT6302
#define PT6302_COMMAND_DCRAM_WRITE 1
#define PT6302_COMMAND_CGRAM_WRITE 2
#define PT6302_COMMAND_ADRAM_WRITE 3
#define PT6302_COMMAND_SET_PORTS   4

#define PT6302_COMMAND_SET_DUTY    5
// options for PT6302_COMMAND_SET_DUTY
#define PT6302_DUTY_MIN            0
#define PT6302_DUTY_MAX            7

#define PT6302_COMMAND_SET_DIGITS  6
// options for PT6302_COMMAND_SET_DIGITS
#define PT6302_DIGITS_MIN          9
#define PT6302_DIGITS_MAX         16
#define PT6302_DIGITS_OFFSET       8

#define PT6302_COMMAND_SET_LIGHT   7
// options for PT6302_COMMAND_SET_LIGHT
#define PT6302_LIGHT_NORMAL        0
#define PT6302_LIGHT_OFF           1
#define PT6302_LIGHT_ON            3

#define PT6302_COMMAND_TESTMODE    8  // do not use

#define LED_Delay                  1  // us, for PT6958
#define VFD_Delay                  8  // us, tDOFF for PT6302

// box_variant stuff
#define I2C_ADDR_STB0899_1         (0xd0 >> 1)  //d0 = 0x68 tuner 1 demod BSLA & BSKA/BXZB
#define I2C_ADDR_STB0899_2         (0xd2 >> 1)  //d2 = 0x69 tuner 2 demod BSLA only
#define I2C_ADDR_STV090x           (0xd0 >> 1)  //d0 = 0x68 tuner 1/2 demod BZZB only
#define I2C_BUS                    0

#define STB0899_NCOARSE            0xf1b3
#define STB0899_DEV_ID             0xf000
#define STV090x_MID                0xf100

// IOCTL definitions
#define VFDDISPLAYCHARS            0xc0425a00
#define VFDBRIGHTNESS              0xc0425a03
#define VFDDISPLAYWRITEONOFF       0xc0425a05
#define VFDDRIVERINIT              0xc0425a08
#define VFDICONDISPLAYONOFF        0xc0425a0a

#define VFDSETFAN                  0xc0425af6
#define VFDLEDBRIGHTNESS           0xc0425af8
#define VFDSETLED                  0xc0425afe
#define VFDSETMODE                 0xc0425aff

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
	int level;
};

struct set_light_s
{
	int onoff;
};

struct set_fan_s
{
	int speed;
};

/* time must be given as follows:
 * time[0] & time[1] = MJD
 * time[2] = hour
 * time[3] = min
 * time[4] = sec
 */
//struct set_standby_s
//{
//	char time[5];
//};

//struct set_time_s
//{
//	char time[5];
//};

//struct set_timeformat_s
//{
//	char format;
//};

//struct get_version_s
//{
//	unsigned int data[2]; // data[0] = boot loader version, data[1] = resellerID
//};

/* This will set the mode temporarily (for one ioctl)
 * to the desired mode. Currently the "normal" mode
 * is the compatible vfd mode
 */
struct set_mode_s
{
	int compat; /* 0 = compatibility mode to vfd driver; 1 = adb_box mode */
};

struct adb_box_fp_ioctl_data
{
	union
	{
		struct set_icon_s icon;
		struct set_led_s led;
		struct set_brightness_s brightness;
		struct set_light_s light;
		struct set_mode_s mode;
//		struct set_standby_s standby;
//		struct set_time_s time;
		struct set_fan_s fan;
	} u;
};

struct vfd_ioctl_data
{
	unsigned char address;
	unsigned char data[64];
	unsigned char length;
};

struct fp_driver
{
	struct semaphore sem;
	int              opencount;
};

/***************************************************************
 *
 * Icons for VFD
 *
 */ 
enum  // icon numbers and their names
{
	ICON_MIN,       // 00
	ICON_REC,       // 01
	ICON_TIMESHIFT, // 02
	ICON_TIMER,     // 03
	ICON_HD,        // 04
	ICON_USB,       // 05
	ICON_SCRAMBLED, // 06
	ICON_DOLBY,     // 07
	ICON_MUTE,      // 08
	ICON_TUNER1,    // 09
	ICON_TUNER2,    // 10
	ICON_MP3,       // 11
	ICON_REPEAT,    // 12
	ICON_PLAY,      // 13
	ICON_STOP,      // 14
	ICON_PAUSE,     // 15
	ICON_REWIND,    // 16
	ICON_FF,        // 17
	ICON_STEP_BACK, // 18
	ICON_STEP_FWD,  // 19
	ICON_TV,        // 20
	ICON_RADIO,     // 21
	ICON_MAX,       // 22
	ICON_SPINNER    // 23
};

/***************************************************************
 *
 * Spinners for VFD
 *
 */ 
enum  // spinner numbers and their names
{
	SPINNER_00,  // 00
	SPINNER_01,  // 01
	SPINNER_02,  // 02
	SPINNER_03,  // 03
	SPINNER_04,  // 04
	SPINNER_05,  // 05
	SPINNER_06,  // 06
	SPINNER_07,  // 07
	SPINNER_08,  // 08
	SPINNER_09,  // 09
	SPINNER_10,  // 10
	SPINNER_11,  // 11
	SPINNER_12,  // 12
	SPINNER_13,  // 13
	SPINNER_14,  // 14
	SPINNER_15   // 15
};

// Thread stuff
#define THREAD_STATUS_RUNNING 0
#define THREAD_STATUS_STOPPED 1
#define THREAD_STATUS_INIT    2
#define THREAD_STATUS_HALTED  3  // (semaphore down)

typedef struct
{
	int state;
	int period;
	int status;
	int enable;
	struct task_struct *task;
	struct semaphore sem;
} tIconState;

struct iconToInternal
{
	char *name;
	u16 icon;
	unsigned char pixeldata[5];
};

int icon_thread(void *arg);


// struct to store current values
struct saved_data_s
{
	int length;
//	int ledlength;
//	int vfdlength;
	char leddata[8];
	char vfddata[16];
//	int brightness;
//	int display_on;
	int icon_state[ICON_MAX + 2];  // state of each icon
	int icon_count;  // number of icons switched on
	int icon_list[8];  // holds the numbers of the last eight icons set
};

// symbols shared between modules
extern int adb_5800_fp_init_func(void);
extern void pt6xxx_free_pio_pins(void);
extern int pt6302_write_dcram(unsigned char addr, unsigned char *data, unsigned char len);
extern int pt6302_write_cgram(unsigned char addr, unsigned char *data, unsigned char len);
extern void ReadKey(void);
extern struct file_operations vfd_fops;
extern struct saved_data_s lastdata;
extern struct iconToInternal vfdIcons[];
extern struct semaphore led_text_thread_sem;
extern struct task_struct *led_text_task;
extern int led_text_thread_status;
extern struct semaphore vfd_text_thread_sem;
extern struct task_struct *vfd_text_task;
extern int vfd_text_thread_status;

#endif  // ADB_5800_FP_H
// vim:ts=4
