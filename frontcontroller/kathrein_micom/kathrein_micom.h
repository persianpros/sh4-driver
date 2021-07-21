/*****************************************************************************
 *
 * kathrein_micom_file.h
 *
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add irq handling
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
 *************************************************************************
 *
 * This driver covers the following models:
 * 
 * Kathrein UFS912
 * Kathrein UFS913
 * Kathrein UFS922
 * Kathrein UFC960
 *
 *************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * -----------------------------------------------------------------------
 * 20170312 Audioniek       Add support for dprintk(0,... (print always).
 * 20210703 Audioniek       Add support for VFDSETFAN (ufs922 only).
 *
 *************************************************************************
 */
#ifndef _kathrein_micom_h
#define _kathrein_micom_h

extern short paramDebug;
#define TAGDEBUG "[kathrein micom] "

#ifndef dprintk
#define dprintk(level, x...) do \
{ \
	if (((paramDebug) && (paramDebug >= level)) || (level == 0)) \
	{ \
		printk(TAGDEBUG x); \
	} \
} while (0)
#endif

#define VFD_MAJOR           147
#define FRONTPANEL_MINOR_RC 1
#define LASTMINOR           2

typedef struct
{
	struct file      *fp;
	int              read;
	struct semaphore sem;
} tFrontPanelOpen;

extern tFrontPanelOpen FrontPanelOpen[LASTMINOR];

/* ioctl numbers ->hacky */
#define VFDDISPLAYCHARS      0xc0425a00
#define VFDCGRAMWRITE1       0xc0425a01
#define VFDCGRAMWRITE2       0x40425a01
#define VFDBRIGHTNESS        0xc0425a03
#define VFDDISPLAYWRITEONOFF 0xc0425a05
#define VFDDRIVERINIT        0xc0425a08
#define VFDICONDISPLAYONOFF  0xc0425a0a

#define VFDSETRCCODE         0xc0425af5  // CAUTION: was 0xc0425af6
//#define VFDSETPOWERONTIME    0xc0425af6
#if defined(UFS922)
#define VFDSETFAN            0xc0425af6
#endif
#define VFDGETVERSION        0xc0425af7
#define VFDLEDBRIGHTNESS     0xc0425af8
#define VFDGETWAKEUPMODE     0xc0425af9
#define VFDGETTIME           0xc0425afa
#define VFDSETTIME           0xc0425afb
#define VFDSTANDBY           0xc0425afc
#define VFDREBOOT            0xc0425afd
#define VFDSETLED            0xc0425afe
#define VFDSETMODE           0xc0425aff

#define NO_ACK               0
#define NEED_ACK             1

/* List of commands to frontprocessor */
//                          Opcode    par0       par1   par2 par3  par4  par5  par6  par7  returns
#define CmdSetModelcode     0x03  //  model#     -      -    -     -     -     -     -      -
#define CmdGetVersion       0x05  //  -          -      -    -     -     -     -     -      2 bytes, tens and units
#define CmdSetLED           0x06  //  LED#       -      -    -     -     -     -     -      -
#define CmdSetLEDBrightness 0x07  //  brightness -      -    -     -     -     -     -      -
#define CmdSetIcon          0x11  //  icon#      -      -    -     -     -     -     -      -
#define CmdClearIcon        0x12  //  icon#      -      -    -     -     -     -     -      -
#define CmdClearLED         0x22  //  LED#       -      -    -     -     -     -     -      -
#define CmdSetVFDText       0x21  //  *text      #bytes -    -     -     -     -     -      -
#define CmdSetVFDBrightness 0x25  //  brightness -      -    -     -     -     -     -      -
#define CmdSetTime          0x31  //  mjdh       mjdL   hour min   sec   -     -     -      -
#define CmdSetWakeUpTime    0x32  //  mjdh       mjdL   hour min   sec   -     -     -      -
#define CmdClearWakeUpTime  0x33  //  -          -      -    -     -     -     -     -      -
#define CmdGetTime          0x39  //  -          -      -    -     -     -     -     -      5 bytes: mjdH, mjdL, h, m, s
#define CmdSetDeepStandby   0x41  //  -          -      -    -     -     -     -     -      -
#define CmdGetWakeUpMode    0x43  //  -          -      -    -     -     -     -     -      1 byte, mode
#define CmdReboot           0x46  //  -          -      -    -     -     -     -     -      -
#define CmdWriteString      0x21  //  string     length -    -     -     -     -     -      -  
#define CmdSetRCcode        0x55  //  0x02       0xff   0x80 0x48  code  -     -     -      -

struct set_brightness_s
{
	char level;
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

#if defined(UFS922)
struct set_fan_s
{
	int speed;
};
#endif

/* time must be given as follows:
 * time[0] & time[1] = MJD
 * time[2] = hour
 * time[3] = min
 * time[4] = sec
 */
struct set_standby_s
{
	char time[5];
};

struct set_time_s
{
	char time[5];
};

/* This will set the mode temporarily (for one ioctl)
 * to the desired mode. Currently the "normal" mode
 * is the compatible vfd mode
 */
struct set_mode_s
{
	int compat; /* 0 = compatibility mode to vfd driver; 1 = micom mode */
};

struct micom_ioctl_data
{
	union
	{
		struct set_icon_s icon;
		struct set_led_s led;
		struct set_brightness_s brightness;
		struct set_light_s light;
		struct set_mode_s mode;
		struct set_standby_s standby;
		struct set_time_s time;
#if defined(UFS922)
		struct set_fan_s fan;
#endif
	} u;
};

struct vfd_ioctl_data
{
	unsigned char start_address;
	unsigned char data[64];
	unsigned char length;
};

struct saved_data_s
{
	int           length;
	char          data[20];
	unsigned char brightness;
	unsigned char ledbrightness;
	int           display_on;
};

#if defined(UFS922)
enum
{
	LED_AUX = 0x1,
	LED_LIST,
	LED_POWER,
	LED_TV_R,
	LED_VOL,
	LED_WHEEL
};
#endif

#if defined(UFS912) || defined(UFS913)
enum
{
	LED_GREEN = 0x2,
	LED_RED,
	LED_LEFT,
	LED_RIGHT
};
#endif

#if defined(UFC960)
enum
{
	LED_GREEN = 0x2,
	LED_RED
};
#endif

#if defined(UFS912) \
 || defined(UFS913) \
 || defined(UFS922) \
 || defined(UFC960)
enum
{
	ICON_MIN = 0x0,
	ICON_USB = 0x1,
	ICON_HD,
	ICON_HDD,
	ICON_SCRAMBLED,
	ICON_BLUETOOTH,
	ICON_MP3,
	ICON_RADIO,
	ICON_DOLBY,
	ICON_EMAIL,
	ICON_MUTE,
	ICON_PLAY,
	ICON_PAUSE,
	ICON_FF,
	ICON_REW,
	ICON_REC,
	ICON_TIMER,
	ICON_MAX
};
#endif

#if defined(UFS922) \
 || defined(UFS912) \
 || defined(UFS913) \
 || defined(UFC969) \
 || defined(UFH969)
#define VFD_LENGTH 16
#define VFD_CHARSIZE 1
#elif defined(UFI510) \
 ||   defined(UFC960)
#define VFD_LENGTH 12
#define VFD_CHARSIZE 2
#else
#define VFD_LENGTH 16
#define VFD_CHARSIZE 1
#endif

extern struct saved_data_s lastdata;
extern int rtc_offset;
extern int micom_init_func(void);
extern void copyData(unsigned char *data, int len);
extern void getRCData(unsigned char *data, int *len);
void dumpValues(void);

extern int errorOccured;
extern char ioctl_data[8];

extern struct file_operations vfd_fops;
#endif  // _kathrein_micom_h
// vim:ts=4
