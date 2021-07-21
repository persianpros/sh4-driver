/****************************************************************************
 *
 * cuberevo_micom.h
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
 * header file for CubeRevo front panel driver.
 *
 ***************************************************************************/
#ifndef _cuberevo_micom_h
#define _cuberevo_micom_h

extern short paramDebug;

#define TAGDEBUG "[cuberevo_micom] "

#ifndef dprintk
#define dprintk(level, x...) do { \
		if ((paramDebug) && (paramDebug >= level)) printk(TAGDEBUG x); \
	} while (0)
#endif

typedef struct
{
	struct file      *fp;
	int               read;
	struct semaphore  sem;

} tFrontPanelOpen;

#define FRONTPANEL_MINOR_RC  1
#define LASTMINOR            2

extern tFrontPanelOpen FrontPanelOpen[LASTMINOR];

#define VFD_MAJOR            147

#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
#define SPINNER_THREAD_STATUS_RUNNING 0
#define SPINNER_THREAD_STATUS_STOPPED 1
#define SPINNER_THREAD_STATUS_INIT    2
#define SPINNER_THREAD_STATUS_HALTED  3  // (semaphore down)
//#define SPINNER_THREAD_STATUS_PAUSED  4  // (state == 0)
#endif  // CUBEREVO

/* IOCTL numbers -> hacky! */
#define VFDDISPLAYCHARS      0xc0425a00
#define VFDBRIGHTNESS        0xc0425a03
#define VFDPWRLED            0xc0425a04  /* obsolete, use VFDSETLED (0xc0425afe) */
#define VFDDISPLAYWRITEONOFF 0xc0425a05
#define VFDDRIVERINIT        0xc0425a08
#define VFDICONDISPLAYONOFF  0xc0425a0a
// comment next line to leave VFDTEST capabilities out
//#define VFDTEST              0xc0425af0  // added by audioniek
#define VFDCLEARICONS        0xc0425af6
#define VFDSETRF             0xc0425af7
#define VFDSETFAN            0xc0425af8
#define VFDGETWAKEUPMODE     0xc0425af9
#define VFDGETTIME           0xc0425afa
#define VFDSETTIME           0xc0425afb
#define VFDSTANDBY           0xc0425afc
#define VFDREBOOT            0xc0425afd
#define VFDSETLED            0xc0425afe
#define VFDSETMODE           0xc0425aff

#define VFDGETWAKEUPTIME     0xc0425b00
#define VFDGETVERSION        0xc0425b01
#define VFDSETDISPLAYTIME    0xc0425b02
#define VFDSETTIMEMODE       0xc0425b03
#define VFDSETWAKEUPTIME     0xc0425b04
#define VFDLEDBRIGHTNESS     0xc0425b05  /* Cuberevo/micom specific */

// comment next line if you want left aligned text display
#define CENTERED_DISPLAY

#if defined(CENTERED_DISPLAY)
#if defined(CUBEREVO_250HD) \
 || defined(CUBEREVO_MINI_FTA)
#undef CENTERED_DISPLAY  // never center on LED models
#endif
#endif

#define cNumberSymbols      8

/* commands to the fp */
#define VFD_GETA0              0xA0
#define VFD_GETWAKEUPSTATUS    0xA1
#define VFD_GETA2              0xA2
#define VFD_GETRAM             0xA3
#define VFD_GETDATETIME        0xA4
#define VFD_GETMICOM           0xA5
#define VFD_GETWAKEUP          0xA6  /* wakeup time */
#define VFD_SETWAKEUPDATE      0xC0
#define VFD_SETWAKEUPTIME      0xC1
#define VFD_SETDATETIME        0xC2
#define VFD_SETBRIGHTNESS      0xC3
#define VFD_SETVFDTIME         0xC4
#define VFD_SETLED             0xC5
#define VFD_SETLEDSLOW         0xC6
#define VFD_SETLEDFAST         0xC7
#define VFD_SETSHUTDOWN        0xC8
#define VFD_SETRAM             0xC9
#define VFD_SETTIME            0xCA
#define VFD_SETCB              0xCB
#define VFD_SETFANON           0xCC
#define VFD_SETFANOFF          0xCD
#define VFD_SETRFMODULATORON   0xCE
#define VFD_SETRFMODULATOROFF  0xCF
#define VFD_SETCHAR            0xD0
#define VFD_SETDISPLAYTEXT     0xD1
#define VFD_SETD2              0xD2
#define VFD_SETD3              0xD3
#define VFD_SETD4              0xD4
#define VFD_SETCLEARTEXT       0xD5
#define VFD_SETD6              0xD6
#define VFD_SETMODETIME        0xD7
#define VFD_SETSEGMENTI        0xD8
#define VFD_SETSEGMENTII       0xD9
#define VFD_SETCLEARSEGMENTS   0xDA

/***************************************************************************
 *
 * Icon definitions.
 *
 ***************************************************************************/
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD) 
enum  // for 12 character dot matrix
{
	ICON_MIN = 0,    // 00
	ICON_STANDBY,    // 01
	ICON_SAT,        // 02
	ICON_REC,        // 03
	ICON_TIMESHIFT,  // 04
	ICON_TIMER,      // 05
	ICON_HD,         // 06
	ICON_USB,        // 07
	ICON_SCRAMBLED,  // 08
	ICON_DOLBY,      // 09
	ICON_MUTE,       // 10
	ICON_TUNER1,     // 11
	ICON_TUNER2,     // 12
	ICON_MP3,        // 13
	ICON_REPEAT,     // 14
	ICON_PLAY,       // 15
	ICON_Circ0,      // 16
	ICON_Circ1,      // 17
	ICON_Circ2,      // 18
	ICON_Circ3,      // 19
	ICON_Circ4,      // 20
	ICON_Circ5,      // 21
	ICON_Circ6,      // 22
	ICON_Circ7,      // 23
	ICON_Circ8,      // 24
	ICON_FILE,       // 25
	ICON_TER,        // 26
	ICON_480i,       // 27
	ICON_480p,       // 28
	ICON_576i,       // 29
	ICON_576p,       // 30
	ICON_720p,       // 31
	ICON_1080i,      // 32
	ICON_1080p,      // 33
//	ICON_COLON1,     // (34)
//	ICON_COLON2,     // (35)
//	ICON_COLON3,     // (36)
	ICON_TV,         // 34 (37)
	ICON_RADIO,      // 35 (38)
	ICON_MAX,        // 36 (39)
	ICON_SPINNER     // 37 (40)
};

#elif defined(CUBEREVO_MINI) \
 ||   defined(CUBEREVO_MINI2) \
 ||   defined(CUBEREVO_2000HD) \
 ||   defined(CUBEREVO_3000HD)
enum  // for 14 character dot matrix
{
	ICON_MIN = 0,  // 0
	ICON_REC,
	ICON_TIMER,  // 2
	ICON_TIMESHIFT,
	ICON_PLAY,  // 4
	ICON_PAUSE,
	ICON_HD,  // 6
	ICON_DOLBY,
	ICON_MAX  // 8
};
#else
enum  // for LED models
{
	ICON_MIN = 0,  // 0
	ICON_MAX = 0
};
#endif

struct iconToInternal
{
	char *name;
	u16 icon;
	u8 codemsb;
	u8 codelsb;
	u8 sgm;
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD) 
	u8 codemsb2;
	u8 codelsb2;
	u8 sgm2;
#endif
};

struct set_test_s
{
	unsigned char data[14];
};

struct set_brightness_s
{
	int level;
};

struct set_led_s
{
	int led_nr;
	int on;
};

struct set_on_off_s
{
	int on;
};

struct set_icon_s
{
	int icon_nr;
	int on;
};

/* YMDhms */
struct get_time_s
{
	char time[6];
};

/* YYMMDDhhmm */
struct set_standby_s
{
	char time[10];
};

/* YYMMDDhhmmss */
struct set_time_s
{
	char time[12];
};

struct get_wakeupstatus_s
{
	char status;
};

/* this sets the mode temporarily (for one IOCTL)
 * to the desired mode. currently the "normal" mode
 * is the compatible vfd mode
 */
struct set_mode_s
{
	int compat; /* 0 = compatibility mode to vfd driver; 1 = micom mode */
};

/* 0 = 12dot 12seg, 1 = 13grid, 2 = 12 dot 14seg, 3 = 7seg */
struct get_version_s
{
	int version;
};

struct set_time_mode_s
{
	int twentyFour;
};

struct micom_ioctl_data
{
	union
	{
		struct set_test_s test;
		struct set_icon_s icon;
		struct set_led_s led;
		struct set_on_off_s fan;
		struct set_on_off_s rf;
		struct set_brightness_s brightness;
		struct set_mode_s mode;
		struct set_standby_s standby;
		struct set_time_s time;
		struct get_time_s get_time;
		struct get_wakeupstatus_s status;
		struct get_time_s get_wakeup_time;
		struct set_standby_s wakeup_time;
		struct get_version_s version;
		struct set_on_off_s display_time;
		struct set_time_mode_s time_mode;
	} u;
};

struct vfd_ioctl_data
{
	unsigned char start;
	unsigned char data[64];
	unsigned char length;
};

struct saved_data_s
{
	int  length;  // length of last string written to fp display
	char data[128];  // last string written to fp display
//	int  brightness;  // display on/off seems to retain previous brigtness
	int  display_on;
	int  time_mode;
#if !defined(CUBEREVO_250HD) \
 && !defined(CUBEREVO_MINI_FTA)
	int  fan;  // fan state
	int  icon_state[ICON_MAX + 2];
#endif
};

#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
typedef struct
{
	int state;
	int period;
	int status;
	int enable;
	struct task_struct *task;
	struct semaphore sem;
} tSpinnerState;
#endif

void dumpValues(void);
extern int micom_init_func(void);
extern void copyData(unsigned char *data, int len);
extern void getRCData(unsigned char *data, int *len);
extern int micomWriteCommand(char *buffer, int len, int needAck);

extern int errorOccured;
extern char *gmt_offset;  // module param, string
extern int rtc_offset;

/* number of display characters */
extern int front_seg_num;

extern struct file_operations vfd_fops;

extern struct saved_data_s lastdata;

#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
extern tSpinnerState spinner_state;
extern struct iconToInternal micomIcons[];
#endif

#endif  // _cuberevo_micom_h
// vim:ts=4
