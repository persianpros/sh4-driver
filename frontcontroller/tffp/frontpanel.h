#ifndef FRONTPANEL_H
#define FRONTPANEL_H
/*
  This device driver uses the following device entries:

  /dev/fpc      c 62 0      # FP control (read/write, for clock- and timer operations, shutdown etc.)
  /dev/rc       c 62 1      # Remote Control (read-only)
  /dev/fplarge  c 62 2      # Direct access to the 8 character display (write-only)
  /dev/fpsmall  c 62 3      # Direct access to the 4 character display (write-only)
 */

#include "tftypes.h"

#define FRONTPANEL_MAJOR		62    /* experimental major number */
#define FRONTPANEL_MINOR_FPC            0
#define FRONTPANEL_MINOR_RC             1
#define FRONTPANEL_MINOR_FPLARGE        2
#define FRONTPANEL_MINOR_FPSMALL        3

/* RWSS SSSS SSSS SSSS KKKK KKKK NNNN NNNN
   R = read
   W = write
   S = parameter size
   K = ioctl magic = 0x3a
   N = command number
*/
/* IOCTL definitions for tffpctl */
#define FRONTPANELGETTIME               0x40003a00 | (sizeof(frontpanel_ioctl_time) << 16)
#define FRONTPANELSETTIME               0x40003a01
#define FRONTPANELSYNCSYSTIME           0x40003a02
#define FRONTPANELCLEARTIMER            0x40003a03
#define FRONTPANELSETTIMER              0x40003a04
#define FRONTPANELBRIGHTNESS            0x40003a05 | (sizeof(frontpanel_ioctl_brightness) << 16)
#define FRONTPANELIRFILTER1             0x40003a06
#define FRONTPANELIRFILTER2             0x40003a07
#define FRONTPANELIRFILTER3             0x40003a08
#define FRONTPANELIRFILTER4             0x40003a09
#define FRONTPANELPOWEROFF              0x40003a0a
#define FRONTPANELBOOTREASON            0x40003a0b | (sizeof(frontpanel_ioctl_bootreason) << 16)
#define FRONTPANELCLEAR                 0x40003a0c
#define FRONTPANELTYPEMATICDELAY        0x40003a0d | (sizeof(frontpanel_ioctl_typematicdelay) << 16)
#define FRONTPANELTYPEMATICRATE         0x40003a0e | (sizeof(frontpanel_ioctl_typematicrate) << 16)
#define FRONTPANELKEYEMULATION          0x40003a0f | (sizeof(frontpanel_ioctl_keyemulation) << 16)
#define FRONTPANELREBOOT                0x40003a10
#define FRONTPANELSYNCFPTIME            0x40003a11
#define FRONTPANELSETGMTOFFSET          0x40003a12
#define FRONTPANELRESEND                0x40003a13
#define FRONTPANELALLCAPS               0x40003a14 | (sizeof(frontpanel_ioctl_allcaps) << 16)
#define FRONTPANELSCROLLMODE            0x40003a15 | (sizeof(frontpanel_ioctl_scrollmode) << 16)
#define FRONTPANELICON                  0x40003a20 | (sizeof(frontpanel_ioctl_icons) << 16)
#define FRONTPANELSPINNER               0x40003a21 | (sizeof(frontpanel_ioctl_spinner) << 16)

/* IOCTL command definitions for tffpctl */
#define _FRONTPANELGETTIME              0x00
#define _FRONTPANELSETTIME              0x01
#define _FRONTPANELSYNCTIME             0x02
#define _FRONTPANELCLEARTIMER           0x03
#define _FRONTPANELSETTIMER             0x04
#define _FRONTPANELBRIGHTNESS           0x05
#define _FRONTPANELIRFILTER1            0x06
#define _FRONTPANELIRFILTER2            0x07
#define _FRONTPANELIRFILTER3            0x08
#define _FRONTPANELIRFILTER4            0x09
#define _FRONTPANELPOWEROFF             0x0a
#define _FRONTPANELBOOTREASON           0x0b
#define _FRONTPANELCLEAR                0x0c
#define _FRONTPANELTYPEMATICDELAY       0x0d
#define _FRONTPANELTYPEMATICRATE        0x0e
#define _FRONTPANELKEYEMULATION         0x0f
#define _FRONTPANELREBOOT               0x10
#define _FRONTPANELRESEND               0x13
#define _FRONTPANELALLCAPS              0x14
#define _FRONTPANELSCROLLMODE           0x15
#define _FRONTPANELICON                 0x20
#define _FRONTPANELSPINNER              0x21

/* IOCTL definitions for duckbox enigma2 */
#define VFDMAGIC                        0xc0425a00
#define VFDDISPLAYCHARS                 0xc0425a00
#define VFDBRIGHTNESS                   0xc0425a03
#define VFDDISPLAYWRITEONOFF            0xc0425a05
#define VFDICONDISPLAYONOFF             0xc0425a0a
#define VFDPOWEROFF                     0xc0425af5
#define VFDSETPOWERONTIME               0xc0425af6
#define VFDGETWAKEUPMODE                0xc0425af9
#define VFDGETTIME                      0xc0425afa
#define VFDSETTIME                      0xc0425afb
#define VFDSTANDBY                      0xc0425afc
#define VFDREBOOT                       0xc0425afd
//#define VFDSETTIME2                     0xc0425afd //aotom has this
#define VFDDISPLAYCLR                   0xc0425b00
#define VFDGETWAKEUPTIME                0xc0425b03

#define STASC1IRQ                       120
#define BUFFERSIZE                      256     //must be 2 ^ n

#define KEYEMUTF7700                    0
#define KEYEMUUFS910                    1
#define KEYEMUTF7700LKP                 2

//The following icons belong to block 1
#define FPICON_IRDOT                    0x00000001
#define FPICON_POWER                    0x00000002
#define FPICON_COLON                    0x00000004
#define FPICON_AM                       0x00000008
#define FPICON_PM                       0x00000010
#define FPICON_TIMER                    0x00000020
#define FPICON_AC3                      0x00000040
#define FPICON_TIMESHIFT                0x00000080
#define FPICON_TV                       0x00000100
#define FPICON_MUSIC                    0x00000200
#define FPICON_DISH                     0x00000400
#define FPICON_MP3                      0x00000800
#define FPICON_TUNER1                   0x00001000
#define FPICON_TUNER2                   0x00002000
#define FPICON_REC                      0x00004000
#define FPICON_MUTE                     0x00008000
#define FPICON_PAUSE                    0x00010000
#define FPICON_FWD                      0x00020000
#define FPICON_RWD                      0x00040000
#define FPICON_xxx2                     0x00080000
#define FPICON_PLAY                     0x00100000
#define FPICON_xxx4                     0x00200000
#define FPICON_NETWORK                  0x00400000
#define FPICON_DOLBY                    0x00800000
#define FPICON_ATTN                     0x01000000
#define FPICON_DOLLAR                   0x02000000
#define FPICON_AUTOREWRIGHT             0x04000000
#define FPICON_AUTOREWLEFT              0x08000000

//The following icons belong to block 2
#define FPICON_CDCENTER                 0x00000001
#define FPICON_CD1                      0x00000002
#define FPICON_CD2                      0x00000004
#define FPICON_CD3                      0x00000008
#define FPICON_CD4                      0x00000010
#define FPICON_CD5                      0x00000020
#define FPICON_CD6                      0x00000040
#define FPICON_CD7                      0x00000080
#define FPICON_CD8                      0x00000100
#define FPICON_CD9                      0x00000200
#define FPICON_CD10                     0x00000400
#define FPICON_CD11                     0x00000800
#define FPICON_CD12                     0x00001000

#define FPICON_HDD                      0x00002000
#define FPICON_HDDFRAME                 0x00004000
#define FPICON_HDD1                     0x00008000
#define FPICON_HDD2                     0x00010000
#define FPICON_HDD3                     0x00020000
#define FPICON_HDD4                     0x00040000
#define FPICON_HDD5                     0x00080000
#define FPICON_HDD6                     0x00100000
#define FPICON_HDD7                     0x00200000
#define FPICON_HDD8                     0x00400000
#define FPICON_HDDFULL                  0x00800000


typedef struct
{
	word  year;
	word  month;
	word  day;
	word  dow;
	char  sdow[4];
	word  hour;
	word  min;
	word  sec;
	dword now;
} frontpanel_ioctl_time;

typedef struct
{
	byte onoff;
} frontpanel_ioctl_displayonoff;

typedef struct
{
	byte bright;
} frontpanel_ioctl_brightness;
typedef struct
{
	byte reason;
} frontpanel_ioctl_bootreason;

typedef struct
{
	byte onoff;
} frontpanel_ioctl_irfilter;

typedef struct
{
	dword Icons1;
	dword Icons2;
	byte  BlinkMode;
} frontpanel_ioctl_icons;

typedef struct
{
	byte TypematicDelay;
} frontpanel_ioctl_typematicdelay;

typedef struct
{
	byte TypematicRate;
} frontpanel_ioctl_typematicrate;

typedef struct
{
	byte KeyEmulation;
} frontpanel_ioctl_keyemulation;

typedef struct
{
	byte AllCaps;
} frontpanel_ioctl_allcaps;

typedef struct
{
	byte Spinner;
} frontpanel_ioctl_spinner;

typedef struct
{
	byte ScrollMode;
	byte ScrollPause;
	byte ScrollDelay;
} frontpanel_ioctl_scrollmode;

typedef enum
{
	/* Common legacy icons (not all of them present): */
	VFD_ICON_HD = 0x01,
	VFD_ICON_HDD,
	VFD_ICON_LOCK,
	VFD_ICON_BT,
	VFD_ICON_MP3,
	VFD_ICON_MUSIC,
	VFD_ICON_DD,
	VFD_ICON_MAIL,
	VFD_ICON_MUTE,
	VFD_ICON_PLAY,
	VFD_ICON_PAUSE,
	VFD_ICON_FF,
	VFD_ICON_FR,
	VFD_ICON_REC,
	VFD_ICON_CLOCK,
	/* Additional unique TF77X0 icons
       The CD icons: */
	VFD_ICON_CD1 = 0x20,
	VFD_ICON_CD2,
	VFD_ICON_CD3,
	VFD_ICON_CD4,
	VFD_ICON_CD5,
	VFD_ICON_CD6,
	VFD_ICON_CD7,
	VFD_ICON_CD8,
	VFD_ICON_CD9,
	VFD_ICON_CD10,
	VFD_ICON_CDCENTER = 0x2f,
	/* The HDD level display icons: */
	VFD_ICON_HDD1 = 0x30,
	VFD_ICON_HDD_1,
	VFD_ICON_HDD_2,
	VFD_ICON_HDD_3,
	VFD_ICON_HDD_4,
	VFD_ICON_HDD_5,
	VFD_ICON_HDD_6,
	VFD_ICON_HDD_7,
	VFD_ICON_HDD_8,
	VFD_ICON_HDD_FRAME,
	VFD_ICON_HDD_FULL = 0x3a,
	/* The remaining TF77X0 icons,
	   approximately in display order: */
	VFD_ICON_MP3_2 = 0x40,
	VFD_ICON_AC3,
	VFD_ICON_TIMESHIFT,
	VFD_ICON_TV,
	VFD_ICON_RADIO,
	VFD_ICON_SAT,
	VFD_ICON_REC2,
	VFD_ICON_RECONE,
	VFD_ICON_RECTWO,
	VFD_ICON_REWIND,
	VFD_ICON_STEPBACK,
	VFD_ICON_PLAY2,
	VFD_ICON_STEPFWD,
	VFD_ICON_FASTFWD,
	VFD_ICON_PAUSE2,
	VFD_ICON_MUTE2,
	VFD_ICON_REPEATL,
	VFD_ICON_REPEATR,
	VFD_ICON_DOLLAR,
	VFD_ICON_ATTN,
	VFD_ICON_DOLBY,
	VFD_ICON_NETWORK, 
	VFD_ICON_AM,
	VFD_ICON_TIMER,
	VFD_ICON_PM,
	VFD_ICON_DOT,
	VFD_ICON_POWER,
	VFD_ICON_COLON,
	VFD_ICON_SPINNER, //(0x5c)
	VFD_ICON_ALL = 0x7f
} VFD_ICON;


/* These are used by the generic ioctl routines */
struct set_brightness_s
{
	int level;
};

struct set_icon_s
{
	int icon_nr;
	int on;
};

struct set_light_s
{
	int onoff;
};

struct set_display_time_s
{
	int on;
};

/* time must be given as follows:
 * time[0] & time[1] = mjd ???
 * time[2] = hour
 * time[3] = min
 * time[4] = sec
 */
//struct set_standby_s
//{
//	char time[5];
//};

struct set_time_s
{
	char time[5];
};

struct tffp_ioctl_data
{
	union
	{
		struct set_icon_s icon;
		struct set_brightness_s brightness;
		struct set_light_s light;
		struct set_display_time_s display_time;
//		struct set_mode_s mode;
//		struct set_standby_s standby;
		struct set_time_s time;
	} u;
};

struct vfd_ioctl_data
{
	unsigned char start_address;
	unsigned char data[64];
	unsigned char length;
};

//Global spinner variables
static int Spinner_on;
static int Spinner_state;
static int Segment_flag[11];

extern void vfdSetGmtWakeupTime(time_t time);
extern void vfdSetGmtTime(time_t time);
extern time_t vfdGetGmtTime(void);
extern time_t vfdGetLocalTime(void);
extern int getBootReason(void);

#endif
