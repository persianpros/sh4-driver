#ifndef _123_micom
#define _123_micom

extern short paramDebug;

#define TAGDEBUG "[micom] "

#ifndef dprintk
#define dprintk(level, x...) do { \
		if ((paramDebug) && (paramDebug >= level)) printk(TAGDEBUG x); \
	} while (0)
#endif

extern int micom_init_func(void);
extern void copyData(unsigned char *data, int len);
extern void getRCData(unsigned char *data, int *len);
void dumpValues(void);

extern int errorOccured;
extern char ioctl_data[8];

extern struct file_operations vfd_fops;

typedef struct
{
	struct file      *fp;
	int              read;
	struct semaphore sem;
} tFrontPanelOpen;

#define FRONTPANEL_MINOR_RC  1
#define LASTMINOR            2

extern tFrontPanelOpen FrontPanelOpen[LASTMINOR];

#define VFD_MAJOR            147

/* ioctl numbers ->hacky */
#define VFDBRIGHTNESS        0xc0425a03
#define VFDDRIVERINIT        0xc0425a08
#define VFDICONDISPLAYONOFF  0xc0425a0a
#define VFDDISPLAYWRITEONOFF 0xc0425a05
#define VFDDISPLAYCHARS      0xc0425a00

//#define VFDSETPOWERONTIME    0xc0425af6
#define VFDSETRCCODE         0xc0425af6
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

/* this setups the mode temporarily (for one ioctl)
 * to the desired mode. currently the "normal" mode
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
	} u;
};

struct vfd_ioctl_data
{
	unsigned char start_address;
	unsigned char data[64];
	unsigned char length;
};

#if defined(UFS922) || defined(UFC960)
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

#if defined(UFS912) || defined(UFS913) || defined(UFS922) || defined(UFC960)
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

#if defined(UFS922) || defined(UFS912) || defined(UFS913) || defined(UFC969) || defined(UFH969)
#define VFD_LENGTH 16
#define VFD_CHARSIZE 1
#elif defined(UFI510) || defined(UFC960)
#define VFD_LENGTH 12
#define VFD_CHARSIZE 2
#else
#define VFD_LENGTH 16
#define VFD_CHARSIZE 1
#endif

#endif
