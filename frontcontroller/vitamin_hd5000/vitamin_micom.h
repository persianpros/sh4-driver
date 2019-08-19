#ifndef _vitamin_micom_h
#define _vitamin_micom_h
/*
 */

extern short paramDebug;

#define TAGDEBUG "[vitamin_micom] "

#define dprintk(level, x...) do \
	{ \
		if (((paramDebug) && (paramDebug >= level)) || level == 0)\
		{ \
			printk(TAGDEBUG x); \
		} \
	} \
	while (0)

extern int micom_init_func(void);
extern void copyData(unsigned char *data, int len);
extern void getRCData(unsigned char *data, int *len);
void dumpValues(void);

extern int errorOccured;

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

#define SPINNER_THREAD_STATUS_RUNNING 0
#define SPINNER_THREAD_STATUS_STOPPED 1
#define SPINNER_THREAD_STATUS_INIT    2
#define SPINNER_THREAD_STATUS_HALTED  3 // (semaphore down)
//#define SPINNER_THREAD_STATUS_PAUSED  4 // (state == 0)

/* ioctl numbers ->hacky */
#define VFDDISPLAYCHARS      0xc0425a00
#define VFDBRIGHTNESS        0xc0425a03
#define VFDDISPLAYWRITEONOFF 0xc0425a05
#define VFDDRIVERINIT        0xc0425a08
#define VFDICONDISPLAYONOFF  0xc0425a0a
// comment next line to leave VFDTEST capabilities out
#define VFDTEST              0xc0425af0  // added by audioniek
#define VFDGETVERSION        0xc0425af7
#define VFDLEDBRIGHTNESS     0xc0425af8
#define VFDGETWAKEUPMODE     0xc0425af9
#define VFDGETTIME           0xc0425afa
#define VFDSETTIME           0xc0425afb
#define VFDSTANDBY           0xc0425afc
#define VFDREBOOT            0xc0425afd
#define VFDSETLED            0xc0425afe  // Blue LED (power)
#define VFDSETMODE           0xc0425aff

#define VFDSETREDLED         0xc0425af6  // !! unique for vitamin, does not work
#define VFDSETGREENLED       0xc0425af5  // !! unique for vitamin, does not work
#define VFDDISPLAYCLR        0xc0425b00

#define	NO_ICON	             0x0000

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
	int on;
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

/* YMDhms */
struct set_time_s
{
	char time[6];
};

struct get_wakeupstatus_s
{
	char status;
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
		struct set_test_s test;
		struct set_icon_s icon;
		struct set_led_s led;
		struct set_brightness_s brightness;
		struct set_light_s light;
		struct set_mode_s mode;
		struct set_time_s time;
		struct get_wakeupstatus_s status;
	} u;
};

struct vfd_ioctl_data
{
	unsigned char start_address;
	unsigned char data[64];
	unsigned char length;
};

/* Vitamin stuff */

#define MC_MSG_KEYIN 0x01
#define VFD_MAX_LEN  11

#define IR_CUSTOM_HI 0x01
#define IR_CUSTOM_LO 0xFE
#define IR_POWER     0x5F
#define FRONT_POWER  0x00

#define MAX_WAIT     10

/********************************************************************************************************************************************
 *
 * List of possible commands to the front processor
 *
 * Length is opcode plus parameters
 *
 *      Command name  Opcode
 */
enum MICOM_CLA_E
{
	CLASS_BOOTCHECK = 0x01,  /* loader only */
	CLASS_MCTX      = 0x02,  // TX on/off (cmd)
	CLASS_0X05      = 0x05,  // get version (cmd)
	CLASS_KEY       = 0x10,  // response on front panel key
	CLASS_IR        = 0x20,  // response on remote control
	CLASS_CLOCK     = 0x30,  // set time (time + day) (cmd?)
	CLASS_0X39      = 0x39,  // get time (cmd?)
	CLASS_CLOCKHI   = 0x40,  // set time (century, year, month) (cmd?)
	CLASS_0X43      = 0x43,  // get wake up mode
	CLASS_STATE     = 0x50,  // not used?
	CLASS_POWER     = 0x60,  // power control (cmd)
	CLASS_RESET     = 0x70,  // not used?
	CLASS_WDT       = 0x80,  // watch dog on/off (cmd)
	CLASS_MUTE      = 0x90,  // mute request on/off (cmd)
	CLASS_STATE_REQ = 0xA0,  // not used?
	CLASS_TIME      = 0xB0,  // not used?
	CLASS_DISPLAY   = 0xC0,  // not used, 4 chars max.
	CLASS_FIX_IR    = 0xD0,  // set IR format (cmd)
	CLASS_BLOCK     = 0xE0,  // Tx Data Blocking on/off (cmd)
	CLASS_ON_TIME   = 0xF0,  // set wake up time?
	CLASS_FIGURE    = 0xF1,  /* VFD ICON */
	CLASS_FIGURE2   = 0xF2,  /* VFD ICON */
	CLASS_LED       = 0xF3,  // LED brightness (cmd)?
	CLASS_SENSOR    = 0xF4,  // not used?
	CLASS_DISPLAY2  = 0xF5,  // display text (cmd)
	CLASS_INTENSITY = 0xF6,  /* VFD BRIGHTNESS */
	CLASS_DISP_ALL  = 0xF7,  // not used?
	CLASS_WRITEFONT = 0xF8,  // not used?
	CLASS_IR_LED    = 0xF9,  /* control IR LED */
	CLASS_CIRCLE    = 0xFA,  // icon: circle class
	CLASS_BLUELED   = 0xFB,  // control LEDs (cmd)
	CLASS_ECHO      = 0xFF,  /* for debug */
};

enum
{
	POWER_DOWN = 0,
	POWER_UP,
	POWER_SLEEP,
	POWER_RESET,
};  // power down (CLASS_POWER) options

enum
{
	RESET_0 = 0,
	RESET_POWER_ON,
};  // wake up reasons

/***************************************************************************
 *
 * Icon definitions.
 *
 ***************************************************************************/
enum
{
	ICON_MIN = 0x01,
	ICON_TV = ICON_MIN,
	ICON_RADIO,
	ICON_REPEAT,
	ICON_CIRCLE,
	ICON_RECORD,
	ICON_POWER,
	ICON_PWR_CIRC,
	ICON_DOLBY,
	ICON_MUTE,
	ICON_DOLLAR,
	ICON_LOCK,
	ICON_CLOCK,
	ICON_HD,
	ICON_DVD,
	ICON_MP3,
	ICON_NET,
	ICON_USB,
	ICON_MC,
	ICON_ALL, // 19
	ICON_MAX = ICON_ALL
};

extern int rtc_offset;
#endif
// vim:ts=4
