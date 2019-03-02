#ifndef _cuberevo_micom_h
#define _cuberevo_micom_h
/*
 */

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
extern char *gmt_offset;  // module param, string
extern int rtc_offset;
/* number of display characters */
extern int front_seg_num;

extern struct file_operations vfd_fops;

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

/* IOCTL numbers -> hacky! */
#define VFDDISPLAYCHARS      0xc0425a00
#define VFDBRIGHTNESS        0xc0425a03
#define VFDPWRLED            0xc0425a04 /* obsolete, use VFDSETLED (0xc0425afe) */
#define VFDDISPLAYWRITEONOFF 0xc0425a05
#define VFDDRIVERINIT        0xc0425a08
#define VFDICONDISPLAYONOFF  0xc0425a0a
// comment next line to leave VFDTEST capabilities out
#define VFDTEST              0xc0425af0 // added by audioniek
#define VFDCLEARICONS	     0xc0425af6
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
#define VFDLEDBRIGHTNESS     0xc0425b05 /* Cuberevo/micom specific */

// comment next line if you want left aligned text display
#define CENTERED_DISPLAY

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

#define LEAPYEAR(year) (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year) (LEAPYEAR(year) ? 366 : 365)
static const int _ytab[2][12] =
{
	{ 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
	{ 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};

#endif
// vim:ts=4
