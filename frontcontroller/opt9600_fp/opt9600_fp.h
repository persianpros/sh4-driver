#ifndef _opt9600_fp_h
#define _opt9600_fp_h
/*
 */

extern short paramDebug;  // debug print level is zero as default (0=nothing, 1= errors, 10=some detail, 20=more detail, 50=open/close functions, 100=all)
#define TAGDEBUG "[opt9600_fp] "
#define dprintk(level, x...) \
do \
{ \
	if ((paramDebug) && (paramDebug >= level) || level == 0) \
	printk(TAGDEBUG x); \
} while (0)

extern int mcom_init_func(void);
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

#define FRONTPANEL_MINOR_RC   1
#define LASTMINOR             2

extern tFrontPanelOpen FrontPanelOpen[LASTMINOR];

#define VFD_MAJOR            147

/* ioctl numbers ->hacky */
#define VFDDRIVERINIT         0xc0425a08
#define VFDDISPLAYWRITEONOFF  0xc0425a05
#define VFDDISPLAYCHARS       0xc0425a00

#define VFDGETWAKEUPMODE      0xc0425af9
#define VFDGETTIME            0xc0425afa
#define VFDSETTIME            0xc0425afb
#define VFDSTANDBY            0xc0425afc

#define ACK_WAIT_TIME msecs_to_jiffies(1000)

#define cPackageSizeStandby  5
#define cPackageSizeKeyIn    3

#define cGetTimeSize         11
#define cGetVersionSize      6
#define cGetWakeupReasonSize 10
#define cGetResponseSize     3

#define cMinimumSize         3

#define _MCU_KEYIN           0xF1
#define _MCU_DISPLAY         0xED
#define _MCU_STANDBY         0xE5
#define _MCU_NOWTIME         0xEC
#define _MCU_WAKEUP          0xEA
#define _MCU_BOOT            0xEB
#define _MCU_TIME            0xFC
#define _MCU_RESPONSE        0xC5
#define _MCU_ICON            0xE1
#define _MCU_VERSION         0xDB
#define _MCU_REBOOT          0xE6
#define _MCU_WAKEUPREASON    0xE7
#define _MCU_PRIVATE         0xE7
#define _MCU_KEYCODE         0xE9

#define _TAG                 0
#define _LEN                 1
#define _VAL                 2

/* time must be given as follows:
 * time[0] & time[1] = mjd ???
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
	time_t localTime;
};

struct opt9600_fp_ioctl_data
{
	union
	{
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

#endif  // _opt9600_fp_h
// vim:ts=4
