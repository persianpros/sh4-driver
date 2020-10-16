#ifndef __AOTOM_MAIN_H__
#define __AOTOM_MAIN_H__

#ifndef __KERNEL__
typedef unsigned char u8;

typedef unsigned short u16;

typedef unsigned int u32;
#endif

extern short paramDebug;  // debug print level is zero as default (0=nothing, 1= errors, 10=some detail, 20=more detail, 50=open/close functions, 100=all)
#define TAGDEBUG "[aotom_vip] "
#define dprintk(level, x...) \
do \
{ \
	if ((paramDebug) && (paramDebug >= level) || level == 0) \
	printk(TAGDEBUG x); \
} while (0)

#define YWPANEL_MAX_LED_LENGTH        4
#define YWPANEL_MAX_VFD_LENGTH        8  //note: this is the length of the text part
#define YWPANEL_MAX_DVFD_LENGTH10     10 //note: this is the length when clock is on
#define YWPANEL_MAX_DVFD_LENGTH16     16 //note: this is the length when clock is off
#define LOG_OFF                       0
#define LOG_ON                        1
#define LED_RED                       0
#define LASTLED                       1
/* Uncomment next line to enable lower case letters on VFD */
//#define VFD_LOWER_CASE                1

/* Uncomment next line to enable code for LED models */
//#define FP_LEDS                       1

/* Uncomment next line to enable code for DVFD models */
//#define FP_DVFD                       1

#define VFDDISPLAYCHARS               0xc0425a00
#define VFDBRIGHTNESS                 0xc0425a03
#define VFDDISPLAYWRITEONOFF          0xc0425a05
#define VFDDRIVERINIT                 0xc0425a08
#define VFDICONDISPLAYONOFF           0xc0425a0a

#define VFDGETBLUEKEY                 0xc0425af1
#define VFDSETBLUEKEY                 0xc0425af2
#define VFDGETSTBYKEY                 0xc0425af3
#define VFDSETSTBYKEY                 0xc0425af4
#define VFDPOWEROFF                   0xc0425af5
#define VFDSETPOWERONTIME             0xc0425af6
#define VFDGETVERSION                 0xc0425af7
#define VFDGETSTARTUPSTATE            0xc0425af8
#define VFDGETWAKEUPMODE              0xc0425af9
#define VFDGETTIME                    0xc0425afa
#define VFDSETTIME                    0xc0425afb
#define VFDSTANDBY                    0xc0425afc
#define VFDSETTIME2                   0xc0425afd  // seife, set 'complete' time...
#define VFDSETLED                     0xc0425afe
#define VFDSETMODE                    0xc0425aff
#define VFDDISPLAYCLR                 0xc0425b00
#define VFDGETLOOPSTATE               0xc0425b01
#define VFDSETLOOPSTATE               0xc0425b02
#define VFDGETWAKEUPTIME              0xc0425b03  // added by Audioniek
#define VFDSETDISPLAYTIME             0xc0425b04  // added by Audioniek (Cuberevo uses 0xc0425b02)
#define VFDGETDISPLAYTIME             0xc0425b05  // added by Audioniek

#define INVALID_KEY                   -1
#define VFD_MAJOR                     147

#define	I2C_BUS_NUM                   2
#define	I2C_BUS_ADD                   (0x50 >> 1)  // this is important, not 0x50

#define YWPANEL_FP_INFO_MAX_LENGTH    10
#define YWPANEL_FP_DATA_MAX_LENGTH    38

#define YWPANEL_KEYBOARD

#define REMOTE_SLAVE_ADDRESS          0x40bd0000  /* slave address is 5 */
#define REMOTE_SLAVE_ADDRESS_NEW      0xc03f0000  /* sz 2008-06-26 add new remote*/
#define REMOTE_SLAVE_ADDRESS_EDISION1 0x22dd0000
#define REMOTE_SLAVE_ADDRESS_EDISION2 0XCC330000
#define REMOTE_SLAVE_ADDRESS_GOLDEN   0x48b70000  /* slave address is 5 */
#define REMOTE_TOPFIELD_MASK          0x4fb0000

typedef unsigned int YWOS_ClockMsec;

typedef struct YWPANEL_I2CData_s
{
	u8 writeBuffLen;
	u8 writeBuff[YWPANEL_FP_DATA_MAX_LENGTH];
	u8 readBuff[YWPANEL_FP_INFO_MAX_LENGTH];

} YWPANEL_I2CData_t;

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
struct set_standby_s
{
	char time[5];
};

struct set_time_s
{
	char time[5];
};

struct set_key_s
{
	int key_nr;
	u32 key;
};

/* This changes the mode temporarily (for one IOCTL)
 * to the desired mode. Currently the "normal" mode
 * is the compatible VFD mode
 */
struct set_mode_s
{
	int compat; /* 0 = compatibility mode to VFD driver; 1 = nuvoton mode */
};

struct aotom_ioctl_data
{
	union
	{
		struct set_icon_s icon;
		struct set_brightness_s brightness;
		struct set_led_s led;
		struct set_light_s light;
		struct set_display_time_s display_time;
		struct set_mode_s mode;
		struct set_standby_s standby;
		struct set_time_s time;
		struct set_key_s key;
	} u;
};

struct vfd_ioctl_data
{
	unsigned char start_address;
	unsigned char data[64];
	unsigned char length;
};

#if 1  // key definitions for remote controls
enum
{
	KEY_DIGIT0 = 11,
	KEY_DIGIT1 = 2,
	KEY_DIGIT2 = 3,
	KEY_DIGIT3 = 4,
	KEY_DIGIT4 = 5,
	KEY_DIGIT5 = 6,
	KEY_DIGIT6 = 7,
	KEY_DIGIT7 = 8,
	KEY_DIGIT8 = 9,
	KEY_DIGIT9 = 10
};

enum
{
	POWER_KEY        = 88,

	TIME_SET_KEY     = 87,
	UHF_KEY          = 68,
	VFormat_KEY      = 67,
	MUTE_KEY         = 66,

	TVSAT_KEY        = 65,
	MUSIC_KEY        = 64,
	FIND_KEY         = 63,
	FAV_KEY          = 62,

	MENU_KEY         = 102, // HOME
	i_KEY            = 61,
	EPG_KEY          = 18,
	EXIT_KEY         = 48,	// B
	RECALL_KEY       = 30,
	RECORD_KEY       = 19,

	UP_KEY           = 103, // UP
	DOWN_KEY         = 108, // DOWN
	LEFT_KEY         = 105, // LEFT
	RIGHT_KEY        = 106, // RIGHT
	SELECT_KEY       = 28,  // ENTER

	PLAY_KEY         = 25,
	PAGE_UP_KEY      = 104, // P_UP
	PAUSE_KEY        = 22,
	PAGE_DOWN_KEY    = 109, // P_DOWN

	STOP_KEY         = 20,
	SLOW_MOTION_KEY  = 50,
	FAST_REWIND_KEY  = 33,
	FAST_FORWARD_KEY = 49,

	DOCMENT_KEY      = 32,
	SWITCH_PIC_KEY   = 17,
	PALY_MODE_KEY    = 24,
	USB_KEY          = 111,

	RADIO_KEY        = 110,
	SAT_KEY          = 15,
	F1_KEY           = 59,
	F2_KEY           = 60,

	RED_KEY          = 44,  // Z
	GREEN_KEY        = 45,  // X
	YELLOW_KEY       = 46,  // C
	BLUE_KEY         = 47   // V
};
#endif

// Icon names for VFD
typedef enum LogNum_e
{
/*----------------------------------11G-------------------------------------*/
	AOTOM_PLAY_FASTBACKWARD = 11 * 16 + 1,  // Icon 1
	AOTOM_PLAY_PREV,
	AOTOM_PLAY_HEAD = AOTOM_PLAY_PREV,  // deprecated
	AOTOM_PLAY,
	AOTOM_PLAY_LOG = AOTOM_PLAY,        // deprecated
	AOTOM_NEXT,
	AOTOM_PLAY_TAIL = AOTOM_NEXT,       // deprecated
	AOTOM_PLAY_FASTFORWARD,
	AOTOM_PLAY_PAUSE,
	AOTOM_REC1,
	AOTOM_MUTE,
	AOTOM_LOOP,
	AOTOM_CYCLE = AOTOM_LOOP,           // deprecated
	AOTOM_DOLBYDIGITAL,
	AOTOM_DUBI = AOTOM_DOLBYDIGITAL,    // deprecated
	AOTOM_CA,
	AOTOM_CI,
	AOTOM_USB,
	AOTOM_DOUBLESCREEN,
	AOTOM_REC2,
/*----------------------------------12G-------------------------------------*/
	AOTOM_HDD_A8 = 12 * 16 + 1,
	AOTOM_HDD_A7,
	AOTOM_HDD_A6,
	AOTOM_HDD_A5,
	AOTOM_HDD_A4,
	AOTOM_HDD_A3,
	AOTOM_HDD_FULL,
	AOTOM_HDD_A2,
	AOTOM_HDD_A1,
	AOTOM_MP3,
	AOTOM_AC3,
	AOTOM_TV,
	AOTOM_TVMODE_LOG = AOTOM_TV,        // deprecated
	AOTOM_AUDIO,
	AOTOM_ALERT,
	AOTOM_HDD_FRAME,
	AOTOM_HDD_A9 = AOTOM_HDD_FRAME,     // deprecated
/*----------------------------------13G-------------------------------------*/
	AOTOM_CLOCK_PM = 13 * 16 + 1,
	AOTOM_CLOCK_AM,
	AOTOM_CLOCK,
	AOTOM_TIME_SECOND,
	AOTOM_DOT2,
	AOTOM_STANDBY,
	AOTOM_TERRESTRIAL,
	AOTOM_TER = AOTOM_TERRESTRIAL,      // deprecated
	AOTOM_DISK_S3,
	AOTOM_DISK_S2,
	AOTOM_DISK_S1,
	AOTOM_DISK_CIRCLE,
	AOTOM_DISK_S0 = AOTOM_DISK_CIRCLE,  // deprecated
	AOTOM_SATELLITE,
	AOTOM_SAT = AOTOM_SATELLITE,        // deprecated
	AOTOM_TIMESHIFT,
	AOTOM_DOT1,
	AOTOM_CABLE,
	AOTOM_CAB = AOTOM_CABLE,            // deprecated
	AOTOM_LAST = AOTOM_CABLE,
	AOTOM_ALL,
	VICON_LAST = 13 * 16 + 15,
/*----------------------------------End-------------------------------------*/
	LogNum_Max,
} LogNum_t;

enum
{  // Icon names for VFD
/*------------------------------- Simple numbering -------------------------*/
	ICON_FIRST = 1,  // Icon 1 (grid 11)
	ICON_REWIND = ICON_FIRST,
	ICON_PLAY_STEPBACK,
	ICON_PLAY_LOG,
	ICON_PLAY_STEP,
	ICON_FASTFORWARD,
	ICON_PAUSE,
	ICON_REC1,
	ICON_MUTE,
	ICON_REPEAT,
	ICON_DOLBY,
	ICON_CA,
	ICON_CI,
	ICON_USB,
	ICON_DOUBLESCREEN,
	ICON_REC2,
	ICON_HDD_A8, // Icon 16 (grid 12)
	ICON_HDD_A7,
	ICON_HDD_A6,
	ICON_HDD_A5,
	ICON_HDD_A4,
	ICON_HDD_A3,
	ICON_HDD_FULL,
	ICON_HDD_A2,
	ICON_HDD_A1,
	ICON_MP3,
	ICON_AC3,
	ICON_TVMODE,
	ICON_AUDIO,
	ICON_ALERT,
	ICON_HDD_GRID,
	ICON_CLOCK_PM, // Icon 31 (grid 13)
	ICON_CLOCK_AM,
	ICON_TIMER,
	ICON_TIME_COLON,
	ICON_DOT2,
	ICON_STANDBY,
	ICON_TER,
	ICON_DISK_S3,
	ICON_DISK_S2,
	ICON_DISK_S1,
	ICON_DISK_CIRCLE,
	ICON_SAT,
	ICON_TIMESHIFT,
	ICON_DOT1,
	ICON_CABLE,
	ICON_LAST = ICON_CABLE,
	ICON_ALL,
	ICON_SPINNER, // 47
};

typedef enum
{
	REMOTE_OLD,
	REMOTE_NEW,
	REMOTE_TOPFIELD,
	REMOTE_EDISION1,  // RC1?
	REMOTE_EDISION2,  // RC2?
	REMOTE_GOLDEN,
	REMOTE_UNKNOWN
} REMOTE_TYPE;

typedef enum FPMode_e
{
	FPWRITEMODE,
	FPREADMODE
} FPMode_t;

typedef enum SegNum_e
{
	SEGNUM1 = 0,
	SEGNUM2
} SegNum_t;

typedef struct SegAddrVal_s
{
	u8 Segaddr1;
	u8 Segaddr2;
	u8 CurrValue1;
	u8 CurrValue2;
} SegAddrVal_t;

typedef struct VFD_Format_s
{
	unsigned char LogNum;
	unsigned char LogSta;
} VFD_Format_t;

typedef struct VFD_Time_s
{
	unsigned char hour;
	unsigned char mint;
} VFD_Time_t;

typedef struct
{
	int state;  // on or off
	int period;
	int status;
	int enable;
	struct task_struct *thread_task;
} tThreadState;

typedef enum YWPANEL_DataType_e
{
	YWPANEL_DATATYPE_LBD = 0x01,
	YWPANEL_DATATYPE_LCD,
	YWPANEL_DATATYPE_LED,
	YWPANEL_DATATYPE_VFD,
	YWPANEL_DATATYPE_DVFD,
	YWPANEL_DATATYPE_SCANKEY,
	YWPANEL_DATATYPE_IRKEY,

	YWPANEL_DATATYPE_GETVERSION,
	YWPANEL_DATATYPE_GETFPSTATE,
	YWPANEL_DATATYPE_SETFPSTATE,
	YWPANEL_DATATYPE_GETCPUSTATE,
	YWPANEL_DATATYPE_SETCPUSTATE,

	YWPANEL_DATATYPE_GETSTBYKEY1,
	YWPANEL_DATATYPE_GETSTBYKEY2,
	YWPANEL_DATATYPE_GETSTBYKEY3,
	YWPANEL_DATATYPE_GETSTBYKEY4,
	YWPANEL_DATATYPE_GETSTBYKEY5,
	YWPANEL_DATATYPE_SETSTBYKEY1,
	YWPANEL_DATATYPE_SETSTBYKEY2,
	YWPANEL_DATATYPE_SETSTBYKEY3,
	YWPANEL_DATATYPE_SETSTBYKEY4,
	YWPANEL_DATATYPE_SETSTBYKEY5,
	YWPANEL_DATATYPE_GETBLUEKEY1,
	YWPANEL_DATATYPE_GETBLUEKEY2,
	YWPANEL_DATATYPE_GETBLUEKEY3,
	YWPANEL_DATATYPE_GETBLUEKEY4,
	YWPANEL_DATATYPE_GETBLUEKEY5,
	YWPANEL_DATATYPE_SETBLUEKEY1,
	YWPANEL_DATATYPE_SETBLUEKEY2,
	YWPANEL_DATATYPE_SETBLUEKEY3,
	YWPANEL_DATATYPE_SETBLUEKEY4,
	YWPANEL_DATATYPE_SETBLUEKEY5,

	YWPANEL_DATATYPE_GETIRCODE,
	YWPANEL_DATATYPE_SETIRCODE,

	YWPANEL_DATATYPE_GETENCRYPTMODE,
	YWPANEL_DATATYPE_SETENCRYPTMODE,
	YWPANEL_DATATYPE_GETENCRYPTKEY,
	YWPANEL_DATATYPE_SETENCRYPTKEY,

	YWPANEL_DATATYPE_GETVERIFYSTATE,
	YWPANEL_DATATYPE_SETVERIFYSTATE,

	YWPANEL_DATATYPE_GETTIME,
	YWPANEL_DATATYPE_SETTIME,
	YWPANEL_DATATYPE_CONTROLTIMER,

	YWPANEL_DATATYPE_SETPOWERONTIME,
	YWPANEL_DATATYPE_GETPOWERONTIME,

	YWPANEL_DATATYPE_GETFPSTANDBYSTATE,
	YWPANEL_DATATYPE_SETFPSTANDBYSTATE,

	YWPANEL_DATATYPE_GETPOWERONSTATE,  /* 0x77 */
	YWPANEL_DATATYPE_SETPOWERONSTATE,  /* 0x78 */
	YWPANEL_DATATYPE_GETSTARTUPSTATE,  /* 0x79 */
	YWPANEL_DATATYPE_SETSTARTUPSTATE,
	YWPANEL_DATATYPE_GETLOOPSTATE,     /* 0x80 */
	YWPANEL_DATATYPE_SETLOOPSTATE,     /* 0x81 */

	YWPANEL_DATATYPE_NUM
} YWPANEL_DataType_t;

typedef struct YWPANEL_LBDData_s
{
	u8 value;
} YWPANEL_LBDData_t;

typedef struct YWPANEL_LEDData_s
{
	u8 led1;
	u8 led2;
	u8 led3;
	u8 led4;
} YWPANEL_LEDData_t;

typedef struct YWPANEL_LCDData_s
{
	u8 value;
} YWPANEL_LCDData_t;

typedef enum YWPANEL_VFDDataType_e
{
	YWPANEL_VFD_SETTING = 1,
	YWPANEL_VFD_DISPLAY,
	YWPANEL_VFD_READKEY,
	YWPANEL_VFD_DISPLAYSTRING
} YWPANEL_VFDDataType_t;

typedef struct YWPANEL_VFDData_s
{
	YWPANEL_VFDDataType_t type; /* 1- setting  2- display  3- readscankey  4- displaystring */
	u8 setValue;         // if type == YWPANEL_VFD_SETTING
	u8 address[16];      // if type == YWPANEL_VFD_DISPLAY
	u8 DisplayValue[16]; // if type == YWPANEL_VFD_DISPLAYSTRING
	u8 key;              // if type == YWPANEL_VFD_READKEY

} YWPANEL_VFDData_t;

#if defined(FP_DVFD)
typedef enum YWPANEL_DVFDDataType_e
{
	YWPANEL_DVFD_SETTING = 1,
	YWPANEL_DVFD_DISPLAY,
	YWPANEL_DVFD_DISPLAYSTRING,
	YWPANEL_DVFD_DISPLAYSYNC,
	YWPANEL_DVFD_SETTIMEMODE,
	YWPANEL_DVFD_GETTIMEMODE
} YWPANEL_DVFDDataType_t;

typedef struct YWPANEL_DVFDData_s
{
	YWPANEL_DVFDDataType_t type;
	u8 setValue;            // if type == YWPANEL_DVFD_SETTING
	u8 ulen;                // if type == YWPANEL_DVFD_DISPLAY_STRING
	u8 address[16];         // if type == YWPANEL_DVFD_DISPLAY_STRING
	u8 DisplayValue[16][5]; // if type == YWPANEL_DVFD_DISPLAY_STRING
} YWPANEL_DVFDData_t;
#endif

typedef struct YWPANEL_IRKEY_s
{
	u32 customCode;
	u32 dataCode;
} YWPANEL_IRKEY_t;

typedef struct YWPANEL_ScanKey_s
{
	u32 keyValue;
} YWPANEL_ScanKey_t;

typedef struct YWPANEL_StandByKey_s
{
	u32 key;
} YWPANEL_StandByKey_t;

typedef enum YWPANEL_IRCODE_e
{
	YWPANEL_IRCODE_NONE,
	YWPANEL_IRCODE_NEC = 0x01,
	YWPANEL_IRCODE_RC5,
	YWPANEL_IRCODE_RC6,
	YWPANEL_IRCODE_PILIPS
} YWPANEL_IRCODE_T;

typedef struct YWPANEL_IRCode_s
{
	YWPANEL_IRCODE_T code;
} YWPANEL_IRCode_t;

typedef enum YWPANEL_ENCRYPEMODE_e
{
	YWPANEL_ENCRYPEMODE_NONE = 0x00,
	YWPANEL_ENCRYPEMODE_ODDBIT,
	YWPANEL_ENCRYPEMODE_EVENBIT,
	YWPANEL_ENCRYPEMODE_RAMDONBIT
} YWPANEL_ENCRYPEMODE_t;

typedef struct YWPANEL_EncryptMode_s
{
	YWPANEL_ENCRYPEMODE_t mode;
} YWPANEL_EncryptMode_t;

typedef struct YWPANEL_EncryptKey_s
{
	u32 key;
} YWPANEL_EncryptKey_t;

typedef enum YWPANEL_VERIFYSTATE_e
{
	YWPANEL_VERIFYSTATE_NONE = 0,
	YWPANEL_VERIFYSTATE_CRC16,
	YWPANEL_VERIFYSTATE_CRC32,
	YWPANEL_VERIFYSTATE_CHECKSUM
} YWPANEL_VERIFYSTATE_t;

typedef struct YWPANEL_VerifyState_s
{
	YWPANEL_VERIFYSTATE_t state;
} YWPANEL_VerifyState_t;

typedef struct YWPANEL_Time_s
{
	u32 second;
} YWPANEL_Time_t;

typedef struct YWPANEL_ControlTimer_s
{
	int startFlag; // 0 - stop  1 - start
} YWPANEL_ControlTimer_t;

typedef struct YWPANEL_FpStandbyState_s
{
	int On;  // 0 - off  1 - on
} YWPANEL_FpStandbyState_t;

typedef struct YWPANEL_BlueKey_s
{
	u32 key;
} YWPANEL_BlueKey_t;

typedef struct YWFP_Format_s
{
	u8 LogNum;
	u8 LogSta;
} YWFP_Format_T;

typedef struct YWFP_Time_s
{
	u8 hour;
	u8 mint;
} YWFP_Time_T;

typedef enum YWPANEL_FPSTATE_e
{
	YWPANEL_FPSTATE_UNKNOWN,
	YWPANEL_FPSTATE_STANDBYOFF = 1,
	YWPANEL_FPSTATE_STANDBYON
} YWPANEL_FPSTATE_t;

typedef enum YWPANEL_CPUSTATE_s
{
	YWPANEL_CPUSTATE_UNKNOWN,
	YWPANEL_CPUSTATE_RUNNING = 1,
	YWPANEL_CPUSTATE_STANDBY
} YWPANEL_CPUSTATE_t;

typedef enum YWPANEL_LBDStatus_e
{
	YWPANEL_LBD_STATUS_OFF,
	YWPANEL_LBD_STATUS_ON,
	YWPANEL_LBD_STATUS_FL
} YWPANEL_LBDStatus_t;

typedef struct YWPANEL_CpuState_s
{
	YWPANEL_CPUSTATE_t state;
} YWPANEL_CpuState_t;

typedef struct YWFP_FuncKey_s
{
	u8 key_index;
	u32 key_value;
} YWFP_FuncKey_t;

typedef enum YWFP_TYPE_s
{
	YWFP_UNKNOWN,  // Not detected
	YWFP_COMMON,   // common type without deep standby
	YWFP_STAND_BY  // deep standby supported
} YWFP_TYPE_t;

typedef struct YWFP_INFO_s
{
	YWFP_TYPE_t fp_type;
} YWFP_INFO_t;

typedef enum YWPANEL_POWERONSTATE_e
{
	YWPANEL_POWERONSTATE_UNKNOWN,
	YWPANEL_POWERONSTATE_RUNNING = 1,
	YWPANEL_POWERONSTATE_CHECKPOWERBIT
} YWPANEL_POWERONSTATE_t;

typedef struct YWPANEL_PowerOnState_s
{
	YWPANEL_POWERONSTATE_t state;
} YWPANEL_PowerOnState_t;

typedef enum YWPANEL_STARTUPSTATE_e
{
	YWPANEL_STARTUPSTATE_UNKNOWN = 0,
	YWPANEL_STARTUPSTATE_ELECTRIFY = 1,  // power on
	YWPANEL_STARTUPSTATE_STANDBY,        // from standby
	YWPANEL_STARTUPSTATE_TIMER           // woken by timer
} YWPANEL_STARTUPSTATE_t;

typedef struct YWPANEL_StartUpState_s
{
	YWPANEL_STARTUPSTATE_t state;
} YWPANEL_StartUpState_t;

typedef enum YWPANEL_LOOPSTATE_e
{
	YWPANEL_LOOPSTATE_UNKNOWN = 0,
	YWPANEL_LOOPSTATE_LOOPOFF = 1,
	YWPANEL_LOOPSTATE_LOOPON
} YWPANEL_LOOPSTATE_t;

typedef struct YWPANEL_LoopState_s
{
	YWPANEL_LOOPSTATE_t state;
} YWPANEL_LoopState_t;

typedef enum YWPANEL_LBDType_e
{ // LED masks
	YWPANEL_LBD_TYPE_POWER  = (1 << 0),
	YWPANEL_LBD_TYPE_SIGNAL = (1 << 1),
	YWPANEL_LBD_TYPE_MAIL   = (1 << 2),
	YWPANEL_LBD_TYPE_AUDIO  = (1 << 3)
} YWPANEL_LBDType_t;

typedef enum YWPANEL_FP_MCUTYPE_e
{
	YWPANEL_FP_MCUTYPE_UNKNOWN = 0,
	YWPANEL_FP_MCUTYPE_AVR_ATTING48, // AVR MCU
	YWPANEL_FP_MCUTYPE_AVR_ATTING88,
	YWPANEL_FP_MCUTYPE_MAX
} YWPANEL_FP_MCUTYPE_t;

typedef enum YWPANEL_FP_DispType_e
{
	YWPANEL_FP_DISPTYPE_UNKNOWN = 0,
	YWPANEL_FP_DISPTYPE_VFD     = (1 << 0),
	YWPANEL_FP_DISPTYPE_LCD     = (1 << 1),
	YWPANEL_FP_DISPTYPE_DVFD    = (3),
	YWPANEL_FP_DISPTYPE_LED     = (1 << 2),
	YWPANEL_FP_DISPTYPE_LBD     = (1 << 3)
} YWPANEL_FP_DispType_t;

typedef struct YWPANEL_Version_s
{
	YWPANEL_FP_MCUTYPE_t CpuType;
	u8 DisplayInfo;
	u8 scankeyNum;
	u8 swMajorVersion;
	u8 swSubVersion;
} YWPANEL_Version_t;

typedef struct YWPANEL_FPData_s
{
	YWPANEL_DataType_t dataType;
	union
	{
		YWPANEL_Version_t        version;
		YWPANEL_LBDData_t        lbdData;
		YWPANEL_LEDData_t        ledData;
		YWPANEL_LCDData_t        lcdData;
		YWPANEL_VFDData_t        vfdData;
#if defined(FP_DVFD)
		YWPANEL_DVFDData_t       dvfdData;
#endif
		YWPANEL_IRKEY_t          IrkeyData;
		YWPANEL_ScanKey_t        ScanKeyData;
		YWPANEL_CpuState_t       CpuState;
		YWPANEL_StandByKey_t     stbyKey;
		YWPANEL_IRCode_t         irCode;
		YWPANEL_EncryptMode_t    EncryptMode;
		YWPANEL_EncryptKey_t     EncryptKey;
		YWPANEL_VerifyState_t    verifyState;
		YWPANEL_Time_t           time;
		YWPANEL_ControlTimer_t   TimeState;
		YWPANEL_FpStandbyState_t FpStandbyState;
		YWPANEL_BlueKey_t        BlueKey;
		YWPANEL_PowerOnState_t   PowerOnState;
		YWPANEL_StartUpState_t   StartUpState;
		YWPANEL_LoopState_t      LoopState;
	} data;
	int ack;
} YWPANEL_FPData_t;

int YWPANEL_FP_Init(void);
extern int (*YWPANEL_FP_Initialize)(void);
extern int (*YWPANEL_FP_Term)(void);
extern int (*YWPANEL_FP_ShowIcon)(LogNum_t, int);
extern int (*YWPANEL_FP_ShowTime)(u8 hh, u8 mm);
extern int (*YWPANEL_FP_ShowTimeOff)(void);
extern int (*YWPANEL_FP_SetBrightness)(int);
extern u8  (*YWPANEL_FP_ScanKeyboard)(void);
extern int (*YWPANEL_FP_ShowString)(char *);
extern int (*YWPANEL_FP_ShowContent)(void);
extern int (*YWPANEL_FP_ShowContentOff)(void);
extern void create_proc_fp(void);
extern void remove_proc_fp(void);

extern int YWPANEL_width;
#define THREAD_STATUS_RUNNING 0
#define THREAD_STATUS_STOPPED 1
#define THREAD_STATUS_INIT    2
extern tThreadState led_state[LASTLED + 1];
extern tThreadState spinner_state;
extern int scroll_repeats;  // 0 = no, number is times??
extern int scroll_delay;  // // scroll speed between character shift
extern int initial_scroll_delay;  // wait time to start scrolling
extern int final_scroll_delay;  // wait time to start final display

//int YWPANEL_FP_GetRevision(char * version);
YWPANEL_FPSTATE_t YWPANEL_FP_GetFPStatus(void);
int YWPANEL_FP_SetFPStatus(YWPANEL_FPSTATE_t state);
YWPANEL_CPUSTATE_t YWPANEL_FP_GetCpuStatus(void);
int YWPANEL_FP_SetCpuStatus(YWPANEL_CPUSTATE_t state);
int YWPANEL_FP_ControlTimer(int on);
YWPANEL_POWERONSTATE_t YWPANEL_FP_GetPowerOnStatus(void);
int YWPANEL_FP_SetPowerOnStatus(YWPANEL_POWERONSTATE_t state);
u32 YWPANEL_FP_GetTime(void);
int YWPANEL_FP_SetTime(u32 value);
u32 YWPANEL_FP_GetStandByKey(u8 index);
int YWPANEL_FP_SetStandByKey(u8 index, u8 key);
u32 YWPANEL_FP_GetBlueKey(u8 index);
int YWPANEL_FP_SetBlueKey(u8 index, u8 key);
int YWPANEL_LBD_SetStatus(YWPANEL_LBDStatus_t LBDStatus);
int YWPANEL_FP_GetStartUpState(YWPANEL_STARTUPSTATE_t *state);
int YWPANEL_FP_GetVersion(YWPANEL_Version_t *version);
u32 YWPANEL_FP_GetIRKey(void);
int YWPANEL_FP_SetPowerOnTime(u32 value);
u32 YWPANEL_FP_GetPowerOnTime(void);
int YWPANEL_FP_GetKeyValue(void);
int YWPANEL_FP_SetLed(int which, int on);

#endif /* __AOTOM_MAIN_H__ */
// vim:ts=4
