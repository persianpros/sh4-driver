/*
 * nuvoton.h
 *
 * Frontpanel driver for Fortis HDBOX and Octagon 1008
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
 *
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 20130929 Audioniek
 *
 ****************************************************************************************/

#ifndef nuvoton_h
#define nuvoton_h

extern short paramDebug;
#define TAGDEBUG "[nuvoton] "

#ifndef dprintk
#define dprintk(level, x...) do \
{ \
	if ((paramDebug) && (paramDebug >= level)) printk(TAGDEBUG x); \
} while (0)
#endif

/****************************************************************************************/

#if defined(OCTAGON1008) || defined(HS7420) || defined(HS7429)
#define DISP_SIZE 8
#elif defined(FORTIS_HDBOX) || defined(ATEVIO7500)
#define DISP_SIZE 12
#elif defined(HS7810A) || defined(HS7119) || defined(HS7819)
#define DISP_SIZE 4
#elif defined(HS7110)
#define DISP_SIZE 0
#endif

typedef struct
{
	struct file     *fp;
	int              read;
	struct semaphore sem;

} tFrontPanelOpen;

#define FRONTPANEL_MINOR_RC   1
#define LASTMINOR             2

extern tFrontPanelOpen FrontPanelOpen[LASTMINOR];

#define VFD_MAJOR             147
#define SOP                   0x02
#define EOP                   0x03

#if defined(FORTIS_HDBOX) || defined(ATEVIO7500)
#define ICON_THREAD_STATUS_RUNNING 0
#define ICON_THREAD_STATUS_STOPPED 1
#define ICON_THREAD_STATUS_INIT    2
#define ICON_THREAD_STATUS_HALTED  3  // (semaphore down)
//#define ICON_THREAD_STATUS_PAUSED  4  // (state == 0)
#endif

/* ioctl numbers ->hacky */
#define VFDDISPLAYCHARS       0xc0425a00
#define VFDBRIGHTNESS         0xc0425a03
//#define VFDPWRLED             0xc0425a04 /* added by zeroone, also used in fp_control/global.h ; set PowerLed Brightness on HDBOX*/
#define VFDDISPLAYWRITEONOFF  0xc0425a05
#define VFDDRIVERINIT         0xc0425a08
#define VFDICONDISPLAYONOFF   0xc0425a0a

#define VFDTEST               0xc0425af0 // added by audioniek
#define VFDGETBLUEKEY         0xc0425af1 // unused, used by other boxes
#define VFDSETBLUEKEY         0xc0425af2 // unused, used by other boxes
#define VFDGETSTBYKEY         0xc0425af3 // unused, used by other boxes
#define VFDSETSTBYKEY         0xc0425af4 // unused, used by other boxes
#define VFDPOWEROFF           0xc0425af5 // unused, used by other boxes
#define VFDSETPOWERONTIME     0xc0425af6 // unused, used by other boxes
#define VFDGETVERSION         0xc0425af7
#define VFDGETSTARTUPSTATE    0xc0425af8 // unused, used by other boxes
#define VFDGETWAKEUPMODE      0xc0425af9
#define VFDGETTIME            0xc0425afa
#define VFDSETTIME            0xc0425afb
#define VFDSTANDBY            0xc0425afc
//
#define VFDSETTIME2           0xc0425afd // unused, used by other boxes
#define VFDSETLED             0xc0425afe
#define VFDSETMODE            0xc0425aff
#define VFDDISPLAYCLR         0xc0425b00 // unused, used by other boxes
#define VFDGETLOOPSTATE       0xc0425b01 // unused, used by other boxes
#define VFDSETLOOPSTATE       0xc0425b02 // unused, used by other boxes
#define VFDGETWAKEUPTIME      0xc0425b03 // added by audioniek, unused, used by other boxes
#define VFDSETTIMEFORMAT      0xc0425b04 // added by audioniek

#if defined(FORTIS_HDBOX) || defined(OCTAGON1008)
#define RESELLER_OFFSET 0x000000f0  // offset to 32 bit word in mtd0 that holds the resellerID & loader version
#elif defined(ATEVIO7500) || defined(HS7110) || defined(HS7420) || defined(HS7810A) || defined(HS7119) || defined(HS7429) || defined(HS7819)
#define RESELLER_OFFSET 0x00000430
#endif

/***************************************************************************
 *
 * Icon definitions.
 *
 ***************************************************************************/
#if defined(OCTAGON1008)
enum  // HS9510 icon numbers and their names
{
	ICON_MIN,     // 00
	ICON_DOLBY,   // 01
	ICON_DTS,     // 02
	ICON_VIDEO,   // 03
	ICON_AUDIO,   // 04
	ICON_LINK,    // 05
	ICON_HDD,     // 06
	ICON_DISC,    // 07
	ICON_DVB,     // 08
	ICON_DVD,     // 09
	ICON_TIMER,   // 10
	ICON_TIME,    // 11
	ICON_CARD,    // 12
	ICON_1,       // 13
	ICON_2,       // 14
	ICON_KEY,     // 15
	ICON_16_9,    // 16
	ICON_USB,     // 17
	ICON_CRYPTED, // 18
	ICON_PLAY,    // 19
	ICON_REWIND,  // 20
	ICON_PAUSE,   // 21
	ICON_FF,      // 22
	ICON_NONE,    // 23
	ICON_REC,     // 24
	ICON_ARROW,   // 25
	ICON_COLON1,  // 26
	ICON_COLON2,  // 27
	ICON_COLON3,  // 28
	ICON_MAX      // 29
};

#elif defined(HS7420) || defined(HS7429)
/***************************************************************************
 *
 * Icons for HS742X
 *
 */
enum //HS742X icon numbers and their names
{
	ICON_MIN,     // 00
	ICON_DOT,     // 01, pos 7, byte 2, bit 7
	ICON_COLON1,  // 02, pos 6, byte 2, bit 7
	ICON_COLON2,  // 03, pos 4, byte 2, bit 7
	ICON_COLON3,  // 04, pos 2, byte 2, bit 7
	ICON_MAX      // 05
};

#elif defined(HS7119)
/***************************************************************
 *
 * Icons for HS7119
 *
 */ 
enum //HS7119 icon numbers and their names
{
	ICON_MIN,       // 00
	ICON_COLON,     // 01
	ICON_MAX        // 02
};

#elif defined(HS7810A) || defined(HS7819)
/***************************************************************
 *
 * Icons for HS7810A and HS7819
 *
 */ 
enum //HS7810A/7819 icon numbers and their names
{
	ICON_MIN,       // 00
	ICON_COLON,     // 01
	ICON_PERIOD1,   // 02
	ICON_PERIOD2,   // 03
	ICON_PERIOD3,   // 04
	ICON_MAX        // 05
};

#elif defined(FORTIS_HDBOX)
/***************************************************************************
 *
 * Icons for FS9000/9200
 *
 */
enum //FS9X00 icon numbers and their names
{
	ICON_MIN,       // 00
	ICON_STANDBY,   // 01
	ICON_SAT,       // 02
	ICON_REC,       // 03
	ICON_TIMESHIFT, // 04
	ICON_TIMER,     // 05
	ICON_HD,        // 06
	ICON_USB,       // 07
	ICON_SCRAMBLED, // 08
	ICON_DOLBY,     // 09
	ICON_MUTE,      // 10
	ICON_TUNER1,    // 11
	ICON_TUNER2,    // 12
	ICON_MP3,       // 13
	ICON_REPEAT,    // 14
	ICON_PLAY,      // 15
	ICON_Circ0,     // 16
	ICON_Circ1,     // 17
	ICON_Circ2,     // 18
	ICON_Circ3,     // 19
	ICON_Circ4,     // 20
	ICON_Circ5,     // 21
	ICON_Circ6,     // 22
	ICON_Circ7,     // 23
	ICON_Circ8,     // 24
	ICON_FILE,      // 25
	ICON_TER,       // 26
	ICON_480i,      // 27
	ICON_480p,      // 28
	ICON_576i,      // 29
	ICON_576p,      // 30
	ICON_720p,      // 31
	ICON_1080i,     // 32
	ICON_1080p,     // 33
	ICON_COLON1,    // 34
	ICON_COLON2,    // 35
	ICON_COLON3,    // 36
	ICON_COLON4,    // 37
	ICON_TV,        // 38
	ICON_RADIO,     // 39
	ICON_MAX,       // 40
	ICON_SPINNER    // 41
};

#elif defined(ATEVIO7500)
/***************************************************************
 *
 * Icons for HS8200
 *
 */
enum //HS8200 icon numbers and their names
{
	ICON_MIN,       // 00
	ICON_STANDBY,   // 01
	ICON_REC,       // 02
	ICON_TIMESHIFT, // 03
	ICON_TIMER,     // 04
	ICON_HD,        // 05
	ICON_USB,       // 06
	ICON_SCRAMBLED, // 07
	ICON_DOLBY,     // 08
	ICON_MUTE,      // 09
	ICON_TUNER1,    // 10
	ICON_TUNER2,    // 11
	ICON_MP3,       // 12
	ICON_REPEAT,    // 13
	ICON_PLAY,      // 14
	ICON_STOP,      // 15
	ICON_PAUSE,     // 16
	ICON_REWIND,    // 17
	ICON_FF,        // 18
	ICON_STEP_BACK, // 19
	ICON_STEP_FWD,  // 20
	ICON_TV,        // 21
	ICON_RADIO,     // 22
	ICON_MAX,       // 23
	ICON_SPINNER    // 24
};
#endif  // Icon definitions
/* End of character and icon definitions */


/********************************************************************************************************************************************
 *
 * List of possible commands to the front processor
 *
 * Length is opcode plus parameters (without SOP and EOP)
 *
 *      Command name             Opcode  Length Parameters                               Returns
 */
#define cCommandGetMJD           0x10 // 01     -                                        5 bytes: MJDh MJDl hrs mins secs
#define cCommandSetTimeFormat    0x11 // 02     1:format byte                            nothing
                                      //          0x80 ->12h, 0x81->24h
#define cCommandSetMJD           0x15 // 06     1:MJDh 2:MJDl 3:hrs 4:mins 5:secs        nothing

#define cCommandSetPowerOff      0x20 //        not used in nuvoton

#if defined(HS7110) || defined(HS7119) || defined(HS7810A) || defined(HS7819)
#define cCommandSetLEDBrightness 0x23 // 04     1:0x03, 2:level, 3:0x02                  nothing
#define cCommandSetLEDText       0x24 // 05     1:data pos1                              nothing
                                      //        2:data pos2
                                      //        3:data pos3
                                      //        4:data pos4
#endif

#define cCommandPowerOffReplay   0x31 // 02     1:1=power off, 2=power on?
#define cCommandSetBootOn        0x40 // 01     -
#define cCommandSetBootOnExt     0x41 // 02     1:?, not used in nuvoton

#define cCommandSetWakeupTime    0x72 // 03     1:hrs  2:mins                            nothing
#define cCommandSetWakeupMJD     0x74 // 05     1:MJDh 2:MJDl 3:hrs 4:mins               nothing

#define cCommandGetPowerOnSrc    0x80 // 01                                              1 byte: wake up reason (0=power-on, 1= from deep standby, 2=timer?)
                                                                                         //TODO: returns error on HS742X

#define cCommandSetLed           0x93 // 04     1:LED# 2:level 3:previous level?         nothing

#define cCommandGetIrCode        0xa0 // 01                                              14 bytes
#define cCommandGetIrCodeExt     0xa1 // 02     1:?                                      1 byte
#define cCommandSetIrCode        0xa5 // 06     1:?, 2:?, 3:?, 4:?, 5:?                  nothing

#define cCommandGetPort          0xb2
#define cCommandSetPort          0xb3

#define cCommandSetIconI         0xc2 // 03     1:registernumber, 2:bitmask for on/off   nothing
#define cCommandSetIconII        0xc6 // 06     1:start registernumber,                  nothing
                                      //        2:bitmask for on/off (start reg),
                                      //        3:bitmask for on/off (start reg + 1),
                                      //        4:bitmask for on/off (start reg + 2),
                                      //        5:bitmask for on/off (start reg + 3),
                                      //        6:bitmask for on/off (start reg + 4)
                                      //        NOTE: only works after SOP 0xc2 0x10 0x00 EOP is sent

#if defined(OCTAGON1008) || defined(HS7420) || defined(HS7429)
#define cCommandSetVFD           0xc4 // 05     1:digit#, 2:data, 3:data, 4:data         nothing
#else
#define cCommandSetVFDI          0xcc // 12     1:0x11 2-12:11 characters                nothing
#define cCommandSetVFDII         0xcd // 13     1:0x11 2-13:12 characters                nothing
                                      //        (displayed in upper case)
#define cCommandSetVFDIII        0xce // 14     1:0x11 2-14:13 characters                nothing
#endif

#define cCommandSetVFDBrightness 0xd2 // 03     1:0 2:level                              nothing

#define cCommandGetFrontInfo     0xd0 // 01     - not used in nuvoton                    2 bytes 0:00, 1:01

#if defined(ATEVIO7500)
#define cCommand_E9              0xe9 // 10     ? not used in nuvoton                    9 bytes
#define cCommand_EB              0xeb // 12     ? not used in nuvoton                    nothing
#define cCommand_F4              0xf4 // 05     ? not used in nuvoton                    nothing
#define cCommand_F9              0xf9 // 10     ? not used in nuvoton                    9 bytes
#endif

/****************************************************************************************/

struct set_test_s
{
	char data[19];
};

struct set_brightness_s
{
	int level;
};

struct set_pwrled_s
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

struct set_timeformat_s
{
	char format;
};

struct get_version_s
{
	unsigned int data[2]; // data[0] = boot loader version, data[1] = resellerID
};

/* This will set the mode temporarily (for one ioctl)
 * to the desired mode. Currently the "normal" mode
 * is the compatible vfd mode
 */
struct set_mode_s
{
	int compat; /* 0 = compatibility mode to vfd driver; 1 = nuvoton mode */
};

struct nuvoton_ioctl_data
{
	union
	{
		struct set_test_s test;
		struct set_icon_s icon;
		struct set_led_s led;
		struct set_brightness_s brightness;
		struct set_pwrled_s pwrled;
		struct set_light_s light;
		struct set_mode_s mode;
		struct set_standby_s standby;
		struct set_time_s time;
		struct set_timeformat_s timeformat;
		struct get_version_s version;
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
	int length;
	char data[128];
	int brightness;
	int display_on;
#if !defined(HS7110)
	int icon_state[ICON_MAX + 2];
#if defined(ATEVIO7500)
	int icon_count; // number of icons switched on
#endif
#endif
#if defined(HS7119) || defined(HS7810A) || defined(HS7819)
	unsigned char buf[7];
#endif
};
extern struct saved_data_s lastdata;

#if defined(OCTAGON1008) || defined(HS7420) || defined(HS7429)
struct vfd_buffer
{
	u8 buf0;
	u8 buf1;
	u8 buf2;
};
#endif

#if defined(FORTIS_HDBOX) || defined(ATEVIO7500)
typedef struct
{
	int state;
	int period;
	int status;
	int enable;
	struct task_struct *task;
	struct semaphore sem;
} tIconState;
#endif

extern int nuvoton_init_func(void);
extern void copyData(unsigned char *data, int len);
extern void getRCData(unsigned char *data, int *len);
extern int nuvotonSetIcon(int which, int on);
extern int nuvotonWriteCommand(char *buffer, int len, int needAck);
#if defined(FORTIS_HDBOX)
extern tIconState spinner_state;
#endif
#if defined(ATEVIO7500)
extern tIconState spinner_state;
extern tIconState icon_state;
extern int icon_thread(void *arg);
#endif
void dumpValues(void);

extern u8 regs[0x100];  // array with copy values of FP registers
extern int errorOccured;
extern struct file_operations vfd_fops;
extern char *gmt_offset;  // module param, string
extern int rtc_offset;

#endif  //nuvoton_h
// vim:ts=4
