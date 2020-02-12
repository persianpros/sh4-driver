/*
 * fortis_4g.h
 *
 * Frontpanel driver for Fortis 4th generation receivers
 *
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add irq handling
 * (c) 2014 Audioniek: Rewritten and adapted for Fortis 4G
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
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 20141108 Audioniek       Start of work, based on nuvoton.h.
 * 20150205 Audioniek       FOREVER_2424HD, GPV8000 and EP8000 models added.
 *
 ****************************************************************************************/

#ifndef _fortis_4gh
#define _fortis_4gh

extern short paramDebug;
#define TAGDEBUG "[fortis_4g_fp] "

#ifndef dprintk
#define dprintk(level, x...) do { if ((paramDebug) && (paramDebug >= level)) printk(TAGDEBUG x); } while (0)
#endif

/****************************************************************************************/

#define VFD_MAJOR             147

#define FRONTPANEL_MINOR_VFD  0
#define FRONTPANEL_MINOR_RC   1
#define LASTMINOR             1

/* ioctl numbers ->hacky */
#define VFDDISPLAYCHARS       0xc0425a00
#define VFDBRIGHTNESS         0xc0425a03
#define VFDPWRLED             0xc0425a04
#define VFDDISPLAYWRITEONOFF  0xc0425a05
#define VFDDRIVERINIT         0xc0425a08
#define VFDICONDISPLAYONOFF   0xc0425a0a

#define VFDTEST               0xc0425af0 // added by audioniek
#define VFDGETBLUEKEY         0xc0425af1 // unused, used by other boxes
#define VFDSETBLUEKEY         0xc0425af2 // unused, used by other boxes
#define VFDGETSTBYKEY         0xc0425af3 // unused, used by other boxes
#define VFDSETSTBYKEY         0xc0425af4 // unused, used by other boxes
#define VFDPOWEROFF           0xc0425af5 // unused, used by other boxes
#define VFDSETWAKEUPTIME      0xc0425af6
#define VFDGETVERSION         0xc0425af7 // unused, used by other boxes
#define VFDGETSTARTUPSTATE    0xc0425af8 // unused, used by other boxes
#define VFDGETWAKEUPMODE      0xc0425af9
#define VFDGETTIME            0xc0425afa
#define VFDSETTIME            0xc0425afb
#define VFDSTANDBY            0xc0425afc
//
#define VFDREBOOT             0xc0425afd
#define VFDSETTIME2           0xc0425afd // assigned twice: used by other boxes
#define VFDSETLED             0xc0425afe // same as SETPWRLED, Fortis 4G model cannot dim their LEDs
#define VFDSETMODE            0xc0425aff
#define VFDDISPLAYCLR         0xc0425b00 // unused, used by other boxes
#define VFDGETLOOPSTATE       0xc0425b01 // unused, used by other boxes
#define VFDSETDISPLAYTIME     0xc0425b02 // added by audioniek, used by other boxes
#define VFDGETWAKEUPTIME      0xc0425b03 // added by audioniek, used by other boxes
#define VFDSETTIMEFORMAT      0xc0425b04 // added by audioniek, unused, used by other boxes

/* Defines for the FP LEDs and icons */
/* LED numbers are bitmapped, LED_MAX should be the sum of the other defined LEDs */
#if defined(DP7001) || defined(FOREVER_2424HD)
#define LED_GREEN      1
#define LED_LOGO       2
#define MAX_LED        3 // X: standby (red, not controllable), 1: remote (green), 2: logo
#define MAX_LED_BRIGHT 1 // LED is either on or off
#define MAX_BRIGHT     8
#elif defined(FOREVER_NANOSMART) // 1: standby (red), (green) seems to be not directly controllable (off when red is on)
#define LED_GREEN      1
#define MAX_LED        1
#define MAX_LED_BRIGHT 1 // LED is either on or off
#define MAX_BRIGHT     8
#elif defined(FOREVER_3434HD) // 1: standby (red), 2: blue, 4: logo
#define LED_RED        1
#define LED_BLUE       2
#define LED_LOGO       4
#define MAX_LED        7
#elif defined(GPV8000) || defined(EP8000) || defined(EPP8000) || defined(FOREVER_9898HD) // 1: standby (red), 2: logo
#define LED_RED        1
#define LED_LOGO       2
#define MAX_LED        3
#endif

#define MAX_BRIGHT     8  // display brightness

/* Defines for the icons */
#if defined(FOREVER_9898HD) || defined(GPV8000) || defined(EP8000) || defined(EPP8000) || defined(FOREVER_3434HD)
#define ICON_MIN       0
#define ICON_DOT       1
#define ICON_REC       ICON_DOT
#define ICON_COLON1    2
#define ICON_COLON2    3
#define ICON_COLON3    4
#define ICON_MAX       5
#define ICON_ALL       ICON_MAX
#endif

/* Defines for the FP display */
#if defined(FOREVER_NANOSMART) || defined(DP7001)|| defined(FOREVER_2424HD)
#define DISP_SIZE 4
#elif defined(FOREVER_9898HD) || defined(GPV8000) || defined(EP8000) || defined(EPP8000) || defined(FOREVER_3434HD)
#define DISP_SIZE 8
#endif

struct saved_data_s
{
	int length;
	char data[128];
};

//String data displayed last on FP
static struct saved_data_s lastdata;

struct set_icon_s
{
	int icon_nr;
	int on;
};

struct set_led_s
{
	int led_nr;
	int level; //level was chosen here because some Fortis models can dim their LEDs
};

struct set_brightness_s
{
	int level;
};

struct set_light_s
{
	int onoff;
};

/* This will set the mode temporarily (for one ioctl)
 * to the desired mode. Currently the "normal" mode
 * is the compatible vfd mode
 */
struct set_mode_s
{
	int compat; /* 0 = compatibility mode to vfd driver; 1 = nuvoton mode */
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

struct set_timeformat_s
{
	char format;
};

struct set_timedisplay_s
{
	char onoff;
};

struct fortis_4g_ioctl_data
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
		struct set_timeformat_s timeformat;
		struct set_timedisplay_s timedisplay;
	} u;
};

struct vfd_ioctl_data
{
	unsigned char start_address;
	unsigned char data[64];
	unsigned char length;
};

/****************************************************************************************
 * Interface to the I/O parts
 *
 * Each core part must have these routines:
 */

extern int fortis_4gSetIcon(int which, int on);
extern int fortis_4gSetLED(int which, int level);
extern int fortis_4gSetBrightness(int level);
extern int fortis_4gSetDisplayOnOff(char on);
//extern int fortis_4gSetStandby(char *time);
//extern int fortis_4gSetTime(char *time);
//extern int fortis_4gGetTime(void);
extern int fortis_4gGetWakeUpMode(void);
extern int fortis_4gSetTimeFormat(char format);
extern int fortis_4gSetTimeDisplayOnOff(char onoff);
extern int fortis_4gWriteString(unsigned char *aBuf, int len);
extern int fortis_4gInit_core(void); //Initialize the I/O part
extern void fortis_4gExit_core(void); //Cleanup the I/O part
/****************************************************************************************/
#endif //_fortis_4g

