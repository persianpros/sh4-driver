/****************************************************************************
 *
 * opt9600_fp.h
 *
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add irq handling
 * (c) 2020 Audioniek: ported to Opticum HD 9600 (TS)
 *
 * Largely based on cn_micom, enhanced and ported to Opticum HD 9600 (TS)
 * by Audioniek.
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
 ****************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------
 * 20201223 Audioniek       Initial version, based on cn_micom.
 *
 ****************************************************************************/
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

#define FRONTPANEL_MINOR_RC      1
#define LASTMINOR                2

extern tFrontPanelOpen FrontPanelOpen[LASTMINOR];

#define VFD_MAJOR                147

#if 0
#define PRN_LOG(msg, args...)    printk(msg, ##args)
#define PRN_DUMP(str, data, len) mcom_Dump(str, data, len)
#else
//#define PRN_LOG(msg, args...)
//#define PRN_DUMP(str, data, len)
#endif

#define MAX_MCOM_MSG             30

#define MCU_COMM_PORT            UART_1
#define MCU_COMM_TIMEOUT         1000

typedef enum
{
	_VFD_OLD,
	_VFD,
	_7SEG,
	_UNKNOWN
} DISPLAYTYPE;

#if defined(OPT9600) \
 || defined(OPT9600PRIMA)
#define DISPLAY_WIDTH            8  // VFD models
#else
#define DISPLAY_WIDTH            4  // LED model
#endif

/* ioctl numbers ->hacky */
#define VFDDISPLAYCHARS       0xc0425a00
#define VFDBRIGHTNESS         0xc0425a03  // not supported by hardware?
#define VFDDISPLAYWRITEONOFF  0xc0425a05
#define VFDDRIVERINIT         0xc0425a08
#define VFDICONDISPLAYONOFF   0xc0425a0a  // not supported by hardware

#define VFDGETWAKEUPMODE      0xc0425af9  // not supported by hardware?
#define VFDGETTIME            0xc0425afa
#define VFDSETTIME            0xc0425afb
#define VFDSTANDBY            0xc0425afc
#define VFDSETLED             0xc0425afe  // not supported by hardware?
#define VFDSETMODE            0xc0425aff

#define ACK_WAIT_TIME msecs_to_jiffies(1000)

#define cPackageSizeStandby   5
#define cPackageSizeKeyIn     3

#define cGetTimeSize          8  // was 11
#define cGetVersionSize       5  // was 6
#define cGetWakeupReasonSize  10
#define cGetResponseSize      3

#define cMinimumSize          3

// commands to front processor
#define _MCU_RESPONSE         0xC5  // set checksum over previous command (_MCU_BOOT only?)
#define _MCU_ICON             0xE1  // set icon (not used or implemnented?)
#define _MCU_STANDBY          0xE5  // switch to standby
#define _MCU_REBOOT           0xE6
#define _MCU_WAKEUPREASON     0xE7  // request wakeup reason?
#define _MCU_PRIVATE          0xE7
#define _MCU_KEYCODE          0xE9
#define _MCU_WAKEUP           0xEA  // set uC wakup time
#define _MCU_BOOT             0xEB  // request FP uC boot / switch off (start/stop front uC?)
#define _MCU_NOWTIME          0xEC  // set uC current time
#define _MCU_DISPLAY          0xED  // display text

// answer tags from front processor
#define EVENT_ANSWER_RESPONSE      0xc5  // missing RESPONSE command or unknown command alert
#define EVENT_ANSWER_VERSION       0xdb  // version info response
#define EVENT_ANSWER_WAKEUP_REASON 0xe7
#define EVENT_ANSWER_KEYIN         0xf1  // key press response
#define EVENT_ANSWER_TIME          0xfc  // report uC time
//#define _MCU_TIME             0xFC  // report uC time
//#define _MCU_KEYIN            0xF1

#define _TAG                  0  // offset for command byte in command string
#define _LEN                  1  // offset for length byte in command string
#define _VAL                  2  // offset for argument byte(s) in command string

/* time must be given as follows:
 * time[0] = year
 * time[1] = month
 * time[2] = day
 * time[3] = hour
 * time[4] = min  // NOTE: front processor does not use seconds
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
