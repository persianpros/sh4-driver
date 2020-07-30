/***************************************************************************/
/* Name :   Topfield TF7700 Front Panel Driver Module                      */
/*          ./drivers/topfield/frontpanel.c                                */
/*                                                                         */
/* Author:  Gustav Gans                                                    */
/*                                                                         */
/* Descr.:  Controls the front panel clock, timers, keys and display       */
/*                                                                         */
/* Licence: This file is subject to the terms and conditions of the GNU    */
/* General Public License version 2.                                       */
/*                                                                         */
/* 2008-06-10 V1.0   First release                                         */
/* 2008-07-19 V2.0   Added the key buffer and the /dev/rc device           */
/*                   Added the TypematicDelay and TypematicRate            */
/*                   Added the KeyEmulationMode (0=TF7700, 1=UFS910)       */
/* 2008-07-28 V2.1   Adapted to Enigma2                                    */
/* 2008-08-21 V2.2   Added poll() operation                                */
/* 2008-12-31        introduced a thread for interrupt processing          */
/*                   added handling for the new VFD interface              */
/*                   added issuing a standby key to the application        */
/* 2009-01-05 V3.0   added suppression of multiple shutdown commands       */
/*                   replaced the module author                            */
/* 2009-01-06 V3.1   Added reboot control code                             */
/* 2009-02-08 V3.2   Added GMT offset handling                             */
/* 2009-03-19 V3.3   Added mapping of VFD codes 0-9 to CD segments         */
/* 2009-03-19 V3.4   Changed the standby button logic                      */
/*                   Added a 3 sec emergency shutdown                      */
/* 2009-03-19 V3.5   Changed mapping of VFD codes 0x20-0x29 to CD segments */
/*                   Fixed a break statement                               */
/* 2009-03-22 V3.6   Fixed a buffer overflow which garbled the VFD         */
/* 2009-03-28 V3.7   Added the --resend command                            */
/* 2009-03-30 V3.8   Fixed the issue with late acknowledge of shutdown     */
/*                   requests                                              */
/*                   Increased the forced shutdown delay to 5 seconds      */
/* 2009-04-10 V3.9   Added mapping of the keys |< and >| to VolumeDown     */
/*                   and VolumeUp                                          */
/* 2009-06-25 V4.0   Added support for UTF-8 strings                       */
/*                   Added the option to show all characters of the large  */
/*                   display in capital letters                            */
/* 2009-06-29 V4.1   Added the ability to scroll longer strings            */
/* 2009-07-01 V4.1a  Trims trailing space to prevent the scrolling of text */
/*                   with less than 8 chars                                */
/* 2009-07-27 V4.2   Removed filtering of the power-off key.               */
/* 2009-08-08 V4.3   Added support for configuration in EEPROM             */
/* 2009-10-13 V4.4   Added typematic rate control for the power-off key    */
/* 2009-10-13 V4.5   Fixed error in function TranslateUTFString.           */
/*                   In special casses the ending '\0' has been skipped    */
/* 2011-07-18 V4.6   Add quick hack for long key press                     */
/* 2011-07-23 V4.7   Add LKP emulation mode                                */
/* 2015-09-19 V4.8   All icons controllable and driver made compatible     */
/*                   with VFD-Icons plugin. Tab/space cleanup. Indenting   */
/*                   made consistent, including use of braces. Fixed all   */
/*                   compiler warnings.                                    */
/* 2015-09-25 V4.9   IOCTL brightness control and display on/off added.    */
/* 2015-10-13 V4.10  RTC driver added.                                     */
/* 2015-11-06 V4.11  Spinner added.                                        */
/* 2018-12-28 V4.11a /proc/stb/lcd/sybol_circle added; icon definitions    */
/*                    moved to frontpanel.h.                               */
/***************************************************************************/

#define VERSION         "V4.11a"
//#define DEBUG

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <stdarg.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/termios.h>
#include <linux/poll.h>
#include <linux/ctype.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>

#include "frontpanel.h"
#include "stb7109regs.h"
#include "VFDSegmentMap.h"
#include "tffp_config.h"

extern void create_proc_fp(void);
extern void remove_proc_fp(void);

#define LASTMINOR                 3
#define IOCTLMAGIC                0x3a

#define FPSHUTDOWN                0x20
#define FPPOWEROFF                0x21
#define FPSHUTDOWNACK             0x31
#define FPGETDISPLAYCONTROL       0x40
#define FPREQDATETIMENEW          0x30
#define FPDATETIMENEW             0x25
#define FPKEYPRESSFP              0x51
#define FPKEYPRESS                0x61
#define FPREQBOOTREASON           0x80
#define FPBOOTREASON              0x81
#define FPTIMERCLEAR              0x72
#define FPREQTIMERNEW             0x82 //experimental
#define FPTIMERNEW                0x83 //experimental
#define FPTIMERSET                0x84

#define RTC_NAME "tffp-rtc"
static struct platform_device *rtc_pdev;


typedef struct
{
	struct file*     fp;
	struct semaphore sem;
} tFrontPanelOpen;

static wait_queue_head_t wq;

struct semaphore rx_int_sem;

static time_t gmtWakeupTime = 0;

static tFrontPanelOpen FrontPanelOpen [LASTMINOR + 1];     //remembers the file handle for all minor numbers

byte save_bright; //remembers brightness level for VFDDISPLAYWRITEONOFF

typedef enum
{
	VFD_7,
	VFD_14,
	VFD_17
} DISPLAYTYPE;

static byte              DisplayBufferLED [4];
static byte              DisplayBufferVFD [48], DisplayBufferVFDCurrent [48];
static byte              RCVBuffer [BUFFERSIZE];
static int               RCVBufferStart = 0, RCVBufferEnd = 0;
static char              b[256];
static struct timer_list timer;
static byte              IconMask [] = {0xff, 0xcf, 0xff, 0xff, 0xff, 0x7f, 0xfe, 0x3f,
                                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0xfe, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff,
                                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0xfc, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff};
static byte              IconBlinkMode[48][4];
static byte              DefaultTypematicDelay = 3;
static byte              DefaultTypematicRate = 1;
static byte              KeyBuffer [256], KeyBufferStart = 0, KeyBufferEnd = 0;
static byte              KeyEmulationMode = 1; // Kathrein mode by default
static struct termios    termAttributes;
static tTffpConfig       tffpConfig;

static void SendFPByte(byte Data)
{
	byte         *ASC_3_TX_BUFF = (byte*)(ASC3BaseAddress + ASC_TX_BUFF);
	unsigned int *ASC_3_INT_STA = (unsigned int*)(ASC3BaseAddress + ASC_INT_STA);
	dword        Counter = 100000;

	while (((*ASC_3_INT_STA & ASC_INT_STA_THE) == 0) && --Counter)
	{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	// We are too fast, lets make a break
	udelay(0);
#endif
	}

	*ASC_3_TX_BUFF = Data;
}

static void SendFPString(byte *s)
{
	unsigned int i;

#ifdef DEBUG
	if (s)
	{
		printk("FP: SendString(SOP ");
		for (i = 0; i <= (s[0] & 0x0f); i++)
		{
			printk ("%2.2x ", s[i]);
		}
		printk("EOP)\n");
	}
#endif

	if (s)
	{
		SendFPByte (2);
		SendFPByte (s[0]);
		for (i = 1; i <= (s[0] & 0x0f); i++)
		{
			SendFPByte (s[i]);
		}
		SendFPByte (3);
	}
}

static void SendFPData(byte Len, byte Data, ...)
{
	unsigned int i;
	va_list      ap;
	byte         DispData [20];

#ifdef DEBUG
	printk("FP: SendFPData(Len=%d,...)\n", Len);
#endif

	DispData [0] = Data;
	va_start (ap, Data);
	for (i = 1; i < Len; i++)
	{
		DispData [i] = (byte) va_arg (ap, unsigned int);
	}
	va_end (ap);
	SendFPString (DispData);
}

static inline byte GetBufferByte(int Offset)
{
	return RCVBuffer[(RCVBufferEnd + Offset) & (BUFFERSIZE - 1)];
}

static void VFDSendToDisplay(bool Force)
{
	byte DispBuffer [10];

#ifdef DEBUG
	printk("FP: VFDSendToDisplay(Force=%s)\n", Force ? "true" : "false");
#endif

	DispBuffer[0] = 0x99;

	if (Force || memcmp (&DisplayBufferVFD[0x00], &DisplayBufferVFDCurrent[0x00], 8))
	{
		DispBuffer[1] = 0x00;
		memcpy(&DispBuffer[2], &DisplayBufferVFD[0x00], 8);
		SendFPString(DispBuffer);
	}

	if (Force || memcmp (&DisplayBufferVFD[0x08], &DisplayBufferVFDCurrent[0x08], 8))
	{
		DispBuffer[1] = 0x08;
		memcpy(&DispBuffer[2], &DisplayBufferVFD[0x08], 8);
		SendFPString(DispBuffer);
	}

	if (Force || memcmp (&DisplayBufferVFD[0x10], &DisplayBufferVFDCurrent[0x10], 8))
	{
		DispBuffer[1] = 0x10;
		memcpy(&DispBuffer[2], &DisplayBufferVFD[0x10], 8);
		SendFPString(DispBuffer);
	}

	if (Force || memcmp (&DisplayBufferVFD[0x18], &DisplayBufferVFDCurrent[0x18], 8))
	{
		DispBuffer[1] = 0x18;
		memcpy(&DispBuffer[2], &DisplayBufferVFD[0x18], 8);
		SendFPString(DispBuffer);
	}

	if (Force || memcmp (&DisplayBufferVFD[0x20], &DisplayBufferVFDCurrent[0x20], 8))
	{
		DispBuffer[1] = 0x20;
		memcpy(&DispBuffer[2], &DisplayBufferVFD[0x20], 8);
		SendFPString(DispBuffer);
	}

	if (Force || memcmp (&DisplayBufferVFD[0x28], &DisplayBufferVFDCurrent[0x28], 8))
	{
		DispBuffer[1] = 0x28;
		memcpy(&DispBuffer[2], &DisplayBufferVFD[0x28], 8);
		SendFPString(DispBuffer);
	}

	memcpy(DisplayBufferVFDCurrent, DisplayBufferVFD, sizeof (DisplayBufferVFD));
}

static void VFDClearBuffer(void)
{
#ifdef DEBUG
	printk("FP: VFDClearBuffer()\n");
#endif

	memset(DisplayBufferLED, 0, sizeof (DisplayBufferLED));
	memset(DisplayBufferVFD, 0, sizeof (DisplayBufferVFD));
	memset(DisplayBufferVFDCurrent, 0, sizeof (DisplayBufferVFDCurrent));
	SendFPData(2, 0x91, 0x00);
}

static void VFDInit(void)
{
#ifdef DEBUG
	printk("FP: VFDInit()\n");
#endif

	//Set bits 0 to 2 of PIO 5 for alternative functions (UART3)
	*(unsigned int*)(PIO5BaseAddress + PIO_CLR_PnC0) = 0x07;
	*(unsigned int*)(PIO5BaseAddress + PIO_CLR_PnC1) = 0x06;
	*(unsigned int*)(PIO5BaseAddress + PIO_SET_PnC1) = 0x01;
	*(unsigned int*)(PIO5BaseAddress + PIO_SET_PnC2) = 0x07;

	*(unsigned int*)(ASC3BaseAddress + ASC_INT_EN)   = 0x00000000;
	*(unsigned int*)(ASC3BaseAddress + ASC_CTRL)     = 0x00001589;
	*(unsigned int*)(ASC3BaseAddress + ASC_TIMEOUT)  = 0x00000010;
	*(unsigned int*)(ASC3BaseAddress + ASC_BAUDRATE) = 0x000000c9;
	*(unsigned int*)(ASC3BaseAddress + ASC_TX_RST)   = 0;
	*(unsigned int*)(ASC3BaseAddress + ASC_RX_RST)   = 0;
	VFDClearBuffer();
}

static unsigned int VFDTranslateSegments(byte Character, DISPLAYTYPE DisplayType)
{
	unsigned int code = 0;

	switch (DisplayType)
	{
		case VFD_7:
		{
			code = VFDSegmentMap7[Character];
			break;
		}
		case VFD_14:
		{
			code = VFDSegmentMap14[Character];
			break;
		}
		case VFD_17:
		{
			code = VFDSegmentMap17[Character];
			break;
		}
	}
	return code;
}

static void VFDSetDisplayDigit(byte Character, DISPLAYTYPE DisplayType, byte Digit)
{
	switch (DisplayType)
	{
		case VFD_7:
		{
			if (Digit < 4) DisplayBufferLED [Digit] = VFDTranslateSegments(Character, DisplayType);
			break;
		}
		case VFD_14:
		{
			unsigned int s = VFDTranslateSegments (Character, VFD_14);

			/*     7    6    5   4    3    2    1    0
			//01                      1d   1e   1c   1l
			//02   1m   1k  1g1  1g2  1b   1f   1j   1h
			//03   1i   1a  2d   2e   2c   2l   2m   2k
			//04   2g1  2g2 2b   2f   2j   2h   2i   2a
			//05   3d   3e  3c   3l   3m   3k   3g1
			//06   3g2  3b  3f   3j   3h   3i   3a
			//07        4d  4e   4c   4l   4m   4k
			//08   4g1  4g2  4b  4f   4j   4h   4i   4a */

			switch (Digit)
			{
			/*Source: m l k j i h g2 g1 f e d c b a*/
				case 0:
				{
					DisplayBufferVFD[1] = (DisplayBufferVFD[1] & 0xf0)
					                    | ((s >> 12) & 0x01)
					                    | ((s >>  1) & 0x02)
				                            | ((s >>  2) & 0x04)
				                            | ( s        & 0x08);

					DisplayBufferVFD[2] = ((s >>  8) & 0x01)
					                    | ((s >>  9) & 0x02)
					                    | ((s >>  3) & 0x04)
					                    | ((s <<  2) & 0x08)
					                    | ((s >>  3) & 0x10)
					                    | ((s >>  1) & 0x20)
					                    | ((s >>  5) & 0x40)
					                    | ((s >>  6) & 0x80);

					DisplayBufferVFD[3] = (DisplayBufferVFD[3] & 0x3f)
					                    | ((s <<  6) & 0x40)
					                    | ((s >>  2) & 0x80);
					break;
				}
				case 1:
				{
					DisplayBufferVFD[3] = (DisplayBufferVFD[3] & 0xc0)
					                    | ((s >> 11) & 0x01)
					                    | ((s >> 12) & 0x02)
					                    | ((s >> 10) & 0x04)
					                    | ((s <<  1) & 0x08)
					                    | ( s        & 0x10)
					                    | ((s <<  2) & 0x20);

					DisplayBufferVFD[4] = ( s        & 0x01)
					                    | ((s >>  8) & 0x02)
					                    | ((s >>  6) & 0x04)
					                    | ((s >>  7) & 0x08)
					                    | ((s >>  1) & 0x10)
					                    | ((s <<  4) & 0x20)
					                    | ((s >>  1) & 0x40)
					                    | ((s <<  1) & 0x80);
					break;
				}
				case 2:
				{
					DisplayBufferVFD[5] = (DisplayBufferVFD[5] & 0x80)
					                    | ((s >>  6) & 0x01)
					                    | ((s >> 10) & 0x02)
					                    | ((s >> 11) & 0x04)
					                    | ((s >>  9) & 0x08)
					                    | ((s <<  2) & 0x10)
					                    | ((s <<  1) & 0x20)
					                    | ((s <<  3) & 0x40);

					DisplayBufferVFD[6] = (DisplayBufferVFD[6] & 0x01)
					                    | ((s <<  1) & 0x02)
					                    | ((s >>  7) & 0x04)
					                    | ((s >>  5) & 0x08)
					                    | ((s >>  6) & 0x10)
					                    | ( s        & 0x20)
					                    | ((s <<  5) & 0x40)
					                    | ( s        & 0x80);
					break;
				}
				case 3:
				{
					DisplayBufferVFD[7] = (DisplayBufferVFD[7] & 0xc0)
					                    | ((s >> 11) & 0x01)
					                    | ((s >> 12) & 0x02)
					                    | ((s >> 10) & 0x04)
					                    | ((s <<  1) & 0x08)
					                    | ( s        & 0x10)
					                    | ((s <<  2) & 0x20);

					DisplayBufferVFD[8] = ( s        & 0x01)
					                    | ((s >>  8) & 0x02)
					                    | ((s >>  6) & 0x04)
					                    | ((s >>  7) & 0x08)
					                    | ((s >>  1) & 0x10)
					                    | ((s <<  4) & 0x20)
					                    | ((s >>  1) & 0x40)
					                    | ((s <<  1) & 0x80);
					break;
				}
			}
			break;
		}
		case VFD_17:
		{
			unsigned int s = VFDTranslateSegments (Character, VFD_17);

			/*Source: m   l k j i   h g3 g2 g1   f e d2 d1   c b a2 a1*/
			switch (Digit)
			{
				case 0:
				{
					DisplayBufferVFD[33] = (DisplayBufferVFD[33] & 0xd5)
					                     | ((s >>  5) & 0x02)
					                     | ((s >>  2) & 0x08)
					                     | ((s <<  1) & 0x20);

					DisplayBufferVFD[34] = (DisplayBufferVFD[34] & 0x55)
					                     | ((s >> 13) & 0x02)
                                                             | ((s >> 13) & 0x08)
					                     | ((s >> 10) & 0x20)
					                     | ((s <<  4) & 0x80);

					DisplayBufferVFD[35] = (DisplayBufferVFD[35] & 0x55)
					                     | ((s >>  1) & 0x02)
					                     | ((s >>  7) & 0x08)
					                     | ((s >>  4) & 0x20)
					                     | ((s >>  1) & 0x80);

					DisplayBufferVFD[36] = (DisplayBufferVFD[36] & 0x55)
					                     | ((s >> 11) & 0x02)
					                     | ((s >>  8) & 0x08)
					                     | ((s >>  8) & 0x20)
					                     | ( s        & 0x80);

					DisplayBufferVFD[37] = (DisplayBufferVFD[37] & 0x55)
					                     | ((s <<  4) & 0x20)
					                     | ((s <<  7) & 0x80);
					break;
				}
				case 1:
				{
					DisplayBufferVFD[17] = (DisplayBufferVFD[17] & 0xd5)
					                     | ((s >>  5) & 0x02)
					                     | ((s >>  2) & 0x08)
					                     | ((s <<  1) & 0x20);

					DisplayBufferVFD[18] = (DisplayBufferVFD[18] & 0x55)
					                     | ((s >> 13) & 0x02)
					                     | ((s >> 13) & 0x08)
					                     | ((s >> 10) & 0x20)
					                     | ((s <<  4) & 0x80);

					DisplayBufferVFD[19] = (DisplayBufferVFD[19] & 0x55)
					                     | ((s >>  1) & 0x02)
					                     | ((s >>  7) & 0x08)
					                     | ((s >>  4) & 0x20)
					                     | ((s >>  1) & 0x80);

					DisplayBufferVFD[20] = (DisplayBufferVFD[20] & 0x55)
					                     | ((s >> 11) & 0x02)
					                     | ((s >>  8) & 0x08)
					                     | ((s >>  8) & 0x20)
					                     | ( s        & 0x80);

					DisplayBufferVFD[21] = (DisplayBufferVFD[21] & 0x55)
					                     | ((s <<  4) & 0x20)
					                     | ((s <<  7) & 0x80);
					break;
				}
				case 2:
				{
					DisplayBufferVFD[17] = (DisplayBufferVFD[17] & 0xea)
					                     | ((s >>  6) & 0x01)
					                     | ((s >>  3) & 0x04)
					                     | ( s        & 0x10);

					DisplayBufferVFD[18] = (DisplayBufferVFD[18] & 0xaa)
					                     | ((s >> 14) & 0x01)
					                     | ((s >> 14) & 0x04)
					                     | ((s >> 11) & 0x10)
					                     | ((s <<  3) & 0x40);

					DisplayBufferVFD[19] = (DisplayBufferVFD[19] & 0xaa)
					                     | ((s >>  2) & 0x01)
					                     | ((s >>  8) & 0x04)
					                     | ((s >>  5) & 0x10)
					                     | ((s >>  2) & 0x40);

					DisplayBufferVFD[20] = (DisplayBufferVFD[20] & 0xaa)
					                     | ((s >> 12) & 0x01)
					                     | ((s >>  9) & 0x04)
					                     | ((s >>  9) & 0x10)
					                     | ((s >>  1) & 0x40);

					DisplayBufferVFD[21] = (DisplayBufferVFD[21] & 0xaf)
					                     | ((s <<  3) & 0x10)
					                     | ((s <<  6) & 0x40);
					break;
				}
				case 3:
				{
					DisplayBufferVFD[37] = (DisplayBufferVFD[37] & 0xf5)
					                     | ((s >>  4) & 0x02)
					                     | ((s >>  1) & 0x08);

					DisplayBufferVFD[38] = (DisplayBufferVFD[38] & 0x55)
					                     | ((s >> 15) & 0x02)
					                     | ((s >> 12) & 0x08)
					                     | ((s <<  2) & 0x20)
					                     | ((s <<  1) & 0x80);

					DisplayBufferVFD[39] = (DisplayBufferVFD[39] & 0x55)
					                     | ((s >>  9) & 0x02)
					                     | ((s >>  6) & 0x08)
					                     | ((s >>  3) & 0x20)
					                     | ((s >>  7) & 0x80);

					DisplayBufferVFD[40] = (DisplayBufferVFD[40] & 0x55)
					                     | ((s >> 10) & 0x02)
					                     | ((s >> 10) & 0x08)
					                     | ((s >>  2) & 0x20)
					                     | ((s <<  5) & 0x80);

					DisplayBufferVFD[41] = (DisplayBufferVFD[41] & 0x57)
					                     | ((s <<  2) & 0x08)
					                     | ((s <<  5) & 0x20)
					                     | ((s >>  5) & 0x80);
					break;
				}
				case 4:
				{
					DisplayBufferVFD[37] = (DisplayBufferVFD[37] & 0xfa)
					                     | ((s >>  5) & 0x01)
					                     | ((s >>  2) & 0x04);

					DisplayBufferVFD[38] = (DisplayBufferVFD[38] & 0xaa)
					                     | ((s >> 16) & 0x01)
					                     | ((s >> 13) & 0x04)
					                     | ((s <<  1) & 0x10)
					                     | ( s        & 0x40);

					DisplayBufferVFD[39] = (DisplayBufferVFD[39] & 0xaa)
					                     | ((s >> 10) & 0x01)
					                     | ((s >>  7) & 0x04)
					                     | ((s >>  4) & 0x10)
					                     | ((s >>  8) & 0x40);

					DisplayBufferVFD[40] = (DisplayBufferVFD[40] & 0xaa)
					                     | ((s >> 11) & 0x01)
					                     | ((s >> 11) & 0x04)
					                     | ((s >>  3) & 0x10)
					                     | ((s <<  4) & 0x40);

					DisplayBufferVFD[41] = (DisplayBufferVFD[41] & 0xab)
					                     | ((s <<  1) & 0x04)
					                     | ((s <<  4) & 0x10)
					                     | ((s >>  6) & 0x40);
					break;
				}
				case 5:
				{
					DisplayBufferVFD[21] = (DisplayBufferVFD[21] & 0xf5)
					                     | ((s >>  4) & 0x02)
					                     | ((s >>  1) & 0x08);

					DisplayBufferVFD[22] = (DisplayBufferVFD[22] & 0x55)
					                     | ((s >> 15) & 0x02)
					                     | ((s >> 12) & 0x08)
					                     | ((s <<  2) & 0x20)
					                     | ((s <<  1) & 0x80);

					DisplayBufferVFD[23] = (DisplayBufferVFD[23] & 0x55)
					                     | ((s >>  9) & 0x02)
					                     | ((s >>  6) & 0x08)
					                     | ((s >>  3) & 0x20)
					                     | ((s >>  7) & 0x80);

					DisplayBufferVFD[24] = (DisplayBufferVFD[24] & 0x55)
					                     | ((s >> 10) & 0x02)
					                     | ((s >> 10) & 0x08)
					                     | ((s >>  2) & 0x20)
					                     | ((s <<  5) & 0x80);

					DisplayBufferVFD[25] = (DisplayBufferVFD[25] & 0x57)
					                     | ((s <<  2) & 0x08)
					                     | ((s <<  5) & 0x20)
					                     | ((s >>  5) & 0x80);
					break;
				}
				case 6:
				{
					DisplayBufferVFD[21] = (DisplayBufferVFD[21] & 0xfa)
					                     | ((s >>  5) & 0x01)
					                     | ((s >>  2) & 0x04);

					DisplayBufferVFD[22] = (DisplayBufferVFD[22] & 0xaa)
					                     | ((s >> 16) & 0x01)
					                     | ((s >> 13) & 0x04)
					                     | ((s <<  1) & 0x10)
					                     | ( s        & 0x40);

					DisplayBufferVFD[23] = (DisplayBufferVFD[23] & 0xaa)
					                     | ((s >> 10) & 0x01)
					                     | ((s >>  7) & 0x04)
					                     | ((s >>  4) & 0x10)
					                     | ((s >>  8) & 0x40);

					DisplayBufferVFD[24] = (DisplayBufferVFD[24] & 0xaa)
					                     | ((s >> 11) & 0x01)
					                     | ((s >> 11) & 0x04)
					                     | ((s >>  3) & 0x10)
					                     | ((s <<  4) & 0x40);

					DisplayBufferVFD[25] = (DisplayBufferVFD[25] & 0xab)
					                     | ((s <<  1) & 0x04)
					                     | ((s <<  4) & 0x10)
					                     | ((s >>  6) & 0x40);
					break;
				}
				case 7:
				{
					DisplayBufferVFD[33] = (DisplayBufferVFD[33] & 0xea)
					                     | ((s >>  6) & 0x01)
					                     | ((s >>  3) & 0x04)
					                     | ( s        & 0x10);

					DisplayBufferVFD[34] = (DisplayBufferVFD[34] & 0xaa)
					                     | ((s >> 14) & 0x01)
					                     | ((s >> 14) & 0x04)
					                     | ((s >> 11) & 0x10)
					                     | ((s <<  3) & 0x40);

					DisplayBufferVFD[35] = (DisplayBufferVFD[35] & 0xaa)
					                     | ((s >>  2) & 0x01)
					                     | ((s >>  8) & 0x04)
					                     | ((s >>  5) & 0x10)
					                     | ((s >>  2) & 0x40);

					DisplayBufferVFD[36] = (DisplayBufferVFD[36] & 0xaa)
					                     | ((s >> 12) & 0x01)
					                     | ((s >>  9) & 0x04)
					                     | ((s >>  9) & 0x10)
					                     | ((s >>  1) & 0x40);

					DisplayBufferVFD[37] = (DisplayBufferVFD[37] & 0xaf)
					                     | ((s <<  3) & 0x10)
					                     | ((s <<  6) & 0x40);
					break;
				}
			}
			break;
		}
	}
}

static void VFDSetDisplaySmallString(char *s)
{
	unsigned int i, j;

#ifdef DEBUG
	printk("FP: VFDSetDisplaySmallString(s=%s)\n", s);
#endif

	if (s)
	{
		i = strlen (s);

		for (j = 0; j < 4; j++)
		{
			if (i > j)
			{
				VFDSetDisplayDigit(s[j], VFD_14, j);
			}
			else
			{
				VFDSetDisplayDigit(' ', VFD_14, j);
			}
		}
	}
	VFDSendToDisplay(false);
}

static void VFDSetDisplayLargeString(char *s)
{
	unsigned int i, j;
	char         *c;

#ifdef DEBUG
	printk("FP: VFDSetDisplayLargeString(s=%s)\n", s);
#endif

	if (s)
	{
		if (tffpConfig.allCaps)
		{
			c = s;
			while (*c)
			{
				*c = toupper(*c);
				c++;
			}
		}

		i = strlen (s);

		for (j = 0; j < 8; j++)
		{
			if (i > j)
			{
				VFDSetDisplayDigit(s[j], VFD_17, j);
			}
			else
			{
				VFDSetDisplayDigit(' ', VFD_17, j);
			}
		}
	}
	VFDSendToDisplay(false);
}

static void TranslateUTFString(unsigned char *s)
{
	//This routine modifies a string buffer, so that multibyte UTF strings are converted into single byte...
	//Characters, which do not fit into 8 bit are replaced by '*'

	unsigned char *Source;
	unsigned char *Dest;

	if (s && s[0])
	{
		Source = &s[0];
		Dest = &s[0];

		while (*Source)
		{
			if (*Source > 0xbf)
			{
				// Hit a Unicode character.
				// Translatable characters can be found between C280 and C3BF.
				if (*Source > 0xc3)
				{
					//Invalid character
					*Dest = '*';
					if ((*Source & 0xe0) == 0xc0)
					{
						Source++;
					}
					else if ((*Source & 0xf0) == 0xe0)
					{
						Source++;
						if (*Source)
						{
							Source++;
						}
					}
					else if ((*Source & 0xf8) == 0xf0)
					{
						Source++;
						if (*Source)
						{
							Source++;
						}
						if (*Source)
						{
							Source++;
						}
					}
				}
				else
				{
					*Dest = ((Source[0] & 0x03) << 6) | (Source[1] & 0x3f);
					Source++;
				}
			}
			else
			{
				*Dest = *Source;
			}
			if (*Source)
			{
				Source++;
				Dest++;
			}
		}
		*Dest = '\0';
	}
}

/**********************************************************************************************/
/* Code for playing with the time on the FP                                                   */
/**********************************************************************************************/
char                           sdow[][4] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
static frontpanel_ioctl_time   fptime;
int                            FPREQDATETIMENEWPending = 0;
static DECLARE_WAIT_QUEUE_HEAD (FPREQDATETIMENEW_queue);
int                            FPREQTIMERNEWPending = 0;
static DECLARE_WAIT_QUEUE_HEAD (FPREQTIMERNEW_queue);

frontpanel_ioctl_time *VFDReqDateTime(void)
{
#ifdef DEBUG
	printk("FP: VFDReqDateTime()\n");
#endif

	FPREQDATETIMENEWPending = 1;
	SendFPData(1, FPREQDATETIMENEW);
	wait_event_interruptible_timeout(FPREQDATETIMENEW_queue, !FPREQDATETIMENEWPending, HZ);

	return &fptime;
}

frontpanel_ioctl_time *VFDReqTimerDateTime(void)
{
#ifdef DEBUG
	printk("FP: VFDReqTimerDateTime()\n");
#endif

	FPREQTIMERNEWPending = 1;
	SendFPData(1, FPREQTIMERNEW);
	wait_event_interruptible_timeout(FPREQTIMERNEW_queue, !FPREQTIMERNEWPending, HZ);

	return &fptime;
}
static void InterpretDateTime(void)
{
	word w;

#ifdef DEBUG
	printk("FP: InterpretDateTime()\n");
#endif

	w = (GetBufferByte (2) << 8) | GetBufferByte(3);

	fptime.year  = (w >> 9) + 2000;
	fptime.month = (w >> 5) & 0x0f;
	fptime.day   = w & 0x1f;
	fptime.dow   = GetBufferByte(4) >> 5;
	sprintf (fptime.sdow, sdow[fptime.dow]);

	fptime.hour  = GetBufferByte(4) & 0x1f;
	fptime.min   = GetBufferByte(5);
	fptime.sec   = GetBufferByte(6);
	fptime.now = mktime(fptime.year, fptime.month, fptime.day, fptime.hour, fptime.min, fptime.sec);

	printk ("FP: clock set to %s %2.2d-%2.2d-%4.4d %2.2d:%2.2d:%2.2d\n", fptime.sdow, fptime.day, fptime.month, fptime.year, fptime.hour, fptime.min, fptime.sec);

	FPREQDATETIMENEWPending = 0;
	wake_up_interruptible(&FPREQDATETIMENEW_queue);
}

void VFDSetTime(frontpanel_ioctl_time *fptime)
{
	char s[6];
	word w;

#ifdef DEBUG
	printk("FP: VFDSetTime(*fptime=%p)\n", fptime);
#endif
/*
 w is a 16 bit sequence representing the date, consisting of:
   7 bits: year minus 2000
   4 bits: month
   5 bits: day
 */
	w = (((fptime->year - 2000) & 0x7f) << 9) | ((fptime->month & 0x0f) << 5) | (fptime->day & 0x1f);
	s[0] = FPDATETIMENEW;
	s[1] = w >> 8;
	s[2] = w & 0xff;
	s[3] = fptime->hour;
	s[4] = fptime->min;
	s[5] = fptime->sec;
	SendFPString(s);
}

/**********************************************************************************************/
/* Code for remote and frontpanel buttons                                                     */
/**********************************************************************************************/
unsigned char ufs910map[256] =
{
	0x58, 0x59, 0x5b, 0x5a, 0xff, 0xff, 0x39, 0xff, // 0x00 - 0x07
	0xff, 0xff, 0xff, 0xff, 0x0d, 0x6e, 0x6f, 0x70,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, // 0x10
	0x08, 0x09, 0x54, 0x4c, 0x55, 0x0f, 0xff, 0x5c,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0x20
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0x30
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0x21, 0x38, 0x3c, // 0x40
	0x20, 0xff, 0x31, 0x37, 0xff, 0x6d, 0xff, 0xff,
	0x11, 0xff, 0x10, 0xff, 0xff, 0xff, 0xff, 0xff, // 0x50
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0x60
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0x70
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static inline void AddKeyBuffer(byte Key)
{
	int tmp = (KeyBufferStart + 1) & 0xff;
#ifdef DEBUG
	printk("FP: AddKeyBuffer(Key=%x)\n", Key);
#endif

	/* check for buffer overflow, if it is the case then discard the new character */
	if (tmp != KeyBufferEnd)
	{
		/* no overflow */
		KeyBuffer[KeyBufferStart] = Key;
		KeyBufferStart = tmp;
	}
}

static void AddKeyToBuffer(byte Key, bool FrontPanel)
{
//TODO: add feedback
	switch (KeyEmulationMode)
	{
		case KEYEMUTF7700:
		case KEYEMUTF7700LKP:
		{
			AddKeyBuffer(FrontPanel ? FPKEYPRESSFP : FPKEYPRESS);
			AddKeyBuffer(Key);
			wake_up_interruptible(&wq);
			break;
		}
		case KEYEMUUFS910:
		{
			if (ufs910map[Key] == 0xff)
			{
				printk("FP: undefined UFS910 key (TF=0x%2.2x)\n", Key);
			}
			else
			{
				AddKeyBuffer (ufs910map[Key]);
				wake_up_interruptible(&wq);
			}
			break;
		}
	}
}

static void InterpretKeyPresses(void)
{
	static byte TypematicDelay = 0;
	static byte TypematicRate = 0;
	byte        Key;

	Key = GetBufferByte (2);
	if (KeyEmulationMode == KEYEMUTF7700LKP)
	{
		AddKeyToBuffer(Key, GetBufferByte(1) == FPKEYPRESSFP);
	}
	else
	{
		if ((Key & 0x80) == 0) //First burst
		{
			AddKeyToBuffer(Key, GetBufferByte(1) == FPKEYPRESSFP);
			TypematicDelay = DefaultTypematicDelay;
			TypematicRate  = DefaultTypematicRate;
		}
		else
		{
			if (TypematicDelay == 0)
			{
				if (TypematicRate == 0)
				{
					AddKeyToBuffer(Key & 0x7f, GetBufferByte(1) == FPKEYPRESSFP);
					TypematicRate = DefaultTypematicRate;
				}
				else
				{
					TypematicRate--;
				}
			}
			else
			{
				TypematicDelay--;
			}
		}
	}
}

/**********************************************************************************************/
/* Code for getting the boot reason from the FP                                               */
/*                                                                                            */
/* As the 'boot reason' doesn't change during the systems up time, it's requested only once   */
/*                                                                                            */
/**********************************************************************************************/
static frontpanel_ioctl_bootreason fpbootreason;

static void VFDGetBootReason(void)
{
#ifdef DEBUG
	printk("FP: VFDGetBootReason()\n");
#endif

	SendFPData(1, FPREQBOOTREASON);
}

static void InterpretBootReason(void)
{
#ifdef DEBUG
	printk("FP: InterpretBootReason()\n");
#endif

	fpbootreason.reason = GetBufferByte(2);
}

int getBootReason()
{
	return fpbootreason.reason;
}

/**********************************************************************************************/
/* Code for turning off the power via the FP                                                  */
/**********************************************************************************************/
static void VFDShutdown(void)
{
#ifdef DEBUG
	printk("FP: VFDShutdown()\n");
#endif

	SendFPData(2, 0x21, 0x01);
}

static void VFDReboot(void)
{
#ifdef DEBUG
	printk("FP: VFDReboot()\n");
#endif

	SendFPData(2, 0x31, 0x06);
}

/**********************************************************************************************/
/* Code for setting the front panel VFD brightness                                            */
/**********************************************************************************************/
static void VFDBrightness(byte Brightness)
{
// BUG/TODO: level 0 is max brightness
	byte BrightData[]={0x00, 0x01, 0x02, 0x08, 0x10, 0x20};

#ifdef DEBUG
	printk("FP: VFDBrightness (Brightness = %d)\n", Brightness);
#endif

	if ((Brightness >= 0) && (Brightness < 6))
	{
		SendFPData(3, 0xa2, 0x08, BrightData[Brightness]);
	}
}

/**********************************************************************************************/
/* Code for working with timers                                                               */
/**********************************************************************************************/
static void VFDClearTimers(void)
{
#ifdef DEBUG
	printk("FP: VFDClearTimers()\n");
#endif

	SendFPData(3, FPTIMERCLEAR, 0xff, 0xff);
}

void VFDSetTimer(frontpanel_ioctl_time *fptime)
{
	char s[6];
	word w;

#ifdef DEBUG
	printk("FP: VFDSetTimer (pfptime = %p)\n", fptime);
#endif

	w = (((fptime->year - 2000) & 0x7f) << 9)
	  | ((fptime->month & 0x0f) << 5)
	  | (fptime->day & 0x1f);

	s[0] = FPTIMERSET;
	s[1] = w >> 8;
	s[2] = w & 0xff;
	s[3] = fptime->hour;
	s[4] = fptime->min;
	SendFPString(s);
}

/**********************************************************************************************/
/* Code for enabling and disabling IR remote modes                                            */
/**********************************************************************************************/
byte FPIRData [] = {0xa5, 0x00, 0x02, 0x34, 0x0a, 0x00,
                    0xa5, 0x01, 0x49, 0x00, 0x0a, 0x00,
                    0xa5, 0x02, 0x49, 0x99, 0x0a, 0x00,
                    0xa5, 0x03, 0x20, 0xdf, 0x0a, 0x00};

static void VFDSetIRMode(byte Mode, byte OnOff)
{
#ifdef DEBUG
	printk("FP: VFDSetIRMode(Mode0%d, OnOff=%d)\n", Mode, OnOff);
#endif

	if (Mode < 4)
	{
		FPIRData [Mode * 6 + 1] = (FPIRData [Mode * 6 + 1] & 0x0f) | (OnOff << 4);
		FPIRData [Mode * 6 + 5] = (FPIRData [Mode * 6 + 1] + FPIRData [Mode * 6 + 2] + FPIRData [Mode * 6 + 3] + FPIRData [Mode * 6 + 4] - 1) & 0xff;
		SendFPString(&FPIRData [Mode * 6]);
	}
}

/**********************************************************************************************/
/* Code for showing the icons                                                                 */
/**********************************************************************************************/
static inline void SetIconBits(byte Reg, byte Bit, byte Mode)
{
	byte i, j;

	i = 1 << Bit;
	j = i ^ 0xff;
	IconBlinkMode[Reg][0] = (IconBlinkMode[Reg][0] & j) | ((Mode & 0x01) ? i : 0);
	IconBlinkMode[Reg][1] = (IconBlinkMode[Reg][1] & j) | ((Mode & 0x02) ? i : 0);
	IconBlinkMode[Reg][2] = (IconBlinkMode[Reg][2] & j) | ((Mode & 0x04) ? i : 0);
	IconBlinkMode[Reg][3] = (IconBlinkMode[Reg][3] & j) | ((Mode & 0x08) ? i : 0);
}

static void VFDSetIcons(dword Icons1, dword Icons2, byte BlinkMode)
{
#ifdef DEBUG
	printk("FP: VFDSetIcons(Icons1=0x%08x, Icons2=0x%08x, BlinkMode=%d)\n", (int)Icons1, (int)Icons2, BlinkMode);
#endif

	if (Icons1 & FPICON_IRDOT)        SetIconBits( 1, 4, BlinkMode);
	if (Icons1 & FPICON_POWER)        SetIconBits( 1, 5, BlinkMode);
	if (Icons1 & FPICON_COLON)        SetIconBits( 5, 7, BlinkMode);
	if (Icons1 & FPICON_PM)           SetIconBits( 6, 0, BlinkMode);
	if (Icons1 & FPICON_TIMER)        SetIconBits( 7, 6, BlinkMode);
	if (Icons1 & FPICON_AM)           SetIconBits( 7, 7, BlinkMode);
	if (Icons1 & FPICON_AC3)          SetIconBits(27, 0, BlinkMode);
	if (Icons1 & FPICON_TIMESHIFT)    SetIconBits(27, 1, BlinkMode);
	if (Icons1 & FPICON_TV)           SetIconBits(27, 2, BlinkMode);
	if (Icons1 & FPICON_MUSIC)        SetIconBits(27, 3, BlinkMode);
	if (Icons1 & FPICON_DISH)         SetIconBits(27, 4, BlinkMode);
	if (Icons1 & FPICON_MP3)          SetIconBits(28, 7, BlinkMode);
	if (Icons1 & FPICON_TUNER1)       SetIconBits(41, 0, BlinkMode);
	if (Icons1 & FPICON_REC)          SetIconBits(41, 1, BlinkMode);
	if (Icons1 & FPICON_PAUSE)        SetIconBits(42, 1, BlinkMode);
	if (Icons1 & FPICON_FWD)          SetIconBits(42, 2, BlinkMode);
	if (Icons1 & FPICON_xxx2)         SetIconBits(42, 3, BlinkMode);
	if (Icons1 & FPICON_PLAY)         SetIconBits(42, 4, BlinkMode);
	if (Icons1 & FPICON_xxx4)         SetIconBits(42, 5, BlinkMode);
	if (Icons1 & FPICON_RWD)          SetIconBits(42, 6, BlinkMode);
	if (Icons1 & FPICON_TUNER2)       SetIconBits(42, 7, BlinkMode);
	if (Icons1 & FPICON_ATTN)         SetIconBits(43, 4, BlinkMode);
	if (Icons1 & FPICON_AUTOREWLEFT)  SetIconBits(43, 7, BlinkMode);
	if (Icons1 & FPICON_AUTOREWRIGHT) SetIconBits(43, 6, BlinkMode);
	if (Icons1 & FPICON_DOLBY)        SetIconBits(43, 3, BlinkMode);
	if (Icons1 & FPICON_DOLLAR)       SetIconBits(43, 5, BlinkMode);
	if (Icons1 & FPICON_MUTE)         SetIconBits(42, 0, BlinkMode);
	if (Icons1 & FPICON_NETWORK)      SetIconBits(43, 2, BlinkMode);
	if (Icons2 & FPICON_CD12)         SetIconBits(25, 0, BlinkMode);
	if (Icons2 & FPICON_CDCENTER)     SetIconBits(25, 1, BlinkMode);
	if (Icons2 & FPICON_CD8)          SetIconBits(26, 0, BlinkMode);
	if (Icons2 & FPICON_CD7)          SetIconBits(26, 1, BlinkMode);
	if (Icons2 & FPICON_CD6)          SetIconBits(26, 2, BlinkMode);
	if (Icons2 & FPICON_CD5)          SetIconBits(26, 3, BlinkMode);
	if (Icons2 & FPICON_CD4)          SetIconBits(26, 4, BlinkMode);
	if (Icons2 & FPICON_CD3)          SetIconBits(26, 5, BlinkMode);
	if (Icons2 & FPICON_CD2)          SetIconBits(26, 6, BlinkMode);
	if (Icons2 & FPICON_CD1)          SetIconBits(26, 7, BlinkMode);
	if (Icons2 & FPICON_CD11)         SetIconBits(27, 5, BlinkMode);
	if (Icons2 & FPICON_CD10)         SetIconBits(27, 6, BlinkMode);
	if (Icons2 & FPICON_CD9)          SetIconBits(27, 7, BlinkMode);
	if (Icons2 & FPICON_HDD4)         SetIconBits(28, 0, BlinkMode);
	if (Icons2 & FPICON_HDD5)         SetIconBits(28, 1, BlinkMode);
	if (Icons2 & FPICON_HDD6)         SetIconBits(28, 2, BlinkMode);
	if (Icons2 & FPICON_HDD7)         SetIconBits(28, 3, BlinkMode);
	if (Icons2 & FPICON_HDD8)         SetIconBits(28, 4, BlinkMode);
	if (Icons2 & FPICON_HDDFRAME)     SetIconBits(28, 5, BlinkMode);
	if (Icons2 & FPICON_HDDFULL)      SetIconBits(28, 6, BlinkMode);
	if (Icons2 & FPICON_HDD)          SetIconBits(29, 4, BlinkMode);
	if (Icons2 & FPICON_HDD1)         SetIconBits(29, 5, BlinkMode);
	if (Icons2 & FPICON_HDD2)         SetIconBits(29, 6, BlinkMode);
	if (Icons2 & FPICON_HDD3)         SetIconBits(29, 7, BlinkMode);
}

/**********************************************************************************************/
/* Spinner task                                                                               */
/**********************************************************************************************/
static void CD_icons_onoff(byte onoff)
{
	int i;

	SetIconBits(26, 7, onoff); //CD segment 1
	SetIconBits(26, 6, onoff);

	SetIconBits(26, 5, onoff); //CD segment 2

	SetIconBits(26, 4, onoff); //CD segment 3

	SetIconBits(26, 3, onoff); //CD segment 4

	SetIconBits(26, 2, onoff); //CD segment 5

	SetIconBits(26, 1, onoff); //CD segment 6
	SetIconBits(26, 0, onoff);

	SetIconBits(27, 7, onoff); //CD segment 7

	SetIconBits(27, 6, onoff); //CD segment 8

	SetIconBits(27, 5, onoff); //CD segment 9

	SetIconBits(25, 0, onoff); //CD segment 10

	for (i = 0; i < 11; i++)
	{
		Segment_flag[i] = onoff ? 1 : 0;
	}
}

static void SpinnerTimer(void)
{
	// called every 10ms
	static int cntdown = 0;
	byte on_off;

	if (!Spinner_state && Spinner_on) //Spinner was switched on
	{
//		SetIconBits(25, 1, 0xf); //switch CD center on
		CD_icons_onoff(0); //start with all segments off
		Spinner_state = 1; //and state 1
		cntdown = 0;
	}

	if (Spinner_state && !Spinner_on) //Spinner was switched off
	{
//		SetIconBits(25, 1, 0); //switch CD center off
		CD_icons_onoff(0); //switch all segments off
		Spinner_state = 0;
	}

	if (Spinner_on)
	{
		if (cntdown >= 20) // one cycle will last 20 x 10ms x 10 states = 2s
		{
			on_off = Segment_flag[Spinner_state] ? 0 : 0xf;

			switch (Spinner_state)
			{
				case 1:
				{
					SetIconBits(26, 7, on_off); //CD segment 1
					SetIconBits(26, 6, on_off);
					break;
				}
				case 2:
				{
					SetIconBits(26, 5, on_off); //CD segment 2
					break;
				}
				case 3:
				{
					SetIconBits(26, 4, on_off); //CD segment 3
					break;
				}
				case 4:
				{
					SetIconBits(26, 3, on_off); //CD segment 4
					break;
				}
				case 5:
				{
					SetIconBits(26, 2, on_off); //CD segment 5
					break;
				}
				case 6:
				{
					SetIconBits(26, 1, on_off); //CD segment 6
					SetIconBits(26, 0, on_off);
					break;
				}
				case 7:
				{
					SetIconBits(27, 7, on_off); //CD segment 7
					break;
				}
				case 8:
				{
					SetIconBits(27, 6, on_off); //CD segment 8
					break;
				}
				case 9:
				{
					SetIconBits(27, 5, on_off); //CD segment 9
					break;
				}
				case 10:
				{
					SetIconBits(25, 0, on_off); //CD segment 10
					break;
				}
			}
			Segment_flag[Spinner_state] = !Segment_flag[Spinner_state];

			Spinner_state++;
			if (Spinner_state == 11)
			{
				Spinner_state = 1;
			}

			cntdown = 0;
		}
		else
		{
			cntdown++;
		}
	}
}

/**********************************************************************************************/
/* Scrolling task                                                                             */
/**********************************************************************************************/
typedef enum
{
	SS_InitChars,
	SS_WaitPauseTimeout,
	SS_ScrollText,
	SS_WaitDelayTimeout,
	SS_ShortString
} eScrollState;

static frontpanel_ioctl_scrollmode ScrollMode;
static int                         ScrollOnce = 1;
static eScrollState                ScrollState = SS_InitChars;
static char                        Message[40], ScrollText[40];

static void ScrollTimer(void)
{
	static int CurrentDelay = 0;

	switch(ScrollState)
	{
		case SS_InitChars:
		{
			//Display the first 8 chars and set the delay counter
			//to the delay to the first scroll movement
			strncpy(ScrollText, Message, sizeof(ScrollText));
			ScrollText[sizeof(ScrollText) - 1] = '\0';
			VFDSetDisplayLargeString(ScrollText);
			CurrentDelay = ScrollMode.ScrollPause;

			if (strlen(Message) <= 8)
			{
				//No need to scroll
				ScrollState = SS_ShortString;
				break;
			}
			//Check if the scroll mode has changed in the meantime
			switch(ScrollMode.ScrollMode)
			{
				case 0:
				{
					ScrollState = SS_ShortString;
					break;
				}
				case 1:
				{
					if (ScrollOnce)
					{
						ScrollState = SS_WaitPauseTimeout;
						ScrollOnce = 0;
					}
					else
					{
						ScrollState = SS_ShortString;
					}
					break;
				}
				case 2:
				{
					ScrollState = SS_WaitPauseTimeout;
					break;
				}
			}
			break;
		}
		case SS_WaitPauseTimeout:
		{
			//Wait until the counter reaches zero, then start the scroll
			CurrentDelay--;
			if (CurrentDelay == 0)
			{
				ScrollState = SS_ScrollText;
			}
			break;
		}
		case SS_ScrollText:
		{
			char *s, *d;

			if (ScrollText[0])
			{
				//Shift the text, display it and wait ScrollDelay
				d = &ScrollText[0];
				s = &ScrollText[1];
				do
				{
					*d = *s;
					s++;
					d++;
				}
				while (*d);

				VFDSetDisplayLargeString(ScrollText);

				CurrentDelay = ScrollMode.ScrollDelay;
				ScrollState = SS_WaitDelayTimeout;
			}
			else
			{
				//Text has completely fallen out on the left side of the VFD
				ScrollState = SS_InitChars;
			}
			break;
		}
		case SS_WaitDelayTimeout:
		{
			CurrentDelay--;
			if (CurrentDelay == 0)
			{
				ScrollState = SS_ScrollText;
			}
			break;
		}
		case SS_ShortString:
		{
			//Text fits into the 8 char display, no need to do anything
			break;
		}
	}
}

static void ShowText(char *text)
{
	int i;

	//Save the text and reset the scroll state
	memset(Message, 0, sizeof(Message));
	strncpy(Message, text, sizeof(Message) - 1);

	//Trim trailing spaces
	if (*Message)
	{
		i = strlen(Message) - 1;
		while ((i > 0) && Message[i] == ' ')
		{
			i--;
		}
		Message[i + 1] = '\0';
	}

	ScrollState = SS_InitChars;
	ScrollOnce = 1;
}

/**********************************************************************************************/
/* Other code                                                                                 */
/**********************************************************************************************/
#if 0
static void VFDGetControl(void)
{
#ifdef DEBUG
	printk("FP: VFDGetControl()\n");
#endif

	SendFPData(1, 0x40);
}
#endif

static int frontpanel_init_func(void)
{
#ifdef DEBUG
	printk("FP: frontpanel_init_func()\n");
#endif

	printk("FP: Topfield TF77X0HDPVR front panel module %s initializing\n", VERSION);

	VFDInit();
//	ShowText("LINUX");

	return 0;
}

static void FPCommandInterpreter(void)
{
	int Cmd, CmdLen, i;

#ifdef DEBUG
	printk("FP: FPCommandInterpreter()\n");
#endif

	Cmd    = GetBufferByte(1);
	CmdLen = Cmd & 0x0f;

	switch (Cmd)
	{
		case FPSHUTDOWN:
		{
			static byte TypematicRate = 0;
#ifdef DEBUG
			printk("FP: FPSHUTDOWN\n");
#endif

			if (KeyEmulationMode == KEYEMUTF7700LKP)
			{
				AddKeyBuffer (FPKEYPRESSFP);
				AddKeyBuffer (0x0c); //KEY_POWER_FAKE
				wake_up_interruptible(&wq);
			}
			else
			{
				if (TypematicRate < 1)
				{
					/* send standby key to the application */
					if (KeyEmulationMode == KEYEMUTF7700)
					{
						AddKeyBuffer (FPKEYPRESSFP);
					}
					AddKeyBuffer (0x0c);
					TypematicRate = DefaultTypematicRate;
					wake_up_interruptible(&wq);
				}
				else
				{
					TypematicRate--;
				}
			}
			break;
		}
		case FPDATETIMENEW:
		{
			InterpretDateTime();
			//update_persistent_clock(now);
			break;
		}
		case FPBOOTREASON:
		{
			InterpretBootReason();
			break;
		}
		case FPTIMERNEW:
		{
			InterpretDateTime();
			//update_persistent_clock(now);
			break;
		}
		case FPKEYPRESS:
		case FPKEYPRESSFP:
		{
			/* reset the bootreason to disable autoshutdown if a key was pressed */
			if ( fpbootreason.reason )
			{
				fpbootreason.reason = 0;
				// Switch off power icon to indicate that no automatic shutdown for auto timers will be done
				VFDSetIcons(FPICON_POWER, 0x0, 0x0);
			}
			SetIconBits(1, 4, 0x0f);
			InterpretKeyPresses();
			SetIconBits(1, 4, 0);

			break;
		}
		default:
		{
			b[0] = '\0';
			for (i = 1; i <= CmdLen + 1; i++)
			{
				sprintf (&b[strlen(b)], "%2.2x ", GetBufferByte(i));
			}
			//printk ("FP: unknown cmd %s\n", b);
		}
	}
}

int fpTask(void * dummy)
{
	int i, j, BuffSize, CmdLen;

	daemonize("frontpanel");

	allow_signal(SIGTERM);

	while (1)
	{
		int dataAvailable;

		if (down_interruptible (&rx_int_sem))
		{
			break;
		}
		dataAvailable = 1;

		while (dataAvailable)
		{
			//Check for a valid RCV buffer
			//Find the next 0x02
			i = RCVBufferEnd;
			while ((i != RCVBufferStart) && (RCVBuffer[i] != 2))
			{
				i = (i + 1) & (BUFFERSIZE - 1);
			}
			if (RCVBufferEnd != i)
			{
				//There is spurious data without the SOT (0x02). Ignore leading 0x00
				b[0] = '\0';
				j = RCVBufferEnd;
				while (j != i)
				{
					if (b[0] || RCVBuffer[j])
					{
						sprintf (&b[strlen(b)], "%2.2x ",  RCVBuffer[j]);
					}
					j = (j + 1) & (BUFFERSIZE - 1);
				}
				if (b[0])
				{
					printk ("FP: Invalid data %s\n", b);
				}
				RCVBufferEnd = i;
			}

			//Reached end of buffer with locating a 0x02?
			if (RCVBufferEnd != RCVBufferStart)
			{
				//check if there are already enough bytes for a full command
				BuffSize = RCVBufferStart - RCVBufferEnd;
				if (BuffSize < 0)
				{
					BuffSize = BuffSize + BUFFERSIZE;
				}
				if (BuffSize > 2)
				{
					CmdLen = GetBufferByte(1) & 0x0f;
					if (BuffSize >= CmdLen + 3)
					{
						//Enough data to interpret command
						FPCommandInterpreter();
						RCVBufferEnd = (RCVBufferEnd + CmdLen + 3) & (BUFFERSIZE - 1);
					}
					else
					{
						dataAvailable = 0;
					}
				}
				else
				{
					dataAvailable = 0;
				}
			}
			else
			{
				dataAvailable = 0;
			}
		}
	}
	return 0;
}

/**********************************************************************************************/
/* Interrupt handler                                                                          */
/*                                                                                            */
/**********************************************************************************************/
static irqreturn_t FP_interrupt(int irq, void *dev_id)
{
	byte         *ASC_3_RX_BUFF = (byte*)(ASC3BaseAddress + ASC_RX_BUFF);
	unsigned int *ASC_3_INT_STA = (unsigned int*)(ASC3BaseAddress + ASC_INT_STA);
	int dataArrived = 0;
	static int msgStart = 0;

#ifdef DEBUG
	printk("FP: FP_interrupt(irq=%d, *dev_id=%p\n", irq, dev_id);
#endif

	// copy data into the ring buffer
	while (*ASC_3_INT_STA & ASC_INT_STA_RBF)
	{
		RCVBuffer [RCVBufferStart] = *ASC_3_RX_BUFF;
		if (RCVBuffer[RCVBufferStart] == 0x02)
		{
			msgStart = 1;
		}
		else
		{
			if (msgStart && (RCVBuffer[RCVBufferStart] == 0x20))
			{
				SendFPData(2, FPSHUTDOWNACK, 0x02);
				SendFPData(1, FPGETDISPLAYCONTROL);
			}
			msgStart = 0;
		}
		RCVBufferStart = (RCVBufferStart + 1) % BUFFERSIZE;

		dataArrived = 1;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		// We are too fast, lets make a break
		udelay(0);
#endif

		if (RCVBufferStart == RCVBufferEnd)
		{
			printk ("FP: RCV buffer overflow!!!\n");
		}
	}
	// signal the data arrival
	if (dataArrived)
	{
		up(&rx_int_sem);
	}
	return IRQ_HANDLED;
}

static void FP_Timer(void)
{
	static byte DelayCounter = 0;
	int    i;

	for (i = 0; i < 48; i++)
	{
		DisplayBufferVFD[i] = (DisplayBufferVFD[i] & IconMask[i]) | IconBlinkMode[i][DelayCounter];
	}

	VFDSendToDisplay(false);

	DelayCounter = (DelayCounter + 1) & 0x03;
}

// This is a 10ms timer. It is responsible to call ScrollTimer() and SpinnerTimer() every time
// and FP_Timer() every 250ms
static void HighRes_FP_Timer(dword arg)
{
	static int Counter = 0;

	ScrollTimer();
	SpinnerTimer();

	if (Counter > 24)
	{
		FP_Timer();
		Counter = 0;
	}
	else
	{
		Counter++;
	}

	/* fill the data for our timer function */
	init_timer(&timer);
	timer.data = 0;
	timer.function = HighRes_FP_Timer;
	timer.expires = jiffies + (HZ / 100);
	add_timer(&timer);
}

/**********************************************************************************************/
/* fops                                                                                       */
/*                                                                                            */
/**********************************************************************************************/
static ssize_t FrontPaneldev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char* kernel_buf = kmalloc(len, GFP_KERNEL);
	int   i;

#ifdef DEBUG
	printk("FP: FrontPaneldev_write(*filp=%p, *buff=%p, len=%d, off=%d", filp, buff, (int)len, (int)*off);
#endif

	for (i = 0; i <= LASTMINOR; i++)
	{
		if (FrontPanelOpen[i].fp == filp)
		{

#ifdef DEBUG
			printk (", minor=%d)\n", i);
#endif

			if (kernel_buf == NULL)
			{
				return -1;
			}
			copy_from_user(kernel_buf, buff, len);
			kernel_buf[len - 1] = '\0';

			switch (i)
			{
				case FRONTPANEL_MINOR_FPC:        //IOCTL only
				case FRONTPANEL_MINOR_RC:         //read-only
				{
					len = -1;
					break;
				}
				case FRONTPANEL_MINOR_FPLARGE:
				{
					TranslateUTFString(kernel_buf);

					//ShowText() replaces VFDSetDisplayLargeString() to incorporate scrolling
					ShowText(kernel_buf);
					//VFDSetDisplayLargeString (kernel_buf);
					break;
				}
				case FRONTPANEL_MINOR_FPSMALL:
				{
					VFDSetDisplaySmallString (kernel_buf);
					break;
				}
			}
			kfree (kernel_buf);
			return len;
		}
	}
	kfree (kernel_buf);
	return 0;
}

static unsigned int FrontPaneldev_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(filp, &wq, wait);
	if (KeyBufferStart != KeyBufferEnd)
	{
		mask = POLLIN | POLLRDNORM;
	}
	return mask;
}

static ssize_t FrontPaneldev_read(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char kernel_buff [16];
	int  i, j;

#ifdef DEBUG
	printk("FP: FrontPaneldev_read(*filp=%p, *buff=%p, len=%d, off=%d", filp, buff, (int)len, (int)*off);
#endif

	for (i = 0; i <= LASTMINOR; i++)
	{
		if (FrontPanelOpen[i].fp == filp)
		{
#ifdef DEBUG
			printk (", minor=%d)\n", i);
#endif

			switch (i)
			{
				case FRONTPANEL_MINOR_FPC:        //IOCTL only
				case FRONTPANEL_MINOR_FPLARGE:    //write-only
				case FRONTPANEL_MINOR_FPSMALL:    //write-only
				{
					return -EUSERS;
				}
				case FRONTPANEL_MINOR_RC:
				{
					byte BytesPerKey = 2;

					while (KeyBufferStart == KeyBufferEnd)
					{
						if (wait_event_interruptible(wq, (KeyBufferStart != KeyBufferEnd)))
						{
							return -ERESTARTSYS;
						}
					}
					switch (KeyEmulationMode)
					{
						case KEYEMUTF7700:
						case KEYEMUTF7700LKP:
						{
							BytesPerKey = 2;
							for (j = 0; j < BytesPerKey; j++)
							{
								kernel_buff[j] = KeyBuffer[KeyBufferEnd];
								KeyBufferEnd = (KeyBufferEnd + 1) & 0xff;
							}
							break;
						}
						case KEYEMUUFS910:
						{
							BytesPerKey = 2;
							sprintf(kernel_buff, "%02x", KeyBuffer[KeyBufferEnd]);
							KeyBufferEnd = (KeyBufferEnd + 1) & 0xff;
							break;
						}
					}
					if (down_interruptible(&FrontPanelOpen[i].sem))
					{
#ifdef DEBUG
						printk("FP: FrontPaneldev_read = -ERESTARTSYS\n");
#endif
						return -ERESTARTSYS;
					}
					copy_to_user((void*)buff, kernel_buff, BytesPerKey);
					up (&FrontPanelOpen[i].sem);
#ifdef DEBUG
					printk("FP: FrontPaneldev_read = %d\n", BytesPerKey);
#endif
					return BytesPerKey;
				}
			}
#ifdef DEBUG
			printk("FP: FrontPaneldev_read = 0\n");
#endif
			return 0;
		}
	}
#ifdef DEBUG
	printk("FP: FrontPaneldev_read = -EUSERS\n");
#endif
	return -EUSERS;
}

static int FrontPaneldev_open(struct inode *Inode, struct file *filp)
{
	int minor;

	minor = MINOR(Inode->i_rdev);

#ifdef DEBUG
	printk ("FP: FrontPaneldev_open(minor=%d, *filp=%p, *oldfp=%p\n", minor, filp, FrontPanelOpen[minor].fp);
#endif

	if (FrontPanelOpen[minor].fp != NULL)
	{
#ifdef DEBUG
		printk("FP: FrontPaneldev_open = -EUSERS\n");
#endif
		return -EUSERS;
	}
	FrontPanelOpen[minor].fp = filp;

	return 0;
}

static int FrontPaneldev_close(struct inode *Inode, struct file *filp)
{
	int minor;

	minor = MINOR(Inode->i_rdev);

#ifdef DEBUG
	printk ("FP: FrontPaneldev_close(minor=%d, *filp=%p, *oldfp=%p\n", minor, filp, FrontPanelOpen[minor].fp);
#endif

	if (FrontPanelOpen[minor].fp == NULL)
	{
		return -EUSERS;
	}
	FrontPanelOpen[minor].fp = NULL;
	return 0;
}

#define LEAPYEAR(year) (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year) (LEAPYEAR(year) ? 366 : 365)
static const int _ytab[2][12] =
{
	{ 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
	{ 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};

static frontpanel_ioctl_time * gmtime(register const time_t time)
{
	static frontpanel_ioctl_time fptime;
	register unsigned long dayclock, dayno;
	int year = 1970;

	dayclock = (unsigned long)time % 86400;
	dayno = (unsigned long)time / 86400;

	fptime.sec = dayclock % 60;
	fptime.min = (dayclock % 3600) / 60;
	fptime.hour = dayclock / 3600;
	fptime.dow = (dayno + 4) % 7;       /* day 0 was a thursday */
	while (dayno >= YEARSIZE(year))
	{
		dayno -= YEARSIZE(year);
		year++;
	}
	fptime.year = year;
	fptime.month = 0;
	while (dayno >= _ytab[LEAPYEAR(year)][fptime.month])
	{
		dayno -= _ytab[LEAPYEAR(year)][fptime.month];
		fptime.month++;
	}
	fptime.day = dayno + 1;
	fptime.month++;

	return &fptime;
}

void vfdSetGmtWakeupTime(time_t time)
{
	frontpanel_ioctl_time *pTime;

	/* store the wakeup time */
	gmtWakeupTime = time;

	/* apply GMT offset */
	time += tffpConfig.gmtOffset;

	pTime = gmtime(time);

	VFDSetTimer(pTime);
	printk("\nWakeup time: %02d:%02d:%02d %02d-%02d-%04d (GMT)\n", pTime->hour, pTime->min,
		 pTime->sec, pTime->day, pTime->month, pTime->year);
}

void vfdSetGmtTime(time_t time)
{
	frontpanel_ioctl_time *pTime;

	/* apply GMT offset */
	time += tffpConfig.gmtOffset;

	pTime = gmtime(time);
#ifdef DEBUG
	printk("\nTime= %02d:%02d:%02d %02d-%02d-%04d (GMT, offset=%dh)\n", pTime->hour, pTime->min,
		 pTime->sec, pTime->day, pTime->month, pTime->year, tffpConfig.gmtOffset/3600);
#endif
	VFDSetTime(pTime);
}

void vfdSetLocalTime(time_t time)
{
	frontpanel_ioctl_time *pTime;

	pTime = gmtime(time);
#ifdef DEBUG
	printk("\nTime= %02d:%02d:%02d %02d-%02d-%04d (local)\n", pTime->hour, pTime->min,
		 pTime->sec, pTime->day, pTime->month, pTime->year);
#endif
	VFDSetTime(pTime);
}

time_t vfdGetGmtTime()
{
	frontpanel_ioctl_time *pTime = VFDReqDateTime();
	time_t time;

	/* convert to seconds since 1970 */
	time = mktime(pTime->year, pTime->month, pTime->day, pTime->hour, pTime->min, pTime->sec);

	/* apply the GMT offset */
	time -= tffpConfig.gmtOffset;

	return time;
}

time_t vfdGetLocalTime()
{
	frontpanel_ioctl_time *pTime = VFDReqDateTime();
	time_t time;

	/* convert to seconds since 1970 */
	time = mktime(pTime->year, pTime->month, pTime->day, pTime->hour, pTime->min, pTime->sec);

	return time;
}

time_t vfdCalcuTime(struct set_time_s *Time)
{
	time_t uTime;
	word mjd = (((Time->time[0] & 0xff) << 8) + (Time->time[1] & 0xff) - 40587);
	byte hour = Time->time[2] & 0xff;
	byte min = Time->time[3] & 0xff;
	byte sec = Time->time[4] & 0xff;

	uTime = (mjd * 86400) + (hour * 3600) + (min * 60) + sec;
#ifdef DEBUG
	printk("FP: %s MJD=%u %02d:%02d:%02d (uTime=%u)\n", __func__, mjd, hour, min, sec, (int)uTime);
#endif
	return uTime;
}

static int FrontPaneldev_ioctl(struct inode *Inode, struct file *File, unsigned int cmd, dword arg)
{
	int minor;
	int ret = 0;

	minor = MINOR(Inode->i_rdev);

#ifdef DEBUG
	printk ("FP: FrontPaneldev_ioctl(minor=%d, *File=%p, cmd=0x%x, arg=0x%x\n", minor, File, cmd, (int)arg);
#endif

	//Quick hack to ignore any tcsetattr

	if (cmd == TCGETS)
	{
		copy_to_user((void*)arg, &termAttributes, sizeof(struct termios));
		return 0;
	}

	if (cmd == 0x5404)
	{
		copy_from_user(&termAttributes, (const void*)arg, sizeof(struct termios));
		return 0;
	}

	if (FrontPanelOpen[minor].fp == NULL || minor != FRONTPANEL_MINOR_FPC)
	{
		return -EUSERS;
	}

	if ((cmd & 0xfffffe00) == VFDMAGIC)
	{
		int res = 0;
		struct tffp_ioctl_data tffp_data;
		struct vfd_ioctl_data vfdData;
		copy_from_user(&vfdData, (const void*)arg, sizeof(vfdData));


		switch (cmd)
		{
			case VFDICONDISPLAYONOFF:
			case VFDSETTIME:
			case VFDBRIGHTNESS:
			case VFDDISPLAYWRITEONOFF:
			{
				if (copy_from_user(&tffp_data, (void *)arg, sizeof(tffp_data)))
				{
					return -EFAULT;
				}
			}
		}

		switch (cmd)
		{
			case VFDDISPLAYCHARS:
			{
				if (vfdData.length >= (sizeof(vfdData.data) - 1))
				{
					vfdData.length = sizeof(vfdData.data) - 1;
				}
				vfdData.data[vfdData.length] = 0;
				TranslateUTFString(vfdData.data);

				//ShowText() replaces VFDSetDisplayLargeString() to incorporate scrolling
				ShowText(vfdData.data);
				//VFDSetDisplayLargeString(vfdData.data);
				break;
			}
			case VFDICONDISPLAYONOFF:
			{
				int icon_nr = tffp_data.u.icon.icon_nr;
				byte onoff = tffp_data.u.icon.on ? 0xf : 0;

				//e2 icons work around
				if (icon_nr >= 256)
				{
					icon_nr >>= 8;
					/* E2 icon translation can be inserted here */
				}
				switch (icon_nr) //get icon number
				{
					/* The VFD driver of enigma2 issues codes during the start
					   phase, map them to the CD segments to indicate progress
					   These are icon numbers 32-41 & 47.
					 */
					case VFD_ICON_CD1: //0x20, note: two segments
					{
						SetIconBits(26, 7, onoff);
						SetIconBits(26, 6, onoff);
						break;
					}
					case VFD_ICON_CD2: //0x21
					{
						SetIconBits(26, 5, onoff);
						break;
					}
					case VFD_ICON_CD3: //0x22
					{
						SetIconBits(26, 4, onoff);
						break;
					}
					case VFD_ICON_CD4: //0x23
					{
						SetIconBits(26, 3, onoff);
						break;
					}
					case VFD_ICON_CD5: //0x24
					{
						SetIconBits(26, 2, onoff);
						break;
					}
					case VFD_ICON_CD6: //0x25, note: two segments
					{
						SetIconBits(26, 1, onoff);
						SetIconBits(26, 0, onoff);
						break;
					}
					case VFD_ICON_CD7: //0x26
					{
						SetIconBits(27, 7, onoff);
						break;
					}
					case VFD_ICON_CD8: //0x27
					{
						SetIconBits(27, 6, onoff);
						break;
					}
					case VFD_ICON_CD9: //0x28
					{
						SetIconBits(27, 5, onoff);
						break;
					}
					case VFD_ICON_CD10: //0x29
					{
						SetIconBits(25, 0, onoff);
						break;
					}
					case VFD_ICON_CDCENTER: //0x2F
					{
						SetIconBits(25, 1, onoff);
						break;
					}

					/* Icons to build the HDD level display.
					   These are icon numbers 48-58.
					   Please note that these are also used by the TopfieldVFD plugin.
					 */
					case VFD_ICON_HDD1: //0x30
					case VFD_ICON_HDD: //0x02
					{
						SetIconBits(29, 4, onoff);
						break;
					}
					case VFD_ICON_HDD_1: //0x31
					{
						SetIconBits(29, 5, onoff);
						break;
					}
					case VFD_ICON_HDD_2: //0x32
					{
						SetIconBits(29, 6, onoff);
						break;
					}
					case VFD_ICON_HDD_3: //0x33
					{
						SetIconBits(29, 7, onoff);
						break;
					}
					case VFD_ICON_HDD_4: //0x34
					{
						SetIconBits(28, 0, onoff);
						break;
					}
					case VFD_ICON_HDD_5: //0x35
					{
						SetIconBits(28, 1, onoff);
						break;
					}
					case VFD_ICON_HDD_6: //0x36
					{
						SetIconBits(28, 2, onoff);
						break;
					}
					case VFD_ICON_HDD_7: //0x37
					{
						SetIconBits(28, 3, onoff);
						break;
					}
					case VFD_ICON_HDD_8: //0x38
					{
						SetIconBits(28, 4, onoff);
						break;
					}
					case VFD_ICON_HDD_FRAME: //0x39
					{
						SetIconBits(28, 5, onoff);
						break;
					}
					case VFD_ICON_HDD_FULL: //0x3A
					{
						SetIconBits(28, 6, onoff);
						break;
					}
					/* The remaining icons, approximately in
					   display order. Some are assigned twice or three times
					   to remain compatible with existing drivers and/or to
					   support the standard VFD-Icons plugin.
					 */
					case VFD_ICON_MP3_2: //0x40
					case VFD_ICON_MP3: //0x05
					case 21: //MP3, added, sent by VFD-Icons
					{
						SetIconBits(28, 7, onoff);
						break;
					}
					case VFD_ICON_AC3: //0x41
					{
						SetIconBits(27, 0, onoff);
						break;
					}
					case VFD_ICON_TIMESHIFT: //0x42
					{
						SetIconBits(27, 1, onoff);
						break;
					}
					case VFD_ICON_TV: //0x43
					{
						SetIconBits(27, 2, onoff);
						break;
					}
					case VFD_ICON_RADIO: //0x44
					case VFD_ICON_MUSIC: //0x06
					{
						SetIconBits(27, 3, onoff);
						break;
					}
					case VFD_ICON_SAT: //0x45
					{
						SetIconBits(27, 4, onoff);
						break;
					}
					case VFD_ICON_REC2: //0x46
					case VFD_ICON_REC: //0x0E
					case 30: //record, added, sent by VFD-Icons
					{
						SetIconBits(41, 1, onoff);
						break;
					}
					case VFD_ICON_RECONE: //0x47
					{
						SetIconBits(41, 0, onoff);
						break;
					}
					case VFD_ICON_RECTWO: //0x48
					{
						SetIconBits(42, 7, onoff);
						break;
					}
					case VFD_ICON_REWIND: //0x49
					case VFD_ICON_FR: //0x0D
					{
						SetIconBits(42, 6, onoff);
						break;
					}
					case VFD_ICON_STEPBACK: //0x4A
					{
						SetIconBits(42, 5, onoff);
						break;
					}
					case VFD_ICON_PLAY2: //0x4B
					case VFD_ICON_PLAY: //0x0A
					case 26: //play, added, sent by VFD-Icons
					{
						SetIconBits(42, 4, onoff);
						break;
					}
					case VFD_ICON_STEPFWD: //0x4C
					{
						SetIconBits(42, 3, onoff);
						break;
					}
					case VFD_ICON_FASTFWD: //0x4D
					case VFD_ICON_FF: //0x0C
					{
						SetIconBits(42, 2, onoff);
						break;
					}
					case VFD_ICON_PAUSE2: //0x4E
					case VFD_ICON_PAUSE: //0x0B
					{
						SetIconBits(42, 1, onoff);
						break;
					}
					case VFD_ICON_MUTE2: //0x4F
					case VFD_ICON_MUTE: //0x09
					{
						SetIconBits(42, 0, onoff);
						break;
					}
					case VFD_ICON_REPEATL: //0x50, note: used by TopfieldVFD plugin for network display
					{
						SetIconBits(43, 7, onoff);
						break;
					}
					case VFD_ICON_REPEATR: //0x51, note: used by TopfieldVFD plugin for network display
					{
						SetIconBits(43, 6, onoff);
						break;
					}
					case VFD_ICON_DOLLAR: //0x52
					case 19: //scrambled, added, sent by VFD-Icons
					{
						SetIconBits(43, 5, onoff);
						break;
					}
					case VFD_ICON_ATTN: //0x53
					{
						SetIconBits(43, 4, onoff);
						break;
					}
					case VFD_ICON_DOLBY: //0x54
					case VFD_ICON_DD: //0x07
					case 23: //DD, added, sent by VFD-Icons
					{
						SetIconBits(43, 3, onoff);
						break;
					}
					case VFD_ICON_NETWORK: //0x55, 
					case 17: //added, sent by VFD-Icons (misused as HD indicator)
					{
						SetIconBits(43, 2, onoff);
						break;
					}
					case VFD_ICON_AM: //0x56
					{
						SetIconBits(7, 7, onoff);
						break;
					}
					case VFD_ICON_TIMER: //0x57
					case VFD_ICON_CLOCK: //0x0F
					{
						SetIconBits(7, 6, onoff);
						break;
					}
					case VFD_ICON_PM: //0x58
					{
						SetIconBits(6, 0, onoff);
						break;
					}
					case VFD_ICON_DOT: //0x59, note: RC feedback
					{
						SetIconBits(1, 4, onoff);
						break;
					}
					case VFD_ICON_POWER: //0x5A, note: controlled by frontprocessor also
					{
						SetIconBits(1, 5, onoff);
						break;
					}
					case VFD_ICON_COLON: //0x5B, note: controlled by frontprocessor also
					{
						SetIconBits(5, 7, onoff);
						break;
					}
					case VFD_ICON_SPINNER: //0x5C
					{
						Spinner_on = onoff ? 1 : 0;
						break;
					}
					case VFD_ICON_ALL: //0x7f
					{
						VFDSetIcons(0x0fffffff, 0x00fffffff, onoff);
						break;
					}
					default:
					{
						printk("FP: unknown icon number %0x2x\n", vfdData.data[0]);
						return -EINVAL;
					}
				}
				break;
			}
			case VFDBRIGHTNESS:
			{
				if (tffp_data.u.brightness.level < 0)
				{
					tffp_data.u.brightness.level = 0;
				}
				else if (tffp_data.u.brightness.level > 5)
				{
					tffp_data.u.brightness.level = 5;
				}

				if (tffp_data.u.brightness.level == save_bright)
				{
					break;
				}
#ifdef DEBUG
				printk("FP: %s Set brightness level 0x%02X\n", __func__, tffp_data.u.brightness.level);
#endif
				if (tffpConfig.brightness != tffp_data.u.brightness.level)
				{
					tffpConfig.brightness = tffp_data.u.brightness.level;
					writeTffpConfig(&tffpConfig);
				}
				VFDBrightness(tffp_data.u.brightness.level);

				save_bright = tffp_data.u.brightness.level;

				break;
			}
			case VFDDISPLAYWRITEONOFF:
			{
				printk("FP: %s Set light 0x%02X\n", __func__, tffp_data.u.light.onoff);

				switch (tffp_data.u.light.onoff)
				{
					case 0: //whole display off
					{
						VFDBrightness(1);
						break;
					}
					case 1: //whole display on
					{
						VFDBrightness(save_bright);
						break;
					}
					default:
					{
						res = -EINVAL;
						break;
					}
				}
				break;
			}
			case VFDDISPLAYCLR:
			{
				VFDClearBuffer();
				Spinner_on = 0;
				CD_icons_onoff(0);
				memset(IconBlinkMode, 0, sizeof(IconBlinkMode));
				break;
			}
			case VFDSETTIME:
			{
				time_t uTime;

				uTime = vfdCalcuTime(&tffp_data.u.time);

				vfdSetLocalTime(uTime);
//				VFDReqDateTime();
				break;
			}
//			case VFDSETTIME2:
//			{
//				u32 uTime = 0;
//
//				res = get_user(uTime, (int *)arg);
//				if (! res)
//				{
//					vfdSetLocalTime(uTime);
//					VFDReqDateTime();
//				}
//				break;
//			}
			case VFDGETTIME:
			{
				u32 uTime = 0;
				
//				uTime = vfdGetGmtTime();
				uTime = vfdGetLocalTime();
#ifdef DEBUG
				printk("FP: %s FP time: %d\n", __func__, uTime);
#endif
				res = put_user(uTime, (int *) arg);
				break;
			}
			case VFDSETPOWERONTIME:
			{
				time_t uTime = 0;

				get_user(uTime, (int *)arg);
#ifdef DEBUG
				printk("FP: %s Set FP wake up time to: %u (uTime)\n", __func__, (int)uTime);
#endif
				vfdSetGmtWakeupTime(uTime);
				break;
			}
//			case VFDGETWAKEUPTIME:
//			{
//				u32 uTime = 0;
//
//				printk("FP: %s Power on time: %d\n", __func__, uTime);
//				res = put_user(uTime, (int *)arg);
//				break;
//			}
			case VFDGETWAKEUPMODE:
			{
				//The reason value has already been cached upon module startup
				int reason = getBootReason();

				switch (reason) //convert reason to a value expected by fp_control
				{
						case 2: //timer?
						{
							reason = 3;
							break;
						}
//						case 1: //power on,  or reboot?
//						{
//							reason = 1;
//							break;
//						}
						case 0: //from deep standby?
						{
							reason = 2;
							break;
						}
						case 3: //?
						default:
						{
							reason = 0; //unknown
							break;
						}
				}
				copy_to_user((void*)arg, &reason, sizeof(frontpanel_ioctl_bootreason));
				break;
			}
			case VFDREBOOT:
			{
				Spinner_on = 0;
				CD_icons_onoff(0);
				VFDReboot();
				break;
			}
			case VFDPOWEROFF:
			{
				Spinner_on = 0;
				CD_icons_onoff(0);
				VFDClearBuffer();
				memset(IconBlinkMode, 0, sizeof(IconBlinkMode));
				VFDShutdown();
				break;
			}
//			case VFDSTANDBY:
//			{
//				u32 uTime = 0;
//
//				get_user(uTime, (int *) arg);
//				vfdSetGmtWakeupTime(uTime);
//				VFDClearBuffer();
//				memset(IconBlinkMode, 0, sizeof(IconBlinkMode));
//				res = 0;
//				break;
//			}
			case 0x5305:
			{
				//Neutrino sends this
				break;
			}
			default:
			{
				printk("FP: unknown VFD IOCTL command %x\n", cmd);
				return -EINVAL;
			}
		}
		return res;
	}
	else if (((cmd >> 8) & 0xff) != IOCTLMAGIC)
	{
		printk("FP: invalid IOCTL magic 0x%x\n", cmd);
		return -ENOTTY;
	}

	switch(_IOC_NR(cmd))
	{
		case _FRONTPANELGETTIME:
		{
			VFDReqDateTime();
			copy_to_user((void*)arg, &fptime, sizeof(frontpanel_ioctl_time));
			break;
		}
		case _FRONTPANELSETTIME:
		{
			copy_from_user(&fptime, (const void*)arg, sizeof(frontpanel_ioctl_time));
			VFDSetTime(&fptime);
			VFDReqDateTime();
			break;
		}
		case _IOC_NR(FRONTPANELSYNCFPTIME):
		{
			/* set the FP time to the current system time */
			struct timeval tv;
			do_gettimeofday(&tv);
			vfdSetGmtTime(tv.tv_sec);
			break;
		}
		case _IOC_NR(FRONTPANELSETGMTOFFSET):
		{
			struct timeval tv;
			do_gettimeofday(&tv);
			/* update the GMT offset if necessary, accept the GMT offset
			   only if the system time is later than 01/01/2004 */
			if ((tffpConfig.gmtOffset != arg) && (tv.tv_sec > 1072224000))
			{
				time_t newTime;

				if (tffpConfig.gmtOffset == -1)
				{
					/* no valid GMT offset, set the new value and update from
						 system time */
					newTime = tv.tv_sec;
				}
				else
				{
					/* GMT offset valid, adjust the FP time */
					VFDReqDateTime();
					newTime = fptime.now - tffpConfig.gmtOffset;
				}
				tffpConfig.gmtOffset = arg;

				/* adjust the FP time */
				vfdSetGmtTime(newTime);
				
				writeTffpConfig(&tffpConfig);

				printk("New GMT offset is %ds\n", tffpConfig.gmtOffset);

				/* update the wakeup time if set */
				if (gmtWakeupTime)
				{
					vfdSetGmtWakeupTime(gmtWakeupTime);
				}
			}
			break;
		}
		case _IOC_NR(FRONTPANELSYNCSYSTIME):
		{
			VFDReqDateTime();
			/* set the system time to the current FP time */
			{
				struct timespec ts;

				ts.tv_sec = fptime.now - tffpConfig.gmtOffset;
				ts.tv_nsec = 0;
				do_settimeofday (&ts);
			}
			break;
		}
		case _FRONTPANELCLEARTIMER:
		{
			VFDClearTimers();
			break;
		}
		case _FRONTPANELSETTIMER:
		{
			copy_from_user(&fptime, (const void*)arg, sizeof(frontpanel_ioctl_time));
			VFDSetTimer(&fptime);
			break;
		}
		case _FRONTPANELIRFILTER1:
		{
			frontpanel_ioctl_irfilter fpdata;

			if (arg)
			{
				copy_from_user(&fpdata, (const void*)arg, sizeof(frontpanel_ioctl_irfilter));
				if (tffpConfig.irFilter1 != fpdata.onoff)
				{
					tffpConfig.irFilter1 = fpdata.onoff;
					writeTffpConfig(&tffpConfig);
				}
				VFDSetIRMode (0, fpdata.onoff);
			}
			break;
		}
		case _FRONTPANELIRFILTER2:
		{
			frontpanel_ioctl_irfilter fpdata;

			if (arg)
			{
				copy_from_user(&fpdata, (const void*)arg, sizeof(frontpanel_ioctl_irfilter));
				if (tffpConfig.irFilter2 != fpdata.onoff)
				{
					tffpConfig.irFilter2 = fpdata.onoff;
					writeTffpConfig(&tffpConfig);
				}
				VFDSetIRMode (1, fpdata.onoff);
			}
			break;
		}
		case _FRONTPANELIRFILTER3:
		{
			frontpanel_ioctl_irfilter fpdata;

			if (arg)
			{
				copy_from_user(&fpdata, (const void*)arg, sizeof(frontpanel_ioctl_irfilter));
				if (tffpConfig.irFilter3 != fpdata.onoff)
				{
					tffpConfig.irFilter3 = fpdata.onoff;
					writeTffpConfig(&tffpConfig);
				}
				VFDSetIRMode (2, fpdata.onoff);
			}
			break;
		}
		case _FRONTPANELIRFILTER4:
		{
			frontpanel_ioctl_irfilter fpdata;

			if (arg)
			{
				copy_from_user(&fpdata, (const void*)arg, sizeof(frontpanel_ioctl_irfilter));
				if (tffpConfig.irFilter4 != fpdata.onoff)
				{
					tffpConfig.irFilter4 = fpdata.onoff;
					writeTffpConfig(&tffpConfig);
				}
				VFDSetIRMode (3, fpdata.onoff);
			}
			break;
		}
		case _FRONTPANELBRIGHTNESS:
		{
			frontpanel_ioctl_brightness *fpdata;

			if (arg)
			{
				fpdata = (frontpanel_ioctl_brightness*)arg;
				if (tffpConfig.brightness != fpdata->bright)
				{
					tffpConfig.brightness = fpdata->bright;
					writeTffpConfig(&tffpConfig);
				}
				VFDBrightness(fpdata->bright);
			}
			break;
		}
		case _FRONTPANELBOOTREASON:
		{
			//The reason value has already been cached upon module startup
			copy_to_user((void*)arg, &fpbootreason, sizeof(frontpanel_ioctl_bootreason));
			break;
		}
		case _FRONTPANELPOWEROFF:
		{
			VFDShutdown();
			break;
		}
		case _FRONTPANELCLEAR:
		{
			VFDClearBuffer();
			memset(IconBlinkMode, 0, sizeof(IconBlinkMode));
			break;
		}
		case _FRONTPANELTYPEMATICDELAY:
		{
			frontpanel_ioctl_typematicdelay fpdata;

			if (arg)
			{
				copy_from_user(&fpdata, (const void*)arg, sizeof(frontpanel_ioctl_typematicdelay));
				if (tffpConfig.typematicDelay != fpdata.TypematicDelay)
				{
					tffpConfig.typematicDelay = fpdata.TypematicDelay;
					writeTffpConfig(&tffpConfig);
				}
				DefaultTypematicDelay = fpdata.TypematicDelay;
			}
			break;
		}
		case _FRONTPANELTYPEMATICRATE:
		{
			frontpanel_ioctl_typematicrate fpdata;

			if (arg)
			{
				copy_from_user(&fpdata, (const void*)arg, sizeof(frontpanel_ioctl_typematicrate));
				if (tffpConfig.typematicRate != fpdata.TypematicRate)
				{
					tffpConfig.typematicRate = fpdata.TypematicRate;
					writeTffpConfig(&tffpConfig);
				}
				DefaultTypematicRate = fpdata.TypematicRate;
			}
			break;
		}
		case _FRONTPANELKEYEMULATION:
		{
			frontpanel_ioctl_keyemulation fpdata;

			if (arg)
			{
				copy_from_user(&fpdata, (const void*)arg, sizeof(frontpanel_ioctl_keyemulation));
				KeyEmulationMode = fpdata.KeyEmulation;
				KeyBufferStart =  KeyBufferEnd = 0;
			}
			break;
		}
		case _FRONTPANELREBOOT:
		{
			VFDReboot();
			break;
		}
		case _FRONTPANELRESEND:
		{
			VFDSendToDisplay(true);
			break;
		}
		case _FRONTPANELALLCAPS:
		{
			frontpanel_ioctl_allcaps fpdata;

			if (arg)
			{
				copy_from_user(&fpdata, (const void*)arg, sizeof(frontpanel_ioctl_allcaps));
				if (tffpConfig.allCaps != fpdata.AllCaps)
				{
					tffpConfig.allCaps = fpdata.AllCaps;
					writeTffpConfig(&tffpConfig);
				}
			}
			break;
		}
		case _FRONTPANELSCROLLMODE:
		{
			if (arg)
			{
				copy_from_user(&ScrollMode, (const void*)arg, sizeof(frontpanel_ioctl_scrollmode));
				if (ScrollMode.ScrollPause == 0)
				{
					ScrollMode.ScrollPause = 1;
				}
				if (ScrollMode.ScrollDelay == 0)
				{
					ScrollMode.ScrollDelay = 1;
				}
				if ((ScrollMode.ScrollMode != tffpConfig.scrollMode)
				|| (ScrollMode.ScrollDelay != tffpConfig.scrollDelay)
				|| (ScrollMode.ScrollPause != tffpConfig.scrollPause))
				{
					tffpConfig.scrollMode = ScrollMode.ScrollMode;
					tffpConfig.scrollDelay = ScrollMode.ScrollDelay;
					tffpConfig.scrollPause = ScrollMode.ScrollPause;
					writeTffpConfig(&tffpConfig);
				}
				ScrollOnce = 1;
				ScrollState = SS_InitChars;
			}
			break;
		}
		case _FRONTPANELICON:
		{
			frontpanel_ioctl_icons fpdata;

			if (arg)
			{
				copy_from_user(&fpdata, (const void*)arg, sizeof(frontpanel_ioctl_icons));
				VFDSetIcons(fpdata.Icons1, fpdata.Icons2, fpdata.BlinkMode);
			}
			break;
		}
		case _FRONTPANELSPINNER:
		{
			frontpanel_ioctl_spinner fpdata;

			if (arg)
			{
				copy_from_user(&fpdata, (const void*)arg, sizeof(frontpanel_ioctl_spinner));
				Spinner_on = fpdata.Spinner ? 1 : 0;
			}
			break;
		}
		default:
		{
			printk("FP: unknown IOCTL 0x%x(0x%08x)\n", cmd, (int)arg);
			ret = -ENOTTY;
		}
	}
	return ret;
}

/**********************************************************************************************/
/* RTC code                                                                                   */
/* (based on code taken from aotom driver)                                                    */
/**********************************************************************************************/
static int tffp_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	u32 uTime = 0;

#ifdef DEBUG
	printk("FP: %s >\n", __func__);
#endif

	uTime = vfdGetGmtTime();
	rtc_time_to_tm(uTime, tm);

#ifdef DEBUG
	printk("FP: %s < %d\n", __func__, uTime);
#endif

	return 0;
}

static int tm2time(struct rtc_time *tm)
{
	return mktime(tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
}

static int tffp_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int res = 0;
	u32 uTime = tm2time(tm);

#ifdef DEBUG
	printk("FP: %s > uTime %d\n", __func__, uTime);
#endif

	vfdSetGmtTime(uTime);

#ifdef DEBUG
	printk("FP: %s < res: %d\n", __func__, res);
#endif

	return res;
}

static int tffp_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	u32 a_time = 0;

#ifdef DEBUG
	printk("FP: %s >\n", __func__);
#endif

//	a_time = FP_GetPowerOnTime();
	if (al->enabled)
	{
		rtc_time_to_tm(a_time, &al->time);
	}
#ifdef DEBUG
	printk("FP: %s < Enabled: %d RTC alarm time: %d time: %d\n", __func__, al->enabled, (int)&a_time, a_time);
#endif

	return -EINVAL;
}

static int tffp_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	u32 a_time = 0;

#ifdef DEBUG
	printk("FP: %s >\n", __func__);
#endif

	if (al->enabled)
	{
		a_time = tm2time(&al->time);
	}
	vfdSetGmtWakeupTime(a_time);
#ifdef DEBUG
	printk("FP: %s < Enabled: %d time: %d (GMT)\n", __func__, al->enabled, a_time);
#endif

	return 0;
}

static const struct rtc_class_ops tffp_rtc_ops =
{
	.read_time  = tffp_rtc_read_time,
	.set_time   = tffp_rtc_set_time,
	.read_alarm = tffp_rtc_read_alarm,
	.set_alarm  = tffp_rtc_set_alarm
};

static int __devinit tffp_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;

#ifdef DEBUG
	printk("FP: %s >\n", __func__);
#endif

	printk("Topfield front panel real time clock\n");
	pdev->dev.power.can_wakeup = 1;
	rtc = rtc_device_register("tffp_rtc", &pdev->dev, &tffp_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
	{
		return PTR_ERR(rtc);
	}
	platform_set_drvdata(pdev, rtc);
#ifdef DEBUG
	printk("FP: %s < %p\n", __func__, rtc);
#endif

	return 0;
}

static int __devexit tffp_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);

#ifdef DEBUG
	printk("FP: %s %p >\n", __func__, rtc);
#endif
	rtc_device_unregister(rtc);
	platform_set_drvdata(pdev, NULL);
#ifdef DEBUG
	printk("FP: %s <\n", __func__);
#endif

	return 0;
}

/**********************************************************************************************/
/* Real Time Clock Driver Interface                                                           */
/*                                                                                            */
/**********************************************************************************************/
static struct platform_driver tffp_rtc_driver =
{
	.probe = tffp_rtc_probe,
	.remove = __devexit_p(tffp_rtc_remove),
	.driver =
	{
		.name	= RTC_NAME,
		.owner	= THIS_MODULE
	},
};

/**********************************************************************************************/
/* Frontpanel Driver Interface                                                                           */
/*                                                                                            */
/**********************************************************************************************/
static struct file_operations frontpanel_fops =
{
	.owner   = THIS_MODULE,
	.open    = FrontPaneldev_open,
	.ioctl   = FrontPaneldev_ioctl,
	.read    = (void*)FrontPaneldev_read,
	.poll    = (void*)FrontPaneldev_poll,
	.write   = FrontPaneldev_write,
	.release = FrontPaneldev_close
};

static int __init frontpanel_init_module(void)
{
	unsigned int *ASC_3_INT_EN = (unsigned int*)(ASC3BaseAddress + ASC_INT_EN);
	unsigned int *ASC_3_CTRL   = (unsigned int*)(ASC3BaseAddress + ASC_CTRL);
	int          i;

	//Disable all ASC 3 interrupts
	*ASC_3_INT_EN = *ASC_3_INT_EN & ~0x000001ff;

	if (register_chrdev(FRONTPANEL_MAJOR, "FrontPanel", &frontpanel_fops))
	{
		printk("FP: unable to get major %d\n",FRONTPANEL_MAJOR);
	}
	else
	{
		frontpanel_init_func();
	}
	//Initialize the variables
	memset(FrontPanelOpen, 0, (LASTMINOR + 1) * sizeof (tFrontPanelOpen));
	memset(IconBlinkMode, 0, sizeof(IconBlinkMode));
	ScrollMode.ScrollMode  =   1; //scroll once
	ScrollMode.ScrollPause = 100; //wait 1s until start of scroll
	ScrollMode.ScrollDelay =  20; //scroll every 200ms
	Message[0] = '\0';

	// read and apply the settings
	readTffpConfig(&tffpConfig);
	DefaultTypematicRate = tffpConfig.typematicRate;
	DefaultTypematicDelay = tffpConfig.typematicDelay;
	ScrollMode.ScrollMode  = tffpConfig.scrollMode;
	ScrollMode.ScrollPause = tffpConfig.scrollPause;
	ScrollMode.ScrollDelay = tffpConfig.scrollDelay;
	/* transmit applicable settings to the VFD */
	VFDSetIRMode (0, tffpConfig.irFilter1);
	VFDSetIRMode (1, tffpConfig.irFilter2);
	VFDSetIRMode (2, tffpConfig.irFilter3);
	VFDSetIRMode (3, tffpConfig.irFilter4);
	VFDBrightness(tffpConfig.brightness);

	for (i = 0; i <= LASTMINOR; i++)
	{
		sema_init(&FrontPanelOpen[i].sem, 1);
	}
	sema_init(&rx_int_sem, 0);

	kernel_thread(fpTask, NULL, 0);

	init_waitqueue_head(&wq);

	//Enable the FIFO
	*ASC_3_CTRL = *ASC_3_CTRL | ASC_CTRL_FIFO_EN;

	printk("Stored GMT offset is %d\n", tffpConfig.gmtOffset);

	//Link the ASC 3 interupt to our handler and enable the receive buffer IE flag
	i = request_irq(STASC1IRQ, (void*)FP_interrupt, 0, "FP_serial", NULL);
	if (!i)
	{
		struct timespec ts = {0, 0};
		*ASC_3_INT_EN = *ASC_3_INT_EN | 0x00000001;

		if (tffpConfig.gmtOffset != -1)
		{
			/* GMT offset valid */
			/* retrieve the time from the FP */
			VFDReqDateTime();
			ts.tv_sec = fptime.now - tffpConfig.gmtOffset;
		}
		/* else: set 1970/01/01 to enforce getting the time
			 from transponder */

		/* set the system time */
		do_settimeofday (&ts);

		VFDGetBootReason();
		VFDClearTimers();

		/* get the VFD control */
		SendFPData(1, 0x40);
		/* unknown code sequence */
		SendFPData(2, 0x51, 0x5a);
		SendFPData(5, 0x54, 0xff, 0xff, 0x00, 0x00);
		/* set power status (?) */
		SendFPData(2, 0x41, 0x88);
		/* set time format to 24h */
		SendFPData(2, 0x11, 0x81);
	}
	else
	{
		printk("FP: Cannot get irq\n");
	}
	memset (&termAttributes, 0, sizeof (struct termios));

	// switch spinner off
	Spinner_on = 0;

	//Start the timer
	HighRes_FP_Timer(0);

	//Handle RTC
	i = platform_driver_register(&tffp_rtc_driver);
	if (i)
	{
		printk("FP: %s RTC platform_driver_register failed: %d\n", __func__, i);
	}
	else
	{
		rtc_pdev = platform_device_register_simple(RTC_NAME, -1, NULL, 0);
	}

	if (IS_ERR(rtc_pdev))
	{
		printk("FP: %s RTC platform_device_register_simple failed: %ld\n", __func__, PTR_ERR(rtc_pdev));
	}

	create_proc_fp();

	return 0;
}

static void __exit frontpanel_cleanup_module(void)
{
	unsigned int *ASC_3_INT_EN = (unsigned int*)(ASC3BaseAddress + ASC_INT_EN);

	//RTC
	platform_driver_unregister(&tffp_rtc_driver);
	platform_set_drvdata(rtc_pdev, NULL);
	platform_device_unregister(rtc_pdev);

	*ASC_3_INT_EN = *ASC_3_INT_EN & ~0x000001ff;
	Spinner_on = 0;  // switch spinner off
	free_irq(STASC1IRQ, NULL);
	del_timer(&timer);
	VFDClearBuffer();
	unregister_chrdev(FRONTPANEL_MAJOR,"FrontPanel");
	printk("FP: Topfield TF77X0HDPVR front panel module unloading\n");

	remove_proc_fp();
}

module_init(frontpanel_init_module);
module_exit(frontpanel_cleanup_module);

MODULE_DESCRIPTION("FrontPanel module for Topfield TF77X0HDPVR " VERSION);
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");

// vim:ts=4
