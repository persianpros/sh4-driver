/*
 * vitamin_micom_file.c  for Showbox Vitamin HD 5000
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
******************************************************************************
 *
 * The Showbox Vitamin HD 5000 front panel processor is not aware of the time
 * in general. It is possible to set a time, which is apparently only used to
 * display the time during standby. The front panel process does not control
 * waking up for a recording.
 * All this stems from the fact that this model does not have a deep standby
 * mode; it is either on, in E2 controlled standby or completely off.
 * As a consequence, this receiver cannot make timed recordings from deep
 * standby. This is also the reason that this driver does not handle any wake
 * up time related functions with the front panel processor: they are not
 * present in that processor.
 *
******************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * ----------------------------------------------------------------------------
 * 20190519 Audioniek       Clean up code; consistent use of dprintk.
 * 20190521 Audioniek       VFDTEST added.
 * 20190522 Audioniek       Add vfdmode.
 * 20190522 Audioniek       Add text scrolling in /dev/fd.
 * 20190522 Audioniek       Add dprintk(0, (always print if printk configured)
 * 20190522 Audioniek       LED states inverted, 1 is now on instead of off.
 */

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/termbits.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
#include <linux/stm/pio.h>
#else
#include <linux/stpio.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/poll.h>

#include "vitamin_micom.h"
#include "vitamin_micom_asc.h"
#include "../vfd/utf.h"

extern void ack_sem_up(void);
extern int  ack_sem_down(void);
extern void micom_putc(unsigned char data);

struct semaphore write_sem;

int errorOccured = 0;
static char ioctl_data[20];

tFrontPanelOpen FrontPanelOpen [LASTMINOR];
int rtc_offset;

// place to hold last display string,
struct saved_data_s
{
	int  length;
	char data[128];
	int  ledBlue;
	int  ledGreen;
	int  ledRed;
};

static struct saved_data_s lastdata;

/*******************************************************
 *
 * Code to communicate with the front processor.
 *
 */
void write_sem_up(void)
{
	up(&write_sem);
}

int write_sem_down(void)
{
	return down_interruptible(&write_sem);
}

void copyData(unsigned char *data, int len)
{
	dprintk(10, "%s > len=%d\n", __func__, len);
	memcpy(ioctl_data, data, len);
}

int micomWriteCommand(char command, char *buffer, int len, int needLen)
//int micomWriteCommand(char command, char *buffer, int len, int needAck, int needLen)
//int micomWriteCommand(char command, char *buffer, int len, int needAck)
{
	// needLen: when set to nonzero, also send argument length
	//
	// Command structure:
	//
	// Normal commands are 7 bytes long. The first byte
	// is the byte indicating the command, the following bytes
	// respresent the data of the command. Unused bytes should
	// be set to zero, as not every command uses all six available
	// bytes to transfer data.
	//
	// The only conmmand that is longer is that for displaying text.
	// The text can be up to 11 bytes long and agfter the command byte
	// a byte is sent containing the text length, after which the bytes
	// with the text follow. The command should always be 13 bytes long
	// so if shorter zero bytes should be added to obtain a command
	// length of 13 in total.

	int i;

	dprintk(100, "%s >\n", __func__);

	if (len > 255)
	{
		len = 255;
	}
	/* slow down a bit to prevent overruns */
	udelay(1000);

	dprintk(90, "Put: %02X (command)\n", command & 0xff);
	serial_putc(command);
	if (needLen)
//	if (command & 0xff == 0xf5)
	{
		dprintk(90, "Put: %02X (len)\n", len & 0xff);
		serial_putc((char)len);
	}
	for (i = 0; i < len; i++)
	{
		if (needLen)
//		if (command & 0xff == 0xf5)
		{
			dprintk(90, "Put: %02X '%c' (text)\n", buffer[i], buffer[i]);
		}
		else
		{
			dprintk(90, "Put: %02X (data)\n", buffer[i] & 0xff);
		}
		serial_putc(buffer[i]);
	}
	udelay(1000);
#if 0  // There are no commands that require an ACK at the moment
	if (needAck)
	{
		if (ack_sem_down())
		{
			dprintk(0, "%s: return -ERESTARTSYS\n", __func__);
			return -ERESTARTSYS;
		}
	}
#endif
	dprintk(100, "%s <\n", __func__);
	return 0;
}

/*******************************************************************
 *
 * Icon definitions.
 *
 * code: bits 0 and 1 select icon group 0, 1 or 2
 *       remaining bits are used as bit mask.
 */
struct iconToInternal
{
	u16 code;
	char *name;
} VitaminIcons[] =
{ //
	// #      Code     Name   
	/*00 */ { NO_ICON, "none"          },
	/*01 */ { 0x2001,  "ICON_TV"       },  // CLASS_FIGURE2
	/*02 */ { 0x1001,  "ICON_RADIO"    },  // CLASS_FIGURE2
	/*03 */ { 0x0801,  "ICON_REPEAT"   },  // CLASS_FIGURE2
	/*04 */ { 0x0502,  "ICON_CIRCLE"   },  // spinner
	/*05 */ { 0x8000,  "ICON_RECORD"   },
	/*06 */ { 0x4001,  "ICON_POWER"    },  // CLASS_FIGURE2
	/*07 */ { 0x8001,  "ICON_PWR_CIRC" },  // CLASS_FIGURE2
	/*08 */ { 0x4000,  "ICON_DOLBY"    },
	/*09 */ { 0x2000,  "ICON_MUTE"     },
	/*0a */ { 0x1000,  "ICON_DOLLAR"   },
	/*0b */ { 0x0800,  "ICON_LOCK"     },
	/*0c */ { 0x0400,  "ICON_CLOCK"    },
	/*0d */ { 0x0200,  "ICON_HD"       },
	/*0e */ { 0x0100,  "ICON_DVD"      },
	/*0f */ { 0x0080,  "ICON_MP3"      },
	/*10 */ { 0x0040,  "ICON_NET"      },
	/*11 */ { 0x0020,  "ICON_USB"      },
	/*12 */ { 0x0010,  "ICON_MC"       },
};

int gVfdIconState  = 0;  // CLASS_FIGURE
int gVfdIconState2 = 0;  // CLASS_FIGURE2
int gVfdIconCircle = 0;  // spinner

/*******************************************************************
 *
 * Code for the functions of the driver.
 *
 */

/*******************************************************
 *
 * micomSetIcon: (re)sets an icon on the front panel
 *               display.
 *
 */
// TODO: enable different speeds for the spinner
int micomSetIcon(int which, int on)
{
	unsigned char buffer[8], i;
	unsigned short icon_sel;
	int res = 0;

	if ((on < 0 || on > 1) && which != ICON_CIRCLE)
	{
			dprintk(0, "Illegal icon state %d (allowed: 0 or 1)\n", on);
			return -EINVAL;
	}
	if (which < ICON_MIN || which > ICON_MAX - 1)
	{
		dprintk(0, "Illegal icon number %d (0x%02X), (allowed: %d-%d)\n", which, which, ICON_MIN, ICON_MAX - 1);
		return -EINVAL;
	}
	dprintk(10, "Set icon %s (%02d) to %s\n", VitaminIcons[which].name, which, on ? "on" : "off");

	icon_sel = VitaminIcons[which].code & 0x03;  // select icon group
	which = VitaminIcons[which].code & ~0x03;  // get bit mask

	//  update state bytes
	if (on == 0)
	{
		if (!icon_sel)  // if group 0
		{
			gVfdIconState &= ~which;  // remove bitmask
		}
		else if (icon_sel == 1)  // if group 1
		{
			gVfdIconState2 &= ~which;  // remove bitmask
		}
		else if (icon_sel == 2)  // if group 2 (icon circle only)
		{
			gVfdIconCircle = 0x40; // spinner off
		}
	}
	else  // icon on
	{
		if (!icon_sel)
		{
			gVfdIconState |= which;  // add bit mask
		}
		else if (icon_sel == 1)
		{
			gVfdIconState2 |= which;  // add bit mask
		}
		else if (icon_sel == 2)
		{
			// TODO: insert code for spinning speed
			gVfdIconCircle = 0x0f;  // get speed
		}
	}
	if (!icon_sel)  // handle group 0 (uses CLASS_FIGURE)
	{
		memset(buffer, 0, sizeof(buffer));
		buffer[0] = (unsigned char)((gVfdIconState >> 8) & 0xff);  // get upper byte (icons)
		buffer[1] = (unsigned char)(gVfdIconState & 0xff);  // get lower byte (states)
		res = micomWriteCommand( CLASS_FIGURE, buffer, 2, 0); 
	}
	else if (icon_sel == 1)  // handle group 1 (uses CLASS_FIGURE2)
	{
		memset(buffer, 0, sizeof(buffer));
		buffer[0] = (unsigned char)((gVfdIconState2 >> 8) & 0xff);  // get upper byte
		res = micomWriteCommand( CLASS_FIGURE2, buffer, 2, 0); 
		mdelay(10);
	}
	else if (icon_sel == 2)  // handle the spinner (uses CLASS_CIRCLE)
	{
		memset(buffer, 0, sizeof(buffer));
		buffer[0] = (unsigned char)(gVfdIconCircle & 0xff);  // get speed and bit mask
		res = micomWriteCommand( CLASS_CIRCLE, buffer, 1, 0); 
		mdelay(10);
	}
	else
	{
		return res;
	}
}
/* export for later use in e2_proc */
EXPORT_SYMBOL(micomSetIcon);

/*******************************************************************
 *
 * micomSetBLUELED: sets the state of the blue LED on the
 *                  front panel display.
 *
 * Parameter on is a bit mask:
 * bit 0 = LED on (1) or off (0)
 */
int micomSetBLUELED(int on)
{
	char buffer[8];
	int  res = 0;

	dprintk(100, "%s > %d\n", __func__,  on);
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = (on == 0 ? 1 : 0);
	res = micomWriteCommand(CLASS_BLUELED, buffer, 1, 0);
	lastdata.ledBlue = on;
	dprintk(100, "%s <\n", __func__);
	return res;
}
/* export for later use in e2_proc */
EXPORT_SYMBOL(micomSetBLUELED);

/*******************************************************************
 *
 * micomSetGREENLED: sets the state of the green LED on front panel display.
 *
 * Parameter on is a bit mask:
 * bit 0 = LED off (0) or on (1)
 * FIXME: does not work
 */
int micomSetGREENLED(int on)
{
	char buffer[8];
	int  res = 0;

	dprintk(100, "%s > %d\n", __func__,  on);
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = 0xA0 + (on == 0 ? 1 : 0);

	res = micomWriteCommand(CLASS_BLUELED, buffer, 1, 0);
	lastdata.ledGreen = on;
	dprintk(100, "%s <\n", __func__);
	return res;
}
/* export for later use in e2_proc */
//EXPORT_SYMBOL(micomSetGREENLED);


/*******************************************************************
 *
 * micomSetREDLED: sets the state of the red LED on front panel display.
 *
 * Parameter on is a bit mask:
 * bit 0 = LED off (0) or on (1)
 * FIXME: does not work
 */
int micomSetREDLED(int on)
{
	char buffer[8];
	int  res = 0;

	dprintk(100, "%s > %d\n", __func__,  on);
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = 0xB0 + (on == 0 ? 1 : 0);
	res = micomWriteCommand(CLASS_BLUELED, buffer, 1, 0);
	lastdata.ledRed = on;
	dprintk(100, "%s <\n", __func__);
	return res;
}
/* export for later use in e2_proc */
//EXPORT_SYMBOL(micomSetREDLED);


/****************************************************************
 *
 * micomSetBrightness: sets brightness of front panel display.
 *
 */
int micomSetBrightness(int level)
{
	char buffer[1];
	int  res = 0;

	dprintk(100, "%s > %d\n", __func__, level);
	if (level < 0 || level > 7)
	{
		dprintk(0,"Brightness out of range %d\n", level);
		return -EINVAL;
	}
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = level;
	res = micomWriteCommand(CLASS_INTENSITY, buffer, 1, 0);
	dprintk(100, "%s <\n", __func__);
	return res;
}
/* export for later use in e2_proc */
EXPORT_SYMBOL(micomSetBrightness);

#if 0
/****************************************************************
 *
 * micomSetLedBrightness: sets brightness of front panel LEDs.
 *
 */
int micomSetLedBrightness(int level)
{
	// uses CLASS_LED cmd?
	// does nothing; hardware seems to be not capable of LED dimming
	return 0;
}
//EXPORT_SYMBOL(micomSetLedBrightness);
#endif

/**************************************************
 *
 * micomSetStandby: switches receiver off
 *                  (deep standby).
 */
int micomSetStandby(void)
{
	char buffer[8];
	int  res = 0;

	dprintk(100, "%s >\n", __func__);
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = POWER_DOWN;  // 0x00
	buffer[1] = 0x00;
	res = micomWriteCommand(CLASS_POWER, buffer, 2, 0);
	dprintk(100, "%s <\n", __func__);
	return res;
}

#if 0
/**************************************************
 *
 * micomSetTime: sets front panel clock time.
 *
 */
int micomSetTime(char *time)
{
	char buffer[4];
	int  res = 0;

	dprintk(100, "%s > \n", __func__);
	dprintk(10, "Time to set: %02d:%02d:%02d\n", time[2], time[1], time[0]);

	/* time: HH MM SS DD mm YY */

	memset(buffer, 0, sizeof(buffer));
//	buffer[1] = time[0]; /* hour */
//	buffer[2] = time[1]; /* minute */
//	buffer[3] = time[2]; /* second */
//	res = micomWriteCommand(CLASS_CLOCK, buffer, 3, 0);

	dprintk(10, "Date to set: %02d:%02d:%04d\n", time[5], time[4], time[3]);
	memset(buffer, 0, sizeof(buffer));
//	buffer[0] = 20; /* year high 20 */
//	buffer[1] = time[5]; /* year low */
//	buffer[2] = time[4]; /* month */
//	buffer[3] = time[3]; /* day */
//	res = micomWriteCommand(CLASS_CLOCKHI, buffer, 4, 0);

	dprintk(100, "%s <\n", __func__);
	return res;
}
#endif

#if 0
/****************************************************************
 *
 * micomGetVersion: get version/release date of front processor.
 *
 * Note: only returns a code representing the display type
 * TODO: return actual version string through procfs
 * FIXME: null in original
 */
int micomGetVersion(void)
{
	char buffer[8];
	int  res = 0;
	dprintk(100, "%s >\n", __func__);

	memset(buffer, 0, sizeof(buffer));

	errorOccured   = 0;
	res = micomWriteCommand(CLASS_0X05, buffer, 7, 0);

	if (res < 0)
	{
		printk("%s < res %d\n", __func__, res);
		return res;
	}
	if (errorOccured)
	0
		memset(ioctl_data, 0, sizeof(buffer));
		dprintk(0, "%s: error occurred\n", __func__);

		res = -ETIMEDOUT;
	}
	else
	{
		/* version received ->noop here */
		dprintk(10, "version received\n");
	}
	dprintk(100, "%s <\n", __func__);
	return res;
}
#endif


#if 0
/*************************************************************
 *
 * micomGetWakeUpMode: read wake up reason from front panel.
 * FIXME: null in original
 */
int micomGetWakeUpMode(void)
{
	int  res = 0;
	char buffer[8];

	dprintk(100, "%s >\n", __func__);
	memset(buffer, 0, sizeof(buffer));
	errorOccured   = 0;
	res = micomWriteCommand(CLASS_0X43, buffer, 7, 0);

	if (res < 0)
	{
		printk("%s < res %d\n", __func__, res);
		return res;
	}
	if (errorOccured)
	{
		memset(ioctl_data, 0, sizeof(buffer));
		printk("error\n");
		res = -ETIMEDOUT;
	}
	else
	{
		/* time received ->noop here */
		dprintk(0, "time received\n");
	}
	dprintk(100, "%s <\n", __func__);
	return res;
}
#endif

/****************************************************************
 *
 * micomReboot: reboot receiver.
 *
 */
int micomReboot(void)
{
	char buffer[8];
	int  res = 0;

	dprintk(100, "%s >\n", __func__);
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = 0;
	buffer[1] = POWER_RESET;
	res = micomWriteCommand(CLASS_RESET, buffer, 2, 0);
	dprintk(100, "%s <\n", __func__);
	return res;
}

#if defined(VFDTEST)
/*************************************************************
 *
 * micomVfdTest: Send arbitrary bytes to front processor
 *               and read back status and if applicable
 *               return bytes (for development purposes).
 *
 */
int micomVfdTest(unsigned char *data)
{
	unsigned char buffer[12];
	int res = -1, sendlen;
	int i;

	dprintk(100, "%s > len = %d, command = 0x%02x\n", __func__, data[0], data[1]);

	sendlen = data[0] - 1;

	if (data[1] == CLASS_DISPLAY2)  // requires sending length
	{
		sendlen = 11;
		memset(buffer, 0x20, sizeof(buffer));  // fill buffer with spaces
	}
	else
	{
		memset(buffer, 0, sizeof(buffer));  // fill buffer with zeroes
	}	
	memcpy(buffer, data + 2, sendlen); // copy the remaining bytes to send
	memset(ioctl_data, 0, sizeof(ioctl_data));

#if 0
	dprintk(1, "sendlen = %d, buffer:\n", sendlen);
	for (i = 0; i < sendlen; i++)
	{
		dprintk(1, "buffer[%02d] = 0x%02x\n", i, buffer[i]);
	}
	if (paramDebug >= 50)
	{
		printk("[vitamin_micom] Send command: [0x%02x],", data[1] & 0xff);
		for (i = 0; i < sendlen - 1; i++)
		{
			printk(" 0x%02x", buffer[i] & 0xff);
		}
		printk("\n");
	}
#endif
	errorOccured = 0;
	res = micomWriteCommand(data[1] & 0xff, buffer, sendlen, (data[1] == CLASS_DISPLAY2 ? 1 : 0));

	if (errorOccured == 1)
	{
		/* error */
		memset(data + 1, 0, sizeof(data) - 1);
		dprintk(0, "Error in function %s\n", __func__);
		res = data[0] = -ETIMEDOUT;
	}
	else
	{
		data[0] = 0;  // command went OK
		for (i = 0; i < 6; i++) // we can receive up 6 bytes back TODO: test this
		{
			data[i + 1] = ioctl_data[i];  // copy return data
		}
	}
	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}
#endif

/*****************************************************************
 *
 * micomWriteString: Display a text on the front panel display.
 *
 * Note: does not scroll; displays the first VFD_MAX_LEN
 *       characters. Scrolling is available by writing to
 *       /dev/vfd: this scrolls a maximum of 64 characters once
 *       if the text length exceeds VFD_MAX_LEN.
 * Note: Always displays 11 characters. Shorter texts are
 *       padded by adding spaces, keeping the display clean.
 */
int micomWriteString(unsigned char *aBuf, int len)
{
	unsigned char bBuf[VFD_MAX_LEN + 1];
	int i = 0;
	int j = 0;
	int res = 0;

	dprintk(100, "%s >\n", __func__);

	/* fill buffer  with spaces */
	memset(bBuf, ' ', sizeof(bBuf));
	memcpy(bBuf, aBuf, (len > VFD_MAX_LEN) ? VFD_MAX_LEN : len);

	res = micomWriteCommand(CLASS_DISPLAY2, bBuf, VFD_MAX_LEN, 1);
	/* save last string written to fp */
	memcpy(&lastdata.data, bBuf, VFD_MAX_LEN);
	lastdata.length = VFD_MAX_LEN;

	dprintk(100, "%s <\n", __func__);
	return res;
}

/****************************************************************
 *
 * Clear text display.
 *
 */
int clear_display(void)
{
	unsigned char bBuf[12];
	int res = 0;

	dprintk(100, "%s >\n", __func__);

	memset(bBuf, ' ', sizeof(bBuf));
	res = micomWriteString(bBuf, VFD_MAX_LEN);
	dprintk(100, "%s <\n", __func__);
	return res;
}

/****************************************************************
 *
 * Reset all icons.
 *
 */
int micomClearIcons(void)
{
	char buffer[2];
	int  res = 0;

	// CLASS_FIGURE
	memset(buffer, 0, sizeof(buffer)); 
//	buffer[0] = 0;
//	buffer[1] = 0;
	res = micomWriteCommand(CLASS_FIGURE, buffer, 2, 0); 
	mdelay(10);

	// CLASS_FIGURE2
//	buffer[0] = 0;
	res |= micomWriteCommand(CLASS_FIGURE2, buffer, 1, 0); 
	mdelay(10);

	// CLASS_CIRCLE, note: just switching the spinner off often leaves one segment on
	buffer[0] = 0x0f; // spinner on
	res |= micomWriteCommand(CLASS_CIRCLE, buffer, 1, 0); 
	mdelay(10);
	// CLASS_CIRCLE
	buffer[0] = 0x040;  // spinner off
	res |= micomWriteCommand(CLASS_CIRCLE, buffer, 1, 0); 
	mdelay(10);
	return res;
}

/****************************************************************
 *
 * micomSetDisplayOnOff: switch entine front panel display on or
 *                       off, retaining display text and LED and
 *                       icon states.
 */
int micomSetDisplayOnOff(int onoff)
{
	int  res = 0, i;
	char buffer[VFD_MAX_LEN];
	char buf[12];

	dprintk(100, "%s >\n", __func__);
	dprintk(10, "Switch display %s\n", (onoff == 0 ? "off" : "on"));

	if (onoff)
	{
		memset(buffer, ' ', sizeof(buffer));
		for (i = 0; i < lastdata.length; i++)
		{
			buffer[i] = lastdata.data[i];
		}
		res = micomWriteCommand(CLASS_DISPLAY2, buffer, lastdata.length, 1);
				
		dprintk(10, "Group 0 icons: gVfdIconState = 0x%04x\n", gVfdIconState);
		// handle group 0 (CLASS_FIGURE)
		memset(buffer, 0, sizeof(buffer));
		buffer[0] = (unsigned char)((gVfdIconState >> 8) & 0xff);  // get upper byte (icons)
		buffer[1] = (unsigned char)(gVfdIconState & 0xff);  // get lower byte (states)
		res = micomWriteCommand( CLASS_FIGURE, buffer, 2, 0); 
		mdelay(10);

		dprintk(10, "Group 1 icons: gVfdIconState2 = 0x%02x\n", (gVfdIconState2 >> 8) & 0xff);
		// handle group 1 (CLASS_FIGURE2)
		memset(buffer, 0, sizeof(buffer));
		buffer[0] = (unsigned char)((gVfdIconState2 >> 8) & 0xff);  // get upper byte
		buffer[1] = 0;
		res |= micomWriteCommand( CLASS_FIGURE2, buffer, 2, 0); 

		dprintk(10, "Group 1 icons: gVfdIconCircle = 0x%02x\n", gVfdIconCircle & 0xff);
		// handle the spinner (CLASS_CIRCLE)
		// note: just switching the spinner off often leaves one segment on
		buffer[0] = (unsigned char)(gVfdIconCircle & 0xff);  // get speed and bit mask;
		res |= micomWriteCommand(CLASS_CIRCLE, buffer, 1, 0); 
		mdelay(10);

		res |= micomSetBLUELED(lastdata.ledBlue);
//		res |= micomSetGREENLED(lastdata.ledGreen);
//		res |= micomSetREDLED(lastdata.ledRed);
	}
	else
	{
		memset(buffer, ' ', sizeof(buffer));
		res = micomWriteCommand(CLASS_DISPLAY2, buffer, VFD_MAX_LEN, 1);
		res |= micomClearIcons();
		res |= micomSetBLUELED(0);
//		res |= micomSetGREENLED(0);
//		res |= micomSetREDLED(0);
	}
	dprintk(100, "%s <\n", __func__);
	return res;
}

/****************************************************************
 *
 * micomInitialize: initialize the front processor.
 *
 */
int micomInitialize(void)
{
	char buffer[8];
	int  res = 0;

	dprintk(100, "%s >\n", __func__);

	/* MICOM Tx On */
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = 1;
	res = micomWriteCommand(CLASS_MCTX, buffer, 7, 0);

	/* MICOM Tx Data Blocking Off */
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = 0;
	res |= micomWriteCommand(CLASS_BLOCK, buffer, 7, 0);

	/* Mute request On */
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = 1;
	res |= micomWriteCommand(CLASS_MUTE, buffer, 7, 0);
	/* Watchdog On */
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = 1;
	res |= micomWriteCommand(CLASS_WDT, buffer, 7, 0);

	/* Send custom IR data */
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = IR_CUSTOM_HI;  // 0x01
	buffer[1] = IR_CUSTOM_LO;  // 0xfe
	buffer[2] = IR_POWER;  // 0x5f
	buffer[3] = FRONT_POWER; // 0x00
	buffer[4] = 0x01; // last power  (on?)
	res |= micomWriteCommand(CLASS_FIX_IR, buffer, 5, 0);

	dprintk(100, "%s <\n", __func__);
	return res;
}

/****************************************************************
 *
 * micom_init_func: initialize the driver.
 *
 */
int micom_init_func(void)
{
	int vLoop;
	int res;

	dprintk(100, "%s >\n", __func__);

	sema_init(&write_sem, 1);

	dprintk(0, "Vitamin HD5000 VFD/MICOM module initializing\n");

	micomInitialize();  // initialize the front processor

	res = micomSetBrightness(5);
	msleep(10);
	res |= micomClearIcons();
	msleep(10);
	res |= micomSetBLUELED(0);  // Blue LED off
	msleep(10);
	res |= micomSetGREENLED(0);  // Green LED off
	msleep(10);
	res |= micomSetREDLED(0);  // Red LED off
	msleep(10);
#if 0
	res |= micomWriteString(" Vitamin HD", strlen(" Vitamin HD"));
#else
	res |= clear_display();
#endif
	dprintk(100, "%s <\n", __func__);
	return res;
}

/****************************************
 *
 * Code for writing to /dev/vfd
 *
 */
static ssize_t MICOMdev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char *kernel_buf;
	int minor, vLoop, res = 0;
	int llen;
	char buf[64];
	int offset;
	int pos;
	char *b;

	dprintk(100, "%s > (len %d, offs %d)\n", __func__, len, (int) *off);

	if (len == 0)
	{
		return len;
	}
	minor = -1;
	for (vLoop = 0; vLoop < LASTMINOR; vLoop++)
	{
		if (FrontPanelOpen[vLoop].fp == filp)
		{
			minor = vLoop;
		}
	}
	if (minor == -1)
	{
		dprintk(0, "Error: Bad Minor\n");
		return -1; //FIXME
	}
	dprintk(70, "minor = %d\n", minor);

	/* do not write to the remote control */
	if (minor == FRONTPANEL_MINOR_RC)
	{
		return -EOPNOTSUPP;
	}
	kernel_buf = kmalloc(len, GFP_KERNEL);

	if (kernel_buf == NULL)
	{
		dprintk(0, "%s return -ENOMEM <\n", __func__);
		return -ENOMEM;
	}
	copy_from_user(kernel_buf, buff, len);

	if (write_sem_down())
	{
		return -ERESTARTSYS;
	}
	llen = len;
	if (llen >= 64)  // do not display more than 64 characters
	{
		llen = 64;
	}

	/* strip if present, a new line at the end */
	if (kernel_buf[len - 1] == '\n')
	{
		llen--;
	}

	if (llen <= VFD_MAX_LEN)  //no scroll
	{
		res = micomWriteString(kernel_buf, llen);
	}
	else  // scroll, display string is longer than display length
	{
		// initial display starting at 3rd position to ease reading
		memset(buf, ' ', sizeof(buf));
		offset = 3;
		memcpy(buf + offset, kernel_buf, llen);
		llen += offset;
		buf[llen + VFD_MAX_LEN] = '\0';  // terminate string

		// scroll text
		b = buf;
		for (pos = 0; pos < llen; pos++)
		{
			res |= micomWriteString(b + pos, VFD_MAX_LEN);
			// sleep 300 ms
			msleep(300);
		}
		res |= clear_display();

		// final display
		res |= micomWriteString(buf + offset, VFD_MAX_LEN);
	}

	kfree(kernel_buf);

	write_sem_up();

	dprintk(70, "%s < res=%d len=%d\n", __func__, res, len);

	if (res < 0)
	{
		return res;
	}
	else
	{
		return len;
	}
}

/**********************************
 *
 * Code for reading from /dev/vfd
 *
 */
static ssize_t MICOMdev_read(struct file *filp, char __user *buff, size_t len, loff_t *off)
{
	int minor, vLoop;
	dprintk(100, "%s > (len %d, offs %d)\n", __func__, len, (int) *off);

	minor = -1;
	for (vLoop = 0; vLoop < LASTMINOR; vLoop++)
	{
		if (FrontPanelOpen[vLoop].fp == filp)
		{
			minor = vLoop;
		}
	}
	if (minor == -1)
	{
		dprintk(0, "Error: bad Minor\n");
		return -EUSERS;
	}
	dprintk(100, "minor = %d\n", minor);

	if (minor == FRONTPANEL_MINOR_RC)
	{
		int           size = 0;
		unsigned char data[20];

		memset(data, 0, sizeof(data));

		getRCData(data, &size);

		if (size > 0)
		{
			if (down_interruptible(&FrontPanelOpen[minor].sem))
			{
				return -ERESTARTSYS;
			}
			copy_to_user(buff, data, size);

			up(&FrontPanelOpen[minor].sem);

			dprintk(150, "%s < %d\n", __func__, size);
			return size;
		}
		dumpValues();
		return 0;
	}
	/* copy the current display string to the user */
	if (down_interruptible(&FrontPanelOpen[minor].sem))
	{
		dprintk(0, "%s return -ERESTARTSYS <\n", __func__);
		return -ERESTARTSYS;
	}
	if (FrontPanelOpen[minor].read == lastdata.length)
	{
		FrontPanelOpen[minor].read = 0;

		up(&FrontPanelOpen[minor].sem);
		dprintk(0, "%s return 0 <\n", __func__);
		return 0;
	}
	if (len > lastdata.length)
	{
		len = lastdata.length;
	}
	/* fixme: needs revision because of utf8! */
	if (len > VFD_MAX_LEN)
	{
		len = VFD_MAX_LEN;
	}
	FrontPanelOpen[minor].read = len;
	copy_to_user(buff, lastdata.data, len);
	up(&FrontPanelOpen[minor].sem);
	dprintk(100, "%s < (len %d)\n", __func__, len);
	return len;
}

int MICOMdev_open(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(100, "%s >\n", __func__);

	/* needed! otherwise a racecondition can occur */
	if (down_interruptible(&write_sem))
	{
		return -ERESTARTSYS;
	}
	minor = MINOR(inode->i_rdev);
	dprintk(70, "open minor %d\n", minor);

	if (FrontPanelOpen[minor].fp != NULL)
	{
		dprintk(0, "%s: return -EUSERS\n", __func__);
		up(&write_sem);
		return -EUSERS;
	}
	FrontPanelOpen[minor].fp = filp;
	FrontPanelOpen[minor].read = 0;
	up(&write_sem);
	dprintk(100, "%s <\n", __func__);
	return 0;

}

int MICOMdev_close(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(100, "%s >\n", __func__);
	minor = MINOR(inode->i_rdev);
	dprintk(70, "close minor %d\n", minor);

	if (FrontPanelOpen[minor].fp == NULL)
	{
		dprintk(0, "%s: return -EUSERS\n", __func__);
		return -EUSERS;
	}
	FrontPanelOpen[minor].fp = NULL;
	FrontPanelOpen[minor].read = 0;
	dprintk(100, "%s <\n", __func__);
	return 0;
}

/****************************************
 *
 * IOCTL handling.
 *
 */
static int MICOMdev_ioctl(struct inode *Inode, struct file *File, unsigned int cmd, unsigned long arg)
{

	static int mode = 0;
	struct micom_ioctl_data *micom = (struct micom_ioctl_data *)arg;
	struct vfd_ioctl_data *data = (struct vfd_ioctl_data *)arg;  // mode 0
	int res = 0;
	int icon_nr, on;
	int level;
	int i;

	dprintk(100, "%s > cmd = 0x%08x\n", __func__, cmd);

	if (write_sem_down())
	{
		dprintk(0, "%s: return -ERESTARTSYS\n", __func__);
		return -ERESTARTSYS;
	}
	// perform IOCTL
	switch (cmd)
	{
		case VFDSETMODE:
		{
			mode = micom->u.mode.compat;
			break;
		}
		case VFDSETLED:
		{
			res = micomSetBLUELED(micom->u.led.on);
			break;
		}
		case VFDSETREDLED:
		{
			res = micomSetREDLED(micom->u.led.on);
			break;
		}
		case VFDSETGREENLED:
		{
			res = micomSetGREENLED(micom->u.led.on);
			break;
		}
		case VFDBRIGHTNESS:
		{
			if (mode == 0)
			{
				level = data->start_address;
				dprintk(10, "%s Set display brightness to %d (mode 0)\n", __func__, level);
			}
			else
			{
				level = micom->u.brightness.level;
				dprintk(10, "%s Set display brightness to %d (mode 1)\n", __func__, level);
			}
			res = micomSetBrightness(level);
			break;
		}
		case VFDDISPLAYCLR:
		{
			res = clear_display();
			mode = 0;
			break;
		}
		case VFDDRIVERINIT:
		{
			res = micom_init_func();
			mode = 0;
			break;
		}
		case VFDICONDISPLAYONOFF:
		{
			icon_nr = (mode == 0 ? data->data[0] : micom->u.icon.icon_nr);
			on =      (mode == 0 ? data->data[4] : micom->u.icon.on);

			// Part one: translate E2 icon numbers to own icon numbers (vfd mode only)
			if (mode == 0)  // vfd mode
			{
				dprintk(10, "Handle Enigma2 icon %d\n", icon_nr);
				switch (icon_nr)
				{
					case 0x13:  // crypted
					{
						icon_nr = ICON_DOLLAR;
						break;
					}
					case 0x17:  // dolby
					{
						icon_nr = ICON_DOLBY;
						break;
					}
					case 0x15:  // MP3
					{
						icon_nr = ICON_MP3;
						break;
					}
					case 0x11:  // HD
					{
						icon_nr = ICON_HD;
						break;
					}
					case 0x1e:  // record
					{
						icon_nr = ICON_RECORD;
						break;
					}
					default:
					{
						break;
					}
				}
				dprintk(10, "E2 icon number translated too: %d\n", icon_nr);
			}
			if (icon_nr == ICON_ALL)
			{
				for (i = ICON_MIN; i < ICON_MAX; i++)
				{
					res = micomSetIcon(i, on);
				}
			}
			else
			{
				res = micomSetIcon(icon_nr, on);

			}
			mode = 0;  // fall back to vfd mode
			break;
		}
		case VFDSTANDBY:
		{
			clear_display();
			dprintk(10, "Set deep standby mode\n");
//			res = micomSetStandby(micom->u.standby.time);
			res = micomSetStandby();
			break;
		}
		case VFDREBOOT:
		{
			res = micomReboot();
			mode = 0;
			break;
		}
#if 0
		case VFDSETTIME:
		{
			if (micom->u.time.time != 0)
			{
				dprintk(10, "Set frontpanel time to (MJD=) %d - %02d:%02d:%02d (local)\n", (micom->u.time.time[0] & 0xff) * 256 + (micom->u.time.time[1] & 0xff),
					micom->u.time.time[2], micom->u.time.time[3], micom->u.time.time[4]);
				res = micomSetTime(micom->u.time.time);
			}
			mode = 0;  // go back to vfd mode
			break;
		}
#endif
		case VFDDISPLAYCHARS:
		{
			if (mode == 0)
			{
				data->data[data->length] = 0;  // terminate string to show
				dprintk(10, "Write string (mode 0): [%s] (length = %d)\n", data->data, data->length);

				res = micomWriteString(data->data, data->length);
			}
			else
			{
				dprintk(10, "Write string (mode 1): not supported!\n");
				mode = 0;  // go back to vfd mode
			}
			res = micomWriteString(data->data, data->length);
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
				res = micomSetDisplayOnOff(micom->u.light.onoff);
				mode = 0;  // go back to vfd mode
				break;
		}
#if defined(VFDTEST)
		case VFDTEST:
		{
			res = micomVfdTest(micom->u.test.data);
			res |= copy_to_user((void *)arg, &(micom->u.test.data), 12);
			mode = 0;  // go back to vfd mode
			break;
		}
#endif
		case 0x5305:
		{
			break;
		}
		case VFDLEDBRIGHTNESS:
		case VFDGETWAKEUPMODE:
		case VFDGETTIME:
		case VFDSETTIME:
		{
			dprintk(0, "Unsupported IOCTL 0x%x\n", cmd);
			mode = 0;  // go back to vfd mode
			break;
		}
		default:
		{
			dprintk(0, "Unknown IOCTL 0x%x\n", cmd);
			mode = 0;  // go back to vfd mode
			break;
		}
	}
	write_sem_up();
	dprintk(100, "%s <\n", __func__);
	return res;
}

struct file_operations vfd_fops =
{
	.owner    = THIS_MODULE,
	.ioctl    = MICOMdev_ioctl,
	.write    = MICOMdev_write,
	.read     = MICOMdev_read,
	.open     = MICOMdev_open,
	.release  = MICOMdev_close
};
// vim:ts=4
