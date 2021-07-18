/****************************************************************************
 *
 * kathren_micom_main.c
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
 * Description:
 *
 * Kathrein UFS922 MICOM Kernelmodule ported from MARUSYS uboot source,
 * from vfd driver and from tf7700 frontpanel handling.
 *
 * Devices:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *  - /dev/rc  (reading of key events)
 *  - /dev/rtc (battery clock)
 *
 * TODO:
 * - implement a real led and button driver?!
 * - implement a real event driver?!
 *
 * - FOR UFS912:
 * two commands that are not implemented:
 * 0x55 -> no answer
 * 0x55 0x02 0xff 0x80 0x46 0x01 0x00 0x00
 *
 *****************************************************************************
 *
 * This driver covers the following models:
 * 
 * Kathrein UFS912
 * Kathrein UFS913
 * Kathrein UFS922
 * Kathrein UFC960
 *
 *****************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * ----------------------------------------------------------------------------
 * 20210703 Audioniek       Added code for all icons on/off.
 * 20210703 Audioniek       Add support for VFDSETFAN (ufs922 only).
 *
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
// For RTC
#if defined CONFIG_RTC_CLASS
#include <linux/rtc.h>
#include <linux/platform_device.h>
#endif

#include "kathrein_micom.h"
#include "kathrein_micom_asc.h"


//----------------------------------------------
#define EVENT_BTN                  0xd1
#define EVENT_RC                   0xd2
#define EVENT_ERR                  0xF5
#define EVENT_OK1                  0xFA
#define EVENT_OK2                  0xF1
#define EVENT_ANSWER_GETTIME       0xB9
#define EVENT_ANSWER_WAKEUP_REASON 0x77
#define EVENT_ANSWER_VERSION       0x85

#define DATA_BTN_EVENT   0
#define DATA_BTN_KEYCODE 1
#define DATA_BTN_NEXTKEY 4

// defaults for module parameters
short paramDebug = 0;
int waitTime = 1000;
char *gmt_offset = "3600";  // GMT offset is plus one hour as default

static unsigned char expectEventData = 0;
static unsigned char expectEventId = 1;

#define ACK_WAIT_TIME msecs_to_jiffies(500)

#define cPackageSize         8
#define cGetTimeSize         8
#define cGetWakeupReasonSize 8
#define cGetVersionSize      8

#define cMinimumSize         6

#define BUFFERSIZE           256     //must be 2 ^ n

static unsigned char RCVBuffer [BUFFERSIZE];
static int           RCVBufferStart = 0, RCVBufferEnd = 0;

static unsigned char KeyBuffer [BUFFERSIZE];
static int           KeyBufferStart = 0, KeyBufferEnd = 0;

static unsigned char OutBuffer [BUFFERSIZE];
static int           OutBufferStart = 0, OutBufferEnd = 0;

static wait_queue_head_t wq;
static wait_queue_head_t rx_wq;
static wait_queue_head_t ack_wq;
static int dataReady = 0;


#if defined(UFS922)
unsigned long fan_registers;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
struct stpio_pin* fan_pin1;
#endif
struct stpio_pin* fan_pin;
#endif

#if defined CONFIG_RTC_CLASS
int rtc_offset = 3600;
#define RTC_NAME "micom-rtc"
static struct platform_device *rtc_pdev;

extern int micomGetTime(void);
extern int micomSetTime(char *time);
//extern int micomGetWakeUpTime(char *time);
extern int micomSetWakeUpTime(char *time);
int date2days(int year, int mon, int day, int *yday);

// place to save the alarm time set
char al_sec;
char al_min;
char al_hour;
char al_day;
char al_month;
char al_year;
#endif

//----------------------------------------------

/* queue data ->transmission is done in the irq */
void micom_putc(unsigned char data)
{
	unsigned int *ASC_X_INT_EN = (unsigned int *)(ASCXBaseAddress + ASC_INT_EN);

	OutBuffer [OutBufferStart] = data;
	OutBufferStart = (OutBufferStart + 1) % BUFFERSIZE;

	/* if irq is not enabled, enable it */
	if (!(*ASC_X_INT_EN & ASC_INT_STA_THE))
	{
		*ASC_X_INT_EN = *ASC_X_INT_EN | ASC_INT_STA_THE;
	}
}

//----------------------------------------------

void ack_sem_up(void)
{
	dataReady = 1;
	wake_up_interruptible(&ack_wq);
}

int ack_sem_down(void)
{
	int err = 0;

	dataReady = 0;

	err  = wait_event_interruptible_timeout(ack_wq, dataReady == 1, ACK_WAIT_TIME);
	if (err == -ERESTARTSYS)
	{
		dprintk(1, "%s: wait_event_interruptible failed\n", __func__);
		return err;
	}
	else if (err == 0)
	{
		dprintk(1, "%s: Timeout waiting on ACK\n", __func__);
	}
	else
	{
		dprintk(50, "Command processed - remaining jiffies %d\n", err);
	}
	return 0;
}
EXPORT_SYMBOL(ack_sem_down);

//------------------------------------------------------------------
int getLen(int expectedLen)
{
	int i, j, len;

	i = 0;
	j = RCVBufferEnd;

	while (1)
	{
		if (RCVBuffer[j] == 0x0d)
		{
			if ((expectedLen == -1) || (i == expectedLen - 1))
			{
				break;
			}
		}
		j++;
		i++;
		if (j >= BUFFERSIZE)
		{
			j = 0;
		}
		if (j == RCVBufferStart)
		{
			i = -1;
			break;
		}
	}
	len = i + 1;
	return len;
}

void handleCopyData(int len)
{
	int i, j;

	unsigned char *data = kmalloc(len, GFP_KERNEL);

	i = 0;
	j = RCVBufferEnd;

	while (i != len)
	{
		data[i] = RCVBuffer[j];
		j++;
		i++;

		if (j >= BUFFERSIZE)
		{
			j = 0;
		}
		if (j == RCVBufferStart)
		{
			break;
		}
	}
	copyData(data, len);
	kfree(data);
}

void dumpData(void)
{
	int i, j, len;

	len = getLen(-1);

	if (len == 0)
	{
		return;
	}
	i = RCVBufferEnd;

	for (j = 0; j < len; j++)
	{
//		dprintk(1, "Received data[%02d]=0x%02x\n", i, data[i]);	
		i++;
		if (i >= BUFFERSIZE)
		{
			i = 0;
		}
		if (i == RCVBufferStart)
		{
			i = -1;
			break;
		}
	}
	printk("\n");
}

void dumpValues(void)
{
	dprintk(100, "BufferStart %d, BufferEnd %d, len %d\n", RCVBufferStart, RCVBufferEnd, getLen(-1));

	if (RCVBufferStart != RCVBufferEnd)
	{
		if (paramDebug >= 50)
		{
			dumpData();
		}
	}
}

void getRCData(unsigned char *data, int *len)
{
	int i, j;

	dprintk(100, "%s >, KeyStart %d KeyEnd %d\n", __func__, KeyBufferStart, KeyBufferEnd);

	while (KeyBufferStart == KeyBufferEnd)
	{
		dprintk(200, "%s %d - %d\n", __func__, KeyBufferStart, KeyBufferEnd);

		if (wait_event_interruptible(wq, KeyBufferStart != KeyBufferEnd))
		{
			dprintk(1, "%s: wait_event_interruptible failed\n", __func__);
			return;
		}
	}
	dprintk(50, "%s up\n", __func__);
	i = 0;
	j = KeyBufferEnd;
	*len = cPackageSize;

	while (i != *len)
	{
		data[i] = KeyBuffer[j];
		j++;
		i++;
		if (j >= BUFFERSIZE)
		{
			j = 0;
		}
		if (j == KeyBufferStart)
		{
			break;
		}
	}
	KeyBufferEnd = (KeyBufferEnd + cPackageSize) % BUFFERSIZE;
	dprintk(100, "%s <len %d, Start %d End %d\n", __func__, *len, KeyBufferStart, KeyBufferEnd);
}

static void processResponse(void)
{
	int len, i;

	if (paramDebug >= 100)
	{
		dumpData();
	}
	if (expectEventId)
	{
		/* DATA_BTN_EVENT can be wrapped to start */
		int index = (RCVBufferEnd + DATA_BTN_EVENT) % BUFFERSIZE;

		expectEventData = RCVBuffer[index];

		expectEventId = 0;
	}
	dprintk(100, "event 0x%02x\n", expectEventData);

	if (expectEventData)
	{
		switch (expectEventData)
		{
			case EVENT_BTN:
			{
				len = getLen(cPackageSize);

				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cPackageSize)
				{
					goto out_switch;
				}
				dprintk(30, "EVENT_BTN complete\n");

				if (paramDebug >= 50)
				{
					dumpData();
				}
				/* copy data */
				for (i = 0; i < cPackageSize; i++)
				{
					int from, to;

					from = (RCVBufferEnd + i) % BUFFERSIZE;
					to = KeyBufferStart % BUFFERSIZE;

					KeyBuffer[to] = RCVBuffer[from];

					KeyBufferStart = (KeyBufferStart + 1) % BUFFERSIZE;
				}
				wake_up_interruptible(&wq);
				RCVBufferEnd = (RCVBufferEnd + cPackageSize) % BUFFERSIZE;
				break;
			}
			case EVENT_RC:
			{
				len = getLen(cPackageSize);

				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cPackageSize)
				{
					goto out_switch;
				}
				dprintk(30, "EVENT_RC complete\n");
				dprintk(50, "Buffer start=%d, Buffer end=%d\n",  RCVBufferStart,  RCVBufferEnd);

				if (paramDebug >= 50)
				{
					dumpData();
				}
				/* copy data */
				for (i = 0; i < cPackageSize; i++)
				{
					int from, to;

					from = (RCVBufferEnd + i) % BUFFERSIZE;
					to = KeyBufferStart % BUFFERSIZE;

					KeyBuffer[to] = RCVBuffer[from];

					KeyBufferStart = (KeyBufferStart + 1) % BUFFERSIZE;
				}
				wake_up_interruptible(&wq);
				RCVBufferEnd = (RCVBufferEnd + cPackageSize) % BUFFERSIZE;
				break;
			}
			case EVENT_ERR:
			{
				len = getLen(-1);

				if (len == 0)
				{
					goto out_switch;
				}
				dprintk(1, "Neg. response received\n");

				/* if there is a waiter for an acknowledge ... */
				errorOccured = 1;
				ack_sem_up();

				/* discard all data */
				RCVBufferEnd = (RCVBufferEnd + len) % BUFFERSIZE;
				break;
			}
			case EVENT_OK1:
			case EVENT_OK2:
			{
				len = getLen(-1);

				if (len == 0)
				{
					goto out_switch;
				}
				dprintk(50, "EVENT_OK1/2: Pos. response received\n");

				/* if there is a waiter for an acknowledge ... */
				errorOccured = 0;
				ack_sem_up();

				RCVBufferEnd = (RCVBufferEnd + len) % BUFFERSIZE;
				break;
			}
			case EVENT_ANSWER_GETTIME:
			{
				len = getLen(cGetTimeSize);

				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cGetTimeSize)
				{
					goto out_switch;
				}
				handleCopyData(len);

				/* if there is a waiter for an acknowledge ... */
				dprintk(50, "EVENT_ANSWER_GETTIME: Pos. response received\n");
				errorOccured = 0;
				ack_sem_up();

				RCVBufferEnd = (RCVBufferEnd + cGetTimeSize) % BUFFERSIZE;
				break;
			}
			case EVENT_ANSWER_WAKEUP_REASON:
			{
				len = getLen(cGetWakeupReasonSize);

				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cGetWakeupReasonSize)
				{
					goto out_switch;
				}
				handleCopyData(len);

				/* if there is a waiter for an acknowledge ... */
				dprintk(50, "EVENT_ANSWER_WAKEUP_REASON: Pos. response received\n");
				errorOccured = 0;
				ack_sem_up();

				RCVBufferEnd = (RCVBufferEnd + cGetWakeupReasonSize) % BUFFERSIZE;
				break;
			}
			case EVENT_ANSWER_VERSION:
			{
				len = getLen(cGetVersionSize);

				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cGetVersionSize)
				{
					goto out_switch;
				}
				handleCopyData(len);
				/* if there is a waiter for an acknowledge ... */
				dprintk(50, "EVENT_ANSWER_VERSION: Pos. response received\n");
				errorOccured = 0;
				ack_sem_up();
				RCVBufferEnd = (RCVBufferEnd + cGetVersionSize) % BUFFERSIZE;
				break;
			}
			default: // Ignore Response
			{
				dprintk(1, "Invalid/unknown Response 0x%02x\n", expectEventData);
				dprintk(50, "Buffer start=%d, Buffer end=%d\n",  RCVBufferStart,  RCVBufferEnd);
				dumpData();
				/* discard all data, because this happens currently
				 * sometimes. dont know the problem here.
				 */
				RCVBufferEnd = RCVBufferStart;
				break;
			}
		}
	}

out_switch:
	expectEventId = 1;
	expectEventData = 0;
}

static irqreturn_t FP_interrupt(int irq, void *dev_id)
{
	unsigned int  *ASC_X_INT_STA = (unsigned int *)(ASCXBaseAddress + ASC_INT_STA);
	unsigned int  *ASC_X_INT_EN  = (unsigned int *)(ASCXBaseAddress + ASC_INT_EN);
	unsigned char *ASC_X_RX_BUFF = (unsigned char *)(ASCXBaseAddress + ASC_RX_BUFF);
	char          *ASC_X_TX_BUFF = (char *)(ASCXBaseAddress + ASC_TX_BUFF);
	unsigned char dataArrived = 0;

	// Run this loop as long as data is ready to be read: RBF
	while (*ASC_X_INT_STA & ASC_INT_STA_RBF)
	{
		// Save the new read byte at the proper place in the received buffer
		RCVBuffer[RCVBufferStart] = *ASC_X_RX_BUFF;

		dprintk(201, "%s: RCVBuffer[%03u] = %02X\n", RCVBufferStart, RCVBuffer[RCVBufferStart], __func__);
		// We are too fast, let's make a break
		udelay(0);

		// Increase the received buffer counter, reset if > than max BUFFERSIZE
		// TODO: Who resets this counter? nobody always write until buffer is full seems a bad idea
		RCVBufferStart = (RCVBufferStart + 1) % BUFFERSIZE;

		dataArrived = 1;

		// If the buffer counter == the buffer end, throw error.
		// What is this ?
		dprintk(201, "RCVBufferStart(%03u) == RCVBufferEnd(%03u)\n", RCVBufferStart, RCVBufferEnd);
		if (RCVBufferStart == RCVBufferEnd)
		{
			dprintk(1, "%s: FP: RCV buffer overflow!!!\n", __func__);
		}
	}
	if (dataArrived)
	{
		wake_up_interruptible(&rx_wq);
	}
	while ((*ASC_X_INT_STA & ASC_INT_STA_THE)
	    && (*ASC_X_INT_EN & ASC_INT_STA_THE)
	    && (OutBufferStart != OutBufferEnd))
	{
		*ASC_X_TX_BUFF = OutBuffer[OutBufferEnd];
		OutBufferEnd = (OutBufferEnd + 1) % BUFFERSIZE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		// We are too fast, lets make a break
		udelay(0);
#endif
	}

	/* if all the data is transmitted disable irq, otherwise
	 * system is overflowed with irq's
	 */
	if (OutBufferStart == OutBufferEnd)
	{
		if (*ASC_X_INT_EN & ASC_INT_STA_THE)
		{
			*ASC_X_INT_EN &= ~ASC_INT_STA_THE;
		}
	}
	return IRQ_HANDLED;
}

int micomTask(void *dummy)
{
	daemonize("micomTask");

	allow_signal(SIGTERM);

	while (1)
	{
		int dataAvailable = 0;

		if (wait_event_interruptible(rx_wq, (RCVBufferStart != RCVBufferEnd)))
		{
			dprintk(1, "wait_event_interruptible failed\n");
			continue;
		}
		if (RCVBufferStart != RCVBufferEnd)
		{
			dataAvailable = 1;
		}
		while (dataAvailable)
		{
			processResponse();

			if (RCVBufferStart == RCVBufferEnd)
			{
				dataAvailable = 0;
			}
			dprintk(150, "start %d end %d\n",  RCVBufferStart,  RCVBufferEnd);
		}
	}
	printk("micomTask died!\n");
	return 0;
}

#if defined(UFS922)
/******************************************************
 *
 * Fan speed code
 *
 * Code taken from fan_ufs922.c
 *
 */
static int __init init_fan_module(void)
{
	fan_registers = (unsigned long)ioremap(0x18010000, 0x100);
	dprintk(50, "fan_registers = 0x%.8lx\n", fan_registers);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
	fan_pin1 = stpio_request_pin (4, 7, "fan ctrl", STPIO_ALT_OUT);
	fan_pin = stpio_request_pin (4, 5, "fan ctrl", STPIO_OUT);
	stpio_set_pin(fan_pin, 1);
	dprintk(50, "fan pin %p\n", fan_pin);
#else
	fan_pin = stpio_request_pin (4, 7, "fan ctrl", STPIO_ALT_OUT);
#endif

	// not sure if first one is necessary
	ctrl_outl(0x200, fan_registers + 0x50);
	
	// set a default speed, because default is zero
	ctrl_outl(130, fan_registers + 0x4);
	return 0;
}

static void __exit cleanup_fan_module(void)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17) 
	if (fan_pin1 != NULL)
	{
		stpio_free_pin (fan_pin1);
	}
	if (fan_pin != NULL)
	{
		stpio_set_pin(fan_pin, 0);
		stpio_free_pin (fan_pin);
	}	
#else
	if (fan_pin != NULL)
	{
		stpio_free_pin(fan_pin);
	}
#endif
}
// end of fan driver code
#endif

/*----- RTC driver -----*/
#if defined CONFIG_RTC_CLASS
/* struct rtc_time
{
	int tm_sec;
	int tm_min;
	int tm_hour;
	int tm_mday;
	int tm_mon;
	int tm_year;
	int tm_wday;
	int tm_yday;
	int tm_isdst;
}
*/

#define LEAPYEAR(year) (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year) (LEAPYEAR(year) ? 366 : 365)
static const int _ytab[2][12] =
{
	{ 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
	{ 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};

int date2days(int year, int mon, int day, int *yday)
{
	int days;

	// process the days in the current month
	day--; // do not count today
	days = day;

	// process the days in the remaining months of this year
	mon--; // do not count the current month, as it is done already
	while (mon > 0)
	{
		days += _ytab[LEAPYEAR(year)][mon - 1];
		mon--;
	}
	*yday = days;
	// process the remaining years
	year--; // do not count current year, as it is done already
	while (year >= 0)
	{
		days += YEARSIZE(year);
		year--;
	}
	days += 10957; // add days since linux epoch
	return days;
}

static int rtc_time2tm(char *time, struct rtc_time *tm)
{ // 6 bytes (ioctl_data[?]) -> struct rtc_time
	int year, mon, day, days, yday;
	int hour, min, sec;

	dprintk(100, "%s >\n", __func__);
	dprintk(5, "Time to convert: %02x-%02x-20%02x %02x:%02x:%02x\n", (int)time[3],(int)time[4], (int)time[5], (int)time[2], (int)time[1], (int)time[0]);
	// calculate the number of days since 01-01-1970 (Linux epoch)
	// the RTC starts at 01-01-2000 as it has no century
	year        = ((time[5] >> 4) * 10) + (time[5] & 0x0f);
	tm->tm_year = year + 100; // RTC starts at 01-01-2000, struct rtc_time at 1900
	mon         = ((time[4] >> 4) * 10) + (time[4] & 0x0f);
	tm->tm_mon  = mon - 1;
	tm->tm_mday = ((time[3] >> 4) * 10) + (time[3] & 0x0f);
	tm->tm_hour = ((time[2] >> 4) * 10) + (time[2] & 0x0f);
	tm->tm_min  = ((time[1] >> 4) * 10) + (time[1] & 0x0f);
	tm->tm_sec  = ((time[0] >> 4) * 10) + (time[0] & 0x0f);
	dprintk(5, "Date: %02d-%02d-%04d, time: %02d:%02d:%02d\n", tm->tm_mday, tm->tm_mon + 1, tm->tm_year + 1900, tm->tm_hour, tm->tm_min, tm->tm_sec);

	// calculate the number of days since linux epoch (00:00:00 UTC 01/01/1970)
	days = date2days(tm->tm_year, mon, tm->tm_mday, &yday);
	tm->tm_wday = (days + 4) % 7; // 01-01-1970 was a Thursday
	tm->tm_yday = yday;
	tm->tm_isdst = -1;
	dprintk(5, "%s weekday = %1d, yearday = %3d\n", __func__, tm->tm_wday, tm->tm_yday);
	dprintk(100, "%s <\n", __func__);
	return 0;	
}

static int tm2rtc_time(char *uTime, struct rtc_time *tm)
{ // struct rtc_time -> 6 bytes YMDhms
	dprintk(5, "Time to convert: %02d-%02d-20%02d %02d:%02d:%02dx\n", tm->tm_mday, tm->tm_mon + 1, tm->tm_year - 100, tm->tm_hour, tm->tm_min, tm->tm_sec);
	uTime[0] = ((tm->tm_sec / 10) << 4) + tm->tm_sec % 10;
	uTime[1] = ((tm->tm_min / 10) << 4) + tm->tm_min % 10;
	uTime[2] = ((tm->tm_hour / 10) << 4) + tm->tm_hour % 10;
	uTime[3] = ((tm->tm_mday / 10) << 4) + tm->tm_mday % 10;
	uTime[4] = (((tm->tm_mon + 1) / 10) << 4) + (tm->tm_mon + 1) % 10;
	uTime[5] = (((tm->tm_year - 100) / 10) << 4) + (tm->tm_year - 100) % 10;
	dprintk(5, "Converted time: %02x-%02x-20%02x %02x:%02x:%02x\n", (int)uTime[3],(int)uTime[4], (int)uTime[5], (int)uTime[2], (int)uTime[1], (int)uTime[0]);
	return 0;
}

static int micom_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	int res = 0;
	
	dprintk(5, "%s >\n", __func__);
	res = micomGetTime();	
	dprintk(10, "FP/RTC time: %02x:%02x:%02x\n", (int)ioctl_data[2], (int)ioctl_data[1], (int)ioctl_data[0]);
	dprintk(10, "FP/RTC date: %02x-%02x-20%02x\n", (int)ioctl_data[3], (int)ioctl_data[4], (int)ioctl_data[5]);
	rtc_time2tm(ioctl_data, tm);
	dprintk(5, "%s <\n", __func__);
	return 0;
}

static int micom_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int res = 0;
	char uTime[6];

	res = tm2rtc_time(uTime, tm);
	res |= micomSetTime(uTime);
	dprintk(5, "%s < res: %d\n", __func__, res);
	return res;
}

static int micom_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *al)
{
/*
struct rtc_wkalrm
{
	unsigned char   enabled
	unsigned char   pending
	struct rtc_time time
}
*/
	char a_time[6];
	//int res = 0;

	dprintk(5, "%s >\n", __func__);
//	res = micomGetWakeUpTime(a_time);

	a_time[0] = al_sec;
	a_time[1] = al_min;
	a_time[2] = al_hour;
	a_time[3] = al_day;
	a_time[4] = al_month;
	a_time[5] = al_year;

	if (al->enabled)
	{
		rtc_time2tm(a_time, &al->time);
	}
	dprintk(5, "%s < Enabled: %d RTC alarm time: %d time: %d\n", __func__, al->enabled, (int)&a_time, a_time);
	return 0;
}

static int micom_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	char a_time[6];
	int res = 0;

	dprintk(5, "%s >\n", __func__);
	if (al->enabled)
	{
		res |= tm2rtc_time(a_time, &al->time);
	}
	res = micomSetWakeUpTime(a_time);
	// save store alarm time in globals, as the fp
	// has no command to read the wakeup time
	if (!res)
	{
		al_sec   = a_time[0];
		al_min   = a_time[1];
		al_hour  = a_time[2];
		al_day   = a_time[3];
		al_month = a_time[4];
		al_year  = a_time[5];
	}
	dprintk(5, "%s < Enabled: %d alarm time: %02d-%02d-20%02d %02d:%02d:%02d\n", __func__,	(int)al->enabled, (int)al_day, (int)al_month, (int)al_year, (int)al_hour, (int)al_min, (int)al_sec);
	return 0;
}

static const struct rtc_class_ops micom_rtc_ops =
{
	.read_time  = micom_rtc_read_time,
	.set_time   = micom_rtc_set_time,
	.read_alarm = micom_rtc_read_alarm,
	.set_alarm  = micom_rtc_set_alarm
};

static int __devinit micom_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;

	/* I have no idea where the pdev comes from, but it needs the can_wakeup = 1
	 * otherwise we don't get the wakealarm sysfs attribute... :-) */
	dprintk(5, "%s >\n", __func__);
	printk("Micom front panel real time clock\n");
	pdev->dev.power.can_wakeup = 1;
	rtc = rtc_device_register("micom", &pdev->dev, &micom_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
	{
		return PTR_ERR(rtc);
	}
	platform_set_drvdata(pdev, rtc);
	dprintk(5, "%s < %p\n", __func__, rtc);
	return 0;
}

static int __devexit micom_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	dprintk(5, "%s %p >\n", __func__, rtc);
	rtc_device_unregister(rtc);
	platform_set_drvdata(pdev, NULL);
	dprintk(5, "%s <\n", __func__);
	return 0;
}

static struct platform_driver micom_rtc_driver =
{
	.probe = micom_rtc_probe,
	.remove = __devexit_p(micom_rtc_remove),
	.driver =
	{
		.name	= RTC_NAME,
		.owner	= THIS_MODULE
	},
};
#endif
//----------------------------------------------

extern void create_proc_fp(void);
extern void remove_proc_fp(void);

static int __init micom_init_module(void)
{
	int i = 0;

	// Address for Interrupt enable/disable
	unsigned int *ASC_X_INT_EN = (unsigned int *)(ASCXBaseAddress + ASC_INT_EN);
	// Address for FiFo enable/disable
	unsigned int *ASC_X_CTRL   = (unsigned int *)(ASCXBaseAddress + ASC_CTRL);

	dprintk(100, "%s >\n", __func__);

	//Disable all ASC interrupts
	*ASC_X_INT_EN = *ASC_X_INT_EN & ~0x000001ff;

	serial_init();

	init_waitqueue_head(&wq);
	init_waitqueue_head(&rx_wq);
	init_waitqueue_head(&ack_wq);

	for (i = 0; i < LASTMINOR; i++)
	{
		sema_init(&FrontPanelOpen[i].sem, 1);
	}
	kernel_thread(micomTask, NULL, 0);
	// Enable the FIFO
	*ASC_X_CTRL = *ASC_X_CTRL | ASC_CTRL_FIFO_EN;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
	i = request_irq(InterruptLine, (void *)FP_interrupt, IRQF_DISABLED /* SA_INTERRUPT */, "FP_serial", NULL);
#else
	i = request_irq(InterruptLine, (void *)FP_interrupt, SA_INTERRUPT, "FP_serial", NULL);
#endif

	if (!i)
	{
		*ASC_X_INT_EN = *ASC_X_INT_EN | 0x00000001;
	}
	else
	{
		dprintk(1, "Cannot get IRQ for fp asc\n");
	}
	msleep(waitTime);
	micom_init_func();

	if (register_chrdev(VFD_MAJOR, "VFD", &vfd_fops))
	{
		dprintk(1, "%s: Unable to get major %d for VFD\n", __func__, VFD_MAJOR);
	}

#if defined CONFIG_RTC_CLASS
	i = platform_driver_register(&micom_rtc_driver);
	if (i)
	{
		dprintk(1, "%s platform_driver_register failed: %d\n", __func__, i);
	}
	else
	{
		rtc_pdev = platform_device_register_simple(RTC_NAME, -1, NULL, 0);
	}

	if (IS_ERR(rtc_pdev))
	{
		dprintk(1, "%s platform_device_register_simple failed: %ld\n", __func__, PTR_ERR(rtc_pdev));
	}
#endif
#if defined(UFS922)
	// fan driver
	init_fan_module();
#endif
	create_proc_fp();
	dprintk(100, "%s <\n", __func__);
	return 0;
}

static void __exit micom_cleanup_module(void)
{
	dprintk(0, "Frontcontroller module unloading\n");

#if defined CONFIG_RTC_CLASS
	platform_driver_unregister(&micom_rtc_driver);
	platform_set_drvdata(rtc_pdev, NULL);
	platform_device_unregister(rtc_pdev);
#endif

	remove_proc_fp();
#if defined(UFS922)
	// fan driver
	cleanup_fan_module();
#endif
	unregister_chrdev(VFD_MAJOR, "VFD");

	free_irq(InterruptLine, NULL);
}

//----------------------------------------------

module_init(micom_init_module);
module_exit(micom_cleanup_module);

module_param(waitTime, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(waitTime, "Wait before init in ms (default=1000)");

module_param(gmt_offset, charp, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gmt_offset, "GMT offset (default=3600");

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled(default) >0=enabled(debuglevel)");

MODULE_DESCRIPTION("MICOM frontcontroller module, Kathrein version");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4

