/*
 * cuberevo_micom_main.c
 *
 * (c) 2011 konfetti
 * partly copied from user space implementation from M.Majoor
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
 * Driver for the frontcontroller of the following receivers:
 *
 * Cuberevo: 250HD / Mini / Mini II / (no model name) / 2000HD / 3000HD / 9500HD
 * AB      : IPbox91HD / IPbox900HD/IPbox910HD / IPbox9000HD
 * Vizyon  : revolution 810HD /820HD PVR / 8000HD PVR  
 *
 * Devices:
 *  - /dev/vfd  (vfd ioctls and read/write function)
 *  - /dev/rc   (reading of key events)
 *  - /dev/rtc0 (real time clock)
 *
 *
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 20190215 Audioniek       procfs added.
 * 20190216 Audioniek       Start of work on RTC driver.
 *
 ****************************************************************************************/
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

#include "cuberevo_micom.h"
#include "cuberevo_micom_asc.h"

//----------------------------------------------

#define EVENT_BTN_HI                     0xe1
#define EVENT_BTN_LO                     0xe2

#define EVENT_RC                         0xe0

#define EVENT_ANSWER_GETWAKEUP_SEC       0xe3
#define EVENT_ANSWER_GETWAKEUP_MIN       0xe4
#define EVENT_ANSWER_GETWAKEUP_HOUR      0xe5
#define EVENT_ANSWER_GETWAKEUP_DAY       0xe6
#define EVENT_ANSWER_GETWAKEUP_MONTH     0xe7
#define EVENT_ANSWER_GETWAKEUP_YEAR      0xe8

#define EVENT_ANSWER_GETMICOM_DAY        0xe9
#define EVENT_ANSWER_GETMICOM_MONTH      0xea
#define EVENT_ANSWER_GETMICOM_YEAR       0xeb

#define EVENT_ANSWER_RAM                 0xec
#define EVENT_ANSWER_WAKEUP_STATUS       0xed

#define EVENT_ANSWER_UNKNOWN1            0xee
#define EVENT_ANSWER_UNKNOWN2            0xef

// Globals
const char *driver_version = "1.08Audioniek";
short paramDebug = 0;
int waitTime = 1000;
static unsigned char expectEventData = 0;
static unsigned char expectEventId   = 1;
char *gmt_offset = "3600";  // GMT offset is plus one hour as default
int rtc_offset;
#define RTC_NAME "micom-rtc"
static struct platform_device *rtc_pdev;

extern int micomGetTime(char *time);
extern int micomSetTime(char *time);
extern int micomGetWakeUpTime(char *time);
extern int micomSetWakeUpTime(char *time);
int date2days(int year, int mon, int day, int *yday);

//----------------------------------------------

#define ACK_WAIT_TIME msecs_to_jiffies(500)

#define cPackageSize             2
#define cPackageSizeMicom        6
#define cPackageSizeDateTime     12
#define cPackageSizeWakeUpReason 2

#define cMinimumSize             2

#define BUFFERSIZE               512 // must be 2 ^ n

static unsigned char RCVBuffer [BUFFERSIZE];
static int RCVBufferStart = 0, RCVBufferEnd = 0;

static unsigned char KeyBuffer [BUFFERSIZE];
static int KeyBufferStart = 0, KeyBufferEnd = 0;

static unsigned char OutBuffer [BUFFERSIZE];
static int OutBufferStart = 0, OutBufferEnd = 0;

static wait_queue_head_t wq;
static wait_queue_head_t rx_wq;
static wait_queue_head_t ack_wq;
static int dataReady = 0;

/****************************************************************************************/

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
		dprintk(1, "wait_event_interruptible failed\n");
		return err;
	}
	else if (err == 0)
	{
		dprintk(1, "Timeout waiting on ACK\n");
	}
	else
	{
		dprintk(20, "Command processed - remaining jiffies %d\n", err);
	}
	return 0;
}
EXPORT_SYMBOL(ack_sem_down);

//------------------------------------------------------------------

int getLen(void)
{
	int len;

	len = RCVBufferStart - RCVBufferEnd;

	if (len < 0)
	{
		len += BUFFERSIZE;
	}
	return len;
}

void getRCData(unsigned char *data, int *len)
{
	int i, j;

	*len = 0;

	while (KeyBufferStart == KeyBufferEnd)
	{
		dprintk(200, "%s %d - %d\n", __func__, KeyBufferStart, KeyBufferEnd);

		if (wait_event_interruptible(wq, KeyBufferStart != KeyBufferEnd))
		{
			dprintk(1, "wait_event_interruptible failed\n");
			return;
		}
	}

	i = KeyBufferEnd % BUFFERSIZE;

	*len = cPackageSize;

	if (*len <= 0)
	{
		*len = 0;
		return;
	}

	i = 0;
	j = KeyBufferEnd;

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
	KeyBufferEnd = (KeyBufferEnd + *len) % BUFFERSIZE;
	dprintk(150, " < len %d, data[0] 0x%x, data[1] 0x%x, End %d\n", *len, data[0], data[1], KeyBufferEnd);
}

void handleCopyData(int len)
{
	int i, j;

	unsigned char *data = kmalloc(len / 2, GFP_KERNEL);

	i = 0;
	j = (RCVBufferEnd + 1) % BUFFERSIZE;

	while (i != len / 2)
	{
		dprintk(50, "%d. = 0x%02x\n", j,  RCVBuffer[j]);
		data[i] = RCVBuffer[j];
		j += 2; // filter answer tag
		j %= BUFFERSIZE;
		i++;

		if (j == RCVBufferStart)
		{
			break;
		}
	}
	copyData(data, len / 2);
	kfree(data);
}

void dumpData(void)
{
	int i, j, len;

	if (paramDebug < 150)
	{
		return;
	}
	len = getLen();

	if (len == 0)
	{
		return;
	}
	i = RCVBufferEnd;
	for (j = 0; j < len; j++)
	{
		dprintk(1, "Dump #%02d: 0x%02x\n", i, RCVBuffer[i]);
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
	dprintk(150, "BufferStart %d, BufferEnd %d, len %d\n", RCVBufferStart, RCVBufferEnd, getLen());

	if (RCVBufferStart != RCVBufferEnd)
	{
		if (paramDebug >= 50)
		{
			dumpData();
		}
	}
}

static void processResponse(void)
{
	int len, i;

	dumpData();

	if (expectEventId)
	{
		expectEventData = RCVBuffer[RCVBufferEnd];
		expectEventId = 0;
	}
	len = getLen();

	dprintk(100, "event 0x%02x %d %d %d\n", expectEventData, RCVBufferStart, RCVBufferEnd, len);

	if (expectEventData)
	{
		switch (expectEventData)
		{
			case EVENT_BTN_HI:
			case EVENT_BTN_LO:
			{
				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cPackageSize)
				{
					goto out_switch;
				}
				dprintk(1, "EVENT_BTN complete\n");
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
				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cPackageSize)
				{
					goto out_switch;
				}
				dprintk(1, "EVENT_RC complete %d %d\n", RCVBufferStart, RCVBufferEnd);
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
			case EVENT_ANSWER_GETMICOM_DAY:
			case EVENT_ANSWER_GETMICOM_MONTH:
			case EVENT_ANSWER_GETMICOM_YEAR:
			{
				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cPackageSizeMicom)
				{
					goto out_switch;
				}
				handleCopyData(len);
				dprintk(1, "Pos. response received (0x%0x)\n", expectEventData);
				errorOccured = 0;
				ack_sem_up();
				RCVBufferEnd = (RCVBufferEnd + cPackageSizeMicom) % BUFFERSIZE;
				break;
			}
			case EVENT_ANSWER_GETWAKEUP_SEC:
			case EVENT_ANSWER_GETWAKEUP_MIN:
			case EVENT_ANSWER_GETWAKEUP_HOUR:
			case EVENT_ANSWER_GETWAKEUP_DAY:
			case EVENT_ANSWER_GETWAKEUP_MONTH:
			case EVENT_ANSWER_GETWAKEUP_YEAR:
			{
				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cPackageSizeDateTime)
				{
					goto out_switch;
				}
				handleCopyData(len);
				dprintk(1, "Pos. response received (0x%0x)\n", expectEventData);
				errorOccured = 0;
				ack_sem_up();
				RCVBufferEnd = (RCVBufferEnd + cPackageSizeDateTime) % BUFFERSIZE;
				break;
			}
			case EVENT_ANSWER_RAM:
			case EVENT_ANSWER_UNKNOWN1:
			case EVENT_ANSWER_UNKNOWN2:
			{
				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cPackageSize)
				{
					goto out_switch;
				}
				handleCopyData(len);
				dprintk(1, "Pos. response received (0x%0x)\n", expectEventData);
				errorOccured = 0;
				ack_sem_up();
				RCVBufferEnd = (RCVBufferEnd + cPackageSize) % BUFFERSIZE;
				break;
			}
			case EVENT_ANSWER_WAKEUP_STATUS:
			{
				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cPackageSizeWakeUpReason)
				{
					goto out_switch;
				}
				handleCopyData(len);
				dprintk(1, "Pos. response received (0x%0x)\n", expectEventData);
				errorOccured = 0;
				ack_sem_up();
				RCVBufferEnd = (RCVBufferEnd + cPackageSizeWakeUpReason) % BUFFERSIZE;
				break;
			}
			default: // Ignore Response
			{
				dprintk(1, "Invalid Response %02x\n", expectEventData);
				dprintk(1, "start %d end %d\n",  RCVBufferStart,  RCVBufferEnd);
				dumpData();

				/* discard all data, because this happens currently
				 * sometimes. Do not know the problem here.
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
	unsigned int *ASC_X_INT_STA = (unsigned int *)(ASCXBaseAddress + ASC_INT_STA);
	unsigned int *ASC_X_INT_EN  = (unsigned int *)(ASCXBaseAddress + ASC_INT_EN);
	char         *ASC_X_RX_BUFF = (char *)(ASCXBaseAddress + ASC_RX_BUFF);
	char         *ASC_X_TX_BUFF = (char *)(ASCXBaseAddress + ASC_TX_BUFF);
	int          dataArrived = 0;

	if (paramDebug > 100)
	{
		dprintk(1, "i - ");
	}
	while (*ASC_X_INT_STA & ASC_INT_STA_RBF)
	{
		RCVBuffer [RCVBufferStart] = *ASC_X_RX_BUFF;
		RCVBufferStart = (RCVBufferStart + 1) % BUFFERSIZE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		// We are too fast, let us make a break
		udelay(0);
#endif
		dataArrived = 1;

		if (RCVBufferStart == RCVBufferEnd)
		{
			dprintk(1, "FP: RCV buffer overflow!!! (%d - %d)\n", RCVBufferStart, RCVBufferEnd);
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
		// We are too fast, let us make a break
		udelay(0);
#endif
	}
	/* If all the data is transmitted disable irq, otherwise
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
	dprintk(1, "micomTask died!\n");
	return 0;
}

//-- RTC driver -----------------------------------
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

static int rtc_time2tm(char *time, struct rtc_time *tm)
{ // 6 byte string YMDhms -> struct rtc_time
	int year, mon, day, days, yday;
	int hour, min, sec;

	dprintk(100, "%s >\n", __func__);
	// calculate the number of days since 01-01-1970 (Linux epoch)
	// the RTC starts at 01-01-2000 as it has no century
	year        = ((time[5] >> 4) * 10) + (time[5] & 0x0f);
	tm->tm_year = year + 100; // RTC starts at 01-01-2000
	mon         = ((time[4] >> 4) * 10) + (time[4] & 0x0f);
	tm->tm_mon  = mon - 1;
	tm->tm_mday = ((time[3] >> 4) * 10) + (time[3] & 0x0f);
	tm->tm_hour = ((time[2] >> 4) * 10) + (time[2] & 0x0f);
	tm->tm_min  = ((time[1] >> 4) * 10) + (time[1] & 0x0f);
	tm->tm_sec  = ((time[0] >> 4) * 10) + (time[0] & 0x0f);

	// calculate the number of days since linux epoch
	days = date2days(tm->tm_year, mon, tm->tm_mday, &yday);
	tm->tm_wday = (days + 4) % 7; // 01-01-1970 was a Thursday
	tm->tm_yday = yday;
	tm->tm_isdst = -1;
//	dprintk(5, "%s weekday = %1d, yearday = %3d\n", __func__, tm->tm_wday, tm->tm_yday);
	dprintk(100, "%s <\n", __func__);
	return 0;	
}

static int tm2rtc_time(char *uTime, struct rtc_time *tm)
{ // struct rtc_time -> YYMMDDhhmmss
	uTime[11] = (tm->tm_sec % 10) + '0'; // secs, units
	uTime[10] = (tm->tm_sec / 10) + '0'; // secs, tens
	uTime[9]  = (tm->tm_min % 10) + '0';
	uTime[8]  = (tm->tm_min / 10) + '0';;
	uTime[7]  = (tm->tm_hour % 10) + '0';;
	uTime[6]  = (tm->tm_hour / 10) + '0';;
	uTime[5]  = (tm->tm_mday % 10) + '0';;
	uTime[4]  = (tm->tm_mday / 10) + '0';;
	uTime[3]  = ((tm->tm_mon + 1) % 10) + '0';
	uTime[2]  = ((tm->tm_mon + 1) / 10) + '0';
	uTime[1]  = ((tm->tm_year - 100) % 10) + '0'; // RTC has no century
	uTime[0]  = ((tm->tm_year - 100) / 10) + '0'; // year, tens
	return 0;
}

static int micom_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	int res = -1;
	char fptime[6];

	dprintk(100, "%s >\n", __func__);
	memset(fptime, 0, sizeof(fptime));
	res = micomGetTime(fptime);  // get front panel clock time (local)
	res |= rtc_time2tm(fptime, tm);
	dprintk(100, "%s < res: %d\n", __func__, res);
	return res;
}

static int micom_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	char fptime[12];
	int res = -1;

	dprintk(100, "%s >\n", __func__);
	tm2rtc_time(fptime, tm);
	res = micomSetTime(fptime);
	dprintk(100, "%s < res: %d\n", __func__, res);
	return res;
}

static int micom_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	char fptime[6];
	int res = -1;

	dprintk(100, "%s >\n", __func__);
	memset(fptime, 0, sizeof(fptime));
	res = micomGetWakeUpTime(fptime);
	if (al->enabled)
	{
		rtc_time2tm(fptime, &al->time);
	}
	dprintk(5, "%s Enabled: %s RTC alarm time: %02x:%02x:%02x %02x-%02x-20%02x\n", __func__,
		al->enabled ? "yes" : "no", fptime[2], fptime[1], fptime[0], fptime[3], fptime[4], fptime[5]);
	dprintk(100, "%s <\n", __func__);
	return res;
}

static int micom_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	char fptime[12];
	int res = -1;

	dprintk(100, "%s >\n", __func__);
	dprintk(5, "%s > RTC alarm time to set: %02d:%02d:%02d %02d-%02d-%04d (enabled = %s)\n", __func__,
		al->time.tm_hour, al->time.tm_min, al->time.tm_sec,
		al->time.tm_mday, al->time.tm_mon + 1, al->time.tm_year + 1900,
		al->enabled ? "yes" : "no");
	if (al->enabled)
	{
		res = tm2rtc_time(fptime, &al->time);
	}
	res = micomSetWakeUpTime(fptime); // to be tested
	dprintk(100, "%s <\n", __func__);
	return res;
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
	 * otherwise we do not get the wakealarm sysfs attribute... :-) */
	dprintk(100, "%s >\n", __func__);
	dprintk(5,"Probe: Micom front panel real time clock\n");
	pdev->dev.power.can_wakeup = 1;
	rtc = rtc_device_register("micom", &pdev->dev, &micom_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
	{
		return PTR_ERR(rtc);
	}
	platform_set_drvdata(pdev, rtc);
	dprintk(100, "%s < %p\n", __func__, rtc);
	return 0;
}

static int __devexit micom_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	dprintk(100, "%s %p >\n", __func__, rtc);
	rtc_device_unregister(rtc);
	platform_set_drvdata(pdev, NULL);
	dprintk(100, "%s <\n", __func__);
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
#endif // CONFIG_RTC_CLASS

//-- module functions -------------------------------------------

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

	// Disable all ASC 2 interrupts
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
	i = request_irq(InterruptLine, (void *)FP_interrupt, IRQF_DISABLED, "FP_serial", NULL);
#else
	i = request_irq(InterruptLine, (void *)FP_interrupt, SA_INTERRUPT, "FP_serial", NULL);
#endif
	if (!i)
	{
		*ASC_X_INT_EN = *ASC_X_INT_EN | ASC_INT_STA_RBF;
	}
	else
	{
		dprintk(1, "Cannot get irq\n");
	}
	msleep(waitTime);
	micom_init_func();

	if (register_chrdev(VFD_MAJOR, "VFD", &vfd_fops))
	{
		dprintk(1, "Unable to get major %d for VFD/MICOM\n", VFD_MAJOR);
	}

#if defined CONFIG_RTC_CLASS
	i = platform_driver_register(&micom_rtc_driver);
	if (i)
	{
		dprintk(5, "%s platform_driver_register for RTC failed: %d\n", __func__, i);
	}
	else
	{
		rtc_pdev = platform_device_register_simple(RTC_NAME, -1, NULL, 0);
	}

	if (IS_ERR(rtc_pdev))
	{
		dprintk(5, "%s platform_device_register_simple for RTC failed: %ld\n", __func__, PTR_ERR(rtc_pdev));
	}
#endif
	create_proc_fp();

	dprintk(100, "%s <\n", __func__);
	return 0;
}

static void __exit micom_cleanup_module(void)
{
	printk("[micom] MICOM front processor module unloading\n");
	remove_proc_fp();

#if defined CONFIG_RTC_CLASS
	platform_driver_unregister(&micom_rtc_driver);
	platform_set_drvdata(rtc_pdev, NULL);
	platform_device_unregister(rtc_pdev);
#endif
	unregister_chrdev(VFD_MAJOR, "VFD");
	free_irq(InterruptLine, NULL);
}

//----------------------------------------------

module_init(micom_init_module);
module_exit(micom_cleanup_module);

#if defined(CUBEREVO_MINI)
MODULE_DESCRIPTION("MICOM frontcontroller module (CubeRevo Mini)");
#elif defined(CUBEREVO_MINI2)
MODULE_DESCRIPTION("MICOM frontcontroller module (CubeRevo Mini II)");
#elif defined(CUBEREVO_250HD)
MODULE_DESCRIPTION("MICOM frontcontroller module (CubeRevo 250HD)");
#elif defined(CUBEREVO)
MODULE_DESCRIPTION("MICOM frontcontroller module (CubeRevo)");
#elif defined(CUBEREVO_2000HD)
MODULE_DESCRIPTION("MICOM frontcontroller module (CubeRevo 2000HD)");
#elif defined(CUBEREVO_3000HD)
MODULE_DESCRIPTION("MICOM frontcontroller module (CubeRevo 3000HD)");
#else
MODULE_DESCRIPTION("MICOM frontcontroller module (Unknown)");
#endif

MODULE_AUTHOR("Konfetti, Audioniek");
MODULE_LICENSE("GPL");

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

module_param(waitTime, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(waitTime, "Wait before init in ms (default=1000)");

module_param(gmt_offset, charp, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gmt_offset, "GMT offset (default=3600");
// vim:ts=4
