/*
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

//----------------------------------------------
short paramDebug = 0;

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

static wait_queue_head_t   wq;
static wait_queue_head_t   rx_wq;
static wait_queue_head_t   ack_wq;
static int dataReady = 0;

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
		printk("[micom] %s: wait_event_interruptible failed\n", __func__);
		return err;
	}
	else if (err == 0)
	{
		printk("[micom] %s: Timeout waiting on ACK\n", __func__);
	}
	else
	{
		dprintk(30, "Command processed - remaining jiffies %d\n", err);
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
		printk("0x%02x ", RCVBuffer[i]);
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
	dprintk(50, "BuffersStart %d, BufferEnd %d, len %d\n", RCVBufferStart, RCVBufferEnd, getLen(-1));

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

	dprintk(50, "%s >, KeyStart %d KeyEnd %d\n", __func__, KeyBufferStart, KeyBufferEnd);

	while (KeyBufferStart == KeyBufferEnd)
	{
		dprintk(200, "%s %d - %d\n", __func__, KeyBufferStart, KeyBufferEnd);

		if (wait_event_interruptible(wq, KeyBufferStart != KeyBufferEnd))
		{
			printk("[micom] %s: wait_event_interruptible failed\n", __func__);
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
	dprintk(50, "%s <len %d, Start %d End %d\n", __func__, *len, KeyBufferStart, KeyBufferEnd);
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
				len = getLen(cPackageSize);

				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cPackageSize)
				{
					goto out_switch;
				}
				dprintk(1, "EVENT_RC complete\n");
				dprintk(30, "Buffer start=%d, Buffer end=%d\n",  RCVBufferStart,  RCVBufferEnd);

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
				dprintk(30, "EVENT_OK1/2: Pos. response received\n");

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
				dprintk(30, "EVENT_ANSWER_GETTIME: Pos. response received\n");
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
				dprintk(30, "EVENT_ANSWER_WAKEUP_REASON: Pos. response received\n");
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
				dprintk(30, "EVENT_ANSWER_VERSION: Pos. response received\n");
				errorOccured = 0;
				ack_sem_up();
				RCVBufferEnd = (RCVBufferEnd + cGetVersionSize) % BUFFERSIZE;
				break;
			}
			default: // Ignore Response
			{
				dprintk(1, "Invalid/unknown Response %02x\n", expectEventData);
				dprintk(30, "Buffer start=%d, Buffer end=%d\n",  RCVBufferStart,  RCVBufferEnd);
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
			printk("[micom] %s: FP: RCV buffer overflow!!!\n", __func__);
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
			printk("wait_event_interruptible failed\n");
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

#if 0
//----------------------------------------------

/*----- RTC driver -----*/
static int micom_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	u32 uTime = 0;

	dprintk(5, "%s >\n", __func__);
	uTime = YWPANEL_FP_GetTime();
	rtc_time_to_tm(uTime, tm);
	dprintk(5, "%s < %d\n", __func__, uTime);
	return 0;
}

static int tm2time(struct rtc_time *tm)
{
	return mktime(tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
}

static int micom_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int res = 0;

	u32 uTime = tm2time(tm);
	dprintk(5, "%s > uTime %d\n", __func__, uTime);
//	res = YWPANEL_FP_SetTime(uTime);
//	YWPANEL_FP_ControlTimer(true);
	dprintk(5, "%s < res: %d\n", __func__, res);
	return res;
}
static int micom_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	u32 a_time = 0;

	dprintk(5, "%s >\n", __func__);
//	a_time = YWPANEL_FP_GetPowerOnTime();
	if (al->enabled)
	{
		rtc_time_to_tm(a_time, &al->time);
	}
	dprintk(5, "%s < Enabled: %d RTC alarm time: %d time: %d\n", __func__, al->enabled, (int)&a_time, a_time);
	return 0;
}

static int micom_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	u32 a_time = 0;
	dprintk(5, "%s >\n", __func__);
	if (al->enabled)
	{
		a_time = tm2time(&al->time);
	}
//	YWPANEL_FP_SetPowerOnTime(a_time);
	dprintk(5, "%s < Enabled: %d time: %d\n", __func__, al->enabled, a_time);
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

	dprintk(5, "%s >\n", __func__);

	//Disable all ASC 2 interrupts
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
	//Enable the FIFO
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
		printk("FP: Cannot get irq\n");
	}
	msleep(1000);
	micom_init_func();

	if (register_chrdev(VFD_MAJOR, "VFD", &vfd_fops))
	{
		printk("unable to get major %d for VFD/MICOM\n", VFD_MAJOR);
	}

#if 0
	i = platform_driver_register(&micom_rtc_driver);
	if (i)
	{
		dprintk(5, "%s platform_driver_register failed: %d\n", __func__, i);
	}
	else
	{
		rtc_pdev = platform_device_register_simple(RTC_NAME, -1, NULL, 0);
	}

	if (IS_ERR(rtc_pdev))
	{
		dprintk(5, "%s platform_device_register_simple failed: %ld\n", __func__, PTR_ERR(rtc_pdev));
	}
#endif

	create_proc_fp();
	dprintk(10, "%s <\n", __func__);
	return 0;
}

static void __exit micom_cleanup_module(void)
{
	printk("MICOM frontcontroller module unloading\n");

//	platform_driver_unregister(&micom_rtc_driver);
//	platform_set_drvdata(rtc_pdev, NULL);
//	platform_device_unregister(rtc_pdev);
	remove_proc_fp();

	unregister_chrdev(VFD_MAJOR, "VFD");

	free_irq(InterruptLine, NULL);
}

//----------------------------------------------

module_init(micom_init_module);
module_exit(micom_cleanup_module);

MODULE_DESCRIPTION("MICOM frontcontroller module");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

