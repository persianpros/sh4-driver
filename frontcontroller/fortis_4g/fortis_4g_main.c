/*
 * fortis_4g_main.c
 *
 * Frontpanel driver for Fortis 4th generation receivers
 * FOREVER_NANOSMART, FOREVER_9898HD(?), DP7001 , FOREVER_2424HD, GPV8000, EP8000 & EPP8000.
 *
 * Based on code written by:
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add nuvoton irq handling
 *
 * (c) 2014 Audioniek: rewritten for Fortis 4G
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
 * Devices:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *  - /dev/rc  (reading of key events)
 *
 *
 * Justification for this (separate) Fortis front panel module:
 * By tradition, the first 3 generations of Fortis HD receivers used the
 * nuvoton module for their front panel support. This module handles the front
 * panel display, the front panel keyboard, clock for the timer(s) and the
 * remote control.
 *
 * The 4th generation of Fortis receivers no longer have a separate front-
 * processor. The display and front panel keyboard are handled by non-
 * intelligent ICs, the remote control is handled by lirc and the timers by
 * the RTC in the SoC.
 * Deep standby mode used to be power down everything except the front
 * processor, on the 4G models the main board is put in "hibernation".
 * The hardware architecture is quite different, so that is reason one.
 *
 * The second reason is that the nuvoton code has become quite cluttered
 * because of the number and variants of receivers supported. Adding these
 * new receivers having a different hardware approach to the nuvoton code
 * would probably clutter it up even more.
 *
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 20141108 Audioniek       Start of work, based on nuvoton_main.c.
 * 20141108 Audioniek       All code for older kernel versions removed.
 * 20150205 Audioniek       FOREVER_2424HD, GPV8000 and EP8000 models added.
 *
 ****************************************************************************************/
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/termbits.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/stm/pio.h>
//#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/poll.h>
#include <linux/rtc.h>
#include <linux/unistd.h>
#include <asm/ioctl.h>
#include <linux/ioctl.h>
#include <linux/platform_device.h>

//possibly missing:
#include <linux/input.h>
#include <linux/workqueue.h>
/* for rtc / reboot_notifier hooks */
//#include <linux/notifier.h>
//#include <linux/reboot.h>
//#include <linux/rtc.h>
//#include <linux/unistd.h>
//#include <asm/ioctl.h>
//#include <linux/ioctl.h>
//#include <linux/platform_device.h>

#include "fortis_4g.h"
#include "fortis_4g_utf.h"
#if defined(FOREVER_NANOSMART) || defined(DP7001) || defined(FOREVER_2424HD)
#include <linux/i2c.h>
#include "fortis_4g_i2c.h"
#elif defined(FOREVER_9898HD) || defined(DP7001) || defined(GPV8000) || defined(EP8000) || defined(EPP8000)
#include "fortis_4g_pio.h"
#endif

short paramDebug = 10;
int waitTime = 1000;
static char rtc_device[] = "rtc0";
struct semaphore write_sem;
static char ioctl_data[5];
static int open_count = 0;

/* Display thread */
static struct semaphore display_thread_sem;
static struct task_struct *display_task = 0;
static struct vfd_ioctl_data last_display_data;

#define DISPLAY_THREAD_RUNNING 0
#define DISPLAY_THREAD_STOPPED 1
#define DISPLAY_THREAD_INIT    2
static int display_thread_status = DISPLAY_THREAD_STOPPED;

/****************************************************************************************/

/* SoC RTC routines */
int leapyear(int year)
{
	return (!(year % 400) || ((year % 100) && !(year % 4))) & 0x01;
}  

int rtcdate2mjd(struct rtc_time *rtc)
{
	int mjd = 15020; // 1-1-1900, 00:00) why 2 off?
	int i;
	static int doy[] = {0,31,59,90,120,151,181,212,243,273,304,334};

	mjd += doy[rtc->tm_mon] + rtc->tm_mday - 1;

	if (leapyear(rtc->tm_year + 1900) && (rtc->tm_mon > 1))
	{
		mjd++;
	}

	for (i = 0; i < rtc->tm_year; i++)
	{
		mjd += 365 + leapyear(i + 1900);
	}
	return mjd;
}

struct rtc_time mjd2rtcdate(int mjd)
{
	struct rtc_time yymd;
	static int day_year[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
	int leap;

	mjd -= 40587; // linux epoch 1-1-1970 midnight
	yymd.tm_year = 70;
	yymd.tm_mon = 1;
	yymd.tm_mday = 1;

//	dprintk(10, "mjd (minus linux epoch) is %5d\n", mjd);
	while (mjd > 0)
	{
		leap = leapyear(yymd.tm_year + 1900);
		if (mjd  > (364 + leap))
		{
			mjd -= 365 + leap;
			yymd.tm_year++;
		}
		else
		{
//			dprintk(10, "days left is %d\n", mjd);
			if ((mjd >= day_year[yymd.tm_mon]))
			{
				mjd -= day_year[yymd.tm_mon];
				yymd.tm_mon++;

				if  ((yymd.tm_mon == 2) && leap)
				{
					mjd--;
				}
			}
			else
			{
				yymd.tm_mday += mjd;
				mjd = -1; //stop process
			}
		}
	}
	yymd.tm_mon--;
	return yymd;
}

int fortis_4gGetTime(void)
{
	int ret;
	struct rtc_time tm;
	struct rtc_device *rtc;
	int mjd;

	dprintk(100, "%s >\n", __func__);

	memset(ioctl_data, 0, 5);

	rtc = rtc_class_open(rtc_device);

	if (rtc == NULL)
	{
		dprintk(1, "Error: opening RTC device %s failed\n", rtc_device);
		return -ENODEV;
	}

	ret = rtc_read_time(rtc, &tm);

	if (ret)
	{
		dprintk(1, "Error: unable to read the RTC time\n");
		return -ENODEV;
	}

	ret = rtc_valid_tm(&tm);

	if (ret)
	{
		dprintk(1, "Error: RTC time invalid\n");
		return -EINVAL;
	}

	rtc_class_close(rtc);

	/* transfer date to caller in MJD */
	mjd = rtcdate2mjd(&tm);

	ioctl_data[0] = mjd >> 8;
	ioctl_data[1] = mjd & 0xff;
	ioctl_data[2] = tm.tm_hour;
	ioctl_data[3] = tm.tm_min;
	ioctl_data[4] = tm.tm_sec;

	dprintk(10, "Current RTC time/date is %02d:%02d:%02d %02d-%02d-%4d (MJD = %05d) (UTC)\n",
		tm.tm_hour, tm.tm_min, tm.tm_sec, tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900, mjd);
	dprintk(100, "%s <\n", __func__);
	return ret;
}

int fortis_4gSetTime(char *mjd_time)
{
	int ret;
	struct rtc_time tm;
	struct rtc_device *rtc;
	int mjd;

	/* convert from MJD to ymdhms */
	mjd = ((mjd_time[0] & 0xff) << 8) + (mjd_time[1] & 0xff);
	if (mjd < 40587)
	{
		memset(mjd_time, 0, 5);
		dprintk(1, "Error: time to set is before Linux epoch\n");
		return -EINVAL;
	}
	tm = mjd2rtcdate(mjd);
	tm.tm_hour = mjd_time[2];
	tm.tm_min = mjd_time[3];
	tm.tm_sec = mjd_time[4];

	dprintk(10, "Setting RTC time/date to %02d:%02d:%02d %02d-%02d-%4d (UTC)\n",
		tm.tm_hour, tm.tm_min, tm.tm_sec, tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900);

	rtc = rtc_class_open(rtc_device);

	if (rtc == NULL)
	{
		memset(mjd_time, 0, 5);
		dprintk(1, "Error: opening RTC device %s failed\n", rtc_device);
		return -ENODEV;
	}

	ret = rtc_set_time(rtc, &tm);

	rtc_class_close(rtc);

	if (ret)
	{
		dprintk(1, "Error: unable to set the RTC\n");
		return -ENODEV;
	}
	return ret;
}

int fortis_4gGetWakeupTime(void)
{
	int ret;
	struct rtc_wkalrm alarm;
	struct rtc_device *rtc;
	int mjd;
//	dprintk(10, "%s >\n", __func__);

	rtc = rtc_class_open(rtc_device);

	if (rtc == NULL)
	{
		dprintk(1, "Error: opening RTC device %s failed\n", rtc_device);
		return -ENODEV;
	}

	ret = rtc_read_alarm(rtc, &alarm);

	if (ret)
	{
		dprintk(1, "Error: unable to read the RTC alarm\n");
		return -ENODEV;
	}

	ret = rtc_valid_tm(&alarm.time);

	if (ret)
	{
		dprintk(1, "Error: RTC alarm time invalid\n");
		return -EINVAL;
	}

	/* transfer alarm date to caller in MJD */
	mjd = rtcdate2mjd(&alarm.time);

	ioctl_data[0] = mjd >> 8;
	ioctl_data[1] = mjd & 0xff;
	ioctl_data[2] = alarm.time.tm_hour;
	ioctl_data[3] = alarm.time.tm_min;
	ioctl_data[4] = alarm.time.tm_sec;

	dprintk(10, "Current RTC alarm time/date is %02d:%02d:%02d %02d-%02d-%4d (MJD = %05d) (UTC) Enabled = %1d\n",
		alarm.time.tm_hour, alarm.time.tm_min, alarm.time.tm_sec, alarm.time.tm_mday, alarm.time.tm_mon + 1, alarm.time.tm_year + 1900, mjd, alarm.enabled);

	return ret;
}

static int fortis_4g_rtc_set_alarm(char *alarm_time, char enabled)
{
	int ret;
	struct rtc_wkalrm alarm;
	struct rtc_device *rtc;
	int mjd;

	dprintk(100, "%s >\n", __func__);

	/* convert from MJD to ymdhms */
	mjd = ((alarm_time[0] & 0xff) << 8) + (alarm_time[1] & 0xff);
	if (mjd < 40587)
	{
		memset(alarm_time, 0, 5);
		dprintk(1, "Error: alarm time to set is before Linux epoch\n");
		return -EINVAL;
	}

	alarm.time = mjd2rtcdate(mjd);
	alarm.time.tm_hour = alarm_time[2];
	alarm.time.tm_min = alarm_time[3];
	alarm.time.tm_sec = alarm_time[4];
	alarm.enabled = enabled;

	dprintk(10, "Setting RTC alarm time/date to %02d:%02d:%02d %02d-%02d-%4d (MJD = %5d) (UTC) Enable = %d\n",
		alarm.time.tm_hour, alarm.time.tm_min, alarm.time.tm_sec, alarm.time.tm_mday, alarm.time.tm_mon + 1, alarm.time.tm_year + 1900, mjd, alarm.enabled);

	rtc = rtc_class_open(rtc_device);

	if (rtc == NULL)
	{
		memset(alarm_time, 0, 5);
		dprintk(1, "Error: opening RTC device %s failed\n", rtc_device);
		return -ENODEV;
	}

	ret = rtc_set_alarm(rtc, &alarm);

	rtc_class_close(rtc);

	if (ret)
	{
		dprintk(1, "Error: unable to set the RTC alarm time\n");
		return -EINVAL;
	}
	return ret;
}

int fortis_4gSetStandby(char *time)
{
//	char     buffer[8];
//	char     power_off[] = {SOP, cCommandPowerOffReplay, 0x01, EOP};
	int      ret = -1;

	dprintk(100, "%s >\n", __func__);

	/* set wakeup time */
	ret = fortis_4g_rtc_set_alarm(time, 1);

	/* now power off */
//	ret |= ?;

	dprintk(100, "%s <\n", __func__);
	return ret;
}
/* End RTC routines */

/* Display thread (taken from aotom driver, thank you author(s)) */
static int clear_display(void)
{
	unsigned char bBuf[DISP_SIZE];
	int res = -1;

	memset(bBuf, ' ', DISP_SIZE);
	res = fortis_4gWriteString(bBuf, DISP_SIZE);
	return res;
}

int utf8charlen(unsigned char c)
{
	if (!c)
	{
		return 0;
	}
	if (!(c >> 7))		// 0xxxxxxx
	{
		return 1;
	}
	if (c >> 5 == 6)	// 110xxxxx 10xxxxxx
	{
		return 2;
	}
	if (c >> 4 == 14)	// 1110xxxx 10xxxxxx 10xxxxxx
	{
		return 3;
	}
	if (c >> 3 == 30)	// 11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
	{
		return 4;
	}
	return 0;
}

int utf8strlen(char *s, int len)
{
	int i = 0, ulen = 0;

	while (i < len)
	{
		int trailing = utf8charlen((unsigned char)s[i]);
		if (!trailing)
		{
			return ulen;
		}
		trailing--, i++;
		if (trailing && (i >= len))
		{
			return ulen;
		}

		while (trailing)
		{
			if (i >= len || (unsigned char)s[i] >> 6 != 2)
			{
				return ulen;
			}
			trailing--;
			i++;
		}
		ulen++;
	}
	return ulen; // can be UTF8 (or pure ASCII, at least no non-UTF-8 8bit characters)
}

static int display_thread(void *arg)
{
	struct vfd_ioctl_data *display_data = (struct vfd_ioctl_data *)arg;
	unsigned char bBuf[65]; //display buffer
	unsigned char sBuf[64 + DISP_SIZE + 1]; //scroll buffer
	int i, j;
	int len;
	int res = -1;
	int utf8len;

	len = display_data->length;
	memcpy(bBuf, display_data->data, len);
	bBuf[len] = 0;

	utf8len = utf8strlen(bBuf, len);

	display_thread_status = DISPLAY_THREAD_RUNNING;

	if (utf8len > DISP_SIZE) //scroll
	{ //TODO: insert UTF8
		for (i = 0; i <= utf8len; i++)
		{
			if (kthread_should_stop())
			{
				display_thread_status = DISPLAY_THREAD_STOPPED;
				return 0;
			}
			memset(sBuf, ' ', 64 + DISP_SIZE);
			memcpy(sBuf, bBuf + i, len - i);
//			sBuf[len + DISP_SIZE] = 0;
//			dprintk(10, "%s Scroll display #%d [%s]\n", __func__, i, sBuf);
			res = fortis_4gWriteString(sBuf, DISP_SIZE);
			// sleep 300 ms
			for (j = 0; j < 12; j++)
			{
				if (kthread_should_stop())
				{
					display_thread_status = DISPLAY_THREAD_STOPPED;
					return res;
				}
				msleep(25);
			}
		}
		utf8len = DISP_SIZE; //set character count for final display
	}
	if (utf8len > 0)
	{
		res = fortis_4gWriteString(bBuf, utf8len);
	}
	else
	{
		res = clear_display();
	}
	display_thread_status = DISPLAY_THREAD_STOPPED;
	return res;
}

static int run_display_thread(struct vfd_ioctl_data *display_data)
{
	if (down_interruptible(&display_thread_sem))
	{
		return -ERESTARTSYS;
	}

	/* return if there is already a draw task running for the same text */
	if ((display_thread_status != DISPLAY_THREAD_STOPPED)
	&& display_task
	&& (last_display_data.length == display_data->length)
	&& !memcmp(&last_display_data.data, display_data->data, display_data->length))
	{
		up(&display_thread_sem);
		return 0;
	}

	display_data->data[display_data->length] = 0;
	memcpy(&last_display_data, display_data, sizeof(struct vfd_ioctl_data));

	/* new text, stop existing thread, if any */
	if ((display_thread_status != DISPLAY_THREAD_STOPPED) && display_task)
	{
		kthread_stop(display_task);
		while ((display_thread_status != DISPLAY_THREAD_STOPPED))
		{
			msleep(1);
		}
	}

	display_thread_status = DISPLAY_THREAD_INIT;
	display_task = kthread_run(display_thread, display_data, "display thread");

	/* Wait until thread has copied the argument */
	while (display_thread_status == DISPLAY_THREAD_INIT)
	{
		msleep(1);
	}

	up(&display_thread_sem);
	return 0;
}
/* End display thread */

int fortis_4gInit_func(void)
{
//	char standby_disable[] = {SOP, cCommandPowerOffReplay, 0x02, EOP};
//	char init1[] = {SOP, cCommandSetBootOn, EOP};
//	char init2[] = {SOP, cCommandSetTimeFormat, 0x81, EOP};       //set 24h clock format ?
//	char init3[] = {SOP, cCommandSetWakeupTime, 0xff, 0xff, EOP}; /* delete/invalidate wakeup time ? */
#if defined(FOREVER_9898HD) || defined(DP7001) || defined(FOREVER_2424HD) || defined(GPV8000) || defined(EP8000) || defined(EPP8000)
	//logo brightness on, other LEDs off
	//done already by I/O init 
#elif defined(FOREVER_NANOSMART)
	//all LEDs off (=green on)
	//done already by I/O init 
#endif
	int  vLoop;
	int  res = -1;

	dprintk(100, "%s >\n", __func__);

	sema_init(&write_sem, 1);
	/* Initialize the display thread */
	sema_init(&display_thread_sem, 1);
#if defined(FOREVER_NANOSMART)
	printk("Fortis FOREVER_NANOSMART LED");
#elif defined(FOREVER_9898HD)
	printk("Fortis FOREVER_9898HD VFD");
#elif defined(DP7001)
	printk("Fortis DP7001 LED");
#elif defined(FOREVER_2424HD)
	printk("Fortis FOREVER_2424HD LED");
#elif defined(GPV8000)
	printk("Fortis GPV8000 VFD");
#elif defined(EP8000)
	printk("Fortis EP8000 VFD");
#elif defined(EPP8000)
	printk("Fortis EPP8000 VFD");
#else
	printk("Fortis 4G");
#endif
	printk(" front panel module initializing\n");

	/* must be called before standby_disable */
//	res = fortis_4gWriteCommand(init1, sizeof(init1), 0);  //SetBootOn

	/* setup: frontpanel should not power down the receiver if standby is selected */
//	res = fortis_4gWriteCommand(standby_disable, sizeof(standby_disable), 0);

//	res |= fortis_4gWriteCommand(init2, sizeof(init2), 0);  //SetTimeFormat
//	res |= fortis_4gWriteCommand(init3, sizeof(init3), 0);  //Clear wakeuptime

	res |= fortis_4gSetBrightness(0);
/* The following models can control display brightness */
#if defined(FOREVER_NANOSMART) || defined(FOREVER_9898HD) || defined(DP7001) || defined(FOREVER_2424HD) || defined(GPV8000) || defined(EP8000) || defined(EPP8000)
	for (vLoop = 0; vLoop <= MAX_BRIGHT; vLoop++)
	{
		res |= fortis_4gSetBrightness(vLoop);
		msleep(200);
	}
#endif
#if defined(FOREVER_NANOSMART) || defined(DP7001) || defined(FOREVER_2424HD)
	res |= fortis_4gWriteString("----", 4);
#elif defined(FOREVER_9898HD) || defined(GPV8000) || defined(EP8000) || defined(EPP8000)
	res |= fortis_4gWriteString("T.Ducktales", strlen("T.Ducktales"));
#endif
	dprintk(100, "%s <\n", __func__);
	return 0;
}

/* Code for writing to /dev/vfd */
static ssize_t FORTIS4Gdev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	int i;
	int res = 0;
//	int llen;
	char *kernel_buf;
//	unsigned char sBuf[64 + DISP_SIZE];
	struct vfd_ioctl_data display_data;

	dprintk(100, "%s > (len %d, offs %d)\n", __func__, len, (int) *off);

	if ((len == 0) || (DISP_SIZE == 0))
	{
		return len;
	}

	kernel_buf = kmalloc(len + 1, GFP_KERNEL);

	if (kernel_buf == NULL)
	{
		printk("[fortis_4g] %s returns no memory <\n", __func__);
		return -ENOMEM;
	}

	copy_from_user(kernel_buf, buff, len);

	display_data.length = len;

#if defined(DP7001)
	for (i = 2; i < 5; i++)
	{
		if (kernel_buf[i] == '.')
		{
			display_data.length--; // correct scroll when 2nd, 3rd and/or 4th char is a period
		}
	}
	if (kernel_buf[2] == '.' && kernel_buf[3] == ':')
	{
		display_data.length--; // correct scroll when 2nd is a period and 3rd char is a colon
	}
#endif
#if defined(FOREVER_NANOSMART) || defined(DP7001)
	if (kernel_buf[2] == ':')
	{
		display_data.length--; // correct scroll when 2nd char is a colon
	}
#endif

	if (display_data.length >= 64) //do not display more than 64 characters
	{
		display_data.length = 64;
	}

	if (kernel_buf[len - 1] == '\n')
	{
		display_data.length--;
	}
#if 0
	if (display_data.length > DISP_SIZE)
	{
		for (i = 0; i <= display_data.length; i++)
		{
			memset(sBuf, ' ', 64 + DISP_SIZE); //fill buffer with spaces
			memcpy(sBuf, kernel_buf + i, display_data.length - i);
//			sBuf[display_data.length + DISP_SIZE] = 0;
//			dprintk(10, "%s Scroll display #%d [%s]\n", __func__, i, sBuf);
			res = fortis_4gWriteString(sBuf, DISP_SIZE);
			msleep(300);
		}
		display_data.length = DISP_SIZE; //set character count for final display
	}

	res = fortis_4gWriteString(kernel_buf, display_data.length); //final scroll or non-scroll display

	kfree(kernel_buf);

	up(&write_sem);
#else
	memcpy(display_data.data, kernel_buf, display_data.length);
	res = run_display_thread(&display_data);

	kfree(kernel_buf);
#endif
	dprintk(70, "%s < res %d len %d\n", __func__, res, len);

	if (res < 0)
	{
		return res;
	}
	return len;
}

/* Code for reading from /dev/vfd */
#if 0
static ssize_t FORTIS4Gdev_read(struct file *filp, char __user *buff, size_t len, loff_t *off)
{
//	int minor, vLoop;
//	int vLoop;

	dprintk(100, "%s > (len %d, offs %d)\n", __func__, len, (int) *off);

	if (len == 0)
	{
		return len;
	}

//	minor = -1;
//	for (vLoop = 0; vLoop < LASTMINOR; vLoop++)
//	{
	if (FrontPanelOpen[0].fp == filp)
	{
		dprintk(10, "minor = %d\n", 0);
//		minor = 0;
	}
	else
//	}

//	if (minor == -1)
	{
		printk("[fortis_4g] Error: Bad Minor\n");
		return -EUSERS;
	}

	/* Copy the current display string to the user */
	if (down_interruptible(&FrontPanelOpen[0].sem))
	{
		printk("[fortis_4g] %s < return ERESTARTSYS\n", __func__);
		return -ERESTARTSYS;
	}

	if (FrontPanelOpen[0].read == lastdata.length)
	{
		FrontPanelOpen[0].read = 0;
		up(&FrontPanelOpen[0].sem);
		printk("[fortis_4g] %s < return 0\n", __func__);
		return 0;
	}

	if (len > lastdata.length)
	{
		len = lastdata.length;
	}

	/* fixme: needs revision because of utf8! */
	if (len > 16) //TODO: not 64?
	{
		len = 16;
	}

	FrontPanelOpen[0].read = len;
	copy_to_user(buff, lastdata.data, len);
	up(&FrontPanelOpen[0].sem);
	dprintk(100, "%s < (len %d)\n", __func__, len);
	return len;
}
#endif

int FORTIS4Gdev_open(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(100, "%s >\n", __func__);

	minor = MINOR(inode->i_rdev);

	dprintk(70, "Open minor %d\n", minor);

	if (minor != FRONTPANEL_MINOR_VFD)
	{
		return -ENOTSUPP;
	}
	open_count++;

	dprintk(100, "%s <\n", __func__);
	return 0;
}

int FORTIS4Gdev_close(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(100, "%s >\n", __func__);

	minor = MINOR(inode->i_rdev);

	dprintk(50, "Close minor %d\n", minor);

	if (open_count > 0)
	{
		open_count--;
	}
	dprintk(100, "%s <\n", __func__);
	return 0;
}

static int FORTIS4Gdev_ioctl(struct inode *Inode, struct file *File, unsigned int cmd, unsigned long arg)
{
	static int mode = 0;
	struct fortis_4g_ioctl_data *fortis_4g = (struct fortis_4g_ioctl_data *)arg;
//	struct vfd_ioctl_data display_data = (struct vfd_ioctl_data *)arg;
	struct vfd_ioctl_data disp_data;
	int res = 0;
#if defined(FOREVER_9898HD) || defined(GPV8000) || defined(EP8000) || defined(EPP8000)
	int i, icon_nr = 0, on = 0;
#endif

	dprintk(100, "%s > 0x%.8x\n", __func__, cmd);

	if (down_interruptible(&write_sem))
	{
		return -ERESTARTSYS;
	}

	switch (cmd)
	{
		case VFDSETMODE:
		{
			mode = fortis_4g->u.mode.compat;
			break;
		}
		case VFDSETLED:
		case VFDPWRLED:
		{
			if (fortis_4g->u.led.level < 0)
			{
				fortis_4g->u.led.level = 0;
			}
			if (fortis_4g->u.led.level > MAX_LED_BRIGHT)
			{
				fortis_4g->u.led.level = MAX_LED_BRIGHT;
			}
			res = fortis_4gSetLED(fortis_4g->u.led.led_nr, fortis_4g->u.led.level);
			break;
		}
		case VFDBRIGHTNESS:
		{
#if defined(FOREVER_NANOSMART) || defined(FOREVER_9898HD) || defined(DP7001) || defined(FOREVER_2424HD) || defined(GPV8000) || defined(EP8000) || defined(EPP8000)
			if (mode == 0)
			{
				res = fortis_4gSetBrightness(disp_data.start_address);
			}
			else
			{
				if (fortis_4g->u.brightness.level < 0)
				{
					fortis_4g->u.brightness.level = 0;
				}
				else if (fortis_4g->u.brightness.level > MAX_BRIGHT)
				{
					fortis_4g->u.brightness.level = MAX_BRIGHT;
				}
				res = fortis_4gSetBrightness(fortis_4g->u.brightness.level);
			}
#endif //receivers that cannot control display brightness
			mode = 0;
			break;
		}
		case VFDDRIVERINIT:
		{
			res = fortis_4gInit_func();
			mode = 0;
			break;
		}
		case VFDICONDISPLAYONOFF:
		{
#if defined(FOREVER_9898HD) || defined(GPV8000) || defined(EP8000) || defined(EPP8000)
			if (mode == 0) //vfd mode
			{
				icon_nr = disp_data.data[0];
				on = disp_data.data[4];
				dprintk(5, "%s Set icon %d to %d (mode 0)\n", __func__, icon_nr, on);

				switch (icon_nr)
				{
					case 30:    //record
					{
						icon_nr = ICON_REC;
						break;
					}
					default:
					{
						break;
					}
				}
				if (on != 0)
				{
					on = 1;
				}
				if (icon_nr != ICON_MAX)
				{
					res = fortis_4gSetIcon(icon_nr, on);
				}
				else
				{
					for (i = ICON_MIN + 1; i < ICON_MAX; i++)
					{
						res = fortis_4gSetIcon(i, on);
					}
				}
			}
			else
			{
				//compatible mode
				dprintk(5, "%s Set icon %d to %d (mode 1)\n", __func__, fortis_4g->u.icon.icon_nr, fortis_4g->u.icon.on);
				if (fortis_4g->u.icon.icon_nr != ICON_MAX)
				{
					res = fortis_4gSetIcon(fortis_4g->u.icon.icon_nr, fortis_4g->u.icon.on);
				}
				else
				{
					for (i = ICON_MIN + 1; i <  ICON_MAX; i++)
					{
						res = fortis_4gSetIcon(i, fortis_4g->u.icon.on);
					}
				}
			}
#else
			printk("[fortis_4g] Set icon not supported on this Fortis 4G receiver.\n");
#endif
			mode = 0;
			res = 0;
			break;
		}
		case VFDSTANDBY:
		{
			dprintk(5, "Set standby mode, wake up time: (MJD = %d) - %02d:%02d:%02d (UTC)\n", (fortis_4g->u.standby.time[0] & 0xFF) * 256 + (fortis_4g->u.standby.time[1] & 0xFF), fortis_4g->u.standby.time[2], fortis_4g->u.standby.time[3], fortis_4g->u.standby.time[4]);
			res = fortis_4gSetStandby(fortis_4g->u.standby.time);
			break;
		}
		case VFDREBOOT:
		{
			dprintk(5, "Reboot, wake up time: (MJD = %d) - %02d:%02d:%02d (UTC)\n", (fortis_4g->u.standby.time[0] & 0xFF) * 256 + (fortis_4g->u.standby.time[1] & 0xFF), fortis_4g->u.standby.time[2], fortis_4g->u.standby.time[3], fortis_4g->u.standby.time[4]);
//			res = fortis_4gSetStandby(fortis_4g->u.standby.time);
			res = 0;
			break;
		}
		case VFDSETTIME:
		{
			if (fortis_4g->u.time.time != 0)
			{
				dprintk(5, "Set SoC RTC time to (MJD = %d) - %02d:%02d:%02d (UTC)\n", (fortis_4g->u.time.time[0] & 0xFF) * 256 + (fortis_4g->u.time.time[1] & 0xFF), fortis_4g->u.time.time[2], fortis_4g->u.time.time[3], fortis_4g->u.time.time[4]);
				res = fortis_4gSetTime(fortis_4g->u.time.time);
			}
			break;
		}
		case VFDGETTIME:
		{
			res = fortis_4gGetTime();
//			res = fortis_4g_rtc_read_time(*ioctl_data)
			dprintk(5, "Get Soc RTC time: (MJD = %d) - %02d:%02d:%02d (UTC)\n", (ioctl_data[0] & 0xFF) * 256 + (ioctl_data[1] & 0xFF), ioctl_data[2], ioctl_data[3], ioctl_data[4]);
			copy_to_user((void *)arg, &ioctl_data, 5);
			break;
		}
		case VFDSETWAKEUPTIME:
		{
			if (fortis_4g->u.time.time != 0)
			{
				dprintk(5, "Set SoC RTC alarm time to (MJD = %d) - %02d:%02d:%02d (UTC)\n", (fortis_4g->u.time.time[0] & 0xFF) * 256 + (fortis_4g->u.time.time[1] & 0xFF), fortis_4g->u.time.time[2], fortis_4g->u.time.time[3], fortis_4g->u.time.time[4]);
				res = fortis_4g_rtc_set_alarm(fortis_4g->u.time.time, 1);
			}
			break;
		}
		case VFDGETWAKEUPTIME:
		{
			res = fortis_4gGetWakeupTime();
			dprintk(5, "Get SoC RTC alarm time: (MJD = %d) - %02d:%02d:%02d (UTC)\n", (ioctl_data[0] & 0xFF) * 256 + (ioctl_data[1] & 0xFF), ioctl_data[2], ioctl_data[3], ioctl_data[4]);
			copy_to_user((void *)arg, &ioctl_data, 5);
			break;
		}
		case VFDGETWAKEUPMODE:
		{
			res = fortis_4gGetWakeUpMode();
			copy_to_user((void *)arg, &ioctl_data, 1);
			break;
		}
		case VFDDISPLAYCHARS:
		{
			if (mode == 0)
			{
				dprintk(5, "Write string (mode 0): %s (length = %d)\n", disp_data.data, disp_data.length);
#if 0
				res = fortis_4gWriteString(display_data.data, display_data.length);
#else
				if (copy_from_user(&disp_data, (void *) arg, sizeof(disp_data)))
				{
					return -EFAULT;
				}
				if (disp_data.length > sizeof(disp_data.data))
				{
					disp_data.length = sizeof(disp_data.data);
				}
				while ((disp_data.length > 0) && (disp_data.data[disp_data.length - 1 ] == '\n'))
				{
					disp_data.length--;
				}
				res = run_display_thread(&disp_data);
#endif
			}
			else
			{
				dprintk(5, "Write string (mode 1): not supported!\n");
			}
			mode = 0;
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			res = fortis_4gSetDisplayOnOff(fortis_4g->u.light.onoff);
			mode = 0;  //go back to vfd mode
			break;
		}
		case VFDSETDISPLAYTIME:
		{
			res = fortis_4gSetTimeDisplayOnOff(fortis_4g->u.timedisplay.onoff);
			mode = 0;  //go back to vfd mode
			break;
		}
		case VFDSETTIMEFORMAT:
		{
//			if (fortis_4g->u.timeformat.format != 0)
//			{
//				fortis_4g->u.timeformat.format = 1;
//			}
			dprintk(5, "Set time format to: %d\n", fortis_4g->u.timeformat.format);
			res = fortis_4gSetTimeFormat(fortis_4g->u.timeformat.format);
			mode = 0; //go back to vfd mode
			break;
		}
		case 0x5305:
		{
			mode = 0; //go back to vfd mode
			break;
		}
		case 0x5401:
		case VFDGETBLUEKEY:
		case VFDSETBLUEKEY:
		case VFDGETSTBYKEY:
		case VFDSETSTBYKEY:
		case VFDPOWEROFF:
		case VFDGETVERSION:
		case VFDGETSTARTUPSTATE:
		case VFDDISPLAYCLR:
		case VFDGETLOOPSTATE:
		{
			printk("[fortis_4g] Unsupported IOCTL 0x%x for this receiver.\n", cmd);
			mode = 0;
			break;
		}
		default:
		{
			printk("[fortis_4g] Unknown IOCTL 0x%x.\n", cmd);
			mode = 0;
			break;
		}
	}
	up(&write_sem);
	dprintk(100, "%s <\n", __func__);
	return res;
}

struct file_operations vfd_fops =
{
	.owner   = THIS_MODULE,
	.ioctl   = FORTIS4Gdev_ioctl,
	.write   = FORTIS4Gdev_write,
//	.read    = FORTIS4Gdev_read,
	.open    = FORTIS4Gdev_open,
	.release = FORTIS4Gdev_close
};

#if 0
/*----- Reboot notifier -----*/
static int fortis_4g_reboot_event(struct notifier_block *nb, unsigned long event, void *ptr)
{
	switch (event)
	{
		case SYS_POWER_OFF:
		{
			YWPANEL_FP_ShowString("Power off");
			break;
		}
		case SYS_HALT:
		{
			YWPANEL_FP_ShowString("Halt");
			break;
		}
		default:
		{
			VFD_clr();
			YWPANEL_FP_ShowString("Reboot");
			return NOTIFY_DONE;
		}
	}
	msleep(1000);
	clear_display();
	YWPANEL_FP_ControlTimer(true);
	YWPANEL_FP_SetCpuStatus(YWPANEL_CPUSTATE_STANDBY);
	return NOTIFY_DONE;
}

static struct notifier_block fortis_4g_reboot_block =
{
	.notifier_call = fortis_4g_reboot_event,
	.priority = INT_MAX
};
#endif

static int __init fortis4g_init_module(void)
{
//	int i = 0;
	int err;
//	struct rtc_time rtc_tm;

	dprintk(100, "%s <\n", __func__);
	msleep(waitTime);

	/* Initialize the I/O part */
	err = fortis_4gInit_core();
	if (err)
	{
		return err;
	}

	/* Initialize the IOCTL part */
	fortis_4gInit_func();

//	register_reboot_notifier(&fortis_4g_reboot_block);

	/* Initialize the RTC */
	fortis_4gGetTime();	

	if (register_chrdev(VFD_MAJOR, "VFD", &vfd_fops))
	{
		printk("[fortis_4g_fp] Unable to get major %d\n", VFD_MAJOR);
	}

	dprintk(100, "%s <\n", __func__);
	return 0;
}

//static int led_thread_active(void)
//{
//	int i;
//
//	for (i = 0; i < LASTLED; i++)
//	{
//		if (!led_state[i].status && led_state[i].led_task)	
//		{
//			return -1;
//		}
//	}
//	return 0;
//}

static void __exit fortis4g_cleanup_module(void)
{
//	int i;

//	unregister_reboot_notifier(fortis_4g_reboot_block);

	if ((display_thread_status != DISPLAY_THREAD_STOPPED) && display_task)
	{
		kthread_stop(display_task);
	}

//	for (i = 0; i < LASTLED; i++)
//	{
//		if (!led_state[i].status && led_state[i].led_task)
//		{
//			up(&led_state[i].led_sem);
//			kthread_stop(led_state[i].led_task);
//		}
//	}

//	while ((display_thread_status != DISPLAY_THREAD_STOPPED) && !led_thread_active())
//	{
//		msleep(1);
//	}

	fortis_4gExit_core(); //clean up the I/O part

	unregister_chrdev(VFD_MAJOR, "VFD");
	printk("Fortis 4G frontcontroller module unloading\n");
}

//----------------------------------------------

module_init(fortis4g_init_module);
module_exit(fortis4g_cleanup_module);

MODULE_DESCRIPTION("Fortis 4G frontcontroller module");
MODULE_AUTHOR("Dagobert, Schischu, Konfetti, Spider-Team, oSaoYa & Audioniek");
MODULE_LICENSE("GPL");

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

module_param(waitTime, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(waitTime, "Wait before init in ms (default=1000)");

