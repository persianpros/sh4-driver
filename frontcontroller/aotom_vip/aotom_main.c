/****************************************************************************
 *
 * aotom_main.c
 *
 * (c) 2010 Spider-Team
 * (c) 2011 oSaoYa
 * (c) 2012-2013 Stefan Seyfried
 * (c) 2012-2013 martii
 * (c) 2013-2020 Audioniek
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
 * Fulan/Edision Argus VIP1/2 front panel driver.
 *
 * Devices:
 *	- /dev/vfd (vfd ioctls and read/write function)
 *
 *
 ****************************************************************************
 *
 * Notes: This driver has been modified using the original code supplied in
 *        other repositories, and then enhanced/debugged using code from
 *        aotom_spark.
 *        As it is likely that all Edision Argus VIP1/2s have a VFD frontpanel
 *        all code referring to LED and DVFD panels has been conditioned
 *        out. It can be compiled by removing two comments in aotom_main.h
 *        if this is desired.
 *
 ****************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------
 * 201????? Spider Team?    Initial version.
 * 20200630 Audioniek       First working version.
 * 20200702 Audioniek       Front panel key feedback restored.
 * 20200702 Audioniek       Debug output unified, now uses paramDebug
 *                          throughout.
 * 20200703 Audioniek       VFDDISPLAYWRITEONOFF added.
 * 20200703 Audioniek       procfs added.
 * 20200703 Audioniek       Dummy RTC added, copied from aotom_spark.
 * 20200703 Audioniek       Spinner added, copied from aotom_spark.
 * 
 ****************************************************************************/

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/termbits.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
// RTC
#include <linux/rtc.h>
#include <linux/platform_device.h>

#include <linux/pm.h>

#include "aotom_main.h"

static const char Revision[] = "Revision: 0.9Audioniek";
short paramDebug = 0;
  //debug print level is zero as default (0=nothing, 1= errors, 10=some detail, 20=more detail, 100=open/close, 150=all)

#define DISPLAYWIDTH_MAX  8  // ! assumes VFD

#define INVALID_KEY      -1

#define NO_KEY_PRESS     -1
#define KEY_PRESS_DOWN    1
#define KEY_PRESS_UP      0

static char *gmt = "+3600";  // GMT offset is plus one hour (Europe) as default
int scroll_repeats = 1;  // 0 = no, number is times??
int scroll_delay = 20;  // wait time (x 10 ms) between scrolls
int initial_scroll_delay = 40;  // wait time (x 10 ms) to start scrolling
int final_scroll_delay = 50;  // wait time (x 10 ms) to start final display

typedef struct
{
	int minor;
	int open_count;
} tFrontPanelOpen;

#define FRONTPANEL_MINOR_VFD 0
#define FRONTPANEL_MINOR_RC  1
#define LASTMINOR            2

static tFrontPanelOpen FrontPanelOpen [LASTMINOR];

tThreadState led_state[LASTLED + 1];
tThreadState spinner_state;

#define BUFFERSIZE 256  // must be 2^n

struct receive_s
{
	int len;
	unsigned char buffer[BUFFERSIZE];
};

#define cMaxReceiveQueue 100
static wait_queue_head_t wq;

static struct receive_s receive[cMaxReceiveQueue];
static int receiveCount = 0;

static struct semaphore write_sem;
static struct semaphore receive_sem;
static struct semaphore draw_thread_sem;
static struct semaphore spinner_sem;

static struct task_struct *draw_task = 0;
static int draw_thread_status = THREAD_STATUS_STOPPED;

#define RTC_NAME "aotom-rtc"
static struct platform_device *rtc_pdev;

/****************************************************************************/

static int VFD_Show_Time(u8 hh, u8 mm, u8 ss)
{
	if ((hh > 23) || (mm > 59) || (ss > 59))
	{
		dprintk(1, "%s Invalid time!\n", __func__);
		return -1;
	}
	return YWPANEL_FP_SetTime((hh * 3600) + (mm * 60) + ss);
}

int aotomSetBrightness(int level)
{
	int res = 0;

	if (level < 0)
	{
		level = 0;
	}
	else if (level > 7)
	{
		level = 7;
	}
	dprintk(10, "%s Set brightness level 0x%02X\n", __func__, level);
	res = YWPANEL_FP_SetBrightness(level);

	return res;
}

int aotomSetIcon(int which, int on)
{
	int res = 0;

	dprintk(100, "%s > %d, %d\n", __func__, which, on);
	if (which < ICON_FIRST || which > ICON_LAST)
	{
		dprintk(1,"Icon number out of range %d (1- %d)\n", which, ICON_LAST);
		return -EINVAL;
	}
	which = (((which - 1) / 15) + 11) * 16 + ((which - 1) % 15) + 1;
	res = YWPANEL_FP_ShowIcon(which, on);

	dprintk(100, "%s <\n", __func__);
	return res;
}

void VFD_set_all_icons(int onoff)
{
	int i;

	for (i = ICON_FIRST; i <= ICON_LAST; i++)
	{
		aotomSetIcon(i, onoff);
	}
	if (onoff == LOG_OFF && spinner_state.state != LOG_OFF)
	{
		spinner_state.state = LOG_OFF;
		spinner_state.period = 0;
	}
}

void clear_display(void)
{
	YWPANEL_FP_ShowString("        ");
}

static void VFD_clr(void)
{
//	YWPANEL_FP_ShowTimeOff();  // switch clock off, does not work
	clear_display();
	VFD_set_all_icons(0);
}

static int draw_thread(void *arg)
{
	struct vfd_ioctl_data *data = (struct vfd_ioctl_data *)arg;
	char buf[sizeof(data->data) + 2 * DISPLAYWIDTH_MAX];
	int len = data->length;
	int off = 0;

	if (len > YWPANEL_width)
	{
		memset(buf, ' ', sizeof(buf));
		off = YWPANEL_width - 1;
		memcpy(buf + off, data->data, len);
		len += off;
		buf[len + YWPANEL_width] = 0;
	}
	else
	{
		memcpy(buf, data->data, len);
		buf[len] = 0;
	}
	draw_thread_status = THREAD_STATUS_RUNNING;

	if (len > YWPANEL_width)  // determine scroll
	{
		int pos;

		for (pos = 0; pos < len; pos++)
		{
			int i;

			if (kthread_should_stop())
			{
				draw_thread_status = THREAD_STATUS_STOPPED;
				return 0;
			}
			YWPANEL_FP_ShowString(buf + pos);
			// sleep scroll delay
			for (i = 0; i < 10; i++)
			{
				if (kthread_should_stop())
				{
					draw_thread_status = THREAD_STATUS_STOPPED;
					return 0;
				}
				msleep(scroll_delay);
			}
		}
	}
	clear_display();
	// final display
	if (len > 0)
	{
		YWPANEL_FP_ShowString(buf + off);
	}
	draw_thread_status = THREAD_STATUS_STOPPED;
	return 0;
}

static int spinner_thread(void *arg)
{
	int phase = 0;

	if (spinner_state.status == THREAD_STATUS_RUNNING)
	{
		return 0;
	}
	spinner_state.status = THREAD_STATUS_RUNNING;

	while (!kthread_should_stop())
	{
		if (!down_interruptible(&spinner_sem))
		{
			if (kthread_should_stop())
			{
				break;
			}

			while (!down_trylock(&spinner_sem));  // make sure semaphore is at 0

			aotomSetIcon(ICON_DISK_CIRCLE, LOG_ON);  // switch centre circle on
			while ((spinner_state.state) && !kthread_should_stop())
			{
				switch (phase)  // display a rotating disc in 5 phases
				{
					case 0:
					{
						aotomSetIcon(ICON_DISK_S1, LOG_ON);
						aotomSetIcon(ICON_DISK_S2, LOG_OFF);
						aotomSetIcon(ICON_DISK_S3, LOG_OFF);
						break;
					}
					case 1:
					{
						aotomSetIcon(ICON_DISK_S1, LOG_ON);
						aotomSetIcon(ICON_DISK_S2, LOG_ON);
						aotomSetIcon(ICON_DISK_S3, LOG_OFF);
						break;
					}
					case 2:
					{
						aotomSetIcon(ICON_DISK_S1, LOG_ON);
						aotomSetIcon(ICON_DISK_S2, LOG_ON);
						aotomSetIcon(ICON_DISK_S3, LOG_ON);
						break;
					}
					case 3:
					{
						aotomSetIcon(ICON_DISK_S1, LOG_OFF);
						aotomSetIcon(ICON_DISK_S2, LOG_ON);
						aotomSetIcon(ICON_DISK_S3, LOG_ON);
						break;
					}
					case 4:
					{
						aotomSetIcon(ICON_DISK_S1, LOG_OFF);
						aotomSetIcon(ICON_DISK_S2, LOG_OFF);
						aotomSetIcon(ICON_DISK_S3, LOG_ON);
						break;
					}
				}
				phase++;
				phase %= 5;
				msleep(spinner_state.period * 10);
			}
			aotomSetIcon(ICON_DISK_S3, LOG_OFF);
			aotomSetIcon(ICON_DISK_S2, LOG_OFF);
			aotomSetIcon(ICON_DISK_S1, LOG_OFF);
			aotomSetIcon(ICON_DISK_CIRCLE, LOG_OFF);
		}
	}
	spinner_state.status = THREAD_STATUS_STOPPED;
	spinner_state.thread_task = 0;
	return 0;
}

static struct vfd_ioctl_data last_draw_data;

int run_draw_thread(struct vfd_ioctl_data *draw_data)
{
	if (down_interruptible(&draw_thread_sem))
	{
		return -ERESTARTSYS;
	}

	// return if there is already a draw task running for the same text
	if (!draw_thread_status
	&&  draw_task
	&&  (last_draw_data.length == draw_data->length)
	&&  !memcmp(&last_draw_data.data, draw_data->data, draw_data->length))
	{
		up(&draw_thread_sem);
		return 0;
	}

	memcpy(&last_draw_data, draw_data, sizeof(struct vfd_ioctl_data));

	// stop existing thread, if any
	if (!draw_thread_status && draw_task)
	{
		kthread_stop(draw_task);
		while (!draw_thread_status)
		{
			msleep(1);
		}
		draw_task = 0;
	}
	if (draw_data->length < YWPANEL_width)
	{
		char buf[DISPLAYWIDTH_MAX + 1];
		memset(buf, 0, sizeof(buf));
		memset(buf, ' ', sizeof(buf) - 1);
		if (draw_data->length)
		{
			memcpy(buf, draw_data->data, draw_data->length);
		}
		YWPANEL_FP_ShowString(buf);
	}
	else
	{
		draw_thread_status = THREAD_STATUS_INIT;
		draw_task = kthread_run(draw_thread, draw_data, "draw_thread");

		// wait until thread has copied the argument
		while (draw_thread_status == THREAD_STATUS_INIT)
		{
			msleep(1);
		}
	}
	up(&draw_thread_sem);
	return 0;
}

int aotomWriteText(char *buf, size_t len)
{
	int res = 0;
	struct vfd_ioctl_data data;

	if (len > sizeof(data.data))
	{
		data.length = sizeof(data.data);
	}
	else
	{
		data.length = len;
	}

	while ((data.length > 0) && (buf[data.length - 1 ] == '\n'))
	{
		data.length--;
	}

	if (data.length > sizeof(data.data))
	{
		len = data.length = sizeof(data.data);
	}
	memcpy(data.data, buf, data.length);
	res = run_draw_thread(&data);

	return res;
}

int aotomSetTime(char *time)
{
	int res = 0;

	dprintk(100, "%s >\n", __func__);
	dprintk(20, "%s time: %02d:%02d:%02d\n", __func__, time[2], time[3], time[4]);

	res = VFD_Show_Time(time[2], time[3], time[4]);
	dprintk(100, "%s <\n", __func__);
	return res;
}

int vfd_init_func(void)
{
	dprintk(100, "%s >\n", __func__);
	printk("Edision VFD module initializing\n");
	return 0;
}

/*************************************************
 *
 * Code to handle /dev/vfd
 *
 */
static ssize_t AOTOMdev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char *kernel_buf;
	int res = 0;

	struct vfd_ioctl_data data;

	dprintk(100, "%s > (len %d, offs %d)\n", __func__, len, (int) *off);

	if (((tFrontPanelOpen *)(filp->private_data))->minor != FRONTPANEL_MINOR_VFD)
	{
		dprintk(1, "Error: Bad Minor\n");
		return -EOPNOTSUPP;
	}

	kernel_buf = kmalloc(len + 1, GFP_KERNEL);

	memset(kernel_buf, 0, len + 1);
	memset(&data, 0, sizeof(struct vfd_ioctl_data));

	if (kernel_buf == NULL)
	{
		dprintk(1, "%s returns no mem <\n", __func__);
		return -ENOMEM;
	}
	copy_from_user(kernel_buf, buff, len);

	if (len > sizeof(data.data))
	{
		data.length = sizeof(data.data);
	}
	else
	{
		data.length = len;
	}
	while ((data.length > 0) && (kernel_buf[data.length - 1 ] == '\n'))
	{
		data.length--;
	}
	if (data.length > sizeof(data.data))
	{
		len = data.length = sizeof(data.data);
	}
//	TODO: insert UTF8 handling
//	dprintk(10, "%s Text: %s, length: %d\n", __func__, kernel_buf, len); 	
	memcpy(data.data, kernel_buf, data.length);
// TODO: insert scrolling
	res = run_draw_thread(&data);

	kfree(kernel_buf);

	dprintk(100, "%s < res %d len %d\n", __func__, res, len);

	if (res < 0)
	{
		return res;
	}
	return len;
}

#if defined(FP_LEDS)
static void flashLED(int led, int ms)
{
	if (!led_state[led].thread_task || ms < 1)
	{
		return;
	}
	led_state[led].period = ms;
	up(&led_state_sem);
}
#endif

int show_spinner(int period)
{
	if (!spinner_state.thread_task || period < 1)
	{
		return 1;
	}
	spinner_state.period = period;
	up(&spinner_sem);
	return 0;
}

static ssize_t AOTOMdev_read(struct file *filp, char __user *buff, size_t len, loff_t *off)
{
	dprintk(100, "%s > (len %d, offs %d)\n", __func__, len, (int)*off);

	if (((tFrontPanelOpen *)(filp->private_data))->minor == FRONTPANEL_MINOR_RC)
	{
		while (receiveCount == 0)
		{
			if (wait_event_interruptible(wq, receiveCount > 0))
			{
				return -ERESTARTSYS;
			}
		}
//		flashLED(LED_GREEN, 100);  // TODO: Edision Argus VIP2 has no controllable LEDs

		/* 0. claim semaphore */
		if (down_interruptible(&receive_sem))
		{
			return -ERESTARTSYS;
		}
		/* 1. copy data to user */
		copy_to_user(buff, receive[0].buffer, receive[0].len);

		/* 2. copy all entries to start and decrease receiveCount */
		receiveCount--;
		memmove(&receive[0], &receive[1], (cMaxReceiveQueue - 1) * sizeof(struct receive_s));

		/* 3. free semaphore */
		up(&receive_sem);

		return receive[0].len;
	}
	return -EOPNOTSUPP;
}

static int AOTOMdev_open(struct inode *inode, struct file *filp)
{
	int minor;

	minor = MINOR(inode->i_rdev);

	if (minor == FRONTPANEL_MINOR_RC && FrontPanelOpen[minor].open_count)
	{
		dprintk(1, "%s EUSER\n", __func__);
		return -EUSERS;
	}
	FrontPanelOpen[minor].open_count++;
	filp->private_data = &FrontPanelOpen[minor];

	return 0;
}

static int AOTOMdev_close(struct inode *inode, struct file *filp)
{
	int minor;

	minor = MINOR(inode->i_rdev);

	if (FrontPanelOpen[minor].open_count > 0)
	{
		FrontPanelOpen[minor].open_count--;
	}
	return 0;
}

static struct aotom_ioctl_data aotom_data;
static struct vfd_ioctl_data vfd_data;

static int AOTOMdev_ioctl(struct inode *Inode, struct file *File, unsigned int cmd, unsigned long arg)
{
	int icon_nr = 0;
	static int mode = 0;
	int res = -EINVAL;
	dprintk(100, "%s > IOCTL= 0x%.8x\n", __func__, cmd);

	if (down_interruptible(&write_sem))
	{
		return -ERESTARTSYS;
	}
	switch (cmd)
	{
		case VFDSETMODE:  // These IOCTLs have input data
		case VFDSETLED:
		case VFDICONDISPLAYONOFF:
		case VFDDISPLAYWRITEONOFF:
		case VFDSETTIME:
		case VFDBRIGHTNESS:
		{
			if (copy_from_user(&aotom_data, (void *) arg, sizeof(aotom_data)))
			{
				return -EFAULT;
			}
		}
	}

	switch (cmd)  // Handle IOCTL
	{
		case VFDSETMODE:
		{
			mode = aotom_data.u.mode.compat;
			break;
		}
		case VFDSETLED:
		{
#if defined(FP_LEDS)
			if (aotom_data.u.led.led_nr > -1 && aotom_data.u.led.led_nr < LED_MAX)
			{
				switch (aotom_data.u.led.on)
				{
					case LOG_OFF:
					case LOG_ON:
					{
						res = YWPANEL_FP_SetLed(aotom_data.u.led.led_nr, aotom_data.u.led.on);
						led_state[aotom_data.u.led.led_nr].state = aotom_data.u.led.on;
						break;
					}
					default:  // toggle (for aotom_data.u.led.on * 10) ms
					{
						dprintk(10, "%s Blink LED %d for 10ms\n", __func__, led_nr);
						flashLED(aotom_data.u.led.led_nr, aotom_data.u.led.on * 10);
						res = 0;  //was missing, reporting blink as illegal value
					}
				}
			}
#endif
			break;
		}
		case VFDBRIGHTNESS:
		{
			res = aotomSetBrightness(aotom_data.u.brightness.level);
			break;
		}
		case VFDICONDISPLAYONOFF:
		{
#if defined(FP_LEDS)
			switch (aotom_data.u.icon.icon_nr)
			{
				case 0:
				{
					res = YWPANEL_FP_SetLed(LED_RED, aotom_data.u.icon.on);
					led_state[LED_RED].state = aotom_data.u.icon.on;
					break;
				}
				case 35:
				{
					res = YWPANEL_FP_SetLed(LED_GREEN, aotom_data.u.icon.on);
					led_state[LED_GREEN].state = aotom_data.u.icon.on;
					break;
				}
				default:
				{
					break;
				}
			}
#endif
			icon_nr = aotom_data.u.icon.icon_nr;
			//e2 icons work around
			if (icon_nr >= 256)
			{
				icon_nr = icon_nr / 256;
				switch (icon_nr)
				{
					case 17:
					{
						icon_nr = ICON_DOUBLESCREEN;
						break;
					}
					case 19:
					{
						icon_nr = ICON_CA;
						break;
					}
					case 21:
					{
						icon_nr = ICON_MP3;
						break;
					}
					case 23:
					{
						icon_nr = ICON_AC3;
						break;
					}
					case 26:
					{
						icon_nr = ICON_PLAY_LOG;
						break;
					}
					case 30:
					{
						icon_nr = ICON_REC1;
						break;
					}
					case 38:
					case 39:
					case 40:
					case 41:
					{
						break;  // CD segments & circle
					}
					case 47:
					{
						icon_nr = ICON_SPINNER;
						break;
					}
					default:
					{
							dprintk(20, "Tried to set unknown icon number %d.\n", icon_nr);
//							icon_nr = 29; //no additional symbols at the moment: show alert instead
							break;
					}
				}
			}
			if (aotom_data.u.icon.on != LOG_OFF && icon_nr != ICON_SPINNER)
			{
				aotom_data.u.icon.on = LOG_ON;
			}
// display icon
			switch (icon_nr)
			{
				case ICON_ALL:
				{
					VFD_set_all_icons(aotom_data.u.icon.on);
					res = 0;
					break;
				}
				case ICON_SPINNER:
				{
					spinner_state.state = aotom_data.u.icon.on;
					if (aotom_data.u.icon.on == 1)
					{
						res = show_spinner(40);  // start spinner thread, default speed
					}
					else 
					{
						res = show_spinner(aotom_data.u.icon.on * 5);  // start spinner thread, user speed
						
					}
					break;
				}
				case -1:
				{
					break;
				}
				default:
				{
					res = aotomSetIcon(icon_nr, aotom_data.u.icon.on);
				}
			}
			mode = 0;
			break;
		}
		case VFDSETPOWERONTIME:
		{
			u32 uTime = 0;

			get_user(uTime, (int *)arg);
			YWPANEL_FP_SetPowerOnTime(uTime);
			res = 0;
			break;
		}
		case VFDSTANDBY:
		{
			u32 uTime = 0;
			get_user(uTime, (int *) arg);

			YWPANEL_FP_SetPowerOnTime(uTime);
			clear_display();
			YWPANEL_FP_ControlTimer(true);
			YWPANEL_FP_SetCpuStatus(YWPANEL_CPUSTATE_STANDBY);
			res = 0;
			break;
		}
		case VFDSETTIME2:
		{
			u32 uTime = 0;
			res = get_user(uTime, (int *)arg);
			if (! res)
			{
				dprintk(5, "%s Set FP time to: %d\n", __func__, uTime);
				res = YWPANEL_FP_SetTime(uTime);
				YWPANEL_FP_ControlTimer(true);
			}
			break;
		}
		case VFDSETTIME:
		{
			dprintk(5, "%s Set FP time to: %d\n", __func__, (int)aotom_data.u.time.time);
			res = aotomSetTime(aotom_data.u.time.time);
			break;
		}
		case VFDGETTIME:
		{
			u32 uTime = 0;

			uTime = YWPANEL_FP_GetTime();
			dprintk(5, "%s FP time: %d\n", __func__, uTime);
			res = put_user(uTime, (int *)arg);
			break;
		}
		case VFDGETWAKEUPMODE:
		{
			break;
		}
		case VFDGETWAKEUPTIME:
		{
			u32 uTime = 0;
			uTime = YWPANEL_FP_GetPowerOnTime();
			dprintk(5, "%s Power on time: %d\n", __func__, uTime);
			res = put_user(uTime, (int *) arg);
			break;
		}
		case VFDSETDISPLAYTIME:
		{
			res = 0;  // do nothing (VFD has clock always on)
			break;
		}
		case VFDGETDISPLAYTIME:
		{
			res = put_user(1, (int *)arg);  // VFD has clock always on
			break;
		}
		case VFDDISPLAYCHARS:
		{
			if (mode == 0)
			{
				if (copy_from_user(&vfd_data, (void *)arg, sizeof(vfd_data)))
				{
					return -EFAULT;
				}
				if (vfd_data.length > sizeof(vfd_data.data))
				{
					vfd_data.length = sizeof(vfd_data.data);
				}
				while ((vfd_data.length > 0) && (vfd_data.data[vfd_data.length - 1 ] == '\n'))
				{
					vfd_data.length--;
				}
				res = run_draw_thread(&vfd_data);
			}
			else
			{
				mode = 0;
			}
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			dprintk(5, "%s Set light 0x%02X\n", __func__, aotom_data.u.light.onoff);
			switch (aotom_data.u.light.onoff)
			{
				case 0:  // whole display off
				{
#if defined(FP_LEDS)
					YWPANEL_FP_SetLed(LED_RED, LOG_OFF);
					if (fp_type != FP_VFD)
					{
						YWPANEL_FP_SetLed(LED_GREEN, LOG_OFF);
					}
#endif
					res = YWPANEL_FP_ShowContentOff();
					break;
				}
				case 1:  // whole display on
				{
					res = YWPANEL_FP_ShowContent();
#if defined(FP_LEDS)
					YWPANEL_FP_SetLed(LED_RED, led_state[LED_RED].state);
					if (fp_type != FP_VFD)
					{
						YWPANEL_FP_SetLed(LED_GREEN, led_state[LED_GREEN].state);
					}
#endif
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
			vfd_data.length = 0;
			res = run_draw_thread(&vfd_data);
			break;
		}
		case 0x5305:
		{
			res = 0;
			break;
		}
		case 0x5401:
		{
			res = 0;
			break;
		}
		case VFDGETSTARTUPSTATE:
		{
			YWPANEL_STARTUPSTATE_t State;
			if (YWPANEL_FP_GetStartUpState(&State))
			{
				res = put_user(State, (int *)arg);
			}
			break;
		}
		case VFDGETVERSION:
		{
			YWPANEL_Version_t fpanel_version;
			const char *fp_type[9] = { "Unknown", "VFD", "LCD", "DVFD", "LED", "?", "?", "?", "LBD" };

			memset(&fpanel_version, 0, sizeof(YWPANEL_Version_t));

			if (YWPANEL_FP_GetVersion(&fpanel_version))
			{
				dprintk(50, "%s Front panel CPU type         : %d\n", __func__, fpanel_version.CpuType);
				dprintk(50, "%s Front panel software version : %d.%d\n", __func__, fpanel_version.swMajorVersion, fpanel_version.swSubVersion);
				dprintk(50, "%s Front panel displaytype      : %s\n", __func__, fp_type[fpanel_version.DisplayInfo]);
#if defined(FP_DVFD)
				if (fpanel_version.DisplayInfo == 3)
				{
					dprintk(50, "%s Time mode                   : %s\n", __func__, tm_type[bTimeMode]);
				}
#endif
				dprintk(50, "%s # of front panel keys        : %d\n", __func__, fpanel_version.scankeyNum);
				dprintk(50, "%s Number of version bytes     : %d\n", __func__, sizeof(fpanel_version));
				res = put_user(fpanel_version.DisplayInfo, (int *)arg);
				res |= copy_to_user((char *)arg, &fpanel_version, sizeof(fpanel_version));
			}
			break;
		}
		default:
		{
			dprintk(0, "Unknown IOCTL 0x%08x\n", cmd);
			mode = 0;
			break;
		}
	}
	up(&write_sem);
	return res;
}

static unsigned int AOTOMdev_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(filp, &wq, wait);

	if (receiveCount > 0)
	{
		mask = POLLIN | POLLRDNORM;
	}
	return mask;
}

static struct file_operations vfd_fops =
{
	.owner   = THIS_MODULE,
	.ioctl   = AOTOMdev_ioctl,
	.write   = AOTOMdev_write,
	.read    = AOTOMdev_read,
	.poll    = AOTOMdev_poll,
	.open    = AOTOMdev_open,
	.release = AOTOMdev_close
};

/*----- Button driver -------*/
static char *button_driver_name = "Edision front panel button driver";
static struct input_dev *button_dev;
static int bad_polling = 1;
static struct workqueue_struct *fpwq;

static void button_bad_polling(struct work_struct *work)
{
	int btn_pressed = 0;
	int report_key = 0;

	while (bad_polling == 1)
	{
		int button_value;
		msleep(50);
		button_value = YWPANEL_FP_GetKeyValue();
		if (button_value != INVALID_KEY)
		{
			dprintk(20, "[BTN] Got button: 0x%02X\n", button_value);
			aotomSetIcon(ICON_DOT2, LOG_ON);
			if (1 == btn_pressed)
			{
				if (report_key == button_value)
				{
					continue;
				}
				input_report_key(button_dev, report_key, 0);
				input_sync(button_dev);
			}
			report_key = button_value;
			btn_pressed = 1;
			switch (button_value)
			{
				case KEY_LEFT:
				case KEY_RIGHT:
				case KEY_UP:
				case KEY_DOWN:
				case KEY_OK:
				case KEY_MENU:
				case KEY_EXIT:
				case KEY_POWER:
				{
					input_report_key(button_dev, button_value, 1);
					input_sync(button_dev);
					break;
				}
				default:
				{
					dprintk(1, "[BTN] Unknown button_value 0x%02x\n", button_value);
				}
			}
		}
		else
		{
			if (btn_pressed)
			{
				btn_pressed = 0;
				aotomSetIcon(ICON_DOT2, LOG_OFF);
				input_report_key(button_dev, report_key, 0);
				input_sync(button_dev);
			}
		}
	}
	bad_polling = 2;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 17)
static DECLARE_WORK(button_obj, button_bad_polling);
#else
static DECLARE_WORK(button_obj, button_bad_polling, NULL);
#endif

static int button_input_open(struct input_dev *dev)
{
	fpwq = create_workqueue("button");
	if (queue_work(fpwq, &button_obj))
	{
		dprintk(10, "[BTN] Queue_work successful.\n");
		return 0;
	}
	dprintk(10, "[BTN] Queue_work not successful, exiting.\n");
	return 1;
}

static void button_input_close(struct input_dev *dev)
{
	bad_polling = 0;
	while (bad_polling != 2)
	{
		msleep(1);
	}
	bad_polling = 1;

	if (fpwq)
	{
		destroy_workqueue(fpwq);
		dprintk(10, "[BTN] Workqueue destroyed.\n");
	}
}

int button_dev_init(void)
{
	int error;

	dprintk(10, "[BTN] Allocating and registering button device\n");

	button_dev = input_allocate_device();
	if (!button_dev)
	{
		return -ENOMEM;
	}

	button_dev->name  = button_driver_name;
	button_dev->open  = button_input_open;
	button_dev->close = button_input_close;

	set_bit(EV_KEY,    button_dev->evbit);
	set_bit(KEY_UP,    button_dev->keybit);
	set_bit(KEY_DOWN,  button_dev->keybit);
	set_bit(KEY_LEFT,  button_dev->keybit);
	set_bit(KEY_RIGHT, button_dev->keybit);
	set_bit(KEY_POWER, button_dev->keybit);
	set_bit(KEY_MENU,  button_dev->keybit);
	set_bit(KEY_OK,    button_dev->keybit);
	set_bit(KEY_EXIT,  button_dev->keybit);  // note: VIP2 has no exit key on the front panel

	error = input_register_device(button_dev);
	if (error)
	{
		input_free_device(button_dev);
	}
	return error;
}

void button_dev_exit(void)
{
	dprintk(10, "[BTN] Unregistering button device.\n");
	input_unregister_device(button_dev);
}

static void aotom_standby(void)
{
	while (down_interruptible(&write_sem))
	{
		msleep(10);
	}
	dprintk(20, "Initiating standby with pm_power_off hook.\n");
	clear_display();
	YWPANEL_FP_ControlTimer(true);
	YWPANEL_FP_SetCpuStatus(YWPANEL_CPUSTATE_STANDBY);
	// not reached
}

/*----- RTC driver -----*/
static int aotom_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	u32 uTime = 0;

	dprintk(100, "%s >\n", __func__);
	uTime = YWPANEL_FP_GetTime();
	rtc_time_to_tm(uTime, tm);
	dprintk(100, "%s < %d\n", __func__, uTime);
	return 0;
}

static int tm2time(struct rtc_time *tm)
{
	return mktime(tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
}

static int aotom_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int res = 0;

	u32 uTime = tm2time(tm);
	dprintk(100, "%s > uTime %d\n", __func__, uTime);
	res = YWPANEL_FP_SetTime(uTime);
	YWPANEL_FP_ControlTimer(true);
	dprintk(100, "%s < res: %d\n", __func__, res);
	return res;
}

static int aotom_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	u32 a_time = 0;

	dprintk(100, "%s >\n", __func__);
	a_time = YWPANEL_FP_GetPowerOnTime();
	if (al->enabled)
	{
		rtc_time_to_tm(a_time, &al->time);
	}
	dprintk(100, "%s < Enabled: %d RTC alarm time: %d time: %d\n", __func__, al->enabled, (int)&a_time, a_time);
	return 0;
}

static int aotom_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	u32 a_time = 0;
	dprintk(100, "%s >\n", __func__);
	if (al->enabled)
	{
		a_time = tm2time(&al->time);
	}
	YWPANEL_FP_SetPowerOnTime(a_time);
	dprintk(100, "%s < Enabled: %d time: %d\n", __func__, al->enabled, a_time);
	return 0;
}

static const struct rtc_class_ops aotom_rtc_ops =
{
	.read_time  = aotom_rtc_read_time,
	.set_time   = aotom_rtc_set_time,
	.read_alarm = aotom_rtc_read_alarm,
	.set_alarm  = aotom_rtc_set_alarm
};

static int __devinit aotom_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;

	/* I have no idea where the pdev comes from, but it needs the can_wakeup = 1
	 * otherwise we don't get the wakealarm sysfs attribute... :-) */
	dprintk(100, "%s >\n", __func__);
	printk("Edision front panel real time clock\n");
	pdev->dev.power.can_wakeup = 1;
	rtc = rtc_device_register("aotom", &pdev->dev, &aotom_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
	{
		return PTR_ERR(rtc);
	}
	platform_set_drvdata(pdev, rtc);
	dprintk(100, "%s < %p\n", __func__, rtc);
	return 0;
}

static int __devexit aotom_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	dprintk(100, "%s %p >\n", __func__, rtc);
	rtc_device_unregister(rtc);
	platform_set_drvdata(pdev, NULL);
	dprintk(100, "%s <\n", __func__);
	return 0;
}

static struct platform_driver aotom_rtc_driver =
{
	.probe = aotom_rtc_probe,
	.remove = __devexit_p(aotom_rtc_remove),
	.driver =
	{
		.name	= RTC_NAME,
		.owner	= THIS_MODULE
	},
};

static int __init aotom_init_module(void)
{
	int i;

	dprintk(0, "Edision Argus VIP1/2 front panel driver %s\n", Revision);

	if (YWPANEL_FP_Init())
	{
		dprintk(1, "Unable to initialize module.\n");
		return -1;
	}
	VFD_clr();  // clear display
	if (button_dev_init() != 0)  // initialize front panel button driver
	{
		return -1;
	}
	if (register_chrdev(VFD_MAJOR, "VFD", &vfd_fops))
	{
		dprintk(1, "Unable to get major %d for VFD.\n", VFD_MAJOR);
	}
	if (pm_power_off)
	{
		dprintk(10, "pm_power_off hook already applied! Do not register anything\n");
	}
	else
	{
		pm_power_off = aotom_standby;
	}
	sema_init(&write_sem, 1);
	sema_init(&receive_sem, 1);
	sema_init(&draw_thread_sem, 1);  // Initialize text_draw thread

	for (i = 0; i < LASTMINOR; i++)
	{
		FrontPanelOpen[i].open_count = 0;
		FrontPanelOpen[i].minor = i;
	}

#if defined(FP_LEDS)
/* Initialize LED thread */
	for (i = 0; i < LASTLED; i++)
	{
		led_state[i].state = LOG_OFF;
		led_state[i].period = 0;
		led_state[i].status = THREAD_STATUS_STOPPED;
		sema_init(&led_sem, 0);
		led_state[i].thread_task = kthread_run(led_thread, (void *)i, "led_thread");
	}
#endif
/* Initialize spinner thread */
	{
		spinner_state.state = LOG_OFF;
		spinner_state.period = 0;
		spinner_state.status = THREAD_STATUS_STOPPED;
		sema_init(&spinner_sem, 0);
		spinner_state.thread_task = kthread_run(spinner_thread, (void *)i, "spinner_thread");
	}

/* Initialize RTC driver */
	i = platform_driver_register(&aotom_rtc_driver);
	if (i)
	{
		dprintk(1, "%s platform_driver_register for RTC failed: %d\n", __func__, i);
	}
	else
	{
		rtc_pdev = platform_device_register_simple(RTC_NAME, -1, NULL, 0);
	}

	if (IS_ERR(rtc_pdev))
	{
		dprintk(1, "%s platform_device_register_simple for RTC failed: %ld\n", __func__, PTR_ERR(rtc_pdev));
	}

/* Initialize procfs part */
	create_proc_fp();
	dprintk(100, "%s <\n", __func__);

	return 0;
}

#if defined(FP_LEDS)
static int led_thread_active(void)
{
	int i;

	for (i = 0; i < LASTLED; i++)
	{
		if (!led_state[i].status && led_state[i].thread_task)
		{
			return -1;
		}
	}
	return 0;
}
#endif

static int spinner_thread_active(void)
{
	if (!spinner_state.status && spinner_state.thread_task)
	{
		return -1;
	}
	return 0;
}

static void __exit aotom_cleanup_module(void)
{
	int i;

	if (aotom_standby == pm_power_off)
	{
		pm_power_off = NULL;
	}

	platform_device_unregister(rtc_pdev);
	remove_proc_fp();

	if (!draw_thread_status && draw_task)
	{
		kthread_stop(draw_task);
	}
#if defined(FP_LEDS)
	for (i = 0; i < LASTLED; i++)
	{
		if (!led_state[i].status && led_state[i].thread_task)
		{
			up(&led_sem);
			kthread_stop(led_state[i].thread_task);
		}
	}
#endif
	if (!spinner_state.status && spinner_state.thread_task)
	{
		up(&spinner_sem);
		kthread_stop(spinner_state.thread_task);
	}

#if defined(FP_LEDS)
	while (!draw_thread_status && !led_thread_active() && !spinner_thread_active())
#else
	while (!draw_thread_status && !spinner_thread_active())
#endif
	{
		msleep(1);
	}
	dprintk(5, "[BTN] Unloading...\n");
	button_dev_exit();

	unregister_chrdev(VFD_MAJOR, "VFD");
	YWPANEL_FP_Term();
	printk("Edision front panel module unloading.\n");
}

module_init(aotom_init_module);
module_exit(aotom_cleanup_module);

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

module_param(gmt, charp, 0);
MODULE_PARM_DESC(gmt, "GMT offset (default +3600");

MODULE_DESCRIPTION("VFD module for Edision argus VIP V2 and VIP2 receivers");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
