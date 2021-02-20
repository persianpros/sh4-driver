/*
 * aotom_main.c
 *
 * (c) 2010 Spider-Team
 * (c) 2011 oSaoYa
 * (c) 2012-2013 Stefan Seyfried
 * (c) 2012-2013 martii
 * (c) 2013-2021 Audioniek
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
 * Fulan front panel driver.
 *
 * Devices:
 *	- /dev/vfd (vfd ioctls and read/write function)
 *
 *
 ****************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------
 * 20130904 Audioniek       Keypress feedback restored on Spark & Spark7162.
 * 20130904 Audioniek       Display width now box dependent.
 * 20130905 Audioniek       Functionality of VFDDISPLAYWRITEONOFF added.
 * 20130905 Audioniek       VFD_clr also switches the LEDs off.
 * 20130906 Audioniek       Several bugs in AOTOMdev_ioctl SETLED fixed.
 * 20130908 Audioniek       GetVersion implemented, multi byte return on
 *                          sparks, single byte return on others.
 * 20130911 Audioniek       VFDPOWEROFF and VFDSTANDBY now clear entire
 *                          display, not just text area.
 * 20130911 Audioniek       VFDGETWAKEUPTIME added.
 * 20130912 Audioniek       VFD_Show_Time and aotomSetTime now handle seconds.
 * 20130917 Audioniek       VFDDISPLAYWRITEONOFF 0 switches display off in
 *                          stead of setting brightness level to zero.
 * 20130923 Audioniek       aotom_rtc_read_alarm made functional.
 * 20140604 Audioniek       DVFD front panel support added.
 * 20140609 Audioniek       VFDDISPLAYWRITEONOFF also handles the LEDs.
 * 20140612 Audioniek       UTF8 support added (thnx martii).
 * 20140616 Audioniek       Spinner (icon 47) added on Spark7162, based on
 *                          an idea by martii. Arg2 controls spinning speed.
 * 20141113 Audioniek       Fixed: local keypress feedback did not work
 *                          on Spark7162.
 * 20150325 Audioniek       Local keyboard press and RC feedback through
 *                          green LED on DVFD.
 * 20150327 Audioniek       Fixed compiler problem with Spark.
 * 20150329 Audioniek       DVFD icons supported.
 * 20150402 Audioniek       VFDSETDISPLAYTIME on DVFD models fixed: argument
 *                          was not passed.
 * 20150405 Audioniek       Fixed some wrong reports on icon numbers; (re)set
 *                          all icons switches spinner off on VFD models.
 * 20150410 Audioniek       VFDGETDISPLAYTIME added.
 * 20150411 Audioniek       Corrected mistake in clear_display routine.
 * 20150520 Audioniek       Removed a lot of conditionals that assumed only
 *                          Spark7162's have a (D)VFD display. Display type
 *                          already was set at driver start and is now used
 *                          throughout instead of box model. Should fix
 *                          display problem on Edision Pingulux Plus (Spark
 *                          with (D)VFD).
 * 20151114 Audioniek       Fixed erroneous 'Tried to set illegal icon number'
 *                          message with CD segment icons.
 * 20151116 Audioniek       procfs added.
 * 20160511 Audioniek       /proc/stb/fp/rtc converted to UTC,
 *                          /proc/stb/fp/rtc_offset automatically set when
 *                          /proc/stb/fp/rtc is written to.
 * 20181203 Audioniek       /proc/stb/lcd/symbol_circle support added,
 *                          E2 start code on write progress removed.
 * 20181204 Audioniek       Spinner slowed down by a factor of 10; when
 *                          switched on with a value of 1, default speed is
 *                          1 rpm; otherwise argument times 50 ms is the time
 *                          for one revolution.
 * 20210210 Audioniek       Report correct CPU name of front processor.
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
/* for RTC / reboot_notifier hooks */
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>

#include "aotom_main.h"

short paramDebug = 0;  //debug print level is zero as default (0=nothing, 1= open/close functions, 5=some detail, 10=all)
#define TAGDEBUG "[aotom] "
#define dprintk(level, x...) do { \
		if ((paramDebug) && (paramDebug > level)) printk(TAGDEBUG x); \
	} while (0)

#define NO_KEY_PRESS     -1
#define KEY_PRESS_DOWN    1
#define KEY_PRESS_UP      0

static char *gmt = "+0000";  // GMT offset is zero as default

static int open_count = 0;

#define FRONTPANEL_MINOR_VFD 0


tLedState led_state[LASTLED + 1];

static struct semaphore write_sem;
static struct semaphore draw_thread_sem;
static struct semaphore led_sem;

static struct task_struct *draw_task = 0;
#define THREAD_STATUS_RUNNING 0
#define THREAD_STATUS_STOPPED 1
#define THREAD_STATUS_INIT 2
static int draw_thread_status = THREAD_STATUS_STOPPED;

#define RTC_NAME "aotom-rtc"
static struct platform_device *rtc_pdev;

extern YWPANEL_Version_t panel_version;

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
	dprintk(5, "%s Set brightness level 0x%02X\n", __func__, level);
	res = YWPANEL_FP_SetBrightness(level);

	return res;
}

int aotomSetIcon(int which, int on)
{
	int res = 0;
	int first, last;

	dprintk(5, "%s > Icon number %d, state %d\n", __func__, which, on);

	if (fp_type == FP_DVFD)
	{
		first = DICON_FIRST;
		last = DICON_LAST;
	}
	else
	{
		first = ICON_FIRST;
		last = ICON_LAST;
	}

	if (which < first || which > last)
	{
		printk("[aotom] Icon number %d out of range.\n", which);
		return -EINVAL;
	}

	if (fp_type == FP_VFD)
	{
		which = (((which - 1) / 15) + 11) * 16 + ((which - 1) % 15) + 1;
	}
	res = YWPANEL_FP_ShowIcon(which, on);

	dprintk(10, "%s <\n", __func__);
	return res;
}

void VFD_set_all_icons(int onoff)
{
	int i, first, last;

	if (fp_type == FP_DVFD)
	{
		first = DICON_FIRST;
		last = DICON_LAST;
	}
	else
	{
		first = ICON_FIRST;
		last = ICON_LAST;
		led_state[LED_SPINNER].state = 0;  // spinner off
	}

	for (i = first; i < last + 1; i++)
	{
		aotomSetIcon(i, onoff);
	}
}

void clear_display(void)
{
	char clrstr[16];

	memset(clrstr, 0, 16);
	memset(clrstr, ' ', YWPANEL_width);
	YWPANEL_FP_ShowString(clrstr);
}

static void VFD_clr(void)
{
	clear_display();

	if (fp_type != FP_LED)
	{
		VFD_set_all_icons(LOG_OFF);
	}

	if (fp_type != FP_VFD)
	{
		YWPANEL_FP_SetLed(LED_GREEN, LOG_OFF);
	}
	YWPANEL_FP_SetLed(LED_RED, LOG_OFF);
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

static int draw_thread(void *arg)
{
	struct vfd_ioctl_data *data = (struct vfd_ioctl_data *)arg;
	unsigned char buf[sizeof(data->data) + 2 * YWPANEL_width];
	int len = data->length;
	int off = 0;
	int saved = 0;
	int utf8len = utf8strlen(data->data, data->length);

	if (fp_type == FP_LED && len > 2 && (data->data[2] == '.' || data->data[2] == ','|| data->data[2] == ':'))
	{
		saved = 1;
	}
	if (utf8len - saved > YWPANEL_width)
	{
		memset(buf, ' ', sizeof(buf));
		off = YWPANEL_width - 1;
		memcpy(buf + off, data->data, len);
		len += off;
		utf8len += off;
		buf[len + YWPANEL_width] = 0;
	}
	else
	{
		memcpy(buf, data->data, len);
		buf[len] = 0;
	}
	draw_thread_status = THREAD_STATUS_RUNNING;

	if (utf8len - saved > YWPANEL_width + 1)
	{
		unsigned char *b = buf;
		int pos;
		for (pos = 0; pos < utf8len; pos++)
		{
			int i;
			char dot = 0;

			if (kthread_should_stop())
			{
				draw_thread_status = THREAD_STATUS_STOPPED;
				return 0;
			}
			if (fp_type == FP_LED && (b + 2 < buf + sizeof(buf)) && (b[2] == '.' || b[2] == ','|| b[2] == ':'))
			{
				dot = b[2];
			}
			if (dot)
			{
				b[2] = ' ';
			}
			YWPANEL_FP_ShowString(b);
			if (dot)
			{
				b[2] = dot;
			}

			// sleep 200 ms
			for (i = 0; i < 5; i++)
			{
				if (kthread_should_stop())
				{
					draw_thread_status = THREAD_STATUS_STOPPED;
					return 0;
				}
				msleep(40);
			}
			// advance to next UTF-8 character
			b += utf8charlen(*b);
		}
	}
	if (utf8len > 0)
	{
		YWPANEL_FP_ShowString(buf + off);
	}
	else
	{
		clear_display();
	}
	draw_thread_status = THREAD_STATUS_STOPPED;
	return 0;
}

static int led_thread(void *arg)
{
	int led = (int)arg;

	// toggle LED status for a given time period
	led_state[led].status = THREAD_STATUS_RUNNING;

	while (!kthread_should_stop())
	{
		if (!down_interruptible(&led_sem))
		{
			if (kthread_should_stop())
			{
				break;
			}
			while (!down_trylock(&led_sem));  // make sure semaphore is at 0
			{
				YWPANEL_FP_SetLed(led, !led_state[led].state); // toggle LED
			}

			while ((led_state[led].period > 0) && !kthread_should_stop())
			{
				msleep(10);
				led_state[led].period -= 10;
			}
			// switch LED back to state previously set
			YWPANEL_FP_SetLed(led, led_state[led].state);
		}
	}
	led_state[led].status = THREAD_STATUS_STOPPED;
	led_state[led].led_task = 0;
	return 0;
}

static int spinner_thread(void *arg)
{
	int led = (int)arg;
	int i = 0;

	if (led_state[led].status == THREAD_STATUS_RUNNING)
	{
		return 0;
	}
	led_state[led].status = THREAD_STATUS_RUNNING;

	while (!kthread_should_stop())
	{
		if (!down_interruptible(&led_sem))
		{
			if (kthread_should_stop())
			{
				break;
			}
			while (!down_trylock(&led_sem));

			aotomSetIcon(ICON_DISK_CIRCLE, LOG_ON);
			while ((led_state[led].state) && !kthread_should_stop())
			{
				switch (i) // display a rotating disc in 5 states
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
				i++;
				i %= 5;
				msleep(led_state[led].period * 10);
			}
			aotomSetIcon(ICON_DISK_S3, LOG_OFF);
			aotomSetIcon(ICON_DISK_S2, LOG_OFF);
			aotomSetIcon(ICON_DISK_S1, LOG_OFF);
			aotomSetIcon(ICON_DISK_CIRCLE, LOG_OFF);
		}
	}
	led_state[led].status = THREAD_STATUS_STOPPED;
	led_state[led].led_task = 0;
	return 0;
}

static struct vfd_ioctl_data last_draw_data;

static int run_draw_thread(struct vfd_ioctl_data *draw_data)
{
	if (down_interruptible(&draw_thread_sem))
	{
		return -ERESTARTSYS;
	}

	// return if there is already a draw task running for the same text
	if ((draw_thread_status != THREAD_STATUS_STOPPED)
	&&  draw_task
	&&  (last_draw_data.length == draw_data->length)
	&&  !memcmp(&last_draw_data.data, draw_data->data, draw_data->length))
	{
		up(&draw_thread_sem);
		return 0;
	}

	memcpy(&last_draw_data, draw_data, sizeof(struct vfd_ioctl_data));

	// stop existing thread, if any
	if ((draw_thread_status != THREAD_STATUS_STOPPED) && draw_task)
	{
		kthread_stop(draw_task);
		while ((draw_thread_status != THREAD_STATUS_STOPPED))
		{
			msleep(1);
		}
	}
	draw_thread_status = THREAD_STATUS_INIT;
	draw_task = kthread_run(draw_thread, draw_data, "draw_thread");

	//wait until thread has copied the argument
	while (draw_thread_status == THREAD_STATUS_INIT)
	{
		msleep(1);
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
//EXPORT_SYMBOL(aotomWriteText);

int aotomSetTime(char *time)
{
	int res = 0;

	dprintk(5, "%s >\n", __func__);
	dprintk(10, "%s time: %02d:%02d:%02d\n", __func__, time[2], time[3], time[4]);

	res = VFD_Show_Time(time[2], time[3], time[4]);
	YWPANEL_FP_ControlTimer(true);
	dprintk(5, "%s <\n", __func__);
	return res;
}

int vfd_init_func(void)
{
	dprintk(5, "%s >\n", __func__);
	printk("Fulan VFD module initializing\n");
	return 0;
}

static ssize_t AOTOMdev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char *kernel_buf;
	int res = 0;

	struct vfd_ioctl_data data;

	dprintk(5, "%s > (len %d, offs %d)\n", __func__, len, (int) *off);

	kernel_buf = kmalloc(len + 1, GFP_KERNEL);

	memset(kernel_buf, 0, len + 1);
	memset(&data, 0, sizeof(struct vfd_ioctl_data));

	if (kernel_buf == NULL)
	{
		printk("%s returns no memory <\n", __func__);
		return -ENOMEM;
	}
	copy_from_user(kernel_buf, buff, len);

	aotomWriteText(kernel_buf, len);

	kfree(kernel_buf);

	dprintk(10, "%s < res %d len %d\n", __func__, res, len);

	if (res < 0)
	{
		return res;
	}
	return len;
}

void flashLED(int led, int ms)
{
	if (!led_state[led].led_task || ms < 1)
	{
		return;
	}
	if (ms == 1)	// if default for on
	{
		ms = 20;	// get default speed (1 rotation takes 5 x 10 x 20 ms = 1 second)
	}
	led_state[led].period = ms;
	up(&led_sem);
}

static int AOTOMdev_open(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(5, "%s >\n", __func__);

	minor = MINOR(inode->i_rdev);

	dprintk(1, "Open minor %d\n", minor);

	if (minor != FRONTPANEL_MINOR_VFD)
	{
		return -ENOTSUPP;
	}
	open_count++;

	dprintk(5, "%s <\n", __func__);
	return 0;
}

static int AOTOMdev_close(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(5, "%s >\n", __func__);

	minor = MINOR(inode->i_rdev);

	dprintk(1, "Close minor %d\n", minor);

	if (open_count > 0)
	{
		open_count--;
	}

	dprintk(5, "%s <\n", __func__);
	return 0;
}

static struct aotom_ioctl_data aotom_data;
static struct vfd_ioctl_data vfd_data;

static int AOTOMdev_ioctl(struct inode *Inode, struct file *File, unsigned int cmd, unsigned long arg)
{
	static int mode = 0;
	int res = -EINVAL;
	dprintk(5, "%s > 0x%.8x\n", __func__, cmd);

	if (down_interruptible(&write_sem))
	{
		return -ERESTARTSYS;
	}

	switch (cmd)
	{
		case VFDSETMODE:
		case VFDSETLED:
		case VFDICONDISPLAYONOFF:
		case VFDSETTIME:
		case VFDBRIGHTNESS:
		case VFDDISPLAYWRITEONOFF:
		case VFDGETSTBYKEY:
		case VFDSETSTBYKEY:
		case VFDGETBLUEKEY:
		case VFDSETBLUEKEY:
		case VFDSETDISPLAYTIME:
		{
			if (copy_from_user(&aotom_data, (void *) arg, sizeof(aotom_data)))
			{
				return -EFAULT;
			}
		}
	}

	switch (cmd)
	{
		case VFDSETMODE:
		{
			mode = aotom_data.u.mode.compat;
			break;
		}
		case VFDSETLED:
		{
			int led_nr = aotom_data.u.led.led_nr;

			if (led_nr > -1 && led_nr < LASTLED)
			{
				switch (aotom_data.u.led.on)
				{
					case LOG_OFF:
					case LOG_ON:
					{
						dprintk(10, "%s Set LED %d to 0x%2X\n", __func__, led_nr, aotom_data.u.led.on);
						res = YWPANEL_FP_SetLed(led_nr, aotom_data.u.led.on);
						led_state[led_nr].state = aotom_data.u.led.on;
						break;
					}
					default:  // toggle for (aotom_data.u.led.on * 10) ms
					{
						dprintk(10, "%s Blink LED %d for 10ms\n", __func__, led_nr);
						flashLED(led_nr, aotom_data.u.led.on * 10);
						res = 0;  //was missing, reporting blink as illegal value
					}
				}
			}
			break;
		}
		case VFDBRIGHTNESS:
		{
// TODO: fix DVFD in aotom_vfd
			if (fp_type)
			{
				res = aotomSetBrightness(aotom_data.u.brightness.level);
			}
			else
			{
				res = 0;
			}
			break;
		}
		case VFDICONDISPLAYONOFF:
		{
			if (fp_type == FP_LED)
			{
				switch (aotom_data.u.icon.icon_nr)
				{
					case 0: //icon number is the same as the LED number
					{
						res = YWPANEL_FP_SetLed(LED_RED, aotom_data.u.icon.on);
						led_state[LED_RED].state = aotom_data.u.icon.on;
						break;
					}
					case 1: //icon number is the same as the LED number
					case ICON_DOT2: //(RC feedback)
					{
						res = YWPANEL_FP_SetLed(LED_GREEN, aotom_data.u.icon.on);
						led_state[LED_GREEN].state = aotom_data.u.icon.on;
						break;
					}
					case ICON_ALL:
					{
						led_state[LED_RED].state = aotom_data.u.icon.on;
						led_state[LED_GREEN].state = aotom_data.u.icon.on;
						YWPANEL_FP_SetLed(LED_RED, aotom_data.u.icon.on);
						res = YWPANEL_FP_SetLed(LED_GREEN, aotom_data.u.icon.on);
						break;
					}
					default:
					{
						res = 0;
						break;
					}
				}
			}
			else //VFD and DVFD
			{
				int icon_nr = aotom_data.u.icon.icon_nr;
				//e2 icons work around
				if (icon_nr >= 256)
				{
					icon_nr >>= 8;
					if (fp_type == FP_VFD)
					{
						switch (icon_nr)
						{
							case 17:
							{
								icon_nr = ICON_DOUBLESCREEN; //widescreen
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
								icon_nr = ICON_PLAY_LOG; //Play
								break;
							}
							case 30:
							{
								icon_nr = ICON_REC1;
								break;
							}
							case 38: //CD segments & circle
							case 39:
							case 40:
							case 41:
							{
								break;
							}
							case 47:
							{
								icon_nr = ICON_SPINNER;
								break;
							}
							default:
							{
								printk("[aotom] Tried to set unknown icon number %d.\n", icon_nr);
//								icon_nr = 29; //no additional symbols at the moment: show alert instead
								break;
							}
						}
					}
					else //DVFD
					{
						switch (icon_nr)
						{
							case 17:
							{
								icon_nr = ICON_DOUBLESCREEN2; //widescreen
								break;
							}
							case 19:
							{
								icon_nr = ICON_CA2;
								break;
							}
							case 23:
							{
								icon_nr = ICON_DOLBY2;
								break;
							}
							default:
							{
								break;
							}
						}
					}
				}
// translate VFD icons that also exist on DVFD to their DVFD number
				if (fp_type == FP_DVFD)
				{
					switch (icon_nr)
					{
						case ICON_CA:
						{
							icon_nr = ICON_CA2;
							break;
						}
						case ICON_CI:
						{
							icon_nr = ICON_CI2;
							break;
						}
						case ICON_USB:
						{
							icon_nr = ICON_USB2;
							break;
						}
						case ICON_DOUBLESCREEN:
						{
//							icon_nr = ICON_DOUBLESCREEN2;
							icon_nr = ICON_HD;
							break;
						}
						case ICON_MUTE:
						{
							icon_nr = ICON_MUTE2;
							break;
						}
						case ICON_AC3:
						case ICON_DOLBY:
						{
							icon_nr = ICON_DOLBY2;
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
						if (fp_type == FP_VFD)
						{
							led_state[LED_SPINNER].state = aotom_data.u.icon.on;
							if (aotom_data.u.icon.on)
							{
								flashLED(LED_SPINNER, aotom_data.u.icon.on * 10); // start spinner thread
							}
						}
						res = 0;
						break;
					}
					case ICON_DOT2: // RC feedback from evremote2
					{
						if (fp_type == FP_DVFD)
						{
							res = YWPANEL_FP_SetLed(LED_GREEN, aotom_data.u.icon.on);
							led_state[LED_GREEN].state = aotom_data.u.icon.on;
						}
						else
						{
							res = aotomSetIcon(icon_nr, aotom_data.u.icon.on);
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
		case VFDPOWEROFF:
		{
			VFD_clr();
			YWPANEL_FP_ControlTimer(true);
			YWPANEL_FP_SetCpuStatus(YWPANEL_CPUSTATE_STANDBY);
			res = 0;
			break;
		}
		case VFDSTANDBY:
		{
			u32 uTime = 0;
			get_user(uTime, (int *) arg);

			YWPANEL_FP_SetPowerOnTime(uTime);
			VFD_clr();
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
			if (fp_type == FP_DVFD)
			{
				YWPANEL_FP_DvfdSetTimeMode(aotom_data.u.display_time.on);
			}
			res = 0;
			break;
		}
		case VFDGETDISPLAYTIME:
		{
			int TimeMode;

			if (fp_type)
			{
				if (fp_type == FP_DVFD)
				{
					TimeMode = bTimeMode;
				}
				else
				{
					TimeMode = 1; // clock is always on on VFD
				}
				res = put_user(TimeMode, (int *)arg);
			}
			else //LED
			{
				res = 0;
			}
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
				res = aotomWriteText(vfd_data.data, vfd_data.length);

				if (res >= 0)
				{
					res = vfd_data.length;
				}
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
				case 0: //whole display off
				{
					YWPANEL_FP_SetLed(LED_RED, LOG_OFF);
					if (fp_type != FP_VFD)
					{
						YWPANEL_FP_SetLed(LED_GREEN, LOG_OFF);
					}
					res = YWPANEL_FP_ShowContentOff();
					break;
				}
				case 1: //whole display on
				{
					res = YWPANEL_FP_ShowContent();
					YWPANEL_FP_SetLed(LED_RED, led_state[LED_RED].state);
					if (fp_type != FP_VFD)
					{
						YWPANEL_FP_SetLed(LED_GREEN, led_state[LED_GREEN].state);
					}
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
		case VFDGETSTARTUPSTATE:
		{
			YWPANEL_STARTUPSTATE_t state;

			if (YWPANEL_FP_GetStartUpState(&state))
			{
				dprintk(5, "%s Front panel startup state: %02X\n", __func__, state);
				res = put_user(state, (int *) arg);
			}
			break;
		}
		case VFDSETLOOPSTATE:
		{
			YWPANEL_LOOPSTATE_t state = YWPANEL_LOOPSTATE_UNKNOWN;

			res = get_user(state, (int *)arg);
			if (!res)
			{
				res = YWPANEL_FP_SetLoopState(state);
				dprintk(5, "%s Front panel loop state set to: %02X\n", __func__, state);
			}
			break;
		}
		case VFDGETLOOPSTATE:
		{
			YWPANEL_LOOPSTATE_t state;

			if (YWPANEL_FP_GetLoopState(&state))
			{
				dprintk(5, "%s Front panel loop state: %02X\n", __func__, state);
				res = put_user(state, (int *)arg);
			}
			break;
		}
		case VFDGETVERSION:
		{
			YWPANEL_Version_t fpanel_version;
			const char *fp_type[9] = { "Unknown", "VFD", "LCD", "DVFD", "LED", "?", "?", "?", "LBD" };
			const char *tm_type[2] = { "Off", "On" };

			memset(&fpanel_version, 0, sizeof(YWPANEL_Version_t));

			if (YWPANEL_FP_GetVersion(&fpanel_version))
			{
				dprintk(1, "%s Front panel CPU type         : %s\n", __func__, (fpanel_version.CpuType == 2 ? "ATtiny88" : "ATtiny48"));
				dprintk(1, "%s Front panel software version : %d.%d\n", __func__, fpanel_version.swMajorVersion, fpanel_version.swSubVersion);
				dprintk(1, "%s Front panel displaytype      : %s\n", __func__, fp_type[fpanel_version.DisplayInfo]);
				if (fpanel_version.DisplayInfo == 3)
				{
					dprintk(1, "%s Time mode                   : %s\n", __func__, tm_type[bTimeMode]);
				}
				dprintk(1, "%s # of front panel keys        : %d\n", __func__, fpanel_version.scankeyNum);
				dprintk(1, "%s Number of version bytes     : %d\n", __func__, sizeof(fpanel_version));
				res = put_user(fpanel_version.DisplayInfo, (int *)arg);
				res |= copy_to_user((char *)arg, &fpanel_version, sizeof(fpanel_version));
			}
			break;
		}
		case VFDGETBLUEKEY:
		case VFDGETSTBYKEY:
		{
			if (YWPANEL_FP_GetKey(cmd == VFDGETBLUEKEY, aotom_data.u.key.key_nr, &aotom_data.u.key.key))
			{
				res = copy_to_user((void *)arg, &aotom_data, sizeof(aotom_data));
			}
			break;
		}
		case VFDSETBLUEKEY:
		case VFDSETSTBYKEY:
		{
			res = !YWPANEL_FP_SetKey(cmd == VFDSETBLUEKEY, aotom_data.u.key.key_nr, aotom_data.u.key.key);
			break;
		}
		default:
		{
			printk("[aotom] Unknown IOCTL 0x%08x.\n", cmd);
		}
		case 0x5305:
		case 0x5401:
		{
			mode = 0;
			break;
		}
	}
	up(&write_sem);
	dprintk(5, "%s <\n", __func__);
	return res;
}

static struct file_operations vfd_fops =
{
	.owner   = THIS_MODULE,
	.ioctl   = AOTOMdev_ioctl,
	.write   = AOTOMdev_write,
	.open    = AOTOMdev_open,
	.release = AOTOMdev_close
};

/*----- Button driver -------*/
static char *button_driver_name = "Fulan front panel buttons driver";
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
			dprintk(5, "Got button: %02X\n", button_value);

			switch (fp_type)
			{
				case FP_VFD:
				{
					aotomSetIcon(ICON_DOT2, LOG_ON);
					break;
				}
				case FP_LED:
				case FP_DVFD:
				{
					YWPANEL_FP_SetLed(LED_GREEN, LOG_ON);
					break;
				}
				default:
				{
					flashLED(LED_GREEN, 100);
					break;
				}
			}

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
					dprintk(5, "[BTN] Unknown button_value %02X\n", button_value);
				}
			}
		}
		else
		{
			if (btn_pressed)
			{
				btn_pressed = 0;

				switch (fp_type)
				{
					case FP_VFD:
					{
						aotomSetIcon(ICON_DOT2, LOG_OFF);
						break;
					}
					case FP_LED:
					case FP_DVFD:
					{
						YWPANEL_FP_SetLed(LED_GREEN, LOG_OFF);
						break;
					}
				}

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
		dprintk(5, "[BTN] Queue_work successful.\n");
		return 0;
	}
	dprintk(5, "[BTN] Queue_work not successful, exiting.\n");
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
		dprintk(5, "[BTN] Workqueue destroyed.\n");
	}
}

int button_dev_init(void)
{
	int error;

	dprintk(5, "[BTN] Allocating and registering button device\n");

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
	set_bit(KEY_EXIT,  button_dev->keybit);

	error = input_register_device(button_dev);
	if (error)
	{
		input_free_device(button_dev);
	}
	return error;
}

void button_dev_exit(void)
{
	dprintk(5, "[BTN] Unregistering button device.\n");
	input_unregister_device(button_dev);
}

/*----- Reboot notifier -----*/
static int aotom_reboot_event(struct notifier_block *nb, unsigned long event, void *ptr)
{
	switch (event)
	{
		case SYS_POWER_OFF:
		{
			if (!fp_type)
			{
				YWPANEL_FP_ShowString("Power off");
			}
			break;
		}
		case SYS_HALT:
		{
//			if (!fp_type)
//			{
//				YWPANEL_FP_ShowString("Halt");
//			}
			break;
		}
		default:
		{
			VFD_clr();
//			if (!fp_type)
//			{
//				YWPANEL_FP_ShowString("Reboot");
//			}
//			else
//			{
//				YWPANEL_FP_ShowString("Rebt");
//			}
			return NOTIFY_DONE;
		}
	}
	msleep(1000);
	clear_display();
	YWPANEL_FP_ControlTimer(true);
	YWPANEL_FP_SetCpuStatus(YWPANEL_CPUSTATE_STANDBY);
	return NOTIFY_DONE;
}

static struct notifier_block aotom_reboot_block =
{
	.notifier_call = aotom_reboot_event,
	.priority = INT_MAX
};

/*----- RTC driver -----*/
static int aotom_rtc_read_time(struct device *dev, struct rtc_time *tm)
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

static int aotom_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int res = 0;

	u32 uTime = tm2time(tm);
	dprintk(5, "%s > uTime %d\n", __func__, uTime);
	res = YWPANEL_FP_SetTime(uTime);
	YWPANEL_FP_ControlTimer(true);
	dprintk(5, "%s < res: %d\n", __func__, res);
	return res;
}

static int aotom_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	u32 a_time = 0;

	dprintk(5, "%s >\n", __func__);
	a_time = YWPANEL_FP_GetPowerOnTime();
	if (al->enabled)
	{
		rtc_time_to_tm(a_time, &al->time);
	}
	dprintk(5, "%s < Enabled: %d RTC alarm time: %d time: %d\n", __func__, al->enabled, (int)&a_time, a_time);
	return 0;
}

static int aotom_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	u32 a_time = 0;
	dprintk(5, "%s >\n", __func__);
	if (al->enabled)
	{
		a_time = tm2time(&al->time);
	}
	YWPANEL_FP_SetPowerOnTime(a_time);
	dprintk(5, "%s < Enabled: %d time: %d\n", __func__, al->enabled, a_time);
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
	dprintk(5, "%s >\n", __func__);
	printk("Fulan front panel real time clock\n");
	pdev->dev.power.can_wakeup = 1;
	rtc = rtc_device_register("aotom", &pdev->dev, &aotom_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
	{
		return PTR_ERR(rtc);
	}
	platform_set_drvdata(pdev, rtc);
	dprintk(5, "%s < %p\n", __func__, rtc);
	return 0;
}

static int __devexit aotom_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	dprintk(5, "%s %p >\n", __func__, rtc);
	rtc_device_unregister(rtc);
	platform_set_drvdata(pdev, NULL);
	dprintk(5, "%s <\n", __func__);
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

extern void create_proc_fp(void);
extern void remove_proc_fp(void);

static int __init aotom_init_module(void)
{
	int i;

	dprintk(5, "%s >\n", __func__);
	printk("Fulan front panel driver %s\n", Revision);

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
	sema_init(&write_sem, 1);
	sema_init(&draw_thread_sem, 1);

	for (i = 0; i < LASTLED; i++)
	{
		led_state[i].state = LOG_OFF;
		led_state[i].period = 0;
		led_state[i].status = THREAD_STATUS_STOPPED;
		sema_init(&led_sem, 0);
		if (i == LED_SPINNER && fp_type == FP_VFD)
		{
			led_state[LED_SPINNER].led_task = kthread_run(spinner_thread, (void *) LED_SPINNER, "spinner_thread");
		}
		else
		{
			led_state[i].led_task = kthread_run(led_thread, (void *)i, "led_thread");
		}
	}

	register_reboot_notifier(&aotom_reboot_block);

	i = platform_driver_register(&aotom_rtc_driver);
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

	create_proc_fp();

	dprintk(5, "%s <\n", __func__);

	return 0;
}

static int led_thread_active(void)
{
	int i;

	for (i = 0; i < LASTLED; i++)
	{
		if (!led_state[i].status && led_state[i].led_task)
		{
			return -1;
		}
	}
	return 0;
}

static void __exit aotom_cleanup_module(void)
{
	int i;

	unregister_reboot_notifier(&aotom_reboot_block);
	platform_driver_unregister(&aotom_rtc_driver);
	platform_set_drvdata(rtc_pdev, NULL);
	platform_device_unregister(rtc_pdev);
	remove_proc_fp();

	if ((draw_thread_status != THREAD_STATUS_STOPPED) && draw_task)
	{
		kthread_stop(draw_task);
	}

	for (i = 0; i < LASTLED; i++)
	{
		if (!led_state[i].status && led_state[i].led_task)
		{
			up(&led_sem);
			kthread_stop(led_state[i].led_task);
		}
	}

	while ((draw_thread_status != THREAD_STATUS_STOPPED) && !led_thread_active())
	{
		msleep(1);
	}
	dprintk(5, "[BTN] Unloading...\n");
	button_dev_exit();

	unregister_chrdev(VFD_MAJOR, "VFD");
	YWPANEL_FP_Term();
	printk("Fulan front panel module unloading.\n");
}

module_init(aotom_init_module);
module_exit(aotom_cleanup_module);

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

module_param(gmt, charp, 0);
MODULE_PARM_DESC(gmt, "GMT offset (default +0000");

MODULE_DESCRIPTION("VFD module for Fulan receivers");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
