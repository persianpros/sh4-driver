/*
 * aotom_procfs.c
 *
 * (c) ? Gustav Gans
 * (c) 2014 skl
 * (c) 2015-2020 Audioniek
 *
 * Version for Edision Argus VIP2
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
 * procfs for Edision front panel driver.
 *
 *
 ****************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------
 * 20200701 Audioniek       Initial version based on aotom_procfs.c
 *                          for Spark.
 * 20200703 Audioniek       /proc/stb/lcd/symbol_circle support added.
 * 20200703 Audioniek       /proc/stb/lcd/symbol_timeshift support added.
 * 
 ****************************************************************************/

#include <linux/proc_fs.h>      /* proc fs */
#include <asm/uaccess.h>        /* copy_from_user */
#include <linux/time.h>
//#include <time.h>
#include <linux/kernel.h>
#include "aotom_main.h"

/*
 *  /proc/------
 *             |
 *             +--- progress (rw)           Progress of E2 startup in %
 *
 *  /proc/stb/fp
 *             |
 *             +--- aotom (w)               Direct control of LEDs and icons
 *             +--- version (r)             SW version of front processor (hundreds = major, ten/units = minor)
 *             +--- rtc (rw)                RTC time (UTC, seconds since Unix epoch))
 *             +--- rtc_offset (rw)         RTC offset in seconds from UTC, default = 3600 seconds
 *             +--- wakeup_time (rw)        Next wakeup time (absolute, local, seconds since Unix epoch)
 *             +--- was_timer_wakeup (r)    Wakeup reason (1 = timer, 0 = other)
 *             +--- led0_pattern (rw)       Blink pattern for red LED (currently limited to on (0xffffffff) or off (0))
 *             +--- led1_pattern (rw)       Blink pattern for green LED (currently limited to on (0xffffffff) or off (0))
 *             +--- led_patternspeed (rw)   Blink speed for pattern (not implemented)
 *             +--- oled_brightness (w)     Direct control of display brightness ((D)VFD only)
 *             +--- displaytype (r)         Type of display (0=LED, 1=VFD, 2=DVFD)
 *             +--- timemode (rw)           Time display on/off (write on DVFD only)
 *             +--- text (w)                Direct writing of display text
 *
 *  /proc/stb/lcd/
 *             |
 *             +--- symbol_circle (rw)       Control of spinner (VFD only)
 *             +--- symbol_timeshift (rw)    Control of TimeShift icon (#43, VFD only)
 *
 *  /proc/stb/power/
 *             |
 *             +--- standbyled (w)          Standby LED enable
 */

/* from e2procfs */
extern int install_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc, void *data);
extern int remove_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc);

/* from other aotom modules */
extern int aotomSetIcon(int which, int on);
#if defined(FP_LEDS)
extern void flashLED(int led, int ms);
#endif
extern int show_spinner(int period);
extern void VFD_set_all_icons(int onoff);
extern int aotomWriteText(char *buf, size_t len);
extern int aotomSetBrightness(int level);
extern void clear_display(void);

/* Globals */
static int rtc_offset = 3600;
static u32 wakeup_time;
static int progress = 0;
static int symbol_circle = 0;
static int symbol_timeshift = 0;
static u32 led0_pattern = 0;
#if defined(FP_LEDS)
static u32 led1_pattern = 0;
#endif
static int led_pattern_speed = 20;

int aotomEnableLed(int which, int on)
{
	int res = 0;

	if (which < 0 || which >= LASTLED)
	{
		dprintk(1, "LED number out of range %d\n", which);
		return -EINVAL;
	}
	led_state[which].enable = on;
	return res;
}

static int text_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	int ret = -ENOMEM;

	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;
		if (copy_from_user(page, buf, count) == 0)
		{
			ret = aotomWriteText(page, count);
			if (ret >= 0)
			{
				ret = count;
			}
		}
		free_page((unsigned long)page);
	}
	return ret;
}

static int progress_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char* page;
	ssize_t ret = -ENOMEM;
	char* myString;

	page = (char*)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;
		if (copy_from_user(page, buf, count))
		{
			goto out;
		}
		myString = (char*) kmalloc(count + 1, GFP_KERNEL);
		strncpy(myString, page, count);
		myString[count - 1] = '\0';

		sscanf(myString, "%d", &progress);
//		dprintk(1, "progress = %2d\n", progress);
		kfree(myString);

		/* always return count to avoid endless loop */
		ret = count;
	}

out:
	free_page((unsigned long)page);
	return ret;
}

static int progress_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page, "%d", progress);
	}
	return len;
}

static int symbol_circle_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char* page;
	ssize_t ret = -ENOMEM;
	char* myString;

	page = (char*)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;
		if (copy_from_user(page, buf, count))
		{
			goto out;
		}
		myString = (char*) kmalloc(count + 1, GFP_KERNEL);
		strncpy(myString, page, count);
		myString[count - 1] = '\0';

		sscanf(myString, "%d", &symbol_circle);
		kfree(myString);

		if (symbol_circle > 255)
		{
			symbol_circle = 255;
		}
		spinner_state.state = ((symbol_circle < 1) ? 0 : 1);
		if (symbol_circle != 0)
		{
			if (symbol_circle == 1)
			{
				ret = show_spinner(40);  // start spinner thread (default value)
			}
			else
			{
				ret = show_spinner(symbol_circle * 5);  // start spinner thread (user value)
			}
		}
		if (ret >= 0)
		{
			ret = count;
		}
		/* always return count to avoid endless loop */
		ret = count;
		
	}

out:
	free_page((unsigned long)page);
	return ret;
}

static int symbol_circle_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page, "%d", symbol_circle);
	}
	return len;
}

static int symbol_timeshift_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char* page;
	ssize_t ret = -ENOMEM;
	char* myString;

	page = (char*)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;
		if (copy_from_user(page, buf, count))
		{
			goto out;
		}
		myString = (char*) kmalloc(count + 1, GFP_KERNEL);
		strncpy(myString, page, count);
		myString[count - 1] = '\0';

		sscanf(myString, "%d", &symbol_timeshift);
		kfree(myString);

		if (symbol_timeshift > 0)
		{
			symbol_timeshift = 1;
		}
		else
		{
			symbol_timeshift = 0;
		}
		aotomSetIcon(ICON_TIMESHIFT, symbol_timeshift);
		/* always return count to avoid endless loop */
		ret = count;
		
	}

out:
	free_page((unsigned long)page);
	return ret;
}

static int symbol_timeshift_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page, "%d", symbol_timeshift);
	}
	return len;
}

// Proc for accessing quick control of aotom (by skl)
// String format: fxy
// f is "i" (for icons)
// x is 0/1 and indicates if the icon must be off or on
// y is the icon number (between 1 and 46, for icons, with y==46, all the icons are set)
static int aotom_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	int which, on;
	int ret = -ENOMEM;

	page = (char *)__get_free_page(GFP_KERNEL);
	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buf, count) == 0)
		{
			page[count - 1] = '\0';

			if (count > 3)
			{
				which = (int)simple_strtol(page + 2, NULL, 10);
				on = (page[1] == '0') ? 0 : 1;

				switch (page[0])
				{
//					case 'l':
//					{
//						if ((which >= 0) && (which < LASTLED))
//						{
//							YWPANEL_FP_SetLed(which, on);
//						}
//						break;
//					}
					case 'i':
					{
						if (which == ICON_ALL)
						{
							VFD_set_all_icons(on);
						}
						else if ((which >= ICON_FIRST) && (which <= ICON_SPINNER))  // !assumes VFD
						{
							aotomSetIcon(which, on);
						}
						break;
					}
				}
			}
			ret = count;
		}
		free_page((unsigned long)page);
	}
	return ret;
}

static int rtc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	u32 rtc_time = YWPANEL_FP_GetTime(); // UTC

	if (NULL != page)
	{
		len = sprintf(page, "%u\n", rtc_time - rtc_offset);
	}
	return len;
}

static int rtc_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page = NULL;
	ssize_t ret = -ENOMEM;
	u32 argument = 0;
	int test = -1;
	char *myString = kmalloc(count + 1, GFP_KERNEL);

	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buf, count))
		{
			goto out;
		}
		strncpy(myString, page, count);
		myString[count] = '\0';

		test = sscanf (myString, "%u", &argument);

		if (test > 0)
		{
			YWPANEL_FP_SetTime(argument + rtc_offset);
			YWPANEL_FP_ControlTimer(true);
		}
		/* always return count to avoid endless loop */
		ret = count;
	}

out:
	free_page((unsigned long)page);
	kfree(myString);
	return ret;
}

static int rtc_offset_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page, "%d\n", rtc_offset);
	}
	return len;
}

static int rtc_offset_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page = NULL;
	ssize_t ret = -ENOMEM;
	int test = -1;
	char *myString = kmalloc(count + 1, GFP_KERNEL);

	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;
		if (copy_from_user(page, buf, count))
		{
			goto out;
		}
		strncpy(myString, page, count);
		myString[count] = '\0';

		test = sscanf (myString, "%d", &rtc_offset);

		/* always return count to avoid endless loop */
		ret = count;
	}
out:
	free_page((unsigned long)page);
	kfree(myString);
	dprintk(100, "%s <\n", __func__);
	return ret;
}

static int wakeup_time_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page = NULL;
	ssize_t ret = -ENOMEM;
	int test = -1;
	char *myString = kmalloc(count + 1, GFP_KERNEL);

	dprintk(100, "%s > %s\n", __func__, myString);

	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buf, count))
		{
			goto out;
		}
		strncpy(myString, page, count);
		myString[count] = '\0';

		test = sscanf(myString, "%u", &wakeup_time);

		if (0 < test)
		{
			YWPANEL_FP_SetPowerOnTime(wakeup_time + rtc_offset);
//			YWPANEL_FP_SetPowerOnTime(wakeup_time); //RTC is in local time
		}
		/* always return count to avoid endless loop */
		ret = count;
	}
out:
	free_page((unsigned long)page);
	kfree(myString);
	dprintk(100, "%s <\n", __func__);
	return ret;
}

static int wakeup_time_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (NULL != page)
	{
		wakeup_time = YWPANEL_FP_GetPowerOnTime();
		len = sprintf(page, "%u\n", wakeup_time - rtc_offset);
//		len = sprintf(page, "%u\n", wakeup_time); // RTC uses local time?
	}
	return len;
}

static int was_timer_wakeup_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	int res = 0;
	YWPANEL_STARTUPSTATE_t State;

	if (NULL != page)
	{
		YWPANEL_FP_GetStartUpState(&State);
		if (State == YWPANEL_STARTUPSTATE_TIMER)
		{
			res = 1;
		}
		len = sprintf(page,"%d\n", res);
	}
	return len;
}

static int fp_version_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;
	int version;
	YWPANEL_Version_t fpanel_version;

	memset(&fpanel_version, 0, sizeof(YWPANEL_Version_t));

	if (YWPANEL_FP_GetVersion(&fpanel_version))
	{
		version = (int)(fpanel_version.swMajorVersion * 100 + fpanel_version.swSubVersion);
	}
	else
	{
		return -EFAULT;
	}

	len = sprintf(page, "%d\n", version);
	return len;
}

static int led_pattern_write(struct file *file, const char __user *buf, unsigned long count, void *data, int which)
{
	char *page;
	long pattern;
	int ret = -ENOMEM;

	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buf, count) == 0)
		{
			page[count - 1] = '\0';
			pattern = simple_strtol(page, NULL, 16);

#if defined(FP_LEDS)
			if (which == 1)
			{
				led1_pattern = pattern;
			}
			else
			{
				led0_pattern = pattern;
			}
#else
			led0_pattern = pattern;
#endif

//TODO: Not implemented completely; only the cases 0 and 0xffffffff (ever off/on) are handled
//Other patterns are blink patternd in time, so handling those should be done in a timer

			if (pattern == 0)
			{
				YWPANEL_FP_SetLed(which, 0);
			}
			else if (pattern == 0xffffffff)
			{
				YWPANEL_FP_SetLed(which, 1);
			}
			ret = count;
		}
		free_page((unsigned long)page);
	}
	return ret;
}

static int led0_pattern_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page, "%08x\n", led0_pattern);
	}
	return len;
}

static int led0_pattern_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	return led_pattern_write(file, buf, count, data, 0);
}

#if defined(FP_LEDS)
static int led1_pattern_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page, "%08x\n", led1_pattern);
	}
	return len;
}
static int led1_pattern_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	return led_pattern_write(file, buf, count, data, 1);
}
#endif

static int led_pattern_speed_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	int ret = -ENOMEM;

	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buf, count) == 0)
		{
			page[count - 1] = '\0';
			led_pattern_speed = (int)simple_strtol(page, NULL, 10);

			ret = count;
		}
		free_page((unsigned long)page);
	}
	return ret;
}

static int led_pattern_speed_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page, "%d\n", led_pattern_speed);
	}
	return len;
}

static int oled_brightness_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	long level;
	int ret = -ENOMEM;

	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buf, count) == 0)
		{
			page[count - 1] = '\0';

			level = simple_strtol(page, NULL, 10);
			aotomSetBrightness((int)level);
			ret = count;
		}
		free_page((unsigned long)page);
	}
	return ret;
}

static int power_standbyled_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	int ret = -ENOMEM;

	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buf, count) == 0)
		{
			page[count - 1] = '\0';

			if (strcmp(page, "on") == 0)
			{
				aotomEnableLed(0, 1);
			}
			else if (strcmp(page, "off") == 0)
			{
				aotomEnableLed(0, 0);
			}
			ret = count;
		}
		free_page((unsigned long)page);
	}
	return ret;
}

static int displaytype_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page,"%d\n", 1);  // always return VFD
	}
	return len;
}

static int timemode_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	int Mode;

	if (NULL != page)
	{
#if defined(FP_DVFD)
		if ((!bTimeMode && fp_type == FP_DVFD) || (fp_type == FP_LED))
		{
			Mode = 0;
		}
		else
		{
			Mode = 1;
		}
#else
		Mode = 1;
#endif
		len = sprintf(page,"%d\n", Mode);
	}
	return len;
}

#if defined(FP_DVFD)
static int timemode_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	int ret = -ENOMEM;
	int Mode;

	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buf, count) == 0)
		{
			page[count - 1] = '\0';

			if (fp_type == FP_DVFD)
			{
				if (simple_strtol(page, NULL, 10) != 0)
				{
					Mode = 1;
				}
				else
				{
					Mode = 0;
				}
				YWPANEL_FP_DvfdSetTimeMode(Mode);
			}
			ret = count;
		}
		free_page((unsigned long)page);
	}
	return ret;
}
#endif

#if 0
static int null_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	return count;
}
#endif

struct fp_procs
{
	char *name;
	read_proc_t *read_proc;
	write_proc_t *write_proc;
} fp_procs[] =
{
//	{ "progress", progress_read, progress_write },
	{ "stb/fp/aotom", NULL, aotom_write },
	{ "stb/fp/displaytype", displaytype_read, NULL },
	{ "stb/fp/led0_pattern", led0_pattern_read, led0_pattern_write },
#if defined(FP_LEDS)
	{ "stb/fp/led1_pattern", led1_pattern_read, led1_pattern_write },
#endif
	{ "stb/fp/led_pattern_speed", led_pattern_speed_read, led_pattern_speed_write },
	{ "stb/fp/oled_brightness", NULL, oled_brightness_write },
	{ "stb/fp/rtc", rtc_read, rtc_write },
	{ "stb/fp/rtc_offset", rtc_offset_read, rtc_offset_write },
	{ "stb/fp/text", NULL, text_write },
#if defined(FP_DVFD)
	{ "stb/fp/timemode", timemode_read, timemode_write },
#else
	{ "stb/fp/timemode", timemode_read, NULL },
#endif
	{ "stb/fp/wakeup_time", wakeup_time_read, wakeup_time_write },
	{ "stb/fp/was_timer_wakeup", was_timer_wakeup_read, NULL },
	{ "stb/fp/version", fp_version_read, NULL },
	{ "stb/lcd/symbol_circle", symbol_circle_read, symbol_circle_write },
	{ "stb/lcd/symbol_timeshift", symbol_timeshift_read, symbol_timeshift_write },
	{ "stb/power/standbyled", NULL, power_standbyled_write },
};

void create_proc_fp(void)
{
	int i;

	for (i = 0; i < sizeof(fp_procs) / sizeof(fp_procs[0]); i++)
	{
		install_e2_procs(fp_procs[i].name, fp_procs[i].read_proc, fp_procs[i].write_proc, NULL);
	}
}

void remove_proc_fp(void)
{
	int i;

	for (i = sizeof(fp_procs) / sizeof(fp_procs[0]) - 1; i >= 0; i--)
	{
		remove_e2_procs(fp_procs[i].name, fp_procs[i].read_proc, fp_procs[i].write_proc);
	}
}
// vim:ts=4
