/*
 * fortis_4g_fp_procfs.c
 *
 * Frontpanel driver for Fortis 4th generation receivers, procfs part
 *
 * Based on code written by:
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add nuvoton irq handling
 *
 * (c) 2014-2020 Audioniek: rewritten for Fortis 4th generation receivers
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
 * 20191202 Audioniek       Start of work, based on nuvoton_profs.c.
 * 20191208 Audioniek       FOREVER_3434HD added.
 * 20191229 Audioniek       /proc/stb/info/model_name added.
 *
 ****************************************************************************************/
#include <linux/proc_fs.h>      /* proc fs */
#include <asm/uaccess.h>        /* copy_from_user */
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include "fortis_4g_fp.h"
#include "fortis_4g_names.h"

#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[fortis_4g_fp_proc_fs] "

/*
 *  /proc/------
 *             |
 *             +--- progress (rw)           Progress of E2 startup in %
 *
 *  /proc/stb/info
 *             |
 *             +--- model_name (r)          Actual Fortis reseller name
 *
 *  /proc/stb/fp
 *             |
 *             +--- version (r)             SW version of boot loader (hundreds = major, tens/units = minor)
 *             +--- resellerID (r)          resellerID in boot loader
 *             +--- rtc (rw)                RTC time (UTC, seconds since Unix epoch))
 *             +--- rtc_offset (rw)         RTC offset in seconds from UTC
 *             +--- wakeup_time (rw)        Next wakeup time (absolute, local, seconds since Unix epoch)
 *             +--- was_timer_wakeup (r)    Wakeup reason (1 = timer, 0 = other)
 *             +--- led0_pattern (rw)       Blink pattern for LED 1 (currently limited to on (0xffffffff) or off (0))
 *             +--- led1_pattern (rw)       Blink pattern for LED 2 (currently limited to on (0xffffffff) or off (0))
 *             +--- led_patternspeed (rw)   Blink speed for pattern (not implemented)
 *             +--- oled_brightness (w)     Direct control of display brightness
 *             +--- text (w)                Direct writing of display text
 *
 */

/* from e2procfs */
extern int install_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc, void *data);
extern int remove_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc);

/* from other fortis_4g modules */
// extern int fortis_4gWriteString(char *buf, size_t len);
// #if defined(....)  All models can control display brightness
// extern int fortis_4gSetBrightness(int level);
// #endif

/* Globals */
static int progress = 0;
#if defined(FOREVER_3434HD) \
 || defined(FOREVER_9898HD) \
 || defined(GPV8000) \
 || defined(EP8000) \
 || defined(EPP8000)
// static int symbol_circle = 0;
static int old_icon_state;
#endif
static u32 led0_pattern = 0;
static u32 led1_pattern = 0;
static int led_pattern_speed = 20;


static int progress_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char* page;
	char* myString;
	ssize_t ret = -ENOMEM;

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
		len = sprintf(page,"%d", progress);
	}
	return len;
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
			ret = fortis_4gWriteString(page, count);
			if (ret >= 0)
			{
				ret = count;
			}
		}
		free_page((unsigned long)page);
	}
	return ret;
}

static int fp_version_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;
	unsigned int data[2];

	if (fortis_4gGetVersion(data) == 0)
	{
		len = sprintf(page, "%d.%02d\n", (int)data[0] / 100, (int)data[0] % 100);
	}
	else
	{
		return -EFAULT;
	}
	return len;
}

static int fp_reseller_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;
	unsigned int data[2];

	if (fortis_4gGetVersion(data) == 0)
	{
		len = sprintf(page, "%08X\n", (int)data[1]);
	}
	else
	{
		return -EFAULT;
	}
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

			if (which == 1)
			{
				led1_pattern = pattern;
			}
			else
			{
				led0_pattern = pattern;
			}

// TODO: Not implemented completely; only the cases 0 and 0xffffffff (ever off/on) are handled
// Other patterns are blink patterned in time, so handling those should be done in a thread

			if (pattern == 0)
			{
				fortis_4gSetLED(which + 1, 0);
			}
			else if (pattern == 0xffffffff)
			{
				fortis_4gSetLED(which+ 1 , 1);
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

// All models can control display brightness
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
			fortis_4gSetBrightness((int)level);
			ret = count;
		}
		free_page((unsigned long)page);
	}
	return ret;
}

static int model_name_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;
	unsigned int data[2];
	unsigned char reseller1;
	unsigned char reseller2;
	unsigned char reseller3;
	unsigned char **table = NULL;
	
	if (fortis_4gGetVersion(data) == 0)
	{
		reseller1 = data[1] >> 24;
		reseller2 = (data[1] >> 16) & 0xff;
		reseller3 = (data[1] >> 8) & 0xff;

		switch ((reseller1 << 8) + reseller3)
		{
			case 0x2900:  // FOREVER_NANOSMART
			{
				table = FOREVER_NANOSMART_table;
				break;
			}
			case 0x2901:  // DP7001
			{
				table = DP7001_table;
				break;
			}
			case 0x2902:  // FOREVER_9898HD
			default:
			{
				table = FOREVER_9898HD_table;
				break;
			}
			case 0x2903:  // FOREVER_3434HD
			{
				table = FOREVER_3434HD_table;
				break;
			}
			case 0x2910:  // FOREVER_2424HD
			{
				table = FOREVER_2424HD_table;
				break;
			}
			case 0x2A00:  // EPP8000
			{
				table = EPP8000_table;
				break;
			}
			case 0x2A01:  // EP8000
			{
				if ((data[1] & 0xff) == 0x01)  // evaluate 4th reseller byte
				{
					table = EP8000_1_table;
				}
				else
				{
					table = EP8000_0_table;
				}
				break;
			}
			case 0x2920:  // GPV8000
			{
				table = GPV8000_table;
				break;
			}
		}
	}
	if (NULL != page)
	{
		len = sprintf(page, "%s\n", table[reseller2]);
	}
	return len;
}

/*
static int null_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	return count;
}
*/

struct fp_procs
{
	char *name;
	read_proc_t *read_proc;
	write_proc_t *write_proc;
} fp_procs[] =
{
	{ "progress", progress_read, progress_write },
	{ "stb/info/model_name", model_name_read, NULL },
#if defined USE_RTC
	{ "stb/fp/rtc", read_rtc, write_rtc },
	{ "stb/fp/rtc_offset", read_rtc_offset, write_rtc_offset },
#endif
	{ "stb/fp/led0_pattern", led0_pattern_read, led0_pattern_write },
	{ "stb/fp/led1_pattern", led1_pattern_read, led1_pattern_write },
	{ "stb/fp/led_pattern_speed", led_pattern_speed_read, led_pattern_speed_write },
	{ "stb/fp/oled_brightness", NULL, oled_brightness_write },
	{ "stb/fp/text", NULL, text_write },
//	{ "stb/fp/wakeup_time", wakeup_time_read, wakeup_time_write },
//	{ "stb/fp/was_timer_wakeup", was_timer_wakeup_read, NULL },
	{ "stb/fp/version", fp_version_read, NULL },
	{ "stb/fp/resellerID", fp_reseller_read, NULL },
//#endif
};

void create_proc_fp(void)
{
	int i;

	for (i = 0; i < sizeof(fp_procs)/sizeof(fp_procs[0]); i++)
	{
		install_e2_procs(fp_procs[i].name, fp_procs[i].read_proc, fp_procs[i].write_proc, NULL);
	}
}

void remove_proc_fp(void)
{
	int i;

	for (i = sizeof(fp_procs)/sizeof(fp_procs[0]) - 1; i >= 0; i--)
	{
		remove_e2_procs(fp_procs[i].name, fp_procs[i].read_proc, fp_procs[i].write_proc);
	}
}
// vim:ts=4
