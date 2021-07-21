/*************************************************************************
 *
 * ufs910_fp_procfs.c
 *
 * 19. Jul 2021 - Audioniek
 *
 * Kathrein UFS910 VFD Kernel module - procfs part
 *
 * Replaces boxtype.ko
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
 *************************************************************************
 *
 * Frontpanel procfs driver for Kathrein UFS910
 *
 *************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * -----------------------------------------------------------------------
 * 20210719 Audioniek       Initial version.
 *
 *************************************************************************
 */
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <asm/uaccess.h>  /* copy_from_user */
#include "ufs910_fp.h"

/*
 *  /proc/------
 *             |
 *             +--- progress (rw)           Progress of E2 startup in %
 *
 *  /proc/stb/power
 *             |
 *             +--- vfd (rw)                Front panel display on/off
 *
 *  /proc/stb/fp
 *             |
 *             +--- model_name (r)          Reseller name, based on resellerID
 *             +--- led0_pattern (rw)       Blink pattern for LED 1 (currently limited to on (0xffffffff) or off (0))
 *             +--- led1_pattern (rw)       Blink pattern for LED 2 (currently limited to on (0xffffffff) or off (0))
 *             +--- led_patternspeed (rw)   Blink speed for pattern (not implemented)
 *             +--- oled_brightness (w)     Direct control of display brightness
 *             +--- text (w)                Direct writing of display text
 *
 *  /proc/stb/info
 *             |
 *             +--- OEM (r)                 Name of OEM manufacturer
 *             +--- brand (r)               Name of reseller
 *             +--- model_name (r)          Model name of reseller
 *
 */

extern int boxtype;  // ufs910 14W: 1 or 3, ufs910 1W: 0

/* from e2procfs */
extern int install_e2_procs(char *path, read_proc_t *read_func, write_proc_t *write_func, void *data);
extern int remove_e2_procs(char *path, read_proc_t *read_func, write_proc_t *write_func);

/* from other ufs910_fp modules */
extern int ufs910_fp_SetLED(int which, int on);
extern int ufs910_fp_WriteString(unsigned char *Buf, int len);
extern int ufs910_fp_SetBrightness(int level);
extern int ufs910_fp_SetDisplayOnOff(int on);

/* Globals */
static int progress = 0;
static int vfd_on = 1;
static u32 led0_pattern = 0;
static u32 led1_pattern = 0;
//static u32 led2_pattern = 0;
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

static int vfd_onoff_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char* page;
	ssize_t ret = -ENOMEM;
	char* myString;
	int i = 0;

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

		sscanf(myString, "%d", &vfd_on);
		kfree(myString);

		vfd_on = (vfd_on == 0 ? 0 : 1);
		ret = ufs910_fp_SetDisplayOnOff((char)vfd_on);
		if (ret)
		{
			goto out;
		}
		/* always return count to avoid endless loop */
		ret = count;
		goto out;
	}

out:
	free_page((unsigned long)page);
	return ret;
}

static int vfd_onoff_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page,"%d", lastdata.display_on);
	}
	return len;
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
			ret = ufs910_fp_WriteString(page, count);
			if (ret >= 0)
			{
				ret = count;
			}
		}
		free_page((unsigned long)page);
	}
	return ret;
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

//TODO: Not implemented completely; only the cases 0 and 0xffffffff (ever off/on) are handled
//Other patterns are blink patterned in time, so handling those should be done in a thread

			if (pattern == 0)
			{
				ufs910_fp_SetLED(which + 1, 0);
			}
			else if (pattern == 0xffffffff)
			{
				ufs910_fp_SetLED(which + 1 , 1);
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
			ufs910_fp_SetBrightness((int)level);
			ret = count;
		}
		free_page((unsigned long)page);
	}
	return ret;
}

static int oem_name_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page, "Marusys\n");
	}
	return len;
}

static int brand_name_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page, "Kathrein\n");
	}
	return len;
}

static int model_name_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page, "UFS910\n");
	}
	return len;
}

int proc_boxtype_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int      len = 0;
	unsigned int value;
	
	dprintk(100, "%s >\n", __func__);

	len = sprintf(page, "%d\n", boxtype);
	return len;
}

struct e2_procs
{
	char         *name;
	read_proc_t  *read_proc;
	write_proc_t *write_proc;
} ufs910_e2_procs[] =
{
	{ "boxtype", proc_boxtype_read, NULL },
	{ "progress", progress_read, progress_write },
	{ "stb/fp/led0_pattern", led0_pattern_read, led0_pattern_write },
	{ "stb/fp/led1_pattern", led1_pattern_read, led1_pattern_write },
	{ "stb/fp/led_pattern_speed", led_pattern_speed_read, led_pattern_speed_write },
	{ "stb/fp/oled_brightness", NULL, oled_brightness_write },
	{ "stb/power/vfd", vfd_onoff_read, vfd_onoff_write },
	{ "stb/fp/text", NULL, text_write },
	{ "stb/info/OEM", oem_name_read, NULL },
	{ "stb/info/brand", brand_name_read, NULL },
	{ "stb/info/model_name", model_name_read, NULL },
};

void create_proc_fp(void)
{
	int i;

	for (i = 0; i < sizeof(ufs910_e2_procs) / sizeof(ufs910_e2_procs[0]); i++)
	{
		install_e2_procs(ufs910_e2_procs[i].name, ufs910_e2_procs[i].read_proc, ufs910_e2_procs[i].write_proc, NULL);
	}
}

void remove_proc_fp(void)
{
	int i;

	for (i = sizeof(ufs910_e2_procs) / sizeof(ufs910_e2_procs[0]) - 1; i >= 0; i--)
	{
		remove_e2_procs(ufs910_e2_procs[i].name, ufs910_e2_procs[i].read_proc, ufs910_e2_procs[i].write_proc);
	}
}
// vim:ts=4
