/*
 * vitamin__micom_procfs.c
 *
 * (c) ? Gustav Gans
 * (c) 2019 Audioniek
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
 * procfs for CubeRevo front panel driver.
 *
 *
 ****************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------
 * 20190519 Audioniek       Initial version based on nuvoton_procfs.c.
 * 
 ****************************************************************************/

#include <linux/proc_fs.h>  /* proc fs */
#include <asm/uaccess.h>   /* copy_from_user */
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include "vitamin_micom.h"

/*
 *  /proc/------
 *             |
 *             +--- progress (rw)           Progress of E2 startup in %
 *
 *  /proc/stb/fp
 *             |
 *             +--- version (r)             SW version of front panel (hundreds = major, tens/units = minor)
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
 *  /proc/stb/lcd/
 *             |
 *             +--- symbol_circle (rw)      Control of spinner
 */

/* from e2procfs */
extern int install_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc, void *data);
extern int remove_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc);

/* from other micom modules */
extern int micomWriteString(char *buf, size_t len);
extern int micomSetBrightness(int level);
extern int micomSetIcon(int which, int on);
extern int micomSetBLUELED(int on);

/* Globals */
static int progress = 0;
static int symbol_circle = 0;
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

static int symbol_circle_write(struct file *file, const char __user *buf, unsigned long count, void *data)
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
		myString = (char*)kmalloc(count + 1, GFP_KERNEL);
		strncpy(myString, page, count);
		myString[count - 1] = '\0';

		sscanf(myString, "%d", &symbol_circle);
		kfree(myString);

		micomSetIcon(ICON_CIRCLE, symbol_circle & 0x0f);

		/* always return count to avoid endless loop */
		ret = count;
		goto out;
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
		len = sprintf(page,"%d", symbol_circle);
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
			ret = micomWriteString(page, count);
			if (ret >= 0)
			{
				ret = count;
			}
		}
		free_page((unsigned long)page);
	}
	return ret;
}

#if 0
/* Time subroutines */
/*
struct tm {
int	tm_sec      //seconds after the minute 0-61*
int	tm_min      //minutes after the hour   0-59
int	tm_hour     //hours since midnight     0-23
int	tm_mday     //day of the month         1-31
int	tm_mon      //months since January     0-11
int	tm_year     //years since 1900
int	tm_wday     //days since Sunday        0-6
int	tm_yday     //days since January 1     0-365
}

time_t long seconds //UTC since epoch  
*/

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

time_t calcGetMicomTime(char *time)
{ //  6 byte string YMDhms -> seconds since epoch
	time_t time_local;
	int year, mon, day, days;
	int hour, min, sec;

	year = ((time[5] >> 4) * 10) + (time[5] & 0x0f);
	mon  = ((time[4] >> 4) * 10) + (time[4] & 0x0f);
	day  = ((time[3] >> 4) * 10) + (time[3] & 0x0f);
	hour = ((time[2] >> 4) * 10) + (time[2] & 0x0f);
	min  = ((time[1] >> 4) * 10) + (time[1] & 0x0f);
	sec  = ((time[0] >> 4) * 10) + (time[0] & 0x0f);

	// calculate the number of days since linux epoch
	days = date2days(year, mon, day, 0);
	time_local += days * 86400;
	// add the time
	time_local += (hour * 3600);
	time_local += (min * 60);
	time_local += sec;
	return time_local;
}

void calcSetMicomTime(time_t theTime, char *destString)
{ // seconds since epoch -> 12 byte string YYMMDDhhmmss
	struct tm calc_tm;
	register unsigned long dayclock, dayno;
	int year = 1970; // linux epoch

	dayclock = (unsigned long)theTime % 86400;
	dayno = (unsigned long)theTime / 86400;
	//dprintk(1, "%s dayclock: %u, dayno: %u\n", __func__, (int)dayclock, (int)dayno);

	calc_tm.tm_sec = dayclock % 60;
	calc_tm.tm_min = (dayclock % 3600) / 60;
	calc_tm.tm_hour = dayclock / 3600;
	calc_tm.tm_wday = (dayno + 4) % 7;  // day 0 was a thursday

	// calculate the year
	while (dayno >= YEARSIZE(year))
	{
		dayno -= YEARSIZE(year);
		year++;
	}
	calc_tm.tm_year = year - 2000; // RTC starts at 01-01-2000
	calc_tm.tm_yday = dayno;

	// calculate the month
	calc_tm.tm_mon = 0;
	while (dayno >= _ytab[LEAPYEAR(year)][calc_tm.tm_mon])
	{
		dayno -= _ytab[LEAPYEAR(year)][calc_tm.tm_mon];
		calc_tm.tm_mon++;
	}
	calc_tm.tm_mon++;
	calc_tm.tm_mday++;
	destString[0] = (calc_tm.tm_year >> 4) + '0';   // tens
	destString[1] = (calc_tm.tm_year & 0x0f) + '0'; // units
	destString[2] = (calc_tm.tm_mon >> 4) + '0';
	destString[3] = (calc_tm.tm_mon & 0x0f) + '0';
	destString[4] = (calc_tm.tm_mday >> 4) + '0';
	destString[5] = (calc_tm.tm_mday & 0x0f) + '0';
	destString[6] = (calc_tm.tm_hour >> 4) + '0';
	destString[7] = (calc_tm.tm_hour & 0x0f) + '0';
	destString[8] = (calc_tm.tm_min >> 4) + '0';
	destString[9] = (calc_tm.tm_min & 0x0f) + '0';
	destString[10] = (calc_tm.tm_sec >> 4) + '0';
	destString[11] = (calc_tm.tm_sec & 0x0f) + '0';
}

static int read_rtc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	int res;
	long rtc_time;
	char fptime[6];

	memset(fptime, 0, sizeof(fptime));
	res = micomGetTime(fptime);  // get front panel clock time (local)
	rtc_time = calcGetMicomTime(fptime) - rtc_offset;  // return UTC

	if (NULL != page)
	{
		len = sprintf(page,"%u\n", (int)(rtc_time));
	}
	return len;
}

static int write_rtc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char *page = NULL;
	ssize_t ret = -ENOMEM;
	time_t argument = 0;
	int test = -1;
	char *myString = kmalloc(count + 1, GFP_KERNEL);
	char time[12];

	dprintk(50, "%s >\n", __func__);
	memset(time, 0, sizeof(time));
	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buffer, count))
		{
			goto out;
		}
		strncpy(myString, page, count);
		myString[count] = '\0';

		test = sscanf (myString, "%u", (unsigned int *)&argument);

		if (test > 0)
		{
			calcSetMicomTime((argument + rtc_offset), time);
			ret = micomSetTime(time); // update front panel clock in local time
		}
		/* always return count to avoid endless loop */
		ret = count;
	}

out:
	free_page((unsigned long)page);
	kfree(myString);
	return ret;
}

static int read_rtc_offset(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page, "%d\n", rtc_offset);
	}
	return len;
}

static int write_rtc_offset(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char *page = NULL;
	ssize_t ret = -ENOMEM;
	int test = -1;
	char *myString = kmalloc(count + 1, GFP_KERNEL);

	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;
		if (copy_from_user(page, buffer, count))
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
	return ret;
}
#endif

#if 0
static int wakeup_time_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char *page = NULL;
	ssize_t ret = -ENOMEM;
	time_t wakeup_time = 0;
	int test = -1;
	char *myString = kmalloc(count + 1, GFP_KERNEL);
	char wtime[12];

	memset(wtime, 0, sizeof(wtime));
	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buffer, count))
		{
			goto out;
		}
		strncpy(myString, page, count);
		myString[count] = '\0';

		test = sscanf(myString, "%u", (unsigned int *)&wakeup_time);

		if (0 < test)
		{
////			calcSetMicomTime((wakeup_time + rtc_offset), wtime);  //set FP wake up time (local)
//			calcSetMicomTime((wakeup_time), wtime);  //set FP wake up time (UTC)
//			ret = micomSetWakeUpTime(wtime);
		}
		/* always return count to avoid endless loop */
		ret = count;
	}

out:
	free_page((unsigned long)page);
	kfree(myString);
//	dprintk(10, "%s <\n", __func__);
	return ret;
}

static int wakeup_time_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	char wtime[5];
	int	w_time = 0;

	if (NULL != page)
	{
		
//		micomGetWakeUpTime(wtime);  // get wakeup time from FP clock (local)
////		w_time = calcGetMicomTime(wtime) - rtc_offset;  // UTC
//		w_time = calcGetMicomTime(wtime);  // UTC

		len = sprintf(page, "%u\n", w_time);
	}
	return len;
}
#endif

#if 0
static int was_timer_wakeup_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	int res = 0;
	int wakeup_mode = -1;

	dprintk(50, "%s >\n", __func__);
	if (NULL != page)
	{
//		res = micomGetWakeUpMode(&wakeup_mode);
		res = micomGetWakeUpMode();
		if (res == 0)
		{
			if (wakeup_mode == 2) // if timer wakeup
			{
				wakeup_mode = 1;
			}
			else
			{
				wakeup_mode = 0;
			}
		}
		else
		{
				wakeup_mode = -1;
		}
		len = sprintf(page,"%d\n", wakeup_mode);
	}
	return len;
}
#endif

#if 0
static int fp_version_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{ // TODO: return string with year
	int len = 0;
	int version = -1;

	switch (front_seg_num)
	{
		case 12:
		{
			version = 0;
			break;
		}
		case 13:
		{
			version = 1;
			break;
		}
		case 14:
		{
			version = 2;
			break;
		}
		case 4:
		{
			version = 3;
			break;
		}
		default:
		{
			return -EFAULT;
		}
	}
	len = sprintf(page, "%d\n", version);
	return len;
}
#endif

static int led_pattern_write(struct file *file, const char __user *buf, unsigned long count, void *data, int which)
{
	char *page;
	long pattern;
	int val;
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

//TODO: Not implemented completely; only the cases 0 (off) and 0xffffffff (on)
// are handled
// Other patterns are blink patterned in time, so handling those should be done in a thread

			switch (pattern)
			{
				case 0xffffffff:
				{
					val = 1;
					break;
				}
				default:
				case 0:
				{
					val = 0;
					break;
				}
			}
			micomSetBLUELED(val);
		}
		ret = count;
	}
	free_page((unsigned long)page);
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
			micomSetBrightness((int)level);
			ret = count;
		}
		free_page((unsigned long)page);
	}
	return ret;
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
	{ "stb/fp/led0_pattern", led0_pattern_read, led0_pattern_write },
	{ "stb/fp/led1_pattern", led1_pattern_read, led1_pattern_write },
	{ "stb/fp/led_pattern_speed", led_pattern_speed_read, led_pattern_speed_write },
	{ "stb/fp/oled_brightness", NULL, oled_brightness_write },
	{ "stb/fp/text", NULL, text_write },
	{ "stb/lcd/symbol_circle", symbol_circle_read, symbol_circle_write }
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
