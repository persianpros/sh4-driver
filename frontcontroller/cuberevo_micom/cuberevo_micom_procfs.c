/*
 * cuberevo_micom_procfs.c
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
 * 20190215 Audioniek       Initial version based on nuvoton_procfs.c.
 * 20210522 Audioniek       Version number is now the actual number read from
 *                          the front processor.
 * 20210423 Audioniek       /proc/stb/lcd/symbol_timeshift support added.
 * 20210606 Audioniek       /proc/stb/lcd/symbol_circle support added.
 * 20210606 Audioniek       /proc/stb/power support added.
 * 20210704 Audioniek       /proc/stb/fp_fan_pwm added (CubeRevo
 *                          & 9500HD only).
 * 
 ****************************************************************************/

#include <linux/proc_fs.h>      /* proc fs */
#include <asm/uaccess.h>        /* copy_from_user */
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include "cuberevo_micom.h"

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
 *             +--- version (r)             SW version of front panel
 *             +--- fan (rw)                Control of fan (on/off) (CUBEREVO & 9500HD only)
 *             +--- fan_pwm (rw)            Control of fan speed (CUBEREVO & 9500HD only)
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
 *             +--- symbol_circle (rw)       Control of spinner (CUBEREVO & 9500HD only)
 *             +--- symbol_timeshift (rw)    Control of timeshift icon (CUBEREVO, MINI, MINI2, 2000HD, 3000HD & 9500HD only)
 */

/* from e2procfs */
extern int install_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc, void *data);
extern int remove_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc);

/* from other micom modules */
extern int micomWriteString(char *buf, size_t len, int center_flag);
extern int micomSetBrightness(int level);
extern void clear_display(void);
extern int micomGetTime(char *time);
extern int micomSetTime(char *time);
extern int micomSetLED(int level);
extern int micomGetWakeUpTime(char *time);
extern int micomSetWakeUpTime(char *time);
extern int micomGetWakeUpMode(int *wakeup_mode);
extern int micomSetFan(int on);
extern int micomSetIcon(int which, int on);
extern int micomSetDisplayOnOff(char level);
//extern int micomGetVersion(unsigned int *data);
//extern static int rtc_time2tm(char *uTime, struct rtc_time *tm);

/* version of fp */
extern int micom_ver, micom_major, micom_minor;


/* Globals */
static int progress = 0;
static u32 led0_pattern = 0;
static u32 led1_pattern = 0;
static int led_pattern_speed = 20;
static int vfd_on = 1;
#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD) \
 || defined(CUBEREVO) \
 || defined(CUBEREVO_9500)
static int timeshift = 0;
#endif
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
static int fan_pwm = 128;
static int spinner_speed = 0;
#endif

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

#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
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

		sscanf(myString, "%d", &spinner_speed);
		kfree(myString);

		if (spinner_speed != 0)
		{  // spinner on
			if (spinner_speed > 255)
			{
				spinner_speed = 255;  // set maximum value
			}
			if (spinner_state.state == 0)  // if spinner not active
			{
				if (spinner_speed == 1)  // handle special value 1
				{
					spinner_state.period = 1000;
				}
				else
				{
					spinner_state.period = spinner_speed * 40;  // set user specified speed
				}
				spinner_state.state = 1;
				lastdata.icon_state[ICON_SPINNER] = 1;
				up(&spinner_state.sem);
			}
		}
		else
		{  // spinner off
			if (spinner_state.state != 0)
			{
				spinner_state.state = 0;
				lastdata.icon_state[ICON_SPINNER] = 0;
			}
		}
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
		len = sprintf(page,"%d", spinner_speed);
	}
	return len;
}
#endif

#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD) \
 || defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
static int timeshift_write(struct file *file, const char __user *buf, unsigned long count, void *data)
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

		sscanf(myString, "%d", &timeshift);
		kfree(myString);

		timeshift = (timeshift == 0 ? 0 : 1);
		ret = micomSetIcon(ICON_TIMESHIFT, timeshift);
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

static int timeshift_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (NULL != page)
	{
		len = sprintf(page,"%d", timeshift);
	}
	return len;
}
#endif

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
		ret = micomSetDisplayOnOff((char)vfd_on);
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
#if defined(CENTERED_DISPLAY)
			ret = micomWriteString(page, count, 1);
#else
			ret = micomWriteString(page, count, 0);
#endif
			if (ret >= 0)
			{
				ret = count;
			}
		}
		free_page((unsigned long)page);
	}
	return ret;
}

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

#define LEAPYEAR(year) (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year) (LEAPYEAR(year) ? 366 : 365)
static const int _ytab[2][12] =
{
	{ 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
	{ 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};

int date2days(int year, int mon, int day)
{
	int days;

//	dprintk(100, "%s >\n", __func__);
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
	// process the remaining years
	year--; // do not count current year, as it is done already
	while (year >= 0)
	{
		days += YEARSIZE(year);
		year--;
	}
	days += 10957; // add days since linux epoch
//	dprintk(100, "%s < (days = %d)\n", __func__, days);
	return days;
}

time_t calcGetMicomTime(char *time)
{ //  6 byte string YMDhms -> seconds since epoch
	time_t time_local;
	int year, mon, day, days;
	int hour, min, sec;

//	dprintk(100, "%s >\n", __func__);
	year = ((time[5] >> 4) * 10) + (time[5] & 0x0f);
	mon  = ((time[4] >> 4) * 10) + (time[4] & 0x0f);
	day  = ((time[3] >> 4) * 10) + (time[3] & 0x0f);
	hour = ((time[2] >> 4) * 10) + (time[2] & 0x0f);
	min  = ((time[1] >> 4) * 10) + (time[1] & 0x0f);
	sec  = ((time[0] >> 4) * 10) + (time[0] & 0x0f);

//	dprintk(10, "%s Time/date: %02d:%02d:%02d %02d-%02d-20%02d\n", __func__, hour, min, sec, day, mon, year);
	// calculate the number of days since linux epoch
	days = date2days(year, mon, day);
//	dprintk(10, "%s days = %d\n", __func__, days);
	time_local += days * 86400;
//	dprintk(10, "%s time_local = %d\n", __func__, time_local);
	// add the time
	time_local += (hour * 3600);
	time_local += (min * 60);
	time_local += sec;
//	dprintk(100, "%s < (time_local = %d seconds)\n", __func__, time_local);
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
	dprintk(1, "Time/date: %02x:%02x:%02x %02x-%02x-20%02x\n", fptime[2], fptime[1], fptime[0], fptime[3], fptime[4], fptime[5]);
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
			calcSetMicomTime((wakeup_time + rtc_offset), wtime);  //set FP wake up time (local)
			ret = micomSetWakeUpTime(wtime);
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
	int	w_time;

	if (NULL != page)
	{
		
		micomGetWakeUpTime(wtime);  // get wakeup time from FP clock (local)
		w_time = calcGetMicomTime(wtime) - rtc_offset;  // UTC

		len = sprintf(page, "%u\n", w_time);
	}
	return len;
}

static int was_timer_wakeup_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	int res = 0;
	int wakeup_mode = -1;

	dprintk(50, "%s >\n", __func__);
	if (NULL != page)
	{
		res = micomGetWakeUpMode(&wakeup_mode);

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

static int fp_version_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;

	len = sprintf(page, "%d.%02d.%02d\n", micom_ver, micom_major, micom_minor);
	return len;
}

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

//TODO: Not implemented completely; only the cases 0 (off), 0xffffffff (on),
// 0x0000ffff) or 0xffff0000 (slow blink) and 0x00ff00ff or 0xff00ff00 (fast blink)
// are handled
// Other patterns are blink patterned in time, so handling those should be done in a thread

			switch (pattern)
			{
				case 0xffffffff:
				{
					val = 1;
					break;
				}
				case 0x0000ffff:
				case 0xffff0000:
				{
					val = 3;
					break;
				}
				case 0x00ff00ff:
				case 0xff00ff00:
				{
					val = 5;
					break;
				}
				default:
				case 0:
				{
					val = 0;
					break;
				}
			}
			micomSetLED(val);
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

#if !defined(CUBEREVO_MINI_FTA) \
 && !defined(CUBEREVO_250HD)
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
#endif

#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
static int fan_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	char wtime[5];
	int	w_time;

	if (NULL != page)
	{
		
		len = sprintf(page, "%d\n", lastdata.fan);
	}
	return len;
}

static int fan_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	int on;
	int ret = -ENOMEM;

	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buf, count) == 0)
		{
			page[count - 1] = '\0';

			on = (simple_strtol(page, NULL, 10) == 0 ? 0 : 1);
			micomSetFan((int)on);
			ret = count;
		}
		free_page((unsigned long)page);
	}
	return ret;
}

static int fan_pwm_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	char wtime[5];
	int	w_time;

	// Note: On CubeRevo & 9500HD the fan can only be switched on or off;
    //       the frontprocessor however automatically controls the speed.
	//       This functions always returns the value written before, although
    //       this does not control the actual fan speed.
	if (NULL != page)
	{
		
		len = sprintf(page, "%d\n", fan_pwm);
	}
	return len;
}

static int fan_pwm_write(struct file *file, const char __user *buf, unsigned long count, void *data)
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

			// Note: On CubeRevo & 9500HD the fan can only be switched on or off;
		    //       the frontprocessor however automatically controls the speed.
			//       This functions always returns the value written before, although
		    //       this does not control the actual fan speed.
			fan_pwm = simple_strtol(page, NULL, 10);
			micomSetFan(fan_pwm == 0 ? 0 : 1);
			ret = count;
		}
		free_page((unsigned long)page);
	}
	return ret;
}
#endif

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
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
	{ "stb/fp/fan", fan_read, fan_write },
	{ "stb/fp/fan_pwm", fan_pwm_read, fan_pwm_write },
#endif
	{ "stb/fp/rtc", read_rtc, write_rtc },
	{ "stb/fp/rtc_offset", read_rtc_offset, write_rtc_offset },
	{ "stb/fp/led0_pattern", led0_pattern_read, led0_pattern_write },
	{ "stb/fp/led1_pattern", led1_pattern_read, led1_pattern_write },
	{ "stb/fp/led_pattern_speed", led_pattern_speed_read, led_pattern_speed_write },
#if !defined(CUBEREVO_MINI_FTA) \
 && !defined(CUBEREVO_250HD)
	{ "stb/fp/oled_brightness", NULL, oled_brightness_write },
#endif
	{ "stb/fp/text", NULL, text_write },
	{ "stb/fp/wakeup_time", wakeup_time_read, wakeup_time_write },
	{ "stb/fp/was_timer_wakeup", was_timer_wakeup_read, NULL },
	{ "stb/fp/version", fp_version_read, NULL },
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
	{ "stb/lcd/symbol_circle", symbol_circle_read, symbol_circle_write },
#endif
	{ "stb/power/vfd", vfd_onoff_read, vfd_onoff_write },
#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD) \
 || defined(CUBEREVO) \
 || defined(CUBEREVO_9500)
	{ "stb/lcd/symbol_timeshift", timeshift_read, timeshift_write }
#endif
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
