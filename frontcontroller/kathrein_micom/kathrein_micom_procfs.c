/*
 * kathrein_micom_procfs.c
 *
 * (c) ? Gustav Gans
 * (c) 2015 Audioniek
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
 * procfs for Fortis front panel driver.
 *
 *
 ****************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------
 * 20170313 Audioniek       Initial version based on tffpprocfs.c.
 * 
 ****************************************************************************/

#include <linux/proc_fs.h>      /* proc fs */
#include <asm/uaccess.h>        /* copy_from_user */
#include <linux/time.h>
#include <linux/kernel.h>
#include "kathrein_micom.h"

/*
 *  /proc/stb/fp
 *             |
 *             +--- version (r)             SW version of front processor (hundreds = major, ten/units = minor)
 *             +--- rtc (rw)                RTC time (UTC, seconds since Unix epoch))
 *             +--- rtc_offset (rw)         RTC offset in seconds from UTC
 *             +--- wakeup_time (rw)        Next wakeup time (absolute, local, seconds since Unix epoch)
 *             +--- was_timer_wakeup (r)    Wakeup reason (1 = timer, 0 = other)
 *             +--- led0_pattern (rw)       Blink pattern for LED 1 (currently limited to on (0xffffffff) or off (0))
 *             +--- led1_pattern (rw)       Blink pattern for LED 2 (currently limited to on (0xffffffff) or off (0))
 *             +--- led_patternspeed (rw)   Blink speed for pattern (not implemented)
 *             +--- oled_brightness (w)     Direct control of display brightness
 *             +--- text (w)                Direct writing of display text
 */

/* from e2procfs */
extern int install_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc, void *data);
extern int remove_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc);

/* from other micom modules */
//extern int micomSetIcon(int which, int on);
//extern void VFD_set_all_icons(int onoff);
extern int micomWriteString(char *buf, size_t len);
extern int micomSetBrightness(int level);
extern void clear_display(void);
extern int micomGetTime(void);
extern int micomGetWakeUpMode(void);
extern int micomGetVersion(void);
extern int micomSetTime(char *time);
extern int micomSetLED(int which, int level);
//extern int micomGetWakeUpTime(char *time);
extern int micomSetWakeUpTime(char *time);

/* Globals */
static int rtc_offset = 3600;
//static int progress = 0;
//static int progress_done = 0;
static u32 led0_pattern = 0;
static u32 led1_pattern = 0;
static int led_pattern_speed = 20;

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
			ret = micomWriteString(page, count - 1);
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
int	tm_yday   	//days since January 1     0-365
}

time_t long seconds //UTC since epoch  
*/

time_t calcGetMicomTime(char *time)
{ //mjd hh:mm:ss -> seconds since epoch
	unsigned int    mjd     = ((time[1] & 0xff) * 256) + (time[2] & 0xff);
	unsigned long   epoch   = ((mjd - 40587) * 86400);
	unsigned int    hour    = time[3] & 0xff;
	unsigned int    min     = time[4] & 0xff;
	unsigned int    sec     = time[5] & 0xff;

	epoch += (hour * 3600 + min * 60 + sec);
	dprintk(10, "Converting time (MJD=) %d - %02d:%02d:%02d to %d seconds\n", (time[1] & 0xff) * 256 + (time[2] & 0xff),
				time[3], time[4], time[5], epoch);
	return epoch;
}

#define LEAPYEAR(year) (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year) (LEAPYEAR(year) ? 366 : 365)
static const int _ytab[2][12] =
{
	{ 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
	{ 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};

static struct tm * gmtime(register const time_t time)
{ //seconds since epoch -> struct tm
	static struct tm fptime;
	register unsigned long dayclock, dayno;
	int year = 70;

//	dprintk(10, "%s > Time to convert: %d seconds\n", __func__, (int)time);
	dayclock = (unsigned long)time % 86400;
	dayno = (unsigned long)time / 86400;

	fptime.tm_sec = dayclock % 60;
	fptime.tm_min = (dayclock % 3600) / 60;
	fptime.tm_hour = dayclock / 3600;
	fptime.tm_wday = (dayno + 4) % 7;       /* day 0 was a thursday */
	while (dayno >= YEARSIZE(year))
	{
		dayno -= YEARSIZE(year);
		year++;
	}
	fptime.tm_year = year;
	fptime.tm_yday = dayno;
	fptime.tm_mon = 0;
	while (dayno >= _ytab[LEAPYEAR(year)][fptime.tm_mon])
	{
		dayno -= _ytab[LEAPYEAR(year)][fptime.tm_mon];
		fptime.tm_mon++;
	}
	fptime.tm_mday = dayno;
//	fptime.tm_isdst = -1;
//	dprintk(10, "%s < Converted time: %02d:%02d:%02d %02d-%02d-%04d\n", __func__, fptime.tm_hour, fptime.tm_min, fptime.tm_sec, fptime.tm_mday, fptime.tm_mon + 1, fptime.tm_year + 1900);

	return &fptime;
}

int getMJD(struct tm *theTime)
{ //struct tm (date) -> mjd
	int mjd = 0;
	int year;

//	dprintk(10, "%s > Date to convert: %02d-%02d-%04d\n", __func__, theTime->tm_mday, theTime->tm_mon + 1 , theTime->tm_year + 1900);
	year  = theTime->tm_year - 1; // do not count current year
	while (year >= 70)
	{
		mjd += 365;
		if (LEAPYEAR(year))
		{
			mjd++;
		}
		year--;
	}
	mjd += theTime->tm_yday;
	mjd += 40587;  // mjd starts on midnight 17-11-1858 which is 40587 days before unix epoch
//	dprintk(10, "%s < MJD = %d\n", __func__, mjd);

	return mjd;
}

void calcSetMicomTime(time_t theTime, char *destString)
{ //seconds since epoch -> mjd h:m:s
	struct tm *now_tm;
	int mjd;

	now_tm = gmtime(theTime);
	mjd = getMJD(now_tm);
	destString[0] = (mjd >> 8);
	destString[1] = (mjd & 0xff);
	destString[2] = now_tm->tm_hour;
	destString[3] = now_tm->tm_min;
	destString[4] = now_tm->tm_sec;
//	dprintk(10, "Time to set: (MJD=) %d - %02d:%02d:%02d\n", (destString[0] & 0xff) * 256 + (destString[1] & 0xff),
//				destString[2], destString[3], destString[4]);
}

static int read_rtc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	int res;
	long rtc_time;

	res = micomGetTime();
//	dprintk(10, "Frontpanel time: (MJD=) %d - %02d:%02d:%02d (UTC)\n", (ioctl_data[1] & 0xff) * 256 + (ioctl_data[2] & 0xff),
//				ioctl_data[3], ioctl_data[4], ioctl_data[5]);
	rtc_time = calcGetMicomTime(ioctl_data);

	if (NULL != page)
	{
//		len = sprintf(page,"%u\n", (int)(rtc_time - rtc_offset));
		len = sprintf(page,"%u\n", (int)(rtc_time)); //Micom FP is in UTC
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
	char time[5];

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
//			calcSetMicomTime((argument + rtc_offset), time); //set time as u32
			calcSetMicomTime((argument), time); //set time as u32 (Micom FP is in UTC)
			ret = micomSetTime(time);
		}
		/* always return count to avoid endless loop */
		ret = count;
	}
out:
	free_page((unsigned long)page);
	kfree(myString);
	dprintk(10, "%s <\n", __func__);
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
	dprintk(10, "%s <\n", __func__);
	return ret;
}

static int wakeup_time_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char *page = NULL;
	ssize_t ret = -ENOMEM;
	time_t wakeup_time = 0;
	int test = -1;
	char *myString = kmalloc(count + 1, GFP_KERNEL);
	char wtime[5];

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
		dprintk(10, "%s > %s\n", __func__, myString);

		test = sscanf(myString, "%u", (unsigned int *)&wakeup_time);

		if (0 < test)
		{
//			calcSetMicomTime((wakeup_time + rtc_offset), wtime); //set time as u32
			calcSetMicomTime((wakeup_time), wtime); //set time as u32 (Micom FP is in UTC)
			ret = micomSetWakeUpTime(wtime);
		}
		/* always return count to avoid endless loop */
		ret = count;
	}
out:
	free_page((unsigned long)page);
	kfree(myString);
	dprintk(10, "%s <\n", __func__);
	return ret;
}

#if 0
static int wakeup_time_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	char wtime[5];
	int	w_time;

	if (NULL != page)
	{
		
		micomGetWakeUpTime(wtime);
		w_time = calcGetMicomTime(wtime);

//		len = sprintf(page, "%u\n", w_time - rtc_offset);
		len = sprintf(page, "%u\n", w_time); //Micom FP uses UTC
	}
	return len;
}
#endif

static int was_timer_wakeup_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	int res = 0;
	int wakeup_mode = -1;

	if (NULL != page)
	{
		res = micomGetWakeUpMode();

		if (res == 0)
		{
			dprintk(10, "%s > wakeup_mode= %02x\n", __func__, ioctl_data[1] & 0xff);
			if (ioctl_data[1] & 0xff == 0xc3) // if timer wakeup
			{
				wakeup_mode = 1;
			}
			else
			{
				wakeup_mode = 0;
			}
		}
		len = sprintf(page,"%d\n", wakeup_mode);
	}
	return len;
}

static int fp_version_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;

	if (micomGetVersion() == 0)
	{
		len = sprintf(page, "%d\n", (ioctl_data[1] * 10) | ioctl_data[2]);
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

//TODO: Not implemented completely; only the cases 0 and 0xffffffff (ever off/on) are handled
//Other patterns are blink patterned in time, so handling those should be done in a timer

			if (pattern == 0)
			{
				micomSetLED(which + 2, 0);
			}
			else if (pattern == 0xffffffff)
			{
				micomSetLED(which + 2, 1);
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
	{ "stb/fp/rtc", read_rtc, write_rtc },
	{ "stb/fp/rtc_offset", read_rtc_offset, write_rtc_offset },
	{ "stb/fp/led0_pattern", led0_pattern_read, led0_pattern_write },
	{ "stb/fp/led1_pattern", led1_pattern_read, led1_pattern_write },
	{ "stb/fp/led_pattern_speed", led_pattern_speed_read, led_pattern_speed_write },
	{ "stb/fp/oled_brightness", NULL, oled_brightness_write },
	{ "stb/fp/text", NULL, text_write },
//	{ "stb/fp/wakeup_time", wakeup_time_read, wakeup_time_write },
	{ "stb/fp/wakeup_time", NULL, wakeup_time_write },
	{ "stb/fp/was_timer_wakeup", was_timer_wakeup_read, NULL },
	{ "stb/fp/version", fp_version_read, NULL },
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
