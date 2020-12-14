/*
 * nuvoton_procfs.c
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
 * 20160523 Audioniek       Initial version based on tffpprocfs.c.
 * 20170206 Audioniek       /proc/stb/fp/resellerID added.
 * 20170207 Audioniek       /proc/stb/fp/resellerID and /proc/stb/fp/version
 *                          were reversed.
 * 20170415 Audioniek       /proc/progress handling added.
 * 20170417 Audioniek       /proc/stb/fp/rtc behaviour changed. It is now
 *                          in UTC as Enigma2 expects that. Front panel clock
 *                          is maintained in local time to ensure correct
 *                          front panel clock display when in deep standby.
 * 20181211 Audioniek       E2 startup display removed in anticipation of
 *                          adding it to E2 itself.
 * 20190102 Audioniek       /proc/stb/lcd/symbol_circle support added.
 * 20191202 Audioniek       /proc/stb/lcd/symbol_timeshift support added.
 * 
 ****************************************************************************/

#include <linux/proc_fs.h>      /* proc fs */
#include <asm/uaccess.h>        /* copy_from_user */
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include "nuvoton.h"
#include "fortis_names.h"

/*
 *  /proc/------
 *             |
 *             +--- progress (rw)           Progress of E2 startup in %
 *
 *  /proc/stb/fp
 *             |
 *             +--- model_name (r)          Reseller name, based on resellerID
 *
 *  /proc/stb/fp
 *             |
 *             +--- version (r)             SW version of boot loader (hundreds = major, ten/units = minor)
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
 *  /proc/stb/lcd/
 *             |
 *             +--- symbol_circle (rw)       Control of spinner (FORTIS_HDBOX & ATEVIO7500 only)
 *             +--- symbol_timeshift (rw)    Control of timeshift icon (FORTIS_HDBOX & ATEVIO7500 only)
 */

/* from e2procfs */
extern int install_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc, void *data);
extern int remove_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc);

/* from other nuvoton modules */
extern int nuvotonWriteString(char *buf, size_t len);
#if !defined(HS7110)
extern int nuvotonSetBrightness(int level);
#endif
extern void clear_display(void);
extern int nuvotonGetTime(char *time);
extern int nuvotonGetWakeUpMode(int *wakeup_mode);
extern int nuvotonGetVersion(unsigned int *data);
extern int nuvotonSetTime(char *time);
extern int nuvotonSetLED(int which, int level);
//extern int nuvotonSetIcon(int which, int on);
extern int nuvotonGetWakeUpTime(char *time);
extern int nuvotonSetWakeUpTime(char *time);

/* Globals */
static int progress = 0;
#if defined(FORTIS_HDBOX) \
 || defined(ATEVIO7500)
static int symbol_circle = 0;
static int timeshift = 0;
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

#if defined(FORTIS_HDBOX) \
 || defined(ATEVIO7500)
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
		myString = (char*) kmalloc(count + 1, GFP_KERNEL);
		strncpy(myString, page, count);
		myString[count - 1] = '\0';

		sscanf(myString, "%d", &symbol_circle);
		kfree(myString);

		if (symbol_circle != 0)
		{  // spinner on
			if (symbol_circle > 255)
			{
				symbol_circle = 255;  // set maximum value
			}
			if (spinner_state.state == 0)  // if spinner not active
			{
#if defined(ATEVIO7500)
				if (icon_state.state != 0)
				{  // stop icon thread if active
					old_icon_state = icon_state.state;
//					dprintk(50, "%s Stop icon thread\n", __func__);
					i = 0;
					icon_state.state = 0;
					do
					{
						msleep(250);
						i++;
					}
					while (icon_state.status != ICON_THREAD_STATUS_HALTED && i < 4);  // time out of 1 second
//					dprintk(50, "%s Icon thread stopped\n", __func__);
				}
				if (symbol_circle == 1)  // handle special value 1
				{
					spinner_state.period = 250;  // set standard speed
				}
				else
				{
					spinner_state.period = symbol_circle * 10;  // set user specified speed
				}
#else  // (FORTIS_HDBOX)
				if (symbol_circle == 1)  // handle special value 1
				{
					spinner_state.period = 1000;
				}
				else
				{
					spinner_state.period = symbol_circle * 40;  // set user specified speed
				}
#endif
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
#if defined(ATEVIO7500)
//				dprintk(50, "%s Stop spinner thread\n", __func__);
				i = 0;
				do
				{
					msleep(250);
					i++;
				}
				while (spinner_state.status != ICON_THREAD_STATUS_HALTED && i < 4);  // time out of 1 second
//				dprintk(50, "%s Spinner thread stopped\n", __func__);
#endif
			}
#if defined(ATEVIO7500)
			if (old_icon_state != 0)  // restart icon thread when it was active
			{
				icon_state.state = old_icon_state;
				up(&icon_state.sem);
			}
#endif
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
		len = sprintf(page,"%d", symbol_circle);
	}
	return len;
}

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
		ret = nuvotonSetIcon(ICON_TIMESHIFT, timeshift);
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
			ret = nuvotonWriteString(page, count);
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

time_t calcGetNuvotonTime(char *time)
{ //mjd hh:mm:ss -> seconds since epoch
	unsigned int    mjd     = ((time[0] & 0xff) * 256) + (time[1] & 0xff);
	unsigned long   epoch   = ((mjd - 40587) * 86400);
	unsigned int    hour    = time[2] & 0xff;
	unsigned int    min     = time[3] & 0xff;
	unsigned int    sec     = time[4] & 0xff;

	epoch += (hour * 3600 + min * 60 + sec);
	return epoch;
}

#define LEAPYEAR(year) (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year) (LEAPYEAR(year) ? 366 : 365)
static const int _ytab[2][12] =
{
	{ 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
	{ 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};

static struct tm *gmtime(register const time_t time)
{ //seconds since epoch -> struct tm
	static struct tm fptime;
	register unsigned long dayclock, dayno;
	int year = 70;

//	dprintk(5, "%s < Time to convert: %d seconds\n", __func__, (int)time);
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

	return &fptime;
}

int getMJD(struct tm *theTime)
{ //struct tm (date) -> mjd
	int mjd = 0;
	int year;

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

	return mjd;
}

void calcSetNuvotonTime(time_t theTime, char *destString)
{ //seconds since epoch -> mjd h:m:s
	struct tm *now_tm;
	int mjd;

//	dprintk(10, "%s > Time to calcluate: %d %02d:%02d:%02d\n", __func__, (int)theTime);
	now_tm = gmtime(theTime);
	mjd = getMJD(now_tm);
	destString[0] = (mjd >> 8);
	destString[1] = (mjd & 0xff);
	destString[2] = now_tm->tm_hour;
	destString[3] = now_tm->tm_min;
	destString[4] = now_tm->tm_sec;
//	dprintk(10, "%s < Calculated time: MJD=%d %02d:%02d:%02d\n", __func__, mjd, destString[2], destString[3], destString[4]);

}

static int read_rtc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	int res;
	long rtc_time;
	char fptime[5];

	res = nuvotonGetTime(fptime);  // get front panel clock time (local)
	rtc_time = calcGetNuvotonTime(fptime) - rtc_offset;  // return UTC

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
			calcSetNuvotonTime((argument + rtc_offset), time);
			ret = nuvotonSetTime(time); // update front panel clock in local time
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

		test = sscanf(myString, "%u", (unsigned int *)&wakeup_time);

		if (0 < test)
		{
			calcSetNuvotonTime((wakeup_time + rtc_offset), wtime);  //set FP wake up time (local)
			ret = nuvotonSetWakeUpTime(wtime);
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
		
		nuvotonGetWakeUpTime(wtime);  // get wakeup time from FP clock (local)
		w_time = calcGetNuvotonTime(wtime) - rtc_offset;  // UTC

		len = sprintf(page, "%u\n", w_time);
	}
	return len;
}

static int was_timer_wakeup_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	int res = 0;
	int wakeup_mode = -1;

	if (NULL != page)
	{
		res = nuvotonGetWakeUpMode(&wakeup_mode);

		if (res == 0)
		{
			if (wakeup_mode == 3) // if timer wakeup
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
	unsigned int data[2];

	if (nuvotonGetVersion(data) == 0)
	{
		len = sprintf(page, "%d\n", (int)data[0]);
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

	if (nuvotonGetVersion(data) == 0)
	{
		dprintk(1, "%s resellerID = %08X\n", __func__, data[1]);
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

//TODO: Not implemented completely; only the cases 0 and 0xffffffff (ever off/on) are handled
//Other patterns are blink patterned in time, so handling those should be done in a timer

			if (pattern == 0)
			{
				nuvotonSetLED(which + 1, 0);
			}
			else if (pattern == 0xffffffff)
			{
#if defined(FORTIS_HDBOX) || defined(ATEVIO7500)
				nuvotonSetLED(which + 1, 31);
#else
				nuvotonSetLED(which+ 1 , 7);
#endif
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

#if !defined(HS7110)
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
			nuvotonSetBrightness((int)level);
			ret = count;
		}
		free_page((unsigned long)page);
	}
	return ret;
}
#endif

static int model_name_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;
	unsigned int data[2];
	unsigned char reseller1;
	unsigned char reseller2;
	unsigned char reseller3;
	unsigned char reseller4;
	unsigned char **table = NULL;
	
	len = nuvotonGetVersion(data);
	if (len == 0)
	{
		reseller1 = data[1] >> 24;
		reseller2 = (data[1] >> 16) & 0xff;
		reseller3 = (data[1] >> 8) & 0xff;
		reseller4 = (data[1]) & 0xff;

		dprintk(1,"Found resellerID %02x%02x%02x%02x\n", reseller1, reseller2, reseller3, reseller4);
		switch ((reseller1 << 8) + reseller3)
		{
			case 0x2000:  // FS9000
			{
				table = FS9000_table;
				break;
			}
			case 0x2001:  // FS9200
			{
				table = FS9200_table;
				break;
			}
			case 0x2003:  // HS9510
			{
				if (reseller4 == 0x03)  // evaluate 4th reseller byte
				{
					table = HS9510_3_table;
				}
				else if (reseller4 == 0x02)  // evaluate 4th reseller byte
				{
					table = HS9510_2_table;
				}
				else if (reseller4 == 0x01)  // evaluate 4th reseller byte
				{
					table = HS9510_1_table;
				}
				else
				{
					table = HS9510_0_table;
				}
				break;
			}
			case 0x2006:  // ??
			{
				table = FS9000_06_table;
				break;
			}
			case 0x2300:  // HS8200
			default:
			{
				table = HS8200_table;
				break;
			}
			case 0x2500:  // HS7420
			{
				table = HS7420_table;
				break;
			}
			case 0x2502:  // HS7110
			{
				table = HS7110_table;
				break;
			}
			case 0x2503:  // HS7810A
			{
				table = HS7810A_table;
				break;
			}
			case 0x2700:  // HS7119
			{
				table = HS7119_table;
				break;
			}
			case 0x2701:  // HS7119_01
			{
				table = HS7119_01_table;
				break;
			}
			case 0x2710:  // HS7119_10
			{
				table = HS7119_10_table;
				break;
			}
			case 0x2720:  // HS7819
			{
				table = HS7819_table;
				break;
			}
			case 0x2730:  // HS7429
			{
				table = HS7429_table;
				break;
			}
		}
	}
	else
	{
		dprintk(1, "Get version failed (ret = %d).\n", len);
		return -1;
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
	{ "stb/fp/rtc", read_rtc, write_rtc },
	{ "stb/fp/rtc_offset", read_rtc_offset, write_rtc_offset },
	{ "stb/fp/led0_pattern", led0_pattern_read, led0_pattern_write },
	{ "stb/fp/led1_pattern", led1_pattern_read, led1_pattern_write },
	{ "stb/fp/led_pattern_speed", led_pattern_speed_read, led_pattern_speed_write },
#if !defined(HS7110)
	{ "stb/fp/oled_brightness", NULL, oled_brightness_write },
#else
	{ "stb/fp/oled_brightness", NULL, NULL },
#endif
	{ "stb/fp/text", NULL, text_write },
	{ "stb/fp/wakeup_time", wakeup_time_read, wakeup_time_write },
	{ "stb/fp/was_timer_wakeup", was_timer_wakeup_read, NULL },
	{ "stb/fp/version", fp_version_read, NULL },
	{ "stb/fp/resellerID", fp_reseller_read, NULL },
#if defined(FORTIS_HDBOX) \
 || defined(ATEVIO7500)
	{ "stb/lcd/symbol_circle", symbol_circle_read, symbol_circle_write },
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
