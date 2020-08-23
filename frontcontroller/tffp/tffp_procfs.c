#include <linux/proc_fs.h>      /* proc fs */
#include <asm/uaccess.h>        /* copy_from_user */
#include <linux/time.h>
#include <linux/kernel.h>
#include "frontpanel.h"

/*
/proc/stb/fp/version", "r"
/proc/stb/fp/wakeup_time", "w"
/proc/stb/fp/wakeup_time", "r"
/proc/stb/fp/was_timer_wakeup", "r"
/proc/stb/fp/was_timer_wakeup", "w"
/proc/stb/fp/oled_brightness", "w"
/proc/stb/fp/vcr_fns", "r"
/proc/stb/fp/events", "r"
/proc/stb/fp/rtc", "w"
/proc/stb/fp/rtc", "r"
/proc/stb/fp/lnb_sense%d", m_slotid);
*/
/*
 *  /proc/stb/fp
 *             |
 *             +--- events (ro)
 *             |
 *             +--- lnb_sense1
 *             |
 *             +--- lnb_sense2
 *             |
 *             +--- led0_pattern (w)
 *             |
 *             +--- led_pattern_speed (w)
 *             |
 *             +--- oled_brightness (w)
 *             |
 *             +--- rtc (rw)
 *             |
 *             +--- vcr_fns
 *             |
 *             +--- version
 *             |
 *             +--- wakeup_time (rw) <- dbox frontpanel wakeuptime
 *             |
 *             +--- was_timer_wakeup (rw)
 *
 *  /proc/stb/lcd/
 *             |
 *             +--- symbol_circle (rw)       Control of spinner
 *             |
 *             +--- symbol_timeshift (rw)    Control of timeshift icon
 */

extern int install_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc, void *data);
extern int remove_e2_procs(char *name, read_proc_t *read_proc, write_proc_t *write_proc);

extern void SetIconBits(byte Reg, byte Bit, byte Mode);

static int symbol_circle = 0;
static int symbol_timeshift = 0;
#if 0
static int lnb_sense1_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	printk("%s %ld\n", __FUNCTION__, count);

	return count;
}

static int lnb_sense1_read (char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	printk("%s %d\n", __FUNCTION__, count);

	return 0;
}

static int lnb_sense2_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	printk("%s %ld\n", __FUNCTION__, count);

	return count;
}

static int lnb_sense2_read (char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	printk("%s %d\n", __FUNCTION__, count);

	return 0;
}
#endif

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

		Spinner_on = (symbol_circle == 0 ? 0 : 1);

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
		len = sprintf(page,"%d", symbol_circle);
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
		
		SetIconBits(27, 1, symbol_timeshift == 0 ? 0 : 0x0f);

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
		len = sprintf(page,"%d", symbol_timeshift);
	}
	return len;
}

static int led0_pattern_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	int ret = -ENOMEM;

	printk("%s(%ld, ", __FUNCTION__, count);

	page = (char *)__get_free_page(GFP_KERNEL);
	if (page)
	{
		ret = -EFAULT;
		if (copy_from_user(page, buf, count) == 0)
		{
			page[count] = 0;
			printk("%s", page);
			ret = count;
		}
		free_page((unsigned long)page);
	}
	printk(")\n");

	return ret;
}

static int led_pattern_speed_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	int ret = -ENOMEM;

	printk("%s(%ld, ", __FUNCTION__, count);

	page = (char *)__get_free_page(GFP_KERNEL);
	if (page)
	{
		ret = -EFAULT;
		if (copy_from_user(page, buf, count) == 0)
		{
			page[count] = 0;
			printk("%s", page);
			ret = count;
		}
		free_page((unsigned long)page);
	}
	printk(")\n");

	return ret;
}

static int version_read (char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;
	len = sprintf(page, "0\n");

	printk("%s %d\n", __FUNCTION__, count);

	return len;
}

static int wakeup_time_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	int ret = -ENOMEM;
	long seconds;

	//printk("%s(%ld, ", __FUNCTION__, count);

	page = (char *)__get_free_page(GFP_KERNEL);
	if (page)
	{
		ret = -EFAULT;
		if (copy_from_user(page, buf, count) == 0)
		{
			page[count] = 0;
			seconds = simple_strtol(page, NULL, 0);
			vfdSetGmtWakeupTime(seconds);
			ret = count;
		}
		free_page((unsigned long)page);
	}
	//printk(")\n");

	return ret;
}

static int was_timer_wakeup_read (char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;
	int boot_reason = getBootReason();

	len = sprintf(page, "%d\n", (boot_reason == 2) ? 1 : 0);

	//printk("%s %d\n", __FUNCTION__, count);

	return len;
}

static int rtc_read (char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;

	len = sprintf(page, "%ld\n", vfdGetGmtTime());

	printk("%s %d\n", __FUNCTION__, count);

	return len;
}

static int rtc_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	time_t seconds;
	int ret = -ENOMEM;

	//printk("%s(%ld, ", __FUNCTION__, count);

	page = (char *)__get_free_page(GFP_KERNEL);
	if (page)
	{
		ret = -EFAULT;
		if (copy_from_user(page, buf, count) == 0)
		{
			page[count] = 0;
			//printk("%s", page);
			seconds = simple_strtol(page, NULL, 0);
			vfdSetGmtTime(seconds);
			ret = count;
		}
		free_page((unsigned long)page);
	}
	//printk(")\n");

	return ret;
}

struct fp_procs
{
	char *name;
	read_proc_t *read_proc;
	write_proc_t *write_proc;
} fp_procs[] =
{
	{ "stb/fp/rtc", rtc_read, rtc_write },
	{ "stb/fp/version", version_read, NULL },
	{ "stb/fp/wakeup_time", NULL, wakeup_time_write },
	{ "stb/fp/was_timer_wakeup", was_timer_wakeup_read, NULL },
	{ "stb/fp/led0_pattern", NULL, led0_pattern_write },
	{ "stb/fp/led_pattern_speed", NULL, led_pattern_speed_write },
	{ "stb/lcd/symbol_circle", symbol_circle_read, symbol_circle_write },
	{ "stb/lcd/symbol_timeshift", symbol_timeshift_read, symbol_timeshift_write }
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

