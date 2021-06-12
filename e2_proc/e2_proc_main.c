/*
 * e2_proc_main.c
 *
 * (c) 2009 teamducktales
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
 */

/*
 * Description:
 *
 * progress
 *  |
 * bus
 *  |
 *  ----------- nim_sockets
 *  |
 * stb
 *  |
 *  ----------- audio
 *  |             |
 *  |             --------- ac3
 *  |             |
 *  |             --------- audio_delay_pcm
 *  |             |
 *  |             --------- audio_delay_bitstream
 *  |             |
 *  |             --------- audio_j1_mute
 *  |             |
 *  |             --------- ac3_choices
 *  |             |
 *  |             ---------
 *  |
 *  ----------- video
 *  |             |
 *  |             --------- alpha
 *  |             |
 *  |             --------- aspect
 *  |             |
 *  |             --------- aspect_choices
 *  |             |
 *  |             --------- hdmi_colorspace
 *  |             |
 *  |             --------- hdmi_colorspace_choices
 *  |             |
 *  |             --------- force_dvi
 *  |             |
 *  |             --------- policy
 *  |             |
 *  |             --------- policy_choices
 *  |             |
 *  |             --------- videomode
 *  |             |
 *  |             --------- videomode_50hz
 *  |             |
 *  |             --------- videomode_60hz
 *  |             |
 *  |             --------- videomode_choices
 *  |             |
 *  |             --------- videomode_preferred
 *  |             |
 *  |             --------- pal_v_start
 *  |             |
 *  |             --------- pal_v_end
 *  |             |
 *  |             --------- pal_h_start
 *  |             |
 *  |             --------- pal_h_end
 *  |
 *  ---------- avs
 *  |           |
 *  |           --------- 0
 *  |               |
 *  |               --------- colorformat <-colorformat in generlell, hdmi and scart
 *  |               |
 *  |               --------- colorformat_choices
 *  |               |
 *  |               --------- fb <-fastblanking
 *  |               |
 *  |               --------- sb <-slowblanking
 *  |               |
 *  |               --------- volume
 *  |               |
 *  |               --------- input  <-Input, Scart VCR Input or Encoder
 *  |               |
 *  |               --------- input_choices
 *  |               |
 *  |               --------- standby
 *  |
 *  ---------- denc
 *  |           |
 *  |           --------- 0
 *  |               |
 *  |               --------- wss
 *  |               |
 *  |               ---------
 *  |               |
 *  |               ---------
 *  |
 *  ---------- info
 *  |           |
 *  |           --------- model_name
 *  |
 *  ---------- fp (this is wrong used for e2 I think. on dm800 this is frontprocessor and there is another proc entry for frontend)
 *  |           |
 *  |           --------- lnb_sense1
 *  |           |
 *  |           --------- lnb_sense2
 *  |           |
 *  |           --------- led0_pattern
 *  |           |
 *  |           --------- led_pattern_speed
 *  |           |
 *  |           --------- version
 *  |           |
 *  |           --------- wakeup_time <- dbox frontpanel wakeuptime
 *  |           |
 *  |           --------- was_timer_wakeup
 *  |
 *  |
 *  ---------- hdmi
 *  |           |
 *  |           --------- enable_hdmi_resets
 *  |           |
 *  |           --------- audio_source
 *  |           |
 *  |           ---------
 *  |
 *  ---------- info
 *  |           |
 *  |           --------- model <- Version String of out Box
 *  |           |
 *  |           --------- chipset <- Version String of chipset
 *  |
 *  ---------- tsmux
 *  |           |
 *  |           --------- input0
 *  |           |
 *  |           --------- input1
 *  |           |
 *  |           --------- ci0_input
 *  |           |
 *  |           --------- ci1_input
 *  |           |
 *  |           --------- lnb_b_input
 *  |           |
 *  |           ---------
 *  |
 *  ---------- misc
 *  |           |
 *  |           --------- 12V_output <- VIP1_V1, HL101, OPT9600, IPBOX9900 only
 *  |
 *  ---------- tuner (dagoberts tuner entry ;-) )
 *  |           |
 *  |           ---------
 *  |
 *  ---------- vmpeg
 *  |           |
 *  |           --------- 0/1
 *  |               |
 *  |               --------- dst_left   \
 *  |               |                     |
 *  |               --------- dst_top     |
 *  |               |                      >  PIG WINDOW SIZE AND POSITION
 *  |               --------- dst_width   |
 *  |               |                     |
 *  |               --------- dst_height /
 *  |               |
 *  |               --------- dst_all (Dagobert: Dont confuse player by setting value one after each other)
 *  |
 *  |               |TODO
 *  |               | v
 *  |               --------- yres
 *  |               |
 *  |               --------- xres
 *  |               |
 *  |               --------- framerate
 *  |               |
 *  |               --------- progressive
 *  |               |
 *  |               --------- aspect
 *  |               |
 *  |               --------- pep_apply
 *  |
 *  ---------- fb
 *  |           |
 *  |           --------- 3dmode
 *  |           |
 *  |           --------- znorm
 *  |           |
 *  |           --------- dst_left   \
 *  |           |                     |
 *  |           --------- dst_top     |
 *  |           |                      >  PIG WINDOW SIZE AND POSITION
 *  |           --------- dst_width   |
 *  |           |                     |
 *  |           --------- dst_height /
 *  |
 *  ---------- ir
 *  |           |
 *  |           --------- rc
 *  |                      |
 *  |                      --------- type <- Type number of remote control in use
 *  ---------- lcd
 *  |           |
 *  |           --------- scroll_repeats <- number of scrolls
 *  |           |
 *  |           --------- scroll_delay <- pause time between scrolls
 *  |           |
 *  |           --------- initial_scroll_delay <- wait time after initial display
 *  |           |
 *  |           --------- final_scroll_delay <- wait time before final display
 *  |           |
 *  |           --------- symbol_circle <- control for spinner (if spinner available)
 *  |           |
 *  |           --------- symbol_timeshift <- control for timeshift icon (if present)
 *
 */

#include <linux/proc_fs.h>  	/* proc fs */
#include <asm/uaccess.h>    	/* copy_from_user */

#include <linux/version.h>
#include <linux/string.h>
#include <linux/module.h>
#if defined(HL101) \
 || defined(VIP1_V1) \
 || defined(OPT9600)
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#  include <linux/stpio.h>
#else
#  include <linux/stm/pio.h>
#endif  // kernel version
#endif  // vip1_v1

typedef int (*proc_read_t)(char *page, char **start, off_t off, int count, int *eof, void *data_unused);
typedef int (*proc_write_t)(struct file *file, const char __user *buf, unsigned long count, void *data);

#define cProcDir	1
#define cProcEntry	2

// For 12V output
#if defined(HL101) \
 || defined(VIP1_V1)
struct stpio_pin *_12v_pin;
#endif

struct ProcStructure_s
{
	int   type;
	char *name;
	struct proc_dir_entry *entry;
	proc_read_t read_proc;
	proc_write_t write_proc;
	void *instance; /* needed for cpp stuff */
	void *identifier; /* needed for cpp stuff */
};

static int get_player_version(char *page, char **start, off_t off, int count, int *eof, void *data)
{
#if defined(PLAYER_191)
	int len = sprintf(page, "player191\n");
#elif defined(PLAYER_179)
	int len = sprintf(page, "player179\n");
#elif defined(PLAYER_131)
	int len = sprintf(page, "player131\n");
#else
	int len = sprintf(page, "unknown\n");
#endif
	return len;
}

static int info_model_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
#if defined(CUBEREVO)
	int len = sprintf(page, "cuberevo\n");
#elif defined(CUBEREVO_MINI)
	int len = sprintf(page, "cuberevo_mini\n");
#elif defined(CUBEREVO_MINI2)
	int len = sprintf(page, "cuberevo_mini2\n");
#elif defined(CUBEREVO_250HD)
	int len = sprintf(page, "cuberevo_250hd\n");
#elif defined(CUBEREVO_MINI_FTA)
	int len = sprintf(page, "cuberevo_mini_fta\n");
#elif defined(CUBEREVO_2000HD)
	int len = sprintf(page, "cuberevo_2000hd\n");
#elif defined(CUBEREVO_9500HD)
	int len = sprintf(page, "cuberevo_9500hd\n");
#elif defined(CUBEREVO_3000HD)
	int len = sprintf(page, "cuberevo_3000hd\n");
#elif defined(TF7700)
	int len = sprintf(page, "tf7700\n");
#elif defined(HL101)
	int len = sprintf(page, "hl101\n");
#elif defined(VIP1_V1)
	int len = sprintf(page, "vip1_v1\n");
#elif defined(VIP1_V2)
	int len = sprintf(page, "vip1_v2\n");
#elif defined(VIP2)
	int len = sprintf(page, "vip2\n");
#elif defined(UFS910)
	int len = sprintf(page, "ufs910\n");
#elif defined(UFS922)
	int len = sprintf(page, "ufs922\n");
#elif defined(UFC960)
	int len = sprintf(page, "ufc960\n");
#elif defined(UFS912)
	int len = sprintf(page, "ufs912\n");
#elif defined(UFS913)
	int len = sprintf(page, "ufs913\n");
#elif defined(SPARK)
	int len = sprintf(page, "spark\n");
#elif defined(SPARK7162)
	int len = sprintf(page, "spark7162\n");
#elif defined(FORTIS_HDBOX)
	int len = sprintf(page, "fortis_hdbox\n");
#elif defined(OCTAGON1008)
	int len = sprintf(page, "octagon1008\n");
#elif defined(ATEVIO7500)
	int len = sprintf(page, "atevio7500\n");
#elif defined(HS7110)
	int len = sprintf(page, "hs7110\n");
#elif defined(HS7119)
	int len = sprintf(page, "hs7119\n");
#elif defined(HS7420)
	int len = sprintf(page, "hs7420\n");
#elif defined(HS7429)
	int len = sprintf(page, "hs7429\n");
#elif defined(HS7810A)
	int len = sprintf(page, "hs7810a\n");
#elif defined(HS7819)
	int len = sprintf(page, "hs7819\n");
#elif defined(ATEMIO520)
	int len = sprintf(page, "atemio520\n");
#elif defined(ATEMIO530)
	int len = sprintf(page, "atemio530\n");
#elif defined(IPBOX9900)
	int len = sprintf(page, "ipbox9900\n");
#elif defined(IPBOX99)
	int len = sprintf(page, "ipbox99\n");
#elif defined(IPBOX55)
	int len = sprintf(page, "ipbox55\n");
#elif defined(ADB_BOX)
	int len = sprintf(page, "adb_box\n");
#elif defined(VITAMIN_HD5000)
	int len = sprintf(page, "vitamin_hd5000\n");
#elif defined(SAGEMCOM88)
	int len = sprintf(page, "sagemcom88\n");
#elif defined(ARIVALINK200)
	int len = sprintf(page, "arivalink200\n");
#elif defined(PACE7241)
	int len = sprintf(page, "pace7241\n");
#elif defined(OPT9600)
	int len = sprintf(page, "opt9600\n");
#elif defined(ADB_2850)
	int len = sprintf(page, "adb_2850\n");
#elif defined(QBOXHD)
	int len = sprintf(page, "qboxhd\n");
#elif defined(QBOXHD_MINI)
	int len = sprintf(page, "qboxhd_mini\n");
#elif defined(HOMECAST5101)
	int len = sprintf(page, "hs5101\n");
#elif defined(FOREVER_NANOSMART)
	int len = sprintf(page, "forever_nanosmart\n");
#elif defined(FOREVER_9898HD)
	int len = sprintf(page, "forever_9898hd\n");
#elif defined(DP7001)
	int len = sprintf(page, "dp7001\n");
#elif defined(FOREVER_3434HD)
	int len = sprintf(page, "forever_3434hd\n");
#elif defined(FOREVER_2424HD)
	int len = sprintf(page, "forever_2424hd\n");
#elif defined(GPV8000)
	int len = sprintf(page, "gpv8000\n");
#elif defined(EP8000)
	int len = sprintf(page, "ep8000\n");
#elif defined(EPP8000)
	int len = sprintf(page, "epp8000\n");
#else
	int len = sprintf(page, "unknown\n");
#endif
	return len;
}

static char *scroll_repeats = NULL;
static char *scroll_delay = NULL;
static char *initial_scroll_delay = NULL;
static char *final_scroll_delay = NULL;

static int info_scroll_repeats_read (char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (scroll_repeats == NULL)
	{
		len = sprintf(page, "0\n");
	}
	else
	{
		len = sprintf(page, scroll_repeats);
	}
	return len;
}

int info_scroll_repeats_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	ssize_t ret = -ENOMEM;

	char *myString = kmalloc(count + 1, GFP_KERNEL);
#ifdef VERY_VERBOSE
	printk("%s %ld - ", __func__, count);
#endif
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
#ifdef VERY_VERBOSE
		printk("%s\n", myString);
#endif
		if (scroll_repeats != NULL)
		{
			kfree(scroll_repeats);
		}
		scroll_repeats = myString;

		/* always return count to avoid endless loop */
		ret = count;
	}
out:
	free_page((unsigned long)page);

	if (scroll_repeats != myString)
	{
		kfree(myString);
	}
	return ret;
}

static int info_scroll_delay_read (char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (scroll_delay == NULL)
	{
		len = sprintf(page, "0\n");
	}
	else
	{
		len = sprintf(page, scroll_delay);
	}
	return len;
}

int info_scroll_delay_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	ssize_t ret = -ENOMEM;

	char *myString = kmalloc(count + 1, GFP_KERNEL);
#ifdef VERY_VERBOSE
	printk("%s %ld - ", __func__, count);
#endif
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
#ifdef VERY_VERBOSE
		printk("%s\n", myString);
#endif
		if (scroll_delay != NULL)
		{
			kfree(scroll_delay);
		}
		scroll_delay = myString;

		/* always return count to avoid endless loop */
		ret = count;
	}
out:
	free_page((unsigned long)page);

	if (scroll_delay != myString)
	{
		kfree(myString);
	}
	return ret;
}

static int info_initial_scroll_delay_read (char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (initial_scroll_delay == NULL)
	{
		len = sprintf(page, "0\n");
	}
	else
	{
		len = sprintf(page, initial_scroll_delay);
	}
	return len;
}

int info_initial_scroll_delay_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	ssize_t ret = -ENOMEM;

	char *myString = kmalloc(count + 1, GFP_KERNEL);
#ifdef VERY_VERBOSE
	printk("%s %ld - ", __func__, count);
#endif
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
#ifdef VERY_VERBOSE
		printk("%s\n", myString);
#endif
		if (initial_scroll_delay != NULL)
		{
			kfree(initial_scroll_delay);
		}
		initial_scroll_delay = myString;

		/* always return count to avoid endless loop */
		ret = count;
	}
out:
	free_page((unsigned long)page);

	if (initial_scroll_delay != myString)
	{
		kfree(myString);
	}
	return ret;
}

static int info_final_scroll_delay_read (char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (final_scroll_delay == NULL)
	{
		len = sprintf(page, "0\n");
	}
	else
	{
		len = sprintf(page, final_scroll_delay);
	}
	return len;
}

int info_final_scroll_delay_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	ssize_t ret = -ENOMEM;

	char *myString = kmalloc(count + 1, GFP_KERNEL);
#ifdef VERY_VERBOSE
	printk("%s %ld - ", __func__, count);
#endif
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
#ifdef VERY_VERBOSE
		printk("%s\n", myString);
#endif
		if (final_scroll_delay != NULL)
		{
			kfree(final_scroll_delay);
		}
		final_scroll_delay = myString;

		/* always return count to avoid endless loop */
		ret = count;
	}
out:
	free_page((unsigned long)page);

	if (final_scroll_delay != myString)
	{
		kfree(myString);
	}
	return ret;
}

static char *rc_type = NULL;

static int info_rctype_read (char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (rc_type == NULL)
	{
		len = sprintf(page, "0\n");
	}
	else
	{
		len = sprintf(page, rc_type);
	}
	return len;
}

int info_rctype_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	ssize_t ret = -ENOMEM;

	char *myString = kmalloc(count + 1, GFP_KERNEL);
#ifdef VERY_VERBOSE
	printk("%s %ld - ", __func__, count);
#endif
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
#ifdef VERY_VERBOSE
		printk("%s\n", myString);
#endif
		if (rc_type != NULL)
		{
			kfree(rc_type);
		}
		rc_type = myString;

		/* always return count to avoid endless loop */
		ret = count;
	}
out:
	free_page((unsigned long)page);

	if (rc_type != myString)
	{
		kfree(myString);
	}
	return ret;
}

static int info_chipset_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
#if defined(UFS910) \
 || defined(ADB_BOX)
	int len = sprintf(page, "STi7100\n");
#elif defined(ATEVIO7500) \
 || defined(UFS913) \
 || defined(SAGEMCOM88) \
 || defined(PACE7241)
	int len = sprintf(page, "STi7105\n");
#elif defined(FORTIS_HDBOX) \
 || defined(HL101) \
 || defined(OCTAGON1008) \
 || defined(TF7700) \
 || defined(UFS922) \
 || defined(UFC960) \
 || defined(VIP1_V1) \
 || defined(VIP1_V2) \
 || defined(VIP2) \
 || defined(CUBEREVO) \
 || defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_250HD) \
 || defined(CUBEREVO_MINI_FTA) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_9500HD) \
 || defined(CUBEREVO_3000HD) \
 || defined(IPBOX9900) \
 || defined(IPBOX99) \
 || defined(IPBOX55) \
 || defined(ARIVALINK200) \
 || defined(OPT9600)
	int len = sprintf(page, "STi7109\n");
#elif defined(UFS912) \
 || defined(HS7110) \
 || defined(HS7810A) \
 || defined(HS7420) \
 || defined(HS7429) \
 || defined(HS7119) \
 || defined(HS7819) \
 || defined(ATEMIO520) \
 || defined(ATEMIO530) \
 || defined(SPARK) \
 || defined(VITAMIN_HD5000) \
 || defined(ADB_2850)
	int len = sprintf(page, "STi7111\n");
#elif defined(SPARK7162)
	int len = sprintf(page, "STi7162\n");
#elif defined(FOREVER_NANOSMART) || defined(FOREVER_9898HD) || defined(DP7001) || defined(FOREVER_2424HD) || defined(FOREVER_3434HD)
	int len = sprintf(page, "STiH237 (Cardiff)\n");
#elif defined(EP8000) || defined(EPP8000)
	int len = sprintf(page, "STiH239 (Newport)\n");
#elif defined(GPV8000)
	int len = sprintf(page, "STiH253 (Firenze)\n");
#else
	int len = sprintf(page, "unknown\n");
#endif
	return len;
}

static char *three_d_mode = NULL;

static int three_d_mode_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (three_d_mode == NULL)
	{
		len = sprintf(page, "off\n");
	}
	else
	{
		len = sprintf(page, three_d_mode);
	}

	return len;
}

static int three_d_mode_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	ssize_t ret = -ENOMEM;

	char *myString = kmalloc(count + 1, GFP_KERNEL);
#ifdef VERY_VERBOSE
	printk("%s %ld - ", __func__, count);
#endif
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
#ifdef VERY_VERBOSE
		printk("%s\n", myString);
#endif
		if (strncmp("sbs", myString, 3) == 0 || strncmp("sidebyside", myString, 10) == 0)
		{
			if (three_d_mode != NULL)
			{
				kfree(three_d_mode);
			}
			three_d_mode = myString;
		}
		else if (strncmp("tab", myString, 3) == 0 || strncmp("topandbottom", myString, 12) == 0)
		{
			if (three_d_mode != NULL)
			{
				kfree(three_d_mode);
			}
			three_d_mode = myString;
		}
		else if (strncmp("off", myString, 3) == 0 || strncmp("auto", myString, 4) == 0)
		{
			if (three_d_mode != NULL)
			{
				kfree(three_d_mode);
			}
			three_d_mode = myString;
		}

		/* always return count to avoid endless loop */
		ret = count;
	}
out:
	free_page((unsigned long)page);

	if (three_d_mode != myString)
	{
		kfree(myString);
	}
	return ret;
}

static char *wakeup_time = NULL;

static int wakeup_time_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (wakeup_time == NULL)
	{
		len = sprintf(page, "%ld", LONG_MAX);
	}
	else
	{
		len = sprintf(page, wakeup_time);
	}
	return len;
}

static int wakeup_time_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	ssize_t ret = -ENOMEM;

	char *myString = kmalloc(count + 1, GFP_KERNEL);
#ifdef VERY_VERBOSE
	printk("%s %ld - ", __func__, count);
#endif
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
#ifdef VERY_VERBOSE
		printk("%s\n", myString);
#endif
		if (wakeup_time != NULL)
		{
			kfree(wakeup_time);
		}
		wakeup_time = myString;

		/* always return count to avoid endless loop */
		ret = count;
	}
out:
	free_page((unsigned long)page);

	if (wakeup_time != myString)
	{
		kfree(myString);
	}
	return ret;
}

#if defined(IPBOX9900) \
 || defined(HL101) \
 || defined(VIP1_V1)
int _12v_isON = 0;

void set_12v(int onoff)
{
#if defined(HL101) \
 || defined(VIP1_V1)
	if (onoff)
	{
		stpio_set_pin(_12v_pin, 1);
	}
	else
	{
		stpio_set_pin(_12v_pin, 0);
	}
#endif
}

int proc_misc_12V_output_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char *page;
	ssize_t ret = -ENOMEM;
	char *myString;
#ifdef VERY_VERBOSE
	printk("%s %ld\n", __func__, count);
#endif
	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buf, count))
		{
			goto out;
		}
		page[count] = 0;
		//printk("%s", page);

		myString = (char *) kmalloc(count + 1, GFP_KERNEL);
		strncpy(myString, page, count);
		myString[count] = '\0';

		if (!strncmp("on", myString, count))
		{
			_12v_isON = 1;
		}
		if (!strncmp("off", myString, count))
		{
			_12v_isON = 0;
		}
		kfree(myString);
		ret = count;
	}
#if defined(HL101) \
 || defined(VIP1_V1)
//	set_12v(_12v_isON);  // set 12V output
#endif
	ret = count;

out:
	free_page((unsigned long)page);
	return ret;
}
EXPORT_SYMBOL(_12v_isON);

int proc_misc_12V_output_read(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	int len = 0;
#ifdef VERY_VERBOSE
	printk("%s %d\n", __func__, count);
#endif

	if (_12v_isON)
	{
		len = sprintf(page, "on\n");
	}
	else
	{
		len = sprintf(page, "off\n");
	}
	return len;
}
#endif  // IPBOX9900, VIP1_V1, HL101

static int zero_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = sprintf(page, "0");

	return len;
}

static int default_write_proc(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	return count;
}

struct ProcStructure_s e2Proc[] =
{
#if defined(UFS910)
	{cProcEntry, "boxtype",                                                          NULL, NULL, NULL, NULL, ""},
#endif
	{cProcEntry, "progress",                                                         NULL, NULL, NULL, NULL, ""},

	{cProcEntry, "bus/nim_sockets",                                                  NULL, NULL, NULL, NULL, ""},
	{cProcDir,   "stb",                                                              NULL, NULL, NULL, NULL, ""},
	{cProcDir,   "stb/audio",                                                        NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/audio/ac3",                                                    NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/audio/audio_delay_pcm",                                        NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/audio/audio_delay_bitstream",                                  NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/audio/j1_mute",                                                NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/audio/ac3_choices",                                            NULL, NULL, NULL, NULL, ""},

	{cProcDir,   "stb/info",                                                         NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/info/model",                                                   NULL, info_model_read, NULL, NULL, ""},
	{cProcEntry, "stb/info/chipset",                                                 NULL, info_chipset_read, NULL, NULL, ""},
	{cProcEntry, "stb/info/boxtype",                                                 NULL, info_model_read, NULL, NULL, ""},
#if defined(ADB_BOX)
	{cProcEntry, "stb/info/adb_variant",                                             NULL, NULL, NULL, NULL, ""},
#endif
#if defined(FORTIS_HDBOX) \
 || defined(OCTAGON1008) \
 || defined(ATEVIO7500) \
 || defined(HS7110) \
 || defined(HS7119) \
 || defined(HS7420) \
 || defined(HS7429) \
 || defined(HS7810A) \
 || defined(HS7819) \
 || defined(SPARK) \
 || defined(SPARK7162) \
 || defined(FOREVER_NANOSMART) \
 || defined(FOREVER_9898HD) \
 || defined(DP7001) \
 || defined(FOREVER_3434HD) \
 || defined(FOREVER_2424HD) \
 || defined(GPV8000) \
 || defined(EP8000) \
 || defined(EPP8000)
	{cProcEntry, "stb/info/OEM",                                                     NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/info/brand",                                                   NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/info/model_name",                                              NULL, NULL, NULL, NULL, ""},
#if defined(SPARK) \
 || defined(SPARK7162)
	{cProcEntry, "stb/info/stb_id",                                                  NULL, NULL, NULL, NULL, ""},
#endif
#endif

	{cProcDir,   "stb/ir",                                                           NULL, NULL, NULL, NULL, ""},
	{cProcDir,   "stb/ir/rc",                                                        NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/ir/rc/type",                                                   NULL, info_rctype_read, info_rctype_write, NULL, ""},

	{cProcDir,   "stb/lcd",                                                          NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/lcd/scroll_repeats",                                           NULL, info_scroll_repeats_read, info_scroll_repeats_write, NULL, ""},
	{cProcEntry, "stb/lcd/scroll_delay",                                             NULL, info_scroll_delay_read, info_scroll_delay_write, NULL, ""},
	{cProcEntry, "stb/lcd/initial_scroll_delay",                                     NULL, info_initial_scroll_delay_read, info_initial_scroll_delay_write, NULL, ""},
	{cProcEntry, "stb/lcd/final_scroll_delay",                                       NULL, info_final_scroll_delay_read, info_final_scroll_delay_write, NULL, ""},

#if defined(ADB_BOX) \
 || defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD) \
 || defined(CUBEREVO_3000HD) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_250HD) \
 || defined(FORTIS_HDBOX) \
 || defined(ATEVIO7500) \
 || defined(SPARK) \
 || defined(SPARK7162) \
 || defined(TF7700) \
 || defined(VITAMIN_HD5000) \
 || defined(HL101) \
 || defined(VIP1_V1) \
 || defined(VIP1_V2) \
 || defined(VIP2)
	{cProcEntry, "stb/lcd/symbol_circle",                                            NULL, NULL, NULL, NULL, ""},
#endif
#if defined(SPARK7162) \
 || defined(TF7700) \
 || defined(HL101) \
 || defined(VIP1_V1) \
 || defined(VIP1_V2) \
 || defined(VIP2)
	{cProcEntry, "stb/lcd/symbol_hdd",                                               NULL, NULL, NULL, NULL, ""},
#endif
#if defined(ADB_BOX) \
 || defined(FORTIS_HDBOX) \
 || defined(ATEVIO7500) \
 || defined(CUBEREVO) \
 || defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD) \
 || defined(SPARK) \
 || defined(SPARK7162) \
 || defined(TF7700) \
 || defined(HL101) \
 || defined(VIP1_V1) \
 || defined(VIP1_V2) \
 || defined(VIP2)
	{cProcEntry, "stb/lcd/symbol_timeshift",                                         NULL, NULL, NULL, NULL, ""},
#endif

	{cProcDir,   "stb/video",                                                        NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/alpha",                                                  NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/aspect",                                                 NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/aspect_choices",                                         NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/hdmi_colorspace",                                        NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/hdmi_colorspace_choices",                                NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/force_dvi",                                              NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/policy",                                                 NULL, NULL, NULL, NULL, ""},
#if defined(QBOXHD) || defined(QBOXHD_MINI)
	{cProcEntry, "stb/video/policy2"                                                , NULL, NULL, NULL, NULL, ""},
#endif
	{cProcEntry, "stb/video/policy_choices",                                         NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/videomode",                                              NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/3d_mode",                                                NULL, three_d_mode_read, three_d_mode_write, NULL, ""},
	{cProcEntry, "stb/video/videomode_50hz",                                         NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/videomode_60hz",                                         NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/videomode_choices",                                      NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/videomode_preferred",                                    NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/pal_v_start",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/pal_v_end",                                              NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/pal_h_start",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/pal_h_end",                                              NULL, NULL, NULL, NULL, ""},

	{cProcDir,   "stb/avs",                                                          NULL, NULL, NULL, NULL, ""},
	{cProcDir,   "stb/avs/0",                                                        NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/avs/0/colorformat",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/avs/0/colorformat_choices",                                    NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/avs/0/fb",                                                     NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/avs/0/input",                                                  NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/avs/0/sb",                                                     NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/avs/0/volume",                                                 NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/avs/0/input_choices",                                          NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/avs/0/standby",                                                NULL, NULL, NULL, NULL, ""},

	{cProcDir,   "stb/denc",                                                         NULL, NULL, NULL, NULL, ""},
	{cProcDir,   "stb/denc/0",                                                       NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/denc/0/wss",                                                   NULL, NULL, NULL, NULL, ""},

	{cProcDir,   "stb/fb",                                                           NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/fb/dst_left",                                                  NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/fb/dst_top",                                                   NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/fb/dst_width",                                                 NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/fb/dst_height",                                                NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/fb/3dmode",                                                    NULL, three_d_mode_read, three_d_mode_write, NULL, ""},
	{cProcEntry, "stb/fb/znorm",                                                     NULL, NULL, default_write_proc, NULL, ""},

	{cProcDir,   "stb/fp",                                                           NULL, NULL, NULL, NULL, ""},
//	{cProcEntry, "stb/fp/lnb_sense1",                                                NULL, NULL, NULL, NULL, ""},
//	{cProcEntry, "stb/fp/lnb_sense2",                                                NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/fp/led0_pattern",                                              NULL, NULL, default_write_proc, NULL, ""},
#if !defined(VIP1_V2) \
 && !defined(VIP2)
	{cProcEntry, "stb/fp/led1_pattern",                                              NULL, NULL, default_write_proc, NULL, ""},
#endif
#if defined(ADB_BOX)
	{cProcEntry, "stb/fp/led2_pattern",                                              NULL, NULL, default_write_proc, NULL, ""},
	{cProcEntry, "stb/fp/led3_pattern",                                              NULL, NULL, default_write_proc, NULL, ""},
#endif
	{cProcEntry, "stb/fp/led_pattern_speed",                                         NULL, NULL, default_write_proc, NULL, ""},
#if !defined(CUBEREVO_250HD) \
 && !defined(CUBEREVO_MINI_FTA)
	{cProcEntry, "stb/fp/oled_brightness",                                           NULL, NULL, NULL, NULL, ""},
#endif
	{cProcEntry, "stb/fp/rtc",                                                       NULL, zero_read, default_write_proc, NULL, ""},
	{cProcEntry, "stb/fp/rtc_offset",                                                NULL, zero_read, default_write_proc, NULL, ""},
	{cProcEntry, "stb/fp/text",                                                      NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/fp/version",                                                   NULL, zero_read, NULL, NULL, ""},
	{cProcEntry, "stb/fp/wakeup_time",                                               NULL, wakeup_time_read, wakeup_time_write, NULL, ""},
	{cProcEntry, "stb/fp/was_timer_wakeup",                                          NULL, NULL, NULL, NULL, ""},

#if defined(FORTIS_HDBOX) \
 || defined(OCTAGON1008) \
 || defined(ATEVIO7500) \
 || defined(HS7110) \
 || defined(HS7119) \
 || defined(HS7420) \
 || defined(HS7429) \
 || defined(HS7810A) \
 || defined(HS7819) \
 || defined(FOREVER_NANOSMART) \
 || defined(FOREVER_9898HD) \
 || defined(DP7001) \
 || defined(FOREVER_3434HD) \
 || defined(FOREVER_2424HD) \
 || defined(GPV8000) \
 || defined(EP8000) \
 || defined(EPP8000)
	{cProcEntry, "stb/fp/resellerID",                                                NULL, NULL, NULL, NULL, ""},
#endif

#if defined(SPARK) \
 || defined(SPARK7162) \
 || defined(VIP1_V1) \
 || defined(VIP1_V2) \
 || defined(VIP2)
	{cProcEntry, "stb/fp/aotom",                                                     NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/fp/displaytype",                                               NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/fp/timemode",                                                  NULL, NULL, NULL, NULL, ""},
	{cProcDir,   "stb/power",                                                        NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/power/standbyled",                                             NULL, NULL, NULL, NULL, ""},
#endif
   	{cProcEntry, "stb/power/vfd",                                                    NULL, NULL, NULL, NULL, ""},

   	{cProcDir,   "stb/tsmux",                                                        NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/tsmux/input0",                                                 NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/tsmux/input1",                                                 NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/tsmux/ci0_input",                                              NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/tsmux/ci1_input",                                              NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/tsmux/lnb_b_input",                                            NULL, NULL, NULL, NULL, ""},

	{cProcDir, "stb/misc",                                                           NULL, NULL, NULL, NULL, ""},
#if defined(IPBOX9900) \
 || defined(HL101) \
 || defined(VIP1_V1)
	{cProcEntry, "stb/misc/12V_output",                                              NULL, proc_misc_12V_output_read, proc_misc_12V_output_write, NULL, ""},
#endif

	{cProcDir,   "stb/vmpeg",                                                        NULL, NULL, NULL, NULL, ""},
	{cProcDir,   "stb/vmpeg/0",                                                      NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/0/dst_apply",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/0/dst_left",                                             NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/0/dst_top",                                              NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/0/dst_width",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/0/dst_height",                                           NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/0/dst_all",                                              NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/0/yres",                                                 NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/0/xres",                                                 NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/0/aspect",                                               NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/0/framerate",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/0/pep_apply",                                            NULL, NULL, NULL, NULL, ""},

	{cProcDir,   "stb/vmpeg/1",                                                      NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/1/dst_apply",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/1/dst_left",                                             NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/1/dst_top",                                              NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/1/dst_width",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/1/dst_height",                                           NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/1/dst_all",                                              NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/1/yres",                                                 NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/1/xres",                                                 NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/1/aspect",                                               NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/1/framerate",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/vmpeg/1/pep_apply",                                            NULL, NULL, NULL, NULL, ""},

	{cProcDir,   "stb/hdmi",                                                         NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/hdmi/enable_hdmi_resets",                                      NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/hdmi/output",                                                  NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/hdmi/output_choices",                                          NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/hdmi/audio_source",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/hdmi/audio_source_choices",                                    NULL, NULL, NULL, NULL, ""},

	{cProcDir,   "stb/stream",                                                       NULL, NULL, NULL, NULL, ""},
	{cProcDir,   "stb/stream/policy",                                                NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/stream/policy/AV_SYNC",                                        NULL, NULL, NULL, NULL, "AV_SYNC"},
	{cProcEntry, "stb/stream/policy/TRICK_MODE_AUDIO",                               NULL, NULL, NULL, NULL, "TRICK_MODE_AUDIO"},
	{cProcEntry, "stb/stream/policy/PLAY_24FPS_VIDEO_AT_25FPS",                      NULL, NULL, NULL, NULL, "PLAY_24FPS_VIDEO_AT_25FPS"},
	{cProcEntry, "stb/stream/policy/MASTER_CLOCK",                                   NULL, NULL, NULL, NULL, "MASTER_CLOCK"},
	{cProcEntry, "stb/stream/policy/EXTERNAL_TIME_MAPPING",                          NULL, NULL, NULL, NULL, "EXTERNAL_TIME_MAPPING"},
	{cProcEntry, "stb/stream/policy/DISPLAY_FIRST_FRAME_EARLY",                      NULL, NULL, NULL, NULL, "DISPLAY_FIRST_FRAME_EARLY"},
	{cProcEntry, "stb/stream/policy/STREAM_ONLY_KEY_FRAMES",                         NULL, NULL, NULL, NULL, "STREAM_ONLY_KEY_FRAMES"},
	{cProcEntry, "stb/stream/policy/STREAM_SINGLE_GROUP_BETWEEN_DISCONTINUITIES",    NULL, NULL, NULL, NULL, "STREAM_SINGLE_GROUP_BETWEEN_DISCONTINUITIES"},
	{cProcEntry, "stb/stream/policy/PLAYOUT_ON_TERMINATE",                           NULL, NULL, NULL, NULL, "PLAYOUT_ON_TERMINATE"},
	{cProcEntry, "stb/stream/policy/PLAYOUT_ON_SWITCH",                              NULL, NULL, NULL, NULL, "PLAYOUT_ON_SWITCH"},
	{cProcEntry, "stb/stream/policy/PLAYOUT_ON_DRAIN",                               NULL, NULL, NULL, NULL, "PLAYOUT_ON_DRAIN"},
	{cProcEntry, "stb/stream/policy/TRICK_MODE_DOMAIN",                              NULL, NULL, NULL, NULL, "TRICK_MODE_DOMAIN"},
	{cProcEntry, "stb/stream/policy/DISCARD_LATE_FRAMES",                            NULL, NULL, NULL, NULL, "DISCARD_LATE_FRAMES"},
	{cProcEntry, "stb/stream/policy/REBASE_ON_DATA_DELIVERY_LATE",                   NULL, NULL, NULL, NULL, "REBASE_ON_DATA_DELIVERY_LATE"},
	{cProcEntry, "stb/stream/policy/REBASE_ON_FRAME_DECODE_LATE",                    NULL, NULL, NULL, NULL, "REBASE_ON_FRAME_DECODE_LATE"},
	{cProcEntry, "stb/stream/policy/LOWER_CODEC_DECODE_LIMITS_ON_FRAME_DECODE_LATE", NULL, NULL, NULL, NULL, "LOWER_CODEC_DECODE_LIMITS_ON_FRAME_DECODE_LATE"},
	{cProcEntry, "stb/stream/policy/H264_ALLOW_NON_IDR_RESYNCHRONIZATION",           NULL, NULL, NULL, NULL, "H264_ALLOW_NON_IDR_RESYNCHRONIZATION"},
	{cProcEntry, "stb/stream/policy/MPEG2_IGNORE_PROGESSIVE_FRAME_FLAG",             NULL, NULL, NULL, NULL, "MPEG2_IGNORE_PROGESSIVE_FRAME_FLAG"},

	{cProcDir,   "stb/video/plane",                                                  NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/plane/psi_brightness",                                   NULL, NULL, NULL, NULL, "psi_brightness"},
	{cProcEntry, "stb/video/plane/psi_saturation",                                   NULL, NULL, NULL, NULL, "psi_saturation"},
	{cProcEntry, "stb/video/plane/psi_contrast",                                     NULL, NULL, NULL, NULL, "psi_contrast"},
	{cProcEntry, "stb/video/plane/psi_tint",                                         NULL, NULL, NULL, NULL, "psi_tint"},
	{cProcEntry, "stb/video/plane/psi_apply",                                        NULL, NULL, NULL, NULL, "psi_apply"},
#if defined(UFS912) \
 || defined(UFS913) \
 || defined(ATEVIO7500) \
 || defined(HS7110) \
 || defined(HS7119) \
 || defined(ATEMIO520) \
 || defined(ATEMIO530) \
 || defined(HS7420) \
 || defined(HS7429) \
 || defined(HS7810A) \
 || defined(HS7819) \
 || defined(SPARK) \
 || defined(SPARK7162) \
 || defined(SAGEMCOM88) \
 || defined(VITAMIN_HD5000) \
 || defined(OPT9600) \
 || defined(FOREVER_NANOSMART) \
 || defined(FOREVER_9898HD) \
 || defined(DP7001) \
 || defined(FOREVER_3434HD) \
 || defined(FOREVER_2424HD) \
 || defined(GPV8000) \
 || defined(EP8000) \
 || defined(EPP8000)
	{cProcDir,   "stb/cec",                                                          NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/cec/state_activesource",                                       NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/cec/state_standby",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/cec/state_cecaddress",                                         NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/cec/onetouchplay",                                             NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/cec/systemstandby",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/cec/event_poll",                                               NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/cec/send",                                                     NULL, NULL, NULL, NULL, ""},
#endif
	/* dagobert: the dei settings can be used for all 7109 architectures to affect the de-interlacer */
#if defined(FORTIS_HDBOX) \
 || defined(HL101) \
 || defined(OCTAGON1008) \
 || defined(TF7700) \
 || defined(UFS922) \
 || defined(UFC960) \
 || defined(VIP1_V1) \
 || defined(VIP1_V2) \
 || defined(VIP2) \
 || defined(CUBEREVO) \
 || defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_250HD) \
 || defined(CUBEREVO_MINI_FTA) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_9500HD) \
 || defined(CUBEREVO_3000HD) \
 || defined(IPBOX9900) \
 || defined(IPBOX99) \
 || defined(IPBOX55) \
 || defined(ARIVALINK200) \
 || defined(OPT9600)
	/* dagobert: the dei settings can be used for all 7109 architectures to affect the de-interlacer */
	{cProcEntry, "stb/video/plane/dei_fmd",                                          NULL, NULL, NULL, NULL, "dei_fmd"},
	{cProcEntry, "stb/video/plane/dei_mode",                                         NULL, NULL, NULL, NULL, "dei_mode"},
	{cProcEntry, "stb/video/plane/dei_ctrl",                                         NULL, NULL, NULL, NULL, "dei_ctrl"},
#endif

#if defined(IPBOX9900) \
 || defined(IPBOX99)
	{cProcEntry, "stb/misc/fan",                                                     NULL, NULL, NULL, NULL, ""},
#endif

// Enigma2 implementation of fan control, doubles the historic various SH4 ones... (TODO: add these)
#if defined(CUBEREVO) \
 || defined(IPBOX9900) \
 || defined(IPBOX99) \
 || defined(ADB_BOX) \
 || defined(SAGEMCOM88) \
 || defined(PACE7241)
	{cProcEntry, "stb/fp/fan",                                                       NULL, NULL, NULL, NULL, ""},
#endif
#if defined(ADB_BOX) \
 || defined(IPBOX9900) \
 || defined(IPBOX99)
	{cProcEntry, "stb/fp/fan_pwm",                                                   NULL, NULL, NULL, NULL, ""},
#endif

#if defined(ADB_BOX) \
 || defined(SAGEMCOM88)
	{cProcDir,   "stb/fan",                                                          NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/fan/fan_ctrl",                                                 NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/hdmi/cec",                                                     NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/switch_type",                                            NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/switch",                                                 NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/video/switch_choices",                                         NULL, NULL, NULL, NULL, ""},
#elif defined(ARIVALINK200)
	{cProcEntry, "stb/hdmi/cec",                                                     NULL, NULL, NULL, NULL, ""},
#endif

	{cProcDir,   "stb/player",                                                       NULL, NULL, NULL, NULL, ""},
	{cProcEntry, "stb/player/version",                                               NULL, get_player_version, NULL, NULL, ""}
};

static int cpp_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int i;

	/* find the entry */
	for (i = 0; i < sizeof(e2Proc) / sizeof(e2Proc[0]); i++)
	{
		if ((e2Proc[i].identifier != NULL)
		&& (strlen(e2Proc[i].identifier) > 0)
		&& (strcmp(e2Proc[i].identifier, data) == 0)
		&& (e2Proc[i].read_proc != NULL))
		{
			return e2Proc[i].read_proc(page, start, off, count, eof, e2Proc[i].instance);
		}
	}
	return 0;
}

/* we need this function because the cpp modules cannot include
 * the linux kernel headers and therefore we miss some functions
 * (e.g. copy_from_user)
 * so here we make the dirty stuff and then call the c-function
 * in the cpp module which can cast the instance and call the
 * real method ;-)
 */
static int cpp_write_proc(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	int i;
	char *page;
	ssize_t ret = -ENOMEM;

	page = (char *)__get_free_page(GFP_KERNEL);

	if (page)
	{
		ret = -EFAULT;

		if (copy_from_user(page, buf, count))
		{
			goto out;
		}
		/* find the entry */
		for (i = 0; i < sizeof(e2Proc) / sizeof(e2Proc[0]); i++)
		{
			if ((e2Proc[i].identifier != NULL)
			&& (strlen(e2Proc[i].identifier) > 0)
			&& (strcmp(e2Proc[i].identifier, data) == 0)
			&& (e2Proc[i].write_proc != NULL))
			{
				ret = e2Proc[i].write_proc(file, (const char __user *) page, count, e2Proc[i].instance);
			}
		}
	}
out:
	free_page((unsigned long)page);

	return ret;
}

struct proc_dir_entry *find_proc_dir(char *name)
{
	int i;

	for (i = 0; i < sizeof(e2Proc) / sizeof(e2Proc[0]); i++)
	{
		if ((e2Proc[i].type == cProcDir) && (strcmp(name, e2Proc[i].name) == 0))
		{
			return e2Proc[i].entry;
		}
	}
	return NULL;
}

/* the function returns the directry name */
char *dirname(char *name)
{
	static char path[100];
	int i = 0;
	int pos = 0;

	while ((i < sizeof(path)) && (name[i] != 0))
	{
		if (name[i] == '/')
		{
			pos = i;
		}
		path[i] = name[i];
		i++;
	}
	path[i] = 0;
	path[pos] = 0;

	return path;
}

/* the function returns the base name */
char *basename(char *name)
{
	int i = 0;
	int pos = 0;

	while (name[i] != 0)
	{
		if (name[i] == '/')
		{
			pos = i;
		}
		i++;
	}

	if (name[pos] == '/')
	{
		pos++;
	}
	return name + pos;
}

int install_e2_procs(char *path, read_proc_t *read_func, write_proc_t *write_func, void *data)
{
	int i;

	/* find the entry */
	for (i = 0; i < sizeof(e2Proc) / sizeof(e2Proc[0]); i++)
	{
		if ((e2Proc[i].type == cProcEntry)
		&& (strcmp(path, e2Proc[i].name) == 0))
		{
			if (e2Proc[i].entry == NULL)
			{
				printk("%s(): entry not available '%s'\n", __func__, path);
			}
			else
			{
				/* check whether the default entry is installed */
				if ((e2Proc[i].entry->read_proc != e2Proc[i].read_proc) || (e2Proc[i].entry->write_proc != e2Proc[i].write_proc))
				{
					printk("%s(): entry already in use '%s'\n", __func__, path);
				}
				else
				{
					/* install the provided functions */
					e2Proc[i].entry->read_proc = read_func;
					e2Proc[i].entry->write_proc = write_func;
					e2Proc[i].entry->data = data;
				}
			}
			break;
		}
	}

	if (i == sizeof(e2Proc) / sizeof(e2Proc[0]))
	{
		printk("%s(): entry not found '%s'\n", __func__, path);
	}
	return 0;
}
EXPORT_SYMBOL(install_e2_procs);

int cpp_install_e2_procs(const char *path, read_proc_t *read_func, write_proc_t *write_func, void *instance)
{
	int i;
#ifdef VERY_VERBOSE
	printk("%s: %s\n", __func__, path);
#endif

	/* find the entry */
	for (i = 0; i < sizeof(e2Proc) / sizeof(e2Proc[0]); i++)
	{
		if ((e2Proc[i].type == cProcEntry) && (strcmp(path, e2Proc[i].name) == 0))
		{
			if (e2Proc[i].entry == NULL)
			{
				printk("%s(): entry not available '%s'\n", __func__, path);
//dagobert: i would prefer to make this dynamic for player purpose
//it would be nice I think; think on it later!!!!!!!
			}
			else
			{
				/* install the provided functions */
				e2Proc[i].entry->read_proc = cpp_read_proc;
				e2Proc[i].entry->write_proc = cpp_write_proc;
				e2Proc[i].entry->data = e2Proc[i].identifier;

				e2Proc[i].read_proc = read_func;
				e2Proc[i].write_proc = write_func;
				e2Proc[i].instance = instance;
			}
			break;
		}
	}

	if (i == sizeof(e2Proc) / sizeof(e2Proc[0]))
	{
		printk("%s(): entry not found '%s'\n", __func__, path);
	}
	return 0;
}
EXPORT_SYMBOL(cpp_install_e2_procs);

int remove_e2_procs(char *path, read_proc_t *read_func, write_proc_t *write_func)
{
	int i;

	/* find the entry */
	for (i = 0; i < sizeof(e2Proc) / sizeof(e2Proc[0]); i++)
	{
		if ((e2Proc[i].type == cProcEntry) && (strcmp(path, e2Proc[i].name) == 0))
		{
			if (e2Proc[i].entry == NULL)
			{
				printk("%s(): entry not available '%s'\n", __func__, path);
			}
			else
			{
				/* replace the entry with the default */
				if (e2Proc[i].entry->read_proc == read_func)
				{
					e2Proc[i].entry->read_proc = e2Proc[i].read_proc;
				}
				else
				{
					printk("%s(): different read_procs '%s' (%p, %p)\n", __func__, path, e2Proc[i].entry->read_proc, read_func);
				}

				if (e2Proc[i].entry->write_proc == write_func)
				{
					e2Proc[i].entry->write_proc = e2Proc[i].write_proc;
				}
				else
				{
					printk("%s(): different write_procs '%s' (%p, %p)\n", __func__, path, e2Proc[i].entry->write_proc, write_func);
				}
			}
			break;
		}
	}

	if (i == sizeof(e2Proc) / sizeof(e2Proc[0]))
	{
		printk("%s(): entry not found '%s'\n", __func__, path);
	}
	return 0;
}

EXPORT_SYMBOL(remove_e2_procs);

int cpp_remove_e2_procs(const char *path, read_proc_t *read_func, write_proc_t *write_func)
{
	int i;

	/* find the entry */
	for (i = 0; i < sizeof(e2Proc) / sizeof(e2Proc[0]); i++)
	{
		if ((e2Proc[i].type == cProcEntry)
		&& (strcmp(path, e2Proc[i].name) == 0))
		{
			if (e2Proc[i].entry == NULL)
			{
				printk("%s(): entry not available '%s'\n", __func__, path);
			}
			else
			{
				e2Proc[i].instance = NULL;

				if (e2Proc[i].read_proc == read_func)
				{
					e2Proc[i].read_proc = NULL;
#ifdef VERY_VERBOSE
					printk("%s(): removed '%s, %s' (%p, %p)\n", __func__, path, e2Proc[i].name, e2Proc[i].read_proc, read_func);
#endif
				}
				else
				{
					printk("%s(): different read_procs '%s, %s' (%p, %p)\n", __func__, path, e2Proc[i].name, e2Proc[i].read_proc, read_func);
				}

				if (e2Proc[i].write_proc == write_func)
				{
					e2Proc[i].write_proc = NULL;
				}
				else
				{
					printk("%s(): different write_procs '%s' (%p, %p)\n", __func__, path, e2Proc[i].write_proc, write_func);
				}
			}
			break;
		}
	}

	if (i == sizeof(e2Proc) / sizeof(e2Proc[0]))
	{
		printk("%s(): entry not found '%s'\n", __func__, path);
	}
	return 0;
}
EXPORT_SYMBOL(cpp_remove_e2_procs);

static int __init e2_proc_init_module(void)
{
	int i;
	char *path;
	char *name;

	for (i = 0; i < sizeof(e2Proc) / sizeof(e2Proc[0]); i++)
	{
		path = dirname(e2Proc[i].name);
		name = basename(e2Proc[i].name);

		switch (e2Proc[i].type)
		{
			case cProcDir:
			{
				e2Proc[i].entry = proc_mkdir(name, find_proc_dir(path));

				if (e2Proc[i].entry == NULL)
				{
					printk("%s(): could not create entry %s\n", __func__, e2Proc[i].name);
				}
				break;
			}
			case cProcEntry:
			{
				if (strcmp("bus", path) == 0)
				{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
					e2Proc[i].entry = create_proc_entry(name, 0, proc_bus);
#else
					e2Proc[i].entry = create_proc_entry(e2Proc[i].name, 0, NULL);
#endif
				}
				else
				{
					e2Proc[i].entry = create_proc_entry(name, 0, find_proc_dir(path));
				}

				if (e2Proc[i].entry != NULL)
				{
					e2Proc[i].entry->read_proc = e2Proc[i].read_proc;
					e2Proc[i].entry->write_proc = e2Proc[i].write_proc;
				}
				else
				{
					printk("%s(): could not create entry %s\n", __func__, e2Proc[i].name);
				}
				break;
			}
			default:
			{
				printk("%s(): invalid type %d\n", __func__, e2Proc[i].type);
			}
		}
	}
#if defined(HL101) \
 || defined(VIP1_V1)
	_12v_pin = stpio_request_pin(4, 6, "12V_CTL", STPIO_OUT);

	if (_12v_pin == NULL)
	{
		printk("Allocating PIO 4.6 for 12V output failed\n");
	}
	else
	{
		set_12v(0);  // switch 12V output off
	}
#endif
	return 0;
}

static void __exit e2_proc_cleanup_module(void)
{
	int i;
	char *name;

	for (i = sizeof(e2Proc) / sizeof(e2Proc[0]) - 1; i >= 0; i--)
	{
		name = basename(e2Proc[i].name);
		remove_proc_entry(name, e2Proc[i].entry->parent);
	}
}

module_init(e2_proc_init_module);
module_exit(e2_proc_cleanup_module);

MODULE_DESCRIPTION("procfs module with enigma2 support");
MODULE_AUTHOR("Team Ducktales");
MODULE_LICENSE("GPL");
// vim:ts=4
