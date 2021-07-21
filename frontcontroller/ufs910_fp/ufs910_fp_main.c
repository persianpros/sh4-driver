/*************************************************************************
 *
 * ufs910_fp_main.c
 *
 * 11. Nov 2007 - captaintrip
 * 09. Jul 2021 - Audioniek
 *
 * Kathrein UFS910 VFD Kernel module - main part
 * Erster version portiert aus den MARUSYS uboot sourcen
 *
 * Note: previous separate button driver for 14W models has been
 *       integrated
 *
 * The front panel display consists of a VFD tube Futaba 16-BT-136INK which
 * has a 1x16 character VFD display with 16 icons and a built-in driver
 * IC that seems to be somewhat compatible with the OKI ML9208.
 * The built-in display controller does not provide key scanning
 * facilities.
 *
 * The display controller is controlled through 3 PIO pins.
 *
 * The frontpanel processor on 1W models is controlled through asc3
 * (/dev/AStty1).
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
 * Frontpanel driver for Kathrein UFS910
 *
 *************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * -----------------------------------------------------------------------
 * 20210707 Audioniek       Add standard dprintk debugging display.
 * 20210707 Audioniek       PIO pin control changed.
 * 20210712 Audioniek       Add code for all icons on/off.
 * 20210712 Audioniek       Text write to /dev/vfd now scrolls once if
 *                          length exceeds VFD_LENGTH.
 * 20210712 Audioniek       Driver init clears all icons.
 * 20210713 Audioniek       Display controller initialization according to
 *                          datasheet recommendation.
 * 20210714 Audioniek       Add mode 0 switching.
 * 20210714 Audioniek       Add Enigma2 icon processing.
 * 20210718 Audioniek       Add VFDSETLED (14W models only).
 * 20210719 Audioniek       Front panel button driver added (14W models
 *                          only).
 * 20210720 Audioniek       Split into file, main and procfs parts.
 *
 *************************************************************************
 */
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
#include <linux/stm/pio.h>
#else
#include <linux/stpio.h>
#endif

// for button driver
#include <linux/input.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#include "ufs910_fp.h"

short paramDebug = 0;
int waitTime = 1000;

int boxtype = 0;  // ufs910 14W: 1 or 3, ufs910 1W: 0

static tVFDOpen vOpen;

extern int ufs910_fp_SetLED(int which, int on);

/****************************************
 *
 * Front panel button driver (14W only)
 *
 * Code largely taken from button.c by
 * captaintrip and aotom_main.c.
 *
 * TODO: wheel press and wheel rotate
 *
 */
static char *button_driver_name = "Kathrein UFS910 (14W) front panel button driver";
static struct input_dev *button_dev;
static int bad_polling = 1;

static struct workqueue_struct *fpwq;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
void button_bad_polling(void)
#else
void button_bad_polling(struct work_struct *work)
#endif
{
	static unsigned char button_value = 0;
	int btn_pressed = 0;
	int report_key = 0;
	int ret = 0;

	while (bad_polling == 1)
	{
		msleep(50);
		button_value = (STPIO_GET_PIN(PIO_PORT(1), 0) << 2) | (STPIO_GET_PIN(PIO_PORT(1), 1) << 1) | (STPIO_GET_PIN(PIO_PORT(1), 2) << 0);
		if (button_value != 7)
		{
			switch (button_value)  // convert button input to key value
			{
				case 6:  // v-format
				{
//					dprintk(30, "V-FORMAT pressed\n");
					button_value = FP_VFORMAT_KEY;
					break;
				}
				case 4:  // menu
				{
//					dprintk(30, "MENU pressed\n");
					button_value = FP_MENU_KEY;
					break;
				}
				case 3:  // option
				{
//					dprintk(30, "OPTION pressed\n");
					button_value = FP_OPTION_KEY;
					break;
				}
				case 5:  // exit
				{
//					dprintk(30, "EXIT pressed\n");
					button_value = FP_EXIT_KEY;
					break;
				}
			}
			dprintk(70, "Got button: %02x\n", button_value);

			ret |= ufs910_fp_SetLED(LED_GREEN, 1);  // give feed back through green LED

			if (btn_pressed == 1)  // if already a key down
			{
				if (report_key == button_value)  // and the same as last time
				{
					continue;  // proceed
				}
				input_report_key(button_dev, report_key, 0);  // else report last key released
				input_sync(button_dev);
			}
			report_key = button_value;
			btn_pressed = 1;  // flag key down

			switch (button_value)
			{
				case FP_VFORMAT_KEY:
				case FP_MENU_KEY:
				case FP_OPTION_KEY:
				case FP_EXIT_KEY:
				{
//					dprintk(30, "Report key 0x%02x\n", button_value);  // report new key
					input_report_key(button_dev, button_value, 1);
					input_sync(button_dev);
					break;
				}
				default:
				{
					dprintk(1, "[BTN] unknown button_value %02x", button_value);
				}
			}
		}
		else  // key has been released
		{
			if (btn_pressed)
			{
//				dprintk(30, "Key release\n");
				btn_pressed = 0;
				ret |= ufs910_fp_SetLED(LED_GREEN, 0);
				input_report_key(button_dev, report_key, 0);
				input_sync(button_dev);
			}
		}
	}
	bad_polling = 2;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static DECLARE_WORK(button_obj, button_bad_polling, NULL); 
#else
static DECLARE_WORK(button_obj, button_bad_polling); 
#endif
static int button_input_open(struct input_dev *dev)
{
	fpwq = create_workqueue("button");                   
	if (queue_work(fpwq, &button_obj))
	{                    
		dprintk(20, "[BTN] Queue_work successful...\n");
	}
	else
	{
		dprintk(1, "[BTN] Queue_work not successful, exiting...\n");
		return 1;
	}
	return 0;
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
		dprintk(20, "[BTN] Workqueue destroyed\n");
	}
}

int button_dev_init(void)
{
	int error;

	dprintk(20, "[BTN] Allocating and registering button device\n");

	button_dev = input_allocate_device();
	if (!button_dev)
	{
		return -ENOMEM;
	}

	button_dev->name = button_driver_name;
	button_dev->open = button_input_open;
	button_dev->close = button_input_close;

	set_bit(EV_KEY, button_dev->evbit);
	set_bit(FP_VFORMAT_KEY, button_dev->keybit); 
	set_bit(FP_MENU_KEY, button_dev->keybit); 
	set_bit(FP_OPTION_KEY, button_dev->keybit); 
	set_bit(FP_EXIT_KEY, button_dev->keybit); 

	error = input_register_device(button_dev);
	if (error)
	{
		input_free_device(button_dev);
	}
	return error;
}

void button_dev_exit(void)
{
	dprintk(20, "[BTN] unregistering button device\n");
	input_unregister_device(button_dev);
}

int __init button_init(void)
{
	dprintk(20, "[BTN] initializing ...\n");

	if (button_dev_init() != 0)
	{
		return 1;
	}
	return 0;
}

void __exit button_exit(void)
{
	dprintk(20, "[BTN] unloading ...\n");

	button_dev_exit();
}

/***********************************************************
 *
 * Module code.
 *
 */

/****************************************
 *
 * Determine box variant
 *
 * Code largely taken from boxtype.c
 *
 */
void get_box_variant(void)
{
	boxtype = (STPIO_GET_PIN(PIO_PORT(4), 5) << 1) | STPIO_GET_PIN(PIO_PORT(4), 4);

	if (paramDebug > 19)
	{
		if (boxtype == 0)
		{
			dprintk(0, "1W");
		}
		else
		{
			dprintk(0, "14W");
		}
		printk(" standby model detected\n");
	}
}

extern void create_proc_fp(void);
extern void remove_proc_fp(void);

/***********************************************************
 *
 * Initialize driver.
 *
 */
static int __init vfd_init_module(void)
{
	dprintk(100, "%s >\n", __func__);
	printk("Kathrein UFS910 front panel driver\n");

	get_box_variant();  // determine box variant

	if (boxtype == 0)  // 1W model detected
	{
	}
	else  // 14W model
	{
		if (button_dev_init() != 0)  // initialize front panel button driver
		{
			dprintk(1, "Unable to initialize front panel button driver.\n");
			return -1;
		}
	}
	msleep(waitTime);

	if (register_chrdev(VFD_MAJOR, "VFD", &vfd_fops))
	{
		dprintk(1, "%s: Unable to get major %d for VFD\n", __func__, VFD_MAJOR);
	}
	else
	{
		vfd_init_func();
	}
	/* konfetti */
	vOpen.fp = NULL;
	sema_init(&vOpen.sem, 1);

	create_proc_fp();

	dprintk(100, "%s <\n", __func__);
	return 0;
}

/***********************************************************
 *
 * Unload driver.
 *
 */
static void __exit vfd_cleanup_module(void)
{
	remove_proc_fp();

	if (boxtype == 0)  // 1W model detected
	{
	}
	else  // 14W model
	{
		dprintk(20, "[BTN] Unloading...\n");
		button_dev_exit();
	}
	unregister_chrdev(VFD_MAJOR, "VFD");
	dprintk(0, "Kathrein UFS910 VFD module unloaded\n");
}

module_init(vfd_init_module);
module_exit(vfd_cleanup_module);

module_param(waitTime, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(waitTime, "Wait before init in ms (default=1000)");

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

MODULE_DESCRIPTION("VFD module for Kathrein UFS910 (Q'n'D port from MARUSYS)");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
