/****************************************************************************************
 *
 * adb_5800_fp_main.c
 *
 * (c) 20?? Freebox
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
 * Front panel driver for ADB ITI-5800S(X), BSKA, BSLA, BXZB and BZZB models;
 * Button, LEDs, LED display and VFD driver, main part.
 *
 * Devices:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 201????? freebox         Initial version       
 * 20190730 Audioniek       Beginning of rewrite.
 * 20190803 Audioniek       Add PT6302-001 UTF-8 support.
 * 20190804 Audioniek       Add VFDSETLED and VFDLEDBRIGHTNESS.
 * 20190804 Audioniek       Add IOCTL compatibility mode (handy for fp_control).
 * 20190805 Audioniek       procfs added.
 * 20190806 Audioniek       Scrolling on /dev/vfd added.
 * 20190807 Audioniek       Procfs adb_variant added; replaces boxtype.
 * 20190807 Audioniek       Automatic selection of key lay out & display type
 *                          when no module parameters are specified -> one driver for
 *                          all variants.
 * 20190809 Audioniek       Split in nuvoton style between main, file and procfs parts.
 * 20190812 Audioniek       Icon thread added.
 * 20190813 Audioniek       Spinner thread added.
 * 20190813 Audioniek       Fan driver added.
 * 20190814 Audioniek       Display all icons added.
 * 20190903 Audioniek       Separate text display threads for VFD and LED.
 * 20191103 Audioniek       Fix bug in handling ICON_MAX in icon thread.
 * 20200504 Audioniek       Removed dual display capability.
 *
 ****************************************************************************************/
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/input.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/stm/pio.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/i2c.h>

#include "adb_5800_fp.h"

// Global variables
static int buttoninterval = 100 /* ms */;  // shortened as key response was too sluggish
static struct timer_list button_timer;
struct stpio_pin *key_int;
char button_reset = 0;
unsigned char key_group1, key_group2;
unsigned int  key_front = 0;

static char *button_driver_name = "nBox frontpanel buttons";
static struct input_dev *button_dev;

static struct fp_driver fp;

short paramDebug  = 0;   // module parameter default
int   display_led = -1;  // module parameter default
int   display_vfd = -1;  // module parameter default
int   rec_key     = -1;  // module parameter default: front panel keyboard layout
int   key_layout;        // active front panel key layout: 0=MENU/EPG/RES, 1=EPG/REC/RES
unsigned long fan_registers;
struct stpio_pin* fan_pin;

tIconState spinner_state;
tIconState icon_state;

// Global variables (defined in adb_5800_fp_file.c)
extern int box_variant;   // 1=BSKA, 2=BSLA, 3=BXZB, 4=BZZB, 5=cable, 0=unknown
extern int display_type;  // active display: 0=LED, 1=VFD, 2=both

/******************************************************
 *
 * Front panel button driver
 *
 */
static void button_delay(unsigned long dummy)
{
	dprintk(150, "%s >\n", __func__);
	if (button_reset == 100)  // if RES was pressed 5 times
	{
		kernel_restart(NULL);  // reset
	}
	else
	{
		if (key_front > 0)
		{
			input_report_key(button_dev, key_front, 0);
			input_sync(button_dev);
			key_front = 0;
		}
		enable_irq(FPANEL_PORT2_IRQ);
	}
	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Button IRQ handler
 *
 */
irqreturn_t fpanel_irq_handler(void *dev1, void *dev2)
{
	dprintk(150, "%s >\n", __func__);
	disable_irq(FPANEL_PORT2_IRQ);

	if ((stpio_get_pin(key_int) == 1) && (key_front == 0))
	{
		ReadKey();
		key_front = 0;

		/* keygroup1: columns KS1 and KS2, bit 0&4=row K1, bit1&5=row K2, bit2&6=row K3, bit3&7=row K4
		 * keygroup2: columns KS3 and KS4, bit 0&4=row K1, bit1&5=row K2, bit2&6=row K3, bit3&7=row K4 */
		if (key_group2 == 0)  // no key on KS3 / KS4
		{
			switch (key_group1)  // get KS1 / KS2
			{
				case 64:  //  KS2 row K3
				{
					key_front = KEY_RIGHT;
					break;
				}
				case 32:  // KS2 row K2
				{
					if (key_layout == 0)
					{
						key_front = KEY_MENU;  // BSKA/BXZB: MENU
					}
					else
					{
						key_front = KEY_EPG;  // BSLA/BZZB: EPG
					}
					break;
				}
				case 16: // KS2 row K1
				{
					key_front = KEY_UP;
					break;
				}
				case 4:  // KS1 row K3
				{
					key_front = KEY_HOME;  // RES
					break;
				}
				case 2:  // KS1 row K2
				{
					key_front = KEY_OK;
					break;
				}
				case 1: // KS1 row K1
				{
					key_front = KEY_POWER;
					break;
				}
			}
		}
		else
		{
			switch (key_group2)
			{
				case 4:  // KS3 row K3
				{
					if (key_layout == 0)
					{
						key_front = KEY_EPG;  // BSKA/BXZB: EPG
					}
					else
					{
						key_front = KEY_RECORD;  // BSLA/BZZB: REC
					}
					break;
				}
				case 2:  // KS3 row K2
				{
					key_front = KEY_LEFT;
					break;
				}
				case 1:  // KS3 row K1
				{
					key_front = KEY_DOWN;
					break;
				}
			}
		}
		if (key_front > 0)
		{
			if (key_front == KEY_HOME)  // emergency reboot: press res 5 times
			{
				button_reset++;
				if (button_reset > 4)
				{
					dprintk(10, "Emergency reboot !!!\n");
					button_reset = 100;
				}
			}
			else
			{
				button_reset = 0;
			}
			input_report_key(button_dev, key_front, 1);
			input_sync(button_dev);
			dprintk(10, "Key: %d (%s)\n", key_front, __func__);
			init_timer(&button_timer);
			button_timer.function = button_delay;
			mod_timer(&button_timer, jiffies + buttoninterval);
		}
		else
		{
			enable_irq(FPANEL_PORT2_IRQ);
		}
	}
	else
	{
		enable_irq(FPANEL_PORT2_IRQ);
	}
	dprintk(150, "%s <\n", __func__);
	return IRQ_HANDLED;
}

static int button_input_open(struct input_dev *dev)
{
	dprintk(150, "%s >\n", __func__);
	dprintk(150, "%s <\n", __func__);
	return 0;
}

static void button_input_close(struct input_dev *dev)
{
	// do nothing
}
// end of button driver code

/******************************************************
 *
 * Fan speed code
 *
 * Code taken from adb_box_fan.c
 *
 */
static int __init init_fan_module(void)
{
	fan_registers = (unsigned long)ioremap(0x18010000, 0x100);

	fan_pin = stpio_request_pin(4, 7, "fan ctrl", STPIO_ALT_OUT);

	// not sure if first one is necessary
	ctrl_outl(0x200, fan_registers + 0x50);
	
	// set a default speed, because otherwise the default is zero
	ctrl_outl(0xaa, fan_registers + 0x04);
	return 0;
}

static void __exit cleanup_fan_module(void)
{
	if (fan_pin != NULL)
	{
		stpio_free_pin(fan_pin);
	}
}
// end of fan driver code

/*********************************************************************
 *
 * Spinner thread code.
 *
 *
 */

/*********************************************************************
 *
 * Bit patterns for the spinner.
 *
 * A character on the VFD display is simply a pixel matrix
 * consisting of five columns of seven pixels high.
 * Column 1 is the leftmost.
 *
 * Format of pixel data:
 *
 * pixeldata0: column 1
 *  ...
 * pixeldata4: column 5
 *
 * Each bit in the pixel data represents one pixel in the column,
 * with the LSbit being the top pixel and bit 6 the bottom one.
 * The pixel will be on when the corresponding bit is one.
 * The resulting map is:
 *
 *  Data        byte0 byte1 byte2 byte3 byte4
 *              -----------------------------
 *  Bitmask     0x01  0x01  0x01  0x01  0x01 <- row not used for spinner
 *              0x02  0x02  0x02  0x02  0x02
 *              0x04  0x04  0x04  0x04  0x04
 *              0x08  0x08  0x08  0x08  0x08
 *              0x10  0x10  0x10  0x10  0x10
 *              0x20  0x20  0x20  0x20  0x20
 *              0x40  0x40  0x40  0x40  0x40
 *
 */
struct iconToInternal spinnerIcons[] =
{
	/*- Name ---------- icon# ----- data0 data1 data2 data3 data4-----*/
	{ "SPINNER_00"    , SPINNER_00, {0x00, 0x00, 0x0e, 0x00, 0x00}},  // 00
	{ "SPINNER_01"    , SPINNER_01, {0x00, 0x00, 0x0e, 0x04, 0x02}},  // 01
	{ "SPINNER_02"    , SPINNER_02, {0x00, 0x00, 0x0e, 0x0c, 0x0a}},  // 02
	{ "SPINNER_03"    , SPINNER_03, {0x00, 0x00, 0x0e, 0x1c, 0x2a}},  // 03
	{ "SPINNER_04"    , SPINNER_04, {0x00, 0x00, 0x3e, 0x1c, 0x2a}},  // 04
	{ "SPINNER_05"    , SPINNER_05, {0x20, 0x10, 0x3e, 0x1c, 0x2a}},  // 05
	{ "SPINNER_06"    , SPINNER_06, {0x28, 0x18, 0x3e, 0x1c, 0x2a}},  // 06
	{ "SPINNER_07"    , SPINNER_07, {0x2a, 0x1c, 0x3e, 0x1c, 0x2a}},  // 07
	{ "SPINNER_08"    , SPINNER_08, {0x2a, 0x1c, 0x38, 0x1c, 0x2a}},  // 08
	{ "SPINNER_09"    , SPINNER_09, {0x2a, 0x1c, 0x38, 0x18, 0x28}},  // 09
	{ "SPINNER_10"    , SPINNER_10, {0x2a, 0x1c, 0x38, 0x10, 0x20}},  // 10
	{ "SPINNER_11"    , SPINNER_11, {0x2a, 0x1c, 0x38, 0x00, 0x00}},  // 11
	{ "SPINNER_12"    , SPINNER_12, {0x2a, 0x1c, 0x08, 0x00, 0x00}},  // 12
	{ "SPINNER_13"    , SPINNER_13, {0x0a, 0x0c, 0x08, 0x00, 0x00}},  // 13
	{ "SPINNER_14"    , SPINNER_14, {0x02, 0x04, 0x08, 0x00, 0x00}},  // 14
	{ "SPINNER_15"    , SPINNER_15, {0x00, 0x00, 0x08, 0x00, 0x00}},  // 15
};

/*********************************************************************
 *
 * Load 8 spinner patterns into PT6302 CGRAM.
 *
 *
 */
int load_spinner_pattern(int offset)
{
	int res = 0;
	int i;

	for (i = 0; i < 8; i++)
	{
		res |= pt6302_write_cgram(i, spinnerIcons[i + offset].pixeldata, 1);
	}
	return res;
}

/*********************************************************************
 *
 * spinner_thread: Thread to display the spinner on VFD.
 *
 * Taken from nuvoton driver
 *
 */
static int spinner_thread(void *arg)
{
	int i;
	int res = 0;
	unsigned char icon_char[1];

	if (spinner_state.status == THREAD_STATUS_RUNNING)
	{
		return 0;
	}
	dprintk(150, "%s: starting\n", __func__);
	spinner_state.status = THREAD_STATUS_INIT;

	dprintk(150, "%s: started\n", __func__);
	spinner_state.status = THREAD_STATUS_RUNNING;

	while (!kthread_should_stop())
	{
		if (!down_interruptible(&spinner_state.sem))
		{
			if (kthread_should_stop())
			{
				goto stop;
			}

			while (!down_trylock(&spinner_state.sem));
			{
				dprintk(50, "%s: Start spinner, period = %d ms\n", __func__, spinner_state.period);
				icon_char[0] = 0x00;  // use character 0 as icon
				res |= pt6302_write_dcram(0, icon_char, 1);
				while ((spinner_state.state) && !kthread_should_stop())
				{
					spinner_state.status = THREAD_STATUS_RUNNING;
					for (i = 0; i < 16; i++)
					{
						if (i == 0 || i == 8)
						{
							res |= load_spinner_pattern(i);
						}
						icon_char[0] = i % 8;
						res |= pt6302_write_dcram(0, icon_char, 1);  // update character 0 (rightmost position)
						msleep(spinner_state.period);
					}
				}
				spinner_state.status = THREAD_STATUS_HALTED;
			}

stop:
			icon_char[0] = 0x20;
			res |= pt6302_write_dcram(0, icon_char, 1);  // update character 0 (rightmost position)
			dprintk(50, "%s: Spinner off\n", __func__);
		}
	}
	spinner_state.status = THREAD_STATUS_STOPPED;
	spinner_state.task = 0;
	dprintk(150, "%s: stopped\n", __func__);
	return res;
}
// end of spinner thread code

/*********************************************************************
 *
 * Icon thread code.
 *
 */

/*********************************************************************
 *
 * Load 8 or (ICON_MAX - offset) consequtive icon patterns into
 * PT6302 CGRAM.
 *
 *
 */
int load_icon_patterns(int offset)
{
	int res = 0;
	int i, j;
	extern struct iconToInternal vfdIcons[];

	j = (ICON_MAX - offset < 8 ? ICON_MAX - offset : 8);

	for (i = 0; i < j; i++)
	{
		res |=pt6302_write_cgram(i, vfdIcons[i + offset].pixeldata, 1);
	}
	return res;
}
/*********************************************************************
 *
 * icon_thread: Thread to display multiple icons on VFD.
 *
 * Taken from nuvoton driver
 *
 */
int icon_thread(void *arg)
{
	int i = 0;
	int res = 0;
	unsigned char icon_char[1];

	if (icon_state.status == THREAD_STATUS_RUNNING || lastdata.icon_state[ICON_SPINNER])
	{
		return 0;  // if spinner or ourselves running, exit
	}
	icon_state.status = THREAD_STATUS_INIT;

	icon_state.status = THREAD_STATUS_RUNNING;

	while (!kthread_should_stop())
	{
		if (!down_interruptible(&icon_state.sem))
		{
			if (kthread_should_stop())
			{
				goto stop_icon;
			}
			while (!down_trylock(&icon_state.sem));
			{
				while ((icon_state.state) && !kthread_should_stop())
				{
					icon_state.status = THREAD_STATUS_RUNNING;
					if (lastdata.icon_state[ICON_MAX] == 1)
					{
						// handle ICON_MAX
						for (i = ICON_MIN + 1; i < ICON_MAX; i++)
						{
							if ((i - 1) % 8 == 0)
							{
								load_icon_patterns(i);
							}
							icon_char[0] = (i - 1) % 8;
							res |= pt6302_write_dcram(0, icon_char, 1);  // update character 0 (rightmost position)
							if (!icon_state.state)
							{
								break;
							}
							msleep(1500);
						}
						if (!icon_state.state)
						{
							icon_char[0] = 0x20;
						}
					}
					else
					{
						for (i = 0; i < lastdata.icon_count; i++)
						{
							icon_char[0] = i;
							res |= pt6302_write_dcram(0, icon_char, 1);  // update character 0 (rightmost position)
							msleep(3000);
						}
						if (!icon_state.state)
						{
							icon_char[0] = (lastdata.icon_count == 1 ? 0x00 : 0x20);
						}
					}
					if (!icon_state.state)
					{
						res |= pt6302_write_dcram(0, icon_char, 1);  // set character 0 to end value
						break;
					}
				}
				icon_state.status = THREAD_STATUS_HALTED;
			}
		}
	}

stop_icon:
	icon_state.status = THREAD_STATUS_STOPPED;
	icon_state.task = 0;
	dprintk(100, "%s stopped\n", __func__);
	return res;
}
// end of icon thread code

/*********************************************************************
 *
 * Module functions.
 *
 */
extern void create_proc_fp(void);
extern void remove_proc_fp(void);

//static void __exit vfd_module_exit(void)
static void fp_module_exit(void)
{
	printk(TAGDEBUG"ADB ITI-5800S(X) front processor module unloading\n");
	remove_proc_fp();

	if ((led_text_thread_status != THREAD_STATUS_STOPPED) && led_text_task)
	{
			dprintk(50, "Stopping LED text write thread\n");
			kthread_stop(led_text_task);
	}

	if ((vfd_text_thread_status != THREAD_STATUS_STOPPED) && vfd_text_task)
	{
		dprintk(50, "Stopping VFD text write thread\n");
		kthread_stop(vfd_text_task);
	}

	if (!(spinner_state.status == THREAD_STATUS_STOPPED) && spinner_state.task)
	{
		dprintk(50, "Stopping spinner thread\n");
		up(&icon_state.sem);
		kthread_stop(spinner_state.task);
	}

	if (!(icon_state.status == THREAD_STATUS_STOPPED) && icon_state.task)
	{
		dprintk(50, "Stopping icon thread\n");
		up(&icon_state.sem);
		kthread_stop(icon_state.task);
	}

	while (led_text_thread_status != THREAD_STATUS_STOPPED
	&&     vfd_text_thread_status != THREAD_STATUS_STOPPED
	&&     spinner_state.status != THREAD_STATUS_STOPPED
	&&     icon_state.status != THREAD_STATUS_STOPPED)
	{
		msleep(1);
	}
	dprintk(50, "All threads stopped\n");

	// fan driver
	cleanup_fan_module();

	dprintk(50, "Unregister character device %d\n", VFD_MAJOR);
	unregister_chrdev(VFD_MAJOR, "vfd");
	stpio_free_irq(key_int);
	pt6xxx_free_pio_pins();
	dprintk(50, "Unregister front panel button device %d\n");
	input_unregister_device(button_dev);
	dprintk(150, "%s <\n", __func__);
}

static int __init fp_module_init(void)
{
	int error;
	int ret;
	extern int display_type;

//	msleep(waitTime);
	ret = adb_5800_fp_init_func();
	if (ret)
	{
		goto fp_module_init_fail;
	}
	dprintk(50, "Register character device %d\n", VFD_MAJOR);
	if (register_chrdev(VFD_MAJOR, "vfd", &vfd_fops))
	{
		dprintk(1, "Unable to get major %d for adb_5800_fp driver\n", VFD_MAJOR);
		goto fp_module_init_fail;
	}

	// fan driver
	if (box_variant & 0x02)  // BSLA or BZZB
	{
		init_fan_module();
	}
	// button driver
	dprintk(50, "Request STPIO %d,%d for key interrupt\n", PORT_KEY_INT, PIN_KEY_INT);
	key_int = stpio_request_pin(PORT_KEY_INT, PIN_KEY_INT, "Key_int_front", STPIO_IN);

	if (key_int == NULL)
	{
		dprintk(1, "Request STPIO for key_int failed; abort\n");
		goto fp_module_init_fail;
	}
	stpio_set_pin(key_int, 1);

	dprintk(150, "stpio_flagged_request_irq IRQ_TYPE_LEVEL_LOW\n");
	stpio_flagged_request_irq(key_int, 0, (void *)fpanel_irq_handler, NULL, (long unsigned int)NULL);

	button_dev = input_allocate_device();
	if (!button_dev)
	{
		goto fp_module_init_fail;
	}
	button_dev->name = button_driver_name;
	button_dev->open = button_input_open;
	button_dev->close = button_input_close;

	set_bit(EV_KEY, button_dev->evbit);
	set_bit(KEY_MENU, button_dev->keybit);
	set_bit(KEY_EPG, button_dev->keybit);
	set_bit(KEY_RECORD, button_dev->keybit);
	set_bit(KEY_HOME, button_dev->keybit);
	set_bit(KEY_UP, button_dev->keybit);
	set_bit(KEY_DOWN, button_dev->keybit);
	set_bit(KEY_RIGHT, button_dev->keybit);
	set_bit(KEY_LEFT, button_dev->keybit);
	set_bit(KEY_OK, button_dev->keybit);
	set_bit(KEY_POWER, button_dev->keybit);

	dprintk(50, "Register front panel button device %d\n");
	error = input_register_device(button_dev);
	if (error)
	{
		input_free_device(button_dev);
		dprintk(1, "Request input_register_device failed; abort\n");
		goto fp_module_init_fail;
	}

	if (display_type)  // VFD
	{
		sema_init(&vfd_text_thread_sem, 1);  // initialize VFD text write semaphore
		spinner_state.state = 0;
		spinner_state.period = 0;
		spinner_state.status = THREAD_STATUS_STOPPED;
		sema_init(&spinner_state.sem, 0);
		spinner_state.task = kthread_run(spinner_thread, (void *) ICON_SPINNER, "spinner_thread");
		icon_state.state = 0;
		icon_state.period = 0;
		icon_state.status = THREAD_STATUS_STOPPED;
		sema_init(&icon_state.sem, 0);
		icon_state.task = kthread_run(icon_thread, (void *)ICON_MIN, "icon_thread");
		lastdata.icon_count = 0;
		memset(lastdata.icon_list, 0, sizeof(lastdata.icon_list));
	}
	else
	{
		sema_init(&led_text_thread_sem, 1);  // initialize LED text write semaphore
	}

	create_proc_fp();
	return ret;

fp_module_init_fail:
	fp_module_exit();
	return -EIO;
}

module_init(fp_module_init);
module_exit(fp_module_exit);

module_param(display_led, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(display_led, "Override display type to LED default=auto, 0=off, >0=on");

module_param(display_vfd, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(display_vfd, "Override display type to VFD default= auto, 0=off, >0=on");

module_param(rec_key, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(rec_key, "Override front panel key layout 0=BSKA/BXZB, >0=BSLA/BZZB (default)");

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled (default), >1=enabled (level_");

MODULE_DESCRIPTION("ADB ITI-5800S(X) front panel driver");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
