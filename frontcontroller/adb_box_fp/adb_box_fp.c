/*
 * adb_box_fp.c
 *
 * (c) 20?? B4Team?
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
 * Front panel driver for nBOX ITI-5800S(X), BSKA, BSLA, BXXB and BZZB variants;
 * generic part: driver code for VFD & front panel buttons.
 *
 * Devices:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *  - /dev/rc  (reading of key events)
 *
 *
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 20130929 B4team?         Initial version
 * 20190723 Audioniek       start of major rewrite.
 * 20190??? Audioniek       procfs added.
 *
 ****************************************************************************************/

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
#include <linux/stm/pio.h>
#else
#include <linux/stpio.h>
#endif

#include "adb_box_fp.h"
#include "pio.h"

/* <-----buttons-----> */
#include <linux/input.h>
#include <linux/version.h>
#include <linux/workqueue.h>

// Global variables (module parameters)
int paramDebug = 0;  // debug level
int delay      = 5;
static int rec = 1;  // distinguishes between recorder(1) / non-recorder(0) (bsla/bzzb vs. bska/bxzb)

// Globals (button routines)
static int           button_polling = 1;
unsigned char        key_group1 = 0, key_group2 = 0;
static unsigned int  key_button = 0;
static unsigned char pt6958_ram[PT6958_RAM_SIZE];

//int SCP_PORT = 0;

struct __vfd_scp *vfd_scp_pt6302_drv = NULL;

/* <-----button assignment-----> */
/*KEY  CODE1  CODE2
  POW    2      0
  OK     4      0
  UP    20      0
  DOWN   0      2
  LEFT   0      4
  RIGHT 80      0
  EPG   40      0
  REC    0      8
  RES    8      0
*/

/*****************************************************
 *
 * Code for front panel button driver 
 *
 */

/*****************************************************
 *
 * button_poll
 *
 * Scan for pressed front panel keys
 *
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
void button_poll(void)
#else
void button_poll(struct work_struct *ignored)
#endif
{
	while (button_polling == 1)
	{
		msleep(BUTTON_POLL_MSLEEP);

		if (FP_GETKEY)
		{
			pt6958_write(PT6958_CMD_READ_KEY, NULL, 0);

			udelay(1);

			key_group1 = pt6958_read_key() & 0xfe;
			key_group2 = pt6958_read_key() & 0xfe;

			dprintk(0, "Key group [%X:%X]\n", key_group1, key_group2);

			if ((key_group1 == 0x02) && (key_group2 == 0x00))
			{
				key_button = KEY_POWER;
			}
			if ((key_group1 == 0x04) && (key_group2 == 0x00))
			{
				key_button = KEY_OK;
			}
			if ((key_group1 == 0x20) && (key_group2 == 0x00))
			{
				key_button = KEY_UP;
			}
			if ((key_group1 == 0x00) && (key_group2 == 0x02))
			{
				key_button = KEY_DOWN;
			}
			if ((key_group1 == 0x00) && (key_group2 == 0x04))
			{
				key_button = KEY_LEFT;
			}
			if ((key_group1 == 0x80) && (key_group2 == 0x00))
			{
				key_button = KEY_RIGHT;
			}
			if ((key_group1 == 0x40) && (key_group2 == 0x00))
			{
				if (rec)  // if bsla/bzzb variant
				{
					key_button = KEY_EPG;
				}
				else  // bska/bxzb variant
				{
					key_button = KEY_MENU;
				}
			}
			if ((key_group1 == 0x00) && (key_group2 == 0x08))
			{
				if (rec)  // if bsla/bzzb variant
				{
					key_button = KEY_RECORD;
				}
				else  // bska/bxzb variant
				{
					key_button = KEY_EPG;
				}
			}
			if ((key_group1 == 0x08) && (key_group2 == 0x00))
			{
				key_button = KEY_HOME;  // RES
			}
			if ((key_group1 == 0x0A) && (key_group2 == 0x00))  // RES + OK
			{
				dprintk(0, " reboot...\n");
				msleep(250);
				button_reset = stpio_request_pin(3, 2, "btn_reset", STPIO_OUT);
			}
			if ((key_group1 == 0x08) && (key_group2 == 0x2))
			{
				key_button = KEY_COMPOSE;  //RES + DOWN
			}
			if (key_button > 0)
			{
				dprintk(1, "fp_key: %d\n", key_button);
				input_report_key(button_dev, key_button, 1);
				input_sync(button_dev);
				msleep(250);

				input_report_key(button_dev, key_button, 0);
				input_sync(button_dev);

				// clean up variables
				key_group1 = 0;
				key_group2 = 0;
				key_button = 0;
			}
		}
	}
}

/*****************************************************
 *
 * button_input_open
 *
 * create queue for panel keys
 *
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static DECLARE_WORK(button_obj, button_poll, NULL);
#else
static DECLARE_WORK(button_obj, button_poll);
#endif

static int button_input_open(struct input_dev *dev)
{
	button_wq = create_workqueue("button");
	if (queue_work(button_wq, &button_obj))
	{
		dprintk(10, "queue_work successful...\n");
	}
	else
	{
		dprintk(1, "queue_work not successful, exiting...\n");
		return 1;
	}
	return 0;
}

/*****************************************************
 *
 * button_input_close
 *
 * destroy queue for panel keys
 *
 */
static void button_input_close(struct input_dev *dev)
{
	button_polling = 0;
	msleep(55);
	button_polling = 1;

	if (button_wq)
	{
		destroy_workqueue(button_wq);
		dprintk(10, "workqueue destroyed.\n");
	}
}

/******************************************************
 *
 * button_dev_init
 *
 * Initialize the front panel button driver.
 *
 */
int button_dev_init(void)
{
	int error;

	dprintk(100, "%s <\n", __func__);

// PT6958 DOUT
	dprintk(150, "PT6958 DOUT pin: request STPIO %d, %d (%s, STPIO_IN)\n", BUTTON_PIO_PORT_DOUT, BUTTON_PIO_PIN_DOUT, "btn_dout");
	button_dout = stpio_request_pin(BUTTON_PIO_PORT_DOUT, BUTTON_PIO_PIN_DOUT, "btn_dout", STPIO_IN);  // [ IN  (Hi-Z) ]
	if (button_dout == NULL)
	{
		dprintk(1, "Request STPIO btn_dout failed; abort.\n");
		return -EIO;
	}
// PT6958 STB
	dprintk(150, "PT6958 STB pin: request STPIO %d, %d (%s, STPIO_OUT)\n", BUTTON_PIO_PORT_STB, BUTTON_PIO_PIN_STB, "btn_stb");
	button_stb = stpio_request_pin(BUTTON_PIO_PORT_STB, BUTTON_PIO_PIN_STB, "btn_stb", STPIO_OUT);  // [ OUT (push-pull) ]
	if (button_stb == NULL)
	{
		dprintk(1, "Request STPIO btn_stb failed; abort.\n");
		return -EIO;
	}

//reset (invoked on RES + OK, resets CPU?)
#if 0
	dprintk(150, "PT6302 RST pin: request STPIO %d ,%d (%s, STPIO_OUT\n", BUTTON_PIO_PORT_RESET, BUTTON_PIO_PIN_RESET, "btn_reset");
	button_reset = stpio_request_pin(BUTTON_PIO_PORT_RESET, BUTTON_PIO_PIN_RESET, "btn_reset", STPIO_OUT);
	if (button_reset == NULL)
	{
		dprintk(1, "Request STPIO btn_reset failed; abort.\n");
		return -EIO;
	}
#endif

	dprintk(10, "Allocating and registering button device...\n");

	button_dev = input_allocate_device();
	if (!button_dev)
	{
		return -ENOMEM;
	}
	button_dev->name  =  button_driver_name;
	button_dev->open  =  button_input_open;
	button_dev->close =  button_input_close;

	set_bit(EV_KEY,    	 button_dev->evbit);
	set_bit(KEY_UP,    	 button_dev->keybit);
	set_bit(KEY_DOWN,  	 button_dev->keybit);
	set_bit(KEY_LEFT,  	 button_dev->keybit);
	set_bit(KEY_RIGHT, 	 button_dev->keybit);
	set_bit(KEY_OK,      button_dev->keybit);
	set_bit(KEY_POWER, 	 button_dev->keybit);
	set_bit(KEY_MENU,  	 button_dev->keybit);
	set_bit(KEY_EPG,   	 button_dev->keybit);
	set_bit(KEY_HOME, 	 button_dev->keybit);
	set_bit(KEY_RESTART, button_dev->keybit);  // RES + OK
	set_bit(KEY_COMPOSE, button_dev->keybit);  // RES + DOWN

	error = input_register_device(button_dev);
	if (error)
	{
		input_free_device(button_dev);
		dprintk(1, "Error registering button device!\n");
		return error;
	}
	memset(pt6958_ram, 0, PT6958_RAM_SIZE);

	pt6958_write(PT6958_CMD_WRITE_INC, NULL, 0);

	pt6958_write(PT6958_CMD_ADDR_SET, pt6958_ram, PT6958_RAM_SIZE);
	pt6958_write(PT6958_CMD_DISPLAY_ON, NULL, 0);  // LED display off?
//	printk(" done.\n");
	return 0;
}

/******************************************************
 *
 * button_dev_exit
 *
 * Stop the front panel button driver.
 *
 */
void button_dev_exit(void)
{
	dprintk(0, "Unregistering button device");
	input_unregister_device(button_dev);
	// free button driver pio pins
	if (button_stb)
	{
		stpio_free_pin(button_stb);
	}
	if (button_dout)
	{
		stpio_free_pin(button_dout);
	}
#if 0  // reset pin does not exist on PT6958
	if (button_reset)
	{
		stpio_free_pin(button_reset);
	}
#endif
	printk(" done.\n");
}

/*****************************************************
 *
 * Frontpanel VFD driver
 *
 */

/******************************************************
 *
 * vfd_pin_free
 *
 * Remove the PT6302 control pin driver.
 *
 */
static void vfd_pin_free(struct pt6302_pin_driver *vfd_pin)
{
	if (vfd_pin == NULL)
	{
		return;
	}
	if (vfd_pin->pt6302_cs)
	{
		stpio_set_pin(vfd_pin->pt6302_cs, 1);  // deselect PT6302
	}
	if (vfd_pin->pt6302_din)
	{
		stpio_free_pin(vfd_pin->pt6302_din);
	}
	if (vfd_pin->pt6302_clk)
	{
		stpio_free_pin(vfd_pin->pt6302_clk);
	}
	if (vfd_pin->pt6302_cs)
	{
		stpio_free_pin(vfd_pin->pt6302_cs);
	}
	if (vfd_pin)
	{
		kfree(vfd_pin);
	}
	vfd_pin = NULL;
	dprintk(10, "Removed front panel PT6302 control pin driver.\n");
};

/******************************************************
 *
 * vfd_pin_init
 *
 * Initialize the PT6302 control pin driver.
 *
 */
static struct pt6302_pin_driver *vfd_pin_init(void)
{
	struct pt6302_pin_driver *pt6302_pin_drv = NULL;

	dprintk(100, "%s >\n", __func__);

	pt6302_pin_drv = (struct pt6302_pin_driver *)kzalloc(sizeof(struct pt6302_pin_driver), GFP_KERNEL);
	if (pt6302_pin_drv == NULL)
	{
		dprintk(1, "Unable to allocate pt6302_pin_driver struct; abort.\n");
		goto vfd_pin_init_fail;
	}
	/* Allocate pio pins for the PT6302 */
	dprintk(150, "PT6302 CS pin: request STPIO %d, %d (%s, STPIO_OUT).\n", VFD_PIO_PORT_CS, VFD_PIO_PIN_CS, stpio_vfd_cs);
	pt6302_pin_drv->pt6302_cs = stpio_request_pin(VFD_PIO_PORT_CS, VFD_PIO_PIN_CS, stpio_vfd_cs, STPIO_OUT);

	if (pt6302_pin_drv->pt6302_cs == NULL)
	{
		dprintk(1, "Request CS STPIO failed; abort\n");
		goto vfd_pin_init_fail;
	}

	dprintk(150, "PT6302 CLK pin: request STPIO %d, %d (%s, STPIO_OUT).\n", VFD_PIO_PORT_CLK, VFD_PIO_PIN_CLK, stpio_vfd_clk);
	pt6302_pin_drv->pt6302_clk = stpio_request_pin(VFD_PIO_PORT_CLK, VFD_PIO_PIN_CLK, stpio_vfd_clk, STPIO_OUT);

	if (pt6302_pin_drv->pt6302_clk == NULL)
	{
		dprintk(1, "Request CLK STPIO failed; abort.\n");
		goto vfd_pin_init_fail;
	}

	dprintk(150, "PT6302 DIN pin: request STPIO %d, %d (%s, STPIO_BIDIR).\n", VFD_PIO_PORT_DIN, VFD_PIO_PIN_DIN, stpio_vfd_din);
	pt6302_pin_drv->pt6302_din = stpio_request_pin(VFD_PIO_PORT_DIN, VFD_PIO_PIN_DIN, stpio_vfd_din, STPIO_BIDIR);

	if (pt6302_pin_drv->pt6302_din == NULL)
	{
		dprintk(1, "Request DIN STPIO failed; abort.\n");
		goto vfd_pin_init_fail;
	}
	dprintk(100, "%s <\n", __func__);
	return pt6302_pin_drv;

vfd_pin_init_fail:
	vfd_pin_free(pt6302_pin_drv);
	dprintk(100, "%s < (fail)\n", __func__);
	return 0;
}

static struct vfd_driver vfd;

/*****************************************************
 *
 * Write to /dev/vfd (display text)
 *
 * Writes a text on the VFD display
 * TODO: add LED display, add scrolling
 *
 */
static ssize_t vfd_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
	unsigned char *kbuf;
	size_t        wlen;

	dprintk(10, "write: len = %d (max. %d), off = %d.\n", len, VFD_MAX_CHARS, (int)*off);

	if (len == 0)
	{
		return len;
	}
	kbuf = kmalloc(len, GFP_KERNEL);
	if (kbuf == NULL)
	{
		dprintk(1, "Unable to allocate kernel write buffer\n");
		return -ENOMEM;
	}
	copy_from_user(kbuf, buf, len);

	wlen = len;
	if (kbuf[len - 1] == '\n')  // remove trailing new line
	{
		kbuf[len - 1] = '\0';
		wlen--;
	}
// TODO: add scrolling?
	if (wlen > VFD_MAX_CHARS)  // display VFD_MAX_CHARS maximum
	{
		wlen = VFD_MAX_CHARS;
	}
	dprintk(10, "write : len = %d, wlen = %d, kbuf = '%s'.\n", len, wlen, kbuf);

	pt6302_write_dcram(vfd.pt6302_drv, 0, kbuf, wlen);
	return len;
}

/*****************************************************
 *
 * Read from /dev/vfd
 *
 */
static ssize_t vfd_read(struct file *filp, char *buf, size_t len, loff_t *off)
{
	// TODO: return current display string
	return len;
}

/*****************************************************
 *
 * vfd_open
 *
 * Open /dev/vfd
 *
 */
static int vfd_open(struct inode *inode, struct file *file)
{
	if (down_interruptible(&(vfd.sem)))
	{
		dprintk(10, "interrupted while waiting for semaphore.\n");
		return -ERESTARTSYS;
	}
	if (vfd.opencount > 0)
	{
		dprintk(10, "device already opened.\n");
		up(&(vfd.sem));
		return -EUSERS;
	}
	vfd.opencount++;
	up(&(vfd.sem));

	pt6302_set_light(vfd.pt6302_drv, PT6302_LIGHT_NORMAL);
	return 0;
}

/*****************************************************
 *
 * vfd_open
 *
 * Close /dev/vfd
 *
 */
static int vfd_close(struct inode *inode, struct file *file)
{
	//pt6302_set_light(vfd.pt6302_drv, PT6302_LIGHTS_OFF);
	vfd.opencount = 0;
	return 0;
}

/******************************************************
 *
 * fp_module_init
 *
 * Stop the front panel driver.
 *
 */
static void fp_module_exit(void)
{
	dprintk(10, "Button driver unloading ...");
	button_dev_exit();
	printk(" done.\n");

	dprintk(10, "PT6302 driver unloading.\n");
	unregister_chrdev(VFD_MAJOR, "vfd");
	if (vfd.pt6302_drv)
	{
		pt6302_free(vfd.pt6302_drv);  // stop PT6302 driver
	}
	if (vfd.pt6302_pin_drv)
	{
		vfd_pin_free(vfd.pt6302_pin_drv);  // stop PT6302 pin driver
	}
	vfd.pt6302_drv = NULL;
	vfd.pt6302_pin_drv  = NULL;
	printk(" done.\n");
}

static struct file_operations vfd_fops;

/******************************************************
 *
 * fp_module_init
 *
 * Initialize the front panel driver.
 *
 */
static int __init fp_module_init(void)
{
	dprintk(0, "Front panel PT6302 & PT6958 driver\n");
	dprintk(1, "Driver initializing...\n");

	if (rec != 0)  // limit rec values to 0 (bska/bxzb) or 1 (bxzb/bzzb)
	{
		rec = 1;
	}

	vfd.pt6302_pin_drv = NULL;  // PT6302 pin driver
	vfd.pt6302_drv = NULL;  // PT6302 chip driver

	dprintk(10, "Initialize PT6302 pin driver.\n");
	vfd.pt6302_pin_drv = vfd_pin_init();
	if (vfd.pt6302_pin_drv == NULL)
	{
		dprintk(1, "Unable to init PT6302 pin driver; abort.\n");
		goto fp_init_fail;
	}
//	dprintk(10, " done.\n");

	dprintk(10, "Initialize PT6302 chip driver.\n");
	vfd.pt6302_drv = pt6302_init(vfd.pt6302_pin_drv);
	if (vfd.pt6302_drv == NULL)
	{
		dprintk(1, "Unable to init PT6302 chip driver; abort.\n");
		goto fp_init_fail;
	}
//	dprintk(10, " done\n");

	dprintk(10, "Register character device %d.\n", VFD_MAJOR);
	if (register_chrdev(VFD_MAJOR, "vfd", &vfd_fops))
	{
		dprintk(1, "Registering major %d failed; abort.\n", VFD_MAJOR);
		goto fp_init_fail;
	}
	sema_init(&(vfd.sem), 1);
	vfd.opencount = 0;
//	dprintk(10, " done.\n");

	dprintk(10, "Initializing button driver.\n");
	if (button_dev_init() != 0)
	{
		dprintk(1, "Initializing button driver failed; abort.\n");
		goto fp_init_fail;
	}
//	printk(" done.\n");

	pt6958_led_control(PT6958_CMD_ADDR_LED1, 2);  // power LED: green
	pt6302_setup(vfd.pt6302_drv);  // initialize VFD display
	dprintk(1, "Driver initialized successfully.\n");
	return 0;

fp_init_fail:
	fp_module_exit();
	dprintk(1, "Driver initialization failed.\n");
	return -EIO;
}

/*****************************************************
 *
 * Frontpanel IOCTL
 *
 */
static int vfd_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct vfd_ioctl_data vfddata;

	switch (cmd)  // get data arguments
	{
		case VFDIOC_DCRAMWRITE:
		case VFDIOC_BRIGHTNESS:
		case VFDIOC_DISPLAYWRITEONOFF:
		case VFDIOC_ICONDISPLAYONOFF:
		{
			copy_from_user(&vfddata, (void *)arg, sizeof(struct vfd_ioctl_data));
			break;
		}
	}

	switch (cmd)  // do actual IOCTL
	{
		case VFDIOC_DCRAMWRITE:
		{
			return pt6302_write_dcram(vfd.pt6302_drv, vfddata.address, vfddata.data, vfddata.length);
			break;
		}
		case VFDIOC_BRIGHTNESS:
		{
			pt6302_set_brightness(vfd.pt6302_drv, vfddata.address);
//			pt6958_set_brightness(vfd.pt6302_drv, vfddata.address);
			break;
		}
		case VFDIOC_DISPLAYWRITEONOFF:
		{
			pt6302_set_light(vfd.pt6302_drv, vfddata.address);
//			pt6958_set_light(vfd.pt6302_drv, vfddata.address);
			break;
		}
		case VFDIOC_ICONDISPLAYONOFF:
		{
			//pt_6958_set_icon(vfddata.address, vfddata.data, vfddata.length);  // icons and other farts
			//pt_6958_set_icon(data );  // icons and other farts

			//dprintk(10, "VFDICONDISPLAYONOFF %x:%x", vfddata.data[0], vfddata.data[4]);

			if (vfddata.data[0] != 0x00000023)
			{
				dprintk(10,"VFDICONDISPLAYONOFF %x:%x\n", vfddata.data[0], vfddata.data[4]);
			}
			switch (vfddata.data[0])
			{
				// VFD driver of enigma2 issues codes during the start
				// phase, map them to the CD segments to indicate progress
				case VFD_LED_IR:
				{
					pt6958_led_control(PT6958_CMD_ADDR_LED1, vfddata.data[4] ? 2 : 1); // power LED: green or red 
					break;
				}
				case VFD_LED_POW:
				{
					pt6958_led_control(PT6958_CMD_ADDR_LED1, vfddata.data[4] ? 2 : 0);  // power LED: green or off 
					break;
				}
				case VFD_LED_REC:
				{
					dprintk(1, "REC LED:%x\n", vfddata.data[4]);
					pt6958_led_control(PT6958_CMD_ADDR_LED2, vfddata.data[4] ? 1 : 0); // recording/timer LED
					break;
				}
				case VFD_LED_PAUSE:  // controls power LED colour
				{
					pt6958_led_control(PT6958_CMD_ADDR_LED1, vfddata.data[4] ? 3 : 2);  // pause: power LED yellow, else green
					break;
				}
				case VFD_LED_LOCK:
				{
					pt6958_led_control(PT6958_CMD_ADDR_LED3, vfddata.data[4] ? 1 : 0);  // at LED
					break;
				}
				case VFD_LED_HD:
				{
					pt6958_led_control(PT6958_CMD_ADDR_LED4, vfddata.data[4] ? 1 : 0); // alert LED
					break;
				}
				default:
				{
					return 0;
				}
			}
			break;
		}
		case VFDIOC_DRIVERINIT:
		{
			pt6302_setup(vfd.pt6302_drv);
//			pt6958_setup(vfd.pt6302_drv);
			break;
		}
		default:
		{
			dprintk(1, "unknown IOCTL %08x\n", cmd);
			break;
		}
	}
	return 0;
}

static struct file_operations vfd_fops =
{
	.owner   = THIS_MODULE,
	.ioctl   = vfd_ioctl,
	.write   = vfd_write,
	.read    = vfd_read,
	.open    = vfd_open,
	.release = vfd_close
};

module_init(fp_module_init);
module_exit(fp_module_exit);

module_param(paramDebug, int, 0644);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

module_param(delay, int, 0644);
MODULE_PARM_DESC(delay, "scp delay (default 5");

module_param(rec, int, 0644);
MODULE_PARM_DESC(rec, "Receiver model 1=recorder(default), other=no recorder");

MODULE_DESCRIPTION("Front panel PT6302 & PT6958 driver");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
