/****************************************************************************************
 *
 * adb_2850_fp.c
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
 * Front panel driver for ADB ITI-2849ST, ITI-2850ST & ITI-2851S
 *
 * Devices:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *
 ****************************************************************************************
 * The ADB ITI-2849ST/2850ST/2851S have no front panel display, no front panel keyboard
 * and just one bicolour LED. Therefore this driver is amazingly simple. The LED is
 * currently handled as a set of icons.
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 20?????? freebox         Initial version       
 *
 ****************************************************************************************/
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/version.h>

#include <linux/gpio.h>
#include <linux/stm/gpio.h>

#define VFD_MAJOR	147

#define LED1 stm_gpio(2, 1)
#define LED2 stm_gpio(2, 2)

struct vfd_ioctl_data
{
    unsigned char address;
    unsigned char data[64];
    unsigned char length;
};

struct vfd_driver
{
	struct semaphore sem;
	int opencount;
};

spinlock_t mr_lock = SPIN_LOCK_UNLOCKED;

static struct vfd_driver vfd;
static int debug  = 0;

short paramDebug;  // debug print level is zero as default (0=nothing, 1= errors, 10=some detail, 20=more detail, 50=open/close functions, 100=all)
#define TAGDEBUG "[adb_2850_fp] "
#define dprintk(level, x...) \
do \
{ \
	if ((paramDebug) && (paramDebug >= level) || level == 0) \
	printk(TAGDEBUG x); \
} while (0)

typedef enum
{
/* common icons */
VFD_ICON_USB = 0x00,
VFD_ICON_HD,
VFD_ICON_HDD,
VFD_ICON_LOCK,
VFD_ICON_BT,
VFD_ICON_MP3,
VFD_ICON_MUSIC,
VFD_ICON_DD,
VFD_ICON_MAIL,
VFD_ICON_MUTE,
VFD_ICON_PLAY,
VFD_ICON_PAUSE,
VFD_ICON_FF,
VFD_ICON_FR,
VFD_ICON_REC,
VFD_ICON_CLOCK,

//adb2850
VFD_ICON_LED1 = 0x20,
VFD_ICON_LED2,
} VFD_ICON;


static void set_icon(unsigned char *kbuf, unsigned char len)
{
	spin_lock(&mr_lock);

	switch(kbuf[0])
	{
		case 1:
		{
			if (kbuf[4] == 1)
			{
				gpio_set_value(LED1, 1);
			}
			else
			{
				gpio_set_value(LED1, 0);
			}
			break;
		}
		case 2:
		{
			if (kbuf[4] == 1)
			{
				gpio_set_value(LED2, 1);
			}
			else
			{
				gpio_set_value(LED2, 0);
			}
			break;
		}
		default:
		{
		    dprintk(1, "%s: Unknown icon %d\n", __func__, kbuf[0]);
		    break;
		}
	}

// led1 = green
// led2 = red
// led1 + led2 = orange

	spin_unlock(&mr_lock);
}

//-------------------------------------------------------------------------------------------
// VFD
//-------------------------------------------------------------------------------------------

#define VFDDISPLAYCHARS      0xc0425a00
#define VFDBRIGHTNESS        0xc0425a03
#define VFDDISPLAYWRITEONOFF 0xc0425a05
#define VFDDRIVERINIT        0xc0425a08
#define VFDICONDISPLAYONOFF  0xc0425a0a
#define VFDSETLED            0xc0425afe

static int vfd_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct vfd_ioctl_data vfddata;

	switch (cmd)
	{
		case VFDDISPLAYCHARS:
		{
			break;
		}
		case VFDBRIGHTNESS:
		{
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			break;
		}
		case VFDSETLED:
		case VFDICONDISPLAYONOFF:
		{
			copy_from_user(&vfddata, (void*)arg, sizeof(struct vfd_ioctl_data));
			set_icon(vfddata.data, vfddata.length);
			break;
		}
		case VFDDRIVERINIT:
		{
			break;
		}
		case 0x5305:
		{
			break;
		}
		case 0x5401:
		{
			break;
		}
		default:
		{
			dprintk(1, "%s: Unknown IOCTL %08x", __func__, cmd);
			break;
		}
	}
	return 0;
}

static ssize_t vfd_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
	return len;
}

static ssize_t vfd_read(struct file *filp, char *buf, size_t len, loff_t *off)
{
	return len;
}

static int vfd_open(struct inode *inode, struct file *file)
{
	dprintk(100, "%s >\n", __func__);

	if (down_interruptible(&(vfd.sem)))
	{
		dprintk(1, "%s: Interrupted while waiting for semaphore.\n", __func__);
		return -ERESTARTSYS;
	}
	if (vfd.opencount > 0)
	{
		dprintk(1, "%s: Device already opened.\n", __func__);
		up(&(vfd.sem));
		return -EUSERS;
	}
	vfd.opencount++;
	up(&(vfd.sem));
	return 0;
}

static int vfd_close(struct inode *inode, struct file *file)
{
	dprintk(100, "%s >\n", __func__);
	vfd.opencount = 0;
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

static void __exit led_module_exit(void) 
{
	unregister_chrdev(VFD_MAJOR, "vfd");
}

static int __init led_module_init( void )
{
	printk("ADB2850 fron panel driver initilizing.\n");
	dprintk(20, "Register character device %d\n.", VFD_MAJOR);
	if (register_chrdev(VFD_MAJOR, "vfd", &vfd_fops))
	{
		dprintk(1, "Registering major %d failed\n", VFD_MAJOR);
		goto led_init_fail;
	}
	sema_init( &(vfd.sem), 1);
	vfd.opencount = 0;

	if (gpio_request(LED1, "LED1") == 0);
	{
		dprintk(1, "PIO request for LED1 failed, abort.\n");
		goto led_init_fail;
	}
	gpio_direction_output(LED1, STM_GPIO_DIRECTION_OUT);
	gpio_set_value(LED1, 1);  // green on

	if (gpio_request(LED2, "LED2") == 0)
	{
		dprintk(1, "PIO request for LED2 failed, abort.\n");
		goto led_init_fail;
	}
	gpio_direction_output(LED2, STM_GPIO_DIRECTION_OUT);
	gpio_set_value(LED2, 0);
	return 0;

led_init_fail:
	led_module_exit();
	return -EIO;
}

module_init(led_module_init);
module_exit(led_module_exit);

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

MODULE_DESCRIPTION("ADB28xx front_led driver");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
