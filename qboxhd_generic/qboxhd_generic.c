/*
 * 	Copyright (C) 2010 Duolabs Srl
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
#include <linux/devfs_fs_kernel.h>
#endif

#include <linux/poll.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/ioctl.h>
#include <linux/cdev.h>

#include <linux/device.h>
#include <linux/delay.h>

#include <qboxhd_generic.h>

#ifdef CONFIG_SH_QBOXHD_1_0
#define MOD               "-HD"
#elif  CONFIG_SH_QBOXHD_MINI_1_0
#define MOD               "-Mini"
#else
#define MOD               ""
#endif

#define QBX_GENERIC_VERSION       "0.0.4"MOD

static int debug;
#define dprintk(args...) \
	do { \
		if (debug) printk("qboxhd_generic: " args); \
	} while (0)

unsigned int base_address;

static unsigned char inuse;

static void reg_writeonly_l(unsigned int base_address, unsigned int reg, unsigned int val)
{
    unsigned int reg_address;

    reg_address = base_address + reg;
    ctrl_outl(val, reg_address);
}

static struct cdev rst_cdev;


#if defined CONFIG_SH_QBOXHD_MINI_1_0
/**
 * Dummy funcs needed by stmdvb.ko
 */
#include <dvbdev.h>
int setCiSource(int slot, int source)
{
    printk(KERN_INFO "%s - DUMMY FUNCTION \n",__func__);
    return 0;
}

int init_ci_controller(struct dvb_adapter* dvb_adap)
{
    printk(KERN_INFO "%s - DUMMY FUNCTION \n",__func__);
    return 0;
}
EXPORT_SYMBOL(init_ci_controller);
EXPORT_SYMBOL(setCiSource);
#endif

static int
rst_open (struct inode *inode, struct file *file)
{
	unsigned int rst_id;
	int rc = 0;

	dprintk("Opening device!\n");
	rst_id=MINOR(inode->i_rdev);
	if(rst_id<RST_NUMBER_OF_CONTROLLERS)
	{
		if(inuse == 0)
		{
			inuse++;
			dprintk("rst_%d char device available\n", rst_id);
		}
		else
		{
			dprintk("rst_%d char device is busy\n", rst_id);
			rc = -EBUSY;
		}
	}
	else
	{
		dprintk("Couldn't open device with [%d] as MINOR number\n", rst_id);
		rc = -ENODEV;
	}
	return (rc);
}

static int
rst_release (struct inode *inode, struct file *file)
{
	unsigned int rst_id;
	int rc = 0;
	dprintk("Closing device!\n");
	rst_id=MINOR(inode->i_rdev);
	if(rst_id<RST_NUMBER_OF_CONTROLLERS)
	{
		if (inuse > 0)
		{
			inuse--;
		}
		else
		{
			dprintk("Device rst_%d is not opend\n",	rst_id);
			rc = -EINVAL;
		}
	}
	else
	{
		dprintk("Couldn't close device with [%d] as MINOR number\n", rst_id);
		rc = -ENODEV;
	}
	return rc;
}

static ssize_t
rst_read (struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return (-1);
}

static ssize_t
rst_write (struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return (-1);
}

static int
rst_ioctl (struct inode *inode, struct file *file,
              unsigned int cmd, unsigned long arg)
{
	unsigned int rst_id;
	int data;
	int res=-ENOSYS;

	rst_id=MINOR(inode->i_rdev);

	if(rst_id > RST_NUMBER_OF_CONTROLLERS)
		return res;

	switch (cmd)
	{
		case IOCTL_ACTIVATION_RST://1
			dprintk("IOCTL_ACTIVATION_RST\n");
			copy_from_user((void *) &data,
							(const void *) arg,
						    sizeof(data));
			if(data==DEACT_RST)
			{
				printk(" EXIT to the reset of tuner.... DATA: %d\n",data);
				reg_writeonly_l(base_address, PIO_SET_PnOUT, 0x40);
				#if defined CONFIG_SH_QBOXHD_MINI_1_0
				reg_writeonly_l(base_address, PIO_SET_PnOUT, 0x04);
				#endif
			}
			else if(data==ACT_RST)
			{
				printk(" ENTER to the reset of tuner.... DATA: %d\n",data);
				reg_writeonly_l(base_address, PIO_CLR_PnOUT, 0x40);
				#if defined CONFIG_SH_QBOXHD_MINI_1_0
				reg_writeonly_l(base_address, PIO_CLR_PnOUT, 0x04);
				#endif
			}
			else
				printk("Unknow command for IOCTL_ACTIVATION_RST ... parameter value: %d\n", data);
			res=1;
		break;
#if defined CONFIG_SH_QBOXHD_MINI_1_0
		case IOCTL_RST_TUNER_0://100
			dprintk("IOCTL_RST_TUNER_0\n");
			copy_from_user((void *) &data,
							(const void *) arg,
						    sizeof(data));
			if(data==DEACT_RST)
			{
				printk(" EXIT to the reset of tuner 0.... DATA: %d\n",data);
				reg_writeonly_l(base_address, PIO_SET_PnOUT, 0x40);
			}
			else if(data==ACT_RST)
			{
				printk(" ENTER to the reset of tuner 0.... DATA: %d\n",data);
				reg_writeonly_l(base_address, PIO_CLR_PnOUT, 0x40);
			}
			else
				printk("Unknow command for IOCTL_RST_TUNER_0 ... parameter value: %d\n", data);
			res=100;
		case IOCTL_RST_TUNER_1://101
			dprintk("IOCTL_RST_TUNER_1\n");
			copy_from_user((void *) &data,
							(const void *) arg,
						    sizeof(data));
			if(data==DEACT_RST)
			{
				printk(" EXIT to the reset of tuner 1.... DATA: %d\n",data);
				reg_writeonly_l(base_address, PIO_SET_PnOUT, 0x04);
			}
			else if(data==ACT_RST)
			{
				printk(" ENTER to the reset of tuner 1.... DATA: %d\n",data);
				reg_writeonly_l(base_address, PIO_CLR_PnOUT, 0x04);
			}
			else
				printk("Unknow command for IOCTL_RST_TUNER_1 ... parameter value: %d\n", data);
			res=101;
#endif
		default:
			printk("Unknown ioctl code: %d\n",cmd);
		break;
	}
	return res;
}
/*********************************************************************************/
static struct file_operations Fops =
{
    .owner   = THIS_MODULE,
    .open    = rst_open,
    .release = rst_release,
    .read    = rst_read,
    .write   = rst_write,
    .ioctl   = rst_ioctl
};

static int __init qboxhd_generic_init(void)
{
	dev_t dev = MKDEV(MAJOR_NUM, MINOR_START);
	if(register_chrdev_region(dev, RST_NUMBER_OF_CONTROLLERS, DEVICE_NAME) < 0)
	{
		printk("Couldn't register '%s' driver region\n", DEVICE_NAME);
		return -1;
	}
	cdev_init(&rst_cdev, &Fops);
	rst_cdev.owner = THIS_MODULE;
	rst_cdev.ops = &Fops;
	if (cdev_add(&rst_cdev, dev, RST_NUMBER_OF_CONTROLLERS) < 0)
	{
		printk("Couldn't register '%s' driver\n", DEVICE_NAME);
		return -1;
	}

	printk("qboxhd_generic driver: %s\n", QBX_GENERIC_VERSION);

    dprintk("Resetting tuners\n");

	base_address = (unsigned long)ioremap(BASE_ADDRESS, PIO0_IO_SIZE);

	/* Set GPIO 0[6] for output (tuner 0). Datasheet page 213-217 */
    reg_writeonly_l(base_address, PIO_CLR_PnC0, 0x40);
    reg_writeonly_l(base_address, PIO_SET_PnC1, 0x40);
    reg_writeonly_l(base_address, PIO_CLR_PnC2, 0x40);

#if defined CONFIG_SH_QBOXHD_MINI_1_0
	/* Set GPIO 0[2] for output (tuner 1). Datasheet page 213-217 */
    reg_writeonly_l(base_address, PIO_CLR_PnC0, 0x04);
    reg_writeonly_l(base_address, PIO_SET_PnC1, 0x04);
    reg_writeonly_l(base_address, PIO_CLR_PnC2, 0x04);
#endif

    /* Set it low, wait 150ms, and set it high */
    reg_writeonly_l(base_address, PIO_CLR_PnOUT, 0x40);
#if defined CONFIG_SH_QBOXHD_MINI_1_0
    /* Set it low, wait 150ms, and set it high */
    reg_writeonly_l(base_address, PIO_CLR_PnOUT, 0x04);
#endif
    mdelay(150);
    reg_writeonly_l(base_address, PIO_SET_PnOUT, 0x40);
#if defined CONFIG_SH_QBOXHD_MINI_1_0
    /* Set it low, wait 150ms, and set it high */
    reg_writeonly_l(base_address, PIO_SET_PnOUT, 0x04);
#endif

	return 0;
}

static void __exit qboxhd_generic_exit(void)
{
	dev_t dev = MKDEV(MAJOR_NUM, MINOR_START);
	unregister_chrdev_region(dev, RST_NUMBER_OF_CONTROLLERS);

    cdev_del(&rst_cdev);

    iounmap((void *)base_address);
}

module_init(qboxhd_generic_init);
module_exit(qboxhd_generic_exit);


module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off frontend debugging (default:off).");

MODULE_DESCRIPTION("Set generic GPIO's of QBox HD");
MODULE_AUTHOR("Pedro Aguilar");
MODULE_LICENSE("GPL");
MODULE_VERSION(QBX_GENERIC_VERSION);

