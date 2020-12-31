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

#include "scart_mini.h"


/* 
 * GPIO for give to tv the scart signals
 */
#define BASE_ADDRESS	    0x18022000 /* Port 2 */
#define PIO0_IO_SIZE        0x1000
#define PIO_CLR_PnC0        0x28
#define PIO_CLR_PnC1        0x38
#define PIO_CLR_PnC2        0x48
#define PIO_SET_PnC0        0x24
#define PIO_SET_PnC1        0x34
#define PIO_SET_PnC2        0x44

#define PIO_CLR_PnOUT       0x08
#define PIO_SET_PnOUT       0x04




static int debug;
#define dprintk(args...) \
	do { \
		if (debug) printk("scart_mini: " args); \
	} while (0)

unsigned int base_address;
static unsigned char inuse;

static void reg_writeonly_l(unsigned int base_address, unsigned int reg, unsigned int val)
{
    unsigned int reg_address;
    
    reg_address = base_address + reg;
    ctrl_outl(val, reg_address);
}

static struct cdev scart_cdev;

static Scart_Status_st status;

/*********************************************************************************/
static int
scart_open (struct inode *inode, struct file *file)
{
	unsigned int scart_id;
	int rc = 0;

	dprintk("Opening device!\n");
	scart_id=MINOR(inode->i_rdev);
	if(scart_id<SCART_NUMBER_OF_CONTROLLERS)
	{
		if(inuse == 0)
		{	
			inuse++;
			dprintk("scart_%d char device available\n", scart_id);
		}
		else
		{
			dprintk("scart_%d char device is busy\n", scart_id);
			rc = -EBUSY;
		}
	}
	else
	{
		dprintk("Couldn't open device with [%d] as MINOR number\n", scart_id);
		rc = -ENODEV;
	}
	return (rc);
}

static int
scart_release (struct inode *inode, struct file *file)
{
	unsigned int scart_id;
	int rc = 0;
	dprintk("Closing device!\n");
	scart_id=MINOR(inode->i_rdev);
	if(scart_id<SCART_NUMBER_OF_CONTROLLERS)
	{
		if (inuse > 0)
		{
			inuse--;
		}
		else
		{
			dprintk("Device scart_%d is not opend\n",	scart_id);
			rc = -EINVAL;
		}
	}
	else
	{
		dprintk("Couldn't close device with [%d] as MINOR number\n", scart_id);
		rc = -ENODEV;
	}
	return rc;
}

static ssize_t
scart_read (struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return (-1);
}

static ssize_t
scart_write (struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return (-1);
}

static int
scart_ioctl (struct inode *inode, struct file *file,
              unsigned int cmd, unsigned long arg)
{
	unsigned int scart_id;
	int data;
	int res=-ENOSYS;
	
	
	scart_id=MINOR(inode->i_rdev);
		
	if(scart_id > SCART_NUMBER_OF_CONTROLLERS)
		return res;

	switch (cmd)
	{
		case IOCTL_SET_ASPECT_RATIO://1
			dprintk("IOCTL_SET_ASPECT_RATIO\n");
			copy_from_user((void *) &data,
							(const void *) arg,
						    sizeof(data));
			if(data==AR_4_3)
			{
				printk(" Set the aspect ratio to 4/3 ... DATA: %d\n",data);
				reg_writeonly_l(base_address, PIO_SET_PnOUT, SCART_CON);	/* Enable */
				reg_writeonly_l(base_address, PIO_CLR_PnOUT, SCART_AR);
				status.scart_cond=EN_SCART;
				status.scart_ar=AR_4_3;
			}
			else if(data==AR_16_9)
			{			
				printk("Set the aspect ratio to 16/9 ... DATA: %d\n",data);
				reg_writeonly_l(base_address, PIO_SET_PnOUT, SCART_CON);	/* Enable */
				reg_writeonly_l(base_address, PIO_SET_PnOUT, SCART_AR);
				status.scart_cond=EN_SCART;
				status.scart_ar=AR_16_9;
			}			
			else
				printk("Unknow command for IOCTL_SET_ASPECT_RATIO ... parameter value: %d\n", data);
			res=1;
		break;
		case IOCTL_FORMAT_COLOR://2
			dprintk("IOCTL_FORMAT_COLOR\n");
			copy_from_user((void *) &data,
							(const void *) arg,
						    sizeof(data));
			if(data==RGB)
			{
				printk("Set scart in RGB ... DATA: %d\n",data);
				reg_writeonly_l(base_address, PIO_SET_PnOUT, SCART_CON);	/* Enable */
				reg_writeonly_l(base_address, PIO_CLR_PnOUT, SCART_FORMAT);
				status.scart_cond=EN_SCART;
				status.scart_f=RGB;
			}
			else if(data==CVBS)
			{			
				printk("Set scart in CVBS ... DATA: %d\n",data);
				reg_writeonly_l(base_address, PIO_SET_PnOUT, SCART_CON);	/* Enable */
				reg_writeonly_l(base_address, PIO_SET_PnOUT, SCART_FORMAT);
				status.scart_cond=EN_SCART;
				status.scart_f=CVBS;
			}			
			else
				printk("Unknow command for IOCTL_FORMAT_COLOR ... parameter value: %d\n", data);
			res=2;
		break;
		case IOCTL_ENABLE_SCART://3
			dprintk("IOCTL_ENABLE_SCART\n");
			copy_from_user((void *) &data,
							(const void *) arg,
						    sizeof(data));
			if(data==DIS_SCART)
			{
				printk("Disable SCART_CON ... DATA: %d\n",data);
				reg_writeonly_l(base_address, PIO_CLR_PnOUT, SCART_CON);
				status.scart_cond=DIS_SCART;
			}
			else if(data==EN_SCART)
			{			
				printk("Enable SCART_CON ... DATA: %d\n",data);
				reg_writeonly_l(base_address, PIO_SET_PnOUT, SCART_CON);
				status.scart_cond=EN_SCART;
			}			
			else
				printk("Unknow command for IOCTL_ENABLE_SCART ... parameter value: %d\n", data);
			res=3;
		break;
		case IOCTL_SCART_STATUS://4
			dprintk("IOCTL_SCART_STATUS\n");
			copy_to_user((void *) arg, 
					(const void *) &status,
					sizeof(Scart_Status_st));
			res=4;
		break;
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
    .open    = scart_open,
    .release = scart_release,
    .read    = scart_read,
    .write   = scart_write,
    .ioctl   = scart_ioctl
};

static int __init scart_mini_init(void)
{
	dev_t dev = MKDEV(MAJOR_NUM, MINOR_START);
	if(register_chrdev_region(dev, SCART_NUMBER_OF_CONTROLLERS, DEVICE_NAME) < 0)
	{
		printk("Couldn't register '%s' driver region\n", DEVICE_NAME);
		return -1;
	}
	cdev_init(&scart_cdev, &Fops);
	scart_cdev.owner = THIS_MODULE;
	scart_cdev.ops = &Fops;
	if (cdev_add(&scart_cdev, dev, SCART_NUMBER_OF_CONTROLLERS) < 0)
	{
		printk("Couldn't register '%s' driver\n", DEVICE_NAME);
		return -1;
	}

	printk("Version of scart_mini driver: 0.0.1\n");

/*************/
    dprintk("Set the GPIOS in output\n");

	base_address = (unsigned long)ioremap(BASE_ADDRESS, PIO0_IO_SIZE);

	/* Set GPIO 0[6] for output. Datasheet page 213-217 */
    reg_writeonly_l(base_address, PIO_CLR_PnC0, SCART_AR);	/* Output */
    reg_writeonly_l(base_address, PIO_SET_PnC1, SCART_AR);
    reg_writeonly_l(base_address, PIO_CLR_PnC2, SCART_AR);

    reg_writeonly_l(base_address, PIO_CLR_PnC0, SCART_FORMAT);	/* Output */
    reg_writeonly_l(base_address, PIO_SET_PnC1, SCART_FORMAT);
    reg_writeonly_l(base_address, PIO_CLR_PnC2, SCART_FORMAT);

    reg_writeonly_l(base_address, PIO_CLR_PnC0, SCART_CON);	/* Output */
    reg_writeonly_l(base_address, PIO_SET_PnC1, SCART_CON);
    reg_writeonly_l(base_address, PIO_CLR_PnC2, SCART_CON);

/*************/

	/* Enable scart ... */
	reg_writeonly_l(base_address, PIO_SET_PnOUT, SCART_CON);
	status.scart_cond=EN_SCART;
	/* ... in CVBS ... */
	reg_writeonly_l(base_address, PIO_SET_PnOUT, SCART_FORMAT);
	status.scart_f=CVBS;
	/* ... and 16/9 */
	reg_writeonly_l(base_address, PIO_SET_PnOUT, SCART_AR);
	status.scart_ar=AR_16_9;

	status.scart_ar=-1;
	status.scart_f=-1;
	status.scart_cond=-1;

	return 0;
}

static void __exit scart_mini_exit(void) 
{
	dev_t dev = MKDEV(MAJOR_NUM, MINOR_START);
	unregister_chrdev_region(dev, SCART_NUMBER_OF_CONTROLLERS);
    
    cdev_del(&scart_cdev);

    iounmap((void *)base_address);
}

module_init(scart_mini_init);
module_exit(scart_mini_exit);


module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off frontend debugging (default:off).");

#define SCART_VERSION       "0.0.1"
MODULE_VERSION(SCART_VERSION);

MODULE_DESCRIPTION("Set generic GPIO's of QBoxHD mini for scart");
MODULE_AUTHOR("Michele Cecchin");
MODULE_LICENSE("GPL");
