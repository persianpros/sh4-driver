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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/hw_irq.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/delay.h>

#include "fpanel.h"

dev_t dev_nr;
struct cdev *fpanel_dev;
static Global_Status_t status;
static unsigned char change_status;

/* For debug */
static int debug;

DECLARE_MUTEX(read_sem);

/* Interrupt for pressing or releasing the button */
irqreturn_t fpanel_irq_handler (int irq, void *dev_id)
{
	int supp=0;

	disable_irq(FPANEL_PORT5_IRQ);
    
	supp=ctrl_inl(status.pio_addr+PIO_PnIN);

	if( (supp&BUTTON_PIN) == BUTTON_PRESSED )
	{
		dprintk("BUTTON_PRESSED\n");
		ctrl_outl(BUTTON_PIN, status.pio_addr+PIO_CLR_PnCOMP);	/* It is dual to RELEASED */
		status.button=BUTTON_PRESSED;
		change_status=1;
	}
	else if( (supp&BUTTON_PIN) == BUTTON_RELEASED )
	{
		dprintk("BUTTON_RELEASED\n");
		ctrl_outl(BUTTON_PIN, status.pio_addr+PIO_SET_PnCOMP); /* It is dual to PRESSED */
		status.button=BUTTON_RELEASED;
		change_status=1;
	}
	else
		dprintk("ERROR!!\n");

    enable_irq(FPANEL_PORT5_IRQ);

    return IRQ_HANDLED;
}

static int fpanel_open( struct inode* inode, struct file* file )
{
	int rc = 0;

	dprintk("Device opened\n");

	return (rc);
}

static int fpanel_release( struct inode* inode, struct file* file )
{
	int rc = 0;

	dprintk("Device closed\n");

	return rc;
}

static ssize_t fpanel_read( struct file *filp, char __user *buf, size_t count, loff_t *f_ops )
{
	ssize_t retval = 0;
	unsigned int supp=0;

	if (down_interruptible(&read_sem) != 0)	
		return -ERESTARTSYS;

	if(status.button==BUTTON_PRESSED)
		supp=BUTTON_PRESSED;
	else
		supp=BUTTON_RELEASED;

	put_user( supp, (unsigned int*)buf );
	retval=4;

	up(&read_sem);

	return retval;
}

static ssize_t fpanel_write( struct file *filp, const char __user *buf, size_t count, loff_t *f_ops )
{
	printk("fpanel: FATAL: Not implemented... \n");
	return (-1);
}

static int fpanel_ioctl( struct inode* inode, struct file* file, unsigned int cmd, unsigned long arg )
{
	int res=-ENOSYS;
	int supp=0;
	unsigned int fpanel_id;

	if ((fpanel_id = MINOR(inode->i_rdev)) >= FPANEL_NUMBER_DEVICES)
		return res;

	switch(cmd)
	{
		case IOCTL_READ_BUTTON_STATUS: //1
			if(status.button==BUTTON_PRESSED)
				supp=BUTTON_PRESSED;
			else
				supp=BUTTON_RELEASED;
			copy_to_user((void *) arg, 
					(const void *) &supp,
					sizeof(int));
			res=1;
			break;
		default:
			printk("fpanel: FATAL: Ioctl not implemented!\n");
			break;	
	}
	return res;
}

static unsigned int fpanel_poll( struct file *filp, poll_table *wait )
{
	unsigned int mask=0;

	if(change_status==1)
	{
		mask |= POLLOUT | POLLWRNORM | POLLIN | POLLRDNORM;
		change_status=0;
	}
	return mask;
}

static struct file_operations fpanel_fops =
{
  .owner   = THIS_MODULE,
  .open    = fpanel_open,
  .release = fpanel_release,
  .read	   = fpanel_read,
  .write   = fpanel_write,
  .ioctl   = fpanel_ioctl,
  .poll    = fpanel_poll,
};

int __init fpanel_init(void)
{
	int retval=0, err;
	unsigned char tries = 0;
	int res;

	status.pio_addr=0;

	status.pio_addr = (unsigned long)ioremap(PIO5_BASE_ADDRESS, (unsigned long)PIO5_IO_SIZE);

	if(!status.pio_addr)
	{
		printk("fpanel: FATAL: Memory allocation error\n");
		return -1;
	}

	dprintk("\nFront Panel Driver interface.\n");
	dev_nr = MKDEV(MAJOR_NR, 0);

	while(!register_chrdev_region(dev_nr, FPANEL_NUMBER_DEVICES, "fpanel_0") && tries < 3)
		tries++;

	if(tries>3)
	{
		printk("fpanel: FATAL: Device allocation failed\n");
		return -1;
	}

	fpanel_dev = cdev_alloc();
	fpanel_dev->ops = &fpanel_fops;
	fpanel_dev->owner = THIS_MODULE;

	err = cdev_add(fpanel_dev, dev_nr, FPANEL_NUMBER_DEVICES);
	if (err)
	{
		printk ("fpanel: FATAL: Cannot add device. Error code: %d\n", err);
		return 1;
	}

	status.button=BUTTON_RELEASED;
	change_status=0;

	/* Set the pin 5.7 in input (high impedance) */
//	ctrl_outl(BUTTON_PIN, fp.pio_addr+PIO_SET_PnC0);
//	ctrl_outl(BUTTON_PIN, fp.pio_addr+PIO_CLR_PnC1);
//	ctrl_outl(BUTTON_PIN, fp.pio_addr+PIO_SET_PnC2);

	/* Set the pin 5.7 in input (wake pull up) */
	ctrl_outl(BUTTON_PIN, status.pio_addr+PIO_CLR_PnC0);
	ctrl_outl(BUTTON_PIN, status.pio_addr+PIO_CLR_PnC1);
	ctrl_outl(BUTTON_PIN, status.pio_addr+PIO_CLR_PnC2);

	/* Set initial condition for interrupt */
	ctrl_outl(BUTTON_RELEASED, status.pio_addr+PIO_PnCOMP);
	ctrl_outl(BUTTON_PIN, status.pio_addr+PIO_PnMASK);

	res = request_irq(FPANEL_PORT5_IRQ, fpanel_irq_handler, IRQF_DISABLED, "fpanel_port5_irq" , NULL);
	if (res < 0)
    {
		printk("fpanel: FATAL: Could not request interrupt\n");	
		return -1;
	}

	dprintk ("Front Panel Module Initialized for QBoxHD mini[%s].\n", MYVERS);

	return retval;
}

void __exit fpanel_exit(void)
{
	cdev_del(fpanel_dev);
	unregister_chrdev_region(dev_nr, 1);
	iounmap((void*)status.pio_addr);

	free_irq(FPANEL_PORT5_IRQ, NULL);

	printk("fpanel: Front Panel Module unloaded\n");
}

module_init(fpanel_init);
module_exit(fpanel_exit);

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off fpanel debugging (default:off).");

MODULE_AUTHOR("Duolabs"); 
MODULE_DESCRIPTION("Front Panel Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.5");

