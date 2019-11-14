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
#include "struct.h"

volatile unsigned short *fpga_adr;
unsigned char minor;
unsigned short old_tmp = 0;

dev_t dev_nr;

struct cdev *fpanel_dev;

DECLARE_MUTEX(set_led);
DECLARE_MUTEX(read_sem);

int fpanel_open( struct inode* inode, struct file* file );
int fpanel_release( struct inode* inode, struct file* file );
int fpanel_ioctl( struct inode* inode, struct file* file, unsigned int cmd, unsigned long arg );
unsigned int fpanel_poll( struct file *filp, poll_table *wait );

static struct file_operations fpanel_fops =
{
  .owner   = THIS_MODULE,
  .ioctl   = fpanel_ioctl,
  .open    = fpanel_open,
  .release = fpanel_release,
  .read	   = fpanel_read,
  .write   = fpanel_write,
  .poll    = fpanel_poll,
};

unsigned short val,old_val;

ssize_t fpanel_read( struct file *filp, char __user *buf, size_t count, loff_t *f_ops )
{
	ssize_t retval = 0;
	unsigned int tmp = 0;

#if 1
	debug("-- Read capsense !! \n");

	if (down_interruptible(&read_sem) != 0)	
		return -ERESTARTSYS;
	
	tmp = fpga_adr[REG1];
        
	debug("tmp = %04X \n ", tmp );

	if( old_tmp && !tmp )  // released button
	{
		put_user(  0x00FF, (unsigned int*)buf );
		retval = 4;
	}
	else if( !old_tmp && tmp )  // pressed button
	{
		put_user(  tmp, (unsigned int*)buf );
		debug(">> PRESSED !!  !! <<\n");
		retval = 4;
	}
	else if( old_tmp==0x25 && tmp==0x0F )  // pressed button
	{
		put_user(  tmp, (unsigned int*)buf );
		debug(">> ENTER STAND BY ! !!  !! <<\n");
		retval = 4;
	}
	else 
	{
		retval= -EFAULT; 		// no button
	}	
	old_tmp=tmp;	

	up(&read_sem); // the semaphore is released !
#endif
	return retval;
}


unsigned int fpanel_poll( struct file *filp, poll_table *wait )
{
    val=fpga_adr[REG1];
    debug("FUNC POLL SENSEWHEEL val == %04X \n", val);
    if(old_val && !val)
    {
        old_val = val;
        return POLLIN;
    }
    if( val && (val!=0xFFFF) )
    {
        old_val = val;
        debug("AVAILABLE DATA... %02X \n", val);
        return POLLIN ;
    }
    else
    {
        old_val = val;
        return 0;
    }
};

ssize_t fpanel_write( struct file *filp, const char __user *buf, size_t count, loff_t *f_ops )
{
	ssize_t retval = 0;
	printk("Not implemented... \n");
	return retval;
}

int __init fpanel_init(void)
{
	int retval=0, err;
	unsigned char tries = 0;

	memset( &send_cmd, 0, sizeof(send_cmd) );

	fpga_adr = (volatile unsigned short *)ioremap((unsigned long)FPGA_BASE, (unsigned long)0x100);

	if(!fpga_adr)
	{
		printk("Memory allocation Error.\n");
		return -1;
	}

	debug("\nFront Panel Driver interface.\n");
	dev_nr = MKDEV(MAJOR_NR, 0);

	while(!register_chrdev_region(dev_nr, NUMBER_OF_DEVICE, "front_panel") && tries < 3)
		tries++;

	if(tries>3)
	{
		printk("Device allocation number fail ! \n");
		return -1;
	}

	fpanel_dev = cdev_alloc();
	fpanel_dev->ops = &fpanel_fops;
	fpanel_dev->owner = THIS_MODULE;

	err = cdev_add(fpanel_dev, dev_nr, NUMBER_OF_DEVICE);
	if (err)
	{
		printk ("Cannot add device. Error code: %d\n", err);
		return 1;
	}
	printk ("Front Panel Module Initialized [%s].\n", MYVERS );
	printk("\n");
	return retval;
}

void __exit fpanel_exit(void)
{
	cdev_del(fpanel_dev);
	unregister_chrdev_region(dev_nr, 1);
	iounmap((void*)fpga_adr);
	printk("Front Panel Module Unload. \n");
}

int fpanel_open( struct inode* inode, struct file* file )
{
	if(iminor(inode))
		debug("routing device open !!!!\n");
	else
		debug("fpanel device open !!!!\n");
	return 0;
}

int fpanel_release( struct inode* inode, struct file* file )
{
	if(iminor(inode))
		debug("routing device release !!!!\n");
	else
		debug("fpanel device release !!!!\n");

	return 0;
}

int fpanel_ioctl( struct inode* inode, struct file* file, unsigned int cmd, unsigned long arg )
{
	int result = 0;
    int cnt = 0;
	/* The semaphore is acquired to registers write */
	if (down_interruptible(&set_led) != 0)	
		return -ERESTARTSYS;

	switch(cmd)
	{
		case TS_ROUTING:
			if(iminor(inode))
			{
				unsigned char ts;
				debug("Ts route: change routing ! \n");
				get_user(ts,(unsigned char*)arg);
				debug("TS value [0x%02X]\n",ts);
				fpga_adr[TS_REG] = ts;
				up(&set_led); 			
				return result;
			}
			else
			{
				printk("Incorrect IOCTL\n");
				up(&set_led); 			
				return -1;
			}
		case SET_LED_COLOR:
			debug(">> SET_LED_COLOR << \n");
			printk("Not yet implemented ... \n");
			break;		
		case SET_LED_OFF:
			debug(">> SET_LED_OFF << \n");
			printk("Not yet implemented ... \n");
			break;		
		case SET_LED_ALL_PANEL_COLOR:
			debug( ">> SET_LED_ALL_PANEL_COLOR << \n" );
			if( copy_from_user( (void*)&comando, (void*)arg, sizeof(comando) ) )
			{
				up(&set_led);
				return result= -EFAULT;			
			}
			debug("H= %d , S= %d, V= %d \n", comando.H, comando.S, comando.V );
			send_cmd.cmd_led_V= ( 0x2<<12 ) | comando.V;
			send_cmd.H_S= ( comando.H<<7 ) | comando.S ;
		
			debug("reg1 = %04x \n", send_cmd.cmd_led_V );
			debug("reg2 = %04x \n", send_cmd.H_S );		
			break;	
		case SET_LED_ALL_PANEL_OFF:
			debug(">> SET_LED_ALL_PANEL_OFF << \n");		
			printk("Not yet implemented ... \n");
			break;
		case SET_ALL_BOARD_COLOR:
			debug(">> SET_ALL_BOARD_COLOR << \n");
			if( copy_from_user( (void*)&comando, (void*)arg, sizeof(comando) ) )
			{
				up(&set_led);
				return result= -EFAULT;			
			}		
			debug("H= %d , S= %d, V= %d \n", comando.H, comando.S, comando.V );		

			send_cmd.cmd_led_V= ( 0x3<<12 ) | comando.V;
			send_cmd.H_S= ( comando.H<<7 ) | comando.S ;		
			debug2("reg1 = %04x \n", send_cmd.cmd_led_V );
			debug2("reg2 = %04x \n", send_cmd.H_S );		
			break;
		case SET_ALL_BOARD_OFF:
			debug(">> SET_ALL_BOARD_OFF << \n");
			printk("Not yet implemented ... \n");	
			break;
		case SET_LED_ALL_COLOR:
			debug(">> SET_LED_ALL_COLOR << \n");	

			if( copy_from_user( (void*)&comando, (void*)arg, sizeof(comando) ) )
			{
				up(&set_led);
				return result= -EFAULT;			
			}
			debug2("H= %d , S= %d, V= %d \n", comando.H, comando.S, comando.V );
			send_cmd.cmd_led_V= ( 0x4<<12 ) | comando.V;
			send_cmd.H_S= ( comando.H<<7 ) | comando.S ;		
			debug2("reg1 = %04x \n", send_cmd.cmd_led_V );
			debug2("reg2 = %04x \n", send_cmd.H_S );		
			break;
        /* new add 12.01.2009 */
        case STANDBY:
            debug(">> STANDBY << \n");
            if( copy_from_user( (void*)&standy_param, (void*)arg, sizeof(standy_param) ) )
		    {
                up(&set_led);
                return result= -EFAULT;
		    }
            debug("standby_mode = %x \n", standy_param.standby_mode);
            debug("led_group_map = %x \n", standy_param.led_group_map);
            send_cmd.cmd_led_V = 0;
            send_cmd.cmd_led_V= ( 0xB<<12 );                      
            send_cmd.H_S = standy_param.standby_mode<<8 | standy_param.led_group_map ;
            debug("Ready to transfer the command, payload = %04X \n", send_cmd.H_S);
            break;
		case SET_LED_ALL_OFF:
			debug(">> SET_LED_ALL_OFF << \n");	
			printk("Not yet implemented ... \n");
			break;
		case SET_DISABLE_SENSE:
			debug("SET_DISABLE_SENSE \n");
			send_cmd.cmd_led_V= ( 0x5<<12 );
			break;
		case SET_ENABLE_SENSE:
			debug("SET_ENABLE_SENSE \n");
			send_cmd.cmd_led_V= ( 0x6<<12 );
			break;
		case WHEEL:
			printk("Not yet implemented ... \n");
			break;
		default:
			printk("Invalid Front Panel IOCLT !\n");
			break;
	}// switch command 
	
	fpga_adr[REG1] = send_cmd.cmd_led_V;
	fpga_adr[REG2] = send_cmd.H_S;
        
	debug("reg write \n");
        
    // wait with timeout !
    cnt = 0;
	while( !fpga_adr[REG2] )
    {
        ++cnt;
        if( cnt > 5000000 )
        {
            up(&set_led); // the semaphore is released
            debug("timeout... \n");
            return -1;  // operation error 
        }
        //udelay(100);
        continue;
    }

	up(&set_led); // the semaphore is released !
	debug("DRIVER IOCTL returns result = %d  \n",result);
	return result;
}

module_init(fpanel_init);
module_exit(fpanel_exit);

MODULE_AUTHOR("Duolabs"); 
MODULE_DESCRIPTION("Front Panel Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(MYVERS);
