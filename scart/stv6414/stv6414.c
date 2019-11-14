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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>

#include <linux/version.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include "stv6414_i2c.h"

unsigned int debug;
#define dprintk(args...) \
	do { \
		if (debug) printk("stv6414: " args); \
	} while (0)

/* Prototypes */
int attach_adapter(struct i2c_adapter *);
int detach_client(struct i2c_client *);
int stv6414_command(struct i2c_client *, unsigned int, void *);
/**************************/

unsigned short ignore[] = { I2C_CLIENT_END };
unsigned short normal_addr[] = { 0x4B, I2C_CLIENT_END };
unsigned short probe_addr[] = {0, 0x4B, I2C_CLIENT_END, I2C_CLIENT_END};

static struct cdev stv6414_cdev;
static unsigned char inuse;

struct i2c_driver stv6414_driver;

struct stv6414_state {
	u8					addr;
	int					errmode;
	struct i2c_adapter*	i2c;
};

static struct i2c_client *new_client = NULL;
static struct stv6414_state * state;

/* In this chip the regs are only write. So I need a way to remember the last value of the registers */
static unsigned char last_value_of_regs[7];

void clear_global_vars(void)
{
	unsigned char i=0;
	
	inuse=0;
	for(i=0;i<7;i++)
		last_value_of_regs[i]=0;
}

int reg_write (struct stv6414_state *state, u8 reg, u8 data)
{
	int ret;
	u8 buf [] = { reg, data };
	struct i2c_msg msg = { .addr = state->addr, .flags = 0, .buf = buf, .len = 2 };

	dprintk("w: addr = 0x%02x : reg = 0x%02x val = 0x%02x\n", msg.addr, reg, data);
	ret = i2c_transfer (state->i2c, &msg, 1);

	if (ret != 1)
		dprintk("%s: writereg error (reg == 0x%02x, val == 0x%02x, ret == %i)\n", __FUNCTION__, reg, data, ret);

	return (ret != 1) ? -EREMOTEIO : 0;
}

u8 reg_read (struct stv6414_state *state, u8 reg)
{
	int ret;
	u8 b0 [] = { reg };
	u8 b1 [] = { 0 };
	struct i2c_msg msg [] = { { .addr = state->addr, .flags = 0, .buf = b0, .len = 1 },
								{ .addr = state->addr, .flags = I2C_M_RD, .buf = b1, .len = 1 } };

	ret = i2c_transfer (state->i2c, msg, 2);
	dprintk("r: addr = 0x%02x : reg = 0x%02x, val = 0x%02x\n", msg[0].addr, reg, b1[0]);

	if (ret != 2)
		dprintk("%s: readreg error (reg == 0x%02x, ret == %i)\n",__FUNCTION__, reg, ret);

	return b1[0];
}

int reg_read_cont (struct stv6414_state* state, u8 reg1, u8 *b, u8 len)
{
	int ret;
	struct i2c_msg msg [] = { { .addr = state->addr, .flags = 0, .buf = &reg1, .len = 1 },
							{ .addr = state->addr, .flags = I2C_M_RD, .buf = b, .len = len } };

	ret = i2c_transfer (state->i2c, msg, 2);

	if (ret != 2)
		dprintk("%s: readreg error (ret == %i)\n", __FUNCTION__, ret);

	return ret == 2 ? 0 : ret;
}

/* This function is used by user to read a register */
unsigned char read_i2c(unsigned char reg_i2c)
{
	return reg_read (state,reg_i2c);
}

/* This function is used by user to write register */
int write_i2c(unsigned char reg_i2c,unsigned char dat)
{
	return reg_write (state,reg_i2c,dat);
}

/* With ".probe" it is possible to choose which i2c bus you want check */
struct i2c_client_address_data addr_data = {
	.normal_i2c	=	ignore,//normal_addr,
	.probe		=	probe_addr,//ignore,
	.ignore		=	ignore
};

int client_attach(struct i2c_adapter *adap, int addr, int type)
{
	char name[I2C_NAME_SIZE] = "stv6414";
	dprintk("Inside %s()\n", __FUNCTION__);

	/* Create and attach the i2c client */
	if ((new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL)) == NULL)
		return -ENOMEM;
	memset(new_client, 0, sizeof(struct i2c_client));

	memcpy(new_client->name, name, sizeof(char) * I2C_NAME_SIZE);
	new_client->addr = addr;
	new_client->adapter = adap;
	new_client->driver = &stv6414_driver;

	i2c_attach_client(new_client);

	/* Init code here */
	if ((state = kmalloc(sizeof(struct stv6414_state), GFP_KERNEL)) == NULL)
		return -ENOMEM;

	state->addr = new_client->addr;
	state->i2c = new_client->adapter;

	dprintk("%s(): Client attached\n", __FUNCTION__);
	return 0;
}

int attach_adapter(struct i2c_adapter *adap)
{
	dprintk("Inside %s()\n", __FUNCTION__);
	return i2c_probe(adap, &addr_data, client_attach);
}

int detach_client(struct i2c_client *client)
{
	dprintk("%s(): Detaching client\n", __FUNCTION__);
	i2c_detach_client(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}

/* For generic ops */
int stv6414_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	dprintk("Inside %s()\n", __FUNCTION__);
	switch (cmd)
	{
		default:
			return -1;
	}
	return 0;
}
/*******************************************************************/
static int
stv6414_open (struct inode *inode, struct file *file)
{
	unsigned int stv6414_id;
	int rc = 0;

	dprintk("Opening device!\n");
	stv6414_id=MINOR(inode->i_rdev);
	if(stv6414_id<STV6414_NUMBER_OF_CONTROLLERS)
	{
		if(inuse == 0)
		{	
			inuse++;
			dprintk("stv6414_%d char device available\n", stv6414_id);
		}
		else
		{
			dprintk("stv6414_%d char device is busy\n", stv6414_id);
			rc = -EBUSY;
		}
	}
	else
	{
		dprintk("Couldn't open device with [%d] as MINOR number\n", stv6414_id);
		rc = -ENODEV;
	}
	return (rc);
}

static int
stv6414_release (struct inode *inode, struct file *file)
{
	unsigned int stv6414_id;
	int rc = 0;
	dprintk("Closing device!\n");
	stv6414_id=MINOR(inode->i_rdev);
	if(stv6414_id<STV6414_NUMBER_OF_CONTROLLERS)
	{
		if (inuse > 0)
		{
			inuse--;
		}
		else
		{
			dprintk("Device stv6414_%d is not opend\n",	stv6414_id);
			rc = -EINVAL;
		}
	}
	else
	{
		dprintk("Couldn't close device with [%d] as MINOR number\n", stv6414_id);
		rc = -ENODEV;
	}
	return rc;
}

static ssize_t
stv6414_read (struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return (-1);
}

static ssize_t
stv6414_write (struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return (-1);
}
/********************************************/
/*	Auxiliary functions for stv6414_ioctl	*/
/********************************************/
#if 0
	------------------------------
	With user-space command i2cset
	------------------------------
	i2cset -y 0 0x4b 0x06 0xff
	i2cset -y 0 0x4b 0x00 0x0a	#0x00	#Attenuation of the scart volume so now it is similar to HDMI audio (-10dB)
	i2cset -y 0 0x4b 0x01 0x09	#0x01	#ENC L/R select and attenuation fo CHINC
	i2cset -y 0 0x4b 0x02 0x09	#Chroma muted and Y/CVBS_ENC & R/C_ENC
	i2cset -y 0 0x4b 0x03 0x00	#for RGB (muted now)
	i2cset -y 0 0x4b 0x05 0x20	#0x00	#tv output 16/9
	i2cset -y 0 0x4b 0x06 0x9E	#0xBE	#0x00	#tv output on, all input off
	i2cset -y 0 0x4b 0x04 0x80
	i2cset -y 0 0x4b 0x03 0x85	#for RGB enable
	#enable VCR too
	i2cset -y 0 0x4b 0x01 0x19
	i2cset -y 0 0x4b 0x02 0x99
	i2cset -y 0 0x4b 0x06 0x8E
#endif

static void stv6414_no_bypass(void)
{
	write_i2c(REG06_STANDBY,0xFF);
	//write_i2c(REG00_AUDIO_OUTPUT,0x0A);	/* -10dB */
	write_i2c(REG00_AUDIO_OUTPUT,0x06);		/* -6dB */
	write_i2c(REG01_AUDIO_SELECTION,0x09);
	write_i2c(REG02_VIDEO_SELECTION,0x09);
	write_i2c(REG03_VIDEO_RGB,0x00);
	write_i2c(REG05_OUTPUT_FORMAT,0x20);
	write_i2c(REG06_STANDBY,0x9E);
	write_i2c(REG04_MISC_CTRL,0x80);
	write_i2c(REG03_VIDEO_RGB,0x85);

	/* Enable VCR too */
	write_i2c(REG01_AUDIO_SELECTION,0x19);
	write_i2c(REG02_VIDEO_SELECTION,0x99);
	write_i2c(REG06_STANDBY,0x8E);

	/* Update the array of last values */
	//last_value_of_regs[0]=0x0A;
	last_value_of_regs[0]=0x06;
	last_value_of_regs[1]=0x19;
	last_value_of_regs[2]=0x99;
	last_value_of_regs[3]=0x85;
	last_value_of_regs[4]=0x80;
	last_value_of_regs[5]=0x20;
	last_value_of_regs[6]=0x8E;
}

#if 0
	------------------------------
	With user-space command i2cset
	------------------------------
	i2cset -y 0 0x4b 0x06 0xff
	#for decoder in stan-by (VCR<->TV)
	i2cset -y 0 0x4b 0x01 0x2A
	i2cset -y 0 0x4b 0x02 0xBB
	i2cset -y 0 0x4b 0x03 0x8B
	i2cset -y 0 0x4b 0x04 0xC0
	i2cset -y 0 0x4b 0x06 0x9D
	i2cset -y 0 0x4b 0x00 0x01	#no control volume
#endif

static void stv6414_bypass(void)
{
	write_i2c(REG06_STANDBY,0xFF);
	
	/* For decoder in standby (VCR<->TV) */
	write_i2c(REG01_AUDIO_SELECTION,0x2A);
	write_i2c(REG02_VIDEO_SELECTION,0xBB);
	write_i2c(REG03_VIDEO_RGB,0x8B);
	write_i2c(REG04_MISC_CTRL,0xC0);
	write_i2c(REG06_STANDBY,0x9D);
	write_i2c(REG00_AUDIO_OUTPUT,0x01);
	
	/* Update the array of last values */
	last_value_of_regs[0]=0x01;
	last_value_of_regs[1]=0x2A;
	last_value_of_regs[2]=0xBB;
	last_value_of_regs[3]=0x8B;
	last_value_of_regs[4]=0xC0;
	/*last_value_of_regs[5] -> nothing*/
	last_value_of_regs[6]=0x9D;
}

static int
stv6414_ioctl (struct inode *inode, struct file *file,
              unsigned int cmd, unsigned long arg)
{
	unsigned int stv6414_id;
	Register_t reg;
	int data;
	int res=-ENOSYS;
	int par;
	
	stv6414_id=MINOR(inode->i_rdev);
		
	if(stv6414_id > STV6414_NUMBER_OF_CONTROLLERS)
		return res;

	switch (cmd)
	{
		case IOCTL_READ_I2C://1
			dprintk("ioctl IOCTL_READ_I2C\n");
			copy_from_user((void *) &reg,
							(const void *) arg,
						    sizeof(Register_t));
			
			reg.value=read_i2c(reg.offset_addr);
			
			copy_to_user((void *) arg, 
					    (const void *) &reg,
					    sizeof(Register_t));
			res=1;
		break;
		case IOCTL_WRITE_I2C://2		
			dprintk("ioctl IOCTL_WRITE_I2C\n");
			copy_from_user((void *) &reg,
					    (const void *) arg,
					    sizeof(Register_t));
			write_i2c(reg.offset_addr,reg.value);
			last_value_of_regs[reg.offset_addr]=reg.value;
			res=2;
		break;
		case IOCTL_READ_LAST_WRITE_VALUE_OF_REG://3		
			dprintk("ioctl IOCTL_READ_LAST_WRITE_VALUE_OF_REG\n");
			copy_from_user((void *) &reg,
							(const void *) arg,
						    sizeof(Register_t));
			
			reg.value=last_value_of_regs[reg.offset_addr];
			
			copy_to_user((void *) arg, 
					    (const void *) &reg,
					    sizeof(Register_t));
			res=3;
		break;
		case IOCTL_SET_169_43://4
			dprintk("ioctl IOCTL_SET_169_43\n");
			copy_from_user((void *) &data, 
                           (const void *) arg,
                           sizeof(data));
			reg.value=last_value_of_regs[REG05_OUTPUT_FORMAT];
			reg.value&=(~MASK_VCR_TV_FORMAT);
			reg.value|=data;
			reg.offset_addr=REG05_OUTPUT_FORMAT;
			write_i2c(reg.offset_addr,reg.value);
			last_value_of_regs[REG05_OUTPUT_FORMAT]=reg.value;
			
			if(data==TV_16_9)
				dprintk("Set the format tv in TV_16_9\n");
			else
				dprintk("Set the format tv in TV_4_3\n");
			res=4;
		break;			
		case IOCTL_STANDBY_ON_OFF://5
			dprintk("ioctl IOCTL_STANDBY_ON_OFF\n");
			copy_from_user((void *) &data, 
                           (const void *) arg,
                           sizeof(data));
            if(data==STANDBY_ON)
            {
            	dprintk("Set the STANDBY_ON\n");
            	stv6414_bypass();
            }
            else
            {
            	dprintk("Set the STANDBY_OFF\n");
            	stv6414_no_bypass();            
			}
			res=5;
		break;
		case IOCTL_DISABLE_TV_OUTPUT://6 1->disable, 0->enable
			dprintk("ioctl IOCTL_STANDBY_ON_OFF\n");
			copy_from_user((void *) &par, 
                           (const void *) arg,
                           sizeof(par));
            if(par==1)
            {
				dprintk("Disable the tv output");
				write_i2c(REG03_VIDEO_RGB,0x8B);
				write_i2c(REG04_MISC_CTRL,0xC0);
				last_value_of_regs[REG03_VIDEO_RGB]=0x8B;
				last_value_of_regs[REG04_MISC_CTRL]=0xC0;
            }
            else
            {
				dprintk("Enable the tv output");
				write_i2c(REG03_VIDEO_RGB,0x85);
				write_i2c(REG04_MISC_CTRL,0x80);
				last_value_of_regs[REG03_VIDEO_RGB]=0x85;
				last_value_of_regs[REG04_MISC_CTRL]=0x80;
				
			}
			res=6;
		break;
	}
	return res;
}

static struct file_operations stv6414_fops = {
	.owner		=	THIS_MODULE,
	.open		=	stv6414_open,
	.release	=	stv6414_release,
	.read		=	stv6414_read,
	.write		=	stv6414_write,
	.ioctl		=	stv6414_ioctl
};

struct i2c_driver stv6414_driver =
{
	.id             = I2C_DRIVERID_STV6414,
	.driver         = { .name   = "stv6414 driver",
						.owner  = THIS_MODULE },
	.attach_adapter = attach_adapter,
	.detach_client  = detach_client,
	.command        = stv6414_command,
};

/******************************************************************************/

static int __init stv6414_init(void)
{
 	int ret=0;

 	clear_global_vars();
	 	
    dev_t dev = MKDEV(STV6414_MAJOR_NUM, STV6414_MINOR_START);

    cdev_init(&stv6414_cdev, &stv6414_fops);
    stv6414_cdev.owner = THIS_MODULE;
    stv6414_cdev.ops = &stv6414_fops;
    if (cdev_add(&stv6414_cdev, dev, STV6414_NUMBER_OF_CONTROLLERS) < 0)
    {
        printk("Couldn't register '%s' driver\n", STV6414_DEVICE_NAME);
        return -1;
    }
    printk("Registering device '%s', major '%d'\n", STV6414_DEVICE_NAME, STV6414_MAJOR_NUM);
	
    ret=i2c_add_driver(&stv6414_driver);

	/* Set the scart */
 	stv6414_no_bypass();

	printk("Version of stv6414_drv: 0.0.4\n");

	/* i2c command */
	return ret;
}

static void __exit stv6414_exit(void)
{
    cdev_del(&stv6414_cdev);

	/* i2c command */
	i2c_del_driver(&stv6414_driver);

    printk("Unregistering device '%s'\n", STV6414_DEVICE_NAME);
}

module_init(stv6414_init);
module_exit(stv6414_exit);

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off stv6414 debugging (default:off).");

MODULE_DESCRIPTION("I2C driver for the stv6414");
MODULE_AUTHOR("Pedro Aguilar, Michele Cecchin");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.4");



















