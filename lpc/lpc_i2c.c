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

/*
 * Description:
 *
 * stb
 *  |
 *  ---------- lpc
 *  |           |
 *  |           --------- rtc 
 *  |						|
 *  |						|
 *  |						--------- was_timer_wakeup
 *  |						|
 *  |						|
 *  |						--------- support
 *  |						|
 *  |						|
 *  |						--------- wakeup_time
 *  |						|
 *  |						|
 *  |						--------- version
 *  |						|
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
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <linux/proc_fs.h>  	/* proc fs */ 
#include <asm/uaccess.h>    	/* copy_from_user */

#include "lpc_private.h"
#include "lpc_i2c.h"
#include "time_funcs.h"

unsigned int debug;
unsigned int debugl;

/* Debug function for i2c i/o */
#define dprintk(args...) \
	do { \
		if (debug) printk("lpc: " args); \
	} while (0)

/* Debug for interface with user */
#define dprintkl(args...) \
	do { \
		if (debugl) printk("\nlpc: " args); \
	} while (0)


/* Prototypes */
int attach_adapter(struct i2c_adapter *);
int detach_client(struct i2c_client *);
int lpc_command(struct i2c_client *, unsigned int, void *);
/**************************/

unsigned short ignore[] = { I2C_CLIENT_END };
unsigned short normal_addr[] = { 0x1A, I2C_CLIENT_END };

static struct cdev lpc_cdev;
static unsigned char inuse;

struct i2c_driver lpc_driver;

struct lpc_state {
		u8					addr;
		int					errmode;
		struct i2c_adapter*	i2c;
};

static struct i2c_client *new_client = NULL;
static struct lpc_state * state;

static Data_Support_t status;


typedef int (*proc_read_t) (char *page, char **start, off_t off, int count,
		  int *eof, void *data_unused);
typedef int (*proc_write_t) (struct file *file, const char __user *buf,
		   unsigned long count, void *data);

#define cProcDir					1
#define cProcEntry				2 

#define cProcNotExist 		1
#define cProcAlreadyExist 2

struct LpcProcStructure_s
{
		int type;
		int proc_dir_entry_exists;
		char* name;
		struct proc_dir_entry* entry;
		proc_read_t read_proc;
		proc_write_t write_proc;
};

/*****************************************************************/
/* Clear the variables of the global status */
void clear_support_struct(void)
{
		inuse=0;
		status.temp_cur=0xFF;
		status.temp_max=0xFF;
		status.temp_min=0xFF;
		status.fan_power=0xFF;
		status.pwm.status_pin=0xFF;
		status.pwm.pwm_value=0xFF;
		status.version_lpc=0;
}

int reg_write (struct lpc_state *state, u8 reg, u8 data)
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

u8 reg_read (struct lpc_state *state, u8 reg)
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

/***********************************************/
/*     I2C write function for protocol lpc     */
/***********************************************/
int reg_write_prot (struct lpc_state *state, u8 reg, u32 data)
{
		int ret;
		u8 buf[5];

		buf[0]=reg;
		buf[1] = (unsigned char)((data & 0xFF000000)>>24);
		buf[2] = (unsigned char)((data & 0x00FF0000)>>16);
		buf[3] = (unsigned char)((data & 0x0000FF00)>>8);
		buf[4] = (unsigned char)((data & 0x000000FF)>>0);

		{
			struct i2c_msg msg = { .addr = state->addr, .flags = 0, .buf = buf, .len = 5 };

			ret = i2c_transfer (state->i2c, &msg, 1);
		}

		if (ret != 1)
			dprintk("%s: writereg error (reg == 0x%02x, val == 0x%02x, ret == %i)\n", __FUNCTION__, reg, data, ret);
		else
			dprintk("%s: write (reg == 0x%02x, val == 0x%02x, ret == %i)\n", __FUNCTION__, reg, data, ret);

		return (ret != 1) ? -EREMOTEIO : 0;
}
/***********************************************/

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

struct i2c_client_address_data addr_data = {
		.normal_i2c	=	normal_addr,
		.probe		=	ignore,
		.ignore		=	ignore
};

int client_attach(struct i2c_adapter *adap, int addr, int type)
{
		char name[I2C_NAME_SIZE] = "lpc_drv";
		dprintk("Inside %s()\n", __FUNCTION__);
	
		/* Create and attach the i2c client */
		if ((new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL)) == NULL)
			return -ENOMEM;
		memset(new_client, 0, sizeof(struct i2c_client));

		memcpy(new_client->name, name, sizeof(char) * I2C_NAME_SIZE);
		new_client->addr = addr;
		new_client->adapter = adap;
		new_client->driver = &lpc_driver;

		i2c_attach_client(new_client);

		/* Init code here */
		if ((state = kmalloc(sizeof(struct lpc_state), GFP_KERNEL)) == NULL)
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
int lpc_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
		dprintk("Inside %s()\n", __FUNCTION__);
		switch (cmd)
		{
			default:
				return -1;
		}
		return 0;
}

/***************************************/
/*   Functions of the device 'lpc_0'   */
/***************************************/
static int
lpc_open (struct inode *inode, struct file *file)
{
		unsigned int lpc_id;
		int rc = 0;

		dprintk("Opening device!\n");
		lpc_id=MINOR(inode->i_rdev);
		if(lpc_id<LPC_NUMBER_OF_CONTROLLERS)
		{
			//if(inuse == 0)
			if(inuse < 2) /* permitted double open */
			{	
				inuse++;
				dprintk("lpc_%d char device available\n", lpc_id);
			}
			else
			{
				dprintk("lpc_%d char device is busy\n", lpc_id);
				rc = -EBUSY;
			}
		}
		else
		{
			dprintk("Couldn't open device with [%d] as MINOR number\n", lpc_id);
			rc = -ENODEV;
		}
		return (rc);
}

static int
lpc_release (struct inode *inode, struct file *file)
{
		unsigned int lpc_id;
		int rc = 0;
		dprintk("Closing device!\n");
		lpc_id=MINOR(inode->i_rdev);
		if(lpc_id<LPC_NUMBER_OF_CONTROLLERS)
		{
			if (inuse > 0)
			{
				inuse--;
			}
			else
			{
				dprintk("Device lpc_%d is not opend\n",	lpc_id);
				rc = -EINVAL;
			}
		}
		else
		{
			dprintk("Couldn't close device with [%d] as MINOR number\n", lpc_id);
			rc = -ENODEV;
		}
		return rc;
}

static ssize_t
lpc_read (struct file *file, char *buf, size_t count, loff_t *ppos)
{
		return (-1);
}

static ssize_t
lpc_write (struct file *file, const char *buf, size_t count, loff_t *ppos)
{
		return (-1);
}

static int
lpc_ioctl (struct inode *inode, struct file *file,
              unsigned int cmd, unsigned long arg)
{
		unsigned int lpc_id;
		int res=-ENOSYS;
		Rrgb_Value_t rgb;
		int par;
		short pwm_t;
		unsigned int supp=0;

		lpc_id=MINOR(inode->i_rdev);
		
		if(lpc_id > LPC_NUMBER_OF_CONTROLLERS)
			return res;

		switch (cmd)
		{
			case IOCTL_READ_VERSION://1
				dprintkl("IOCTL_READ_VERSION\n");
				par=reg_read(state,READ_VERSION);
				dprintkl("version: %d\n",par);
				copy_to_user((void *) arg, 
							  (const void *) &par,
							  sizeof(par));
				res=1;
				break;

			case IOCTL_READ_TEMP://2
				dprintkl("IOCTL_READ_TEMP\n");
				par=reg_read(state,READ_TEMP);
				status.temp_cur=par;
				dprintkl("Value of current temperature: %d°C\n",par);
				copy_to_user((void *) arg, 
							  (const void *) &par,
							  sizeof(par));
				res=2;
				break;

			case IOCTL_FAN_POWER://3
				dprintkl("IOCTL_FAN_POWER\n");
				copy_from_user((void *) &par, 
		                         (const void *) arg,
		                         sizeof(par));
				if(status.version_lpc < 2)
				{
					supp|=(par&0xFF);
					supp=(supp<<24);
					reg_write_prot(state,FAN_POWER,supp);
					status.fan_power=par;
				}
				else if(status.version_lpc >= 2)
				{
					supp|=(par&0xFF);
					supp=(supp*8);
					if(supp>255) supp=255;			
					supp=(supp<<24);
					reg_write_prot(state,FAN_POWER,supp);
					status.fan_power=par;
				}
				else
				{
					printk("Unknown version of lpc firmware: %d\n",status.version_lpc);			
					return -ENOSYS;
				}
				res=3;
				break;

			case IOCTL_PWM_VALUE://4
				dprintkl("IOCTL_PWM_VALUE\n");
				copy_from_user((void *) &pwm_t, 
		                         (const void *) arg,
		                         sizeof(pwm_t));

				if(status.version_lpc < 2)
				{			
		
					supp|=(pwm_t&0xFFFF);	// | XX | XX | in/out | pwm_value |
					supp=(supp<<16);			// | in/out | pwm_value | XX | XX |

					reg_write_prot(state,PWM_VALUE,supp);
				
					status.pwm.status_pin=((pwm_t&0xFF00)>>8);
					if(status.pwm.status_pin==OUTPUT_PWM)
						status.pwm.pwm_value=(pwm_t&0xFF);
				}
				else if (status.version_lpc >= 2)
				{
					supp|=(pwm_t&0xFF);//only the value
					supp=(supp/8);
					supp=(supp<<24);
					reg_write_prot(state,PWM_VALUE,supp);	
					status.pwm.pwm_value=(pwm_t&0xFF);
					status.pwm.status_pin=OBSOLETE;
				}
				else
				{
					printk("Unknown version of lpc firmware: %d\n",status.version_lpc);			
					return -ENOSYS;
				}

				res=4;
				break;

			case IOCTL_SET_MAX_MIN_TEMP://5
				dprintkl("IOCTL_SET_MAX_MIN_TEMP\n");
				copy_from_user((void *) &par, 
		                         (const void *) arg,
		                         sizeof(par));

				/* Considerate only the two LSB byte */
				par&=0xFFFF;
				status.temp_min=(par&0xFF);		//min
				status.temp_max=((par>>8)&0xFF);//max

				dprintkl("Minimum Temp:%d\n",status.temp_min);
				dprintkl("Maximum Temp:%d\n",status.temp_max);

				/* Re-formed the parameter for register */
				par=((par<<16)&0xFFFF0000);		

				reg_write_prot(state,CONFIG_FAN_CTL_PAR,par);
				res=5;
				break;

			case IOCTL_SET_RGB://6
				dprintkl("IOCTL_SET_RGB\n");
				copy_from_user((void *) &rgb, 
		                         (const void *) arg,
		                         sizeof(rgb));
				supp|=rgb.red;
				supp=(supp<<8);	
				supp|=rgb.green;
				supp=(supp<<8);	
				supp|=rgb.blue;
				supp=(supp<<8);	
				reg_write_prot(state, SET_RGB, supp); 
				res=6;
				break;

			case IOCTL_WAKEUP_IR_CODE://7
				dprintkl("IOCTL_WAKEUP_IR_CODE\n");
				copy_from_user((void *) &par, 
		                         (const void *) arg,
		                         sizeof(par));
				status.ir_code=0;
				status.ir_code|=(par&0xFFFF);
				status.ir_code=(status.ir_code<<16);
				reg_write_prot(state, CONFIG_RCU_PARAM, status.ir_code); //new wakeup ir code
				res=7;
				break;
			case IOCTL_READ_BYTE://100
				dprintkl("IOCTL_WAKEUP_IR_CODE\n");
				copy_from_user((void *) &par, 
		                         (const void *) arg,
		                         sizeof(par));
				par=reg_read(state,(par&0xFF));
				copy_to_user((void *) arg, 
							  (const void *) &par,
							  sizeof(par));
			default:
				dprintkl("\nUNKNOWN COMMAND\n");
				return -ENOSYS;
		}

		return res;
}

/************************************************/
/* To set the backlight from the others drivers */
/************************************************/
/* 0 < val < 32 */
unsigned char lpc_set_bgr(unsigned char val)
{
		unsigned int supp=0;
		if(status.version_lpc>=2)
		{
			dprintkl("Version is: %d\n",status.version_lpc);
			supp|=(val&0xFF);//only the value
			supp=(supp<<24);
			reg_write_prot(state,PWM_VALUE,supp);	
			status.pwm.pwm_value=((val*8)&0xFF);
			status.pwm.status_pin=OBSOLETE;
		}
		return status.version_lpc;
}
EXPORT_SYMBOL(lpc_set_bgr);

/**********************************************/
static struct file_operations lpc_fops = {
		.owner		=	THIS_MODULE,
		.open			=	lpc_open,
		.release	=	lpc_release,
		.read			=	lpc_read,
		.write		=	lpc_write,
		.ioctl		=	lpc_ioctl,
};

struct i2c_driver lpc_driver =
{
		.id             = I2C_DRIVERID_LPC,
		.driver         = { .name   = "lpc driver",
												.owner  = THIS_MODULE },
		.attach_adapter = attach_adapter,
		.detach_client  = detach_client,
		.command        = lpc_command,
};

/**********************************************/
/*              PROC MANAGEMENT               */
/* Manage /proc file with standard filesystem */
/**********************************************/

/********************/
/*       RTC        */
/********************/
static int lpc_rtc_read(char *page, char **start, off_t off, int count,
                           int *eof, void *data)
{
	int len = 0;
	int rst=0;
	int sec=0;
	dprintkl(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %s()\n", __func__);
	if(status.version_lpc>=3)
	{
		rst|=read_i2c(READ_RTC_TIME+len);
		rst=(rst<<8);
		rst|=read_i2c(READ_RTC_TIME+1);
		rst=(rst<<8);
		rst|=read_i2c(READ_RTC_TIME+2);
		rst=(rst<<8);
		rst|=read_i2c(READ_RTC_TIME+3);
		
		/* Read seconds */		
		sec=read_i2c(SECS_FOR_TIME);

		len = sprintf(page, "%u %u", rst, sec);
		*eof = 1;
		return len;	
	}
	else
		return 0;	
}

static int lpc_rtc_write(struct file *file, const char __user *buf,
                         unsigned long count, void *data)
{
	struct local_tm t;
	local_time_t tmp;
	int mjd1;
	unsigned int value=0;

	dprintkl(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %s()\n", __func__);

	sscanf(buf,"%u",(int *)&tmp);
	local_time_t_to_tm(&tmp, &t);

	dprintkl("------------>t.tm_year: %d\n",t.tm_year+1900);
	dprintkl("------------>t.tm_mon: %d\n",t.tm_mon);
	dprintkl("------------>t.tm_mday: %d\n",t.tm_mday);
	dprintkl("------------>t.tm_hour: %d\n",t.tm_hour);
	dprintkl("------------>t.tm_min: %d\n",t.tm_min);
	dprintkl("------------>t.tm_sec: %d\n",t.tm_sec);

	if(local_tm_to_mjd(&t, &mjd1) < 0)
		dprintkl("ERROR!\n");

	dprintkl("Date in mjd format: %d\n",mjd1);

	value=mjd1;	//date
	value=(value<<8);
	value|=convert_decimal_to_bcd(t.tm_hour);	//hour
	value=(value<<8);
	value|=convert_decimal_to_bcd(t.tm_min);	//minute

	dprintkl("Value to write in data register: 0x%08X\n", value);

	reg_write_prot(state, MODIFY_TIME_DATA, value);

	/* Write the seconds */
	value=0;
	value=convert_decimal_to_bcd(t.tm_sec);
	value=(value<<24);

	dprintkl("Value to write in seconds register: 0x%08X\n", value);

	reg_write_prot(state, SECS_FOR_TIME, value);

	return count;
}

/************************/
/*      WAS WAKEUP      */
/************************/
static int lpc_was_wakeup_read(char *page, char **start, off_t off, int count,
                         			  int *eof, void *data)
{
		dprintkl(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %s()\n", __func__);
		return 0;
}


static int lpc_was_wakeup_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
		dprintkl(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %s()\n", __func__);
		return count;
}

/*************************/
/*        SUPPORT        */
/*************************/
static int lpc_support_read(char *page, char **start, off_t off, int count,
                         			  int *eof, void *data)
{
		int len=0;

		if ( !off )
			dprintkl(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %s()\n", __func__);

		status.temp_cur = reg_read(state, READ_TEMP);
	
		if(status.version_lpc < 2)
		{
				len = sprintf(page, "LPC status:\n\tCurrent Temp: %d°C\n\tMaximum Temp: %d°C\n\tMinimum Temp: %d°C\n\tFan power: %d\n\tStatus pwm pin:%s\n\tPwm value: %d\n", status.temp_cur, status.temp_max, 
										status.temp_min, status.fan_power, 
										((status.pwm.status_pin==OUTPUT_PWM) ? "OUTPUT PIN" : "INPUT PIN"), 
										status.pwm.pwm_value);
		}
		else if(status.version_lpc >= 2)
		{
				len = sprintf(page, "LPC status:\n\tCurrent Temp: %d°C\n\tMaximum Temp: %d°C\n\tMinimum Temp: %d°C\n\tFan power: %d\n\tStatus pwm pin:%s\n\tPwm value: %d\n", status.temp_cur, status.temp_max, 
										status.temp_min, status.fan_power, 
										((status.pwm.status_pin==OBSOLETE) ? "OBSOLETE" : "UNKNOWN"), 
										status.pwm.pwm_value);
		}
		else
		{
				len = sprintf(page, "Unknown the version of lpc firmware : %d\n", status.version_lpc); 	
		}

		*eof = 1;

		return len;
}

static int lpc_support_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	dprintkl(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %s()\n", __func__);

	if (strstr(buf,"halt") != 0)
	{
			dprintkl("Turn OFF the STB\n");
			clear_support_struct();

			/* Config POWER BUTTON */
			reg_write_prot(state,CONFIG_RCU_PARAM,status.ir_code); //new wakeup ir code
			reg_write_prot(state, STB_OFF, 0x11111111);	
	}

	return count;
}

/*************************/
/*      WAKEUP TIME      */
/*************************/
static int lpc_wakeup_time_read(char *page, char **start, off_t off, int count,
                         			  int *eof, void *data)
{
		dprintkl(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %s()\n", __func__);
		return 0;
}

static int lpc_wakeup_time_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
		struct local_tm t;
		local_time_t tmp;
		int mjd1;
		unsigned int value=0;

		dprintkl(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %s()\n", __func__);
	
		sscanf(buf,"%u",(int *)&tmp);
		local_time_t_to_tm(&tmp, &t);

		dprintkl("------------>t.tm_year: %d\n",t.tm_year+1900);
		dprintkl("------------>t.tm_mon: %d\n",t.tm_mon);
		dprintkl("------------>t.tm_mday: %d\n",t.tm_mday);
		dprintkl("------------>t.tm_hour: %d\n",t.tm_hour);
		dprintkl("------------>t.tm_min: %d\n",t.tm_min);
	
		if (local_tm_to_mjd(&t, &mjd1) < 0)	dprintkl("ERROR!\n");

		dprintkl("Date in mjd format: %d\n",mjd1);

		value=mjd1;	//date
		value=(value<<8);
		value|=convert_decimal_to_bcd(t.tm_hour);	//hour
		value=(value<<8);
		value|=convert_decimal_to_bcd(t.tm_min);	//minute

		dprintkl("Value to write in data register: 0x%08X\n",value);
	
		reg_write_prot(state,TIMER_ON_STB,value);

		return count;
}

/********************/
/*       VERSION    */
/********************/
static int lpc_version_read(char *page, char **start, off_t off, int count,
                           int *eof, void *data)
{
		int len = 0;

		dprintkl(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %s()\n", __func__);

		len = sprintf(page, "%d\n", status.version_lpc);

		*eof = 1;

		return len;	
}

struct LpcProcStructure_s lpcProc[] = 
{
		{cProcDir  , cProcNotExist, "stb"                             , NULL, NULL, NULL},
		{cProcDir  , cProcNotExist, "stb/lpc"                    	    , NULL, NULL, NULL},
		{cProcEntry, cProcNotExist, "stb/lpc/rtc"                     , NULL, lpc_rtc_read, lpc_rtc_write},
		{cProcEntry, cProcNotExist, "stb/lpc/was_timer_wakeup"        , NULL, lpc_was_wakeup_read, lpc_was_wakeup_write},
		{cProcEntry, cProcNotExist, "stb/lpc/support"				          , NULL, lpc_support_read, lpc_support_write},
		{cProcEntry, cProcNotExist, "stb/lpc/wakeup_time"			        , NULL, lpc_wakeup_time_read, lpc_wakeup_time_write},
		{cProcEntry, cProcNotExist, "stb/lpc/version"                 , NULL, lpc_version_read, NULL},
};

struct proc_dir_entry *exist_proc_dir(char *name, struct proc_dir_entry *entry) 
{
		int len;
		struct proc_dir_entry *child_proc_dir, *parent_entry;

		if (!entry)
			parent_entry = &proc_root;
		else
			parent_entry = entry;

		len = strlen(name);

		for (child_proc_dir = parent_entry->subdir; child_proc_dir; child_proc_dir=child_proc_dir->next) 
		{
			 	if ((child_proc_dir->namelen == len) && (! memcmp(child_proc_dir->name, name, len)))
					return child_proc_dir;
		}

		return NULL;
}

struct proc_dir_entry * find_proc_dir(char * name)
{
		int i;

		for(i = 0; i < sizeof(lpcProc) / sizeof(lpcProc[0]); i++)
		{
		  if((lpcProc[i].type == cProcDir) && (strcmp(name, lpcProc[i].name) == 0))
		    return lpcProc[i].entry;
		}

		return NULL;
}

/* The function returns the directory name */
char * dirname(char * name)
{
		static char path[100];
		int i = 0;
		int pos = 0;

		while((name[i] != 0) && (i < sizeof(path)))
		{
		  if(name[i] == '/')
		    pos = i;
		  path[i] = name[i];
		  i++;
		}

		path[i] = 0;
		path[pos] = 0;

		return path;
}

/* The function returns the base name */
char * basename(char * name)
{
		int i = 0;
		int pos = 0;

		while(name[i] != 0)
		{
		  if(name[i] == '/')
		    pos = i;
		  i++;
		}

		if(name[pos] == '/')
		  pos++;

		return name + pos;
}

static int install_lpc_procs(void)
{
  int i;
  char *path;
  char *name;

  for(i = 0; i < sizeof(lpcProc) / sizeof(lpcProc[0]); i++)
  {
    path = dirname(lpcProc[i].name);
    name = basename(lpcProc[i].name);
    switch(lpcProc[i].type)
    {
			case cProcDir:

				lpcProc[i].entry = exist_proc_dir(name, find_proc_dir(path));

				if (!lpcProc[i].entry) {
						lpcProc[i].entry = proc_mkdir(name, find_proc_dir(path));
						dprintkl("%s(): not exist subdir for entry %s\n", __func__, lpcProc[i].name);
				}
				else 
				{
						lpcProc[i].proc_dir_entry_exists = cProcAlreadyExist;
						dprintkl("%s(): already exist subdir for entry %s\n", __func__, lpcProc[i].name);
				}

			  if(lpcProc[i].entry == NULL)
						dprintkl("%s(): not exist subdir for entry %s\n", __func__, lpcProc[i].name);

			  break;
			case cProcEntry:
				lpcProc[i].entry = create_proc_entry(name, 0, find_proc_dir(path));

			  if(lpcProc[i].entry != NULL)
			  {
						lpcProc[i].entry->read_proc = lpcProc[i].read_proc;
						lpcProc[i].entry->write_proc = lpcProc[i].write_proc;
			  }
			  else
			  {
						dprintkl("%s(): could not create entry %s\n", __func__, lpcProc[i].name);
			  }
			  break;
			default:
			  dprintkl("%s(): invalid type %d\n", __func__, lpcProc[i].type);
    }
  }

  return 0;
}

static void remove_lpc_procs(void)
{
  int i;
  char *name;

  for(i = sizeof(lpcProc)/sizeof(lpcProc[0]) - 1; i >= 0; i--)
  {
    name = basename(lpcProc[i].name);
		dprintkl("%s(): entry name %s\n", __func__, name);

		if (lpcProc[i].type == cProcDir) {
			if (lpcProc[i].proc_dir_entry_exists == cProcNotExist) {
					dprintkl("%s(): entry %s deleted\n", __func__, name);
					remove_proc_entry(name, lpcProc[i].entry->parent);
			}
			else 
					dprintkl("%s(): entry %s not deleted\n", __func__, name);
		}
		else {
			remove_proc_entry(name, lpcProc[i].entry->parent);
		}
  }
}

/******************************************************************************/
static int __init lpc_init(void)
{
	int ret=0;
	unsigned char value=0;
	dev_t dev;

	clear_support_struct();

	/* In TURN_OFF command, this variable is re-set with the correct value */
	status.ir_code=IR_CODE;

	dev = MKDEV(LPC_MAJOR_NUM, LPC_MINOR_START);

    cdev_init(&lpc_cdev, &lpc_fops);
    lpc_cdev.owner = THIS_MODULE;
    lpc_cdev.ops = &lpc_fops;
    if (cdev_add(&lpc_cdev, dev, LPC_NUMBER_OF_CONTROLLERS) < 0)
    {
        printk("Couldn't register '%s' driver\n", LPC_DEVICE_NAME);
        return -1;
    }
    printk("Registering device '%s', major '%d'\n", LPC_DEVICE_NAME, LPC_MAJOR_NUM);
	
	/* Check if the debug which debug is actived */
	dprintk("Activated 'debug'\n");
	dprintkl("Activated 'debugl'\n");

    ret=i2c_add_driver(&lpc_driver);

	/* Config POWER BUTTON */
	reg_write_prot(state,CONFIG_RCU_PARAM,status.ir_code); //new

//		/* Config date and time to zero */
//		reg_write_prot(state,MODIFY_TIME_DATA,0x00000000);

	/* Config max and min temperature */
	reg_write_prot(state,CONFIG_FAN_CTL_PAR,0x322D0000);//max 32->50 min 2D->45
	status.temp_max=0x32;
	status.temp_min=0x2D;

	/* Read current temperature */
	value=reg_read(state,READ_TEMP);
	dprintkl("Current temperature: %d\n\n",value);
	status.temp_cur=value;

	status.version_lpc=read_i2c(1);
	printk("LPC firmware version: %d\n",status.version_lpc);

	if(status.version_lpc<2)
	{
		/* Config fan */
		reg_write_prot(state,FAN_POWER,0x00);//fan off
		status.fan_power=0;

		/* Config pwm in input*/
		reg_write_prot(state,PWM_VALUE,0x00FF0000);//pin in input, pwm off
		status.pwm.status_pin=INPUT_PWM;
		status.pwm.pwm_value=0xFF;
	}
	else if(status.version_lpc>=2)
	{
		/* Config fan */
		reg_write_prot(state,FAN_POWER,0);//fan off
		status.fan_power=0;

		/* Config pwm in input*/
		reg_write_prot(state,PWM_VALUE,0x1F000000);//max BL
		status.pwm.status_pin=OBSOLETE;
		status.pwm.pwm_value=0xFF;	//0x1F * 8 = 255
	}
	else
	{
		printk("Unknown version: %d\n",status.version_lpc);
		return ret;	
	}

	install_lpc_procs();

	printk("Version of lpc_drv: %s\n", LPC_DRIVER_VERSION);

	return ret;
}

static void __exit lpc_exit(void)
{
    cdev_del(&lpc_cdev);

	/* i2c command */
	i2c_del_driver(&lpc_driver);

	/* proc management */
	remove_lpc_procs();

    printk("Unregistering device '%s'\n", LPC_DEVICE_NAME);
}

module_init(lpc_init);
module_exit(lpc_exit);

module_param(debug, int, 0644);
module_param(debugl, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off LPC debugging (default:off).");
MODULE_PARM_DESC(debugl, "Turn on/off LPC debugging in I/O operation.");

#ifdef CONFIG_SH_QBOXHD_1_0
#define MOD               "-HD"
#elif  CONFIG_SH_QBOXHD_MINI_1_0
#define MOD               "-Mini"
#else
#define MOD               ""
#endif

#define LPC_VERSION       "0.0.6"MOD
MODULE_VERSION(LPC_VERSION);

MODULE_DESCRIPTION("I2C driver for the LPC");
MODULE_AUTHOR("Duolabs");
MODULE_LICENSE("GPL");

