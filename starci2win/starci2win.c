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
#include <linux/fs.h>
#include <linux/mm.h>

#include <linux/poll.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>

#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <asm/semaphore.h>
#include "starci2win_i2c.h"
#include "starci2win.h"
#include "ci_aux_functions.h"

#include <linux/kthread.h>

/*
#ifdef __DRV_DEBUG
#define D_DEBUG(fmt, args...)    printk(KERN_NOTICE "%s: " fmt, __FUNCTION__, ##args)
//#define D_DEBUG(x...)  printk (KERN_DEBUG "Starci: " x)
#else
#define D_DEBUG(x...)  do {} while (0)
#endif
*/

//#define DDD 1
//#define	PRINT_READ	1
//#define PRINT_WRITE	1


#ifdef DDD
	#define D_DEBUG(args...) \
		do { \
			printk("starci2win: " args); \
		} while (0)
#else
	#define D_DEBUG(args...) \
		do { \
		} while (0)
#endif

#define D_ERROR(fmt, args...)    printk(KERN_ERR "%s: " fmt, __FUNCTION__, ##args)

#define NUM_MINORS 1

/* Task status */
#define		NO_CMD		0
#define		DETECT_CI	1
//#define		READ		2
#define		PAUSE		3
#define		IN_PAUSE	4
#define		GO			5
#define		KILL		9

/* Semaphore */
#define		NO_SEM		0
#define		HAS_SEM		1


static unsigned char cmd_detect;
static unsigned char cmd_read[2];

static struct cdev starci_cdev;
/*static*/ State_ci_t ci_device[CI_NUMBER_OF_CONTROLLERS];


/* For the interrupt...and read*/
static void irq_detect_handler(void);
static void read_data_function(void);

/* Semaphore to acquire the 'starci' resource */
DECLARE_MUTEX(starci_sem);

/* Wait queue to check the CI status (present or not) */
wait_queue_head_t wq;

typedef struct
{
	volatile unsigned char slot_th;
	struct task_struct * kth;
	unsigned char has_sem;
}Kth_Control_t;

Kth_Control_t kth[2];

static int read_thread(void *data);

int debug;

typedef enum {
    STARCI_NO_ERROR = 0,
    STARCI_ERROR_INIT_DRIVER = -100,
} StartciError;

unsigned long base_address;
extern CI_Info_Control_t ci_info[CI_NUMBER_OF_CONTROLLERS];
extern Read_Action_t read_op;

static void init_ci_device(unsigned char id);

/****************************************************************************************/
static void irq_detect_handler(void)
{
	unsigned char r=0,r1=0;
	mdelay(3);
	r=read_i2c(0x1A);
	r=(r&MASK_DET);
	if(r&DET_A)
	{
		r1=read_i2c(MOD_A_CTRL);
		if((r1&MODULE_PRESENT)!=MODULE_PRESENT)
		{
			D_DEBUG("CAM 0 IS REMOVED\n");
			ci_device[0].status=MODULE_NOT_PRESENT;
			r1=0;
			do{
				udelay(500); //500us
				printk("+");
//			}while( (ci_device[0].status!=MODULE_EXTRACTED) && (read_op.cams[0].status!=DATA_AVAILABLE) && (read_op.cams[0].status!=FREE) );
			r1++;
			}while( (ci_device[0].status!=MODULE_EXTRACTED) && (read_op.cams[0].status==IN_READ) && (r1<200) ); /* r1<200 = 100ms */

			/* Clean the semaphore */
			if(read_op.cams[0].status==IN_READ)
			{
				down_trylock(&starci_sem);
				up(&starci_sem);
			}

			if(read_op.cams[0].data_slot!=NULL)
				kfree(read_op.cams[0].data_slot);
			init_ci_device(0);
			ci_device[0].isInit=0xEE;
			ci_device[0].status=MODULE_EXTRACTED;
			cmd_read[0]=KILL;
			printk("CAM 0 IS REMOVED\n");
			kth[0].kth=NULL;
		}
		else
		{
			D_DEBUG("CAM 0 IS INSERTED\n");
			ci_device[0].status=MODULE_PRESENT;
			/* Create the thread to poll the CI */
			cmd_read[0]=KILL;
			mdelay(1);
			cmd_read[0]=NO_CMD;
			if(kth[0].kth==NULL)
				kth[0].kth=kthread_run(read_thread,(void *)&(kth[0].slot_th),"Read_thr_0");
			printk("CAM 0 IS INSERTED\n");
		}
	}
	if(r&DET_B)
	{
		r1=read_i2c(MOD_B_CTRL);
		if((r1&MODULE_PRESENT)!=MODULE_PRESENT)
		{
			D_DEBUG("CAM 1 IS REMOVED\n");
			ci_device[1].status=MODULE_NOT_PRESENT;
			r1=0;
			do{
				udelay(500); //500us
				printk("-");
//			}while( (ci_device[1].status!=MODULE_EXTRACTED) && (read_op.cams[1].status!=DATA_AVAILABLE) && (read_op.cams[1].status!=FREE) );
			r1++;
			}while( (ci_device[1].status!=MODULE_EXTRACTED) && (read_op.cams[1].status==IN_READ) && (r1<200) );	/* r1<200 = 100ms */

			/* Clean the semaphore */
			if(read_op.cams[1].status==IN_READ)
			{
				down_trylock(&starci_sem);
				up(&starci_sem);
			}

			if(read_op.cams[1].data_slot!=NULL)
				kfree(read_op.cams[1].data_slot);
			init_ci_device(1);
			ci_device[1].isInit=0xEE;
			ci_device[1].status=MODULE_EXTRACTED;
			cmd_read[1]=KILL;
			printk("CAM 1 IS REMOVED\n");
			kth[1].kth=NULL;
		}
		else
		{
			D_DEBUG("CAM 1 IS INSERTED\n");
			ci_device[1].status=MODULE_PRESENT;
			/* Create the thread to poll the CI */
			cmd_read[1]=KILL;
			mdelay(1);
			cmd_read[1]=NO_CMD;

			if(kth[1].kth==NULL)
				kth[1].kth=kthread_run(read_thread,(void *)&(kth[1].slot_th),"Read_thr_1");
			printk("CAM 1 IS INSERTED\n");
		}
	}

	return 0;
}

static irqreturn_t try_irq(int irq, void *dev_id)
{
	disable_irq(2);
	cmd_detect=DETECT_CI;
	wake_up_interruptible(&wq);
    return IRQ_HANDLED;
}

/**********************************************************************************************/

static void read_data_function(void)
{
	int rs=0;
	read_op.cams[read_op.id_in_read].data_slot=(unsigned char *) kmalloc(read_op.cams[read_op.id_in_read].len_slot,GFP_USER);

	if(read_op.cams[read_op.id_in_read].data_slot!=NULL)
	{
		rs=q_read(read_op.id_in_read,
				read_op.cams[read_op.id_in_read].data_slot,
				read_op.cams[read_op.id_in_read].len_slot);

		if(rs<=0)
		{
			if(rs<0) D_DEBUG("Error when read data\n");
			kfree(read_op.cams[read_op.id_in_read].data_slot);
			read_op.cams[read_op.id_in_read].data_slot=NULL;
			read_op.cams[read_op.id_in_read].len_slot=0;
			if( (rs<0) )//&& (ci_device[read_op.id_in_read].status!=MODULE_PRESENT) )
			{
				ci_device[read_op.id_in_read].status=MODULE_EXTRACTED;
			}
			read_op.cams[read_op.id_in_read].status=FREE;
		}
		else
		{
			read_op.cams[read_op.id_in_read].len_slot=rs;
			read_op.cams[read_op.id_in_read].status=DATA_AVAILABLE;
		}
	}
	else
	{
		read_op.cams[read_op.id_in_read].len_slot=0;
		read_op.cams[read_op.id_in_read].status=FREE;
	}
}

void check_kth_status(void)
{
	unsigned char cnt=0;
	for(cnt=0;cnt<2;cnt++)
	{
		if( (ci_device[cnt].status==MODULE_PRESENT) && 
			(kth[cnt].kth!=NULL) )
		{		
			if( (kth[cnt].kth->exit_state>0) &&
				(kth[cnt].has_sem==HAS_SEM) )
			{
				down_trylock(&starci_sem);
				up(&starci_sem);
				kth[cnt].has_sem=NO_SEM;
				read_op.cams[cnt].status=FREE;
				cmd_read[cnt]=KILL;
				kth[cnt].kth=NULL;
			}
		}
	}
}

static int detect_thread(void *data)
{
	while(1)
	{
		if(cmd_detect==KILL)
		{
			cmd_detect=NO_CMD;
			break;
		}

		if( (ci_device[0].status==MODULE_PRESENT) ||
			(ci_device[1].status==MODULE_PRESENT) )
		{
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(250);	/* 250ms */
			check_kth_status();
		}
		else
		{
			wait_event_interruptible(wq,(cmd_detect==DETECT_CI));
		}	

		if(cmd_detect==DETECT_CI)
		{
			cmd_detect=GO;
			irq_detect_handler();
			enable_irq(2);
		}
	}
	return 0;
}

static int read_thread(void *data)
{
	unsigned char slot_id=*((unsigned char*)data);

	if(read_op.cams[slot_id].data_slot!=NULL)
	{
		kfree(read_op.cams[slot_id].data_slot);
		read_op.cams[slot_id].data_slot=NULL;
		read_op.cams[slot_id].len_slot=0;		
	}
	
	while(1)
	{
		if(cmd_read[slot_id]==KILL)
		{
			cmd_read[slot_id]=NO_CMD;
			break;
		}

		if(cmd_read[slot_id]==PAUSE)
		{
			cmd_read[slot_id]=IN_PAUSE;
		}
		
		/* Poll time */
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(300);	/* 300ms */
		if(cmd_read[slot_id] == GO)
		{
			if( (ci_device[slot_id].status==MODULE_PRESENT) &&
				(ci_device[slot_id].isInit==1) )
			{
				/* Try to acquire the 'starci' resource */
				if (down_interruptible(&starci_sem) != 0)
					return -ERESTARTSYS;

				kth[slot_id].has_sem=HAS_SEM;

				if (read_op.cams[slot_id].status==FREE)
				{
					read_op.cams[slot_id].status=IN_READ;
					read_op.id_in_read=slot_id;
					read_op.cams[slot_id].len_slot=4096;
					if(read_op.cams[read_op.id_in_read].data_slot!=NULL)
					{
						kfree(read_op.cams[read_op.id_in_read].data_slot);
						read_op.cams[read_op.id_in_read].data_slot=NULL;
					}

					read_data_function();
				}
				/* Release the 'starci' resource */
				up(&starci_sem);
				kth[slot_id].has_sem=NO_SEM;
			}
		}
	}
	D_DEBUG("\n======>EXIT FROM THREAD FOR %d ID\n\n",slot_id);
	return 0;
}

/**********************************************************************************************/
void reg_set (unsigned long base_address, unsigned long reg, unsigned int bits, unsigned int mask)
{
    unsigned long reg_address = 0x0;
    unsigned int val;

    reg_address = (unsigned long)(base_address + reg);

    val = ctrl_inl(reg_address);
    if (mask)
    {
        val &= ~(mask);
        ctrl_outl(val, reg_address);
    }
    val |= bits;
    ctrl_outl(val, reg_address);
}

void reg_only_set (unsigned long base_address, unsigned long reg, unsigned int value)
{
    unsigned long reg_address = 0x0;

    reg_address = (unsigned long)(base_address + reg);

    ctrl_outl(value, reg_address);
}

unsigned long reg_get (unsigned long base_address, unsigned long reg)
{
    unsigned long reg_address = 0x0;

    reg_address = (unsigned long)(base_address + reg);

    return ctrl_inl(reg_address);
}

static void init_ci_device(unsigned char id)
{
	ci_info[id].NgBuf=0;
	ci_info[id].linkID=0;
	ci_info[id].SessionProfile=0;
	ci_info[id].in_write=0;
	ci_info[id].to_read=0;
	read_op.cams[id].data_slot=NULL;
	read_op.cams[id].len_slot=0;
	read_op.cams[id].status=FREE;
}

static void init_ci_info(void)
{
	unsigned char i=0;
	for(i=0;i<CI_NUMBER_OF_CONTROLLERS;i++)
		init_ci_device(i);

	read_op.id_in_read=0xFF;
}

static int device_init(void)
{
	unsigned char i=0;
    base_address = (unsigned long)ioremap(STARCI_BASE_ADDRESS, STARTCI_IO_SIZE);

	init_ci_info();

	for(i=0;i<CI_NUMBER_OF_CONTROLLERS;i++)
	{
		ci_device[i].inuse=0;
		ci_device[i].isInit=0xFF;
		ci_device[i].status=0xFF;
	}
	ci_device[0].type_dev=CI;
	ci_device[1].type_dev=CI;

    return STARCI_NO_ERROR;
}

static void device_uninit(void)
{
    iounmap((void *)base_address);

}

unsigned char detect_module(unsigned char slotid)
{
	return ci_device[slotid].status;
}

static int
starci_open (struct inode *inode, struct file *file)
{
	unsigned int ci_id;
	int rc = 0;

	D_DEBUG("Opening device!\n");
	if ((ci_id = MINOR(inode->i_rdev)) <= CI_NUMBER_OF_CONTROLLERS)
	{
		if(ci_id<2)
		{
			if (ci_device[ci_id].inuse == 0)
			{
				ci_device[ci_id].inuse++;
				D_DEBUG("Common Interface[%d] char device available: %d\n", ci_id, ci_device[ci_id].inuse);
			}
			else
			{
				D_DEBUG("ci[%d] is busy\n", ci_id);
				rc = -EBUSY;
			}
		}
		else
		{
			if (ci_device[ci_id].inuse == 0)
			{
				ci_device[ci_id].inuse++;
				D_DEBUG("Common Interface INPUT [%d] char device available: %d\n", (ci_id%2), ci_device[ci_id].inuse);
			}
			else
			{
				D_DEBUG("input[%d] is busy\n", (ci_id%2));
				rc = -EBUSY;
			}
		}
		if(ci_id==1)
		{
			/* Create a thread for the pollig */
		//	test_pthread();
		}
	}
	else
	{
		D_DEBUG("Couldn't open device with [%d] as MINOR number\n", ci_id);
		rc = -ENODEV;
	}
	return (rc);
}

static int
starci_release (struct inode *inode, struct file *file)
{
	unsigned int ci_id;
	int rc = 0;
	D_DEBUG("Closing device!\n");
	if ((ci_id = MINOR(inode->i_rdev)) <= CI_NUMBER_OF_CONTROLLERS)
	{
		if (ci_device[ci_id].inuse > 0)
		{
			ci_device[ci_id].inuse--;
		}
		else
		{
			if(ci_id<2)
			{
				D_DEBUG("Device ci[%d] is not release: %d\n",	ci_id, ci_device[ci_id].inuse);
				rc = -EINVAL;
			}
			else
			{
				D_DEBUG("Device input[%d] is release: %d\n", (ci_id%2), ci_device[ci_id].inuse);
				rc = -EINVAL;
			}

		}
	}
	else
	{
		D_DEBUG("Couldn't close device with [%d] as MINOR number\n", ci_id);
		rc = -ENODEV;
	}
	return rc;
}

static ssize_t
starci_read (struct file *file, char *buf, size_t count, loff_t *ppos)
{
	unsigned int ci_id;
	int ret=0;

	if(count==256)	/* FIXME */
		return 0;

	if ((ci_id = MINOR(file->f_dentry->d_inode->i_rdev)) <= CI_NUMBER_OF_CONTROLLERS)
	{
		if(read_op.cams[ci_id].data_slot!=NULL)
		{
			memcpy(buf,read_op.cams[ci_id].data_slot,read_op.cams[ci_id].len_slot);

			ret=read_op.cams[ci_id].len_slot;
			kfree(read_op.cams[ci_id].data_slot);
			read_op.cams[ci_id].data_slot=NULL;
		}
		read_op.cams[ci_id].len_slot=0;
		read_op.cams[ci_id].status=FREE;
	}

	return ret;
}

static ssize_t
starci_write (struct file *file,const char *buf, size_t count, loff_t *ppos)
{
	unsigned int ci_id;
	int ret=0;
	int cnt=0;
#ifdef PRINT_WRITE
	unsigned int debugppp=0;
#endif

	if ((ci_id = MINOR(file->f_dentry->d_inode->i_rdev)) <= CI_NUMBER_OF_CONTROLLERS)
	{
		if(cmd_read[0]==GO)
			cmd_read[0]=PAUSE;
		if(cmd_read[1]==GO)
			cmd_read[1]=PAUSE;

		while( (cmd_read[0]==PAUSE) || (cmd_read[1]==PAUSE) )
		{
			if(cnt>3000)	/* 3 seconds... */
				break;
			udelay(1000);	/* Wait 1ms	*/
			cnt++;
		}
		if(cnt>3000)
		{
			D_DEBUG("Write isn't possible because the cam is 'IN_READ'\n");
			if(cmd_read[0]==PAUSE)
				cmd_read[0]=GO;
			if(cmd_read[1]==PAUSE)
				cmd_read[1]=GO;
			return 0;
		}

		/* Try to acquire the 'starci' resource */
		if (down_interruptible(&starci_sem) != 0)
			return -ERESTARTSYS;
		kth[ci_id].has_sem=HAS_SEM;

#ifdef PRINT_WRITE
		printk("\n");
		printk("### ALL ### DATA TO WRITE ...%d bytes\n",count);
		for(debugppp=0;debugppp<count;debugppp++)
			printk("[%d]0x%02X, ",debugppp,(unsigned char)buf[debugppp]);
		printk("\n");
		debugppp=0;
#endif
		ci_info[ci_id].in_write=1;
		ret=q_write(ci_id,(char *)buf,count);
		ci_info[ci_id].in_write=0;
		D_DEBUG("Write %d bytes completed\n",ret);

		if(cmd_read[0]==IN_PAUSE)
			cmd_read[0]=GO;
		if(cmd_read[1]==IN_PAUSE)
			cmd_read[1]=GO;
		
		/* Release the 'starci' resource */
		up(&starci_sem);
		kth[ci_id].has_sem=NO_SEM;

		return ret;
	}
	return 0;
}


static int
starci_ioctl (struct inode *inode, struct file *file,
              unsigned int cmd, unsigned long arg)
{
	Module_t module;
	Module_and_ts_t mod_ts;
	Register_t reg;

	unsigned char cis[CIS_LEN];
	int res=-ENOSYS;
	unsigned char result=0,r=0;

	int cnt=0;/* 2 bytes of len */
	unsigned long cnt_1=0;
	unsigned char counter=0;

	unsigned short NGB;
	unsigned int ci_id;
	for(cnt=0;cnt<CIS_LEN;cnt++)
		cis[cnt]=0;

	if ((ci_id = MINOR(inode->i_rdev)) > CI_NUMBER_OF_CONTROLLERS)
		return res;

	switch (cmd)
	{
		case 0:
			/* Set the read thread in pause */
			if(cmd_read[0]==GO)
				cmd_read[0]=PAUSE;
			if(cmd_read[1]==GO)
				cmd_read[1]=PAUSE;
			cnt=0;
	
			while( (cmd_read[0]==PAUSE) || (cmd_read[1]==PAUSE) )
			{
				if(cnt>3000)	/* 3 seconds... */
					break;
				msleep(1);		/* Wait 1ms */
				cnt++;
			}
			if(cnt>3000)
			{
				D_DEBUG("Reset of module isn't possible beacuse the cam is 'IN_READ'\n");
				if(cmd_read[0]==PAUSE)
					cmd_read[0]=GO;
				if(cmd_read[1]==PAUSE)
					cmd_read[1]=GO;
				break;
			}

			/* Try to acquire the 'starci' resource */
			if (down_interruptible(&starci_sem) != 0)
				return -ERESTARTSYS;

			kth[ci_id].has_sem=HAS_SEM;

			ci_device[ci_id].isInit=0xAA;
			counter=ModifyBuffSize(&NGB,ci_id);
			if(counter==1)
			{
				ci_device[ci_id].isInit=1;
				printk("CORRECT INIT\n");
			}
			else
			{
				ci_device[ci_id].isInit=0;
				printk("NO CORRECT INIT\n");
			}
			if(cmd_read[0]==IN_PAUSE)
				cmd_read[0]=GO;
			if(cmd_read[1]==IN_PAUSE)
				cmd_read[1]=GO;

			if(cmd_read[ci_id]!=GO)
				cmd_read[ci_id]=GO;

			/* Release the 'starci' resource */
			up(&starci_sem);
			kth[ci_id].has_sem=NO_SEM;
	
			break;
		case IOCTL_SET_MODE_MODULE://1
			D_DEBUG("ioctl IOCTL_SET_MODE_MODULE\n");
			copy_from_user((void *) &module,
                           (const void *) arg,
                           sizeof(Module_t));
			D_DEBUG("------------>module.module:%d\n",module.module);
			D_DEBUG("------------>module.configuration:%d\n",module.configuration);

			if(module.module==MODULE_A)
			{
				res=read_i2c(MOD_A_CTRL);
				if((res&MODULE_PRESENT)!=0x01)
				{
					/* Only for test */
				//	write_i2c(DEST_MODULE,0x02);//Module A
					D_DEBUG("-------->Module A is not inserted\n");
				}
				else
				{
//					write_i2c(STARCI2WIN_CTRL,0x80);
					write_i2c(STARCI2WIN_CTRL,0x00);
					write_i2c(DEST_MODULE,0x00);
					write_i2c(STARCI2WIN_CTRL,0x01);
					if(module.configuration==CIS)
//						write_i2c(MOD_A_CTRL,0x09);
					write_i2c(MOD_A_CTRL,0x01);
					else
						write_i2c(MOD_A_CTRL,0x05);
					write_i2c(DEST_MODULE,0x02);//Module A
					write_i2c(MP_INTERFACE_CONFIG,0x00);
					write_i2c(POWER_CTRL,0x01);
					write_i2c(MOD_A_ACCESS_TIME,0x44);//set access time to 600ns
				}
			}
			else if(module.module==MODULE_B)
			{
				res=read_i2c(MOD_B_CTRL);
				if((res&MODULE_PRESENT)!=0x01)
				{
					/* Only for test */
				//	write_i2c(DEST_MODULE,0x04);//Module B
					D_DEBUG("-------->Module B is not inserted\n");
				}
				else
				{
//					write_i2c(STARCI2WIN_CTRL,0x80);
					write_i2c(STARCI2WIN_CTRL,0x00);
					write_i2c(DEST_MODULE,0x00);
					write_i2c(STARCI2WIN_CTRL,0x01);
					if(module.configuration==CIS)
//						write_i2c(MOD_B_CTRL,0x09);
					write_i2c(MOD_B_CTRL,0x01);
					else
						write_i2c(MOD_B_CTRL,0x05);
					write_i2c(DEST_MODULE,0x04);//Module B
					write_i2c(MP_INTERFACE_CONFIG,0x00);
					write_i2c(POWER_CTRL,0x01);
					write_i2c(MOD_B_ACCESS_TIME,0x44);//set access time to 600ns

				}
			}
			else if(module.module==MODULE_EXT)
				D_DEBUG("------------>Not implemented YET\n");
			else
				D_DEBUG("------------>BAD MODULE PARAMETER\n");

			break;

		case IOCTL_GET_MODE_MODULE://2
			D_DEBUG("ioctl IOCTL_GET_MODE_MODULE\n");
			copy_from_user((void *) &module,
                           (const void *) arg,
                           sizeof(Module_t));
			D_DEBUG("------------>module.module:%d\n",module.module);
			D_DEBUG("------------>module.configuration:%d\n",module.configuration);

			if(module.module==MODULE_A)
			{
				res=read_i2c(MOD_A_CTRL);
				if((res&MODULE_PRESENT)!=0x01)
				{
					/* Only for test */
					//write_i2c(DEST_MODULE,0x02);//Module A
					D_DEBUG("-------->Module A is not inserted\n");
				}
				else
				{
					res=read_i2c(MOD_A_CTRL);
//					if((res&MODE_MASK)==0x09)
					if((res&MODE_MASK)==0x01)
						module.configuration=CIS;
					else if((res&MODE_MASK)==0x05)
						module.configuration=IO_SPACE;
				}
			}
			else if(module.module==MODULE_B)
			{
				res=read_i2c(MOD_B_CTRL);
				if((res&MODULE_PRESENT)!=0x01)
				{
					/* Only for test */
					//write_i2c(DEST_MODULE,0x02);//Module A
					D_DEBUG("-------->Module A is not inserted\n");
				}
				else
				{
					res=read_i2c(MOD_B_CTRL);
//					if((res&MODE_MASK)==0x09)
					if((res&MODE_MASK)==0x01)
						module.configuration=CIS;
					else if((res&MODE_MASK)==0x05)
						module.configuration=IO_SPACE;
				}
			}
			else if(module.module==MODULE_EXT)
				D_DEBUG("------------>Not implemented YET\n");
			else
				D_DEBUG("------------>BAD MODULE PARAMETER\n");

			copy_to_user((void *) arg,
					(const void *) &module,
					sizeof(Module_t));
			break;

		case IOCTL_READ_CIS://3
			D_DEBUG("ioctl IOCTL_READ_CIS\n");
			/* Little endian read */
			do{
				mdelay(11);
				cis[0]=(unsigned char)ctrl_inb(base_address+0);
				counter++;
			}while( (cis[0]!=0x1D) && (counter<181) );//~ 2 seconds
			if(counter>=181)
			{
				for(cnt=0;cnt<CIS_LEN;cnt++)
					cis[cnt]=0xFF;//return a cis with all bytes at 0xFF
				break;
			}
			for(cnt=0;cnt<CIS_LEN;cnt++)
			{
				mdelay(3);
				cis[cnt]=(unsigned char)ctrl_inb(base_address+cnt_1);
				cnt_1+=4;/* Only even bytes */
			}
			if(cnt==0)
				D_DEBUG("------------>No read\n");
			else
				D_DEBUG("------------>Read complete\n");
			for(cnt=0;cnt<8;cnt++)
			{
				printk("0x%02X ",cis[cnt]);
			}
			printk("\n");
			copy_to_user((void *) arg,
						(const void *) &cis[0],
						CIS_LEN);
			break;
		case IOCTL_WRITE_ADDR://4
			D_DEBUG("ioctl IOCTL_WRITE_ADDR\n");
			copy_from_user((void *) &reg,
					(const void *) arg,
					sizeof(Register_t));
			D_DEBUG("--------->reg.value:%02x\n",reg.value);
			D_DEBUG("--------->reg.offset_addr:%d\n",reg.offset_addr);
			ctrl_outb(reg.value, base_address+(reg.offset_addr*2));
			D_DEBUG("------------>Write register 0x%08X with value 0x%02X\n",base_address+(reg.offset_addr),reg.value);
			break;
		case IOCTL_READ_ADDR://5
			D_DEBUG("ioctl IOCTL_READ_ADDR\n");
			copy_from_user((void *) &reg,
					(const void *) arg,
					sizeof(Register_t));
			reg.value=ctrl_inb(base_address+(reg.offset_addr*2));
			D_DEBUG("------------>Read register 0x%08X with value 0x%02X\n",base_address+(reg.offset_addr),reg.value);
			copy_to_user((void *) arg,
					(const void *) &reg,
					sizeof(Register_t));
			break;

		case IOCTL_DETECT_MODULE://6
			copy_from_user((void *) &module,
                           (const void *) arg,
                           sizeof(Module_t));
			if(module.module==MODULE_A)
			{
				module.configuration=ci_device[0].status;
			}
			else if(module.module==MODULE_B)
			{
				module.configuration=ci_device[1].status;
			}
			copy_to_user((void *) arg,
					(const void *) &module,
					sizeof(Module_t));

			break;

		case IOCTL_CI_SET_POWER://7
				D_DEBUG("ioctl IOCTL_CI_SET_POWER\n");
				copy_from_user((void *) &result,
                           (const void *) arg,
                           sizeof(result));

				if(result==POWER_ON)
				{
					write_i2c(POWER_CTRL,0x01);
					D_DEBUG("TURN_ON\n");
				}
				else if(result==POWER_OFF)
				{
					write_i2c(POWER_CTRL,0x00);
					D_DEBUG("TURN_OFF\n");
				}
				else
					D_DEBUG("-------->Wrong parameter\n");
			break;

		case IOCTL_CI_GET_POWER://8
				D_DEBUG("ioctl IOCTL_CI_GET_POWER\n");
				result=read_i2c(POWER_CTRL);
				if((result&0x01)==0x01)
				{
					result=POWER_ON;
					copy_to_user((void *) arg,
								(const void *) &result,
								sizeof(result));
				}
				else
				{
					result=POWER_OFF;
					copy_to_user((void *) arg,
								(const void *) &result,
								sizeof(result));
				}
				D_DEBUG("-------->POWER result:0x%02X\n",result);
				D_DEBUG("-------->POWER arg:0x%02X\n",*((unsigned char *)arg));
				break;

		case IOCTL_CI_SWF_RESET://9
				D_DEBUG("ioctl IOCTL_CI_SWF_RESET\n");
				copy_from_user((void *) &module,
                           (const void *) arg,
                           sizeof(Module_t));
				D_DEBUG("------------>module.module:%d\n",module.module);
				D_DEBUG("------------>module.configuration:%d\n",module.configuration);

				if(module.module==MODULE_A)
				{
					res=read_i2c(MOD_A_CTRL);

					if((res&MODULE_PRESENT)!=0x01)
						D_DEBUG("------------>Module isn't present\n");
					else
					if((res&MODULE_PRESENT)==0x01)
					{
						write_i2c(MOD_A_CTRL,0x05);//in I/O mode
						ctrl_outb(0x08, base_address+(STARCI_REG_CTRL_STATUS*2));//reset comand of the CAM
						D_DEBUG("------------>Reset the Module\n");
					}
				}

				if(module.module==MODULE_B)
				{
					res=read_i2c(MOD_B_CTRL);

					if((res&MODULE_PRESENT)!=0x01)
						D_DEBUG("------------>Module isn't present\n");
					else
					if((res&MODULE_PRESENT)==0x01)
					{
						write_i2c(MOD_B_CTRL,0x05);//in I/O mode
						ctrl_outb(0x08, base_address+(STARCI_REG_CTRL_STATUS*2));//reset comand of the CAM
						D_DEBUG("------------>Reset the Module\n");
					}

				}
				break;
		case IOCTL_CI_SET_CIS_IO_SPACE_MODE://10
				D_DEBUG("ioctl IOCTL_CI_SET_CIS_IO_SPACE_MODE\n");
				copy_from_user((void *) &module, //no test!!!
                           (const void *) arg,
                           sizeof(Module_t));
				if(module.module==MODULE_A)
				{
					if(module.configuration==CIS)
//						write_i2c(MOD_A_CTRL,0x09);
						write_i2c(MOD_A_CTRL,0x01);
					else
						write_i2c(MOD_A_CTRL,0x05);
				}
				else if(module.module==MODULE_B)
				{
					if(module.configuration==CIS)
//						write_i2c(MOD_B_CTRL,0x09);
						write_i2c(MOD_B_CTRL,0x01);
					else
						write_i2c(MOD_B_CTRL,0x05);
				}
				break;
		case IOCTL_CI_SET_TS_MODE://11
				D_DEBUG("ioctl IOCTL_CI_SET_TS_MODE\n");
				copy_from_user((void *) &mod_ts,
                           (const void *) arg,
                           sizeof(Module_and_ts_t));

                printk("\n");
                printk("Module: 0x%02X\n",mod_ts.module);
                printk("Configuration: 0x%02X\n",mod_ts.configuration);
                printk("Ts_number: 0x%02X\n",mod_ts.ts_number);
                printk("\n");

				if(mod_ts.module==MODULE_A)
				{
					/* Before to change the TS... */
					unsigned char res_tmp=0;
					res_tmp=read_i2c(TWIN_MODE_TS);
					res_tmp&=0xDF; //Clear 1 bit : TSMAP

					write_i2c(SINGLE_MODE_TS,0x00);
					if(mod_ts.ts_number==TS_0)
					{
						res_tmp|=0x80;	//So if the TS input are inverted, they will be inverted (and select twin mode)
						write_i2c(TWIN_MODE_TS,res_tmp);
					}
					else if(mod_ts.ts_number==TS_1)
					{
						res_tmp|=0xA0;	//So if the TS input are inverted, they will be inverted (and select twin mode)
						write_i2c(TWIN_MODE_TS,res_tmp);
					}
					mdelay(3);
					/* ... after trought the module */
					result=read_i2c(MOD_A_CTRL);
					result&=0x9F; //Clear 2 bits so TSIN->TSOUT
					if(mod_ts.configuration==TS_TO_CAM)
					{
						result|=0x60;// TSIN->Module_A->TSOUT
					}
					else
					{
						//result|=0x20;// TSIN->Module_A->TSOUT /* With 00 work but with 01 NO work */
					}
					write_i2c(MOD_A_CTRL,result);

					printk("TS trought Module A\n");
				}
				else if(mod_ts.module==MODULE_B)
				{
					/* Before to change the TS... */
					unsigned char res_tmp=0;
					res_tmp=read_i2c(TWIN_MODE_TS);
					res_tmp&=0xDF; //Clear 1 bit : TSMAP

					write_i2c(SINGLE_MODE_TS,0x00);
					if(mod_ts.ts_number==TS_0)
					{
						res_tmp|=0xA0;	//So if the TS input are inverted, they will be inverted (and select twin mode)
						write_i2c(TWIN_MODE_TS,res_tmp);//TSin0 -> Module B -> TSout0
					}
					else if(mod_ts.ts_number==TS_1)
					{
						res_tmp|=0x80;	//So if the TS input are inverted, they will be inverted (and select twin mode)
						write_i2c(TWIN_MODE_TS,res_tmp);
					}
					mdelay(3);
					/* ... after trought the module */
					result=read_i2c(MOD_B_CTRL);
					result&=0x9F;//Clear 2 bits so TSIN->TSOUT
					if(mod_ts.configuration==TS_TO_CAM)
					{
						result|=0x60;// TSIN->Module_B->TSOUT
					}
					else
					{
						//result|=0x20;// TSIN->Module_B->TSOUT /* With 00 work but with 01 NO work */
					}
					write_i2c(MOD_B_CTRL,result);

					printk("TS trought Module B\n");
				}
				break;
		case IOCTL_CI_GET_TS_MODE://12
				D_DEBUG("ioctl IOCTL_CI_GET_TS_MODE\n");
				copy_from_user((void *) &mod_ts,
                           (const void *) arg,
                           sizeof(Module_and_ts_t));
				if(mod_ts.module==MODULE_A)
				{
					result=read_i2c(MOD_A_CTRL);
					result&=0x60;
					if(result==0x60)
					{
						D_DEBUG("------------>TSIN->Module_A->TSOUT\n");
						mod_ts.configuration=TS_TO_CAM;
					}
					else if(result==0x00)
					{
						D_DEBUG("------------>TSIN->TSOUT\n");
						mod_ts.configuration=TS_BY_PASS;
					}
					else
					{
						D_DEBUG("------------>Strange state: 0x%2X\n",result);
						mod_ts.configuration=0xFF;
					}
				}
				else if(mod_ts.module==MODULE_B)
				{
					result=read_i2c(MOD_B_CTRL);
					result&=0x60;
					if(result == 0x60)
					{
						D_DEBUG("------------>TSIN->Module_B->TSOUT\n");
						mod_ts.configuration=TS_TO_CAM;
					}
					else if(result == 0x00)
					{
						D_DEBUG("------------>TSIN->TSOUT\n");
						mod_ts.configuration=TS_BY_PASS;
					}
					else
					{
						D_DEBUG("------------>Strange state: 0x%2X\n",result);
						mod_ts.configuration=0xFF;
					}
				}
				result=read_i2c(TWIN_MODE_TS);
				result&=0x20;
				if(result==0x20)
				{
					if(mod_ts.module==MODULE_A)
						mod_ts.ts_number=TS_1;
					else if(mod_ts.module==MODULE_B)
						mod_ts.ts_number=TS_0;
				}
				else if(result==0x00)
				{
					if(mod_ts.module==MODULE_A)
						mod_ts.ts_number=TS_0;
					else if(mod_ts.module==MODULE_B)
						mod_ts.ts_number=TS_1;
				}
				else
					D_DEBUG("------------>ERROR\n");

				copy_to_user((void *) arg,
                           (const void *) &mod_ts,
                           sizeof(Module_and_ts_t));
				break;

		case IOCTL_SWITCH_MODULE://13
				D_DEBUG("ioctl IOCTL_SWITCH_MODULE\n");
				copy_from_user((void *) &module,
                           (const void *) arg,
                           sizeof(Module_t));
				if(module.module==MODULE_A)
				{
					write_i2c(DEST_MODULE,DEST_MOD_A);
				}
				else if(module.module==MODULE_B)
				{
					write_i2c(DEST_MODULE,DEST_MOD_B);

				}
				else if(module.module==NONE)
				{
					write_i2c(DEST_MODULE,0x00);// no dest
					D_DEBUG("------------>No dest!!!\n");
				}
				else if(module.module==MODULE_EXT)
					D_DEBUG("------------>Not implemented YET\n");
				else
					D_DEBUG("------------>BAD MODULE PARAMETER\n");
				break;
		case IOCTL_RST_ALL_MODE_MODULE://14
				D_DEBUG("ioctl IOCTL_RST_ALL_MODE_MODULE\n");
				copy_from_user((void *) &module,
                           (const void *) arg,
                           sizeof(Module_t));
				D_DEBUG("------------>module.module:%d\n",module.module);
				D_DEBUG("------------>module.configuration:%d\n",module.configuration);

//				write_i2c(STARCI2WIN_CTRL,0x80);
				write_i2c(STARCI2WIN_CTRL,0x00);
				write_i2c(DEST_MODULE,0x00);
				write_i2c(STARCI2WIN_CTRL,0x01);

				if(module.module==MODULE_A)
					write_i2c(MOD_A_CTRL,0x00);
				else if(module.module==MODULE_B)
					write_i2c(MOD_B_CTRL,0x00);
				else if(module.module==MODULE_EXT)
					D_DEBUG("------------>Not implemented YET\n");
				else
					D_DEBUG("------------>BAD MODULE PARAMETER\n");

				write_i2c(DEST_MODULE,0x00);//Module A
				write_i2c(MP_INTERFACE_CONFIG,0x00);
				write_i2c(POWER_CTRL,0x00);

				break;
		case IOCTL_ACTIVATION_MODULE_RST://15
				D_DEBUG("ioctl IOCTL_SET_MODULE_RST\n");
				copy_from_user((void *) &module,
                           (const void *) arg,
                           sizeof(Module_t));
				printk("IOCTL_ACTIVATION_MODULE_RST....\n");
				printk("module.module:%d\n",module.module);
				printk("module.configuration:%d\n",module.configuration);

				if(module.configuration==1)	/* Enter in RESET */
				{
					printk("Set in reset Module slot %d ... \n",module.module);

					/* Set the read thread in pause */
					if(cmd_read[0]==GO)
						cmd_read[0]=PAUSE;
					if(cmd_read[1]==GO)
						cmd_read[1]=PAUSE;
					cnt=0;
					while( (cmd_read[0]==PAUSE) || (cmd_read[1]==PAUSE) )
					{
						if(cnt>3000)	/* 3 seconds... */
							break;
						udelay(1000);	/* Wait 1ms	*/
						cnt++;
					}
					if(cnt>3000)
					{
						printk("Reset of module isn't possible beacuse the cam is 'IN_READ'\n");
						if(cmd_read[0]==PAUSE)
							cmd_read[0]=GO;
						if(cmd_read[1]==PAUSE)
							cmd_read[1]=GO;
						break;
					}

					if(module.module==MODULE_A)
						reg.offset_addr=MOD_A_CTRL;
					else if(module.module==MODULE_B)
						reg.offset_addr=MOD_B_CTRL;
					else
						printk("Wrong module %d\n",module.module);

					reg.value=0xFF;
					reg.value=read_i2c(reg.offset_addr);
					reg.value|=0x80;	/* HW RESET:reset pin up:-> Now cam is in reset */
					write_i2c(reg.offset_addr,reg.value);

					/* For poll routine*/
					ci_device[ci_id].isInit=0xFF;
					ci_device[ci_id].status=MODULE_EXTRACTED;
				}
				else if(module.configuration==0)	/* Exit in RESET */
				{
					printk("Exit from reset Module slot %d ... \n",module.module);

					if(module.module==MODULE_A)
						reg.offset_addr=MOD_A_CTRL;
					else if(module.module==MODULE_B)
						reg.offset_addr=MOD_B_CTRL;
					else
						printk("Wrong module %d\n",module.module);

					reg.value=0xFF;
					reg.value=read_i2c(reg.offset_addr);
					reg.value&=(~0x80);	/* HW RESET:reset pin down:-> Now cam starts */
					write_i2c(reg.offset_addr,reg.value);

					/* Set the detect and read thread in GO */
					/* For poll routine*/
					ci_device[ci_id].isInit=0xFF;

					/* Software detect of modules */
					/* Modcule A */
					r=read_i2c(MOD_A_CTRL);
					if((r&MODULE_PRESENT)!=MODULE_PRESENT)
					{
						D_DEBUG("manually detect: CAM 0 IS REMOVED\n");
						ci_device[0].status=MODULE_NOT_PRESENT;
					}
					else
					{
						D_DEBUG("manually detect: CAM 0 IS INSERTED\n");
						ci_device[0].status=MODULE_PRESENT;
					}
					/* Module B */
					r=0;
					r=read_i2c(MOD_B_CTRL);
					if((r&MODULE_PRESENT)!=MODULE_PRESENT)
					{
						D_DEBUG("manually detect: CAM 1 IS REMOVED\n");
						ci_device[1].status=MODULE_NOT_PRESENT;
					}
					else
					{
						D_DEBUG("manually detect: CAM 1 IS INSERTED\n");
						ci_device[1].status=MODULE_PRESENT;
					}
				}
				else
					printk("Wrong standby mode ... %d\n",module.configuration);
				break;

		case IOCTL_CI_MODE_FROM_BLACKBOX://16
                          {
                                Ci_Mode_BackBox params;
                                unsigned char tsin=0, tso=0, tsmap=0, ts1b=0, ts2b=0;

				D_DEBUG("ioctl IOCTL_CI_MODE_FROM_BLACKBOX\n");
				copy_from_user((void *) &params, (const void *) arg, sizeof(Ci_Mode_BackBox));
                                D_DEBUG("Mode: %s   TSin1: %s   TSin2: %s   Cam1: %s   Cam2: %s\n\n",
                                        params.mode==SINGLE_M?"Single":"Twin",
                                        params.tsin_1==NONE_S?"None":(params.tsin_1==TS1?"Ts1":"Ts3"),
                                        params.tsin_2==NONE_S?"None":(params.tsin_2==TS2?"Ts2":"Ts3"),
                                        params.cam_1==NONE_S?"None":(params.cam_1==CAM_A?"A":(params.cam_1==CAM_B?"B":"A+B")),
                                        params.cam_2==NONE_S?"None":(params.cam_2==CAM_A?"A":(params.cam_2==CAM_B?"B":"A+B"))
                                        );

                                if ((params.cam_1&CAM_A) || (params.cam_2&CAM_A))
                                {
                                    write_i2c(MOD_A_CTRL, read_i2c(MOD_A_CTRL) | TSOEN | TSIEN);    //Streaming through Cam A
                                }
                                else write_i2c(MOD_A_CTRL, read_i2c(MOD_A_CTRL) & ~(TSOEN | TSIEN));//NO Streaming through Cam A
                                if ((params.cam_1&CAM_B) || (params.cam_2&CAM_B))
                                {
                                    write_i2c(MOD_B_CTRL, read_i2c(MOD_B_CTRL) | TSOEN | TSIEN);    //Streaming through Cam B
                                }
                                else write_i2c(MOD_B_CTRL, read_i2c(MOD_B_CTRL) & ~(TSOEN | TSIEN));//NO Streaming through Cam B

                                if (params.mode == SINGLE_M)                                     //Single mode
                                {
                                    if      (params.tsin_1==TS1) {tsin = 0; tso = 0;}
                                    else if (params.tsin_1==TS3) {tsin = 2; tso = 0;}
                                    else if (params.tsin_2==TS2) {tsin = 1; tso = TSO;}                 //Invert 2 TS outputs
                                    else if (params.tsin_2==TS3) {tsin = 2; tso = TSO;}                 //Invert 2 TS outputs
                                    write_i2c(SINGLE_MODE_TS, 0 | TSI(tsin) | tso);                     //Select streaming source
                                    write_i2c(TWIN_MODE_TS, 0 );                                        //Set single mode
                                }
                                else                                                             //Twin mode
                                {
                                    if (params.cam_1==CAM_AB || params.cam_2==CAM_AB)                   //One stream shall bypass cams
                                    {
                                        if      (params.cam_1==CAM_AB)  ts2b = TS2B;
                                        else if (params.cam_2==CAM_AB)  ts1b = TS1B;
                                    }
                                    else                                                                //Both streams pass through cams
                                    {
                                        if      (params.cam_1==CAM_A || params.cam_2==CAM_B) tsmap = 0;
                                        else if (params.cam_1==CAM_B || params.cam_2==CAM_A) tsmap = TSMAP;     //Invert 2 Cams
                                    }
                                    if      (params.tsin_1==TS1 && params.tsin_2==TS2) tsin = 0;
                                    else if (params.tsin_1==TS2 && params.tsin_2==TS1) tsin = 1;
                                    else if (params.tsin_1==TS1 && params.tsin_2==TS3) tsin = 2;
                                    else if (params.tsin_1==TS3 && params.tsin_2==TS2) tsin = 3;
                                    write_i2c(TWIN_MODE_TS, 0 | tsmap | ts1b | ts2b | MTSI(tsin) | TWIN);   //Set twin mode
                                }

				break;
                          }

		case IOCTL_READ_I2C://100
				D_DEBUG("ioctl IOCTL_READ_I2C\n");
				copy_from_user((void *) &reg,
                           (const void *) arg,
                           sizeof(Register_t));
				reg.value=read_i2c(reg.offset_addr);
				copy_to_user((void *) arg,
                           (const void *) &reg,
                           sizeof(Register_t));
				break;

		case IOCTL_WRITE_I2C://101
				D_DEBUG("ioctl IOCTL_WRITE_I2C\n");
				copy_from_user((void *) &reg,
                           (const void *) arg,
                           sizeof(Register_t));
				write_i2c(reg.offset_addr,reg.value);

				break;

		default:
			break;
	}

	return res;
}

static unsigned int starci_poll(struct file *file, poll_table *wait)
{
	int minor;
	unsigned int mask = 0;

	minor = MINOR(file->f_dentry->d_inode->i_rdev);

	if( (ci_device[minor].status==0xFF) && (ci_device[minor].isInit==0xFF) )
	{
		return mask;
	}

	if(ci_device[minor].status==MODULE_NOT_PRESENT)
		return mask;


	if( (ci_info[0].in_write==1) ||
		(ci_info[1].in_write==1) ||
		(read_op.cams[0].status==IN_READ) ||
		(read_op.cams[1].status==IN_READ) )
	{

		return mask;
	}

	if(ci_device[minor].status==MODULE_EXTRACTED)
	{
		printk("%d MODULE_EXTRACTED\n",minor);
		mask |= POLLOUT | POLLWRNORM | POLLIN | POLLRDNORM;
		ci_device[minor].status=MODULE_NOT_PRESENT;
		return mask ;
 	}

	mask|= POLLOUT | POLLWRNORM;

	if( (read_op.cams[minor].status==DATA_AVAILABLE) ||
		( (ci_device[minor].status==MODULE_PRESENT) && (ci_device[minor].isInit==0xFF) ) ||
		(ci_device[minor].isInit==0xEE) )
		{

			if(ci_device[minor].isInit==0xEE)
				ci_device[minor].isInit=0xFF;
			mask|= POLLIN | POLLRDNORM;
		}

	return mask;
}

static struct file_operations starci_fops = {
	.owner		=	THIS_MODULE,
	.open		=	starci_open,
	.release	=	starci_release,
	.read		=	starci_read,
	.ioctl		=	starci_ioctl,
	.poll		=	starci_poll,
	.write		=	starci_write
};


struct i2c_driver starci2win_driver =
{
        .id             = I2C_DRIVERID_STARCI2WIN,
        .driver         = { .name   = "StarCI2win driver",
                            .owner  = THIS_MODULE },
        .attach_adapter = attach_adapter,
        .detach_client  = detach_client,
        .command        = starci2win_command,
};

/******************************************************************************
 *          DUMMY FUNCTIONS
 ******************************************************************************/
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

/******************************************************************************/

static int __init starci2win_init(void)
{
 	int rc=0, ret=0;
	unsigned char r=0;

    dev_t dev = MKDEV(STARCI_MAJOR_NUM, STARCI_MINOR_START);

    cdev_init(&starci_cdev, &starci_fops);
    starci_cdev.owner = THIS_MODULE;
    starci_cdev.ops = &starci_fops;
    if (cdev_add(&starci_cdev, dev, CI_NUMBER_OF_CONTROLLERS) < 0)
    {
        D_ERROR("Couldn't register '%s' driver\n", STARCI_DEVICE_NAME);
        return -1;
    }
    D_DEBUG("Registering device '%s', major '%d'\n", STARCI_DEVICE_NAME, STARCI_MAJOR_NUM);

	printk("Version of starci2win driver: 0.0.13\n");

	ret=i2c_add_driver(&starci2win_driver);

	/* Reset the starci2win chip */
	write_i2c(0x1F,0x80);
	mdelay(1);
	write_i2c(0x1F,0x00);

	/* Init of wait queue */
	init_waitqueue_head(&wq);

	/* Create the threads for detect and read from the modules */
	cmd_detect=GO;
	cmd_read[0]=cmd_read[1]=NO_CMD;
	read_op.cams[0].status=FREE;
	read_op.cams[1].status=FREE;

	kth[0].kth=kth[1].kth=NULL;
	kth[0].has_sem=kth[1].has_sem=NO_SEM;
	kth[0].slot_th=0;
	kth[1].slot_th=1;

	kthread_run(detect_thread,NULL,"Detect_thread");

	/* Set the irq line of the starci2win */
	write_i2c(0x1C,0x06);
	write_i2c(0x1B,0x03);
	write_i2c(0x1F,0x01);

    if (device_init() != STARCI_NO_ERROR)
    {
        D_ERROR("Couldn't init device\n");
        return -1;
    }

	/*Set the TSO_0 -> TSI_0 and the  TSO_1 -> TSI_1*/
	write_i2c(SINGLE_MODE_TS,0x00);
	write_i2c(TWIN_MODE_TS,0x80);

	/* For the interrupt */
	rc=request_irq(2, &try_irq, IRQF_DISABLED, "starci_int", NULL);	/* In ref board it was the eth interrupt */
	if(rc==-1)	printk("Request irq error!!!\n");

	disable_irq(2);
	irq_detect_handler();
	enable_irq(2);

	/* i2c command */
	return ret;
}

static void __exit starci2win_exit(void)
{
    device_uninit();

    cdev_del(&starci_cdev);

	/* Exit from detect thread and read thread */
	cmd_detect=KILL;
	cmd_read[0]=cmd_read[1]=KILL;
	mdelay(500);

	/* i2c command */
	i2c_del_driver(&starci2win_driver);

    D_DEBUG("Unregistering device '%s'\n", STARCI_DEVICE_NAME);
}

module_init(starci2win_init);
module_exit(starci2win_exit);

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off debugging (default:off).");

MODULE_DESCRIPTION("I2C driver for the StarCI2win");
MODULE_AUTHOR("devel-1");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.13");

