/*************************************************************************
 *
 * ufs910_fp.c
 *
 * 11. Nov 2007 - captaintrip
 *
 * Kathrein UFS910 VFD Kernel module
 * portiert aus den MARUSYS uboot sourcen
 *
 * The front panel display consist of a VFD tube Futaba 16-BT-136INK which
 * has a 1x16 character VFD display with 16 icons and a built-in driver
 * IC that seems to be highly compatible with the OKI ML9208.
 * The built-in display controller does not provide key scanning
 * facilities.
 *
 * The display controller shares its interface (3 PIO pins) with the
 * frontpanel controller, which uses dummy commands not recognized by
 * the display controller for its control.
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
 *************************************************************************
 *
 * Frontpanel driver for Kathrein UFS910
 *
 *************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * -----------------------------------------------------------------------
 * 20210707 Audioniek       Add standard dprintk debugging display.
 *
 *************************************************************************
 */

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>

#include "ufs910_fp.h"
#include "utf.h"

short paramDebug = 0;

/* konfetti: bugfix*/
static struct vfd_ioctl_data lastdata;

/* konfetti: quick and dirty open handling */
typedef struct
{
	struct file      *fp;
	int              read;
	struct semaphore sem;
} tVFDOpen;

static tVFDOpen vOpen;

unsigned char str[64];

/***********************************************************
 *
 * Routines to communicate with the display controller.
 *
 */
void out_buf(unsigned char *buf, int len)
{
	if (len < 64)
	{
		strncpy(str, buf, len);
		str[len] = '\0';
		dprintk(20, "%s\n", str);
	}
	else
	{
		dprintk(1, "overflow\n");
	}
}

/***********************************************************
 *
 * Wait for port ready.
 *
 */
static void Wait_Port_Ready(void)
{
	dprintk(100, "%s >\n", __func__);
	while ((SCP_STATUS & 0x02)  == 0x02)
	{
		dprintk(70, "Wait_Port_Ready::SCP_STATUS = %x\n", SCP_STATUS);
		udelay(1);
	}
	dprintk(100, "%s <\n", __func__);
}

/***********************************************************
 *
 * Write length bytes to display controller.
 *
 */
static int VFD_Write_Char_ML9208(unsigned char data, int current__, int length)
{
	int ii = 0;

	dprintk(100, "%s >\n", __func__);

	if (current__ == 0)  // start position
	{
		STPIO_SET_PIN(PIO_PORT(0), 5, 1);  // toggle
		udelay(10);
		STPIO_SET_PIN(PIO_PORT(0), 5, 0);  // xCS
		udelay(10);
	}

	if (SCP_PORT)
	{
		Wait_Port_Ready();
		SCP_TXD_DATA = data;
		SCP_TXD_START = 0x01;  // start transmit
	}
	else
	{
		for (ii = 0 ; ii < 8 ; ii++)
		{
			STPIO_SET_PIN(PIO_PORT(0), 4, 0);  // set clock pin low
			if ((data  & 0x01) == 0x01)  // get one bit
			{
				STPIO_SET_PIN(PIO_PORT(0), 3, 1);  // if one, raise data pin
			}
			else
			{
				STPIO_SET_PIN(PIO_PORT(0), 3, 0);  // // else drop data pin
			}
			udelay(10);  // wait 10 us
			data = data >> 1;
			STPIO_SET_PIN(PIO_PORT(0), 4, 1);  // set clock pin high
			udelay(10);
		}
	}
	if (SCP_PORT)
	{
		Wait_Port_Ready();
	}
	udelay(20);  // next byte, wait 20 us
	if (current__ == (length - 1))
	{
		udelay(40);
		STPIO_SET_PIN(PIO_PORT(0), 5, 1);  // raise xCS
		udelay(50);
	}
	dprintk(100, "%s <\n", __func__);
	return 0;
}

/***********************************************************
 *
 * Write length bytes in data to the display controller.
 *
 */
static int VFD_Write_Chars(unsigned char *data, int length)
{
	int i = 0;

	dprintk(100, "%s >\n", __func__);

	out_buf(data, length);

	for (i = 0; i < length; i++)
	{
		VFD_Write_Char_ML9208(data[i], i, length);
	}
	dprintk(100, "%s <\n", __func__);
	return 0;
}


/***********************************************************
 *
 * Write a text to the display.
 *
 */
//#define DCRAM_INVERT
static int VFD_DCRAM_Write(struct vfd_ioctl_data *data)
{
	unsigned char write_data[0x31];
	int i = 0;
	int j = 0;

	dprintk(100, "%s >\n", __func__);
	memset(&write_data[1], ' ', 0x30);

#ifdef DCRAM_INVERT
	write_data[0] = (0x10 - data->length - (data->start_address & 0x0f)) | DCRAM_COMMAND;
#else
	write_data[0] = (data->start_address & 0x0f) | DCRAM_COMMAND;
#endif

	while ((i < data->length) && (j < 16))
	{
		udelay(1);
		j++;
#ifdef DCRAM_INVERT
		write_data[data->length - i] = ROM_Char_Table[data->data[i]];
#else
		if (data->data[i] < 0x80)
		{
			write_data[j] = ROM_Char_Table[data->data[i]];
		}
		else if (data->data[i] < 0xE0)
		{
			switch (data->data[i])
			{
				case 0xc2:
				{
					UTF_Char_Table = UTF_C2;
					break;
				}
				case 0xc3:
				{
					UTF_Char_Table = UTF_C3;
					break;
				}
				case 0xc4:
				{
					UTF_Char_Table = UTF_C4;
					break;
				}
				case 0xc5:
				{
					UTF_Char_Table = UTF_C5;
					break;
				}
				case 0xd0:
				{
					UTF_Char_Table = UTF_D0;
					break;
				}
				case 0xd1:
				{
					UTF_Char_Table = UTF_D1;
					break;
				}
				default:
				{
					UTF_Char_Table = NULL;
				}
			}
			i++;
			if (UTF_Char_Table)
			{
				write_data[j] = UTF_Char_Table[data->data[i] & 0x3f];
			}
			else
			{
				sprintf(&write_data[j], "%02x", data->data[i - 1]);
				j += 2;
				write_data[j] = (data->data[i] & 0x3f) | 0x40;
			}
		}
		else
		{
			if (data->data[i] < 0xF0)
			{
				i += 2;
			}
			else if (data->data[i] < 0xF8)
			{
				i += 3;
			}
			else if (data->data[i] < 0xFC)
			{
				i += 4;
			}
			else
			{
				i += 5;
			}
			write_data[j] = 0x20;
		}
#endif
		i++;
	}
	/* konfetti: bugfix*/
	lastdata = *data;
	VFD_Write_Chars(write_data, 17);

	dprintk(100, "%s <\n", __func__);
	return 0;
}

/***********************************************************
 *
 * Define a CGRAM dot pattern.
 *
 */
static int VFD_CGRAM_Write(struct vfd_ioctl_data *data)
{
	unsigned char write_data[6];
	int i = 0;

	dprintk(100, "%s >\n", __func__);

	write_data[0] = (data->start_address & 0x07) | CGRAM_COMMAND;
	if (data->length < 5)
	{
		dprintk(1, "%s < Error: too litte data supplied\n", __func__);
		return -1;
	}

	if (data->length > 5)
	{
		dprintk(1, "%s < Error: Too much data supplied\n");
		return -1;
	}

	for (i = 0 ; i < data->length; i++)
	{
		write_data[i + 1] = data->data[i];
	}
	VFD_Write_Chars(write_data, data->length + 1);

	dprintk(100, "%s <\n", __func__);
	return 0;
}

/***********************************************************
 *
 * Set RC code.
 *
 */
static int VFD_SetRemote(void)
{
	unsigned char write_data[2];

	write_data[0] = 0xe3;  // front controller command
	write_data[1] = 0x0f;  // get RC code data
	VFD_Write_Chars(write_data, 2);
	return 0;
}

/***********************************************************
 *
 * (Re)Set an icon (write ADRAM).
 *
 */
static int VFD_ADRAM_Write(struct vfd_ioctl_data *data)
{
	unsigned char write_data[16];

	write_data[0] = (data->data[0] & 0x0f) | ADRAM_COMMAND;
	write_data[1] = data->data[4];
	//printk("ICON ON/OFF Data = %x, %x\n", write_data[0], write_data[1]);
	dprintk(100, "%s <>\n", __func__);
	VFD_Write_Chars(write_data, 2);
	return 0;
}

/***********************************************************
 *
 * Set brightness.
 *
 */
static int VFD_Display_DutyCycle_Write(struct vfd_ioctl_data *data)
{
	unsigned char write_data[17];
	int ii = 0;

	dprintk(100, "%s >\n", __func__);

	write_data[0] = DIMMING_COMMAND;
	write_data[1] = (data->start_address & 0x07) << 5 | 0xf;  // only use 8 steps, add 15
	data->length = 1;
	VFD_Write_Chars(write_data, data->length + 1);

	write_data[0] = GRAY_LEVEL_DATA_COMMAND;
	write_data[1] = 0xff;
	write_data[2] = 0xff;
	VFD_Write_Chars(write_data, 3);

	write_data[0] = GRAY_LEVEL_ON_COMMAND;
	for (ii = 0; ii < 16  ; ii++)
	{
		write_data[ii + 1] = 0x05;
	}
	VFD_Write_Chars(write_data, 17);
	dprintk(100, "%s <\n", __func__);
	return 0;
}

/***********************************************************
 *
 * Set number of digits to 16.
 *
 */
static int VFD_Number_Of_Digit_Set(struct vfd_ioctl_data *data)
{
	unsigned char write_data[2];

	dprintk(100, "%s >\n", __func__);

	write_data[0] = NUM_DIGIT_COMMAND;
	write_data[1] = 0x0f;
	VFD_Write_Chars(write_data, 2);

	dprintk(100, "%s <\n", __func__);
	return 0;
}

/***********************************************************
 *
 * Switch entire display on or off.
 *
 */
//2 Turn ON : Turn on all vfd light and standby mode.
//2 Turn OFF : Turn off all vfd ligth and go to standby mod.
static int VFD_Display_Write_On_Off(struct vfd_ioctl_data *data)
{
	unsigned char write_data[1];

	dprintk(100, "%s >\n", __func__);
	if (data->start_address == 0x01)
	{
		data->start_address = 0x00;  // light normal
	}
	else
	{
		data->start_address = 0x02;  // light off
	}
	write_data[0] = (data->start_address & 0x03) | LIGHT_ON_COMMAND;
	if (data->length != 0)
	{
		dprintk(1, "%s < return on error <\n", __func__);
		return -1;
	}
	VFD_Write_Chars(write_data, data->length + 1);

	// now standby mode setting
	dprintk(100, "%s <\n", __func__);
	return 0;
}

#if 0
static int VFD_TEST_Write(struct vfd_ioctl_data *data)
{
	unsigned char write_data[5];
	int i;

	dprintk(100, "%s >\n", __func__);
	write_data[0] = data->start_address;
	for (i = 0 ; i < data->length; i++)
	{
		write_data[i + 1] = data->data[i];
	}
	VFD_Write_Chars(write_data, data->length + 1);
	dprintk(100, "%s <\n", __func__);
	return 0;
}
#endif

/***********************************************************
 *
 * (Re)Set an icon.
 *
 */
static int VFD_Icon_Display_On_Off(struct vfd_ioctl_data *data)
{
	dprintk(100, "%s >\n", __func__);

	VFD_ADRAM_Write(data);
	dprintk(100, "%s <\n", __func__);
	return 0;
}

/***********************************************************
 *
 * Routines for the IOCTL functions of the driver.
 *
 */

/***********************************************************
 *
 * Display a text string.
 *
 */
void DisplayVFDString(unsigned char *aBuf, int len)
{
	struct vfd_ioctl_data data;

	dprintk(100, "%s >\n", __func__);
	if (len > 63 || len < 0)
	{
		dprintk(1, "%s < VFD String Length %d is too large\n", __func__, len);
		return;
	}
	data.start_address = 0x00;
	memset(data.data, ' ', 63);
	memcpy(data.data, aBuf, len);
	data.length = len;

	VFD_DCRAM_Write(&data);
	dprintk(100, "%s <\n", __func__);
}

/***********************************************************
 *
 * Initialize the driver.
 *
 */
int vfd_init_func(void)
{
	struct vfd_ioctl_data data;

	dprintk(100, "%s >\n", __func__);
	dprintk(0, "Kathrein UFS910 VFD module initializing\n");
	SCP_PORT = 0;  // use serial controller port
	ROM_Char_Table = ROM_KATHREIN;

	// display write on
	data.start_address = 0x01;  // means light normal
	data.length = 0;
	VFD_Display_Write_On_Off(&data);

	// digit-set ->Dagobert ->Origsoft does this after VFD_Display_Write_On_Off
	data.start_address = NUM_DIGIT_COMMAND;
	data.length = 0;
	VFD_Number_Of_Digit_Set(&data);

	// set full brightness
	data.start_address = 0x07;
	data.length = 0;
	VFD_Display_DutyCycle_Write(&data);

	// clear display
	DisplayVFDString("", 0);

	// VFD_SetRemote();
	dprintk(100, "%s <\n", __func__);
	return 0;
}

/***********************************************************
 *
 * Code for writing to /dev/vfd.
 *
 */
static ssize_t VFDdev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	unsigned char *kernel_buf = kmalloc(len, GFP_KERNEL);

	dprintk(100, "%s > (len %d, offs %lld)\n", __func__, len, *off);
	/* konfetti */
	if (kernel_buf == NULL)
	{
		dprintk(1, "%s < return -ENOMEM\n", __func__);
		return -ENOMEM;
	}
	copy_from_user(kernel_buf, buff, len);

	/* Dagobert: echo add a \n which will be counted as a char
	 */
	if (kernel_buf[len - 1] == '\n')
	{
		DisplayVFDString(kernel_buf, len - 1);
	}
	else
	{
		DisplayVFDString(kernel_buf, len);
	}
	kfree(kernel_buf);
	dprintk(100, "%s <\n", __func__);
	return len;
}

/***********************************************************
 *
 * Code for reading from to /dev/vfd.
 *
 */
static ssize_t VFDdev_read(struct file *filp, char __user *buff, size_t len, loff_t *off)
{
	/* ignore offset or reading of fragments */

	dprintk(100, "%s > (len %d, offs %lld)\n", __func__, len, *off);
	if (vOpen.fp != filp)
	{
		dprintk(1, "%s < return -EUSERS\n", __func__);
		return -EUSERS;
	}
	if (down_interruptible(&vOpen.sem))
	{
		dprintk(1, "%s < return -ERESTARTSYS\n", __func__);
		return -ERESTARTSYS;
	}
	if (vOpen.read == lastdata.length)
	{
		vOpen.read = 0;

		up(&vOpen.sem);
		dprintk(1, "%s < return 0\n", __func__);
		return 0;
	}
	if (len > lastdata.length)
	{
		len = lastdata.length;
	}
	if (len > 16)
	{
		len = 16;
	}
	vOpen.read = len;
	copy_to_user(buff, lastdata.data, len);

	up(&vOpen.sem);

	dprintk(100, "%s < (len %d)\n", __func__, len);
	return len;
}

/***********************************************************
 *
 * Open /dev/vfd.
 *
 */
int VFDdev_open(struct inode *inode, struct file *filp)
{
	dprintk(100, "%s >\n", __func__);

	if (vOpen.fp != NULL)
	{
		dprintk(1, "%s < return -EUSERS\n", __func__);
		return -EUSERS;
	}
	vOpen.fp = filp;
	vOpen.read = 0;

	dprintk(100, "%s <\n", __func__);
	return 0;
}

/***********************************************************
 *
 * Close /dev/vfd.
 *
 */
int VFDdev_close(struct inode *inode, struct file *filp)
{
	dprintk(100, "%s >\n", __func__);

	if (vOpen.fp == filp)
	{
		vOpen.fp = NULL;
		vOpen.read = 0;
	}
	dprintk(100, "%s <\n", __func__);
	return 0;
}
/* ende konfetti */

/***********************************************************
 *
 * IOCTL code.
 *
 */
static int VFDdev_ioctl(struct inode *Inode, struct file *File, unsigned int cmd, unsigned long arg)
{
	dprintk(10, "%s > 0x%.8x\n", __func__, cmd);

	switch (cmd)
	{
		case VFDDCRAMWRITE:
		{
			VFD_DCRAM_Write((struct vfd_ioctl_data *)arg);
			break;
		}
		case VFDBRIGHTNESS:
		{
			VFD_Display_DutyCycle_Write((struct vfd_ioctl_data *)arg);
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			VFD_Display_Write_On_Off((struct vfd_ioctl_data *)arg);
			break;
		}
		case VFDDRIVERINIT:
		{
			vfd_init_func();
			break;
		}
		case VFDICONDISPLAYONOFF:
		{
			VFD_Icon_Display_On_Off((struct vfd_ioctl_data *)arg);
			break;
		}
		case VFDCGRAMWRITE:
		{
			VFD_CGRAM_Write((struct vfd_ioctl_data *)arg);
			break;
		}
		case VFDCGRAMWRITE2:
		{
			VFD_CGRAM_Write((struct vfd_ioctl_data *)arg);
			break;
		}
		case 0x5305:
		{
			break;
		}
		default:
		{
			dprintk(1, "%s: unknown IOCTL 0x%x\n", __func__, cmd);
			break;
		}
	}
	dprintk(100, "%s <\n", __func__);
	return 0;
}

static struct file_operations vfd_fops =
{
	.owner    = THIS_MODULE,
	.ioctl    = VFDdev_ioctl,
	.write    = VFDdev_write,
	.read     = VFDdev_read,
	.open     = VFDdev_open,  /* konfetti */
	.release  = VFDdev_close
};

static int __init vfd_init_module(void)
{
	dprintk(100, "%s >\n", __func__);

	if (register_chrdev(VFD_MAJOR, "VFD", &vfd_fops))
	{
		dprintk(1, "%s: Unable to get major %d for VFD\n", __func__, VFD_MAJOR);
	}
	else
	{
		vfd_init_func();
	}
	/* konfetti */
	vOpen.fp = NULL;
	sema_init(&vOpen.sem, 1);

	dprintk(100, "%s <\n", __func__);
	return 0;
}

static void __exit vfd_cleanup_module(void)
{
	unregister_chrdev(VFD_MAJOR, "VFD");
	dprintk(0, "Kathrein UFS910 VFD module unloading\n");
}

module_init(vfd_init_module);
module_exit(vfd_cleanup_module);

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

MODULE_DESCRIPTION("VFD module for Kathrein UFS910 (Q'n'D port from MARUSYS)");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
