/****************************************************************************************
 *
 * pace_7241_fp.c
 *
 * (c) 20?? j00zek
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
 * Front panel driver for Pace HDS-7241
 *
 * Devices:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *
 ****************************************************************************************
 * The Pace HDS-7241 front panel consists of:
 * - One bicolour LED (red/blue), GPIO driven;
 * - Five keys, GPIO connected;
 * - Intelligent VFD display, 16 characters, 5x7 dot matrix, control pins GPIO connected.
 * 
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 20?????? j00zek          Initial version       
 *
 ****************************************************************************************/
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <linux/delay.h>

#include <linux/workqueue.h>
#include <linux/input.h>

#include <linux/gpio.h>
#include <linux/stm/gpio.h>
#include <linux/stm/pio.h>

#include "../vfd/utf.h"

#define LED1    stm_gpio(4, 0)
#define LED2    stm_gpio(4, 1)

#define KEYPp	stm_gpio(5, 4)
#define KEYPm   stm_gpio(5, 5)

#if 0
#define KEY_R   stm_gpio(5, 6)
#define KEY_L   stm_gpio(5, 7)
#define KEY_PWR stm_gpio(11, 6)
#endif

static char *button_driver_name = "Pace HDS-7241 frontpanel buttons";
static struct input_dev *button_dev;
static struct workqueue_struct *wq;
static int bad_polling = 1;

#define VFD_MAJOR 147

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
static short paramDebug = 0;

#define DBG(fmt, args...) if ( paramDebug > 0 ) printk(KERN_INFO "[VFD] :: " fmt "\n", ## args )
#define ERR(fmt, args...) printk(KERN_ERR "[vfd] :: " fmt "\n", ## args )

/***************************************************
 *
 * Key routines
 *
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
void button_bad_polling(void)
#else
void button_bad_polling(struct work_struct *ignored)
#endif
{
	unsigned int key_front;

	DBG("[%s] >>>\n", __func__);
	while (bad_polling == 1)
	{
		msleep(100);
//		DBG("-%d\n",gpio_get_value(KEY_LEFT));
		key_front = 0;
#if 0
		if (gpio_get_value(KEY_L) == 0)
		{
			key_front = KEY_LEFT;
		}
#endif
		if (gpio_get_value(KEYPm) == 0)
		{
			key_front = KEY_LEFT;
		}
#if 0
		if (gpio_get_value(KEY_PWR) == 0)
		{
			key_front=KEY_POWER;
		}
#endif
#if 0
		// if (gpio_get_value(KEY_R) == 0)
		{
			key_front = KEY_RIGHT;
		}
#endif
		if (gpio_get_value(KEYPp) == 0)
		{
			key_front = KEY_RIGHT;
		}
		if (key_front > 0)
		{
			input_report_key(button_dev, key_front, 1);
			input_sync(button_dev);
			DBG("Key: %d (0x%02x)\n", key_front, key_front);
			msleep(250);
			input_report_key(button_dev, key_front, 0);
			input_sync(button_dev);
		}

	}  //while
	DBG("[BTN] button_bad_polling END\n");
	bad_polling = 2;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static DECLARE_WORK(button_obj, button_bad_polling, NULL);
#else
static DECLARE_WORK(button_obj, button_bad_polling);
#endif

static int button_input_open(struct input_dev *dev)
{
	DBG("[%s] >>>\n", __func__);
	wq = create_workqueue("button");
	if (queue_work(wq, &button_obj))
	{
		DBG("[BTN] queue_work successful ...\n");
	}
	else
	{
		DBG("[BTN] queue_work not successful, exiting ...\n");
		return 1;
	}
	return 0;
}

static void button_input_close(struct input_dev *dev)
{
	DBG("[BTN] button_input_close\n");
	bad_polling = 0;
	while (bad_polling != 2)
	{
		msleep(1);
	}
	bad_polling = 1;

	if (wq)
	{
		DBG("[BTN] workqueue destroy start\n");
		destroy_workqueue(wq);
		DBG("[BTN] workqueue destroyed\n");
	}
}

/***************************************************
 *
 * Display routines
 *
 */

/* VFD code based on vacfludis - vacuum fluorescent display by qrt@qland.de
 * it is using the same protocol to HCS-12SS59T with one difference. 
 * Characters are stored in ASCII, no need to convert them.
*/

#define VFD_RST   stm_gpio(11,5)  // reset by 0
#if 0
#define VFD_VDON  stm_gpio(4, 7)  // Vdisp 0=ON, 1=OFF
#endif
#define VFD_CLK   stm_gpio(3, 4)  // ShiftClockInput' - the serial data is shifted at the rising edge of CLK
#define VFD_DAT   stm_gpio(3, 5)  // serial data
#define VFD_CS    stm_gpio(5, 3)  // enable transfer by 0

#define VFD_DCRAM_WR 0x10  // ccccaaaa dddddddd dddddddd .. -> direct dot matrix control?
#define VFD_CGRAM_WR 0x20  // "                             -> internal character generator
#define VFD_ADRAM_WR 0x30  // ccccaaaa ******dd ******dd .. -> cursor control
#define VFD_DUTY     0x50  // set brightness (value in lower nibble)
#define VFD_NUMDIGIT 0x60  // set number of digits (value in lower nibble, 0 -> 16 digits)
#define VFD_LIGHTS   0x70  // set lights, (value in lower nibble)
// values for VFD_LIGHTS command
#define LINORM       0x00  // normal display
#define LIOFF        0x01  // all segments OFF
#define LION         0x02  // all segments ON

#define VFD_DELAY    8 // us
#define VFD_DEBUG    0

static void vfdSendByte(uint8_t port_digit)
{
	unsigned char i;
	unsigned char digit;
	digit = port_digit;
	
	//DBG("%d = ", digit);
	for (i = 0; i < 8; i++)
	{
/*		if (digit & 1) 
		{
			DBG("1");
		}
		else
		{
			DBG("0");
		}
*/
		gpio_set_value(VFD_DAT, digit & 0x01);
		udelay(1);
		gpio_set_value(VFD_CLK, 1);  // toggle
		udelay(1);
		gpio_set_value(VFD_CLK, 0);  // clk
		udelay(2);
		gpio_set_value(VFD_CLK, 1);  // pin
		udelay(2);
		digit >>= 1;  // get next bit
	}
	DBG("\n");
	udelay(VFD_DELAY);
}

static void vfdSelect(unsigned char s)
{
	if (s)
	{
		// enable transfer mode
		gpio_set_value(VFD_CS, 1);
		udelay(VFD_DELAY * 2);
		gpio_set_value(VFD_CS, 0);
		udelay(VFD_DELAY * 2);
	}
	else
	{
		// disable transfer mode
		udelay(VFD_DELAY * 2);
		gpio_set_value(VFD_CS, 1);
	}
}

static void vfdSendCMD(uint8_t vfdcmd, uint8_t vfdarg)
{
	DBG("vfdcmd=%d, vfdarg=%d >> (vfdcmd | vfdarg) = %d\n",vfdcmd, vfdarg, (vfdcmd | vfdarg));

	// transferring data
	vfdSelect(1);  // assert CS
	vfdSendByte(vfdcmd | vfdarg);
	vfdSelect(0);  // deassert CS
}

int VFD_WriteFront(unsigned char* data, unsigned char len )
{
	unsigned char lcd_buf[16] = { 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 };
	int i = 0;
	int j = 0;

	while ((i < len) && (j < 16))
	{
		if (data[i] == '\n' || data[i] == 0)
		{
			DBG("[%s] BREAK CHAR detected (0x%X)\n", __func__, data[i]);
			break;
		}
		else if (data[i] < 0x20)
		{
			DBG("[%s] NON_PRINTABLE_CHAR '0x%X'\n", __func__, data[i]);
			i++;
		}
		else if (data[i] < 0x80)
		{
			lcd_buf[j] = data[i];
			DBG("[%s] data[%i] = '0x%X'\n", __func__, j, data[i]);
			j++;
		}
		else if (data[i] < 0xE0)
		{
			DBG("[%s] UTF_Char_Table= 0x%X", __func__, data[i]);
			switch (data[i])
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
				DBG("[%s] UTF_Char= 0x%X, index=%i", __func__, UTF_Char_Table[data[i] & 0x3f], i);
				lcd_buf[j] = UTF_Char_Table[data[i] & 0x3f];
				j++;
			}
		}
		else
		{
			if (data[i] < 0xF0)
			{
				i += 2;
			}
			else if (data[i] < 0xF8)
			{
				i += 3;
			}
			else if (data[i] < 0xFC)
			{
				i += 4;
			}
			else
			{
				i += 5;
			}
		}
		i++;
	}
	// Fill remaining buffer with spaces
	if (j < 16)
	{
		for (i = j; i < 16; i++)
		{
			lcd_buf[i] = 0x20;
		}
	}
	// enable transfer mode
	vfdSelect(1);
	// transfer data
	vfdSendByte(VFD_DCRAM_WR | 0);  // set first char (display has auto address increment)
	for(i = 0; i < 16; i++)
	{
		//DBG("[%s] CHAR=%d >> ",__func__, lcd_buf[i]);
		vfdSendByte(lcd_buf[i]);
	}
	// disable transfer mode
	vfdSelect(0);
	return 0;
}

/***************************************************
 *
 * LED routines
 *
 */
static void VFD_setIcon(unsigned char *kbuf, unsigned char len)
{
	spin_lock(&mr_lock);

	switch (kbuf[0])
	{
		case 1:  // red
		{
			if (kbuf[4] == 1)
			{
				gpio_set_value(LED1, 0);
			}
			else
			{
				gpio_set_value(LED1, 1);
			}
			break;
		}
		case 2:  // blue
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
			ERR("icon unknown %d", kbuf[0]);
			break;
		}
	}
	spin_unlock(&mr_lock);
}


/***************************************************
 *
 * Driver init routine
 *
 */
static void VFD_init(void)
{
	DBG("[%s] >>>\n", __func__);
	//reset VFD
#if 0
	gpio_set_value(VFD_VDON, 0);
	udelay(2000);
#endif
	gpio_set_value(VFD_RST, 1);  // toggle
	udelay(2000);
	gpio_set_value(VFD_RST, 0);  // rst
	udelay(2000);
	gpio_set_value(VFD_RST, 1);  // pin
	udelay(2000);
	vfdSendCMD(VFD_NUMDIGIT, 15); // ? should be zero for 16 digits?
	vfdSendCMD(VFD_DUTY, 15);  // maximum brightness
	vfdSendCMD(VFD_LIGHTS, LINORM); // normal display 
	VFD_WriteFront("", 0);  // clear display
}

/***************************************************
 *
 * Set brightness
 *
 */
static void VFD_setBrightness(int level)
{
	DBG("[%s] >>>\n", __func__);
	if (level < 0)
	{
		vfdSendCMD(VFD_DUTY, 0);
	}
	else if (level > 15)
	{
		vfdSendCMD(VFD_DUTY, 15);
	}
	else
	{
		vfdSendCMD(VFD_DUTY, level);
	}
}

/***************************************************
 *
 * IOCTL routines
 *
 */
#define VFDIOC_DCRAMWRITE        0xc0425a00
#define VFDIOC_BRIGHTNESS        0xc0425a03
#define VFDIOC_DISPLAYWRITEONOFF 0xc0425a05
#define VFDIOC_DRIVERINIT        0xc0425a08
#define VFDIOC_ICONDISPLAYONOFF  0xc0425a0a

static int vfd_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct vfd_ioctl_data vfddata;

	DBG("[%s] >>>\n", __func__);
	switch (cmd)
	{
		case VFDIOC_DCRAMWRITE:
		{
			copy_from_user(&vfddata, (void *)arg, sizeof(struct vfd_ioctl_data));
			return VFD_WriteFront(vfddata.data, vfddata.length);
			break;
		}
		case VFDIOC_BRIGHTNESS:
		{
			copy_from_user(&vfddata, (void *)arg, sizeof(struct vfd_ioctl_data));
			VFD_setBrightness(vfddata.address);
			break;
		}
		case VFDIOC_DISPLAYWRITEONOFF:
		{
			break;
		}
		case VFDIOC_ICONDISPLAYONOFF:
		{
			copy_from_user(&vfddata, (void *)arg, sizeof(struct vfd_ioctl_data));
			VFD_setIcon(vfddata.data, vfddata.length);
			break;
		}
		case VFDIOC_DRIVERINIT:
		{
			VFD_init();
			break;
		}
		default:
		{
			ERR("[vfd] unknown ioctl %08x", cmd);
			break;
		}
	}
	return 0;
}

static ssize_t vfd_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
	DBG("[%s] text = '%s', len= %d\n", __func__, buf, len);
	VFD_WriteFront((unsigned char*)buf, len);
	return len;
}

static ssize_t vfd_read(struct file *filp, char *buf, size_t len, loff_t *off)
{
	return len;
}

static int vfd_open(struct inode *inode, struct file *file)
{
	DBG("[%s] >>>\n", __func__);

	if (down_interruptible(&(vfd.sem)))
	{
		DBG("interrupted while waiting for semaphore.");
		return -ERESTARTSYS;
	}
	if (vfd.opencount > 0)
	{
		DBG("device already opened.");
		up(&(vfd.sem));
		return -EUSERS;
	}
	vfd.opencount++;
	up(&(vfd.sem));
	return 0;
}

static int vfd_close(struct inode *inode, struct file *file)
{
	DBG("[%s] >>>\n", __func__);
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
	DBG("[%s] >>>\n", __func__);
	unregister_chrdev(VFD_MAJOR, "vfd");
	input_unregister_device(button_dev);
}

static int __init led_module_init(void)
{
	int error;

	DBG("VFD PACE7241 init >>>");
	DBG("register character device %d.", VFD_MAJOR);
	if (register_chrdev(VFD_MAJOR, "vfd", &vfd_fops))
	{
		ERR("register major %d failed", VFD_MAJOR);
		goto led_init_fail;
	}
	else
	{
		DBG("character device registered properly");
	}
	sema_init(&(vfd.sem), 1);
	vfd.opencount = 0;

	error = gpio_request(VFD_RST, "VFD_RST");
	if (error)
	{
		ERR("gpio_request VFD_RST failed.");
	}
	else
	{
		DBG("gpio_request VFD_RST successful, setting STM_GPIO_DIRECTION_OUT");
		gpio_direction_output(VFD_RST, STM_GPIO_DIRECTION_OUT);
	}
#if 0	
	error = gpio_request(VFD_VDON, "VFD_VDON");
	if (error)
	{
		ERR("gpio_request VFD_VDON failed.");
	}
	else
	{
		DBG("gpio_request VFD_VDON successful, setting STM_GPIO_DIRECTION_OUT");
		gpio_direction_output(VFD_VDON, STM_GPIO_DIRECTION_OUT);
		gpio_set_value(VFD_VDON, 1);
	}
#endif	
	error = gpio_request(VFD_DAT, "VFD_DAT");
	if (error)
	{
		ERR("gpio_request VFD_DAT failed.");
	}
	else
	{
		DBG("gpio_request VFD_DAT successful, setting STM_GPIO_DIRECTION_OUT");
		gpio_direction_output(VFD_DAT, STM_GPIO_DIRECTION_OUT);
		gpio_set_value(VFD_DAT, 0);
	}
	error = gpio_request(VFD_CS, "VFD_CS");
	if (error)
	{
		ERR("gpio_request VFD_CS failed.");
	}
	else
	{
		DBG("gpio_request VFD_CS successful, setting STM_GPIO_DIRECTION_OUT");
		gpio_direction_output(VFD_CS, STM_GPIO_DIRECTION_OUT);
		gpio_set_value(VFD_CS, 0);
	}
	VFD_init();
	error = gpio_request(LED1, "LED1");
	if (error)
	{
		ERR("Request LED1 failed. abort.");
		goto led_init_fail;
	}
	else
	{
		DBG("LED1 init successful");
		gpio_direction_output(LED1, STM_GPIO_DIRECTION_OUT);
		gpio_set_value(LED1, 0);//red
	}
	error = gpio_request(LED2, "LED2");
	if (error)
	{
		ERR("Request LED2 failed. abort.");
		goto led_init_fail;
	}
	else
	{
		DBG("LED2 init successful");
		gpio_direction_output(LED2, STM_GPIO_DIRECTION_OUT);
		gpio_set_value(LED2, 1);//blue
	}
	DBG("Keys init");
	error = gpio_request(KEYPm, "KEY_LEFT_");
	if (error)
	{
		ERR("Request KEYPm failed. abort.");
		goto led_init_fail;
	}
	else
	{
		DBG("KEYPm init successful");
	}
//	gpio_direction_output(KEY_LEFT_, STM_GPIO_DIRECTION_IN);
	gpio_direction_input(KEYPm);

	error = gpio_request(KEYPp, "KEY_RIGHT_");
	if (error)
	{
		ERR("Request KEY_RIGHT_ failed. abort.");
		goto led_init_fail;
	}
	else
	{
		DBG("KEYPp init successful");
	}
//	gpio_direction_output(KEY_RIGHT_, STM_GPIO_DIRECTION_IN);
	gpio_direction_input(KEYPp);

	button_dev = input_allocate_device();
	if (!button_dev)
	{
		ERR("Error : input_allocate_device");
		goto led_init_fail;
	}
	else
	{
		DBG("button_dev allocated properly");
	}
	button_dev->name = button_driver_name;
	button_dev->open = button_input_open;
	button_dev->close = button_input_close;

	set_bit(EV_KEY, button_dev->evbit);
	set_bit(KEY_RIGHT, button_dev->keybit);
	set_bit(KEY_LEFT, button_dev->keybit);
	set_bit(KEY_POWER, button_dev->keybit);

	error = input_register_device(button_dev);
	if (error)
	{
		input_free_device(button_dev);
		ERR("Request input_register_device. abort.");
		goto led_init_fail;
	}
	else
	{
		DBG("Request input_register_device successful");
	}
	return 0;

led_init_fail:
	led_module_exit();
	ERR("led_init_fail !!!");
	return -EIO;
}

module_init(led_module_init);
module_exit(led_module_exit);

MODULE_DESCRIPTION("Pace7241 front vfd driver");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled, 1=enabled");
// vim:ts=4
