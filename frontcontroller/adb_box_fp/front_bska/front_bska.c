/*
 * front_bska.c
 *
 * (c) 20?? ??
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
 * Front panel driver for ADB ITI-5800S, BSKA and BXZB models;
 * Button and display driver.
 *
 * Device:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 201????? freebox         Initial version.
 * 20200419 Audioniek       Fixed UTF8 handling.
 * 20200419 Audioniek       More legible front panel font.
 * 20200419 Audioniek       Removed code for quotes, periods and colons.
 * 20200420 Audioniek       /dev/vfd scroll texts longer than 4 once.
 * 20200430 Audioniek       Reading /dev/vfd added.
 * 2020???? Audioniek       procfs added.
 *
 ****************************************************************************************/
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/input.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/stm/pio.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include "pt6958_utf.h"
#include "front_bska.h"

// Global variables
static int buttoninterval = 350 /*ms*/;
static struct timer_list button_timer;

static char *button_driver_name = "nBox front panel buttons driver";
static struct input_dev *button_dev;

struct fp_driver
{
	struct semaphore sem;
	int              opencount;
};

spinlock_t mr_lock = SPIN_LOCK_UNLOCKED;

static Global_Status_t status;

static struct fp_driver led;
static int bad_polling = 1;  // ? is never changed!

// Default module parameters values
short paramDebug = 0;

// LED states
static char led1 = 0;  // power LED
static char led2 = 0;  // timer
static char led3 = 0;  // at
static char led4 = 0;  // alert
int led_brightness = 5;  // default LED brightness

static char button_reset = 0;

static unsigned char key_group1, key_group2;
static unsigned int  key_front = 0;

struct stpio_pin *key_int;
struct stpio_pin *pt6958_din;  // PIO for Din
struct stpio_pin *pt6958_clk;  // PIO for CLK
struct stpio_pin *pt6958_stb;  // PIO for STB

tFrontPanelOpen FrontPanelOpen [LASTMINOR];
struct saved_data_s lastdata;

/******************************************************
 *
 * Routines to communicate with the PT6958
 *
 */

/******************************************************
 *
 * Free the PT6958 PIO pins
 *
 */
static void pt6958_free_pio_pins(void)
{
//	dprintk(100, "%s >\n", __func__);
	if (pt6958_stb)
	{
		stpio_set_pin(pt6958_stb, 1);  // deselect PT6958
	}
	if (pt6958_din)
	{
		stpio_free_pin(pt6958_din);
	}
	if (pt6958_clk)
	{
		stpio_free_pin(pt6958_clk);
	}
	if (pt6958_stb)
	{
		stpio_free_pin(pt6958_stb);
	}
//	dprintk(100, "%s < PT6958 PIO pins freed\n", __func__);
};

/******************************************************
 *
 * Initialize the PT6958 PIO pins
 *
 */
static unsigned char pt6958_init(void)
{
//	dprintk(100, "%s > Initialize PT6958 PIO pins\n", __func__);

// Strobe
//	dprintk(100, "Request STPIO %d,%d for PT6958 STB\n", PORT_STB, PIN_STB);
	pt6958_stb = stpio_request_pin(PORT_STB, PIN_STB, "PT6958_STB", STPIO_OUT);
	if (pt6958_stb == NULL)
	{
		dprintk(1, "Request for STB STPIO failed; abort\n");
		goto pt_init_fail;
	}
// Clock (pt6958_clk)
//	dprintk(100, "Request STPIO %d,%d for PT6958 CLK\n", PORT_CLK, PIN_CLK);
	pt6958_clk = stpio_request_pin(PORT_CLK, PIN_CLK, "PT6958_CLK", STPIO_OUT);
	if (pt6958_clk == NULL)
	{
		dprintk(1, "Request for CLK STPIO failed; abort\n");
		goto pt_init_fail;
	}
// Data (pt6958_din)
//	dprintk(100, "Request STPIO %d,%d for PT6958 Din\n", PORT_DIN, PIN_DIN);
	pt6958_din = stpio_request_pin(PORT_DIN, PIN_DIN, "PT6958_DIN", STPIO_BIDIR);
	if (pt6958_din == NULL)
	{
		dprintk(1, "Request for DIN STPIO failed; abort\n");
		goto pt_init_fail;
	}
	stpio_set_pin(pt6958_stb, 1);  // set all involved PIO pins high
	stpio_set_pin(pt6958_clk, 1);
	stpio_set_pin(pt6958_din, 1);
	dprintk(10, "PT6958 PIO pins allocated\n");
//	dprintk(100, "%s <\n", __func__);
	return 1;

pt_init_fail:
	pt6958_free_pio_pins();
	dprintk(1, "%s < Error initializing PT6958 PIO pins\n", __func__);
	return 0;
};

/****************************************************
 *
 * Routines for the PT6958 (LED display & buttons)
 *
 */

/****************************************************
 *
 * Read one byte from the PT6958 (button data)
 *
 */
static unsigned char pt6958_read_byte(void)
{
	unsigned char i;
	unsigned char data_in = 0;

//	dprintk(100, "%s >\n", __func__);
	for (i = 0; i < 8; i++)  // 8 bits in a byte, LSB first
	{
		stpio_set_pin(pt6958_din, 1);  // data = 1 (key will pull down the pin if pressed)
		stpio_set_pin(pt6958_clk, 0);  // toggle
		udelay(IO_Delay);
		stpio_set_pin(pt6958_clk, 1);  // clock pin
		udelay(IO_Delay);
		data_in = (data_in >> 1) | (stpio_get_pin(pt6958_din) > 0 ? 0x80 : 0);
	}
//	dprintk(100, "%s <\n", __func__);
	return data_in;
}

/****************************************************
 *
 * Write one byte to the PT6958
 *
 */
static void pt6958_send_byte(unsigned char byte)
{
	unsigned char i;

//	dprintk(100, "%s >\n", __func__);
	for (i = 0; i < 8; i++)  // 8 bits in a byte, LSB first
	{
		stpio_set_pin(pt6958_din, byte & 0x01);  // write bit
		stpio_set_pin(pt6958_clk, 0);  // toggle
		udelay(IO_Delay);
		stpio_set_pin(pt6958_clk, 1);  // clock pin
		udelay(IO_Delay);
		byte >>= 1;  // get next bit
	}
//	dprintk(100, "%s <\n", __func__);
}

/****************************************************
 *
 * Write one byte to the PT6958
 *
 */
static void pt6958_send_cmd(unsigned char Value)
{
//	dprintk(100, "%s >\n", __func__);
	stpio_set_pin(pt6958_stb, 0);  // set strobe low
	pt6958_send_byte(Value);  // send byte Value
	stpio_set_pin(pt6958_stb, 1);  // and raise strobe
	udelay(IO_Delay);
//	dprintk(100, "%s <\n", __func__);
}

/****************************************************
 *
 * Write len bytes to the PT6958
 *
 */
static void PT6958_WriteData(unsigned char *data, unsigned int len)
{
	unsigned char i;

//	dprintk(100, "%s >\n", __func__);
	stpio_set_pin(pt6958_stb, 0);  // set strobe low

	for (i = 0; i < len; i++)
	{
		pt6958_send_byte(data[i]);
	}
	stpio_set_pin(pt6958_stb, 1);  // set strobe high
	udelay(IO_Delay);
//	dprintk(100, "%s <\n", __func__);
}

/****************************************************
 *
 * Read button data from the PT6958
 *
 */
static void ReadKey(void)
{
	dprintk(100, "%s >\n", __func__);
	spin_lock(&mr_lock);

	stpio_set_pin(pt6958_stb, 0);  // set strobe low
	pt6958_send_byte(DATA_SETCMD + READ_KEYD);  // send command 01000010b -> Read key data
	key_group1 = pt6958_read_byte();  // Get SG1/KS1 (b0..3), SG2/KS2 (b4..7)
	key_group2 = pt6958_read_byte();  // Get SG3/KS3 (b0..3), SG4/KS4 (b4..7)
//	key_group3 = pt6958_read_byte();  // Get SG5/KS5, SG6/KS6  // not needed
	stpio_set_pin(pt6958_stb, 1);  // set strobe high
	udelay(IO_Delay);

	spin_unlock(&mr_lock);
	dprintk(100, "%s <\n", __func__);
}

/******************************************************
 *
 * Set text on LED display, also set LEDs
 *
 * Note: does not scroll, only displays the first 4
 * characters. For scrolling, use /dev/vfd.
 */
static void PT6958_Show(unsigned char digit1, unsigned char digit2, unsigned char digit3, unsigned char digit4)
{
	spin_lock(&mr_lock);

	pt6958_send_cmd(DATA_SETCMD + 0); // Set test mode off, increment address and write data display mode  01xx0000b
	udelay(IO_Delay);

	stpio_set_pin(pt6958_stb, 0);  // set strobe (latches command)

	pt6958_send_byte(ADDR_SETCMD + 0); // Command 2 address set, (start from 0)   11xx0000b

	pt6958_send_byte(digit1);
	pt6958_send_byte(led1);  // power led: 00-off, 01-red, 02-green, 03-orange

	pt6958_send_byte(digit2);
	pt6958_send_byte(led2);  // timer led

	pt6958_send_byte(digit3);
	pt6958_send_byte(led3);  // mail led (@)

	pt6958_send_byte(digit4);
	pt6958_send_byte(led4);  // alert led

	stpio_set_pin(pt6958_stb, 1);
	udelay(IO_Delay);

	spin_unlock(&mr_lock);
}

/******************************************************
 *
 * Show text string on LED display
 *
 * Accepts any string length; string may hold UTF8
 * coded characters. Only first four printable
 * characters are shown.
 *
 */
static int pt6958_write_text(unsigned char *data, unsigned char len)
{
	unsigned char kbuf[8] = { 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 };
	int i = 0;
	int wlen = 0;
	unsigned char *UTF_Char_Table = NULL;

	/* Strip possible trailing LF */
	if (data[len - 1] == '\n')
	{
		data[len - 1] = '\0';
		len--;
	}

	while ((i < len) && (wlen < 8))
	{
		if (data[i] == '\n' || data[i] == 0)
		{
//			ignore line feeds and nulls
			break;
		}
		else if (data[i] < 0x20)
		{
//			skip non printable control characters
			i++;
		}
		else if (data[i] < 0x80)  // normal ASCII
		{
			kbuf[wlen] = data[i];
			wlen++;
		}
		/* handle UTF-8 characters */
		else if (data[i] < 0xe0)
		{
			switch (data[i])
			{
				case 0xc2:
				{
					UTF_Char_Table = PT6958_UTF_C2;
					break;
				}
				case 0xc3:
				{
					UTF_Char_Table = PT6958_UTF_C3;
					break;
				}
				case 0xc4:
				{
					UTF_Char_Table = PT6958_UTF_C4;
					break;
				}
				case 0xc5:
				{
					UTF_Char_Table = PT6958_UTF_C5;
					break;
				}
#if 0  // cyrillic currently not supported
				case 0xd0:
				{
					UTF_Char_Table = PT6958_UTF_D0;
					break;
				}
				case 0xd1:
				{
					UTF_Char_Table = PT6958_UTF_D1;
					break;
				}
#endif
				default:
				{
					UTF_Char_Table = NULL;
				}
			}
			i++;  // skip UTF lead in
			if (UTF_Char_Table)
			{
				kbuf[wlen] = UTF_Char_Table[data[i] & 0x3f];
				wlen++;
			}
		}
		else
		{
			if (data[i] < 0xf0)
			{
				i += 2;
			}
			else if (data[i] < 0xf8)
			{
				i += 3;
			}
			else if (data[i] < 0xfc)
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
	PT6958_Show(ROM[kbuf[0]], ROM[kbuf[1]], ROM[kbuf[2]], ROM[kbuf[3]]);  // display text
//	dprintk(100, "%s <\n", __func__);
	return 0;
}

/******************************************************
 *
 * Set LED
 *
 */
static void pt6958_set_led(unsigned char *kbuf, unsigned char len)
{
	unsigned char pos = 0, on = 0;

//	dprintk(100, "%s >\n", __func__);
	spin_lock(&mr_lock);

	if (len == 5)
	{
		pos = kbuf[0];  // LED#, 
		on  = kbuf[4];  // state
		on  = (on == 0 ? 0 : 1);  // normalize on value

		switch (pos)
		{
			case 1:  // led power, red
			{
				led1 = on;
				break;
			}
			case 2:  // led power, green
			{
				led1 = (on == 1 ? 2 : 0);
				on = led1;
				pos = 1;
				break;
			}
			case 3:  // timer LED
			{
				led2 = on;
				break;
			}
			case 4:  // @ LED
			{
				led3 = on;
				pos = 5;
				break;
			}
			case 5:  // alert LED
			{
				led4 = on;  // save state
				pos = 7;  // get address command offset
				break;
			}
		}
		pt6958_send_cmd(DATA_SETCMD + ADDR_FIX);  // Set command, normal mode, fixed address, write date to display mode 01xx0100b
		udelay(IO_Delay);

		stpio_set_pin(pt6958_stb, 0);  // select PT6958
		pt6958_send_byte(ADDR_SETCMD + pos);  // set position (11xx????b)
		pt6958_send_byte(on);  // set state (on or off)
		stpio_set_pin(pt6958_stb, 1);  // de select PT6958
		udelay(IO_Delay);
	}
	spin_unlock(&mr_lock);
//	dprintk(100, "%s <\n", __func__);
}

/****************************************************
 *
 * Initialize the PT6958
 *
 */
static void pt6958_setup(void)
{
	unsigned char i;

//	dprintk(100, "%s >\n", __func__);
	pt6958_send_cmd(DATA_SETCMD);  // Command 1, increment address, normal mode
	udelay(IO_Delay);

	stpio_set_pin(pt6958_stb, 0);  // select PT6958
	pt6958_send_byte(ADDR_SETCMD);  // Command 2, RAM address = 0
	for (i = 0; i < 10; i++)  // 10 bytes
	{
		pt6958_send_byte(0);  // clear display RAM (all segments off)
	}
	stpio_set_pin(pt6958_stb, 1);  // and deselect PT6958
	udelay(IO_Delay);
	pt6958_send_cmd(DISP_CTLCMD + DISPLAY_ON + led_brightness);  // Command 3, display control, (Display ON), brightness 11/16 10xx1100b

	led1 = 2;  // power LED green
	led2 = 0;  // timer LED off
	led3 = 0;  // @ LED off
	led4 = 0;  // alert LED off
	PT6958_Show(0x40, 0x40, 0x40, 0x40);  // display ----, power LED green
//	dprintk(100, "%s <\n", __func__);
}

/******************************************************
 *
 * Set brightness of LED display and LEDs
 *
 */
static void pt6958_set_brightness(int level)
{
//	dprintk(100, "%s >\n", __func__);
	spin_lock(&mr_lock);

	if (level < 0)
	{
		level = 0;
	}
	if (level > 7)
	{
		level = 7;
	}
	pt6958_send_cmd(DISP_CTLCMD + DISPLAY_ON + level);  // Command 3, display control, Display ON, brightness level;  // Command 3, display control, (Display ON) 10xx1???b
	led_brightness = level;
	spin_unlock(&mr_lock);
//	dprintk(100, "%s <\n", __func__);
}

/******************************************************
 *
 * Switch LED display on or off
 *
 */
static void pt6958_set_light(int onoff)
{
//	dprintk(100, "%s >\n", __func__);
	spin_lock(&mr_lock);

	pt6958_send_cmd(DISP_CTLCMD + (onoff ? DISPLAY_ON + 4 : 0));

	spin_unlock(&mr_lock);
//	dprintk(100, "%s <\n", __func__);
}


/******************************************************
 *
 * Button driver
 *
 */
static void button_delay(unsigned long dummy)
{
	if (button_reset == 100)
	{
		kernel_restart(NULL);
	}
	else
	{
		if (key_front > 0)
		{
			input_report_key(button_dev, key_front, 0);
			input_sync(button_dev);
			key_front = 0;
		}
		enable_irq(FPANEL_PORT2_IRQ);
	}
}

/******************************************************
 *
 * Button IRQ handler
 *
 */
irqreturn_t fpanel_irq_handler(void *dev1, void *dev2)
{
//	dprintk(100, "%s >\n", __func__);

	disable_irq(FPANEL_PORT2_IRQ);

	if ((stpio_get_pin(key_int) == 1) && (key_front == 0))
	{
		ReadKey();
		key_front = 0;
/* keygroup1:
 *    columns KS1 and KS2,
 *        bits 0 & 4 = row K1,
 *        bits 1 & 5 = row K2,
 *        bits 2 & 6 = row K3,
 *        bit s3 & 7 = row K4
 * keygroup2:
 *    columns KS3 and KS4,
 *        bits 0 & 4 = row K1,
 *        bits 1 & 5 = row K2,
 *        bits 2 & 6 = row K3,
 *        bits 3 & 7 = row K4
 */
		if ((key_group1 == 32) && (key_group2 == 0))
		{
			key_front = KEY_MENU;  // MENU (bska/bxzb)
//			key_front = KEY_EPG;  // EPG (bsla/bzzb)
		}
		if ((key_group1 == 00) && (key_group2 == 4))
		{
			key_front = KEY_EPG;  // EPG (bska/bxzb)
//			key_front = KEY_RECORD;  // REC (bsla/bzzb)
		}
		if ((key_group1 == 04) && (key_group2 == 0))
		{
			key_front = KEY_HOME;  // RES
		}
		if ((key_group1 == 16) && (key_group2 == 0))
		{
			key_front = KEY_UP;  // up
		}
		if ((key_group1 == 00) && (key_group2 == 1))
		{
			key_front = KEY_DOWN;  // down
		}
		if ((key_group1 == 64) && (key_group2 == 0))
		{
			key_front = KEY_RIGHT;  // right
		}
		if ((key_group1 == 00) && (key_group2 == 2))
		{
			key_front = KEY_LEFT;  // left
		}
		if ((key_group1 == 02) && (key_group2 == 0))
		{
			key_front = KEY_OK;  // OK
		}
		if ((key_group1 == 01) && (key_group2 == 0))
		{
			key_front = KEY_POWER;  // power
		}
		if (key_front > 0)
		{
			if (key_front == KEY_HOME)  // emergency reboot: press RES 5 times
			{
				button_reset++;
				if (button_reset > 4)
				{
					dprintk(10, "!!! Restart system !!!\n");
					button_reset = 100;
				}
			}
			else
			{
				button_reset = 0;
			}
			input_report_key(button_dev, key_front, 1);
			input_sync(button_dev);
			dprintk(10, "%s Key: %d\n", __func__, key_front);
			init_timer(&button_timer);
			button_timer.function = button_delay;
			mod_timer(&button_timer, jiffies + buttoninterval);
		}
		else
		{
			enable_irq(FPANEL_PORT2_IRQ);
		}
	}
	else
	{
		enable_irq(FPANEL_PORT2_IRQ);
	}
//	dprintk(100, "%s <\n", __func__);
	return IRQ_HANDLED;
}

static int button_input_open(struct input_dev *dev)
{
	dprintk(100, "%s >\n", __func__);
	dprintk(100, "%s <\n", __func__);
	return 0;
}

static void button_input_close(struct input_dev *dev)
{
	dprintk(100, "%s >\n", __func__);
	dprintk(100, "%s <\n", __func__);
}

/****************************************
 *
 * IOCTL handling.
 *
 */
static int led_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct led_ioctl_data leddata;

//	dprintk(100, "%s >\n", __func__);
	switch (cmd)
	{
		case VFDDISPLAYCHARS:
		case VFDBRIGHTNESS:
		case VFDDISPLAYWRITEONOFF:
		case VFDICONDISPLAYONOFF:
		case VFDSETLED:
		{
			copy_from_user(&leddata, (void *)arg, sizeof(struct led_ioctl_data));
		}
	}
	switch (cmd)
	{
		case VFDDISPLAYCHARS:
		{
			return pt6958_write_text(leddata.data, leddata.length);
		}
		case VFDBRIGHTNESS:
		{
			pt6958_set_brightness(leddata.address);
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			pt6958_set_light(leddata.address);
			break;
		}
		case VFDICONDISPLAYONOFF:
		case VFDSETLED:
		{
			pt6958_set_led(leddata.data, leddata.length);
			break;
		}
		case VFDDRIVERINIT:
		{
			pt6958_setup();
			break;
		}
		case VFDSETMODE:
		{
			break;  // VFDSETMODE is recognized, but not supported
		}
		default:
		{
			dprintk(1, "Unknown IOCTL %08x\n", cmd);
			break;
		}
	}
//	dprintk(100, "%s <\n", __func__);
	return 0;
}

/******************************************************
 *
 * Write string on LED display (/dev/vfd)
 *
 * Texts longer than the display size are scrolled
 * once, up to a maximum of 64 characters.
 *
 */
int clear_display(void)
{
	int ret = 0;

	ret |= pt6958_write_text("    ", LED_DISP_SIZE);
	return ret;
}

static ssize_t led_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
	int ret = 0;
	int i;
	int slen;
	unsigned char scroll_buf[65 + LED_DISP_SIZE];

//	if (buf[len - 1] == '\n')
//	{
//		len--;
//	}
	/* save last string written to fp */
	memcpy(lastdata.data, buf, len);
	lastdata.data[len] = 0;  // terminate string
	lastdata.length = len;
	dprintk(50, "Last display string: [%s], length = %d\n", lastdata.data, lastdata.length);

	if (len == 0)
	{
		ret |= clear_display();
	}
	else
	{
		slen = (len > 64 ? 64 : len);  // display 64 characters max.
		if (buf[slen - 1] == '\n')
		{
			slen--;
		}
		if (slen > LED_DISP_SIZE)
		{
			memset(scroll_buf, 0x20, sizeof(scroll_buf));  // clear scroll buffer
			memcpy(scroll_buf, buf, slen);  // copy text to show in scroll buffer
			for (i = 0; i < slen; i++)
			{
				ret |= pt6958_write_text(scroll_buf + i, ((slen - i) < LED_DISP_SIZE ? LED_DISP_SIZE : (slen - i)));
				msleep(500);
			}				
		}
		ret |= pt6958_write_text((unsigned char *)buf, LED_DISP_SIZE);
	}
	if (ret < 0)
	{
		return ret;
	}
	else
	{
		return len;
	}
}

/******************************************************
 *
 * Read string from LED display (/dev/vfd)
 *
 */
static ssize_t led_read(struct file *file, char *buf, size_t len, loff_t *off)
{
	int minor, vLoop;

//	dprintk(150, "%s > (len %d, offs %d)\n", __func__, len, (int)*off);

	if (len == 0)
	{
		return len;
	}

	minor = -1;
	for (vLoop = 0; vLoop < LASTMINOR; vLoop++)
	{
		if (FrontPanelOpen[vLoop].fp == file)
		{
			minor = vLoop;
		}
	}

	if (minor == -1)
	{
		dprintk(1, "Error Bad Minor\n");
		return -EUSERS;
	}

	dprintk(100, "minor = %d\n", minor);
#if 0
	if (minor == FRONTPANEL_MINOR_RC)
	{
		int size = 0;
		unsigned char data[20];

		memset(data, 0, sizeof(data));

		getRCData(data, &size);

		if (size > 0)
		{
			if (down_interruptible(&FrontPanelOpen[minor].sem))
			{
				return -ERESTARTSYS;
			}

			copy_to_user(buf, data, size);
			up(&FrontPanelOpen[minor].sem);

			dprintk(100, "%s < %d\n", __func__, size);
			return size;
		}
		dumpValues();
		return 0;
	}
#endif

	/* copy the current display string to the user */
//	if (down_interruptible(&FrontPanelOpen[minor].sem))
//	{
//		dprintk(1, "%s return ERESTARTSYS <\n", __func__);
//		return -ERESTARTSYS;
//	}

	if (FrontPanelOpen[minor].read == lastdata.length)
	{
		FrontPanelOpen[minor].read = 0;
//		up(&FrontPanelOpen[minor].sem);
		dprintk(1, "%s < return 0\n", __func__);
		return 0;
	}

	if (len > lastdata.length)
	{
		len = lastdata.length;
	}

	/* fixme: needs revision because of utf8! */
	if (len > 64)
	{
		len = 64;
	}

	FrontPanelOpen[minor].read = len;
	copy_to_user(buf, lastdata.data, len);
//	up(&FrontPanelOpen[minor].sem);
//	dprintk(150, "%s < (len %d)\n", __func__, len);
	return len;
}

static int led_open(struct inode *inode, struct file *file)
{
	int minor;

//	dprintk(150, "%s >\n", __func__);

	/* needed! otherwise a racecondition can occur */
	if (down_interruptible(&(led.sem)))
	{
		return -ERESTARTSYS;
	}

	minor = MINOR(inode->i_rdev);

//	dprintk(100, "open minor %d\n", minor);

	if (FrontPanelOpen[minor].fp != NULL)
	{
		printk("[front_bska] dev_open: EUSER opening minor\n");
		up(&(led.sem));
		return -EUSERS;
	}

	FrontPanelOpen[minor].fp = file;
	FrontPanelOpen[minor].read = 0;

	up(&(led.sem));
//	dprintk(150, "%s <\n", __func__);
	return 0;
}

static int led_close(struct inode *inode, struct file *file)
{
	int minor;

//	dprintk(150, "%s >\n", __func__);
	minor = MINOR(inode->i_rdev);
//	dprintk(20, "close minor %d\n", minor);

	if (FrontPanelOpen[minor].fp == NULL)
	{
		printk("[front_bska] dev_close: EUSER opening minor\n");
		return -EUSERS;
	}
	FrontPanelOpen[minor].fp = NULL;
	FrontPanelOpen[minor].read = 0;
//	dprintk(150, "%s <\n", __func__);
	return 0;
}

static struct file_operations led_fops =
{
	.owner   = THIS_MODULE,
	.ioctl   = led_ioctl,
	.write   = led_write,
	.read    = led_read,
	.open    = led_open,
	.release = led_close
};

//static void __exit led_module_exit(void)
static void led_module_exit(void)
{
//	dprintk(100, "%s >\n", __func__);
	unregister_chrdev(LED_MAJOR, "led");
	stpio_free_irq(key_int);
	pt6958_free_pio_pins();
	input_unregister_device(button_dev);
//	dprintk(100, "%s <\n", __func__);
}

static int __init led_module_init(void)
{
	int error;
	int res;

	dprintk(50, "Initialize ADB ITI-5800S front panel\n");

	dprintk(20, "Probe PT6958\n");
	if (pt6958_init() == 0)
	{
		dprintk(1, "Unable to initinitialize driver; abort\n");
		goto led_init_fail;
	}
	dprintk(10, "Register character device %d\n", LED_MAJOR);
	if (register_chrdev(LED_MAJOR, "led", &led_fops))
	{
		dprintk(1, "Register major %d failed\n", LED_MAJOR);
		goto led_init_fail;
	}
	sema_init(&(led.sem), 1);
	led.opencount = 0;

	pt6958_setup();

	dprintk(10, "Request STPIO %d,%d for key interrupt\n", PORT_KEY_INT, PIN_KEY_INT);
	key_int = stpio_request_pin(PORT_KEY_INT, PIN_KEY_INT, "Key_int_front", STPIO_IN);

	if (key_int == NULL)
	{
		dprintk(1, "Request STPIO for key_int failed; abort\n");
		goto led_init_fail;
	}
	stpio_set_pin(key_int, 1);

	dprintk(100, "stpio_flagged_request_irq IRQ_TYPE_LEVEL_LOW\n");
//	dprintk(10, "> stpio_flagged_request_irq IRQ_TYPE_LEVEL_LOW\n");
	stpio_flagged_request_irq(key_int, 0, (void *)fpanel_irq_handler, NULL, (long unsigned int)NULL);
//	dprintk(10, "< stpio_flagged_request_irq IRQ_TYPE_LEVEL_LOW\n");

	button_dev = input_allocate_device();
	if (!button_dev)
	{
		goto led_init_fail;
	}
	button_dev->name = button_driver_name;
	button_dev->open = button_input_open;
	button_dev->close = button_input_close;

	set_bit(EV_KEY, button_dev->evbit);
	set_bit(KEY_MENU, button_dev->keybit);  // (bska/bxzb)
	set_bit(KEY_EPG, button_dev->keybit);
//	set_bit(KEY_RECORD, button_dev->keybit);  // (bsla/bzzb)
	set_bit(KEY_HOME, button_dev->keybit);
	set_bit(KEY_UP, button_dev->keybit);
	set_bit(KEY_DOWN, button_dev->keybit);
	set_bit(KEY_RIGHT, button_dev->keybit);
	set_bit(KEY_LEFT, button_dev->keybit);
	set_bit(KEY_OK, button_dev->keybit);
	set_bit(KEY_POWER, button_dev->keybit);

	error = input_register_device(button_dev);
	if (error)
	{
		input_free_device(button_dev);
		dprintk(1, "Request input_register_device failed; abort\n");
		goto led_init_fail;
	}
	dprintk(50, "Front panel nBox BSKA/BXZB initialization successful\n");
	return 0;

led_init_fail:
	led_module_exit();
	return -EIO;
}

module_init(led_module_init);
module_exit(led_module_exit);

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled, >0=enabled (level)");

MODULE_DESCRIPTION("ADB ITI-5800S front panel driver");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
