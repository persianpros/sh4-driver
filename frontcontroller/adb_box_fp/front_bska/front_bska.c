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
 * Front panel driver for nBOX ITI-5800, BSKA and BZXB models;
 * Button driver.
 *
 * Devices:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *  - /dev/rc  (reading of key events)
 *
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 20?????? freebox         Initialize
 * 20190728 Audioniek       
 * 2019???? Audioniek       procfs added.
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
#include "../../vfd/utf.h"

#include "front_bska.h"

// Global variables
static int buttoninterval = 350 /*ms*/;
static struct timer_list button_timer;

static char *button_driver_name = "nBox frontpanel buttons";
static struct input_dev *button_dev;

struct led_driver
{
	struct semaphore sem;
	int              opencount;
};

spinlock_t mr_lock = SPIN_LOCK_UNLOCKED;

static Global_Status_t status;

static struct led_driver led;
static int bad_polling   = 1;  // ? is never changed!

short paramDebug         = 0;

// LED states
static char ICON1        = 0; // power LED
static char ICON2        = 0; // timer
static char ICON3        = 0; // at
static char ICON4        = 0; // alert

static char button_reset = 0;

static struct workqueue_struct *wq;

#define LED_Delay    5

static unsigned char key_group1, key_group2;
static unsigned int  key_front = 0;

struct stpio_pin *key_int;
struct stpio_pin *pt6958_din;  // PIO for Din
struct stpio_pin *pt6958_clk;  // PIO for CLK
struct stpio_pin *pt6958_stb;  // PIO for STB

/****************************************************
 *
 * Free the PT6958 PIO pins
 *
 */
static void pt6958_free(void)
{
	dprintk(150, "%s >\n", __func__);
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
	dprintk(150, "%s < PT6958 PIO pins freed\n", __func__);
};

/****************************************************
 *
 * Initialize the PT6958 PIO pins
 *
 */
static unsigned char pt6958_init(void)
{
	dprintk(150, "%s > Initialize pt6958 PIO pins\n", __func__);

// Strobe
	dprintk(100, "Request STPIO %d,%d for PT6958 STB\n", PORT_STB, PIN_STB);
	pt6958_stb = stpio_request_pin(PORT_STB, PIN_STB, "PT6958_STB", STPIO_OUT);
	if (pt6958_stb == NULL)
	{
		dprintk(1, "Request for STB STPIO failed; abort\n");
		goto pt_init_fail;
	}
// Clock (pt6958_clk)
	dprintk(100, "Request STPIO %d,%d for PT6958 CLK", PORT_CLK, PIN_CLK);
	pt6958_clk = stpio_request_pin(PORT_CLK, PIN_CLK, "PT6958_CLK", STPIO_OUT);
	if (pt6958_clk == NULL)
	{
		dprintk(1, "Request for CLK STPIO failed; abort\n");
		goto pt_init_fail;
	}
// Data (pt6958_din)
	dprintk(100, "Request STPIO %d,%d for PT6958 Din\n", PORT_DIN, PIN_DIN);
	pt6958_din = stpio_request_pin(PORT_DIN, PIN_DIN, "PT6958_DIN", STPIO_BIDIR);
	if (pt6958_din == NULL)
	{
		dprintk(1, "Request for DIN STPIO failed; abort\n");
		goto pt_init_fail;
	}
	stpio_set_pin(pt6958_stb, 1);  // set all involved PIO pins high
	stpio_set_pin(pt6958_clk, 1);
	stpio_set_pin(pt6958_din, 1);
	dprintk(150, "%s <\n", __func__);
	return 1;

pt_init_fail:
	pt6958_free();
	dprintk(1, "%s < error initializing PT6958 PIO pins\n", __func__);
	return 0;
};

/****************************************************
 *
 * Read one byte from the PT6958 (button data)
 *
 */
static unsigned char PT6958_ReadChar(void)
{
	unsigned char i;
	unsigned char data_in = 0;

	dprintk(150, "%s >\n", __func__);
	for (i = 0; i < 8; i++)  // 8 bits in a byte, LSB first
	{
		stpio_set_pin(pt6958_din, 1);  // data = 1 (key will pull down the pin if pressed)
		stpio_set_pin(pt6958_clk, 0);  // toggle
		udelay(LED_Delay);
		stpio_set_pin(pt6958_clk, 1);  // clock pin
		udelay(LED_Delay);
		data_in = (data_in >> 1) | (stpio_get_pin(pt6958_din) > 0 ? 0x80 : 0);
	}
	dprintk(150, "%s <\n", __func__);
	return data_in;
}

/****************************************************
 *
 * Write one command byte to the PT6958
 *
 */
static void PT6958_SendChar(unsigned char Value)
{
	unsigned char i;

	dprintk(150, "%s >\n", __func__);
	for (i = 0; i < 8; i++)  // 8 bits in a byte, LSB first
	{
		stpio_set_pin(pt6958_din, Value & 0x01);  // write bit
		stpio_set_pin(pt6958_clk, 0);  // toggle
		udelay(LED_Delay);
		stpio_set_pin(pt6958_clk, 1);  // clock pin
		udelay(LED_Delay);
		Value >>= 1;  // get next bit
	dprintk(150, "%s >\n", __func__);
	}
}

/****************************************************
 *
 * Write one data byte to the PT6958
 *
 */
static void PT6958_WriteChar(unsigned char Value)
{
	dprintk(150, "%s >\n", __func__);
	stpio_set_pin(pt6958_stb, 0);  // set strobe low
	PT6958_SendChar(Value);  // send byte Value
	stpio_set_pin(pt6958_stb, 1);  // and raise strobe
	udelay(LED_Delay);
	dprintk(150, "%s <\n", __func__);
}

/****************************************************
 *
 * Write len data bytes to the PT6958
 *
 */
static void PT6958_WriteData(unsigned char *data, unsigned int len)
{
	unsigned char i;

	dprintk(150, "%s >\n", __func__);
	stpio_set_pin(pt6958_stb, 0);  // set strobe low
	for (i = 0; i < len; i++)
	{
		PT6958_SendChar(data[i]);
	}
	stpio_set_pin(pt6958_stb, 1);  // strobe
	udelay(LED_Delay);
	dprintk(150, "%s <\n", __func__);
}

/****************************************************
 *
 * Read front panel button data from the PT6958
 *
 */
static void ReadKey(void)
{
	dprintk(150, "%s >\n", __func__);
	spin_lock(&mr_lock);

	stpio_set_pin(pt6958_stb, 0);  // set strobe low
	PT6958_SendChar(DATA_SETCMD + READ_KEYD);  // send command 01000010 -> Read key data
	key_group1 = PT6958_ReadChar();  // Get SG1/KS1 (b0..3), SG2/KS2 (b4..7)
	key_group2 = PT6958_ReadChar();  // Get SG3/KS3 (b0..3), SG4/KS4 (b4..7)
//	key_group3 = PT6958_ReadChar();  // Get SG5/KS5, SG6/KS6  // not used
	stpio_set_pin(pt6958_stb, 1);  // raise strobe
	udelay(LED_Delay);  // wait 1 us

	spin_unlock(&mr_lock);
	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Set text and dots on LED display
 *
 */
static void PT6958_Show(unsigned char DIG1, unsigned char DIG2, unsigned char DIG3, unsigned char DIG4,
                        unsigned char DOT1, unsigned char DOT2, unsigned char DOT3, unsigned char DOT4)
{
	dprintk(150, "%s >\n", __func__);
	spin_lock(&mr_lock);

	PT6958_WriteChar(DATA_SETCMD + 0); // Set test mode off, increment address and write data display mode  01xx0000b
	udelay(LED_Delay);

	stpio_set_pin(pt6958_stb, 0);  // set strobe (latches command)

	PT6958_SendChar(ADDR_SETCMD + 0); // Command 2 address set, (start from 0)   11xx0000b
	DIG1 += (DOT1 == 1 ? 0x80 : 0);  // if decimal point set, add to digit data
	PT6958_SendChar(DIG1);
	PT6958_SendChar(ICON1);  //power led: 00-off, 01-red, 02-green, 03-yellow 

	DIG2 += (DOT2 == 1 ? 0x80 : 0);  // if decimal point set, add to digit data
	PT6958_SendChar(DIG2);
	PT6958_SendChar(ICON2);  //timer led

	DIG3 += (DOT1 == 3 ? 0x80 : 0);  // if decimal point set, add to digit data
	PT6958_SendChar(DIG3);
	PT6958_SendChar(ICON3);  //mail led (@)

	DIG4 += (DOT4 == 1 ? 0x80 : 0);  // if decimal point set, add to digit data
	PT6958_SendChar(DIG4);
	PT6958_SendChar(ICON4);  // alert led

	stpio_set_pin(pt6958_stb, 1);
	udelay(LED_Delay);

	spin_unlock(&mr_lock);
	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Set string on LED display
 *
 * Accepts any string length; string may hold UTF8
 * coded characters
 * Periods, single quotes and semicolons are enhanced
 * by setting the periods in the display.
 */
static int PT6958_ShowBuf(unsigned char *data, unsigned char len)
{
	unsigned char z1, z2, z3, z4, k1, k2, k3, k4;
	unsigned char kbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int i = 0;
	int wlen = 0;

	dprintk(150, "%s >\n", __func__);
	/* Strip possible trailing LF */
	if (data[len - 1] == '\n')
	{
		data[len - 1] = '\0';
		len--;
	}
	dprintk(10, "%s > LED text: [%s] (len = %d)\n", __func__, data, len);

	/* handle UTF-8 characters */
	while ((i < len) && (wlen < 8))
	{
		if (data[i] == '\n' || data[i] == 0)
		{
			dprintk(10, "[%s] BREAK CHAR detected (0x%X)\n", __func__, data[i]);
			break;
		}
		else if (data[i] < 0x20)
		{
			dprintk(10, "[%s] NON_PRINTABLE_CHAR '0x%X'\n", __func__, data[i]);
			i++;
		}
		else if (data[i] < 0x80)
		{
			kbuf[wlen] = data[i];
			dprintk(10, "[%s] ANSI_Char_Table '0x%X'\n", __func__, data[i]);
			wlen++;
		}
		else if (data[i] < 0xE0)
		{
			dprintk(10, "[%s] UTF_Char_Table= ", __func__);
			switch (data[i])
			{
				case 0xc2:
				{
					if (paramDebug)
					{
						printk("0x02X\n");
					}
					UTF_Char_Table = UTF_C2;
					break;
				}
				case 0xc3:
				{
					if (paramDebug)
					{
						printk("0x02X\n");
					}
					UTF_Char_Table = UTF_C3;
					break;
				}
				case 0xc4:
				{
					if (paramDebug)
					{
						printk("0x02X\n");
					}
					UTF_Char_Table = UTF_C4;
					break;
				}
				case 0xc5:
				{
					if (paramDebug)
					{
						printk("0x02X\n");
					}
					UTF_Char_Table = UTF_C5;
					break;
				}
				case 0xd0:
				{
					if (paramDebug)
					{
						printk("0x02X\n");
					}
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
			i++;  // ship UTF lead in
			if (UTF_Char_Table)
			{
				dprintk(10, "[%s] UTF_Char = 0x%X, index = %i\n", __func__, UTF_Char_Table[data[i] & 0x3f], i);
				kbuf[wlen] = UTF_Char_Table[data[i] & 0x3f];
				wlen++;
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
	/* end */
	k1 = k2 = k3 = k4 = 0; // clear decimal point flags
	z1 = z2 = z3 = z4 = 0x20;  // set non used positions to space (blank)

// determine the decimal points: if character is period, single quote or colon, replace it by a decimal point
	if ((wlen >= 2) && (data[1] == '.' || data[1] == 0x27 || data[1] == ':'))
	{
		k1 = 1;  // flag period at position 1
	}
	if ((wlen >= 3) && (data[2] == '.' || data[2] == 0x27 || data[2] == ':'))
	{
		k2 = 1;  // flag period at position 2
	}
	if ((wlen >= 4) && (data[3] == '.' || data[3] == 0x27 || data[3] == ':'))
	{
		k3 = 1;  // flag period at position 3
	}
	if ((wlen >= 5) && (data[4] == '.' || data[4] == 0x27 || data[4] == ':'))
	{
		k4 = 1;  // flag period at position 4
	}
// assigning segment data
	if (wlen >= 1)
	{
		z1 = ROM[data[0]];
	}
	if (wlen >= 2)
	{
		z2 = ROM[data[1 + k1]];
	}
	if (wlen >= 3)
	{
		z3 = ROM[data[2 + k1 + k2]];
	}
	if (wlen >= 4)
	{
		z4 = ROM[data[3 + k1 + k2 + k3]];
	}
	PT6958_Show(z1, z2, z3, z4, k1, k2, k3, k4); // display text & decimal points
	dprintk(150, "%s <\n", __func__);
	return 0;
}

/******************************************************
 *
 * Set LEDs through a string of length 5
 *
 */
static void pt6958_set_icon(unsigned char *kbuf, unsigned char len)
{
	unsigned char pos = 0, ico = 0;

	dprintk(150, "%s >\n", __func__);
	spin_lock(&mr_lock);

	if (len == 5)
	{
		pos = kbuf[0];  // 1st character is position 1 = power LED, 
		ico = kbuf[4];  // last character is state

		switch (pos)
		{
			case 1:
			{
				ICON1 = ico;  // led power, red
				pos = ADDR_SETCMD + 1;
				break;
			}
			case 2:  // led power, green
			{
				if (ico == 1)
				{
					ICON1 = 2;  // select green
					ico = 2;
				}
				else
				{
					ICON1 = 0;
					ico = 0;
				}
				pos = ADDR_SETCMD + 1;
				break;
			}
			case 3:  // timer LED
			{
				ICON2 = ico;
				pos = ADDR_SETCMD + 3;
				break;
			}
			case 4:  // @ LED
			{
				ICON3 = ico;
				pos = ADDR_SETCMD + 5;
				break;
			}
			case 5:  // alert LED
			{
				ICON4 = ico;  // save state
				pos = ADDR_SETCMD + 7;  // get address command
				break;
			}
		}
		PT6958_WriteChar(DATA_SETCMD + ADDR_FIX);  // Set command, normal mode, fixed address, write date to display mode 01xx0100b
		udelay(LED_Delay);

		stpio_set_pin(pt6958_stb, 0);  // set strobe low
		PT6958_SendChar(pos);  // set position (11xx????b)
		PT6958_SendChar(ico);  // set state (on or off)
		stpio_set_pin(pt6958_stb, 1);  // set strobe high
		udelay(LED_Delay);
	}
	spin_unlock(&mr_lock);
	dprintk(150, "%s <\n", __func__);
}

/****************************************************
 *
 * Initialize the PT6958
 *
 */
static void PT6958_setup(void)
{
	unsigned char i;

	dprintk(150, "%s >\n", __func__);
	PT6958_WriteChar(DATA_SETCMD);  // Command 1, increment address, normal mode
	udelay(LED_Delay);

	stpio_set_pin(pt6958_stb, 0);  // set strobe low
	PT6958_SendChar(ADDR_SETCMD);  // Command 2, RAM address = 0
	for (i = 0; i < 10; i++)  // 10 bytes
	{
		PT6958_SendChar(0);  // clear display RAM (all segments off)
	}
	stpio_set_pin(pt6958_stb, 1);  // set strobe high
	udelay(LED_Delay);
	PT6958_WriteChar(DISP_CTLCMD + DISPLAY_ON + 4);  // Command 3, display control, (Display ON), brightness 11/16 10xx1100b

	ICON1 = 2;  //power LED green
	ICON2 = 0;
	ICON3 = 0;
	ICON4 = 0;
	PT6958_Show(0x40, 0x40, 0x40, 0x40, 0, 0, 0, 0);  // display ----, periods off, power LED green
	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Set brightness of LED display
 *
 */
static void pt6958_set_brightness(int level)
{
	dprintk(150, "%s >\n", __func__);
	spin_lock(&mr_lock);

	if (level < 0)
	{
		level = 0;
	}
	if (level > 7)
	{
		level = 7;
	}
	PT6958_WriteChar(DISP_CTLCMD + DISPLAY_ON + level);  // Command 3, display control, Display ON, brightness level;  // Command 3, display control, (Display ON) 10xx1???b
	spin_unlock(&mr_lock);
	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Switch LED display on or off
 *
 */
static void pt6958_set_light(int onoff)
{
	dprintk(150, "%s >\n", __func__);
	spin_lock(&mr_lock);

	PT6958_WriteChar(DISP_CTLCMD + (onoff ? DISPLAY_ON + 4 : 0));

	spin_unlock(&mr_lock);
	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Button driver
 *
 */
static void button_delay(unsigned long dummy)
{
	dprintk(150, "%s >\n", __func__);
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
	dprintk(150, "%s <\n", __func__);
}

/******************************************************
 *
 * Button IRQ handler
 *
 */
//void fpanel_irq_handler(void *dev1,void *dev2)
irqreturn_t fpanel_irq_handler(void *dev1, void *dev2)
{
	dprintk(150, "%s >\n", __func__);
	disable_irq(FPANEL_PORT2_IRQ);

	if ((stpio_get_pin(key_int) == 1) && (key_front == 0))
	{
		ReadKey();
		key_front = 0;
/* keygroup1: columns KS1 and KS2, bit 0&4=row K1, bit1&5=row K2, bit2&6=row K3, bit3&7=row K4
 * keygroup2: columns KS3 and KS4, bit 0&4=row K1, bit1&5=row K2, bit2&6=row K3, bit3&7=row K4 */
#if 0
		if (key_group2 == 0) // no key on KS3 / KS4
		{
			switch (key_group1) // get KS1 / KS2
			{
				case 64:  //  KS2 row K3
				{
					key_front = KEY_RIGHT;  // right
					break;
				}
				case 32:  // KS2 row K2
				{
					key_front = KEY_MENU;
//					key_front = KEY_EPG;  // epg
					break;
				}
				case 16: // KS2 row K1
				{
					key_front = KEY_UP;  // up
					break;
				}
				case 4:  // KS1 row K3
				{
					key_front = KEY_HOME;  // res
					break;
				}
				case 2:  // KS1 row K2
				{
					key_front = KEY_OK;  // ok
					break;
				}
				case 1: // KS1 row K1
				{
					key_front = KEY_POWER;  // pwr
					break;
				}
			}
			else
			{
			switch (key_group2)
			{
				case 4:  // KS3 row K3
				{
					key_front = KEY_EPG;  // epg
//					key_front = KEY_RECORD;  // record
					break;
				}
				case 2:  // KS3 row K2
				{
					key_front = KEY_LEFT;  // left
					break;
				}
				case 1:  // KS3 row K1
				{
					key_front = KEY_DOWN;  // down
					break;
				}
			}
		}
#else
		if ((key_group1 == 32) && (key_group2 == 0))
		{
			key_front = KEY_MENU;  // menu
//			key_front = KEY_EPG;  // epg
		}
		if ((key_group1 == 00) && (key_group2 == 4))
		{
			key_front = KEY_EPG;  // epg
//			key_front = KEY_RECORD;  // record
		}
		if ((key_group1 == 04) && (key_group2 == 0))
		{
			key_front = KEY_HOME;  // res
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
			key_front = KEY_OK;  // ok
		}
		if ((key_group1 == 01) && (key_group2 == 0))
		{
			key_front = KEY_POWER;  // pwr
		}
#endif
		if (key_front > 0)
		{
			if (key_front == KEY_HOME)  // emergency reboot: press res 5 times
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
			dprintk(10, "Key: %d (%s)\n", key_front, __func__);
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
	dprintk(150, "%s <\n", __func__);
	return IRQ_HANDLED;
}

//----------------------------------------------------------------------------------
// handle queue

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
void button_bad_polling(void)
#else
void button_bad_polling(struct work_struct *ignored)
#endif
{
	dprintk(150, "%s >\n", __func__);

	while (bad_polling == 1)
	{
		msleep(10);
		if (stpio_get_pin(key_int) == 1)
		{
			ReadKey();
			key_front = 0;
			if ((key_group1 == 32) && (key_group2 == 0))
			{
				key_front = KEY_MENU;  // menu
			}
			if ((key_group1 == 00) && (key_group2 == 4))
			{
				key_front = KEY_EPG;  // epg
			}
			if ((key_group1 == 04) && (key_group2 == 0))
			{
				key_front = KEY_HOME;  // res
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
				key_front = KEY_OK;  // ok
			}
			if ((key_group1 == 01) && (key_group2 == 0))
			{
				key_front = KEY_POWER;  // pwr
			}
			if (key_front > 0)
			{
				if (key_front == KEY_HOME)
				{
					button_reset++;
					if (button_reset > 4)  // emergency reboot: press res 5 times
					{
						dprintk(10, "!!! Restart system !!!\n");
						button_reset = 0;
						kernel_restart(NULL);
					}
				}
				else
				{
					button_reset = 0;
				}
				input_report_key(button_dev, key_front, 1);
				input_sync(button_dev);
				dprintk(10, "Key: %d (%s)\n", key_front, __func__);
				msleep(250);
				input_report_key(button_dev, key_front, 0);
				input_sync(button_dev);
			}
		}
	}  // while
	dprintk(150, "%s <\n", __func__);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static DECLARE_WORK(button_obj, button_bad_polling, NULL);
#else
static DECLARE_WORK(button_obj, button_bad_polling);
#endif

static int button_input_open(struct input_dev *dev)
{
	dprintk(150, "%s >\n", __func__);
	dprintk(150, "%s <\n", __func__);
	return 0;
}

static void button_input_close(struct input_dev *dev)
{
	dprintk(150, "%s >\n", __func__);
	dprintk(150, "%s <\n", __func__);
}

//-------------------------------------------------------------------------------------------
// LED IOCTL
//-------------------------------------------------------------------------------------------

static int led_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct led_ioctl_data leddata;

	dprintk(150, "%s >\n", __func__);
	switch (cmd)
	{
		case VFDDCRAMWRITE:
		case VFDBRIGHTNESS:
		case VFDDISPLAYWRITEONOFF:
		case VFDICONDISPLAYONOFF:
		{
			copy_from_user(&leddata, (void *)arg, sizeof(struct led_ioctl_data));
		}
	}
	switch (cmd)
	{
		case VFDDCRAMWRITE:
		{
			return PT6958_ShowBuf(leddata.data, leddata.length);
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
		{
			pt6958_set_icon(leddata.data, leddata.length);
			break;
		}
		case VFDDRIVERINIT:
		{
			PT6958_setup();
			break;
		}
		default:
		{
			dprintk(1, "Unknown IOCTL %08x\n", cmd);
			break;
		}
	}
	dprintk(150, "%s <\n", __func__);
	return 0;
}

/******************************************************
 *
 * Write string on LED display (/dev/vfd)
 *
 */
static ssize_t led_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
	dprintk(150, "%s > Text = [%s], len= %d\n", __func__, buf, len);
	if (len == 0)
	{
		PT6958_ShowBuf("    ", 4);
	}
	else
	{
		// TODO: implement scrolling 
		PT6958_ShowBuf((char *)buf, len);
	}
	dprintk(150, "%s <\n", __func__);
	return len;
}

/******************************************************
 *
 * Read string from LED display (/dev/vfd)
 *
 */
static ssize_t led_read(struct file *filp, char *buf, size_t len, loff_t *off)
{
	dprintk(150, "%s >\n", __func__);
	// TODO: return current display string
	dprintk(150, "%s <\n", __func__);
	return len;
}

static int led_open(struct inode *inode, struct file *file)
{
	dprintk(150,"%s >\n", __func__);

	if (down_interruptible(&(led.sem)))
	{
		dprintk(1, "Interrupted while waiting for semaphore\n");
		return -ERESTARTSYS;
	}
	if (led.opencount > 0)
	{
		dprintk(1, "Device already opened\n");
		up(&(led.sem));
		return -EUSERS;
	}
	led.opencount++;
	up(&(led.sem));
	dprintk(150, "%s <\n", __func__);
	return 0;
}

static int led_close(struct inode *inode, struct file *file)
{
	dprintk(150, "%s >\n", __func__);
	led.opencount = 0;
	dprintk(150, "%s <\n", __func__);
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
	dprintk(150, "%s >\n", __func__);
	unregister_chrdev(LED_MAJOR, "led");
	stpio_free_irq(key_int);
	pt6958_free();
	input_unregister_device(button_dev);
	dprintk(150, "%s <\n", __func__);
}

static int __init led_module_init(void)
{
	int error;
	int res;

	dprintk(50, "Initialize ADB ITI-5800S front panel\n");

	dprintk(20, "probe PT6958\n");
	if (pt6958_init() == 0)
	{
		dprintk(1, "unable to init driver; abort\n");
		goto led_init_fail;
	}
	dprintk(10, "Registering character device %d\n", LED_MAJOR);
	if (register_chrdev(LED_MAJOR, "led", &led_fops))
	{
		dprintk(1, "register major %d failed\n", LED_MAJOR);
		goto led_init_fail;
	}
	sema_init(&(led.sem), 1);
	led.opencount = 0;

	PT6958_setup();

	dprintk(10, "Request STPIO %d,%d for key interrupt\n", PORT_KEY_INT, PIN_KEY_INT);
	key_int = stpio_request_pin(PORT_KEY_INT, PIN_KEY_INT, "Key_int_front", STPIO_IN);

	if (key_int == NULL)
	{
		dprintk(1, "Request STPIO for key_int failed; abort\n");
		goto led_init_fail;
	}
	stpio_set_pin(key_int, 1);

	dprintk(150, "stpio_flagged_request_irq IRQ_TYPE_LEVEL_LOW\n");
	stpio_flagged_request_irq(key_int, 0, (void *)fpanel_irq_handler, NULL, (long unsigned int)NULL);

	button_dev = input_allocate_device();
	if (!button_dev)
	{
		goto led_init_fail;
	}
	button_dev->name = button_driver_name;
	button_dev->open = button_input_open;
	button_dev->close = button_input_close;

	set_bit(EV_KEY, button_dev->evbit);
	set_bit(KEY_MENU, button_dev->keybit);
	set_bit(KEY_EPG, button_dev->keybit);
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
MODULE_AUTHOR("freebox");
MODULE_LICENSE("GPL");
// vim:ts=4
