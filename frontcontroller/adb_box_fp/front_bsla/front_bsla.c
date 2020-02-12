/*
 * front_bsla.c
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
 * Front panel driver for ADB ITI-5800SX, BSLA and BZZB models;
 * Button and display driver.
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
 * 201????? freebox         Initial version       
 * 20190730 Audioniek       Beginning of rewrire
 * 20160523 Audioniek       procfs added.
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

#include "front_bsla.h"
#include "../../vfd/utf.h"

// Global variables
static int buttoninterval = 350 /*ms*/;
static struct timer_list button_timer;

static char *button_driver_name = "nBox frontpanel buttons";
static struct input_dev *button_dev;

struct vfd_driver
{
	struct semaphore sem;
	int              opencount;
};

spinlock_t mr_lock = SPIN_LOCK_UNLOCKED;

static Global_Status_t status;

static struct vfd_driver vfd;
static int bad_polling  = 1;

short paramDebug = 0;

// LED states
static char ICON1       = 0; // record
static char ICON2       = 0; // power LED
static char ICON3       = 0; // at
static char ICON4       = 0; // alert

static char button_reset = 0;

static struct workqueue_struct *wq;

static unsigned char key_group1, key_group2;
static unsigned int  key_front = 0;

struct stpio_pin *key_int;
struct stpio_pin *pt6xxx_din;  // note: PT6302 & PT6958 Din pins are parallel connected
struct stpio_pin *pt6xxx_clk;  // note: PT6302 & PT6958 CLK pins are parallel connected
struct stpio_pin *pt6958_stb;  // note: PT6958 only, acts a chip select
struct stpio_pin *pt6302_cs;  // note: PT6302 only

typedef union
{
	struct
	{
		uint8_t addr: 4, cmd: 4;  // bits 0-3 = addr, bits 4-7 = cmd
	} dcram;  // PT6302_COMMAND_DCRAM_WRITE command

	struct
	{
		uint8_t addr: 3, reserved: 1, cmd: 4;  // bits 0-2 = addr, bit 3 = *, bits 4-7 = cmd
	} cgram;  // PT6302_COMMAND_CGRAM_WRITE command

	struct
	{
		uint8_t addr: 4, cmd: 4;  // bits 0-3 = addr, bits 4-7 = cmd
	} adram;  // PT6302_COMMAND_ADRAM_WRITE command

	struct
	{
		uint8_t port1: 1, port2: 1, reserved: 2, cmd: 4;  // bits 0 = port 1, bit 1 = port 2, bit 2-3 = *, bits 4-7 = cmd
	} port;  // PT6302_COMMAND_SET_PORTS command

	struct
	{
		uint8_t duty: 3, reserved: 1, cmd: 4;
	} duty;  // PT6302_COMMAND_SET_DUTY command

	struct
	{
		uint8_t digits: 3, reserved: 1, cmd: 4;
	} digits;  // PT6302_COMMAND_SET_DIGITS command

	struct
	{
		uint8_t onoff: 2, reserved: 2, cmd: 4;
	} light;  // PT6302_COMMAND_SET_LIGHT command

	uint8_t all;
} pt6302_command_t;

/****************************************************
 *
 * Routines to communicate with the PT6958 and PT6302
 *
 */

/****************************************************
 *
 * Free the PT6958 and PT6302 PIO pins
 *
 */
static void pt6958_free_pio_pins(void)
{
	if (pt6xxx_din)
	{
		stpio_free_pin(pt6xxx_din);
	}
	if (pt6xxx_clk)
	{
		stpio_free_pin(pt6xxx_clk);
	}
	if (pt6958_stb)
	{
		stpio_free_pin(pt6958_stb);
	}
	if (pt6302_cs)
	{
		stpio_free_pin(pt6302_cs);
	}
	dprintk(10, "PT6958 freed\n");
};

/****************************************************
 *
 * Initialize the PT6958 and PT6302 PIO pins
 *
 */
static unsigned char pt6958_init_pio_pins(void)
{
	dprintk(10, "Initialize PT6302 & PT6958 PIO pins\n");

// PT6302 Chip select
	dprintk(10, "Request STPIO %d,%d for PT6302 CS\n", PORT_CS, PIN_CS);
	pt6302_cs = stpio_request_pin(PORT_CS, PIN_CS, "PT6302_CS", STPIO_OUT);
	if (pt6302_cs == NULL)
	{
		dprintk(1, "Request for STPIO cs failed; abort\n");
		goto pt_init_fail;
	}
// PT6958 Strobe
	dprintk(10, "Request STPIO %d,%d for PT6958 STB\n", PORT_STB, PIN_STB);
	pt6958_stb = stpio_request_pin(PORT_STB, PIN_STB, "PT6958_STB", STPIO_OUT);
	if (pt6958_stb == NULL)
	{
		dprintk(1, "Request for STPIO stb failed; abort\n");
		goto pt_init_fail;
	}
// PT6302 & PT6958 Clock
	dprintk(10, "Request STPIO %d,%d for PT6302/PT6958 CLK\n", PORT_CLK, PIN_CLK);
	pt6xxx_clk = stpio_request_pin(PORT_CLK, PIN_CLK, "PT6958_CLK", STPIO_OUT);
	if (pt6xxx_clk == NULL)
	{
		dprintk(1, "Request for STPIO clk failed; abort\n");
		goto pt_init_fail;
	}
// PT6302 & PT6958 Data in
	dprintk(10, "Request STPIO %d,%d for PT6302/PT6958 Din\n", PORT_DIN, PIN_DIN);
	pt6xxx_din = stpio_request_pin(PORT_DIN, PIN_DIN, "PT6958_DIN", STPIO_BIDIR);
	if (pt6xxx_din == NULL)
	{
		dprintk(1, "Request STPIO din failed; abort\n");
		goto pt_init_fail;
	}
	stpio_set_pin(pt6302_cs, 1);  // set all involved PIO pins high
	stpio_set_pin(pt6958_stb, 1);
	stpio_set_pin(pt6xxx_clk, 1);
	stpio_set_pin(pt6xxx_din, 1);
	return 1;

pt_init_fail:
	pt6958_free_pio_pins();
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

	for (i = 0; i < 8; i++)  // 8 bits in a byte, LSB first
	{
		stpio_set_pin(pt6xxx_din, 1);  // data = 1 (key will pull down the pin if pressed)
		stpio_set_pin(pt6xxx_clk, 0);  // toggle
		udelay(VFD_Delay);
		stpio_set_pin(pt6xxx_clk, 1);  // clock pin
		udelay(VFD_Delay);
		data_in = (data_in >> 1) | (stpio_get_pin(pt6xxx_din) > 0 ? 0x80 : 0);
	}
	return data_in;
}

/****************************************************
 *
 * Write one byte to the PT6958 or PT6302
 *
 */
static void PT6xxx_SendChar(unsigned char Value)
{
	unsigned char i;

	for (i = 0; i < 8; i++)  // 8 bits in a byte, LSB first
	{
		stpio_set_pin(pt6xxx_din, Value & 0x01);  // write bit
		stpio_set_pin(pt6xxx_clk, 0);  // toggle
		udelay(VFD_Delay);
		stpio_set_pin(pt6xxx_clk, 1);  // clock pin
		udelay(VFD_Delay);
		Value >>= 1;
	}
}

/****************************************************
 *
 * Write one byte to the PT6302
 *
 */
static void PT6302_WriteChar(unsigned char Value)
{
	stpio_set_pin(pt6302_cs, 0);  // set chip select
	PT6xxx_SendChar(Value);
	stpio_set_pin(pt6302_cs, 1);  // deselect
	udelay(VFD_Delay);
}

/****************************************************
 *
 * Write one byte to the PT6958
 *
 */
static void PT6958_WriteChar(unsigned char Value)
{
	stpio_set_pin(pt6958_stb, 0);  // set strobe low
	PT6xxx_SendChar(Value);
	stpio_set_pin(pt6958_stb, 1);  // set strobe high
	udelay(VFD_Delay);
}

/****************************************************
 *
 * Write len bytes to the PT6958
 *
 */
static void PT6958_WriteData(unsigned char *data, unsigned int len)
{
	unsigned char i;

	stpio_set_pin(pt6958_stb, 0);  // set strobe low
	for (i = 0; i < len; i++)
	{
		PT6xxx_SendChar(data[i]);
	}
	stpio_set_pin(pt6958_stb, 1);  // set strobe high
	udelay(VFD_Delay);
}

/****************************************************
 *
 * Write len bytes to the PT6302
 *
 */
static void PT6302_WriteData(unsigned char *data, unsigned int len)
{
	unsigned char i;

	stpio_set_pin(pt6302_cs, 0);  // set chip select low
	for (i = 0; i < len; i++)
	{
		PT6xxx_SendChar(data[i]);
	}
	stpio_set_pin(pt6302_cs, 1);  // set chip select high
	udelay(VFD_Delay);
}


/******************************************************
 *
 * PT6958 LED functions
 *
 */

/****************************************************
 *
 * Read button data from the PT6958
 *
 */
static void ReadKey(void)
{
	spin_lock(&mr_lock);

	stpio_set_pin(pt6958_stb, 0);  // set strobe low (selects PT6958)
	PT6xxx_SendChar(DATA_SETCMD + READ_KEYD);  // send command 01000010 -> Read key data
	key_group1 = PT6958_ReadChar();  // Get SG1/KS1, SG2/KS2
	key_group2 = PT6958_ReadChar();  // Get SG3/KS3, SG4/KS4
//	key_group3 = PT6958_ReadChar();  // Get SG5/KS5, SG6/KS6  // not used
	stpio_set_pin(pt6958_stb, 1);  // set strobe high
	udelay(VFD_Delay);

	spin_unlock(&mr_lock);
}

/******************************************************
 *
 * Set text and dots on LED display, also set LEDs
 *
 */
static void PT6958_Show(unsigned char DIG1, unsigned char DIG2, unsigned char DIG3, unsigned char DIG4, unsigned char DOT1, unsigned char DOT2, unsigned char DOT3, unsigned char DOT4)
{
	spin_lock(&mr_lock);

	PT6958_WriteChar(DATA_SETCMD + 0); // Set test mode off, increment address and write data display mode  01xx0000b
	udelay(VFD_Delay);

	stpio_set_pin(pt6958_stb, 0);  // set strobe (latches command)

	PT6xxx_SendChar(ADDR_SETCMD + 0); // Command 2 address set, (start from 0)   11xx0000b
	if (DOT1 == 1)
	{
		DIG1 += 0x80;
	}
	PT6xxx_SendChar(DIG1);
	PT6xxx_SendChar(ICON1);  //power led: 00-off, 01-red, 02-green, 03-yellow

	if (DOT2 == 1)
	{
		DIG2 += 0x80;
	}
	PT6xxx_SendChar(DIG2);
	PT6xxx_SendChar(ICON2);  //rec led

	if (DOT3 == 1)
	{
		DIG3 += 0x80;
	}
	PT6xxx_SendChar(DIG3);
	PT6xxx_SendChar(ICON3);  //mail led (@)

	if (DOT4 == 1)
	{
		DIG4 += 0x80;
	}
	PT6xxx_SendChar(DIG4);
	PT6xxx_SendChar(ICON4);  // alert led

	stpio_set_pin(pt6958_stb, 1);
	udelay(VFD_Delay);

	spin_unlock(&mr_lock);
}

/******************************************************
 *
 * Set string on LED display, also set LEDs
 *
 * Accepts string lengths of 1, 2, 3, 4 & 8
 * First four bytes are text characters,
 * in case of length 8, last four are LED states (1=on,
 * off otherwise) for power, timer mail & alert
 */
static int PT6958_ShowBuf(unsigned char *kbuf, unsigned char len)
{
	unsigned char z1, z2, z3, z4, k1, k2, k3, k4;
//	unsigned char kbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int i = 0;
	int wlen = 0;

	if (kbuf[len - 1] == '\n')
	{
		kbuf[len - 1] = '\0';
		len--;
	}

#if 0
	/* eliminating UTF-8 chars first */
	while ((i < len) && (wlen < 8))
	{
		if (data[i] == '\n' || data[i] == 0)
		{
			dprintk(1, "[%s] BREAK CHAR detected (0x%X)\n", __func__, data[i]);
			break;
		}
		else if (data[i] < 0x20)
		{
			dprintk(2, "[%s] NON_PRINTABLE_CHAR '0x%X'\n", __func__, data[i]);
			i++;
		}
		else if (data[i] < 0x80)
		{
			kbuf[wlen] = data[i];
			DBG("[%s] ANSI_Char_Table '0x%X'\n", __func__, data[i]);
			wlen++;
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
				DBG("[%s] UTF_Char = 0x%X, index = %i", __func__, UTF_Char_Table[data[i] & 0x3f], i);
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
#endif
	if (len > 8)
	{
		len = 8;
	}
	k1 = 0;
	k2 = 0;
	k3 = 0;
	k4 = 0;
	z1 = 0x20;
	z2 = 0x20;
	z3 = 0x20;
	z4 = 0x20;
	if (len == 1)
	{
		z4 = kbuf[0];
	}
	if (len == 2)
	{
		z3 = kbuf[0];
		z4 = kbuf[1];
	}
	if (len == 3)
	{
		z2 = kbuf[0];
		z3 = kbuf[1];
		z4 = kbuf[2];
	}
	if (len == 4)
	{
		z1 = kbuf[0];
		z2 = kbuf[1];
		z3 = kbuf[2];
		z4 = kbuf[3];
	}
	if (len == 8)
	{
		z1 = kbuf[0];
		z2 = kbuf[1];
		z3 = kbuf[2];
		z4 = kbuf[3];
		if (kbuf[4] == '1')  // power LED (green only)
		{
			k1 = 1;
		}
		if (kbuf[5] == '1')  // record LED
		{
			k2 = 1;
		}
		if (kbuf[6] == '1')  // mail LED
		{
			k3 = 1;
		}
		if (kbuf[7] == '1')  // alert LED
		{
			k4 = 1;
		}	
	}
	PT6958_Show(ROM[z1], ROM[z2], ROM[z3], ROM[z4], k1, k2, k3, k4);
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

	spin_lock(&mr_lock);

	if (len == 5)
	{
		pos = kbuf[0];
		ico = kbuf[4];

		if (pos == 1)
		{
			ICON1 = ico;  // led power, 01-red 02-green
			pos = ADDR_SETCMD + 1;
		}
		if (pos == 2)
		{
			if (ico == 1)
			{
				ICON1 = 2;
				ico = 2;
			}
			else
			{
				ICON1 = 0;
				ico = 0;
			}
			pos = ADDR_SETCMD + 1;
		}  // led power: 01-red 02-green
		if (pos == 3)
		{
			ICON2 = ico;
			pos = ADDR_SETCMD + 3;
		}
		if (pos == 4)
		{
			ICON3 = ico;
			pos = ADDR_SETCMD + 5;
		}
		if (pos == 5)
		{
			ICON4 = ico;
			pos = ADDR_SETCMD + 7;
		}
		PT6958_WriteChar(DATA_SETCMD + ADDR_FIX);  // Set command, normal mode, fixed address, write date to display mode 01xx0100b
		udelay(VFD_Delay);

		stpio_set_pin(pt6958_stb, 0);  // set strobe low
		PT6xxx_SendChar(pos);  // address set  11xx????b
		PT6xxx_SendChar(ico);
		stpio_set_pin(pt6958_stb, 1);  // set strobe high
		udelay(VFD_Delay);
	}
	spin_unlock(&mr_lock);
}

/****************************************************
 *
 * Initialize the PT6958
 *
 */
static void PT6958_setup(void)
{
	unsigned char i;

	PT6958_WriteChar(DATA_SETCMD);  // Command 1, increment address, normal mode
	udelay(VFD_Delay);

	stpio_set_pin(pt6958_stb, 0);  // set strobe low
	PT6xxx_SendChar(ADDR_SETCMD);  // Command 2, RAM address = 0
	for (i = 0; i < 10; i++)  // 10 bytes
	{
		PT6xxx_SendChar(0);  // clear display RAM (all segments off)
	}
	stpio_set_pin(pt6958_stb, 1);  // set strobe high
	udelay(VFD_Delay);
	PT6958_WriteChar(DISP_CTLCMD + DISPLAY_ON + 4);  // Command 3, display control, (Display ON), brightness 11/16 10xx1100b

	ICON1 = 0;
	ICON1 = 2;  //power LED green
	ICON2 = 0;
	ICON3 = 0;
	PT6958_Show(0x20, 0x20, 0x20, 0x20, 0, 0, 0, 0);  // blank LED display
}

/******************************************************
 *
 * Set brightness of LED display
 *
 */
static void pt6958_set_brightness(int level)
{
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
}

/******************************************************
 *
 * Switch LED display on or off
 *
 */
static void pt6958_set_light(int onoff)
{
	spin_lock(&mr_lock);

#if 0
	if (onoff == 1)
	{
		PT6958_WriteChar(DISP_CTLCMD + DISPLAY_ON + 4);  // Command 3, display control, (Display ON) brigness 11/16
	}
	else
	{
		PT6958_WriteChar(DISP_CTLCMD);  // Command 3, display control, (Display OFF) 10xx0000b
	}
#else
	PT6958_WriteChar(DISP_CTLCMD + (onoff ? DISPLAY_ON + 4 : 0));
#endif
	spin_unlock(&mr_lock);
}

/******************************************************
 *
 * PT6302 VFD functions
 *
 */

/****************************************************
 *
 * Write PT6302 DCRAM (display text)
 *
 */
static int pt6302_write_dcram(unsigned char addr, unsigned char *text, unsigned char len)
{
	unsigned char    kbuf[16] = {0x20};
	int              i = 0;
	int              j = 0;
	pt6302_command_t cmd;

	uint8_t          wdata[20];
	int              wlen = 0;

	/* eliminating UTF-8 chars first ???*/
	if (len == 0)  // if length is zero,
	{
		return 0; // do nothing
	}
	// Remove trailing terminator or LF
//	if (text[len] == 0x0 || text[len] == 0x0a)
	dprintk(10, "%s > len = %d Text(%d - 1) = [%d] (0x%02x)\n", __func__, len, len - 1, text[len - 1], text[len - 1]);
	if (text[len - 1] == 0x0a)
	{
		text[len - 1] = 0x0;
		len--;
	} 
	dprintk(10, "%s > len = %d Text(%d) = [%d] (0x%02x)\n", __func__, len, len, text[len], text[len]);
	dprintk(10, "%s > Text: [%s] (len = %d)\n", __func__, text, len);
	
#if 0  // PT6958 stuff!, also hangs forever
	while ((i < len) && (wlen < 16))
	{
		PT6958_WriteChar(0x80);  // Command 3 display control, (Display OFF) 10xx0000b
	}
#endif
	memcpy(kbuf, text, len);

	spin_lock(&mr_lock);

	cmd.dcram.cmd  = PT6302_COMMAND_DCRAM_WRITE;
	cmd.dcram.addr = (addr & 0x0f);  // get address (= character position, 0 = rightmost!)

	wdata[0] = cmd.all;
//	dprintk(1, "wdata[0] = 0x%02x\n", wdata[0]);

#if 0
	dprintk(10, "%s Center text\n", __func__);
	/* Center text */
	j = 0;

	if (wlen < 15)  // handle leading spaces
	{
		dprintk(10, "%s Handle leading spaces\n", __func__);
		for (i = 0; i < ((16 - wlen) / 2); i++)
		{
			wdata[i + 1] = pt6302_007_rom_table[0x20];  // set characters to write
			j += 1;
		}
	}
	for (i = 0; i < wlen; i++)  // handle text
	{
		dprintk(10, "%s Handle text\n", __func__);
		wdata[i + 1 + j] = pt6302_007_rom_table[kbuf[(wlen - 1) - i]];
	}

	if (wlen < 16)  // handle trailing spaces
	{
		dprintk(10, "%s Handle trailing spaces\n", __func__);
		for (i = j + wlen; i < 16; i++)
		{
			wdata[i + 1] = pt6302_007_rom_table[0x20];  // set characters to write
		}
	}
#else
	for (i = 0; i < 16; i++) // move display text in reverse order to data to write
	{
		wdata[1 + (15 - i)] = kbuf[i];
	}
//	memcpy(wdata + 1, kbuf, 16);
#endif
	PT6302_WriteData(wdata, 1 + 16); //j00zek: cmd + kbuf size
	spin_unlock(&mr_lock);
	dprintk(10, "%s <\n", __func__);
	return 0;
}

/****************************************************
 *
 * Write PT6302 ADRAM (set symbol data)
 *
 */
static int pt6302_write_adram(unsigned char addr, unsigned char *data, unsigned char len)
{
	pt6302_command_t cmd;
	uint8_t          wdata[20] = {0x0};
	int              i = 0;

	dprintk(10,"%s >\n", __func__);

	spin_lock(&mr_lock);

	cmd.adram.cmd  = PT6302_COMMAND_ADRAM_WRITE;
	cmd.adram.addr = (addr & 0xf);

	wdata[0] = cmd.all;
	dprintk(1, "wdata[0] = 0x%02x\n", wdata[0]);
	if (len > 16 - addr)
	{
		len = 16 - addr;
	}
	for (i = 0; i < len; i++)
	{
		wdata[i + 1] = data[i] & 0x03;
	}
	PT6302_WriteData(wdata, len + 1);
	spin_unlock(&mr_lock);
	dprintk(10,"%s <\n", __func__);
	return 0;
}

/****************************************************
 *
 * Write PT6302 CGRAM (set symbol pixel data)
 *
 * addr = symbol number (0..7)
 * data = symbol data (5 bytes for each column, column 0 (lefmost) first, MSbit ignored)
 * len  = number of symbols to define pixels of (max 9 - addr)
 *
 */
static int pt6302_write_cgram(unsigned char addr, unsigned char *data, unsigned char len)
{
	pt6302_command_t cmd;
	uint8_t          wdata[20];
	int              i = 0, j = 0;

	dprintk(10,"%s >\n", __func__);

	if (len == 0)
	{
		return 0;
	}
	spin_lock(&mr_lock);

	cmd.cgram.cmd  = PT6302_COMMAND_CGRAM_WRITE;
	cmd.cgram.addr = (addr & 0x7);

	wdata[0] = cmd.all;
	dprintk(1, "wdata[0] = 0x%02x\n", wdata[0]);
	if (len > 8 - addr)
	{
		len = 8 - addr;
	}
	for (i = 0; i < len; i++)
	{
		for (j = 0; j < 6; j++)
		{
			wdata[i * 5 + j + 1] = data[i * 5 + j];
		}
	}
	PT6302_WriteData(wdata, len * 5 + 1);
	spin_unlock(&mr_lock);
	dprintk(10,"%s <\n", __func__);
	return 0;
}

/******************************************************
 *
 * Set brightness of VFD display
 *
 */
static void pt6302_set_brightness(int level)
{
	pt6302_command_t cmd;

	dprintk(150, "%s >\n", __func__);

	spin_lock(&mr_lock);

	if (level < PT6302_DUTY_MIN)
	{
		level = PT6302_DUTY_MIN;
	}
	if (level > PT6302_DUTY_MAX)
	{
		level = PT6302_DUTY_MAX;
	}
	cmd.duty.cmd  = PT6302_COMMAND_SET_DUTY;
	cmd.duty.duty = level;

	PT6302_WriteChar(cmd.all);
	spin_unlock(&mr_lock);
	dprintk(150,"%s <\n", __func__);
}

/********************************************************
 *
 * Set number of characters on VFD display
 *
 * Note: sets num + 7 as width, but num = 0 set width 16 
 */
static void pt6302_set_digits(int num)
{
	pt6302_command_t cmd;

	dprintk(10, "%s >\n", __func__);

	spin_lock(&mr_lock);

	if (num < PT6302_DIGITS_MIN)
	{
		num = PT6302_DIGITS_MIN;
	}
	if (num > PT6302_DIGITS_MAX)
	{
		num = PT6302_DIGITS_MAX;
	}
	num = (num == PT6302_DIGITS_MAX) ? 0 : (num - PT6302_DIGITS_OFFSET);

	cmd.digits.cmd    = PT6302_COMMAND_SET_DIGITS;
	cmd.digits.digits = num;

	PT6302_WriteChar(cmd.all);
	spin_unlock(&mr_lock);
	dprintk(10,"%s <\n", __func__);
}

/******************************************************
 *
 * Set VFD display on, off or all segments lit
 *
 */
static void pt6302_set_light(int onoff)
{
	pt6302_command_t cmd;

	dprintk(10, "%s >\n", __func__);

	spin_lock(&mr_lock);

	if (onoff < PT6302_LIGHT_NORMAL || onoff > PT6302_LIGHT_ON)
	{
		onoff = PT6302_LIGHT_ON;
	}
	cmd.light.cmd   = PT6302_COMMAND_SET_LIGHT;
	cmd.light.onoff = onoff;
	PT6302_WriteChar(cmd.all);
	spin_unlock(&mr_lock);
	dprintk(10, "%s <\n", __func__);
}

/******************************************************
 *
 * Set binary output pins of PT6302
 *
 */
static void pt6302_set_port(int port1, int port2)
{
	pt6302_command_t cmd;

	dprintk(100, "%s >\n", __func__);

	spin_lock(&mr_lock);

	cmd.port.cmd   = PT6302_COMMAND_SET_PORTS;
	cmd.port.port1 = (port1) ? 1 : 0;
	cmd.port.port2 = (port2) ? 1 : 0;

	PT6302_WriteChar(cmd.all);
	spin_unlock(&mr_lock);
	dprintk(100,"%s <\n", __func__);
}

/******************************************************
 *
 * Setup VFD display as 16 characters, normal display
 *
 */
static void pt6302_setup(void)
{
	unsigned char buf[40] = {0x0};  // init for ADRAM & CGRAM

	dprintk(10, "%s >\n", __func__);

	pt6302_set_port(1, 0);
	pt6302_set_digits(PT6302_DIGITS_MAX);
	pt6302_set_brightness(PT6302_DUTY_MIN);
	pt6302_set_light(PT6302_LIGHT_NORMAL);
	pt6302_write_dcram(0x00, "                ", 16);  // clear display
	pt6302_write_adram(0x00, buf, 16);  // clear ADRAM
	pt6302_write_cgram(0x00, buf, 40);  // clear CGRAM
	dprintk(10,"%s <\n", __func__);
}

//#define button_polling
//#define button_interrupt
#define	button_interrupt2

#if defined(button_interrupt2)
#warning Compiling with button_interrupt2
//----------------------------------------------------------------------------------
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
//void fpanel_irq_handler(void *dev1,void *dev2)
irqreturn_t fpanel_irq_handler(void *dev1, void *dev2)
{
	disable_irq(FPANEL_PORT2_IRQ);

	if ((stpio_get_pin(key_int) == 1) && (key_front == 0))
	{
		ReadKey();
		key_front = 0;

#if 0
		switch (key_group2)
		{
			case 0:
			{
				switch (key_group1)
				{
					case 64:
					{
						key_front = KEY_RIGHT;  // right
//						break;
					}
					case 32:
					{
						key_front = KEY_MENU;  // menu ?? not present on a BSLA
//						break;
					}
					case 16:
					{
						key_front = KEY_UP;  // up
//						break;
					}
					case 4:
					{
						key_front = KEY_HOME;  // res
//						break;
					}
					case 2:
					{
						key_front = KEY_OK;  // ok
//						break;
					}
					case 1:
					{
							key_front = KEY_POWER;  // pwr
//						break;
					}
					case 0:
					{
						key_front = KEY_POWER;  // pwr
//						break;
					}
				}
			}
			case 4:
			case 2:
			case 0:
			{
				if (key_group1 == 0)
				{
					switch (key_group2)
					{
						case 0:
						{
							key_front = KEY_DOWN;  // down
//							break;
						}
						case 2:
						{
							key_front = KEY_LEFT;  // left
//							break;
						}
						case 4:
						{
							key_front = KEY_EPG;  // epg
//							break;
						}
					}
				}
			}
		}
#else
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
			dprintk(10, "Key: %d\n", key_front);
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
	return IRQ_HANDLED;
}
#endif

#if 0
//#if defined(button_interrupt)
//#warning !!!!!!!!! button_interrupt !!!!!!!!

static void button_delay(unsigned long dummy)
{
	if (button_reset == 100)
	{
		kernel_restart(NULL);
	}
	//pinreset = stpio_request_pin( 3,2, "pinreset", STPIO_OUT );
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

irqreturn_t fpanel_irq_handler(int irq, void *dev_id)
{
	disable_irq(FPANEL_PORT2_IRQ);

	if ((stpio_get_pin(key_int) == 1) && (key_front == 0))
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
					button_reset = 100;
				}
			}
			else
			{
				button_reset = 0;
			}
			input_report_key(button_dev, key_front, 1);
			input_sync(button_dev);
			dprintk(10, "Key: %d\n", key_front);
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
	return IRQ_HANDLED;
}
#endif

#if 0
//#if defined(button_polling)
//#warning !!!!!!!!! button_queue !!!!!!!!
#endif

//----------------------------------------------------------------------------------
// version queue

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
void button_bad_polling(void)
#else
void button_bad_polling(struct work_struct *ignored)
#endif
{
	while (bad_polling == 1)
	{
		msleep(5);
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
				dprintk(10, "Key: %d\n", key_front);
				msleep(250);
				input_report_key(button_dev, key_front, 0);
				input_sync(button_dev);
			}
		}
	}  // while
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static DECLARE_WORK(button_obj, button_bad_polling, NULL);
#else
static DECLARE_WORK(button_obj, button_bad_polling);
#endif

static int button_input_open(struct input_dev *dev)
{
#if 0
//#if defined(button_polling)
	wq = create_workqueue("button");
	if (queue_work(wq, &button_obj))
	{
		dprintk(10, "Button queue_work successful...\n");
	}
	else
	{
		dprintk(10, "Button queue_work not successful; exiting...\n");
		return 1;
	}
#endif
	return 0;
}

static void button_input_close(struct input_dev *dev)
{
#if 0
//#if defined(button_polling)
	bad_polling = 0;
	msleep(55);
	bad_polling = 1;

	if (wq)
	{
		destroy_workqueue(wq);
		dprintk(10, "Button workqueue destroyed\n");
	}
#endif
}

//-------------------------------------------------------------------------------------------
// VFD
//-------------------------------------------------------------------------------------------

static int vfd_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct vfd_ioctl_data vfddata;

	switch (cmd)
	{
		case VFDDCRAMWRITE:
		case VFDBRIGHTNESS:
		case VFDDISPLAYWRITEONOFF:
		case VFDICONDISPLAYONOFF:
		{
			copy_from_user(&vfddata, (void *)arg, sizeof(struct vfd_ioctl_data));
		}
	}
	switch (cmd)
	{
		case VFDDCRAMWRITE:
		{
			pt6302_write_dcram(vfddata.address, vfddata.data, vfddata.length);
			return PT6958_ShowBuf(vfddata.data, vfddata.length);
		}
		case VFDBRIGHTNESS:
		{
			pt6302_set_brightness(vfddata.address);
			pt6958_set_brightness(vfddata.address);
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			pt6302_set_light(vfddata.address);
			pt6958_set_light(vfddata.address);
			break;
		}
		case VFDICONDISPLAYONOFF:
		{
			pt6958_set_icon(vfddata.data, vfddata.length);
			break;
		}
		case VFDDRIVERINIT:
		{
			PT6958_setup();
			pt6302_setup();
			break;
		}
		default:
		{
			dprintk(1, "Unknown IOCTL %08x\n", cmd);
			break;
		}
	}
	return 0;
}

/******************************************************
 *
 * Write string on VFD display (/dev/vfd)
 *
 */
static ssize_t vfd_write_bsla(struct file *filp, const char *buf, size_t len, loff_t *off)
{
	pt6302_write_dcram(0, "                ", 16);  // blank display
	if (len > 0)
	{
		//TODO: implement scrolling 
		pt6302_write_dcram(0, (char *)buf, len);
	}
	return len;
}

/******************************************************
 *
 * Read string from VFD display (/dev/vfd)
 *
 */
static ssize_t vfd_read_bsla(struct file *filp, char *buf, size_t len, loff_t *off)
{
	// TODO: return current display string
	return len;
}

static int vfd_open(struct inode *inode, struct file *file)
{

	dprintk(150,"%s >\n", __func__);

	if (down_interruptible(&(vfd.sem)))
	{
		dprintk(1, "Interrupted while waiting for semaphore\n");
		return -ERESTARTSYS;
	}
	if (vfd.opencount > 0)
	{
		dprintk(1, "Device already opened\n");
		up(&(vfd.sem));
		return -EUSERS;
	}
	vfd.opencount++;
	up(&(vfd.sem));
	return 0;
}

static int vfd_close(struct inode *inode, struct file *file)
{
	dprintk(150, "%s >\n", __func__);
	vfd.opencount = 0;
	return 0;
}

static struct file_operations vfd_fops =
{
	.owner   = THIS_MODULE,
	.ioctl   = vfd_ioctl,
	.write   = vfd_write_bsla,
	.read    = vfd_read_bsla,
	.open    = vfd_open,
	.release = vfd_close
};

static void __exit vfd_module_exit(void)
{
	unregister_chrdev(VFD_MAJOR, "vfd");
#if defined(button_interrupt2)
	stpio_free_irq(key_int);
#endif
	pt6958_free_pio_pins();
	input_unregister_device(button_dev);
}

static int __init vfd_module_init(void)
{
	int error;
	int res;

	dprintk(10, "nBox BSLA/BZZB front panel initializing\n");

	dprintk(10, "Probe PT6958 + PT6302\n");
	if (pt6958_init_pio_pins() == 0)
	{
		dprintk(1, "Unable to initinitialize driver; abort\n");
		goto vfd_init_fail;
	}
	dprintk(10, "Register character device %d\n", VFD_MAJOR);
	if (register_chrdev(VFD_MAJOR, "vfd", &vfd_fops))
	{
		dprintk(1, "Registering major %d failed\n", VFD_MAJOR);
		goto vfd_init_fail;
	}
	sema_init(&(vfd.sem), 1);
	vfd.opencount = 0;

	PT6958_setup();
	pt6302_setup();

	dprintk(10, "Request STPIO %d,%d for key interrupt\n", PORT_KEY_INT, PIN_KEY_INT);
	key_int = stpio_request_pin(PORT_KEY_INT, PIN_KEY_INT, "Key_int_front", STPIO_IN);

	if (key_int == NULL)
	{
		dprintk(1, "Request STIO for key_int failed; abort\n");
		goto vfd_init_fail;
	}
	stpio_set_pin(key_int, 1);

#if defined(button_interrupt2)
	dprintk(10, "> stpio_flagged_request_irq IRQ_TYPE_LEVEL_LOW\n");
	stpio_flagged_request_irq(key_int, 0, (void *)fpanel_irq_handler, NULL, (long unsigned int)NULL);
	dprintk(10, "< stpio_flagged_request_irq IRQ_TYPE_LEVEL_LOW\n");
#endif

	button_dev = input_allocate_device();
	if (!button_dev)
	{
		goto vfd_init_fail;
	}
	button_dev->name = button_driver_name;
	button_dev->open = button_input_open;
	button_dev->close = button_input_close;

	set_bit(EV_KEY, button_dev->evbit);
//	set_bit(KEY_MENU, button_dev->keybit);
	set_bit(KEY_EPG, button_dev->keybit);
	set_bit(KEY_RECORD, button_dev->keybit);
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
		dprintk(1, "Request input_register_device; abort\n");
		goto vfd_init_fail;
	}

#if defined(button_interrupt)
	status.pio_addr = 0;
	status.pio_addr = (unsigned long)ioremap(PIO2_BASE_ADDRESS, (unsigned long)PIO2_IO_SIZE);
	if (!status.pio_addr)
	{
		dprintk(10, "Button PIO map: FATAL: Memory allocation error\n");
		goto vfd_init_fail;
	}
	status.button = BUTTON_RELEASED;

	ctrl_outl(BUTTON_PIN, status.pio_addr + PIO_CLR_PnC0);
	ctrl_outl(BUTTON_PIN, status.pio_addr + PIO_CLR_PnC1);
	ctrl_outl(BUTTON_PIN, status.pio_addr + PIO_CLR_PnC2);

	ctrl_outl(BUTTON_RELEASED, status.pio_addr + PIO_PnCOMP);
	ctrl_outl(BUTTON_PIN, status.pio_addr + PIO_PnMASK);

	res = request_irq(FPANEL_PORT2_IRQ, fpanel_irq_handler, IRQF_DISABLED, "button_irq" , NULL);
	if (res < 0)
	{
		dprintk(10, "Button IRQ: FATAL: Could not request interrupt\n");
		goto vfd_init_fail;
	}
#endif
	dprintk(100, "Front panel nBox BSLA/BZZB initialization successful\n");
	return 0;

vfd_init_fail:
	vfd_module_exit();
	return -EIO;
}

module_init(vfd_module_init);
module_exit(vfd_module_exit);

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled, >0=enabled (level)");

MODULE_DESCRIPTION("nBox PT6958 + PT6302 frontpanel driver");
MODULE_AUTHOR("freebox");
MODULE_LICENSE("GPL");
// vim:ts=4
