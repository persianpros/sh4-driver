/*****************************************************************************
 *
 * proton.c
 *
 * (c) 2010 Spider-Team
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
 *****************************************************************************
 *
 * Spiderbox HL101 frontpanel driver.
 *
 * Devices:
 *	- /dev/vfd (vfd ioctls and read/write function)
 *	- /dev/rc  (reading of key events)
 *
 *****************************************************************************
 *
 * The front panel is constructed around a Princeton PT6311-LQ display driver
 * IC of which the generic switch inputs and the LED outputs are not used.
 *
 * The driver IC is controlled using three PIO driven lines ('bit-banged').
 *
 * The VFD display used is very elaborate as it features:
 * - Clock area, Four 7-segment digits with a colon but without periods;
 * - Text area, Eight 15 segment digits, without periods;
 * - Icons, 45 in total (not counting the colon in the clock area).
 *
 * Possible grid assigment:
 * 00 - 07: Text area (0 = leftmost)
 * 08 - 09: clock area (9 = hours, 8 = minutes)
 * 10 - 12: icons
 *
 * As the controller is to be regarded as non-intelligent and has no time
 * capabilities, the receiver cannot be woken up from deep standby by it. As
 * As a result, the HL101 cannot make timed recordings from deep standby in
 * the normal Enigma2/Neutrino configuration.
 *
 *****************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------
 * 201????? Spider Team?    Initial version.
 * 20200731 Audioniek       Default GMT offset changed to plus one hour.
 * 20200804 Audioniek       Brightness control fixed.
 * 20200805 Audioniek       Display on/off fixed.
 * 20200805 Audioniek       Icons working.
 * 20200814 Audioniek       Fix issues with ICON_ALL.
 *
 *****************************************************************************/

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/termbits.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 17)
#include <linux/stm/pio.h>
#else
#include <linux/stpio.h>
#endif

#include "proton.h"
#include "utf.h"

#include <linux/device.h>  /* class_creatre */
#include <linux/cdev.h>    /* cdev_init */

// Default module parameters
static short paramDebug = 0;
static int gmt_offset = 3600;
static char *gmt = "+3600";  // GMT offset is plus one hour (Europe) as default

static tFrontPanelOpen FrontPanelOpen [LASTMINOR];

/* I make this static, I think if there are more
 * then 100 commands to transmit something went
 * wrong ... point of no return
 */
struct transmit_s transmit[cMaxTransQueue];
struct receive_s receive[cMaxReceiveQueue];

static int transmitCount = 0;
static int receiveCount = 0;

static wait_queue_head_t   wq;

/* waiting retry counter, to stop waiting on ack */
static int waitAckCounter = 0;
static int timeoutOccured = 0;
static int dataReady = 0;

// Place to store previous data
static struct saved_data_s lastdata;

/* last received ioctl command. we dont queue answers
 * from "getter" requests to the fp. they are protected
 * by a semaphore and the threads goes to sleep until
 * the answer has been received or a timeout occurs.
 */
int writePosition = 0;
int readPosition = 0;
unsigned char receivedData[BUFFERSIZE];

unsigned char str[64];
static SegAddrVal_t VfdSegAddr[15];

int vfd_brightness;  // current display brightness
int vfd_show;  // light on/off state
int scroll_flag = 0;

struct rw_semaphore vfd_rws;

struct semaphore write_sem;
struct semaphore rx_int_sem;  /* unused until the irq works */
struct semaphore transmit_sem;
struct semaphore receive_sem;
struct semaphore key_mutex;

static struct semaphore display_sem;

// draw thread
#define THREAD_STATUS_RUNNING 0
#define THREAD_STATUS_STOPPED 1
#define THREAD_STATUS_INIT 2
static struct task_struct *draw_task;
static int draw_thread_status = THREAD_STATUS_STOPPED;

/**************************************************
 *
 * Character table, upper case letters
 * (for 15-segment main text field)
 *
 *
 * Character segment layout:

     aaaaaaa
    fh  j  kb
    f h j k b
    f  hjk  b
     gggimmm
    e  rpn  c
    e r p n c
    er  p  nc
     ddddddd
 
                 7 6 5 4 3 2 1 0 
  address 0 8bit g i m c r p n e
  address 1 7bit   d a b f k j h

  segment a = A1  32 0x20
  segment b = A1  16 0x10
  segment c = A0  16 0x10
  segment d = A1  64 0x40
  segment e = A0   1 0x01
  segment f = A1   8 0x08
  segment g = A0 128 0x80

  segment h = A1   1 0x01
  segment i = A0  64 0x40
  segment j = A1   2 0x02
  segment k = A1   4 0x04
  segment m = A0  32 0x20
  segment n = A0   2 0x02
  segment p = A0   4 0x04
  segment r = A0   8 0x08
*/
unsigned char ASCII[48][2] =
{  //              Char offs ASCII
	{0xF1, 0x38},  // A 0x00 0x41
	{0x74, 0x72},  // B 0x01 0x42
	{0x01, 0x68},  // C 0x02 0x43
	{0x54, 0x72},  // D 0x03 0x44
	{0xE1, 0x68},  // E 0x04 0x45
	{0xE1, 0x28},  // F 0x05 0x46
	{0x71, 0x68},  // G 0x06 0x47
	{0xF1, 0x18},  // H 0x07 0x48
	{0x44, 0x62},  // I 0x08 0x49
	{0x45, 0x22},  // J 0x09 0x4A
	{0xC3, 0x0C},  // K 0x0A 0x4B
	{0x01, 0x48},  // L 0x0B 0x4D
	{0x51, 0x1D},  // M 0x0C 0x4E
	{0x53, 0x19},  // N 0x0D 0x4F
	{0x11, 0x78},  // O 0x0E 0x4F
	{0xE1, 0x38},  // P 0x0F 0x50
	{0x13, 0x78},  // Q 0x10 0x51
	{0xE3, 0x38},  // R 0x11 0x52
	{0xF0, 0x68},  // S 0x12 0x53
	{0x44, 0x22},  // T 0x13 0x54
	{0x11, 0x58},  // U 0x14 0x55
	{0x49, 0x0C},  // V 0x15 0x56
	{0x5B, 0x18},  // W 0x16 0x57
	{0x4A, 0x05},  // X 0x17 0x58
	{0x44, 0x05},  // Y 0x18 0x59
	{0x48, 0x64},  // Z 0x19 0x5A
	{0x01, 0x68},  // [ 0x1A 0x5B
	{0x42, 0x01},  // \ 0x1B 0x5C 
	{0x10, 0x70},  // ] 0x1C 0x5D
	{0x43, 0x09},  //   0x1D 0x5E (back quote)
	{0x00, 0x40},  // _ 0x1E 0x5F

#define OFFSET_ASCII1 (0x2a - 0x1f) // -> 0x0b
	{0xEE, 0x07},  // * 0x1F 0x2a
	{0xE4, 0x02},  // + 0x20 0x2b
	{0x04, 0x00},  //,  0x21 0x2c
	{0xE0, 0x00},  // - 0x22 0x2d
	{0x04, 0x00},  // . 0x23 0x2e
	{0x48, 0x04},  // / 0x24 0x2f
	{0x11, 0x78},  // 0 0x25 0x30
	{0x10, 0x14},  // 1 0x26 0x31
	{0xE1, 0x70},  // 2 0x27 0x32
	{0xF0, 0x70},  // 3 0x28 0x33
	{0xF0, 0x18},  // 4 0x29 0x34
	{0xF0, 0x68},  // 5 0x2A 0x35
	{0xF1, 0x68},  // 6 0x2B 0x36
	{0x10, 0x30},  // 7 0x2C 0x37
	{0xF1, 0x78},  // 8 0x2D 0x38
	{0xF0, 0x78},  // 9 0x2E 0x39
#define OFFSET_ASCII2 0x2f
	{0x00, 0x00}   //   0x2F, End of table, code for a space
};

/**************************************************
 *
 * Character table, digits
 * (for 7-segment main text field)
 *
 * First byte : even digits
 * Second byte: odd digits
 *
 * Character layout:
 *
 *      aaaaaa
 *     f      b
 *     f      b
 *     f      b
 *      gggggg
 *     e      c
 *     e      c
 *     e      c
 *      dddddd
 *
 * Bit allocation:
 *
 * Seg     Odd  Even
 *  a       6    6    (0x40)    
 *  b       5    5    (0x20)
 *  c       2    1    (0x04 / 0x02)
 *  d       0    0    (0x01)
 *  e       1    2    (0x02 / 0x02)
 *  f       4    4    (0x10)
 *  g       3    3    (0x08)
 */
unsigned char NumLib[10][2] =
{ //                     even       odd
  //                    abgfced   abfgecd
	{0x77, 0x77},  // {01110111, 01110111}  0
	{0x24, 0x22},  // {00100100, 00010010}  1
	{0x6B, 0x6D},  // {01101011, 01101101}  2
	{0x6D, 0x6B},  // {01101101, 01101011}  3
	{0x3C, 0x3A},  // {00111100, 00111010}  4
	{0x5D, 0x5B},  // {01011101, 01011011}  5
	{0x5F, 0x5F},  // {01011111, 01011111}  6
	{0x64, 0x62},  // {01100100, 01100010}  7
	{0x7F, 0x7F},  // {01111111, 01111111}  8
	{0x7D, 0x7B}   // {01111101, 01111011}  9
};

/**************************************************
 *
 * Bit-bang routines to drive PT6311
 *
 */
// Set Pio data pin direction
static int PROTONfp_Set_PIO_Mode(PIO_Mode_t Mode_PIO)
{
	int ret = 0;

	if (Mode_PIO == PIO_Out)
	{
		stpio_configure_pin(cfg.data, STPIO_OUT);
	}
	else if (Mode_PIO == PIO_In)
	{
		stpio_configure_pin(cfg.data, STPIO_IN);
	}
	return ret;
}

// Read one byte from PT6311 (key data)
unsigned char PROTONfp_RD(void)
{
	int i;
	unsigned char val = 0, data = 0;

	down_read(&vfd_rws);

	PROTONfp_Set_PIO_Mode(PIO_In);  // switch data pin to input
	for (i = 0; i < 8; i++)
	{
		val >>= 1;
		VFD_CLK_CLR();
		udelay(1);
		data = stpio_get_pin(cfg.data);
		VFD_CLK_SET();
		if (data)
		{
			val |= 0x80;
		}
		VFD_CLK_SET();
		udelay(1);
	}
	udelay(1);
	PROTONfp_Set_PIO_Mode(PIO_Out);  // switch data pin to output
	up_read(&vfd_rws);
	return val;
}

// Write one byte to PT6311
static int VFD_WR(unsigned char data)
{
	int i;

	down_write(&vfd_rws);

	for (i = 0; i < 8; i++)
	{
		VFD_CLK_CLR();
		if (data & 0x01)
		{
			VFD_DATA_SET();
		}
		else
		{
			VFD_DATA_CLR();
		}
		VFD_CLK_SET();
		data >>= 1;
	}
	up_write(&vfd_rws);
	return 0;
}

// Initialize PT6311 segment addressing
void VFD_Seg_Addr_Init(void)
{
	unsigned char i, addr = PT6311_ADDR_SET;  // set address command

	for (i = 0; i < 13; i++)  // grid counter
	{
		VfdSegAddr[i + 1].CurrValue1 = 0;
		VfdSegAddr[i + 1].CurrValue2 = 0;
		VfdSegAddr[i + 1].Segaddr1 = addr;
		VfdSegAddr[i + 1].Segaddr2 = addr + 1;
		addr += 3;
	}
}

static int VFD_Seg_Dig_Seg(unsigned char dignum, SegNum_t segnum, unsigned char val)
{
//	dignum: gridnumber (1 - 13)
//	segnum: byte indicator: first or second
//	val: segment data to write
	unsigned char addr = 0;

	if (segnum < 0 && segnum > 1)
	{
		dprintk(1, "bad segnum parameter!\n");
		return -1;
	}
	VFD_CS_CLR();
	if (segnum == SEGNUM1)
	{
		addr = VfdSegAddr[dignum].Segaddr1;
		VfdSegAddr[dignum].CurrValue1 = val;
	}
	else if (segnum == SEGNUM2)
	{
		addr = VfdSegAddr[dignum].Segaddr2;
		VfdSegAddr[dignum].CurrValue2 = val;
	}
	VFD_WR(addr);
	udelay(10);
	VFD_WR(val);
	VFD_CS_SET();
	return  0;
}

// Set PT6311 mode to read key or write RAM
static int VFD_Set_Mode(VFDMode_t mode)
{
	unsigned char data = 0;

	if (mode == VFDWRITEMODE)
	{
		data = PT6311_DATASET + PT6311_FIXEDADR;  // 0x44 -> 0100 0100 -> Dataset
		VFD_CS_CLR();
		VFD_WR(data);
		VFD_CS_SET();
	}
	else if (mode == VFDREADMODE)
	{
		data = PT6311_DATASET + PT6311_FIXEDADR + PT6311_READKEY;  // 0x46 = 0100 01100 
		VFD_WR(data);
		udelay(5);
	}
	return 0;
}

/**************************************************
 *
 * PT6311 VFD routines.
 *
 */
static int VFD_Show_Content(void)
{
	dprintk(15, "%s > state is on\n", __func__);
	down_interruptible(&display_sem);  // maybe no need. need more testing

	vfd_show = 1;
	VFD_CS_CLR();
	VFD_WR(PT6311_DISP_CTL + PT6311_DISP_ON + (vfd_brightness & 0x07));  // 1000 1bbb
	VFD_CS_SET();

	up(&display_sem);

	udelay(20);
	return 0;
}

static int VFD_Show_Content_Off(void)
{
	dprintk(15, "%s > state is off\n", __func__);

	vfd_show = 0;
	VFD_CS_CLR();
	VFD_WR(PT6311_DISP_CTL + (vfd_brightness & 0x07)); // 10000bbb -> display off
	VFD_CS_SET();
	return 0;
}

static int VFD_Set_Brightness(int level)
{
	dprintk(15, "%s > level = %d\n", __func__, level);

	vfd_brightness = level & 0x07;
	VFD_CS_CLR();
	VFD_WR(PT6311_DISP_CTL + (vfd_show ? PT6311_DISP_ON : 0) + vfd_brightness); // -> 1000?bbb
	VFD_CS_SET();

	return 0;
}

void VFD_Clear_All(void)
{
	int i;

	for (i = 0; i < 13; i++)
	{
		VFD_Seg_Dig_Seg(i + 1, SEGNUM1, 0x00);
//		VFD_Show_Content();
		VfdSegAddr[i + 1].CurrValue1 = 0x00;
		VFD_Seg_Dig_Seg(i + 1, SEGNUM2, 0x00);
//		VFD_Show_Content();
		VfdSegAddr[i + 1].CurrValue2 = 0;
	}
}

// Show one character on specified position (text section)
void VFD_Draw_ASCII_Char(char c, unsigned char position)
{
	if (position < 1 || position > 8)
	{
		dprintk(1, "Character position error! %d\n", position);
		return;
	}
	if (c >= 'A' && c <= 0x5F)  // if upper case character
	{
		c = c - 'A';  // convert to table offset
	}
	else if (c >= 'a' && c <= 'z')
	{
		c = c - 'a';  // convert table offset (upper case)
	}
	else if (c >= '*' && c <= '9')  // if digit
	{
		c = c - OFFSET_ASCII1; // convert to digit offset
	}
	else
	{
		dprintk(1, "Unknown character!\n");
		return;
	}
	VFD_Seg_Dig_Seg(position, SEGNUM1, ASCII[(unsigned char)c][0]);
	VFD_Seg_Dig_Seg(position, SEGNUM2, ASCII[(unsigned char)c][1]);
//	VFD_Show_Content();
}

// Write digit in clock section on specified position
void VFD_Draw_Num(unsigned char c, unsigned char position)
{
	int dignum;

	if (position < 1 || position > 4)
	{
		dprintk(1, "Numeric position error! %d\n", position);
		return;
	}
	if (c > 9)
	{
		dprintk(1, "Illegal value!\n");
		return;
	}
	dignum = 10 - position / 3;
/*	1 -> 10 - (1 / 3) = 10
	2 -> 10 - (2 / 3) = 10
    3 -> 10 - (3 / 3) = 9
    4 -> 10 - (4 / 3) = 9
*/
	if (position % 2 == 1)
	{
		if (NumLib[c][1] & 0x01)
		{
			VFD_Seg_Dig_Seg(dignum, SEGNUM1, VfdSegAddr[dignum].CurrValue1 | 0x80);
		}
		else
		{
			VFD_Seg_Dig_Seg(dignum, SEGNUM1, VfdSegAddr[dignum].CurrValue1 & 0x7F);
		}
		VfdSegAddr[dignum].CurrValue2 = VfdSegAddr[dignum].CurrValue2 & 0x40;  //sz
		VFD_Seg_Dig_Seg(dignum, SEGNUM2, (NumLib[c][1] >> 1) | VfdSegAddr[dignum].CurrValue2);
	}
	else if (position % 2 == 0)
	{
		if ((NumLib[c][0] & 0x01))
		{
			VFD_Seg_Dig_Seg(dignum, SEGNUM2, VfdSegAddr[dignum].CurrValue2 | 0x40);  // SZ  08-05-30
		}
		else
		{
			VFD_Seg_Dig_Seg(dignum, SEGNUM2, VfdSegAddr[dignum].CurrValue2 & 0x3F);
		}
		VfdSegAddr[dignum].CurrValue1 = VfdSegAddr[dignum].CurrValue1 & 0x80;
		VFD_Seg_Dig_Seg(dignum, SEGNUM1, (NumLib[c][0] >> 1) | VfdSegAddr[dignum].CurrValue1);
	}
}

// Write time in clock section
static int VFD_Show_Time(int hh, int mm)
{
	int res = 0;

	if ((hh > 23) && (mm > 59))
	{
		dprintk(1, "%s Bad value!\n", __func__);
		return res;
	}
	VFD_Draw_Num((hh / 10), 1);
	VFD_Draw_Num((hh % 10), 2);
	VFD_Draw_Num((mm / 10), 3);
	VFD_Draw_Num((mm % 10), 4);
//	VFD_Show_Content();
	return 0;
}

// (Re-)set an icon
static int VFD_Show_Icon(LogNum_c log_num, int log_stat)
{
	int dig_num = 0, seg_num = 0;
	SegNum_t seg_part = 0;
	u8  seg_offset = 0;
	u8  addr = 0, val = 0;

	dprintk(150, "%s  > Log_num = 0x%02x, state = 0x%02x\n", __func__, log_num, log_stat);

	if (log_num >= LogNum_Max)
	{
		dprintk(1, "%s Bad value!\n", __func__);
		return -1;
	}
	dig_num = log_num / 16;  // get grid number
	seg_num = log_num % 16;  // get segment number (1 - 15)
	seg_part = seg_num / 9;  // get byte offset (0 - 1)

	VFD_CS_CLR();
	if (seg_part == SEGNUM1)  // if segment number 8 or lower, handle lower byte
	{
		seg_offset = 0x01 << ((seg_num % 9) - 1);  // shift bit into place
		addr = VfdSegAddr[dig_num].Segaddr1;  // get address
		if (log_stat == LOG_ON)  // if icon on
		{
			VfdSegAddr[dig_num].CurrValue1 |= seg_offset;  // set bit
		}
		else
		{
			VfdSegAddr[dig_num].CurrValue1 &= (0xff - seg_offset);  // else clear it
		}
		val = VfdSegAddr[dig_num].CurrValue1;  // get new value for lower 8 bits
	}
	else if (seg_part == SEGNUM2)  // else if segment number > 8
	{
		seg_offset = 0x01 << ((seg_num % 8) - 1);  // shift icon bit into place
		addr = VfdSegAddr[dig_num].Segaddr2;  // get address
		if (log_stat == LOG_ON)  // if icon on
		{
			VfdSegAddr[dig_num].CurrValue2 |= seg_offset;  // set icon bit
		}
		else
		{
			VfdSegAddr[dig_num].CurrValue2 &= (0xff - seg_offset);
		}
		val = VfdSegAddr[dig_num].CurrValue2;
	}
	VFD_WR(addr);  // set address
	udelay(5);
	VFD_WR(val);  // write value
//	udelay(5);
	VFD_CS_SET();
	return 0;
}

void clear_display(void)
{
	int j;

	for (j = 0; j < 8; j++)
	{
		VFD_Seg_Dig_Seg(j + 1, SEGNUM1, 0x00);
		VFD_Seg_Dig_Seg(j + 1, SEGNUM2, 0x00);
	}
}

int draw_thread(void *arg)
{
	struct vfd_ioctl_data *data;
	struct vfd_ioctl_data draw_data;
	int count = 0;
	int pos = 0;
	int j = 0;
	int k = 0;
	unsigned char c0;
	unsigned char c1;
	unsigned char temp;
	unsigned char draw_buf[64][2];

	data = (struct vfd_ioctl_data *)arg;

	draw_data.length = data->length;
	memcpy(draw_data.data, data->data, data->length);

	draw_thread_status = THREAD_STATUS_RUNNING;

	clear_display();

	while (pos < draw_data.length)
	{
		if (kthread_should_stop())
		{
			draw_thread_status = THREAD_STATUS_STOPPED;
			return 0;
		}
		c0 = c1 = temp = 0;

		if (draw_data.data[pos] < 0x80)
		{
			temp = draw_data.data[pos];

			if (temp == '\n' || temp == 0)  // skip NULLs and LFs
			{
				break;
			}
			else if (temp == '(' || temp == ')')  // replace parentheses by spaces
			{
				temp = OFFSET_ASCII2;
			}
			else if (temp >= 'A' && temp <= '_')
			{
				temp = temp - 'A';
			}
			else if (temp >= 'a' && temp <= 'z')
			{
				temp = temp - 'a';
			}
			else if (temp >= '*' && temp <= '9')
			{
				temp = temp - OFFSET_ASCII1;
			}
			else if (temp == ' ')
			{
				temp = OFFSET_ASCII2;
			}
			if (temp < OFFSET_ASCII2 + 1)  // if contained in table
			{
				c0 = ASCII[temp][0];  // get
				c1 = ASCII[temp][1];  // segment data
			}
		}
		// handle possible UTF-8
		else if (draw_data.data[pos] < 0xE0)
		{
			pos++;
			switch (draw_data.data[pos - 1])
			{
				case 0xc2:
				{
					c0 = UTF_C2[draw_data.data[pos] & 0x3f][0];
					c1 = UTF_C2[draw_data.data[pos] & 0x3f][1];
					break;
				}
				case 0xc3:
				{
					c0 = UTF_C3[draw_data.data[pos] & 0x3f][0];
					c1 = UTF_C3[draw_data.data[pos] & 0x3f][1];
					break;
				}
				case 0xc4:
				{
					c0 = UTF_C4[draw_data.data[pos] & 0x3f][0];
					c1 = UTF_C4[draw_data.data[pos] & 0x3f][1];
					break;
				}
				case 0xc5:
				{
					c0 = UTF_C5[draw_data.data[pos] & 0x3f][0];
					c1 = UTF_C5[draw_data.data[pos] & 0x3f][1];
					break;
				}
				case 0xd0:
				{
					c0 = UTF_D0[draw_data.data[pos] & 0x3f][0];
					c1 = UTF_D0[draw_data.data[pos] & 0x3f][1];
					break;
				}
				case 0xd1:
				{
					c0 = UTF_D1[draw_data.data[pos] & 0x3f][0];
					c1 = UTF_D1[draw_data.data[pos] & 0x3f][1];
					break;
				}
			}
		}
		else
		{
			if (draw_data.data[pos] < 0xF0)
			{
				pos += 2;
			}
			else if (draw_data.data[pos] < 0xF8)
			{
				pos += 3;
			}
			else if (draw_data.data[pos] < 0xFC)
			{
				pos += 4;
			}
			else
			{
				pos += 5;
			}
		}
		// end of UTF-8 handling
		draw_buf[count][0] = c0;  // set
		draw_buf[count][1] = c1;  // segment data
		count++;
		pos++;
	}
//#ifdef ENABLE_SCROLL // j00zek: disabled by default, interferes with GUI's
	if (count > DISP_SIZE && scroll_flag)
	{
		pos  = 0;
		while (pos < count)
		{
			if (kthread_should_stop())
			{
				draw_thread_status = THREAD_STATUS_STOPPED;
				return 0;
			}
			k = DISP_SIZE;
			if (count - pos < DISP_SIZE)
			{
				k = count - pos;
			}
			clear_display();

			for (j = 0; j < k; j++)
			{
				VFD_Seg_Dig_Seg(j + 1, SEGNUM1, draw_buf[pos + j][0]);
				VFD_Seg_Dig_Seg(j + 1, SEGNUM2, draw_buf[pos + j][1]);
			}
			msleep(300);
			pos++;
		}
		scroll_flag = 0;
	}
//#endif
	if (count > 0)
	{
		k = DISP_SIZE;
		if (count < DISP_SIZE)
		{
			k = count;
		}
		if (kthread_should_stop())
		{
			draw_thread_status = THREAD_STATUS_STOPPED;
			return 0;
		}
		clear_display();

		for (j = 0; j < k; j++)
		{
			VFD_Seg_Dig_Seg(j + 1, SEGNUM1, draw_buf[j][0]);
			VFD_Seg_Dig_Seg(j + 1, SEGNUM2, draw_buf[j][1]);
		}
//		VFD_Show_Content();
	}

	if (count == 0)
	{
		clear_display();
	}
	draw_thread_status = THREAD_STATUS_STOPPED;
	return 0;
}

int run_draw_thread(struct vfd_ioctl_data *draw_data)
{
	if (!draw_thread_status && draw_task)
	{
		kthread_stop(draw_task);
	}
	// wait thread stop
	while (!draw_thread_status)
	{
		msleep(5);
	}
	msleep(10);

	draw_thread_status = THREAD_STATUS_INIT;
	draw_task = kthread_run(draw_thread, draw_data, "draw_thread");

	// wait thread run
	while (draw_thread_status == THREAD_STATUS_INIT)
	{
		msleep(5);
	}
	return 0;
}

static int VFD_Show_Time_Off(void)
{
	int ret = 0;

	ret = VFD_Seg_Dig_Seg(9, SEGNUM1, 0x00);
	ret = VFD_Seg_Dig_Seg(9, SEGNUM2, 0x00);
	ret = VFD_Seg_Dig_Seg(10, SEGNUM1, 0x00);
	ret = VFD_Seg_Dig_Seg(10, SEGNUM2, 0x00);
	return ret;
}

/**************************************************
 *
 * Keyboard routines.
 *
 */
// Read n bytes key data from PT6311
// !! only returns the last byte
unsigned char PROTONfp_Scan_Keyboard(unsigned char read_num)
{
	unsigned char key_val[read_num];
	unsigned char i = 0, ret;

	VFD_CS_CLR();
	ret = VFD_Set_Mode(VFDREADMODE);  // Set read from to PT6311 data
	if (ret)
	{
		dprintk(1, "%s DEVICE BUSY!\n", __func__);
		return -1;
	}

	for (i = 0; i < read_num; i++)
	{
		key_val[i] = PROTONfp_RD();
	}
	VFD_CS_SET();

	ret = VFD_Set_Mode(VFDWRITEMODE);  // SET write to PT6311 RAM
	if (ret)
	{
		dprintk(1, "%s DEVICE BUSY!\n", __func__);
		return -1;
	}
	return key_val[5];  // return last byte read
}

static int PROTONfp_Get_Key_Value(void)
{
	unsigned char byte = 0;
	int key_val = INVALID_KEY;

	byte = PROTONfp_Scan_Keyboard(6);  // read 6 key value bytes, return the last

	switch (byte)
	{
		case 0x01:
		{
			key_val = EXIT_KEY;
			break;
		}
		case 0x02:
		{
			key_val = KEY_LEFT;
			break;
		}
		case 0x04:
		{
			key_val = KEY_UP;
			break;
		}
		case 0x08:
		{
			key_val = KEY_OK;
			break;
		}
		case 0x10:
		{
			key_val = KEY_RIGHT;
			break;
		}
		case 0x20:
		{
			key_val = KEY_DOWN;
			break;
		}
		case 0x40:
		{
			key_val = KEY_POWER;
			break;
		}
		case 0x80:
		{
			key_val = KEY_MENU;
			break;
		}
		default :
		{
			key_val = INVALID_KEY;
			break;
		}
	}
	return key_val;
}

/**************************************************
 *
 * Time routine.
 *
 */
// Set time to show in clock part
int protonSetTime(int hour, int minute)
{
	char buffer[8];
	int  res = 0;

//	dprintk(150, "%s >\n", __func__);

	dprintk(20, "%s time: %02d:%02d\n", __func__, hour, minute);
	VFD_Show_Time(hour, minute);
//	dprintk(150, "%s <\n", __func__);
	return res;
}

/**************************************************
 *
 * Initialize driver.
 *
 */
int vfd_init_func(void)
{
	dprintk(150, "%s >\n", __func__);
	dprintk(0, "Spider HL101 VFD module initializing\n");

	cfg.data_pin[0] = 3;  // data pin is PIO 3.2
	cfg.data_pin[1] = 2;
	cfg.clk_pin[0] = 3;  // clk pin is PIO 3.4
	cfg.clk_pin[1] = 4;
	cfg.cs_pin[0] = 3;  // strobe pin is PIO 3.5
	cfg.cs_pin[1] = 5;

	cfg.cs  = stpio_request_pin(cfg.cs_pin[0], cfg.cs_pin[1], "VFD CS", STPIO_OUT);
	cfg.clk = stpio_request_pin(cfg.clk_pin[0], cfg.clk_pin[1], "VFD CLK", STPIO_OUT);
	cfg.data = stpio_request_pin(cfg.data_pin[0], cfg.data_pin[1], "VFD DATA", STPIO_OUT);

	if (!cfg.cs || !cfg.data || !cfg.clk)
	{
		dprintk(1, "%s: PIO allocation error!\n", __func__);
		return -1;
	}
	vfd_brightness = 7;  // brightness is maximum
	vfd_show = 1;  // display is on
	init_rwsem(&vfd_rws);

	VFD_CS_CLR();
	VFD_WR(PT6311_DMODESET + PT6311_13DIGIT);  // 0x0c = 0000 1100 -> 13 digit mode
	VFD_CS_SET();

	VFD_Set_Mode(VFDWRITEMODE);  // Set write to PT6311 RAM
	VFD_Seg_Addr_Init();
	VFD_Clear_All();
//	VFD_Show_Content();
	return 0;
}

/**************************************************
 *
 * IOCTL routines.
 *
 */
static void VFD_CLR(void)
{
	VFD_CS_CLR();
	VFD_WR(PT6311_DMODESET + PT6311_13DIGIT);
	VFD_CS_SET();

	VFD_Set_Mode(VFDWRITEMODE);  // set write to PT6311 RAM
	VFD_Seg_Addr_Init();
	VFD_Clear_All();
	VFD_Show_Content();
}

int protonSetIcon(int which, int on)
{
	int res = 0;

	dprintk(150, "%s >\n", __func__);

	if (which < 1 || which > ICON_LAST)
	{
		dprintk(1, "%s Icon number %d out of range\n", __func__, which);
		return -EINVAL;
	}
//	dprintk(20, "%s Icon %d %s\n", __func__, which, (on == 0 ? "off" : "on"));
	which -= 1;
	res = VFD_Show_Icon(((which / 15) + 11) * 16 + (which % 15) + 1, on);
	dprintk(150, "%s <\n", __func__);
	return res;
}
EXPORT_SYMBOL(protonSetIcon);

/****************************************
 *
 * Code for writing to /dev/vfd
 *
 */
void clear_vfd(void)
{
	struct vfd_ioctl_data bBuf;
	int res = 0;

	dprintk(100, "%s >\n", __func__);

	memset(bBuf.data, ' ', sizeof(bBuf.data));
//	bBuf.data[DISP_SIZE] = '\0';  // terminate string
	bBuf.length = DISP_SIZE;
	res = run_draw_thread(&bBuf);
	dprintk(100, "%s <\n", __func__);
}

static ssize_t PROTONdev_write(struct file *filp, const unsigned char *buff, size_t len, loff_t *off)
{
	char *kernel_buf;
	int  minor, vLoop, res = 0;
	int  pos = 0;
	int  offset = 0;
	struct vfd_ioctl_data buf;  // scroll buffer
	char *b;

	struct vfd_ioctl_data data;

	dprintk(150, "%s >\n", __func__);

	if (len == 0)
	{
		return len;
	}

	minor = -1;
	for (vLoop = 0; vLoop < LASTMINOR; vLoop++)
	{
		if (FrontPanelOpen[vLoop].fp == filp)
		{
			minor = vLoop;
		}
	}

	if (minor == -1)
	{
		dprintk(1, "%s Error: Bad Minor\n", __func__);
		return -1; //FIXME
	}
	dprintk(70, "minor = %d\n", minor);

	/* do not write to the remote control */
	if (minor == FRONTPANEL_MINOR_RC)
	{
		return -EOPNOTSUPP;
	}

	kernel_buf = kmalloc(len, GFP_KERNEL);

	if (kernel_buf == NULL)
	{
		dprintk(1, "%s < No memory.\n", __func__);
		return -ENOMEM;
	}
	copy_from_user(kernel_buf, buff, len);

	if (down_interruptible(&write_sem))
	{
		return -ERESTARTSYS;
	}
	data.length = len;

	if (data.length >= 64)  // do not display more than 64 characters
	{
		data.length = 64;
	}

	if (kernel_buf[data.length - 1] == '\n')  // strip trailing LF
	{
		data.length--;
	}
	kernel_buf[data.length] = 0;  // terminate text string

	if (len < 0)
	{
		res = -1;
		dprintk(1, "%s Empty string!\n", __func__);
		return res;
	}
	dprintk(20, "%s Text [%s] (len %d)\n", __func__, kernel_buf, data.length);

#if 0
	if (data.length > DISP_SIZE)  // scroll text, display string is longer than display length
	{
		// initial display starting at 3rd position to ease reading
		memset(buf.data, ' ', sizeof(buf.data));
		offset = 3;
		memcpy(buf.data + offset, kernel_buf, data.length);  // get display text in buffer
		buf.start_address = 0;
		buf.length + data.length + offset;

		b = buf.data;
		for (pos = 0; pos < data.length; pos++)
		{
			memcpy(buf.data, b + pos, DISP_SIZE);
			buf.length = DISP_SIZE;
//			dprintk(5, "%s Text [%s] (len %d)\n", __func__, buf.data, DISP_SIZE);
			res = run_draw_thread(&buf);
			// wait thread stop
			while (!draw_thread_status)
			{
				msleep(5);
			}
			// sleep 300 ms
			msleep(300);
		}
		clear_vfd();
	}
#else
	scroll_flag = 1;
#endif
	// final display, or no scroll
	memcpy(data.data, kernel_buf, DISP_SIZE);
	res = run_draw_thread(&data);

	kfree(kernel_buf);
	up(&write_sem);
	dprintk(150, "%s < res %d len %d\n", __func__, res, len);

	if (res < 0)
	{
		return res;
	}
	else
	{
		return len;
	}
}

/**************************************************
 *
 * Read from /dev/vfd.
 *
 */
static ssize_t PROTONdev_read(struct file *filp, unsigned char __user *buff, size_t len, loff_t *off)
{
	int minor, vLoop;

	dprintk(150, "%s > (len %d, offs %d)\n", __func__, len, (int) *off);

	minor = -1;
	for (vLoop = 0; vLoop < LASTMINOR; vLoop++)
	{
		if (FrontPanelOpen[vLoop].fp == filp)
		{
			minor = vLoop;
		}
	}

	if (minor == -1)
	{
		printk("Error Bad Minor\n");
		return -EUSERS;
	}
	dprintk(50, "minor = %d\n", minor);

	if (minor == FRONTPANEL_MINOR_RC)
	{
		while (receiveCount == 0)
		{
			if (wait_event_interruptible(wq, receiveCount > 0))
			{
				return -ERESTARTSYS;
			}
		}
		/* 0. claim semaphore */
		down_interruptible(&receive_sem);
		/* 1. copy data to user */
		copy_to_user(buff, receive[0].buffer, receive[0].len);
		/* 2. copy all entries to start and decrease receiveCount */
		receiveCount--;
		memmove(&receive[0], &receive[1], 99 * sizeof(struct receive_s));
		/* 3. free semaphore */
		up(&receive_sem);
		return 8;
	}
	/* copy the current display string to the user */
	if (down_interruptible(&FrontPanelOpen[minor].sem))
	{
		dprintk(50, "%s return erestartsys<\n", __func__);
		return -ERESTARTSYS;
	}
	if (FrontPanelOpen[minor].read == lastdata.length)
	{
		FrontPanelOpen[minor].read = 0;

		up(&FrontPanelOpen[minor].sem);
		printk("%s < return 0\n", __func__);
		return 0;
	}

	if (len > lastdata.length)
	{
		len = lastdata.length;
	}
	/* fixme: needs revision because of utf8! */
	if (len > 16)
	{
		len = 16;
	}
	FrontPanelOpen[minor].read = len;
	copy_to_user(buff, lastdata.data, len);

	up(&FrontPanelOpen[minor].sem);

	dprintk(150, "%s < (len %d)\n", __func__, len);
	return len;
}

/**************************************************
 *
 * Open /dev/vfd.
 *
 */
int PROTONdev_open(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(150, "%s >\n", __func__);

	minor = MINOR(inode->i_rdev);

	dprintk(50, "open minor %d\n", minor);

	if (FrontPanelOpen[minor].fp != NULL)
	{
		dprintk(1, "%s < EUSER\n", __func__);
		return -EUSERS;
	}
	FrontPanelOpen[minor].fp = filp;
	FrontPanelOpen[minor].read = 0;

	dprintk(150, "%s <\n", __func__);
	return 0;
}

/**************************************************
 *
 * Close /dev/vfd.
 *
 */
int PROTONdev_close(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(150, "%s >\n", __func__);

	minor = MINOR(inode->i_rdev);

	dprintk(50, "%s close minor %d\n", __func__, minor);

	if (FrontPanelOpen[minor].fp == NULL)
	{
		dprintk(1, "%s < EUSER\n", __func__);
		return -EUSERS;
	}
	FrontPanelOpen[minor].fp = NULL;
	FrontPanelOpen[minor].read = 0;

	dprintk(150, "%s <\n", __func__);
	return 0;
}

/**************************************************
 *
 * /dev/vfd IOCTL handling.
 *
 */
static int PROTONdev_ioctl(struct inode *Inode, struct file *File, unsigned int cmd, unsigned long arg)
{
	static int mode = 0;
	struct proton_ioctl_data proton_data;
	static struct vfd_ioctl_data vfd_data;
	int res = 0;

	dprintk(150, "%s > 0x%.8x\n", __func__, cmd);

	if (down_interruptible(&write_sem))
	{
		return -ERESTARTSYS;
	}

	switch (cmd)
	{
		case VFDSETMODE:
		case VFDICONDISPLAYONOFF:
		case VFDSETTIME:
		case VFDBRIGHTNESS:
		case VFDDISPLAYWRITEONOFF:
		case VFDSETDISPLAYTIME:
		{
			if (copy_from_user(&proton_data, (void *)arg, sizeof(proton_data)))
			{
				return -EFAULT;
			}
		}
	}

	switch (cmd)
	{
		case VFDSETMODE:
		{
			mode = proton_data.u.mode.compat;
			break;
		}
		case VFDSETLED:
		{
			break;
		}
		case VFDBRIGHTNESS:
		{
			if (proton_data.u.brightness.level < 0 || proton_data.u.brightness.level > 7)
			{
				dprintk(1, "Illegal brightness level %d (range 0-7)\n", proton_data.u.brightness.level);
				res = -1;
				break;
			}
//			dprintk(20, "%s Brightness = %d\n", __func__, proton_data.u.brightness.level);
			res = VFD_Set_Brightness(proton_data.u.brightness.level);
			break;
		}
		case VFDICONDISPLAYONOFF:
		{
			int icon_nr = proton_data.u.icon.icon_nr;

			if (icon_nr >= 256)  // Convert icon numbers sent by Enigma2
			{
				icon_nr >>= 8;
				switch (icon_nr)
				{
					case 17:
					{
						icon_nr = ICON_DOUBLESCREEN; // widescreen
						break;
					}
					case 19:
					{
						icon_nr = ICON_CA;
						break;
					}
					case 21:
					{
						icon_nr = ICON_MP3;
						break;
					}
					case 23:
					{
						icon_nr = ICON_AC3;
						break;
					}
					case 26:
					{
						icon_nr = ICON_PLAY; // Play
						break;
					}
					case 30:
					{
						icon_nr = ICON_REC1;
						break;
					}
					case 38: // CD segments & circle
					case 39:
					case 40:
					case 41:
					{
						break;
					}
//					case 47:
//					{
//						icon_nr = ICON_SPINNER;
//						break;
//					}
					default:
					{
						dprintk(0, "Tried to set unknown E2 icon number %d.\n", icon_nr);
						res = -1;
						break;
					}
				}
			}
//			dprintk(20, "%s Icon number = %d, state = %d\n", __func__, icon_nr, proton_data.u.icon.on);
			if (icon_nr == ICON_ALL)
			{
				int i;

				for (i = 1; i <= ICON_LAST; i++)
				{
					res |= protonSetIcon(i, proton_data.u.icon.on);
				}
				mode = 0;
				break;
			}
			else if (icon_nr < 0 || icon_nr > ICON_LAST)
			{
				dprintk(1, "Illegal icon number %d (range 1-%d)\n", icon_nr, ICON_LAST);
				res = -1;
				mode = 0;
				break;
			}
//			dprintk(20, "%s Icon number = %d, state = %d\n", __func__, icon_nr, proton_data.u.icon.on);
			res = protonSetIcon(icon_nr, proton_data.u.icon.on);
			mode = 0;
			break;
		}
		case VFDSTANDBY:
		{
			break;
		}
		case VFDSETTIME:
		{
			dprintk(20, "%s Time to show: = %02d:%02d\n", __func__, proton_data.u.time.time[2], proton_data.u.time.time[3]);
			res = protonSetTime(proton_data.u.time.time[2], proton_data.u.time.time[3]);
			break;
		}
		case VFDGETTIME:
		{
			break;
		}
		case VFDGETWAKEUPMODE:
		{
			break;
		}
#if 0
		case VFDSETDISPLAYTIME:  // switch clock on/off
		{
			dprintk(20, "%s Switch clock %s\n", __func__, (proton_data.u.display_time.on == 0 ? "off": "on");
			res = 0;
			break
		}
		case VFDGETDISPLAYTIME:
		{
			dprintk(20, "%s Clock is %s\n", __func__, (proton_data.u.display_time.on == 0 ? "off": "on");
			res = put_user(TimeMode, proton_data.u.display_time.on);			
			res = 0;
			break
		}
#endif
		case VFDDISPLAYCHARS:
		{
			if (mode == 0)
			{
				struct vfd_ioctl_data *data = (struct vfd_ioctl_data *)arg;

				if (data->length < 0)
				{
					res = -1;
					dprintk(1, "%s: Error: empty string!\n");
				}
				else
				{
					res = run_draw_thread(data);
				}
			}
			else
			{
				mode = 0;
			}
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			dprintk(20, "%s Display state %s\n", __func__, proton_data.u.light.onoff == 0 ? "off" : "on");
			if (proton_data.u.light.onoff == 0)  // 1 = show, 0 = off
			{
				VFD_Show_Content_Off();
			}
			else
			{
				VFD_Show_Content();
			}
			break;
		}
		case VFDDISPLAYCLR:
		{
			if (!draw_thread_status)
			{
				kthread_stop(draw_task);
			}
			// wait thread stop
			while (!draw_thread_status)
			{
				msleep(5);
			}
			VFD_CLR();
			break;
		}
		case 0x5305:
		case 0x5401:
		{
			mode = 0;
			break;
		}
		default:
		{
			dprintk(1, "%s: unknown IOCTL 0x%x\n", __func__, cmd);
			break;
		}
	}
	up(&write_sem);

	dprintk(150, "%s <\n", __func__);
	return res;
}

/**************************************************
 *
 * Generic driver code.
 *
 */
static unsigned int PROTONdev_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(filp, &wq, wait);

	if (receiveCount > 0)
	{
		mask = POLLIN | POLLRDNORM;
	}
	return mask;
}

static struct file_operations vfd_fops =
{
	.owner   = THIS_MODULE,
	.ioctl   = PROTONdev_ioctl,
	.write   = (void *)PROTONdev_write,
	.read    = (void *)PROTONdev_read,
	.poll    = (void *)PROTONdev_poll,
	.open    = PROTONdev_open,
	.release = PROTONdev_close
};

/**************************************************
 *
 * Button driver
 *
 */
static char *button_driver_name = "Spiderbox HL101 frontpanel buttons driver";
static struct input_dev *button_dev;
static int button_value = -1;
static int bad_polling = 1;
static struct workqueue_struct *fpwq;

void button_bad_polling(void)
{
	int btn_pressed = 0;
	int report_key = 0;

	while (bad_polling == 1)
	{
		msleep(50);
		button_value = PROTONfp_Get_Key_Value();
		if (button_value != INVALID_KEY)
		{
			dprintk(50, "%s Got button: 0x%02X\n", __func__, button_value);
			VFD_Show_Icon(DOT2, LOG_ON);
			if (1 == btn_pressed)
			{
				if (report_key != button_value)
				{
					input_report_key(button_dev, report_key, 0);
					input_sync(button_dev);
				}
				else
				{
					continue;
				}
			}
			report_key = button_value;
			btn_pressed = 1;
			switch (button_value)
			{
				case KEY_LEFT:
				case KEY_RIGHT:
				case KEY_UP:
				case KEY_DOWN:
				case KEY_OK:
				case KEY_MENU:
				case KEY_EXIT:
				case KEY_POWER:
				{
					input_report_key(button_dev, button_value, 1);
					input_sync(button_dev);
					break;
				}
				default:
				{
					dprintk(1, "[BTN] Unknown button_value 0x%02x\n", button_value);
				}
			}
		}
		else
		{
			if (btn_pressed)
			{
				btn_pressed = 0;
				msleep(50);
				VFD_Show_Icon(DOT2, LOG_OFF);
				input_report_key(button_dev, report_key, 0);
				input_sync(button_dev);
			}
		}
	}
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 17)
static DECLARE_WORK(button_obj, button_bad_polling);
#else
static DECLARE_WORK(button_obj, button_bad_polling, NULL);
#endif

static int button_input_open(struct input_dev *dev)
{
	fpwq = create_workqueue("button");
	if (queue_work(fpwq, &button_obj))
	{
		dprintk(5, "[BTN] Queue_work successful.\n");
		return 0;
	}
	dprintk(1, "[BTN] Queue_work not successful, exiting.\n");
	return 1;
}

static void button_input_close(struct input_dev *dev)
{
	bad_polling = 0;
#if 0
	while (bad_polling != 2)
	{
		msleep(1);
	}
#else
	msleep(55);
#endif
	bad_polling = 1;

	if (fpwq)
	{
		destroy_workqueue(fpwq);
		dprintk(5, "[BTN] Workqueue destroyed.\n");
	}
}

int button_dev_init(void)
{
	int error;

	dprintk(5, "[BTN] Allocating and registering button device\n");

	button_dev = input_allocate_device();
	if (!button_dev)
	{
		return -ENOMEM;
	}

	button_dev->name  = button_driver_name;
	button_dev->open  = button_input_open;
	button_dev->close = button_input_close;

	set_bit(EV_KEY,    button_dev->evbit);
	set_bit(KEY_UP,    button_dev->keybit);
	set_bit(KEY_DOWN,  button_dev->keybit);
	set_bit(KEY_LEFT,  button_dev->keybit);
	set_bit(KEY_RIGHT, button_dev->keybit);
	set_bit(KEY_POWER, button_dev->keybit);
	set_bit(KEY_MENU,  button_dev->keybit);
	set_bit(KEY_OK,    button_dev->keybit);
	set_bit(KEY_EXIT,  button_dev->keybit);

	error = input_register_device(button_dev);
	if (error)
	{
		input_free_device(button_dev);
	}
	return error;
}

void button_dev_exit(void)
{
	dprintk(5, "[BTN] unregistering button device\n");
	input_unregister_device(button_dev);
}

/**************************************************
 *
 * Driver setup
 *
 */
static struct cdev  vfd_cdev;
static struct class *vfd_class = 0;

static int __init proton_init_module(void)
{
	int i;

	dprintk(150, "%s >\n", __func__);

	sema_init(&display_sem, 1);

	if (vfd_init_func())
	{
		dprintk(1, "Unable to initialize module\n");
		return -1;
	}
	vfd_class = class_create(THIS_MODULE, "proton");
	device_create(vfd_class, NULL, MKDEV(VFD_MAJOR, 0), NULL, "vfd");
	cdev_init(&vfd_cdev, &vfd_fops);
	cdev_add(&vfd_cdev, MKDEV(VFD_MAJOR, 0), 1);

	if (button_dev_init() != 0)
	{
		return -1;
	}
	if (register_chrdev(VFD_MAJOR, "VFD", &vfd_fops))
	{
		dprintk(1, "Unable to get major %d for VFD.\n", VFD_MAJOR);
	}
	sema_init(&write_sem, 1);
	sema_init(&key_mutex, 1);

	for (i = 0; i < LASTMINOR; i++)
	{
		sema_init(&FrontPanelOpen[i].sem, 1);
	}
	dprintk(150, "%s <\n", __func__);
	return 0;
}

static void __exit proton_cleanup_module(void)
{
	if (cfg.data != NULL)
	{
		stpio_free_pin(cfg.data);
	}
	if (cfg.clk != NULL)
	{
		stpio_free_pin(cfg.clk);
	}
	if (cfg.cs != NULL)
	{
		stpio_free_pin(cfg.cs);
	}
	dprintk(5, "[BTN] Unloading...\n");
	button_dev_exit();
	// kthread_stop(time_thread);
	cdev_del(&vfd_cdev);

	unregister_chrdev(VFD_MAJOR, "VFD");
	device_destroy(vfd_class, MKDEV(VFD_MAJOR, 0));
	class_destroy(vfd_class);
	dprintk(0, "HL101 FrontPanel module unloading\n");
}

module_init(proton_init_module);
module_exit(proton_cleanup_module);

module_param(gmt, charp, 0);
MODULE_PARM_DESC(gmt, "GMT offset (default +3600");
module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

MODULE_DESCRIPTION("VFD module for Spiderbox HL101");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
