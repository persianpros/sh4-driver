/*************************************************************************
 *
 * ufs910_fp_file.c
 *
 * 11. Nov 2007 - captaintrip
 * 09. Jul 2021 - Audioniek
 *
 * Kathrein UFS910 VFD Kernel module
 * Erster version portiert aus den MARUSYS uboot sourcen
 *
 * Note: previous separate led driver for 14W models has been
 *       integrated
 *
 * The front panel display consists of a VFD tube Futaba 16-BT-136INK which
 * has a 1x16 character VFD display with 16 icons and a built-in driver
 * IC that seems to be somewhat compatible with the OKI ML9208.
 * The built-in display controller does not provide key scanning
 * facilities.
 *
 * The display controller is controlled through 3 PIO pins.
 *
 * The frontpanel processor on 1W models is controlled through asc3
 * (/dev/AStty1).
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
 * 20210707 Audioniek       PIO pin control changed.
 * 20210712 Audioniek       Add code for all icons on/off.
 * 20210712 Audioniek       Text write to /dev/vfd now scrolls once if
 *                          length exceeds VFD_LENGTH.
 * 20210712 Audioniek       Driver init clears all icons.
 * 20210713 Audioniek       Display controller initialization according to
 *                          datasheet recommendation.
 * 20210714 Audioniek       Add mode 0 switching.
 * 20210714 Audioniek       Add Enigma2 icon processing.
 * 20210718 Audioniek       Add VFDSETLED (14W models only).
 * 20210719 Audioniek       Front panel button driver added (14W models
 *                          only).
 * 20210720 Audioniek       Split into file, main and procfs parts.
 *
 *************************************************************************
 */
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
#include <linux/stm/pio.h>
#else
#include <linux/stpio.h>
#endif

#include "ufs910_fp.h"
#include "utf.h"

int SCP_PORT = 0;

static struct stpio_pin *ledred;
static struct stpio_pin *ledgreen;
static struct stpio_pin *ledorange;

/* konfetti: bugfix*/
struct saved_data_s lastdata;

/* konfetti: quick and dirty open handling */
static tVFDOpen vOpen;

struct __vfd_scp *vfd_scp_ctrl = NULL;

unsigned char str[64];

// display PIO pins
static struct stpio_pin *disp_cs;    // xCS pin
static struct stpio_pin *disp_stb;   // STB pin
static struct stpio_pin *disp_data;  // data pin

/***********************************************************
 *
 * Character definitions.
 *
 */
unsigned char *ROM_Char_Table = NULL;

unsigned char ROM_KATHREIN[128] =
{
	0x00,  // 0x00,
	0x00,  // 0x01, icon #1
	0x01,  // 0x02, icon #2
	0x02,  // 0x03, icon #3
	0x03,  // 0x04, icon #4
	0x04,  // 0x05, icon #5
	0x05,  // 0x06, icon #6
	0x06,  // 0x07, icon #7
	0x10,  // 0x08,
	0x10,  // 0x09,
	0x10,  // 0x0a,
	0x10,  // 0x0b,
	0x10,  // 0x0c,
	0x10,  // 0x0d,
	0x10,  // 0x0e,
	0x10,  // 0x0f,

	0x10,  // 0x10, reserved
	0x11,  // 0x11, reserved
	0x12,  // 0x12, reserved
	0x13,  // 0x13, reserved
	0x14,  // 0x14, reserved
	0x15,  // 0x15, reserved
	0x16,  // 0x16, reserved
	0x17,  // 0x17, reserved
	0x18,  // 0x18, reserved
	0x19,  // 0x19, reserved
	0x1a,  // 0x1a, reserved
	0x1b,  // 0x1b, reserved
	0x1c,  // 0x1c, reserved
	0x10,  // 0x1d, reserved
	0x10,  // 0x1e, reserved
	0x10,  // 0x1f, reserved

	0x20,  // 0x20, <space>
	0x21,  // 0x21, !
	0x22,  // 0x22, "
	0x23,  // 0x23, #
	0x24,  // 0x24, $
	0x25,  // 0x25, %
	0x26,  // 0x26, &
	0x27,  // 0x27, '
	0x28,  // 0x28, (
	0x29,  // 0x29, )
	0x2a,  // 0x2a, *
	0x2b,  // 0x2b, +
	0x2c,  // 0x2c, ,
	0x2d,  // 0x2d, -
	0x2e,  // 0x2e, .
	0x2f,  // 0x2f, /

	0x30,  // 0x30, 0
	0x31,  // 0x31, 1
	0x32,  // 0x32, 2
	0x33,  // 0x33, 3
	0x34,  // 0x34, 4
	0x35,  // 0x35, 5
	0x36,  // 0x36, 6
	0x37,  // 0x37, 7
	0x38,  // 0x38, 8
	0x39,  // 0x39, 9
	0x3a,  // 0x3a, :
	0x3b,  // 0x3b, ;
	0x3c,  // 0x3c, <
	0x3d,  // 0x3d, =
	0x3e,  // 0x3e, >
	0x3f,  // 0x3f, ?

	0x40,  // 0x40, @
	0x41,  // 0x41, A
	0x42,  // 0x42, B
	0x43,  // 0x43, C
	0x44,  // 0x44, D
	0x45,  // 0x45, E
	0x46,  // 0x46, F
	0x47,  // 0x47, G
	0x48,  // 0x48, H
	0x49,  // 0x49, I
	0x4a,  // 0x4a, J
	0x4b,  // 0x4b, K
	0x4c,  // 0x4c, L
	0x4d,  // 0x4d, M
	0x4e,  // 0x4e, N
	0x4f,  // 0x4f, O

	0x50,  // 0x50, P
	0x51,  // 0x51, Q
	0x52,  // 0x52, R
	0x53,  // 0x53, S
	0x54,  // 0x54, T
	0x55,  // 0x55, U
	0x56,  // 0x56, V
	0x57,  // 0x57, W
	0x58,  // 0x58, X
	0x59,  // 0x59, Y
	0x5a,  // 0x5a, Z
	0x5b,  // 0x5b, [
	0x5c,  // 0x5c, 
	0x5d,  // 0x5d, ]
	0x5e,  // 0x5e, ^
	0x5f,  // 0x5f, _

	0x60,  // 0x60, `
	0x61,  // 0x61, a
	0x62,  // 0x62, b
	0x63,  // 0x63, c
	0x64,  // 0x64, d
	0x65,  // 0x65, e
	0x66,  // 0x66, f
	0x67,  // 0x67, g
	0x68,  // 0x68, h
	0x69,  // 0x69, i
	0x6a,  // 0x6a, j
	0x6b,  // 0x6b, k
	0x6c,  // 0x6c, |
	0x6d,  // 0x6d, m
	0x6e,  // 0x6e, n
	0x6f,  // 0x6f, o

	0x70,  // 0x70, p
	0x71,  // 0x71, q
	0x72,  // 0x72, r
	0x73,  // 0x73, s
	0x74,  // 0x74, t
	0x75,  // 0x75, u
	0x76,  // 0x76, v
	0x77,  // 0x77, w
	0x78,  // 0x78, x
	0x79,  // 0x79, y
	0x7a,  // 0x7a, z
	0x7b,  // 0x7b, {
	0x7c,  // 0x7c, |
	0x7d,  // 0x7d, }
	0x7e,  // 0x7e, ~
	0x7f   // 0x7f, DEL

/*  0x80,  // 0x80, Ç
	0x81,  // 0x81, ü
	0x82,  // 0x82, é
	0x83,  // 0x83, â
	0x84,  // 0x84, ä
	0x85,  // 0x85, à
	0x86,  // 0x86, á
	0x87,  // 0x87, ç
	0x88,  // 0x88, ê
	0x89,  // 0x89, ë
	0x8a,  // 0x8a, è
	0x8b,  // 0x8b, ï
	0x8c,  // 0x8c, î
	0x8d,  // 0x8d, ì
	0x8e,  // 0x8e, Ä
	0x8f,  // 0x8f, Â

	0x90,  // 0x90, É
	0x91,  // 0x91, ae
	0x92,  // 0x92, AE
	0x93,  // 0x93, ô
	0x94,  // 0x94, ö
	0x95,  // 0x95, ò
	0x96,  // 0x96, û
	0x97,  // 0x97, ù
	0x98,  // 0x98, ÿ
	0x99,  // 0x99, Ö
	0x9a,  // 0x9a, Ü
	0x9b,  // 0x9b, C|
	0x9c,  // 0x9c, POUND
	0x9d,  // 0x9d, Y-
	0x9e,  // 0x9e, Pt
	0x9f,  // 0x9f, f

	0xa0,  // 0xa0, á
	0xa1,  // 0xa1, í
	0xa2,  // 0xa2, ó
	0xa3,  // 0xa3, ú
	0xa4,  // 0xa4, ñ
	0xa5,  // 0xa5, Ñ
	0xa6,  // 0xa6, a_
	0xa7,  // 0xa7, o_
	0xa8,  // 0xa8, ? upsidedown
	0xa9,  // 0xa9, LT corner
	0xaa,  // 0xaa, RT corner
	0xab,  // 0xab, 1/2
	0xac,  // 0xac, 1/4
	0xad,  // 0xad, í
	0xae,  // 0xae, <<
	0xaf,  // 0xaf, >>

	0xb0,  // 0xb0,
	0xb1,  // 0xb1,
	0xb2,  // 0xb2,
	0xb3,  // 0xb3,
	0xb4,  // 0xb4,
	0xb5,  // 0xb5,
	0xb6,  // 0xb6, O-
	0xb7,  // 0xb7,
	0xb8,  // 0xb8,
	0xb9,  // 0xb9,
	0xba,  // 0xba,
	0xbb,  // 0xbb,
	0xbc,  // 0xbc,
	0xbd,  // 0xbd,
	0xbe,  // 0xbe,
	0xbf,  // 0xbf,

	0xc0,  // 0xc0, reserved
	0xc1,  // 0xc1, reserved
	0xc2,  // 0xc2, reserved
	0xc3,  // 0xc3, reserved
	0xc4,  // 0xc4, reserved
	0xc5,  // 0xc5, reserved
	0xc6,  // 0xc6, reserved
	0xc7,  // 0xc7, reserved
	0xc8,  // 0xc8, reserved
	0xc9,  // 0xc9, reserved
	0xca,  // 0xca, reserved
	0xcb,  // 0xcb, reserved
	0xcc,  // 0xcc, reserved
	0xcd,  // 0xcd, reserved
	0xce,  // 0xce, reserved
	0xcf,  // 0xcf, reserved

	0xd0,  // 0xd0, reserved
	0xd1,  // 0xd1, reserved
	0xd2,  // 0xd2, reserved
	0xd3,  // 0xd3, reserved
	0xd4,  // 0xd4, reserved
	0xd5,  // 0xd5, reserved
	0xd6,  // 0xd6, reserved
	0xd7,  // 0xd7, reserved
	0xd8,  // 0xd8, reserved
	0xd9,  // 0xd9, reserved
	0xda,  // 0xda, reserved
	0xdb,  // 0xdb, reserved
	0xdc,  // 0xdc, reserved
	0xdd,  // 0xdd, reserved
	0xde,  // 0xde, reserved
	0xdf,  // 0xdf, reserved

	0xe0,  // 0xe0, reserved
	0xe1,  // 0xe1, reserved
	0xe2,  // 0xe2, reserved
	0xe3,  // 0xe3, reserved
	0xe4,  // 0xe4, reserved
	0xe5,  // 0xe5, reserved
	0xe6,  // 0xe6, reserved
	0xe7,  // 0xe7, reserved
	0xe8,  // 0xe8, reserved
	0xe9,  // 0xe9, reserved
	0xea,  // 0xea, reserved
	0xeb,  // 0xeb, reserved
	0xec,  // 0xec, reserved
	0xed,  // 0xed, reserved
	0xee,  // 0xee, reserved
	0xef,  // 0xef, reserved

	0xf0,  // 0xf0, reserved
	0xf1,  // 0xf1, reserved
	0xf2,  // 0xf2, reserved
	0xf3,  // 0xf3, reserved
	0xf4,  // 0xf4, reserved
	0xf5,  // 0xf5, reserved
	0xf6,  // 0xf6, reserved
	0xf7,  // 0xf7, reserved
	0xf8,  // 0xf8, reserved
	0xf9,  // 0xf9, reserved
	0xfa,  // 0xfa, reserved
	0xfb,  // 0xfb, reserved
	0xfc,  // 0xfc, reserved
	0xfd,  // 0xfd, reserved
	0xfe,  // 0xfe, reserved
	0xff   // 0xff, reserved
*/
};

/***********************************************************
 *
 * Routines to communicate with the display controller.
 *
 */

/***********************************************************
 *
 * Debug: show command string sent to display controller.
 *
 */
void out_buf(unsigned char *buf, int len)
{
	int i;

	if (len < 64)
	{
		strncpy(str, buf, len);
		str[len] = '\0';
		dprintk(1, "%s: Buffer:", __func__);
		for (i = 0; i < len; i++)
		{
			printk(" 0x%02x", str[i]);
		}
		printk("\n");
	}
	else
	{
		dprintk(1, "%s: Overflow; len = %d (max. 63)\n", __func__, len);
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
 * The parameter current__ controls how the xCS pin is
 * handled:
 * 0         : raises xCS, then sets it to zero, then
 *             sends byte(s)
 * length - 1: after sending bytes: raises xCS
 * other     : leaves xCS in previous state (usually zero)
 *
 * Note: compared to datasheet, timing is extremely
 *       relaxed.
 *
 */
static int ufs910_fp_Send_byte(unsigned char data, int current__, int length)
{
	int i = 0;

	dprintk(150, "%s >\n", __func__);

	if (current__ == 0)  // start position
	{
		stpio_set_pin(disp_cs, 1);  // toggle
		udelay(10);
		stpio_set_pin(disp_cs, 0);  // xCS
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
		for (i = 0; i < 8 ; i++)
		{
			stpio_set_pin(disp_stb, 0);  // set strobe pin low
			stpio_set_pin(disp_data, data & 0x01);  // set bit state
			udelay(10);  // wait 10 us
			data = data >> 1;
			stpio_set_pin(disp_stb, 1);  // set strobe pin high
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
		stpio_set_pin(disp_cs, 1);  // raise xCS
		udelay(50);
	}
	dprintk(150, "%s <\n", __func__);
	return 0;
}

/***********************************************************
 *
 * Write length bytes in data to the display controller.
 *
 */
static int ufs910_fp_Send_Nbytes(unsigned char *data, int length)
{
	int i = 0;

	dprintk(100, "%s >\n", __func__);

	if (paramDebug > 69)
	{
		out_buf(data, length);
	}
	for (i = 0; i < length; i++)
	{
		ufs910_fp_Send_byte(data[i], i, length);
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
static int ufs910_fp_ShowString(struct vfd_ioctl_data *data)
{
	unsigned char write_data[VFD_LENGTH + 1];
	int ret = 0;
	int i = 0;
	int j = 0;

	dprintk(100, "%s >\n", __func__);
	data->data[data->length] = 0;  // terminate string
	dprintk(30, "Text: [%s] (length %d)\n", data->data, data->length);
	memset(write_data, ' ', sizeof(write_data)); // fill buffer with spaces

#ifdef DCRAM_INVERT
	write_data[0] = (0x10 - data->length - (data->start_address & 0x0f)) | DCRAM_COMMAND;
#else
	write_data[0] = (data->start_address & 0x0f) | DCRAM_COMMAND; // add command
#endif

	while ((i < data->length) && (j < VFD_LENGTH))
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
		else if (data->data[i] < 0xe0)
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
			if (data->data[i] < 0xf0)
			{
				i += 2;
			}
			else if (data->data[i] < 0xf8)
			{
				i += 3;
			}
			else if (data->data[i] < 0xfc)
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
	write_data[VFD_LENGTH + 1] = 0;  // terminate string
	dprintk(30, "Final non-UTF-8 text: [%s] (length %d)\n", write_data + 1, VFD_LENGTH);
	/* konfetti: bugfix*/
	/* save last string written to fp */
	memcpy(&lastdata.data, data->data, data->length);
	lastdata.length = data->length;
	ret |= ufs910_fp_Send_Nbytes(write_data, VFD_LENGTH + 1);

	dprintk(100, "%s <\n", __func__);
	return 0;
}

/***********************************************************
 *
 * Define a CGRAM dot pattern.
 *
 */
static int ufs910_fp_CGRAM_Write(struct vfd_ioctl_data *data)
{
	unsigned char write_data[6];
	int i = 0;

	dprintk(100, "%s >\n", __func__);

	write_data[0] = (data->start_address & 0x07) | CGRAM_COMMAND;
	if (data->length != 5)
	{
		dprintk(1, "%s < Error: need exactly 5 bytes (not %d)\n", __func__, data->length);
		return -1;
	}

	for (i = 0; i < data->length; i++)
	{
		write_data[i + 1] = data->data[i];
	}
	i = ufs910_fp_Send_Nbytes(write_data, data->length + 1);
	dprintk(100, "%s <\n", __func__);
	return i;
}

#if 0
/***********************************************************
 *
 * Set RC code.
 *
 * Note: cannot work; sends a byte sequence to the display
 *       controller instead of the fronr processor.
 */
static int VFD_SetRemote(void)
{
	unsigned char write_data[2];

	write_data[0] = 0xe3;  // front controller command?
	write_data[1] = 0x0f;  // get RC code data
	ufs910_fp_Send_Nbytes(write_data, 2);
	return 0;
}
#endif

/***********************************************************
 *
 * Set brightness.
 *
 */
int ufs910_fp_SetBrightness(int level)
{
	unsigned char write_data[17];
	int i = 0;
	int ret = 0;

	dprintk(100, "%s >\n", __func__);

	memset(write_data, 0, sizeof(write_data));

	write_data[0] = DIMMING_COMMAND;
	if (level == 0)
	{
		write_data[1] = 0;
	}
	else
	{
		write_data[1] = (level & 0x07) << 5 | (level + 8);  // only use 8 steps, create gradient
	}
	lastdata.brightness = write_data[1];
	ret |= ufs910_fp_Send_Nbytes(write_data, 2);

	write_data[0] = GRAY_LEVEL_DATA_COMMAND;
	write_data[1] = 0xff;
	write_data[2] = 0xff;
	ret |= ufs910_fp_Send_Nbytes(write_data, 3);

	write_data[0] = GRAY_LEVEL_ON_COMMAND;
	for (i = 0; i < 16; i++)
	{
		write_data[i + 1] = 0x05;
	}
	ret |= ufs910_fp_Send_Nbytes(write_data, 17);
	dprintk(100, "%s <\n", __func__);
	return ret;
}

/***********************************************************
 *
 * Switch entire display on or off.
 *
 */
//2 Turn ON : Turn on all vfd light and standby mode.
//2 Turn OFF : Turn off all vfd light and go to standby mod.
int ufs910_fp_SetDisplayOnOff(int on)
{
	unsigned char write_data[1];

	dprintk(100, "%s >\n", __func__);
	write_data[0] = (on == 0 ? LIGHT_ON_LS : 0) | LIGHT_ON_COMMAND;
	lastdata.display_on = (on ? 1 : 0);
	ufs910_fp_Send_Nbytes(write_data, 1);

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
	ufs910_fp_Send_Nbytes(write_data, data->length + 1);
	dprintk(100, "%s <\n", __func__);
	return 0;
}
#endif

/***********************************************************
 *
 * (Re)Set an icon.
 *
 */
static int ufs910_fp_SetIcon(int which, int on)
{
	unsigned char write_data[16];
	int ret;

	dprintk(100, "%s >\n", __func__);
	dprintk(20, "Set icon %d to %s\n", which, (on ? "on" : "off"));

	write_data[0] = (unsigned char)(which - 1) | ADRAM_COMMAND;
	write_data[1] = (unsigned char)on;
	ret = ufs910_fp_Send_Nbytes(write_data, 2);
	dprintk(100, "%s <\n", __func__);
	return ret;
}

/***********************************************************
 *
 * (Re)Set an LED.
 *
 */
int ufs910_fp_SetLED(int which, int on)
{
	struct stpio_pin *led;
	int ret = 0;

	dprintk(100, "%s >\n", __func__);
	on = (on != 0 ? 1 : 0);
	dprintk(20, "Set LED %d to %s\n", which, (on == 0 ? "off" : "on"));

	if (boxtype != 0)
	{
		on = (on != 0 ? 0 : 1);  // 1 is off
		switch (which)
		{
			case LED_RED:
			{
				led = ledred;
				break;
			}
			case LED_GREEN:
			{
				led = ledgreen;
				break;
			}
			case LED_YELLOW:
			{
				led = ledorange;
				break;
			}
		}
		if (led != NULL)
		{
			stpio_set_pin(led, on);
		}
		else
		{
			dprintk(1, "%s: Error: switching LED %d to %s failed\n", __func__, which, (on == 0 ? "off" : "on"));
			ret = -1;
		}
	}
	else
	{
		dprintk(50, "Controlling LEDs not supported yet on 1W models\n");
	}
	dprintk(100, "%s <\n", __func__);
	return ret;
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
 * Note: does not scroll; displays the first VFD_LENGTH
 *       characters. Scrolling is available by writing to
 *       /dev/vfd: this scrolls a maximum of 64 characters once
 *       if the text length exceeds VFD_LENGTH.
 */
int ufs910_fp_WriteString(unsigned char *aBuf, int len)
{
	int ret = 0;
	struct vfd_ioctl_data data;

	dprintk(100, "%s >\n", __func__);
	if (len > VFD_LENGTH)
	{
		len = VFD_LENGTH;
	}
	if (len <= 0)
	{
		len = 0;
	}
	data.start_address = 0x00;
	memset(data.data, ' ', VFD_LENGTH);
	memcpy(data.data, aBuf, len);
	data.length = len;

	ret |= ufs910_fp_ShowString(&data);
	dprintk(100, "%s <\n", __func__);
	return ret;
}

/***********************************************************
 *
 * Initialize the driver.
 *
 */
int vfd_init_func(void)
{
	int ret = 0;
	int i;
	struct vfd_ioctl_data data;
	unsigned char vfd_data[40];

	dprintk(100, "%s >\n", __func__);
	dprintk(0, "Kathrein UFS910 VFD module initializing\n");
	SCP_PORT = 0;  // do not use serial controller port
	ROM_Char_Table = ROM_KATHREIN;

	// Initialize display
	// 1. Allocate PIO pins for VFD control
	disp_data = stpio_request_pin(0, 3, "Dispdata", STPIO_OUT);
	if (disp_data == NULL)
	{
		dprintk(1, "%s: Error allocating PIO pin (0,3) for display data\n", __func__);
		goto fail;
	}
	disp_stb = stpio_request_pin(0, 4, "Disp_STB", STPIO_OUT);
	if (disp_stb == NULL)
	{
		dprintk(1, "%s: Error allocating PIO pin (0,4) for display strobe\n", __func__);
		goto fail;
	}
	disp_cs = stpio_request_pin(0, 5, "Disp_xCS", STPIO_OUT);
	if (disp_cs == NULL)
	{
		dprintk(1, "%s: Error allocating PIO pin (0,5) for display chip select\n", __func__);
		goto fail;
	}
	dprintk(50, "PIO pins for display drive control allocated\n");

	memset(vfd_data, 0, sizeof(vfd_data));

	// 2. set display off
	vfd_data[0] = (LIGHT_ON_COMMAND | LIGHT_ON_LS & ~LIGHT_ON_HS);
	ret |= ufs910_fp_Send_Nbytes(vfd_data, 1);

	// 3. Set # of digits
	vfd_data[0] = NUM_DIGIT_COMMAND;
	vfd_data[1] = NUM_DIGIT_16;
	ret |= ufs910_fp_Send_Nbytes(vfd_data, 2);

	// 4. Clear DCRAM (character memory)
	vfd_data[0] = DCRAM_COMMAND | 0;  // start at address 0x00
	memset(vfd_data + 1, 0x20, VFD_LENGTH);  // fill character memory with spaces
	ret |= ufs910_fp_Send_Nbytes(vfd_data, VFD_LENGTH + 1);

	// 5. Clear CGRAM (user defined dot patterns)
	for (i = 0; i < 8; i++)  // CGRAM address counter
	{
		vfd_data[0] = CGRAM_COMMAND | i;
		memset(vfd_data + 1, 0, 5);  // each byte defines one column, there are 5 columns
		ret |= ufs910_fp_Send_Nbytes(vfd_data, 6);
	}

	// 6. Clear ADRAM (icons)
	vfd_data[0] = ADRAM_COMMAND | i;
	for (i = 0; i < VFD_LENGTH; i++)  // ADRAM address counter
	{
		vfd_data[i + 1] = 0;
	}
	ret |= ufs910_fp_Send_Nbytes(vfd_data, VFD_LENGTH + 1);

	// 7. Set brightness
	ret |= ufs910_fp_SetBrightness(7);

#if 0
	// 8. switch display on, all segments (display test)
	vfd_data[0] = LIGHT_ON_COMMAND & ~LIGHT_ON_LS | LIGHT_ON_HS;
	ret |= ufs910_fp_Send_Nbytes(vfd_data, 1);
	msleep(1000);
#endif

	// 9. switch display on, normal
	vfd_data[0] = LIGHT_ON_COMMAND & ~LIGHT_ON_LS & ~LIGHT_ON_HS;
	ret |= ufs910_fp_Send_Nbytes(vfd_data, 1);

	// initialize LED PIOs on 14W model
	if (boxtype != 0)  // 14W model detected
	{
		ledred = stpio_request_pin(0, 2, "ledred", STPIO_OUT);
		if (ledred == NULL)
		{
			dprintk(1, "%s: Error allocating PIO pin (0,2) for red LEDe\n", __func__);
			goto fail;
		}
		stpio_set_pin(ledred, 1);

		ledgreen = stpio_request_pin(0, 0, "ledgreen", STPIO_OUT);
		if (ledgreen == NULL)
		{
			dprintk(1, "%s: Error allocating PIO pin (0,0) for green LED\n", __func__);
			goto fail;
		}
		stpio_set_pin(ledgreen, 1);

		ledorange = stpio_request_pin(0, 1, "ledorang", STPIO_OUT);
		if (ledorange == NULL)
		{
			dprintk(1, "%s: Error allocating PIO pin (0,1) for yellow LED\n", __func__);
			goto fail;
		}
		stpio_set_pin(ledorange, 1);
	}

	// VFD_SetRemote();
fail:
	dprintk(100, "%s <\n", __func__);
	return ret;
}

/***********************************************************
 *
 * Code for writing to /dev/vfd.
 *
 * If text to display is longer than the display
 * width, it is scolled once. Maximum text length
 * is 64 characters.
 *
 */
static ssize_t VFDdev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	unsigned char *kernel_buf = kmalloc(len, GFP_KERNEL);
	int ret = 0;
	int pos, offset, llen;
	unsigned char buf[64 + VFD_LENGTH + 2];  // 64 plus maximum length plus two

	dprintk(100, "%s > (len %d, offs %lld)\n", __func__, len, *off);
	/* konfetti */
	if (kernel_buf == NULL)
	{
		dprintk(1, "%s < return -ENOMEM\n", __func__);
		return -ENOMEM;
	}
	copy_from_user(kernel_buf, buff, len);

	/* Dagobert: echo add a \n which will be counted as a char */
	/* Does not add a LF, strips it when one is present at end */
	llen = len;
	if (kernel_buf[len - 1] == '\n')
	{
		kernel_buf[len - 1] = 0;
		llen--;
	}
	if (llen <= VFD_LENGTH)  // no scroll
	{
		ret != ufs910_fp_WriteString(kernel_buf, llen);
	}
	else  // scroll, display string is longer than display length
	{
		memset(buf, ' ', sizeof(buf));
		// initial display starting at 3rd position to ease reading
		offset = 3;
		memcpy(buf + offset, kernel_buf, llen);
		llen += offset;
		buf[llen + VFD_LENGTH - 1] = '\0';  // terminate string

		for (pos = 1; pos < llen; pos++)
		{
			ret |= ufs910_fp_WriteString(buf + pos, llen + VFD_LENGTH);
			// sleep 300 ms
			msleep(300);
		}
		// final display
		ret |= ufs910_fp_WriteString("", 0);
		ret |= ufs910_fp_WriteString(kernel_buf, VFD_LENGTH);
	}
	kfree(kernel_buf);
	dprintk(100, "%s <\n", __func__);
	if (ret < 0)
	{
		return ret;
	}
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
	if (len > VFD_LENGTH)
	{
		len = VFD_LENGTH;
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
	static int mode = 0;
	int ret = 0;
	int i;
	int which;
	int on;
	struct ufs910_fp_ioctl_data *ufs910_fp = (struct ufs910_fp_ioctl_data *)arg;
	struct vfd_ioctl_data *data = (struct vfd_ioctl_data *)arg;

	dprintk(100, "%s > IOCTL = 0x%.8x\n", __func__, cmd);

	switch (cmd)
	{
		case VFDSETMODE:
		{
			mode = ufs910_fp->u.mode.compat;
			break;
		}
		case VFDSETLED:
		{
			ret = ufs910_fp_SetLED(ufs910_fp->u.led.led_nr, ufs910_fp->u.led.on);
			break;
		}
		case VFDDISPLAYCHARS:
		{
			ufs910_fp_ShowString((struct vfd_ioctl_data *)arg);
			break;
		}
		case VFDBRIGHTNESS:
		{
			on = (mode ? ufs910_fp->u.brightness.level : data->start_address);

			if (on < 0)
			{
				on = 0;
			}
			if (on > 7)
			{
				on = 7;
			}
			dprintk(50, "%s Set display brightness to %d (mode %d)\n", __func__, on, mode);
			ret = ufs910_fp_SetBrightness(on);
			mode = 0;  // go back to vfd mode
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			if (mode == 0)
			{
				on = data->start_address;
			}
			else
			{
				on = ufs910_fp->u.light.onoff;
			}
			on = (on != 0 ? 1 : 0);
			dprintk(50, "Switch display %s\n", (on != 0 ? "on" : "off"));
			ret = ufs910_fp_SetDisplayOnOff(ufs910_fp->u.light.onoff);
			mode = 0;  // go back to vfd mode
			break;
		}
		case VFDDRIVERINIT:
		{
			vfd_init_func();
			mode = 0;  // go back to vfd mode
			break;
		}
		case VFDICONDISPLAYONOFF:
		{
			which = (mode == 0 ? data->data[0] : ufs910_fp->u.icon.icon_nr);
			on = (mode == 0 ? data->data[4] : ufs910_fp->u.icon.on);
			on = on != 0 ? 1 : 0;
			dprintk(50, "Set icon %d to %s (mode %d)\n", which, (on ? "on" : "off"), mode);

			// Part one: translate E2 icon numbers to own icon numbers (vfd mode only)
			if (mode == 0 && which > 255)  // vfd mode
			{
				which >> 8;  // E2 send its icon number in the high byte
				switch (which)
				{
					case 0x13:  // crypted
					{
						which = ICON_SCRAMBLED;
						break;
					}
					case 0x17:  // dolby
					{
						which = ICON_DOLBY;
						break;
					}
					case 0x15:  // MP3
					{
						which = ICON_MP3;
						break;
					}
					case 0x11:  // HD
					{
						which = ICON_HD;
						break;
					}
					case 0x1e:  // record
					{
						which = ICON_RECORD;
						break;
					}
					case 0x1a:  // seekable (play)
					{
						which = ICON_PLAY;
						break;
					}
					default:
					{
						break;
					}
				}
			}

			// Part two: decide whether one icon or all
			switch (which)
			{
				case ICON_MAX:
				{
					for (i = 1; i < ICON_MAX; i++)
					{
						ufs910_fp_SetIcon(i, on);
					}
					break;
				}
				default:
				{
					ufs910_fp_SetIcon(which, on);
					break;
				}
			}
			mode = 0;
			break;
		}
		case VFDCGRAMWRITE1:
		{
			ufs910_fp_CGRAM_Write((struct vfd_ioctl_data *)arg);
			mode = 0;
			break;
		}
		case VFDCGRAMWRITE2:
		{
			ufs910_fp_CGRAM_Write((struct vfd_ioctl_data *)arg);
			mode = 0;
			break;
		}
		case 0x5305:
		{
			mode = 0;
			break;
		}
		default:
		{
			dprintk(1, "%s: Unknown IOCTL 0x%x\n", __func__, cmd);
			break;
		}
	}
	dprintk(100, "%s <\n", __func__);
	return 0;
}

struct file_operations vfd_fops =
{
	.owner   = THIS_MODULE,
	.ioctl   = VFDdev_ioctl,
	.write   = VFDdev_write,
	.read    = VFDdev_read,
	.open    = VFDdev_open,  /* konfetti */
	.release = VFDdev_close
};
// vim:ts=4
