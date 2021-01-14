/****************************************************************************
 *
 * opt9600_fp_file.c
 *
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add irq handling
 * (c) 2020 Audioniek: ported to Opticum HD 9600 (TS)
 *
 * Largely based on cn_micom, enhanced and ported to Opticum HD 9600 (TS)
 * by Audioniek.
 *
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
 ****************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------
 * 20201222 Audioniek       Initial version, based on cn_micom.
 * 20201228 Audioniek       Add Opticum HD 9600 specifics.
 * 20201230 Audioniek       Fix Opticum HD 9600 sstartup sequence.
 * 20201231 Audioniek       /dev/vfd scrolls texts longer than DISPLAY_WIDTH.
 *
 ****************************************************************************/

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/termbits.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
#include <linux/stm/pio.h>
#else
#include <linux/stpio.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/poll.h>
#include <linux/syscalls.h>

#include "opt9600_fp.h"
#include "opt9600_fp_asc.h"
#include "opt9600_fp_utf.h"
#include "opt9600_fp_char.h"

extern void ack_sem_up(void);
extern int  ack_sem_down(void);
int mcom_WriteString(unsigned char *aBuf, int len);
extern void mcom_putc(unsigned char data);

struct semaphore write_sem;

int errorOccured = 0;

static unsigned char ioctl_data[12];

tFrontPanelOpen FrontPanelOpen [LASTMINOR];

struct saved_data_s
{
	int   length;
	char  data[128];
};

#if 1
#define PRN_LOG(msg, args...)        printk(msg, ##args)
#define PRN_DUMP(str, data, len)     mcom_Dump(str, data, len)
#else
#define PRN_LOG(msg, args...)
#define PRN_DUMP(str, data, len)
#endif

static time_t        mcom_time_set_time;
static unsigned char mcom_time[12];

static int           mcom_settime_set_time_flag = 0;
static time_t        mcom_settime_set_time;
static time_t        mcom_settime;

static unsigned char mcom_version[4];
static unsigned char mcom_private[8]; // 0000 0000(Boot from AC on), 0000 0001(Boot from standby), 0000 0002(micom did not use this value, bug)

/**************************************************
 *
 * Character table for LED models
 *
 * Order is ASCII.
 */
static char *mcom_ascii_char_7seg[] =
{
	_7SEG_NULL,   // 0x00, unprintable control character
	_7SEG_NULL,   // 0x01, unprintable control character
	_7SEG_NULL,   // 0x02, unprintable control character
	_7SEG_NULL,   // 0x03, unprintable control character
	_7SEG_NULL,   // 0x04, unprintable control character
	_7SEG_NULL,   // 0x05, unprintable control character
	_7SEG_NULL,   // 0x06, unprintable control character
	_7SEG_NULL,   // 0x07, unprintable control character
	_7SEG_NULL,   // 0x08, unprintable control character
	_7SEG_NULL,   // 0x09, unprintable control character
	_7SEG_NULL,   // 0x0a, unprintable control character
	_7SEG_NULL,   // 0x0b, unprintable control character
	_7SEG_NULL,   // 0x0c, unprintable control character
	_7SEG_NULL,   // 0x0d, unprintable control character
	_7SEG_NULL,   // 0x0e, unprintable control character
	_7SEG_NULL,   // 0x0f, unprintable control character

	_7SEG_NULL,   // 0x10, unprintable control character
	_7SEG_NULL,   // 0x11, unprintable control character
	_7SEG_NULL,   // 0x12, unprintable control character
	_7SEG_NULL,   // 0x13, unprintable control character
	_7SEG_NULL,   // 0x14, unprintable control character
	_7SEG_NULL,   // 0x15, unprintable control character
	_7SEG_NULL,   // 0x16, unprintable control character
	_7SEG_NULL,   // 0x17, unprintable control character
	_7SEG_NULL,   // 0x18, unprintable control character
	_7SEG_NULL,   // 0x19, unprintable control character
	_7SEG_NULL,   // 0x1a, unprintable control character
	_7SEG_NULL,   // 0x1b, unprintable control character
	_7SEG_NULL,   // 0x1c, unprintable control character
	_7SEG_NULL,   // 0x1d, unprintable control character
	_7SEG_NULL,   // 0x1e, unprintable control character
	_7SEG_NULL,   // 0x1f, unprintable control character

	_7SEG_NULL,   // 0x20, <space>
	_7SEG_NULL,   // 0x21, !
	_7SEG_NULL,   // 0x22, "
	_7SEG_NULL,   // 0x23, #
	_7SEG_NULL,   // 0x24, $
	_7SEG_NULL,   // 0x25, %
	_7SEG_NULL,   // 0x26, &
	_7SEG_NULL,   // 0x27, '
	_7SEG_NULL,   // 0x28, (
	_7SEG_NULL,   // 0x29, )
	_7SEG_NULL,   // 0x2a, *
	_7SEG_NULL,   // 0x2b, +
	_7SEG_NULL,   // 0x2c, ,
	_7SEG_NULL,   // 0x2d, -
	_7SEG_NULL,   // 0x2e, .
	_7SEG_NULL,   // 0x2f, /

	_7SEG_NUM_0,  // 0x30, 0,
	_7SEG_NUM_1,  // 0x31, 1,
	_7SEG_NUM_2,  // 0x32, 2,
	_7SEG_NUM_3,  // 0x33, 3,
	_7SEG_NUM_4,  // 0x34, 4,
	_7SEG_NUM_5,  // 0x35, 5,
	_7SEG_NUM_6,  // 0x36, 6,
	_7SEG_NUM_7,  // 0x37, 7,
	_7SEG_NUM_8,  // 0x38, 8,
	_7SEG_NUM_9,  // 0x39, 9,
	_7SEG_NULL,   // 0x3a, :
	_7SEG_NULL,   // 0x3b, ;
	_7SEG_NULL,   // 0x3c, <
	_7SEG_NULL,   // 0x3d, =
	_7SEG_NULL,   // 0x3e, >
	_7SEG_NULL,   // 0x3f, ?

	_7SEG_NULL,   // 0x40, @
	_7SEG_CH_A,   // 0x41, A
	_7SEG_CH_B,   // 0x42, B
	_7SEG_CH_C,   // 0x43, C
	_7SEG_CH_D,   // 0x44, D
	_7SEG_CH_E,   // 0x45, E
	_7SEG_CH_F,   // 0x46, F
	_7SEG_CH_G,   // 0x47, G
	_7SEG_CH_H,   // 0x48, H
	_7SEG_CH_I,   // 0x49, I
	_7SEG_CH_J,   // 0x4a, J
	_7SEG_CH_K,   // 0x4b, K
	_7SEG_CH_L,   // 0x4c, L
	_7SEG_CH_M,   // 0x4d, M
	_7SEG_CH_N,   // 0x4e, N
	_7SEG_CH_O,   // 0x4f, O

	_7SEG_CH_P,   // 0x50, P
	_7SEG_CH_Q,   // 0x51, Q
	_7SEG_CH_R,   // 0x52, R
	_7SEG_CH_S,   // 0x53, S
	_7SEG_CH_T,   // 0x54, T
	_7SEG_CH_U,   // 0x55, U
	_7SEG_CH_V,   // 0x56, V
	_7SEG_CH_W,   // 0x57, W
	_7SEG_CH_X,   // 0x58, X
	_7SEG_CH_Y,   // 0x59, Y
	_7SEG_CH_Z,   // 0x5a, Z
	_7SEG_NULL,   // 0x5b  [
	_7SEG_NULL,   // 0x5c, |
	_7SEG_NULL,   // 0x5d, ]
	_7SEG_NULL,   // 0x5e, ^
	_7SEG_NULL,   // 0x5f, _

	_7SEG_NULL,   // 0x60, `
	_7SEG_CH_A,   // 0x61, a
	_7SEG_CH_B,   // 0x62, b
	_7SEG_CH_C,   // 0x63, c
	_7SEG_CH_D,   // 0x64, d
	_7SEG_CH_E,   // 0x65, e
	_7SEG_CH_F,   // 0x66, f
	_7SEG_CH_G,   // 0x67, g
	_7SEG_CH_H,   // 0x68, h
	_7SEG_CH_I,   // 0x69, i
	_7SEG_CH_J,   // 0x6a, j
	_7SEG_CH_K,   // 0x6b, k
	_7SEG_CH_L,   // 0x6c, l
	_7SEG_CH_M,   // 0x6d, m
	_7SEG_CH_N,   // 0x6e, n
	_7SEG_CH_O,   // 0x6f, o

	_7SEG_CH_P,   // 0x70, p
	_7SEG_CH_Q,   // 0x71, q
	_7SEG_CH_R,   // 0x72, r
	_7SEG_CH_S,   // 0x73, s
	_7SEG_CH_T,   // 0x74, t
	_7SEG_CH_U,   // 0x75, u
	_7SEG_CH_V,   // 0x76, v
	_7SEG_CH_W,   // 0x77, w
	_7SEG_CH_X,   // 0x78, x
	_7SEG_CH_Y,   // 0x79, y
	_7SEG_CH_Z,   // 0x7a, z
	_7SEG_NULL,   // 0x7b, {
	_7SEG_NULL,   // 0x7c, backslash
	_7SEG_NULL,   // 0x7d, }
	_7SEG_NULL,   // 0x7e, ~
	_7SEG_NULL    // 0x7f, <DEL>--> all segments on
};

/**************************************************
 *
 * Character table for early VFD models
 *
 * Order is ASCII.
 *
 * NOTE: lower case letters are translated
 * to uppercase
 */
static unsigned char mcom_ascii_char_14seg[] =
{
	0x00,  // 0x00, unprintable control character
	0x00,  // 0x01, unprintable control character
	0x00,  // 0x02, unprintable control character
	0x00,  // 0x03, unprintable control character
	0x00,  // 0x04, unprintable control character
	0x00,  // 0x05, unprintable control character
	0x00,  // 0x06, unprintable control character
	0x00,  // 0x07, unprintable control character
	0x00,  // 0x08, unprintable control character
	0x00,  // 0x09, unprintable control character
	0x00,  // 0x0a, unprintable control character
	0x00,  // 0x0b, unprintable control character
	0x00,  // 0x0c, unprintable control character
	0x00,  // 0x0d, unprintable control character
	0x00,  // 0x0e, unprintable control character
	0x00,  // 0x0f, unprintable control character

	0x00,  // 0x10, unprintable control character
	0x00,  // 0x11, unprintable control character
	0x00,  // 0x12, unprintable control character
	0x00,  // 0x13, unprintable control character
	0x00,  // 0x14, unprintable control character
	0x00,  // 0x15, unprintable control character
	0x00,  // 0x16, unprintable control character
	0x00,  // 0x17, unprintable control character
	0x00,  // 0x18, unprintable control character
	0x00,  // 0x19, unprintable control character
	0x00,  // 0x1a, unprintable control character
	0x00,  // 0x1b, unprintable control character
	0x00,  // 0x1c, unprintable control character
	0x00,  // 0x1d, unprintable control character
	0x00,  // 0x1e, unprintable control character
	0x00,  // 0x1f, unprintable control character

	0x00,  // 0x20, <space>
	0x00,  // 0x21, !
	0x00,  // 0x22, "
	0x00,  // 0x23, #
	0x00,  // 0x24, $
	0x00,  // 0x25, %
	0x00,  // 0x26, &
	0x00,  // 0x27, '
	0x00,  // 0x28, (
	0x00,  // 0x29, )
	0x00,  // 0x2a, *
	0x00,  // 0x2b, +
	0x00,  // 0x2c, ,
	0x00,  // 0x2d, -
	0x00,  // 0x2e, .
	0x00,  // 0x2f, /

	0x02,  // 0x30, 0
	0x03,  // 0x31, 1
	0x04,  // 0x32, 2
	0x05,  // 0x33, 3
	0x06,  // 0x34, 4
	0x07,  // 0x35, 5
	0x08,  // 0x36, 6
	0x09,  // 0x37, 7
	0x0a,  // 0x38, 8
	0x0b,  // 0x39, 9
	0x00,  // 0x3a, :
	0x00,  // 0x3b, ;
	0x00,  // 0x3c, <
	0x00,  // 0x3d, =
	0x00,  // 0x3e, >
	0x00,  // 0x3f, ?

	0x00,  // 0x40, @
	0x0c,  // 0x41, A
	0x0d,  // 0x42, B
	0x0e,  // 0x43, C
	0x0f,  // 0x44, D
	0x10,  // 0x45, E
	0x11,  // 0x46, F
	0x12,  // 0x47, G
	0x13,  // 0x48, H
	0x14,  // 0x49, I
	0x15,  // 0x4a, J
	0x16,  // 0x4b, K
	0x17,  // 0x4c, L
	0x18,  // 0x4d, M
	0x19,  // 0x4e, N
	0x1a,  // 0x4f, O

	0x1b,  // 0x50, P
	0x1c,  // 0x51, Q
	0x1d,  // 0x52, R
	0x1e,  // 0x53, S
	0x1f,  // 0x54, T
	0x20,  // 0x55, U
	0x21,  // 0x56, V
	0x22,  // 0x57, W
	0x23,  // 0x58, X
	0x24,  // 0x59, Y
	0x25,  // 0x5a, Z
	0x00,  // 0x5b  [
	0x00,  // 0x5c, |
	0x00,  // 0x5d, ]
	0x00,  // 0x5e, ^
	0x00,  // 0x5f, _

	0x00,  // 0x60, `
	0x0c,  // 0x61, a
	0x0d,  // 0x62, b
	0x0e,  // 0x63, c
	0x0f,  // 0x64, d
	0x10,  // 0x65, e
	0x11,  // 0x66, f
	0x12,  // 0x67, g
	0x13,  // 0x68, h
	0x14,  // 0x69, i
	0x15,  // 0x6a, j
	0x16,  // 0x6b, k
	0x17,  // 0x6c, l
	0x18,  // 0x6d, m
	0x19,  // 0x6e, n
	0x1a,  // 0x6f, o
               
	0x1b,  // 0x70, p
	0x1c,  // 0x71, q
	0x1d,  // 0x72, r
	0x1e,  // 0x73, s
	0x1f,  // 0x74, t
	0x20,  // 0x75, u
	0x21,  // 0x76, v
	0x22,  // 0x77, w
	0x23,  // 0x78, x
	0x24,  // 0x79, y
	0x25,  // 0x7a, z
	0x00,  // 0x7b, {
	0x00,  // 0x7c, backslash
	0x00,  // 0x7d, }
	0x00,  // 0x7e, ~
	0x00,  // 0x7f, <DEL>
};

/**************************************************
 *
 * Character table for late VFD models
 *
 * Order is ASCII.
 */
static char *mcom_ascii_char_14seg_new[] =
{
	_14SEG_NULL,   // 0x00, unprintable control character
	_14SEG_NULL,   // 0x01, unprintable control character
	_14SEG_NULL,   // 0x02, unprintable control character
	_14SEG_NULL,   // 0x03, unprintable control character
	_14SEG_NULL,   // 0x04, unprintable control character
	_14SEG_NULL,   // 0x05, unprintable control character
	_14SEG_NULL,   // 0x06, unprintable control character
	_14SEG_NULL,   // 0x07, unprintable control character
	_14SEG_NULL,   // 0x08, unprintable control character
	_14SEG_NULL,   // 0x09, unprintable control character
	_14SEG_NULL,   // 0x0a, unprintable control character
	_14SEG_NULL,   // 0x0b, unprintable control character
	_14SEG_NULL,   // 0x0c, unprintable control character
	_14SEG_NULL,   // 0x0d, unprintable control character
	_14SEG_NULL,   // 0x0e, unprintable control character
	_14SEG_NULL,   // 0x0f, unprintable control character

	_14SEG_NULL,   // 0x10, unprintable control character
	_14SEG_NULL,   // 0x11, unprintable control character
	_14SEG_NULL,   // 0x12, unprintable control character
	_14SEG_NULL,   // 0x13, unprintable control character
	_14SEG_NULL,   // 0x14, unprintable control character
	_14SEG_NULL,   // 0x15, unprintable control character
	_14SEG_NULL,   // 0x16, unprintable control character
	_14SEG_NULL,   // 0x17, unprintable control character
	_14SEG_NULL,   // 0x18, unprintable control character
	_14SEG_NULL,   // 0x19, unprintable control character
	_14SEG_NULL,   // 0x1a, unprintable control character
	_14SEG_NULL,   // 0x1b, unprintable control character
	_14SEG_NULL,   // 0x1c, unprintable control character
	_14SEG_NULL,   // 0x1d, unprintable control character
	_14SEG_NULL,   // 0x1e, unprintable control character
	_14SEG_NULL,   // 0x1f, unprintable control character

	_14SEG_NULL,   // 0x20, <space>
	_14SEG_NULL,   // 0x21, !
	_14SEG_NULL,   // 0x22, "
	_14SEG_NULL,   // 0x23, #
	_14SEG_NULL,   // 0x24, $
	_14SEG_NULL,   // 0x25, %
	_14SEG_NULL,   // 0x26, &
	_14SEG_NULL,   // 0x27, '
	_14SEG_NULL,   // 0x28, (
	_14SEG_NULL,   // 0x29, )
	_14SEG_NULL,   // 0x2a, *
	_14SEG_NULL,   // 0x2b, +
	_14SEG_NULL,   // 0x2c, ,
	_14SEG_NULL,   // 0x2d, -
	_14SEG_NULL,   // 0x2e, .
	_14SEG_NULL,   // 0x2f, /

	_14SEG_NUM_0,  // 0x30, 0
	_14SEG_NUM_1,  // 0x31, 1
	_14SEG_NUM_2,  // 0x32, 2
	_14SEG_NUM_3,  // 0x33, 3
	_14SEG_NUM_4,  // 0x34, 4
	_14SEG_NUM_5,  // 0x35, 5
	_14SEG_NUM_6,  // 0x36, 6
	_14SEG_NUM_7,  // 0x37, 7
	_14SEG_NUM_8,  // 0x38, 8
	_14SEG_NUM_9,  // 0x39, 9
	_14SEG_NULL,   // 0x3a, :
	_14SEG_NULL,   // 0x3b, ;
	_14SEG_NULL,   // 0x3c, <
	_14SEG_NULL,   // 0x3d, =
	_14SEG_NULL,   // 0x3e, >
	_14SEG_NULL,   // 0x3f, ?

	_14SEG_NULL,   // 0x40, @
	_14SEG_CH_A,   // 0x41, A
	_14SEG_CH_B,   // 0x42, B
	_14SEG_CH_C,   // 0x43, C
	_14SEG_CH_D,   // 0x44, D
	_14SEG_CH_E,   // 0x45, E
	_14SEG_CH_F,   // 0x46, F
	_14SEG_CH_G,   // 0x47, G
	_14SEG_CH_H,   // 0x48, H
	_14SEG_CH_I,   // 0x49, I
	_14SEG_CH_J,   // 0x4a, J
	_14SEG_CH_K,   // 0x4b, K
	_14SEG_CH_L,   // 0x4c, L
	_14SEG_CH_M,   // 0x4d, M
	_14SEG_CH_N,   // 0x4e, N
	_14SEG_CH_O,   // 0x4f, O

	_14SEG_CH_P,   // 0x50, P
	_14SEG_CH_Q,   // 0x51, Q
	_14SEG_CH_R,   // 0x52, R
	_14SEG_CH_S,   // 0x53, S
	_14SEG_CH_T,   // 0x54, T
	_14SEG_CH_U,   // 0x55, U
	_14SEG_CH_V,   // 0x56, V
	_14SEG_CH_W,   // 0x57, W
	_14SEG_CH_X,   // 0x58, X
	_14SEG_CH_Y,   // 0x59, Y
	_14SEG_CH_Z,   // 0x5a, Z
	_14SEG_NULL,   // 0x5b  [
	_14SEG_NULL,   // 0x5c, |
	_14SEG_NULL,   // 0x5d, ]
	_14SEG_NULL,   // 0x5e, ^
	_14SEG_NULL,   // 0x5f, _

	_14SEG_NULL,   // 0x60, `
	_14SEG_CH_A,   // 0x61, a
	_14SEG_CH_B,   // 0x62, b
	_14SEG_CH_C,   // 0x63, c
	_14SEG_CH_D,   // 0x64, d
	_14SEG_CH_E,   // 0x65, e
	_14SEG_CH_F,   // 0x66, f
	_14SEG_CH_G,   // 0x67, g
	_14SEG_CH_H,   // 0x68, h
	_14SEG_CH_I,   // 0x69, i
	_14SEG_CH_J,   // 0x6a, j
	_14SEG_CH_K,   // 0x6b, k
	_14SEG_CH_L,   // 0x6c, l
	_14SEG_CH_M,   // 0x6d, m
	_14SEG_CH_N,   // 0x6e, n
	_14SEG_CH_O,   // 0x6f, o

	_14SEG_CH_P,   // 0x70, p
	_14SEG_CH_Q,   // 0x71, q
	_14SEG_CH_R,   // 0x72, r
	_14SEG_CH_S,   // 0x73, s
	_14SEG_CH_T,   // 0x74, t
	_14SEG_CH_U,   // 0x75, u
	_14SEG_CH_V,   // 0x76, v
	_14SEG_CH_W,   // 0x77, w
	_14SEG_CH_X,   // 0x78, x
	_14SEG_CH_Y,   // 0x79, y
	_14SEG_CH_Z,   // 0x7a, z
	_14SEG_NULL,   // 0x7b, {
	_14SEG_NULL,   // 0x7c, backslash
	_14SEG_NULL,   // 0x7d, }
	_14SEG_NULL,   // 0x7e, ~
	_14SEG_NULL    // 0x7f, <DEL>--> all segments on
};

static struct saved_data_s lastdata;

/****************************************************************************
 *
 * Driver code.
 *
 */

/*******************************************************
 *
 * Code to communicate with the front processor.
 *
 */
static void mcom_Dump(char *string, unsigned char *v_pData, unsigned short v_szData)
{
	int i;
	char str[10], line[80], txt[25];
	unsigned char    *pData;

	if ((v_pData == NULL) || (v_szData == 0))
	{
		return;
	}

	printk("[ %s [%x]]\n", string, v_szData);

	i = 0;
	pData = v_pData;
	line[0] = '\0';
	while ((unsigned short)(pData - v_pData) < v_szData)
	{
		sprintf(str, "%02x ", *pData);
		strcat(line, str);
		txt[i] = ((*pData >= 0x20) && (*pData < 0x7F))  ? *pData : '.';
		i++;

		if (i == 16)
		{
			txt[i] = '\0';
			printk("%s %s\n", line, txt);
			i = 0;
			line[0] = '\0';
		}
		pData++;
	}
	if (i != 0)
	{
		txt[i] = '\0';
		do
		{
			strcat(line, "   ");
		}
		while (++i < 16);

		printk("%s %s\n\n", line, txt);
	}
}

void write_sem_up(void)
{
	up(&write_sem);
}

int write_sem_down(void)
{
	return down_interruptible(&write_sem);
}

void copyData(unsigned char *data, int len)
{
	dprintk(150, "%s > len %d\n", __func__, len);
	memcpy(ioctl_data, data, len);
}

/*****************************************************************
 *
 * mcom_WriteCommand: Send command to front processor.
 *
 */
int mcomWriteCommand(char *buffer, int len, int needAck)
{
	int i;

	dprintk(150, "%s >\n", __func__);

	if (paramDebug > 150)
	{
		dprintk(150, "Send Front uC command ");
		for (i = 0; i < len; i++)
		{
			printk("0x%02x ", buffer[i] & 0xff);
		}
		printk("\n");
	}

	for (i = 0; i < len; i++)
	{
#ifdef DIRECT_ASC  // not defined anywhere
		serial_putc(buffer[i]);
#else
		mcom_putc(buffer[i]);
#endif
	}

	if (needAck)
	{
		if (ack_sem_down())
		{
			return -ERESTARTSYS;
		}
	}
	dprintk(150, "%s < \n", __func__);
	return 0;
}

/*****************************************************************
 *
 * mcom_Checksum: Calculate checksum byte over payload.
 *
 */
static unsigned char mcom_Checksum(unsigned char *payload, int len)
{
	int           n;
	unsigned char checksum = 0;

	for (n = 0; n < len; n++)
	{
		checksum ^= payload[n];
	}
	return checksum;
}

/*****************************************************************
 *
 * mcom_Checksum: Send checksum response string.
 *
 */
static int mcom_SendResponse(char *buf, int needack)
{
	unsigned char    response[3];
	int                res = 0;

	dprintk(150, "%s >\n", __func__);

	response[_TAG] = _MCU_RESPONSE;
	response[_LEN] = 1;
	response[_VAL] = mcom_Checksum(buf, 2 + buf[_LEN]);

	if (needack)
	{
		errorOccured = 0;
		res = mcomWriteCommand((char *)response, 3, 1);

		if (errorOccured == 1)
		{
			/* error */
			memset(ioctl_data, 0, 8);
			dprintk(1, "%s: Timeout on ack occurred\n",__func__);
			res = -ETIMEDOUT;
		}
	}
	else
	{
		res = mcomWriteCommand((char *)response, 3, 0);
	}
	dprintk(150, "%s <\n", __func__);
	return res;
}

/*****************************************************************
 *
 * mcom_GetDisplayType: Return type of frontpanel display.
 *
 */
static DISPLAYTYPE mcomGetDisplayType(void)
{
	if (mcom_version[0] == 4)
	{
		return _7SEG;
	}
	if (mcom_version[0] == 5)
	{
		return _VFD;
	}
	if (mcom_version[0] == 2)
	{
		return _VFD_OLD;  // old VFD
	}
	return _VFD_OLD;
}

/*******************************************************
 *
 * Time routines.
 *
 */
static unsigned short mcomYMD2MJD(unsigned short Y, unsigned char M, unsigned char D)
{  // converts YMD to 16 bit MJD
	int L;

	Y -= 1900;
	L = ((M == 1) || (M == 2)) ? 1 : 0 ;
	return (unsigned short)(14956 + D + ((Y - L) * 36525 / 100) + ((M + 1 + L * 12) * 306001 / 10000));
}

static void mcom_MJD2YMD(unsigned short usMJD, unsigned short *year, unsigned char *month, unsigned char *day)
{  // converts MJD to to year month day
	int Y, M, D, K;

	Y = (usMJD * 100 - 1507820) / 36525;
	M = ((usMJD * 10000 - 149561000) - (Y * 36525 / 100) * 10000) / 306001;
	D = usMJD - 14956 - (Y * 36525 / 100) - (M * 306001 / 10000);
	K = ((M == 14) || (M == 15)) ? 1 : 0;

	Y = Y + K + 1900;
	M = M - 1 - K * 12;

	*year  = (unsigned short)Y;
	*month = (unsigned char)M;
	*day   = (unsigned char)D;
}

/*******************************************************
 *
 * mcomSetStandby: put receiver in (deep) standby.
 *
 * time = wake up time ?
 */
int mcomSetStandby(char *time)
{
	char comm_buf[16];
	int  res = 0;
	int  showTime = 0;
	int  wakeup = 0;

	dprintk(150, "%s >\n", __func__);

	if (mcomGetDisplayType() == _VFD_OLD || mcomGetDisplayType() == _VFD)
	{
		res = mcom_WriteString("Bye bye", strlen("Bye bye"));
	}
	else
	{
		res = mcom_WriteString("WAIT", strlen("WAIT"));
	}
	memset(comm_buf, 0, sizeof(comm_buf));

	// send "STANDBY"
	comm_buf[_TAG] = _MCU_STANDBY;
	comm_buf[_LEN] = 5;
	comm_buf[_VAL + 0] = (showTime) ? 1 : 0;  // flag: show time in standby?
	comm_buf[_VAL + 1] = 1;  // 24h flag?
	comm_buf[_VAL + 2] = (wakeup) ? 1 : 0;
	comm_buf[_VAL + 3] = 3;  // power on delay
	comm_buf[_VAL + 4] = 1;  // extra data (private data)

//	PRN_DUMP("@@@ SEND STANDBY @@@", comm_buf, 2 + comm_buf[_LEN]);
	res = mcomWriteCommand(comm_buf, 7, 0);

	if (res != 0)
	{
		goto _EXIT_MCOM_STANDBY;
	}
	msleep(100);

	// send "NOWTIME"
	comm_buf[_TAG] = _MCU_NOWTIME;
	comm_buf[_LEN] = 5;

	if (mcom_settime_set_time_flag)
	{
		unsigned short  mjd;
		unsigned short  year;
		unsigned char   month;
		unsigned char   day;
		time_t          curr_time;
		struct timespec tp;

		tp = current_kernel_time();  // get system time

//		PRN_LOG("mcomSetTime: %d\n", tp.tv_sec);
		curr_time = mcom_settime + (tp.tv_sec - mcom_settime_set_time);
		mjd = (curr_time / (24 * 60 * 60)) + 40587;

		mcom_MJD2YMD(mjd, &year, &month, &day);  // calculate MJD

		comm_buf[_VAL + 0] = (year - 2000) & 0xff;  // strip century
		comm_buf[_VAL + 1] = month;
		comm_buf[_VAL + 2] = day;
		comm_buf[_VAL + 3] = (curr_time / (24 * 60 * 60)) / (60 * 60);
		comm_buf[_VAL + 4] = ((curr_time / (24 * 60 * 60)) % (60 * 60)) / 60;

#if 0
		mjd      = (mcom_settime[0] << 8) | mcom_settime[1];
		time_sec =  mcom_settime[2] * 60 * 60
		         +  mcom_settime[3] * 60
		         +  mcom_settime[4]
		         + (tp.tv_sec - mcom_settime_set_time);

		mjd          += time_sec / (24 * 60 * 60);
		curr_time_sec = time_sec % (24 * 60 * 60);

		mcom_MJD2YMD(mjd, &year, &month, &day);

		comm_buf[_VAL + 0] = (year - 2000) & 0xff;
		comm_buf[_VAL + 1] = month;
		comm_buf[_VAL + 2] = day;
		comm_buf[_VAL + 3] = curr_time_sec / (60 * 60);
		comm_buf[_VAL + 4] = (curr_time_sec % (60 * 60)) / 60;
#endif
	}
	else
	{

	}
	PRN_DUMP("@@@ SEND TIME @@@", comm_buf, 2 + comm_buf[_LEN]);

	res = mcomWriteCommand(comm_buf, 7, 0);
	if (res != 0)
	{
		goto _EXIT_MCOM_STANDBY;
	}
	msleep(100);

	if (wakeup)  // 
	{
		/* TO BE DONE!! */
		// send "WAKEUP"
		comm_buf[_TAG] = _MCU_WAKEUP;

		comm_buf[_LEN] = 3;
#if 1
		comm_buf[_VAL + 0] = 0xFF;
		comm_buf[_VAL + 1] = 0xFF;
		comm_buf[_VAL + 2] = 0xFF;
#else
		comm_buf[_VAL + 0] = (after_min >> 16) & 0xFF;
		comm_buf[_VAL + 1] = (after_min >>  8) & 0xFF;
		comm_buf[_VAL + 2] = (after_min >>  0) & 0xFF;
#endif
		PRN_DUMP("@@@ SEND WAKEUP @@@", comm_buf, 2 + comm_buf[_LEN]);
		res = mcomWriteCommand(comm_buf, 5, 0);
		if (res != 0)
		{
			goto _EXIT_MCOM_STANDBY;
		}
		msleep(100);
	}

	// send "PRIVATE"
	comm_buf[_TAG] = _MCU_PRIVATE;
	comm_buf[_LEN] = 8;
	comm_buf[_VAL + 0] = 0;
	comm_buf[_VAL + 1] = 1;
	comm_buf[_VAL + 2] = 2;
	comm_buf[_VAL + 3] = 3;
	comm_buf[_VAL + 4] = 4;
	comm_buf[_VAL + 5] = 5;
	comm_buf[_VAL + 6] = 6;
	comm_buf[_VAL + 7] = 7;

	memset(&comm_buf + _VAL, 1, 8);
	res = mcomWriteCommand(comm_buf, 10, 0);
	if (res != 0)
	{
		goto _EXIT_MCOM_STANDBY;
	}
	msleep(100);

_EXIT_MCOM_STANDBY:
	dprintk(100, "%s <\n", __func__);
	return res;
}

/*******************************************************
 *
 * mcomSetTime: set front processor time.
 *
 */
int mcomSetTime(time_t time)
{
	struct timespec tp;

	tp = current_kernel_time();
	PRN_LOG("mcomSetTime: %d\n", tp.tv_sec);
	mcom_settime_set_time = tp.tv_sec;
	mcom_settime = time;
	mcom_settime_set_time_flag = 1;
	return 0;
}

/*******************************************************
 *
 * mcomGetTime: get front processor time.
 *
 */
int mcomGetTime(void)
{
	unsigned short  cur_mjd;
	unsigned int    cur_time_in_sec;
	unsigned int    time_in_sec;
	unsigned int    passed_time_in_sec;
	struct timespec tp_now;

	cur_mjd = mcomYMD2MJD(mcom_time[0] + 2000, mcom_time[1], mcom_time[2]);

	time_in_sec =   mcom_time[3] * 60 * 60
	            +   mcom_time[4] * 60
	            + ((mcom_time[5] << 16) | (mcom_time[6] << 8) | mcom_time[7]) * 60
	            + (tp_now.tv_sec - mcom_time_set_time);

	cur_mjd        += time_in_sec / (24 * 60 * 60);
	cur_time_in_sec = time_in_sec % (24 * 60 * 60);

	ioctl_data[0] = cur_mjd >> 8;
	ioctl_data[1] = cur_mjd & 0xff;
	ioctl_data[2] = cur_time_in_sec / (60 * 60);
	ioctl_data[3] = (cur_time_in_sec % (60 * 60)) / 60;
	ioctl_data[4] = (cur_time_in_sec % (60 * 60)) % 60;
	return 0;
}

/*******************************************************
 *
 * mcomGetWakeUpMode: get wake up reason.
 *
 * NOTE: currently hard coded to power on.
 */
int mcomGetWakeUpMode(void)
{
	if (mcom_version[0] == 4)  // 4.x.x
	{
		ioctl_data[0] = 0;
	}
	else if (mcom_version[0] == 5)  // 5.x.x
	{
		ioctl_data[0] = 0;
	}
	else if (mcom_version[0] == 2)  // 2.x.x
	{
		ioctl_data[0] = 0;
	}
	else
	{
		ioctl_data[0] = 0;
	}
	return 0;
}

/*****************************************************************
 *
 * mcom_WriteString: Display a text on the front panel display.
 *
 * Note: does not scroll; displays the first DISPLAY_WIDTH
 *       characters. Scrolling is available by writing to
 *       /dev/vfd: this scrolls a maximum of 64 characters once
 *       if the text length exceeds DISPLAY_WIDTH.
 *
 */
int mcom_WriteString(unsigned char *aBuf, int len)
{
	unsigned char bBuf[12];
	int i = 0;
	int j = 0;
	int payload_len;
	int res = 0;

	dprintk(150, "%s >\n", __func__);

	memset(bBuf, ' ', sizeof(bBuf));
	payload_len = 0;

	//TODO: insert UTF-8 processing here

	if (len > DISPLAY_WIDTH)
	{
		len = DISPLAY_WIDTH;  // display DISPLAY_WIDTH characters maximum
	}
	dprintk(50, "Display text: [%s] len = %d\n", aBuf, len);

	for (i = 0; i < len; i++)
	{
		// workaround start
		if (aBuf[i] > 0x7f)  // skip all characters with MSbit set (should not occur after UTF-8 handling) 
		{
			continue;
		}
		// workaround end

		// build command argument
		if (mcomGetDisplayType() == _VFD_OLD)  // old VFD display (HD 9600 models)
		{
			bBuf[i + _VAL] = mcom_ascii_char_14seg[aBuf[i]];
			payload_len++;
		}
		else if (mcomGetDisplayType() == _VFD)  // new VFD display (HD 9600 PRIMA models)
		{
			memcpy(bBuf + _VAL + payload_len, mcom_ascii_char_14seg_new[aBuf[i]], 2);
			payload_len += 2;
		}
		else if (mcomGetDisplayType() == _7SEG)  // 4 digit 7-Seg display (HD 9600 Mini models)
		{
			memcpy(bBuf + _VAL + payload_len, mcom_ascii_char_7seg[aBuf[i]], 1);
			payload_len++;
		}
		else
		{
			// Unknown
		}
	}
	/* complete command write */
	bBuf[0] = _MCU_DISPLAY;
	bBuf[1] = payload_len;

	res = mcomWriteCommand(bBuf, payload_len + 2, 0);
	mcom_SendResponse(bBuf, 0);

	/* save last string written to fp */
	memcpy(&lastdata.data, aBuf, DISPLAY_WIDTH);  // save display text for /dev/fd read
	lastdata.length = len;  // save length  for /dev/fd read

	dprintk(150, "%s <\n", __func__);

	return res;
}

/****************************************************************
 *
 * mcom_init_func: Initialize the driver.
 *
 */
int mcom_init_func(void)
{
	unsigned char comm_buf[20];
	unsigned char k;
	unsigned char checksum;
	int           res;
	int           bootSendCount;
#if !defined(SUPPORT_MINILINE)
	int           seq_count = -1;
#endif

	dprintk(150, "%s >\n", __func__);
	sema_init(&write_sem, 1);

//MCOM_RECOVER :
#if 0
#if !defined(SUPPORT_MINILINE)
	seq_count++;
	if (seq_count > 0)
	{
		mcom_FlushData();
		if ((seq_count % 10) == 0)
		{
			mcom_SetupRetryDisplay(seq_count);
			msleep(1000 * 2);
		}
	}
#endif
#endif

	bootSendCount = 0;

	while (1)
	{
		// send "BOOT"
		comm_buf[_TAG] = _MCU_BOOT;
		comm_buf[_LEN] = 1;
		comm_buf[_VAL] = 0x01;  // dummy

		errorOccured = 0;
		dprintk(50, "Send BOOT command (0x%02x 0x%02x 0x%02x)\n", comm_buf[_TAG], comm_buf[_LEN], comm_buf[_VAL]);
		res = mcomWriteCommand(comm_buf, 3, 1);

		if (errorOccured == 1)
		{
			/* error */
			memset(ioctl_data, 0, 8);
			dprintk(1, "%s: Error sending BOOT command\n");

			msleep(100);
			goto MCOM_RECOVER;
		}

		// receive "TIME" or "VERSION"
		if (ioctl_data[_TAG] == EVENT_ANSWER_TIME)
		{
//			struct timespec     tp;
//
//			tp = current_kernel_time();
//			mcom_time_set_time = tp.tv_sec;
			memcpy(mcom_time, ioctl_data + _VAL, ioctl_data[_LEN]);
			dprintk(50, "Front panel uC time %02X-%02X-20%02X %02X:%02X:%02X\n", mcom_time[2], mcom_time[1], mcom_time[0], mcom_time[3], mcom_time[4], mcom_time[5]);
			break;
		}
		else if (ioctl_data[_TAG] == EVENT_ANSWER_VERSION)
		{
			memcpy(mcom_version, ioctl_data + _VAL, ioctl_data[_LEN]);
			dprintk(50, "Front panel uC version = %X.%02X.%02X\n", ioctl_data[2], ioctl_data[3], ioctl_data[4]);
			break;
		}

		if (bootSendCount++ == 3)
		{
			goto MCOM_RECOVER;
		}
		msleep(100);
	}
	// send "RESPONSE" on reception of "TIME" or "VERSION"
	res = mcom_SendResponse(ioctl_data, 1);
	if (res)
	{
		msleep(100);
		goto MCOM_RECOVER;
	}

	// receive "VERSION" or "TIME"
	if (ioctl_data[_TAG] == EVENT_ANSWER_VERSION)
	{
		memcpy(mcom_version, ioctl_data + _VAL, ioctl_data[_LEN]);
		dprintk(50, "Front panel uC version = %X.%02X.%02X\n", mcom_version[0], mcom_version[1], mcom_version[2]);
	}
	else if (ioctl_data[_TAG] == EVENT_ANSWER_TIME)
	{
		struct timespec tp;

		tp = current_kernel_time();
		mcom_time_set_time = tp.tv_sec;
		memcpy(mcom_time, ioctl_data + _VAL, ioctl_data[_LEN]);
		dprintk(50, "%s Front panel uC time %02x-%02x-%02x %02x:%02x:%02x\n", __func__, ioctl_data[2], ioctl_data[3], ioctl_data[4], ioctl_data[5], ioctl_data[6], ioctl_data[7]);
	}
	else
	{
		msleep(100);
		goto MCOM_RECOVER;
	}

	if (mcom_version[0] == 2)  // _VFD_OLD
	{
		// send "RESPONSE" on reception of "VERSION" or "TIME"
		res = mcom_SendResponse(ioctl_data, 0);
		if (res)
		{
			msleep(100);
			goto MCOM_RECOVER;
		}
	}
	else
	{
		// send "RESPONSE" on reception of "VERSION" or "TIME"
		res = mcom_SendResponse(ioctl_data, 1);
		if (res)
		{
			msleep(100);
			goto MCOM_RECOVER;
		}

		// receive "PRIVATE"
		PRN_DUMP("## RECV:PRIVATE ##", ioctl_data, ioctl_data[_LEN] + 2);
		memcpy(mcom_private, ioctl_data + _VAL, ioctl_data[_LEN]);

		mcom_SendResponse(ioctl_data, 0);
		msleep(100);

		for (k = 0; k < 3; k++)
		{
			// send "KEYCODE"
			comm_buf[_TAG] = _MCU_KEYCODE;
			comm_buf[_LEN] = 4;
			comm_buf[_VAL + 0] = 0x04;
			comm_buf[_VAL + 1] = 0xF3;
			comm_buf[_VAL + 2] = 0x5F;
			comm_buf[_VAL + 3] = 0xA0;

			checksum = mcom_Checksum(comm_buf, 6);

			errorOccured = 0;
//			PRN_DUMP("## SEND:KEYCODE ##", comm_buf, 2/*tag+len*/ + n);
			res = mcomWriteCommand(comm_buf, 6, 1);

			if (errorOccured == 1)
			{
				/* error */
				memset(ioctl_data, 0, 8);
				dprintk(1, "Error sending KEYCODE command\n");

				msleep(100);
				goto MCOM_RECOVER;
			}
			if (res == 0)
			{
				if (checksum == ioctl_data[_VAL])
				{
					break;
				}
			}
		}
		if (k >= 3)
		{
//			msleep(100);
			goto MCOM_RECOVER;
		}
	}
	dprintk(150, "%s <\n", __func__);
	return 0;

MCOM_RECOVER:
	dprintk(1, "%s < Error: Front processor is not responding.\n", __func__);
	return -1;
}

/****************************************
 *
 * Code for writing to /dev/vfd
 *
 */
void clear_display(void)
{
	unsigned char bBuf[8];
	int res = 0;

	dprintk(150, "%s >\n", __func__);

	memset(bBuf, ' ', sizeof(bBuf));
	res = mcom_WriteString(bBuf, DISPLAY_WIDTH);
	dprintk(150, "%s <\n", __func__);
}

static ssize_t MCOMdev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char *kernel_buf;
	int minor;
	int vLoop;
	int res = 0;
	int llen;
	int offset = 0;
	char buf[64];
	char *b;

	dprintk(150, "%s >\n", __func__);
	dprintk(100, "%s len %d, offs %d\n", __func__, len, (int) *off);

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
		dprintk(1, "Error: Bad Minor\n");
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
		dprintk(1, "%s < Return -ENOMEM\n", __func__);
		return -ENOMEM;
	}

	copy_from_user(kernel_buf, buff, len);

	if (write_sem_down())
	{
		return -ERESTARTSYS;
	}
	llen = len;  // preserve len for return value

	if (llen >= 64)  // do not display more than 64 characters
	{
		llen = 64;
	}
	// Strip possible trailing LF
	if (kernel_buf[llen - 1] == '\n')
	{
		llen--;
	}
	kernel_buf[llen] = 0;
//	if (llen <= DISPLAY_WIDTH)  // no scroll
//	{
		res = mcom_WriteString(kernel_buf, llen);
//	}
#if 0
	else  // scroll, display string is longer than display length
	{
		// initial display starting at 2nd position to ease reading
		memset(buf, ' ', sizeof(buf));
		offset = 2;
		memcpy(buf + offset, kernel_buf, llen);
		llen += offset;
		buf[llen + DISPLAY_WIDTH] = '\0';  // terminate string

		// scroll text
		b = buf;
		for (vLoop = 0; vLoop < llen; vLoop++)
		{
			res = mcom_WriteString(b + vLoop, DISPLAY_WIDTH);
			// sleep 300 ms
			msleep(300);
		}
		clear_display();

		// final display
		res = mcom_WriteString(kernel_buf, DISPLAY_WIDTH);
	}
#endif
	kfree(kernel_buf);
	write_sem_up();
	dprintk(70, "%s < res=%d len=%d\n", __func__, res, len);

	if (res < 0)
	{
		return res;
	}
	else
	{
		return len;
	}
}

/**********************************
 *
 * Code for reading from /dev/vfd
 *
 */
static ssize_t MCOMdev_read(struct file *filp, char __user *buff, size_t len, loff_t *off)
{
	int minor, vLoop;

	dprintk(150, "%s > (len = %d, offs = %d)\n", __func__, len, (int) *off);

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
		return -EUSERS;
	}
	dprintk(100, "minor = %d\n", minor);

	if (minor == FRONTPANEL_MINOR_RC)
	{
		int           size = 0;
		unsigned char data[20];

		memset(data, 0, sizeof(data));

		getRCData(data, &size);

		if (size > 0)
		{
			if (down_interruptible(&FrontPanelOpen[minor].sem))
			{
				return -ERESTARTSYS;
			}
			copy_to_user(buff, data, size);

			up(&FrontPanelOpen[minor].sem);

			dprintk(100, "%s < %d\n", __func__, size);
			return size;
		}
		//printk("size = %d\n", size);
		dumpValues();
		return 0;
	}
	/* copy the current display string to the user */
	if (down_interruptible(&FrontPanelOpen[minor].sem))
	{
		printk("%s < Return -ERESTARTSYS\n", __func__);
		return -ERESTARTSYS;
	}
	if (FrontPanelOpen[minor].read == lastdata.length)
	{
		FrontPanelOpen[minor].read = 0;

		up(&FrontPanelOpen[minor].sem);
		dprintk(150, "%s <\n", __func__);
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
	dprintk(150, "%s < (len = %d)\n", __func__, len);
	return len;
}

int MCOMdev_open(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(150, "%s >\n", __func__);

	/* needed! otherwise a race condition can occur */
	if (down_interruptible(&write_sem))
	{
		return -ERESTARTSYS;
	}
	minor = MINOR(inode->i_rdev);

	dprintk(70, "Open minor %d\n", minor);

	if (FrontPanelOpen[minor].fp != NULL)
	{
		dprintk(1, "%s < return -EUSERS\n", __func__);
		up(&write_sem);
		return -EUSERS;
	}
	FrontPanelOpen[minor].fp = filp;
	FrontPanelOpen[minor].read = 0;
	up(&write_sem);
	dprintk(150, "%s <\n", __func__);
	return 0;

}

int MCOMdev_close(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(150, "%s >\n", __func__);

	minor = MINOR(inode->i_rdev);

	dprintk(70, "Close minor %d\n", minor);

	if (FrontPanelOpen[minor].fp == NULL)
	{
		dprintk(1, "%s < return -EUSERS\n", __func__);
		return -EUSERS;
	}
	FrontPanelOpen[minor].fp = NULL;
	FrontPanelOpen[minor].read = 0;

	dprintk(150, "%s <\n", __func__);
	return 0;
}

/****************************************
 *
 * IOCTL handling.
 *
 */
static int MCOMdev_ioctl(struct inode *Inode, struct file *File, unsigned int cmd, unsigned long arg)
{
	struct opt9600_fp_ioctl_data *mcom = (struct opt9600_fp_ioctl_data *)arg;
	int res = 0;

	dprintk(150, "%s > 0x%.8x\n", __func__, cmd);

	if (down_interruptible(&write_sem))
	{
		return -ERESTARTSYS;
	}

	switch (cmd)
	{
		case VFDDRIVERINIT:
		{
			res = mcom_init_func();
			break;
		}
		case VFDSTANDBY:
		{
			res = mcomSetStandby(mcom->u.standby.time);
			break;
		}
		case VFDSETTIME:
		{
			if (mcom->u.time.localTime != 0)
			{
				res = mcomSetTime(mcom->u.time.localTime);
			}
			break;
		}
		case VFDGETTIME:
		{
			res = mcomGetTime();
			copy_to_user((void *)arg, &ioctl_data, 5);
			break;
		}
		case VFDGETWAKEUPMODE:
		{
			res = mcomGetWakeUpMode();
			copy_to_user((void *)arg, &ioctl_data, 1);
			break;
		}
		case VFDDISPLAYCHARS:
		{
			struct vfd_ioctl_data *data = (struct vfd_ioctl_data *)arg;
			res = mcom_WriteString(data->data, data->length);
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			dprintk(1, "VFDDISPLAYWRITEONOFF ->not yet implemented\n");
			break;
		}
		case VFDICONDISPLAYONOFF:
		{
			dprintk(1, "VFDICONDISPLAYONOFF not supported on this receiver\n");
			break;
		}
		case VFDBRIGHTNESS:
		{
			dprintk(1, "VFDBRIGHTNESS not supported on this receiver\n");
			break;
		}
		default:
		{
			dprintk(1, "VFD/CNMicom: unknown IOCTL 0x%x\n", cmd);
			break;
		}
	}
	up(&write_sem);
	dprintk(150, "%s <\n", __func__);
	return res;
}

struct file_operations vfd_fops =
{
	.owner    = THIS_MODULE,
	.ioctl    = MCOMdev_ioctl,
	.write    = MCOMdev_write,
	.read     = MCOMdev_read,
	.open     = MCOMdev_open,
	.release  = MCOMdev_close
};
// vim:ts=4
