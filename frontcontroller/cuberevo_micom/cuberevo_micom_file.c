/*
 * cuberevo_micom_file.c
 *
 * (c) 2011 konfetti
 * partly copied from user space implementation from M.Majoor
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
******************************************************************************
 *
 * This driver covers the following models:
 * 
 * CubeRevo 200HD: 4 character LED (7seg)
 * CubeRevo 250HD / AB IPbox 91HD / Vizyon revolution 800HD: 4 character LED (7seg)
 * CubeRevo Mini / AB IPbox 900HD / Vizyon revolution 810HD: 14 character dot matrix VFD (14seg)
 * CubeRevo Mini II / AB IPbox 910HD / Vizyon revolution 820HD PVR: 14 character dot matrix VFD (14seg)
 * Early CubeRevo / AB IPbox 9000HD / Vizyon revolution 8000HD PVR: 13 character 14 segment VFD (13grid)
 * Late CubeRevo / AB IPbox 9000HD / Vizyon revolution 8000HD PVR: 12 character dot matrix VFD (12grid)
 * CubeRevo 9500HD: 13 character 14 segment VFD (13grid)??
 *
******************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * ----------------------------------------------------------------------------
 * 20190215 Audioniek       proc_FS added.
 * 20190217 Audioniek       SetWakeUpTime added.
 * 20190219 Audioniek       RTC driver added.
 * 20190222 Audioniek       Fix build problems with CubeRevo, 200HD, 9500HD.
 * 20190225 Audioniek       /dev/vfd now scrolls text once when longer than
 *                          display width. Centered or left aligned display
 *                          selectable through #define CENTERED_DISPLAY.
 * 20190227 Audioniek       VFDTEST added.
 * 20190304 Audioniek       UTF8 support on Mini/Mini II/2000HD/3000HD added.
 * 20190306 Audioniek       Fix scrolling problem.
 * 20190308 Audioniek       Fix scrolling problem.
 * 20200524 Audioniek       UTF8 support on 13grid added.
 * 20200524 Audioniek       Full ASCII display including lower case on
 *                          13grid added.
 * 20200620 Audioniek       Do not handle icons on 13grid
 */

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

#include "cuberevo_micom.h"
#include "cuberevo_micom_asc.h"
#if defined(CUBEREVO) \
 || defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD)
#include "cuberevo_micom_utf.h"
#endif

extern const char *driver_version;
extern void ack_sem_up(void);
extern int  ack_sem_down(void);
int micomWriteString(unsigned char *aBuf, int len);
extern void micom_putc(unsigned char data);

struct semaphore write_sem;

int errorOccured = 0;
//int scrolling = 0;  // flag: /dev/vfd is in scroll display
static char ioctl_data[20];

tFrontPanelOpen FrontPanelOpen [LASTMINOR];

struct saved_data_s
{
	int   length;
	char  data[128];
};

int currentDisplayTime = 0;  //display text not time

#if defined(CUBEREVO)
/* animation timer for Play Symbol on 9000HD */
static struct timer_list playTimer;
#endif

/* version date of fp */
int micom_major, micom_minor, micom_year;

/* commands to the fp */
#define VFD_GETA0                0xA0
#define VFD_GETWAKEUPSTATUS      0xA1
#define VFD_GETA2                0xA2
#define VFD_GETRAM               0xA3
#define VFD_GETDATETIME          0xA4
#define VFD_GETMICOM             0xA5
#define VFD_GETWAKEUP            0xA6  /* wakeup time */
#define VFD_SETWAKEUPDATE        0xC0
#define VFD_SETWAKEUPTIME        0xC1
#define VFD_SETDATETIME          0xC2
#define VFD_SETBRIGHTNESS        0xC3
#define VFD_SETVFDTIME           0xC4
#define VFD_SETLED               0xC5
#define VFD_SETLEDSLOW           0xC6
#define VFD_SETLEDFAST           0xC7
#define VFD_SETSHUTDOWN          0xC8
#define VFD_SETRAM               0xC9
#define VFD_SETTIME              0xCA
#define VFD_SETCB                0xCB
#define VFD_SETFANON             0xCC
#define VFD_SETFANOFF            0xCD
#define VFD_SETRFMODULATORON     0xCE
#define VFD_SETRFMODULATOROFF    0xCF
#define VFD_SETCHAR              0xD0
#define VFD_SETDISPLAYTEXT       0xD1
#define VFD_SETD2                0xD2
#define VFD_SETD3                0xD3
#define VFD_SETD4                0xD4
#define VFD_SETCLEARTEXT         0xD5
#define VFD_SETD6                0xD6
#define VFD_SETMODETIME          0xD7
#define VFD_SETSEGMENTI          0xD8
#define VFD_SETSEGMENTII         0xD9
#define VFD_SETCLEARSEGMENTS     0xDA

typedef struct _special_char
{
	unsigned char  ch;
	unsigned short value;
} special_char_t;

static struct saved_data_s lastdata;

/* number of display characters */
int front_seg_num = 13;  // default to CubeRevo 13 character VFD

unsigned short *num2seg;
unsigned short *Char2seg;
unsigned short *LowerChar2seg;
static special_char_t *special2seg;
static int special2seg_size;

/***************************************************************************
 *
 * Character definitions.
 *
 ***************************************************************************/

/*******************************************
 *
 * 13 grid (Early CubeRevo and all 9500HD,
 *          also default)
 *
 * The 13 position VFD has 13 identical
 * 14 segment characters with each a 
 * decimal point.
 * There are no colons or icons.
 *
 * Character layout:
 *
 *    aaaaaaaaa
 *   fj   i   kb
 *   f j  i  k b
 *   f  j i k  b
 *   f   jik   b
 *    gggg hhhh
 *   e   mln   c
 *   e  m l n  c
 *   e m  l  n c
 *   em   l   nc
 *    ddddddddd  p
 *
 * bit = 1 -> segment on
 * bit = 0 -> segment off
 *
 * segment a is lo byte, bit 0 (  1, +0x0001)
 * segment b is lo byte, bit 1 (  2, +0x0002)
 * segment k is lo byte, bit 2 (  4, +0x0004)
 * segment i is lo byte, bit 3 (  8, +0x0008)
 * segment j is lo byte, bit 4 ( 16, +0x0010)
 * segment f is lo byte, bit 5 ( 32, +0x0020)
 * segment g is lo byte, bit 6 ( 64, +0x0040)
 * segment h is lo byte, bit 7 (128, +0x0080)
 *
 * segment c is hi byte, bit 0 (  1, +0x0100)
 * segment n is hi byte, bit 1 (  2, +0x0200)
 * segment l is hi byte, bit 2 (  4, +0x0400)
 * segment m is hi byte, bit 3 (  8, +0x0800)
 * segment e is hi byte, bit 4 ( 16, +0x1000)
 * segment d is hi byte, bit 5 ( 32, +0x2000)
 * segment p is hi byte, bit 6 ( 64, +0x4000, decimal point)
 * segment g is hi byte, bit 7 (128, +0x8000, unused)
 *
 * Character tables are 16 bit words,
 * representing segment patterns.
 */
unsigned short num2seg_13grid[] =
{
	0x3123,  // 0
	0x0408,  // 1
	0x30c3,  // 2
	0x21c3,  // 3
	0x01e2,  // 4
	0x21e1,  // 5
	0x31e1,  // 6
	0x0123,  // 7
	0x31e3,  // 8
	0x21e3,  // 9
};

unsigned short Char2seg_13grid[] =
{
	0x11e3,  // A
	0x25cb,  // B
	0x3021,  // C
	0x250b,  // D
	0x30e1,  // E
	0x10e1,  // F
	0x31a1,  // G
	0x11e2,  // H
	0x2409,  // I
	0x3002,  // J
	0x1264,  // K
	0x3020,  // L
	0x11c0,  // M
	0x13c0,  // N
	0x3123,  // O
	0x10e3,  // P
	0x3323,  // Q
	0x12e3,  // R
	0x21e1,  // S
	0x0409,  // T
	0x3122,  // U
	0x1824,  // V
	0x1b22,  // W
	0x0a14,  // X
	0x04e2,  // Y
	0x2805,  // Z
};

unsigned short LowerChar2seg_13grid[] =
{
	0x31c3,  // a
	0x31e0,  // b
	0x30c0,  // c
	0x31c2,  // d
	0x30e3,  // e
	0x10e1,  // f
	0x21e3,  // g
	0x11e0,  // h
	0x0400,  // i
	0x3102,  // j
	0x12e0,  // k
	0x0408,  // l
	0x15c0,  // m
	0x11c0,  // n
	0x31c0,  // o
	0x10e3,  // p
	0x01e3,  // q
	0x10c0,  // r
	0x21e1,  // s
	0x3060,  // t
	0x3100,  // u
	0x1800,  // v
	0x3500,  // w
	0x0a14,  // x
	0x0814,  // y
	0x2805,  // z
};

special_char_t special2seg_13grid[] =
{
	{ '!',  0x4408 },  // 0x21 !
	{ 0x22, 0x0022 },  // 0x22 "
	{ '#',  0x04c8 },  // 0x23 #
	{ '$',  0x25e9 },  // 0x24 $
	{ '%',  0x0924 },  // 0x25 %
	{ '&',  0x00c0 },  // 0x26 &
	{ 0x27, 0x0020 },  // 0x27 ' 
	{ '(',  0x0204 },  // 0x28 (
	{ ')',  0x0810 },  // 0x29 )
	{ '*',  0x0edc },  // 0x2a *
	{ '+',  0x04c8 },  // 0x2b +
	{ ',',  0x0800 },  // 0x2c ,
	{ '-',  0x00c0 },  // 0x2d -
	{ '.',  0x4000 },  // 0x2e .
	{ '/',  0x0800 },  // 0x2f /
	{ ':',  0x0408 },  // 0x3a :
	{ ';',  0x0810 },  // 0x3b ;
	{ '<',  0x0204 },  // 0x3c <
	{ '=',  0x20c0 },  // 0x3d =
	{ '>',  0x0810 },  // 0x3e >
	{ '?',  0x0425 },  // 0x3f ?
	{ '@',  0x30e3 },  // 0x40 @ 
	{ '[',  0x3021 },  // 0x5b [
	{ '\'', 0x0210 },  // 0x5c backslash
	{ ']',  0x2103 },  // 0x5d ]
	{ '^',  0x0023 },  // 0x5e ^
	{ '_',  0x2000 },  // 0x5f _
	{ '`',  0x0010 },  // 0x60 `
	{ '{',  0x0244 },  // 0x7b {
	{ '|',  0x0408 },  // 0x7c |
	{ '}',  0x0890 },  // 0x7d }
	{ '~',  0x40c0 },  // 0x7e ~
	{ 0x7f, 0x3fff },  // 0x7f DEL
	{ ' ',  0x0000 },  // space (EOT)
};

#if defined(CUBEREVO)
// 12 character dot matrix (Late CubeRevo)
// NOTE: tables are ASCII value minus 0x10 and can be calculated
unsigned short num2seg_12dotmatrix[] =
{
	0x20,	// 0
	0x21,	// 1
	0x22,	// 2
	0x23,	// 3
	0x24,	// 4
	0x25,	// 5
	0x26,	// 6
	0x27,	// 7
	0x28,	// 8
	0x29,	// 9
};

unsigned short Char2seg_12dotmatrix[] =
{
	0x31,	// A
	0x32,	// B
	0x33,	// C
	0x34,	// D
	0x35,	// E
	0x36,	// F
	0x37,	// G
	0x38,	// H
	0x39,	// I
	0x3a,	// J
	0x3b,	// K
	0x3c,	// L
	0x3d,	// M
	0x3e,	// N
	0x3f,	// O
	0x40,	// P
	0x41,	// Q
	0x42,	// R
	0x43,	// S
	0x44,	// T
	0x45,	// U
	0x46,	// V
	0x47,	// W
	0x48,	// X
	0x49,	// Y
	0x4a,	// Z
};

special_char_t special2seg_12dotmatrix[] =
{  // table is largely ASCII minus 0x10
	{ '-',   0x1d },
	{ '\'',  0x90 },
	{ '.',   0x1e },
	{ ' ',   0x10 },
};
#endif

/*******************************************
 *
 * 14 grid (CubeRevo mini, mini II, 2000HD
 *          and 3000HD)
 * digits, upper case letters, lower case
 * letters
 *
 */
#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD)
// 14 character dot matrix (900HD / 910HD / 2000HD / 3000HD)
unsigned short num2seg_14dotmatrix[] =
{  // note: 0x10 lower than ASCII value
	0x20,	// 0
	0x21,	// 1
	0x22,	// 2
	0x23,	// 3
	0x24,	// 4
	0x25,	// 5
	0x26,	// 6
	0x27,	// 7
	0x28,	// 8
	0x29,	// 9
};
#endif
unsigned short Char2seg_14dotmatrix[] =
{  // note: 0x10 lower than ASCII value
	0x31,	// A
	0x32,	// B
	0x33,	// C
	0x34,	// D
	0x35,	// E
	0x36,	// F
	0x37,	// G
	0x38,	// H
	0x39,	// I
	0x3a,	// J
	0x3b,	// K
	0x3c,	// L
	0x3d,	// M
	0x3e,	// N
	0x3f,	// O
	0x40,	// P
	0x41,	// Q
	0x42,	// R
	0x43,	// S
	0x44,	// T
	0x45,	// U
	0x46,	// V
	0x47,	// W
	0x48,	// X
	0x49,	// Y
	0x4a,	// Z
};

unsigned short LowerChar2seg_14dotmatrix[] =
{  // note: 0x10 lower than ASCII value
	0x51,	// a
	0x52,	// b
	0x53,	// c
	0x54,	// d
	0x55,	// e
	0x56,	// f
	0x57,	// g
	0x58,	// h
	0x59,	// i
	0x5a,	// j
	0x5b,	// k
	0x5c,	// l
	0x5d,	// m
	0x5e,	// n
	0x5f,	// o
	0x60,	// p
	0x61,	// q
	0x62,	// r
	0x63,	// s
	0x64,	// t
	0x65,	// u
	0x66,	// v
	0x67,	// w
	0x68,	// x
	0x69,	// y
	0x6a,	// z
};

/*******************************************
 *
 * 14 grid (CubeRevo mini, mini II, 2000HD
 *          and 3000HD)
 * digits, upper case letters, lower case
 * letters
 *
 */
#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD) \
 || defined(CUBEREVO)
// non-alphanumeric
special_char_t special2seg_14dotmatrix[] =
{  // table is largely ASCII minus 0x10
	{ ' ',	0x10 },  // ->> ASCII - 0x10
	{ '!',	0x11 },
	{ '"',	0x12 },
	{ '#',	0x13 },
	{ '$',	0x14 },
	{ '%',	0x15 },
	{ '&',	0x16 },
	{ '#',	0x17 },
	{ '(',	0x18 },
	{ ')',	0x19 },
	{ '*',	0x1a },
	{ '+',	0x1b },
	{ ',',	0x1c },
	{ '-',	0x1d },
	{ '.',	0x1e },
	{ '/',	0x1f },
//	{ '0',	0x20 },
//	{ '1',	0x21 },
//	{ '2',	0x22 },
//	{ '3',	0x23 },
//	{ '4',	0x24 },
//	{ '5',	0x25 },
//	{ '6',	0x26 },
//	{ '7',	0x27 },
//	{ '8',	0x28 },
//	{ '9',	0x29 },
//	{ ':',	0x2a },
//	{ ';',	0x2b },
	{ '<',	0x2c },
	{ '=',	0x2d },
	{ '>',	0x2e },
	{ '?',	0x2f },
	{ '@',	0x30 },
//	{ 'A',	0x31 }
//	     |       
//	{ 'Z',	0x4a },
	{ '[',	0x4b },
//	{ '?',	0x4c },  // yen sign -> c2 a5
	{ '^',	0x4d },
	{ ']',	0x4e },
	{ '_',	0x4f },
//	{ '`',	0x50 },  // back quote
//	{ 'a',	0x51 },
//	     |
//	{ 'z',	0x6a },
	{ '{',	0x6b },
	{ '|',	0x6c },
	{ '}',	0x6d },
	{ '~',	0x6e },
	{ 0x7f,	0x6f },  //DEL, full block  // end of ASCII - 0x10
//	{ '?',  0x70 },  // large alpha
//       |
//	{ '?',	0x7e },  // large omega
//	{ '?',	0x7f },  // large epsilon
//	{ '?',	0x80 },  // pound sign -> c2 a3
//	{ '?',	0x81 },  // paragraph -> c2 a7
//	{ '?',	0x82 },  // large IE diacritic
//	{ '?',	0x83 },  // large IR diacritic
//	{ '?',	0x84 },  // integral sign
//	{ '?',	0x85 },  // invert x
//	{ '?',	0x86 },  // A accent dot
//	{ '?',	0x87 },  // power of -1
//	{ '?',	0x88 },  // power of 2 -> c2 b2
//	{ '?',	0x89 },  // power of 3 -> c2 b3
//	{ '?',	0x8a },  // power of x
//	{ '?',	0x8b },  // 1/2 -> c2 bd
//	{ '?',	0x8c },  // 1/ 
//	{ '?',	0x8d },  // square root
//	{ '?',	0x8e },  // +/- -> c2 b1
//	{ '?',	0x8f },  // paragraph
	{ '\'',	0x90 },
//	{ '?',	0x91 },  // katakana
//	     |
//	{ '?',	0xce },  // katakana
//	{ '?',	0xcf },  // degree sign -> c2 b0
//	{ '?',	0xd0 },  // arrow up
//	{ '?',	0xd1 },  // arrow down
//	{ '?',	0xd2 },  // arrow left
//	{ '?',	0xd3 },  // arrow right
//	{ '?',	0xd4 },  // arrow top left
//	{ '?',	0xd5 },  // arrow top right
//	{ '?',	0xd6 },  // arrow bottom right
//	{ '?',	0xd7 },  // arrow bottom left
//	{ '?',	0xd8 },  // left end measurement
//	{ '?',	0xd9 },  // right end measurement
//	{ '?',	0xda },  // superscript mu
//	{ '?',	0xdb },  // inverted superscript mu
//	{ '?',	0xdc },  // fat <
//	{ '?',	0xdd },  // fat >
//	{ '?',	0xde },  // three dots up
//	{ '?',	0xdf },  // three dots down
//	{ '?',	0xe0 },  // smaller or equal than
//	{ '?',	0xe1 },  // greater or equal than
//	{ '?',	0xe2 },  // unequal sign ->
//	{ '?',	0xe3 },  // equal sign with dots
//	{ '?',	0xe4 },  // two vertical bars
//	{ '?',	0xe5 },  // single vertical bar
//	{ '?',	0xe6 },  // inverted T
//	{ '?',	0xe7 },  // infinite sign
//	{ '?',	0xe8 },  // infinite sign open
//	{ '?',	0xe9 },  // inverted tilde
//	{ '?',	0xea },  // AC symbol
//	{ '?',	0xeb },  // three horizontal lines
//	{ '?',	0xec },  // Ground symbol inverted
//	{ '?',	0xed },  // buzzer
//	{ '?',	0xee },  // collapsed 8
//	{ '?',	0xef },  // small 1
//	{ '?',	0xf0 },  // small 2
//	{ '?',	0xf1 },  // small 3
//	{ '?',	0xf2 },  // small 4
//	{ '?',	0xf3 },  // small 5
//	{ '?',	0xf4 },  // small 6
//	{ '?',	0xf5 },  // small 7
//	{ '?',	0xf6 },  // small 8
//	{ '?',	0xf7 },  // small 9
//	{ '?',	0xf8 },  // small 10
//	{ '?',	0xf9 },  // small 11
//	{ '?',	0xfa },  // small 12
//	{ '?',	0xfb },  // small 13
//	{ '?',	0xfc },  // small 14
//	{ '?',	0xfd },  // small 15
//	{ '?',	0xfe },  // small 16
//	{ '?',	0xff },  // space
};
#endif

#if defined(CUBEREVO_250HD) \
 || defined(CUBEREVO_MINI_FTA)
/*******************************************
 *
 * 7 segment LED (CubeRevo 250HD, miniFTA)
 * digits, letters, special characters
 *
 */
// 7 segment LED display (200HD/250HD)
static unsigned short Char2seg_7seg[] =
{  // letters A - Z
	~0x01 & ~0x02 & ~0x04 & ~0x10 & ~0x20 & ~0x40,
	~0x04 & ~0x08 & ~0x10 & ~0x20 & ~0x40,
	~0x01 & ~0x08 & ~0x10 & ~0x20,
	~0x02 & ~0x04 & ~0x08 & ~0x10 & ~0x40,
	~0x01 & ~0x08 & ~0x10 & ~0x20 & ~0x40,
	~0x01 & ~0x10 & ~0x20 & ~0x40,
	~0x01 & ~0x04 & ~0x08 & ~0x10 & ~0x20,
	~0x04 & ~0x10 & ~0x20 & ~0x40,
	~0x04,
	~0x02 & ~0x04 & ~0x08 & ~0x10,
	~0x01 & ~0x04 & ~0x10 & ~0x20 & ~0x40,
	~0x08 & ~0x10 & ~0x20,
	~0x01 & ~0x02 & ~0x04 & ~0x10 & ~0x20,
	~0x04 & ~0x10 & ~0x40,
	~0x04 & ~0x08 & ~0x10 & ~0x40,
	~0x01 & ~0x02 & ~0x10 & ~0x20 & ~0x40,
	~0x01 & ~0x02 & ~0x08 & ~0x10 & ~0x20 & ~0x40,
	~0x10 & ~0x40,
	~0x04 & ~0x08 & ~0x20 & ~0x40,
	~0x08 & ~0x10 & ~0x20 & ~0x40,
	~0x04 & ~0x08 & ~0x10,
	~0x02 & ~0x04 & ~0x08 & ~0x10 & ~0x20,
	~0x02 & ~0x04 & ~0x08 & ~0x10 & ~0x20 & ~0x40,
	~0x02 & ~0x04 & ~0x10 & ~0x20 & ~0x40,
	~0x02 & ~0x04 & ~0x08 & ~0x20 & ~0x40,
	~0x01 & ~0x02 & ~0x08 & ~0x10,
};

static unsigned short num2seg_7seg[] =
{  // digits 0-9
	0xc0,	// 0
	0xf9,	// 1
	0xa4,	// 2
	0xb0,	// 3
	0x99,	// 4
	0x92,	// 5
	0x82,	// 6
	0xd8,	// 7
	0x80,	// 8
	0x98,	// 9
};

special_char_t special2seg_7seg[] =
{
	{ '-',	0xbf },
	{ '_',	0xf7 },
	{ '.',	0x7f },
	{ ' ',	0xff },
};
#endif

/***************************************************************************
 *
 * Icon definitions.
 *
 ***************************************************************************/
#if defined(CUBEREVO)
enum
{
	ICON_MIN = 0,  // 0
	ICON_STANDBY,
	ICON_SAT,
	ICON_REC,
	ICON_TIMESHIFT,
	ICON_TIMER,  // 5
	ICON_HD,
	ICON_USB,
	ICON_SCRAMBLED,
	ICON_DOLBY,
	ICON_MUTE,  // 10
	ICON_TUNER1,
	ICON_TUNER2,
	ICON_MP3,
	ICON_REPEAT,
	ICON_PLAY,  // 15
	ICON_TER,
	ICON_FILE,
	ICON_480i,
	ICON_480p,
	ICON_576i,  // 20
	ICON_576p,
	ICON_720p,
	ICON_1080i,
	ICON_1080p,
	ICON_PLAY_1,  // 25
	ICON_RADIO,
	ICON_TV,
	ICON_PAUSE,
	ICON_MAX  // 29
};
#elif defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD)
enum
{
	ICON_MIN,  // 0
	ICON_REC,
	ICON_TIMER,  // 2
	ICON_TIMESHIFT,
	ICON_PLAY,  // 4
	ICON_PAUSE,
	ICON_HD,  // 6
	ICON_DOLBY,
	ICON_MAX  // 8
};
#endif

struct iconToInternal
{
	char *name;
	u16 icon;
	u8 codemsb;
	u8 codelsb;
	u8 segment;
};

#if defined(CUBEREVO)
// 12 segment icons
struct iconToInternal micomIcons[] =
{
	/*----------------- SetIcon -------  msb   lsb   segment -----*/
	{ "ICON_STANDBY",   ICON_STANDBY,    0x03, 0x00, 1 },
	{ "ICON_SAT",       ICON_SAT,        0x02, 0x00, 1 },
	{ "ICON_REC",       ICON_REC,        0x00, 0x00, 0 },
	{ "ICON_TIMESHIFT", ICON_TIMESHIFT,  0x01, 0x01, 0 },
	{ "ICON_TIMESHIFT", ICON_TIMESHIFT,  0x01, 0x02, 0 },
	{ "ICON_TIMER",     ICON_TIMER,      0x01, 0x03, 0 },
	{ "ICON_HD",        ICON_HD,         0x01, 0x04, 0 },
	{ "ICON_USB",       ICON_USB,        0x01, 0x05, 0 },
	{ "ICON_SCRAMBLED", ICON_SCRAMBLED,  0x01, 0x06, 0 },  //locked not scrambled
	{ "ICON_DOLBY",     ICON_DOLBY,      0x01, 0x07, 0 },
	{ "ICON_MUTE",      ICON_MUTE,       0x01, 0x08, 0 },
	{ "ICON_TUNER1",    ICON_TUNER1,     0x01, 0x09, 0 },
	{ "ICON_TUNER2",    ICON_TUNER2,     0x01, 0x0a, 0 },
	{ "ICON_MP3",       ICON_MP3,        0x01, 0x0b, 0 },
	{ "ICON_REPEAT",    ICON_REPEAT,     0x01, 0x0c, 0 },
	{ "ICON_Play",      ICON_PLAY,       0x00, 0x00, 1 },
	{ "ICON_Play_1",    ICON_PLAY_1,     0x01, 0x04, 1 },
	{ "ICON_TER",       ICON_TER,        0x02, 0x01, 1 },
	{ "ICON_FILE",      ICON_FILE,       0x02, 0x02, 1 },
	{ "ICON_480i",      ICON_480i,       0x06, 0x04, 1 },
	{ "ICON_480i",      ICON_480i,       0x06, 0x03, 1 },
	{ "ICON_480p",      ICON_480p,       0x06, 0x04, 1 },
	{ "ICON_480p",      ICON_480p,       0x06, 0x02, 1 },
	{ "ICON_576i",      ICON_576i,       0x06, 0x01, 1 },
	{ "ICON_576i",      ICON_576i,       0x06, 0x00, 1 },
	{ "ICON_576p",      ICON_576p,       0x06, 0x01, 1 },
	{ "ICON_576p",      ICON_576p,       0x05, 0x04, 1 },
	{ "ICON_720p",      ICON_720p,       0x05, 0x03, 1 },
	{ "ICON_1080i",     ICON_1080i,      0x05, 0x02, 1 },
	{ "ICON_1080p",     ICON_1080p,      0x05, 0x01, 1 },
	{ "ICON_RADIO",     ICON_RADIO,      0x02, 0x04, 1 },
	{ "ICON_TV",        ICON_TV,         0x02, 0x03, 1 }
};
#elif defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD)
// 14 segment icons
struct iconToInternal micom_14seg_Icons[] =
{
	/*------------------ SetIcon -------  msb   lsb   segment -----*/
	{ "ICON_TIMER",      ICON_TIMER,      0x03, 0x00, 1 },
	{ "ICON_REC",        ICON_REC,        0x02, 0x00, 1 },
	{ "ICON_HD",         ICON_HD,         0x02, 0x04, 1 },
	{ "ICON_Play",       ICON_PLAY,       0x02, 0x01, 1 },
	{ "ICON_PAUSE",      ICON_PAUSE,      0x02, 0x02, 1 },
	{ "ICON_DOLBY",      ICON_DOLBY,      0x02, 0x03, 1 },
	{ "ICON_TIMESHIFT",  ICON_TIMESHIFT,  0x03, 0x01, 1 },
};
#endif
/* End of character and icon definitions */

/***************************************************************************************
 *
 * Code for play display on Late IPbox9000HD / CubeRevo.
 *
 */
#if defined(CUBEREVO)
int micomWriteCommand(char *buffer, int len, int needAck);

#define cNumberSymbols      8
#define ANIMATION_INTERVAL  msecs_to_jiffies(500)

static int current_symbol = 0;
static int animationDie = 0;

struct iconToInternal playIcons[cNumberSymbols] =
{
	{ "ICON_Play",      ICON_PLAY,       0x00, 0x00, 1 },
	{ "ICON_Play",      ICON_PLAY,       0x00, 0x02, 1 },
	{ "ICON_Play",      ICON_PLAY,       0x01, 0x01, 1 },
	{ "ICON_Play",      ICON_PLAY,       0x01, 0x03, 1 },
	{ "ICON_Play",      ICON_PLAY,       0x01, 0x04, 1 },
	{ "ICON_Play",      ICON_PLAY,       0x01, 0x02, 1 },
	{ "ICON_Play",      ICON_PLAY,       0x00, 0x03, 1 },
	{ "ICON_Play",      ICON_PLAY,       0x00, 0x01, 1 },
};

static void animated_play(unsigned long data)
{
	char buffer[5];
	int i;

	current_symbol = (current_symbol + 1) % cNumberSymbols;

	for (i = 0; i < cNumberSymbols; i++)
	{
		memset(buffer, 0, sizeof(buffer));
		if ((i == current_symbol) && (animationDie == 0))
		{
			buffer[0] = VFD_SETSEGMENTI + playIcons[i].segment;
			buffer[1] = 0x01;
			buffer[2] = playIcons[i].codelsb;
			buffer[3] = playIcons[i].codemsb;
			micomWriteCommand(buffer, 5, 0);
		}
		else
		{
			buffer[0] = VFD_SETSEGMENTI + playIcons[i].segment;
			buffer[1] = 0x00;
			buffer[2] = playIcons[i].codelsb;
			buffer[3] = playIcons[i].codemsb;
			micomWriteCommand(buffer, 5, 0);
		}
	}
	if (animationDie == 0)
	{
		/* reschedule the timer */
		playTimer.expires = jiffies + ANIMATION_INTERVAL;
		add_timer(&playTimer);
	}
}
#endif

/*******************************************************
 *
 * Code to communicate with the front processor.
 *
 */
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
	dprintk(150, "%s len %d\n", __func__, len);
	memcpy(ioctl_data, data, len);
}

int micomWriteCommand(char *buffer, int len, int needAck)
{
	int i;

	dprintk(150, "%s >\n", __func__);

	for (i = 0; i < len; i++)
	{
#ifdef DIRECT_ASC
		serial_putc(buffer[i]);
#else
		micom_putc(buffer[i]);
#endif
	}
	if (needAck)
	{
		if (ack_sem_down())
		{
			return -ERESTARTSYS;
		}
	}
	dprintk(150, "%s < %d\n", __func__, 0);
	return 0;
}

/*******************************************************************
 *
 * Code for the functions of the driver.
 *
 */

/*******************************************************************
 *
 * micomSetLED: sets the state of the LED(s) on front panel display.
 *
 * Parameter on is a bit mask:
 * bit 0 = LED on (1) or off (0)
 * bit 1 = slow blink on/off (1 = blink, 0 = steady)
 * bit 2 = fast blink on/off (1 = blink, 0 = steady)
 *
 * Practical values are:
 * 0: LED off
 * 1: LED on;
 * 3: LED blinks slowly
 * 5: LED blinks fast
 */
int micomSetLED(int on)
{
	unsigned char buffer[5];
	int res = 0;

	dprintk(100, "%s > %d\n", __func__, on);

	memset(buffer, 0, sizeof(buffer));

	// even = off, 1 = on, 3 = slow blink, 5 = fast blink, 7 = slow bink
	if (on & 0x1)
	{
		buffer[0] = VFD_SETLED;
		buffer[1] = 0x01;

		res = micomWriteCommand(buffer, 5, 0);

		if (on & 0x2)
		{
			buffer[0] = VFD_SETLEDSLOW;
			buffer[1] = 0x81;  /* continues blink mode, last state will be on */
			res = micomWriteCommand(buffer, 5, 0);
		}
		else if (on & 0x4)
		{
			buffer[0] = VFD_SETLEDFAST;
			buffer[1] = 0x81;  /* continues blink mode, last state will be on */
			res = micomWriteCommand(buffer, 5, 0);
		}
		else
		{
			buffer[0] = VFD_SETLEDFAST;
			buffer[1] = 0x01;  /* stops blink mode, last state will be on */
			res = micomWriteCommand(buffer, 5, 0);
		}
	}
	else  // off
	{
		buffer[0] = VFD_SETLED;
		buffer[1] = 0x00;

		res = micomWriteCommand(buffer, 5, 0);

		buffer[0] = VFD_SETLEDFAST;
		buffer[1] = 0x00;

		res = micomWriteCommand(buffer, 5, 0);

		buffer[0] = VFD_SETLEDSLOW;
		buffer[1] = 0x00;

		res = micomWriteCommand(buffer, 5, 0);
	}
	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}
/* export for later use in e2_proc */
EXPORT_SYMBOL(micomSetLED);

/*******************************************************************
 *
 * micomSetFan: switches fan on or off.
 *
 */
#if defined(CUBEREVO)
int micomSetFan(int on)
{
	unsigned char buffer[5];
	int res = 0;

	dprintk(100, "%s > %d\n", __func__, on);

	memset(buffer, 0, sizeof(buffer));

	/* on:
	 * 0 = off
	 * 1 = on
	 */
	if (on == 1)
	{
		buffer[0] = VFD_SETFANON;
	}
	else
	{
		buffer[0] = VFD_SETFANOFF;
	}
	res = micomWriteCommand(buffer, 5, 0);

	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}
#else
int micomSetFan(int on)
{
	dprintk(1, "%s Only supported on CubeRevo, other models have their fan always on.\n", __func__);
}
#endif

/*******************************************************************
 *
 * micomSetTimeMode: sets 24h format on or off.
 *
 */
int micomSetTimeMode(int twentyFour)
{
	unsigned char buffer[5];
	int res = 0;

	dprintk(100, "%s > %d\n", __func__, twentyFour);

	/* clear display */
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = VFD_SETCLEARTEXT;
	res = micomWriteCommand(buffer, 5, 0);
	buffer[0] = VFD_SETDISPLAYTEXT;
	res = micomWriteCommand(buffer, 5, 0);

	memset(buffer, 0, sizeof(buffer));
	buffer[0] = VFD_SETMODETIME;
	buffer[1] = twentyFour & 0x1;
	res = micomWriteCommand(buffer, 5, 0);

	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

/*******************************************************************
 *
 * micomSetDisplayTime: sets constant display of time on or off.
 *
 * Note: does not restore previous display if turned off.
 *
 */
int micomSetDisplayTime(int on)
{
	unsigned char buffer[5];
	int res = 0;

	dprintk(100, "%s > %d\n", __func__, on);

	/* clear display */
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = VFD_SETCLEARTEXT;
	res = micomWriteCommand(buffer, 5, 0);
	buffer[0] = VFD_SETDISPLAYTEXT;
	res = micomWriteCommand(buffer, 5, 0);

	/* show time */
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = VFD_SETVFDTIME;
	buffer[1] = on;
	res = micomWriteCommand(buffer, 5, 0);

	currentDisplayTime = on;
	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

/*******************************************************************
 *
 * micomSetRF: switches RF modulator on or off.
 *
 */
int micomSetRF(int on)
{
	unsigned char buffer[5];
	int res = 0;

	dprintk(100, "%s > %d\n", __func__, on);

	memset(buffer, 0, sizeof(buffer));
	/* on:
	 * 0 = off
	 * 1 = on
	 */
	if (on == 1)
	{
		buffer[0] = VFD_SETRFMODULATORON;
	}
	else
	{
		buffer[0] = VFD_SETRFMODULATOROFF;
	}
	res = micomWriteCommand(buffer, 5, 0);

	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

/****************************************************************
 *
 * micomSetBrightness: sets brightness of front panel display.
 *
 */
int micomSetBrightness(int level)
{
	unsigned char buffer[5];
	int res = 0;

	dprintk(100, "%s > %d\n", __func__, level);

	if (level < 0 || level > 7)
	{
		printk("[micom] brightness out of range %d\n", level);
		return -EINVAL;
	}
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = VFD_SETBRIGHTNESS;
	buffer[1] = level & 0x07;
	res = micomWriteCommand(buffer, 5, 0);

	dprintk(100, "%s <%d\n", __func__, res);
	return res;
}
/* export for later use in e2_proc */
EXPORT_SYMBOL(micomSetBrightness);

/*******************************************************
 *
 * micomSetIcon: (re)sets an icon on the front panel
 *               display.
 *
 * Note on icon TIMER: Front panel sets this
 * automatically after power on or reboot when a wake
 * up time other than 00:00:00 00-00-80 is found.
 * Initialization of this driver switches all icons
 * off. You may occasionally see the iconTIMER on
 * during startup as a consequence.
 *
 */
int micomSetIcon(int which, int on)
{
	unsigned char buffer[5];
	int vLoop, res = 0;

	if (front_seg_num == 13 || front_seg_num == 4)  // no icons
	{
		return res;
	}
	dprintk(100, "%s > %d, %d\n", __func__, which, on);
#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD) \
 || defined(CUBEREVO)
	if (which < 1 || which >= ICON_MAX)
	{
		printk("[micom] Icon number %d out of range (1-%d)\n", which, ICON_MAX - 1);
		return -EINVAL;
	}
	memset(buffer, 0, sizeof(buffer));
#endif
#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD)
	for (vLoop = 0; vLoop < ARRAY_SIZE(micom_14seg_Icons); vLoop++)
	{
		if ((which & 0xff) == micom_14seg_Icons[vLoop].icon)
		{
			buffer[0] = VFD_SETSEGMENTI + micom_14seg_Icons[vLoop].segment;
			buffer[1] = on;
			buffer[2] = micom_14seg_Icons[vLoop].codelsb;
			buffer[3] = micom_14seg_Icons[vLoop].codemsb;
			res = micomWriteCommand(buffer, 5, 0);
			break;
		}
	}
#elif defined(CUBEREVO)  // Late IPbox 9000HD
	if (front_seg_num == 12)
	{
		if (which == ICON_PLAY)  // handle play icon
		{
			if (on)
			{
				/* display circle */
				buffer[0] = VFD_SETSEGMENTII;
				buffer[1] = 0x01;
				buffer[2] = 0x04;
				buffer[3] = 0x00;
				micomWriteCommand(buffer, 5, 0);
	
				/* display play symbol */
				buffer[0] = VFD_SETSEGMENTII;
				buffer[1] = 0x01;
				buffer[2] = 0x00;
				buffer[3] = 0x01;
				micomWriteCommand(buffer, 5, 0);
	
				current_symbol = 0;
				animationDie = 0;
				playTimer.expires = jiffies + ANIMATION_INTERVAL;
				add_timer(&playTimer);
			}
			else  // PLAY off
			{
				/* clear circle */
				buffer[0] = VFD_SETSEGMENTII;
				buffer[1] = 0x00;
				buffer[2] = 0x04;
				buffer[3] = 0x00;
				micomWriteCommand(buffer, 5, 0);
	
				/* clear play symbol */
				buffer[0] = VFD_SETSEGMENTII;
				buffer[1] = 0x00;
				buffer[2] = 0x00;
				buffer[3] = 0x01;
				micomWriteCommand(buffer, 5, 0);
		
				animationDie = 1;
			}
		}
		else  // other icon than PLAY
		{
			for (vLoop = 0; vLoop < ARRAY_SIZE(micomIcons); vLoop++)
			{
				if ((which & 0xff) == micomIcons[vLoop].icon)
				{
					buffer[0] = VFD_SETSEGMENTI + micomIcons[vLoop].segment;
					buffer[1] = on;
					buffer[2] = micomIcons[vLoop].codelsb;
					buffer[3] = micomIcons[vLoop].codemsb;
					res = micomWriteCommand(buffer, 5, 0);
					/* do not break here because there may be multiple segments */
				}
			}
		}
	}
	else
	{
		dprintk(1, "%s: This model has no icons.\n", __func__);
	}
#elif defined(CUBEREVO_250HD) \
 || defined(CUBEREVO_MINI_FTA)
	dprintk(1, "%s: This model has no icons.\n", __func__);
#endif
	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

/*****************************************************
 *
 * micomClearIcons: clears all icons in the display.
 *
 */
int micomClearIcons(void)
{
	unsigned char buffer[5];
	int  res = 0;

	dprintk(100, "%s >\n", __func__);

	memset(buffer, 0, sizeof(buffer));

	buffer[0] = VFD_SETCLEARSEGMENTS;
	res = micomWriteCommand(buffer, 5, 0);

	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

/****************************************************************
 *
 * micomSetWakeUpTime: sets front panel clock wake up time.
 *
 * Note: the missing seconds could have been used to convey the
 * action (on or off).
 */
int micomSetWakeUpTime(unsigned char *time)  // expected format: YYMMDDhhmm
{
	unsigned char buffer[5];
	int res = -1;

	/* set wakeup time */
	memset(buffer, 0, sizeof(buffer));

	if (time[0] == '\0')
	{
		dprintk(1, "clear wakeup date\n");
		buffer[1] = 0;  // year
		buffer[2] = 0;  // month
		buffer[3] = 0;  // day
	}
	else
	{
		buffer[1] = ((time[0] - '0') << 4) | (time[1] - '0');  // year
		buffer[2] = ((time[2] - '0') << 4) | (time[3] - '0');  // month
		buffer[3] = ((time[4] - '0') << 4) | (time[5] - '0');  // day
	}
	buffer[0] = VFD_SETWAKEUPDATE;
	res = micomWriteCommand(buffer, 5, 0);
	dprintk(10, "set wakeup date to %02x-%02x-20%02x\n", buffer[3], buffer[2], buffer[1]);

	memset(buffer, 0, sizeof(buffer));
	if (time[0] == '\0')
	{
		dprintk(1, "clear wakeup date\n");
		buffer[1] = 0;  // hour
		buffer[2] = 0;  // minute
		buffer[3] = 0;  // switch off
	}
	else
	{
		buffer[1] = ((time[6] - '0') << 4) | (time[7] - '0');  // hour
		buffer[2] = ((time[8] - '0') << 4) | (time[9] - '0');  // minute
		buffer[3] = 1;  // switch on
	}
	buffer[0] = VFD_SETWAKEUPTIME;
	res |= micomWriteCommand(buffer, 5, 0);
	dprintk(10, "set wakeup time to %02x:%02x; action is switch %s\n", buffer[1], buffer[2], ((buffer[3] == 1) ? "on" : "off" ));
	return res;
}

/****************************************************************
 *
 * micomSetStandby: sets wake up time and then switches
 *                  receiver to deep standby.
 *
 */
int micomSetStandby(char *time)  // expected format: YYMMDDhhmm
{
	unsigned char buffer[5];
	int res = -1;

	dprintk(100, "%s >\n", __func__);

//	res = micomWriteString("Bye bye ...", strlen("Bye bye ..."));

	/* set wakeup time */
	res = micomSetWakeUpTime(time);

	/* now power off */
	memset(buffer, 0, sizeof(buffer));

	buffer[0] = VFD_SETSHUTDOWN;
	/* 0 = shutdown
	 * 1 = reboot
	 * 2 = warmoff
	 * 3 = warmon
	 */
	buffer[1] = 0;
	res = micomWriteCommand(buffer, 5, 0);

	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

int micomReboot(void)
{
	unsigned char buffer[5];
	int res = 0;

	dprintk(100, "%s >\n", __func__);

//	res = micomWriteString("Reboot...", strlen("Reboot..."));

	memset(buffer, 0, sizeof(buffer));

	buffer[0] = VFD_SETSHUTDOWN;
	/* 0 = shutdown
	 * 1 = reboot
	 * 2 = warmoff
	 * 3 = warmon
	 */
	buffer[1] = 1;
	res = micomWriteCommand(buffer, 5, 0);

	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

/**************************************************
 *
 * micomSetTime: sets front panel clock time.
 *
 */
int micomSetTime(char *time)  // expected format: YYMMDDhhmmss
{
	unsigned char buffer[5];
	int res = 0;

	dprintk(100, "%s >\n", __func__);

	/* set time */
	memset(buffer, 0, sizeof(buffer));

	dprintk(10, "date to set: %c%c-%c%c-20%c%c\n", time[4], time[5], time[2], time[3], time[0], time[1]);
	dprintk(10, "time to set: %c%c:%c%c:%c%c\n", time[6], time[7], time[8], time[9], time[10], time[11]);

	buffer[0] = VFD_SETDATETIME;
	buffer[1] = ((time[0] - '0') << 4) | (time[1] - '0');  // year
	buffer[2] = ((time[2] - '0') << 4) | (time[3] - '0');  // month
	buffer[3] = ((time[4] - '0') << 4) | (time[5] - '0');  // day
	res = micomWriteCommand(buffer, 5, 0);

	memset(buffer, 0, sizeof(buffer));

	buffer[0] = VFD_SETTIME;
	buffer[1] = ((time[6] - '0') << 4) | (time[7] - '0');  // hour
	buffer[2] = ((time[8] - '0') << 4) | (time[9] - '0');  // minute
	buffer[3] = ((time[10] - '0') << 4) | (time[11] - '0');  //second
	res = micomWriteCommand(buffer, 5, 0);

	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

/*****************************************************
 *
 * micomGetTime: read front panel clock time.
 *
 * Returns 6 bytes:
 * char seconds
 * char minutes
 * char hours
 * char day of the month
 * char month
 * char year (no century)
 */
int micomGetTime(unsigned char *time)
{
	unsigned char buffer[5];
	int  res = 0;

	dprintk(100, "%s >\n", __func__);

	memset(buffer, 0, sizeof(buffer));

	buffer[0] = VFD_GETDATETIME;
	errorOccured = 0;
	res = micomWriteCommand(buffer, 5, 1);

	if (errorOccured == 1)
	{
		/* error */
		memset(ioctl_data, 0, 20);
		printk("error\n");
		res = -ETIMEDOUT;
	}
	else
	{
		dprintk(1, "Time received\n");
		dprintk(10, "FP/RTC time: %02x:%02x:%02x %02x-%02x-20%02x\n",
			ioctl_data[2], ioctl_data[1], ioctl_data[0],
			ioctl_data[3], ioctl_data[4], ioctl_data[5]);
		memcpy(time, ioctl_data, 6);
	}
	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

/****************************************************************
 *
 * micomGetWakeUpTime: read stored wakeup time.
 *
 * Returns 6 bytes:
 * char seconds (not stored)
 * char minutes
 * char hours
 * char day of the month
 * char month
 * char year (no century)
 */
int micomGetWakeUpTime(unsigned char *time)
{
	unsigned char buffer[5];
	int res = 0;

	dprintk(100, "%s >\n", __func__);

	memset(buffer, 0, sizeof(buffer));

	buffer[0] = VFD_GETWAKEUP;

	errorOccured = 0;
	res = micomWriteCommand(buffer, 5, 1);

	if (errorOccured == 1)
	{
		/* error */
		memset(ioctl_data, 0, 20);
		printk("error\n");
		res = -ETIMEDOUT;
	}
	else
	{
		dprintk(1, "Wake up time received\n");
		dprintk(10, "Wake up time: %02x:%02x:%02x %02x-%02x-20%02x\n",
			(int)ioctl_data[2], (int)ioctl_data[1], (int)ioctl_data[0],
			(int)ioctl_data[3], (int)ioctl_data[4], (int)ioctl_data[5]);
		memcpy(time, ioctl_data, 6);
	}
	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

/****************************************************************
 *
 * micomGetVersion: get version/release date of front processor.
 *
 * Note: only returns a code representing the display type
 * TODO: return actual version string through procfs
 */
int micomGetVersion(void)
{
	unsigned char buffer[5];
	int res = -1;

	dprintk(100, "%s >\n", __func__);

#if defined(CUBEREVO_3000HD) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_250HD) \
 || defined(CUBEREVO_MINI_FTA)  // fixme: not sure if true for CUBEREVO250HD/MINI_FTA!!!
	micom_year  = 2008;
	micom_minor = 4;
	res = 0;

#else
	memset(buffer, 0, sizeof(buffer));  // after power on the front processor
	buffer[0] = VFD_GETMICOM;  // does not always respond
	res = micomWriteCommand(buffer, 5, 1);  // so give it a nudge

	memset(buffer, 0, sizeof(buffer));  // and try again
	buffer[0] = VFD_GETMICOM;
	errorOccured = 0;
	res = micomWriteCommand(buffer, 5, 1);

	if (errorOccured == 1)
	{
		/* error */
		memset(ioctl_data, 0, sizeof(ioctl_data));
		printk("error\n");
		res = -ETIMEDOUT;
	}
	else
	{
		char convertDate[128];
		
		dprintk(100, "0x%02x 0x%02x 0x%02x\n", ioctl_data[0], ioctl_data[1], ioctl_data[2]);
		sprintf(convertDate, "%02x %02x %02x\n", ioctl_data[0], ioctl_data[1], ioctl_data[2]);
		sscanf(convertDate, "%d %d %d", &micom_year, &micom_minor, &micom_major);
		micom_year  += 2000;
	}
#endif

	dprintk(1, "Frontpanel version: %02d.%02d.%04d\n", micom_major, micom_minor, micom_year);

#if defined(CUBEREVO_250HD) \
 || defined(CUBEREVO_MINI_FTA)
	front_seg_num    = 4;  // LED models
	num2seg          = num2seg_7seg;
	Char2seg         = Char2seg_7seg;
	LowerChar2seg    = NULL;
	special2seg      = special2seg_7seg;
	special2seg_size = ARRAY_SIZE(special2seg_7seg);
#else  // VFD models
#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_3000HD) \
 || defined(CUBEREVO_2000HD)
	if ((micom_year == 2008) && (micom_minor == 4 || micom_minor == 6))
	{
		front_seg_num    = 14;
		num2seg          = num2seg_14dotmatrix;
		Char2seg         = Char2seg_14dotmatrix;
		LowerChar2seg    = LowerChar2seg_14dotmatrix;
		special2seg      = special2seg_14dotmatrix;
		special2seg_size = ARRAY_SIZE(special2seg_14dotmatrix);
	}
	else
#elif defined(CUBEREVO)
	if ((micom_year == 2008) && (micom_minor == 3))
	{	
		front_seg_num    = 12;
		num2seg          = num2seg_12dotmatrix;
		Char2seg         = Char2seg_12dotmatrix;
		LowerChar2seg    = LowerChar2seg_14dotmatrix;
		special2seg      = special2seg_14dotmatrix;
		special2seg_size = ARRAY_SIZE(special2seg_14dotmatrix);
	}
	else
#endif
	{
		front_seg_num    = 13;
		num2seg          = num2seg_13grid;
		Char2seg         = Char2seg_13grid;
		LowerChar2seg    = LowerChar2seg_13grid;
		special2seg      = special2seg_13grid;
		special2seg_size = ARRAY_SIZE(special2seg_13grid);
	}
#endif  // 250HD
	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

/*************************************************************
 *
 * micomGetWakeUpMode: read wake up reason from front panel.
 *
 * Note: always returns 2 except on power on.
 */
int micomGetWakeUpMode(unsigned char *mode)
{
	unsigned char buffer[5];
	int res = 0;

	dprintk(100, "%s >\n", __func__);

	memset(buffer, 0, sizeof(buffer));

	buffer[0] = VFD_GETWAKEUPSTATUS;

	errorOccured = 0;
	res = micomWriteCommand(buffer, 5, 1);

	if (errorOccured == 1)
	{
		/* error */
		memset(ioctl_data, 0, sizeof(ioctl_data));
		printk("%s Error\n", __func__);
		res = -ETIMEDOUT;
	}
	else
	{
		dprintk(10, "Wake up reason = 0x%02x\n", ioctl_data[0]);
		*mode = ioctl_data[0];
	}
	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

#if defined(VFDTEST)
/*************************************************************
 *
 * micomVfdTest: Send arbitrary bytes to front processor
 *               and read back status and if applicable
 *               return bytes (for development purposes).
 *
 */
int micomVfdTest(unsigned char *data)
{
	unsigned char buffer[5];
	int res = 0xff;
	int i;

	dprintk(100, "%s > len = %d\n", __func__, data[0]);
	for (i = 1; i <= data[0]; i++)
	{
		data[i] = data[i] & 0xff;
		dprintk(1, "%s data[%d] = 0x%02x\n", __func__, i, data[i]);
	}

	if (data[1] < VFD_GETA0 || data[1] > VFD_SETCLEARSEGMENTS)  // if command is not valid
	{
		dprintk(1, "Unknown command, must be between %02x and %02x\n", VFD_GETA0, VFD_SETCLEARSEGMENTS);
		return -1;
	}
	memset(buffer, 0, sizeof(buffer));
	memset(ioctl_data, 0, sizeof(ioctl_data));

	memcpy(buffer, data + 1, 5);  // copy the 5 command bytes

	if (paramDebug >= 50)
	{
		printk("[micom] Send command: [0x%02x] ", buffer[0] & 0xff);
		for (i = 1; i < 5; i++)
		{
			printk("0x%02x ", buffer[i] & 0xff);
		}
		printk("\n");
	}

	errorOccured = 0;
	res = micomWriteCommand(buffer, sizeof(buffer), 1);

	if (errorOccured == 1)
	{
		/* error */
		memset(data + 1, 0, sizeof(data) - 1);
		printk("[micom] Error in function %s\n", __func__);
		res = data[0] = -ETIMEDOUT;
	}
	else
	{
		data[0] = 0;  // command went OK
		data[1] = ((buffer[0] <= VFD_GETWAKEUP) ? 1 : 0);
		for (i = 0; i < 6; i++)  // we can receive up 6 bytes back
		{
			data[i + 2] = ioctl_data[i];  // copy return data
		}
	}
	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}
#endif

/*****************************************************************
 *
 * micomWriteString: Display a text on the front panel display.
 *
 * Note: does not scroll; displays the first DISPLAY_WIDTH
 *       characters. Scrolling is available by writing to
 *       /dev/vfd: this scrolls a maximum of 64 characters once
 *       if the text length exceeds front_seg_num.
 *
 */
inline char toupper(const char c)
{
	if ((c >= 'a') && (c <= 'z'))
	{
		return (c - 0x20);
	}
	return c;
}

inline int trimTrailingBlanks(char *txt, int len)
{
	int i;

	dprintk(100, "%s > String: '%s', len = %d\n", __func__, txt, len);
	for (i = len - 1; (i > 0 && txt[i] == ' '); i--, len--)
	{
		txt[i] = '\0';
	}
	if (len > front_seg_num)
	{
		len = front_seg_num;
	}
	dprintk(100, "%s < String: '%s', len = %d\n", __func__, txt, len);
	return len;
}

int micomWriteString(unsigned char *aBuf, int len)
{
	unsigned char buffer[5];
	unsigned char bBuf[128];
	int           res = 0, i, j;
	int           pos = 0;
	unsigned char space;

	dprintk(100, "%s > String: %s, len = %d\n", __func__, aBuf, len);

	if (currentDisplayTime == 1)
	{
		dprintk(1, "display in time mode -> ignoring display text\n");
		//TODO: store to use when time mode is switched off
		return 0;
	}
	aBuf[len] = '\0';

#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD)
	/* The 14 character front processor cannot display accented letters.
	 * The following code traces for UTF8 sequences for these and
	 * replaces them with the corresponding letter without any accent.
	 * This is not perfect, but at least better than the old practice
	 * of replacing them with spaces.
	 */
	dprintk(50, "%s UTF8 text: [%s], len = %d\n", __func__, aBuf, len);
	memset(bBuf, ' ', sizeof(bBuf));
	j = 0;

	// process aBuf byte by byte
	for (i = 0; i < len; i++)
	{
		if (aBuf[i] == 0x5c)  // handle backslash
		{
			bBuf[j] = 0xa0;
			j++;
		}
		else if (aBuf[i] < 0x80)
		{
			bBuf[j] = aBuf[i];
			j++;
		}
		else if (aBuf[i] < 0xd0)  // if between 0x80 and 0xcf
		{
			switch (aBuf[i])
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
				default:
				{
					dprintk(1, "%s Unsupported extension 0x%02x found\n", __func__, aBuf[i]);
					UTF_Char_Table = NULL;
				}
			}
			i++;  // skip lead in byte
			if (UTF_Char_Table)  // if an applicable table there
			{
				if (UTF_Char_Table[aBuf[i] & 0x3f] != 0)  // if character is printable
				{
					bBuf[j] = UTF_Char_Table[aBuf[i] & 0x3f];  // get character from table
					dprintk(50, "%s character from table: %c\n", __func__, bBuf[j]);
					j++;
				}
				else
				{
					dprintk(1, "%s UTF8 character is unprintable, ignore.\n", __func__);
					i++;  //skip character
				}
			}
		}
		else
		{
			if (aBuf[i] < 0xf0)  // if between 0xe0 and 0xef
			{
				i += 2;  // skip 2 bytes
			}
			else if (aBuf[i] < 0xf8)  // if between 0xf0 and 0xf7
			{
				i += 3;  // skip 3 bytes
			}
			else if (aBuf[i] < 0xfc)  // if between 0xf8 and 0xfb
			{
				i += 4;  // skip 4 bytes
			}
			else  // if between 0xfc and 0xff
			{
				i += 5;  // skip 5 bytes
			}
			bBuf[j] = 0x20;  // else put a space
			j++;
		}
	}		
	len = j;
	bBuf[len] = '\0';  // terminate string
	memcpy(aBuf, bBuf, len);
	dprintk(50, "%s Non-UTF8 text: [%s], len = %d\n", __func__, bBuf, len);
#elif defined(CUBEREVO)
	if (front_seg_num == 13)
	{
		/* The 13 character front processor cannot display accented letters.
		 * The following routine traces for UTF8 sequences for these and
		 * replaces them with the corresponding letter without any accent.
		 * This is not perfect, but at least better than the old practice
		 * of replacing them with spaces.
		 */
		dprintk(50, "%s UTF8 text: [%s], len = %d\n", __func__, aBuf, len);
		memset(bBuf, ' ', sizeof(bBuf));
		j = 0;

		// process aBuf byte by byte
		for (i = 0; i < len; i++)
		{
			if (aBuf[i] < 0x80)
			{
				bBuf[j] = aBuf[i];
				j++;
			}
			else if (aBuf[i] < 0xd0)  // if between 0x80 and 0xcf
			{
				switch (aBuf[i])
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
					default:
					{
						dprintk(1, "%s Unsupported extension 0x%02x found\n", __func__, aBuf[i]);
						UTF_Char_Table = NULL;
					}
				}
				i++;  // skip lead in byte
				if (UTF_Char_Table)  // if an applicable table there
				{
					if (UTF_Char_Table[aBuf[i] & 0x3f] != 0)  // if character is printable
					{
						bBuf[j] = UTF_Char_Table[aBuf[i] & 0x3f];  // get character from table
						dprintk(50, "%s character from table: %c\n", __func__, bBuf[j]);
						j++;
					}
					else
					{
						dprintk(1, "%s UTF8 character is unprintable, ignore.\n", __func__);
						i++;  //skip character
					}
				}
			}
			else
			{
				if (aBuf[i] < 0xf0)  // if between 0xe0 and 0xef
				{
					i += 2;  // skip 2 bytes
				}
				else if (aBuf[i] < 0xf8)  // if between 0xf0 and 0xf7
				{
					i += 3;  // skip 3 bytes
				}
				else if (aBuf[i] < 0xfc)  // if between 0xf8 and 0xfb
				{
					i += 4;  // skip 4 bytes
				}
				else  // if between 0xfc and 0xff
				{
					i += 5;  // skip 5 bytes
				}
				bBuf[j] = 0x20;  // else put a space
				j++;
			}
		}		
		len = j;
		bBuf[len] = '\0';  // terminate string
		memcpy(aBuf, bBuf, len);
		dprintk(50, "%s Non-UTF8 text: [%s], len = %d\n", __func__, bBuf, len);
	}
	else 
	{
		// TODO: insert UTF8 processing for 12seg
	}
#endif
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = VFD_SETCLEARTEXT;
	res = micomWriteCommand(buffer, 5, 0);

//	if (scrolling == 0)
//	{
		len = trimTrailingBlanks(aBuf, len);
//	}

	pos = front_seg_num - len;  // get # of empty characters / trailing spaces
#if defined(CENTERED_DISPLAY) 
	// centered display
	pos /= 2;

	for (i = 0; i < pos; i++)
	{
		bBuf[i] = ' ';
	}
	for (j = 0; j < len && pos < front_seg_num; pos++, i++, j++)
	{
		bBuf[i] = aBuf[j];
	}
	for (; pos < front_seg_num; pos++, i++)
	{
		bBuf[i] = ' ';
	}
#else
	// left aligned display
	memcpy(bBuf, aBuf, len);
	memset(bBuf + len, ' ', pos);
#endif
	len = front_seg_num;

	/* nonprintable chars will be replaced by spaces */
	for (j = 0; j < special2seg_size; j++)
	{
		if (special2seg[j].ch == ' ')
		{
			break;
		}
	}
	space = special2seg[j].value;

	/* set text character by character */
	bBuf[len] = '\0';  // terminate string
	dprintk(50, "Text: %s (len = %d)\n", bBuf, len);
	for (i = 0; i < len; i++)
	{
		unsigned short data;
		unsigned char  ch;

		memset(buffer, 0, sizeof(buffer));

		buffer[0] = VFD_SETCHAR;
		buffer[1] = i & 0xff; /* position */

#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD)
		data = bBuf[i] - 0x10;  // get character from input and convert to display value
#else
		ch = bBuf[i];  // get character from input
		switch (ch)
		{
			case 'A' ... 'Z':  // if uppercase letter
			{
				ch -= 'A' - 'a';  // subtract 0x20
				data = Char2seg[ch - 'a'];  // and get character from table
				break;
			}
			case 'a' ... 'z':  // lower case letter
			{
				if (LowerChar2seg == NULL)  // and no table defined (LED models)
				{
					data = Char2seg[ch - 'a'];  // get uppercase letter from table
				}
				else
				{
					data = LowerChar2seg[ch - 'a'];  // else get lower case letter from table
				}
				break;
			}
			case '0' ... '9':  // if digit
			{
				data = num2seg[ch - '0'];  // get character from table
				break;
			}
			default:  // other
			{
				for (j = 0; j < special2seg_size; j++)  // search for character
				{
					if (special2seg[j].ch == ch)  // if special character table
					{
						break;
					}
				}
				if (j < special2seg_size)  // if found
				{
					data = special2seg[j].value;  // get value
					break;
				}
				else
				{
					dprintk(1, "%s ignore unprintable character \'%c\'\n", __func__, ch);
					data = space;  // and print a space
				}
				break;
			}
		}
#endif
		dprintk(150, "%s data 0x%x \n", __func__, data);

		buffer[2] = data & 0xff;
		buffer[3] = (data >> 8) & 0xff;  // ignored on MINI, MINI2
		res = micomWriteCommand(buffer, 5, 0);
	}
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = VFD_SETDISPLAYTEXT;
	res = micomWriteCommand(buffer, 5, 0);

	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

/****************************************************************
 *
 * micom_init_func: Initialize the driver.
 *
 */
int micom_init_func(void)
{
	int  res = 0;
	int  vLoop;
	unsigned char buffer[5];

	dprintk(100, "%s >\n", __func__);

	sema_init(&write_sem, 1);

	printk("Cuberevo ");
#if defined(CUBEREVO_MINI)
	printk("Mini ");
#elif defined(CUBEREVO_MINI2)
	printk("Mini II ");
#elif defined(CUBEREVO_MINI_FTA)
	printk("200HD ");
#elif defined(CUBEREVO_250HD)
	printk("250HD ");
#elif defined(CUBEREVO_2000HD)
	printk("2000HD ");
#elif defined(CUBEREVO_3000HD)
	printk("3000HD ");
#elif defined(CUBEREVO_9500HD)
	printk("9500HD ");
#endif
	printk("Micom front panel driver version V%s initializing\n", driver_version);

	memset(buffer, 0, sizeof(buffer));  // after power on the front processor
	buffer[0] = VFD_GETMICOM;  // does not always respond
	res = micomWriteCommand(buffer, 5, 1);  // so give it a nudge

	micomGetVersion();
#if defined(CUBEREVO)
	res |= micomSetFan(0);
#endif
	res |= micomSetLED(3);  // LED on and slow blink mode
	res |= micomSetBrightness(7);
	res |= micomSetTimeMode(1);  // 24h mode
	res |= micomSetDisplayTime(0);  // mode = display text
//	res |= micomWriteString("T.Ducktales", strlen("T.Ducktales"));

	/* disable all icons at startup */
#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD) \
 || defined(CUBEREVO)
	for (vLoop = ICON_MIN + 1; vLoop < ICON_MAX; vLoop++)
	{
		micomSetIcon(vLoop, 0);
	}
#endif
#if defined(CUBEREVO)
	if (front_seg_num == 12)
	{
		init_timer(&playTimer);
		playTimer.function = animated_play;
		playTimer.data = 0;
	}
#endif
	// Handle initial GMT offset (may be changed by writing to /proc/stb/fp/rtc_offset)
	res = strict_strtol(gmt_offset, 10, (long *)&rtc_offset);
	if (res && gmt_offset[0] == '+')
	{
		res = strict_strtol(gmt_offset + 1, 10, (long *)&rtc_offset);
	}
	dprintk(100, "%s < 0\n", __func__);
	return 0;
}

/******************************************
 *
 * Code for writing to /dev/vfd (not-test)
 *
 */
void clear_display(void)
{
	unsigned char bBuf[14];
	int res = 0;

	dprintk(100, "%s >\n", __func__);

	memset(bBuf, ' ', sizeof(bBuf));
	res = micomWriteString(bBuf, front_seg_num);
	dprintk(100, "%s <\n", __func__);
}

//#define TEST_COMMANDS
//#define TEST_CHARSET
#if !defined(TEST_CHARSET) \
 && !defined(TEST_COMMANDS)
static ssize_t MICOMdev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char *kernel_buf;
	int minor, vLoop, res = 0;
	int pos, offset, llen;
	unsigned char buf[80];  // 64 plus maximum length plus two

	dprintk(100, "%s > (len %d, off %d)\n", __func__, len, (int)*off);

	if (len == 0)  // || (len == 1 && buff[0] =='\n'))
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
		printk("Error Bad Minor\n");
		return -1;  //FIXME
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
		printk("%s return no memory <\n", __func__);
		return -ENOMEM;
	}
	copy_from_user(kernel_buf, buff, len);

	if (write_sem_down())
	{
		return -ERESTARTSYS;
	}

	llen = len;  // leave len untouched, work with llen

	/* Dagobert: echo add a \n which will not be counted as a char */
	/* Audioniek: Does not add a \n but strips it from string      */
	if (kernel_buf[len - 1] == '\n')
	{
		llen--;
	}
	if (llen >= 64)  // do not display more than 64 characters
	{
		llen = 64;
	}
	kernel_buf[llen] = '\0';  // terminate string

	if (llen <= front_seg_num)  // no scroll
	{
		res = micomWriteString(kernel_buf, llen);
	}
	else  // scroll, display string is longer than display length
	{
//		scrolling = 1;  // flag in scroll
		memset(buf, ' ', sizeof(buf));
		// initial display starting at 3rd position to ease reading
		offset = 3;
		memcpy(buf + offset, kernel_buf, llen);
		llen += offset;
		buf[llen + front_seg_num - 1] = '\0';  // terminate string

		for (pos = 1; pos < llen; pos++)
		{
			res |= micomWriteString(buf + pos, llen + front_seg_num);
			// sleep 300 ms
			msleep(300);
		}
		// final display
		clear_display();
		res |= micomWriteString(kernel_buf, front_seg_num);
//		scrolling = 0;
	}
	kfree(kernel_buf);
	write_sem_up();
	dprintk(70, "%s < res %d len %d\n", __func__, res, len);

	if (res < 0)
	{
		return res;
	}
	else
	{
		return len;
	}
}
#elif defined(TEST_CHARSET)
/**************************************************
 *
 * Code for writing to /dev/vfd (test characterset)
 *
 */
static ssize_t MICOMdev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char *kernel_buf;
	int minor, vLoop, res = 0;
	char *myString = kmalloc(len + 1, GFP_KERNEL);
	int value;
	char buffer[5];

	dprintk(100, "%s > (len %d, offs %d)\n", __func__, len, (int) *off);

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
		printk("Error: Bad Minor\n");
		return -1;  //FIXME
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
		dprintk(1, "%s return no memory <\n", __func__);
		return -ENOMEM;
	}
	copy_from_user(kernel_buf, buff, len);

	if (write_sem_down())
	{
		return -ERESTARTSYS;
	}
	strncpy(myString, kernel_buf, len);
	myString[len] = '\0';

	dprintk(1, "%s String to print: %s\n", __func__, myString);
	sscanf(myString, "%d", &value);
	dprintk(1, "%s Hex value 0x%x\n", value);

	memset(buffer, 0, sizeof(buffer));
	buffer[0] = VFD_SETCLEARTEXT;
	res = micomWriteCommand(buffer, 5, 0);

	memset(buffer, 0, sizeof(buffer));
	buffer[0] = VFD_SETCHAR;
	buffer[1] = 1;
	buffer[2] = value & 0xff;
	buffer[3] = (value >> 8) & 0xff;
	res = micomWriteCommand(buffer, 5, 0);

	kfree(kernel_buf);

	memset(buffer, 0, sizeof(buffer));
	buffer[0] = VFD_SETDISPLAYTEXT;
	res = micomWriteCommand(buffer, 5, 0);

	write_sem_up();

	dprintk(70, "%s < res %d len %d\n", __func__, res, len);

	if (res < 0)
	{
		return res;
	}
	else
	{
		return len;
	}
}
#else
/**********************************************
 *
 * Code for writing to /dev/vfd (test command)
 *
 */
static ssize_t MICOMdev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char *kernel_buf;
	int minor, vLoop, res = 0;
	char *myString = kmalloc(len + 1, GFP_KERNEL);
	int value, arg1, arg2, arg3;
	char buffer[5];
	int i;

	dprintk(100, "%s > (len %d, offs %d)\n", __func__, len, (int) *off);

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
		printk("Error: Bad Minor\n");
		return -1;  //FIXME
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
		dprintk(1, "%s return no memory <\n", __func__);
		return -ENOMEM;
	}
	copy_from_user(kernel_buf, buff, len);

	if (write_sem_down())
	{
		return -ERESTARTSYS;
	}
	strncpy(myString, kernel_buf, len);
	myString[len] = '\0';  // terminate string

	printk(1, "%s String: %s\n", __func__, myString);
	i = sscanf(myString, "%x %x %x %x", &value, &arg1, &arg2, &arg3);
	dprintk("CMD (%d): 0x%x 0x%x 0x%x 0x%x\n", i, value, arg1, arg2, arg3);

	if ((value != 0xd2)  //VFD_SETD2
	&&  (value != 0xd3)  //VFD_SETD3
	&&  (value != 0xd4)  //VFD_SETD4
	&&  (value != 0xd6)  //VFD_SETD6
	&&  (value != 0xcb)  //VFD_SETCB
	&&  (value != 0xa0)  //VFD_GETA0
	&&  (value != 0xa2))  //VFD_GETA2
	{
		write_sem_up();
		return len;
	}
//#define bimmel
#if defined(bimmel)
	for (i = 0; i <= 255; i++)
	{
		dprintk(1, "0x%x ->0x%x\n", value, i);

		memset(buffer, 0, sizeof(buffer));
		buffer[0] = value;
		buffer[1] = 1;
		buffer[2] = i;
		buffer[3] = 0;
		res = micomWriteCommand(buffer, 5, 0);
		msleep(125);
	}
#else
	{
		memset(buffer, 0, sizeof(buffer));
		buffer[0] = value;
		buffer[1] = arg1;
		buffer[2] = arg2;
		buffer[3] = arg3;
		res = micomWriteCommand(buffer, 5, 0);
	}
#endif
	kfree(kernel_buf);

	write_sem_up();

	dprintk(70, "%s < res %d len %d\n", __func__, res, len);

	if (res < 0)
	{
		return res;
	}
	else
	{
		return len;
	}
}
#endif

/**********************************
 *
 * Code for reading from /dev/vfd
 *
 */
static ssize_t MICOMdev_read(struct file *filp, char __user *buff, size_t len, loff_t *off)
{
	int minor, vLoop;

	dprintk(100, "%s > (len %d, offs %d)\n", __func__, len, (int) *off);

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
		printk("Error: Bad Minor\n");
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
		dumpValues();
		return 0;
	}

	/* copy the current display string to the user */
	if (down_interruptible(&FrontPanelOpen[minor].sem))
	{
		printk("%s return erestartsys <\n", __func__);
		return -ERESTARTSYS;
	}
	if (FrontPanelOpen[minor].read == lastdata.length)
	{
		FrontPanelOpen[minor].read = 0;

		up (&FrontPanelOpen[minor].sem);
		printk("%s return 0 <\n", __func__);
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

	up (&FrontPanelOpen[minor].sem);

	dprintk(100, "%s < (len %d)\n", __func__, len);
	return len;
}

int MICOMdev_open(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(100, "%s >\n", __func__);

	/* needed! otherwise a race condition can occur */
	if (down_interruptible (&write_sem))
	{
		return -ERESTARTSYS;
	}
	minor = MINOR(inode->i_rdev);

	dprintk(70, "open minor %d\n", minor);

	if (FrontPanelOpen[minor].fp != NULL)
	{
		printk("EUSER\n");
		up(&write_sem);
		return -EUSERS;
	}
	FrontPanelOpen[minor].fp = filp;
	FrontPanelOpen[minor].read = 0;

	up(&write_sem);
	dprintk(100, "%s <\n", __func__);
	return 0;
}

int MICOMdev_close(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(100, "%s >\n", __func__);

	minor = MINOR(inode->i_rdev);

	dprintk(20, "close minor %d\n", minor);

	if (FrontPanelOpen[minor].fp == NULL)
	{
		printk("EUSER\n");
		return -EUSERS;
	}
	FrontPanelOpen[minor].fp = NULL;
	FrontPanelOpen[minor].read = 0;
	dprintk(100, "%s <\n", __func__);
	return 0;
}

/****************************************
 *
 * IOCTL handling.
 *
 */
static int MICOMdev_ioctl(struct inode *Inode, struct file *File, unsigned int cmd, unsigned long arg)
{
	static int mode = 0;
	struct micom_ioctl_data *micom = (struct micom_ioctl_data *)arg;
	int res = 0;

	dprintk(100, "%s > IOCTL: %.8x\n", __func__, cmd);

	if (down_interruptible(&write_sem))
	{
		return -ERESTARTSYS;
	}
	switch (cmd)
	{
		case VFDSETMODE:
		{
			mode = micom->u.mode.compat;
			break;
		}
		case VFDSETLED:
		{
			res = micomSetLED(micom->u.led.on);
			break;
		}
		case VFDBRIGHTNESS:
		{
			if (mode == 0)
			{
				struct vfd_ioctl_data *data = (struct vfd_ioctl_data *) arg;
				res = micomSetBrightness(data->start);
			}
			else
			{
				res = micomSetBrightness(micom->u.brightness.level);
			}
			mode = 0;
			break;
		}
		case VFDLEDBRIGHTNESS:
		{
			break;			
		}
		case VFDDRIVERINIT:
		{
			res = micom_init_func();
			mode = 0;
			break;
		}
		case VFDICONDISPLAYONOFF:
		{
			if (micom_year > 2007)
			{
				if (mode == 0)
				{
					struct vfd_ioctl_data *data = (struct vfd_ioctl_data *) arg;
					int icon_nr = (data->data[0] & 0xf) + 1;
					int on = data->data[4];
					res = micomSetIcon(icon_nr, on);
				}
				else
				{
					res = micomSetIcon(micom->u.icon.icon_nr, micom->u.icon.on);
				}
			}
			else
			{
				res = 0;
			}
			mode = 0;
			break;
		}
		case VFDSTANDBY:
		{
			res = micomSetStandby(micom->u.standby.time);
			break;
		}
		case VFDREBOOT:
		{
			res = micomReboot();
			break;
		}
		case VFDSETTIME:
		{
			if (micom->u.time.time != 0)
			{
				res = micomSetTime(micom->u.time.time);
			}
			break;
		}
		case VFDSETFAN:
		{
			if (mode == 0)
			{
				struct vfd_ioctl_data *data = (struct vfd_ioctl_data *) arg;
				int on = data->start;
				res = micomSetFan(on);
			}
			else
			{
				res = micomSetFan(micom->u.fan.on);
			}
			break;
		}
		case VFDSETRF:
		{
			res = micomSetRF(micom->u.rf.on);
			break;
		}
		case VFDSETTIMEMODE:
		{
			res = micomSetTimeMode(micom->u.time_mode.twentyFour);
			break;
		}
		case VFDSETDISPLAYTIME:
		{
			res = micomSetDisplayTime(micom->u.display_time.on);
			break;
		}
		case VFDGETTIME:
		{
			res = micomGetTime(micom->u.get_time.time);
			break;
		}
		case VFDGETWAKEUPTIME:
		{
			res = micomGetWakeUpTime(micom->u.get_wakeup_time.time);
			break;
		}
		case VFDSETWAKEUPTIME:
		{
			if (micom->u.time.time != 0)
			{
				res = micomSetWakeUpTime(micom->u.wakeup_time.time);
			}
			break;
		}
		case VFDGETWAKEUPMODE:
		{
			res = micomGetWakeUpMode(&micom->u.status.status);
			break;
		}
		case VFDDISPLAYCHARS:
		{
			if (mode == 0)
			{
				struct vfd_ioctl_data *data = (struct vfd_ioctl_data *) arg;
				res = micomWriteString(data->data, data->length);
			}
			else
			{
				//not supported
			}
			mode = 0;
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			char buffer[5];
			struct vfd_ioctl_data *data = (struct vfd_ioctl_data *) arg;
			int on = data->start;

			if (on)
			{
				break;
			}
			/* clear text */
			memset(buffer, 0, sizeof(buffer));
			buffer[0] = VFD_SETCLEARTEXT;
			res = micomWriteCommand(buffer, 5, 0);

			memset(buffer, 0, sizeof(buffer));
			buffer[0] = VFD_SETDISPLAYTEXT;
			res = micomWriteCommand(buffer, 5, 0);
			/* and fall through to clear icons */
		}
		case VFDCLEARICONS:
		{
			res = 0;
			if (micom_year > 2007)
			{
				res |= micomClearIcons();
			}
			break;
		}
		case VFDGETVERSION:
		{
			if (front_seg_num == 12)
			{
				micom->u.version.version = 0;
			}
			else if (front_seg_num == 13)
			{
				micom->u.version.version = 1;
			}
			else if (front_seg_num == 14)
			{
				micom->u.version.version = 2;
			}
			else if (front_seg_num == 4)
			{
				micom->u.version.version = 3;
			}
			printk("[micom] VFDGETVERSION: version %d\n", micom->u.version.version);
			break;
		}
#if defined(VFDTEST)
		case VFDTEST:
		{
			res = micomVfdTest(micom->u.test.data);
			res |= copy_to_user((void *)arg, &(micom->u.test.data), 12);
			mode = 0;  // go back to vfd mode
			break;
		}
#endif
		case 0x5305:
		{
			mode = 0;  // go back to vfd mode
			break;
		}
		default:
		{
			dprintk(1, "Unknown IOCTL 0x%x.\n", cmd);
			mode = 0;
			break;
		}
	}
	up(&write_sem);
	dprintk(100, "%s <\n", __func__);
	return res;
}

struct file_operations vfd_fops =
{
	.owner   = THIS_MODULE,
	.ioctl   = MICOMdev_ioctl,
	.write   = MICOMdev_write,
	.read    = MICOMdev_read,
	.open    = MICOMdev_open,
	.release = MICOMdev_close
};
// vim:ts=4
