/*****************************************************************************
 *
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
 * CubeRevo 250HD / AB IPBox 91HD / Vizyon revolution 800HD: 4 character LED (7seg)
 * CubeRevo Mini / AB IPBox 900HD / Vizyon revolution 810HD: 14 character dot matrix VFD (14seg)
 * CubeRevo Mini II / AB IBbox 910HD / Vizyon revolution 820HD PVR: 14 character dot matrix VFD (14seg)
 * Early CubeRevo / AB IPBox 9000HD / Vizyon revolution 8000HD PVR: 13 character 14 segment VFD (13grid)
 * Late CubeRevo / AB IPBox 9000HD / Vizyon revolution 8000HD PVR: 12 character dot matrix VFD (12dotmatrix)
 * CubeRevo 2000HD: 14 character dot matrix VFD (14seg) NOT TESTED!
 * CubeRevo 3000HD: 14 character dot matrix VFD (14seg) NOT TESTED!
 * CubeRevo 7000HD: 12 character dot matrix VFD (12dotmatrix) NOT TESTED!
 * CubeRevo 9500HD: 12 character dot matrix VFD (12dotmatrix) NOT TESTED!
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
 * 20200524 Audioniek       UTF-8 support on 13grid added.
 * 20200524 Audioniek       Full ASCII display including lower case on
 *                          13grid added.
 * 20200620 Audioniek       Do not handle icons on 13grid
 * 20210519 Audioniek       Full ASCII display including upper case on
 *                          LED models added.
 * 20210519 Audioniek       No longer treat the front panel software version
 *                          as a date but as a normal 3 digit version
 *                          number.
 * 20210523 Audioniek       Correct CubeRevo: it has either a 13 character
 *                          14 segment display, or a 12 character dot matrix
 *                          display.
 * 20210524 Audioniek       Time mode off restores previous display.
 * 20210530 Audioniek       Improve 13grid fonts.
 * 20210604 Audioniek       Add 12dotmatrix lower case.
 * 20210604 Audioniek       Add 12dotmatrix special characters -> now full ASCII.
 * 20210604 Audioniek       Add 12dotmatrix UTF-8 support.
 * 20210604 Audioniek       Icons on 12dotmatrix overhauled; add code for all icons
 *                          on/off.
 * 20210605 Audioniek       Spinner on 12dotmatrix added.
 * 20210605 Audioniek       Icon translation for Enigma2 added.
 * 20210606 Audioniek       Write string and UTF-8 handling simplified.
 * 20210606 Audioniek       VFDDISPLAYWRITEONOFF restores previous display,
 *                          including icons and spinner.
 * 20210607 Audioniek       Icon processing overhauled, add all icons on/off,
 *                          Separate play icon and add new spinner icon on
 *                          12dotmatrix.
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
#include "cuberevo_micom_utf.h"

extern const char *driver_version;
extern void ack_sem_up(void);
extern int  ack_sem_down(void);
int micomWriteString(unsigned char *aBuf, int len, int center_flag);
extern void micom_putc(unsigned char data);

struct semaphore write_sem;

int errorOccured = 0;
//int scrolling = 0;  // flag: switch off scroll display in /dev/vfd
int currentDisplayTime = 0;  // flag to indicate display time mode
static char ioctl_data[20];

struct saved_data_s lastdata;

tFrontPanelOpen FrontPanelOpen [LASTMINOR];


#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
tSpinnerState spinner_state;
#endif
/* version of fp */
int micom_ver, micom_major, micom_minor;

typedef struct _special_char
{
	unsigned char  ch;
	unsigned short value;
} special_char_t;


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
 * FIXME/TODO: As all models now have full ASCII support, code is
 *             needlessly complicated; it is possible to have one
 *             conversion table per display type now.
 *
 ***************************************************************************/

#if defined(CUBEREVO)
/*******************************************
 *
 * 13 grid (Early CubeRevo and possibly early 9500HD)
 *
 * The 13 character VFD has 13 identical
 * 14 segment characters with a decimal 
 * point each.
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
 *              hi byte, bit 7 (128, +0x8000, unused)
 *
 * Character tables are 16 bit words,
 * representing segment patterns.
 */
unsigned short num2seg_13grid[] =
{
	0x3123,  // 0
	0x0106,  // 1
	0x30c3,  // 2
	0x21c3,  // 3
	0x01e2,  // 4
	0x21e1,  // 5
	0x31e1,  // 6
	0x0103,  // 7
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
	0x1136,  // M
	0x1332,  // N
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
	{ '&',  0x31F0 },  // 0x26 &
	{ 0x27, 0x0020 },  // 0x27 ' 
	{ '(',  0x0204 },  // 0x28 (
	{ ')',  0x0810 },  // 0x29 )
	{ '*',  0x0edc },  // 0x2a *
	{ '+',  0x04c8 },  // 0x2b +
	{ ',',  0x0800 },  // 0x2c ,
	{ '-',  0x00c0 },  // 0x2d -
	{ '.',  0x4000 },  // 0x2e .
	{ '/',  0x0804 },  // 0x2f /
	{ ':',  0x0408 },  // 0x3a :
	{ ';',  0x0810 },  // 0x3b ;
	{ '<',  0x0204 },  // 0x3c <
	{ '=',  0x20c0 },  // 0x3d =
	{ '>',  0x0810 },  // 0x3e >
	{ '?',  0x4425 },  // 0x3f ?
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
#endif

#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD) \
 || defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD)
/***********************************************************
 *
 * The 12 and 14 character displays are very similar
 * technically speaking, as they share both the technology
 * (VFD) and driver chip (Princeton PT6302-003).
 * The characters on both displays are formed using 5x7
 * dot matrix pixel fields.
 * Differences are merely the number of characters and
 * the number of icons:
 * - The 12 character display has 35 icons and three colons
 *   positioned between the characters 6 & 7, 8 & 9 and
 *   10 & 11. In addition it should be noted that the 12
 *   character display has a striking similarity with the
 *   display used on the Fortis FS9000.
 * - The 14 character display has 8 icons and no colons.
 *
 * Due to the similar setup, the 12 and 14 character
 * displays share much code and the character tables.
 */
unsigned short num2seg_pt6302[] =
{  // table is ASCII minus 0x10
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

unsigned short Char2seg_pt6302[] =
{  // table is ASCII minus 0x10
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

unsigned short LowerChar2seg_pt6302[] =
{  // table is ASCII minus 0x10
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

special_char_t special2seg_pt6302[] =
{  // table is largely ASCII minus 0x10
//	{ ' ',	0x10 },
	{ '!',	0x11 },  // ->> ASCII - 0x10
	{ '"',	0x12 },
	{ '#',	0x13 },
	{ '$',	0x14 },
	{ '%',	0x15 },
	{ '&',	0x16 },
	{ 0x27,	0x17 },
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
	{ ':',	0x2a },
	{ ';',	0x2b },
	{ '<',	0x2c },
	{ '=',	0x2d },
	{ '>',	0x2e },
	{ '?',	0x2f },
	{ '@',	0x30 },
//	{ 'A',	0x31 }
//	     |       
//	{ 'Z',	0x4a },
	{ '[',	0x4b },
	{ 0x5c, 0x90 }, 
	{ ']',	0x4d },
	{ '^',	0x4e },
	{ '_',	0x4f },

	{ '`',	0x50 },  // back quote
//	{ 'a',	0x51 },
//	     |
//	{ 'z',	0x6a },
	{ '{',	0x6b },
	{ '|',	0x6c },
	{ '}',	0x6d },
	{ '~',	0x6e },
	{ 0x7f,	0x6f },  // DEL, full block  // end of ASCII - 0x10

//	{ '?',  0x70 },  // large alpha
//       |
//	{ '?',	0x7e },  // large omega
//	{ '?',	0x7f },  // large epsilon

// Special characters, as present in PT6302-003
// Uncommented ones are used in UTF-8 conversions
	{ 0x80,	0x80 },  // pound sign
	{ 0x81,	0x81 },  // paragraph
//	{ '?',	0x82 },  // large IE diacritic
//	{ '?',	0x83 },  // large IR diacritic
//	{ '?',	0x84 },  // integral sign
//	{ '?',	0x85 },  // invert x
//	{ '?',	0x86 },  // A accent dot
//	{ '?',	0x87 },  // power of -1
	{ 0x88,	0x88 },  // power of 2
	{ 0x89,	0x89 },  // power of 3
//	{ '?',	0x8a },  // power of x
//	{ '?',	0x8b },  // 1/2 -> c2 bd
//	{ '?',	0x8c },  // 1/ 
//	{ '?',	0x8d },  // square root
	{ 0x8e,	0x8e },  // +/-
//	{ '?',	0x8f },  // paragraph

	{ '\'',	0x90 },
//	{ '?',	0x91 },  // katakana
//	     |
//	{ '?',	0xce },  // katakana
	{ 0xcf,	0xcf },  // degree sign
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
	{ 0xdc,	0xdc },  // fat <
	{ 0xdd,	0xdd },  // fat >
//	{ '?',	0xde },  // three dots up
//	{ '?',	0xdf },  // three dots down
//	{ '?',	0xe0 },  // smaller or equal than
//	{ '?',	0xe1 },  // greater or equal than
//	{ '?',	0xe2 },  // unequal sign
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
//	{ '?',	0xee },  // ?
//	{ '?',	0xef },  // collapsed 8
//	{ '?',	0xf0 },  // small 1
//	{ '?',	0xf1 },  // small 2
//	{ '?',	0xf2 },  // small 3
//	{ '?',	0xf3 },  // small 4
//	{ '?',	0xf4 },  // small 5
//	{ '?',	0xf5 },  // small 6
//	{ '?',	0xf6 },  // small 7
//	{ '?',	0xf7 },  // small 8
//	{ '?',	0xf8 },  // small 9
//	{ '?',	0xf9 },  // small 10
//	{ '?',	0xfa },  // small 11
//	{ '?',	0xfb },  // small 12
//	{ '?',	0xfc },  // small 13
//	{ '?',	0xfd },  // small 14
//	{ '?',	0xfe },  // small 15
//	{ '?',	0xff },  // small 16
	{ ' ',	0x10 }   // space (EOT)
};
#endif

#if defined(CUBEREVO_250HD) \
 || defined(CUBEREVO_MINI_FTA)
/***************************************************************************
 *
 * Characters for 200HD (mini FTA) and 250HD (LED models)
 *
 *
 * character layout:
 *
 *      aaaaaaaaa
 *     f         b
 *     f         b
 *     f         b  i
 *     f         b
 *      ggggggggg
 *     e         c
 *     e         c  i
 *     e         c
 *     e         c
 *      ddddddddd  h
 *
 * Caution: to display a segment, its bit value must be zero, not one!
 *
 *  segment a is bit 0 (  1)
 *  segment b is bit 1 (  2)
 *  segment c is bit 2 (  4)
 *  segment d is bit 3 (  8)
 *  segment e is bit 4 ( 16, 0x10)
 *  segment f is bit 5 ( 32, 0x20)
 *  segment g is bit 6 ( 64, 0x40)
 *  segment h is bit 7 (128, 0x80)
 *  segment i (center of display) is not discovered yet
 *
 */
unsigned short num2seg_7seg[] =
{
	0xc0,  // 0
	0xf9,  // 1
	0xa4,  // 2
	0xb0,  // 3
	0x99,  // 4
	0x92,  // 5
	0x82,  // 6
	0xf8,  // 7
	0x80,  // 8
	0x90   // 9
};

unsigned short Char2seg_7seg[] =
{
	0x88,  // A
	0x83,  // B
	0xc6,  // C
	0xa1,  // D
	0x86,  // E
	0x8e,  // F
	0xc2,  // G
	0x89,  // H
	0xf9,  // I
	0xe1,  // J
	0x85,  // K
	0xc7,  // L
	0xaa,  // M
	0xc8,  // N
	0xc0,  // O
	0x8c,  // P
	0x98,  // Q
	0x88,  // R
	0x92,  // S
	0x87,  // T
	0xc1,  // U
	0xc1,  // V
	0x81,  // W
	0x89,  // X
	0x99,  // Y
	0xa4   // Z
};

unsigned short LowerChar2seg_7seg[] =
{
	0xa0,  // a
	0x83,  // b
	0xa7,  // c
	0xa1,  // d
	0x84,  // e
	0x8e,  // f
	0x90,  // g
	0x8b,  // h
	0xfb,  // i
	0xf1,  // j
	0x8b,  // k
	0xc7,  // l
	0xaa,  // m
	0xab,  // n
	0xa3,  // o
	0x8c,  // p
	0x98,  // q
	0xaf,  // r
	0x92,  // s
	0x87,  // t
	0xe3,  // u
	0xe3,  // v
	0xe3,  // w
	0x89,  // x
	0x91,  // y
	0xa4   // z
};

special_char_t special2seg_7seg[] =
{
	{ '!',	0x7e },
	{ '"',	0xdd },
	{ '#',	0xa3 },
	{ '$',	0x92 },
	{ '%',	0xad },
	{ '&',	0x82 },
	{ 0x27,	0xfd },  // single quote
	{ '(',	0xc6 },
	{ ')',	0xf0 },
	{ '*',	0x89 },
	{ '+',	0xb9 },
	{ ',',	0xf3 },
	{ '-',	0xbf },
	{ '.',	0x7f },
	{ '/',	0xad },
	{ ':',	0xef },
	{ ';',	0xf3 },
	{ '<',	0xa7 },
	{ '=',	0xb7 },
	{ '>',	0xb3 },
	{ '?',	0xac },
	{ '[',	0xc6 },
	{ 0x5c,	0x9b },  // backslash
	{ ']',	0xf0 },
	{ '^',	0xdc },
	{ '_',	0xf7 },
	{ '`',	0xdf },
	{ '{',	0xc6 },
	{ '|',	0xf0 },
	{ '}',	0xf0 },
	{ '~',	0xbf },
	{ 0x7f,	0xa3 },  // DEL
	{ ' ',	0xff }   // space (EOT)
};
#endif

#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500)
 // for 12 character dot matrix
/*****************************************************************************
 *
 * Icon definitions for 12dotmatrix
 *
 * Table contains five instances of one icon requiring the setting of
 * two segments in the display: ICON_TIMESHIFT, ICON_480i, ICON_480p
 * ICON_576i and ICON_576p. These are flagged by the 2nd msb value having
 * a value other than 0xff (the value for the second segment).
 */
struct iconToInternal micomIcons[] =
{
	/*- Name ---------- Number --------  msb   lsb   sgm  msb2  lsb2  sgm2 -----*/
	{ "ICON_MIN",       0xff,            0x00, 0x00, 0,   0xff, 0xff, 0 },  // dummy entry, 0
	{ "ICON_STANDBY",   ICON_STANDBY,    0x03, 0x00, 1,   0xff, 0xff, 1 },  // 01
	{ "ICON_SAT",       ICON_SAT,        0x02, 0x00, 1,   0xff, 0xff, 1 },  // 02
	{ "ICON_REC",       ICON_REC,        0x00, 0x00, 0,   0xff, 0xff, 0 },  // 03
	{ "ICON_TIMESHIFT", ICON_TIMESHIFT,  0x01, 0x01, 0,   0x01, 0x02, 0 },  // 04
	{ "ICON_TIMER",     ICON_TIMER,      0x01, 0x03, 0,   0xff, 0xff, 0 },  // 05
	{ "ICON_HD",        ICON_HD,         0x01, 0x04, 0,   0xff, 0xff, 0 },  // 06
	{ "ICON_USB",       ICON_USB,        0x01, 0x05, 0,   0xff, 0xff, 0 },  // 07
	{ "ICON_SCRAMBLED", ICON_SCRAMBLED,  0x01, 0x06, 0,   0xff, 0xff, 0 },  // 08, locked not scrambled
	{ "ICON_DOLBY",     ICON_DOLBY,      0x01, 0x07, 0,   0xff, 0xff, 0 },  // 09
	{ "ICON_MUTE",      ICON_MUTE,       0x01, 0x08, 0,   0xff, 0xff, 0 },  // 10
	{ "ICON_TUNER1",    ICON_TUNER1,     0x01, 0x09, 0,   0xff, 0xff, 0 },  // 11
	{ "ICON_TUNER2",    ICON_TUNER2,     0x01, 0x0a, 0,   0xff, 0xff, 0 },  // 12
	{ "ICON_MP3",       ICON_MP3,        0x01, 0x0b, 0,   0xff, 0xff, 0 },  // 13
	{ "ICON_REPEAT",    ICON_REPEAT,     0x01, 0x0c, 0,   0xff, 0xff, 0 },  // 14
	{ "ICON_PLAY",      ICON_PLAY,       0x00, 0x00, 1,   0xff, 0xff, 1 },  // 15, play symbol, handled separately
	{ "ICON_Circ0",     ICON_Circ0,      0x01, 0x04, 1,   0xff, 0xff, 1 },  // 16, center circle, handled separately
	{ "ICON_Circ1",     ICON_Circ1,      0x00, 0x00, 1,   0xff, 0xff, 1 },  // 17
	{ "ICON_Circ2",     ICON_Circ2,      0x00, 0x02, 1,   0xff, 0xff, 1 },  // 18
	{ "ICON_Circ3",     ICON_Circ3,      0x01, 0x01, 1,   0xff, 0xff, 1 },  // 19
	{ "ICON_Circ4",     ICON_Circ4,      0x01, 0x03, 1,   0xff, 0xff, 1 },  // 20
	{ "ICON_Circ5",     ICON_Circ5,      0x01, 0x04, 1,   0xff, 0xff, 1 },  // 21
	{ "ICON_Circ6",     ICON_Circ6,      0x01, 0x02, 1,   0xff, 0xff, 1 },  // 22
	{ "ICON_Circ7",     ICON_Circ7,      0x00, 0x03, 1,   0xff, 0xff, 1 },  // 23
	{ "ICON_Circ8",     ICON_Circ8,      0x00, 0x01, 1,   0xff, 0xff, 1 },  // 24
	{ "ICON_FILE",      ICON_FILE,       0x02, 0x02, 1,   0xff, 0xff, 1 },  // 25
	{ "ICON_TER",       ICON_TER,        0x02, 0x01, 1,   0xff, 0xff, 1 },  // 26
	{ "ICON_480i",      ICON_480i,       0x06, 0x04, 1,   0x06, 0x03, 1 },  // 27
	{ "ICON_480p",      ICON_480p,       0x06, 0x04, 1,   0x06, 0x02, 1 },  // 28
	{ "ICON_576i",      ICON_576i,       0x06, 0x01, 1,   0x06, 0x00, 1 },  // 29
	{ "ICON_576p",      ICON_576p,       0x06, 0x01, 1,   0x05, 0x04, 1 },  // 30
	{ "ICON_720p",      ICON_720p,       0x05, 0x03, 1,   0xff, 0xff, 1 },  // 31
	{ "ICON_1080i",     ICON_1080i,      0x05, 0x02, 1,   0xff, 0xff, 1 },  // 32
	{ "ICON_1080p",     ICON_1080p,      0x05, 0x01, 1,   0xff, 0xff, 1 },  // 33
//	{ "ICON_COLON1",    ICON_COLON1,     0xff, 0x06, 1,   0xff, 0xff, 1 },  // (34)  // TODO: find values
//	{ "ICON_COLON2",    ICON_COLON2,     0xff, 0x08, 1,   0xff, 0xff, 1 },  // (35)  // TODO: find values
//	{ "ICON_COLON3",    ICON_COLON3,     0xff, 0x0a, 1,   0xff, 0xff, 1 },  // (36) // TODO: find values
	{ "ICON_TV",        ICON_TV,         0x02, 0x03, 1,   0xff, 0xff, 1 },  // 34 (37)
	{ "ICON_RADIO",     ICON_RADIO,      0x02, 0x04, 1,   0xff, 0xff, 1 }   // 35 (38)
};
#endif

#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD)
// for 14 character dot matrix
struct iconToInternal micom_14seg_Icons[] =
{
	/*- Name------------ Number --------- msb   lsb   sgm -----*/
	{ "ICON_REC",        ICON_REC,        0x02, 0x00, 1 },
	{ "ICON_TIMER",      ICON_TIMER,      0x03, 0x00, 1 },
	{ "ICON_TIMESHIFT",  ICON_TIMESHIFT,  0x03, 0x01, 1 },
	{ "ICON_PLAY",       ICON_PLAY,       0x02, 0x01, 1 },
	{ "ICON_PAUSE",      ICON_PAUSE,      0x02, 0x02, 1 },
	{ "ICON_HD",         ICON_HD,         0x02, 0x04, 1 },
	{ "ICON_DOLBY",      ICON_DOLBY,      0x02, 0x03, 1 },
};
#endif
/* End of character and icon definitions */


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

	dprintk(100, "%s >\n", __func__);

	if (paramDebug > 149)
	{
		dprintk(1, "Command:");
		for (i = 0; i < len; i++)
		{
			printk(" 0x%02x", buffer[i] & 0xff);
		}
		printk("\n");
	}
	for (i = 0; i < len; i++)
	{
#if defined(DIRECT_ASC)
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
	dprintk(100, "%s < %d\n", __func__, 0);
	return 0;
}

/*******************************************************************
 *
 * Code for the IOCTL functions of the driver.
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
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
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
	lastdata.fan = (on ? 1 : 0);
	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}
#else
int micomSetFan(int on)
{
	dprintk(10, "%s Only supported on CubeRevo, other models have their fan always on.\n", __func__);
	return 0;
}
#endif

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

	dprintk(100, "%s > %d, %d\n", __func__, which, on);
	if (front_seg_num == 13 || front_seg_num == 4)  // no icons
	{
		dprintk(1, "%s: This model has no icons.\n", __func__);
		return res;
	}
#if !defined(CUBEREVO_250HD) \
 && !defined(CUBEREVO_MINI_FTA)
	on = (on ? 0x01 : 0x00);
	if (which < 1 || which >= ICON_MAX)
	{
		dprintk(1, "Icon number %d out of range (valid: 1-%d)\n", which, ICON_MAX - 1);
		return -EINVAL;
	}
	memset(buffer, 0, sizeof(buffer));
	lastdata.icon_state[which] = on;
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
	dprintk(50, "Setting icon number %s (%d) to %s\n", micomIcons[which].name, which, (on ? "on" : "off"));
	if (which == ICON_PLAY)
	{
		/* handle play symbol */
		buffer[0] = VFD_SETSEGMENTII;
		buffer[1] = on;
		buffer[2] = 0x00;
		buffer[3] = 0x01;
		res = micomWriteCommand(buffer, 5, 0);
	}
	else if (which == ICON_Circ0)
	{
		/* handle inner circle */
		buffer[0] = VFD_SETSEGMENTII;
		buffer[1] = on;
		buffer[2] = 0x04;
		buffer[3] = 0x00;
		res = micomWriteCommand(buffer, 5, 0);
	}
	else
	{
		for (vLoop = 0; vLoop < ARRAY_SIZE(micomIcons); vLoop++)
		{
			if ((which & 0xff) == micomIcons[vLoop].icon)
			{
				buffer[0] = VFD_SETSEGMENTI + micomIcons[vLoop].sgm;
				buffer[1] = on;
				buffer[2] = micomIcons[vLoop].codelsb;
				buffer[3] = micomIcons[vLoop].codemsb;
				res = micomWriteCommand(buffer, 5, 0);
	
				if (micomIcons[vLoop].codemsb2 != 0xff)  // icon is a two part icon
				{
					buffer[0] = VFD_SETSEGMENTI + micomIcons[vLoop].sgm2;
					buffer[1] = on;
					buffer[2] = micomIcons[vLoop].codelsb2;
					buffer[3] = micomIcons[vLoop].codemsb2;
					res = micomWriteCommand(buffer, 5, 0);
				}
				break;
			}
		}
	}
#endif
#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD)
	dprintk(50, "Setting icon number %s (%d) to %s\n", micom_14seg_Icons[which].name, which, (on ? "on" : "off"));
	for (vLoop = 0; vLoop < ARRAY_SIZE(micom_14seg_Icons); vLoop++)
	{
		if ((which & 0xff) == micom_14seg_Icons[vLoop].icon)
		{
			buffer[0] = VFD_SETSEGMENTI + micom_14seg_Icons[vLoop].sgm;
			buffer[1] = on;
			buffer[2] = micom_14seg_Icons[vLoop].codelsb;
			buffer[3] = micom_14seg_Icons[vLoop].codemsb;
			res = micomWriteCommand(buffer, 5, 0);
			break;
		}
	}
#endif
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

/*******************************************************************
 *
 * Routine to restore previous text display state. Restored items
 * are:
 * - Display text;
 * - Switched on icons;
 * - Spinner state.
 *
 * The routine does not restore the previous brightness, as the
 * front controller does not change this with display time on/off
 * or display on/off.
 */
int restoreDisplay(void)
{
	int res = 0;
	int i;
	
	// restore display text
	if (currentDisplayTime == 0)
	{
		lastdata.data[lastdata.length] = 0;  // terminate last displayed string
		res |= micomWriteString(lastdata.data, lastdata.length, 0);
	}

#if !defined(CUBEREVO_250HD) \
 && !defined(CUBEREVO_MINI_FTA)
	for (i = 1; i < ICON_MAX; i++)
	{
		if (lastdata.icon_state[i] != 0)
		{
			res |= micomSetIcon(i, 1);
		}
	}
#endif

#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
	// restore spinner
	if (front_seg_num == 12 && lastdata.icon_state[ICON_SPINNER] != 0)
	{
		micomSetIcon(ICON_Circ0, 1);  // restore centre circle
		spinner_state.state = 1;
		up(&spinner_state.sem);
	}
#endif
	return res;
}

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
	res |= micomWriteCommand(buffer, 5, 0);

	currentDisplayTime = on;

	/* If switched off, restore display text */
#if 0
	res |= restoreDisplay();
#else
	// TODO: restore icons and spinner
	if (on == 0)
	{
		lastdata.data[lastdata.length] = 0;  // terminate last displayed string
		res |= micomWriteString(lastdata.data, lastdata.length, 0);
	}
#endif
	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}

// ! Still there for historic reasons, CubeRevos do not have an RF modulator
// # if 0
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
// #endif

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
		dprintk(1, "Brightness out of range %d\n", level);
		return -EINVAL;
	}
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = VFD_SETBRIGHTNESS;
	buffer[1] = level & 0x07;
	res = micomWriteCommand(buffer, 5, 0);

	dprintk(100, "%s < %d\n", __func__, res);
	return res;
}
/* export for later use in e2_proc */
EXPORT_SYMBOL(micomSetBrightness);

/*************************************************************
 *
 * micomSetDisplayOnOff: switch entire display on or off.
 *
 * Note: does not work correctly with time display active;
 *       this is a limitation imposed by the frontprocessor
 *       which updates the time display once each second,
 *       regardless of the display being on or off.
 *
 */
int micomSetDisplayOnOff(char on)
{
	unsigned char buffer[5];
	int res = 0;
	int i;

	on = (on ? 1 : 0);
	lastdata.display_on = on;

	if (on == 0)
	{
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
		if (spinner_state.state)
		{
			lastdata.icon_state[ICON_SPINNER] = 1;
			spinner_state.state = 0;
		}
#endif
		res |= micomClearIcons();

		/* clear text */
		memset(buffer, 0, sizeof(buffer));
		buffer[0] = VFD_SETCLEARTEXT;
		res = micomWriteCommand(buffer, 5, 0);

		memset(buffer, 0, sizeof(buffer));
		buffer[0] = VFD_SETDISPLAYTEXT;
		res = micomWriteCommand(buffer, 5, 0);
	}
	else
	{
		res |= restoreDisplay();
	}
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
		dprintk(1, "Clear wakeup date\n");
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
	dprintk(10, "Set wakeup date to %02x-%02x-20%02x\n", buffer[3], buffer[2], buffer[1]);

	memset(buffer, 0, sizeof(buffer));
	if (time[0] == '\0')
	{
		dprintk(1, "Clear wakeup date\n");
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
	dprintk(10, "Set wakeup time to %02x:%02x; action is switch %s\n", buffer[1], buffer[2], ((buffer[3] == 1) ? "on" : "off" ));
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

//	res = micomWriteString("Bye bye ...", strlen("Bye bye ..."), 0);

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

#if 0
	if (front_seg_num != 4)
	{
		res = micomWriteString("Reboot", strlen("Reboot"), 0);
	}
	else
	{
		res = micomWriteString("rebt", strlen("rebt"), 0);
	}
#endif  // 0
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
 * NOTE: 250HD and mini FTA models seem to be not
 *       aware of the date, as they always return
 *       the same value after reading back the date.
 *
 */
int micomSetTime(char *time)  // expected format: YYMMDDhhmmss
{
	unsigned char buffer[5];
	int res = 0;

	dprintk(100, "%s >\n", __func__);

	/* set time */
	memset(buffer, 0, sizeof(buffer));

	dprintk(10, "Date to set: %c%c-%c%c-20%c%c\n", time[4], time[5], time[2], time[3], time[0], time[1]);
	dprintk(10, "Time to set: %c%c:%c%c:%c%c\n", time[6], time[7], time[8], time[9], time[10], time[11]);

#if !defined(CUBEREVO_250HD) \
 && !defined(CUBEREVO_MINI_FTA)
	buffer[0] = VFD_SETDATETIME;
	buffer[1] = ((time[0] - '0') << 4) | (time[1] - '0');  // year
	buffer[2] = ((time[2] - '0') << 4) | (time[3] - '0');  // month
	buffer[3] = ((time[4] - '0') << 4) | (time[5] - '0');  // day
	res = micomWriteCommand(buffer, 5, 0);
#endif
	memset(buffer, 0, sizeof(buffer));

	buffer[0] = VFD_SETTIME;
	buffer[1] = ((time[6] - '0') << 4) | (time[7] - '0');  // hour
	buffer[2] = ((time[8] - '0') << 4) | (time[9] - '0');  // minute
	buffer[3] = ((time[10] - '0') << 4) | (time[11] - '0');  // second
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
 *
 * NOTE: 250HD and mini FTA models seem to be not
 *       aware of the date, as their frontprocessor
 *       always returns the same values after
 *       reading back the date.
 *       These models always return 01-01-00 as the
 *       date.
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
		memset(ioctl_data, 0, sizeof(ioctl_data));
		dprintk(1, "%s: Error\n", __func__);
		res = -ETIMEDOUT;
	}
	else
	{
		dprintk(20, "Time received\n");
#if !defined(CUBEREVO_250HD) \
 && !defined(CUBEREVO_MINI_FTA)
		dprintk(10, "FP/RTC time: %02x:%02x:%02x\n", (int)ioctl_data[2], (int)ioctl_data[1], (int)ioctl_data[0]);
		dprintk(10, "FP/RTC date: %02x-%02x-20%02x\n", (int)ioctl_data[3], (int)ioctl_data[4], (int)ioctl_data[5]);
#else
		dprintk(10, "FP/RTC time: %02x:%02x:%02x\n", (int)ioctl_data[2], (int)ioctl_data[1], (int)ioctl_data[0]);
		ioctl_data[3] = 1;  // day
		ioctl_data[4] = 1;  // month
		ioctl_data[5] = 0;  // year (no century)
		dprintk(1, "Caution: date returned = 01-01-2000\n");
#endif
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
		memset(ioctl_data, 0, sizeof(ioctl_data));
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
 * micomGetVersion: get version of front processor.
 *
 * Note: leaves version number in variables micom_ver,
 *       micom_major and micom_minor.
 *
 */
int micomGetVersion(void)
{
	unsigned char buffer[5];
	int res = -1;

	dprintk(100, "%s >\n", __func__);

#if defined(CUBEREVO_3000HD) \
 || defined(CUBEREVO_2000HD)
	micom_ver   = 8;
	micom_major = 4;
	micom_minor = 0;
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
		sscanf(convertDate, "%d %d %d", &micom_ver, &micom_major, &micom_minor);
	}
#endif

	dprintk(20, "Frontpanel SW version: %d.%02d.%02d\n", micom_ver, micom_major, micom_minor);

#if defined(CUBEREVO_MINI_FTA) \
 || defined(CUBEREVO_250HD)
	front_seg_num    = 4;
	num2seg          = num2seg_7seg;
	Char2seg         = Char2seg_7seg;
	LowerChar2seg    = LowerChar2seg_7seg;
	special2seg      = special2seg_7seg;
	special2seg_size = ARRAY_SIZE(special2seg_7seg);
#elif defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD)
	front_seg_num    = 14;
	num2seg          = num2seg_pt6302;
	Char2seg         = Char2seg_pt6302;
	LowerChar2seg    = LowerChar2seg_pt6302;
	special2seg      = special2seg_pt6302;
	special2seg_size = ARRAY_SIZE(special2seg_pt6302);
#elif defined(CUBEREVO_9500HD)
	if ((micom_ver == 8) && (micom_major == 4 || micom_major == 6))  // TODO: check values
	{  // early ?
		front_seg_num    = 13;
		num2seg          = num2seg_13grid;
		Char2seg         = Char2seg_13grid;
		LowerChar2seg    = LowerChar2seg_13grid;
		special2seg      = special2seg_13grid;
		special2seg_size = ARRAY_SIZE(special2seg_13grid);
	}
	else if ((micom_ver == 8) && (micom_major == 3))  // TODO: check values
	{  // late ?
		front_seg_num    = 12;
		num2seg          = num2seg_pt6302;
		Char2seg         = Char2seg_pt6302;
		LowerChar2seg    = LowerChar2seg_pt6302;
		special2seg      = special2seg_pt6302;
		special2seg_size = ARRAY_SIZE(special2seg_pt6302);
//	}
#else  // CUBEREVO
	if ((micom_ver == 7) && (micom_major == 7 || micom_major == 6))
	{  // early, no icons
		front_seg_num    = 13;
		num2seg          = num2seg_13grid;
		Char2seg         = Char2seg_13grid;
		LowerChar2seg    = LowerChar2seg_13grid;
		special2seg      = special2seg_13grid;
		special2seg_size = ARRAY_SIZE(special2seg_13grid);
	}
	else if ((micom_ver == 8) && (micom_major == 3))
	{ // late, version 2
		front_seg_num    = 12;
		num2seg          = num2seg_pt6302;
		Char2seg         = Char2seg_pt6302;
		LowerChar2seg    = LowerChar2seg_pt6302;
		special2seg      = special2seg_pt6302;
		special2seg_size = ARRAY_SIZE(special2seg_pt6302);
	}
#endif
	dprintk(100, "%s < %d (front panel width = %d)\n", __func__, res, front_seg_num);
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
		dprintk(1, "%s Error\n", __func__);
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
 * Apart from the text to show, the routine requires a flag to
 * indicate whether or not the text should be shown centered
 * on the front panel display. This is used when showing texts
 * using /dev/vfd: the scrolling text is always shown left
 * aligned, the final text will be centered depending on the
 * #define CENTERED_DISPLAY in cuberevo_micon.h.
 * This code was added as some Neutrino versions always center
 * the texts they send, even if they are longer than the
 * display width. This resulted in a strange 'dancing around
 * the center' behaviour during the final scroll stages.
 *
 */
#if 0  // not used anywhere
inline char toupper(const char c)
{
	if ((c >= 'a') && (c <= 'z'))
	{
		return (c - 0x20);
	}
	return c;
}
#endif

inline int trimTrailingBlanks(char *txt, int len)
{
	int i;

	for (i = len - 1; (i > 0 && txt[i] == ' '); i--, len--)
	{
		txt[i] = '\0';
	}
	if (len > front_seg_num)
	{
		len = front_seg_num;
	}
	return len;
}

int micomWriteString(unsigned char *aBuf, int len, int center_flag)
{
	unsigned char buffer[5];
	unsigned char bBuf[128];
	int           res = 0, i, j;
	int           pos = 0;
	unsigned char space;
	unsigned char *UTF8_C2_table;

	aBuf[len] = '\0';  // terminate string
	dprintk(100, "%s > String: [%s] (len = %d)\n", __func__, aBuf, len);

#if !defined(CUBEREVO_250HD) \
 && !defined(CUBEREVO_MINI_FTA)
	if (front_seg_num == 12 || front_seg_num == 14)
	{
		UTF8_C2_table = UTF8_C2_mini;
	}
	else
	{
		UTF8_C2_table = UTF8_C2;
	}
#else
	UTF8_C2_table = UTF8_C2;
#endif
	/* The front processors cannot display accented letters.
	 * The following code traces for UTF8 sequences for these and
	 * replaces them with the corresponding letter without any accent;
	 * On 12 and 14 character models some UTF-8 encoded characters are
	 * displayed correctly, as far as the character generator in the
	 * PT6302-003 used in the frontpanel provides them.
	 * All this is not perfect, but at least better than the old practice
	 * of replacing them with spaces.
	 */
	dprintk(50, "%s UTF-8 text: [%s], len = %d\n", __func__, aBuf, len);
	memset(bBuf, ' ', sizeof(bBuf));
	j = 0;

	// process aBuf byte by byte
	for (i = 0; i < len; i++)
	{
//		if (front_seg_num == 12 || front_seg_num == 14)
//		{
//			if (aBuf[i] == 0x5c)  // handle backslash
//			{
//				bBuf[j] = 0xa0;
//				j++;
//			}
//		}
//		else if (aBuf[i] < 0x80)
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
					UTF_Char_Table = UTF8_C2_table;
					break;
				}
				case 0xc3:
				{
					UTF_Char_Table = UTF8_C3;
					break;
				}
				case 0xc4:
				{
					UTF_Char_Table = UTF8_C4;
					break;
				}
				case 0xc5:
				{
					UTF_Char_Table = UTF8_C5;
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
					j++;
				}
				else
				{
					dprintk(1, "%s UTF-8 character is unprintable, ignore.\n", __func__);
					i++;  // skip character
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
	dprintk(50, "%s Non-UTF-8 text: [%s], len = %d\n", __func__, bBuf, len);

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
	if (center_flag)
	{
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
	dprintk(50, "Final text: [%s] (len = %d)\n", bBuf, len);

	// save final text
	memcpy(lastdata.data, bBuf, len);
	lastdata.length = len;

	if (currentDisplayTime == 1)
	{
		dprintk(10, "%s: Display in time mode -> ignoring display text\n", __func__);
		return 0;
	}
	for (i = 0; i < len; i++)
	{
		unsigned short data;
		unsigned char  ch;

		memset(buffer, 0, sizeof(buffer));

#if defined(CUBEREVO_MINI_FTA) \
 || defined(CUBEREVO_250HD)
	if (i == 2 && bBuf[2] == ':')
	{
		// colon found on position 2
		dprintk(50, "Colon on position 2 found\n");
		buffer[0] = VFD_SETCHAR;
		buffer[1] = 4;  /* position */
		buffer[2] = 0xff;
		buffer[3] = 0;
		res = micomWriteCommand(buffer, 5, 0);

		// remove colon from text
		len --;
		memcpy(bBuf + 2, bBuf + 3, len);
		bBuf[len] = '\0';
		dprintk(1, "New text: [%s] (len=%d)\n", bBuf, len);
	}
#endif
		buffer[0] = VFD_SETCHAR;
		buffer[1] = i & 0xff;  /* position */

		if (front_seg_num == 14)
		{
			data = bBuf[i] - 0x10;  // get character from input and convert to display value
		}
		else
		{
			ch = bBuf[i];  // get character from input

			switch (ch)
			{
				case 'A' ... 'Z':  // if uppercase letter
				{
					data = Char2seg[ch - 'A'];  // and get character from table
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
						dprintk(1, "%s Unprintable character [0x%02x] ignored\n", __func__, ch);
						data = space;  // and print a space
					}
					break;
				}
			}
		}
		dprintk(150, "%s data 0x%x \n", __func__, data);

		buffer[2] = data & 0xff;
		buffer[3] = (data >> 8) & 0xff;
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

	// blank display
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = VFD_SETCLEARTEXT;
	res = micomWriteCommand(buffer, 5, 0);
	micomGetVersion();
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
	res |= micomSetFan(1);
#endif
#if !defined(CUBEREVO_MINI_FTA) \
 && !defined(CUBEREVO_250HD) 
	res |= micomSetLED(3);  // LED on and slow blink mode
	res |= micomSetBrightness(5);
#endif
	res |= micomSetTimeMode(1);  // 24h mode
	res |= micomSetDisplayTime(0);  // mode = display text
//	res |= micomWriteString("T.Ducktales", strlen("T.Ducktales"), 0);

	/* disable all icons at startup */
#if defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD) \
 || defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
	for (vLoop = ICON_MIN + 1; vLoop < ICON_MAX - 1; vLoop++)
	{
		micomSetIcon(vLoop, 0);
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
	res = micomWriteString(bBuf, front_seg_num, 0);
	dprintk(100, "%s <\n", __func__);
}

//#define TEST_COMMANDS
//#define TEST_CHARSET
#if !defined(TEST_CHARSET) \
 && !defined(TEST_COMMANDS)
/******************************************
 *
 * Write to /dev/vfd (not-test)
 *
 */
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
#if defined(CENTERED_DISPLAY)
		res = micomWriteString(kernel_buf, llen, 1);
#else
		res = micomWriteString(kernel_buf, llen, 0);
#endif
	}
	else  // scroll, display string is longer than display length
	{
		memset(buf, ' ', sizeof(buf));

		// initial display starting at 3rd position to ease reading
		offset = 3;
		memcpy(buf + offset, kernel_buf, llen);
		llen += offset;
		buf[llen + front_seg_num - 1] = '\0';  // terminate string

		for (pos = 1; pos < llen; pos++)
		{
			res |= micomWriteString(buf + pos, llen + front_seg_num, 0);
			// sleep 300 ms
			msleep(300);
		}
		// final display
		clear_display();
#if defined(CENTERED_DISPLAY)
		res |= micomWriteString(kernel_buf, front_seg_num, 1);
#else
		res |= micomWriteString(kernel_buf, front_seg_num, 0);
#endif
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
		dprintk(1, "Error Bad Minor\n");
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
		dprintk(1, "%s < return erestartsys\n", __func__);
		return -ERESTARTSYS;
	}
	if (FrontPanelOpen[minor].read == lastdata.length)
	{
		FrontPanelOpen[minor].read = 0;
		up(&FrontPanelOpen[minor].sem);
		dprintk(100, "%s < [0]\n", __func__);
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
	dprintk(100, "%s < (len %d)\n", __func__, len);
	return len;
}

int MICOMdev_open(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(100, "%s >\n", __func__);

	/* needed! otherwise a race condition can occur */
	if (down_interruptible(&write_sem))
	{
		return -ERESTARTSYS;
	}
	minor = MINOR(inode->i_rdev);
	dprintk(70, "open minor %d\n", minor);

	if (FrontPanelOpen[minor].fp != NULL)
	{
		dprintk(1, "EUSER\n");
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
		dprintk(1, "%s: < -EUSER\n", __func__);
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
	struct vfd_ioctl_data *data = (struct vfd_ioctl_data *)arg;
	int res = 0;
	int i;
	int icon_nr;
	int on;

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
#if !defined(CUBEREVO_250HD) \
 && !defined(CUBEREVO_MINI_FTA)
			if (micom_ver > 7)  // 12dotmatrix and 14grid only
			{
				icon_nr = mode == 0 ? data->data[0] : micom->u.icon.icon_nr;
				on = mode == 0 ? data->data[4] : micom->u.icon.on;
				dprintk(10, "%s Set icon %d to %d (mode %d)\n", __func__, icon_nr, on, mode);
				on = on != 0 ? 1 : 0;

				// Part one: translate E2 icon numbers to own icon numbers (vfd mode only)
				if (mode == 0)  // vfd mode
				{
					switch (icon_nr)
					{
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
						case 0x13:  // crypted
						{
							icon_nr = ICON_SCRAMBLED;
							break;
						}
						case 0x15:  // MP3
						{
							icon_nr = ICON_MP3;
							break;
						}
#endif
						case 0x11:  // HD
						{
							icon_nr = ICON_HD;
							break;
						}
						case 0x17:  // dolby
						{
							icon_nr = ICON_DOLBY;
							break;
						}
						case 0x1a:  // seekable (play)
						{
							icon_nr = ICON_PLAY;
							break;
						}
						case 0x1e:  // record
						{
							icon_nr = ICON_REC;
							break;
						}
						default:
						{
							break;
						}
					}  // end switch
				}  // mode 0

				// Part two: decide wether one icon, all or spinner
				switch (icon_nr)
				{
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
					case ICON_SPINNER:
					{
						spinner_state.state = on;
						lastdata.icon_state[ICON_SPINNER] = on;

						if (on)
						{
							if (on == 1)  // handle default value
							{
								on = 100;  // set default value: 1 change/sec
							}
							spinner_state.period = on * 10;
							up(&spinner_state.sem);
						}
						res = 0;
						break;
					}
#endif
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD) \
 || defined(CUBEREVO_MINI) \
 || defined(CUBEREVO_MINI2) \
 || defined(CUBEREVO_2000HD) \
 || defined(CUBEREVO_3000HD)
					case ICON_MAX:
					{
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
						if (spinner_state.state == 1)  // switch spinner off if on
						{
							dprintk(50, "%s Stop spinner\n", __func__);
							spinner_state.state = 0;
							do
							{
								msleep(250);
							}
							while (spinner_state.status != SPINNER_THREAD_STATUS_HALTED);
							dprintk(50, "%s Spinner stopped\n", __func__);
						}
						//fall through to:
#endif
						for (i = ICON_MIN + 1; i < ICON_MAX; i++)
						{
							res |= micomSetIcon(i, on);
							msleep(1);  // allow the fp some time
						}
						break;
					}
#endif
					default:  // (re)set a single icon
					{
#if defined(CUBEREVO) \
 || defined(CUBEREVO_9500HD)
						if (spinner_state.state == 1 && icon_nr >= ICON_Circ0 && icon_nr <= ICON_Circ8)
						{
							dprintk(50, "%s Stop spinner\n", __func__);
							spinner_state.state = 0;
							do
							{
								msleep(250);
							}
							while (spinner_state.status != SPINNER_THREAD_STATUS_HALTED);
							dprintk(50, "%s Spinner stopped\n", __func__);
							msleep(100);
						}
#endif
						dprintk(50, "%s Set single icon #%d to %d\n", __func__, icon_nr, on);
						res = micomSetIcon(icon_nr, on);
						break;
					}
				}
			}
			else
			{
				res = 0;
			}
#else
			res = 0;
#endif  // !250_HD...

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
// ! Still there for historic reasons, CubeRevos do not have an RF modulator
// #if 0
		case VFDSETRF:
		{
			res = micomSetRF(micom->u.rf.on);
			break;
		}
// #endif
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
//#if defined(CENTERED_DISPLAY)
//				res = micomWriteString(data->data, data->length, 1);
//#else
				res = micomWriteString(data->data, data->length, 0);
//#endif
			}
			else
			{
				// not supported
			}
			mode = 0;
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			char buffer[5];
			struct vfd_ioctl_data *data = (struct vfd_ioctl_data *) arg;
			int on = data->start;

			res = micomSetDisplayOnOff(on);
			mode = 0;
			break;
		}
		case VFDCLEARICONS:
		{
			res = 0;
			if (micom_ver > 7)
			{
				res |= micomClearIcons();
			}
			break;
		}
		case VFDGETVERSION:  // currently return most significant version number only
		{
//			dprintk(20, "Front panel SW version: %d.%02d.%02d\n", micom_ver, micom_major, micom_minor);
			dprintk(20, "Front panel SW version: %d.%02d\n", micom_ver, micom_major);
			micom->u.version.version = (micom_ver * 100) + micom_major;
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
		case 0x5305:  // Neutrino sends this
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
