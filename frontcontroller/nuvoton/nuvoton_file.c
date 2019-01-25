/*
 * nuvoton_file.c
 *
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add irq handling
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
 * Changes
 *
 * Date     By              Description
 * ----------------------------------------------------------------------------
 * 20131001 Audioniek       Added code for all icons on/off.
 * 20131004 Audioniek       Get wake up reason added.
 * 20131008 Audioniek       Added Octagon1008 (HS9510) missing icons.
 * 20131008 Audioniek       SetLED command, setPwrLED is now obsolete as it
 *                          is a subset of SetLED.
 * 20131008 Audioniek       Beginning of Octagon1008 lower case characters.
 * 20131015 Audioniek       VFDDISPLAYWRITEONOFF made functional on HDBOX.
 * 20131026 Audioniek       Octagon1008 lower case characters completed.
 * 20131126 Audioniek       Start of adding text scrolling.
 * 20131128 Audioniek       Text scroll on /dev/vfd working: text scrolls
 *                          once if text is longer than display size,
 * 20131224 Audioniek       except on HS7119, HS7810A & HS7819.
 * 20131205 Audioniek       Errors and doubles corrected in HDBOX icon table.
 * 20131220 Audioniek       Start of work on ATEVIO7500 (Fortis HS8200).
 * 20131224 Audioniek       nuvotonWriteString on HS7119/HS7810A/HS7819
 *                          completely rewritten including special handling
 *                          of periods and colons.
 * 20140210 Audioniek       Start of work on HS7119 & HS7819.
 * 20140403 Audioniek       SetLED for ATEVIO7500 added.
 * 20140415 Audioniek       ATEVIO7500 finished; yet to do: icons.
 * 20140426 Audioniek       HS7119/7810A/7819 work completed.
 * 20140916 Audioniek       Fixed compiler warnings with HS7119/7810A/7819.
 * 20140921 Audioniek       No special handling of periods and colons for
 *                          HS7119: receiver has not got these segments.
 * 20141010 Audioniek       Special handling of colons for HS7119 added:
 *                          DOES have a colon, but no periods.
 * 20141010 Audioniek       HS7119/HS7810A/HS7819 scroll texts longer than
 *                          4 once.
 * 20151231 Audioniek       HS7420/HS7429 added.
 * 20160103 Audioniek       Compiler warning fixed, references to ATEMIO530
 *                          removed.
 * 20160424 Audioniek       VFDGETVERSION added.
 * 20160522 Audioniek       VFDSETPOWERONTIME and VFDGETWAKEUPTIME added.
 * 20160523 Audioniek       procfs added.
 * 20161006 Audioniek       Logo brightness for Octagon1008 added.
 * 20170111 Audioniek       VFDTEST added, conditionally compiled.
 * 20170112 Audioniek       FS9000/9200 use same nuvotonWriteString as HS8200,
 *                          thus have UTF8 support now.
 * 20170113 Audioniek       Version for FS9000/9200 with all icons working on
 *                          both old and new front processors.
 * 20170115 Audioniek       Icon support for HS8200 added.
 * 20170117 Audioniek       Icon support for HS742X added.
 * 20170119 Audioniek       Colon on HS7119/7810A/7819 can also be controlled
 *                          as an icon; periods on HS7810A/7819 as well.
 * 20170120 Audioniek       Spinner for FS9000/9200 added, based on aotom
 *                          code.
 * 20170121 Audioniek       Brightness control for LED models added.
 * 20170123 Audioniek       nuvotonSetDisplayOnOff on FS9000/9200 fixed.
 * 20170123 Audioniek       Typos and small errors removed.
 * 20170125 Audioniek       nuvotonSetDisplayOnOff on HS8200 fixed.
 * 20170126 Audioniek       nuvotonSetDisplayOnOff on HS7119/7420/7429/
 *                          7810A/7819 fixed.
 * 20170128 Audioniek       nuvotonGetVersion now returns actual bootloader
 *                          version instead of string constant.
 * 20170128 Audioniek       Spinner for HS8200 added.
 * 20170202 Audioniek       Multiple icons now displayed in a thread on
 *                          HS8200; spinner code simplified.
 * 20170204 Audioniek       Spinner switched off when using (re)set all icons;
 *                          on FS9000/9200 this is also done when one of the
 *                          circle icons is (re)set.
 * 20170205 Audioniek       nuvotonGetVersion reports resellerID as well.
 * 20170417 Audioniek       New nuvoton module parameter: gmt_offset.
 * 20170418 Audioniek       Init: bug fixed with boot_on string length.
 * 20170419 Audioniek       nuvotonWriteString for HS7810A/7819 rewritten.
 * 20170516 Audioniek       Missing character entries for : and ; added for
 *                          HS9510/7420/7429.
 * 20180113 Audioniek       Fix build error with HS7110.
 *
 *****************************************************************************
 *
 * Note on model designations: Three Fortis models have historically been
 * indicated by a reseller model number instead of the generic internal Fortis
 * model designation:
 *
 * FORTIS_HDBOX: Strictly speaking the FS9000. The chosen name is confusing,
 *               as HDBOX is one of the resellers selling Fortis receivers.
 *               In Germany the FS9000 was sold as Octagon SF1018 and
 *               Atevio AV7000.
 *               The FS9200 is completely hardware compatible with the FS9000
 *               and does not have a separate designation. In Germany this
 *               model was sold as Astro ASR1200.
 * OCTAGON1008:  The HS9510 and its variants. In Germany sold as
 *               Octagon SF1008P, Octagon SF1008SE, Atevio AV700 and
 *               Astro ASR1100.
 * ATEVIO7500:   The HS8200. In Germany sold as Octagon SF1028, Atevio AV7500,
 *               Atemio AV7600 and Opticum Actus Duo.
 * All other Fortis models are designated by the internal Fortis model
 * number that can be found on the component side of the main board, usually
 * near the front panel.
 */


#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/termbits.h>
#include <linux/kthread.h>
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
#include <linux/mtd/mtd.h>
#include <linux/err.h>

#include "nuvoton.h"
#include "nuvoton_asc.h"
#include "nuvoton_utf.h"

#if defined(FORTIS_HDBOX)
tIconState spinner_state;
#elif defined(ATEVIO7500)
tIconState spinner_state;
tIconState icon_state;
#endif

extern void ack_sem_up(void);
extern int  ack_sem_down(void);
int nuvotonWriteString(unsigned char *aBuf, int len);
extern void nuvoton_putc(unsigned char data);

struct semaphore write_sem;
int errorOccured = 0;
extern int dataflag;
static char ioctl_data[9];
static char wakeup_time[5];

tFrontPanelOpen FrontPanelOpen [LASTMINOR];
struct saved_data_s lastdata;
u8 regs[0x100];  // array with copy values of FP registers

/***************************************************************************
 *
 * Character definitions.
 *
 ***************************************************************************/

#if defined(OCTAGON1008) || defined(HS7420) || defined(HS7429)
/***************************************************************************
 *
 * Characters for HS9510/7420/7429 (8 character VFD models)
 *
 * Note on adding HS742X: The front panel controller is almost
 * equal to the one on the Octagon1008 (the board it is built
 * on says HS-9510HD FRONT B-Type) but has a different VFD tube
 * mounted on it.
 * The VFD has eight 15 segment characters and four icons: a red
 * dot and three colons between the main characters.
 * Stock firmware does not control the icons; the red dot is used
 * as remote feedback, a function internal to the front processor.
 *
 * Dagobert: On octagon 1008 the "normal" fp setting does not work.
 * it seems that with this command we set the segment elements
 * direct (not sure here). I dont know how this works but ...
 *
 * Added by Audioniek:
 * Command format for set icon (Octagon1008 and HS742X only):
 * SOP 0xc4 byte_pos byte_1 byte_2 byte_3 EOP
 * byte pos is 0..8 (0=rightmost, 8 is collection of icons on left; HS9510 only)
 *
 * Character layout for positions 0..7:
 *      icon
 *    aaaaaaaaa
 *   fj   i   hb
 *   f j  i  h b
 * p f  j i h  b
 *   f   jih   b
 *    llllgkkkk
 *   e   onm   c
 * p e  o n m  c
 *   e o  n  m c
 *   eo   n   mc
 *    ddddddddd
 *
 * bit = 1 -> segment on
 * bit = 0 -> segment off
 *
 * icon on top is byte_1, bit 0
 *
 * segment l is byte_2, bit 0 (  1)
 * segment c is byte_2, bit 1 (  2)
 * segment e is byte_2, bit 2 (  4)
 * segment m is byte_2, bit 3 (  8)
 * segment n is byte_2, bit 4 ( 16)
 * segment o is byte_2, bit 5 ( 32)
 * segment d is byte_2, bit 6 ( 64)
 * segment p is byte_2, bit 7 (128) (colon, only on positions 1, 3 & 5)
 *
 * segment a is byte_3, bit 0 (  1)
 * segment h is byte_3, bit 1 (  2)
 * segment i is byte_3, bit 2 (  4)
 * segment j is byte_3, bit 3 (  8)
 * segment b is byte_3, bit 4 ( 16)
 * segment f is byte_3, bit 5 ( 32)
 * segment k is byte_3, bit 6 ( 64)
 * segment g is byte_3, bit 7 (128)
 *
 * top icons on positions 0 .. 7: (see below)
 * HS9510 (Octagon1008):
 * (0) ICON_DOLBY, ICON_DTS, ICON_VIDEO, ICON_AUDIO, ICON_LINK, ICON_HDD, ICON_DISC, ICON_DVB (7)
 * HS742X:
 * (0) none, none, ICON_COLON3, none, ICON_COLON2, none, ICON_COLON3, ICON_DOT (7)
 *
 * Position 8 (leftmost!) is a collection of icons (see below, Octagon1008 only)
 */
struct vfd_buffer vfdbuf[9];

struct chars
{
	u8 s1;
	u8 s2;
	u8 character;
} octagon_chars[] =
{
	{0x00, 0x00, ' '},
	{0x10, 0x84, '!'},  //0x21
	{0x00, 0x30, '"'},
	{0x00, 0x00, '#'},
	{0x43, 0xe1, '$'},
	{0x00, 0x00, '%'},
	{0x00, 0x00, '&'},
	{0x00, 0x02, 0x27},
	{0x08, 0x82, '('},
	{0x20, 0x88, ')'},
	{0x39, 0xce, '*'},
	{0x11, 0xc4, '+'},
	{0x21, 0x00, ','},
	{0x01, 0xc0, '-'},
	{0x04, 0x00, '.'},
	{0x20, 0x82, '/'},
	{0x46, 0x31, '0'}, //x030
	{0x02, 0x12, '1'},
	{0x60, 0xd1, '2'},
	{0x43, 0xd1, '3'},
	{0x11, 0xe4, '4'},
	{0x43, 0xe1, '5'},
	{0x47, 0xe1, '6'},
	{0x20, 0x83, '7'},
	{0x47, 0xf1, '8'},
	{0x43, 0xf1, '9'},
	{0x10, 0x04, ':'}, //x03A
	{0x20, 0x04, ';'},
	{0x08, 0x82, '<'},
	{0x41, 0xc0, '='},
	{0x20, 0x88, '>'},
	{0x10, 0x83, '?'},
	{0x45, 0xf1, '@'},
	{0x07, 0xf1, 'A'},
	{0x52, 0xd5, 'B'},
	{0x44, 0x21, 'C'},
	{0x52, 0x95, 'D'},
	{0x45, 0xe1, 'E'},
	{0x05, 0xe1, 'F'},
	{0x46, 0x61, 'G'},
	{0x07, 0xf0, 'H'},
	{0x10, 0x84, 'I'},
	{0x46, 0x10, 'J'},
	{0x0d, 0xA2, 'K'},
	{0x44, 0x20, 'L'},
	{0x06, 0xba, 'M'},
	{0x0e, 0xb8, 'N'},
	{0x46, 0x31, 'O'},
	{0x05, 0xf1, 'P'},
	{0x4e, 0x31, 'Q'},
	{0x0d, 0xf1, 'R'},
	{0x43, 0xe1, 'S'},
	{0x10, 0x85, 'T'},
	{0x46, 0x30, 'U'},
	{0x24, 0xa2, 'V'},
	{0x2e, 0xb0, 'W'},
	{0x28, 0x8a, 'X'},
	{0x10, 0x8a, 'Y'},
	{0x60, 0x83, 'Z'},
	{0x44, 0x21, '['},
	{0x08, 0x88, 0x5C},
	{0x42, 0x11, ']'},
	{0x00, 0x31, '^'},
	{0x40, 0x00, '_'},
	{0x00, 0x08, '`'},
	{0x62, 0xc0, 'a'},  // {0x62, 0xd1, 'a'},
	{0x4d, 0xa0, 'b'},
	{0x45, 0xc0, 'c'},  // {0x44, 0x21, 'c'},
	{0x62, 0xd0, 'd'},
	{0x45, 0xa3, 'e'},
	{0x05, 0xa1, 'f'},
	{0x42, 0xd9, 'g'},
	{0x07, 0xe0, 'h'},
	{0x10, 0x80, 'i'},  // {0x10, 0x84, 'i'},
	{0x42, 0x10, 'j'},  // {0x46, 0x10, 'j'},
	{0x18, 0xc4, 'k'},
	{0x10, 0x84, 'l'},
	{0x17, 0xc0, 'm'},  // {0x16, 0xb5, 'm'},
	{0x07, 0xc0, 'n'},  // {0x06, 0x31, 'n'},
	{0x47, 0xc0, 'o'},  // {0x46, 0x31, 'o'},
	{0x05, 0xa3, 'p'},
	{0x02, 0xd9, 'q'},
	{0x05, 0xc0, 'r'},  // {0x04, 0x21, 'r'},
	{0x43, 0xe1, 's'},
	{0x45, 0xa0, 't'},
	{0x46, 0x00, 'u'},  // {0x46, 0x30, 'u'},
	{0x24, 0x00, 'v'},  // {0x24, 0xa2, 'v'},
	{0x56, 0x00, 'w'},  // {0x56, 0xb0, 'w'},
	{0x28, 0x8a, 'x'},
	{0x42, 0xd8, 'y'},
	{0x60, 0x83, 'z'},
	{0x44, 0x21, '{'},
	{0x10, 0x84, '|'},
	{0x42, 0x11, '}'},
	{0x01, 0xc0, '~'},
	{0xff, 0xff, 0x7f}  //0x7f
};

#elif defined(HS7119) || defined(HS7810A) || defined(HS7819)
/***************************************************************************
 *
 * Characters for HS7119/7810A/7819 (LED models)
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
 *  segment a is bit 0 (  1)
 *  segment b is bit 1 (  2)
 *  segment c is bit 2 (  4)
 *  segment d is bit 3 (  8)
 *  segment e is bit 4 ( 16, 0x10)
 *  segment f is bit 5 ( 32, 0x20)
 *  segment g is bit 6 ( 64, 0x40)
 *  segment h is bit 7 (128, 0x80, positions 2, 3 & 4, cmd byte 2, 3 & 5, not on HS7119)
 *  segment i is bit 7 (128, 0x80, position 3 only, cmd byte 4)
 *  NOTE: period on 1st position cannot be controlled
 */

static int _7seg_fonts[] =
{
//	' '   '!'    '"'   '#'   '$'   '%'   '&'  '
	0x00, 0x06, 0x22, 0x5c, 0x6d, 0x52, 0x7d, 0x02,
//	'('   ')'   '*'   '+'   ','   '-'   '.'   '/'
	0x39, 0x0f, 0x76, 0x46, 0x0c, 0x40, 0x08, 0x52,
//	'0'   '1'   '2'   '3'   '4'   '5'   '6'   '7'
	0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
//	'8'   '9'   ':'   ';'   '<'   '='   '>'   '?'
	0x7f, 0x6f, 0x10, 0x0c, 0x58, 0x48, 0x4c, 0x53,
//	'@'   'A'   'B'   'C'   'D'   'E'   'F'   'G'
	0x7b, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x3d,
//	'H'   'I'   'J'   'K'   'L'   'M'   'N'   'O'
	0x76, 0x06, 0x1e, 0x7a, 0x38, 0x55, 0x37, 0x3f,
//	'P'   'Q'   'R'   'S'   'T'   'U'   'V'   'W'
	0x73, 0x67, 0x77, 0x6d, 0x78, 0x3e, 0x3e, 0x7e,
//	'X'   'Y'   'Z'   '['   '\'   ']'   '^'   '_'
	0x76, 0x66, 0x5b, 0x39, 0x64, 0x0f, 0x23, 0x08,
//	'`'   'a'   'b'   'c'   'd'   'e'   'f'   'g'
	0x20, 0x5f, 0x7c, 0x58, 0x5e, 0x7b, 0x71, 0x6f,
//	'h'   'i'   'j'   'k'   'l'   'm'   'n'  'o'
	0x74, 0x04, 0x0e, 0x78, 0x38, 0x55, 0x54, 0x5c,
//	'p'   'q'   'r'   's'   't'   'u'   'v'   'w'
	0x73, 0x67, 0x50, 0x6d, 0x78, 0x1c, 0x1c, 0x1c,
//	'x'   'y'   'z'   '{'   '|'   '}'   '~'   DEL
	0x76, 0x6e, 0x5b, 0x39, 0x06, 0x0f, 0x40, 0x5c
};
#endif  // Character definitions

/***************************************************************************
 *
 * Icon definitions.
 *
 ***************************************************************************/
#if defined(OCTAGON1008)
/***************************************************************************
 *
 * Icons for HS9510
 *
 * Top icons on positions 0 .. 7: (0=rightmost)
 * (0) ICON_DOLBY, ICON_DTS, ICON_VIDEO, ICON_AUDIO, ICON_LINK, ICON_HDD, ICON_DISC, ICON_DVB (7)
 *
 * Icons on position 8:
 * ICON_DVD     is byte_1, bit 0 (  1)
 *
 * ICON_TIMER   is byte_2, bit 0 (  1)
 * ICON_TIME    is byte_2, bit 1 (  2)
 * ICON_CARD    is byte_2, bit 2 (  4)
 * ICON_1       is byte_2, bit 3 (  8)
 * ICON_2       is byte_2, bit 4 ( 16)
 * ICON_KEY     is byte_2, bit 5 ( 32)
 * ICON_16_9    is byte_2, bit 6 ( 64)
 * ICON_USB     is byte_2, bit 7 (128)
 *
 * ICON_CRYPTED is byte_3, bit 0 (  1)
 * ICON_PLAY    is byte_3, bit 1 (  2)
 * ICON_REWIND  is byte_3, bit 2 (  4)
 * ICON_PAUSE   is byte_3, bit 3 (  8)
 * ICON_FF      is byte_3, bit 4 ( 16)
 * none         is byte_3, bit 5 ( 32)
 * ICON_REC     is byte_3, bit 6 ( 64)
 * ICON_ARROW   is byte_3, bit 7 (128) (RC feedback)
 */
struct iconToInternal
{
	char *name;
	u16 icon;
	u8 pos;    // display position (0=rightmost)
	u8 mask1;  // bitmask for on (byte 1)
	u8 mask2;  // bitmask for on (byte 2)
	u8 mask3;  // bitmask for on (byte 3)
} nuvotonIcons[] =
{
	/*- Name -------- icon -------- pos - mask1 mask2 mask3 -----*/
	{ "ICON_DOLBY"  , ICON_DOLBY  , 0x00, 0x01, 0x00, 0x00},  // 01
	{ "ICON_DTS"    , ICON_DTS    , 0x01, 0x01, 0x00, 0x00},  // 02
	{ "ICON_VIDEO"  , ICON_VIDEO  , 0x02, 0x01, 0x00, 0x00},  // 03
	{ "ICON_AUDIO"  , ICON_AUDIO  , 0x03, 0x01, 0x00, 0x00},  // 04
	{ "ICON_LINK"   , ICON_LINK   , 0x04, 0x01, 0x00, 0x00},  // 05
	{ "ICON_HDD"    , ICON_HDD    , 0x05, 0x01, 0x00, 0x00},  // 06
	{ "ICON_DISC"   , ICON_DISC   , 0x06, 0x01, 0x00, 0x00},  // 07
	{ "ICON_DVB"    , ICON_DVB    , 0x07, 0x01, 0x00, 0x00},  // 08
	{ "ICON_DVD"    , ICON_DVD    , 0x08, 0x01, 0x00, 0x00},  // 09
	{ "ICON_TIMER"  , ICON_TIMER  , 0x08, 0x00, 0x01, 0x00},  // 10
	{ "ICON_TIME"   , ICON_TIME   , 0x08, 0x00, 0x02, 0x00},  // 11
	{ "ICON_CARD"   , ICON_CARD   , 0x08, 0x00, 0x04, 0x00},  // 12
	{ "ICON_1"      , ICON_1      , 0x08, 0x00, 0x08, 0x00},  // 13
	{ "ICON_2"      , ICON_2      , 0x08, 0x00, 0x10, 0x00},  // 14
	{ "ICON_KEY"    , ICON_KEY    , 0x08, 0x00, 0x20, 0x00},  // 15
	{ "ICON_16_9"   , ICON_16_9   , 0x08, 0x00, 0x40, 0x00},  // 16
	{ "ICON_USB"    , ICON_USB    , 0x08, 0x00, 0x80, 0x00},  // 17
	{ "ICON_CRYPTED", ICON_CRYPTED, 0x08, 0x00, 0x00, 0x01},  // 18
	{ "ICON_PLAY"   , ICON_PLAY   , 0x08, 0x00, 0x00, 0x02},  // 19
	{ "ICON_REWIND" , ICON_REWIND , 0x08, 0x00, 0x00, 0x04},  // 20
	{ "ICON_PAUSE"  , ICON_PAUSE  , 0x08, 0x00, 0x00, 0x08},  // 21
	{ "ICON_FF"     , ICON_FF     , 0x08, 0x00, 0x00, 0x10},  // 22
	{ "ICON_NONE"   , ICON_NONE   , 0x08, 0x00, 0x00, 0x20},  // 23
	{ "ICON_REC"    , ICON_REC    , 0x08, 0x00, 0x00, 0x40},  // 24
	{ "ICON_ARROW"  , ICON_ARROW  , 0x08, 0x00, 0x00, 0x80},  // 25
	{ "ICON_COLON1" , ICON_COLON1 , 0x01, 0x00, 0x80, 0x00},  // 26
	{ "ICON_COLON2" , ICON_COLON2 , 0x03, 0x00, 0x80, 0x00},  // 27
	{ "ICON_COLON3" , ICON_COLON3 , 0x05, 0x00, 0x80, 0x00},  // 28
};

#elif defined(HS7420) || defined(HS7429)
/***************************************************************************
 *
 * Icons for HS742X
 *
 */
struct iconToInternal
{
	char *name;
	u16 icon;
	u8 pos;    // display position (0=rightmost)
	u8 mask1;  // bitmask for on (byte 1)
	u8 mask2;  // bitmask for on (byte 2)
	u8 mask3;  // bitmask for on (byte 3)
} nuvotonIcons[] =
{
	/*- Name ------- icon# ------ pos - mask1 mask2 mask3 -----*/
	{ "ICON_DOT"   , ICON_DOT   , 0x07, 0x00, 0x80, 0x00},  // 01
	{ "ICON_COLON1", ICON_COLON1, 0x06, 0x00, 0x80, 0x00},  // 02
	{ "ICON_COLON2", ICON_COLON2, 0x04, 0x00, 0x80, 0x00},  // 03
	{ "ICON_COLON3", ICON_COLON3, 0x02, 0x00, 0x80, 0x00},  // 04
};

#elif defined(FORTIS_HDBOX)
/***************************************************************************
 *
 * Icons for FS9000/9200
 *
 * Icon structure on front panel display: bitmasks and register numbers
 *          0x01    0x02     0x04    0x08    0x20    0x40  mask
 * 20       Circ1   PLAY     SAT     STANDBY -       i(576)
 * 21       Circ8   Circ3    TER     -       1080p   576
 * 22       Circ2   Circ6    FILE    -       1080i   p(480)
 * 23       Circ7   Circ4    TV      -       720p    i(480)
 * 24       Circ0   Circ5    RADIO   -       p(576)  480
 * 30       REC
 * 31       -       TIME     -       -       -       -
 * 32       -       SHIFT    -       -       -       -
 * 33       -       TIMER    -       -       -       -
 * 34       -       HD       -       -       -       -
 * 35       -       USB      -       -       -       -
 * 36       COLON1  SCRAMBLED-       -       -       -
 * 37       -       DOLBY    -       -       -       -
 * 38       -       MUTE     -       -       -       -
 * 39       COLON2  TUNER1   -       -       -       -
 * 3a       COLON3  TUNER2   -       -       -       -
 * 3b       COLON4  MP3      -       -       -       -
 * 3c       -       REPEAT   -       -       -       -
 * register
 * Note: the bits 4 and 7 are not used as a mask.
 */
struct iconToInternal
{
	char *name;
	u16 icon;
	u8 FP_reg1;   // FP register number
	u8 bit_mask1;  // bitmask for on
	u8 FP_reg2;   // FP register number #2 for double icons, 0xff=single icon
	u8 bit_mask2;  // bitmask #2 for on for double icons
} nuvotonIcons[] =
{
	/*- Name ---------- icon# ---------- code1 data1 code2 data2 -----*/
	{ "ICON_STANDBY"  , ICON_STANDBY   , 0x20, 0x08, 0xff, 0x00},  // 01
	{ "ICON_SAT"      , ICON_SAT       , 0x20, 0x04, 0xff, 0x00},  // 02
	{ "ICON_REC"      , ICON_REC       , 0x30, 0x01, 0xff, 0x00},  // 03
	{ "ICON_TIMESHIFT", ICON_TIMESHIFT , 0x31, 0x02, 0x32, 0x02},  // 04
	{ "ICON_TIMER"    , ICON_TIMER     , 0x33, 0x02, 0xff, 0x00},  // 05
	{ "ICON_HD"       , ICON_HD        , 0x34, 0x02, 0xff, 0x00},  // 06
	{ "ICON_USB"      , ICON_USB       , 0x35, 0x02, 0xff, 0x00},  // 07
	{ "ICON_SCRAMBLED", ICON_SCRAMBLED , 0x36, 0x02, 0xff, 0x00},  // 08
	{ "ICON_DOLBY"    , ICON_DOLBY     , 0x37, 0x02, 0xff, 0x00},  // 09
	{ "ICON_MUTE"     , ICON_MUTE      , 0x38, 0x02, 0xff, 0x00},  // 10
	{ "ICON_TUNER1"   , ICON_TUNER1    , 0x39, 0x02, 0xff, 0x00},  // 11
	{ "ICON_TUNER2"   , ICON_TUNER2    , 0x3a, 0x02, 0xff, 0x00},  // 12
	{ "ICON_MP3"      , ICON_MP3       , 0x3b, 0x02, 0xff, 0x00},  // 13
	{ "ICON_REPEAT"   , ICON_REPEAT    , 0x3c, 0x02, 0xff, 0x00},  // 14
	{ "ICON_PLAY"     , ICON_PLAY      , 0x20, 0x02, 0xff, 0x00},  // 15 play in circle
	{ "ICON_Circ0"    , ICON_Circ0     , 0x24, 0x01, 0xff, 0x00},  // 16 inner circle
	{ "ICON_Circ1"    , ICON_Circ1     , 0x20, 0x01, 0xff, 0x00},  // 17 circle, segment 1
	{ "ICON_Circ2"    , ICON_Circ2     , 0x22, 0x01, 0xff, 0x00},  // 18 circle, segment 2
	{ "ICON_Circ3"    , ICON_Circ3     , 0x21, 0x02, 0xff, 0x00},  // 19 circle, segment 3
	{ "ICON_Circ4"    , ICON_Circ4     , 0x23, 0x02, 0xff, 0x00},  // 20 circle, segment 4
	{ "ICON_Circ5"    , ICON_Circ5     , 0x24, 0x02, 0xff, 0x00},  // 21 circle, segment 5
	{ "ICON_Circ6"    , ICON_Circ6     , 0x22, 0x02, 0xff, 0x00},  // 22 circle, segment 6
	{ "ICON_Circ7"    , ICON_Circ7     , 0x23, 0x01, 0xff, 0x00},  // 23 circle, segment 7
	{ "ICON_Circ8"    , ICON_Circ8     , 0x21, 0x01, 0xff, 0x00},  // 24 circle, segment 8
	{ "ICON_FILE"     , ICON_FILE      , 0x22, 0x04, 0xff, 0x00},  // 25
	{ "ICON_TER"      , ICON_TER       , 0x21, 0x04, 0xff, 0x00},  // 26
	{ "ICON_480i"     , ICON_480i      , 0x24, 0x40, 0x23, 0x40},  // 27
	{ "ICON_480p"     , ICON_480p      , 0x24, 0x40, 0x22, 0x40},  // 28
	{ "ICON_576i"     , ICON_576i      , 0x21, 0x40, 0x20, 0x40},  // 29
	{ "ICON_576p"     , ICON_576p      , 0x21, 0x40, 0x24, 0x20},  // 30
	{ "ICON_720p"     , ICON_720p      , 0x23, 0x20, 0xff, 0x00},  // 31
	{ "ICON_1080i"    , ICON_1080i     , 0x22, 0x20, 0xff, 0x00},  // 32
	{ "ICON_1080p"    , ICON_1080p     , 0x21, 0x20, 0xff, 0x00},  // 33
	{ "ICON_COLON1"   , ICON_COLON1    , 0x36, 0x01, 0xff, 0x00},  // 34 ":" left between text
	{ "ICON_COLON2"   , ICON_COLON2    , 0x39, 0x01, 0xff, 0x00},  // 35 ":" middle between text
	{ "ICON_COLON3"   , ICON_COLON3    , 0x3a, 0x01, 0xff, 0x00},  // 36 ":" right between text
	{ "ICON_COLON4"   , ICON_COLON4    , 0x3b, 0x01, 0xff, 0x00},  // 37 ":" right between text (dimmer)
	{ "ICON_TV"       , ICON_TV        , 0x23, 0x04, 0xff, 0x00},  // 38
	{ "ICON_RADIO"    , ICON_RADIO     , 0x24, 0x04, 0xff, 0x00}  // 39
};

#elif defined(ATEVIO7500)
/***************************************************************
 *
 * Icons for HS8200
 *
 * These are displayed in the leftmost character position.
 * Basically only one icon can be displayed at a time; if more
 * are on they are displayed successively in circular fashion in
 * a thread.
 *
 * A character on the HS8200 display is simply a pixel matrix
 * consisting of five columns of seven pixels high.
 * Column 1 is the leftmost.
 *
 * Format of pixel data:
 *
 * pixeldata1: column 1
 *  ...
 * pixeldata5: column 5
 *
 * Each bit in the pixel data represents one pixel in the column, with the
 * LSbit being the top pixel and bit 6 the bottom one. The pixel will be on
 * when the corresponding bit is one. The resulting map is:
 *
 *  Register    0x27 0x28 0x29 0x2a 0x2b
 *              ------------------------
 *  Bitmask     0x01 0x01 0x01 0x01 0x01
 *              0x02 0x02 0x02 0x02 0x02
 *              0x04 0x04 0x04 0x04 0x04
 *              0x08 0x08 0x08 0x08 0x08
 *              0x10 0x10 0x10 0x10 0x10
 *              0x20 0x20 0x20 0x20 0x20
 *              0x40 0x40 0x40 0x40 0x40
 */
struct iconToInternal
{
	char *name;
	u16 icon;
	u8 pixeldata[5];
} nuvotonIcons[] =
{
	/*- Name ---------- icon# ---------- data1 data2 data3 data4 data5-----*/
	{ "ICON_STANDBY"  , ICON_STANDBY  , {0x38, 0x44, 0x5f, 0x44, 0x38}},  // 01
	{ "ICON_REC"      , ICON_REC      , {0x1c, 0x3e, 0x3e, 0x3e, 0x1c}},  // 02
	{ "ICON_TIMESHIFT", ICON_TIMESHIFT, {0x01, 0x3f, 0x01, 0x2c, 0x34}},  // 03
	{ "ICON_TIMER"    , ICON_TIMER    , {0x3e, 0x49, 0x4f, 0x41, 0x3e}},  // 04
	{ "ICON_HD"       , ICON_HD       , {0x3e, 0x08, 0x3e, 0x22, 0x1c}},  // 05
	{ "ICON_USB"      , ICON_USB      , {0x08, 0x0c, 0x1a, 0x2a, 0x28}},  // 06
	{ "ICON_SCRAMBLED", ICON_SCRAMBLED, {0x3c, 0x26, 0x25, 0x26, 0x3c}},  // 07
	{ "ICON_DOLBY"    , ICON_DOLBY    , {0x3e, 0x22, 0x1c, 0x22, 0x3e}},  // 08
	{ "ICON_MUTE"     , ICON_MUTE     , {0x1e, 0x12, 0x1e, 0x21, 0x3f}},  // 09
	{ "ICON_TUNER1"   , ICON_TUNER1   , {0x01, 0x3f, 0x01, 0x04, 0x3e}},  // 10
	{ "ICON_TUNER2"   , ICON_TUNER2   , {0x01, 0x3f, 0x01, 0x34, 0x2c}},  // 11
	{ "ICON_MP3"      , ICON_MP3      , {0x77, 0x37, 0x00, 0x49, 0x77}},  // 12
	{ "ICON_REPEAT"   , ICON_REPEAT   , {0x14, 0x34, 0x14, 0x16, 0x14}},  // 13
	{ "ICON_PLAY"     , ICON_PLAY     , {0x00, 0x7f, 0x3e, 0x1c, 0x08}},  // 14
	{ "ICON_STOP"     , ICON_STOP     , {0x3C, 0x3C, 0x3C, 0x3C, 0x3C}},  // 15
	{ "ICON_PAUSE"    , ICON_PAUSE    , {0x3e, 0x3e, 0x00, 0x3e, 0x3e}},  // 16
	{ "ICON_REWIND"   , ICON_REWIND   , {0x08, 0x1c, 0x08, 0x1c, 0x00}},  // 17
	{ "ICON_FF"       , ICON_FF       , {0x00, 0x1c, 0x08, 0x1c, 0x08}},  // 18
	{ "ICON_STEP_BACK", ICON_STEP_BACK, {0x08, 0x1c, 0x3e, 0x00, 0x3e}},  // 19
	{ "ICON_STEP_FWD" , ICON_STEP_FWD , {0x3e, 0x00, 0x3e, 0x1c, 0x08}},  // 20 
	{ "ICON_TV"       , ICON_TV       , {0x01, 0x3f, 0x1d, 0x20, 0x1c}},  // 21
	{ "ICON_RADIO"    , ICON_RADIO    , {0x78, 0x4a, 0x4c, 0x4a, 0x79}}  // 22
};
#endif  //Icon definitions
/* End of character and icon definitions */

/***************************************************************************************
 *
 * Driver code.
 *
 */

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
	dprintk(50, "%s len %d\n", __func__, len);
	memcpy(ioctl_data, data, len);
}

int nuvotonWriteCommand(char *buffer, int len, int needAck)
{
	int i;

	dprintk(150, "%s >\n", __func__);

	for (i = 0; i < len; i++)
	{
		udelay(1);
#ifdef DIRECT_ASC  // not defined anywhere
		serial_putc(buffer[i]);
#else
		nuvoton_putc(buffer[i]);
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

/*******************************************************
 *
 * nuvotonSetIcon: (re)sets an icon on the front panel
 *                 display.
 *
 */
#if defined (OCTAGON1008) \
 || defined(HS7420) \
 || defined(HS7429)
int nuvotonSetIcon(int which, int on)
{
	char buffer[7];
	int  res = 0;

	dprintk(100, "%s > %d, %d\n", __func__, which, on);

	if (which < ICON_MIN + 1 || which > ICON_MAX - 1)
	{
		printk("[nuvoton] Icon number %d out of range (%d-%d)\n", which, ICON_MIN + 1, ICON_MAX - 1);
		return -EINVAL;
	}

	which--;
	dprintk(10, "Set icon %s (number %d) to %d\n", nuvotonIcons[which].name, which + 1, on);


	memset(buffer, 0, sizeof(buffer));

	buffer[0] = SOP;
	buffer[1] = cCommandSetVFD;
	buffer[2] = nuvotonIcons[which].pos;

	// icons on positions 0..7 and ICON_DVD
	if ((nuvotonIcons[which].mask2 == 0) && (nuvotonIcons[which].mask3 == 0))
	{
		if (on)
		{
			vfdbuf[nuvotonIcons[which].pos].buf0 = buffer[3] = vfdbuf[nuvotonIcons[which].pos].buf0 | 0x01;
		}
		else
		{
			vfdbuf[nuvotonIcons[which].pos].buf0 = buffer[3] = vfdbuf[nuvotonIcons[which].pos].buf0 & 0xfe;
		}
		buffer[4] = vfdbuf[nuvotonIcons[which].pos].buf1;
		buffer[5] = vfdbuf[nuvotonIcons[which].pos].buf2;
	}
	else if (nuvotonIcons[which].mask1 == 0)
	{
		// icons on position 8, and colons
		buffer[3] = vfdbuf[nuvotonIcons[which].pos].buf0;
		if (on)
		{
			vfdbuf[nuvotonIcons[which].pos].buf1 = buffer[4] = vfdbuf[nuvotonIcons[which].pos].buf1 | nuvotonIcons[which].mask2;
			vfdbuf[nuvotonIcons[which].pos].buf2 = buffer[5] = vfdbuf[nuvotonIcons[which].pos].buf2 | nuvotonIcons[which].mask3;
		}
		else
		{
			vfdbuf[nuvotonIcons[which].pos].buf1 = buffer[4] = vfdbuf[nuvotonIcons[which].pos].buf1 & ~nuvotonIcons[which].mask2;
			vfdbuf[nuvotonIcons[which].pos].buf2 = buffer[5] = vfdbuf[nuvotonIcons[which].pos].buf2 & ~nuvotonIcons[which].mask3;
		}
	}
	buffer[6] = EOP;

	res = nuvotonWriteCommand(buffer, 7, 0);

	if (res == 0)
	{
		lastdata.icon_state[which + 1] = on;
	}
	dprintk(100, "%s <\n", __func__);
	return res;
}

#elif defined(FORTIS_HDBOX)
/* ****************************************************************************
 *
 * nuvotonSetIcon, using both cCommandSetIconI and cCommandSetIconII opcodes
 *
 * The icons on the FS9000/9200 can be divided into two classes:
 * - Icons present on registernumbers 0x31 through 0x3c
 *       > These are controlled by cCommandSetIconI (0xc2), which can set
 *         only one icon in a single command. Note that the bit mask for the
 *         icons on the top row always is 0x02 except ICON_REC. This one and
 *         the colons have bit mask 0x01.
 * - Icons present on register numbers 0x20, 0x21, 0x22, 0x23 and 0x24
 *       > These are controlled by opcode cCommandSetIconII (0xc6), which
 *         can set more than one icon in a single command. In addition, in
 *         early production receivers the front processors can also control
 *         these icons using opcode cCommandSetIconI (0xc2), but this does
 *         not work on newer versions.
 *
 * In both classes so called double icons are present: these are in practice
 * icons that are used in pairs, but require setting/resetting of two items
 * in the display. In the cCommandSetIconI class the only icon of this type
 * is ICON_TIME_SHIFT. The cCommandSetIconII class has four of these:
 * ICON_480I, ICON_480P, ICON_576I and ICON_576P.
 *
 */
int nuvotonSetIcon(int which, int on)
{
	char buffer[9];  // buffer to construct front processor command in
	u8 registernumber;
	int res = 0;

	dprintk(100, "%s > %d, %d\n", __func__, which, on);

	if (which < ICON_MIN + 1 || which > ICON_MAX - 1)
	{
		printk("[nuvoton] Icon number %d out of range (%d-%d)\n", which, ICON_MIN + 1, ICON_MAX - 1);
		return -EINVAL;
	}
	dprintk(10, "Set icon %s (number %d) to %d\n", nuvotonIcons[which - 1].name, which, on);

	registernumber = nuvotonIcons[which - 1].FP_reg1;

	switch (which)  // decide whether to use cCommandSetIconI or cCommandSetIconII
	{
		case ICON_TIMESHIFT:
		case ICON_TIMER:
		case ICON_HD:
		case ICON_USB:
		case ICON_SCRAMBLED:
		case ICON_DOLBY:
		case ICON_MUTE:
		case ICON_TUNER1:
		case ICON_TUNER2:
		case ICON_MP3:
		case ICON_REPEAT:
		{
			// bitmask is 0x02, opcode is cCommandSetIconI

			regs[registernumber] = ((on) ? regs[registernumber] | 0x02 : regs[registernumber] & 0xfd);

			memset(buffer, 0, sizeof(buffer));  // clear buffer
			// construct front processor command
			buffer[0] = SOP;
			buffer[1] = cCommandSetIconI;
			buffer[2] = registernumber;
			buffer[3] = regs[registernumber];
			buffer[4] = EOP;

			if (lastdata.display_on)
			{
				res = nuvotonWriteCommand(buffer, 5, 0);
			}
			if (which == ICON_TIMESHIFT)  // handle SHIFT icon
			{
				regs[0x32] = ((on) ? regs[0x32] | 0x02 : regs[0x32] & 0xfd);
				buffer[0] = SOP;
				buffer[1] = cCommandSetIconI;
				buffer[2] = 0x32;
				buffer[3] = regs[0x32];
				buffer[4] = EOP;

				if (lastdata.display_on)
				{
					res |= nuvotonWriteCommand(buffer, 5, 0);
				}
			}
			break;
		}
		case ICON_REC:
		case ICON_COLON1:
		case ICON_COLON2:
		case ICON_COLON3:
		case ICON_COLON4:
		{
			// bitmask is 0x01, opcode is cCommandSetIconI

			regs[registernumber] = ((on) ? regs[registernumber] | 0x01 : regs[registernumber] & 0xfe);

			memset(buffer, 0, sizeof(buffer));  // clear buffer
			// construct front processor command
			buffer[0] = SOP;
			buffer[1] = cCommandSetIconI;
			buffer[2] = registernumber;
			buffer[3] = regs[registernumber];
			buffer[4] = EOP;

			if (lastdata.display_on)
			{
				res = nuvotonWriteCommand(buffer, 5, 0);
			}
			msleep(1);
			break;
		}
		default:
		{
			// opcode is cCommandSetIconII

			which--;
			regs[registernumber] = ((on) ? regs[registernumber] | nuvotonIcons[which].bit_mask1 : regs[registernumber] & ~nuvotonIcons[which].bit_mask1);

			// handle double icons
			if (nuvotonIcons[which].FP_reg2 != 0xff)
			{
				registernumber = nuvotonIcons[which].FP_reg2;
				regs[registernumber] = ((on) ? regs[registernumber] | nuvotonIcons[which].bit_mask2 : regs[registernumber] & ~nuvotonIcons[which].bit_mask2);
			}

			memset(buffer, 0, sizeof(buffer));  //clear buffer
			// construct front processor command
			buffer[0] = SOP;
			buffer[1] = cCommandSetIconII;
			buffer[2] = 0x20;
			buffer[3] = regs[0x20];
			buffer[4] = regs[0x21];
			buffer[5] = regs[0x22];
			buffer[6] = regs[0x23];
			buffer[7] = regs[0x24];
			buffer[8] = EOP;

			if (lastdata.display_on)
			{
				res = nuvotonWriteCommand(buffer, 9, 0);
			}
			which ++;
			break;
		}
	}
	// update lastdata state
	if (res == 0)
	{
		lastdata.icon_state[which] = on;
	}
	dprintk(100, "%s <\n", __func__);
	return res;
}
#elif defined (ATEVIO7500)
int nuvotonSetIcon(int which, int on)
{
	char buffer[9];  // buffer to construct front processor command in
	int res = 0, i;

	dprintk(100, "%s > %d, %d\n", __func__, which, on);

	if (which < ICON_MIN + 1 || which > ICON_MAX - 1)
	{
		printk("[nuvoton] Icon number %d out of range (%d-%d)\n", which, ICON_MIN + 1, ICON_MAX - 1);
		return -EINVAL;
	}
	dprintk(10, "Set icon %s (number %d) to %d\n", nuvotonIcons[which - 1].name, which, on);

	memset(buffer, 0, sizeof(buffer));  //clear buffer

	for (i = 0; i < 5; i++)
	{
		regs[0x27 + i] = ((on) ? nuvotonIcons[which - 1].pixeldata[i] : 0);
	}

	if (lastdata.display_on)
	{
			buffer[0] = SOP;
			buffer[1] = cCommandSetIconII;
			buffer[2] = 0x27;
			buffer[3] = regs[0x27];
			buffer[4] = regs[0x28];
			buffer[5] = regs[0x29];
			buffer[6] = regs[0x2A];
			buffer[7] = regs[0x2B];
			buffer[8] = EOP;
			res |= nuvotonWriteCommand(buffer, 9, 0);
	}

	regs[0x10] = 7;
	buffer[0] = SOP;
	buffer[1] = cCommandSetIconI;
	buffer[2] = 0x10;
	buffer[3] = regs[0x10];
	buffer[4] = EOP;
	res |= nuvotonWriteCommand(buffer, 5, 0);

	if (res == 0)
	{
		lastdata.icon_state[which] = on;
	}
	dprintk(100, "%s <\n", __func__);
	return res;
}
#elif defined(HS7810A) \
 || defined(HS7119) \
 || defined(HS7819)  // LED models
int nuvotonSetIcon(int which, int on)
{
	int res;
	unsigned char cmd_buf[7];

	dprintk(100, "%s > %d, %d\n", __func__, which, on);

	if (which < ICON_MIN + 1 || which > ICON_MAX - 1)
	{
		printk("[nuvoton] Icon number %d out of range (%d-%d)\n", which, ICON_MIN + 1, ICON_MAX - 1);
		return -EINVAL;
	}
	dprintk(10, "Set icon #%d to %d\n", which, on);

#if defined(HS7119)
	lastdata.buf[3] = ((on) ? lastdata.buf[3] | 0x80 : lastdata.buf[3] & 0x7f);
#else  // HS7810A & HS7819
	switch (which)
	{
		case ICON_COLON:
		{
			lastdata.buf[3] = ((on) ? lastdata.buf[3] | 0x80 : lastdata.buf[3] & 0x7f);
			break;
		}
		case ICON_PERIOD1:
		{
			lastdata.buf[2] = ((on) ? lastdata.buf[2] | 0x80 : lastdata.buf[2] & 0x7f);
			break;
		}
		case ICON_PERIOD2:
		{
			lastdata.buf[4] = ((on) ? lastdata.buf[4] | 0x80 : lastdata.buf[4] & 0x7f);
			break;
		}
		case ICON_PERIOD3:
		{
			lastdata.buf[5] = ((on) ? lastdata.buf[5] | 0x80 : lastdata.buf[5] & 0x7f);
			break;
		}
	}
#endif
	memcpy(&cmd_buf, lastdata.buf, 7);
	res = nuvotonWriteCommand(cmd_buf, 7, 0);

//	if (res == 0)
//	{
//		lastdata.icon_state[which] = on;
//	}
	dprintk(100, "%s <\n", __func__);
	return res;
}
#else  // HS7110 (no display)
int nuvotonSetIcon(int which, int on)
{
	dprintk(100, "%s > %d, %d\n", __func__, which, on);
	dprintk(100, "%s <\n", __func__);
	return 0;
}
#endif

/* export for later use in e2_proc */
EXPORT_SYMBOL(nuvotonSetIcon);

/*******************************************************************
 *
 * nuvotonSetLED: sets brightness of an LED on front panel display.
 *
 */
int nuvotonSetLED(int which, int level)
{
	char buffer[6];
	int res = -1;

	dprintk(100, "%s > %d, %d\n", __func__, which, level);

#if defined(OCTAGON1008) \
 || defined(HS7420) \
 || defined(HS7429) \
 || defined(HS7810A) \
 || defined(HS7819)
#define MAX_LED 3    // LED number is a bit mask:
                     // bit 0 = standby (red),
                     // bit 1 = logo (not on all models)
                     // RC feedback (green, on HS78XX) seems to be not controllable

#define MAX_BRIGHT 7
#elif defined(FORTIS_HDBOX) \
 || defined(ATEVIO7500)                    
#define MAX_LED 255  // LED number is a bit mask: bit 0 (  1) = red power
                     //                           bit 1 (  2) = blue power
                     //                           bit 2 (  4) = -
                     //                           bit 3 (  8) = -
                     //                           bit 4 ( 16) = cross left
                     //                           bit 5 ( 32) = cross bottom
                     //                           bit 6 ( 64) = cross top
                     //                           bit 7 (128) = cross right
                     //                           (0xf2 = full cross plus blue)
#define MAX_BRIGHT 31
#elif defined(HS7110) || defined(HS7119)
#define MAX_LED 1    // LED number is a bit mask:
                     // bit 0 = standby (red),
                     // RC feedback (green) seems to be not controllable (off when red is on)

#define MAX_BRIGHT 7
#endif

#if defined(HS7110)
	printk("[nuvoton] Function %s is not supported on HS7110 (yet)\n", __func__);
	return -EINVAL;
#else
	if (which < 1 || which > MAX_LED)
	{
		printk("[nuvoton] LED number %d out of range (1-%d)\n", which, MAX_LED);
		return -EINVAL;
	}

	if (level < 0 || level > MAX_BRIGHT)
	{
		printk("[nuvoton] LED brightness level %d out of range (0-%d)\n", level, MAX_BRIGHT);
		return -EINVAL;
	}
	dprintk(10, "Set LED 0x%02x brightness level to %d\n", which, level);

	memset(buffer, 0, sizeof(buffer));

	buffer[0] = SOP;
	buffer[1] = cCommandSetLed;
	buffer[2] = which;
	buffer[3] = level;
	buffer[4] = (which == 1 ? 8 : 0);  //handle deep standby brightness (red = 8, others = off)
	buffer[5] = EOP;

	res = nuvotonWriteCommand(buffer, 6, 0);

	dprintk(100, "%s <\n", __func__);
	return res;
#endif
}

/* export for later use in e2_proc */
EXPORT_SYMBOL(nuvotonSetLED);

/****************************************************************
 *
 * nuvotonSetBrightness: sets brightness of front panel display.
 *
 */
#if defined(FORTIS_HDBOX) \
 || defined(OCTAGON1008) \
 || defined(ATEVIO7500) \
 || defined(HS7420) \
 || defined(HS7429)  // VFD models
int nuvotonSetBrightness(int level)
{
	char buffer[5];
	int res = -1;

	dprintk(100, "%s > %d\n", __func__, level);
	if (level < 0 || level > 7)
	{
		printk("[nuvoton] VFD brightness level %d out of range (0-%d)\n", level, 7);
		return -EINVAL;
	}
	dprintk(10, "Set VFD brightness level to %d\n", level);

	memset(buffer, 0, sizeof(buffer));

	buffer[0] = SOP;
	buffer[1] = cCommandSetVFDBrightness;
	buffer[2] = 0x00;
	buffer[3] = level;
	buffer[4] = EOP;

	res = nuvotonWriteCommand(buffer, 5, 0);

	lastdata.brightness = level;

	dprintk(100, "%s <\n", __func__);
	return res;
}
#elif defined(HS7119) \
 || defined(HS7810A) \
 || defined(HS7819)
int nuvotonSetBrightness(int level)
{
	char buffer[6];
	int res = -1;

	dprintk(100, "%s > %d\n", __func__, level);
	if (level < 0 || level > 7)
	{
		printk("[nuvoton] LED display brightness level %d out of range (0-%d)\n", level, 7);
		return -EINVAL;
	}
	dprintk(10, "Set LED display brightness level to %d\n", level);

	switch (level)  // adjust level 0..7 to 0..10
	{
		case 7:
		{
			level = 10;
			break;
		}
		case 6:
		{
			level = 8;
			break;
		}
		case 5:
		{
			level = 6;
			break;
		}
		default:
		{
			break;
		}
	}
	memset(buffer, 0, sizeof(buffer));

	buffer[0] = SOP;
	buffer[1] = cCommandSetLEDBrightness;
	buffer[2] = 0x03;
	buffer[3] = level;
	buffer[4] = 0x02;  // brightness in deep standby
	buffer[5] = EOP;

	res = nuvotonWriteCommand(buffer, 6, 0);

	lastdata.brightness = level;

	dprintk(100, "%s <\n", __func__);
	return res;
}
#else  // HS7110
int nuvotonSetBrightness(int level)
{
	dprintk(100, "%s >\n", __func__);
	dprintk(10, "Set LED brightness level not supported.\n");
	dprintk(100, "%s <\n", __func__);
	return -EFAULT;
}
#endif

/* export for later use in e2_proc */
EXPORT_SYMBOL(nuvotonSetBrightness);

/****************************************************************
 *
 * nuvotonSetStandby: sets wake up time and then switches
 *                    receiver to deep standby.
 *
 */
int nuvotonSetStandby(char *wtime)
{
	char buffer[8];
	char power_off[] = {SOP, cCommandPowerOffReplay, 0x01, EOP};
	int res = 0;

	dprintk(100, "%s >\n", __func__);

//	res = nuvotonWriteString("Bye bye ...", strlen("Bye bye ..."));

	/* set wakeup time */
	memset(buffer, 0, sizeof(buffer));

	buffer[0] = SOP;
	buffer[1] = cCommandSetWakeupMJD;

	memcpy(buffer + 2, wtime, 4); /* only 4 because we do not have seconds here */
	buffer[6] = EOP;

	res = nuvotonWriteCommand(buffer, 7, 0);

	/* now go to standby */
	res = nuvotonWriteCommand(power_off, sizeof(power_off), 0);

	dprintk(100, "%s <\n", __func__);
	return res;
}

/**************************************************
 *
 * nuvotonSetTime: sets front panel clock time.
 *
 */
int nuvotonSetTime(char *time)
{
	char buffer[8];
	int res = 0;

	dprintk(100, "%s >\n", __func__);

	memset(buffer, 0, sizeof(buffer));

	buffer[0] = SOP;
	buffer[1] = cCommandSetMJD;

	memcpy(buffer + 2, time, 5);
	buffer[7] = EOP;

	res = nuvotonWriteCommand(buffer, 8, 0);

	dprintk(100, "%s <\n", __func__);
	return res;
}

/*****************************************************
 *
 * nuvotonGetTime: read front panel clock time.
 *
 */
int nuvotonGetTime(char *time)
{
	char buffer[3];
	int res = 0;
	int i;

	dprintk(100, "%s >\n", __func__);

	memset(buffer, 0, sizeof(buffer));

	buffer[0] = SOP;
	buffer[1] = cCommandGetMJD;
	buffer[2] = EOP;

	errorOccured = 0;
	res = nuvotonWriteCommand(buffer, 3, 1);

	if (errorOccured == 1)
	{
		/* error */
		memset(ioctl_data, 0, sizeof(ioctl_data));
		printk("[nuvoton] Error receiving time.\n");
		res = -ETIMEDOUT;
	}
	else
	{
		/* time received */
		printk("[nuvoton] Time received\n");
		dprintk(20, "Time= 0x%04x (MJD) %02d:%02d:%02d (local)\n", ((ioctl_data[0] << 8) + ioctl_data[1]),
				ioctl_data[2], ioctl_data[3], ioctl_data[4]);
		for (i = 0; i < 5; i++)
		{
			time[i] = ioctl_data[i];
		}
	}
	dprintk(100, "%s <\n", __func__);
	return res;
}

/****************************************************************
 *
 * nuvotonSetWakeUpTime: sets front panel clock wake up time.
 *
 */
int nuvotonSetWakeUpTime(char *wtime)
{
	char buffer[7];
	int  res = 0, i;

	dprintk(100, "%s >\n", __func__);

	/* set wakeup time */
	memset(buffer, 0, sizeof(buffer));

	buffer[0] = SOP;
	buffer[1] = cCommandSetWakeupMJD;

	memcpy(buffer + 2, wtime, 4); /* only 4 because we do not have seconds here */
	buffer[6] = EOP;

	res = nuvotonWriteCommand(buffer, 7, 0);

	for (i = 0; i < 4; i++)
	{
		wakeup_time[i] = wtime[i];
	}
	wakeup_time[4] = 0;

	dprintk(100, "%s <\n", __func__);
	return res;
}

/**************************************************
 *
 * nuvotonGetWakeUpTime: read stored wakeup time.
 *
 */
void nuvotonGetWakeUpTime(char *time)
{
	int i;

	for (i = 0; i < 4; i++)
	{
		time[i] = wakeup_time[i];
	}
	time[4] = 0;
}

/*************************************************************
 *
 * nuvotonGetWakeUpMode: read wake up reason from front panel.
 *
 */
int nuvotonGetWakeUpMode(int *wakeup_mode)
{
	char buffer[3];
	int res = 0;
	char *wakeupmode_name[4]= { "Unknown", "Power on", "From deep standby", "Timer" };

	dprintk(100, "%s >\n", __func__);

	memset(buffer, 0, sizeof(buffer));

	buffer[0] = SOP;
	buffer[1] = cCommandGetPowerOnSrc;
	buffer[2] = EOP;

	errorOccured = 0;
	res = nuvotonWriteCommand(buffer, 3, 1);

	if (errorOccured == 1)
	{
		/* error */
		memset(ioctl_data, 0, sizeof(ioctl_data));
		*wakeup_mode = -1;
		printk("[nuvoton] Error receiving wake up mode.\n");
		res = -ETIMEDOUT;
	}
	else
	{
		/* mode received */
//		printk("[nuvoton] Wake up mode received\n");
		*wakeup_mode = ioctl_data[0] + 1;
		printk("[nuvoton] Wakeup mode is: %1d (%s)\n", *wakeup_mode, wakeupmode_name[*wakeup_mode]);
	}
	dprintk(100, "%s <\n", __func__);
	return res;
}

/*************************************************************
 *
 * nuvotonSetTimeFormat: set 12/24h display format for front
 *                       panel clock in standby.
 *
 */
int nuvotonSetTimeFormat(char format)
{
	char buffer[4];
	int  res = -1;

	dprintk(100, "%s >\n", __func__);

	buffer[0] = SOP;
	buffer[1] = cCommandSetTimeFormat;
	buffer[2] = format;
	buffer[3] = EOP;

	res = nuvotonWriteCommand(buffer, 4, 0);

	dprintk(100, "%s <\n", __func__);
	return res;
}

/*************************************************************
 *
 * nuvotonSetDisplayOnOff: switch entire front panel display
 *                         on or off, retaining display text,
 *                         LED states and icon states
 *
 */
#if defined(FORTIS_HDBOX) \
 || defined(OCTAGON1008) \
 || defined(ATEVIO7500) \
 || defined(HS7420) \
 || defined(HS7429)  // VFD models
int nuvotonSetDisplayOnOff(char level)
{
	int  res = 0;
	int  i;
	char buf[128];
	int  len;
	int  ibuf[ICON_MAX];

	dprintk(100, "%s >\n", __func__);

	if (level == 0)
	{
		memcpy(&buf, lastdata.data, lastdata.length);
		len = lastdata.length;
		res |= nuvotonWriteString("            ", DISP_SIZE);
		lastdata.length = len;
		memcpy(&lastdata.data, buf, lastdata.length);
		
		for (i = ICON_MIN + 1; i < ICON_MAX; i++)
		{
				ibuf[i] = lastdata.icon_state[i];
				res |= nuvotonSetIcon(i, 0);
				lastdata.icon_state[i] = ibuf[i];
		}
#if defined(FORTIS_HDBOX)
		if (spinner_state.state)
		{
			lastdata.icon_state[ICON_SPINNER] = 1;
			spinner_state.state = 0;
		}
#elif defined(ATEVIO7500)
		if (spinner_state.state)
		{
			lastdata.icon_state[ICON_SPINNER] = 1;
			spinner_state.state = 0;
		}
#endif
		lastdata.display_on = 0; 
	}
	else
	{
		lastdata.display_on = 1; 
		res |= nuvotonWriteString(lastdata.data, lastdata.length);
		for (i = ICON_MIN + 1; i < ICON_MAX; i++)
		{
			if (lastdata.icon_state[i] == 1)
			{
				res |= nuvotonSetIcon(i, 1);
			}
		}
#if defined(FORTIS_HDBOX) \
 || defined(ATEVIO7500)
		if (lastdata.icon_state[ICON_SPINNER] != 0)
		{
			spinner_state.state = 1;
			up(&spinner_state.sem);
		}
#endif
	}
	dprintk(100, "%s <\n", __func__);
	return res;
}
#elif defined(HS7810A) \
 || defined(HS7119) \
 || defined(HS7819)
int nuvotonSetDisplayOnOff(char level)
{
	int  res = 0;
	char buffer[6];

	dprintk(100, "%s >\n", __func__);
	if (level != 0)
	{
		level = lastdata.brightness;
	}

	memset(buffer, 0, sizeof(buffer));
	buffer[0] = SOP;
	buffer[1] = cCommandSetLEDBrightness;
	buffer[2] = 0x03;
	buffer[3] = level;
	buffer[4] = 0x02;
	buffer[5] = EOP;

	res = nuvotonWriteCommand(buffer, 6, 0);
	dprintk(100, "%s <\n", __func__);
	return res;
}
#else  // HS7110
int nuvotonSetDisplayOnOff(char level)
{
	dprintk(100, "%s >\n", __func__);
	dprintk(10, "Set display on or off not supported.\n");
	dprintk(100, "%s <\n", __func__);
	return -EFAULT;
}
#endif

/*********************************************************
 *
 * nuvotonGetVersion: read version info from front panel.
 *
 * NOTE: The front processor does not seem to have an
 *       opcode to retrieve version information from it;
 *       in stead this function returns the boot loader
 *       version in the format (major * 100) + minor
 *       and the resellerID.
 */
int nuvotonGetVersion(unsigned int *data)
{
	int res = 0;
	int i = 0;
	unsigned char buf[8];
	struct mtd_info *mtd_info = NULL;

	dprintk(100, "%s >\n", __func__);

	mtd_info = get_mtd_device(NULL, 0);  // get mtd0 device data

	if (IS_ERR(mtd_info))
	{
		printk("[nuvoton] No device for mtd0\n");
		return -1;
	}
	if (mtd_info->type == MTD_ABSENT)
	{
		printk("[nuvoton] mtd0 is absent\n");
		put_mtd_device(mtd_info);
		return -1;
	}
#if 0
	if (strcmp(mtd_info->name, "Boot_firmware"))
	{
		printk([nuvoton] "mtd0 is not the Boot_firmware\n");
		put_mtd_device(mtd_info);
		return -1;
	}
	dprintk(10, "MTD name       : %s\n", mtd_info->name);
	dprintk(10, "MTD type       : %u\n", mtd_info->type);
	dprintk(10, "MTD total size : %u bytes\n", (unsigned int)mtd_info->size);
	dprintk(10, "MTD erase size : %u bytes\n", mtd_info->erasesize);
#endif
	res = mtd_info->read(mtd_info, RESELLER_OFFSET, 8, &i, buf);

	if (i != 8)
	{
		printk("[nuvoton] Error reading boot loader info\n");
		return -1;
	}

	// determine version as (100 * major) + minor
	data[0] = 0;
	for (i = 7; i > 4; i--)  // version number is stored big endian
	{
		data[0] *= 100;
		data[0] += (((buf[i] & 0xf0) >> 4) * 10) + (buf[i] & 0x0f);
	}

	// get resellerID
	data[1] = 0;
	for (i = 0; i < 4; i++)
	{
		data[1] <<= 8;
		data[1] += buf[i];
	}

	dprintk(100, "%s <\n", __func__);
	return res;
}

#if defined(VFDTEST)
/*************************************************************
 *
 * nuvotonVfdTest: Send arbitrary bytes to front processor
 *                 and read back status, dataflag and if
 *                 applicable return bytes (for development
 *                 purposes).
 *
 */
int nuvotonVfdTest(char *data)
{
	char buffer[18];
	int  res = 0xff;
	int  i;

	dprintk(100, "%s >\n", __func__);

	memset(buffer, 0, sizeof(buffer));
	memset(ioctl_data, 0, sizeof(ioctl_data));

	buffer[0] = SOP;
	memcpy(buffer + 1, data + 1, data[0]);
	buffer[data[0] + 1] = EOP;

	if (paramDebug >= 50)
	{
		printk("[nuvoton] Send command: 0x%02x (SOP), ", buffer[0] & 0xff);
		for (i = 1; i < (data[0] + 1); i++)
		{
			printk("0x%02x ", buffer[i] & 0xff);
		}
		printk("0x%02x (EOP) (length = 0x%02x\n", buffer[data[0] + 1] & 0xff, data[0] & 0xff);
	}

	errorOccured = 0;
	res = nuvotonWriteCommand(buffer, data[0] + 2, 1);

	if (errorOccured == 1)
	{
		/* error */
		memset(data + 1, 0, 9);
		printk("[nuvoton] Error in function %s\n", __func__);
		res = data[0] = -ETIMEDOUT;
	}
	else
	{
		if (dataflag == 0)
		{
			memset(data, 0, sizeof(data));
		}
		else
		{
			data[0] = 0;  //command went OK
			data[1] = 1;  //data received
			for (i = 0; i <= 8; i++)
			{
				data[i + 2] = ioctl_data[i];  //copy return data
			}
		}
	}
	dprintk(100, "%s <\n", __func__);
	return res;
}
#endif

/*****************************************************************
 *
 * nuvotonWriteString: Display a text on the front panel display.
 *
 * Note: does not scroll; displays the first DISPLAY_WIDTH
 *       characters. Scrolling is available by writing to
 *       /dev/vfd: this scrolls a maximum of 64 characters once
 *       if the text length exceeds DISPLAY_WIDTH.
 *
 */
#if defined(HS7810A) || defined(HS7819)
// 4 character 7-segment LED with colon and periods
int nuvotonWriteString(unsigned char *aBuf, int len)
{
	int i, res;
	int buflen;
	unsigned char cmd_buf[7], bBuf[9];

	dprintk(100, "%s > %d\n", __func__, len);

	memset(cmd_buf, 0, sizeof(cmd_buf));
	memset(bBuf, 0, sizeof(bBuf));

	if (len > 8)
	{
		len = 8;
	}
	buflen = len;

	/* save last string written to fp */
	memcpy(&lastdata.data, aBuf, len);

	/* Handle periods and colon in text in order to set
       the corresponding LED segments in the display. */

	// Copy input string to work buffer
	for (i = 0; i < buflen; i++)
	{
		bBuf[i] = aBuf[i];
	}
	bBuf[buflen] = '\0';  // terminate string

	// handle 3rd character
	if (bBuf[2] == '.')  // if input character is period
	{
			cmd_buf[2] = 0x80;  // switch on period on position 2
			for (i = 3; i <= buflen; i++)
			{
				bBuf[i - 1] = bBuf[i];
			}
			buflen--;
//			dprintk(10, "String (step 3, period pos 2) [%s]\n", bBuf);
	}
	if (bBuf[2] == ':')  // if input character is colon
	{
			cmd_buf[3] = 0x80;  // switch on colon on position 2
			for (i = 3; i <= buflen; i++)
			{
				bBuf[i - 1] = bBuf[i];
			}
			buflen--;
//			dprintk(10, "String (step 3, colon pos 2) [%s]\n", bBuf);
	}				

	// decide if period is on at position 3
	if (bBuf[2] == '.')  // if next input character is period
	{
		cmd_buf[4] = 0x80;  // switch on period on position 3
		bBuf[2] = ' ';  // replace with a space

//		if ((buflen == 3)
//		{
//			bBuf[2] = ' '; // replace with a space
//		}
//		else if (bBuf[3] == '.')
//		{
//			cmd_buf[5] = 0x80;  // switch on period on position 4
//		}
//		else if (buflen > 4)
//		{
//			for (i = 3; i <= buflen; i++)
//			{
//				bBuf[i - 1] = bBuf[i];
//			}
//			buflen--;
//		}
	}
	else if (bBuf[3] == '.')
	{
		cmd_buf[4] = 0x80;  // switch on period on position 3

		if (buflen > 4)
		{
			for (i = 4; i <= buflen; i++)
			{
				bBuf[i - 1] = bBuf[i];
			}
			buflen--;
		}
		else
		{
			bBuf[3] = ' ';
		}
	}
//	dprintk(10, "String (step 4, period pos 3) [%s]\n", bBuf);

	// decide if period is on at position 4
	if (bBuf[3] == '.')
	{
		cmd_buf[5] = 0x80;  // switch on period on position 4
		bBuf[3] = ' ';  // replace with a space
		
		if (buflen > 4)
		{
			for (i = 3; i <= buflen; i++)
			{
				bBuf[i - 1] = bBuf[i];
			}
			buflen--;
		}
	}
	else
	{
		if (bBuf[4] == '.')
		{
			cmd_buf[5] = 0x80;  // switch on period on position 4
//			dprintk(10, "String (step 5a period pos 4) [%s], l=%d\n", bBuf, buflen);
			if (buflen > 4)
			{
				for (i = 5; i <= buflen; i++)
				{
//					dprintk(10, "loop=%d\n", i - buflen + 1);
					bBuf[i - 1] = bBuf[i];
				}
				buflen--;
//				dprintk(10, "String (step 5b period pos 4) [%s], l=%d\n", bBuf, buflen);
			}
//			dprintk(10, "String (step 5c period pos 4) [%s], l=%d\n", bBuf, buflen);
		}
	}
//	dprintk(10, "String (step 5, period pos 4) [%s], l=%d\n", bBuf, buflen);

	if (buflen > 4)
	{
		buflen = 4;
	}
			
	// add character data
	for (i = 0; i < 4; i++)
	{
		if (bBuf[i] < 0x20)
		{
			cmd_buf[i + 2] &= 0x80;
		}
		else
		{
			cmd_buf[i + 2] |= _7seg_fonts[(bBuf[i] - 0x20)];
		}
	}

	if (buflen < 4)
	{
		for (i = buflen; i < 4; i++)
		{
			cmd_buf[i + 2] &= 0x80;
		}
	}

	cmd_buf[0] = SOP;
	cmd_buf[1] = cCommandSetLEDText;
	cmd_buf[6] = EOP;
	res = nuvotonWriteCommand(cmd_buf, 7, 0);

	memcpy(&lastdata.buf, cmd_buf, 7);
	lastdata.length = len;

	dprintk(100, "%s <\n", __func__);
	return res;
}
#elif defined(HS7119)
// 4 character 7-segment LED with colon
int nuvotonWriteString(unsigned char *aBuf, int len)
{
	int i, res;
	int buflen;
	unsigned char cmd_buf[7], bBuf[5];

	dprintk(100, "%s > %d\n", __func__, len);

	memset(cmd_buf, 0, sizeof(cmd_buf));
	buflen = len;
	for (i = 0; i < buflen; i++)
	{
		bBuf[i] = aBuf[i];
	}

	if (aBuf[2] == ':')  // if 3rd character is a colon
	{
		cmd_buf[3] = 0x80;  // switch colon on
		for (i = 2; i < buflen; i++)
		{
			bBuf[i] = bBuf[i + 1];  // add remove it from text
		}
	}

	if (buflen > 4)
	{
		buflen = 4;
	}

	for (i = 0; i < buflen; i++)
	{
		cmd_buf[i + 2] |= _7seg_fonts[(bBuf[i] - 0x20)];
	}

	cmd_buf[0] = SOP;
	cmd_buf[1] = cCommandSetLEDText;
	cmd_buf[6] = EOP;
	res = nuvotonWriteCommand(cmd_buf, 7, 0);

	/* save last string written to fp */
	memcpy(&lastdata.data, aBuf, len);
	memcpy(&lastdata.buf, cmd_buf, 7);
	lastdata.length = len;

	dprintk(100, "%s <\n", __func__);
	return res;
}
#elif defined(OCTAGON1008) \
 || defined(HS7420) \
 || defined(HS7429)
// 8 character 15-segment VFD with icons
int nuvotonWriteString(unsigned char *aBuf, int len)
{
	unsigned char bBuf[7];
	int i = 0, max = 0;
	int j = 0;
	int res = 0;

	dprintk(100, "%s > len = %d\n", __func__, len);
	max = (len > 8) ? 8 : len;
	for (i = max; i < 8; i++)
	{
		aBuf[i] = ' ';  // pad string with spaces at end
	}
	// now display 8 characters from left to right
	for (i = 0; i < 8 ; i++)
	{
		bBuf[0] = SOP;
		bBuf[1] = cCommandSetVFD;
		bBuf[2] = 7 - i;  // position 0x00 = right
		bBuf[3] = vfdbuf[7 - i].buf0;
		bBuf[6] = EOP;

		for (j = 0; j < ARRAY_SIZE(octagon_chars); j++)
		{
			if (octagon_chars[j].character == aBuf[i])  // look up character
			{
				bBuf[4] = octagon_chars[j].s1 | (vfdbuf[7 - i].buf1 & 0x80);  // preserve colon icon state
				vfdbuf[7 - i].buf1 = bBuf[4];
				bBuf[5] = octagon_chars[j].s2;
				vfdbuf[7 - i].buf2 = bBuf[5];
				res |= nuvotonWriteCommand(bBuf, 7, 0);
				break;
			}
		}
	}
	/* save last string written to fp */
	memcpy(&lastdata.data, aBuf, 8);
	lastdata.length = len;

	dprintk(100, "%s <\n", __func__);
	return res;
}
#elif defined(ATEVIO7500) \
 || defined(FORTIS_HDBOX)
// ATEVIO7500  : 13 character dot matrix VFD without colons or icons,
//               leftmost character used as icon display
// FORTIS_HDBOX: 12 character dot matrix VFD with colons and icons
int nuvotonWriteString(unsigned char *aBuf, int len)
{
	unsigned char bBuf[128];
	int i = 0;
	int j = 0;
	int res = 0;

	dprintk(100, "%s >\n", __func__);

	memset(bBuf, ' ', sizeof(bBuf));

	/* start of command write */
	bBuf[0] = SOP;
	bBuf[1] = cCommandSetVFDIII;
	bBuf[2] = 17;  // length of command

	/* save last string written to fp */
	memcpy(&lastdata.data, aBuf, len);
	lastdata.length = len;

	dprintk(70, "len %d\n", len);

	while ((i < len) && (j < 12))
	{
		if (aBuf[i] < 0x80)  // if normal ASCII
		{
			bBuf[j + 3] = aBuf[i];  // stuff buffer from offset 3 on
		}
		else if (aBuf[i] < 0xe0)
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
				bBuf[j + 3] = UTF_Char_Table[aBuf[i] & 0x3f];
			}
			else
			{
				sprintf(&bBuf[j + 3], "%02x", aBuf[i - 1]);
				j += 2;
				bBuf[j + 3] = (aBuf[i] & 0x3f) | 0x40;
			}
		}
		else
		{
			if (aBuf[i] < 0xF0)
			{
				i += 2;
			}
			else if (aBuf[i] < 0xF8)
			{
				i += 3;
			}
			else if (aBuf[i] < 0xFC)
			{
				i += 4;
			}
			else
			{
				i += 5;
			}
			bBuf[j + 3] = 0x20;  // else put a space
		}
		i++;
		j++;
	}
	/* end of command write, string must be padded with spaces */
	bBuf[15] = 0x20;  // Audioniek: this character is not on the display!
	bBuf[16] = EOP;

	if (lastdata.display_on)
	{
		res = nuvotonWriteCommand(bBuf, 17, 0);
	}
	dprintk(100, "%s <\n", __func__);
	return res;
}
#else  // not HS7119, HS7420, HS7429, HS7810A, HS7819, OCTAGON1008, FORTIS_HDBOX or ATEVIO7500 -> HS7110
int nuvotonWriteString(unsigned char *aBuf, int len)
{
	dprintk(100, "%s >\n", __func__);
	dprintk(100, "%s <\n", __func__);
	return -EFAULT;
}
#endif

/****************************************************************
 *
 * nuvoton_init_func: Initialize the driver.
 *
 */
int nuvoton_init_func(void)
{
	// Start of declarations, model dependent

	// FP commands commands common to all models
	char boot_on[] = {SOP, cCommandSetBootOn, EOP};
	char standby_disable[] = {SOP, cCommandPowerOffReplay, 0x02, EOP};

	//FP commands per model
#if defined(FORTIS_HDBOX)
/* Shortened essential factory sequence FS9000/9200
 *
 * SOP ce 10 20 20 20 20 20 20 20 20 20 20 20 20 20 EOP blank display
 * SOP cd 30 20 20 20 20 20 20 20 20 20 20 20 20 EOP    blank display
 * SOP f9 80 00 00 00 00 00 00 00 00 EOP                ?
 * SOP f4 30 0f 00 00 EOP                               ?
 * SOP e9 76 00 00 00 00 00 00 00 00 EOP                ?
 * SOP f9 83 54 2f df 55 10 18 2b b7 EOP                ?
 * SOP eb 83 25 d6 ca cc 6a de 7c ee 00 00 EOP          ?
 * SOP 31 02 EOP                                        cCommandPowerOffReplay 2 (standby_disable)
 * SOP d0 EOP                                           cCommandGetFrontInfo
 * SOP 80 EOP                                           cCommandGetPowerOnSrc
 * SOP c2 10 00 EOP                                     cCommandSetIconI 10 0 (enable cCommandSetIconII)
 * SOP d2 00 07 EOP                                     cCommandSetVFDBrightness 7 ()
 * SOP a5 01 02 f9 10 0b EOP                            cCommandSetIrCode
 * SOP 15 dc d8 00 00 01 EOP                            cCommandSetMJD to 01-09-2013 00:00:01
 * SOP 11 81 EOP                                        cCommandSetTimeFormat
 * SOP 72 ff ff EOP                                     cCommandSetWakeupTime
 */

	char init1[] = {SOP, cCommandSetVFDIII, 0x10, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', EOP};  //blank display
	char init2[] = {SOP, cCommandSetIconI, 0x10, 0x00, EOP};                     // enable cCommandSetIconII
	char init3[] = {SOP, cCommandSetVFDBrightness, 0x00, 0x07, EOP};             // VFD brightness 7
	char init4[] = {SOP, cCommandSetIrCode, 0x01, 0x02, 0xf9, 0x10, 0x0b, EOP};  // set IR code
	char init5[] = {SOP, cCommandSetTimeFormat, 0x81, EOP};                      // set 24h clock format
	char init6[] = {SOP, cCommandSetWakeupTime, 0xff, 0xff, EOP};                // delete/invalidate wakeup time
	char init7[] = {SOP, cCommandSetLed, 0x01, 0x00, 0x08, EOP};                 // power LED (red) off, deep standby brightness 8
	char init8[] = {SOP, cCommandSetLed, 0xf2, 0x08, 0x00, EOP};                 // blue LED plus cross brightness 8, deep standby off

#elif defined(OCTAGON1008)
/* Shortened essential factory sequence HS9510
 *
 * SOP c4 00 00 00 00 03     cCommandSetVFD           -
 * SOP c4 01 00 00 00 03     cCommandSetVFD            \
 * SOP c4 02 00 00 00 03     cCommandSetVFD             |
 * SOP c4 03 00 00 00 03     cCommandSetVFD             |
 * SOP c4 04 00 00 00 03     cCommandSetVFD              > Clear display
 * SOP c4 05 00 00 00 03     cCommandSetVFD             |
 * SOP c4 06 00 00 00 03     cCommandSetVFD             |
 * SOP c4 07 00 00 00 03     cCommandSetVFD            /
 * SOP c4 08 00 00 00 03     cCommandSetVFD           -
 * SOP d2 00 07 03           cCommandSetVFDBrightness 7
 * SOP 31 02 EOP             cCommandPowerOffReplay 2 (standby_disable)
 * SOP d0 EOP                cCommandGetFrontInfo
 * SOP 80 EOP                cCommandGetPowerOnSrc
 * SOP d2 00 07 EOP          cCommandSetVFDBrightness 7
 * SOP a5 01 02 f9 10 0b EOP cCommandSetIrCode
 * SOP 15 dc d8 00 00 01 EOP cCommandSetMJD to 01-09-2013 00:00:01
 * SOP 11 81 EOP             cCommandSetTimeFormat
 * SOP 72 ff ff EOP          cCommandSetWakeupTime
 */

	char init1[] = {SOP, cCommandSetVFD, 0x00, 0x00, 0x00, 0x00, EOP};           // blank display (pos 0)
	char init2[] = {SOP, cCommandSetVFD, 0x01, 0x00, 0x00, 0x00, EOP};           // blank display (pos 1)
	char init3[] = {SOP, cCommandSetVFD, 0x02, 0x00, 0x00, 0x00, EOP};           // blank display (pos 2)
	char init4[] = {SOP, cCommandSetVFD, 0x03, 0x00, 0x00, 0x00, EOP};           // blank display (pos 3)
	char init5[] = {SOP, cCommandSetVFD, 0x04, 0x00, 0x00, 0x00, EOP};           // blank display (pos 4)
	char init6[] = {SOP, cCommandSetVFD, 0x05, 0x00, 0x00, 0x00, EOP};           // blank display (pos 5)
	char init7[] = {SOP, cCommandSetVFD, 0x06, 0x00, 0x00, 0x00, EOP};           // blank display (pos 6)
	char init8[] = {SOP, cCommandSetVFD, 0x07, 0x00, 0x00, 0x00, EOP};           // blank display (pos 7)
	char init9[] = {SOP, cCommandSetVFD, 0x08, 0x00, 0x00, 0x00, EOP};           // blank display (pos 8)
	char initA[] = {SOP, cCommandSetVFDBrightness, 0x00, 0x07, EOP};             // VFD brightness 7
	char initB[] = {SOP, cCommandSetIrCode, 0x01, 0x02, 0xf9, 0x10, 0x0b, EOP};  // set IR code
	char initC[] = {SOP, cCommandSetTimeFormat, 0x81, EOP};                      // set 24h clock format
	char initD[] = {SOP, cCommandSetWakeupTime, 0xff, 0xff, EOP};                // delete/invalidate wakeup time
	char initE[] = {SOP, cCommandSetLed, 0x01, 0x00, 0x08, EOP};                 // red LED off, deep standby brightness 8

#elif defined(HS7420) \
 || defined(HS7429)
/*
 * Shortened essential factory power on sequence HS7429 (note: HS7420 assumed to be similar)
 * SOP 11 81 EOP               cCommandSetTimeFormat 24h
 * SOP 72 ff ff EOP            cCommandSetWakeupMJD to invalid
 * SOP 93 01 00 08 EOP         cCommandSetLed  1 00 08 red LED off
 * SOP 93 f2 08 08 EOP         cCommandSetLed f2 08 08 blue+cross 8 (!)
 * SOP 41 88 EOP               cCommandSetBootOnExt
 * SOP a5 01 00 01 00 01 EOP   cCommandSetIrCode   (note: other data and also uses:
 * SOP a1 01 03                cCommandGetIrCodeExt this one)
 */
	char init1[] = {SOP, cCommandSetVFD, 0x00, 0x00, 0x00, 0x00, EOP};  // blank display (pos 0)
	char init2[] = {SOP, cCommandSetVFD, 0x01, 0x00, 0x00, 0x00, EOP};  // blank display (pos 1)
	char init3[] = {SOP, cCommandSetVFD, 0x02, 0x00, 0x00, 0x00, EOP};  // blank display (pos 2)
	char init4[] = {SOP, cCommandSetVFD, 0x03, 0x00, 0x00, 0x00, EOP};  // blank display (pos 3)
	char init5[] = {SOP, cCommandSetVFD, 0x04, 0x00, 0x00, 0x00, EOP};  // blank display (pos 4)
	char init6[] = {SOP, cCommandSetVFD, 0x05, 0x00, 0x00, 0x00, EOP};  // blank display (pos 5)
	char init7[] = {SOP, cCommandSetVFD, 0x06, 0x00, 0x00, 0x00, EOP};  // blank display (pos 6)
	char init8[] = {SOP, cCommandSetVFD, 0x07, 0x00, 0x00, 0x00, EOP};  // blank display (pos 7)
	char init9[] = {SOP, cCommandSetVFD, 0x08, 0x00, 0x00, 0x00, EOP};  // blank display (pos 8)
	char initA[] = {SOP, cCommandSetVFDBrightness, 0x00, 0x07, EOP};    // VFD brightness 7
	char initB[] = {SOP, cCommandSetTimeFormat, 0x81, EOP};             // set 24h clock format
	char initC[] = {SOP, cCommandSetWakeupTime, 0xff, 0xff, EOP};       // delete/invalidate wakeup time
	char initD[] = {SOP, cCommandSetLed, 0x01, 0x00, 0x08, EOP};        // power LED (red) off, deep standby brightness 8
	char initE[] = {SOP, cCommandSetLed, 0x02, 0x03, 0x00, EOP};        // logo brightness 3, deep standby off

#elif defined(ATEVIO7500)  // sequence is identical to HDBOX
/* Shortened essential factory sequence HS8200
 *
 * SOP ce 10 20 20 20 20 20 20 20 20 20 20 20 20 20 EOP blank display
 * SOP cd 30 20 20 20 20 20 20 20 20 20 20 20 20 EOP    blank display
 * SOP f9 80 00 00 00 00 00 00 00 00 EOP                ?
 * SOP f4 30 0f 00 00 EOP                               ?
 * SOP e9 76 00 00 00 00 00 00 00 00 EOP                ?
 * SOP f9 83 54 2f df 55 10 18 2b b7 EOP                ?
 * SOP eb 83 25 d6 ca cc 6a de 7c ee 00 00 EOP          ?
 * SOP 31 02 EOP                                        cCommandPowerOffReplay 2 (standby_disable)
 * SOP d0 EOP                                           cCommandGetFrontInfo
 * SOP 80 EOP                                           cCommandGetPowerOnSrc
 * SOP c2 10 00 EOP                                     cCommandSetIconI 10 0 (enable cCommandSetIconII/cgram off)
 * SOP d2 00 07 EOP                                     cCommandSetVFDBrightness 7 ()
 * SOP a5 01 02 f9 10 0b EOP                            cCommandSetIrCode
 * SOP 15 dc d8 00 00 01 EOP                            cCommandSetMJD to 01-09-2013 00:00:01
 * SOP 11 81 EOP                                        cCommandSetTimeFormat
 * SOP 72 ff ff EOP                                     cCommandSetWakeupTime
 */

	char init1[] = {SOP, cCommandSetVFDIII, 0x10, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', EOP};  // blank display
	char init2[] = {SOP, cCommandSetIconI, 0x10, 0x00, EOP};                     // enable cCommandSetIconII
	char init3[] = {SOP, cCommandSetVFDBrightness, 0x00, 0x07, EOP};             // VFD brightness 7
	char init4[] = {SOP, cCommandSetIrCode, 0x01, 0x02, 0xf9, 0x10, 0x0b, EOP};  // set IR code
	char init5[] = {SOP, cCommandSetTimeFormat, 0x81, EOP};                      // set 24h clock format
	char init6[] = {SOP, cCommandSetWakeupTime, 0xff, 0xff, EOP};                // delete/invalidate wakeup time
	char init7[] = {SOP, cCommandSetLed, 0x01, 0x00, 0x08, EOP};                 // power LED (red) off, deep standby brightness 8
	char init8[] = {SOP, cCommandSetLed, 0xf2, 0x08, 0x00, EOP};                 // blue LED plus cross brightness 8, deep standby off

#elif defined(HS7119) \
 || defined(HS7810A) \
 || defined(HS7819)
/* Shortened essential factory sequence HS7810A (note: HS7819 assumed to be similar)
 *
 * SOP 23 03 0a 02 EOP         cCommandSetLEDBrightness
 * SOP 24 20 20 20 20 EOP      cCommandSetLEDText "----"
 * SOP d2 00 0a EOP            cCommandSetVFDBrightness 10 (?)
 * SOP 24 6d 78 00 06 EOP      cCommandSetLEDText
 * SOP a5 01 02 f9 10 0b EOP   cCommandSetIrCode
 * SOP 15 dc 9a 00 00 02 EOP   cCommandSetMJD to 01-05-2013 00:00:02
 * SOP 11 81 EOP               cCommandSetTimeFormat 24h
 * SOP 72 ff ff EOP            cCommandSetWakeupTime invalid
 * SOP 93 01 00 08 EOP         cCommandSetLed 1 0 red LED off
 * SOP 93 f2 08 00 EOP         cCommandSetLed blue+cross 8 (!)
 * SOP 10 EOP                  cCommandGetMJD
 * SOP 15 e1 a9 14 18 00 03    cCommandSetMJD to today 18:00:03
 */
	char init1[] = {SOP, cCommandSetLEDBrightness, 0x03, 0x0a, 0x02, EOP};       // set display brightness
	char init2[] = {SOP, cCommandSetIrCode, 0x01, 0x02, 0xf9, 0x10, 0x0b, EOP};  // set IR code
	char init3[] = {SOP, cCommandSetTimeFormat, 0x81, EOP};                      // set 24h clock format
	char init4[] = {SOP, cCommandSetWakeupTime, 0xff, 0xff, EOP};                // delete/invalidate wakeup time
	char init5[] = {SOP, cCommandSetLed, 0x01, 0x00, 0x08, EOP};                 // power LED (red) off, deep standby brightness 8
	char init6[] = {SOP, cCommandSetLed, 0x02, 0x03, 0x00, EOP};                 // logo brightness 3, deep standby off
#else  //HS7110
/* Shortened essential factory sequence HS7110
 *
 * SOP d0 SOP                  cCommandGetFrontInfo
 * SOP 80 SOP                  cCommandGetPowerOnSrc
 * SOP 24 20 20 20 20 SOP      cCommandSetLEDText "----" (!)
 * SOP d2 00 0a SOP            cCommandSetVFDBrightness 10 (!)
 * SOP a5 01 02 f9 10 0b SOP   cCommandSetIrCode
 * SOP 15 dc 5d 00 00 02 SOP   cCommandSetMJD to 01-05-2013 00:00:02
 * SOP 11 81 SOP               cCommandSetTimeFormat 24h
 * SOP 72 ff ff SOP            cCommandSetWakeupTime invalid
 * SOP 93 01 00 08 SOP         cCommandSetLed 1 0 red LED off
 * SOP 93 f2 08 00 SOP         cCommandSetLed f2 8 cross+blue 8 (!)
 * SOP 24 76 5e 5e 7b SOP      cCommandSetLEDText (!)
 * SOP 41 44 SOP               cCommandSetBootOnExt
 * SOP 15 e1 a9 0b 11 31 03    cCommandSetMJD current date/time 
 */
	char init1[] = {SOP, cCommandSetLEDBrightness, 0x03, 0x0a, 0x02, EOP};       // set display brightness
	char init2[] = {SOP, cCommandSetIrCode, 0x01, 0x02, 0xf9, 0x10, 0x0b, EOP};  // set IR code
	char init3[] = {SOP, cCommandSetTimeFormat, 0x81, EOP};                      // set 24h clock format
	char init4[] = {SOP, cCommandSetWakeupTime, 0xff, 0xff, EOP};                // delete/invalidate wakeup time
	char init5[] = {SOP, cCommandSetLed, 0xff, 0x00, 0x00, EOP};                 // all LEDs off (=green on)
#endif

	int  vLoop;
	int  res = 0;
	// End of declarations

	dprintk(100, "%s >\n", __func__);

	lastdata.display_on = 1;
	lastdata.brightness = 7;
	sema_init(&write_sem, 1);

#if defined(OCTAGON1008)
	printk("Fortis HS9510");
#elif defined(FORTIS_HDBOX)
	printk("Fortis FS9000/9200");
#elif defined(ATEVIO7500)
	printk("Fortis HS8200");
#elif defined(HS7110)
	printk("Fortis HS7110");
#elif defined(HS7119)
	printk("Fortis HS7119");
#elif defined(HS7420)
	printk("Fortis HS7420");
#elif defined(HS7429)
	printk("Fortis HS7429");
#elif defined(HS7810A)
	printk("Fortis HS7810");
#elif defined(HS7819)
	printk("Fortis HS7819");
#else
	printk("Fortis");
#endif
	printk(" Nuvoton front panel driver initializing.\n");

	/* Initialise the front processor */

	/* must be called before standby_disable */
	res = nuvotonWriteCommand(boot_on, sizeof(boot_on), 0);  // SetBootOn

	/* setup: frontpanel should not power down the receiver if standby is selected */
	res = nuvotonWriteCommand(standby_disable, sizeof(standby_disable), 0);

	res |= nuvotonWriteCommand(init1, sizeof(init1), 0);
	msleep(1);
	res |= nuvotonWriteCommand(init2, sizeof(init2), 0);
	msleep(1);
	res |= nuvotonWriteCommand(init3, sizeof(init3), 0);
	msleep(1);
	res |= nuvotonWriteCommand(init4, sizeof(init4), 0);
 	res |= nuvotonWriteCommand(init5, sizeof(init5), 0);
#if defined(FORTIS_HDBOX) \
 || defined(OCTAGON1008) \
 || defined(ATEVIO7500) \
 || defined(HS7420) \
 || defined(HS7429) \
 || defined(HS7119) \
 || defined(HS7810A) \
 || defined(HS7819)
	res |= nuvotonWriteCommand(init6, sizeof(init6), 0);
#endif
#if defined(FORTIS_HDBOX) \
 || defined(OCTAGON1008) \
 || defined(ATEVIO7500) \
 || defined(HS7420) \
 || defined(HS7429)
	res |= nuvotonWriteCommand(init7, sizeof(init7), 0);
	res |= nuvotonWriteCommand(init8, sizeof(init8), 0);
#endif
#if defined(OCTAGON1008) \
 || defined(HS7420) \
 || defined(HS7429)
	res |= nuvotonWriteCommand(init9, sizeof(init9), 0);
	res |= nuvotonWriteCommand(initA, sizeof(initA), 0);
	res |= nuvotonWriteCommand(initB, sizeof(initB), 0);
	res |= nuvotonWriteCommand(initC, sizeof(initC), 0);
	res |= nuvotonWriteCommand(initD, sizeof(initD), 0);
	res |= nuvotonWriteCommand(initE, sizeof(initE), 0);
#endif

	for (vLoop = 0x00; vLoop <= 0xff; vLoop++)
	{
		regs[vLoop] = 0x00;  // initialize local shadow registers
	}

//	res |= nuvotonWriteString("SH4", strlen("SH4"));

#if !defined(HS7110)
	for (vLoop = ICON_MIN + 1; vLoop < ICON_MAX; vLoop++)
	{
		res |= nuvotonSetIcon(vLoop, 0);  //switch all icons off
	}
#endif
#if defined(ATEVIO7500)
	lastdata.icon_count = 0;
#endif

#if defined(HS7119) \
 || defined(HS7810A) \
 || defined(HS7819)  //LED models
	res |= nuvotonWriteString("----", 4);  // HS7810A, HS7819 & HS7119: show 4 dashes
#endif
	wakeup_time[0] = 40587 >> 8;  // set initial wakeup time to Linux epoch
	wakeup_time[1] = 40587 & 0xff;
	wakeup_time[2] = 0;
	wakeup_time[3] = 0;
	wakeup_time[4] = 0;

	// Handle initial GMT offset (may be changed by writing to /proc/stb/fp/rtc_offset)
	res = strict_strtol(gmt_offset, 10, (long *)&rtc_offset);
	if (res && gmt_offset[0] == '+')
	{
		res = strict_strtol(gmt_offset + 1, 10, (long *)&rtc_offset);
	}
	dprintk(100, "%s <\n", __func__);
	return 0;
}

/****************************************
 *
 * Code for writing to /dev/vfd
 *
 */
void clear_display(void)
{
	unsigned char bBuf[12];
	int res = 0;

	dprintk(100, "%s >\n", __func__);

	memset(bBuf, ' ', sizeof(bBuf));
	res = nuvotonWriteString(bBuf, DISP_SIZE);
	dprintk(100, "%s <\n", __func__);
}

static ssize_t NUVOTONdev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char *kernel_buf;
	int minor, vLoop, res = 0;
	int llen;
	int pos;
	int offset = 0;
	char buf[64];
	char *b;
	int corr;

	dprintk(100, "%s > (len %d, off %d)\n", __func__, len, (int) *off);

	if ((len == 0) || (DISP_SIZE == 0))
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
		printk("[nuvoton] Error Bad Minor\n");
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
		printk("[nuvoton] %s < No memory.\n", __func__);
		return -ENOMEM;
	}

	copy_from_user(kernel_buf, buff, len);

	if (write_sem_down())
	{
		return -ERESTARTSYS;
	}

	llen = len;

	corr = 0;
#if defined(HS7119) \
 || defined(HS7810A) \
 || defined(HS7819)
	if (kernel_buf[2] == ':')
	{
		corr++;  // correct scroll when 3rd char is a colon
	}
#endif
#if defined(HS7810A) \
 || defined(HS7819)
	if ((kernel_buf[2] == '.') && (kernel_buf[3] == ':'))
	{
		corr += 2;  // correct scroll when 2nd char is a period 3rd char is a colon
	}
	for (pos = 2 + corr; pos < 6 + corr; pos++)
	{
		if (kernel_buf[pos] == '.')
		{
			corr++;  // correct scroll when 3rd, 4th or 5th char is a period
		}
	}
#endif
	if (llen >= 64)  //do not display more than 64 characters
	{
		llen = 64;
	}

	/* Dagobert: echo add a \n which will not be counted as a char */
	/* Audioniek: Does not add a \n but compensates string length as not to count it */
	if (kernel_buf[len - 1] == '\n')
	{
		llen--;
	}

	if (llen <= DISP_SIZE + corr)  //no scroll
	{
		res = nuvotonWriteString(kernel_buf, llen);
	}
	else  // scroll, display string is longer than display length
	{
		// initial display starting at 3rd position to ease reading
		memset(buf, ' ', sizeof(buf));
		offset = 3;
		memcpy(buf + offset, kernel_buf, llen);
		llen += offset;
		buf[llen + DISP_SIZE + corr] = '\0';  // terminate string

		// scroll text
		b = buf;
		for (pos = 0; pos < llen; pos++)
		{
			res = nuvotonWriteString(b + pos, DISP_SIZE + corr);
			// sleep 300 ms
			msleep(300);
		}
		clear_display();

		// final display
		res = nuvotonWriteString(buf + offset, DISP_SIZE + corr);
	}
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
static ssize_t NUVOTONdev_read(struct file *filp, char __user *buff, size_t len, loff_t *off)
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
		printk("[nuvoton] Error Bad Minor\n");
		return -EUSERS;
	}

	dprintk(100, "minor = %d\n", minor);
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
		printk("[nuvoton] %s return erestartsys<\n", __func__);
		return -ERESTARTSYS;
	}

	if (FrontPanelOpen[minor].read == lastdata.length)
	{
		FrontPanelOpen[minor].read = 0;
		up(&FrontPanelOpen[minor].sem);
		printk("[nuvoton] %s < return 0\n", __func__);
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

int NUVOTONdev_open(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(100, "%s >\n", __func__);

	/* needed! otherwise a racecondition can occur */
	if (down_interruptible(&write_sem))
	{
		return -ERESTARTSYS;
	}

	minor = MINOR(inode->i_rdev);

	dprintk(70, "open minor %d\n", minor);

	if (FrontPanelOpen[minor].fp != NULL)
	{
		printk("[nuvoton] dev_open: EUSER opening minor\n");
		up(&write_sem);
		return -EUSERS;
	}

	FrontPanelOpen[minor].fp = filp;
	FrontPanelOpen[minor].read = 0;

	up(&write_sem);
	dprintk(100, "%s <\n", __func__);
	return 0;
}

int NUVOTONdev_close(struct inode *inode, struct file *filp)
{
	int minor;

	dprintk(100, "%s >\n", __func__);
	minor = MINOR(inode->i_rdev);
	dprintk(20, "close minor %d\n", minor);

	if (FrontPanelOpen[minor].fp == NULL)
	{
		printk("[nuvoton] dev_close: EUSER opening minor\n");
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
static int NUVOTONdev_ioctl(struct inode *Inode, struct file *File, unsigned int cmd, unsigned long arg)
{
	static int mode = 0;
	struct nuvoton_ioctl_data *nuvoton = (struct nuvoton_ioctl_data *)arg;
	struct vfd_ioctl_data *data = (struct vfd_ioctl_data *)arg;
	int res = 0;
	int level;
#if !defined(HS7110)
	int icon_nr, on;
	int i;
#endif

	dprintk(100, "%s > 0x%.8x\n", __func__, cmd);

	if (down_interruptible(&write_sem))
	{
		return -ERESTARTSYS;
	}

	switch (cmd)
	{
		case VFDSETMODE:
		{
			mode = nuvoton->u.mode.compat;
			break;
		}
#if 0
		case VFDPWRLED:  // deprecated
#endif
		case VFDSETLED:
		{
			res = nuvotonSetLED(nuvoton->u.led.led_nr, nuvoton->u.led.level);
			break;
		}
		case VFDBRIGHTNESS:
		{
#if !defined(HS7110)
			if (mode == 0)
			{
				level = data->start_address;
				dprintk(10, "%s Set display brightness to %d (mode 0)\n", __func__, level);
			}
			else
			{
				level = nuvoton->u.brightness.level;
				dprintk(10, "%s Set display brightness to %d (mode 1)\n", __func__, data->start_address);
			}
			if (level < 0)
			{
				level = 0;
			}
			else if (level > 7)
			{
				level = 7;
			}
			res = nuvotonSetBrightness(level);
#endif  //HS7110
			mode = 0;
			break;
		}
		case VFDDRIVERINIT:
		{
			res = nuvoton_init_func();
			mode = 0;
			break;
		}
		case VFDICONDISPLAYONOFF:
		{
#if !defined(HS7110)
			icon_nr = mode == 0 ? data->data[0] : nuvoton->u.icon.icon_nr;
			on = mode == 0 ? data->data[4] : nuvoton->u.icon.on;
			dprintk(10, "%s Set icon %d to %d (mode %d)\n", __func__, icon_nr, on, mode);
			on = on != 0 ? 1 : 0;

			// Part one: translate E2 icon numbers to own icon numbers (vfd mode only)
			if (mode == 0)  // vfd mode
			{
				switch (icon_nr)
				{
#if defined(FORTIS_HDBOX) \
 || defined(ATEVIO7500)
					case 0x13:  // crypted
					{
						icon_nr = ICON_SCRAMBLED;
						break;
					}
					case 0x17:  // dolby
					{
						icon_nr = ICON_DOLBY;
						break;
					}
#if !defined(ATEVIO7500)
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
					case 0x1e:  // record
					{
						icon_nr = ICON_REC;
						break;
					}
					case 0x1a:  // seekable (play)
					{
						icon_nr = ICON_PLAY;
						break;
					}
#elif defined(OCTAGON1008)
					case 0x13:  // crypted
					{
						icon_nr = ICON_CRYPTED;
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
#elif defined(HS7420) \
 || defined(HS7429)
					case 0x1e:  // record
					{
						icon_nr = ICON_DOT;
						break;
					}
#endif
					default:
					{
						break;
					}
				}
			}

			// Part two: decide wether one icon, all or spinner
			switch (icon_nr)
			{
#if defined(FORTIS_HDBOX) \
 || defined(ATEVIO7500)
				case ICON_SPINNER:
				{
					if (mode == 0)  // vfd mode
					{
						on = data->data[4];
					}
					else
					{
						on = nuvoton->u.icon.on;
					}
					spinner_state.state = on;
					lastdata.icon_state[ICON_SPINNER] = on;
#if defined(FORTIS_HDBOX)
					if (on)
					{
						if (on == 1)
						{
							on = 100;  // set default value: 1 change/sec
						}
						spinner_state.period = on * 10;
						up(&spinner_state.sem);
					}
#elif defined(ATEVIO7500)
					if (on)
					{
						if (icon_state.state == 1)
						{
							dprintk(50, "%s Stop icon thread\n", __func__);
							icon_state.state = 0;
							do
							{
								msleep(250);
							}
							while (icon_state.status != ICON_THREAD_STATUS_HALTED);
							dprintk(50, "%s Icon thread stopped\n", __func__);
						}
						if (on == 1)  // handle default
						{
							on = 25;  // full cycle takes 16 * 250 ms = 4 sec
						}
						spinner_state.period = on * 10;
						up(&spinner_state.sem);
					}
					else
					{
						if (spinner_state.state == 1)
						{
							dprintk(50, "%s Stop spinner\n", __func__);
							spinner_state.state = 0;
							do
							{
								msleep(250);
							}
							while (spinner_state.status != ICON_THREAD_STATUS_HALTED);
							dprintk(50, "%s Spinner stopped\n", __func__);
						}
						dprintk(50, "%s Restart icon thread\n", __func__);
						icon_state.state = 1;
						up(&icon_state.sem);
					}
#endif
					res = 0;
					break;
				}
#endif  // ATEVIO7500 or FORTIS HD_BOX
				case ICON_MAX:
				{
#if defined(ATEVIO7500)
					if (spinner_state.state == 1)  // switch spinner off if on
					{
						dprintk(50, "%s Stop spinner\n", __func__);
						spinner_state.state = 0;
						do
						{
							msleep(250);
						}
						while (spinner_state.status != ICON_THREAD_STATUS_HALTED);
						dprintk(50, "%s Spinner stopped\n", __func__);
						dprintk(50, "%s Restart icon thread\n", __func__);
						icon_state.state = 1;
						up(&icon_state.sem);
					}

					for (i = ICON_MIN + 1; i < ICON_MAX; i++)
					{
						if (on != lastdata.icon_state[i])
						{
							on ? lastdata.icon_count++ : lastdata.icon_count--;
						}

						if (lastdata.icon_state[i] == 1 && !on)							
						{
							res |= nuvotonSetIcon(i, 0);
						}
							lastdata.icon_state[i] = on;
					}
					break;
#elif defined(FORTIS_HDBOX)
					if (spinner_state.state == 1)  // switch spinner off if on
					{
						dprintk(50, "%s Stop spinner\n", __func__);
						spinner_state.state = 0;
						do
						{
							msleep(250);
						}
						while (spinner_state.status != ICON_THREAD_STATUS_HALTED);
						dprintk(50, "%s Spinner stopped\n", __func__);
					}
					//fall through to:
#endif
					for (i = ICON_MIN + 1; i < ICON_MAX; i++)
					{
						res |= nuvotonSetIcon(i, on);
					}
					break;
				}
				default:  // (re)set a single icon
				{
#if defined(ATEVIO7500)
					if (on != lastdata.icon_state[icon_nr])
					{
						on ? lastdata.icon_count++ : lastdata.icon_count--;
					}
					lastdata.icon_state[icon_nr] = on;

					if (spinner_state.state == 0)  // if spinner off
					{
						if ((icon_state.state == !on) && on)  // and no icon thread yet but icon on
						{
							dprintk(50, "%s Start icon thread\n", __func__);
							icon_state.state = 1;
							up(&icon_state.sem);
						}
						if (!on)
						{
							res = nuvotonSetIcon(icon_nr, 0);
						}
					}
#elif defined(FORTIS_HDBOX)
					if (spinner_state.state == 1 && icon_nr >= ICON_Circ0 && icon_nr <= ICON_Circ8)
					{
						dprintk(50, "%s Stop spinner\n", __func__);
						spinner_state.state = 0;
						do
						{
							msleep(250);
						}
						while (spinner_state.status != ICON_THREAD_STATUS_HALTED);
						dprintk(50, "%s Spinner stopped\n", __func__);
						msleep(100);

					}
					res = nuvotonSetIcon(icon_nr, on);
#else
					dprintk(50, "%s Set single icon #%d to %d\n", __func__, icon_nr, on);
					res = nuvotonSetIcon(icon_nr, on);
#endif
					break;
				}
			}
#endif  // HS7110 (no display)
			mode = 0;
			break;
		}
		case VFDSTANDBY:
		{
			clear_display();
			dprintk(10, "Set deep standby mode, wake up time: (MJD= %d) - %02d:%02d:%02d (local)\n", (nuvoton->u.standby.time[0] & 0xff) * 256 + (nuvoton->u.standby.time[1] & 0xff),
				nuvoton->u.standby.time[2], nuvoton->u.standby.time[3], nuvoton->u.standby.time[4]);
			res = nuvotonSetStandby(nuvoton->u.standby.time);
			break;
		}
		case VFDSETPOWERONTIME:
		{
			dprintk(10, "Set wake up time: (MJD= %d) - %02d:%02d:%02d (local)\n", (nuvoton->u.standby.time[0] & 0xff) * 256 + (nuvoton->u.standby.time[1] & 0xff),
				nuvoton->u.standby.time[2], nuvoton->u.standby.time[3], nuvoton->u.standby.time[4]);
			res = nuvotonSetWakeUpTime(nuvoton->u.standby.time);
			break;
		}
		case VFDGETWAKEUPTIME:
		{
			dprintk(10, "Wake up time: (MJD= %d) - %02d:%02d:%02d (local)\n", (wakeup_time[0] & 0xff) * 256 + (wakeup_time[1] & 0xff),
				wakeup_time[2], wakeup_time[3], wakeup_time[4]);
			copy_to_user((void *)arg, &wakeup_time, 5);
			res = 0;
			break;
		}
		case VFDSETTIME:
		{
			if (nuvoton->u.time.time != 0)
			{
				dprintk(10, "Set frontpanel time to (MJD=) %d - %02d:%02d:%02d (local)\n", (nuvoton->u.time.time[0] & 0xff) * 256 + (nuvoton->u.time.time[1] & 0xff),
					nuvoton->u.time.time[2], nuvoton->u.time.time[3], nuvoton->u.time.time[4]);
				res = nuvotonSetTime(nuvoton->u.time.time);
			}
			break;
		}
		case VFDGETTIME:
		{
			char time[5];

			res = nuvotonGetTime(time);
			dprintk(10, "Get frontpanel time: (MJD=) %d - %02d:%02d:%02d (local)\n", (ioctl_data[0] & 0xff) * 256 + (ioctl_data[1] & 0xff),
				ioctl_data[2], ioctl_data[3], ioctl_data[4]);
			copy_to_user((void *)arg, &ioctl_data, 5);
			break;
		}
		case VFDGETWAKEUPMODE:
		{
			int wakeup_mode = -1;

			res = nuvotonGetWakeUpMode(&wakeup_mode);
			dprintk(10, "Wake up mode = %d\n", wakeup_mode);
			res |= copy_to_user((void *)arg, &wakeup_mode, 1);
			break;
		}
		case VFDDISPLAYCHARS:
		{
#if !defined (HS7110)
			if (mode == 0)
			{
				data->data[data->length]= 0;  // terminate string to show
				dprintk(10, "Write string (mode 0): [%s] (length = %d)\n", data->data, data->length);

				res = nuvotonWriteString(data->data, data->length);
			}
			else
			{
				dprintk(10, "Write string (mode 1): not supported!\n");
				mode = 0;  // go back to vfd mode
			}
#endif
			break;
		}
		case VFDDISPLAYWRITEONOFF:
		{
			res = nuvotonSetDisplayOnOff(nuvoton->u.light.onoff);
			mode = 0;  // go back to vfd mode
			break;
		}
		case VFDSETTIMEFORMAT:
		{
			const char *time_formatxt[2] = { "12h", "24h" };

			if (nuvoton->u.timeformat.format != 0)
			{
				nuvoton->u.timeformat.format = 1;
			}
			dprintk(10, "Set time format to: %s\n", time_formatxt[(int)nuvoton->u.timeformat.format]);
			nuvoton->u.timeformat.format |= 0x80;
			res = nuvotonSetTimeFormat(nuvoton->u.timeformat.format);
			mode = 0;  // go back to vfd mode
			break;
		}
		case VFDGETVERSION:
		{
			unsigned int data[2];
			res = nuvotonGetVersion(data);
			dprintk(10, "Boot loader version is %d.%02d\n", data[0] / 100, data[0] % 100);
			dprintk(10, "reseller ID is %08X\n", data[1]);
			res |= copy_to_user((void *)arg, &(data), sizeof(data));
			mode = 0;  // go back to vfd mode
			break;
		}
#if defined(VFDTEST)
		case VFDTEST:
		{
			res = nuvotonVfdTest(nuvoton->u.test.data);
			res |= copy_to_user((void *)arg, &(nuvoton->u.test.data), ((dataflag == 1) ? 10 : 2));
			mode = 0;	// go back to vfd mode
			break;
		}
#endif
		case 0x5305:
		{
			mode = 0;  // go back to vfd mode
			break;
		}
		case 0x5401:
		case VFDGETBLUEKEY:
		case VFDSETBLUEKEY:
		case VFDGETSTBYKEY:
		case VFDSETSTBYKEY:
		case VFDPOWEROFF:
		case VFDGETSTARTUPSTATE:
		case VFDSETTIME2:
		case VFDDISPLAYCLR:
		case VFDGETLOOPSTATE:
		case VFDSETLOOPSTATE:
		{
			printk("[nuvoton] Unsupported IOCTL 0x%x for this receiver.\n", cmd);
			mode = 0;
			break;
		}
		default:
		{
			printk("[nuvoton] Unknown IOCTL 0x%x.\n", cmd);
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
	.ioctl   = NUVOTONdev_ioctl,
	.write   = NUVOTONdev_write,
	.read    = NUVOTONdev_read,
	.open    = NUVOTONdev_open,
	.release = NUVOTONdev_close
};
// vim:ts=4
