/****************************************************************************
 *
 * opt9600_fp_char.h
 *
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add irq handling
 * (c) 2020 Audioniek: ported to Opticum HD 9600 (TS)
 *
 * Largely based on cn_micom, enhanced and ported to Opticum HD 9600 (TS)
 * by Audioniek.
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
 * 20090306 W.Y.Yi          Original Crenova Micom version
 * 20201222 Audioniek       Initial version, based on cn_micom.
 *
 ****************************************************************************/
#ifndef __OPT9600_CHAR_H__	// {
#define __OPT9600_CHAR_H__


/****************************************************************************

  << 7-Segment display (Opticum HD 9600 mini) >>
  Model : MINILINE
  Front Micom : Version 4.xx.xx

        a
    ---------
   |         |
   |         |
 f |         | b
   |         |
   |    g    |
    ---------
   |         |
   |         |
 e |         | c
   |         |
   |         |
    ---------
        d
                           7 6 5 4  3 2 1 0 
 Segement to bit mapping: (x|g|f|e  d|c|b|a)
 Bit 7 is aways zero.

*****************************************************************************/

#define _7SEG_NULL		"\x00"
#define _7SEG_DASH		"\x40"
#define _7SEG_NUM_0		"\x3F"
#define _7SEG_NUM_1		"\x06"
#define _7SEG_NUM_2		"\x5B"
#define _7SEG_NUM_3		"\x4F"
#define _7SEG_NUM_4		"\x66"
#define _7SEG_NUM_5		"\x6D"
#define _7SEG_NUM_6		"\x7D"
#define _7SEG_NUM_7		"\x07"
#define _7SEG_NUM_8		"\x7F"
#define _7SEG_NUM_9		"\x6F"
#define _7SEG_CH_A		"\x5F"
#define _7SEG_CH_B		"\x0F"
#define _7SEG_CH_C		"\x39"
#define _7SEG_CH_D		"\x5E"
#define _7SEG_CH_E		"\x79"
#define _7SEG_CH_F		"\x71"
#define _7SEG_CH_G		"\x3D"
#define _7SEG_CH_H		"\x76"
#define _7SEG_CH_I		"\x06"
#define _7SEG_CH_J		"\x1E"
#define _7SEG_CH_K		"\x70"
#define _7SEG_CH_L		"\x38"
#define _7SEG_CH_M		"\x55"
#define _7SEG_CH_N		"\x54"
#define _7SEG_CH_O		"\x5C"
#define _7SEG_CH_P		"\x73"
#define _7SEG_CH_Q		"\x3F"
#define _7SEG_CH_R		"\x77"
#define _7SEG_CH_S		"\x6D"
#define _7SEG_CH_T		"\x01"
#define _7SEG_CH_U		"\x1C"
#define _7SEG_CH_V		"\x30"
#define _7SEG_CH_W		"\x6A"
#define _7SEG_CH_X		"\x80"
#define _7SEG_CH_Y		"\x6E"
#define _7SEG_CH_Z		"\x5B"

/****************************************************************************

  << 14-Segment display (Opticum HD 9600 Prima) >>
  Model 		: FLEXLINE, ECOLINE
  Front Micom : Version 3.xx.xx

          0
     -----------
   |\     |     /|
   | \    |8   / |
  5|  \7  |   /9 | 1
   |   \  |  /   |
   | 6  \ | /  10|
     ---- * ----    Middle point is bit 13
   |    / | \    |
   | 12/  |  \11 |
  4|  /   |   \  | 2
   | /    |8   \ |
   |/     |     \|
     -----------
          3

 Data format : (7|6|5|4  3|2|1|0) (x|x|13|12  11|10|9|8)
 Bits marked with x should always be zero.

 NOTE on Opticum HS 9600 non-Prima models: these early versions cannot
 control the individual segments in the display, but instead have a fixed
 set of character definitions built into the front processor (see below).

*****************************************************************************/

#define _14SEG_NULL      "\x00\x00"
#define _14SEG_DASH      "\x40\x24"
#define _14SEG_NUM_0     "\x3F\x32"
#define _14SEG_NUM_1     "\x06\x00"
#define _14SEG_NUM_2     "\x5B\x24"
#define _14SEG_NUM_3     "\x4F\x24"
#define _14SEG_NUM_4     "\x66\x24"
#define _14SEG_NUM_5     "\x69\x28"
#define _14SEG_NUM_6     "\x7D\x24"
#define _14SEG_NUM_7     "\x07\x00"
#define _14SEG_NUM_8     "\x7F\x24"
#define _14SEG_NUM_9     "\x6F\x24"
#define _14SEG_CH_A      "\x77\x24"
#define _14SEG_CH_B      "\x0F\x25"
#define _14SEG_CH_C      "\x39\x00"
#define _14SEG_CH_D      "\x0F\x21"
#define _14SEG_CH_E      "\x79\x24"
#define _14SEG_CH_F      "\x71\x24"
#define _14SEG_CH_G      "\x3D\x24"
#define _14SEG_CH_H      "\x76\x24"
#define _14SEG_CH_I      "\x09\x21"
#define _14SEG_CH_J      "\x1E\x00"
#define _14SEG_CH_K      "\x70\x2A"
#define _14SEG_CH_L      "\x38\x00"
#define _14SEG_CH_M      "\xB6\x22"
#define _14SEG_CH_N      "\xB6\x28"
#define _14SEG_CH_O      "\x3F\x00"
#define _14SEG_CH_P      "\x73\x24"
#define _14SEG_CH_Q      "\x3F\x08"
#define _14SEG_CH_R      "\x73\x2C"
#define _14SEG_CH_S      "\x6D\x24"
#define _14SEG_CH_T      "\x01\x21"
#define _14SEG_CH_U      "\x3E\x00"
#define _14SEG_CH_V      "\x30\x32"
#define _14SEG_CH_W      "\x36\x38"
#define _14SEG_CH_X      "\x80\x3A"
#define _14SEG_CH_Y      "\x6E\x24"
#define _14SEG_CH_Z      "\x09\x32"

// 7 segment equivalents for each individual segment
#define _14SEG_SPECIAL_1 "\x10\x00"
#define _14SEG_SPECIAL_2 "\x20\x00"
#define _14SEG_SPECIAL_3 "\x01\x00"
#define _14SEG_SPECIAL_4 "\x02\x00"
#define _14SEG_SPECIAL_5 "\x04\x00"
#define _14SEG_SPECIAL_6 "\x08\x00"
#define _14SEG_SPECIAL_7 "\x40\x24"

//러시아어 이름은 로마자 표기 임 (ref wiki)
#if defined(_BUYER_SATCOM_RUSSIA) || defined(_BUYER_SOGNO_GERMANY)
#define _14SEG_RUS_a     "\x77\x24"
#define _14SEG_RUS_b     "\x7d\x24"
#define _14SEG_RUS_v     "\x79\x2A"
#define _14SEG_RUS_g     "\x31\x00"
#define _14SEG_RUS_d     "\x0F\x21"
#define _14SEG_RUS_ye    "\x79\x24"
#define _14SEG_RUS_yo    "\x79\x24"
#define _14SEG_RUS_zh    "\xC0\x3F"
#define _14SEG_RUS_z     "\x09\x2A"
#define _14SEG_RUS_i     "\x36\x32"
#define _14SEG_RUS_ekra  "\x36\x32"
#define _14SEG_RUS_k     "\x70\x2A"
#define _14SEG_RUS_el    "\x06\x32"
#define _14SEG_RUS_m     "\xB6\x22"
#define _14SEG_RUS_n     "\x76\x24"
#define _14SEG_RUS_o     "\x3F\x00"
#define _14SEG_RUS_p     "\x37\x00"
#define _14SEG_RUS_r     "\x73\x24"
#define _14SEG_RUS_s     "\x39\x00"
#define _14SEG_RUS_t     "\x01\x21"
#define _14SEG_RUS_u     "\x6E\x24"
#define _14SEG_RUS_f     "\x3F\x21"
#define _14SEG_RUS_kh    "\x80\x3A"
#define _14SEG_RUS_ts    "\x38\x21"
#define _14SEG_RUS_ch    "\x66\x24"
#define _14SEG_RUS_sh    "\x3E\x21"
#define _14SEG_RUS_shch  "\x3E\x21"
#define _14SEG_RUS_tvyor "\x0D\x25"
#define _14SEG_RUS_y     "\x7E\x21"
#define _14SEG_RUS_myah  "\x0C\x25"
#define _14SEG_RUS_eobeo "\x0F\x24"
#define _14SEG_RUS_yu    "\x76\x2A"
#define _14SEG_RUS_ya    "\x27\x34"

#endif // _BUYER_SATCOM_RUSSIA

/****************************************************************************
 *
 * Character mapping of the Opticum HD 9600 non-Prima models
 *
 * The front processor of the non-Prima models cannot control individual
 * display segments, but has a fixed data to character assignment as follows
 * (thnx to corev for this info,
 * see https://www.aaf-digital.info/forum/node/76786):
 *
 * 0x00 = ' '  | 0x10 = 'E'  | 0x20 = 'U'  | 0x30 =      |
 * 0x01 =      | 0x11 = 'F'  | 0x21 = 'V'  | 0x31 =      |
 * 0x02 = '0'  | 0x12 = 'G'  | 0x22 = 'W'  | 0x32 =      |
 * 0x03 = '1'  | 0x13 = 'H'  | 0x23 = 'X'  | 0x33 =      |
 * 0x04 = '2'  | 0x14 = 'I'  | 0x24 = 'Y'  | 0x34 =      |
 * 0x05 = '3'  | 0x15 = 'J'  | 0x25 = 'Z'  | 0x35 =      |
 * 0x06 = '4'  | 0x16 = 'K'  | 0x26 =      | 0x36 =      |
 * 0x07 = '5'  | 0x17 = 'L'  | 0x27 =      | 0x37 =      |
 * 0x08 = '6'  | 0x18 = 'M'  | 0x28 =      | 0x38 =      |
 * 0x09 = '7'  | 0x19 = 'N'  | 0x29 =      | 0x39 =      |
 * 0x0a = '8'  | 0x1a = 'O'  | 0x2a =      | 0x3a =      |
 * 0x0b = '9'  | 0x1b = 'P'  | 0x2b =      | 0x3b =      |
 * 0x0c = 'A'  | 0x1c = 'Q'  | 0x2c =      | 0x3c =      |
 * 0x0d = 'B'  | 0x1d = 'R'  | 0x2d =      | 0x3d =      |
 * 0x0e = 'C'  | 0x1e = 'S'  | 0x2e =      | 0x3e =      |
 * 0x0f = 'D'  | 0x1f = 'T'  | 0x2f =      | 0x3f =      |
 *
 ****************************************************************************/

#endif	// __OPT9600_CHAR_H__
// vim:ts=4
