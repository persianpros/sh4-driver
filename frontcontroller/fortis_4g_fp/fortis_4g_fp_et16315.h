/*
 * fortis_4g_fp_et16315.h
 *
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add irq handling
 * (c) 2014-2019 Audioniek: rewritten for Etek ET16315 / Princeton PT6315
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
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 20141218 Audioniek      Start of work, based on nuvoton_asc.h and the original driver
 *                         for Shenzen Titan Micro Electronics TM1668 written by Pawel
 *                         Moll <pawel.moll@st.com> of STM. Thank you Pawel and nuvoton
 *                         authors.
 * 20191208 Audioniek      FOREVER_3434HD model added.
 *
 ****************************************************************************************/

#ifndef _4g_fp_et16315_h
#define _4g_fp_et16315_h

/* Uncomment next line to enable text
   scrolling in low level VFD driver */
//#define VFD_SCROLL     1

// Logo LEDs are driven from a PIO pin
#if defined(FOREVER_3434HD) \
 || defined(EP8000) \
 || defined(EPP8000) \
 || defined(GPV8000)
#define GPIO_LED_LOGO  stm_gpio(15, 7)
#elif defined(FOREVER_9898HD)
// NOTE: FOREVER_9898HD does not have a lighted logo, but red LED is GPIO driven?
//#define GPIO_LED_RED  stm_gpio(3, 1)  //TODO: GPIO # to be determined
#endif

// LED wiring on ET16315, all VFD models, except FOREVER_9898HD
// Power red:  from ET16315 pin 1
// Additional on FOREVER_3434HD only:
// Power blue: from ET16315 pin 2

// Defines for ET16315
#define ET16315_DIN    stm_gpio(14, 4)
#define ET16315_DOUT   stm_gpio(14, 5)
#define ET16315_CLK    stm_gpio(14, 6)
#define ET16315_STB    stm_gpio(14, 7)

#define ET16315_DRIVER_NAME "et16315_kbd"

/* ET16315 command bytes
1. Display set command: 0b00XXnnnn
 nnnn  Display mode
 ----------------------------
 0000   4 digits, 24 segments
 0001   5 digits, 23 segments
 0010   6 digits, 22 segments
 0011   7 digits, 21 segments
 0100   8 digits, 20 segments <- FP has 8 digits, so command is 0b00XX0100 (0x04)
 0101   9 digits, 19 segments
 0110  10 digits, 18 segments
 0111  11 digits, 17 segments
 1XXX  12 digits, 16 segments
*/
#define ET16315_CMD1_SET_DISPLAY_MODE(mode) \
		(0x00 | ((mode) & 0xf))


#define ET16315_TEST_MODE      0b00001000  // -> option = + 0x08
#define ET16315_FIXED_ADDR     0b00000100  // -> option = + 0x04
#define ET16315_AUTO_ADDR_INC  0b00000000  // -> option = + 0x00
#define ET16315_CMD_WRITE_DATA 0b00000000  // -> option = + 0x00
#define ET16315_CMD_SET_LED    0b00000001  // -> option = + 0x01
#define ET16315_CMD_READ_KEY   0b00000010  // -> option = + 0x02
#define ET16315_CMD_DONTCARE   0b00000011  // not used

/*
2. Data setting command: 0b01XXTAnn
 T: 0=normal mode
    1=test mode
 A: 0=Auto display addres increment
    1=Fixed address
 nn  Mode
 00  Write display data (three bytes / digit) -> 24 bytes for the whole display of 8 digits
 01  Write LED port (next byte written controls output pins: bit=1 is LED on)
 10  Read key scan data (read four bytes of key data)
 11  Not used
*/
#define ET16315_CMD2_DATA_SET(test_mode, fixed_address, command) \
		(0x40 | \
		(test_mode ? ET16315_TEST_MODE : 0x00) | \
		(fixed_address ? ET16315_FIXED_ADDR : ET16315_AUTO_ADDR_INC) | \
		(command & 0x3))

/*
3. Set address command: 0b11nnnnnn
 nnnnnn is binary display address
 digit * 3 is address of LB data (1st display data byte)
 (digit * 3) + 1 is address of MB data (2nd display data byte)
 (digit * 3) + 2 is address of HB data (3rd display data byte)
*/
#define ET16315_CMD3_SET_ADDRESS(address) \
		(0xc0 | (address & 0x3f))


#define ET16315_DISPLAY_ENABLE 0b00001000

/*
4. Display control command: 0b10XXDbbb -> CMD = 0x80 + options
 D: 0=Display off
    1=Display on
 bbb = brightness level, 000 is lowest
*/
#define ET16315_CMD4_DISPLAY_CONTROL(on, brightness) \
		(0x80 | \
		(on ? ET16315_DISPLAY_ENABLE : 0x0) | \
		(brightness & 0x7))

#define ET16315_KEYBOARD_BUFFER_SIZE 1
#define ET16315_DISPLAY_MAX_DIGITS   12
#define ET16315_DISPLAY_BUFFER_SIZE  ET16315_DISPLAY_MAX_DIGITS * 3

struct et16315_char
{
	u8 value0;
	u8 value1;
	u8 value2;  // not needed with 15 segment display, always zero
};

struct et16315_key
{
	u32  mask;  /* as read from "keys" attribute */
	int  code;  /* input event code (KEY_*) */
	char *desc;
};

struct et16315_platform_data
{
	/* Wiring information */
	unsigned gpio_din, gpio_dout, gpio_clk, gpio_stb;

	enum
	{
		et16315_config_4_digits_24_segments = 4,
		et16315_config_5_digits_23_segments,
		et16315_config_6_digits_22_segments,
		et16315_config_7_digits_21_segments,
		et16315_config_8_digits_20_segments,
		et16315_config_9_digits_19_segments,
		et16315_config_10_digits_18_segments,
		et16315_config_11_digits_17_segments,
		et16315_config_12_digits_16_segments
	} digits;

	/* LEDs */
	u8 led;

	/* Keyboard */
	int    keys_num;
	struct et16315_key *keys;
	int    keys_poll;  /* ms */
	char   *driver_name;

	/* Display control */
	int        brightness;  /* initial value, 0 (lowest) - 7 (max) */
	int        on;  /* initial value for display enable */
	struct     et16315_char *char_tbl;
	const char *text;  /* initial display text */
};

struct et16315_chip
{
	spinlock_t lock;  /* access lock */

	unsigned gpio_din, gpio_dout, gpio_clk, gpio_stb;  /* Wiring info */

	/* VFD display */
	int    digits;
	int    on;  // state of display enable
	int    brightness;
	u8     last_display[ET16315_DISPLAY_BUFFER_SIZE];
	struct et16315_char *char_tbl;

	/* LEDs */
	u8 led;

	/* Keyboard */
	struct input_dev *keyboard;
	u32    key_prev;
	int    keys_num;
	struct et16315_key *keys;
	struct delayed_work keys_work;
	int    keys_poll;
	char   *driver_name;
};
#endif  // _4g_fp_et16315_h
// vim:ts=4
