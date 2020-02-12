/*
 * fortis_4g_fp_et6226.h
 *
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add irq handling
 * (c) 2014 Audioniek: rewritten for Etek ET6226 / ETC FD650
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
 * 20141111 Audioniek      Start of work, based on nuvoton_asc.h and the original driver
 *                         for Shenzen Titan Micro Electronics TM1668 written by Pawel
 *                         Moll <pawel.moll@st.com> of STM. Thank you Pawel and nuvoton
 *                         authors.
 * 20200122 Audioniek      Simplify brightness control.
 *
 ****************************************************************************************/

#ifndef _4g_fp_et6226_h
#define _4g_fp_et6226_h

#define LED_GREEN 1
#define LED_LOGO  2

#define MAX_BRIGHT 7

#if defined(DP7001)
#define GPIO_LED_GREEN stm_gpio(13, 2)
#define GPIO_LED_LOGO  stm_gpio(15, 7)
#elif defined(FOREVER_2424HD)
#define GPIO_LED_GREEN stm_gpio(13, 2) //TODO: find correct pin number
#define GPIO_LED_LOGO  stm_gpio(15, 7) //TODO: find correct pin number
#elif defined (FOREVER_NANOSMART)
#define GPIO_LED_GREEN stm_gpio(13, 2) //TODO: find correct pin number
#endif

//Defines for ET6226
#define ET6226_DRIVER_NAME        "et6226_kbd"

#define ET6226_I2C_BUSNUM         4

#define ET6226_DSP_ADDR           0x34
#define ET6226_GET_KEYS_ADDR      0x27

#define ET6226_CTL_ADDR           0x24
#define ET6226_DISPLAY_ENABLE     0b00000001
#define ET6226_SLEEP_MODE         0b00000100
#define ET6226_EIGHT_SEGMENT_MODE 0b00000000
#define ET6226_SEVEN_SEGMENT_MODE 0b00001000

#define ET6226_CTL_DATA(display_enable, sleep_mode, display_mode, brightness) \
	(0x00 | \
	(display_enable ? (ET6226_DISPLAY_ENABLE & !sleep_mode) : 0x0) | \
	(sleep_mode ? (ET6226_SLEEP_MODE & !display_enable) : 0x0) | \
	(display_mode ? (ET6226_SEVEN_SEGMENT_MODE && display_mode) : ET6226_EIGHT_SEGMENT_MODE) | \
	((brightness & 0x07) << 4))
// NOTE: display_enable and sleep mode are not allowed to be "1" at the same time

#define ET6226_KEYBOARD_BUFFER_SIZE 1
#define ET6226_DISPLAY_MAX_DIGITS   4
#define KEY_TOGGLE_MASK             0b01000000

struct et6226_key
{
	u8 value; /* as read from "keys" attribute */
	int code; /* input event code (KEY_*) */
	char *desc;
};

struct et6226_platform_data
{
	/* I2C interface */
	struct i2c_client *client;

	/* LEDs */
	u8 led;

	/* Keyboard */
	struct input_dev *keyboard;
	unsigned int key_prev;
	unsigned int key_code;
	unsigned int key_press;
	char *key_desc;
	int keys_num;
	struct et6226_key *keys;
	struct delayed_work keys_work;
	unsigned int keys_poll;  /* ms */
	char *driver_name;

	/* Display control */
	u8 ctl_reg;
	int sleep_mode;
	int display_mode;
	int on;
	int brightness;
	unsigned char *char_tbl;
	const char *text;  /* initial display text */
};

struct et6226_chip
{
	spinlock_t lock;  /* access lock */

	/* I2C interface */
	struct i2c_client *client;

	/* Display control */
	u8 ctl_reg;
	int on;
	int sleep_mode;
	int display_mode;
	int brightness;
	unsigned char *char_tbl;
	const char *text;  /* initial display text */

	/* Keyboard */
	struct input_dev *keyboard;
	unsigned int key_prev;
	unsigned int key_code;
	unsigned int key_press;
	char *key_desc;
	int keys_num;
	struct et6226_key *keys;
	struct delayed_work keys_work;
	unsigned int keys_poll;
	char *driver_name;
};
#endif  // _4g_fp_et6226_h
// vim:ts=4
