/******************************************************************************
 *
 * fortis_4g_fp_et6226.c
 *
 * Frontpanel driver for Fortis 4th generation receivers, I2C core part
 *
 * Supports: I2C connected Etek ET6226 / ETC FD650 LED/keyboard driver IC
 *
 * Based on code written by:
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add nuvoton irq handling
 *
 * (c) 2014-2019 Audioniek: rewritten for Fortis 4th generation receivers
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
 * Justification for this (separate) Fortis front panel module:
 * By tradition, the first 3 generations of Fortis HD receivers used the
 * nuvoton module for their front panel support. This module handles the front
 * panel display, the front panel keyboard, clock for the timer(s) and the
 * remote control.
 *
 * The 4th generation of Fortis receivers no longer have a separate front-
 * processor. The display and front panel keyboard are handled by non-
 * intelligent ICs, the remote control is handled by lirc and the timers by
 * the RTC in the SoC.
 * Deep standby mode used to be power down everything except the front
 * processor, on the 4G models the main board is put in "hibernation".
 * The hardware architecture is quite different, so that is reason one.
 *
 * The second reason is that the nuvoton code has become quite cluttered
 * because of the number and variants of receivers supported. Adding these
 * new receivers having a different hardware approach to the nuvoton code
 * would probably clutter it up even more.
 *
 *****************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * ----------------------------------------------------------------------------
 * 20141108 Audioniek       Start of work.
 * 20141108 Audioniek       All code for older kernel versions removed.
 * 20150205 Audioniek       FOREVER_2424HD model added.
 * 20200122 Audioniek       Simplify brightness control.
 *
 *****************************************************************************/
//TODO: split display on/off and brightness in stead of combining them with brightness=0 is display off

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/termbits.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/gpio.h>
#include <linux/stm/pio.h>
//#include <linux/stm/gpio.h>
#include <linux/workqueue.h>
//#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/poll.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>

#include "fortis_4g_fp.h"
#include "fortis_4g_fp_et6226.h"

#if defined(FOREVER_NANOSMART) || defined(DP7001) || defined(FOREVER_2424HD)
#define DISP_SIZE 4
#else
#warning Architecture not supported!
#endif

static struct et6226_chip *et6226_data;  // global values for IOCTL routines

/****************************************************************************************
 *
 * Character table for 4 digit 7-segment LED display
 *
 ****************************************************************************************
 *
 * In the Fortis DP7001/7050 front panel the ET6226 driver IC is wired as follows:
 * (TODO/assumption: FOREVER_NANOSMART is the same, apart from decimal points)
 *
 *       a(0)
 *      ------
 *     |      |
 * f(5)|      |b(1)
 *     | g(6) |
 *      ------
 *     |      |
 * e(4)|      |c(2)
 *     |      |
 *      ------  o (7)
 *       d(3)
 *
 * KS1 -> segment a (0x01)
 * KS2 -> segment b (0x02)
 * KS3 -> segment c (0x04)
 * KS4 -> segment d (0x08)
 * KS5 -> segment e (0x10)
 * KS6 -> segment f (0x20)
 * KS7 -> segment g (0x40)
 * KSP -> decimal point or colon (0x80)
 *
 * Connection of decimal points and colon is:
 * GR0 + KSP -> DP of digit 2 (not on FOREVER_NANOSMART)
 * GR1 + KSP -> colon in center
 * GR2 + KSP -> DP of digit 3 (not on FOREVER_NANOSMART)
 * GR3 + KSP -> DP of digit 4 (not on FOREVER_NANOSMART)
 * The DP of digit 1 is not connected and cannot be controlled
 *
 * GR0 is connected to the leftmost digit 1, GR3 to the rightmost
 *
 *
 * The characters below are the same as those used on the HS7119,
 * HS7810A and HS7819 in the nuvoton driver.
 * Order must be ASCII, starting at space (0x20).
 */
static unsigned char et6226_fp_chars[] =
{
	//pgfedcba    pgfedcba    pgfedcba    pgfedcba  segments
	//76543210    76543210    76543210    76543210  bit numbers
	0b00000000, 0b00000110, 0b00100010, 0b01011100, // '  - #'
	0b01101101, 0b01010010, 0b01111101, 0b00000010, // '$' - 0x27
	0b00111001, 0b00001111, 0b01110110, 0b00100110, // '( - +'
	0b00001100, 0b01000000, 0b00001000, 0b01010010, // ', - /'
	0b00111111, 0b00000110, 0b01011011, 0b01001111, // '0 - 3'
	0b01100110, 0b01101101, 0b01111101, 0b00000111, // '4 - 7'
	0b00111111, 0b01101111, 0b00010000, 0b00001100, // '8 - ;'
	0b01011000, 0b00101000, 0b01111101, 0b01010011, // '< - ?'
	0b01111011, 0b01101111, 0b01111100, 0b01011000, // '@ - C'
	0b01011110, 0b01111001, 0b01110001, 0b00111101, // 'D - G'
	0b01110110, 0b00000110, 0b00011110, 0b01111010, // 'H - K'
	0b00111000, 0b01010101, 0b00110111, 0b00111111, // 'L - O'
	0b01110011, 0b01100111, 0b01110111, 0b01101101, // 'P - S'
	0b01111000, 0b00111110, 0b00110111, 0b01111110, // 'T - W'
	0b01110110, 0b01100110, 0b01011011, 0b00111001, // 'X - ['
	0b00110100, 0b00001111, 0b00100011, 0b00010000, // 0x5c -'_'
	0b00100000, 0b01011111, 0b01111100, 0b01011000, // '` - c'
	0b01011110, 0b01111011, 0b01110001, 0b01101111, // 'd - g'
	0b01110100, 0b00000100, 0b00001110, 0b01111000, // 'h - k'
	0b00111000, 0b01010101, 0b01010100, 0b01011100, // 'l - o'
	0b01110011, 0b01100111, 0b00001110, 0b01101101, // 'p - s'
	0b01111000, 0b00011100, 0b00011100, 0b00011100, // 't - w'
	0b01110110, 0b01100111, 0b01011011, 0b00111001, // 'x' - 0x7c
	0b00000110, 0b00001111, 0b00000100, 0b01011100  // '|' - 0x7f
};

/****************************************************************************************
 *
 * Front panel keyboard definitions
 *
 ****************************************************************************************/
/* Keyboard polling state
 0 = stopped
 1 = normal polling
 2 = stop requested
*/
static int polling = 1;
static struct workqueue_struct *keyboard_workqueue;
#if defined(FOREVER_NANOSMART) || defined(DP7001) || defined(FOREVER_2424HD)
/*
  The ET6226 provides following values on key presses:
          GR0      GR1       GR2      GR3
  KS1(a)  01000100 01000101 01000110 01000111
  KS2(b)  01001100 01001101 01001110 01001111
  KS3(c)  01010100 01010101 01010110 01010111
  KS4(d)  01011100 01011101 01011110 01011111
  KS5(e)  01100100 01100101 01100110 01100111
  KS6(f)  01101100 01101101 01101110 01101111
  KS7(g)  01110100 01110101 01110110 01110111
  KS8(p)  01111100 01111101 01111110 01111111

  bit10  = digit number
  bit2   = always 1
  bit543 = segment number - 1
  bit6   = 1 = key pressed, 0 = not pressed
  bit7   = always 0

  On repeated polls after key release with no new key pressed,
  ET6226 returns bit6=0, bit543X01=last key scanned

  CHDN key is between KS2 and GR0 -> keycode = 0x4c/0x0c (0b0X001100)
  CHUP key is between KS3 and GR0 -> keycode = 0x54/0x14 (0b0X010100)
  PWR key is connected to GPIO pin 3.1 (not to the ET6226)
*/
static struct et6226_key front_panel_keys[] =
{
	{ 0x54, KEY_UP,   "Channel Up" },
	{ 0x4c, KEY_DOWN, "Channel Down" }
};
#else
#warning No front panel keys defined!
#endif

/****************************************************************************************
 *
 * Initial values and settings for ET6226
 *
 ****************************************************************************************/
static struct i2c_board_info et6226_i2c_info =
{
	/* Wiring info */
	I2C_BOARD_INFO("et6226", ET6226_CTL_ADDR),
};

static struct et6226_chip et6226_init_values =
{
	/* Display control */
	.ctl_reg = ET6226_DISPLAY_ENABLE + ET6226_EIGHT_SEGMENT_MODE,
	.brightness = 8, /* 0=display off, 8=highest */
	.char_tbl = et6226_fp_chars,
#if defined(DP7001)
	.text = "7001", /* initial display text */
#elif defined(FOREVER_2424HD)
	.text = "7050",
#elif defined(FOREVER_NANOSMART)
	.text = "6010",
#else
	.text = "----",
#endif
	/* Keyboard */
	.keys_num = ARRAY_SIZE(front_panel_keys),
	.keys = front_panel_keys,
	.keys_poll = 80, /* ms, 80 is worst case value for ET6226 */
	.driver_name = "ET6226 front panel keyboard"
};

/****************************************************************************************
 *
 * I2C routines for ET6226
 *
 ****************************************************************************************/
int et6226_i2c_write(struct i2c_client *client, unsigned char address, unsigned char byte)
{
	int ret = 0;

	struct i2c_msg msg = { .addr = address, .flags = 0, .buf = &byte, .len = 1 };

//	dprintk(100, "%s > Write 0x%02x to i2c address 0x%02x\n", __func__, byte, msg.addr);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1)
	{
		dprintk(1, "Error writing 0x%02x to i2c address 0x%02x\n", byte, client->addr);
		return -EFAULT;
	}

//	dprintk(100, "< OK, %d message(s) sent\n", ret);
	return ret;
}

unsigned char et6226_i2c_read(struct i2c_client *client, unsigned char address, unsigned char *data)
{
	int ret;

	struct i2c_msg msg[] = {{ .addr = address, .flags = I2C_M_RD, .buf = data, .len = 1 }};

//	dprintk(100, "%s > Read i2c address 0x%02x\n", __func__, msg[0].addr);

	ret = i2c_transfer(client->adapter, msg, 1);

	if (ret != 1)
	{
		dprintk(1, "Error reading data from address 0x%02x; status=0x%02x\n", msg[0].addr, ret);
		return -EREMOTEIO;
	}
//	dprintk(100, "< OK, %d message(s) sent, data = 0x%02x\n", ret, (unsigned int)*data);
	return 0;
}

#if 0
/* Keyboard input */
static void et6226_keys_poll(struct work_struct *work)
{
/*
  ET6226 provides following values on key presses:
          GR0      GR1       GR2      GR3
  KS1(a)  01000100 01000101 01000110 01000111
  KS2(b)  01001100 01001101 01001110 01001111
  KS3(c)  01010100 01010101 01010110 01010111
  KS4(d)  01011100 01011101 01011110 01011111
  KS5(e)  01100100 01100101 01100110 01100111
  KS6(f)  01101100 01101101 01101110 01101111
  KS7(g)  01110100 01110101 01110110 01110111
  KS8(p)  01111100 01111101 01111110 01111111

  bit10  = digit number
  bit2   = always 1
  bit543 = segment number - 1
  bit6   = key up or down; 1 = pressed, 0 = not pressed?
  bit7   = always 0

  On repeated polls with no new key pressed, ET6226 returns bit6=0, bit543X01=last key scanned

  CHUP key is between KS2 and GR0 -> keycode = 0x4c/0x0c (0b0X001100)
  CHDN key is between KS3 and GR0 -> keycode = 0x54/0x14 (0b0X010100)
*/
	struct et6226_chip *chip = container_of(work, struct et6226_chip, keys_work.work);
	unsigned char keys, diff;
	int i;

	spin_lock(&chip->lock);

	et6226_i2c_read(chip->client, ET6226_GET_KEYS_ADDR, &keys);

	spin_unlock(&chip->lock);

	diff = ((keys ^ chip->key_prev) & 0b01000000);

	if (diff) //key was pressed or released
	{
		for (i = 0; i < chip->keys_num; i++)
		{
			struct et6226_key *key = &chip->keys[i];

			if ((keys | 0b01000000) == key->value)
			{
				input_event(chip->input, EV_KEY, key->code, (keys & 0b01000000));
				input_sync(chip->input);
				dprintk(10, "Key = %s, state = %d\n", key->desc, (diff >> 6)); 
			}
		}
	}
	chip->key_prev = keys;

	schedule_delayed_work(&chip->keys_work, chip->keys_poll);
}
#endif

#if 0
static void et6226_set_sleep_mode(struct et6226_chip *chip, char sleep)
{

	if (sleep && chip->display_enable)
	{
		chip->display_enable = 0; //switch display off
		chip->sleep_mode = ET6226_SLEEP_MODE;
		dprintk(5, "Put ET6226 to sleep\n");
	}
	else
	{
		chip->sleep_mode = 0;
		dprintk(5, "Wake up ET6226\n");
	}
	et6226_i2c_write(chip->client, ET6226_CTL_ADDR, ET6226_CTL_DATA(chip->display_enable, chip->sleep_mode, chip->display_mode, chip->brightness));
}

static int et6226_set_light(struct et6226_chip *chip, char light)
{
	int ret  = -1;
	int display_enable;

	if (chip->sleep_mode)
	{
		dprintk(5, "Sleep mode set, no change\n");
	}
	else
	{
		if (light)
		{
			display_enable = ET6226_DISPLAY_ENABLE; //switch display on
			dprintk(5, "Display on\n");
		}
		else
		{
			display_enable = 0; //switch display off
			dprintk(5, "Display off\n");
		}
	}
	chip->display_enable = display_enable;

	ret = et6226_i2c_write(chip->client, ET6226_CTL_ADDR, ET6226_CTL_DATA(chip->display_enable, chip->sleep_mode, chip->display_mode, chip->brightness));
	return ret;
}
#endif

/* Display routines */
static void et6226_clear(struct et6226_chip *chip)
{
	int i;

	for (i = 0; i < ET6226_DISPLAY_MAX_DIGITS; i++)
	{
		et6226_i2c_write(chip->client, ET6226_DSP_ADDR + i, 0);
	}
}

/****************************************************************************************
 *
 * LED routines
 *
 ****************************************************************************************/
static void init_leds(void)
{
	dprintk(100, "%s >\n", __func__);
	/* Set GPIO direction to out and switch LEDs off */
	gpio_request(GPIO_LED_GREEN, "LED_GREEN");
#if defined(DP7001) || defined(FOREVER_2424HD)
	gpio_direction_output(GPIO_LED_GREEN, 1); //drive of LED_GREEN is inverted
	gpio_request(GPIO_LED_LOGO, "LED_LOGO");
	gpio_direction_output(GPIO_LED_LOGO, 1); //drive of LED_LOGO is inverted
#else
	gpio_direction_output(GPIO_LED_GREEN, 0);
#endif
	msleep(200);
	/* Switch LEDs on */
	fortis_4gSetLED(LED_GREEN, 1);
#if defined(DP7001) || defined(FOREVER_2424HD)
	fortis_4gSetLED(LED_LOGO, 1);
#endif
	msleep(200);
	/* Switch LEDs off */
	fortis_4gSetLED(LED_GREEN, 0);
#if defined(DP7001) || defined(FOREVER_2424HD)
	fortis_4gSetLED(LED_LOGO, 0);
#endif
	msleep(200);
	/* Initial: LOGO on, rest off */
#if defined(DP7001) || defined(FOREVER_2424HD)
	fortis_4gSetLED(LED_LOGO, 1);
#endif
	dprintk(100, "%s <\n", __func__);
}

static void cleanup_leds(void)
{
	gpio_free(GPIO_LED_GREEN);
#if defined(DP7001) || defined(FOREVER_2424HD)
	gpio_free(GPIO_LED_LOGO);
#endif
}

/****************************************************************************************
 *
 * Front panel keyboard device routines
 *
 ****************************************************************************************/
static void keyboard_poll(struct work_struct *work)
{
	int i;

	while (polling == 1)
	{
		unsigned char key_data;
		int key_pressed;
//		struct et6226_chip *chip;

//		spin_lock(&chip->lock);

		et6226_i2c_read(et6226_data->client, ET6226_GET_KEYS_ADDR, &key_data);

//		spin_unlock(&chip->lock);

		/* If key data is different from last time, process it */
		if (key_data ^ et6226_data->key_prev)
		{
			key_pressed = (key_data & KEY_TOGGLE_MASK) >> 6; /* returns pressed (1) or not (0) */

			fortis_4gSetLED(LED_GREEN, key_pressed); /* handle feedback */ //TODO invert LED

			if (key_pressed)  /* if key was pressed, must be new key */
			{
				/* find keycode */
				for (i = 0; i < et6226_data->keys_num; i++)
				{
					struct et6226_key *key = &et6226_data->keys[i];

					if (key_data == key->value)
					{
						et6226_data->key_press = key_pressed;
						et6226_data->key_code = key->code;
						et6226_data->key_desc = key->desc;
//						input_report_key(et6226_data->keyboard, et6226_data->key_code, key_pressed); //report key press
//						input_sync(et6226_data->keyboard);
						dprintk(20, "Press, key = %s, state = %d\n", et6226_data->key_desc, key_pressed);
						break;
					}
					dprintk(1, "[KBD] Unknown key_value %02X, state %d\n", key_data, key_pressed);
				}
			}
			else  /* key was released */
			{
				if (et6226_data->key_press != key_pressed)
				{
//					input_report_key(et6226_data->keyboard, et6226_data->key_code, key_pressed); //report key release
//					input_sync(et6226_data->keyboard);
					dprintk(20, "Release, key = %s, state = %d\n", et6226_data->key_desc, key_pressed);
				}
				et6226_data->key_press = key_pressed; /* flag key release processed */
			}
			et6226_data->key_prev = key_data; /* save current key value for next scan */
		}
		msleep(et6226_data->keys_poll);
	}
	polling = 2;
}

static DECLARE_WORK(keyboard_obj, keyboard_poll);

static int keyboard_input_open(struct input_dev *dev)
{
	keyboard_workqueue = create_workqueue("keyboard");
	if (queue_work(keyboard_workqueue, &keyboard_obj))
	{
		dprintk(5, "[KBD] Work queue created\n");
		return 0;
	}
	dprintk(1, "[KBD] Create work queue failed\n");
	return 1;
}

static void keyboard_input_close(struct input_dev *dev)
{
	polling = 0;
	while (polling != 2)
	{
		msleep(1);
	}
	polling = 1;

	if (keyboard_workqueue)
	{
		destroy_workqueue(keyboard_workqueue);
		dprintk(5, "[KBD] Workqueue destroyed.\n");
	}
}

int keyboard_dev_init(void)
{
	int err = -1;
	unsigned char key_data;
	struct et6226_chip *et6226_init = &et6226_init_values;

	dprintk(5, "[KBD] Allocating and registering ET6226 keyboard device\n");

	et6226_data->keyboard = input_allocate_device();

	if (!et6226_data->keyboard)
	{
		dprintk(1, "Could not allocate memory (ET6226 keyboard device)\n");
		return -ENOMEM;
	}

	et6226_data->keyboard->name  = et6226_init->driver_name;
	et6226_data->keyboard->open  = keyboard_input_open;
	et6226_data->keyboard->close = keyboard_input_close;

	set_bit(EV_KEY,    et6226_data->keyboard->evbit);
	set_bit(KEY_UP,    et6226_data->keyboard->keybit);
	set_bit(KEY_DOWN,  et6226_data->keyboard->keybit);
//	set_bit(KEY_LEFT,  et6226_data->keyboard->keybit);  // key not present
//	set_bit(KEY_RIGHT, et6226_data->keyboard->keybit);  // key not present
//	set_bit(KEY_POWER, et6226_data->keyboard->keybit);  // Power key is directly connected to a PIO

	err = input_register_device(et6226_data->keyboard);
	if (err)
	{
		input_free_device(et6226_data->keyboard);
		dprintk(1, "Could not register ET6226 keyboard device\n");
		return err;
	}
	dprintk(5, "[KBD] ET6226 keyboard device registered\n");
	et6226_data->keys_poll = et6226_init->keys_poll;

	/* Initialize keyboard table */
	et6226_data->keys_num = et6226_init->keys_num;
	et6226_data->keys = et6226_init->keys;
	/* Get 1st key data */
	err = et6226_i2c_read(et6226_data->client, ET6226_GET_KEYS_ADDR, &key_data);
	et6226_data->key_prev = key_data;
	return err;
}

void keyboard_dev_exit(void)
{
	dprintk(5, "[KBD] Unregistering keyboard device.\n");
	input_unregister_device(et6226_data->keyboard);
}

/****************************************************************************************
 *
 * Initialize ET6226 front panel interface and LEDs
 *
 ****************************************************************************************/
int __init fortis_4g_if_init(void)
{
	int err;
	struct i2c_adapter *et6226_adapter;
	struct i2c_client *et6226_client = NULL;
	struct et6226_chip *et6226_init = &et6226_init_values;

	/* Initialize the I2C interface to the ET6226 */
	et6226_data = kzalloc(sizeof(*et6226_data), GFP_KERNEL);
	if (!et6226_data)
	{
		dprintk(1, "kzalloc failed\n");
		return -ENOMEM;
	}
	spin_lock_init(&et6226_data->lock);

	et6226_adapter = i2c_get_adapter(ET6226_I2C_BUSNUM);

	if (!et6226_adapter)
	{
		dprintk(1, "i2c_get_adapter(%02d) failed\n", ET6226_I2C_BUSNUM);
		return -ENODEV;
	}
	et6226_client = i2c_new_device(et6226_adapter, &et6226_i2c_info);
	i2c_put_adapter(et6226_adapter);

	if (!et6226_client)
	{
		dprintk(1, "No i2c client found\n");
		dprintk(100, "%s <\n", __func__);
		return -EIO;
	}
	et6226_data->client = et6226_client;

	/* Initialize the ET6226 */
	et6226_data->on = et6226_init->ctl_reg && ET6226_DISPLAY_ENABLE;
	et6226_data->sleep_mode = et6226_init->ctl_reg && ET6226_SLEEP_MODE;
	et6226_data->display_mode = et6226_init->ctl_reg && ET6226_SEVEN_SEGMENT_MODE;

	et6226_clear(et6226_data);  /* Clear display memory */

	/* Initialize character table */
	et6226_data->char_tbl = et6226_init->char_tbl;

	/* Set display mode and initial brightness */
	fortis_4gSetBrightness(et6226_init->brightness);

	dprintk(10, "Display initialized, brightness %d\n", et6226_data->brightness);

	/* Set the LEDs */
	init_leds();
	dprintk(10, "LEDs initialized\n");

	/* Initialize keyboard interface */
	err = keyboard_dev_init();
	if (err)
	{
		dprintk(1, "Could not initialize ET6226 keyboard interface\n");
		fortis_4gExit_core();
		return err;
	}
	dprintk(10, "ET6226 keyboard initialized, %d keys, poll period is %d ms\n", et6226_data->keys_num, et6226_data->keys_poll);

	dprintk(100, "%s <\n", __func__);
	return 0;
}

/****************************************************************************************
 *
 * IOCTL ET6226 routines
 *
 ****************************************************************************************/
/* Function SetIcon */
int fortis_4gSetIcon(int which, int on)
{
	return 0;
}
/* export for later use in e2_proc */
EXPORT_SYMBOL(fortis_4gSetIcon);

/* Function SetLED (and SetPwrLED) */
int fortis_4gSetLED(int which, int level)
{
	int ret = -1;

	dprintk(100, "%s > %d, %d\n", __func__, which, level);

	if (which < 1 || which > MAX_LED)
	{
		printk("[fortis_4g_et6226] LED number %d out of range (1-%d)\n", which, MAX_LED);
		return -EINVAL;
	}

	if (level < 1)
	{
		level = 0;
	}
	else
	{
		if (MAX_LED_BRIGHT != 1)
		{
			if (level > MAX_LED_BRIGHT)
			{
				printk("[fortis_4g_et6226] LED brightness %d out of range (0-%d)\n", which, MAX_LED_BRIGHT);
				return -EINVAL;
			}
		}
		else
		{
			level = MAX_LED_BRIGHT;
		}
	}
	ret = 0;

	switch(which)
	{
		case LED_GREEN:
		{
#if defined(DP7001) || defined(FOREVER_2424HD)
			/* Note: drive of LED_GREEN is inverted on DP7001/7050 */
			gpio_set_value(GPIO_LED_GREEN, level ? 0 : 1);
#else
			gpio_set_value(GPIO_LED_GREEN, level);
#endif
			break;
		}

#if !defined(FOREVER_NANOSMART)
		case LED_LOGO:
		{
#if defined(DP7001) || defined(FOREVER_2424HD)
			/* Note: drive of LED_LOGO is inverted on DP7001/7050 */
			gpio_set_value(GPIO_LED_LOGO, level ? 0 : 1);
#else
			gpio_set_value(GPIO_LED_LOGO, level);
#endif
			break;
		}
#endif //!defined(FOREVER_NANOSMART)
		default:
		{
			dprintk(1, "%s Unknown LED number %d specified\n", __func__, which);
			ret = -EINVAL;
			break;
		}
	}
	dprintk(100, "%s < RTV = %d\n", __func__, ret);
	return ret;
}

/* export for later use in e2_proc */
EXPORT_SYMBOL(fortis_4gSetLED);

/* Function SetBrightness */
int fortis_4gSetBrightness(int level)
{
	int ret = -1;

	dprintk(100, "%s > Brightness level to set: %d\n", __func__, level);

	if (level < 0 || level > MAX_BRIGHT)
	{
		dprintk(0, "Display brightness level %d out of range (0-%d)\n", level, MAX_BRIGHT);
		return -EINVAL;
	}

	/* Calculate brightness value to write */
	if (level > 6)
	{
		level = 0; // set maximum brightness
	}
	else if (level < 0)
	{
		level = 1; // set lowest brighness
	}
	else
	{
		level++; // set requested brightness
	}
	et6226_data->brightness = level;
	if (et6226_data->sleep_mode)
	{
		dprintk(0, "Sleep mode set, no change\n");
		ret = 0;
	}
	ret = et6226_i2c_write(et6226_data->client, ET6226_CTL_ADDR, ET6226_CTL_DATA(et6226_data->on, et6226_data->sleep_mode, et6226_data->display_mode, et6226_data->brightness));

	dprintk(100, "%s < RTV = %d\n", __func__, ret);
	return ret;
}
/* export for later use in e2_proc */
EXPORT_SYMBOL(fortis_4gSetBrightness);

/* Function SetDisplayOnOff */
int fortis_4gSetDisplayOnOff(int on)
{
	int ret = -1;

//	dprintk(100, "%s >\n", __func__);

	et6226_data->on = on; // remember setting
	if (et6226_data->sleep_mode)
	{
		dprintk(0, "Sleep mode set, no change\n");
		ret = 0;
	}
	ret = et6226_i2c_write(et6226_data->client, ET6226_CTL_ADDR, ET6226_CTL_DATA((on ? ET6226_DISPLAY_ENABLE : 0), et6226_data->sleep_mode, et6226_data->display_mode, et6226_data->brightness));
//	dprintk(100, "%s <\n", __func__);
	return ret;
}

/* Function GetWakeUpMode (to be built) */
int fortis_4gGetWakeUpMode(void)
{
	int ret = 0;

	dprintk(100, "%s >\n", __func__);

//	memset(buffer, 0, 3);

//	buffer[0] = SOP;
//	buffer[1] = cCommandGetPowerOnSrc;
//	buffer[2] = EOP;

//	errorOccured = 0;
//	ret = fortis_4gWriteCommand(buffer, 3, 1);

//	if (errorOccured == 1)
	if (ret)
	{
		/* error */
		printk("[fortis_4g_i2c] GetWakeMode: error\n");
		ret = -ETIMEDOUT;
	}
	else
	{
		/* wake reason mode received ->noop here */
		dprintk(1, "wake up mode received\n");
	}

	dprintk(100, "%s <\n", __func__);
	return ret;
}

/* Function SetTimeFormat (to be built, if needed) */
int fortis_4gSetTimeFormat(char format)
{
//	char buffer[4];
	int  ret = -1;

	dprintk(100, "%s >\n", __func__);

//	buffer[0] = SOP;
//	buffer[1] = cCommandSetTimeFormat;
//	buffer[2] = format;
//	buffer[3] = EOP;

//	dprintk(20, "Time mode command= 0x%02x (SOP) 0x%02x 0x%02x 0x%02x (EOP)\n", buffer[0], buffer[1], buffer[2], buffer[3]);
//	res = fortis_4gWriteCommand(buffer, 4, 0);

	dprintk(100, "%s <\n", __func__);
	return ret;
}

/* Function SetTimeDisplay (to be built, if needed) */
int fortis_4gSetTimeDisplayOnOff(char onoff)
{
	int  ret = -1;

	dprintk(100, "%s >\n", __func__);

	dprintk(1, "SetTimeDisplayOnOff not implemented yet.\n");

	dprintk(100, "%s <\n", __func__);
	return ret;
}

/* Function WriteString */
#if defined(DP7001) || defined(FOREVER_2424HD)
int fortis_4gWriteString(unsigned char *aBuf, int len)
{
	int i, j, ret = -1;
	int buflen = len;
	int strlen;
	int dot_count = 0;
	int colon_count = 0;
	unsigned char bBuf[ET6226_DISPLAY_MAX_DIGITS];  // display string without replaceable colon/periods
	unsigned char cmd_buf[ET6226_DISPLAY_MAX_DIGITS] = { 0, };  // bytes to send to ET6226

	dprintk(100, "%s > %d\n", __func__, len);

	aBuf[len] = 0;
	dprintk(10, "Display [%s] (%d characters)\n", aBuf, len);

	bBuf[0] = aBuf[0];
	bBuf[1] = aBuf[1];
	i = 2;  // input character counter
	j = 2;  // output digit counter

	while ((j <= ET6226_DISPLAY_MAX_DIGITS) && (i < buflen))
	{
		if (aBuf[i] == '.')  // if character is period
		{
			if (i == 2)
			{
				cmd_buf[0] |= 0x80;
			}
			else
			{
				if (j >= 2)
				{
					cmd_buf[i - 1 - colon_count] |= 0x80;
				}
			}
			dot_count++;
		}
		else if ((aBuf[i] == ':') && (j == 2))
		{
			cmd_buf[1] |= 0x80;  // flag colon on
			colon_count++;
		}
		else
		{
			bBuf[j] = aBuf[i];  // copy next character
			j++;
		}
		i++;
	}
	strlen = buflen - dot_count - colon_count;

	if (strlen > ET6226_DISPLAY_MAX_DIGITS)
	{
		strlen = ET6226_DISPLAY_MAX_DIGITS;
	}
	for (i = 0; i < ET6226_DISPLAY_MAX_DIGITS; i++)
	{
		cmd_buf[i] |= et6226_data->char_tbl[(bBuf[i] - 0x20)];
	}
	if (strlen < 4)
	{
		for (i = strlen; i < ET6226_DISPLAY_MAX_DIGITS; i++)
		{
			cmd_buf[i] &= 0x80;
		}
	}
	for (i = 0; i < ET6226_DISPLAY_MAX_DIGITS; i++)
	{
		ret = et6226_i2c_write(et6226_data->client, ET6226_DSP_ADDR + i, cmd_buf[i]);
	}
	/* save last string written to fp */
	memcpy(&lastdata.data, aBuf, 64);
	lastdata.length = len;

	dprintk(100, "%s <\n", __func__);
	return ret;
}
#elif defined(FOREVER_NANOSMART)
int fortis_4gWriteString(unsigned char *aBuf, int len)
{
	int i, ret;
	int buflen;
	unsigned char bBuf[ET6226_DISPLAY_MAX_DIGITS];
	unsigned char cmd_buf[ET6226_DISPLAY_MAX_DIGITS] = { 0 };

	dprintk(100, "%s > %d\n", __func__, len);

	buflen = len;
	bBuf[0] = aBuf[0];
	bBuf[1] = aBuf[1];

/* Connection of colon is:
 * GR2 + KSP -> colon in center (buf[1])
 */
	if (aBuf[2] == ':') // if 3rd character is a colon
	{
		cmd_buf[1] |= 0x80; // switch colon on
		bBuf[2] = aBuf[3]; // shift rest of buffer
		bBuf[3] = aBuf[4]; // forward
	}
	else
	{
		bBuf[2] = aBuf[2];
		bBuf[3] = aBuf[3];
	}

	if (buflen > ET6226_DISPLAY_MAX_DIGITS)
	{
		buflen = ET6226_DISPLAY_MAX_DIGITS;
	}

	for (i = 0; i < buflen; i++)
	{
		cmd_buf[i] |= et6226_data->char_tbl[(bBuf[i] - 0x20)];
	}

	for (i = 0; i < ET6226_DISPLAY_MAX_DIGITS; i++)
	{
		ret |= et6226_i2c_write(et6226_data->client, ET6226_DSP_ADDR + i, cmd_buf[i]);
	}

	/* save last string written to fp */
	memcpy(&lastdata.data, aBuf, 128);
	lastdata.length = len;

	dprintk(100, "%s <\n", __func__);
	return ret;
}
#else  // not FOREVER_NANOSMART or DP7001/7050
int fortis_4gWriteString(unsigned char* aBuf, int len)
{
	dprintk(100, "%s >\n", __func__);
	dprintk(100, "%s <\n", __func__);
	return -EFAULT;
}
#endif

int fortis_4gInit_core(void)
{
	int res;

	dprintk(100, "%s >\n", __func__);

	res = fortis_4g_if_init();
	return res;

	dprintk(100, "%s <\n", __func__);
}

void fortis_4gExit_core(void)
{
	dprintk(100, "%s >\n", __func__);

	/* free LED GPIO pins */
	cleanup_leds();	

	dprintk(5, "[KBD] Unloading...\n");
	keyboard_dev_exit();

	i2c_unregister_device(et6226_data->client);

	dprintk(100, "%s <\n", __func__);
}
// vim:ts=4
