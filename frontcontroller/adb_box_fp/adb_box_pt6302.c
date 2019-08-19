/*
 * adb_box_pt6302.c
 *
 * (c) 20?? B4team??
 * (c) 2019 Audioniek
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
 * Front panel driver for nBOX ITI-5800SX (BSLA and BZZB variants),
 * VFD display driver for PT6302.
 *
 *
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 20130929 B4team?         Initial version
 * 20190723 Audioniek       start of major rewrite.
 * 20190??? Audioniek       procfs added.
 *
 ****************************************************************************************/

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
#include <linux/stm/pio.h>
#else
#include <linux/stpio.h>
#endif

#include "adb_box_fp.h"
#include "adb_box_pt6302.h"
#include "adb_box_table.h"  // character table


/*****************************************************
 *
 * Code for front panel button driver 
 *
 */

/*****************************************************
 *
 * pt6302_access_char
 *
 * 
 *
 */
static char pt6302_access_char(struct pt6302_pin_driver *vfd_pin, int dout)
{
	uint8_t din   = 0;
	int     outen = (dout < 0) ? 0 : 1, i;

	for (i = 0; i < 8; i++)
	{
		if (outen)
		{
			stpio_set_pin(vfd_pin->pt6302_din, (dout & 1) == 1);
		}
		stpio_set_pin(vfd_pin->pt6302_clk, 0);
		udelay(delay);
		stpio_set_pin(vfd_pin->pt6302_clk, 1);
		udelay(delay);
		din  = (din >> 1) | (stpio_get_pin(vfd_pin->pt6302_din) > 0 ? 0x80 : 0);
		dout = dout >> 1;
	}
	return din;
};

/*****************************************************
 *
 * pt6302_write_char
 *
 * 
 *
 */
static inline void pt6302_write_char(struct pt6302_pin_driver *vfd_pin, char data)  // used
{
	stpio_set_pin(vfd_pin->pt6302_cs, 0);
	pt6302_access_char(vfd_pin, data);
	stpio_set_pin(vfd_pin->pt6302_cs, 1);
};

#if 0
/*****************************************************
 *
 * pt6302_read_char
 *
 * 
 *
 */
static inline char pt6302_read_char(struct pt6302_pin_driver *vfd_pin)
{
	stpio_set_pin(vfd_pin->pt6302_cs, 0);
	return pt6302_access_char(vfd_pin, -1);
	stpio_set_pin(vfd_pin->pt6302_cs, 1);
};
#endif

/*****************************************************
 *
 * pt6302_write_data
 *
 * write len bytes to PT6302
 *
 */
static void pt6302_write_data(struct pt6302_pin_driver *vfd_pin, char *data, int len)
{
	int i;

	stpio_set_pin(vfd_pin->pt6302_cs, 0);  // enable PT6302

	for (i = 0; i < len; i++)
	{
		pt6302_access_char(vfd_pin, data[i]);
	}
	stpio_set_pin(vfd_pin->pt6302_cs, 1);
};

#if 0
/*****************************************************
 *
 * pt6302_read_data
 *
 * read len bytes from PT6302
 *
 */
static int pt6302_read_data(struct pt6302_pin_driver *vfd_pin, char *data, int len)
{
	int i;

	stpio_set_pin(vfd_pin->pt6302_cs, 0);

	for (i = 0; i < len; i++)
	{
		data[i] = pt6302_access_char(vfd_pin, -1);
	}
	stpio_set_pin(vfd_pin->pt6302_cs, 1);
	return len;
};
#endif

void pt6302_free(struct pt6302_driver *ptd);

/*****************************************************
 *
 * pt6302_init
 *
 * Initialize PT6302 display driver
 *
 */
struct pt6302_driver *pt6302_init(struct pt6302_pin_driver *vfd_pin_driv)
{
	struct pt6302_driver *ptd = NULL;

	dprintk(10, "%s (pt6302_pin_drv = %p)\n", __func__, vfd_pin_driv);

	if (vfd_pin_driv == NULL)
	{
		dprintk(1, "vfd_pin driver not initialized; abort.");
		return NULL;
	}
	ptd = (struct pt6302_driver *)kzalloc(sizeof(struct pt6302_driver), GFP_KERNEL);
	if (ptd == NULL)
	{
		dprintk(1, "Unable to allocate pt6302 driver struct; abort.");
		goto pt6302_init_fail;
	}
	ptd->pt6302_pin_drv = vfd_pin_driv;
	return ptd;

pt6302_init_fail:
	pt6302_free(ptd);
	return NULL;
}

/*****************************************************
 *
 * pt6302_free
 *
 * Stop PT6302 display driver
 *
 */
void pt6302_free(struct pt6302_driver *ptd)
{
	if (ptd == NULL)
	{
		return;
	}
//	pt6302_set_light(ptd, PT6302_LIGHTS_OFF);
//	pt6302_set_brightness(ptd, PT6302_DUTY_MAX);
//	pt6302_set_digits(ptd, PT6302_DIGITS_MIN);
//	pt6302_set_ports(ptd, 0, 0);

	ptd->pt6302_pin_drv = NULL;
}

/******************************************************
 *
 * pt6302_write_dcram
 *
 * Displays a string on the VFD display.
 *
 */
int pt6302_write_dcram(struct pt6302_driver *ptd, unsigned char addr, unsigned char *data, unsigned char len)
{
	pt6302_command_t cmd;
	uint8_t          *wdata;
	int              i = 0;
	int              j = 0;
	int              char_index = 0x00;
	char             led_txt[16];

	dprintk(1, "Text [%s] (len =%d)\n", data, len); // display what comes on screen

//define L3D 1
#if defined(L3D)
	if ((data[0] == 'l') && (data[1] == '3') && (data[2] == 'd')) // if string starts with l3d
	{
		goto led_control;  // set LEDs
	}
#endif
	wdata = kmalloc(1 + len_vfd, GFP_KERNEL);

	if (wdata == NULL)
	{
		dprintk(1,"Unable to allocate write buffer of %d bytes.\n", len_vfd + 1);
		return -ENOMEM;
	}

	memset(wdata, ' ', len_vfd + 1); // fill buffer with spaces
	cmd.dcram.cmd  = PT6302_COMMAND_DCRAM_WRITE;
	cmd.dcram.addr = (addr & 0xf);
	wdata[0] = cmd.all;

	len = (len <= len_vfd) ? len : len_vfd;

	// clean LED
	for (i = 0; i < 16; i++)
	{
		led_txt[i] = 0x20;  //space
	}

	// skipping characters outside STANDARD table
	for (i = 0; i < len; i++)
	{
		if (data[i] < 0x80)
		{
			wdata[len_vfd - j] = pt6302_adb_box_rom_table[data[i]];  // select VFD display
			j++;
		}
		else
		{
			dprintk(1, "[%x]\n", data[i]); // display the characters > 0x7f on VFD and LED

			char_index = 0x00;

			if ((data[i] == 0xc4) && (data[i + 1] == 0x85))
			{
				char_index = 0x61;  //0xc4,0x85 ą
			}
			if ((data[i] == 0xc4) && (data[i + 1] == 0x87))
			{
				char_index = 0x63;  //0xc4,0x87 ć
			}
			if ((data[i] == 0xc4) && (data[i + 1] == 0x99))
			{
				char_index = 0x65;  //0xc4,0x99 ę
			}
			if ((data[i] == 0xc5) && (data[i + 1] == 0x82))
			{
				char_index = 0x6c;  //0xc5,0x82 ł
			}
			if ((data[i] == 0xc5) && (data[i + 1] == 0x84))
			{
				char_index = 0x6e;  //0xc3,0xb3 ń
			}
			if ((data[i] == 0x87) && (data[i + 1] == 0xc5))
			{
				char_index = 0x73;  //0x87,0xc5 ś
			}
			if ((data[i] == 0xc3) && (data[i + 1] == 0xb3))
			{
				char_index = 0x6f;  //0xc3,0xb3 ó
			}
			if ((data[i] == 0xc5) && (data[i + 1] == 0xbc))
			{
				char_index = 0x7a;  //0xc5,0xbc ż
			}
			if ((data[i] == 0xc5) && (data[i + 1] == 0xba))
			{
				char_index = 0x7a;  //0xc5,0xba ź
			}
			if (char_index != 0x00)
			{
				wdata[len_vfd - j] = pt6302_adb_box_rom_table[char_index];  // select the display
				j++;
			}
		}
	}
	pt6302_write_data(ptd->pt6302_pin_drv, wdata, len_vfd + 1);

// show text of PT6958 LED display as well
	for (i = 0; i < 16; i++)
	{
		led_txt[i] = wdata[len_vfd - i];
	}
	pt6958_display(led_txt);
	return 0;

#if defined(L3D)
led_control:  // Set the LEDs
	dprintk(1, "led: 0x%x stan: 0x%x", data[4], data[6]);

	if ((data[4] == '1') && (data[6] == '0'))
	{
		pt6958_led_control(PT6958_CMD_ADDR_LED1, 0);  // set LED1 off
	}
	if ((data[4] == '1') && (data[6] == '1'))
	{
		pt6958_led_control(PT6958_CMD_ADDR_LED1, 1);  // set LED1 on (red)
	}
	if ((data[4] == '1') && (data[6] == '2'))
	{
		pt6958_led_control(PT6958_CMD_ADDR_LED1, 2);  // set LED1 on (green)
	}
	if ((data[4] == '1') && (data[6] == '3'))
	{
		pt6958_led_control(PT6958_CMD_ADDR_LED1, 3);  // set LED1 on (yellow)
	}

	if ((data[4] == '2') && (data[6] == '0'))
	{
		pt6958_led_control(PT6958_CMD_ADDR_LED2, 0);  // set LED2 off
	}
	if ((data[4] == '2') && (data[6] == '1'))
	{
		pt6958_led_control(PT6958_CMD_ADDR_LED2, 1);  // set LED2 on
	}

	if ((data[4] == '3') && (data[6] == '0'))
	{
		pt6958_led_control(PT6958_CMD_ADDR_LED3, 0);  // set LED3 off
	}
	if ((data[4] == '3') && (data[6] == '1'))
	{
		pt6958_led_control(PT6958_CMD_ADDR_LED3, 1);  // set LED3 on
	}

	if ((data[4] == '4') && (data[6] == '0'))
	{
		pt6958_led_control(PT6958_CMD_ADDR_LED4, 0);  // set LED4 off
	}
	if ((data[4] == '4') && (data[6] == '1'))
	{
		pt6958_led_control(PT6958_CMD_ADDR_LED4, 1);  // set LED4 on
	}
#endif
	return 0;

}

/******************************************************
 *
 * pt6302_set_brightness
 *
 * Sets brightness of the VFD display.
 *
 */
void pt6302_set_brightness(struct pt6302_driver *ptd, int level)
{
	pt6302_command_t cmd;

	if (level < PT6302_DUTY_MIN)
	{
		level = PT6302_DUTY_MIN;
	}
	if (level > PT6302_DUTY_MAX)
	{
		level = PT6302_DUTY_MAX;
	}
	cmd.duty.cmd  = PT6302_COMMAND_SET_DUTY;
	cmd.duty.duty = level;

	pt6302_write_char(ptd->pt6302_pin_drv, cmd.all);
}

/******************************************************
 *
 * pt6302_set_digits
 *
 * Sets width of the VFD display.
 *
 */
static void pt6302_set_digits(struct pt6302_driver *ptd, int num)
{
	pt6302_command_t cmd;

	if (num < PT6302_DIGITS_MIN)
	{
		num = PT6302_DIGITS_MIN;
	}
	if (num > PT6302_DIGITS_MAX)
	{
		num = PT6302_DIGITS_MAX;
	}
	num = (num == PT6302_DIGITS_MAX) ? 0 : (num - PT6302_DIGITS_OFFSET);

	cmd.digits.cmd    = PT6302_COMMAND_SET_DIGITS;
	cmd.digits.digits = num;

	pt6302_write_char(ptd->pt6302_pin_drv, cmd.all);
}

/******************************************************
 *
 * pt6302_set_light
 *
 * Sets VFD display to normal display, all segments off
 * or all segments on.
 *
 */
void pt6302_set_light(struct pt6302_driver *ptd, int onoff)
{
	pt6302_command_t cmd;

	if (onoff < PT6302_LIGHT_NORMAL || onoff > PT6302_LIGHT_ON)
	{
		onoff = PT6302_LIGHT_ON; // if illegal input, set all segments on?
	}
	cmd.lights.cmd   = PT6302_COMMAND_SET_LIGHTS;
	cmd.lights.onoff = onoff;

	pt6302_write_char(ptd->pt6302_pin_drv, cmd.all);

	// brightness of display and LEDs
//	pt6958_write(PT6958_CMD_DISPLAY_ON, NULL, 0);
}

/******************************************************
 *
 * pt6302_set_ports
 *
 * Sets the binary output pins of the PT6302 to the
 * states specified.
 *
 */
void pt6302_set_ports(struct pt6302_driver *ptd, int port1, int port2)
{
	pt6302_command_t cmd;

	cmd.port.cmd   = PT6302_COMMAND_SET_PORTS;
	cmd.port.port1 = (port1) ? 1 : 0;
	cmd.port.port2 = (port2) ? 1 : 0;

	pt6302_write_char(ptd->pt6302_pin_drv, cmd.all);
}

/******************************************************
 *
 * pt6302_setup
 *
 * Initializes the VFD display.
 *
 */
void pt6302_setup(struct pt6302_driver *pfd)
{
	int i;

	dprintk(1, "Initialize PT6302 VFD controller.");

	pt6302_set_ports(pfd, 1, 0);

	pt6302_set_digits(pfd, PT6302_DIGITS_MAX);  // display width = 16
	pt6302_set_brightness(pfd, PT6302_DUTY_NORMAL);
	pt6302_set_light(pfd, PT6302_LIGHT_NORMAL);

	dprintk(1, "Setup of PT6302 completed.");

#if 0 // skip driver sign on
	if (rec)
	{
		pt6302_write_dcram(pfd, 0x0, "       []       ", 16);  // welcome
		pt6958_display("-  -");
		for (i = 0; i < 50; i++)
		{
			udelay(2500);
		}
		pt6302_write_dcram(pfd, 0x0, "      [  ]      ", 16);  // welcome
		for (i = 0; i < 50; i++)
		{
			udelay(2500);
		}
		pt6302_write_dcram(pfd, 0x0, "     [ ** ]     ", 16);  // welcome
		pt6958_display(" -- ");
		for (i = 0; i < 50; i++)
		{
			udelay(2500);
		}
		pt6302_write_dcram(pfd, 0x0, "    [ *  * ]    ", 16);  // welcome
		for (i = 0; i < 50; i++)
		{
			udelay(2500);
		}
		pt6302_write_dcram(pfd, 0x0, "   [ *    * ]   ", 16);  // welcome
		pt6958_display(" bm ");
		for (i = 0; i < 50; i++)
		{
			udelay(1500);
		}
		pt6302_write_dcram(pfd, 0x0, "  [ *  Bm  * ]  ", 16);  // welcome
		for (i = 0; i < 50; i++)
		{
			udelay(2500);
		}
		pt6302_write_dcram(pfd, 0x0, " [ *  B4am  * ] ", 16);  // welcome
		pt6958_display("b4tm");
		for (i = 0; i < 50; i++)
		{
			udelay(2500);
		}
		pt6302_write_dcram(pfd, 0x0, "[ *  B4Team  * ]", 16);  // welcome
		pt6958_display("b4tm");
		for (i = 0; i < 150; i++)
		{
			udelay(4000);
		}
		pt6302_write_dcram(pfd, 0x0, "[ *= B4Team =* ]", 16);  // welcome
		pt6958_display("05.03");
	}
	else
	{
		pt6958_display(led_txt);
	}
	for (i = 0; i < 150; i++)
	{
		udelay(5000);
	}
	pt6302_write_dcram(pfd, 0, "[              ]", 16);  // welcome
#else
	pt6302_write_dcram(pfd, 0, "                ", 16);
#endif
	pt6958_display("    ");  // blank LED display  // blank VFD display
//	pt6958_led_control(PT6958_CMD_ADDR_LED1, 2); power green (alaready done in driver init)
}

// vim:ts=4
