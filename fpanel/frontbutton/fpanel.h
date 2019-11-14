/*
 * 	Copyright (C) 2010 Duolabs Srl
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#define MYVERS "0.0.5"  // 26 apr 2010

#define dprintk(args...) \
	do { \
		if (debug) printk("[fpanel] " args); \
	} while (0)


#define SUCCESS 0
#define FAIL -1

#define MAJOR_NR                    175
#define LED_MAGIC                   0xBC
#define FPANEL_NUMBER_DEVICES       1

#define DEVICE_NAME					fpanel_0 /* The equivalent in qboxhd is sw0 */

#define FPANEL_PORT5_IRQ			113


/* When the button is released the logical value is 1 (so in the port is 0x80: bit 7),
	when the button is pressed the logical value is 0 (so in te port is 0x00) */
#define BUTTON_PRESSED				0x00	//
#define BUTTON_RELEASED				0x80

/**************/
/* For button */
/**************/
#define PIO5_BASE_ADDRESS   0x18025000 
#define PIO5_IO_SIZE        0x1000
#define PIO_CLR_PnC0        0x28
#define PIO_CLR_PnC1        0x38
#define PIO_CLR_PnC2        0x48
#define PIO_SET_PnC0        0x24
#define PIO_SET_PnC1        0x34
#define PIO_SET_PnC2        0x44
#define PIO_PnMASK          0x60
#define PIO_PnCOMP          0x50

#define PIO_SET_PnCOMP		0x54
#define PIO_CLR_PnCOMP		0x58

#define PIO_PnIN			0x10

#define BUTTON_PIN			0x80 // The input pin of the button is port 5 pin 7

typedef struct
{
	unsigned int pio_addr;
	unsigned char button;
}Global_Status_t;


/* IOCTL */

#define IOCTL_READ_BUTTON_STATUS		_IOR(LED_MAGIC, 1, int)



