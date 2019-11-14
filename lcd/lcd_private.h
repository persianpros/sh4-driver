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

/*****************************
 * MACROS
 *****************************/

#define EMI_BASE_ADDRESS     0x1000000
#define EMI_IO_SIZE          0x1000

#define PIO5_BASE_ADDRESS   0x18025000  /* GPIO 5[4] */
#define PIO5_IO_SIZE        0x1000
#define PIO_CLR_PnC0        0x28
#define PIO_CLR_PnC1        0x38
#define PIO_CLR_PnC2        0x48
#define PIO_SET_PnC0        0x24
#define PIO_SET_PnC1        0x34
#define PIO_SET_PnC2        0x44

#define PIO_CLR_PnOUT       0x08
#define PIO_SET_PnOUT       0x04

#define	FPGA_BASE_ADDRESS	0x3800000
#define FPGA_IO_SIZE		0x1000
#define	REG_BRIGHTNESS		0x3A


#define NUM_MINORS 1

#define LCD_MAJOR_NUM        172
#define LCD_MINOR_START      0


#define PDEBUG(fmt, args...)                                             \
    do {                                                                 \
        if (debug) printk(KERN_NOTICE "%s: " fmt, __FUNCTION__, ##args); \
    } while (0)

#define PINFO(fmt, args...)         printk(KERN_INFO "" fmt, ##args)
#define PERROR(fmt, args...)        printk(KERN_ERR "%s: " fmt, __FUNCTION__, ##args)

/*****************************
 * DATA TYPES
 *****************************/

/**< LCD errors */
typedef enum 
{
    LCD_NO_ERROR = 0,
    LCD_ERROR_INIT_DRIVER = -100,
} LCDError;

/**< LCD Control Block */
typedef struct lcd_cb_st
{
    UINT32 base_address_emi;    /** LCD registers start here */
    UINT32 base_address_pio5;   /** GPIO 5 base address */
    UINT32 fpga_base_address;   /** FPGA module base address */
    UINT8  driver_inuse;        /** Only one app can access the device */
} LCD_CONTROL_BLOCK;

