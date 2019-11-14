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

/* 
 * GPIO for resetting tuners
 */
#define BASE_ADDRESS	    0x18020000	/* Port 0 */

#define PIO0_IO_SIZE        0x1000
#define PIO_CLR_PnC0        0x28
#define PIO_CLR_PnC1        0x38
#define PIO_CLR_PnC2        0x48
#define PIO_SET_PnC0        0x24
#define PIO_SET_PnC1        0x34
#define PIO_SET_PnC2        0x44

#define PIO_CLR_PnOUT       0x08
#define PIO_SET_PnOUT       0x04


/******* Char dev driver *******/
#define DEVICE_NAME                 "rst_0"
#define MAJOR_NUM			        178     /**< Major num of char dev */
#define MINOR_START                 0       /**< Starting minor for char dev */
#define	RST_NUMBER_OF_CONTROLLERS	1


#define	DEACT_RST					0
#define	ACT_RST						1

#if defined CONFIG_SH_QBOXHD_MINI_1_0
#define TUNER_0						0
#define TUNER_1						1
#endif


/* Ioctl cmd table */
#define RST_IOW_MAGIC			    'R'

#define IOCTL_ACTIVATION_RST		_IOW(RST_IOW_MAGIC, 1,  int)
#if defined CONFIG_SH_QBOXHD_MINI_1_0
#define IOCTL_RST_TUNER_0			_IOW(RST_IOW_MAGIC, 100,  int)
#define IOCTL_RST_TUNER_1			_IOW(RST_IOW_MAGIC, 101,  int)
#endif

