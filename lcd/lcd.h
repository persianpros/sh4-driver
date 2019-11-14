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

#if defined CONFIG_SH_QBOXHD_MINI_1_0
	#define DISPLAY_WIDTH				160     
	#define DISPLAY_HEIGHT				128     
#else
	#define DISPLAY_WIDTH				320     /**< QVGA res: x = 320 */
	#define DISPLAY_HEIGHT				240     /**< QVGA res: y = 240 */
#endif

#define DISPLAY_BYPP				2

#define	MIN_BRG						0
#define	MAX_BRG						63


#define LCD_DEVICE_NAME				"lcd"	

#define LCD_IOC_MAGIC				'l'
#define LCD_IOCTL_RS				_IOR(LCD_IOC_MAGIC, 0, int)		/* Set Register Select */
#define LCD_IOCTL_STANDBY			_IOWR(LCD_IOC_MAGIC, 1, int)	/* Stan-by on/off */
//#define LCD_IOCTL_NEWLINE			_IOR(LCD_IOC_MAGIC, 2, int)		/* New line in Y */
#define LCD_IOCTL_ON_OFF_BRIGHTNESS	_IOWR(LCD_IOC_MAGIC, 3, int)   	/* On Off brightness */
#define LCD_IOCTL_REG_BRIGHTNESS	_IOWR(LCD_IOC_MAGIC, 4, short)   	/* On Off brightness */
#define LCD_IOCTL_MMAP_WRITE		_IOWR(LCD_IOC_MAGIC, 5, short)   	/* Start and stop to write on display with mmap function */

/* Only for QBoxHD mini */
#define LCD_IOCTL_READ_ID			_IOWR(LCD_IOC_MAGIC, 6, unsigned int)   	/* Return the value of regs DA DB DC */

#define LCD_IOC_NR					7

#define BRIGHTNESS_OFF				0x00
#define BRIGHTNESS_ON				0x01

#define	LCD_STOP					0x00
#define	LCD_START					0x01



/*****************************
 * DATA TYPES
 *****************************/

typedef char                    INT8;
typedef unsigned char           UINT8;

typedef short int               INT16; 
typedef unsigned short int      UINT16;

typedef int                     INT32;
typedef unsigned int            UINT32;

typedef long long int           INT64;
typedef unsigned long long int  UINT64;

typedef double                  FINT64;



