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
typedef struct
{
	int scart_ar;
	int scart_f;
	int scart_cond;
} Scart_Status_st;


/******* Char dev driver *******/
#define DEVICE_NAME			"scart_0"

#define MAJOR_NUM        	177 /* The same of stv6414 driver in QBOXHD */
#define MINOR_START      	0

#define	SCART_NUMBER_OF_CONTROLLERS	1


/* Define for ioctl to move the GPIOs*/

#define	SCART_AR		0x04	/* TV-FNC */	
#define	SCART_FORMAT	0x08	/* TV-FB */
#define	SCART_CON		0x10	/* TV-FNC */

#define AR_4_3			0
#define	AR_16_9			1

#define	RGB				0
#define	CVBS			1

#define	DIS_SCART		0
#define	EN_SCART		1

/* Ioctl cmd table */
#define SCART_IOW_MAGIC			    'S'

#define IOCTL_SET_ASPECT_RATIO		_IOW(SCART_IOW_MAGIC, 1,  int) /* 0 -> 4/3 , 1 -> 16/9 */
#define IOCTL_FORMAT_COLOR			_IOW(SCART_IOW_MAGIC, 2,  int) /* 0 -> RGB , 1 -> CVBS */
#define IOCTL_ENABLE_SCART			_IOW(SCART_IOW_MAGIC, 3,  int) /* 0 -> disable , 1 -> enable */
#define IOCTL_SCART_STATUS			_IOWR(SCART_IOW_MAGIC, 4,  Scart_Status_st) /* Current status of Scart GPIOs */





