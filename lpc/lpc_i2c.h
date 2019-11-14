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
#ifndef __LPD_I2C_H
#define __LPD_I2C_H

typedef struct
{
	unsigned char red;
	unsigned char green;
	unsigned char blue;
}Rrgb_Value_t;

#define LPC_DRIVER_VERSION	"0.0.6"

#define	INPUT_PWM					0x00
#define	OUTPUT_PWM				0x01
#define	OBSOLETE					0x02

#define IOCTL_LPC_MAGIC				'L'

#define	IOCTL_READ_VERSION			_IOWR(IOCTL_LPC_MAGIC, 1, int)	// a address
#define	IOCTL_READ_TEMP				_IOWR(IOCTL_LPC_MAGIC, 2, int)	// a address
#define	IOCTL_FAN_POWER				_IOWR(IOCTL_LPC_MAGIC, 3, int)	//from 0 to 31 (pwm)
#define	IOCTL_PWM_VALUE				_IOWR(IOCTL_LPC_MAGIC, 4, short)	//| XX | XX | in/out | pwm_value | -> INPUT=0 OUTPUT=1 
#define	IOCTL_SET_MAX_MIN_TEMP		_IOWR(IOCTL_LPC_MAGIC, 5, int)	// a address of short -> (Max_byte|Min_byte)
#define	IOCTL_SET_RGB				_IOWR(IOCTL_LPC_MAGIC, 6, Rrgb_Value_t)	// rgb (5bit for each color)
#define	IOCTL_WAKEUP_IR_CODE		_IOWR(IOCTL_LPC_MAGIC, 7, short)	// ir_code to wakeup the device with lpc */
#define	IOCTL_READ_BYTE				_IOWR(IOCTL_LPC_MAGIC, 100, int)	// ir_code to wakeup the device with lpc */

#endif// __LPD_I2C_H


