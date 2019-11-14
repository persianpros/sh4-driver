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
#include <linux/kernel.h>
#include <linux/i2c.h>

#define I2C_DRIVERID_STV6414	 0x78
#define I2C_BUS			0

#define	STV6414_DEVICE_NAME				"stv6414_0"  
#define	STV6414_NUMBER_OF_CONTROLLERS	1

typedef struct
{
	unsigned short offset_addr;
	unsigned char value;
}Register_t;

#define STV6414_MAJOR_NUM        	177
#define STV6414_MINOR_START      	0

#define	REG00_AUDIO_OUTPUT			0x00
#define	REG01_AUDIO_SELECTION		0x01
#define	REG02_VIDEO_SELECTION		0x02
#define	REG03_VIDEO_RGB				0x03
#define	REG04_MISC_CTRL				0x04
#define	REG05_OUTPUT_FORMAT			0x05
#define	REG06_STANDBY				0x06

/* Format */
#if 0
	|VCR format|TV format| ???????? |
	|  7   6   |  5   4  |	3 2 1 0 |
	|  0   0   |  0   0  |  x x x x |	--->	VCR e TV Input
	|  0   1   |  0   1  |  x x x x |	--->	VCR e TV Output < 2
	|  1   0   |  1   0  |  x x x x |	--->	VCR e TV Output 16/9
	|  1   1   |  1   1  |  x x x x |	--->	VCR e TV Output 4/3
#endif


#define	TV_16_9						0x20
#define	TV_4_3						0x30
#define	MASK_VCR_TV_FORMAT			0xF0

#define	STANDBY_OFF					0x00
#define STANDBY_ON					0x01

#define IOCTL_STV6414_MAGIC					'S'
#define	IOCTL_READ_I2C						_IOW(IOCTL_STV6414_MAGIC, 1, Register_t)
#define	IOCTL_WRITE_I2C						_IOW(IOCTL_STV6414_MAGIC, 2, Register_t)
#define IOCTL_READ_LAST_WRITE_VALUE_OF_REG	_IOW(IOCTL_STV6414_MAGIC, 3, Register_t)
#define	IOCTL_SET_169_43					_IOW(IOCTL_STV6414_MAGIC, 4, int)
#define	IOCTL_STANDBY_ON_OFF				_IOW(IOCTL_STV6414_MAGIC, 5, int)
#define	IOCTL_DISABLE_TV_OUTPUT				_IOW(IOCTL_STV6414_MAGIC, 6, int)



