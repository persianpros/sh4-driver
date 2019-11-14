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

#ifndef __STARCI2WIN_I2C_H__
#define __STARCI2WIN_I2C_H__

#include <linux/kernel.h>
#include <linux/i2c.h>

#define I2C_DRIVERID_STARCI2WIN	 0x77
#define I2C_BUS			1


#define dprintk(args...) \
	do { \
		if (debug) printk("starci2win: " args); \
	} while (0)

/* Register of STARCI2WIN */
#if 0
	00 --> Module A Control Register
	01 --> Module A auto select mask high Register
	02 --> Module A auto select mask low Register
	03 --> Module A auto select pattern high Register
	04 --> Module A auto select pattern low Register
	05 --> Memory access A cycle time Register
	06 --> RFU
	07 --> RFU
	08 --> RFU
	09 --> Module B Control Register
	0A --> Module B auto select mask high Register
	0B --> Module B auto select mask low Register
	0C --> Module B auto select pattern high Register
	0D --> Module B auto select pattern low Register
	0E --> Memory access B cycle time Register
	0F --> RFU
	10 --> Single mode TS Control Register
	11 --> Twin mode TS Control Register
	12 --> External access auto select mask high Register
	13 --> External access auto select mask low Register
	14 --> External access auto select pattern high Register
	15 --> External access auto select pattern low Register
	16 --> RFU
	17 --> Destination select Register
	18 --> Power control Register
	19 --> RFU
	1A --> Interrupt Status Register
	1B --> Interrupt Mask Register
	1C --> Interrupt Config Register
	1D --> Microprocessor Interface Config Register
	1E --> Microprocessor wait/ack Config Register
	1F --> StarCI2Win Control Register
#endif

#define	MOD_A_CTRL			0x00
#define	MOD_A_ACCESS_TIME	0x05
#define	MOD_B_CTRL			0x09
#define	MOD_B_ACCESS_TIME	0x0E
#define	SINGLE_MODE_TS		0x10
#define	TWIN_MODE_TS		0x11
#define	DEST_MODULE			0x17
#define	POWER_CTRL			0x18
#define	MP_INTERFACE_CONFIG	0x1D
#define	STARCI2WIN_CTRL		0x1F

/* For detect the source of the interrupt */
#define	MASK_DET			0x03
#define	DET_A				0x01
#define	DET_B				0x02

//Bit masks
#define TSIEN                   0x20
#define TSOEN                   0x40
#define TSO                     0x01
#define TSI(x)                  (x<<1)
#define ALL_TS1                 0x20
#define ALL_TS2                 0x10
#define TSMAP                   0x20
#define MTSI(x)                 (x<<1)
#define TWIN                    0x80
#define TS1B                    0x10
#define TS2B                    0x08



int attach_adapter(struct i2c_adapter *);
int detach_client(struct i2c_client *);
int starci2win_command(struct i2c_client *, unsigned int, void *);

unsigned char read_i2c(unsigned short reg_i2c);
int write_i2c(unsigned short reg_i2c,unsigned char dat);

#endif ///__STARCI2WIN_I2C_H__


