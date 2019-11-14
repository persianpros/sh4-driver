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

#ifndef __STARCI2WIN_H__
#define __STARCI2WIN_H__

#include <linux/ioctl.h>

typedef struct
{
	unsigned char module;			/* Module A, Module B, EXT*/
	unsigned char configuration;	/* CIS mode or "Normal" mode */
}Module_t;

typedef struct
{
	unsigned short offset_addr;	/* Module A, Module B, EXT*/
	unsigned char value;		/* CIS mode or "Normal" mode */
}Register_t;

typedef struct
{
	unsigned type_dev;
	unsigned char inuse;
	unsigned char isInit;
	unsigned char status;
}State_ci_t;

typedef struct
{
	unsigned char module;			/* Module A, Module B, EXT*/
	unsigned char ts_number;		/* TS through Cam */
	unsigned char configuration;	/* CIS mode or "Normal" mode */
}Module_and_ts_t;

typedef struct
{
        unsigned char mode;                     //1: Single     2: Twin
        unsigned char tsin_1;                   //0: None       1: Ts0      2: Ts2
        unsigned char tsin_2;                   //0: None       1: Ts1      2: Ts2
        unsigned char cam_1;                    //0: None       1: A        2: B        3: A+B
        unsigned char cam_2;                    //0: None       1: A        2: B        3: A+B
}Ci_Mode_BackBox;


#define SINGLE_M     1
#define TWIN_M       2
#define NONE_S       0
#define TS1         1
#define TS2         1
#define TS3         2
#define CAM_A       1
#define CAM_B       2
#define CAM_AB      3


#define TS_0		0
#define	TS_1		1

enum
{
	CI=0,
	INPUT
};

#define	STARCI_DEVICE_NAME				"STARCI2WIN"  
#define CI_DEVICE_NAME  	    		"ci"
//#define INPUT_DEVICE_NAME  	    		"ci_input"

#define	CI_NUMBER_OF_CONTROLLERS		2


#define STARCI_MAJOR_NUM        		170
#define STARCI_MINOR_START      		0

#define IOCTL_STARCI_MAGIC				't'
#define IOCTL_SET_MODE_MODULE			_IOWR(IOCTL_STARCI_MAGIC, 1, Module_t)
#define	IOCTL_GET_MODE_MODULE			_IOWR(IOCTL_STARCI_MAGIC, 2, Module_t)
#define IOCTL_READ_CIS					_IOWR(IOCTL_STARCI_MAGIC, 3, Module_t)
#define IOCTL_WRITE_ADDR				_IOWR(IOCTL_STARCI_MAGIC, 4, Module_t)
#define IOCTL_READ_ADDR					_IOWR(IOCTL_STARCI_MAGIC, 5, Module_t)
#define IOCTL_DETECT_MODULE				_IOWR(IOCTL_STARCI_MAGIC, 6, Module_t)

#define IOCTL_CI_SET_POWER				_IOWR(IOCTL_STARCI_MAGIC, 7, Module_t)
#define IOCTL_CI_GET_POWER				_IOWR(IOCTL_STARCI_MAGIC, 8, Module_t)
#define IOCTL_CI_SWF_RESET				_IOWR(IOCTL_STARCI_MAGIC, 9, Module_t)
#define IOCTL_CI_SET_CIS_IO_SPACE_MODE	_IOWR(IOCTL_STARCI_MAGIC, 10, Module_t)
#define	IOCTL_CI_SET_TS_MODE			_IOWR(IOCTL_STARCI_MAGIC, 11, Module_and_ts_t)
#define	IOCTL_CI_GET_TS_MODE			_IOWR(IOCTL_STARCI_MAGIC, 12, Module_and_ts_t)
#define	IOCTL_SWITCH_MODULE				_IOWR(IOCTL_STARCI_MAGIC, 13, Module_t)
#define	IOCTL_RST_ALL_MODE_MODULE		_IOWR(IOCTL_STARCI_MAGIC, 14, Module_t)

#define	IOCTL_ACTIVATION_MODULE_RST		_IOWR(IOCTL_STARCI_MAGIC, 15, Module_t)

#define	IOCTL_CI_MODE_FROM_BLACKBOX		_IOWR(IOCTL_STARCI_MAGIC, 16, Ci_Mode_BackBox)

#define	IOCTL_READ_I2C					_IOWR(IOCTL_STARCI_MAGIC, 100, Module_t)
#define	IOCTL_WRITE_I2C					_IOWR(IOCTL_STARCI_MAGIC, 101, Module_t)


#define STARTCI_IO_SIZE         	0x1000
#define STARCI_BASE_ADDRESS     	0x2000000

#define STARCI_REG_DATA				0x0
#define STARCI_REG_CTRL_STATUS		0x1
#define STARCI_REG_SIZE_LSB			0x2
#define STARCI_REG_SIZE_MSB			0x3

/* STATUS REGISTER: -> | DA | FR | R | R | R | R | WE | RE | */
/*	Define of the bits Status Register	*/
#define	DA_BIT_STATUS				0x80
#define	FR_BIT_STATUS				0x40
#define	WE_BIT_STATUS				0x02
#define	RE_BIT_STATUS				0x01

/* COMMAND REGISTER: -> | R | R | R | R | RS | SR | SW | HC | */
/*	Define of the bits Command Register	*/
#define	RS_BIT_COMMAND				0x08
#define	SR_BIT_COMMAND				0x04
#define	SW_BIT_COMMAND				0x02
#define	HC_BIT_COMMAND				0x01



#define 	MODULE_EXTRACTED		0x02
#define 	MODULE_PRESENT			0x01
#define 	MODULE_NOT_PRESENT		0x00



#define		POWER_ON				0x01
#define		POWER_OFF				0x00


#define		CIS_LEN					256

/******		Module indicator	******/
#define			NONE				0xAA
#define			MODULE_A			0x00
#define			MODULE_B			0x01
#define			MODULE_EXT			0x02

/******		Access module mode	******/
#define			IO_SPACE			0x00
#define			CIS					0x01
#define			MODE_MASK			0x0C

#define			TS_BY_PASS			0x02	/* TSIN->TSOUT */
#define			TS_TO_CAM			0x03	/* TSIN->Module->TSOUT */

#define			DEST_MOD_A			0x02
#define			DEST_MOD_B			0x04

unsigned char detect_module(unsigned char slotid);

#endif ///__STARCI2WIN_H__



