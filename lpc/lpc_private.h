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
#ifndef __LPD_PRIVATE_H
#define __LPD_PRIVATE_H

#define PINFO(fmt, args...)         printk(KERN_INFO " " fmt, ##args)
#define PERROR(fmt, args...)        printk(KERN_ERR "[%s]: " fmt, __FUNCTION__, ##args)

#if defined QBOXHD_MINI
#define I2C_BUS			1
#else
#define I2C_BUS			0
#endif

#define	I2C_DRIVERID_LPC			0x71	/* the same of stv6414 + 1 */
	
#define	LPC_DEVICE_NAME				"lpc_0"  
#define	LPC_NUMBER_OF_CONTROLLERS	1

#define LPC_MAJOR_NUM        		179	/* a free Major */
#define LPC_MINOR_START      		0

//#define IR_CODE						0xDFA00000	//old remote
#define	IR_CODE						0xFF5F0000	//new remote
/****************************************/
/*				Register				*/
/****************************************/
/* Read regs */
#define READ_VERSION				0x01
#define	READ_TEMP					0x02
#define	READ_RTC_TIME				0x10
#define	READ_TIMER					0x14
#define	SECS_FOR_TIME				0x1B
#define	READ_LED_R					0x20
#define	READ_LED_G					0x21
#define	READ_LED_B					0x22
#define	READ_BRG					0x23
#define	READ_IR_ADDR				0x30
#define	READ_IR_PWR					0x31
#define	READ_TEMP_MAX				0x32
#define	READ_TEMP_MIN				0x33
#define	READ_FAN_PWR				0x34

/* Write regs */
#define	TIMER_ON_STB				0x15
#define	SET_RGB						0x17
#define	MODIFY_TIME_DATA			0x1A
#define	SECS_FOR_TIME				0x1B
#define	STB_OFF						0x1C
#define	FAN_POWER					0x22
#define	PWM_VALUE					0x23
#define	CONFIG_RCU_PARAM			0x25
#define	CONFIG_FAN_CTL_PAR			0x2A

typedef struct
{
	unsigned char status_pin;	
	unsigned char pwm_value;
} Pwm_Info_t;


/* Structure for global status */
typedef struct
{
	unsigned char temp_cur;
	unsigned char temp_max;
	unsigned char temp_min;
	unsigned char fan_power;
	Pwm_Info_t pwm;
	unsigned char version_lpc;
	int ir_code;
} Data_Support_t;

#endif

