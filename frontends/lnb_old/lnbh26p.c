/*
 *   lnbh26p.c - ST Microelectronics LNBH26P LNB power controller
 *
 *   Written by Audioniek, based on A8293 code.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/types.h>

#include <linux/i2c.h>

#include "lnb_core.h"
#define TAGDEBUG "[LNB-LNBH26P] "

int lnbh26p_write_reg(struct i2c_client *client, unsigned char reg, unsigned char data)
{
	int ret = 0;
	unsigned char *_data = (unsigned char*)&data;
	unsigned char write_bytes[2] = { reg, _data[0] };

//	dprintk(20, "%s write 0x%02x to register 0x%02x>\n", __func__, data, reg);

	ret = i2c_master_send(client, write_bytes, 2);
	if (ret != 2)
	{
		dprintk(1, "Error writing 0x%02x to register 0x%02x\n", data, reg);
		return -EFAULT;
	}

//	dprintk(20, "< OK, %d registers written\n", ret - 1);
	return ret;
}

int lnbh26p_writeb(struct i2c_client *client, unsigned char byte)
{
	int ret = 0;

	ret = i2c_master_send(client, &byte, 1);

	if (ret != 1)
	{
		dprintk(1, "Error writing 0x%02x\n", byte);
		return -EFAULT;
	}
	return ret;
}

unsigned short lnbh26p_read_status(struct i2c_client *client)
/* A read sequence is started by first writing the chip address followed by
 * the first register number to be read.
 *
 * The LNBH26P registers can then be read by reading the chip address and
 * the LNBH26P will send the contents of the register asked for as first
 * byte after which the remaining bytes of the registers following it are
 * sent. Transmission by the LNBH26P will stop when a stop bit is sent.
 *
 * This routine always reads two bytes starting at register zero,
 * which results in reading the two status registers.
 */
{
	unsigned char status;

//	dprintk(10, "%s >\n", __func__);

	lnbh26p_writeb(client, 0); //write starting register number

	status = 0;

	if (2 != i2c_master_recv(client, &status, 2))
	{
		dprintk(1, "Error reading status data\n");
		return -1;
	}

//	dprintk(10, "%s < OK, status = 0x%04x\n", __func__, status);

	return status;
}

unsigned char lnbh26p_read_reg(struct i2c_client *client, int reg)
{
	unsigned char data = 0;

	lnbh26p_writeb(client, reg); //write starting register number

	if (1 != i2c_master_recv(client, &data, 1))  //read register
	{
		dprintk(1, "Error reading data register 2\n");
		return -1;
	}

//	dprintk(10, "%s < OK, register #%d data = 0x%02x\n", __func__, 2, data);

	return data;
}

int lnbh26p_command_kernel(struct i2c_client *client, unsigned int cmd, void *arg)
{
	unsigned char data_register = 0;
	unsigned char register2;

//	dprintk(20, "%s (%x)\n", __func__, cmd);

	register2 = lnbh26p_read_reg(client, 2);

	switch (cmd)
	{
		case LNB_VOLTAGE_OFF:
		{
			dprintk(20, "Switch LNB power off (tuner 1)\n");
			data_register = (register2 & 0b00001111) | 0b00000000; // LNB1 power off
			break;
		}
		case LNB_VOLTAGE_VER:
		{
			dprintk(20, "Set LNB voltage vertical (tuner 1)\n");
			data_register = (register2 & 0b00001111) | 0b00010000; // LNB1 voltage = 13.000 V
			break;
		}
		case LNB_VOLTAGE_HOR:
		{
			dprintk(20, "Set LNB voltage horizontal (tuner 1)\n");
			data_register = (register2 & 0b00001111) | 0b10000000; // LNB1 voltage = 18.150 V
			break;
		}
		case LNB_VOLTAGE_OFF_2:
		{
			dprintk(10, "Switch LNB power off (tuner 2)\n");
			data_register = (register2 & 0b11110000) | 0b00000000; // LNB2 power off
			break;
		}
		case LNB_VOLTAGE_VER_2:
		{
			dprintk(10, "Set LNB voltage vertical (tuner 2)\n");
			data_register = (register2 & 0b11110000) | 0b00000001; // LNB2 voltage = 13.000 V
			break;
		}
		case LNB_VOLTAGE_HOR_2:
		{
			dprintk(10, "Set LNB voltage horizontal (tuner2)\n");
			data_register = (register2 & 0b11110000) | 0b00001000; // LNB2 voltage = 18.150 V
			break;
		}
		default:
		{
			dprintk(10, "Unknown LNB IOCTL 0x%08x (LNB power off, both tuners)\n", cmd);
			data_register = 0b00000000; // LNB1/2 voltage = 0V
			break;
		}
	}
//	if ((LNB_VOLTAGE_1V)
//	{
//		if (data_register & 0b00001111)
//		{
//			data_register += 0b00000011; //add 1V to LNB1 voltage
//		}
//		if (data_register & 0b11110000)
//		{
//			data_register += 0b00110000; //add 1V to LNB2 voltage
//		}
//	}
	lnbh26p_write_reg(client, 2, data_register);

	return 0;
}

int lnbh26p_command(struct i2c_client *client, unsigned int cmd, void *arg )
{
	return lnbh26p_command_kernel(client, cmd, NULL);
}

/*
 * The LNBH26PQR is connected with its ADDR pin to GND -> I2C address is 0x10>>1 = 0x08
 *
 * There are two status registers (register numbers 0 and 1, read only) and four internal control registers
 * All are accessed by successive reading/writing in sequence to I2C address 0x08,
 * after first writing the first register number to be read/written.
 *
 * At power on all control register bits are set to zero
 *
 * Note: in the Fortis EPP8000 tuner 1 is connected to LNBH26P output B, tuner 2 to output A
 *
 * Register DATA1: b7 b6 b5 b4 b3 b2 b1 b0 (register #2)
 *                  |        |  |        |
 *                  |        |  ---------------- LNB voltage select for output A
 *                  |        |  0  0  0  0       output off (0 V)
 *                  |        |  0  0  0  1       13.000 V -----------
 *                  |        |  0  0  1  0       13.333 V           |
 *                  |        |  0  0  1  1       13.667 V           |
 *                  |        |  0  1  0  0       14.000 V           |- Vertical
 *                  |        |  0  1  0  1       14.333 V           |
 *                  |        |  0  1  1  0       14.667 V           |
 *                  |        |  0  1  1  1       15.000 V -----------
 *                  |        |  1  0  0  0       18.150 V -----------
 *                  |        |  1  0  0  1       18.483 V           |
 *                  |        |  1  0  1  0       18.817 V           |
 *                  |        |  1  0  1  1       19.150 V           |
 *                  |        |  1  1  0  0       19.483 V           |- Horizontal
 *                  |        |  1  1  0  1       19.817 V           |
 *                  |        |  1  1  1  0       20.150 V           |
 *                  |        |  1  1  1  1       20.483 V -----------
 *                  |        |
 *                  ---------------------------- LNB voltage select for output B
 *                  0  0  0  0                   output off (0 V)
 *                  0  0  0  1                   13.000 V -----------
 *                  0  0  1  0                   13.333 V           |
 *                  0  0  1  1                   13.667 V           |
 *                  0  1  0  0                   14.000 V           |- Vertical
 *                  0  1  0  1                   14.333 V           |
 *                  0  1  1  0                   14.667 V           |
 *                  0  1  1  1                   15.000 V -----------
 *                  1  0  0  0                   18.150 V -----------
 *                  1  0  0  1                   18.483 V           |
 *                  1  0  1  0                   18.817 V           |
 *                  1  0  1  1                   19.150 V           |
 *                  1  1  0  0                   19.483 V           |- Horizontal
 *                  1  1  0  1                   19.817 V           |
 *                  1  1  1  0                   20.150 V           |
 *                  1  1  1  1                   20.483 V -----------
 *
 * Register DATA2: b7 b6 b5 b4 b3 b2 b1 b0 (register #3)
 *                  |  |  |  |  |  |  |  |
 *                  |  |  |  |  |  |  |  ------- 22kHz tone enable, 0=off, 1=external control --
 *                  |  |  |  |  |  |   --------- Low power mode enable, 1=active, 0=off        |
 *                  |  |  |  |  |  ------------- DSQIN-A is external 22kHz (1) or gate (0)     | -Output A
 *                  |  |  |  |  ---------------- Reserved, to be written as 0                 --
 *                  |  |  |  ------------------- 22kHz tone enable, 0=off, 1=external control --
 *                  |  |  ---------------------- Low power mode enable, 1=active, 0=off        |
 *                  |  ------------------------- DSQIN-B is external 22kHz (1) or gate (0)     | -Output B
 *                  ---------------------------- Reserved, to be written as 0                 --
 *
 * Register DATA3: b7 b6 b5 b4 b3 b2 b1 b0 (register #4)
 *                  |  |  |  |  |  |  |  |
 *                  |  |  |  |  |  |  |  ------- current limit is max (0) or normal (1)            --
 *                  |  |  |  |  |  |  ---------- DC-DC inductor max. switching is 2.5A(1) or 4A(0)  |
 *                  |  |  |  |  |  ------------- Dynamic LNB current limit is on(0) or off(1)       | - Output A
 *                  |  |  |  |  ---------------- Dynamic CL timer is 90ms(0) or 180ms(1)           --
 *                  |  |  |  ------------------- current limit is max (0) or normal (1)            --
 *                  |  |  ---------------------- DC-DC inductor max. switching is 2.5A(1) or 4A(0)  |
 *                  |  ------------------------- Dynamic LNB current limit is on(0) or off(1)       | - Output B
 *                  ---------------------------- Dynamic CL timer is 90ms(0) or 180ms(1)           --
 *
 * Register DATA4: b7 b6 b5 b4 b3 b2 b1 b0 (register #5)
 *                  |  |  |  |  |  |  |  |
 *                  |  |  |  |  |  |  |  ------- IMON diagnostic mode on(1) or off(0)
 *                  |  |  |  |  |  |   --------- Reserved (write as zero)
 *                  |  |  |  |  |  ------------- Reserved (write as zero)
 *                  |  |  |  |  ---------------- Auto overload recover yes(0) or no(1)
 *                  |  |  ---------------------- Reserved (write as zero)
 *                  |  ------------------------- Auto thermal overload recovery yes(0) or (1)
 *                  ---------------------------- DC-DC converter uses low ESR capacitor yes(0) or no(1)
 */
int lnbh26p_init(struct i2c_client *client)
{
	int i;
	unsigned char data_register[4];
	
//	dprintk(10, "%s\n", __func__);

	data_register[0] = 0b00000000; // LNB power off for both tuners
	data_register[1] = 0b00010001; // 22kHz internal, low power mode off, DSQIN is 22kHz gate for both tuners
	data_register[2] = 0b00110011; // normal current limit, 2.5A max switch current, dynamic current on, 90ms dynamic CL time for both tuners
	data_register[3] = 0b00000000; // IMON off, auto overload recovery, auto thermal recovery, low ESR capacitor

	for (i = 0; i < 4; i++)
	{
		lnbh26p_write_reg(client, i + 2, data_register[i]);
	}

#if 0
	dprintk(1, "%s LNB power is off\n", __func__);
	msleep(4000);
	lnbh26p_command_kernel(client, LNB_VOLTAGE_VER, NULL);
	dprintk(1, "%s LNB voltage is 13V (tuner 1)\n", __func__);
	msleep(4000);
	lnbh26p_command_kernel(client, LNB_VOLTAGE_HOR, NULL);
	dprintk(1, "%s LNB voltage is 18V (tuner 1)\n", __func__);
	msleep(4000);
	lnbh26p_command_kernel(client, LNB_VOLTAGE_VER_2, NULL);
	dprintk(1, "%s LNB voltage is 13V (tuner 2)\n", __func__);
	msleep(4000);
	lnbh26p_command_kernel(client, LNB_VOLTAGE_HOR_2, NULL);
	dprintk(1, "%s LNB voltage is 18V (tuner 2)\n", __func__);
	msleep(4000);
	for (i = 0; i < 4; i++)
	{
		lnbh26p_write_reg(client, i + 2, data_register[i]);
	}
	dprintk(1, "%s LNB power is off\n", __func__);
#endif
	return 0;
}

