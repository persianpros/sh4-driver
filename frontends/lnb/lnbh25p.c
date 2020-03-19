/*
 *   lnbh25p.c - ST Microelectronics LNBH25P LNB power controller
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
#define TAGDEBUG "[LNB-LNBH25P] "

int lnbh25p_write_reg(struct i2c_client *client, unsigned char reg, unsigned char data)
{
	int ret = 0;
	unsigned char *_data = (unsigned char*)&data;
	unsigned char write_bytes[2] = { reg, _data[0] };

	dprintk(50, "%s write 0x%02x to register 0x%02x>\n", __func__, data, reg);

	ret = i2c_master_send(client, write_bytes, 2);
	if (ret != 2)
	{
		dprintk(1, "Error writing 0x%02x to register 0x%02x\n", data, reg);
		return -EFAULT;
	}

	dprintk(50, "< OK, %d registers written\n", ret - 1);
	return ret;
}

int lnbh25p_writeb(struct i2c_client *client, unsigned char byte)
{
	int ret = 0;

	dprintk(50, "%s write 0x%02x >\n", __func__, byte);

	ret = i2c_master_send(client, &byte, 1);
	if (ret != 1)
	{
		dprintk(1, "Error writing 0x%02x\n", byte);
		return -EFAULT;
	}

	dprintk(50, "< OK, %d bytes written\n", ret);
	return ret;
}

unsigned short lnbh25p_read_status(struct i2c_client *client)
/* A read sequence is started by first writing the chip address followed by
 * the first register number to be read.
 *
 * The LNBH25P registers can then be read by reading by reading the chip
 * address and will send the contents of the  register asked for as first
 * byte after which the remaining bytes of the registers following it are
 * sent. Transmission by the LNBH25P will stop when a stop bit is sent.
 *
 * This routine always reads two bytes starting at register zero,
 * which results in reading the two status registers.
 */
{
	unsigned char status;

	dprintk(50, "%s >\n", __func__);

	lnbh25p_writeb(client, 0); // write starting register number

	status = 0;

	if (2 != i2c_master_recv(client, &status, 2))
	{
		dprintk(1, "Error reading status data\n");
		return -1;
	}

	dprintk(50, "%s < OK, status = 0x%04x\n", __func__, status);

	return status;
}

int lnbh25p_command_kernel(struct i2c_client *client, unsigned int cmd, void *arg)
{
//	int i;
	unsigned char data_register = 0;

	dprintk(50, "%s (%x)\n", __func__, cmd);

	switch (cmd)
	{
		case LNB_VOLTAGE_OFF:
		{
			dprintk(20, "Switch LNB power off\n");
			data_register = 0b00000000;  // LNB power off
			break;
		}
		case LNB_VOLTAGE_VER:
		{
			dprintk(20, "Set LNB voltage vertical\n");
			data_register = 0b00000001;  // LNB voltage = 13.000 V
			break;
		}
		case LNB_VOLTAGE_HOR:
		{
			dprintk(20, "Set LNB voltage horizontal\n");
			data_register = 0b00001000;  // LNB voltage = 18.150 V
			break;
		}
		default:
		{
			dprintk(1, "Unknown LNB IOCTL 0x%08x (LNB power off)\n", cmd);
			data_register = 0b00000000;  // LNB voltage = 0V
			break;
		}
	}
//	if ((LNB_VOLTAGE_1V) && (data_register & 0b00001111))
//	{
//		data_register += 0b00000011;  // add 1V to LNB voltage
//	}

//	for (i = 0; i < 4; i++)
//	{
		lnbh25p_write_reg(client, 2, data_register);
//	}
	return 0;
}

int lnbh25p_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	return lnbh25p_command_kernel(client, cmd, NULL);
}

/*
 * The LNBH25PQR is connected with its ADDR pin to GND -> I2C address is 0x10 >> 1 = 0x08
 *
 * There are two status registers (register numbers 0 and 1, read only) and four internal control registers.
 * All are accessed by successive reading/writing in sequence to I2C address 0x08,
 * after first writing the first register number to be read/written.
 *
 * At power on all control register bits are set to zero
 *
 * Register DATA1: b7 b6 b5 b4 b3 b2 b1 b0 (register #2)
 *                  |        |  |        |
 *                  |        |  ---------------- LNB voltage select
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
 *                  ---------------------------- Reserved, to be written as 0
 *
 * Register DATA2: b7 b6 b5 b4 b3 b2 b1 b0 (register #3)
 *                  |           |  |  |  |
 *                  |           |  |  |  ------- 22kHz tone enable, 0=off, 1=external control
 *                  |           |  |   --------- Low power mode enable, 1=active, 0=off
 *                  |           |  ------------- DSQIN is external 22kHz (1) or gate (0)
 *                  |           |
 *                  ---------------------------- Reserved, to be written as 0
 *
 * Register DATA3: b7 b6 b5 b4 b3 b2 b1 b0 (register #4)
 *                  |        |  |  |  |  |
 *                  |        |  |  |  |  ------- current limit is max (0) or normal (1)
 *                  |        |  |  |  ---------- DC-DC inductor max. switching is 2.5A(1) or 4A(0)
 *                  |        |  |  ------------- Dynamic LNB current limit is on(0) or off(1)
 *                  |        |  ---------------- Dynamic CL timer is 90ms(0) or 180ms(1)
 *                  |        |
 *                  ---------------------------- Reserved, to be written as 0
 *
 * Register DATA4: b7 b6 b5 b4 b3 b2 b1 b0 (register #5)
 *                  |  |  |  |  |  |  |  |
 *                  |  |  |  |  |  |  |  ------- IMON diagnostic mode on(1) or off(0)
 *                  |  |  |  |  |  |   --------- Reserved (don't care)
 *                  |  |  |  |  |  ------------- Reserved (don't care)
 *                  |  |  |  |  ---------------- Auto overload recover yes(0) or no(1)
 *                  |  |  ---------------------- Reserved  (don't care)
 *                  |  ------------------------- Auto thermal overload recovery yes(0) or (1)
 *                  ---------------------------- DC-DC converter uses low ESR capacitor yes(0) or no(1)
 */

int lnbh25p_init(struct i2c_client *client)
{
	int i;
	unsigned char data_register[4];

	dprintk(10, "%s\n", __func__);

	data_register[0] = 0b00000000;  // LNB power off
	data_register[1] = 0b00000001;  // 22kHz internal, low power mode off, DSQIN is 22kHz gate
	data_register[2] = 0b00000011;  // normal current limit, 2.5A max switch current, dynamic current on, 90ms dynamic CL time
	data_register[3] = 0b00000000;  // IMON off, auto overload recovery, auto thermal recovery, low ESR capacitor

	for (i = 0; i < 4; i++)
	{
		lnbh25p_write_reg(client, i + 2, data_register[i]);
	}
	dprintk(1, "%s LNB power is off\n", __func__);
#if 0  // test routine
	msleep(4000);
	lnbh25p_command_kernel(client, LNB_VOLTAGE_VER, NULL);
	dprintk(1, "%s LNB voltage is 13V\n", __func__);
	msleep(4000);
	lnbh25p_command_kernel(client, LNB_VOLTAGE_HOR, NULL);
	dprintk(1, "%s LNB voltage is 18V\n", __func__);
	msleep(4000);
	for (i = 0; i < 4; i++)
	{
		lnbh25p_write_reg(client, i + 2, data_register[i]);
	}
	dprintk(1, "%s LNB power is off\n", __func__);
#endif
	return 0;
}
// vim:ts=4
