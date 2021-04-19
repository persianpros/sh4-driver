/*
 * TI SN761640 DVB-T(2) tuner driver
 *
 * Version for Fulan Spark7162 (Frontend Sharp ????1ED5469)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Remarks:
 * If the authors dataheet of the Texas Instrument SN761640 tuner is
 * correct, this driver has an issue:
 * Byte 6 is never written to the tuner. This results in the routine
 * sn761640_set_params using a write sequence (bytes 1-5) not allowed
 * by the data sheet.
 *
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "dvb_frontend.h"
#include "sn761640.h"

extern int paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[sn761640t] "

struct sn761640t_state
{
	struct dvb_frontend          *fe;
	struct i2c_adapter           *i2c;
	const struct sn761640_config *config;

	u32                          frequency;  // note: also in sn761640_config
	u32                          bandwidth;  // note: also in sn761640_config
};

/*******************************************
 *
 * I2C routines
 *
 */
static int sn761640_i2c_read(struct sn761640t_state *state, u8 *buf)
{
	const struct sn761640_config *config = state->config;
	int err = 0;
	struct i2c_msg msg = { .addr = config->addr, .flags = I2C_M_RD, .buf = buf, .len = 2 };

//	dprintk(70, "%s: state->i2c=<0x%x>, config->addr = 0x%02x\n", __func__, state->i2c, config->addr);

	err = i2c_transfer(state->i2c, &msg, 1);
	if (err != 1)
	{
		goto exit;
	}
	return err;

exit:
	dprintk(1, "%s: I/O Error err=<%d>\n", __func__, err);
	return err;
}

static int sn761640_i2c_write(struct sn761640t_state *state, u8 *buf, u8 length)
{
	const struct sn761640_config *config = state->config;
	int err = 0;
	struct i2c_msg msg = { .addr = config->addr, .flags = 0, .buf = buf, .len = length };

#if 0
	if (paramDebug >= 50)
	{
		int i;

		dprintk(1, "%s: I2C message =", __func__);
		for (i = 0; i < length; i++)
		{
			printk(" 0x%02x", buf[i]);
		}
		printk(", addr = 0x%02x\n",config->addr);
	}
#endif
	err = i2c_transfer(state->i2c, &msg, 1);
	if (err != 1)
	{
		goto exit;
	}
	return err;

exit:
	dprintk(1, "%s: I/O Error err=<%d>\n", __func__, err);
	return err;
}

static int sn761640_get_status(struct dvb_frontend *fe, u32 *status)
{
	struct sn761640t_state *state = fe->tuner_priv;
	u8 result[2] = { 0 };
	int err = 0;

	*status = 0;
	err = sn761640_i2c_read(state, result);
//	dprintk(70, "%s: result[0] = %d\n", __func__, result[0]);
	if (err < 0)
	{
		goto exit;
	}
	if (result[0] & 0x40)
	{
		dprintk(20, "Tuner is locked\n");
		*status = 1;
	}
	return err;

exit:
	dprintk(1, "%s: I/O Error\n", __func__);
	return err;
}

/*
 *  Data read format of the TI SN761640
 *
 *  byte1:   1   |   1   |   0   |   0   |   0   |  MA1  |  MA0  |   1    Address byte
 *  byte2:  POR  |   FL  |   1   |   1   |   X   |   A2  |   A1  |  A0
 *
 *  byte1 = address
 *  byte2:
 *	POR  = Power on Reset (Vcc H=<2.2v L>2.2v)
 *	FL   = Phase Lock (H=lock L=unlock)
 *	A0-2 = ADC digital data (P5=0)
 *	X    = Don't care (ignore)
 *
 * Only POR can be used to check the tuner is present
 *
 * Caution: after reading byte2 the IC reverts to write mode;
 *          continuing to read may corrupt tuning data.
 */

/*
 *  Data write format of the TI SN761640
 *
 *  byte1:   1   |   1   |   0   |   0   |   0   |  MA1  |  MA0  |  0       Address byte   (ADB)
 *  byte2:   0   |  N14  |  N13  |  N12  |  N11  |  N10  |   N9  |  N8      Divider byte 1 (DB1)
 *  byte3:   N7  |   N6  |   N5  |   N4  |   N3  |   N2  |   N1  |  N0      Divider byte 2 (DB2)
 *  byte4:   1   |   0   |  ATP2 |  ATP1 |  ATP0 |  RS2  |  RS1  |  RS0     Control byte 1 (CB1)
 *  byte5:  CP1  |  CP0  |  AISL |   P5  |  BS4  |  BS3  |  BS2  |  BS0     Band switch byte (BB)
 *  byte6:   1   |   1   |  ATC  |  MODE |DISCGA | IFDA  |  CP2  |  XLO     Control byte 2 (CB2)
 *
 *  byte1 = address
 *
 *  Possible write orders
 *  1) byte1 -> byte2 -> byte3 -> byte4 -> byte5 > byte6
 *  2) byte1 -> byte2 -> byte3
 *  3) byte1 -> byte4 -> byte5 -> byte6
 *  4) byte1 -> byte4 -> byte5
 *  5) byte1 -> byte6
 *
 * Note that driver uses the sequence  byte1 -> byte2 -> byte3 -> byte4 -> byte5 to
 * set up the tuner. This sequence is not specified as valid.
 *
 */

static long sn761640_calculate_mop_xtal(void)
{
	return 4000;  // kHz
}

/**************************************************
 *
 * Determine charge bump and band switch bits for
 * byte5 (BB).
 *
 */
static void sn761640_calculate_mop_uv_cp(u32 freq, int *cp, int *uv)
{
//	int i;
//	int cp_value = 350;
//	int *CP_DATA = kzalloc(sizeof(int) * 601, GFP_KERNEL);

	/* charge pump lib */
//	for (i = 0; i <= 600; i++)
//	{
//		CP_DATA[i] = 0;
//	}
//	CP_DATA[350] = 2;  // 350 uA
//	CP_DATA[600] = 3;  // 600 uA

	if (freq >= 147000 && freq < 430000)
	{
		*uv = 2;  // band is VHF Hi (note: sets BS3 low)
		if (freq < 400000)
		{
			*cp = 2;  // 350 uA
		}
		else
		{
			*cp = 3;  // 600 uA
		}
	}
	else if (freq >= 430000)
	{
		*uv = 4;  // band is UHF (note: sets BS3 high)
		if (freq < 763000)
		{
			*cp = 2;
		}
		else
		{
			*cp = 3;
		}
	}
//	*cp = CP_DATA[cp_value];
//	kfree(CP_DATA);
}

/**************************************************
 *
 * Calculate step size from byte4 in kHz / 10
 *
 * Basic formula: Fxtal / (step ratio + .5)
 *
 * Note: always uses mop_step ratio = 24,
 *       assuming RS2-0 will be set to 000b
 *
 */
static long sn761640_calculate_mop_step(int *byte)
{
	long mop_step_ratio = 24, mop_freq_step;

#if 1
	// assumes RS2-0 will bet set to 000b later on
#else
	int RS210 = 0;

	RS210 = (byte[3] & 0x07);  // get byte4 RS bits

	switch (RS210)
	{
		case 0:
		default:
		{
			mop_step_ratio = 24;
			break;
		}
		case 1:
		{
			mop_step_ratio = 28.;
			break;
		}
		case 2:
		{
			mop_step_ratio = 50.;
			break;
		}
		case 3:
		{
			mop_step_ratio = 64.;
			break;
		}
		case 4:
		{
			mop_step_ratio = 128.;
			break;
		}
		case 5:
		{
			mop_step_ratio = 80.;
			break;
		}
	}
#endif
	mop_freq_step = ((long)(sn761640_calculate_mop_xtal() * 10000) / mop_step_ratio + 5);  // kHz
	return mop_freq_step;
}

u64 __udivdi3(u64 n, u64 d);


/***********************************************************
 *
 * Calculate divider bytes (byte2 & byte3)
 *
 */
static void sn761640_calculate_mop_divider(u32 freq, int *byte)
{
	long data;
	u64  i64Freq;

	i64Freq = (u64)freq * 100000;
//	i64Freq += (u64)3616666700;  // satisfy
	i64Freq += 3616666700LL;  // compiler
	i64Freq = __udivdi3(i64Freq, sn761640_calculate_mop_step(byte));
	i64Freq += 5;
	i64Freq /= 10;
	data = (long)i64Freq;
//	dprintk(150, "%s: data = %ld\n", __func__, data);
	*(byte + 1) = (int)((data >> 8) & 0x7F);  // byte2 (DB1), MSbit clear
	*(byte + 2) = (int)(data & 0xFF);         // byte3 (DB2)
}

/***********************************************************
 *
 * Set bandwidth bit P5 in byte5 (BB)
 *
 */
static void sn761640_calculate_mop_bw(u32 baud, int *byte)
{
	if (baud > 7500)  // BW = 8M
	{
		byte[4] |= 0x10;  // BW setting: set P5
	}
	else if (baud <= 6500)  // BW = 6M
	{
		byte[4] &= 0xEF;  // BW setting: clear P5
//		byte[4] |= 0x00;  // BW setting
	}
	else  // BW = 7M
	{
		byte[4] &= 0xEF;  // BW setting: clear P5
//		byte[4] |= 0x00;  // BW setting
	}
}

/***********************************************************
 *
 * Set charge pump, bandwidth and band switch bits in byte5
 *
 */
static void sn761640_calculate_mop_ic(u32 freq, u32 baud, int *byte)  // kHz
{
	sn761640_calculate_mop_divider(freq, byte);  // calculate and set byte2 and byte3 (DB1 & DB2)
	{
		int cp = 0, uv = 0;

		sn761640_calculate_mop_uv_cp(freq, &cp, &uv);  // determine band switch and charge pump bits for byte5 (BB)
		*(byte + 4) &= 0x38;  // byte5 (BB): clear CP and BS3-1 bits (note: BS4 is untouched)
		*(byte + 4) |= uv;  // byte5 (BB): add band switch (BS4-1) bits
		*(byte + 4) |= (cp << 6);  // byte5 (BB): add CP bits
		dprintk(20, "%s: bandwidth is %d kHz\n", __func__, baud);
		sn761640_calculate_mop_bw(baud, byte);  // set P5 bit
	}
}

static void tuner_sn761640t_CalWrBuffer(u32 Frequency, u32 BandWidth, u8 *pcIOBuffer)
{
	int buffer[6];

	memset(buffer, 0, sizeof(buffer));
	sn761640_calculate_mop_ic(Frequency, BandWidth * 1000, buffer);

	*(pcIOBuffer + 0) = (u8)buffer[1];  // byte2 (DB1)
	*(pcIOBuffer + 1) = (u8)buffer[2];  // byte3 (DB2)
	*(pcIOBuffer + 2) = 0x80;  // (u8)buffer[3]; set byte4 (CB1) to Msbit=1, ATP2-0=000b, RS2-0=000b
	*(pcIOBuffer + 3) = (u8)buffer[4];  // byte5 (BB)
	*(pcIOBuffer + 4) = 0xE1;  // set byte6 (CB2) to ATC=0, MODE=0, DISCGA=0, IFDA=0, CP2=0, XLO=1
}

static int sn761640_set_params(struct dvb_frontend *fe, struct dvb_frontend_parameters *params)
{
	struct sn761640t_state     *state = fe->tuner_priv;
	u8                         ucIOBuffer[6];
	int                        err = 0;
	u32                        status = 0;
	u32                        f = params->frequency;
	struct dvb_ofdm_parameters *op = &params->u.ofdm;

	dprintk(100, "%s >\n", __func__);
	dprintk(70, "%s: Frequency = %d kHz, Bandwidth = %d MHz\n", __func__, f / 1000, 8 - op->bandwidth - BANDWIDTH_8_MHZ);

	tuner_sn761640t_CalWrBuffer(f / 1000, 8 - op->bandwidth - BANDWIDTH_8_MHZ, ucIOBuffer);

	/* open i2c repeater gate */
	if (fe->ops.i2c_gate_ctrl)
	{
		if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
		{
			goto exit;
		}
	}
	/* Set params */
	err = sn761640_i2c_write(state, ucIOBuffer, 4);  // FIXME!! byte 6 is not written?
	if (err < 0)
	{
		goto exit;
	}
	if (fe->ops.i2c_gate_ctrl)
	{
		if (fe->ops.i2c_gate_ctrl(fe, 0) < 0)
		{
			goto exit;
		}
	}
	ucIOBuffer[2] = ucIOBuffer[4];

	if (fe->ops.i2c_gate_ctrl)
	{
		if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
		{
			goto exit;
		}
	}
	err = sn761640_i2c_write(state, ucIOBuffer, 4);

	if (err < 0)
	{
		goto exit;
	}
	if (fe->ops.i2c_gate_ctrl)
	{
		if (fe->ops.i2c_gate_ctrl(fe, 0) < 0)
		{
			goto exit;
		}
	}
	//msleep(1000);
	if (fe->ops.i2c_gate_ctrl)
	{
		if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
		{
			goto exit;
		}
	}
	sn761640_get_status(fe, &status);
	dprintk(100, "%s < status = %d\n", __func__, status);
	return 0;

exit:
	dprintk(1, "%s: I/O Error\n", __func__);
	return err;
}

static int sn761640_release(struct dvb_frontend *fe)
{
	struct sn761640t_state *state = fe->tuner_priv;

	fe->tuner_priv = NULL;
	kfree(state);
	return 0;
}

static int sn761640_status(struct dvb_frontend *fe, u32 *status)
{
	int err = 0;
	if (fe->ops.i2c_gate_ctrl)
	{
		if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
		{
			goto exit;
		}
	}
	err = sn761640_get_status(fe, status);
	if (err < 0)
	{
		if (fe->ops.i2c_gate_ctrl)
		{
			if (fe->ops.i2c_gate_ctrl(fe, 0) < 0)
			{
				goto exit;
			}
		}
		goto exit;
	}
	if (fe->ops.i2c_gate_ctrl)
	{
		if (fe->ops.i2c_gate_ctrl(fe, 0) < 0)
		{
			goto exit;
		}
	}
	return 0;

exit:
	dprintk(1, "%s: Error\n", __func__);
	return err;
}

static int sn761640_get_identify(struct dvb_frontend *fe)
{
	struct sn761640t_state *state = fe->tuner_priv;
	u8 result[2] = { 0, 0 };
	int err = 0;

	err = sn761640_i2c_read(state, result);
	if (err < 0)
	{
		goto exit;
	}
	if ((result[0] & 0x70) != 0x70)
	{
		return -1;
	}
	dprintk(20, "Found Texas Instruments SN761640 tuner\n");
	return err;

exit:
	dprintk(1, "%s: I/O Error\n", __func__);
	return err;
}

static struct dvb_tuner_ops sn761640t_ops =
{
	.set_params = sn761640_set_params,
	.release    = sn761640_release,
	.get_status = sn761640_status,
};

static int sn761640_identify(struct dvb_frontend *fe)
{
	int err = 0;
	unsigned char ucIOBuffer[10];
	struct sn761640t_state *state = fe->tuner_priv;

//	dprintk(100, "%s >\n", __func__);
	fe->ops.init(fe);

	if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
	{
		goto exit;
	}
	tuner_sn761640t_CalWrBuffer(474000, 8, ucIOBuffer);

	if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
	{
		goto exit;
	}
	err = sn761640_i2c_write(state, ucIOBuffer, 4);
	if (fe->ops.i2c_gate_ctrl(fe, 0) < 0)
	{
		goto exit;
	}
	msleep(500);
	if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
	{
		goto exit;
	}
	ucIOBuffer[2] = ucIOBuffer[4];
	err = sn761640_i2c_write(state, ucIOBuffer, 4);
	if (fe->ops.i2c_gate_ctrl(fe, 0) < 0)
	{
		goto exit;
	}
	msleep(500);

	if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
	{
		goto exit;
	}
	err = sn761640_get_identify(fe);
	if (err < 0)
	{
		goto exit;
	}
	if (fe->ops.i2c_gate_ctrl(fe, 0) < 0)
	{
		goto exit;
	}
//	dprintk(100, "%s < (OK)\n", __func__);
	return err;

exit:
//	dprintk(100, "%s < (-1)\n", __func__);
	return -1;
}

struct dvb_frontend *sn761640t_attach(struct dvb_frontend *fe, const struct sn761640_config *config, struct i2c_adapter *i2c)
{
	int err = 0;
	struct sn761640t_state *state = NULL;
	struct dvb_tuner_info *pInfo = NULL;

	dprintk(100, "%s >\n", __func__);
	state = kzalloc(sizeof(struct sn761640t_state), GFP_KERNEL);
	if (state == NULL)
	{
		goto exit;
	}
	state->config     = config;
	state->i2c        = i2c;
	state->fe         = fe;
	fe->tuner_priv    = state;
	fe->ops.tuner_ops = sn761640t_ops;
	pInfo             = &fe->ops.tuner_ops.info;

	memcpy(pInfo->name, config->name, sizeof(config->name));
	err = sn761640_identify(fe);
	if (err < 0)
	{
		goto exit;
	}
	dprintk(20, "Attaching %s tuner\n", pInfo->name);
	return fe;

exit:
	memset(&fe->ops.tuner_ops, 0, sizeof(struct dvb_tuner_ops));
	kfree(state);
	dprintk(1, "Error: %s tuner not found\n", pInfo->name);
	return NULL;
}
EXPORT_SYMBOL(sn761640t_attach);
// vim:ts=4
