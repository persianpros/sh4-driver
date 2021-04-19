/*
 * TI SN761640 DVB-C tuner driver
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
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "dvb_frontend.h"
#include "sn761640.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wframe-larger-than="

extern int paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[sn761640c] "

struct sn761640c_state
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
static int sn761640_i2c_read(struct sn761640c_state *state, u8 *buf)
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

static int sn761640_i2c_write(struct sn761640c_state *state, u8 *buf, u8 length)
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
	struct sn761640c_state *state = fe->tuner_priv;
	u8 result[2] = {0};
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

static long sn761640_calculate_mop_xtal(void)
{
	return 4000;  // kHz
}

static void sn761640_calculate_mop_uv_cp(u32 freq, int *cp, int *uv)
{
	int i;
	int cp_value = 599;
	int CP_DATA[601];

	/* charge pump lib */
	for (i = 0; i <= 600; i++)
	{
		CP_DATA[i] = 0;
	}
	CP_DATA[350] = 2;
	CP_DATA[600] = 3;

	if (freq >= 51000 && freq <= 147000)  // lwj add for 5469 low band
	{
		*uv = 1;
		cp_value = 600;
	}
	else if (freq > 147000 && freq < 430000)
	{
		*uv = 2;
		if (freq < 400000)
		{
			cp_value = 350;
		}
		else
		{
			cp_value = 600;
		}
	}
	else if (freq >= 430000)
	{
		*uv = 4;
		if (freq < 763000)
		{
			cp_value = 350;
		}
		else
		{
			cp_value = 600;
		}
	}
	*cp = CP_DATA[cp_value];
}

static long sn761640_calculate_mop_step(int *byte)
{
	int  byte4;
	long mop_step_ratio, mop_freq_step;
	int  R210 = 0;

	byte4 = byte[3];
	R210 = (byte4 & 0x07);
#if 1
	mop_step_ratio = 64;  // lwj change 24 to 64 T:166.67K,divider ratio is 24; C:62.5K, divider ratio is 64
#else
	switch (R210)
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

static void sn761640_calculate_mop_divider(u32 freq, int *byte)
{
	long data;
	u64  i64Freq;

	i64Freq = (u64)freq * 100000;
//	i64Freq += (u64)3612500000;
	i64Freq += 3612500000LL;
	i64Freq = __udivdi3(i64Freq, sn761640_calculate_mop_step(byte));
	i64Freq += 5;
	i64Freq /= 10;
	data = (long)i64Freq;
//	dprintk(150, "%s: data = %ld\n", __func__, data);
	*(byte + 1) = (int)((data >> 8) & 0x7F);  // byte2
	*(byte + 2) = (int)(data & 0xFF);         // byte3
}

static void sn761640_calculate_mop_bw(u32 baud, int *byte)
{
	if (baud > 7500)  // BW = 8M
	{
		byte[4] |= 0x10;  // BW setting
	}
	else if ((6500 < baud) && (baud <= 7500))  // BW = 7M
	{
		byte[4] &= 0xEF;  // BW setting
//		byte[4] |= 0x00;  // BW setting
	}
	else if (baud <= 6500)  // BW = 6M
	{
		byte[4] &= 0xEF;  // BW setting
//		byte[4] |= 0x00;  // BW setting
	}
}

static void sn761640_calculate_mop_ic(u32 freq, u32 baud, int *byte)  // kHz
{
	sn761640_calculate_mop_divider(freq, byte);
	{
		int cp = 0, uv = 0;

		sn761640_calculate_mop_uv_cp(freq, &cp, &uv);
		*(byte + 4) &= 0x38;
		*(byte + 4) |= uv;
		*(byte + 4) |= (cp << 6);
		sn761640_calculate_mop_bw(baud, byte);
	}
}

static void tuner_sn761640c_CalWrBuffer(u32 Frequency, u8 *pcIOBuffer)
{
	int buffer[10];
	u32 BandWidth = 8;
	memset(buffer, 0, sizeof(buffer));
	sn761640_calculate_mop_ic(Frequency, BandWidth * 1000, buffer);

	*pcIOBuffer = (u8)buffer[1];
	*(pcIOBuffer + 1) = (u8)buffer[2];
	*(pcIOBuffer + 2) = 0x83;  // (u8)buffer[3];  // lwj change 0x80 to 0x83 for cable
	*(pcIOBuffer + 3) = (u8)buffer[4];
	*(pcIOBuffer + 4) = 0xC1;  // set byte5
}

static int sn761640_set_params(struct dvb_frontend *fe, struct dvb_frontend_parameters *params)
{
	struct sn761640c_state     *state = fe->tuner_priv;
	u8                         ucIOBuffer[6];
	int                        err = 0;
	u32                        status = 0;
	u32                        f = params->frequency;
//	struct dvb_ofdm_parameters *op = &params->u.ofdm;

//	dprintk(100, "%s > f = %d kHz, bandwidth = %d MHz\n", __func__, f, op->bandwidth);

	tuner_sn761640c_CalWrBuffer(f / 1000, ucIOBuffer);

	/* open i2c repeater gate */
	if (fe->ops.i2c_gate_ctrl)
	{
		if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
		{
			goto exit;
		}
	}
	/* Set params */
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
	struct sn761640c_state *state = fe->tuner_priv;

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
	struct sn761640c_state *state = fe->tuner_priv;
	u8 result[2] = { 0, 0 };
	int err = 0;

	err = sn761640_i2c_read(state, result);
//	dprintk(70, "%s: result[0] = %x\n", __func__, result[0]);
//	dprintk(70, "%s: result[1] = %x\n", __func__, result[1]);
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

static struct dvb_tuner_ops sn761640c_ops =
{
	.set_params = sn761640_set_params,
	.release    = sn761640_release,
	.get_status = sn761640_status,
};

static int sn761640_identify(struct dvb_frontend *fe)
{
	int err = 0;
	unsigned char ucIOBuffer[10];
	struct sn761640c_state *state = fe->tuner_priv;

//	dprintk(100, "%s >\n", __func__);
	fe->ops.init(fe);

	if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
	{
		goto exit;
	}
	tuner_sn761640c_CalWrBuffer(474000, ucIOBuffer);

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

struct dvb_frontend *sn761640c_attach(struct dvb_frontend *fe, const struct sn761640_config *config, struct i2c_adapter *i2c)
{
	int err = 0;
	struct sn761640c_state *state = NULL;
	struct dvb_tuner_info *pInfo = NULL;

	dprintk(100, "%s >\n", __func__);
	state = kzalloc(sizeof(struct sn761640c_state), GFP_KERNEL);
	if (state == NULL)
	{
		goto exit;
	}
	state->config     = config;
	state->i2c        = i2c;
	state->fe         = fe;
	fe->tuner_priv    = state;
	fe->ops.tuner_ops = sn761640c_ops;
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
EXPORT_SYMBOL(sn761640c_attach);
#pragma GCC diagnostic pop
// vim:ts=4
