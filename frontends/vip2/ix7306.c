/****************************************************************************
 *
 * IX7306 8PSK/QPSK tuner driver
 * Copyright (C) Manu Abraham <abraham.manu@gmail.com>
 *
 * Version for:
 * Edision argus VIP (1 pluggable tuner)
 * Edision argus VIP2 (2 pluggable tuners)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ***************************************************************************/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "dvb_frontend.h"
#include "ix7306.h"

extern short paramDebug;  // debug print level is zero as default (0=nothing, 1= errors, 10=some detail, 20=more detail, 50=open/close functions, 100=all)
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[ix7306] "
#if !defined dprintk
//#undef dprintk
//#endif
#define dprintk(level, x...) \
do \
{ \
	if ((paramDebug) && (paramDebug >= level) || level == 0) \
	printk(TAGDEBUG x); \
} while (0)
#endif

struct ix7306_state
{
	struct dvb_frontend        *fe;
	struct i2c_adapter         *i2c;
	const struct ix7306_config *config;

	/* state cache */
	u32 frequency;
	u32 bandwidth;
};

static int reg_data[10];

/*
 * I2C routines
 */

// write len bytes to IX7306
static int ix7306_write(struct ix7306_state *state, u8 *buf, u8 len)
{
	const struct ix7306_config *config = state->config;
	int err = 0;
	struct i2c_msg msg = { .addr = config->addr, .flags = 0, .buf = buf, .len = len };

	if (paramDebug >= 150)
	{
		int i;

		dprintk(150, "I2C message = ");
		for (i = 0; i < len; i++)
		{
			printk("0x%02x ", buf[i]);
		}
		printk("\n");
	}

	err = i2c_transfer(state->i2c, &msg, 1);
	if (err != 1)
	{
		dprintk(50, "msg.addr     = 0x%02x\n", msg.addr);
		dprintk(50, "config->addr = 0x%02x\n", config->addr);
		dprintk(1, "%s: write error, err = %d\n", __func__, err);
		return -1;
	}
	return 0;
}

// read one byte from IX7306
static int ix7306_read(struct ix7306_state *state, u8 *buf)
{
	const struct ix7306_config *config = state->config;
	int err = 0;
	u8  tmp = 1;
	struct i2c_msg msg[] =
	{
		{ .addr = config->addr, .flags = 0,        .buf = &tmp, .len = 1 },
		{ .addr = config->addr, .flags = I2C_M_RD, .buf = buf,  .len = 1 }
	};

	state->fe->ops.i2c_gate_ctrl(state->fe, 0);
	// msleep(12);
	state->fe->ops.i2c_gate_ctrl(state->fe, 1);
	err = i2c_transfer(state->i2c, &msg[1], 1);
	if (err != 1)
	{
		dprintk(1, "%s: read error, err=%d\n", __func__, err);
		return -1;
	}
	return 0;
}

/*-------------------------------------------------------------*

        bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
 byte1
 byte2
 byte3
 byte4
 byte5  BA2  BA1       BA0  PSC            DIV/TS1

*--------------------------------------------------------------*/

static int calculate_pll_xtal(void)
{
	return 4000;
}

static int calculate_dividing_factor(int *byte_)
{
	int PSC;

	PSC = (*(byte_ + 4) >> 4);
	PSC &= 0x01;
	if (PSC)
	{
		return 16;  // PSC = 1
	}
	else
	{
		return 32;  // PSC = 0
	}
}

static int calculate_pll_step(int byte4)
{
	int REF;
	int R;
	int pll_step;

	REF = byte4 & 0x01;

	if (REF == 0)
	{
		R = 4;
	}
	else
	{
		R = 8;
	}
	pll_step = calculate_pll_xtal() / R;
	return pll_step;
}

static void calculate_pll_divider(long freq, int *byte_)
{
	long data;
	int P, N, A;

	if (freq < 1024000)
	{
		byte_[3] |= 0x01;
	}
	else
	{
		byte_[3] &= 0xfe;
	}
	if (freq < 1450000)
	{
		if (freq < 1300000)
		{
			byte_[3] &= 0xbf;
		}
		else if (freq < 1375000)
		{
			byte_[3] &= 0xdf;
		}
	}
	else
	{
		if (freq < 1850000)
		{
			byte_[3] &= 0xbf;
		}
		else if (freq < 2045000)
		{
			byte_[3] &= 0xdf;
		}
	}
	data = (long)((freq * 10LL) / calculate_pll_step(*(byte_ + 3)) + 5) / 10 ;

	P = calculate_dividing_factor(byte_);
	N = data / P;
	A = data - P * N;

	data = (N << 5) | A;
	// 040408
	// BG should not be changed...
	*(byte_ + 1) &= 0x60;  // byte2: BG set
	*(byte_ + 1) |= (int)((data >> 8) & 0x1F);  // byte2
	*(byte_ + 2) = (int)(data & 0xFF);  // byte3
}

static void calculate_pll_lpf(long LPF, int *byte)
{
	int data, PD2, PD3, PD4, PD5;

	data = (int)(LPF / 1000 / 2 - 2);
	PD2 = (data >> 3) & 0x01;
	PD3 = (data >> 2) & 0x01;
	PD4 = (data >> 1) & 0x01;
	PD5 = (data) & 0x01;
	*(byte + 3) &= 0xE7;
	*(byte + 4) &= 0xF3;
	*(byte + 3) |= ((PD5 << 4) | (PD4 << 3));
	*(byte + 4) |= ((PD3 << 3) | (PD2 << 2));
}

static long calculate_LPF(long tuner_bw)
{
	long LPF;

	if ((tuner_bw > 34000))
	{
		LPF = 34000;
	}
	if ((tuner_bw > 32000) && (tuner_bw <= 34000))
	{
		LPF = 34000;
	}
	if ((tuner_bw > 30000) && (tuner_bw <= 32000))
	{
		LPF = 32000;
	}
	if ((tuner_bw > 28000) && (tuner_bw <= 30000))
	{
		LPF = 30000;
	}
	if ((tuner_bw > 26000) && (tuner_bw <= 28000))
	{
		LPF = 28000;
	}
	if ((tuner_bw > 24000) && (tuner_bw <= 26000))
	{
		LPF = 26000;
	}
	if ((tuner_bw > 22000) && (tuner_bw <= 24000))
	{
		LPF = 24000;
	}
	if ((tuner_bw > 20000) && (tuner_bw <= 22000))
	{
		LPF = 22000;
	}
	if ((tuner_bw > 18000) && (tuner_bw <= 20000))
	{
		LPF = 20000;
	}
	if ((tuner_bw > 16000) && (tuner_bw <= 18000))
	{
		LPF = 18000;
	}
	if ((tuner_bw > 14000) && (tuner_bw <= 16000))
	{
		LPF = 16000;
	}
	if ((tuner_bw > 12000) && (tuner_bw <= 14000))
	{
		LPF = 16000;
	}
	if ((tuner_bw > 10000) && (tuner_bw <= 12000))
	{
		LPF = 14000;
	}
	if ((tuner_bw > 8000) && (tuner_bw <= 10000))
	{
		LPF = 14000;
	}
	if ((tuner_bw > 6000) && (tuner_bw <= 8000))
	{
		LPF = 12000;
	}
	if ((tuner_bw > 4000) && (tuner_bw <= 6000))
	{
		LPF = 10000;
	}
	if (tuner_bw <= 4000)
	{
		LPF = 10000;
	}
	dprintk(50, "%s: LPF = %d\n", __func__, LPF);
	return LPF;
}

static void calculate_pll_lpf_bw(long tuner_bw, int *byte_)
{
	long LPF;

	if (tuner_bw > 0)  // calculate LPF automatically
	{
		LPF = calculate_LPF(tuner_bw);
		calculate_pll_lpf(LPF, byte_);
	}
}

static void calculate_band_to_byte(int band, int *byte_)
{
	// byte5
	int PSC;
	int DIV, BA210;
	if ((band == 1) || (band == 2)) PSC = 1;
	else PSC = 0;
	*(byte_ + 4) &= 0xEF;
	*(byte_ + 4) |= (PSC << 4);
	//
	if (band == 1)
	{
		DIV = 1;
		BA210 = 0x05;
	}
	if (band == 2)
	{
		DIV = 1;
		BA210 = 0x06;
	}
	if (band == 3)
	{
		DIV = 1;
		BA210 = 0x07;
	}
	if (band == 4)
	{
		DIV = 0;
		BA210 = 0x01;
	}
	if (band == 5)
	{
		DIV = 0;
		BA210 = 0x02;
	}
	if (band == 6)
	{
		DIV = 0;
		BA210 = 0x03;
	}
	if (band == 7)
	{
		DIV = 0;
		BA210 = 0x04;
	}
	if (band == 8)
	{
		DIV = 0;
		BA210 = 0x05;
	}
	if (band == 9)
	{
		DIV = 0;
		BA210 = 0x06;
	}
	*(byte_ + 4) &= 0x1D;
	*(byte_ + 4) |= (DIV << 1);
	*(byte_ + 4) |= (BA210 << 5);
}

static int calculate_local_frequency_band(long freq)
{
	int band;

	if (freq < 986000)
	{
		band = 1;
	}
	if (freq >=  986000 && freq < 1073000)
	{
		band = 2;
	}
	if (freq >= 1073000 && freq < 1154000)
	{
		band = 3;
	}
	if (freq >= 1154000 && freq < 1291000)
	{
		band = 4;
	}
	if (freq >= 1291000 && freq < 1447000)
	{
		band = 5;
	}
	if (freq >= 1447000 && freq < 1615000)
	{
		band = 6;
	}
	if (freq >= 1615000 && freq < 1791000)
	{
		band = 7;
	}
	if (freq >= 1791000 && freq < 1972000)
	{
		band = 8;
	}
	if (freq >= 1972000)
	{
		band = 9;
	}
	dprintk(50, "%s: Band = %d\n", __func__, band);
	return band;
}

static void calculate_pll_vco(long freq, int *byte_)
{
	int band;

//	dprintk(100, "%s >\n", __func__);
	band = calculate_local_frequency_band(freq);
	calculate_band_to_byte(band, byte_);
}

static void pll_setdata(struct dvb_frontend *fe, int *byte_)
{
	struct ix7306_state *state = fe->tuner_priv;
	u8                  ucOperData[5];
	u8                  byte1, byte3, byte4;

	// in this function, we operate on ucOperData instead of byte_
	memset(ucOperData, 0, sizeof(ucOperData));
	ucOperData[0] = *(byte_ + 1);
	ucOperData[1] = *(byte_ + 2);
	ucOperData[2] = *(byte_ + 3);
	ucOperData[3] = *(byte_ + 4);

	byte1 = ucOperData[0];  // byte2
	byte3 = ucOperData[2];  // byte4
	byte4 = ucOperData[3];  // byte5

	ucOperData[2] &= 0xE3;  // TM = 0, LPF = 4MHz
	ucOperData[3] &= 0xF3;  // LPF = 4MHz
	// byte2 / BG = 01 (VCO wait: 2ms)
	ucOperData[0] &= 0x9F;
	ucOperData[0] |= 0x20;

#if 0
	/* open i2c repeater gate */
	if (fe->ops.i2c_gate_ctrl(fe,1) < 0)
	{
		goto err;
	}
#endif
	/* write tuner */
	if (ix7306_write(state, ucOperData, 4) < 0)
	{
		goto err;
	}
	if (fe->ops.i2c_gate_ctrl(fe, 0) < 0)
	{
		goto err;
	}
	ucOperData[2] |= 0x04;  // TM = 1

	/* open i2c repeater gate */
	if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
	{
		goto err;
	}
	/* write tuner */
	if (ix7306_write(state, ucOperData + 2, 1) < 0)
	{
		goto err;
	}
	if (fe->ops.i2c_gate_ctrl(fe, 0) < 0)
	{
		goto err;
	}
	msleep(12);
	ucOperData[2] = byte3;
	// [040108] TM bit always finished with "1".
	ucOperData[2] |= 0x04;  // TM = 1
	/********************************************************************
	*(byte_+3) |= 0x04;  // TM = 1
	*(byte_+3) = byte4;  // byte_4 original value
	********************************************************************/
	ucOperData[3] = byte4;  // byte_5 original value

	/* open i2c repeater gate */
	if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
	{
		goto err;
	}
	/* write tuner */
	if (ix7306_write(state, ucOperData + 2, 2) < 0)
	{
		goto err;
	}
	if (fe->ops.i2c_gate_ctrl(fe, 0) < 0)
	{
		goto err;
	}
	ucOperData[0] = byte1;
	/* open i2c repeater gate */
	if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
	{
		goto err;
	}
	/* write tuner */
	if (ix7306_write(state, ucOperData, 1) < 0)
	{
		goto err;
	}
	return;

err:
	dprintk(1, "%s: I2C write failed\n", __func__);
}

static void tuner_set_freq(struct dvb_frontend *fe, long freq, long tuner_bw, int *byte_)
{
	/* set byte5 BA2 BA1 BA0 PSC DIV/TS1 */
	dprintk(50, "%s - Frequency to tune to: %u MHz (tuner)\n", __func__, freq / 1000);
	dprintk(50, "%s - Bandwidth to set    : %u kHz (tuner)\n", __func__, tuner_bw / 1000);
	calculate_pll_vco(freq, byte_);
	calculate_pll_divider(freq, byte_);
	calculate_pll_lpf_bw(tuner_bw, byte_);
	pll_setdata(fe, byte_);
}

static int ix7306_set_freq(struct dvb_frontend *fe, u32 freq_KHz, u32 tuner_BW)
{
	u32 tuner_bw_K = tuner_BW / 1000;

	memset(reg_data, 0, sizeof(reg_data));
	reg_data[1] = 0x44;
	reg_data[2] = 0x7e;
	reg_data[3] = 0xe1;
	reg_data[4] = 0x42;
	tuner_set_freq(fe, freq_KHz, tuner_bw_K, &reg_data[0]);
	dprintk(20, "Frequency and bandwidth written to tuner\n");
	return 0;
}

static int ix7306_set_state(struct dvb_frontend *fe, enum tuner_param param, struct tuner_state *tstate)
{
	struct ix7306_state *state = fe->tuner_priv;

	if (param & DVBFE_TUNER_FREQUENCY)
	{
		state->frequency = tstate->frequency;
		state->bandwidth = tstate->bandwidth;
		dprintk(20, "Frequency = %u MHz - Bandwidth = %u kHz (state)\n", state->frequency / 1000, state->bandwidth / 1000);
		ix7306_set_freq(fe, state->frequency, state->bandwidth);
	}
	else
	{
		dprintk(1, "%s: Unknown parameter (param=%d)\n", __func__, param);
		return -EINVAL;
	}
	return 0;
}

static int ix7306_get_state(struct dvb_frontend *fe, enum tuner_param param, struct tuner_state *tstate)
{
	struct ix7306_state *state = fe->tuner_priv;
	int err = 0;

	switch (param)
	{
		case DVBFE_TUNER_FREQUENCY:
		{
			tstate->frequency = state->frequency;
			dprintk(20, "Tuned to %u kHz\n", tstate->frequency);
			break;
		}
		case DVBFE_TUNER_BANDWIDTH:
		{
			tstate->bandwidth = state->bandwidth; /* FIXME! need to calculate Bandwidth */
			dprintk(20, "Bandwidth set to %u kHz\n", tstate->bandwidth / 1000);
			break;
		}
		default:
		{
			dprintk(1, "%s: Unknown parameter (param=%d)\n", __func__, param);
			err = -EINVAL;
			break;
		}
	}
	return err;
}

static int ix7306_get_status(struct dvb_frontend *fe, u32 *status)
{
	struct ix7306_state *state = fe->tuner_priv;
	u8 result[2];
	int err = 0;

	*status = 0;
	err = ix7306_read(state, result);
	if (err < 0)
	{
		dprintk(1, "%s: I/O Error reading from tuner\n", __func__);
		return err;
	}
	if (result[0] & 0x40)
	{
		dprintk(20, "%s: Tuner Phase Locked\n", __func__);
		*status = 1;
	}
	else
	{
		dprintk(1, "%s: Tuner Not Phase Locked (result - 0x%02x, 0x%02x)\n", __func__, result[0], result[1]);
	}
	return err;
}

static int ix7306_release(struct dvb_frontend *fe)
{
	struct ix7306_state *state = fe->tuner_priv;

	fe->tuner_priv = NULL;
	kfree(state);
	return 0;
}

#define REF_OSC_FREQ 4000 /* 4MHZ */
static int ix7306_set_params(struct dvb_frontend *fe, struct dvb_frontend_parameters *p)
{
	struct ix7306_state *state = fe->tuner_priv;
	int                 err = 0;
	u32                 freq, sym;
	u8                  data[4];
	u16                 Npro, tmp;
	u32                 Rs, BW;
	u8                  Nswa;
	u8                  LPF = 15;
	u8                  BA = 1;
	u8                  DIV = 0;
	int                 result = 0;
#if 0
	u32                 ratio = 135;
#endif

	freq = p->frequency;
	sym = p->u.qpsk.symbol_rate;
	Rs = sym;

	if (Rs == 0)
	{
		Rs = 45000;
	}
#if 1
	BW = Rs * 135 / 200;
	BW = BW * 130 / 100;

	if (Rs < 6500)
	{
		BW = BW + 3000;
	}
	BW = BW + 2000;
	BW = BW * 108 / 100;
#else
	if (ratio == 0)
	{
		BW = 34000;
	}
	else
	{
		BW = Rs * ratio / 100;
	}
#endif
	if (BW <= 10000)
	{
		BW = 10000;
		LPF = 3;
	}
	if (BW > 34000)
	{
		BW = 34000;
	}
	else if (BW <= 12000)
	{
		LPF = 4;
	}
	else if (BW <= 14000)
	{
		LPF = 5;
	}
	else if (BW <= 16000)
	{
		LPF = 6;
	}
	else if (BW <= 18000)
	{
		LPF = 7;
	}
	else if (BW <= 20000)
	{
		LPF = 8;
	}
	else if (BW <= 22000)
	{
		LPF = 9;
	}
	else if (BW <= 24000)
	{
		LPF = 10;
	}
	else if (BW <= 26000)
	{
		LPF = 11;
	}
	else if (BW <= 28000)
	{
		LPF = 12;
	}
	else if (BW <= 30000)
	{
		LPF = 13;
	}
	else if (BW <= 32000)
	{
		LPF = 14;
	}
	else
	{
		LPF = 15;
	}
	// Determine DIV
	if (freq <= 1154)
	{
		DIV = 1;
	}
	else
	{
		DIV = 0;
	}
	// Determine BA
	if (freq <= 986)
	{
		BA = 5;
	}
	else if (freq <= 1073)
	{
		BA = 6;
	}
	else if (freq <= 1154)
	{
		BA = 7;
	}
	else if (freq <= 1291)
	{
		BA = 1;
	}
	else if (freq <= 1447)
	{
		BA = 2;
	}
	else if (freq <= 1615)
	{
		BA = 3;
	}
	else if (freq <= 1791)
	{
		BA = 4;
	}
	else if (freq <= 1972)
	{
		BA = 5;
	}
	else  // if (freq <= 2150)
	{
		BA = 6;
	}
	tmp = freq * 1000 * 8 / REF_OSC_FREQ;
	Nswa = tmp % 32;
	Npro = tmp / 32;
	data[0] = (u8)((Npro >> 3) & 0x1F);
	data[0] = data[0] | 0x40;
	data[1] = Nswa | (((u8)Npro & 0x07) << 5);
	data[2] = 0xE1;
	data[3] = (BA << 5) | (DIV << 1);

	if (ix7306_write(state, data, 4) < 0)
	{
		dprintk(1, "%s: I2C write error (param1)\n", __func__);
		return result;
	}

	data[2] = 0xE5;
	if (ix7306_write(state, data + 2, 1) < 0)
	{
		dprintk(1, "%s: I2C write error (param2)\n", __func__);
		return result;
	}
	mdelay(10);

	data[2] |= ((LPF & 0x01) << 4) | ((LPF & 0x02) << 2);
	data[3] |= ((LPF & 0x04) << 1) | ((LPF & 0x08) >> 1);

	if (ix7306_write(state, data + 2, 2) < 0)
	{
		dprintk(1, "%s: I2C write error (param3)\n", __func__);
		return result;
	}
	return err;
}

static unsigned char init_data1[] = { 0x44, 0x7e, 0xe1, 0x42 };
static unsigned char init_data2[] = { 0xe5 };
static unsigned char init_data3[] = { 0xfd, 0x0d };

static int ix7306_init(struct dvb_frontend *fe)
{
	int result = 0;
	struct ix7306_state *state = fe->tuner_priv;

	if (fe->ops.i2c_gate_ctrl(fe, 1) < 0)
	{
		dprintk(1, "%s: I2C gate_ctrl error\n", __func__);
		return -1;
	}
	/* tuner */
	if (ix7306_write(state, init_data1, sizeof(init_data1)) < 0)
	{
		dprintk(1, "%s: I2C write error (init_data1)\n", __func__);
		return result;
	}
	if (ix7306_write(state, init_data2, sizeof(init_data2)) < 0)
	{
		dprintk(1, "%s: I2C write error (init_data2)\n", __func__);
		return result;
	}
	//osal_delay(10000);
	mdelay(10);
	if (ix7306_write(state, init_data3, sizeof(init_data3)) < 0)
	{
		dprintk(1, "%s: I2C write error (init_data3)\n", __func__);
		return result;
	}
	return result;
}

static struct dvb_tuner_ops ix7306_ops =
{
	.info =
	{
		.name           = "IX7306",
		.frequency_min  =  950000,
		.frequency_max  = 2150000,
		.frequency_step = 0
	},
	.set_state          = ix7306_set_state,
	.get_state          = ix7306_get_state,
	.get_status         = ix7306_get_status,
	.set_params         = ix7306_set_params,
	.release            = ix7306_release,
	.init               = ix7306_init
};

int ix7306_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct dvb_frontend_ops *frontend_ops = NULL;
	struct dvb_tuner_ops    *tuner_ops = NULL;
	struct tuner_state      t_state;
	int err = 0;

	if (&fe->ops)
	{
		frontend_ops = &fe->ops;
	}
	if (&frontend_ops->tuner_ops)
	{
		tuner_ops = &frontend_ops->tuner_ops;
	}
	if (tuner_ops->get_state)
	{
		if ((err = tuner_ops->get_state(fe, DVBFE_TUNER_FREQUENCY, &t_state)) < 0)
		{
			dprintk(1, "%s: Invalid parameter\n", __func__);
			return err;
		}
		*frequency = t_state.frequency;
		dprintk(20, "Frequency = %d MHz\n", t_state.frequency / 1000);
	}
	return 0;
}

int ix7306_set_frequency(struct dvb_frontend *fe, u32 frequency)
{
	struct ix7306_state     *state = fe->tuner_priv;
	struct dvb_frontend_ops *frontend_ops = NULL;
	struct dvb_tuner_ops    *tuner_ops = NULL;
	struct tuner_state      t_state;
	int err = 0;

	t_state.frequency = frequency;
	t_state.bandwidth = state->bandwidth;
	if (&fe->ops)
	{
		frontend_ops = &fe->ops;
	}
	if (&frontend_ops->tuner_ops)
	{
		tuner_ops = &frontend_ops->tuner_ops;
	}
	if (tuner_ops && tuner_ops->set_state)
	{
		if ((err = tuner_ops->set_state(fe, DVBFE_TUNER_FREQUENCY, &t_state)) < 0)
		{
			dprintk(1, "%s: Invalid parameter\n", __func__);
			return err;
		}
	}
	dprintk(20, "Frequency set: %u MHz\n", t_state.frequency / 1000);
	return 0;
}

int ix7306_set_bandwidth(struct dvb_frontend *fe, u32 bandwidth)
{
	struct ix7306_state *state = fe->tuner_priv;

	state->bandwidth = bandwidth;
//	ix7306_set_freq (fe, state->frequency, state->bandwidth);
	return 0;
}

int ix7306_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth)
{
	struct dvb_frontend_ops *frontend_ops = &fe->ops;
	struct dvb_tuner_ops    *tuner_ops = &frontend_ops->tuner_ops;
	struct tuner_state      t_state;
	int err = 0;

	if (&fe->ops)
	{
		frontend_ops = &fe->ops;
	}
	if (&frontend_ops->tuner_ops)
	{
		tuner_ops = &frontend_ops->tuner_ops;
	}
	if (tuner_ops->get_state)
	{
		if ((err = tuner_ops->get_state(fe, DVBFE_TUNER_BANDWIDTH, &t_state)) < 0)
		{
			dprintk(1, "%s: Invalid parameter\n", __func__);
			return err;
		}
		*bandwidth = t_state.bandwidth;
	}
	dprintk(20, "%s: Bandwidth = %d kHz\n", __func__, t_state.bandwidth / 1000);
	return 0;
}

struct dvb_frontend *ix7306_attach(struct dvb_frontend *fe, const struct ix7306_config *config, struct i2c_adapter *i2c)
{
	struct ix7306_state *state = NULL;

	if ((state = kzalloc(sizeof(struct ix7306_state), GFP_KERNEL)) == NULL)
	{
		goto exit;
	}
	state->config     = config;
	state->i2c        = i2c;
	state->fe         = fe;
	state->bandwidth  = 125000;  // dummy value, bandwidth is calculated
	fe->tuner_priv    = state;
	fe->ops.tuner_ops = ix7306_ops;
	dprintk(20, "Attaching %s 8PSK/QPSK tuner\n", config->name);
	return fe;
exit:
	kfree(state);
	return NULL;
}
EXPORT_SYMBOL(ix7306_attach);
// vim:ts=4
