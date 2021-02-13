/*
	Sharp IX2470 Silicon tuner driver

	Parts Copyright (C) Manu Abraham <abraham.manu@gmail.com>
	Parts Copyright (C) Malcom Priestley

	Adapted for Opticum HD (TS) 9600 by Audioniek

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <asm/div64.h>

#include "dvb_frontend.h"
#include "ix2470.h"

#define I2C_ADDR_IX2470  (0xc0)

#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[ix2470] "

/*
 *  Data read format of the Sharp IX2470VA
 *
 *  byte1:   1   |   1   |   0   |   0   |   0   |  MA1  |  MA0  |  1
 *  byte2:  POR  |   FL  |  RD2  |  RD1  |  RD0  |   X   |   X   |  X
 *
 *  byte1 = address
 *  byte2;
 *	POR   = Power on Reset (Vcc H=<2.2v L>2.2v)
 *	FL    = Phase Lock (H=lock L=unlock)
 *	RD0-2 = Reserved for internal operations (ignore)
 *	X     = Don't care (ignore)
 *
 * Only POR can be used to check the tuner is present
 *
 * Caution: after byte2 the I2C reverts to write mode;
 *          continuing to read may corrupt tuning data.
 */

/*
 *  Data write format of the Sharp IX2470VA
 *
 *  byte1:   1   |   1   |   0   |   0   |   0   | 0(MA1)| 0(MA0)|  0
 *  byte2:   0   |  BG1  |  BG2  |   N8  |   N7  |   N6  |  N5   |  N4
 *  byte3:   N3  |   N2  |   N1  |   A5  |   A4  |   A3  |   A2  |  A1
 *  byte4:   1   | 1(C1) | 1(C0) |  PD5  |  PD4  |   TM  | 0(RTS)| 1(REF)
 *  byte5:   BA2 |  BA1  |  BA0  |  PSC  |  PD3  |PD2/TS2|DIV/TS1|PD0/TS0
 *
 *  byte1 = address
 *
 *  Write order
 *  1) byte1 -> byte2 -> byte3 -> byte4 -> byte5
 *  2) byte1 -> byte4 -> byte5 -> byte2 -> byte3
 *  3) byte1 -> byte2 -> byte3 -> byte4
 *  4) byte1 -> byte4 -> byte5 -> byte2
 *  5) byte1 -> byte2 -> byte3
 *  6) byte1 -> byte4 -> byte5
 *  7) byte1 -> byte2
 *  8) byte1 -> byte4
 *
 *  Recommended Setup
 *  1 -> 8 -> 6
 */

static const struct ix2470_cfg ix2470va_cfg =
{
	.name       = "Sharp IX2470VA",
	.addr       = I2C_ADDR_IX2470,
	.step_size 	= IX2470_STEP_1000,
	.bb_gain    = IX2470_GAIN_2dB,  // -2db Attenuation
	.c_pump     = IX2470_CP_1200uA,  // datasheet says always use this value (C1=1, C0=1 -> 1.2mA)
	.t_lock     = 0  // use default wait time for lock (200ms)
};

struct ix2470_state
{
	struct avl2108_equipment_s equipment;
	u8                         internal;
	struct i2c_adapter         *i2c;
	struct ix2470_cfg          *ix2470cfg;
	u8                         byte[4];  // current values of byte 2 through 5
	u32                        frequency;
};

static inline u32 ix2470_do_div(u64 n, u32 d)
{
	do_div(n, d);
	return n;
}

static const struct ix2470_losc_sel
{
	u32 f_ll;  // low limit (input)
	u32 f_ul;  // hi limit  (input)

	u8 ba;  // resulting band
	u8 psc;  // resulting psc
	u8 div;  // resulting DIV
	u8 ba210; // resulting BA
} losc_sel[] =
{	// f_ll     f_ul    ba    psc   div   ba210
	{  950000,  986000, 0x01, 0x00, 0x01, 0x05 },
	{  986000, 1073000, 0x02, 0x00, 0x01, 0x06 },
	{ 1073000, 1154000, 0x03, 0x01, 0x01, 0x07 },
	{ 1154000, 1291000, 0x04, 0x01, 0x00, 0x01 },
	{ 1291000, 1447000, 0x05, 0x01, 0x00, 0x02 },
	{ 1447000, 1615000, 0x06, 0x01, 0x00, 0x03 },
	{ 1615000, 1791000, 0x07, 0x01, 0x00, 0x04 },
	{ 1791000, 1972000, 0x08, 0x01, 0x00, 0x05 },
	{ 1972000, 2150000, 0x09, 0x01, 0x00, 0x06 }
};

static const int step[2] = { 1000, 500 };

static const struct ix2470_lpf_sel
{
	u8 val;
	u32 fc;  // MHz
} lpf_sel[] =
{
	{ 0x0c, 10 },
	{ 0x02, 12 },
	{ 0x0a, 14 },
	{ 0x06, 16 },
	{ 0x0e, 18 },
	{ 0x01, 20 },
	{ 0x09, 22 },
	{ 0x05, 24 },
	{ 0x0d, 26 },
	{ 0x03, 28 },
	{ 0x0b, 30 },
	{ 0x07, 32 },
	{ 0x0f, 34 }
};

/* **************** generic functions ********** */

u16 ix2470_tuner_lock_status(struct dvb_frontend *fe)
{
	struct ix2470_state *ix2470 = fe->tuner_priv;
	u16 ret = AVL2108_OK;
	u8  data;

	dprintk(150, "%s >\n", __func__);
	if (ix2470->internal)
	{
		return ret;
	}
	else
	{
		ret = ix2470->equipment.demod_i2c_repeater_recv(fe->demodulator_priv, &data, 1);
		dprintk(100, "Tuner status: 0x%02x\n", data);

		if (ret == AVL2108_OK)
		{
			if (data & (1 << 7))
			{
				ret = AVL2108_ERROR_GENERIC;
				dprintk(1, "Error: Tuner not powered on\n");
			}			
			if (data & (1 << 6))
			{
				ret = AVL2108_OK;
				dprintk(70, "Tuner Phase Locked\n");
			}
			else
			{
				ret = AVL2108_ERROR_GENERIC;
				dprintk(1, "Tuner not Locked\n");
			}
		}
		dprintk(150, "%s < (lock status: %u)\n", __func__, (u32)ret);
		return ret;
	}
	
}

u16 ix2470_tuner_lock(struct dvb_frontend *fe, u32 freq, u32 srate, u32 lpf)
{
	struct ix2470_state *ix2470  = fe->tuner_priv;
	const struct ix2470_cfg *cfg = &ix2470va_cfg;
	u16 ret = AVL2108_OK;
	u32 i;
	u32 data;
	u32 bandwidth = (srate * 27) / 32000000;
	u8 byte[4];
	u8 bb_gain, c_pump, P, N, A, DIV, BA210;
	u8 REF, PD23, PD45, LPF;

//	dprintk(150, "%s >\n", __func__);

	freq *= 100;  /* convert to kHz */
	dprintk(70, "Frequency %d kHz, Symbol rate %d Ms/s, LPF %d.%1d MHz\n", freq, srate / 1000, lpf / 10, lpf % 10);

	// check frequency
	if ((freq < 950000) || (freq > 2150000))
	{
		dprintk(1, "%s Frequency [%llu] out of range \n", __func__, freq);
		return -EINVAL;
	}
	if (freq <= 0)
	{
		return -EINVAL;
	}

	// set frequency

	// set BB gain
	if (cfg->bb_gain)
	{
		bb_gain = cfg->bb_gain;
	}
	else
	{
		bb_gain = IX2470_GAIN_2dB;  // default -2db Attenuation
	}

	// set charge pump
#if 1
	if (cfg->c_pump)
	{
		c_pump = cfg->c_pump;
	}
	else
	{
		c_pump = IX2470_CP_1200uA;  // default 1.2mA
	}

	c_pump = IX2470_CP_1200uA;  // datasheet says always use this value (C1=1, C0=1 -> 1.2mA)
#else
	// calculate C1 and C0
	c_pump = IX2470_CP_1200uA;  // default 1.2mA
	if (freq < 1450000)
	{
		if (freq < 1300000)
		{
			c_pump = IX2470_CP_260uA;  // C1, C0 = 0, 1
		}
		else if (freq < 1375000)
		{
			c_pump = IX2470_CP_555uA;  // C1, C0 = 1, 0
		}
	}
	else
	{
		if (freq < 1850000)
		{
			c_pump = IX2470_CP_260uA;  // C1, C0 = 0, 1
		}
		else if (freq < 2045000)
		{
			c_pump = IX2470_CP_555uA;  // C1, C0 = 1, 0
		}
	}
#endif

	// determine REF (step size)
	REF = (freq > 1024000 ? 0 : 1);
//	REF = 1;  // datasheet says always use this value

	// determine band, P(SC), DIV and BA210
	for (i = 0; i < ARRAY_SIZE(losc_sel); i++)
	{
		if (losc_sel[i].f_ll <= freq && freq < losc_sel[i].f_ul)
		{
			dprintk(70, "Freq: %d kHz, (LL: %d MHz, UL: %d MHz), Band: %d, PSC: %d, DIV: %d, BA210: %d, REF: %d\n",
				freq,
				losc_sel[i].f_ll / 1000,
				losc_sel[i].f_ul / 1000,
				losc_sel[i].ba,
				losc_sel[i].psc,
				losc_sel[i].div,
				losc_sel[i].ba210,
				REF);
			break;
		}
	}
	if (i >= ARRAY_SIZE(losc_sel))
	{
		dprintk(1, "%s Error: out of array bounds while setting frequency!, i=%d\n", __func__, i);
		goto err;
	}
	// determine P (prescaler divisor)
	P = (losc_sel[i].psc == 1 ? 16 : 32);

	/*
	 * Calculate divider values
	 *
	 * Applicable formula:
	 *
	 * fvco = ((P * N) + A) * (fosc) / R
	 *		fvco = tune frequency in MHz
	 *		fosc = 4 MHz
	 *		R    = 8 (REF = 1) or 4 (REF = 0)
	 *      NOTE: fosc / R is smaller than 1 -> only deal with (P * N) + A
	 *
	 *		A     = swallow division ratio (0 - 31; A < N)
	 *		N     = Programmable division ratio (5 - 255)
	 *		P     = Prescaler division ratio (set by PSC == 1 ? 16 : 32)
	 *
	 * Actual formula used:
	 * fvco / fosc = ((P * N) + A) / R
	 * fvco / fosc * R = (P * N) + A
	 *
	 * 1) fosc is 4000 kHz
	 * 2) R = 4 (REF = 0) or 8 (REF = 1)
	 * 3) (P * N) + A = (fvco * R) / 4000, therefore:
	 * (P * N) + A = fvco / 1000 (REF = 0)
	 * (P * N) + A = fvco / 500 (REF = 1)
	 * 
	 */
	data = ix2470_do_div(freq, step[REF]);  // freq is in kHz
	N = data / P;  // N = data / P
	A = data - (P * N);  // A = data - P * N

#if 1
	// Check: A must be smaller than N
	if (A > N)
	{
		dprintk(1, "Error: A is not smaller than N! (A = %d, N = %d)\n", A, N);
		goto err;
	}
	else if (N < 5)
	{
		dprintk(1, "%s Error: N is smaller than 5 (N = %d)\n", __func__, N);
		goto err;
	}
	else if (N > 255)
	{
		dprintk(1, "%s Error: N is greater than than 255 (N = %d)\n", __func__, N);
		goto err;
	}
	else if (A > 31)
	{
		dprintk(1, "%s Error: A is greater than than 31 (A = %d)\n", __func__, A);
		goto err;
	}
	else
	{
		dprintk(50, "Values OK; P = %d  N = %d (division ratio)  A = %d (swallow division)\n", P, N, A);
	}
	// Calculate receiving frequency back
	dprintk(50, "Resulting fVCO: %d kHz, frequency: %d kHz\n", ((P * N) + A) * step[REF], freq);
#endif

	// Set byte 2
	byte[0]  = 0;  // clear all bits
	byte[0] |= ((bb_gain & 0x3) << 5) | (N >> 3);
	// set byte 3
	byte[1]  = 0;  // clear all bits
	byte[1] |= ((N & 0x07) << 5) | (A & 0x1f);
	// set byte 4
	byte[2] = 0;  // clear all bits (C bits, PD5, PD4 (must be zero), TM (must be zero), RTS (should be 0), and REF (always 0))
	byte[2] |= 0x80 | ((c_pump & 0x3) << 5) | REF;  // set MSB, add C0, C1, REF
	// set byte 5
	byte[3] = 0;  // clear all bits (PD3, PD2 must be zero)
	byte[3] |= (losc_sel[i].ba210 << 5) | ((losc_sel[i].psc) << 4) | (losc_sel[i].div << 1) | (1 /* PO */ << 0);

	// step 1: send byte2, byte 3, byte 4 & byte 5 to tuner
	ret = ix2470->equipment.demod_i2c_repeater_send(fe->demodulator_priv, &byte[0], 4);
	if (ret != AVL2108_OK)
	{
		dprintk(1, "%s < I/O error, ret=%d\n", __func__, ret);
		return ret;
	}

	// step 2: resend byte 4, now with TM=1 (VCO/LPF adjustment mode set)
	byte[2] = 0x80 | ((c_pump & 0x3) << 5) | (1 /* TM */ << 2) | REF;  // set MSB, add C0, C1, TM =1, REF
	ret = ix2470->equipment.demod_i2c_repeater_send(fe->demodulator_priv, &byte[2], 1);
	if (ret != AVL2108_OK)
	{
		dprintk(1, "%s < I/O error, ret=%d\n", __func__, ret);
		return ret;
	}
	ix2470->frequency = freq;
	// end set frequency

	// step 3: wait 10 ms
	msleep(10);

	// Determine PD bits (LPF setting)

	for (i = 0; i < ARRAY_SIZE(lpf_sel); i++)
	{
		if (bandwidth <= lpf_sel[i].fc)
		{
			dprintk(70, "BW = %d MHz, Value to set: 0x%02x\n", bandwidth, (u32)lpf_sel[i].val);
			break;
		}
	}
	if (i >= ARRAY_SIZE(lpf_sel))
	{
		dprintk(1, "%s Error: out of array bounds while setting BW!, i = %d", __func__, i);
		goto err;
	}
	// set byte 4
	byte[2] |= ((lpf_sel[i].val >> 2) & 0x03) << 3;  // add PD5, PD4
	// set byte 5
	byte[3] |= (lpf_sel[i].val & 0x03) << 2;         // add PD3, PD2

	// step 4: send byte 4 & 5 
	ret = ix2470->equipment.demod_i2c_repeater_send(fe->demodulator_priv, &byte[2], 2);
	if (ret)
	{
		dprintk(1, "%s I/O error, ret=%d", __func__, ret);
		goto err;
	}
	// end of set lpf (bandwidth)

	if (cfg->t_lock)
	{
		msleep(cfg->t_lock);
	}
	else
	{
		msleep(200);
	}
	dprintk(150, "%s < (status %u)\n", __func__, ret);
err:
	return ret;
}

u16 ix2470_tuner_init(struct dvb_frontend *fe)
{
	u16 ret = AVL2108_OK;
#if 0
	u8  byte[4];  // bytes to write to IX2470
	struct ix2470_state *ix2470 = fe->tuner_priv;

	static unsigned char init_data1[] =
	{
		0x44,  // byte 2: MSB=0; BG=2; N6=1  
		0x7e,  // byte 3: N2=1, N1=1; A1=0 
		0xe1,  // byte 4: MSB=1; charge pump= 1.2mA; PD45=0; TM=0; RTS=0; REF=1
		0x42   // byte 5: BA210=2; PSC=0; PD23=2; DIV=1; PD0=0
	};
	static unsigned char init_data2[] =
	{
		0xe5  // byte 4: MSB=1; charge pump=1.2mA; PD45=0; TM=1; RTS=0; REF=1
	};
	static unsigned char init_data3[] =
	{
		0xfd,  // byte 4: MSB=1; charge pump=1.2mA; PD45=1,1; TM=1; RTS=0; REF=1
		0x0d   // byte 5: BA210=0; PSC=0; P23=1,1; DIV=0; PD0=1
	};

	/* Step one: send initial four bytes */
	// BBgain = -2dB
	// N = 35
	// A = 30
	// Fvco = ((32 * 35) + 30) * 4000 / 8 = 
	// LPF = 12MHz
	ret = ix2470->equipment.demod_i2c_repeater_send(fe->demodulator_priv, &init_data1, 4);
	if (ret != AVL2108_OK)
	{
			dprintk(1, "%s < I/O error, ret = %d\n", __func__, ret);
			return ret;
	}

	/* Step two: resend byte 4 with TM set */
	ret = ix2470->equipment.demod_i2c_repeater_send(fe->demodulator_priv, &init_data2, 1);
	if (ret != AVL2108_OK)
	{
			dprintk(1, "%s < I/O error, ret = %d\n", __func__, ret);
			return ret;
	}

	/* Step three: wait 10 ms */
	msleep(10);

	/* Step four: resend bytes 4 and 5 with with PD set to 34MHz */
	ret = ix2470->equipment.demod_i2c_repeater_send(fe->demodulator_priv, &init_data3, 2);
	if (ret != AVL2108_OK)
	{
			dprintk(1, "%s < I/O error, ret = %d\n", __func__, ret);
			return ret;
	}

	/* Save current values in state */
	memcpy(byte, ix2470->byte, sizeof(byte));
#endif
	dprintk(150, "%s < (status %u)\n", __func__, ret);
	return ret;
}

int ix2470_attach(struct dvb_frontend *fe, void *demod_priv, struct avl2108_equipment_s *equipment, u8 internal, struct i2c_adapter *i2c)
{
	struct ix2470_state *ix2470;
	const struct ix2470_cfg *cfg = &ix2470va_cfg;

	dprintk(150, "%s >\n", __func__);

	ix2470 = kzalloc(sizeof (struct ix2470_state), GFP_KERNEL);
	if (!ix2470)
	{
		dprintk(1, "%s kzalloc failed\n", __func__);
		goto exit;
	}

	fe->tuner_priv    = ix2470;
	ix2470->equipment = *equipment;
	ix2470->internal  = internal;
	ix2470->i2c       = i2c;

	equipment->tuner_load_fw     = NULL;
	equipment->tuner_init        = ix2470_tuner_init;
	equipment->tuner_lock        = ix2470_tuner_lock;
	equipment->tuner_lock_status = ix2470_tuner_lock_status;
	dprintk(70, "Attaching %s QPSK/8PSK tuner\n", cfg->name);
	dprintk(150, "%s < (OK)\n", __func__);
	return 0;

exit:
	kfree(ix2470);
	dprintk(1, "%s < (Error)\n", __func__);
	return -1;
}
EXPORT_SYMBOL(ix2470_attach);
// vim:ts=4
