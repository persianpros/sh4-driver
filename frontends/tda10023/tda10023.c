/*
    TDA10023  - DVB-C decoder
    (as used in Philips CU1216-3 NIM and the Reelbox DVB-C tuner card)

	Modifyied for the QBoxHD
	Copyright (C) 2009 Pedro Aguilar (pedro at duolabs dot com)

    Copyright (C) 2005 Georg Acher, BayCom GmbH (acher at baycom dot de)
    Copyright (c) 2006 Hartmut Birr (e9hack at gmail dot com)

    Remotely based on tda10021.c
    Copyright (C) 1999 Convergence Integrated Media GmbH <ralph@convergence.de>
    Copyright (C) 2004 Markus Schulz <msc@antzsystem.de>
		   Support for TDA10021

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
#include <linux/string.h>
#include <linux/slab.h>

#include <asm/div64.h>

#include "dvb_frontend.h"
#include "tda1002x.h"

#define REG0_INIT_VAL 0x23

struct tda10023_state {
	struct i2c_adapter* i2c;
	/* configuration settings */
	const struct tda10023_config *config;
	struct dvb_frontend frontend;

	u8 pwm;
	u8 reg0;

	/* clock settings */
	u32 xtal;
	u8 pll_m;
	u8 pll_p;
	u8 pll_n;
	u32 sysclk;
};

static int debug = 0;

#define dprintk(args...)  						\
	do {              							\
		 if (debug)                             \
			 printk("tda10023: DEBUG: " args);	\
	} while (0)

/*****************************
 * I2C register read/write ops
 *****************************/

static u8 tda10023_readreg (struct tda10023_state* state, u8 reg)
{
	u8 b0 [] = { reg };
	u8 b1 [] = { 0 };
	struct i2c_msg msg [] = { { .addr = state->config->demod_address, .flags = 0, .buf = b0, .len = 1 },
				  { .addr = state->config->demod_address, .flags = I2C_M_RD, .buf = b1, .len = 1 } };
	int ret;

	udelay(100);
	ret = i2c_transfer (state->i2c, msg, 2);
	if (ret != 2) {
		int num = state->frontend.dvb ? state->frontend.dvb->num : -1;
		printk(KERN_ERR "DVB: TDA10023(%d): %s: readreg error "
			"(reg == 0x%02x, ret == %i)\n",
			num, __func__, reg, ret);
	}
	return b1[0];
}

static int tda10023_writereg32(struct tda10023_state* state, u8 reg, u8 * data, u8 size)
{
	u8 buf_big[5], buf_small[3];
    struct i2c_msg msg;
    int ret;
	
	if (size == 2) {
		buf_small[0] = reg;
		buf_small[1] = data[0];
		buf_small[2] = data[1];
	}
	else {
		buf_big[0] = reg;
		buf_big[1] = data[0];
		buf_big[2] = data[1];
		buf_big[3] = data[2];
		buf_big[4] = data[3];
	}

	msg.addr = state->config->demod_address;
	msg.flags = 0;
	if (size == 2)
		msg.buf = buf_small;
	else
		msg.buf = buf_big;
	msg.len = size + 1 ;

    if ((ret = i2c_transfer(state->i2c, &msg, 1)) != 1) {
        printk(KERN_ERR "%s(): error: %i, size %d\n", __func__, ret, size);
        return -EREMOTEIO;
    }
    return (ret != 1) ? -EREMOTEIO : 0;
}

/* @brief PLL model: NXP 6509A, i2c address: 0x
 *		  For writting to the PLL we need to set the demod in by-pass mode,
 *		  then write to the PLL address 0xC0, and finally unset the demod's by-pass mode
 * @param state Contains the current state of the tuner
 * @param reg Register
 * @param data Data to be written
 * @param size Data size
 * @return 1 if successful, 0 otherwise
 */
static int tda10023_writepll(struct tda10023_state* state, u8 reg, u8 * data, u8 size)
{
	u8 buf_big[4], buf_small[2];
	struct i2c_msg msg;
	int ret;

	if (size == 1) {
		buf_small[0] = reg;
		buf_small[1] = data[0];
	}
	else {
		buf_big[0] = reg;
		buf_big[1] = data[0];
		buf_big[2] = data[1];
		buf_big[3] = data[2];
	}

	msg.addr = 0x60;
	msg.flags = 0;
	if (size == 1)
		msg.buf = buf_small;
	else
		msg.buf = buf_big;
	msg.len = size + 1;

	udelay(100);
    if ((ret = i2c_transfer(state->i2c, &msg, 1)) != 1) {
        printk(KERN_ERR "%s(): error: %i, size %d\n", __func__, ret, size);
        return -EREMOTEIO;
	}
	return (ret != 1) ? -EREMOTEIO : 0;
}

static int tda10023_writereg (struct tda10023_state* state, u8 reg, u8 data)
{
	u8 buf[] = { reg, data };
	struct i2c_msg msg = { .addr = state->config->demod_address, .flags = 0, .buf = buf, .len = 2 };
	int ret;

	udelay(100);
	ret = i2c_transfer (state->i2c, &msg, 1);
	if (ret != 1) {
		int num = state->frontend.dvb ? state->frontend.dvb->num : -1;
		printk(KERN_ERR "DVB: TDA10023(%d): %s, writereg error "
			"(reg == 0x%02x, val == 0x%02x, ret == %i)\n",
			num, __func__, reg, data, ret);
	}
	return (ret != 1) ? -EREMOTEIO : 0;
}


static int tda10023_writebit (struct tda10023_state* state, u8 reg, u8 mask, u8 data)
{
	if (mask==0xff)
		return tda10023_writereg(state, reg, data);
	else {
		u8 val;
		val=tda10023_readreg(state,reg);
		val&=~mask;
		val|=(data&mask);
		return tda10023_writereg(state, reg, val);
	}
}

static void tda10023_writetab(struct tda10023_state* state, u8* tab)
{
	u8 r,m,v;
	while (1) {
		r=*tab++;
		m=*tab++;
		v=*tab++;
		if (r==0xff) {
			if (m==0xff)
				break;
			else
				msleep(m);
		}
		else
			tda10023_writebit(state,r,m,v);
	}
}

static int lock_tuner(struct tda10023_state* state)
{
	u8 buf[2] = { 0x0f, 0xc0 };
	struct i2c_msg msg = {.addr=state->config->demod_address, .flags=0, .buf=buf, .len=2};

	if(i2c_transfer(state->i2c, &msg, 1) != 1)
	{
		printk("tda10023: lock tuner fails\n");
		return -EREMOTEIO;
	}
	return 0;
}

static int unlock_tuner(struct tda10023_state* state)
{
	u8 buf[2] = { 0x0f, 0x40 };
	struct i2c_msg msg_post={.addr=state->config->demod_address, .flags=0, .buf=buf, .len=2};

	if(i2c_transfer(state->i2c, &msg_post, 1) != 1)
	{
		printk("tda10023: unlock tuner fails\n");
		return -EREMOTEIO;
	}
	return 0;
}

/*****************************
 * File ops
 *****************************/

static int tda10023_set_symbolrate (struct tda10023_state* state, u32 sr)
{
	s32 BDR;
	s32 BDRI;
	s16 SFIL=0;
	u16 NDEC = 0;
	u8 all[4];

	/* avoid floating point operations multiplying syscloc and divider
	   by 10 */
	u32 sysclk_x_10 = state->sysclk * 10;

	if (sr < (u32)(sysclk_x_10 / 984)) {
		NDEC = 3;
		SFIL = 1;
	} else if (sr < (u32)(sysclk_x_10 / 640)) {
		NDEC = 3;
		SFIL = 0;
	} else if (sr < (u32)(sysclk_x_10 / 492)) {
		NDEC = 2;
		SFIL = 1;
	} else if (sr < (u32)(sysclk_x_10 / 320)) {
		NDEC = 2;
		SFIL = 0;
	} else if (sr < (u32)(sysclk_x_10 / 246)) {
		NDEC = 1;
		SFIL = 1;
	} else if (sr < (u32)(sysclk_x_10 / 160)) {
		NDEC = 1;
		SFIL = 0;
	} else if (sr < (u32)(sysclk_x_10 / 123)) {
		NDEC = 0;
		SFIL = 1;
	} else {
		NDEC = 0;
		SFIL = 0;
	}

	tda10023_writebit(state, 0x3d, 0x80, (SFIL << 7));
	tda10023_writebit(state, 0x03, 0xc0, (NDEC << 6));

	BDRI = (state->sysclk) * 16;
	BDRI >>= NDEC;
	BDRI += sr / 2;
	BDRI /= sr;

	if (BDRI > 255)
		BDRI = 255;

	{
		u64 BDRX;

		BDRX = 1 << (24 + NDEC);
		BDRX *= sr;
		do_div(BDRX, state->sysclk); 	/* BDRX/=SYSCLK; */

		BDR = (s32)BDRX;
	}
	//dprintk("Symbolrate %i, BDR %i BDRI %i, NDEC %i\n", sr, BDR, BDRI, NDEC);

	all[0] = BDR & 255;
	all[1] = ((BDR >> 8) & 255);
	all[2] = ((BDR >> 16) & 31);
	all[3] = BDRI;
	tda10023_writereg32(state, 0x0a, all, 4);

	return 0;
}

static int tda10023_sleep(struct dvb_frontend* fe)
{
	struct tda10023_state* state = fe->demodulator_priv;

	tda10023_writereg (state, 0x1b, 0x02);  /* Power down */
	tda10023_writereg (state, 0x00, 0x80);  /* Stand-by */

	return 0;
}

static int tda10023_init(struct dvb_frontend *fe)
{
	struct tda10023_state* state = fe->demodulator_priv;
	u8 tda10023_inittab[] = {
	/*  reg   mask  val   */
		0x2a, 0x03, 0x03,  /* PLL3, By-pass, Power down */
		0xff, 0x64, 0x00,  /* Sleep 100ms */
		0x28, 0xff, (state->pll_m-1), /* PLL1 */
		0x29, 0xff, ((state->pll_p-1)<<6)|(state->pll_n-1), /* PLL2 */
		0x00, 0xff, REG0_INIT_VAL,	/* GPR FSAMPLING */
		0x2a, 0x03, 0x00,  /* PLL3 PSACLK */
		0xff, 0x64, 0x00,  /* Sleep 100ms */
		0x1f, 0x80, 0x00,  /* RESET */
		0x10, 0xc0, 0x80,  /* BER depth */
		0x0e, 0xff, 0x82,  /* GAIN1 */
		0x03, 0x08, 0x08,  /* CLKCONF DYN */
		0x2e, 0xb0, 0x20,  /* AGCCONF2 */
		0x01, 0xff, 0x48,  /* AGCREF */
		0x1e, 0x84, 0x84,  /* CONTROL IND */
		0x1b, 0xff, 0xc8,  /* ADC TWOS */
		0x2e, 0x08, 0x00,  /* AGC Polarization */
		0x2e, 0x08, 0x00,  /* AGC Polarization */
		0x3b, 0xff, 0xff,  /* IFMAX */
		0x3c, 0xff, 0x96,  /* IFMIN */
		0x35, 0xff, 0xff,  /* TUNMAX */
		0x36, 0xff, 0x00,  /* TUNMIN */
		0x06, 0xff, 0x77,  /* EQCONF1 */
		0x1c, 0x30, 0x30,  /* EQCONF2 */
		0x37, 0xff, 0xf2,  /* DELTAF_LSB */
		0x38, 0xff, 0xff,  /* DELTAF_MSB */
		0x02, 0xff, 0x93,  /* AGCCONF1 */
		0x04, 0xff, 0x5c,  /* INTP2 */
		0x2d, 0xff, 0xf6,  /* SWEEP */
		0x12, 0x01, 0x01,  /* INTP1 */
		0x2b, 0x01, 0x01,  /* INTS1 */
		0x20, 0xff, 0x04,  /* INTP2 */
		0x2c, 0x40, 0x00,  /* INTP/S */
		0x2c, 0x03, 0x00,  /* INTP/S */
		0x00, 0x03, 0x02,  /* Soft reset */
		0xff, 0x64, 0x00,  /* Sleep 100ms */
		0xff, 0xff, 0xff
	};

	dprintk("DVB: TDA10023(%d): init chip\n", fe->dvb->num);
	tda10023_readreg(state, 0x28);
	tda10023_readreg(state, 0x29);
	tda10023_readreg(state, 0x00);

	tda10023_writereg (state, 0x00, 0x2b);  /* Wake up */
	mdelay(200);

	tda10023_writetab(state, tda10023_inittab);
	return 0;
}

static int tda10023_set_parameters (struct dvb_frontend *fe,
			    struct dvb_frontend_parameters *p)
{
	struct tda10023_state* state = fe->demodulator_priv;
	u8 pll_data[3], BBbyte;
	u32 ABbyte;
	u32 freq = p->frequency;
	int qam = p->u.qam.modulation;
	static int qamvals[6][6] = {
		//  QAM   LOCKTHR  MSETH   AREF AGCREFNYQ  ERAGCNYQ_THD
		{ (5<<2),  0x78,    0x8c,   0x96,   0x78,   0x4c  },  // 4 QAM
		{ (0<<2),  0x87,    0xa2,   0x91,   0x8c,   0x57  },  // 16 QAM
		{ (1<<2),  0x64,    0x74,   0x96,   0x8c,   0x57  },  // 32 QAM
		{ (2<<2),  0x46,    0x43,   0x6a,   0x6a,   0x44  },  // 64 QAM
		{ (3<<2),  0x36,    0x34,   0x7e,   0x78,   0x4c  },  // 128 QAM
		{ (4<<2),  0x26,    0x23,   0x6c,   0x5c,   0x3c  },  // 256 QAM
	};

	if (qam < 0 || qam > 5)
		return -EINVAL;

	if (fe->ops.tuner_ops.set_params) {
		fe->ops.tuner_ops.set_params(fe, p);
		if (fe->ops.i2c_gate_ctrl) fe->ops.i2c_gate_ctrl(fe, 0);
	}

	tda10023_writebit(state, 0x02, 0xff, 0x93);
	tda10023_writebit (state, 0x04, 0x40, 0x40);	/* Set inversion */

	tda10023_writebit(state, 0x0f, 0xc0, 0xc0);		/* Set demod by-pass for writting to PLL */
	pll_data[0] = 0x20;
	tda10023_writepll(state, 0xde, pll_data, 1);
	tda10023_writebit(state, 0x0f, 0xc0, 0x40);		/* Remove demod by-pass */

	tda10023_writebit(state, 0x0f, 0xc0, 0xc0);		/* Set demod by-pass for writting to PLL */
	ABbyte = p->frequency + 36166700;
	ABbyte /= 62500;
	pll_data[0] = (ABbyte >> 8) & 0x7f;
	pll_data[1] = ABbyte;
	pll_data[2] = 0xc6;
	if (freq < 161000000)
		BBbyte = 0x01;
	else if (freq > 447000000)
		BBbyte = 0x04;
	else
		BBbyte = 0x02;
	pll_data[3] = BBbyte;
	dprintk("pll_data[0]: 0x%02x, pll_data[1]: 0x%02x, pll_data[2]: 0x%02x, pll_data[3]: 0x%02x\n",
		pll_data[0], pll_data[1], pll_data[2], pll_data[3]);
	tda10023_writepll(state, pll_data[0], pll_data + 1, 3);
	tda10023_writebit(state, 0x0f, 0xc0, 0x40);		/* Remove demod by-pass */

	mdelay(2);
	tda10023_set_symbolrate(state, p->u.qam.symbol_rate);	/* Set symbol rate */

	mdelay(2);
	tda10023_writebit (state, 0x04, 0x40, 0x40);	/* Set inversion */

	/* Set modulation */
	tda10023_writereg (state, 0x05, qamvals[qam][1]);
	tda10023_writereg (state, 0x08, qamvals[qam][2]);
	tda10023_writereg (state, 0x09, qamvals[qam][3]);
	tda10023_writereg (state, 0xb4, qamvals[qam][4]);
	tda10023_writereg (state, 0xb6, qamvals[qam][5]);

	tda10023_writebit(state, 0x00, 0x1c, qamvals[qam][0]);
	tda10023_writebit(state, 0x00, 0x03, 0x02);

	return 0;
}

static int tda10023_read_status(struct dvb_frontend* fe, fe_status_t* status)
{
	struct tda10023_state* state = fe->demodulator_priv;
	int sync, i = 0;

	*status = 0;

	while (i < 100) {
		sync = tda10023_readreg(state, 0x11);
		tda10023_readreg(state, 0xbb);

		if (sync & 2)
			*status |= FE_HAS_SIGNAL|FE_HAS_CARRIER;

		if (sync & 4)
			*status |= FE_HAS_SYNC|FE_HAS_VITERBI;

		if (sync & 8)
			*status |= FE_HAS_LOCK;

			i++;
	}

	dprintk("%s(): status: 0x%02x\n", __func__, *status);

	return 0;
}

static int tda10023_read_ber(struct dvb_frontend* fe, u32* ber)
{
	struct tda10023_state* state = fe->demodulator_priv;
	u8 a,b,c;

	a=tda10023_readreg(state, 0x14);
	b=tda10023_readreg(state, 0x15);
	c=tda10023_readreg(state, 0x16) & 0xf;
	tda10023_writebit (state, 0x10, 0xc0, 0x00);

	*ber = a | (b << 8) | (c << 16);
	return 0;
}

static int tda10023_read_signal_strength(struct dvb_frontend* fe, u16* strength)
{
	struct tda10023_state* state = fe->demodulator_priv;
	u8 ifgain=tda10023_readreg(state, 0x2f);

	u16 gain = ((255-tda10023_readreg(state, 0x17))) + (255-ifgain)/16;
	// Max raw value is about 0xb0 -> Normalize to >0xf0 after 0x90
	if (gain>0x90)
		gain=gain+2*(gain-0x90);
	if (gain>255)
		gain=255;

	*strength = (gain<<8)|gain;
	return 0;
}

static int tda10023_read_snr(struct dvb_frontend* fe, u16* snr)
{
	struct tda10023_state* state = fe->demodulator_priv;

	u8 quality = ~tda10023_readreg(state, 0x18);
	*snr = (quality << 8) | quality;
	return 0;
}

static int tda10023_read_ucblocks(struct dvb_frontend* fe, u32* ucblocks)
{
	struct tda10023_state* state = fe->demodulator_priv;
	u8 a,b,c,d;
	a= tda10023_readreg (state, 0x74);
	b= tda10023_readreg (state, 0x75);
	c= tda10023_readreg (state, 0x76);
	d= tda10023_readreg (state, 0x77);
	*ucblocks = a | (b<<8)|(c<<16)|(d<<24);

	tda10023_writebit (state, 0x10, 0x20,0x00);
	tda10023_writebit (state, 0x10, 0x20,0x20);
	tda10023_writebit (state, 0x13, 0x01, 0x00);

	return 0;
}

static int tda10023_get_frontend(struct dvb_frontend* fe, struct dvb_frontend_parameters *p)
{
	struct tda10023_state* state = fe->demodulator_priv;
	int sync,inv;
	s8 afc = 0;

	sync = tda10023_readreg(state, 0x11);
	afc = tda10023_readreg(state, 0x19);
	inv = tda10023_readreg(state, 0x04);

	if (debug) {
		/* AFC only valid when carrier has been recovered */
		printk(sync & 2 ? "DVB: TDA10023(%d): AFC (%d) %dHz\n" :
				  "DVB: TDA10023(%d): [AFC (%d) %dHz]\n",
			state->frontend.dvb->num, afc,
		       -((s32)p->u.qam.symbol_rate * afc) >> 10);
	}

	p->inversion = (inv&0x20?0:1);
	p->u.qam.modulation = ((state->reg0 >> 2) & 7) + QAM_16;

	p->u.qam.fec_inner = FEC_NONE;
	p->frequency = ((p->frequency + 31250) / 62500) * 62500;

	if (sync & 2)
		p->frequency -= ((s32)p->u.qam.symbol_rate * afc) >> 10;

	return 0;
}

static int tda10023_i2c_gate_ctrl(struct dvb_frontend* fe, int enable)
{
	struct tda10023_state* state = fe->demodulator_priv;

	if (enable)
		lock_tuner(state);
	else
		unlock_tuner(state);

	return 0;
}

static void tda10023_release(struct dvb_frontend* fe)
{
	struct tda10023_state* state = fe->demodulator_priv;

	kfree(state);
}

static struct dvb_frontend_ops tda10023_ops;

struct dvb_frontend *tda10023_attach(const struct tda10023_config *config,
				     struct i2c_adapter *i2c,
				     u8 pwm)
{
	struct tda10023_state* state = NULL;

	/* allocate memory for the internal state */
	state = kzalloc(sizeof(struct tda10023_state), GFP_KERNEL);
	if (state == NULL) goto error;

	/* setup the state */
	state->config = config;
	state->i2c = i2c;

	tda10023_readreg(state, 0x28);
	tda10023_readreg(state, 0x29);
	tda10023_readreg(state, 0x00);

	/* create dvb_frontend */
	memcpy(&state->frontend.ops, &tda10023_ops, sizeof(struct dvb_frontend_ops));
	state->pwm = pwm;
	state->reg0 = REG0_INIT_VAL;
	if (state->config->xtal) {
		state->xtal  = state->config->xtal;
		state->pll_m = state->config->pll_m;
		state->pll_p = state->config->pll_p;
		state->pll_n = state->config->pll_n;
	} else {
		/* set default values if not defined in config */
		state->xtal  = 28920000;
		state->pll_m = 8;
		state->pll_p = 4;
		state->pll_n = 1;
	}

	/* calc sysclk */
	state->sysclk = (state->xtal * state->pll_m / \
			(state->pll_n * state->pll_p));

	state->frontend.ops.info.symbol_rate_min = (state->sysclk/2)/64;
	state->frontend.ops.info.symbol_rate_max = (state->sysclk/2)/4;

	/* dprintk("DVB: TDA10023 %s: xtal:%d pll_m:%d pll_p:%d pll_n:%d\n",
		__func__, state->xtal, state->pll_m, state->pll_p, state->pll_n); */

	state->frontend.demodulator_priv = state;
	return &state->frontend;

error:
	kfree(state);
	return NULL;
}

static struct dvb_frontend_ops tda10023_ops = {

	.info = {
		.name = "NXP TDA10023 DVB-C",
		.type = FE_QAM,
		.frequency_stepsize = 62500,
		.frequency_min =  47000000,
		.frequency_max = 862000000,
		.symbol_rate_min = 0,  /* set in tda10023_attach */
		.symbol_rate_max = 0,  /* set in tda10023_attach */
	#if 0
		.frequency_tolerance = ???,
		.symbol_rate_tolerance = ???,  /* ppm */  /* == 8% (spec p. 5) */
	#endif
		.caps = 0x400 | //FE_CAN_QAM_4
			FE_CAN_QAM_16 | FE_CAN_QAM_32 | FE_CAN_QAM_64 |
			FE_CAN_QAM_128 | FE_CAN_QAM_256 |
			FE_CAN_FEC_AUTO
	},

	.release = tda10023_release,

	.init = tda10023_init,
	.sleep = tda10023_sleep,
	.i2c_gate_ctrl = tda10023_i2c_gate_ctrl,

	.set_frontend = tda10023_set_parameters,
	.get_frontend = tda10023_get_frontend,

	.read_status = tda10023_read_status,
	.read_ber = tda10023_read_ber,
	.read_signal_strength = tda10023_read_signal_strength,
	.read_snr = tda10023_read_snr,
	.read_ucblocks = tda10023_read_ucblocks,
};

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activates frontend debugging (default:0)");

MODULE_DESCRIPTION("NXP TDA10023 DVB-C demodulator driver");
MODULE_AUTHOR("Georg Acher, Hartmut Birr, Pedro Aguilar");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(tda10023_attach);
