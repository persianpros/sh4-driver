/*
	IX2470 Silicon tuner driver

	Copyright (C) Manu Abraham <abraham.manu@gmail.com>

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

#ifndef __IX2470_H
#define __IX2470_H

#include "avl2108.h"

enum ix2470_step
{
	IX2470_STEP_1000 = 0,  // 1000 kHz, do not divide
	IX2470_STEP_500        //  500 kHz, divide by 2
};

enum ix2470_bbgain
{
	IX2470_GAIN_0dB = 1,  //  0dB Att
	IX2470_GAIN_2dB,      // -2dB Att
	IX2470_GAIN_4dB       // -4dB Att
};

enum ix2470_cpump
{
	IX2470_CP_120uA,	/*  120uA */
	IX2470_CP_260uA,	/*  260uA */
	IX2470_CP_555uA,	/*  555uA */
	IX2470_CP_1200uA,	/* 1200uA (always use this value according to datasheet) */
};

struct ix2470_cfg
{
	u8 name[32];
	u8 addr;
	u8 step_size;
	u8 bb_gain;
	u8 c_pump;
	u8 t_lock;
};

// tuner functions for demod
u16 tuner_load_fw(struct dvb_frontend *fe);
u16 tuner_init(struct dvb_frontend *fe);
u16 tuner_lock(struct dvb_frontend *fe, u32 frequency, u32 srate, u32 _lfp);
u16 tuner_lock_status(struct dvb_frontend *fe);

// AVL2108 demod functions
extern u16 demod_i2c_repeater_send(void *state, u8 *buf, u16 size);
extern u16 demod_i2c_repeater_recv(void *state, u8 *buf, u16 size);
extern u16 demod_i2c_write(void *state, u8 *buf, u16 buf_size);
extern u16 demod_i2c_write16(void *state, u32 addr, u16 data);
extern u16 demod_send_op(u8 ucOpCmd, void *state);
extern u16 demod_get_op_status(void *state);
extern u16 demod_i2c_read16(void *state, u32 addr, u16 *data);

extern int ix2470_attach(struct dvb_frontend *fe, void *demod_priv, struct avl2108_equipment_s *equipment, u8 internal, struct i2c_adapter *i2c);

#endif /* __IX2470_H */
// vim:ts=4
