/****************************************************************************
 *
 * STM STV0367 DVB-T/C demodulator driver
 *
 * Version for Fulan Spark7162
 *
 * Note: actual device is part of the STi7162 SoC
 *
 * Original code:
 * Copyright (C), 2011-2016, AV Frontier Tech. Co., Ltd.
 *
 */
#ifndef __STV0367_H
#define __STV0367_H

#include <linux/dvb/frontend.h>
#include "dvb_frontend.h"

extern struct dvb_frontend *dvb_stv0367_ter_attach(struct i2c_adapter *i2c);
extern struct dvb_frontend *dvb_stv0367_cab_attach(struct i2c_adapter *i2c);
extern int demod_stv0367_identify(struct i2c_adapter *i2c, u8 ucID);
extern s32 PowOf2(s32 number);

#endif  // __STV0367_H
// vim:ts=4
