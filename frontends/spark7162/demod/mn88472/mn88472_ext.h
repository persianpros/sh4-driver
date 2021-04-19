/****************************************************************************
 *
 * Panasonic MN88472 DVB-T(2)/C demodulator driver
 *
 * Version for Fulan Spark7162
 *
 * Supports Panasonic MN88472 with MaxLinear MxL301 or
 * MaxLinear MxL603 tuner
 *
 * Original code:
 * Copyright (C), 2013-2018, AV Frontier Tech. Co., Ltd.
 *
 * Date: 2013.12.16
 *
 */
#include <linux/dvb/frontend.h>
#include "dvb_frontend.h"

extern struct dvb_frontend *dvb_mn88472_attach(struct i2c_adapter *i2c, u8 system);
extern struct dvb_frontend *dvb_mn88472earda_attach(struct i2c_adapter *i2c, u8 system);
extern int demod_mn88472_identify(struct i2c_adapter *i2c_adap, u8 ucID);
// vim:ts=4
