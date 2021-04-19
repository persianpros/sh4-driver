/*****************************************************************************
 *
 * Copyright (C), 2011-2016, AV Frontier Tech. Co., Ltd.
 *
 * File: m3501_ext.h
 *
 *
 *****************************************************************************/
#ifndef __M3501_EXT_H
#define __M3501_EXT_H

#ifdef __cplusplus
extern "C" {
#endif

struct m3501_config
{
	u32 i;
	u8 name[32];
	struct i2c_adapter *pI2c;
};

extern struct dvb_frontend *dvb_m3501_fe_qpsk_attach(const struct m3501_config *config, struct i2c_adapter *i2c);

#ifdef __cplusplus
}
#endif

#endif  // __M3501_EXT_H
// vim:ts=4
