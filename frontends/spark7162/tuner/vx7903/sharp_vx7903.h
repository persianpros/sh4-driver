/*****************************************************************************
 *
 * Copyright (C)2008 Ali Corporation. All Rights Reserved.
 *
 * File: vx7903.h
 *
 * Description: This file contains Sharp BS2S7VZ7903 tuner basic function.
 *
 * History:
 *       Date            Author              Version     Reason
 *       ============    =============       =========   =================
 *   1.  2011-10-16      Russell Luo         Ver 0.1     Create file.
 *
 *****************************************************************************/
#ifndef __VX7903_H
#define __VX7903_H

#define REF_OSC_FREQ 4000 /* 4MHZ */

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef dprintk
#define dprintk(level, x...) do \
{ \
	if ((paramDebug) && ((paramDebug >= level) || level == 0)) \
	{ \
		printk(TAGDEBUG x); \
	} \
} while (0)
#endif

struct sharp_vx7903_config
{
	u8 name[32];
	u8 addr;
};

struct sharp_vx7903_i2c_config
{
	u8 addr;
	u8 DemodI2cAddr;

};
struct vx7903_state
{
	struct dvb_frontend            *fe;
	struct i2c_adapter             *i2c;
	struct sharp_vx7903_i2c_config *config;

	/* state cache */
	u32 frequency;
	u32 symbolrate;
	u32 bandwidth;

	u32 index;
};

int nim_vx7903_init(u8 *ptuner_id);

int nim_vx7903_control(u8 tuner_id, u32 freq, u32 sym);
int nim_vx7903_status(u8 tuner_id, u8 *lock);

struct dvb_frontend *vx7903_attach(struct dvb_frontend *fe, struct sharp_vx7903_i2c_config *i2cConfig, struct i2c_adapter *i2c);
int vx7903_get_frequency(struct dvb_frontend *fe, u32 *frequency);
int vx7903_set_frequency(struct dvb_frontend *fe, u32 frequency);
int vx7903_set_bandwidth(struct dvb_frontend *fe, u32 bandwidth);
int vx7903_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth);
int vx7903_get_status(struct dvb_frontend *fe, u32 *status);
int tuner_sharp_vx7903_identify(struct dvb_frontend *fe, struct sharp_vx7903_i2c_config *I2cConfig, struct i2c_adapter *i2c);

#ifdef __cplusplus
}
#endif

#endif /* __VX7903_H__ */
// vim:ts=4
