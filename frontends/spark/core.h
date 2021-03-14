#ifndef __CORE_H__
#define __CORE_H__

#include <linux/dvb/frontend.h>
#include <linux/module.h>
#include <linux/dvb/version.h>

#include <linux/mutex.h>
#include "dvbdev.h"
#include "demux.h"
#include "dvb_demux.h"
#include "dmxdev.h"
#include "dvb_filter.h"
#include "dvb_net.h"
#include "dvb_ca_en50221.h"
#include "dvb_frontend.h"
#include "stv6110x.h"
#include "ix7306.h"
#include "stv090x.h"

#define dprintk(level, x...) do \
{ \
	if ((paramDebug) && ((paramDebug >= level) || level == 0)) \
	{ \
		printk(TAGDEBUG x); \
	} \
} while (0)

enum tuner_type
{
	STV6110X,
	IX7306,
	VX7903,
	TunerUnknown
};

struct core_config
{
	struct i2c_adapter *i2c_adap;            /* i2c bus of the tuner */
	u8                 i2c_addr;             /* i2c address of the tuner */
	u8                 i2c_addr_lnb_supply;  /* i2c address of the lnb_supply */
	u8                 vertical;             /* i2c value */
	u8                 horizontal;           /* i2c value */
	struct stpio_pin   *tuner_enable_pin;
	u8                 tuner_enable_act;     /* active state of the pin */
};

struct fe_core_state
{
	struct dvb_frontend_ops ops;
	struct dvb_frontend frontend;

	const struct core_config *config;

	int thread_id;
	int not_responding;
};

struct core_info
{
	char *name;
	int type;
};

/* place to store all the necessary device information */
struct core
{
	struct dvb_adapter *dvb_adapter;
	struct dvb_frontend *frontend[MAX_TUNERS_PER_ADAPTER];
	struct core_config *pCfgCore;
};

extern void st90x_register_frontend(struct dvb_adapter *dvb_adap);

#endif // __CORE_H__
// vim:ts=4
