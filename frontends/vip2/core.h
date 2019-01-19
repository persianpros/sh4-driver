#ifndef __CORE_DVB__
#define __CORE_DVB__

#include "dvb_frontend.h"
#include "dvbdev.h"
#include "demux.h"
#include "dvb_demux.h"
#include "dmxdev.h"
#include "dvb_filter.h"
#include "dvb_net.h"
#include "dvb_ca_en50221.h"

#include <linux/dvb/frontend.h>
#include "dvb_frontend.h"

#include <linux/module.h>
#include <linux/mutex.h>
#include "stv090x.h"

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#  include <linux/stpio.h>
#else
#  include <linux/stm/pio.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
#include "compat.h"
#endif

#define MAX_DVB_ADAPTERS 4
#define MAX_TUNERS_PER_ADAPTER 4

#define SharpS2			0x68	// SharpS2 Tuner I2C Adresse auf Bus 1 und/oder Bus2 (Vip2 und/oder Vip1v2)

struct tuner_config
{
	int adapter; /* DVB adapter number */
	int i2c_bus; /* i2c adapter number */
	u8 i2c_addr; /* i2c address of the tuner */
	u8 addr;     /* i2c address of the tuner */
	
	u8 fe_rst_tuner_a; /* set voltage control of the Tuner A*/
	u8 fe_rst_tuner_b; /* set voltage control of the Tuner B*/
};

enum Shiftregister_ix7306 {
	FE1_1318 	= 0,
	FE1_RST		= 1,
	FE1_LNB_EN	= 2,
	FE1_1419	= 3,
	FE0_RST		= 4,
	FE0_LNB_EN	= 5,
	FE0_1419	= 6,
	FE0_1318	= 7,
};

struct core_config
{
	struct tuner_config 	*tuner;
	struct i2c_adapter	*i2c_adap; /* i2c bus of the tuner */
	u8			i2c_addr; /* i2c address of the tuner */
	u8			i2c_addr_lnb_supply; /* i2c address of the lnb_supply */
	u8 			addr;     /* i2c address of the tuner */
	u8			vertical; /* i2c value */
	u8			horizontal; /* i2c value */
	int 			Bus0;
	int 			Bus1;
	u8 			Status;
};

struct fe_core_state {
	struct dvb_frontend_ops 		ops;
	struct dvb_frontend 			frontend;

	const struct core_config* 		config;

	int					thread_id;

	int				       	not_responding;

};

struct core_info {
	char *name;
	int type;
};

/* place to store all the necessary device information */
struct core {

	/* devices */
	struct dvb_device dvb_dev;
	struct dvb_net dvb_net;

	struct core_info *card;

	unsigned char *grabbing;

	struct tasklet_struct fidb_tasklet;
	struct tasklet_struct vpe_tasklet;

	struct dmxdev dmxdev;
	struct dvb_demux demux;

	struct dmx_frontend hw_frontend;
	struct dmx_frontend mem_frontend;

	int ci_present;
	int video_port;

	u32 buffer_width;
	u32 buffer_height;
	u32 buffer_size;
	u32 buffer_warning_threshold;
	u32 buffer_warnings;
	unsigned long buffer_warning_time;

	u32 ttbp;
	int feeding;

	spinlock_t feedlock;

	spinlock_t debilock;

	struct dvb_adapter *	dvb_adapter;
	struct dvb_frontend*	frontend[MAX_TUNERS_PER_ADAPTER];
	int (*read_fe_status)(struct dvb_frontend *fe, fe_status_t *status);
	int fe_synced;

	void *priv;
};

extern void hc595_out(unsigned char ctls, int state);
extern void fe_core_register_frontend(struct dvb_adapter *dvb_adap);
#endif
