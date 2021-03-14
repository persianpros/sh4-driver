/************************************************************************
 *
 * Frontend core driver
 *
 * Customized for: Fulan Spark
 *
 * Supports:
 * DVB-S(2): STM STV090x demodulator, Sharp 7306 tuner
 * DVB-S(2): STM STV090x demodulator, Sharp 7903 tuner
 * DVB-S(2): STM STV090x demodulator, STM STV6110x tuner (default)
 *
 * Note: STM STV090x demodulator is part of STi7111 Soc
 *       Detection of tuner type is automatic; if neither Sharp tuner
 *       is found, STM STX6110x is assumed.
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
 ************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * ----------------------------------------------------------------------
 * 201????? Spider Team?    Initial version.
 * 20210305 Audioniek       Cleanup, sync with Spark7162 code.
 * 
 ************************************************************************/
#if !defined(SPARK)
#error: Wrong receiver model!
#endif

#include "core.h"
#include <linux/platform_device.h>
#include <asm/system.h>
#include <asm/io.h>
#include <linux/version.h>
#include <linux/dvb/dmx.h>
#include <linux/proc_fs.h>
#include <linux/stm/pio.h>

#include <pvr_config.h>

#include "vx7903.h"  // tuner Sharp  vx7903

short paramDebug = 0;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[spark-fe] "

static struct core *core[MAX_DVB_ADAPTERS];

static struct stv090x_config stv090x_config =
{
	.device              = STX7111,  // demodulator is part of Sti7111 SoC
	.demod_mode          = STV090x_DUAL,
	.clk_mode            = STV090x_CLK_EXT,
	.xtal                = 30000000,
	.address             = 0x68,
	.ref_clk             = 16000000,
	.ts1_mode            = STV090x_TSMODE_DVBCI,
	.ts2_mode            = STV090x_TSMODE_SERIAL_CONTINUOUS,
	.ts1_clk             = 0,
	.ts2_clk             = 0,
	.repeater_level      = STV090x_RPTLEVEL_64,

	.adc1_range          = STV090x_ADC_1Vpp,
	.agc_rf1_inv         = 1,
	.tuner_init          = NULL,
	.tuner_set_mode      = NULL,
	.tuner_set_frequency = NULL,
	.tuner_get_frequency = NULL,
	.tuner_set_bandwidth = NULL,
	.tuner_get_bandwidth = NULL,
	.tuner_set_bbgain    = NULL,
	.tuner_get_bbgain    = NULL,
	.tuner_set_refclk    = NULL,
	.tuner_get_status    = NULL,
};

static struct stv6110x_config stv6110x_config =
{
	.addr   = (0xc0 >> 1),
	.refclk = 16000000,
};

static struct ix7306_config bs2s7hz7306a_config =
{
	.name      = "Sharp BS2S7HZ7306A",
	.addr      = (0xc0 >> 1),
	.step_size = IX7306_STEP_1000,
	.bb_lpf    = IX7306_LPF_10,
	.bb_gain   = IX7306_GAIN_0dB,
};

static struct vx7903_config bs2s7vz7903a_config =
{
	.name         = "Sharp BS2S7VZ79303A",
	.addr         = (0xc0 >> 1),
	.DemodI2cAddr = 0x68,
};

/*********************************************************
 *
 * Determine tuner type
 *
 */
void frontend_find_TunerDevice(enum tuner_type *ptunerType, struct i2c_adapter *i2c, struct dvb_frontend *frontend)
{
	int identify = 0;

	identify = tuner_vx7903_identify(frontend, &bs2s7vz7903a_config, (void *)i2c);
	if (identify == 0)
	{
		dprintk(20, "Found %s tuner\n", bs2s7vz7903a_config.name);
		*ptunerType = VX7903;
		return;
	}
	identify = tuner_ix7306_identify(frontend, &bs2s7hz7306a_config, i2c);
	if (identify == 0)
	{
		dprintk(20, "Found %s tuner\n", bs2s7hz7306a_config.name);
		*ptunerType = IX7306;
		return;
	}
	//add new tuner identify here
	*ptunerType = TunerUnknown;
	printk("device unknown\n");
}

/*********************************************************
 *
 * Initialize frontend
 *
 */
static struct dvb_frontend *frontend_init(struct core_config *cfg, int i)
{
	struct tuner_devctl *ctl;
	struct dvb_frontend *frontend = NULL;
	enum tuner_type tunerType = TunerUnknown;

	dprintk(100, "%s >\n", __func__);
	frontend = stv090x_attach(&stv090x_config, cfg->i2c_adap, STV090x_DEMODULATOR_0, STV090x_TUNER1);
	frontend_find_TunerDevice(&tunerType, cfg->i2c_adap, frontend);
	if (frontend)
	{
		dprintk(20, "Attached STV090x demodulator\n", __func__);
		switch (tunerType)
		{
			case IX7306:
			{
				if (dvb_attach(ix7306_attach, frontend, &bs2s7hz7306a_config, cfg->i2c_adap))
				{
					dprintk(20, "Attached %s tuner\n", bs2s7hz7306a_config.name);
					stv090x_config.agc_rf1_inv = 1;
					stv090x_config.tuner_set_frequency = frontend->ops.tuner_ops.set_frequency;
					stv090x_config.tuner_get_frequency = frontend->ops.tuner_ops.get_frequency;
					stv090x_config.tuner_set_bandwidth = frontend->ops.tuner_ops.set_bandwidth;
					stv090x_config.tuner_get_bandwidth = frontend->ops.tuner_ops.get_bandwidth;
					stv090x_config.tuner_get_status    = frontend->ops.tuner_ops.get_status;
				}
				else
				{
					dprintk(1, "%s: Error attaching %s tuner\n", __func__, bs2s7hz7306a_config.name);
					goto error_out;
				}
				break;
			}
			case VX7903:
			{
				if (dvb_attach(vx7903_attach, frontend, &bs2s7vz7903a_config, cfg->i2c_adap))
				{
					dprintk(20, "Attached %s tuner\n", bs2s7vz7903a_config.name);
					stv090x_config.agc_rf1_inv = 1;
					stv090x_config.tuner_set_frequency = frontend->ops.tuner_ops.set_frequency;
					stv090x_config.tuner_get_frequency = frontend->ops.tuner_ops.get_frequency;
					stv090x_config.tuner_set_bandwidth = frontend->ops.tuner_ops.set_bandwidth;
					stv090x_config.tuner_get_bandwidth = frontend->ops.tuner_ops.get_bandwidth;
					stv090x_config.tuner_get_status    = frontend->ops.tuner_ops.get_status;
				}
				else
				{
					dprintk(1, "%s: Error attaching %s tuner\n", __func__, bs2s7vz7903a_config.name);
					goto error_out;
				}
				break;
			}
			case STV6110X:  // CAUTION: this model is not actively identified
			default:
			{
				ctl = dvb_attach(stv6110x_attach, frontend, &stv6110x_config, cfg->i2c_adap);
				if (ctl)
				{
					dprintk(20, "Attached STM STV6110x tuner\n", __func__);
					stv090x_config.agc_rf1_inv = 0;
					stv090x_config.tuner_init          = ctl->tuner_init;
					stv090x_config.tuner_set_mode      = ctl->tuner_set_mode;
					stv090x_config.tuner_set_frequency = ctl->tuner_set_frequency;
					stv090x_config.tuner_get_frequency = ctl->tuner_get_frequency;
					stv090x_config.tuner_set_bandwidth = ctl->tuner_set_bandwidth;
					stv090x_config.tuner_get_bandwidth = ctl->tuner_get_bandwidth;
					stv090x_config.tuner_set_bbgain    = ctl->tuner_set_bbgain;
					stv090x_config.tuner_get_bbgain    = ctl->tuner_get_bbgain;
					stv090x_config.tuner_set_refclk    = ctl->tuner_set_refclk;
					stv090x_config.tuner_get_status    = ctl->tuner_get_status;
				}
				else
				{
					dprintk(1, "%s: Error attaching STM STV6110x tuner\n", __func__);
					goto error_out;
				}
				break;
			}
		}
	}
	else
	{
		dprintk(1,  "%s: Error attaching\n", __func__);
		goto error_out;
	}
	if (frontend->ops.init)
	{
		frontend->ops.init(frontend);
	}
	return frontend;

error_out:
	dprintk(1, "%s: Frontend registration failed!\n");
	if (frontend)
	{
		dvb_frontend_detach(frontend);
	}
	return NULL;
}

static void frontend_term(struct dvb_frontend *frontend)
{
	dprintk(100,  "%s >\n", __func__);
	dvb_frontend_detach(frontend);
	dprintk(100,  "%s <\n", __func__);
}

static struct dvb_frontend *init_stv090x_device(struct dvb_adapter *adapter, struct plat_tuner_config *tuner_cfg, int i)
{
	struct fe_core_state *state;
	struct dvb_frontend *frontend;
	struct core_config *cfg;

	dprintk(100, "%s> (I2C bus = %d) \n", __func__, tuner_cfg->i2c_bus);
	cfg = kmalloc(sizeof(struct core_config), GFP_KERNEL);
	if (cfg == NULL)
	{
		dprintk(1, "%s: kmalloc failed\n", __func__);
		return NULL;
	}
	memset(cfg, 0, sizeof(struct core_config));

	/* initialize the config data */
	cfg->i2c_adap = i2c_get_adapter(tuner_cfg->i2c_bus);
//	dprintk(70, "%s: i2c adapter = 0x%0x\n", __func__, (unsigned int)cfg->i2c_adap);
	core[0]->pCfgCore = cfg;
	cfg->i2c_addr = tuner_cfg->i2c_addr;
//	dprintk(70, "%s: demodulator I2X addr = %02x\n", __func__, cfg->i2c_addr);
//	dprintk(70, "%s: tuner enable PIO = %d.%d\n", __func__, tuner_cfg->tuner_enable[0], tuner_cfg->tuner_enable[1]);
	frontend = frontend_init(cfg, i);
	if (frontend == NULL)
	{
		return NULL;
	}
//	dprintk(1,  "%s: Call dvb_register_frontend (adapter = 0x%x)\n", __func__, (unsigned int) adapter);
	if (dvb_register_frontend(adapter, frontend))
	{
		dprintk(1, "%s: Frontend registration failed !\n", __func__);
		if (frontend->ops.release)
		{
			frontend->ops.release(frontend);
		}
		return NULL;
	}
	state = frontend->demodulator_priv;
	return frontend;
}

static void term_stv090x_device(struct dvb_frontend *frontend)
{
	struct core_config *cfg;

	dprintk(100, "%s >\n", __func__);
	dvb_unregister_frontend(frontend);
	frontend_term(frontend);
	cfg = core[0]->pCfgCore;

	if (cfg->i2c_adap)
	{
		i2c_put_adapter(cfg->i2c_adap);
		cfg->i2c_adap = NULL;
	}
	kfree(cfg);
	core[0]->pCfgCore = NULL;
	dprintk(100, "%s <\n", __func__);
}

struct plat_tuner_config tuner_resources[] =
{
	[0] =
	{
		.adapter  = 0,
		.i2c_bus  = 3,
		.i2c_addr = 0x68,
	},
};

int stv090x_register_frontend(struct dvb_adapter *dvb_adap)
{
	int i = 0;
	int vLoop = 0;

	dprintk(100, "%s >\n", __func__);
	core[i] = (struct core *)kmalloc(sizeof(struct core), GFP_KERNEL);
	if (!core[i])
	{
		dprintk(1, "%s: Could not allocate memory\n", __func__);
		return 0;
	}
	memset(core[i], 0, sizeof(struct core));
	core[i]->dvb_adapter = dvb_adap;
	dvb_adap->priv = core[i];
//	dprintk(20, "Tuner = %d\n", ARRAY_SIZE(tuner_resources));
	for (vLoop = 0; vLoop < ARRAY_SIZE(tuner_resources); vLoop++)
	{
		if (core[i]->frontend[vLoop] == NULL)
		{
			dprintk(50, "%s: Initialize tuner %d\n", __func__, vLoop);
			core[i]->frontend[vLoop] = init_stv090x_device(core[i]->dvb_adapter, &tuner_resources[vLoop], vLoop);
		}
	}
	dprintk(100, "%s <\n", __func__);
	return 0;
}
EXPORT_SYMBOL(stv090x_register_frontend);

void stv090x_unregister_frontend(struct dvb_adapter *dvb_adap)
{
	int i = 0;
	int vLoop = 0;

	dprintk(100, "%s >\n", __func__);
	for (vLoop = 0; vLoop < ARRAY_SIZE(tuner_resources); vLoop++)
	{
		dprintk(50, "%s: Unregister tuner %d\n", __func__, vLoop);
		if (core[i]->frontend[vLoop])
		{
			term_stv090x_device(core[i]->frontend[vLoop]);
		}
	}
	core[i] = dvb_adap->priv;
	if (!core[i])
	{
		return;
	}
	kfree(core[i]);
	dprintk(100, "%s <\n", __func__);
	return;
}
EXPORT_SYMBOL(stv090x_unregister_frontend);

int __init stv090x_init(void)
{
	dprintk(50, "STV090x driver loaded\n");
	return 0;
}

static void __exit stv090x_exit(void)
{
	dprintk(50, "STV090x driver unloaded\n");
}

module_init(stv090x_init);
module_exit(stv090x_exit);

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

MODULE_DESCRIPTION("Tunerdriver");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
