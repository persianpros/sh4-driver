/************************************************************************
 *
 * Frontend core driver
 *
 * Customized for:
 * Pace HDS7241/91
 *
 * Supports:
 * DVB-S(2): STM STV090x demodulator, STM STV6110x tuner
 * DVB-T   : Philips/NXP TDA10048 demodulator, Philips/NXP TDA18218 tuner
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
 * 
 ************************************************************************/
#if !defined(PACE7241)
#error: Wrong receiver model!
#endif

//#define DVBT

#include "core.h"
#include "stv090x.h"
#include "stv6110x.h"

#include <linux/platform_device.h>
#include <asm/system.h>
#include <asm/io.h>
#include <linux/dvb/dmx.h>
#include <linux/proc_fs.h>
#include <pvr_config.h>
#include <linux/gpio.h>

#include "tda10048.h"
#include "tda18218.h"

#define I2C_ADDR_STV6110X_1 (0x60)
#define I2C_ADDR_STV6110X_2 (0x61)
#define I2C_ADDR_STV090X	(0x6A)

#if defined DVBT
#define I2C_ADDR_TDA18218   (0x60)  // c0 -> 0x60
#define I2C_ADDR_TDA10048   (0x08)  // 10 -> 0x08
#endif

#define RESET stm_gpio(6, 7)

short paramDebug = 0;  // debug print level is zero as default (0=nothing, 1= errors, 10=some detail, 20=more detail, 100=open/close functions, 150=all)

static struct core *core[MAX_DVB_ADAPTERS];

static struct stv090x_config stv090x_config =
{
	.device              = STV0900,
	.demod_mode          = STV090x_DUAL,
	.clk_mode            = STV090x_CLK_EXT,

	.xtal                = 15000000,
	.address             = I2C_ADDR_STV090X,

	.ts1_mode            = STV090x_TSMODE_SERIAL_CONTINUOUS,
	.ts2_mode            = STV090x_TSMODE_SERIAL_CONTINUOUS,
	.ts1_clk             = 0,
	.ts2_clk             = 0,

	.lnb_enable          = NULL,
	.lnb_vsel            = NULL,

	.repeater_level      = STV090x_RPTLEVEL_16,

	.tuner_init          = NULL,
	.tuner_sleep         = NULL,
	.tuner_set_mode      = NULL,
	.tuner_set_frequency = NULL,
	.tuner_get_frequency = NULL,
	.tuner_set_bandwidth = NULL,
	.tuner_get_bandwidth = NULL,
	.tuner_set_bbgain    = NULL,
	.tuner_get_bbgain    = NULL,
	.tuner_set_refclk    = NULL,
	.tuner_get_status    = NULL
};

static struct stv6110x_config stv6110x_config1 =
{
	.addr   = I2C_ADDR_STV6110X_1,
	.refclk = 16000000,
};

static struct stv6110x_config stv6110x_config2 =
{
	.addr   = I2C_ADDR_STV6110X_2,
	.refclk = 16000000,
};

#ifdef DVBT
static struct tda10048_config tda10048_config_ =
{
	.demod_address    = I2C_ADDR_TDA10048,
	.output_mode      = TDA10048_SERIAL_OUTPUT,
	.fwbulkwritelen   = TDA10048_BULKWRITE_200,
	.inversion        = TDA10048_INVERSION_ON,
	.dtv6_if_freq_khz = TDA10048_IF_3300,
	.dtv7_if_freq_khz = TDA10048_IF_3500,
	.dtv8_if_freq_khz = TDA10048_IF_4000,
	.clk_freq_khz     = TDA10048_CLK_16000,
};

static struct tda18218_config tda18218_config_=
{
	.i2c_address = I2C_ADDR_TDA18218,  // 0xc1
	.i2c_wr_max  = 21,  /* max wr bytes AF9015 I2C adap can handle at once */
};
#endif

static struct dvb_frontend *frontend_init(struct core_config *cfg, int i)
{
	struct dvb_frontend    *frontend = NULL;
	struct stv6110x_devctl *ctl = NULL;

	dprintk(100, "%s >\n", __func__);
	if (i == 0)
	{
		frontend = stv090x_attach(&stv090x_config, cfg->i2c_adap, STV090x_DEMODULATOR_0);
		if (frontend)
		{
			dprintk(20, "%s: Attached STV090X demodulator 0\n", __func__);

			ctl = dvb_attach(stv6110x_attach, frontend, &stv6110x_config1, cfg->i2c_adap);
			if (ctl)
			{
				dprintk(20, "%s: Attached STV6110x tuner\n", __func__);
				stv090x_config.tuner_init          = ctl->tuner_init;
				stv090x_config.tuner_sleep         = ctl->tuner_sleep;
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
				dprintk(1, "Error attaching STV6110X %s\n", __func__);
				goto error_out;
			}
		}
		else
		{
			dprintk(1, "%s: Error attaching STV090X demodulator 0\n", __func__);
			goto error_out;
		}
	}
	if (i == 1)
	{
		frontend = stv090x_attach(&stv090x_config, cfg->i2c_adap, STV090x_DEMODULATOR_1);
		if (frontend)
		{
			dprintk(20, "%s: Attached STV090X demodulator 1\n", __func__);
		
			ctl = dvb_attach(stv6110x_attach, frontend, &stv6110x_config2, cfg->i2c_adap);
			if (ctl)
			{
				dprintk(20,"%s: Attached STV6110X tuner\n", __func__);
				stv090x_config.tuner_init	  	  = ctl->tuner_init;
				stv090x_config.tuner_set_mode	  = ctl->tuner_set_mode;
				stv090x_config.tuner_set_frequency = ctl->tuner_set_frequency;
				stv090x_config.tuner_get_frequency = ctl->tuner_get_frequency;
				stv090x_config.tuner_set_bandwidth = ctl->tuner_set_bandwidth;
				stv090x_config.tuner_get_bandwidth = ctl->tuner_get_bandwidth;
				stv090x_config.tuner_set_bbgain	  = ctl->tuner_set_bbgain;
				stv090x_config.tuner_get_bbgain	  = ctl->tuner_get_bbgain;
				stv090x_config.tuner_set_refclk	  = ctl->tuner_set_refclk;
				stv090x_config.tuner_get_status	  = ctl->tuner_get_status;
			}
			else
			{
				dprintk(1, "Error attaching STV6110X %s\n", __func__);
				goto error_out;
			}
		}
		else
		{
			dprintk(1, "%s: Error attaching STV090X demodulator 1\n", __func__);
			goto error_out;
		}
	}
#ifdef DVBT
	if (i == 2)
	{
		dprintk(20, "DVB-T TDA10048\n");
		frontend = dvb_attach(tda10048_attach, &tda10048_config_, cfg->i2c_adap);
		if (frontend)
		{
			dprintk(20, "%s: Attaching TDA18218 tuner\n", __func__);
			if (dvb_attach(tda18218_attach, frontend, cfg->i2c_adap, &tda18218_config_)== NULL)
			{
				dprintk(1, "%s: Error attaching TDA18218 tuner\n", __func__);
				goto error_out;
			}
			else
			{
				dprintk(20, "%s: TDA18218 tuner attached\n", __func__);
			}
		}
		else
		{
			dprintk(1, "%s: Error attaching TDA10048 demodulator\n", __func__);
			goto error_out;
		}
	}
#endif
	return frontend;

error_out:
	dprintk(1, "%s: Frontend registration failed!\n", __func__);
	if (frontend)
	{
		dvb_frontend_detach(frontend);
	}
	return NULL;
}

static struct dvb_frontend *init_fe_device(struct dvb_adapter *adapter, struct plat_tuner_config *tuner_cfg, int i)
{
	struct fe_core_state *state;
	struct dvb_frontend *frontend;
	struct core_config *cfg;

	dprintk(100, "%s > (I2C bus = %d)\n", __func__, tuner_cfg->i2c_bus);

	cfg = kmalloc(sizeof(struct core_config), GFP_KERNEL);
	if (cfg == NULL)
	{
		dprintk(1, "%s: kmalloc failed\n", __func__);
		return NULL;
	}
	else
	{
		dprintk (100, "%s: kmalloc succeeded\n", __func__);
	}
	/* initialize the config data */
	cfg->i2c_adap = i2c_get_adapter(tuner_cfg->i2c_bus);
	dprintk(20, "I2C adapter = 0x%0x\n", (int)cfg->i2c_adap);

	cfg->i2c_addr = tuner_cfg->i2c_addr;

	if (cfg->i2c_adap == NULL)
	{
		dprintk(1, "%s: Failed to allocate %s resources\n", (cfg->i2c_adap == NULL) ? "i2c" : "STPIO error");
		kfree(cfg);
		return NULL;
	}
	else
	{
		dprintk(20, "Allocating resources successful\n");
	}
	frontend = frontend_init(cfg, i);

	if (frontend == NULL)
	{
		dprintk(1, "%s: No frontend found!\n", __func__);
		return NULL;
	}
	dprintk(20, "Registering frontend (adapter = 0x%x)\n", (unsigned int)adapter);

	if (dvb_register_frontend(adapter, frontend))
	{
		dprintk(1, "%s: Frontend registration failed!\n", __func__);
		if (frontend->ops.release)
		{
			frontend->ops.release(frontend);
		}
		return NULL;
	}
	state = frontend->demodulator_priv;
	dprintk(20, "%s: Registering frontend successful\n", __func__);
	return frontend;
}

struct plat_tuner_config tuner_resources[] =
{
	[0] =
	{
		.adapter  = 0,
		.i2c_bus  = 0,
		.i2c_addr = I2C_ADDR_STV090X,
	},
	[1] =
	{
		.adapter  = 0,
		.i2c_bus  = 0,
		.i2c_addr = I2C_ADDR_STV090X,
	},
#ifdef DVBT
	[2] =
	{
		.adapter  = 0,
		.i2c_bus  = 0,
		.i2c_addr = 0x08,
	},
#endif
};

void stv090x_register_frontend(struct dvb_adapter *dvb_adap)
{
	int i = 0;
	int vLoop = 0;

	dprintk(10, "%s: Pace HDS7241 frontend core\n", __func__);
	/* Initialize front reset */
	if (gpio_request(RESET, "RESET_STV0900") == 0)
	{
		dprintk(1, "%s: Request RESET_STV0900 PIO pin failed\n", __func__);
	}
	else
	{
		dprintk(20, "Request RESET_STV0900 PIO pin successful\n");

		/* Reset tuner */
		gpio_direction_output(RESET, STM_GPIO_DIRECTION_OUT);
		gpio_set_value(RESET, 0);
		msleep(100);
		gpio_set_value(RESET, 1);
		msleep(500);
	}
	core[i] = (struct core*)kmalloc(sizeof(struct core),GFP_KERNEL);
	if (!core[i])
	{
		return;
	}
	memset(core[i], 0, sizeof(struct core));

	core[i]->dvb_adapter = dvb_adap;
	dvb_adap->priv = core[i];

	dprintk(20, "# of tuners: %d\n", ARRAY_SIZE(tuner_resources));

	for (vLoop = 0; vLoop < ARRAY_SIZE(tuner_resources); vLoop++)
	{
		if (core[i]->frontend[vLoop] == NULL)
		{
			dprintk(20, "%s: Initialize tuner %d\n", __func__, vLoop);
			core[i]->frontend[vLoop] = init_fe_device(core[i]->dvb_adapter, &tuner_resources[vLoop], vLoop);
		}
	}
	dprintk(100, "%s: <\n", __func__);
	return;
}
EXPORT_SYMBOL(stv090x_register_frontend);

int __init fe_core_init(void)
{
	dprintk(20, "Frontend core loaded\n");
	return 0;
}

static void __exit fe_core_exit(void)
{
	dprintk(20, "Frontend core unloaded\n");
}

module_init(fe_core_init);
module_exit(fe_core_exit);

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

MODULE_DESCRIPTION("Front end driver for Pace HDS7241/91");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
