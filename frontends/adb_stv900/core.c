/************************************************************************
 *
 * Frontend core driver
 *
 * Customized for adb_box, bzzb model.
 *
 * Dual tuner frontend with STM STV0900, 2x STM STB6100 and ISL6422.
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
 ************************************************************************/

#include "core.h"
/* Demodulators */
#include "stv090x.h"

/* Tuners */
#include "stb6100.h"
#include "stb6100_cfg.h"

#include <linux/platform_device.h>
#include <asm/system.h>
#include <asm/io.h>
#include <linux/dvb/dmx.h>
#include <linux/proc_fs.h>
#include <pvr_config.h>

#define I2C_ADDR_STB6100_1 (0x60)
#define I2C_ADDR_STB6100_2 (0x63)
#define I2C_ADDR_STV090X   (0x68)

short paramDebug = 0;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[adb_stv0900] "

static struct core *core[MAX_DVB_ADAPTERS];

static struct stv090x_config stv090x_config_1 =
{
	.device              = STV0900,
	.demod_mode          = STV090x_DUAL  /* STV090x_SINGLE */,
	.clk_mode            = STV090x_CLK_EXT,


	.xtal                = 27000000,
	.address             = I2C_ADDR_STV090X,

	.ts1_mode            = STV090x_TSMODE_SERIAL_CONTINUOUS,
	.ts2_mode            = STV090x_TSMODE_SERIAL_CONTINUOUS,
	.ts1_clk             = 0,
	.ts2_clk             = 0,

	.lnb_enable          = NULL,
	.lnb_vsel            = NULL,

	.repeater_level	     = STV090x_RPTLEVEL_16,

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

static struct stv090x_config stv090x_config_2 =
{
	.device              = STV0900,
	.demod_mode          = STV090x_DUAL,
	.clk_mode            = STV090x_CLK_EXT,


	.xtal                = 27000000,
	.address             = I2C_ADDR_STV090X,

	.ts1_mode            = STV090x_TSMODE_SERIAL_CONTINUOUS,
	.ts2_mode            = STV090x_TSMODE_SERIAL_CONTINUOUS,
	.ts1_clk             = 0,
	.ts2_clk             = 0,

	.lnb_enable          = NULL,
	.lnb_vsel            = NULL,

	.repeater_level      = STV090x_RPTLEVEL_16,

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

static struct stb6100_config stb6100_config_1 =
{
	.tuner_address       = I2C_ADDR_STB6100_1,
	.refclock            = 27000000
};

static struct stb6100_config stb6100_config_2 =
{
	.tuner_address       = I2C_ADDR_STB6100_2,
	.refclock            = 27000000
};

static struct dvb_frontend *frontend_init(struct core_config *cfg, int i)
{
	struct dvb_frontend *frontend = NULL;

	dprintk(50, "%s >\n", __func__);

	if (i > 1)
	{
		return NULL;
	}

	if (i == 0)
	{
		frontend = dvb_attach(stv090x_attach, &stv090x_config_1, cfg->i2c_adap, STV090x_DEMODULATOR_0);
	}
	else
	{
		frontend = dvb_attach(stv090x_attach, &stv090x_config_2, cfg->i2c_adap, STV090x_DEMODULATOR_1);
	}

	if (frontend)
	{
		dprintk(50, "%s: STV0900 attached\n", __func__);

		if (i == 0)
		{
			if (dvb_attach(stb6100_attach, frontend, &stb6100_config_1, cfg->i2c_adap) == 0)
			{
				dprintk(1, "%s Error attaching STB6100(1)\n", __func__);
				goto error_out;
			}
			else
			{
				dprintk(50, "STB6100_1 attached\n");

				stv090x_config_1.tuner_get_frequency = stb6100_get_frequency;
				stv090x_config_1.tuner_set_frequency = stb6100_set_frequency;
				stv090x_config_1.tuner_set_bandwidth = stb6100_set_bandwidth;
				stv090x_config_1.tuner_get_bandwidth = stb6100_get_bandwidth;
				stv090x_config_1.tuner_get_status    = frontend->ops.tuner_ops.get_status;

			}
		}
		else
		{
			if (dvb_attach(stb6100_attach, frontend, &stb6100_config_2, cfg->i2c_adap) == 0)
			{
				dprintk(1, "%s Error attaching STB6100(2)\n", __func__);
				goto error_out;
			}
			else
			{
				dprintk(50, "STB6100_2 attached\n");

				stv090x_config_2.tuner_get_frequency = stb6100_get_frequency;
				stv090x_config_2.tuner_set_frequency = stb6100_set_frequency;
				stv090x_config_2.tuner_set_bandwidth = stb6100_set_bandwidth;
				stv090x_config_2.tuner_get_bandwidth = stb6100_get_bandwidth;
				stv090x_config_2.tuner_get_status    = frontend->ops.tuner_ops.get_status;
			}
		}
	}
	else
	{
		dprintk(1, "%s: Error attaching STB0899\n", __func__);
		goto error_out;
	}
	return frontend;

error_out:
	dprintk(1, "%s Frontend registration failed!\n", __func__);
	if (frontend)
	{
		dvb_frontend_detach(frontend);
	}
	return NULL;
}

static struct dvb_frontend *init_fe_device(struct dvb_adapter *adapter, struct plat_tuner_config *tuner_cfg, int i)
{
	struct fe_core_state *state;
	struct dvb_frontend  *frontend;
	struct core_config   *cfg;

//	dprintk(50, "%s > (I2C bus = %d)\n", __func__, tuner_cfg->i2c_bus);

	cfg = kmalloc(sizeof(struct core_config), GFP_KERNEL);
	if (cfg == NULL)
	{
		dprintk(1, "%s kmalloc failed\n", __func__);
		return NULL;
	}
	/* initialize the config data */
	cfg->i2c_adap = i2c_get_adapter(tuner_cfg->i2c_bus);

//	dprintk(50, "%s i2c adapter = 0x%0x, bus = 0x%0x\n", __func__, cfg->i2c_adap, tuner_cfg->i2c_bus);

	cfg->i2c_addr = tuner_cfg->i2c_addr;

	if (cfg->i2c_adap == NULL)
	{
		dprintk(1, "%s Failed to allocate resources (%s)\n", __func__, (cfg->i2c_adap == NULL) ? "i2c" : "STPIO error");
		kfree(cfg);
		return NULL;
	}
	frontend = frontend_init(cfg, i);

	dprintk(50, "%s: frontend_init (frontend = 0x%x)\n", __func__, (unsigned int) frontend);

	if (frontend == NULL)
	{
		dprintk(1, "%s No frontend found !\n", __func__);
		return NULL;
	}
//	dprintk(50, "%s: Call dvb_register_frontend (adapter = 0x%x)\n", __func__, (unsigned int) adapter);

	if (dvb_register_frontend(adapter, frontend))
	{
		dprintk(1,"%s: Frontend registration failed\n", __func__);
		if (frontend->ops.release)
		{
			frontend->ops.release(frontend);
		}
		return NULL;
	}
	state = frontend->demodulator_priv;
	return frontend;
}

struct plat_tuner_config tuner_resources[] =
{
	[0] =
	{
		.adapter  = 0,
		.i2c_bus  = 0,
//		.i2c_addr = I2C_ADDR_STB6100_1
	},
	[1] =
	{
		.adapter  = 0,
		.i2c_bus  = 0,
//		.i2c_addr = I2C_ADDR_STB6100_2
	},
};

void fe_core_register_frontend(struct dvb_adapter *dvb_adap)
{
	int i = 0;
	int vLoop = 0;

	dprintk(100, "%s: >\n", __func__);

	core[i] = (struct core *)kmalloc(sizeof(struct core), GFP_KERNEL);
	if (!core[i])
	{
		return;
	}
	memset(core[i], 0, sizeof(struct core));

	core[i]->dvb_adapter = dvb_adap;
	dvb_adap->priv = core[i];

	dprintk(20, "Number of tuner(s) = %d\n", ARRAY_SIZE(tuner_resources));

	for (vLoop = 0; vLoop < ARRAY_SIZE(tuner_resources); vLoop++)
	{
		if (core[i]->frontend[vLoop] == NULL)
		{
//			dprintk(50, "%s: Initialize tuner %d i2c_addr: 0x%.2x\n", __func__, vLoop, tuner_resources[vLoop].i2c_addr);
			core[i]->frontend[vLoop] = init_fe_device(core[i]->dvb_adapter, &tuner_resources[vLoop], vLoop);
		}
	}
	dprintk(100, "%s: <\n", __func__);
	return;
}
EXPORT_SYMBOL(fe_core_register_frontend);

int __init fe_core_init(void)
{
//	dprintk(50, "%s Frontend core loaded\n", __func__);
	return 0;
}

static void __exit fe_core_exit(void)
{
	dprintk(50, "Frontend core unloaded\n");
}

module_init(fe_core_init);
module_exit(fe_core_exit);

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled (default), >1=enabled (level_");

MODULE_DESCRIPTION("STV0900 Frontend core");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
