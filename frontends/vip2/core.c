/************************************************************************
 *
 * Frontend core driver
 *
 * Customized for:
 * Edision argus VIP (1 pluggable tuner)
 * Edision argus VIP2 (2 pluggable tuners)
 *
 * Supports:
 * DVB-S(2): STM STV090x demodulator, Sharp 7306 tuner (default)
 * DVB-S(2): STM STV090x demodulator, STM STV6110x tuner
 * DVB-T   : Intel CE6353 demodulator, Sharp 6465 tuner (default tuner 2)
 * DVB-C   : Philips/NXP TDA10023 demodulator, LG 031 tuner
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
 * 20200726 Audioniek       VIP1_V2 (1 pluggable tuner) integrated.
 * 
 ************************************************************************/
#if !defined(VIP1_V2) \
 && !defined(VIP2)
#error: Wrong receiver model!
#endif

#include "core.h"
#if defined TUNER_STV6110
#include "stv6110x.h"
#endif
#include "stv090x.h"
#if defined TUNER_IX7306
#include "ix7306.h"
#endif
#include "ce6353.h"
#include "sharp6465.h"
#include "tda1002x.h"
#include "lg031.h"

#include <linux/platform_device.h>
#include <asm/system.h>
#include <asm/io.h>
#include <linux/dvb/dmx.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <pvr_config.h>

#define I2C_ADDR_STV090X   (0xd0 >> 1)  // d0 -> 0x68
#define I2C_ADDR_STV6110X  (0xc0 >> 1)  // c0 -> 0x60
#define I2C_ADDR_IX7306    (0xc0 >> 1)  // c0 -> 0x60
#define I2C_ADDR_CE6353    (0x1e >> 1)  // 1e -> 0x0f
#define I2C_ADDR_SHARP6465 (0xc2 >> 1)  // c2 -> 0x61
#define I2C_ADDR_TDA10023  (0x18 >> 1)  // 18 -> 0x0c
#define I2C_ADDR_LG031     (0xc6 >> 1)  // c6 -> 0x63

/* On the VIP2, a dual DVB-S(2) tuner setup is not
 * supported because both tuner slots share the same LNB
 * making true dual tuner operation not possible.
 */

#if defined(TUNER_IX7306)
#define CLK_EXT_IX7306 		4000000
#define CLK_EXT_STV6110 	16000000
#elif defined(TUNER_STV6110)
#define CLK_EXT_IX7306 		8000000
#define CLK_EXT_STV6110 	16000000
#else
#error "You must define a tuner type..."
#endif

short paramDebug = 0;  // debug print level is zero as default (0=nothing, 1= errors, 10=some detail, 20=more detail, 100=open/close functions, 150=all)

// Default names
#if defined(VIP2)
static char *demod1 = "stv090x";
static char *demod2 = "ce6353";
static char *tuner1 = "sharp7306";
static char *tuner2 = "sharp6465";

static int demodType1;
static int demodType2;
static int tunerType1;
static int tunerType2;
#else  // VIP1_V2
static char *demod = "stv090x";
static char *tuner = "sharp7306";

static int demodType;
static int tunerType;
#endif

static struct core *core[MAX_DVB_ADAPTERS];

enum
{
	STV090X,
	CE6353,
	TDA10023,
};

static char *demod_name[] =
{
	"stv090x",
	"ce6353",
	"tda10023",
};

enum
{
	SHARP7306,
	STV6110X,
	SHARP6465,
	LG031,
};

static char *tuner_name[] =
{
	"Sharp 7306",
	"STM STV6110x",
	"Sharp 6465",
	"LG 031",
};

enum fe_ctl
{
	FE1_1318 	= 0,
	FE1_RST		= 1,
	FE1_LNB_EN	= 2,
	FE1_1419	= 3,
	FE0_RST		= 4,
	FE0_LNB_EN	= 5,
	FE0_1419	= 6,
	FE0_1318	= 7,
};

/* PIO definitions for 74HC595 LNB drive */
static unsigned char fctl = 0;

struct stpio_pin *srclk;  // shift clock
struct stpio_pin *lclk;   // latch clock
struct stpio_pin *sda;    // serial data

#define SRCLK_CLR() {stpio_set_pin(srclk, 0);}
#define SRCLK_SET() {stpio_set_pin(srclk, 1);}

#define LCLK_CLR() {stpio_set_pin(lclk, 0);}
#define LCLK_SET() {stpio_set_pin(lclk, 1);}

#define SDA_CLR() {stpio_set_pin(sda, 0);}
#define SDA_SET() {stpio_set_pin(sda, 1);}

void hc595_out(unsigned char ctls, int state)
{
	int i;

	if (state)
	{
		fctl |= 1 << ctls;
	}
	else
	{
		fctl &= ~(1 << ctls);
	}
	/*
	 * clear everything out just in case to
	 * prepare shift register for bit shifting
	 */
	SDA_CLR();
	SRCLK_CLR();
	udelay(10);

	for (i = 7; i >= 0; i--)
	{
		SRCLK_CLR();
		if (fctl & (1 << i))
		{
			SDA_SET();
		}
		else
		{
			SDA_CLR();
		}
		udelay(1);
		SRCLK_SET();
		udelay(1);
	}
	LCLK_CLR();  // latch data
	udelay(1);
	LCLK_SET();
}
EXPORT_SYMBOL(hc595_out);

static struct stv090x_config stv090x_config =
{
	.device              = STV0903,
	.demod_mode          = STV090x_DUAL,  /* STV090x_SINGLE */
	.clk_mode            = STV090x_CLK_EXT,

	.xtal                = CLK_EXT_IX7306,
	.address             = I2C_ADDR_STV090X,

	.ts1_mode            = STV090x_TSMODE_DVBCI,  /* STV090x_TSMODE_PARALLEL_PUNCTURED */ /* STV090x_TSMODE_SERIAL_CONTINUOUS */
	.ts2_mode            = STV090x_TSMODE_DVBCI,  /* STV090x_TSMODE_PARALLEL_PUNCTURED */ /* STV090x_TSMODE_SERIAL_CONTINUOUS */
	.ts1_clk             = 0,
	.ts2_clk             = 0,

	.fe_rst              = 0,
	.fe_lnb_en           = 0,
	.fe_1419             = 0,
	.fe_1318             = 0,

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
	.tuner_get_status    = NULL
};

#if defined TUNER_STV6110
static struct stv6110x_config stv6110x_config =
{
	.addr    = I2C_ADDR_STV6110X,
	.refclk  = CLK_EXT_STV6110,
	.clk_div = 2,
};
#endif

#if defined TUNER_IX7306
static const struct ix7306_config bs2s7hz7306a_config =
{
	.name      = "Sharp BS2S7HZ7306A",
	.addr      = I2C_ADDR_IX7306,
	.step_size = IX7306_STEP_1000,
	.bb_lpf    = IX7306_LPF_12,
	.bb_gain   = IX7306_GAIN_2dB,
};
#endif

static struct ce6353_config ce6353_config =
{
	.demod_address = I2C_ADDR_CE6353,
	.no_tuner      = 1,
	.parallel_ts   = 1,
};

static const struct sharp6465_config s6465_config =
{
	.name      = "Sharp 6465",
	.addr      = I2C_ADDR_SHARP6465,
	.bandwidth = BANDWIDTH_8_MHZ,

	.Frequency = 500000,
	.IF        = 36167,
	.TunerStep = 16667,
};

static struct tda10023_config philips_tda10023_config =
{
	.demod_address = I2C_ADDR_TDA10023,
	.invert        = 1,
};

static struct lg031_config lg_lg031_config =
{
	.addr = I2C_ADDR_LG031,
};

static struct dvb_frontend *frontend_get_by_type(struct core_config *cfg, int iDemodType, int iTunerType)
{
	struct stv6110x_devctl *ctl   = NULL;
	struct dvb_frontend *frontend = NULL;
	switch (iDemodType)
	{
		case STV090X:
		{
			frontend = dvb_attach(stv090x_attach, &stv090x_config, cfg->i2c_adap, STV090x_DEMODULATOR_0);
			if (frontend)
			{
				stv090x_config.fe_rst 	  = cfg->tuner->fe_rst;
				stv090x_config.fe_1318 	  = cfg->tuner->fe_1318;
				stv090x_config.fe_1419 	  = cfg->tuner->fe_1419;
				stv090x_config.fe_lnb_en  = cfg->tuner->fe_lnb_en;
				dprintk(20, "STV090x demodulator attached\n");

				switch (iTunerType)
				{
#if defined TUNER_IX7306
					case SHARP7306:
					default:
					{
						if (dvb_attach(ix7306_attach, frontend, &bs2s7hz7306a_config, cfg->i2c_adap))
						{
							dprintk(20, "IX7306 tuner attached\n");
//							stv090x_config.tuner_init          = ix7306_tuner_init;
							stv090x_config.tuner_set_frequency = ix7306_set_frequency;
							stv090x_config.tuner_get_frequency = ix7306_get_frequency;
							stv090x_config.tuner_set_bandwidth = ix7306_set_bandwidth;
							stv090x_config.tuner_get_bandwidth = ix7306_get_bandwidth;
							stv090x_config.tuner_get_status    = frontend->ops.tuner_ops.get_status;
						}
						else
						{
							dprintk(1, "%s: Error attaching IX7306 tuner\n", __func__);
							goto error_out;
						}
						break;
					}
#elif defined TUNER_STV6110
					case STV6110X:
					{
						ctl = dvb_attach(stv6110x_attach, frontend, &stv6110x_config, cfg->i2c_adap);
						if (ctl)
						{
							dprintk(20, "STV6110x tuner attached\n");
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
							dprintk(1, "%s: Error attaching STV6110x tuner\n", __func__);
							goto error_out;
						}
						break;
					}
#endif
				}
			}
			else
			{
				dprintk(1, "%s: Error attaching STV090x demodulator\n", __func__);
				goto error_out;
			}
			break;
		}
		case CE6353:
		{
			frontend = dvb_attach(ce6353_attach, &ce6353_config, cfg->i2c_adap);
			if (frontend)
			{
				dprintk(20, "CE6353 demodulator attached\n");
				switch (iTunerType)
				{
					case SHARP6465:
					{
						if (dvb_attach(sharp6465_attach, frontend, &s6465_config, cfg->i2c_adap))
						{
							dprintk(20, "Sharp 6465 tuner attached\n");
						}
						else
						{
							dprintk(1, "%s: Error attaching Sharp 6465 tuner\n", __func__);
							goto error_out;
						}
						break;
					}
					default:
					{
						dprintk(1, "%s: Error: Unknown tuner\n", __func__);
						goto error_out;
					}
				}
			}
			else
			{
				dprintk(1, "%s: Error attaching CE6353 demodulator\n", __func__);
				goto error_out;
			}
			break;
		}
		case TDA10023:
		{
			frontend = dvb_attach(tda10023_attach, &philips_tda10023_config, cfg->i2c_adap, 0x48);
			if (frontend)
			{
				dprintk(20, "TDA10023 demodulator attached\n");
				switch (iTunerType)
				{
					case LG031:
					{
						if (dvb_attach(lg031_attach, frontend, &lg_lg031_config, cfg->i2c_adap))
						{
							dprintk(20, "LG031 tuner attached\n");
						}
						else
						{
							dprintk(1, "%s: Error attaching LG031 tuner\n", __func__);
							goto error_out;
						}
						break;
					}
					default:
					{
						dprintk(1, "%s: Error: Unknown tuner\n", __func__);
						goto error_out;
					}
				}
			}
			else
			{
				dprintk(1, "%s: Error attaching TDA10023 demodulator\n", __func__);
				goto error_out;
			}
			break;
		}
		default:
		{
			dprintk(1, "%s: Error: unknown demodulator\n", __func__);
			goto error_out;
		}
	}
	return frontend;

error_out:
	dprintk(1, "%s: Frontend registration failed!\n", __func__);
	if (frontend)
	{
		dvb_frontend_detach(frontend);
	}
	return NULL;
}

static struct dvb_frontend *frontend_init(struct core_config *cfg, int i)
{
	struct dvb_frontend *frontend = NULL;

	dprintk(100, "%s >\n", __func__);

#if defined(VIP2)
	if (i == 0)
	{
		frontend = frontend_get_by_type(cfg, demodType1, tunerType1);
	}
	else
	{
		frontend = frontend_get_by_type(cfg, demodType2, tunerType2);
	}
#else
	frontend = frontend_get_by_type(cfg, demodType, tunerType);
#endif
	return frontend;
}

static struct dvb_frontend *init_fe_device(struct dvb_adapter *adapter, struct tuner_config *tuner_cfg, int i)
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
	/* initialize the config data */
	cfg->tuner = tuner_cfg;
	cfg->i2c_adap = i2c_get_adapter(tuner_cfg->i2c_bus);
	dprintk(20, "I2C adapter = 0x%0x\n", (int)cfg->i2c_adap);

	if (cfg->i2c_adap == NULL)
	{
		dprintk(1, "Failed to allocate I2C resources for stv090x\n");
		kfree(cfg);
		return NULL;
	}
	/* toggle tuner reset */
	hc595_out(tuner_cfg->fe_rst, 0);
	msleep(50);
	hc595_out(tuner_cfg->fe_rst, 1);
	msleep(250);
	/*
	 * PLL state should be stable now. Ideally, we should check
	 * for PLL LOCK status. But well, never mind!
	 */
	frontend = frontend_init(cfg, i);

	if (frontend == NULL)
	{
		dprintk(1, "%s: No frontend found!\n", __func__);
		return NULL;
	}
	dprintk(20, "Registering frontend (adapter = 0x%x)\n", (unsigned int)adapter);

	frontend->id = i;
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
	dprintk(20, "Registering frontend successful\n");
	return frontend;
}

struct tuner_config tuner_resources[] =
{
	[0] =
	{
		.adapter   = 0,
		.i2c_bus   = 0,
		.fe_rst    = FE0_RST,
		.fe_1318   = FE0_1318,
		.fe_1419   = FE0_1419,
		.fe_lnb_en = FE0_LNB_EN
	},
#if defined(VIP2)
	[1] =
	{
		.adapter   = 0,
		.i2c_bus   = 1,
		.fe_rst    = FE1_RST,
		.fe_1318   = FE1_1318,
		.fe_1419   = FE1_1419,
		.fe_lnb_en = FE1_LNB_EN
	},
#endif
};

void fe_core_register_frontend(struct dvb_adapter *dvb_adap)
{
	int i = 0;
	int vLoop = 0;

#if defined(VIP2)
	dprintk(0, "Spider-Team plug and play frontend core for Edision argus VIP2\n");
#else  // VIP1_V2
	dprintk(0, "Spider-Team plug and play frontend core for Edision argus VIP V2\n");
#endif
	core[i] = (struct core *)kmalloc(sizeof(struct core), GFP_KERNEL);
	if (!core[i])
	{
		return;
	}
	memset(core[i], 0, sizeof(struct core));

	core[i]->dvb_adapter = dvb_adap;
	dvb_adap->priv = core[i];

#if defined(VIP2)
	dprintk(20, "# of tuner(s): %d\n", ARRAY_SIZE(tuner_resources));
#endif
	dprintk(20, "Allocating LNB control PIO pins\n");
	srclk = stpio_request_pin(2, 2, "HC595_SRCLK", STPIO_OUT);  // data clock pin
	lclk  = stpio_request_pin(2, 3, "HC595_LCLK", STPIO_OUT);  // latch pin
	sda   = stpio_request_pin(2, 4, "HC595_SDA", STPIO_OUT);  // data pin

	if ((srclk == NULL)
    ||  (lclk == NULL)
    ||  (sda == NULL))
	{
		dprintk(1, "Allocating LNB control PIO pins failed\n");

		if (srclk != NULL)
		{
			stpio_free_pin(srclk);
			dprintk(1, "Error while allocating HC595 srclk pin (PIO 2.2)\n");
		}
		if (lclk != NULL)
		{
			stpio_free_pin(lclk);
			dprintk(1, "Error while allocating HC595 lclk pin (PIO 2.3)\n");
		}
		if (sda != NULL)
		{
			stpio_free_pin(sda);
			dprintk(1, "Error while allocating HC595 sda pin (PIO 2.4)\n");
		}
		return;
	}

	for (vLoop = 0; vLoop < ARRAY_SIZE(tuner_resources); vLoop++)
	{
		if (core[i]->frontend[vLoop] == NULL)
		{
			dprintk(20, "Initialize frontend %d\n", vLoop);
			core[i]->frontend[vLoop] = init_fe_device(core[i]->dvb_adapter, &tuner_resources[vLoop], vLoop);
		}
	}
	dprintk(100, "%s: <\n", __func__);
	return;
}
EXPORT_SYMBOL(fe_core_register_frontend);

static int fe_get_demod_type(char *pcDemod)
{
	int iDemodType = STV090X;

	if ((pcDemod[0] == 0) || (strcmp("stv090x", pcDemod) == 0))
	{
		iDemodType = STV090X;
	}
	else if (strcmp("ce6353", pcDemod) == 0)
	{
		iDemodType = CE6353;
	}
	else if (strcmp("tda10023", pcDemod) == 0)
	{
		iDemodType = TDA10023;
	}
	return iDemodType;
}

static int fe_get_tuner_type(char *pcTuner)
{
	int iTunerType = SHARP7306;

	if ((pcTuner[0] == 0)
	||  (strcmp("sharp7306", pcTuner) == 0))
	{
		iTunerType = SHARP7306;
	}
	else if (strcmp("stv6110x", pcTuner) == 0)
	{
		iTunerType = STV6110X;
	}
	else if (strcmp("sharp6465", pcTuner) == 0)
	{
		iTunerType = SHARP6465;
	}
	else if (strcmp("lg031", pcTuner) == 0)
	{
		iTunerType = LG031;
	}
	return iTunerType;
}

static void printk_demod_name_and_type(int iDemodType)
{
	switch (iDemodType)
	{
		case STV090X:
		{
			printk("STM STV090X DVB-S2");
			break;
		}
		case CE6353:
		{
			printk("Intel CE6353 DVB-T");
			break;
		}
		case TDA10023:
		{
			printk("NXP TDA10023 DVB-C");
			break;
		}
	}
	printk(" - ");
}

int __init fe_core_init(void)
{
#if defined(VIP2)
	/****** FRONT END 0 ********/
	demodType1 = fe_get_demod_type(demod1);
	tunerType1 = fe_get_tuner_type(tuner1);

	dprintk(0, "Demodulator1: ");
	printk_demod_name_and_type(demodType1);
	printk("Tuner1: %s\n", tuner_name[tunerType1]);

	/****** FRONT END 1 ********/
	demodType2 = fe_get_demod_type(demod2);
	tunerType2 = fe_get_tuner_type(tuner2);

	dprintk(0, "Demodulator2: ");
	printk_demod_name_and_type(demodType2);
	printk("Tuner2: %s\n", tuner_name[tunerType2]);
#else  // VIP1_V2
	/****** FRONT END 0 ********/
	demodType = fe_get_demod_type(demod);
	tunerType = fe_get_tuner_type(tuner);

	dprintk(0, "Demodulator: ");
	printk_demod_name_and_type(demodType);
	printk("Tuner: %s\n", tuner_name[tunerType]);
#endif
	dprintk(20, "Frontend core loaded\n");
	return 0;
}

static void __exit fe_core_exit(void)
{
	dprintk(20, "Frontend core unloaded\n");
}

module_init(fe_core_init);
module_exit(fe_core_exit);

#if defined(VIP2)
module_param(demod1, charp, 0);
module_param(demod2, charp, 0);
MODULE_PARM_DESC(demod1, "demodulator1 stv090x(default), ce6353, tda10023");  // default: DVB-S2
MODULE_PARM_DESC(demod2, "demodulator2 stv090x, ce6353(default), tda10023");  // default: DVB-T

module_param(tuner1, charp, 0);
module_param(tuner2, charp, 0);
MODULE_PARM_DESC(tuner1, "tuner1 sharp7306(default), stv6110x, sharp6465, lg031");  // default: DVB-S2 Sharp 7306
MODULE_PARM_DESC(tuner2, "tuner2 sharp7306, stv6110x, sharp6465(default), lg031");  // default: DVB-T
#else  // VIP1_V2
module_param(demod, charp, 0);
MODULE_PARM_DESC(demod, "demodulator stv090x(default), ce6353, tda10023");

module_param(tuner, charp, 0);
MODULE_PARM_DESC(tuner, "tuner sharp7306(default), stv6110x, sharp6465, lg031");  // default: DVB-S2
#endif

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

MODULE_DESCRIPTION("Front end driver");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
