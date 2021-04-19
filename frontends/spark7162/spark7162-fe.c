/****************************************************************************
 *
 * Spark7162 frontend driver
 * 
 * Original code Copyright (C), 2011-2016, AV Frontier Tech. Co., Ltd.
 *
 * Supports the following frontends:
 *
 * Standard   Make  Model             Demodulator       Tuner
 * ------------------------------------------------------------------
 * DVB-S(2)   Sharp    BS2S7VZ7306A   Ali M3501         Sharp IX7306
 * DVB-S(2)   Sharp    BS2S7VZ7903A   Ali M3501         Sharp VX7903
 * DVB-T/C    Sharp    ????1ED5469    STM STV0367       TI SN761640
 * DVB-T(2)/C Sharp    VA4M1EE6169    Panasonic MN88472 MaxLinear MxL301
 * DVB-T(2)/C Eardatek EDU-1165II5VB  Panasonic MN88472 MaxLinear MxL603
 *
 * Detection of which front end(s) are present is automatic and is
 * performed when the driver is initialized. Frontends not detected
 * are replaced by a dummy front end; thus the driver always provides
 * three frontends. This allter feature is required as there were
 * spark7162 receivers built with either, one, two or (the most common)
 * three frontends.
 *
 * Notes:
 * Ali M3501 demodulators mounted on main board; not in frontends;
 * STM STV0367 demodulator is part of the STi7162 SoC.
 * These facts lead to only the Sharp VA4M1EE6169 and Eardatek
 * EDU-1165II5VB beining true front ends; the other 'tin cans' are
 * merely tuners.
 *
 * Tuner I2C control:
 * Sharp BS2S7VZ7306A     : through demodulator I2C repeater 
 * Sharp BS2S7VZ7903A     : through demodulator I2C repeater 
 * Sharp ????1ED5469      : through demodulator I2C repeater 
 * Sharp VA4M1EE6169      : direct
 * Eardatek EDU-1165II5VB : direct
 *
 * It should also be noted that this model receiver was manufactured
 * in a large number of variants, depending on reseller name and date
 * of production. Models without T(2)/C or T/C tuner but with two DVB-S(2)
 * exist as well as examples with one DVB-S(2) and the T(2)/C or T/C tuner.
 * Upon initialization, the driver will automatically determine which
 * frontend(s) are present in the receiver and configure them accordingly.
 *
 * The final note is about the tuner order. This driver initializes
 * the tuners in this order:
 * The outboard DVB-S(2) will be tuner A in Enigma2;
 * The inboard DVB-S(2) will be tuner B in Enigma2;
 * The DVB-T(2)/C or DVB-T/C tuner will be tuner C in Enigma2.
 * This order is different than most frontend drivers for the Spark7162,
 * which have the DVB-T(2)/C or DVB-T/C tuner as Tuner A and the outboard
 * DVB-S(2) as tuner C. This different order requires a different
 * setup in sti-pti.c in the player2 driver.
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
 * The current version of this driver has two issues to be fixed:
 * 1. The +5V active antenna power output of the DVB-T/C tuner cannot
 *    be controlled currently. Some code is present to facilitiatie this,
 *    but the mechanism that actually controls this setting has not been
 *    found out yet.
 *    The current situation is that the +5V active antenna power output
 *    is always switched on. This is not a very desiriable condition
 *    on DVB-C. Fortunately the output is short circuit proof.
 *    The control of the power output on the T2/C tuner works as intended
 *    and is initialized as off and controllabe in DVB-T(2) mode and as
 *    always off in DVB-C mode.
 * 2. The DVB-T/C tuners of all test receivers of the author have a
 *    TI SN741640 tuner chip installed. The driver currently uses a setup
 *    sequence not specified in the data sheet as valid to setup the tuner
 *    and never writes the sixth initialisation byte, thus relying on the
 *    fact that the power-on reset value of the byte has the correct value.
 *  
 *
 ***************************************************************************/
#if !defined(SPARK7162)
#error: Wrong receiver model!
#endif

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/dvb/version.h>

#include "ywdefs.h"

#include "nim_mn88472.h"

#include "dvbdev.h"
#include "dmxdev.h"
#include "dvb_frontend.h"
#include "dvb_demux.h"

#include "dvb_dummy_fe.h"

#include "spark7162-fe.h"

// demodulators
#include "m3501_ext.h"
#include "stv0367.h"
#include "mn88472_ext.h"

// tuners
#include "sharp_ix7306.h"
#include "sharp_vx7903.h"
#include "sn761640.h"
#include "mxl301.h"

// Tuner I2C addresses
#define I2C_ADDR_IX7306    (0xc0 >> 1)  // 0x60, DVB-S(2), early production
#define I2C_ADDR_VX7903    (0xc0 >> 1)  // 0x60, DVB-S(2), late production
#define I2C_ADDR_SN761640  (0xc2 >> 1)  // 0x61, DVB-T/C, early production (AS input is 0.5 * Vcc)
#define I2C_ADDR_MXL301    (0x38 >> 1)  // 0x1c, DVB-T(2)/C, late production

#define MAX_TER_DEMOD_TYPES 2

typedef struct DemodIdentifyDbase_s
{
	u8 DemodID;

	int (*Demod_identify)(struct i2c_adapter *i2c, u8 ucID);
	int (*Demod_Register_T)(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppFrontend, struct i2c_adapter **ppI2c);
	int (*Demod_Register_C)(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppFrontend, struct i2c_adapter **ppI2c);
} DemodIdentifyDbase_T;

enum
{
	UNION_TUNER_T,
	UNION_TUNER_C,
	UNION_TUNER_NUMS,
};

static int eUnionTunerType = UNION_TUNER_T;
static char *UnionTunerType = "t";

int paramDebug = 0;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[spark7162-fe] "

struct spark_tuner_config
{
	int adapter;   /* DVB adapter number */
	int i2c_bus;   /* i2c adapter number */
	int i2c_addr;  /* i2c address */
};

struct spark_dvb_adapter_adddata
{
	struct dvb_frontend *pM3501_frontend;  // SAT tuner 1
	struct dvb_frontend *pM3501_frontend_2;  // SAT tuner 2
	struct dvb_frontend *pSTV0367_frontend_t;  // T/C tuner in T mode
	struct dvb_frontend *pSTV0367_frontend_c;  // T/C tuner in C mode

	struct i2c_adapter  *qpsk_i2c_adap;  // SAT tuner 1
	struct i2c_adapter  *qpsk_i2c_adap_2;  // SAT tuner 2
	struct i2c_adapter  *ter_i2c_adap;  // T/C tuner in T mode
	struct i2c_adapter  *cab_i2c_adap;  // T/C tuner in C mode
};

struct dvb_adapter  spark_dvb_adapter;
struct device       spark_device;

// DVB-S(2) tuner, early production
static const struct sharp_ix7306_config bs2s7hz7306a_config =
{
	.name       = "Sharp BS2S7HZ7306A",
	.addr       = I2C_ADDR_IX7306,
	.step_size  = IX7306_STEP_1000,
	.bb_lpf     = IX7306_LPF_12,
	.bb_gain    = IX7306_GAIN_2dB,
};

// DVB-S(2) tuner, late production
static const struct sharp_vx7903_config bs2s7vz7903a_config =
{
	.name         = "Sharp BS2S7VZ7903A",
	.addr         = I2C_ADDR_VX7903,
//	.DemodI2cAddr = 0x33,  // Ali M3501
};

// DVB-T tuner (in Sharp ????1ED5469)
static const struct sn761640_config sn761640t_config =
{
	.name       = "Texas Instruments SN761640 (DVB-T)",
	.addr       = I2C_ADDR_SN761640,
	.bandwidth  = BANDWIDTH_8_MHZ,

	.Frequency  = 500000,
	.IF         = 36167,
	.TunerStep  = 16667,
};

// DVB-C tuner (in Sharp ????1ED5469)
static const struct sn761640_config sn761640c_config =
{
	.name       = "Texas Instruments SN761640 (DVB-C)",
	.addr       = I2C_ADDR_SN761640,
};

// DVB-T(2) tuner (late production, in Sharp VA4M1EE6169)
static const struct MxL301_config mxl301_config =
{
	.name       = "MaxLinear MxL301",
	.addr       = I2C_ADDR_MXL301,
	.bandwidth  = BANDWIDTH_8_MHZ,

	.Frequency  = 500000,
	.IF         = 36167,
	.TunerStep  = 16667,
};

// DVB-T(2) tuner (late production, in Eardatek EDU-1165II5VB)
static const struct MxL301_config mxl603_config =
{
	.name       = "MaxLinear MxL603",
	.addr       = I2C_ADDR_MXL301,
	.bandwidth  = BANDWIDTH_8_MHZ,

	.Frequency  = 500000,
	.IF         = 36167,
	.TunerStep  = 16667,
};

static struct sharp_vx7903_i2c_config vx7903_i2cConfig =
{
	.addr = 0x60,
	.DemodI2cAddr = 0x33,
};

// I2C data for frontends
struct spark_tuner_config tuner_resources[] =
{
	[0] =  // leftmost DVB-S(2) tuner (marked TUNER1 on rear panel)
	{
		.adapter = 0,   /* DVB adapter number */
		.i2c_bus = 1,   /* i2c bus number */
	},
	[1] = // center DVB-S(2) tuner (marked TUNER2 on rear panel)
	{
		.adapter = 0,   /* DVB adapter number */
		.i2c_bus = 3,   /* i2c bus number */
	},
	[2] =  // DVB-T(2)/C tuner (marked TUNER3 T2/C on rear panel) 
	{
		.adapter = 0,   /* DVB adapter number */
		.i2c_bus = 2,   /* i2c bus number */
	},
	[3] =  // DVB-T/C tuner (marked TUNER3 T/C on rear panel)
	{
		.adapter = 0,   /* DVB adapter number */
		.i2c_bus = 2,   /* i2c bus number */
	},
};

enum tuner_type  // pertains to DVB-S(2) tuners
{
//	STB6100,   // not used
//	STV6110X,  // not used
	IX7306,
	VX7903,
	TunerUnknown,
};

void frontend_find_TunerDevice(enum tuner_type *ptunerType, struct i2c_adapter *i2c, struct dvb_frontend *frontend)
{
	int identify = 0;

	identify = tuner_sharp_vx7903_identify(frontend, &vx7903_i2cConfig, (void *)i2c);
	if (identify == 0)
	{
		dprintk(20, "Found tuner %s\n", bs2s7vz7903a_config.name);
		*ptunerType = VX7903;
		return;
	}
	identify = tuner_sharp_ix7306_identify(frontend, &bs2s7hz7306a_config, i2c);
	if (identify == 0)
	{
		dprintk(20, "Found tuner %s\n", bs2s7hz7306a_config.name);
		*ptunerType = IX7306;
		return;
	}
	// add new tuner identify here
	*ptunerType = TunerUnknown;
	dprintk(1, "Error: No known tuner found\n");
}

int spark_dvb_attach_s(struct dvb_adapter *dvb_adap, const struct m3501_config *config, struct dvb_frontend **ppFrontend)
{
	struct dvb_frontend *pFrontend = NULL;
	enum tuner_type tunerType = TunerUnknown;

	pFrontend = dvb_m3501_fe_qpsk_attach(config, config->pI2c);
	if (!pFrontend)
	{
		dprintk(1, "%s: Error attaching M3501\n", __func__);
		return -1;
	}
	dprintk(20, "Ali M3501 demodulator attached\n");
	frontend_find_TunerDevice(&tunerType, config->pI2c, pFrontend);

	switch (tunerType)
	{
		case VX7903:
		{
			if (!dvb_attach(vx7903_attach, pFrontend, &vx7903_i2cConfig, config->pI2c))
			{
				dprintk(1, "%s: Error attaching %s tuner\n", __func__, bs2s7vz7903a_config.name);
				dvb_frontend_detach(pFrontend);
				return -1;
			}
			dprintk(20, "Attached %s tuner\n", bs2s7vz7903a_config.name);
			break;
		}
		case IX7306:
		default:
		{
			if (!dvb_attach(ix7306_attach, pFrontend, &bs2s7hz7306a_config, config->pI2c))
			{
				dprintk(1, "%s: Error attaching %s tuner\n", __func__, bs2s7hz7306a_config.name);
				dvb_frontend_detach(pFrontend);
				return -1;
			}
			dprintk(20, "Attached %s tuner\n", bs2s7hz7306a_config.name);
			break;
		}
	}
	*ppFrontend = pFrontend;
	return 0;
}

int spark_dvb_register_s(struct dvb_adapter *dvb_adap, int tuner_resource, struct dvb_frontend **ppFrontend, struct i2c_adapter **ppI2c)
{
	struct dvb_frontend *pFrontend;
	struct i2c_adapter  *pI2c;
	struct m3501_config m3501config;

	dprintk(50, "%s: I2C bus = %d\n", __func__, tuner_resources[tuner_resource].i2c_bus);
	pI2c = i2c_get_adapter(tuner_resources[tuner_resource].i2c_bus);

	if (!pI2c)
	{
		return -1;
	}
	m3501config.pI2c = pI2c;
	m3501config.i = tuner_resource;
	if (0 == tuner_resource)
	{
		snprintf(m3501config.name, sizeof(m3501config.name), "Ali M3501-2");
	}
	else
	{
		snprintf(m3501config.name, sizeof(m3501config.name), "Ali M3501-1");
	}

	if (spark_dvb_attach_s(dvb_adap, &m3501config, &pFrontend))
	{
		pFrontend = dvb_dummy_fe_qpsk_attach();
	}
	pFrontend->id = tuner_resource;

	if (dvb_register_frontend(dvb_adap, pFrontend))
	{
		dprintk(1, "%s: Ali M3501 Frontend registration failed!\n", __func__);
		dvb_frontend_detach(pFrontend);
		i2c_put_adapter(pI2c);
		return -1;
	}
	*ppFrontend = pFrontend;
	*ppI2c      = pI2c;
	return 0;
}

int spark_dvb_register_s0(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppFrontend, struct i2c_adapter **ppI2c)
{
	return spark_dvb_register_s(dvb_adap, 0, ppFrontend, ppI2c);
}

int spark_dvb_register_s1(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppFrontend, struct i2c_adapter **ppI2c)
{
	return spark_dvb_register_s(dvb_adap, 1, ppFrontend, ppI2c);
}

int spark_dvb_attach_t(struct dvb_adapter *dvb_adap, struct i2c_adapter *pI2c, struct dvb_frontend **ppFrontend)
{
	struct dvb_frontend *pFrontend = NULL;

	pFrontend = dvb_stv0367_ter_attach(pI2c);

	if (!pFrontend)
	{
		dprintk(1, "%s: Error attaching STM STV0367 demodulator (DVB-T mode)\n", __func__);
		return -1;
	}
	dprintk(20, "STM STV0367 demodulator (DVB-T mode) attached\n");

	if (!dvb_attach(sn761640t_attach, pFrontend, &sn761640t_config, pI2c))
	{
		dprintk(1, "%s: Error attaching %s tuner\n", __func__, sn761640t_config.name);
		dvb_frontend_detach(pFrontend);
		return -1;
	}
	dprintk(20, "Attached %s tuner\n", sn761640t_config.name);
	*ppFrontend = pFrontend;
	return 0;
}

// T2 add by yanbinL
int spark_dvb_attach_T2(struct dvb_adapter *dvb_adap, struct i2c_adapter *pI2c, struct dvb_frontend **ppFrontend, u8 system)
{
	struct dvb_frontend *pFrontend = NULL;

	pFrontend = dvb_mn88472_attach(pI2c, system);

	if (!pFrontend)
	{
		dprintk(1, "%s: Error attaching Sharp T2/C frontend, trying Eardatek frontend\n", __func__);
		pFrontend = dvb_mn88472earda_attach(pI2c, system);
		if (!pFrontend)
		{
			dprintk(1, "%s: Error attaching Eardatek frontend demodulator\n", __func__);
			return -1;
		}
		else
		{
			dprintk(20, "MN88472 (Eardatek T2/C frontend) attached\n");
			if (!dvb_attach(mxlx0x_attach, pFrontend, &mxl603_config, pI2c))
			{
				dprintk(1, "%s: Error attaching %s tuner\n", __func__, mxl603_config.name);
				dvb_frontend_detach(pFrontend);
				return -1;
			}
			dprintk(20, "Attached %s tuner\n", mxl603_config.name);
		}
	}
	else
	{
		dprintk(20, "MN88472 (Sharp T2/C frontend) attached\n");
		if (!dvb_attach(mxlx0x_attach, pFrontend, &mxl301_config, pI2c))
		{
			dprintk(1, "%s: Error attaching %s tuner\n", __func__, mxl301_config.name);
			dvb_frontend_detach(pFrontend);
			return -1;
		}
		dprintk(20, "Attached %s tuner\n", mxl301_config.name);
	}
	*ppFrontend = pFrontend;
	return 0;
}

int spark_dvb_register_t(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppFrontend, struct i2c_adapter **ppI2c)
{
	struct dvb_frontend *pFrontend;

	if (spark_dvb_attach_t(dvb_adap, *ppI2c, &pFrontend))
	{
		pFrontend = dvb_dummy_fe_ofdm_attach();
	}
	pFrontend->id = 3;

	if (dvb_register_frontend(dvb_adap, pFrontend))
	{
		dprintk(1, "STM STV0367 (DVB-T mode): Frontend registration failed!\n");
		dvb_frontend_detach(pFrontend);
		i2c_put_adapter(*ppI2c);
		return -1;
	}
	*ppFrontend = pFrontend;
	return 0;
}

// T2 add by yanbin
int spark_dvb_register_T2(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppFrontend, struct i2c_adapter **ppI2c)
{
	struct dvb_frontend *pFrontend;

	if (spark_dvb_attach_T2(dvb_adap, *ppI2c, &pFrontend, DEMOD_BANK_T2))
	{
		i2c_put_adapter(*ppI2c);
		return -1;
	}
	pFrontend->id = 3;

	if (dvb_register_frontend(dvb_adap, pFrontend))
	{
		dprintk(1, "MN88472 (DVB-T2 mode): Frontend registration failed!\n");
		dvb_frontend_detach(pFrontend);
		i2c_put_adapter(*ppI2c);
		return -1;
	}
	// struct dvb_frontend_private *fepriv = pFrontend->frontend_priv;
	*ppFrontend = pFrontend;
	return 0;
}

int spark_dvb_register_c_T2(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppFrontend, struct i2c_adapter **ppI2c)
{
	struct dvb_frontend *pFrontend;

	if (spark_dvb_attach_T2(dvb_adap, *ppI2c, &pFrontend, DEMOD_BANK_C))
	{
		i2c_put_adapter(*ppI2c);
		return -1;
	}
	pFrontend->id = 3;

	if (dvb_register_frontend(dvb_adap, pFrontend))
	{
		dprintk(1, "MN88472 (DVB-C mode): Frontend registration failed!\n");
		dvb_frontend_detach(pFrontend);
		i2c_put_adapter(*ppI2c);
		return -1;
	}
	// struct dvb_frontend_private *fepriv = pFrontend->frontend_priv;
	*ppFrontend = pFrontend;
	return 0;
}

int spark_dvb_attach_c(struct dvb_adapter *dvb_adap, struct i2c_adapter *pI2c, struct dvb_frontend **ppFrontend)
{
	struct dvb_frontend *pFrontend = NULL;

	pFrontend = dvb_stv0367_cab_attach(pI2c);

	if (!pFrontend)
	{
		dprintk(1, "%s: Error attaching STM STV0367 demodulator (DVB-C mode)\n", __func__);
		return -1;
	}
	dprintk(20, "STM STV0367 demodulator (DVB-C mode) attached\n");

	if (!dvb_attach(sn761640c_attach, pFrontend, &sn761640c_config, pI2c))
	{
		dprintk(1, "%s: Error attaching %s tuner\n", __func__, sn761640c_config.name);
		dvb_frontend_detach(pFrontend);
		return -1;
	}
	dprintk(20, "Attached %s tuner\n", sn761640c_config.name);
	*ppFrontend = pFrontend;
	return 0;
}

int spark_dvb_register_c(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppCabFrontend, struct i2c_adapter **ppCabI2c)
{
	struct dvb_frontend *pFrontend;
	struct i2c_adapter  *pI2c;

	pI2c = i2c_get_adapter(2);
	dprintk(100, "%s > pI2c = 0x%0x\n", __func__, (int)pI2c);

	if (spark_dvb_attach_c(dvb_adap, pI2c, &pFrontend))
	{
		pFrontend = dvb_dummy_fe_qam_attach();
	}
	pFrontend->id = 3;
	if (dvb_register_frontend(dvb_adap, pFrontend))
	{
		dprintk(1, "STM STV0367 demodulator (DVB-C mode): Frontend registration failed!\n");
		dvb_frontend_detach(pFrontend);
		i2c_put_adapter(pI2c);
		return -1;
	}
	*ppCabFrontend = pFrontend;
	*ppCabI2c      = pI2c;
	return 0;
}

static DemodIdentifyDbase_T DemodIdentifyTable[MAX_TER_DEMOD_TYPES] =
{
	{
		// Panasonic MN88472 demodulator
		.DemodID          = 0x02,
		.Demod_identify   = demod_mn88472_identify,
		.Demod_Register_T = spark_dvb_register_T2,
		.Demod_Register_C = spark_dvb_register_c_T2
	},
	{
		// STM STV0367 demodulator
		.DemodID          = 0x60,
		.Demod_identify   = demod_stv0367_identify,
		.Demod_Register_T = spark_dvb_register_t,
		.Demod_Register_C = spark_dvb_register_c
	},
};

int spark_dvb_register_dummy_t(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppFrontend, struct i2c_adapter **ppI2c)
{
	struct dvb_frontend *pFrontend;

	pFrontend = dvb_dummy_fe_ofdm_attach();

	pFrontend->id = 3;

	if (dvb_register_frontend(dvb_adap, pFrontend))
	{
		dprintk(1, "%s: dummy_fe (DVB-T(2) mode): Frontend registration failed!\n", __func__);
		dvb_frontend_detach(pFrontend);
		i2c_put_adapter(*ppI2c);
		return -1;
	}
	*ppFrontend = pFrontend;
	return 0;
}

int spark_dvb_register_dummy_c(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppFrontend, struct i2c_adapter **ppI2c)
{
	struct dvb_frontend *pFrontend;

	pFrontend = dvb_dummy_fe_qam_attach();
	pFrontend->id = 3;

	if (dvb_register_frontend(dvb_adap, pFrontend))
	{
		dprintk(1, "%s: dummy_fe (DVB-C mode): Frontend registration failed!\n", __func__);
		dvb_frontend_detach(pFrontend);
		i2c_put_adapter(*ppI2c);
		return -1;
	}
	*ppFrontend = pFrontend;
	return 0;
}

int spark_dvb_AutoRegister_TER(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppFrontend, struct i2c_adapter **ppI2c)
{
	int ret = -1;
	struct i2c_adapter *pI2c;
	int i;

	pI2c = i2c_get_adapter(2);
	//dprintk(50, "%s: pI2c = 0x%0x\n", __func__, (int)pI2c);
	if (!pI2c)
	{
		return -1;
	}
	for (i = 0; i < MAX_TER_DEMOD_TYPES; i++)
	{
		if (0 == (DemodIdentifyTable[i].Demod_identify)(pI2c, DemodIdentifyTable[i].DemodID))
		{
			*ppI2c = pI2c;
			ret = DemodIdentifyTable[i].Demod_Register_T(dvb_adap, ppFrontend, ppI2c);
			break;
		}
	}

	if (MAX_TER_DEMOD_TYPES == i)
	{
		*ppI2c = pI2c;
		ret = spark_dvb_register_dummy_t(dvb_adap, ppFrontend, ppI2c);
	}
	return ret;
}

int spark_dvb_AutoRegister_Cab(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppFrontend, struct i2c_adapter **ppI2c)
{
	int ret = -1;
	struct i2c_adapter *pI2c;
	int i;

	dprintk(100, "%s >\n", __func__);
	pI2c = i2c_get_adapter(2);
	//dprintk(50, "%s: pI2c = 0x%0x\n", __func__, (int)pI2c);
	if (!pI2c)
	{
		return -1;
	}
	for (i = 0; i < 2; i++)
	{
		if (0 == (DemodIdentifyTable[i].Demod_identify)(pI2c, DemodIdentifyTable[i].DemodID))
		{
			*ppI2c = pI2c;
			ret = DemodIdentifyTable[i].Demod_Register_C(dvb_adap, ppFrontend, ppI2c);
			break;
		}
	}
	if (MAX_TER_DEMOD_TYPES == i)
	{
		*ppI2c = pI2c;
		ret = spark_dvb_register_dummy_c(dvb_adap, ppFrontend, ppI2c);
	}
	return ret;
}

int spark_dvb_register_tc_by_type(struct dvb_adapter *dvb_adap, int iTunerType)
{
	int iRet = -1;
	struct spark_dvb_adapter_adddata *pDvbAddData;

	pDvbAddData = (struct spark_dvb_adapter_adddata *)dvb_adap->priv;

	if (UNION_TUNER_T == iTunerType)
	{
		iRet = spark_dvb_AutoRegister_TER(dvb_adap, &pDvbAddData->pSTV0367_frontend_t, &pDvbAddData->ter_i2c_adap);
	}
	else if (UNION_TUNER_C == iTunerType)
	{
		iRet = spark_dvb_AutoRegister_Cab(dvb_adap, &pDvbAddData->pSTV0367_frontend_c, &pDvbAddData->cab_i2c_adap);
	}
	else
	{
		iRet = spark_dvb_register_t(dvb_adap, &pDvbAddData->pSTV0367_frontend_t, &pDvbAddData->ter_i2c_adap);
	}
	return iRet;
}

int spark_dvb_register_tc(struct dvb_adapter *dvb_adap, struct dvb_frontend **ppTerFrontend, struct i2c_adapter **ppTerI2c, struct dvb_frontend **ppCabFrontend, struct i2c_adapter **ppCabI2c)
{
	spark_dvb_register_t(dvb_adap, ppTerFrontend, ppTerI2c);
	spark_dvb_register_c(dvb_adap, ppCabFrontend, ppCabI2c);
	return 0;
}

int spark_dvb_unregister_f(struct dvb_frontend *pFrontend, struct i2c_adapter *pI2c)
{
	dvb_unregister_frontend(pFrontend);
	dvb_frontend_detach(pFrontend);
	i2c_put_adapter(pI2c);
	return 0;
}

int spark_dvb_unregister_s(struct dvb_frontend *pFrontend, struct i2c_adapter *pI2c)
{
	return spark_dvb_unregister_f(pFrontend, pI2c);
}

int spark_dvb_unregister_s0(struct dvb_adapter *dvb_adap)
{
	struct dvb_frontend *pFrontend;
	struct i2c_adapter *pI2c;
	struct spark_dvb_adapter_adddata *pDvbAddData;

	pDvbAddData = (struct spark_dvb_adapter_adddata *)dvb_adap->priv;

	pFrontend = pDvbAddData->pM3501_frontend;
	pI2c = pDvbAddData->qpsk_i2c_adap;

	spark_dvb_unregister_s(pFrontend, pI2c);
	return 0;
}

int spark_dvb_unregister_s1(struct dvb_adapter *dvb_adap)
{
	struct dvb_frontend *pFrontend;
	struct i2c_adapter *pI2c;
	struct spark_dvb_adapter_adddata *pDvbAddData;

	pDvbAddData = (struct spark_dvb_adapter_adddata *)dvb_adap->priv;

	pFrontend = pDvbAddData->pM3501_frontend_2;
	pI2c = pDvbAddData->qpsk_i2c_adap_2;

	spark_dvb_unregister_s(pFrontend, pI2c);
	return 0;
}

int spark_dvb_unregister_t(struct dvb_adapter *dvb_adap)
{
	struct dvb_frontend *pFrontend;
	struct i2c_adapter *pI2c;
	struct spark_dvb_adapter_adddata *pDvbAddData;

	pDvbAddData = (struct spark_dvb_adapter_adddata *)dvb_adap->priv;
	pFrontend = pDvbAddData->pSTV0367_frontend_t;
	pI2c = pDvbAddData->ter_i2c_adap;

	spark_dvb_unregister_f(pFrontend, pI2c);
	return 0;
}

int spark_dvb_unregister_c(struct dvb_adapter *dvb_adap)
{
	struct dvb_frontend *pFrontend;
	struct i2c_adapter *pI2c;
	struct spark_dvb_adapter_adddata *pDvbAddData;
	pDvbAddData = (struct spark_dvb_adapter_adddata *)dvb_adap->priv;
	pFrontend = pDvbAddData->pSTV0367_frontend_c;
	pI2c = pDvbAddData->cab_i2c_adap;
	spark_dvb_unregister_f(pFrontend, pI2c);
	return 0;
}

int spark_dvb_unregister_tc_by_type(struct dvb_adapter *dvb_adap, int iTunerType)
{
	int iRet = -1;

	if (UNION_TUNER_T == iTunerType)
	{
		iRet = spark_dvb_unregister_t(dvb_adap);
	}
	else if (UNION_TUNER_C == iTunerType)
	{
		iRet = spark_dvb_unregister_c(dvb_adap);
	}
	else
	{
		iRet = spark_dvb_unregister_t(dvb_adap);
	}
	return iRet;
}

int UnionTunerConfig(char *pcTunerType)
{
	int eTunerType = UNION_TUNER_T;

	dprintk(100, "%s: > Tuner type to set: %s\n", __func__, pcTunerType);
//	if (!pcTunerType)
//	{
//		return UNION_TUNER_T;
//	}
	if (!strcmp("t", pcTunerType))
	{
		dprintk(20, "Setting T(2)/C tuner to T(2)\n");
		eTunerType = UNION_TUNER_T;
	}
	else if (!strcmp("c", pcTunerType))
	{
		eTunerType = UNION_TUNER_C;
		dprintk(20, "Setting T(2)/C tuner to C\n");
	}
	else
	{
		dprintk(1, "%s: Unknown tuner type %s, defaulting to t\n", __func__, pcTunerType);
		eTunerType = UNION_TUNER_T;
	}
	dprintk(100, "%s: <\n", __func__);
	return eTunerType;
}

int spark7162_register_frontend(struct dvb_adapter *dvb_adap)
{
	struct spark_dvb_adapter_adddata *pDvbAddData;

	pDvbAddData = kmalloc(sizeof(struct spark_dvb_adapter_adddata), GFP_KERNEL);

	memset(pDvbAddData, 0, sizeof(struct spark_dvb_adapter_adddata));

	dvb_adap->priv = (void *)pDvbAddData;

	spark_dvb_register_s1(dvb_adap, &pDvbAddData->pM3501_frontend_2, &pDvbAddData->qpsk_i2c_adap_2);
	spark_dvb_register_s0(dvb_adap, &pDvbAddData->pM3501_frontend, &pDvbAddData->qpsk_i2c_adap);
	// determine tuner3 type (T or C)
	eUnionTunerType = UnionTunerConfig(UnionTunerType);
	spark_dvb_register_tc_by_type(dvb_adap, eUnionTunerType);
	return 0;
}
EXPORT_SYMBOL(spark7162_register_frontend);

void spark7162_unregister_frontend(struct dvb_adapter *dvb_adap)
{
	struct spark_dvb_adapter_adddata *pDvbAddData;

	pDvbAddData = (struct spark_dvb_adapter_adddata *)dvb_adap->priv;
#if 1
	spark_dvb_unregister_s0(dvb_adap);

//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);

	spark_dvb_unregister_s1(dvb_adap);

//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
#endif
	spark_dvb_unregister_tc_by_type(dvb_adap, eUnionTunerType);

//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);

	kfree(pDvbAddData);
}
EXPORT_SYMBOL(spark7162_unregister_frontend);

int __init spark_init(void)
{
	int ret;
#if (DVB_API_VERSION > 3)
	short int AdapterNumbers[] = { -1 };
#endif
	dprintk(100, "%s >\n", __func__);

#if (DVB_API_VERSION > 3)
	ret = dvb_register_adapter(&spark_dvb_adapter, "spark7162-fe", THIS_MODULE, &spark_device, AdapterNumbers);
#else
	ret = dvb_register_adapter(&spark_dvb_adapter, "spark7162-fe", THIS_MODULE, &spark_device);
#endif
//	dprintk(50, "ret = %d\n", ret);

	ret = spark7162_register_frontend(&spark_dvb_adapter);
	if (ret < 0)
	{
		dvb_unregister_adapter(&spark_dvb_adapter);
		return -1;
	}
	dprintk(100, "%s < %d\n", __func__, ret);
	return ret;
}

void __exit spark_cleanup(void)
{
	dprintk(100, "%s >\n", __func__);
	spark7162_unregister_frontend(&spark_dvb_adapter);
	dvb_unregister_adapter(&spark_dvb_adapter);
	dprintk(100, "%s <\n", __func__);
}

//#define M3501_SINGLE
#ifdef M3501_SINGLE
module_init(spark_init);
module_exit(spark_cleanup);
#endif

module_param(UnionTunerType, charp, 0);
MODULE_PARM_DESC(UnionTunerType, "UnionTunerType (t, c)");

module_param(paramDebug, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled, >0=enabled (level)");
// vim:ts=4
