/*
 * @brief avl2108_platform.c
 *
 * @author konfetti
 *
 * Version for Opticum HD 9600 and Opticum HD 9600 PRIMA series.
 *
 * 	Copyright (C) 2011 duckbox
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/platform_device.h>

#include "avl2108_platform.h"
#include "avl2108_reg.h"
#include "avl2108.h"

short paramDebug = 0;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[avl2108_platform] "

/*
 * DVB-S(2) Frontend is a Sharp BS2F7VZ7700 (AVL2108 demodulator + IX2470VA tuner),
 * connected as follows:
 * DVBTPWREN#  : PIO2.5 (power enable for DVB-T, active low)
 * DVBS2OE#    : PIO3.3 (?, active low)
 * DVBS2PWREN# : PIO4.6 (power enable for DVB-S(2), active low)
 * Tuner RESET : PIO5.3 (active low)
 *  
 * LNB power driver is an STM LNBP12, connected as follows:
 *
 * Voltage select (pin 4, 13V = low)           : PIO2.2
 * Enable/power off (pin 5, Enable = high)     : PIO5.2
 * Tone enable (pin 7, high = tone on)         : PIO2.3 (currently not used by driver, initialized to 0)
 * 1V Vout lift (pin 9, LLC input, high = +1V) : PIO2.6
 *
 * Driver currently does not support the DVB-T frontend of TS models
 * and disables it in the driver initialization by powering it off.
 *
 */
static struct avl_private_data_s avl_tuner_priv =
{
	.ref_freq         = 1,
	.demod_freq       = 11200,  /* fixme: the next three could be determined by the pll config!!! */
	.fec_freq         = 16800,
	.mpeg_freq        = 22400,
	.i2c_speed_khz    = TUNER_I2C_CLK,
	.agc_polarization = AGC_POL_INVERT,
	.mpeg_mode        = MPEG_FORMAT_TS_PAR,
	.mpeg_serial      = MPEG_MODE_PARALLEL,
	.mpeg_clk_mode    = MPEG_CLK_MODE_RISING,
	.max_lpf          = 320,
	.pll_config       = 5,
	.usedTuner        = cTUNER_EXT_IX2470,
	.usedLNB          = cLNB_LNBP12,
	.lpf              = 340,
	.lock_mode        = LOCK_MODE_ADAPTIVE,
	.iq_swap          = CI_FLAG_IQ_NO_SWAPPED,
	.auto_iq_swap     = CI_FLAG_IQ_AUTO_BIT_AUTO,
	.agc_ref          = 0x0
};

static struct platform_frontend_s avl2108_config =
{
	.numFrontends = 1,
	.frontendList = (struct platform_frontend_config_s[])
	{
		[0] =
		{
			.name         = "avl2108",

			.tuner_enable = { 5, 3, 1 },  // group, bit, state for active
			.lnb          = { 5, 2, 1, 2, 2, 0 },  // enable (group, bit, state for on), voltage select (group, bit, state for 13V)
			.i2c_bus      = 1,

			.demod_i2c    = 0x0C, // 0x18 >> 1
//			NOTE: I2C address used for the IX2470 determines tthe value of its ADR input
			.tuner_i2c    = 0xC0, // ADR input voltage < 0.1 * Vcc
//			.tuner_i2c    = 0xC2, // ADR input open
//			.tuner_i2c    = 0xC4, // 0.4 * Vcc < ADR input voltage < 0.6 * Vcc
//			.tuner_i2c    = 0xC6, // ADR input voltage > 0.9 * Vcc
			.private      = &avl_tuner_priv
		},
	},
};

static struct platform_device avl2108_device =
{
	.name    = "avl2108",
	.id      = -1,
	.dev     =
	{
		.platform_data = &avl2108_config,
	},
	.num_resources        = 0,
	.resource             = NULL
};

static struct platform_device *platform[] __initdata =
{
	&avl2108_device
};

int __init avl2108_platform_init(void)
{
	int ret;

	dprintk(150, "%s >\n", __func__);
	ret = platform_add_devices(platform, sizeof(platform) / sizeof(struct platform_device *));
	if (ret != 0)
	{
		dprintk(1, "Failed to register AVL2108 platform device\n");
	}
	dprintk(150, "%s < (ret = %d)\n", __func__, ret);
	return ret;
}

module_init(avl2108_platform_init);

MODULE_DESCRIPTION("Unified AVL2108 driver using platform device model");

MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");
// vim:ts=4
