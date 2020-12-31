/*
 * Frontend Platform registration
 *
 * Copyright (C) 2009-2010 Duolabs Spa
 * Author: Pedro Aguilar (pedro@duolabs.com)
 *
 * Based on mb442.h
 *
 * Copyright (C) 2006 STMicroelectronics Limited
 * Author: Peter Bennett <peter.bennett@st.com>
 *
 * May be copied or modified under the terms of the GNU General Public
 * License.  See linux/COPYING for more information.
 */
#ifndef _QBOXHD_H_
#define _QBOXHD_H_

#include <linux/stm/stm-frontend.h>

/*
 * The position of the arrays define the adapter number.
 * Eg. If the DVB-S2 tuner is the first element of the array, it will be adapter0
 * Eg. If the DVB-T tuner is the second element of the array, it will be adapter1
 * demod_i2c_address = 0xff is used for auto detecting address
 */
static struct plat_frontend_channel frontend_channels[] = {
    {   
		.demod_model	   = STM_DEMOD_CX24116,
		.tuner_model	   = STM_TUNER_CX24116,
        .option            = STM_DISEQC_STONCHIP | STM_LNB_LNBP20 | STM_INVERT_CLOCK,
        .demod_i2c_bus     = 1,
        .demod_i2c_address = 0xff,
        .tuner_i2c_address = 0xff,
        .lnb_i2c_bus       = 0,
        .lnb_i2c_address   = 0x08,
        .lock              = 0,
        .drop              = 0,
        .pio_reset_bank    = 0x1,
        .pio_reset_pin     = 0x4
    },
    {   
		.demod_model	   = STM_DEMOD_AVL2108,
		.tuner_model	   = STM_TUNER_STV6306,
        .option            = STM_DISEQC_STONCHIP | STM_LNB_LNBP20 | STM_INVERT_CLOCK,
        .demod_i2c_bus     = 1,
        .demod_i2c_address = 0xff,
        .tuner_i2c_address = 0xff,
        .lnb_i2c_bus       = 0,
        .lnb_i2c_address   = 0x08,
        .lock              = 0,
        .drop              = 0,
        .pio_reset_bank    = 0x1,
        .pio_reset_pin     = 0x4
    },
    {  
		.demod_model	   = STM_DEMOD_ZL10353,
		.tuner_model	   = STM_TUNER_ZL10353,
        .option            = STM_INVERT_CLOCK,
        .demod_i2c_bus     = 1,
        .demod_i2c_address = 0xff,
        .tuner_i2c_address = 0xff,
        .lnb_i2c_bus       = 0,
        .lnb_i2c_address   = 0,
        .lock              = 0,
        .drop              = 0,
        .pio_reset_bank    = 0x1,
        .pio_reset_pin     = 0x4
    },
    {   
		.demod_model	   = STM_DEMOD_TDA10048,
		.tuner_model	   = STM_TUNER_TDA10048,
        .option            = STM_INVERT_CLOCK,
        .demod_i2c_bus     = 1,
        .demod_i2c_address = 0xff,
        .tuner_i2c_address = 0xff,
        .lnb_i2c_bus       = 0,
        .lnb_i2c_address   = 0,
        .lock              = 0,
        .drop              = 0,
        .pio_reset_bank    = 0x1,
        .pio_reset_pin     = 0x4
    },
    {  
		.demod_model	   = STM_DEMOD_TDA10023,
		.tuner_model	   = STM_TUNER_CU1216LS,
        .option            = STM_INVERT_CLOCK,
        .demod_i2c_bus     = 1,
        .demod_i2c_address = 0xff,
        .tuner_i2c_address = 0xff,
        .lnb_i2c_bus       = 0,
        .lnb_i2c_address   = 0,
        .lock              = 0,
        .drop              = 0,
        .pio_reset_bank    = 0x1,
        .pio_reset_pin     = 0x4
    }
};

static struct plat_frontend_config frontend_config = {
    .tsm_base_address     = 0x19242000,
    .tsm_sram_buffer_size = 0xc00,
    .tsm_num_pti_alt_out  = 1,
    .tsm_num_1394_alt_out = 1,
    .nr_nims          	  = ARRAY_SIZE(frontend_channels),
    .nims             	  = frontend_channels
};

static struct platform_device frontend_device = {
	.name	= "nimtuner",
	.id		= -1,
	.dev	= {
		.platform_data	= &frontend_config,
	},
};

static struct platform_device *board_7109[] __initdata = {
	&frontend_device,
};

#define BOARD_SPECIFIC_CONFIG
static int register_board_drivers(void)
{
	return platform_add_devices(board_7109, sizeof(board_7109) / sizeof(struct platform_device*));
}

#endif
