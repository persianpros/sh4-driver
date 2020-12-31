/*
 * @file nim_tuner.h
 * @brief Export NIMs funcionality: init, attach and get NIMs
 *
 * Copyright (c) 2010 Duolabs spa
 *
 * @author Pedro Aguilar <pedro@duolabs.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef _NIM_TUNER_H
#define _NIM_TUNER_H

#include <linux/stm/stm-frontend.h>
#include "dvbdev.h"

#if defined(QBOXHD)
#define NIMS_NUM_MAX	3	/**< Max number of supported NIMs */
#elif defined(QBOXHD_MINI)
#define NIMS_NUM_MAX	2	/**< Max number of supported NIMs */
#endif

#if defined(QBOXHD) || defined(QBOXHD_MINI)

/* Used for exporting NIMs info in /proc thorugh e2_proc_bus */
struct nim_info_st {
    char    type[16];
    char    name[16];
    int     nim_pos;
};

int nim_tuner_init_frontend(struct dvb_adapter *dvb_adap);
int nim_tuner_attach_frontend(struct dvb_adapter *dvb_adap, int nr_adapters);
int get_nims_num_found(int *tuners_found);
int get_nims_config(struct plat_frontend_config * config);
int get_nims_info(struct nim_info_st *nims_info);

#endif
#endif	/* _NIM_TUNER_H */
