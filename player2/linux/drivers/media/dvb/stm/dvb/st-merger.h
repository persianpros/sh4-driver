/*
 * STM TSMerger driver
 *
 * Copyright (c) STMicroelectronics 2006
 *
 * Author:Peter Bennett <peter.bennett@st.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef _ST_MERGER_H
#define _ST_MERGER_H

#include <linux/stm/stm-frontend.h>

#if defined(QBOXHD) || defined(QBOXHD_MINI)
void stm_tsm_init ( int cfg, struct plat_frontend_config *nims_config);
#else
void stm_tsm_init(int cfg);
#endif
void stm_tsm_release(void);
void stm_tsm_interrupt(void);

#endif

