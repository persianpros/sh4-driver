/*
 * Panasonic MN88472 DVB-T(2)/C demodulator driver
 *
 * Version for Fulan Spark7162
 *
 * Original code:
 * Copyright (C), 2013-2018, AV Frontier Tech. Co., Ltd.
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
 */
#ifndef __MN88472_H
#define __MN88472_H

#include "nim_mn88472.h"

#ifdef __cplusplus
extern "C" {
#endif

u32 demod_mn88472_Open(u8 Handle);
u32 demod_mn88472_Close(u8 Index);
u32 demod_mn88472_IsLocked(u8 Handle, BOOL *IsLocked);
u32 demod_mn88472_GetSignalInfo(u8 Handle, u32 *Quality, u32 *Intensity, u32 *Ber);
u32 demod_mn88472_ScanFreq(struct dvb_frontend_parameters *p, struct nim_device *dev, u8 System, u8 plp_id);
u32 demod_mn88472earda_ScanFreq(struct dvb_frontend_parameters *p, struct nim_device *dev, u8 System, u8 plp_id);
void nim_config_EARDATEK11658(struct COFDM_TUNER_CONFIG_API *Tuner_API_T, u32 i2c_id, u8 idx);

#ifdef __cplusplus
}
#endif

#endif  // __MN88472_H
// vim:ts=4
