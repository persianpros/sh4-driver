/*********************************************************************
 *
 * STM STV0367 DVB-T/C demodulator driver, DVB-C mode
 *
 * Driver for STM STV0367 DVB-T & DVB-C demodulator IC.
 *
 * Version for Fulan Spark7162
 *
 * Note: actual device is part of the STi7162 SoC
 *
 * Copyright (C), 2011-2016, AV Frontier Tech. Co., Ltd.
 *
 * Date: 2011.05.09
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 * GNU General Public License for more details.
 */
#ifndef __STV0367_INNER_H
#define __STV0367_INNER_H

#ifdef __cplusplus
extern "C" {
#endif

u16 ChipGetRegAddress(u32 FieldId);
int ChipGetFieldMask(u32 FieldId);
int ChipGetFieldSign(u32 FieldId);
int ChipGetFieldPosition(u8 Mask);
int ChipGetFieldBits(int mask, int Position);
s32 ChipGetRegisterIndex(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u16 RegId);
void stv0367_write(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, unsigned char *pcData, int nbdata);
void stv0367_read(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, unsigned char *pcData, int NbRegs);

#ifdef __cplusplus
}
#endif

#endif  // __STV0367_INNER_H
// vim:ts=4
