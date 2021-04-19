/****************************************************************************
 *
 * STM STV0367 DVB-T/C demodulator driver, DVB-T mode
 *
 * Version for Fulan Spark7162
 *
 * Note: actual device is part of the STi7162 SoC
 *
 * Original code:
 * Copyright (C), 2005-2010, AV Frontier Tech. Co., Ltd.
 *
 * File: stv0367_ter.h
 *
 * History:
 *  Date            Athor       Version   Reason
 *  ============    =========   =======   =================
 *  2005-10-12      chm         1.0       File creation
 *
 ****************************************************************************/
#ifndef _D0367_TER_H
#define _D0367_TER_H

#define NINV 0
#define INV 1

extern STCHIP_Register_t Def367Val[STV0367TER_NBREGS];

void stv0367ter_init(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, TUNER_TunerType_T TunerType);
u32 demod_stv0367ter_Repeat(u32 DemodIOHandle, u32 TunerIOHandle, TUNER_IOARCH_Operation_t Operation,
			 u16 SubAddr, u8 *Data, u32 TransferSize, u32 Timeout);
u32 demod_stv0367ter_Open(u8 Index);
u32 demod_stv0367ter_GetSignalInfo(u8 Index, u32 *Quality, u32 *Intensity, u32 *Ber);
u32 demod_stv0367ter_IsLocked(u8 Handle, BOOL *IsLocked);
u32 demod_stv0367ter_Close(u8 Index);
u32 demod_stv0367ter_SetStandby(u8 Handle);
u32 demod_stv0367ter_ScanFreq(u8 Handle);
u32 demod_stv0367ter_Reset(u8 Handle);
u32 demod_stv0367ter_Open_test(u32 IOHandle);
#endif  // _STV0367_TER_H
// vim:ts=4
