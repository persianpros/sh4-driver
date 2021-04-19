/****************************************************************************
 *
 * STM STV0367 DVB-T/C demodulator driver, DVB-C mode
 *
 * Version for Fulan Spark7162
 *
 * Note: actual device is part of the STi7162 SoC
 *
 * Original code:
 * Copyright (C), 2005-2010, AV Frontier Tech. Co., Ltd.
 *
 * File:  stv0376_qam.h
 *
 *     Date            Author      Version   Reason
 *     ============    =========   =======   =================
 *     2010-12-28      lwj         1.0
 *
 **************************************************************************/

#ifndef _STV0367_CAB_H
#define _STV0367_CAB_H

extern u32 FirstTimeBER[3];

/* functions --------------------------------------------------------------- */

u32 stv0367cab_init(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, TUNER_TunerType_T TunerType);
u32 stv0367cab_Sleep(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle);
u32 stv0367cab_Wake(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle);
u32 stv0367cab_I2C_gate_On(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle);
u32 stv0367cab_I2C_gate_Off(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle);
u32 demod_stv0367cab_Identify(u32 IOHandle, u8 ucID, u8 *pucActualID);
u32 demod_stv0367cab_Repeat(u32 DemodIOHandle, u32 TunerIOHandle, TUNER_IOARCH_Operation_t Operation,
		u16 SubAddr, u8 *Data, u32 TransferSize, u32 Timeout);
u32 demod_stv0367cab_Open(u8 Index);
u32 demod_stv0367cab_GetSignalInfo(u8 Index, u32 *Quality, u32 *Intensity, u32 *Ber);
u32 demod_stv0367cab_IsLocked(u8 Handle, BOOL *IsLocked);
u32 demod_stv0367cab_Close(u8 Index);
u32 demod_stv0367cab_SetStandby(u8 Handle);
u32 demod_stv0367cab_ScanFreq(u8 Handle);
u32 demod_stv0367cab_Reset(u8 Handle);
u32 demod_stv0367cab_Open_test(u32 IOHandle);

extern s32 PowOf2(s32 number);

#endif  // _STV0367_CAB_H
// vim:ts=4
