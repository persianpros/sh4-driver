/*****************************************************************************
 * Copyright (C)2011 FULAN Corporation. All Rights Reserved.
 *
 * File: tun_mxl301.h
 *
 * Description: MxL301 Tuner.
 * History:
 *    Date         Author        Version   Reason
 *    ============ ============= ========= =================
 * 1. 2011/08/26   dmq           Ver 0.1   Create file.
 *
 ****************************************************************************/

#ifndef __TUN_MXL301_H__
#define __TUN_MXL301_H__

#include <types.h>
#include "nim_tuner.h"

#ifdef __cplusplus
extern "C"
{
#endif

int tun_mxl301_i2c_write(u32 tuner_id, u8 *pArray, u32 count);
int tun_mxl301_i2c_read(u32 tuner_id, u8 Addr, u8 *mData);

int tun_mxl301_init(u32 *tuner_idx, struct COFDM_TUNER_CONFIG_EXT *ptrTuner_Config);
int tun_mxl301_control(u32 tuner_idx, u32 freq, u8 bandwidth, u8 AGC_Time_Const, u8 *data, u8 _i2c_cmd);
int tun_mxl301_status(u32 tuner_idx, u8 *lock);

#ifdef __cplusplus
}
#endif

#endif // __TUN_MXL301_H__
// vim:ts=4
