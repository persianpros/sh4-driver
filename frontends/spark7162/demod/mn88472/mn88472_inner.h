/****************************************************************************
 *
 * Panasonic MN88472 DVB-T(2)/C demodulator driver
 *
 * Version for Fulan Spark7162
 *
 * Supports Panasonic MN88472 with MaxLinear MxL301 or
 * MaxLinear MxL603 tuner
 *
 * Original code:
 * Copyright (C), 2013-2018, AV Frontier Tech. Co., Ltd.
 *
 * Date: 2013.12.11
 *
 */
#ifndef __MN88472_INNER_H
#define __MN88472_INNER_H

#include <types.h>
#if !defined(MODULE)
#include <stdio.h>
#endif

typedef u32 OSAL_ID;

#include "nim_mn88472.h"

#ifdef __cplusplus
extern "C" {
#endif

#define QAM0                  1
#define QAM4                  2
#define QAM8                  3
#define QAM16                 4
#define QAM32                 5
#define QAM64                 6
#define QAM128                7
#define QAM256                8
#define PANASONIC_DEMODULATOR 1

#define OSAL_INVALID_ID 0

#define HLD_DEV_TYPE_NIM 0x01050000  /* NIM (Demodulator + tuner) */
#define HLD_DEV_STATS_UP 0x01000001  /* Device is up */
#define HLD_DEV_STATS_RUNNING 0x01000002  /* Device is running */

#define dev_alloc(s, Type, Size) YWOS_Malloc(Size)
#define dev_free(p) YWOS_Free(p)
#define dev_register(dev) SUCCESS
//#define MALLOC(Size) YWOS_Malloc(Size)
//#define FREE(p) YWOS_Free(p)
//#define MEMCPY memcpy
//#define MEMSET memset
#define osal_flag_delete(flag_id)

void osal_task_sleep(u32 ms);
OSAL_ID osal_mutex_create(void);
#define osal_mutex_lock(osalid, time)
#define osal_mutex_unlock(osalid)
#define osal_mutex_delete(osalid)

#ifdef __cplusplus
}
#endif

#endif  // __MN88472_INNER_H
// vim:ts=4
