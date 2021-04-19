/************************************************************
 *
 *  Copyright (C), 2012-2017, AV Frontier Tech. Co., Ltd.
 *
 *  Date: 2012.09.07
 *
 ************************************************************/
#ifndef __YWDEFS_H
#define __YWDEFS_H

#include <linux/delay.h>
#include <linux/slab.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DEFINED_BOOL
#define DEFINED_BOOL
typedef int BOOL;
/* BOOL type constant values */
#ifndef TRUE
#define TRUE (1 == 1)
#endif
#ifndef FALSE
#define FALSE (!TRUE)
#endif
#endif

#ifndef NULL
#define NULL 0
#endif

#define YWTUNERi_MAX_TUNER_NUM            4

#define YW_NO_ERROR                       0
#define YWHAL_ERROR_NO_MEMORY             -1
#define YWHAL_ERROR_INVALID_HANDLE        -2
#define YWHAL_ERROR_BAD_PARAMETER         -3
#define YWHAL_ERROR_FEATURE_NOT_SUPPORTED -4
#define YWHAL_ERROR_UNKNOWN_DEVICE        -5
#define YWHAL_ERROR_NO_DEV                -8  /* Device not exist on PCI */
#define YWHAL_ERROR_ID_FULL               -21  /* No more ID available */
#define ERR_FAILURE                       -9

#define SUCCESS                           0  /* Success return */
#define RET_SUCCESS                       ((INT32)0)

#ifdef __cplusplus
}
#endif

#endif  // __YWDEFS_H
// vim:ts=4
