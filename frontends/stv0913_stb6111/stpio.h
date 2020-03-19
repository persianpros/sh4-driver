/*****************************************************************************/
/* COPYRIGHT (C) 2009 STMicroelectronics - All Rights Reserved               */
/* ST makes no warranty express or implied including but not limited to,     */
/* any warranty of                                                           */
/*                                                                           */
/*   (i)  merchantability or fitness for a particular purpose and/or         */
/*   (ii) requirements, for a particular purpose in relation to the LICENSED */
/*        MATERIALS, which is provided “AS IS”, WITH ALL FAULTS. ST does not */
/*        represent or warrant that the LICENSED MATERIALS provided here     */
/*        under is free of infringement of any third party patents,          */
/*        copyrights,trade secrets or other intellectual property rights.    */
/*        ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE       */
/*        EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                    */
/*                                                                           */
/*****************************************************************************/
/**
 @File   stpio.h
 @brief



*/
#ifndef __STPIO_H
#define __STPIO_H

/* Includes ---------------------------------------------------------------- */
//#include "stddefs.h"
#if defined(ST_OS20) || defined(ST_OS21)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define STPIO_DRIVER_ID    68
#define STPIO_DRIVER_BASE  (STPIO_DRIVER_ID << 16)

/* Bit mask type and possible values */
typedef U8 STPIO_BitMask_t;

#define PIO_BIT_0    1
#define PIO_BIT_1    2
#define PIO_BIT_2    4
#define PIO_BIT_3    8
#define PIO_BIT_4    16
#define PIO_BIT_5    32
#define PIO_BIT_6    64
#define PIO_BIT_7    128

#define PIO_MAPPED_LENGTH 4096
/* API structures which contain input parameter data */

typedef U32 STPIO_Handle_t;

/************************************************
STPIO_BitConfig_t - type to define
the functionality of bits in a PIO port.
************************************************/
typedef enum STPIO_BitConfig_e
{
    STPIO_BIT_NOT_SPECIFIED = 0,
    STPIO_BIT_BIDIRECTIONAL = 1,
    STPIO_BIT_OUTPUT = 2,
    STPIO_BIT_INPUT = 4,
    STPIO_BIT_ALTERNATE_OUTPUT = 6,
    STPIO_BIT_ALTERNATE_BIDIRECTIONAL = 7,
    STPIO_BIT_BIDIRECTIONAL_HIGH = 9,    /*4th bit remains HIGH*/
    STPIO_BIT_OUTPUT_HIGH = 10           /*4th bit remains HIGH*/
} STPIO_BitConfig_t;

/************************************************
STPIO_CompareClear_t - define the behaviour of
the driver when a compare pattern interrupt is
fired.
************************************************/
typedef enum STPIO_CompareClear_e
{
    STPIO_COMPARE_CLEAR_AUTO,
    STPIO_COMPARE_CLEAR_MANUAL
} STPIO_CompareClear_t;

/************************************************
STPIO_Compare_t - Structure relating
to the PIO compare registers.
************************************************/
typedef struct STPIO_Compare_s
{
    STPIO_BitMask_t    CompareEnable;
    STPIO_BitMask_t    ComparePattern;
} STPIO_Compare_t;

/************************************************
STPIO_PIOBit_t - Structure defining a PIO port by
name and one or more bits within that port.
************************************************/
typedef struct STPIO_PIOBit_s
{
    ST_DeviceName_t    PortName;
    STPIO_BitMask_t    BitMask;
} STPIO_PIOBit_t;

/************************************************
STPIO_InitParams_t - Structure containing fixed
specific information for initializing the PIO.
************************************************/
typedef struct STPIO_InitParams_s
{
    U32                 *BaseAddress;
#ifdef ST_OS21
    interrupt_name_t    InterruptNumber;
#else
    U32                 InterruptNumber;
#endif
    U32                 InterruptLevel;
    ST_Partition_t      *DriverPartition;
} STPIO_InitParams_t;

/************************************************
STPIO_OpenParams_t - Structure containing specific
information for opening the PIO device driver.
************************************************/
typedef struct STPIO_OpenParams_s
{
    STPIO_BitMask_t         ReservedBits;
    STPIO_BitConfig_t       BitConfigure[8];
    void (* IntHandler)(STPIO_Handle_t Handle, STPIO_BitMask_t ActiveBits);
} STPIO_OpenParams_t;

/************************************************
STPIO_Status_t - Current driver status.
************************************************/
typedef struct STPIO_Status_s
{
    STPIO_BitMask_t         BitMask;
    STPIO_BitConfig_t       BitConfigure[8];
} STPIO_Status_t;

/************************************************
STPIO_TermParams_t - Termination parameters.
************************************************/
typedef struct STPIO_TermParams_s
{
    BOOL ForceTerminate;
} STPIO_TermParams_t;

/* Function Prototypes */
/* Init */
ST_ErrorCode_t STPIO_Init (const ST_DeviceName_t DeviceName,
                           STPIO_InitParams_t *InitParams);
/* Init with cold reset */
ST_ErrorCode_t STPIO_InitNoReset (const ST_DeviceName_t DeviceName,
                                  STPIO_InitParams_t *InitParams);
/* Open */
ST_ErrorCode_t STPIO_Open (const ST_DeviceName_t DeviceName,
                           STPIO_OpenParams_t *OpenParams,
                           STPIO_Handle_t *Handle);
/* Close */
ST_ErrorCode_t STPIO_Close (STPIO_Handle_t Handle);
/* Write */
ST_ErrorCode_t STPIO_Write (STPIO_Handle_t Handle, U8 Buffer);
/* Read */
ST_ErrorCode_t STPIO_Read (STPIO_Handle_t Handle, U8 *Buffer);
/* Set */
ST_ErrorCode_t STPIO_Set (STPIO_Handle_t Handle, STPIO_BitMask_t BitMask);
/* Clear */
ST_ErrorCode_t STPIO_Clear (STPIO_Handle_t Handle, STPIO_BitMask_t BitMask);
/* GetCompare */
ST_ErrorCode_t STPIO_GetCompare (STPIO_Handle_t Handle,
                                 STPIO_Compare_t *CompareStatus);
/* GetCompareClear */
ST_ErrorCode_t STPIO_GetCompareClear (STPIO_Handle_t Handle,
                                      STPIO_CompareClear_t *CompareClear_p);
/* SetCompare */
ST_ErrorCode_t STPIO_SetCompare (STPIO_Handle_t Handle,
                                 STPIO_Compare_t *CompareStatus);
/* SetCompareClear */
ST_ErrorCode_t STPIO_SetCompareClear(STPIO_Handle_t Handle,
                                     STPIO_CompareClear_t CompareClear);
/* SetConfig */
ST_ErrorCode_t STPIO_SetConfig (STPIO_Handle_t Handle,
                                const STPIO_BitConfig_t BitConfigure[8]);

#ifndef ST_OSLINUX
/* GetStatus */
ST_ErrorCode_t STPIO_GetStatus (STPIO_Handle_t Handle,
                                STPIO_Status_t *Status);

#endif
/* GetBaseAddress */
ST_ErrorCode_t STPIO_GetBaseAddress (ST_DeviceName_t DeviceName,
                                     U32             **BaseAddress);

/* GetRevision */
ST_Revision_t STPIO_GetRevision (void);

/* Term */
ST_ErrorCode_t STPIO_Term (const ST_DeviceName_t DeviceName,
                           STPIO_TermParams_t *TermParams);

#ifdef __cplusplus
}
#endif

#endif /* _STPIO_H */
