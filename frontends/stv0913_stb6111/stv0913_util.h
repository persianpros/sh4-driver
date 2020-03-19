/*****************************************************************************/
/* COPYRIGHT (C) 2009 STMicroelectronics - All Rights Reserved               */
/* ST makes no warranty express or implied including but not limited to,     */
/* any warranty of                                                           */
/*                                                                           */
/*   (i)  merchantability or fitness for a particular purpose and/or         */
/*   (ii) requirements, for a particular purpose in relation to the LICENSED */
/*        MATERIALS, which is provided 'AS IS', WITH ALL FAULTS. ST does not */
/*        represent or warrant that the LICENSED MATERIALS provided here     */
/*        under is free of infringement of any third party patents,          */
/*        copyrights,trade secrets or other intellectual property rights.    */
/*        ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE       */
/*        EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                    */
/*                                                                           */
/*****************************************************************************/
/**
@File   STV0900_util.h
@brief
*/
#ifndef H_STV0913UTIL
#define H_STV0913UTIL
#include "stv0913_drv.h"
#include "stv0913_init.h"
#include "fe_sat_tuner.h" 

/* Counter enum */
typedef enum
{
	FE_STV0913_COUNTER1 = 0,
	FE_STV0913_COUNTER2 = 1
} FE_STV0913_ERRORCOUNTER;

/*PER/BER enum*/
typedef enum
{
	FE_STV0913_BER = 0,
	FE_STV0913_BER_BIT7=1,
	FE_STV0913_PER = 2,
	FE_STV0913_BYTEER = 3
} FE_STV0913_BerPer;

/*CNR estimation enum*/
typedef enum
{
	FE_STV0913_UNNORMDATAM,
	FE_STV0913_UNNORMDATAMSQ,
	FE_STV0913_NORMDATAM,
	FE_STV0913_NORMDATAMSQ,
	FE_STV0913_UNNORMPLHM,
	FE_STV0913_UNNORMPLHMSQ,
	FE_STV0913_NORMPLHM,
	FE_STV0913_NORMPLHMSQ
} FE_STV0913_CnrRegUsed;

/****************************************************************
 Util FUNCTIONS
****************************************************************/
#ifdef CHIP_STAPI
STCHIP_Error_t FE_STV0913_RepeaterFn(STCHIP_Handle_t hChip,BOOL State, U8 *Buffer);
#else
STCHIP_Error_t FE_STV0913_RepeaterFn(STCHIP_Handle_t hChip,BOOL State);
#endif
U32 FE_STV0913_GetMclkFreq(STCHIP_Handle_t hChip, U32 ExtClk);

void FE_STV0913_SetSymbolRate(STCHIP_Handle_t hChip,U32 MasterClock,U32 SymbolRate);
BOOL FE_STV0913_WaitForLock(STCHIP_Handle_t hChip,S32 DemodTimeOut,S32 FecTimeOut) ;

U32 FE_STV0913_CarrierWidth(U32 SymbolRate, FE_STV0913_RollOff_t RollOff);

S32 FE_STV0913_GetCarrierFrequency(STCHIP_Handle_t hChip,U32 MasterClock );
U32 FE_STV0913_GetSymbolRate(STCHIP_Handle_t hChip,U32 MasterClock);
S32 FE_STV0913_CarrierGetQuality(STCHIP_Handle_t hChip,FE_STV0913_LOOKUP_t *lookup);
U32 FE_STV0913_GetErrorCount(STCHIP_Handle_t hChip,FE_STV0913_ERRORCOUNTER Counter);
#endif
// vim:ts=4
