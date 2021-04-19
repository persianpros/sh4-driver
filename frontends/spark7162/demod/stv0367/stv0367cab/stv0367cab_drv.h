/*****************************************************************************/
/* COPYRIGHT (C) 2009 STMicroelectronics - All Rights Reserved               */
/* ST makes no warranty express or implied including but not limited to,     */
/* any warranty of                                                           */
/*                                                                           */
/*   (i)  merchantability or fitness for a particular purpose and/or         */
/*   (ii) requirements, for a particular purpose in relation to the LICENSED */
/*        MATERIALS, which is provided "AS IS", WITH ALL FAULTS. ST does not */
/*        represent or warrant that the LICENSED MATERIALS provided here     */
/*        under is free of infringement of any third party patents,          */
/*        copyrights,trade secrets or other intellectual property rights.    */
/*        ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE       */
/*        EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                    */
/*                                                                           */
/*****************************************************************************/
/**
 @File   stv0367cab_drv.h
 @brief

*/
#ifndef _STV0367CAB_DRV_H
#define _STV0367CAB_DRV_H

typedef enum
{
	FE_LLA_NO_ERROR,
	FE_LLA_INVALID_HANDLE,
	FE_LLA_ALLOCATION,
	FE_LLA_BAD_PARAMETER,
	FE_LLA_ALREADY_INITIALIZED,
	FE_LLA_I2C_ERROR,
	FE_LLA_SEARCH_FAILED,
	FE_LLA_TRACKING_FAILED,
	FE_LLA_TERM_FAILED

} FE_LLA_Error_t;  /* Error Type */  // lwj add

typedef enum
{
	FE_CAB_MOD_QAM4,
	FE_CAB_MOD_QAM16,
	FE_CAB_MOD_QAM32,
	FE_CAB_MOD_QAM64,
	FE_CAB_MOD_QAM128,
	FE_CAB_MOD_QAM256,
	FE_CAB_MOD_QAM512,
	FE_CAB_MOD_QAM1024
} FE_CAB_Modulation_t;

/* Monitoring structure */
typedef struct
{
	u32 FE_367cab_TotalBlocks,
	    FE_367cab_TotalBlocksOld,
	    FE_367cab_TotalBlocksOffset,
	    FE_367cab_TotalCB,
	    FE_367cab_TotalCBOld,
	    FE_367cab_TotalCBOffset,
	    FE_367cab_TotalUCB,
	    FE_367cab_TotalUCBOld,
	    FE_367cab_TotalUCBOffset,
	    FE_367cab_BER_Reg,
	    FE_367cab_BER_U32,
	    FE_367cab_Saturation,
	    FE_367cab_WaitingTime;
} FE_367cab_Monitor;

typedef enum
{
	FE_367cab_NOTUNER,
	FE_367cab_NOAGC,
	FE_367cab_NOSIGNAL,
	FE_367cab_NOTIMING,
	FE_367cab_TIMINGOK,
	FE_367cab_NOCARRIER,
	FE_367cab_CARRIEROK,
	FE_367cab_NOBLIND,
	FE_367cab_BLINDOK,
	FE_367cab_NODEMOD,
	FE_367cab_DEMODOK,
	FE_367cab_DATAOK
} FE_367cab_SIGNALTYPE_t;

typedef enum
{
	FE_CAB_SPECT_NORMAL,
	FE_CAB_SPECT_INVERTED
} FE_CAB_SpectInv_t;

typedef struct
{
	u32 TotalBlocks;
	u32 TotalBlocksOffset;
	u32 TotalCB;
	u32 TotalCBOffset;
	u32 TotalUCB;
	u32 TotalUCBOffset;
} FE_CAB_PacketCounter_t;

typedef struct
{
	BOOL Locked;                        /* channel found              */
	u32  Frequency_kHz;                 /* found frequency (in kHz)   */
	u32  SymbolRate_Bds;                /* found symbol rate (in Bds) */
	FE_CAB_Modulation_t Modulation;     /* modulation                 */
	FE_CAB_SpectInv_t SpectInv;         /* Spectrum Inversion         */
	u8   Interleaver;                   /* Interleaver in FEC B mode  */
	FE_CAB_PacketCounter_t PacketCounter;
} FE_CAB_SearchResult_t;

/****************************************************************
 SEARCH STRUCTURES
 ****************************************************************/
typedef struct
{
	FE_367cab_SIGNALTYPE_t State;

	s32 Crystal_Hz,                     /* Crystal frequency (Hz)                       */
	    IF_Freq_kHz,
	    Frequency_kHz,                  /* Current tuner frequency (KHz)                */
	    SymbolRate_Bds,                 /* Symbol rate (Bds)                            */
	    MasterClock_Hz,                 /* Master clock frequency (Hz)                  */
	    AdcClock_Hz,                    /* ADC clock frequency (Hz)                     */
	    SearchRange_Hz,                 /* Search range (Hz)                            */
	    DerotOffset_Hz;                 /* Derotator offset during software zigzag (Hz) */
	u32 FirstTimeBER;
	FE_CAB_Modulation_t Modulation;     /* QAM Size                                     */
	FE_367cab_Monitor Monitor_results;  /* Monitoring counters                          */
	FE_CAB_SearchResult_t DemodResult;  /* Search results                               */
} FE_367cab_InternalParams_t;

int FE_STV0367QAM_GetSignalInfo(u8 Handle, u32 *CN_dBx10, u32 *Power_dBmx10, u32 *Ber, u32 *FirstTimeBER);
BOOL FE_367cab_Status(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle);
u32 FE_367cab_SetDerotFreq(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u32 AdcClk_Hz, s32 DerotFreq_Hz);
u32 FE_367cab_GetADCFreq(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u32 ExtClk_Hz);
u32 FE_367cab_GetMclkFreq(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u32 ExtClk_Hz);
FE_CAB_Modulation_t FE_367cab_SetQamSize(TUNER_ScanTaskParam_T *Inst, TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u32 SearchFreq_kHz, u32 SymbolRate, FE_CAB_Modulation_t QAMSize);
u32 FE_367cab_SetSymbolRate(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u32 AdcClk_Hz, u32 MasterClk_Hz, u32 SymbolRate, FE_CAB_Modulation_t QAMSize);
u32 FE_367cab_GetDerotFreq(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u32 AdcClk_Hz);
u32 FE_367cab_GetSymbolRate(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u32 MasterClk_Hz);

s32 FE_STV0367cab_GetSnr(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle);
s32 FE_STV0367cab_GetPower(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle);
s32 FE_STV0367cab_GetErrors(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle);
int FE_STV0367cab_GetSignalInfo(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u32 *CN_dBx10, u32 *Power_dBmx10, u32 *Ber, u32 *FirstTimeBER);
FE_CAB_Modulation_t stv0367cab_SetQamSize(TUNER_TunerType_T TunerType, TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u32 SearchFreq_kHz,
									   u32 SymbolRate, FE_CAB_Modulation_t QAMSize);

extern s32 PowOf2(s32 number);

#endif // _STV0367CAB_DRV_H
// vim:ts=4
