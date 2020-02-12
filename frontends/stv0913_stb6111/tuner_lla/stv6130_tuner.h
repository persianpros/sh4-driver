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
 @File   stv6130_tuner.h
 @brief



*/
#ifndef H_STV6130_TUNER
	#define H_STV6130_TUNER

	#include "chip.h"
	#include "fe_sat_tuner.h"

	#if defined(CHIP_STAPI) 	/*For STAPI use only*/
		#include "dbtypes.h"	/*for Install function*/
		#include "stfrontend.h"    /*for Install Function*/
	#endif

	#define	STV6130_NBREGS 14
	#define STV6130_NBFIELDS 31

	/* functions --------------------------------------------------------------- */

	/* create instance of tuner register mappings */

	TUNER_Error_t STV6130_TunerInit(void *pTunerInit,STCHIP_Handle_t *TunerHandle);
	TUNER_Error_t STV6130_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn);

	TUNER_Error_t STV6130_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency);
	U32 STV6130_TunerGetFrequency(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6130_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth);
	U32 STV6130_TunerGetBandwidth(STCHIP_Handle_t hTuner);
	BOOL STV6130_TunerGetStatus(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6130_TunerWrite(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6130_TunerRead(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6130_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain);
	S32 STV6130_TunerGetGain(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6130_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider);

	TUNER_Error_t STV6130_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn);
	BOOL STV6130_TunerGetAttenuator(STCHIP_Handle_t hTuner);

	TUNER_Model_t	STV6130_TunerGetModel(STCHIP_Handle_t hTuner);
	void			STV6130_TunerSetIQ_Wiring(STCHIP_Handle_t hTuner, S8 IQ_Wiring);
	S8		STV6130_TunerGetIQ_Wiring(STCHIP_Handle_t hTuner);
	void			STV6130_TunerSetReferenceFreq(STCHIP_Handle_t hTuner, U32 Reference);
	U32				STV6130_TunerGetReferenceFreq(STCHIP_Handle_t hTuner);
	void			STV6130_TunerSetIF_Freq(STCHIP_Handle_t hTuner, U32 IF);
	U32				STV6130_TunerGetIF_Freq(STCHIP_Handle_t hTuner);
	void STV6130_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect);
	U8 STV6130_TunerGetBandSelect(STCHIP_Handle_t hChip);
	TUNER_Error_t STV6130_TunerTerm(STCHIP_Handle_t hTuner);

	#if defined(CHIP_STAPI) /*For STAPI use only*/
		ST_ErrorCode_t STFRONTEND_TUNER_STV6130_Install(STFRONTEND_tunerDbase_t *Tuner, STFRONTEND_TunerType_t TunerType);
		ST_ErrorCode_t STFRONTEND_TUNER_STV6130_Uninstall(STFRONTEND_tunerDbase_t *Tuner);
	#endif

#endif
