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
 @File   stv6110_tuner.h
 @brief



*/
#ifndef H_STV6110_TUNER
	#define H_STV6110_TUNER

	#include "chip.h"
	#include "fe_sat_tuner.h"

	#if defined(CHIP_STAPI) /*For STAPI use only*/
		#include "dbtypes.h"	/*for Install function*/
		#include "stfrontend.h"    /*for Install Function*/
	#endif

	#define	STV6110_NBREGS 8 /*Number fo registers in STV6110*/
	#define STV6110_NBFIELDS 22 /*Number of fields in STV6110*/


	/* functions --------------------------------------------------------------- */

	/* create instance of tuner register mappings */

	TUNER_Error_t STV6110_TunerInit(void *pTunerInit,STCHIP_Handle_t *TunerHandle);
	TUNER_Error_t STV6110_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn);

	TUNER_Error_t STV6110_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency);
	U32 STV6110_TunerGetFrequency(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6110_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth);
	U32 STV6110_TunerGetBandwidth(STCHIP_Handle_t hTuner);
	BOOL STV6110_TunerGetStatus(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6110_TunerWrite(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6110_TunerRead(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6110_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain);
	S32 STV6110_TunerGetGain(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6110_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider);

	TUNER_Error_t STV6110_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn);
	BOOL STV6110_TunerGetAttenuator(STCHIP_Handle_t hTuner);


	TUNER_Model_t	STV6110_TunerGetModel(STCHIP_Handle_t hTuner);
	void			STV6110_TunerSetIQ_Wiring(STCHIP_Handle_t hTuner, S8 IQ_Wiring);
	S8		STV6110_TunerGetIQ_Wiring(STCHIP_Handle_t hTuner);
	void			STV6110_TunerSetReferenceFreq(STCHIP_Handle_t hTuner, U32 Reference);
	U32				STV6110_TunerGetReferenceFreq(STCHIP_Handle_t hTuner);
	void			STV6110_TunerSetIF_Freq(STCHIP_Handle_t hTuner, U32 IF);
	U32				STV6110_TunerGetIF_Freq(STCHIP_Handle_t hTuner);
	void STV6110_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect);
	U8 STV6110_TunerGetBandSelect(STCHIP_Handle_t hChip);
	S32 STV6110_TunerGetRFGain(STCHIP_Handle_t hTuner,U32 AGCIntegrator,S32 BBGain); 


	TUNER_Error_t STV6110_TunerTerm(STCHIP_Handle_t hTuner);

	#if defined(CHIP_STAPI) /*For STAPI use only*/
		ST_ErrorCode_t STFRONTEND_TUNER_STV6110_Install(STFRONTEND_tunerDbase_t *Tuner, STFRONTEND_TunerType_t TunerType);
		ST_ErrorCode_t STFRONTEND_TUNER_STV6110_Uninstall(STFRONTEND_tunerDbase_t *Tuner);
	#endif

#endif
