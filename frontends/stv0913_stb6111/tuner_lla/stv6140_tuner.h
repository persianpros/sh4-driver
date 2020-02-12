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
 @File   stv6140_tuner.h
 @brief



*/
#ifndef H_STV6140_TUNER
	#define H_STV6140_TUNER

	#include "chip.h"
	#include "fe_sat_tuner.h"

	#if defined(CHIP_STAPI) /*For STAPI use only*/
		#include "dbtypes.h"	/*for Install function*/
		#include "stfrontend.h"    /*for Install Function*/
	#endif

	#define	STV6140_NBREGS 16 /*Number fo registers in STV6140*/
	#define STV6140_NBFIELDS 61 /*Number of fields in STV6140*/


	/* functions --------------------------------------------------------------- */

	/* create instance of tuner register mappings */

	TUNER_Error_t STV6140_TunerInit(void *pTunerInit,STCHIP_Handle_t *TunerHandle);
	TUNER_Error_t STV6140_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn);

	TUNER_Error_t STV6140_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency);
	U32 STV6140_TunerGetFrequency(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6140_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth);
	U32 STV6140_TunerGetBandwidth(STCHIP_Handle_t hTuner);
	BOOL STV6140_TunerGetStatus(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6140_TunerWrite(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6140_TunerRead(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6140_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain);
	S32 STV6140_TunerGetGain(STCHIP_Handle_t hTuner);
	TUNER_Error_t STV6140_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider);

	TUNER_Error_t STV6140_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn);
	BOOL STV6140_TunerGetAttenuator(STCHIP_Handle_t hTuner);


	TUNER_Model_t	STV6140_TunerGetModel(STCHIP_Handle_t hTuner);
	void			STV6140_TunerSetIQ_Wiring(STCHIP_Handle_t hTuner, S8 IQ_Wiring);
	S8		STV6140_TunerGetIQ_Wiring(STCHIP_Handle_t hTuner);
	void			STV6140_TunerSetReferenceFreq(STCHIP_Handle_t hTuner, U32 Reference);
	U32				STV6140_TunerGetReferenceFreq(STCHIP_Handle_t hTuner);
	void			STV6140_TunerSetIF_Freq(STCHIP_Handle_t hTuner, U32 IF);
	U32				STV6140_TunerGetIF_Freq(STCHIP_Handle_t hTuner);
	void STV6140_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect);
	U8 STV6140_TunerGetBandSelect(STCHIP_Handle_t hChip);

	TUNER_Error_t STV6140_TunerRFSelect(STCHIP_Handle_t htuner );


	TUNER_Error_t STV6140_TunerTerm(STCHIP_Handle_t hTuner);

	#if defined(CHIP_STAPI) /*For STAPI use only*/
		ST_ErrorCode_t STFRONTEND_TUNER_STV6140_Install(STFRONTEND_tunerDbase_t *Tuner, STFRONTEND_TunerType_t TunerType);
		ST_ErrorCode_t STFRONTEND_TUNER_STV6140_Uninstall(STFRONTEND_tunerDbase_t *Tuner);
	#endif

#endif
