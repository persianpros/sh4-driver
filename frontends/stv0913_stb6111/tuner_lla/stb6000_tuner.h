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
 @File   stb6000_tuner.h
 @brief



*/
#ifndef H_STB6000_TUNER
	#define H_STB6000_TUNER

	#include "chip.h"
	#include "fe_sat_tuner.h"


	#if defined(CHIP_STAPI) /*For STAPI use only*/
		#include "dbtypes.h"	/*for Install function*/
		#include "stfrontend.h"    /*for Install Function*/
	#endif

	#define STB6000_NBREGS	12
	#define STB6000_NBFIELDS	21

	/* functions --------------------------------------------------------------- */

	/* create instance of tuner register mappings */

	TUNER_Error_t STB6000_TunerInit(void *pTunerInit,STCHIP_Handle_t *TunerHandle);
	TUNER_Error_t STB6000_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn);

	TUNER_Error_t STB6000_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency);
	U32 STB6000_TunerGetFrequency(STCHIP_Handle_t hTuner);
	TUNER_Error_t STB6000_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth);
	U32 STB6000_TunerGetBandwidth(STCHIP_Handle_t hTuner);
	BOOL STB6000_TunerGetStatus(STCHIP_Handle_t hTuner);
	TUNER_Error_t STB6000_TunerWrite(STCHIP_Handle_t hTuner);
	TUNER_Error_t STB6000_TunerRead(STCHIP_Handle_t hTuner);
	TUNER_Error_t STB6000_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain);
	S32           STB6000_TunerGetGain(STCHIP_Handle_t hTuner);
	TUNER_Error_t STB6000_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider);

	TUNER_Error_t STB6000_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn);
	BOOL STB6000_TunerGetAttenuator(STCHIP_Handle_t hTuner);


	TUNER_Model_t	STB6000_TunerGetModel(STCHIP_Handle_t hTuner);
	void			STB6000_TunerSetIQ_Wiring(STCHIP_Handle_t hTuner, S8 IQ_Wiring);
	S8		STB6000_TunerGetIQ_Wiring(STCHIP_Handle_t hTuner);
	void			STB6000_TunerSetReferenceFreq(STCHIP_Handle_t hTuner, U32 Reference);
	U32				STB6000_TunerGetReferenceFreq(STCHIP_Handle_t hTuner);
	void			STB6000_TunerSetIF_Freq(STCHIP_Handle_t hTuner, U32 IF);
	U32				STB6000_TunerGetIF_Freq(STCHIP_Handle_t hTuner);
	void STB6000_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect);
	U8 STB6000_TunerGetBandSelect(STCHIP_Handle_t hChip);

	TUNER_Error_t STB6000_TunerTerm(STCHIP_Handle_t hTuner);


	#if defined(CHIP_STAPI) /*For STAPI use only*/
		ST_ErrorCode_t STFRONTEND_TUNER_STB6000_Install(STFRONTEND_tunerDbase_t *Tuner, STFRONTEND_TunerType_t TunerType);
		ST_ErrorCode_t STFRONTEND_TUNER_STB6000_Uninstall(STFRONTEND_tunerDbase_t *Tuner);
	#endif

#endif
