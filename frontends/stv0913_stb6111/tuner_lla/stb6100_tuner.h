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
 @File   stb6100_tuner.h
 @brief



*/
#ifndef H_STB6100_TUNER
	#define H_STB6100_TUNER

	#include "chip.h"
	#include "fe_sat_tuner.h"

	#if defined(CHIP_STAPI) /*For STAPI use only*/
		#include "dbtypes.h"	/*for Install function*/
		#include "stfrontend.h"    /*for Install Function*/
	#endif

	#define STB6100_NBREGS	12 /*Number of Registers in STB6100*/
	#define STB6100_NBFIELDS	21

	/* functions --------------------------------------------------------------- */

	/* create instance of tuner register mappings */

	TUNER_Error_t STB6100_TunerInit(void *pTunerInit,STCHIP_Handle_t *TunerHandle);
	TUNER_Error_t STB6100_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn);

	TUNER_Error_t STB6100_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency);
	U32 STB6100_TunerGetFrequency(STCHIP_Handle_t hTuner);
	TUNER_Error_t STB6100_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth);
	U32 STB6100_TunerGetBandwidth(STCHIP_Handle_t hTuner);
	BOOL STB6100_TunerGetStatus(STCHIP_Handle_t hTuner);
	TUNER_Error_t STB6100_TunerWrite(STCHIP_Handle_t hTuner);
	TUNER_Error_t STB6100_TunerRead(STCHIP_Handle_t hTuner);
	TUNER_Error_t STB6100_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain);
	S32 STB6100_TunerGetGain(STCHIP_Handle_t hTuner);
	TUNER_Error_t STB6100_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider);

	TUNER_Error_t STB6100_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn);
	BOOL STB6100_TunerGetAttenuator(STCHIP_Handle_t hTuner);


	TUNER_Model_t	STB6100_TunerGetModel(STCHIP_Handle_t hTuner);
	void			STB6100_TunerSetIQ_Wiring(STCHIP_Handle_t hTuner, S8 IQ_Wiring);
	S8		STB6100_TunerGetIQ_Wiring(STCHIP_Handle_t hTuner);
	void			STB6100_TunerSetReferenceFreq(STCHIP_Handle_t hTuner, U32 Reference);
	U32				STB6100_TunerGetReferenceFreq(STCHIP_Handle_t hTuner);
	void			STB6100_TunerSetIF_Freq(STCHIP_Handle_t hTuner, U32 IF);
	U32				STB6100_TunerGetIF_Freq(STCHIP_Handle_t hTuner);
	void STB6100_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect);
	U8 STB6100_TunerGetBandSelect(STCHIP_Handle_t hChip);

	TUNER_Error_t STB6100_TunerTerm(STCHIP_Handle_t hTuner);

	#if defined(CHIP_STAPI) /*For STAPI use only*/
		ST_ErrorCode_t STFRONTEND_TUNER_STB6100_Install(STFRONTEND_tunerDbase_t *Tuner, STFRONTEND_TunerType_t TunerType);
		ST_ErrorCode_t STFRONTEND_TUNER_STB6100_Uninstall(STFRONTEND_tunerDbase_t *Tuner);
	#endif

#endif
