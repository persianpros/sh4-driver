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
 @File   sat_null_tuner.c
 @brief



*/
#ifndef ST_OSLINUX
	#include "string.h"
	#include "stdlib.h"
#endif

#include "globaldefs.h"
#include "fe_sat_tuner.h"
#include "sat_null_tuner.h"


TUNER_Error_t SAT_NULL_TunerInit(void *pTunerInit_v,STCHIP_Handle_t *TunerHandle)
{
	SAT_TUNER_Params_t	hTunerParams = NULL;
	STCHIP_Info_t	ChipInfo;
	STCHIP_Handle_t hTuner = NULL;
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t pTunerInit = (SAT_TUNER_Params_t)pTunerInit_v; /*Changed to retain common function prototype for all Tuners: Change ///AG*/

    #ifdef HOST_PC
    hTunerParams = calloc (1,sizeof(SAT_TUNER_InitParams_t));	/* Allocation of the chip structure	*/
    #endif

    #ifdef CHIP_STAPI
	if(*TunerHandle)
	{
		hTunerParams = (SAT_TUNER_Params_t)((*TunerHandle)->pData);/*Change ///AG*/
		memcpy(&ChipInfo, *TunerHandle,sizeof(STCHIP_Info_t)); /*Copy settings already contained in hTuner to ChipInfo: Change ///AG*/
	}
	#endif


    if(hTunerParams != NULL)
    {
    	hTunerParams->Model		= pTunerInit->Model;    	/* Tuner model */
    	hTunerParams->Reference = pTunerInit->Reference;	/* Reference Clock in Hz */
    	hTunerParams->IF = pTunerInit->IF; 					/* IF Hz intermediate frequency */
    	hTunerParams->IQ_Wiring = pTunerInit->IQ_Wiring;	/* hardware IQ invertion */
    	hTunerParams->BandSelect = pTunerInit->BandSelect;	/* Wide band tuner (6130 like, band selection) */
    	hTunerParams->DemodModel = pTunerInit->DemodModel;
		/*fix Coverity CID 15039*/		
		if(strlen((char*)pTunerInit->TunerName)<MAXNAMESIZE)
		{
			strcpy((char *)ChipInfo.Name,(char*)pTunerInit->TunerName);	/* Tuner name */
		}
		else
		{
			error=TUNER_TYPE_ERR;
		}
    	ChipInfo.RepeaterHost = pTunerInit->RepeaterHost;		/* Repeater host */
		ChipInfo.RepeaterFn = pTunerInit->RepeaterFn;			/* Repeater enable/disable function */
		ChipInfo.Repeater = TRUE;								/* Tuner need to enable repeater */
		ChipInfo.I2cAddr = pTunerInit->TunerI2cAddress;			/* Init tuner I2C address */


    	ChipInfo.pData = hTunerParams;				/* Store tunerparams pointer into Chip structure */


		/* fill elements of external chip data structure */
		ChipInfo.NbRegs   = SAT_NULL_NBREGS;
		ChipInfo.NbFields = SAT_NULL_NBFIELDS;
		ChipInfo.ChipMode = STCHIP_MODE_NOSUBADR_RD;
		ChipInfo.WrStart  = 0;
		ChipInfo.WrSize   = 0;
		ChipInfo.RdStart  = 0;
		ChipInfo.RdSize   = 0;

		#ifdef HOST_PC				/* Change ///AG*/
			hTuner = ChipOpen(&ChipInfo);
			(*TunerHandle) = hTuner;
		#endif

		#ifdef CHIP_STAPI
			hTuner = *TunerHandle;/*obtain hTuner : change ///AG*/
			memcpy(hTuner, &ChipInfo, sizeof(STCHIP_Info_t)); /* Copy the ChipInfo to hTuner: Change ///AG*/
		#endif

		if((*TunerHandle)== NULL)
			error=TUNER_INVALID_HANDLE;

	}
	else
		error=TUNER_INVALID_HANDLE;

	return error;

}

TUNER_Error_t SAT_NULL_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn)
{
	return TUNER_NO_ERR;
}

TUNER_Error_t SAT_NULL_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency)
{
	return TUNER_NO_ERR;
}

U32 SAT_NULL_TunerGetFrequency(STCHIP_Handle_t hTuner)
{
	return 0;
}


TUNER_Error_t SAT_NULL_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth)
{
	return TUNER_NO_ERR;
}

U32 SAT_NULL_TunerGetBandwidth(STCHIP_Handle_t hTuner)
{
	return 0;
}

TUNER_Error_t SAT_NULL_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain)
{
	return TUNER_NO_ERR;
}

S32 SAT_NULL_TunerGetGain(STCHIP_Handle_t hTuner)
{
	return 0;
}

TUNER_Error_t SAT_NULL_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider)
{
	return TUNER_NO_ERR;
}


TUNER_Error_t SAT_NULL_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn)
{
	return TUNER_NO_ERR;
}
BOOL SAT_NULL_TunerGetAttenuator(STCHIP_Handle_t hTuner)
{
	return FALSE;
}

BOOL SAT_NULL_TunerGetStatus(STCHIP_Handle_t hTuner)
{
	return FALSE;
}

TUNER_Error_t SAT_NULL_TunerWrite(STCHIP_Handle_t hTuner)
{
	return TUNER_NO_ERR;
}

TUNER_Error_t SAT_NULL_TunerRead(STCHIP_Handle_t hTuner)
{
	return TUNER_NO_ERR;
}

TUNER_Model_t SAT_NULL_TunerGetModel(STCHIP_Handle_t hChip)
{
	TUNER_Model_t model = TUNER_NULL;

	if(hChip && hChip->pData)
	{
		model = ((SAT_TUNER_Params_t) (hChip->pData))->Model;
	}

	return model;
}

void SAT_NULL_TunerSetIQ_Wiring(STCHIP_Handle_t hChip,  S8 IQ_Wiring)
{
}

S8 SAT_NULL_TunerGetIQ_Wiring(STCHIP_Handle_t hChip)
{
	return (S8)TUNER_IQ_NORMAL;
}

void SAT_NULL_TunerSetReferenceFreq(STCHIP_Handle_t hChip, U32 Reference)
{
}

U32 SAT_NULL_TunerGetReferenceFreq(STCHIP_Handle_t hChip)
{
	return 0;
}

void SAT_NULL_TunerSetIF_Freq(STCHIP_Handle_t hChip, U32 IF)
{
}

U32 SAT_NULL_TunerGetIF_Freq(STCHIP_Handle_t hChip)
{
	return 0;
}

void SAT_NULL_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect)
{

}

U8 SAT_NULL_TunerGetBandSelect(STCHIP_Handle_t hChip)
{
	return 0;
}


TUNER_Error_t SAT_NULL_TunerTerm(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR;

	if(hTuner)
	{
		#ifndef CHIP_STAPI
			if(hTuner->pData)
				free(hTuner->pData);

			ChipClose(hTuner);
		#endif
	}

	return error;
}




