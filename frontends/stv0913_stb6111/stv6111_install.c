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
 @File   stv6111_install.c                                                     
 @brief                                                                         
                                                                               
                                                                               
                                                                               
*/
#include "sysdbase.h"
#include "stfrontend.h"
#include "dbtypes.h"

#include "stv6111_tuner.h"
#include "stfrontend_traces.h"



/*-------------------------------------------------------------------------
					 Private Types 
------------------------------------------------------------------------- */

/*-------------------------------------------------------------------------
					 Private Constants 
------------------------------------------------------------------------- */

/*-------------------------------------------------------------------------
					 Private Variables
---------------------------------------------------------------------- */

/*-------------------------------------------------------------------------
					 Private Macros 
----------------------------------------------------------------------- */

/*-------------------------------------------------------------------------
					Private Function prototypes
----------------------------------------------------------------------- */
ST_ErrorCode_t STFRONTEND_TUNER_STV6111_MemInit(U32 Index);
ST_ErrorCode_t STFRONTEND_TUNER_STV6111_MemTerm(U32 Index);

/*-------------------------------------------------------------------------
 					Functions		
 --------------------------------------------------------------------- */

/*This function */



 ST_ErrorCode_t STFRONTEND_TUNER_STV6111_Install(STFRONTEND_tunerDbase_t *Tuner, STFRONTEND_TunerType_t TunerType)
 {
 	
    ST_ErrorCode_t Error = ST_NO_ERROR;

	SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_IN,LEVEL0,"%s"," ");
	
	if(TunerType !=  STFRONTEND_TUNER_STV6111){
			STTBX_Print(("Wrong Install function called"));
			SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_OUT,LEVEL0,"%s"," ");
			return(STFRONTEND_ERROR_ID);
	}
	
      	SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,INFO,LEVEL0," %s Installing sat:Tuner:STV6111...",__FUNCTION__);

       /* mark ID in database */
	Tuner->ID = STFRONTEND_TUNER_STV6111;
 	
	
    /* map API */

	Tuner->tuner_Init = STFRONTEND_TUNER_STV6111_MemInit;
	Tuner->tuner_Term = STFRONTEND_TUNER_STV6111_MemTerm; 
	/*LLA functions*/
	Tuner->tuner_Open = STV6111_TunerInit;
	/*ST_ErrorCode_t (*tuner_Close)(TUNER_Handle_t  Handle, TUNER_CloseParams_t *CloseParams); Not inuse -AG*/


	Tuner->tuner_SetFrequency = STV6111_TunerSetFrequency;
	Tuner->tuner_SetBandWidth = STV6111_TunerSetBandwidth;
	Tuner->tuner_GetStatus = STV6111_TunerGetStatus;
	Tuner->tuner_GetFrequency = STV6111_TunerGetFrequency;

	Tuner->tuner_SetReferenceFreq = STV6111_TunerSetReferenceFreq;
	Tuner->tuner_SetOutputClock = STV6111_TunerSetOutputClock;
	
	Tuner->tuner_SetGain = STV6111_TunerSetGain;
	Tuner->tuner_GetGain = STV6111_TunerGetGain;

	Tuner->tuner_TunerSetAttenuator = STV6111_TunerSetAttenuator;
	Tuner->tuner_SetStandby = STV6111_TunerSetStandby;
	
	Tuner->tuner_SetIQ_Wiring= STV6111_TunerSetIQ_Wiring;
	Tuner->tuner_GetIQ_Wiring= STV6111_TunerGetIQ_Wiring;
    /*...............................
     Set other function pointers here
    */
	

    SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_OUT,LEVEL0,"%s"," ");
    return(Error);
}

 ST_ErrorCode_t STFRONTEND_TUNER_STV6111_Uninstall(STFRONTEND_tunerDbase_t *Tuner)
 {
 	
    ST_ErrorCode_t Error = ST_NO_ERROR;

	SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_IN,LEVEL0,"%s"," ");

	if(NULL==Tuner)
	{
		SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,ERROR,LEVEL0, "%s pointer is NULL, Generating Error = ST_ERROR_BAD_PARAMETER", __FUNCTION__);
		SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_OUT,LEVEL0,"%s"," ");
		return(ST_ERROR_BAD_PARAMETER);
	}

	if(Tuner->ID !=  STFRONTEND_TUNER_STV6111){
			STTBX_Print(("Wrong Uninstall function called"));
			SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_OUT,LEVEL0,"%s"," ");
			return(STFRONTEND_ERROR_ID);
	}
	
      	SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,INFO,LEVEL0," %s Uninstalling sat:Tuner:STV6111...",__FUNCTION__);

 	Tuner->ID = STFRONTEND_TUNER_NONE;
	/* unmap API */

	Tuner->tuner_Init = NULL;
	Tuner->tuner_Term = NULL; 
	/*LLA functions*/
	Tuner->tuner_Open = NULL;
	Tuner->tuner_SetFrequency = NULL;
	Tuner->tuner_SetBandWidth = NULL;
	Tuner->tuner_GetStatus = NULL;
	Tuner->tuner_GetFrequency = NULL;

	Tuner->tuner_SetReferenceFreq = NULL;
	Tuner->tuner_SetOutputClock = NULL;
	Tuner->tuner_SetGain = NULL;
	Tuner->tuner_GetGain = NULL ;
 	Tuner->tuner_TunerSetAttenuator = NULL;
 	Tuner->tuner_SetStandby = NULL;

	Tuner->tuner_SetIQ_Wiring= NULL;
	Tuner->tuner_GetIQ_Wiring= NULL;
    /*...............................
     Clear other function pointers here
    */
    SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_OUT,LEVEL0,"%s"," ");
    return(Error);
}

ST_ErrorCode_t STFRONTEND_TUNER_STV6111_MemInit(U32 Index)
{
	ST_ErrorCode_t Error = ST_NO_ERROR;
	STFRONTEND_InstanceDbase_t *Inst;

	SAT_TUNER_Params_t pTunerParams;
	Inst = STFRONTEND_GetDrvInst();
	SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_IN,LEVEL0,"%s",Inst[Index].STFRONTEND_Name);

		/*Allocate Memory for TunerInternalParams*/
	pTunerParams = STOS_MemoryAllocateClear(Inst[Index].DriverPartition, 1 ,sizeof( SAT_TUNER_InitParams_t ));
	if(pTunerParams == NULL)
	{
		SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,ERROR,LEVEL0,"%s %s Failed To allocate memory, Generating ERROR = ST_ERROR_NO_MEMORY",__FUNCTION__, Inst[Index].STFRONTEND_Name);
		SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_OUT,LEVEL0,"%s",Inst[Index].STFRONTEND_Name);
		return(ST_ERROR_NO_MEMORY);
	}	
	/*Associate TunerInternalParams with Instance, Necessary for tuner_wrapper.c*/
	pTunerParams->TopLevelHandle = Index;

	/*Associate pTunerParams with hTuner->pData*/
	Inst[Index].Media_DBase.sat.TunerInst.hTuner->pData = (void *)pTunerParams;

	/*Allocate memory for regmap*/
	Inst[Index].Media_DBase.sat.TunerInst.hTuner->pRegMapImage = STOS_MemoryAllocateClear(Inst[Index].DriverPartition, STV6111_NBREGS,sizeof(STCHIP_Register_t));
	if(Inst[Index].Media_DBase.sat.TunerInst.hTuner->pRegMapImage  == NULL){
			SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,ERROR,LEVEL0,"%s %s Failed To allocate memory, Generating ERROR = ST_ERROR_NO_MEMORY",__FUNCTION__, Inst[Index].STFRONTEND_Name);
			SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_OUT,LEVEL0,"%s",Inst[Index].STFRONTEND_Name);
			return(ST_ERROR_NO_MEMORY);
	}	
	
	SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_OUT,LEVEL0,"%s",Inst[Index].STFRONTEND_Name);
	return(Error);
}

ST_ErrorCode_t STFRONTEND_TUNER_STV6111_MemTerm(U32 Index)
{
	ST_ErrorCode_t Error = ST_NO_ERROR;
	STFRONTEND_InstanceDbase_t *Inst;

	
	Inst = STFRONTEND_GetDrvInst();
	SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_IN,LEVEL0,"%s",Inst[Index].STFRONTEND_Name);

	/*DeAllocate memory for regmap*/
	if(Inst[Index].Media_DBase.sat.TunerInst.hTuner->pRegMapImage != NULL)
	{
	STOS_MemoryDeallocate(Inst[Index].DriverPartition, Inst[Index].Media_DBase.sat.TunerInst.hTuner->pRegMapImage);
	Inst[Index].Media_DBase.sat.TunerInst.hTuner->pRegMapImage=NULL;
	}
	/*DeAllocate Memory for TunerInternalParams*/
	
	if(Inst[Index].Media_DBase.sat.TunerInst.hTuner->pData != NULL)
	{
	STOS_MemoryDeallocate(Inst[Index].DriverPartition, Inst[Index].Media_DBase.sat.TunerInst.hTuner->pData);
	Inst[Index].Media_DBase.sat.TunerInst.hTuner->pData=NULL;
	}
	SET_TRACE(STFRONTEND_DRIVERS_SAT_TUNER_CID,API_OUT,LEVEL0,"%s",Inst[Index].STFRONTEND_Name);
	return(Error);

}


