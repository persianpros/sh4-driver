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
 @File   d0913.c                                                               
 @brief                                                                         
                                                                               
                                                                               
                                                                               
*/
#include "d0913.h"
#include "stv0913_drv.h"
#include "sysdbase.h"
#include "stfrontend_traces.h"
#include "stvvglna.h"
#include "stv0913_util.h"

#include "stfe_utilities.h"
#include "fe_sat_tuner.h"
#include "fesat_commlla_str.h"
#include "globaldefs.h"
#include "sattask.h" /*for SATTASK_EnterExitStandby()*/
#include "searchutil.h" /*for STFRONTEND_Sat_ResetSearchResults() */

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
FE_Sat_Tuner_t STV0913_FindTunerType(const STFRONTEND_TunerType_t STFRONTEND_TunerType);
TUNER_Model_t STV0913_TunerModelMap(const STFRONTEND_TunerType_t STFRONTEND_TunerType);
STFRONTEND_FECRate_t STV0913_GetFECRate_n_Map(const FE_Sat_TrackingStandard_t FoundStandard,const FE_Sat_Rate_t FoundPunctureRate,const STFRONTEND_ModeCode_t modCode);
STFRONTEND_FECType_t STV0913_FoundStandardMap(const FE_Sat_TrackingStandard_t Standard);
STFRONTEND_Modulation_t STV0913_GetModulation(const STFRONTEND_FECType_t FoundStandard,const STFRONTEND_ModeCode_t modCode);

U32 STV0913_GetOtherPathHandle(STFRONTEND_Handle_t Handle);
FE_Sat_Rate_t STFRONTEND_to_913LLA_FECMap(STFRONTEND_FECRate_t FECRate);

 void Prepare_913_LLASearchParams(STFRONTEND_Handle_t Handle,FE_STV0913_SearchParams_t *pSearch_p);
 void Prepare_913_LLAParams(STFRONTEND_InitParams_t* InitParam_p,FE_Sat_InitParams_t *LLA_InitParams_p,FE_STV0913_TSConfig_t * PathTSConfig);
void Prepare_913_SearchResult(STFRONTEND_SAT_SearchResult_t * pSTFRONTEND_Results,FE_STV0913_SearchResult_t *pResults_p);
void Update_913_SearchResult(STFRONTEND_SAT_SearchResult_t *pSTFRONTEND_Info,FE_STV0913_SignalInfo_t * SignalInfo_p);
 ST_ErrorCode_t  get_diseqc_mode_913(FE_STV0913_Handle_t Handle,FE_Sat_DiseqC_TxMode *Mode_p, unsigned char *tx_data, U32 *Nbdata, STFRONTEND_DiSEqCSendPacket_t *pDiSEqCSendPacket);
FE_Sat_Rate_t STFRONTEND_to_913LLA_ModulationMap(STFRONTEND_Modulation_t Modulation);

/*-------------------------------------------------------------------------
      Functions  
 --------------------------------------------------------------------- */
ST_ErrorCode_t STFRONTEND_DEMOD_STV0913_Install(STFRONTEND_demodDbase_t *Demod, STFRONTEND_DemodType_t DemodType )
{
 
    ST_ErrorCode_t Error = ST_NO_ERROR;

 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s"," ");
 
 if(DemodType !=  STFRONTEND_DEMOD_STV0913){
   STTBX_Print(("Wrong Install function called"));
   SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s"," ");
   return(STFRONTEND_ERROR_ID);
 }
 
     SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0," %s Installing sat:demod:STV0913...",__FUNCTION__);

 /* mark ID in database */
 Demod->ID = DemodType;
 /* map API */
 Demod->demod_Init    = demod_stv0913_Init;
 Demod->demod_Open    = demod_stv0913_Open;
 Demod->demod_Close   = demod_stv0913_Close;
 Demod->demod_Term   = demod_stv0913_Term;

 Demod->demod_StandByMode   = demod_stv0913_StandbyMode;
 Demod->demod_Diseqc    = demod_stv0913_DiseqcSendReceive;
 Demod->demod_Search= demod_stv0913_Search;
 Demod->demod_Tracking   = demod_stv0913_Tracking;
 Demod->demod_Unlock = demod_stv0913_Unlock;
 Demod->demod_Abort = demod_stv0913_Abort;
 Demod->demod_CustomiseControl=demod_stv0913_DiSEqC_Env_Mode_Select;
 Demod->demod_SetContinous_Tone=demod_stv0913_SetContinous_Tone;
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s"," ");
 return(Error);
}

ST_ErrorCode_t STFRONTEND_DEMOD_STV0913_Uninstall(STFRONTEND_demodDbase_t *Demod)
{
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s"," ");
 if(NULL==Demod)
 {
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,ERROR,LEVEL0, "%s pointer is NULL, Generating Error = ST_ERROR_BAD_PARAMETER", __FUNCTION__);
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s"," ");
  return(ST_ERROR_BAD_PARAMETER);
 }
 if(Demod->ID !=  STFRONTEND_DEMOD_STV0913){
   STTBX_Print(("Wrong Uninstall function called"));
   SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s"," ");
   return(STFRONTEND_ERROR_ID);
 }

     SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0," %s Uninstalling sat:demod:STV0913...",__FUNCTION__);

 Demod->ID = STFRONTEND_DEMOD_NONE;
 /* unmap API */
 Demod->demod_Init    = NULL;
 Demod->demod_Open    = NULL;
 Demod->demod_Close   = NULL;
 Demod->demod_Term   = NULL;

 Demod->demod_StandByMode   = NULL;
 Demod->demod_Search = NULL;
 Demod->demod_Tracking   = NULL;
 Demod->demod_Unlock= NULL;
 Demod->demod_Abort  = NULL;
 Demod->demod_CustomiseControl=NULL;

 Demod->demod_SetContinous_Tone=NULL;
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s"," ");
 return ST_NO_ERROR;
 
}

ST_ErrorCode_t demod_stv0913_Init(U32 index)
{
 ST_ErrorCode_t Error = ST_NO_ERROR;
 STFRONTEND_InstanceDbase_t *Inst;

 /* Parameters Specific to 913*/
  FE_STV0913_InternalParams_t *pParams;
 
 Inst = STFRONTEND_GetDrvInst();
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s",Inst[index].STFRONTEND_Name);

 /*Allocate memory for tuner structures*/
 if((Inst[index].Media_DBase.sat.TunerInst.TunerDB->tuner_Init) !=NULL){
  Error = (Inst[index].Media_DBase.sat.TunerInst.TunerDB->tuner_Init)(index);
 }
 else{
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[index].STFRONTEND_Name);
  return(STFRONTEND_ERROR_POINTER);
 }
 if(!Error){ /*tuner init is successful*/
    /*Allocate memory for Demod's InternalParams structure*/
    pParams = STOS_MemoryAllocateClear(Inst[index].DriverPartition, 1 ,sizeof( FE_STV0913_InternalParams_t ));
    if(pParams == NULL)
    {
     SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,ERROR,LEVEL0,"%s %s Failed To allocate memory, Generating ERROR = ST_ERROR_NO_MEMORY",__FUNCTION__, Inst[index].STFRONTEND_Name);
     SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[index].STFRONTEND_Name);
     return(ST_ERROR_NO_MEMORY);
    }
  
    /*Associate Demod Chip handle to pParams*/
    pParams->hDemod = Inst[index].Media_DBase.sat.DemodInst.hDemod;
  
  
    /*Allocate Memory for pRegMapImage (formerly called DeviceMap)*/
    pParams->hDemod->pRegMapImage = STOS_MemoryAllocateClear(Inst[index].DriverPartition, STV0913_NBREGS,sizeof(STCHIP_Register_t));
    if(pParams->hDemod->pRegMapImage == NULL)
    {
     pParams->hDemod = NULL;
     if(pParams != NULL)
 	{
     	STOS_MemoryDeallocate(Inst[index].DriverPartition, pParams);
  	 pParams=NULL;
	}	
     SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,ERROR,LEVEL0,"%s %s Failed To allocate memory, Generating ERROR = ST_ERROR_NO_MEMORY",__FUNCTION__, Inst[index].STFRONTEND_Name);
     SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[index].STFRONTEND_Name);
     return(ST_ERROR_NO_MEMORY);
    } 
    pParams->TopLevelHandle = index;
   
    pParams->hTuner =  (TUNER_Handle_t)Inst[index].Media_DBase.sat.TunerInst.hTuner;
 
    /*Associate Instance with pParams, so that one can access pParams from Instance */
    Inst[index].Media_DBase.sat.DemodInst.DemodInternalParams = (void *)pParams;
  }
 else{/*if (!Error) tuner init*/
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,ERROR,LEVEL0,"%s %s Error in Tuner Init, Error =0x%x",__FUNCTION__, Inst[index].STFRONTEND_Name, Error);
 }
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[index].STFRONTEND_Name);
 return(Error);
}

void Prepare_913_LLAParams(STFRONTEND_InitParams_t* InitParam_p,FE_Sat_InitParams_t *LLA_InitParams_p,FE_STV0913_TSConfig_t * PathTSConfig)
{
strncpy(LLA_InitParams_p->DemodName,"STV0913", sizeof(LLA_InitParams_p->DemodName));    /* Demod name */
 LLA_InitParams_p->DemodI2cAddr     = InitParam_p->DemodIO.Address;
 LLA_InitParams_p->DemodRefClock    =  InitParam_p->DemodClock;  /* Refrence clock for the Demodulator in Hz (27MHz,30MHz,....)*/
 LLA_InitParams_p->DemodType   = FE_SAT_DUAL; /* use the 913 as a dual demodulator 
               (the 913 could also be used as a single demod to enable the DVBS2 16APSK and 32APSK modes)*/
 LLA_InitParams_p->RollOff    =   FE_SAT_35;   /* NYQUIST Filter value (used for DVBS1/DSS, DVBS2 is automatic) */

 /*Path1 parameters*/
 LLA_InitParams_p->TunerHWControlType   =   STV0913_FindTunerType(InitParam_p->TunerType);  /* Tuner1 HW controller Type */

 LLA_InitParams_p->PathTSClock = InitParam_p->TSModeParams.TSOutputMode;
 
 LLA_InitParams_p->TunerModel  = STV0913_TunerModelMap(InitParam_p->TunerType); /* Tuner 1 model (SW control) */
 strncpy(LLA_InitParams_p->TunerName,"Tuner1", sizeof(LLA_InitParams_p->TunerName));/* Tuner 1 name  (not required for STFRONTEND) */
 LLA_InitParams_p->Tuner_I2cAddr  = InitParam_p->TunerIO.Address; /* Tuner 1 I2c address */
 LLA_InitParams_p->TunerRefClock  = InitParam_p->TunerClock;  /* Tuner 1 Refrence/Input clock for the Tuner in Hz (27MHz,30MHz,16MHz....)*/
 LLA_InitParams_p->TunerOutClkDivider = InitParam_p->TunerOutClkDiv;  /*Tuner 1 Clock Divider for 6110 1,2,4 or 8 */
 LLA_InitParams_p->TunerIQ_Inversion = InitParam_p->Tuner_IQwiring; /* Tuner 1 Global IQ setting (coonction of IQ from the tuner1 to the demod */
 LLA_InitParams_p->TunerBasebandGain = TUNER_BBGAIN(LLA_InitParams_p->TunerModel);    /* Tuner 1 base band gain */
if(InitParam_p->TunerType == STFRONTEND_TUNER_STV6120_Tuner2){
	 LLA_InitParams_p->TunerBandSelect = LBRFD_HBRFC;
	 LLA_InitParams_p->TunerRF_Source= FE_RF_SOURCE_C;
	LLA_InitParams_p->TunerInputSelect= TUNER_INPUT2;
	/*printf("\n***STFRONTEND_TUNER_STV6120_Tuner2 is selected");*/
}else {
	LLA_InitParams_p->TunerBandSelect =0;/* Used for 6120 only */

}
 /*printf("\n<%s,%d>TunerInitParams.BAndSelect is %d..RFSource1 =%d Inputselect1=%d",__func__,__LINE__,LLA_InitParams_p->TunerBandSelect,LLA_InitParams_p->TunerRF_Source,LLA_InitParams_p->TunerInputSelect);
 printf("\n<%s>..tunermodel =%d ..Tunertype=%d",__func__,LLA_InitParams_p->TunerModel,InitParam_p->TunerType);
 */
 PathTSConfig->TSClockPolarity = InitParam_p->TSModeParams.ClockPolarity;
 PathTSConfig->TSMode = InitParam_p->TSModeParams.TSOutputMode;
 PathTSConfig->TSParityBytes = InitParam_p->TSModeParams.ParityBytesEnable;
 PathTSConfig->TSSpeedControl = InitParam_p->TSModeParams.DataRateControl;
 PathTSConfig->TSSyncByteEnable = InitParam_p->TSModeParams.SyncByteEnable;
 PathTSConfig->TSSwap = InitParam_p->TSModeParams.Swap;/*MSB<->LSB swap in Serial mode*/
 PathTSConfig->TSClockRate = InitParam_p->TSModeParams.TSClockRate;/*Only valid if DataRateControl is set to MANUAL*/


}


/*The following Routine return ST_NO_ERROR for 913 path1
    However for 913path2 it calls LLA's FE_STV0913_Init() function*/
ST_ErrorCode_t demod_stv0913_Open(STFRONTEND_Handle_t Handle)
{
 ST_ErrorCode_t Error = ST_NO_ERROR;
 STFRONTEND_InstanceDbase_t *Inst;
 FE_Sat_InitParams_t LLA_InitParams;
 FE_STV0913_InternalParams_t *pParams;
 FE_STV0913_TSConfig_t PathTSConfig;
  U32 CurrentPathTS_Speed;
 
 Inst = STFRONTEND_GetDrvInst();
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
 
 memset(&LLA_InitParams,'\0',sizeof(FE_Sat_InitParams_t));
 pParams = (FE_STV0913_InternalParams_t *)Inst[Handle].Media_DBase.sat.DemodInst.DemodInternalParams;
 

 /*Fill the Init params, this step is identical to the one used in LLA application filiing InitParams for device*/
Prepare_913_LLAParams(Inst[Handle].InitParm,& LLA_InitParams,&PathTSConfig); 

 /*Debug prints, The following prints will display the intitialisation parameters*/
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Demod I2C Address = 0x%x",__FUNCTION__, Inst[Handle].STFRONTEND_Name,LLA_InitParams.DemodI2cAddr);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Demod Ref Clock = %u",__FUNCTION__, Inst[Handle].STFRONTEND_Name,LLA_InitParams.DemodRefClock );
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Tuner Model = %d",__FUNCTION__, Inst[Handle].STFRONTEND_Name, LLA_InitParams.TunerModel);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Tuner I2C Address = 0x%x",__FUNCTION__, Inst[Handle].STFRONTEND_Name,LLA_InitParams.Tuner_I2cAddr );
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Tuner Ref Clock = %u",__FUNCTION__, Inst[Handle].STFRONTEND_Name, LLA_InitParams.TunerRefClock);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Tuner Output Clk Divider = %u",__FUNCTION__, Inst[Handle].STFRONTEND_Name, LLA_InitParams.TunerOutClkDivider);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Tuner Baseband Gain = %d",__FUNCTION__, Inst[Handle].STFRONTEND_Name,LLA_InitParams.TunerBasebandGain);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Tuner IQ Wiring = %d (0: Normal, 1: Swapped)",__FUNCTION__, Inst[Handle].STFRONTEND_Name,LLA_InitParams.TunerIQ_Inversion);

 
 /*Now calling the LLA init() which initialises Demod and call Tuner Initialisation functions*/
 /*Note: DemodHandle has actually been obtained during demod_stv0913_init, when memory was allocated*/
 Error = FE_STV0913_Init(&LLA_InitParams, &Inst[Handle].Media_DBase.sat.DemodInst.DemodInternalParams);
 if(Error){
   SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,ERROR,LEVEL0, "%s %s Error in LLA init, Error = 0x%x",__FUNCTION__, Inst[Handle].STFRONTEND_Name,Error);
   SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
   return(Error);
 }
 
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s TSMode = %d (0: Default, 1: Serial_cont, 2: Serial Punc; 3: Parallel Punc;4 : DVB-CI)",__FUNCTION__, Inst[Handle].STFRONTEND_Name,PathTSConfig.TSMode);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s ClockPolarity= %d (0: Default, 1:RisingEdge, 2: FallingEdge )",__FUNCTION__, Inst[Handle].STFRONTEND_Name,PathTSConfig.TSClockPolarity);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s SyncByteEnable= %d (0: Default, 1:ON, 2:OFF )",__FUNCTION__, Inst[Handle].STFRONTEND_Name,PathTSConfig.TSSyncByteEnable);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s ParityByteEnable= %d (0: Default, 1:ON, 2:OFF )",__FUNCTION__, Inst[Handle].STFRONTEND_Name,PathTSConfig.TSParityBytes);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s TS_SPEED_CONTROL= %d (0: Default, 1:MANUAL, 2:AUTO )",__FUNCTION__, Inst[Handle].STFRONTEND_Name,PathTSConfig.TSSpeedControl);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s TSSwap= %d (0: Default, 1:ON, 2:OFF )",__FUNCTION__, Inst[Handle].STFRONTEND_Name,PathTSConfig.TSSwap);
 
 Error= FE_STV0913_SetTSoutput((FE_STV0913_Handle_t)pParams,&PathTSConfig);
 
 Error |=  FE_STV0913_SetTSConfig((FE_STV0913_Handle_t)pParams,&PathTSConfig,  &CurrentPathTS_Speed);
  if(Error){
   SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,ERROR,LEVEL0, "%s %s Error in LLA SetTS Config, Error = 0x%x",__FUNCTION__, Inst[Handle].STFRONTEND_Name,Error);
   SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
   return(Error);
 }
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s TS_SPEED = %d Hz",__FUNCTION__, Inst[Handle].STFRONTEND_Name,CurrentPathTS_Speed);
 
 /*add additional features here if any*/

 
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
 return(Error);
}

ST_ErrorCode_t demod_stv0913_Close(STFRONTEND_Handle_t Handle)
{
 ST_ErrorCode_t Error = ST_NO_ERROR;
 
 STFRONTEND_InstanceDbase_t *Inst;
 FE_STV0913_InternalParams_t *pParams;

 Inst= STFRONTEND_GetDrvInst();
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);

 /*call TunerClose*/

 pParams = (FE_STV0913_InternalParams_t *)Inst[Handle].Media_DBase.sat.DemodInst.DemodInternalParams;
 memset(pParams->hDemod->pRegMapImage, '\0', sizeof(STCHIP_Register_t)*STV0913_NBREGS);
 
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
 return(Error);
}

ST_ErrorCode_t demod_stv0913_Term(STFRONTEND_Handle_t Handle)
{
 ST_ErrorCode_t Error = ST_NO_ERROR;
 STFRONTEND_InstanceDbase_t *Inst;
 FE_STV0913_InternalParams_t *pParams;


 Inst= STFRONTEND_GetDrvInst();
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);

 /*call TunerTerm*/
  /*Deallocate memory for tuner structures*/
 if((Inst[Handle].Media_DBase.sat.TunerInst.TunerDB->tuner_Term) !=NULL){
  Error = (Inst[Handle].Media_DBase.sat.TunerInst.TunerDB->tuner_Term)(Handle);
 }
 else{
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
  return(STFRONTEND_ERROR_POINTER);
 }

 pParams = (FE_STV0913_InternalParams_t *)Inst[Handle].Media_DBase.sat.DemodInst.DemodInternalParams;

if(pParams != NULL)
{
	if(pParams->hDemod->pRegMapImage != NULL)
 	{
 	STOS_MemoryDeallocate(Inst[Handle].DriverPartition, pParams->hDemod->pRegMapImage);
 	pParams->hDemod->pRegMapImage=NULL;
	}
 pParams->hDemod = NULL;
 pParams->hTuner = NULL;

 STOS_MemoryDeallocate(Inst[Handle].DriverPartition, pParams);
 pParams=NULL;
}
 
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
 return(Error);
}

ST_ErrorCode_t demod_stv0913_StandbyMode(STFRONTEND_Handle_t Handle, STFRONTEND_StandByMode_t PowerMode)
{
 ST_ErrorCode_t Error;
 STFRONTEND_InstanceDbase_t *Inst;

 Inst= STFRONTEND_GetDrvInst();
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);

 Error = FE_STV0913_SetStandby((FE_STV0913_Handle_t)Inst[Handle].Media_DBase.sat.DemodInst.DemodInternalParams, PowerMode);
 
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
 return(Error);
}
void Prepare_913_LLASearchParams(STFRONTEND_Handle_t Handle,FE_STV0913_SearchParams_t *pSearch_p)
{
STFRONTEND_InstanceDbase_t *Inst;
Inst = STFRONTEND_GetDrvInst();

pSearch_p->Frequency = Inst[Handle].Media_DBase.sat.SearchFreq;
 pSearch_p->SymbolRate = Inst[Handle].Media_DBase.sat.SearchParams.SymbolRate;

if(Inst[Handle].taskData.AcqMode== STFRONTEND_SETFREQUENCY){ 
  /*if SetFrequency API() used this parameter*/
  pSearch_p->SearchRange  = 10000000;/*10 MHz*/
 }
 else {
  pSearch_p->SearchRange = ABS(Inst[Handle].Media_DBase.sat.ScanInfo.FrequencyStep)*1000;
  
 }

 pSearch_p->Standard = Inst[Handle].Media_DBase.sat.SearchParams.FECType;
 pSearch_p->IQ_Inversion =FE_SAT_IQ_AUTO ;
 pSearch_p->SearchAlgo = Inst[Handle].Media_DBase.sat.SearchParams.SearchAlgo;
 if(pSearch_p->SearchAlgo == FE_SAT_BLIND_SEARCH)
 {
	pSearch_p->Modulation = FE_SAT_MOD_UNKNOWN;
 }
 pSearch_p->Path = FE_SAT_DEMOD_1;
 pSearch_p->PunctureRate =STFRONTEND_to_913LLA_FECMap(Inst[Handle].Media_DBase.sat.SearchParams.FECRates);
 //pSearch_p->Modulation = FE_SAT_MOD_UNKNOWN;
 pSearch_p->Modcode = FE_SAT_MODCODE_UNKNOWN;
}

void Prepare_913_SearchResult(STFRONTEND_SAT_SearchResult_t * pSTFRONTEND_Results,FE_STV0913_SearchResult_t *pResults_p)
	{    
	pSTFRONTEND_Results->SymbolRate = pResults_p->SymbolRate; 
	pSTFRONTEND_Results->FECRate = STV0913_GetFECRate_n_Map(pResults_p->Standard,pResults_p->PunctureRate,pResults_p->ModCode);
	pSTFRONTEND_Results->Pilots = pResults_p->Pilots;/*No need of mapping*/    
	pSTFRONTEND_Results->Framelength = pResults_p->FrameLength;/*No need of mapping*/   
	pSTFRONTEND_Results->ModCode = pResults_p->ModCode;/*No need of mapping*/   
	pSTFRONTEND_Results->Spectrum = pResults_p->Spectrum;/*No need of mapping*/    
	pSTFRONTEND_Results->FECType = STV0913_FoundStandardMap(pResults_p->Standard);/*Found Standard*/    
	pSTFRONTEND_Results->Modulation =STV0913_GetModulation(pSTFRONTEND_Results->FECType,pSTFRONTEND_Results->ModCode);   
	pSTFRONTEND_Results->RollOff = pResults_p->RollOff;/*No need of mapping*/

}

ST_ErrorCode_t demod_stv0913_Search(STFRONTEND_Handle_t Handle, BOOL *IsLocked)
{
 ST_ErrorCode_t Error=ST_NO_ERROR;
 STFRONTEND_InstanceDbase_t *Inst;
 FE_STV0913_SearchParams_t pSearch;
 FE_STV0913_SearchResult_t pResults;
 void *DemodHandle ;
 STFRONTEND_SAT_SearchResult_t *pSTFRONTEND_Results;
 
 Inst = STFRONTEND_GetDrvInst();
 
 memset(&pResults,'\0',sizeof(FE_STV0913_SearchResult_t));
 memset(&pSearch,'\0',sizeof(FE_STV0913_SearchParams_t));
 
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
 DemodHandle = Inst[Handle].Media_DBase.sat.DemodInst.DemodInternalParams;
 pSTFRONTEND_Results = &Inst[Handle].CurrentTunerInfo.STFRONTEND_SearchResult.SatSearchRes;

 STFRONTEND_Sat_ResetSearchResults(Handle);  

if(Inst[Handle].Media_DBase.sat.SearchParams.SearchAlgo != STFRONTEND_BLIND_SEARCH)
{       /*8PSK is valid with DVBS2 and AUTO */
	if(Inst[Handle].Media_DBase.sat.SearchParams.Modulation==STFRONTEND_MOD_ALL ||
	    Inst[Handle].Media_DBase.sat.SearchParams.Modulation==STFRONTEND_MOD_QPSK ||
	    ((Inst[Handle].Media_DBase.sat.SearchParams.Modulation==STFRONTEND_MOD_8PSK) 
	        && ((Inst[Handle].Media_DBase.sat.SearchParams.FECType==STFRONTEND_FEC_AUTO_SEARCH ) ||(Inst[Handle].Media_DBase.sat.SearchParams.FECType==STFRONTEND_FEC_DVBS2_STANDARD)) )||
	    Inst[Handle].Media_DBase.sat.SearchParams.Modulation==STFRONTEND_MOD_16APSK ||
	    Inst[Handle].Media_DBase.sat.SearchParams.Modulation==STFRONTEND_MOD_32APSK )
		pSearch.Modulation =STFRONTEND_to_913LLA_ModulationMap(Inst[Handle].Media_DBase.sat.SearchParams.Modulation);
	else
		return FE_LLA_BAD_PARAMETER ;
}

Prepare_913_LLASearchParams(Handle,&pSearch); 
if((Inst[Handle].Media_DBase.sat.SearchParams.addtnlSearchParams!=NULL) && (pSearch.SearchAlgo == FE_SAT_BLIND_SEARCH))
{
 	/*BlindScan Updates.*/
    if(((STFRONTEND_Additional_SearchParams_t *)Inst[Handle].Media_DBase.sat.SearchParams.addtnlSearchParams)->Addnl.SatVer1.Blind_Optimize.Optimize_On == TRUE)
	{
		pSearch.SearchRange = pSearch.SymbolRate/2 ;/*search range=MinSR/2 >>for blindearch it is 1Msps so its always 500000 */
		pSearch.Doublefail=Inst[Handle].Media_DBase.sat.ScanInfo.Doublefail;
		pSearch.Directblind=Inst[Handle].Media_DBase.sat.ScanInfo.Directblind;
		//printf("\n<d0913.c> Freq=%d SearchRange=%d..DoubleFail=%d..Directblind=%d \n",pSearch.Frequency,pSearch.SearchRange,pSearch.Doublefail,pSearch.Directblind);
		if(ABS(pSearch.Frequency -STFRONTEND_Sat_IFfromFreq(Handle,Inst[Handle].Media_DBase.sat.ScanInfo.StartFreq)) <100)
		   Inst[Handle].Media_DBase.sat.ScanInfo.Frequency_Prev_tp = 0;
    	}	
 }
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s \n -----Calling LLA Search-------",__FUNCTION__,Inst[Handle].STFRONTEND_Name);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Demod Handle = 0x%x ",__FUNCTION__, Inst[Handle].STFRONTEND_Name,(U32)DemodHandle);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Search Frequency(IF) = %u KHz",__FUNCTION__, Inst[Handle].STFRONTEND_Name, pSearch.Frequency);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Symbol Rate =   %u Symb/s",__FUNCTION__, Inst[Handle].STFRONTEND_Name, pSearch.SymbolRate);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Search Range = %u Hz",__FUNCTION__, Inst[Handle].STFRONTEND_Name, pSearch.SearchRange);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Modcode = %d (for DVBS2 only) ",__FUNCTION__, Inst[Handle].STFRONTEND_Name, pSearch.Modcode);
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0, "%s %s Search Algo = %d (0: Blind, 1: Cold, 2: Warm)",__FUNCTION__, Inst[Handle].STFRONTEND_Name, pSearch.SearchAlgo);
 
if(Inst[Handle].Media_DBase.sat.SearchParams.addtnlSearchParams!=NULL)
Error=FE_STV0913_DVBS2_SetGoldCodeX((FE_STV0913_Handle_t)DemodHandle, 
		((STFRONTEND_Additional_SearchParams_t *)Inst[Handle].Media_DBase.sat.SearchParams.addtnlSearchParams)->Addnl.SatVer1.GoldCode);

 Error = FE_STV0913_Search((FE_STV0913_Handle_t)DemodHandle, &pSearch, &pResults, pSearch.Directblind);
if(pSearch.SearchAlgo != FE_SAT_BLIND_SEARCH)
{
	 if(!Error){
	   *IsLocked =   pResults.Locked;   
	   if(pResults.Locked)
	   {
	    SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s Demod Locked at IF = %u kHz\n",__FUNCTION__,Inst[Handle].STFRONTEND_Name,pResults.Frequency);
	  
	    Inst[Handle].CurrentTunerInfo.Frequency = pResults.Frequency;
	    Prepare_913_SearchResult( pSTFRONTEND_Results, &pResults);
	  
	   }
	 }
	 else if((Error == FE_LLA_SEARCH_FAILED|| Error == FE_LLA_TRACKING_FAILED || Error == FE_LLA_TUNER_NOSIGNAL) ) {
	   Error = ST_NO_ERROR;/*these non-serious errors are not to be reported to upper layers */
	  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s \n -----LLA Lock failed-------RESET search reselts \n",__FUNCTION__,Inst[Handle].STFRONTEND_Name);
	   STFRONTEND_Sat_ResetSearchResults(Handle);
	 }
	 else{
	   SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,ERROR,LEVEL0,"%s %s Found Error in LLA Search, Error = 0x%x",__FUNCTION__, Inst[Handle].STFRONTEND_Name,Error);
	 }
}
else{
	/*Update the Scaninfo Parameters that are currently available in pSearch.*/	 
	Inst[Handle].Media_DBase.sat.ScanInfo.TunerIndexJump = 	pSearch.TunerIndexJump ;
	
	/*printf("<d0913>TunerIndJump=%d KHZ  Freq=%d SignalType =%d (0>>TunerJmp//1>>NOSignal//2>>SearchFAIL)\n",Inst[Handle].Media_DBase.sat.ScanInfo.TunerIndexJump,
			Inst[Handle].Media_DBase.sat.ScanInfo.SearchFrequency,Inst[Handle].Media_DBase.sat.ScanInfo.Blind_SignalType);
  	*/
	 /*For new BlindScan Algo some error types are returned from the driver in case of LOCK*/
	if((Error == FE_LLA_NO_ERROR) ||(Error == FE_LLA_NODATA) ){
	   *IsLocked =   pResults.Locked;   
	   if(pResults.Locked)
	   {
	    SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s Demod Locked at IF = %u kHz",__FUNCTION__,Inst[Handle].STFRONTEND_Name,pResults.Frequency);
		if( (pSearch.SearchAlgo== FE_SAT_BLIND_SEARCH) 
			&& (pResults.Locked ==TRUE)
			&&(ABS((Inst[Handle].Media_DBase.sat.ScanInfo.Frequency_Prev_tp- pResults.Frequency))< 1000) )
			{
				 /*A duplicate tp is detected ;Ignore it */
				
				  *IsLocked =0; //hardcoding to LOCK failed
				  Inst[Handle].Media_DBase.sat.ScanInfo.Blind_SignalType  = STFRONTEND_LLA_TUNER_NO_DATA;
				  //printf("\n***A duplicate tp is detected pResults.FREQUENCY =%d; prev_tp =%d Ignore it..SignalType=%d ****",pResults.Frequency,Inst[Handle].Media_DBase.sat.ScanInfo.Frequency_Prev_tp,Inst[Handle].Media_DBase.sat.ScanInfo.Blind_SignalType);	
				  Inst[Handle].Media_DBase.sat.ScanInfo.Re_detected_tp++;
				  //printf("<d0913> ..re-detected =%d*****\n",Inst[Handle].Media_DBase.sat.ScanInfo.Re_detected_tp);
			}
			 //printf("\n<d0913> pResults.FREQUENCY =%d; prev_tp =%d ***",pResults.Frequency,Inst[Handle].Media_DBase.sat.ScanInfo.Frequency_Prev_tp);	
			Prepare_913_SearchResult( pSTFRONTEND_Results, &pResults);
			Inst[Handle].CurrentTunerInfo.Frequency = pResults.Frequency;
			/*With the new BlindScan Algo its possible to get the same tp locked more than once*/
			Inst[Handle].Media_DBase.sat.ScanInfo.Frequency_Prev_tp = pResults.Frequency;
		        
  
	   }
	 }
	 else if((Error == FE_LLA_SEARCH_FAILED|| Error == FE_LLA_TRACKING_FAILED) ) {
	   Error = ST_NO_ERROR;/*these non-serious errors are not to be reported to upper layers */
	   SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s \n -----LLA Lock failed-------",__FUNCTION__,Inst[Handle].STFRONTEND_Name);
	   STFRONTEND_Sat_ResetSearchResults(Handle);
	   Inst[Handle].Media_DBase.sat.ScanInfo.Blind_SignalType=STFRONTEND_LLA_TUNER_SEARCH_FAIL;
	   pSearch.TunerIndexJump = pSearch.TunerIndexJump/2 ;
	 }
	 else if(Error == FE_LLA_TUNER_JUMP) 
	 {
			/*Do nothing these error are to be passed to the Sattask as signals.*/
			Inst[Handle].Media_DBase.sat.ScanInfo.Blind_SignalType = STFRONTEND_LLA_TUNER_JUMP;
			pSearch.TunerIndexJump = 16000 ;/*TunerIndexjump value for use in next Search call .*/
	 }		
	  else if(Error == FE_LLA_TUNER_NOSIGNAL)  
	  	/*Do nothing these error are to be passed to the Sattask as signals.*/
			Inst[Handle].Media_DBase.sat.ScanInfo.Blind_SignalType = STFRONTEND_LLA_TUNER_NO_SIGNAL;
	 else {
	   SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,ERROR,LEVEL0,"%s %s Found Error in LLA Search, Error = 0x%x ",__FUNCTION__, Inst[Handle].STFRONTEND_Name,Error);
	 }
//printf("<d0913>signalType=%d\n",Inst[Handle].Media_DBase.sat.ScanInfo.Blind_SignalType);
}
 
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
 return(Error);
}


void Update_913_SearchResult(STFRONTEND_SAT_SearchResult_t *pSTFRONTEND_Info,FE_STV0913_SignalInfo_t * SignalInfo_p)
{
   pSTFRONTEND_Info->SymbolRate = SignalInfo_p->SymbolRate;
  pSTFRONTEND_Info->FECRate = STV0913_GetFECRate_n_Map(SignalInfo_p->Standard, SignalInfo_p->PunctureRate,SignalInfo_p->ModCode);
  pSTFRONTEND_Info->Pilots = SignalInfo_p->Pilots;/*No need of mapping*/
  pSTFRONTEND_Info->Framelength = SignalInfo_p->FrameLength;/*No need of mapping*/
  pSTFRONTEND_Info->ModCode = SignalInfo_p->ModCode;/*No need of mapping*/
  pSTFRONTEND_Info->Spectrum = SignalInfo_p->Spectrum;/*No need of mapping*/
  pSTFRONTEND_Info->FECType = STV0913_FoundStandardMap(SignalInfo_p->Standard);/*Found Standard*/
  pSTFRONTEND_Info->Modulation =STV0913_GetModulation(pSTFRONTEND_Info->FECType,pSTFRONTEND_Info->ModCode);
  pSTFRONTEND_Info->RollOff  = SignalInfo_p->RollOff;  
  pSTFRONTEND_Info->SignalQuality = SignalInfo_p->C_N/10 + ( (SignalInfo_p->C_N % 10)>=5 ? 1 : 0 );/*Round off C/N*/
  pSTFRONTEND_Info->BitErrorRate = SignalInfo_p->BER;
  pSTFRONTEND_Info->RFLevel = SignalInfo_p->Power;

  }


ST_ErrorCode_t demod_stv0913_Tracking(STFRONTEND_Handle_t Handle, BOOL *IsLocked)
{
 ST_ErrorCode_t Error=ST_NO_ERROR;
 STFRONTEND_InstanceDbase_t *Inst;
 FE_STV0913_Handle_t DemodHandle;
 FE_STV0913_SignalInfo_t SignalInfo;
 FE_Sat_TrackingInfo_t TrackingInfo;
 STFRONTEND_SAT_SearchResult_t *pSTFRONTEND_Info;
// U32 PacketsErrorCount =0 /*, TotalPacketsCount  */;
 
 Inst = STFRONTEND_GetDrvInst();
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
  memset(&TrackingInfo,'\0',sizeof(FE_Sat_TrackingInfo_t));

 DemodHandle = Inst[Handle].Media_DBase.sat.DemodInst.DemodInternalParams;
 pSTFRONTEND_Info = &Inst[Handle].CurrentTunerInfo.STFRONTEND_SearchResult.SatSearchRes;


if(Inst[Handle].taskData.AcqMode == STFRONTEND_SETFREQUENCY)
 {

    *IsLocked = FE_STV0913_Status(DemodHandle);
     //Sometimes while tracking L=0 ;so wait a little and retry
	if((*IsLocked ==FALSE) &&( Inst[Handle].Flags.FirstTunerInfoCall) )
	{
	        WAIT_N_MS(200);
		*IsLocked = FE_STV0913_Status(DemodHandle);
	}
    if(*IsLocked)
      {

        if(Inst[Handle].Flags.FirstTunerInfoCall)  /* First tracking state after a successfull acquisition Get full params update*/
        {
        /*Check the tuner if the tuner is 6120 then make call to 6120_HMR_Filter*/
        if((Inst[Handle].InitParm->TunerType == STFRONTEND_TUNER_STV6120_Tuner1) ||(Inst[Handle].InitParm->TunerType == STFRONTEND_TUNER_STV6120_Tuner2))
	     Error=FE_STV0913_STV6120_HMRFilter(DemodHandle);
       /*	printf("\n<%s>>>If first tunerInfo calll ok case",__func__);*/
        Error  |= FE_STV0913_GetSignalInfo(DemodHandle,&SignalInfo);
		Inst[Handle].CurrentTunerInfo.Frequency = SignalInfo.Frequency;
		/*printf("\n<%s>Freq=%d",__func__,Inst[Handle].CurrentTunerInfo.Frequency);*/
             SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s Calling GetSignalInfo-------",__FUNCTION__,Inst[Handle].STFRONTEND_Name);
       Update_913_SearchResult( pSTFRONTEND_Info,&SignalInfo); 
 	}
       else
      {

        Error = FE_STV0913_Tracking(DemodHandle, &TrackingInfo);
        pSTFRONTEND_Info->SignalQuality = TrackingInfo.C_N/10 + ( (TrackingInfo.C_N % 10)>=5 ? 1 : 0 );/*Round off C/N*/

	if(Inst[Handle].Flags.BER_Algo != STFRONTEND_BER_ALGO_APPLI_BERWINDOW)
        pSTFRONTEND_Info->BitErrorRate = TrackingInfo.BER;

	pSTFRONTEND_Info->RFLevel = TrackingInfo.Power;
        Inst[Handle].CurrentTunerInfo.Frequency = TrackingInfo.Frequency_IF;
	/*printf("\n<%s>Freq=%d",__func__,Inst[Handle].CurrentTunerInfo.Frequency);*/
      }
   }
}
else
{
    SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s Calling GetSignalInfo-------\n",__FUNCTION__,Inst[Handle].STFRONTEND_Name);
    Error = FE_STV0913_GetSignalInfo(DemodHandle,&SignalInfo);
    if(!Error) {
	if((Inst[Handle].taskData.AcqMode == STFRONTEND_BLINDSCAN) && (SignalInfo.Locked ==FALSE))
	  *IsLocked =TRUE;
	else
	  *IsLocked = SignalInfo.Locked;

   Inst[Handle].CurrentTunerInfo.Frequency = SignalInfo.Frequency;
Update_913_SearchResult( pSTFRONTEND_Info,&SignalInfo); 
 // printf("\n<%s>Freq=%d Lock =%d\n",__func__,Inst[Handle].CurrentTunerInfo.Frequency,SignalInfo.Locked);


   }
}
 if((!(*IsLocked)) && (Inst[Handle].Media_DBase.sat.SearchParams.SearchAlgo != STFRONTEND_BLIND_SEARCH)) /*If lock failed ;except in BlindScan*/
 {
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s \n -----LLA Lock failed-------",__FUNCTION__,Inst[Handle].STFRONTEND_Name);

  STFRONTEND_Sat_ResetSearchResults(Handle);
 }

  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s \n -----LLA Signal INFO-------",__FUNCTION__,Inst[Handle].STFRONTEND_Name);
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s IF = %u kHz",__FUNCTION__,Inst[Handle].STFRONTEND_Name, SignalInfo.Frequency);
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s SymbolRate = %d",__FUNCTION__,Inst[Handle].STFRONTEND_Name, SignalInfo.SymbolRate);
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s Pilots = %d (valid for DVBS2-0: OFF, 1: ON)",__FUNCTION__,Inst[Handle].STFRONTEND_Name,SignalInfo.Pilots);
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s Framelength = %d (valid for DVBS2-0: Long, 1: Short)",__FUNCTION__,Inst[Handle].STFRONTEND_Name, SignalInfo.FrameLength);
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s ModCode = %d (valid for DVBS2)",__FUNCTION__,Inst[Handle].STFRONTEND_Name, SignalInfo.ModCode);
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s Spectrum= %d (0: Normal, 1 :Swapped)",__FUNCTION__,Inst[Handle].STFRONTEND_Name, SignalInfo.Spectrum);
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s RollOff = %d (0: 0.35, 1: 0.25, 2 :0.20, 3: Unknown",__FUNCTION__,Inst[Handle].STFRONTEND_Name, SignalInfo.RollOff);
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s Q = %d",__FUNCTION__,Inst[Handle].STFRONTEND_Name, SignalInfo.C_N);
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s BER = %d *10e-7",__FUNCTION__,Inst[Handle].STFRONTEND_Name,SignalInfo.BER);
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,INFO,LEVEL0,"%s %s RFlevel = %d",__FUNCTION__,Inst[Handle].STFRONTEND_Name, SignalInfo.Power);

 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
 return(Error);

}

ST_ErrorCode_t demod_stv0913_Unlock(STFRONTEND_Handle_t Handle)
{
 ST_ErrorCode_t Error;
 STFRONTEND_InstanceDbase_t *Inst;

 Inst= STFRONTEND_GetDrvInst();
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);

 Error = FE_STV0913_Unlock(Inst[Handle].Media_DBase.sat.DemodInst.DemodInternalParams);
 
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
 return(Error);
}

ST_ErrorCode_t  get_diseqc_mode_913(FE_STV0913_Handle_t Handle,FE_Sat_DiseqC_TxMode *Mode_p, unsigned char *tx_data, U32 *Nbdata, STFRONTEND_DiSEqCSendPacket_t *pDiSEqCSendPacket)
{U8 i=0;
ST_ErrorCode_t Error=ST_NO_ERROR;
 FE_STV0913_InternalParams_t	*pParams = NULL;
		
  pParams = (FE_STV0913_InternalParams_t	*) Handle;
 
 switch (pDiSEqCSendPacket->DiSEqCCommandType)
{ 
	case STFRONTEND_DiSEqC_CONTINUOUS_TONE_BURST_ON: /*Equivalent to continous mode*/
	*Mode_p = FE_SAT_22KHZ_Continues; 
	tx_data[0] = 2;
	*Nbdata = 1;
	break;  
	case STFRONTEND_DiSEqC_TONE_BURST_SEND_0_UNMODULATED:   /*send of 0 for 12.5 ms ;continuous tone*/  /*Equivalent to MODE SA*/
		if(pParams->DiSEqC_ENV_Mode_Selection)
			*Mode_p  = FE_SAT_DISEQC_3_3_ENVELOP; 		
		else
		       *Mode_p  = FE_SAT_DISEQC_3_3_PWM; /*Writing 0x011 in DISTX_MODE--*/
		tx_data[0] = 0;/*Writing one byte of value 0x00 */
		*Nbdata = 1;
		break; 
	case STFRONTEND_DiSEqC_TONE_BURST_SEND_0_MODULATED:     /*0-2/3 duty cycle tone*/
		tx_data[0] = 0;
		if(pParams->DiSEqC_ENV_Mode_Selection)
			*Mode_p = FE_SAT_DISEQC_2_3_ENVELOP;	
		else
			*Mode_p = FE_SAT_DISEQC_2_3_PWM;					/*DISTX_MODE --0x010*/
		*Nbdata = 1;
	break; 
	case STFRONTEND_DiSEqC_TONE_BURST_SEND_1:     /*1-1/3 duty cycle tone modulated*/ /*Equivalent to MODE SB*/ 
		/* *Mode_p  = 0x2;*/  
       		if(pParams->DiSEqC_ENV_Mode_Selection)
			*Mode_p = FE_SAT_DISEQC_2_3_ENVELOP;	
		else
			*Mode_p = FE_SAT_DISEQC_2_3_PWM;		  			/*DISTX_MODE --0x010*/
		tx_data[0] = 0xff;							/*One byte of 0xff */
		*Nbdata = 1;
       	
	break;
	case STFRONTEND_DiSEqC_COMMAND:       /*DiSEqC (1.2/2)command */  
		if(pParams->DiSEqC_ENV_Mode_Selection)
			*Mode_p = FE_SAT_DISEQC_2_3_ENVELOP;	
		else
	*Mode_p = FE_SAT_DISEQC_2_3_PWM; 
	for(i=0;i<pDiSEqCSendPacket->TotalNoOfBytes;i++)
           tx_data[i]=*(pDiSEqCSendPacket->pFrmAddCmdData+i);
	*Nbdata = pDiSEqCSendPacket->TotalNoOfBytes;
	break;   
	default:  
	Error=ST_ERROR_BAD_PARAMETER; 
	break;
} 
 return (Error);
}



ST_ErrorCode_t demod_stv0913_DiseqcSendReceive(STFRONTEND_Handle_t Handle,STFRONTEND_DiSEqCSendPacket_t *pDiSEqCSendPacket,STFRONTEND_DiSEqCResponsePacket_t *pDiSEqCResponsePacket ,STFRONTEND_DISEQC_Version_t DiSEqCVersion )
{
 ST_ErrorCode_t Error=ST_NO_ERROR;
 STFRONTEND_InstanceDbase_t *Inst;
 LNB_Config_t      *LnbConfig_p; 
 FE_Sat_DiseqC_TxMode Mode;
 unsigned char tx_data[8];
 U32 NbData=1;
 	FE_STV0913_Handle_t DemodHandle;  
	
	
 Inst = STFRONTEND_GetDrvInst();
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
 
 LnbConfig_p = (LNB_Config_t*)(Inst[Handle].Media_DBase.sat.LnbInst.hLnb->pData), sizeof(LNB_Config_t);
DemodHandle = Inst[Handle].Media_DBase.sat.DemodInst.DemodInternalParams;    
	
 /* If framing byte wants response (DiSEqC 2.0) report error*/
  Error = get_diseqc_mode_913(DemodHandle,&Mode, tx_data, &NbData, pDiSEqCSendPacket);
 if(Error != ST_NO_ERROR)
 	return Error;
   
Error = FE_STV0913_DiseqcInit(DemodHandle ,Mode);
	if (Error !=ST_NO_ERROR) 	
		return(Error);

 	/*Configure LNB to transmit */
 	LnbConfig_p->ToneState= STFRONTEND_LNB_TONE_22KHZ;
 	Error = (Inst[Handle].Media_DBase.sat.LnbInst.LnbDB->lnb_tone_enable)(Handle, LnbConfig_p);
  	if(Error != ST_NO_ERROR)
 		return Error;
  
	/*The Tx*/ 
	Error = FE_STV0913_DiseqcSend(DemodHandle ,tx_data ,NbData);

	/*...THE Receiver Section...*/
	 LnbConfig_p->ToneState= STFRONTEND_LNB_TONE_OFF;
	 Error = (Inst[Handle].Media_DBase.sat.LnbInst.LnbDB->lnb_tone_enable)(Handle,LnbConfig_p );
	
	/*Rx Section*/
	if(DiSEqCVersion>STFRONTEND_DISEQC_VER_1_2)
	{	
		pDiSEqCResponsePacket->TotalBytesReceived =0;
		/*initialise with zero*/	
		Error = FE_STV0913_DiseqcReceive(DemodHandle ,(U8 *)pDiSEqCResponsePacket->ReceivedFrmAddData ,&NbData);
		pDiSEqCResponsePacket->TotalBytesReceived=(unsigned char)NbData;
	} 

	LnbConfig_p->TtxMode = STFRONTEND_LNB_TX ;
	Error = (Inst[Handle].Media_DBase.sat.LnbInst.LnbDB->lnb_SetConfig)(Handle,LnbConfig_p );

 
 SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
 return(Error);

}


ST_ErrorCode_t demod_stv0913_Abort(STFRONTEND_Handle_t Handle, BOOL Abort)
{
  ST_ErrorCode_t Error=ST_NO_ERROR;
  STFRONTEND_InstanceDbase_t *Inst;
  
  Inst = STFRONTEND_GetDrvInst(); 

  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);

  /*Call LLA Abort function */
  Error =FE_STV0913_SetAbortFlag(Inst[Handle].Media_DBase.sat.DemodInst.DemodInternalParams, Abort);

  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
  return(Error);
}
ST_ErrorCode_t demod_stv0913_DiSEqC_Env_Mode_Select(STFRONTEND_Handle_t Handle, STFRONTEND_Control_t ControlFunction, void *InParams, void *OutParams)
{
  ST_ErrorCode_t Error=ST_NO_ERROR;
  STFRONTEND_InstanceDbase_t *Inst;
  FE_STV0913_InternalParams_t	*pParams = NULL; 
  Inst = STFRONTEND_GetDrvInst(); 
  
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
   
  pParams = (FE_STV0913_InternalParams_t	*) Inst[Handle].Media_DBase.sat.DemodInst.DemodInternalParams;
  /*Setting the ENVelope mode flag to be TRUE*/
  pParams->DiSEqC_ENV_Mode_Selection=TRUE;
 
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
  return(Error);
}

/************************************************************

   Conversion utilities & other STV0913 utility functions
   
*************************************************************/
FE_Sat_Rate_t STFRONTEND_to_913LLA_FECMap(STFRONTEND_FECRate_t FECRate)
{
  FE_Sat_Rate_t lla_fecrate = FE_SAT_PR_UNKNOWN;
  switch(FECRate)
  {
    case STFRONTEND_FEC_ALL: lla_fecrate = FE_SAT_PR_UNKNOWN;
    default:
    break;
    case STFRONTEND_FEC_1_2: lla_fecrate = FE_SAT_PR_1_2;
    break;
    case STFRONTEND_FEC_2_3: lla_fecrate = FE_SAT_PR_2_3;
    break; 
    case STFRONTEND_FEC_3_4: lla_fecrate = FE_SAT_PR_3_4;
    break; 
    case STFRONTEND_FEC_4_5: lla_fecrate = FE_SAT_PR_4_5;
    break; 
    case STFRONTEND_FEC_5_6: lla_fecrate = FE_SAT_PR_5_6;
    break; 
    case STFRONTEND_FEC_6_7: lla_fecrate = FE_SAT_PR_6_7;
    break; 
    case STFRONTEND_FEC_7_8: lla_fecrate = FE_SAT_PR_7_8;
    break; 
    case STFRONTEND_FEC_8_9: lla_fecrate = FE_SAT_PR_8_9;
    break; 
  }
  return(lla_fecrate);
}

FE_Sat_Rate_t STFRONTEND_to_913LLA_ModulationMap(STFRONTEND_Modulation_t Modulation)
{
  FE_Sat_Modulation_t lla_modulation = FE_SAT_MOD_UNKNOWN;
  switch(Modulation)
  {
    case STFRONTEND_MOD_ALL:lla_modulation=FE_SAT_MOD_UNKNOWN;
    default:
    break;	 	
    case STFRONTEND_MOD_QPSK: lla_modulation = FE_SAT_MOD_QPSK;
    break;
    case STFRONTEND_MOD_8PSK: lla_modulation = FE_SAT_MOD_8PSK;
    break;
    case STFRONTEND_MOD_16APSK: lla_modulation = FE_SAT_MOD_16APSK;
    break;
    case STFRONTEND_MOD_32APSK: lla_modulation = FE_SAT_MOD_32APSK;
    break;
  }
  return(lla_modulation);
}

STFRONTEND_FECRate_t STV0913_GetFECRate_n_Map(const FE_Sat_TrackingStandard_t FoundStandard,const FE_Sat_Rate_t FoundPunctureRate,const STFRONTEND_ModeCode_t modCode)
{
 #define SIZE_TABLE 8 /*At the time of writing this code only 8 puncture rates supported by LLA*/
 const int  STV0913_ModCodLUT[30] = {
           /*STFRONTEND_MODECODE_DUMMY_PLF, STFRONTEND_MOD_NONE,*/    STFRONTEND_FEC_NONE,
    /*STFRONTEND_MODECODE_QPSK_14,   STFRONTEND_MOD_QPSK, */   STFRONTEND_FEC_1_4,
    /*STFRONTEND_MODECODE_QPSK_13,   STFRONTEND_MOD_QPSK,*/    STFRONTEND_FEC_1_3,
    /*STFRONTEND_MODECODE_QPSK_25,    STFRONTEND_MOD_QPSK,*/   STFRONTEND_FEC_2_5,
    /*STFRONTEND_MODECODE_QPSK_12,    STFRONTEND_MOD_QPSK,*/   STFRONTEND_FEC_1_2,
    /*STFRONTEND_MODECODE_QPSK_35,    STFRONTEND_MOD_QPSK,*/   STFRONTEND_FEC_3_5,
    /*STFRONTEND_MODECODE_QPSK_23,    STFRONTEND_MOD_QPSK,*/   STFRONTEND_FEC_2_3,
    /*STFRONTEND_MODECODE_QPSK_34,    STFRONTEND_MOD_QPSK, */  STFRONTEND_FEC_3_4,
    /*STFRONTEND_MODECODE_QPSK_45,    STFRONTEND_MOD_QPSK,*/   STFRONTEND_FEC_4_5,
    /*STFRONTEND_MODECODE_QPSK_56,    STFRONTEND_MOD_QPSK,*/   STFRONTEND_FEC_5_6,
    /*STFRONTEND_MODECODE_QPSK_89,    STFRONTEND_MOD_QPSK,*/   STFRONTEND_FEC_8_9,
    /*STFRONTEND_MODECODE_QPSK_910,   STFRONTEND_MOD_QPSK, */  STFRONTEND_FEC_9_10,
    /*STFRONTEND_MODECODE_8PSK_35,    STFRONTEND_MOD_8PSK,*/   STFRONTEND_FEC_3_5,
    /*STFRONTEND_MODECODE_8PSK_23,    STFRONTEND_MOD_8PSK,*/   STFRONTEND_FEC_2_3,
    /*STFRONTEND_MODECODE_8PSK_34,    STFRONTEND_MOD_8PSK,*/   STFRONTEND_FEC_3_4,
    /*STFRONTEND_MODECODE_8PSK_56,    STFRONTEND_MOD_8PSK,*/   STFRONTEND_FEC_5_6,
    /*STFRONTEND_MODECODE_8PSK_89,   STFRONTEND_MOD_8PSK,*/    STFRONTEND_FEC_8_9,
    /*STFRONTEND_MODECODE_8PSK_910,   STFRONTEND_MOD_8PSK,*/   STFRONTEND_FEC_9_10,
    /*STFRONTEND_MODECODE_16APSK_23,  STFRONTEND_MOD_16APSK,*/ STFRONTEND_FEC_2_3,
    /*STFRONTEND_MODECODE_16APSK_34,  STFRONTEND_MOD_16APSK,*/ STFRONTEND_FEC_3_4,
    /*STFRONTEND_MODECODE_16APSK_45,  STFRONTEND_MOD_16APSK,*/ STFRONTEND_FEC_4_5,
    /*STFRONTEND_MODECODE_16APSK_56,  STFRONTEND_MOD_16APSK,*/ STFRONTEND_FEC_5_6,
    /*STFRONTEND_MODECODE_16APSK_89,  STFRONTEND_MOD_16APSK,*/ STFRONTEND_FEC_8_9,
    /*STFRONTEND_MODECODE_16APSK_910, STFRONTEND_MOD_16APSK,*/ STFRONTEND_FEC_9_10,
    /*STFRONTEND_MODECODE_32APSK_34,  STFRONTEND_MOD_32APSK,*/ STFRONTEND_FEC_3_4,
    /*STFRONTEND_MODECODE_32APSK_45, STFRONTEND_MOD_32APSK,*/  STFRONTEND_FEC_4_5,
    /*STFRONTEND_MODECODE_32APSK_56, STFRONTEND_MOD_32APSK,*/  STFRONTEND_FEC_5_6,
    /*STFRONTEND_MODECODE_32APSK_89,  STFRONTEND_MOD_32APSK,*/ STFRONTEND_FEC_8_9,
    /*STFRONTEND_MODECODE_32APSK_910,STFRONTEND_MOD_32APSK,*/  STFRONTEND_FEC_9_10,
   /*  STFRONTEND_MODECODE_UNKNOWN, STFRONTEND_MOD_NONE, */STFRONTEND_FEC_NONE
 };

 U32 ConvertTable[SIZE_TABLE] = {
       STFRONTEND_FEC_1_2,/*Only these FEC Rates are supported by STV0913*/
       STFRONTEND_FEC_2_3,
       STFRONTEND_FEC_3_4,
       STFRONTEND_FEC_4_5,
       STFRONTEND_FEC_5_6,
       STFRONTEND_FEC_6_7,
       STFRONTEND_FEC_7_8,
       STFRONTEND_FEC_8_9
      };

 switch(FoundStandard)
 {
  case FE_SAT_DVBS2_STANDARD:
   return(STV0913_ModCodLUT[modCode]);
  break;

  case FE_SAT_DVBS1_STANDARD:
  case FE_SAT_DSS_STANDARD:
  default:
   if(FoundPunctureRate>SIZE_TABLE-1)
    return STFRONTEND_FEC_NONE;
   else
    return ConvertTable[FoundPunctureRate];
  
 }
 
}

STFRONTEND_FECType_t STV0913_FoundStandardMap(const FE_Sat_TrackingStandard_t Standard)
{
 U32 ConvertTable[3] = {
       STFRONTEND_FEC_DVBS1_STANDARD,
       STFRONTEND_FEC_DVBS2_STANDARD,
       STFRONTEND_FEC_DSS_STANDARD
      };
 if(Standard>=3)/*At the time of writing this code only 3 standards supported by 913*/
  return STFRONTEND_FEC_UNKNOWN_STANDARD;
 else
  return ConvertTable[Standard];
 
}
TUNER_Model_t STV0913_TunerModelMap(const STFRONTEND_TunerType_t STFRONTEND_TunerType)
{
 
 switch(STFRONTEND_TunerType)
 {
  case STFRONTEND_TUNER_STB6000:
   return(TUNER_STB6000);
  case STFRONTEND_TUNER_STV6110:
   return(TUNER_STV6110);
 case STFRONTEND_TUNER_STV6111:
   return(TUNER_STV6111);
  case STFRONTEND_TUNER_STB6100:
   return(TUNER_STB6100);
  case STFRONTEND_TUNER_STV6120_Tuner1:
   return(TUNER_STV6120_Tuner1);
   case STFRONTEND_TUNER_STV6120_Tuner2:
   return(TUNER_STV6120_Tuner2);
  case STFRONTEND_TUNER_STV6120:
   return(TUNER_STV6120);
  case STFRONTEND_TUNER_STV6130:
   return(TUNER_STV6130);
  default: 
   return(-1);
 }
   
}

FE_Sat_Tuner_t STV0913_FindTunerType(const STFRONTEND_TunerType_t STFRONTEND_TunerType)
{ 
 switch(STFRONTEND_TunerType)
 {
  case STFRONTEND_AUTO_STB6000:
   return(FE_SAT_AUTO_STB6000);
  case STFRONTEND_AUTO_STV6110:
   return(FE_SAT_AUTO_STV6110);
  case STFRONTEND_AUTO_STV6111:
   return(FE_SAT_AUTO_STV6111);
  case STFRONTEND_AUTO_STB6100:
   return(FE_SAT_AUTO_STB6100);
  case STFRONTEND_AUTO_STV6120_Tuner1:
   return(FE_SAT_AUTO_STV6120_Tuner1);
    case STFRONTEND_AUTO_STV6120_Tuner2:
   return(FE_SAT_AUTO_STV6120_Tuner2);
  case STFRONTEND_AUTO_STV6120:
   return(FE_SAT_AUTO_STV6120);
  default: 
   return(FE_SAT_SW_TUNER);
 }
}

STFRONTEND_Modulation_t STV0913_GetModulation(const STFRONTEND_FECType_t FoundStandard, const STFRONTEND_ModeCode_t modCode)
{
 const int  STV0913_ModCodLUT[30][2] = {
           {/*STFRONTEND_MODECODE_DUMMY_PLF,*/ STFRONTEND_MOD_NONE,    STFRONTEND_FEC_NONE},
    {/*STFRONTEND_MODECODE_QPSK_14,*/   STFRONTEND_MOD_QPSK,    STFRONTEND_FEC_1_4},
    {/*STFRONTEND_MODECODE_QPSK_13,*/   STFRONTEND_MOD_QPSK,    STFRONTEND_FEC_1_3},
    {/*STFRONTEND_MODECODE_QPSK_25,*/    STFRONTEND_MOD_QPSK,   STFRONTEND_FEC_2_5},
    {/*STFRONTEND_MODECODE_QPSK_12,*/    STFRONTEND_MOD_QPSK,   STFRONTEND_FEC_1_2},
    {/*STFRONTEND_MODECODE_QPSK_35,*/    STFRONTEND_MOD_QPSK,   STFRONTEND_FEC_3_5},
    {/*STFRONTEND_MODECODE_QPSK_23,*/    STFRONTEND_MOD_QPSK,   STFRONTEND_FEC_2_3},
    {/*STFRONTEND_MODECODE_QPSK_34,*/    STFRONTEND_MOD_QPSK,   STFRONTEND_FEC_3_4},
    {/*STFRONTEND_MODECODE_QPSK_45,*/    STFRONTEND_MOD_QPSK,   STFRONTEND_FEC_4_5},
    {/*STFRONTEND_MODECODE_QPSK_56,*/    STFRONTEND_MOD_QPSK,   STFRONTEND_FEC_5_6},
    {/*STFRONTEND_MODECODE_QPSK_89,*/    STFRONTEND_MOD_QPSK,   STFRONTEND_FEC_8_9},
    {/*STFRONTEND_MODECODE_QPSK_910,*/   STFRONTEND_MOD_QPSK,   STFRONTEND_FEC_9_10},
    {/*STFRONTEND_MODECODE_8PSK_35,*/    STFRONTEND_MOD_8PSK,   STFRONTEND_FEC_3_5},
    {/*STFRONTEND_MODECODE_8PSK_23,*/    STFRONTEND_MOD_8PSK,   STFRONTEND_FEC_2_3},
    {/*STFRONTEND_MODECODE_8PSK_34,*/    STFRONTEND_MOD_8PSK,   STFRONTEND_FEC_3_4},
    {/*STFRONTEND_MODECODE_8PSK_56,*/    STFRONTEND_MOD_8PSK,   STFRONTEND_FEC_5_6},
    {/*STFRONTEND_MODECODE_8PSK_89,*/    STFRONTEND_MOD_8PSK,   STFRONTEND_FEC_8_9},
    {/*STFRONTEND_MODECODE_8PSK_910,*/   STFRONTEND_MOD_8PSK,   STFRONTEND_FEC_9_10},
    {/*STFRONTEND_MODECODE_16APSK_23,*/  STFRONTEND_MOD_16APSK, STFRONTEND_FEC_2_3},
    {/*STFRONTEND_MODECODE_16APSK_34,*/  STFRONTEND_MOD_16APSK, STFRONTEND_FEC_3_4},
    {/*STFRONTEND_MODECODE_16APSK_45,*/  STFRONTEND_MOD_16APSK, STFRONTEND_FEC_4_5},
    {/*STFRONTEND_MODECODE_16APSK_56,*/  STFRONTEND_MOD_16APSK, STFRONTEND_FEC_5_6},
    {/*STFRONTEND_MODECODE_16APSK_89,*/  STFRONTEND_MOD_16APSK, STFRONTEND_FEC_8_9},
    {/*STFRONTEND_MODECODE_16APSK_910,*/ STFRONTEND_MOD_16APSK, STFRONTEND_FEC_9_10},
    {/*STFRONTEND_MODECODE_32APSK_34,*/  STFRONTEND_MOD_32APSK, STFRONTEND_FEC_3_4},
    {/*STFRONTEND_MODECODE_32APSK_45,*/  STFRONTEND_MOD_32APSK, STFRONTEND_FEC_4_5},
    {/*STFRONTEND_MODECODE_32APSK_56,*/  STFRONTEND_MOD_32APSK, STFRONTEND_FEC_5_6},
    {/*STFRONTEND_MODECODE_32APSK_89,*/  STFRONTEND_MOD_32APSK, STFRONTEND_FEC_8_9},
    {/*STFRONTEND_MODECODE_32APSK_910,*/ STFRONTEND_MOD_32APSK, STFRONTEND_FEC_9_10},
    {/*  STFRONTEND_MODECODE_UNKNOWN,*/ STFRONTEND_MOD_NONE, STFRONTEND_FEC_NONE}
 };

 switch(FoundStandard)
 {
  case STFRONTEND_FEC_DVBS2_STANDARD:
   return(STV0913_ModCodLUT[modCode][0]);
  break;

  case STFRONTEND_FEC_DVBS1_STANDARD:
  case STFRONTEND_FEC_DSS_STANDARD:
  default:
   return(STFRONTEND_MOD_QPSK);
 }
 
}
ST_ErrorCode_t demod_stv0913_SetContinous_Tone(STFRONTEND_Handle_t Handle,STFRONTEND_LNBToneState_t tonestate)
{
  ST_ErrorCode_t Error=ST_NO_ERROR;
  STFRONTEND_InstanceDbase_t *Inst;
  FE_STV0913_Handle_t DemodHandle;
  
  Inst = STFRONTEND_GetDrvInst(); 
  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_IN,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);

  DemodHandle = Inst[Handle].Media_DBase.sat.DemodInst.DemodInternalParams;
  
  /*Call LLA Abort function */
  Error =FE_STV0913_SetContinous_Tone(DemodHandle,tonestate);

  SET_TRACE(STFRONTEND_DRIVERS_SAT_DEMOD_CID,API_OUT,LEVEL0,"%s",Inst[Handle].STFRONTEND_Name);
  return(Error);
}
