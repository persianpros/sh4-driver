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
 @File   d0913.h                                                               
 @brief                                                                         
                                                                               
                                                                               
                                                                               
*/
#ifndef __D0913_H 
 #define __D0913_H
/* Includes ------------------------------------------------------------ */
#include "dbtypes.h" 
#include "stfrontend.h"

/* Exported Types ------------------------------------------------------ */
 
/* Exported Constants -------------------------------------------------- */
 
/* Exported Variables -------------------------------------------------- */
 
/* Exported Macros ----------------------------------------------------- */
 
/* Exported Functions -------------------------------------------------- */
 ST_ErrorCode_t STFRONTEND_DEMOD_STV0913_Install(STFRONTEND_demodDbase_t *Demod, STFRONTEND_DemodType_t DemodType );
 ST_ErrorCode_t STFRONTEND_DEMOD_STV0913_Uninstall(STFRONTEND_demodDbase_t *Demod);
 ST_ErrorCode_t demod_stv0913_Init(STFRONTEND_Handle_t Handle);
 ST_ErrorCode_t demod_stv0913_StandbyMode(STFRONTEND_Handle_t Handle, STFRONTEND_StandByMode_t PowerMode);
 ST_ErrorCode_t demod_stv0913_Open(STFRONTEND_Handle_t Handle);
 ST_ErrorCode_t demod_stv0913_Close(STFRONTEND_Handle_t Handle);
 ST_ErrorCode_t demod_stv0913_Term(STFRONTEND_Handle_t Handle);
 ST_ErrorCode_t demod_stv0913_DiseqcSendReceive(STFRONTEND_Handle_t Handle,STFRONTEND_DiSEqCSendPacket_t *pDiSEqCSendPacket,STFRONTEND_DiSEqCResponsePacket_t *pDiSEqCResponsePacket, STFRONTEND_DISEQC_Version_t DiSEqCVersion );
 ST_ErrorCode_t demod_stv0913_Search(STFRONTEND_Handle_t Handle , BOOL *IsLocked);
 ST_ErrorCode_t demod_stv0913_Tracking(STFRONTEND_Handle_t Handle, BOOL *IsLocked);
 ST_ErrorCode_t demod_stv0913_Unlock(STFRONTEND_Handle_t Handle);
 ST_ErrorCode_t demod_stv0913_Abort(STFRONTEND_Handle_t Handle, BOOL Abort);
 ST_ErrorCode_t demod_stv0913_DiSEqC_Env_Mode_Select(STFRONTEND_Handle_t Handle, STFRONTEND_Control_t ControlFunction, void *InParams, void *OutParams);
 ST_ErrorCode_t  demod_stv0913_SetContinous_Tone(STFRONTEND_Handle_t Handle,STFRONTEND_LNBToneState_t tonestate);

#endif /* #ifndef __D0913_H */
/* End of D0913.h */

