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
 @File   stvvglna.c                                             
 @brief                                                                         
                                                                               
                                                                               
                                                                               
*/
#ifndef ST_OSLINUX  
	#include "string.h"
	#include "stdlib.h"
#endif

#include "globaldefs.h"
#include "stvvglna.h"



STCHIP_Error_t STVVGLNA_Init(void *pVGLNAInit_v,STCHIP_Handle_t *ChipHandle)
{
	STCHIP_Info_t	ChipInfo;
	STCHIP_Handle_t hChip = NULL;
	STCHIP_Error_t error=CHIPERR_NO_ERROR; 
	SAT_VGLNA_Params_t pVGLNAInit = (SAT_VGLNA_Params_t)pVGLNAInit_v; 
 	 
	STCHIP_Register_t	DefSTVVGLNAVal[STVVGLNA_NBREGS]=
 	 {
 	 	{RSTVVGLNA_REG0	,0x20	},
 	 	{RSTVVGLNA_REG1	,0x0F	},
 	 	{RSTVVGLNA_REG2	,0x50	},
 	 	{RSTVVGLNA_REG3	,0x20	}
 	 };
 	 
 	 

    #ifdef CHIP_STAPI
	/*Fix coverity Warning*/
	if(*ChipHandle)
	{
		memcpy(&ChipInfo, *ChipHandle,sizeof(STCHIP_Info_t)); /*Copy settings already contained in hTuner to ChipInfo: Change ///AG*/
    	}
	else
	{
		error=CHIPERR_INVALID_HANDLE;
	}
	#endif
	
    if(pVGLNAInit != NULL)
    {
 		if(strlen((char*)pVGLNAInit->Name)<MAXNAMESIZE)
		{
			strcpy((char *)ChipInfo.Name,(char*)pVGLNAInit->Name);	/* Tuner name */
		}
		else
		{
			error=TUNER_TYPE_ERR;
		}
    	       ChipInfo.RepeaterHost = pVGLNAInit->RepeaterHost;		/* Repeater host */
		ChipInfo.RepeaterFn = pVGLNAInit->RepeaterFn;			/* Repeater enable/disable function */
		ChipInfo.Repeater = pVGLNAInit->Repeater;								/* Tuner need to enable repeater */
		ChipInfo.I2cAddr = pVGLNAInit->I2cAddress;			/* Init tuner I2C address */
 
		ChipInfo.NbRegs   = STVVGLNA_NBREGS;
		ChipInfo.NbFields = STVVGLNA_NBFIELDS;
		ChipInfo.ChipMode = STCHIP_MODE_SUBADR_8;
		ChipInfo.WrStart  = RSTVVGLNA_REG0;
		ChipInfo.WrSize   = STVVGLNA_NBREGS;
		ChipInfo.RdStart  = RSTVVGLNA_REG0;
		ChipInfo.RdSize   = STVVGLNA_NBREGS;
		ChipInfo.pData  = NULL;

		#ifdef HOST_PC				
			hChip = ChipOpen(&ChipInfo);
			(*ChipHandle) = hChip;
		#endif
	
		#ifdef CHIP_STAPI
			hChip = *ChipHandle;
			memcpy(hChip, &ChipInfo, sizeof(STCHIP_Info_t)); 

		#endif	
	
		if((*ChipHandle)== NULL)
			error=CHIPERR_INVALID_HANDLE; 
	
		if(hChip != NULL)
		{
			/*******************************
			**   CHIP REGISTER MAP IMAGE INITIALIZATION
			**     ----------------------
			********************************/
			ChipUpdateDefaultValues(hChip,DefSTVVGLNAVal);
			hChip->ChipID=ChipGetOneRegister(hChip,RSTVVGLNA_REG0);
			error =hChip->Error;
				
		}

	}
	else
		error=CHIPERR_INVALID_HANDLE;
	
	return error;

}


STCHIP_Error_t STVVGLNA_SetStandby(STCHIP_Handle_t hChip, U8 StandbyOn)
{	
	STCHIP_Error_t error=CHIPERR_NO_ERROR;
	
	if(hChip!=NULL)
	{
		if(StandbyOn)
		{
			error=ChipSetField(hChip,FSTVVGLNA_LNAGCPWD,1);
		}
		else
		{
			error=ChipSetField(hChip,FSTVVGLNA_LNAGCPWD,0);
		}
		
	}
	else
		error=CHIPERR_INVALID_HANDLE;
	
	return error;
}

STCHIP_Error_t STVVGLNA_GetStatus(STCHIP_Handle_t hChip, U8 *Status)
{
	STCHIP_Error_t error=CHIPERR_NO_ERROR;
	U8 flagAgcLow,flagAgcHigh;
	
	if(hChip!=NULL)
	{
		flagAgcHigh=ChipGetField(hChip,FSTVVGLNA_RFAGCHIGH);
		flagAgcLow=ChipGetField(hChip,FSTVVGLNA_RFAGCLOW);
		
		if(flagAgcHigh)
		{
			(*Status)=VGLNA_RFAGC_HIGH;
		}
		else if(flagAgcLow)
		{
			(*Status)=(U8)VGLNA_RFAGC_LOW;
		}
		else
			(*Status)=(U8)VGLNA_RFAGC_NORMAL; 
		
		/*error=ChipGetError(hChip);	*/
	}
	else
		error=CHIPERR_INVALID_HANDLE;


	return error;
}


STCHIP_Error_t STVVGLNA_GetGain(STCHIP_Handle_t hChip,S32 *Gain)
{

	S32 VGO,
		swlnaGain;
	
	STCHIP_Error_t error=CHIPERR_NO_ERROR;
	

	if(hChip!=NULL)
	{
		VGO=ChipGetField(hChip,FSTVVGLNA_VGO);	
		/*error=ChipGetError(hChip);*/
		
		/*Trig to read the VGO and SWLNAGAIN val*/
		ChipSetFieldImage(hChip,FSTVVGLNA_GETAGC,1);
		ChipSetRegisters(hChip,RSTVVGLNA_REG1,1);
		WAIT_N_MS(5);
		
		/*(31-VGO[4:0]) * 13/31 + (3-SWLNAGAIN[1:0])*6 */ 
		VGO=ChipGetField(hChip,FSTVVGLNA_VGO);
		swlnaGain=ChipGetField(hChip,FSTVVGLNA_SWLNAGAIN);
		(*Gain)=(31-VGO)*13;
		(*Gain)/=31;
		(*Gain)+=(6*(3-swlnaGain));
		 
	}
	else
		error=CHIPERR_INVALID_HANDLE;
	
	return error;
}

STCHIP_Error_t STVVGLNA_Term(STCHIP_Handle_t hChip)
{
	STCHIP_Error_t error = CHIPERR_NO_ERROR;
	
	if(hChip)
	{
		#ifndef CHIP_STAPI 
			if(hChip->pData)	
				free(hChip->pData);
			
			
		#endif
		ChipClose(hChip); /*Changes are required for STAPI as well*/
	}
	
	return error;
}




