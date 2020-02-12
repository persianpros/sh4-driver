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
 @File   stvvglna.h                                             
 @brief                                                                         
                                                                               
                                                                               
                                                                               
*/
#ifndef H_STVVGLNA
	#define H_STVVGLNA
	
	#include "chip.h"

	#if defined(CHIP_STAPI) /*For STAPI use only*/
		#include "dbtypes.h"	/*for Install function*/
	#endif

/*REG0*/
#define RSTVVGLNA_REG0  0x0000
#define FSTVVGLNA_LNA_IDENT  0x00000080
#define FSTVVGLNA_CUT_IDENT  0x00000060
#define FSTVVGLNA_RELEASE_ID  0x00000010
#define FSTVVGLNA_AGCTUPD  0x00000008
#define FSTVVGLNA_AGCTLOCK  0x00000004
#define FSTVVGLNA_RFAGCHIGH  0x00000002
#define FSTVVGLNA_RFAGCLOW  0x00000001

/*REG1*/
#define RSTVVGLNA_REG1  0x0001
#define FSTVVGLNA_LNAGCPWD  0x00010080
#define FSTVVGLNA_GETOFF  0x00010040
#define FSTVVGLNA_GETAGC  0x00010020
#define FSTVVGLNA_VGO  0x0001001f

/*REG2*/
#define RSTVVGLNA_REG2  0x0002
#define FSTVVGLNA_PATH2OFF  0x00020080
#define FSTVVGLNA_RFAGCPREF  0x00020070
#define FSTVVGLNA_PATH1OFF  0x00020008
#define FSTVVGLNA_RFAGCMODE  0x00020007

/*REG3*/
#define RSTVVGLNA_REG3  0x0003
#define FSTVVGLNA_SELGAIN  0x00030080
#define FSTVVGLNA_LCAL  0x00030070
#define FSTVVGLNA_RFAGCUPDATE  0x00030008
#define FSTVVGLNA_RFAGCCALSTART  0x00030004
#define FSTVVGLNA_SWLNAGAIN  0x00030003

	#define STVVGLNA_NBREGS	4
	#define STVVGLNA_NBFIELDS	20	
	
	typedef enum
	{
		VGLNA_RFAGC_HIGH,
		VGLNA_RFAGC_LOW,
		VGLNA_RFAGC_NORMAL
		
	}SAT_VGLNA_STATUS;
	
	typedef struct
    {
		ST_String_t 		Name;		/* Tuner name */
		U8 					I2cAddress;			/* Tuner I2C address */
		U32            		Reference;	/* reference frequency (Hz) */ 
		
		BOOL Repeater;
		STCHIP_Handle_t 	RepeaterHost;
		STCHIP_RepeaterFn_t RepeaterFn;

		FE_DEMOD_Model_t	DemodModel;		/* Demod used with the tuner */ 
		
		#ifdef CHIP_STAPI
			U32 TopLevelHandle; /*Used in STTUNER */
		#endif
		void *additionalParams;
    }
    SAT_VGLNA_InitParams_t;
	
	typedef SAT_VGLNA_InitParams_t *SAT_VGLNA_Params_t; 

	
	/* functions --------------------------------------------------------------- */

	/* create instance of chip register mappings */
	
	STCHIP_Error_t STVVGLNA_Init(void *pVGLNAInit,STCHIP_Handle_t *ChipHandle);
	
	STCHIP_Error_t STVVGLNA_SetStandby(STCHIP_Handle_t hChip, U8 StandbyOn); 
	STCHIP_Error_t STVVGLNA_GetStatus(STCHIP_Handle_t hChip, U8 *Status);
	STCHIP_Error_t STVVGLNA_GetGain(STCHIP_Handle_t hChip,S32 *Gain) ;
	
	

	STCHIP_Error_t STVVGLNA_Term(STCHIP_Handle_t hChip);
	
	
	#if defined(CHIP_STAPI) /*For STAPI use only*/
		//ST_ErrorCode_t STFRONTEND_STVVGLNA_Install(STFRONTEND_tunerDbase_t *Tuner, STFRONTEND_TunerType_t TunerType);
		//ST_ErrorCode_t STFRONTEND_STVVGLNA_Uninstall(STFRONTEND_tunerDbase_t *Tuner);
	#endif

#endif


