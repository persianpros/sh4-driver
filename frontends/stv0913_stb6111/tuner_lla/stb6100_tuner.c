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
 @File   stb6100_tuner.c
 @brief



*/
#ifndef ST_OSLINUX
	#include "string.h"
	#include "stdlib.h"
#endif

#include "globaldefs.h"
#include "fe_sat_tuner.h"
#include "stb6100_tuner.h"


	/* RFMagic STB6100 tuner definition */

		/*	TESTR0	*/
#define RSTB6100_TESTR0		0x0000
#define FSTB6100_TESTR0		0x00000001

/*	VCO	*/
#define RSTB6100_VCO		0x0001
#define FSTB6100_OSCH		0x00010080
#define FSTB6100_OCK		0x00010060
#define FSTB6100_ODIV		0x00010010
#define FSTB6100_OSM		0x0001000f

/*	NI	*/
#define RSTB6100_NI		0x0002
#define FSTB6100_NI		0x000200ff

/*	NF_LSB	*/
#define RSTB6100_NF_LSB		0x0003
#define FSTB6100_NF_LSB		0x000300ff

/*	K	*/
#define RSTB6100_K		0x0004
#define FSTB6100_K		0x000400c0
#define FSTB6100_PSD2		0x00040004
#define FSTB6100_NF_MSB		0x00040003

/*	G	*/
#define RSTB6100_G		0x0005
#define FSTB6100_GCT		0x000500e0
#define FSTB6100_G		0x0005000f

/*	F	*/
#define RSTB6100_F		0x0006
#define FSTB6100_F		0x0006001f

/*	DLB	*/
#define RSTB6100_DLB		0x0007
#define FSTB6100_DLB		0x00070038

/*	TEST1	*/
#define RSTB6100_TEST1		0x0008
#define FSTB6100_TEST1		0x000800ff

/*	TEST2	*/
#define RSTB6100_TEST2		0x0009
#define FSTB6100_FCCK		0x00090040

/*	LPEN	*/
#define RSTB6100_LPEN		0x000a
#define FSTB6100_BEN		0x000a0080
#define FSTB6100_OSCP		0x000a0040
#define FSTB6100_SYNP		0x000a0020
#define FSTB6100_LPEN		0x000a0010

/*	TEST3	*/
#define RSTB6100_TEST3		0x000b
#define FSTB6100_TEST3		0x000b00ff




/*Nbregs and Nbfields moved to stb6100_tuner.h : Change ///AG*/



static U32 STB6K_LOOKUP[11][3]=		{/* low			high	  osm */
										{950000,		999999,		0xA},/*Added {} to remove warnings Change :AG*/
										{1000000,	1075000,	0xC},
										{1075001,	1199999,	0x0},
										{1200000,	1299999,	0x1},
										{1300000,	1369999,	0x2},
										{1370000,	1469999,	0x4},
										{1470000,	1529999,	0x5},
										{1530000,	1649999,	0x6},
										{1650000,	1799999,	0x8},
										{1800000,	1949999,	0xA},
										{1950000,	2150000,	0xC}
									};


TUNER_Error_t STB6100_TunerInit(void *pTunerInit_v,STCHIP_Handle_t *TunerHandle)
{
	SAT_TUNER_Params_t	hTunerParams = NULL;
	STCHIP_Info_t	ChipInfo;
	STCHIP_Handle_t hTuner = NULL;
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t pTunerInit = (SAT_TUNER_Params_t)pTunerInit_v; /*Changed to retain common function prototype for all Tuners: Change ///AG*/

    STCHIP_Register_t DefSTB6100Val[STB6100_NBREGS]=
	{
		{RSTB6100_TESTR0	,0x81	},
		{RSTB6100_VCO		,0x66	},
		{RSTB6100_NI		,0x39	},
		{RSTB6100_NF_LSB	,0xd0	},
		{RSTB6100_K			,0x3c	},
		{RSTB6100_G			,0x37	},
		{RSTB6100_F			,0xd4	},
		{RSTB6100_DLB		,0xcc	},
		{RSTB6100_TEST1		,0x8f	},
		{RSTB6100_TEST2		,0x0d	},
		{RSTB6100_LPEN		,0xfb	},
		{RSTB6100_TEST3		,0xde	}
	};

    /*
    **   REGISTER CONFIGURATION
    **     ----------------------
    */
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
    	hTunerParams->DemodModel = pTunerInit->DemodModel;  /* Demod Model used with this tuner */

		/*fix Coverity CID 15040*/		
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

	    ChipInfo.NbRegs   = STB6100_NBREGS;
		ChipInfo.NbFields = STB6100_NBFIELDS;
   		ChipInfo.ChipMode = STCHIP_MODE_NOSUBADR_RD;
		ChipInfo.WrStart  = RSTB6100_VCO;
		ChipInfo.WrSize   = 11;
		ChipInfo.RdStart  = RSTB6100_TESTR0;
		ChipInfo.RdSize   = 12;

		#ifdef HOST_PC
		    hTuner = ChipOpen(&ChipInfo);
		    (*TunerHandle) = hTuner;
		#endif

		#ifdef CHIP_STAPI
		    hTuner = *TunerHandle;/*obtain hTuner : change ///AG*/
		    memcpy(hTuner, &ChipInfo, sizeof(STCHIP_Info_t)); /* Copy the ChipInfo to hTuner: Change ///AG*/
		#endif
		if((*TunerHandle)== NULL)
		{
			error=TUNER_INVALID_HANDLE;
		}
		if(hTuner != NULL)
		{
			/*******************************
	    	**   CHIP REGISTER MAP IMAGE INITIALIZATION
	    	**     ----------------------
	    	********************************/
			ChipUpdateDefaultValues(hTuner,DefSTB6100Val);

		 	/*******************************
			 **   REGISTER CONFIGURATION
			 **     ----------------------
			 ********************************/

			 /*Update the clock divider before registers initialization */
			/*Allowed values 1,2,4 and 8*/
			if(pTunerInit->OutputClockDivider  == 0) /*Tuner output clock not used ==> stop it*/
			{
				ChipSetFieldImage(hTuner,FSTB6100_K,3);
			}
			else	/*Tuner output clock used ==> enable it*/
			{
				ChipSetFieldImage(hTuner,FSTB6100_K,0);
			}

		    /*I2C registers initialization*/

 			error=STB6100_TunerWrite(hTuner);
		}
	}
	else
		error=TUNER_INVALID_HANDLE;

	return error;
}

TUNER_Error_t STB6100_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

 	if(hTuner)
 	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
 	}
 	else
 	{
		error=TUNER_INVALID_HANDLE;	
 	}

 	if(hTunerParams && !hTuner->Error)	
	{
		if(StandbyOn) /*Power down ON*/
		{
			ChipSetFieldImage(hTuner,FSTB6100_BEN,0);
			ChipSetFieldImage(hTuner,FSTB6100_OSCP,0);
			ChipSetFieldImage(hTuner,FSTB6100_SYNP,0);
		}
		else	/*Power down OFF*/
		{
			ChipSetFieldImage(hTuner,FSTB6100_BEN,1);
			ChipSetFieldImage(hTuner,FSTB6100_OSCP,1);
			ChipSetFieldImage(hTuner,FSTB6100_SYNP,1);
		}
		error = STB6100_TunerWrite(hTuner);

	}
	return error;
}

TUNER_Error_t STB6100_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

	U32 frequency;
	U32 stepsize;
	U32 nbsteps;
	U32 divider;
	U32 swallow;

	U8 vco,u8;

 	if(hTuner)
 	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
 	}
 	else
 	{
		error=TUNER_INVALID_HANDLE;	
 	}

 	if(hTunerParams && !hTuner->Error)	
	{


				ChipSetFieldImage(hTuner,FSTB6100_ODIV,Frequency <= 1075000);

				if((Frequency<=1075000)||(Frequency>1325000))
					ChipSetFieldImage(hTuner,FSTB6100_PSD2,1);
				else
					ChipSetFieldImage(hTuner,FSTB6100_PSD2,0);

			  	u8=0;
				while(!INRANGE(STB6K_LOOKUP[u8][0],Frequency,STB6K_LOOKUP[u8][1])&& (u8<10)) u8++;
				if(Frequency<950000)
					vco=STB6K_LOOKUP[0][2];
				else if(Frequency>2150000)
					vco=STB6K_LOOKUP[10][2];
				else
					vco=STB6K_LOOKUP[u8][2];

				ChipSetFieldImage(hTuner,FSTB6100_OSM,vco);

				frequency=2*Frequency*(ChipGetFieldImage(hTuner,FSTB6100_ODIV)+1); /*Flo=2*Ftuner*(ODIV+1)*/


				hTunerParams->Reference/=1000; /*Refrence in Khz*/
				/*flo=Fxtal*(DIV2+1)*(Ni+Nf/2^9)*/

				/*Ni = floor (fVCO / (fXTAL * P))*/
				divider=(frequency/hTunerParams->Reference)/(ChipGetFieldImage(hTuner,FSTB6100_PSD2)+1);

				/*round ((fVCO / (fXTAL * P) - Ni) * 2^9)*/
				stepsize=frequency-divider*(ChipGetFieldImage(hTuner,FSTB6100_PSD2)+1)*(hTunerParams->Reference); /**/
				nbsteps=(stepsize*PowOf2(9))/((ChipGetFieldImage(hTuner,FSTB6100_PSD2)+1)*(hTunerParams->Reference));
				swallow=nbsteps;

				ChipSetFieldImage(hTuner,FSTB6100_NI,divider);
				ChipSetFieldImage(hTuner,FSTB6100_NF_LSB,(swallow&0xFF));
				ChipSetFieldImage(hTuner,FSTB6100_NF_MSB,(swallow>>8));
				hTunerParams->Reference*=1000;
				ChipSetFieldImage(hTuner,FSTB6100_LPEN,0);	/* PLL loop disabled */
				ChipSetFieldImage(hTuner,FSTB6100_OSCH,1);	/* VCO search enabled */
				ChipSetFieldImage(hTuner,FSTB6100_OCK,3);		/* VCO search clock off */
				error = STB6100_TunerWrite(hTuner);

				ChipSetFieldImage(hTuner,FSTB6100_LPEN,1);	/* PLL loop enabled */
				error = STB6100_TunerWrite(hTuner);

				ChipSetFieldImage(hTuner,FSTB6100_OSCH,1);	/* VCO search enabled */
				ChipSetFieldImage(hTuner,FSTB6100_OCK,0);	    /* VCO fast search*/
				error = STB6100_TunerWrite(hTuner);

				ChipWaitOrAbort(hTuner,5);	/* The LO locking time is 5ms maximum */


				ChipSetFieldImage(hTuner,FSTB6100_OSCH,0);	/* VCO search disabled */
				ChipSetFieldImage(hTuner,FSTB6100_OCK,3);		/* VCO search clock off */

				ChipSetFieldImage(hTuner,FSTB6100_FCCK,0);	/* LPF BW setting clock disabled */
				error = STB6100_TunerWrite(hTuner);

	}

	return error;
}

U32 STB6100_TunerGetFrequency(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL; 
	U32 frequency = 0;
	U32 divider = 0;
	U32 swallow;
	U32 psd2;


	/*Fix Coverity Warning*/
 	if(hTuner)
 	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
 	}
 	else
 	{
		error=TUNER_INVALID_HANDLE;	
 	}

 	if(hTunerParams && !hTuner->Error)	
	{

				STB6100_TunerRead(hTuner);
				swallow=(ChipGetFieldImage(hTuner,FSTB6100_NF_MSB)<<8)  /*Nf val*/
						+ChipGetFieldImage(hTuner,FSTB6100_NF_LSB);


				/*Ni = floor (fVCO / (fXTAL * P))*/
				divider=ChipGetFieldImage(hTuner,FSTB6100_NI); /*NI val*/
				psd2=ChipGetFieldImage(hTuner,FSTB6100_PSD2);

				frequency=(((1+psd2)*(hTunerParams->Reference/1000)*swallow)/PowOf2(9));
				frequency+=(hTunerParams->Reference/1000) * (divider)*(1+psd2);
				/*Flo=Fxtal*P*(Ni+Nf/2^9) . P=DIV2+1 */

				frequency=frequency/((ChipGetFieldImage(hTuner,FSTB6100_ODIV)+1)*2);
	}
	return frequency;
}


TUNER_Error_t STB6100_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	U8 u8;
		/*Deleted unused variable 'filter' & 'i' to remove warnings: Change ///AG*/


 	if(hTuner)
 	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
 	}
 	else
 	{
		error=TUNER_INVALID_HANDLE;	
 	}

 	if(hTunerParams && !hTuner->Error)	
	{
				if((Bandwidth/2) > 36000000)   /*F[4:0] BW/2 max =31+5=36 mhz for F=31*/
					u8 = 31;
				else if((Bandwidth/2) < 5000000) /* BW/2 min = 5Mhz for F=0 */
					u8 = 0;
				else							 /*if 5 < BW/2 < 36*/
					u8 = (Bandwidth/2)/1000000 - 5;

				ChipSetFieldImage(hTuner,FSTB6100_FCCK,1); /* FCL_Clk=FXTAL/FCL=1Mhz */
				STB6100_TunerWrite(hTuner);
				ChipSetFieldImage(hTuner,FSTB6100_F,u8);
				error = STB6100_TunerWrite(hTuner);
				ChipWaitOrAbort(hTuner,1);  /* Don't forget a short delay (1 ms), before setting FCCK to 0 */
				ChipSetFieldImage(hTuner,FSTB6100_FCCK,0); /*FCL turned off*/
				error=STB6100_TunerWrite(hTuner);

	}

	return error;
}

U32 STB6100_TunerGetBandwidth(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	U32 bandwidth = 0;
	U8 u8 = 0;
	
 	/*Fix Coverity Warning*/
 	if(hTuner)
 	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
 	}
 	else
 	{
		error=TUNER_INVALID_HANDLE;	
 	}

	if(hTunerParams && !hTuner->Error)	
	{
				STB6100_TunerRead(hTuner);
				u8 = ChipGetFieldImage(hTuner,FSTB6100_F);

				bandwidth = (u8+5)*2000000;	/* x2 for ZIF tuner BW/2=F+5 Mhz*/
	}

	return bandwidth;
}

TUNER_Error_t STB6100_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

 	if(hTuner)
 	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
 	}
 	else
 	{
		error=TUNER_INVALID_HANDLE;	
 	}

 	if(hTunerParams && !hTuner->Error)	
	{
				ChipSetFieldImage(hTuner,FSTB6100_G,(Gain/2)+7);
				error = STB6100_TunerWrite(hTuner);
	}

	return error;
}

S32 STB6100_TunerGetGain(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	S32 gain=0;
	
	/*Fix Coverity Warning*/
 	if(hTuner)
 	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
 	}
 	else
 	{
		error=TUNER_INVALID_HANDLE;	
 	}

 	if(hTunerParams && !hTuner->Error)	
	{
		gain=ChipGetFieldImage(hTuner,FSTB6100_G);
		gain = (gain-7)*2;
	}

	return gain;
}


TUNER_Error_t STB6100_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider)
{
	/*sets the crystal oscillator divisor value, for the output clock
	Divider =:
	0 ==> Tuner output clock not used
	1 ==> divide by 1
	2 ==> divide by 2
	4 ==> divide by 4
	8 ==> divide by 8
	*/
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

 	if(hTuner)
 	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
 	}
 	else
 	{
		error=TUNER_INVALID_HANDLE;	
 	}

 	if(hTunerParams && !hTuner->Error)	
	{
		if(Divider == 0) /*Tuner output clock not used ==> stop it*/
		{
			ChipSetFieldImage(hTuner,FSTB6100_K,3);
			error = STB6100_TunerWrite(hTuner);
		}
		else	/*Tuner output clock used ==> enable it*/
		{
			ChipSetFieldImage(hTuner,FSTB6100_K,0);
			error = STB6100_TunerWrite(hTuner);
		}
	}

	return error;
}

TUNER_Error_t STB6100_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn)
{
	return TUNER_NO_ERR;
}
BOOL STB6100_TunerGetAttenuator(STCHIP_Handle_t hTuner)
{
	return FALSE;
}

BOOL STB6100_TunerGetStatus(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	BOOL locked = FALSE;
	/*Deleted unused variable 'u8' & 'error' to remove warnings: Change ///AG*/

	 /*Fix Coverity Warning*/
 	if(hTuner)
 	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
 	}
 	else
 	{
		error=TUNER_INVALID_HANDLE;	
 	}

 	if(hTunerParams && !hTuner->Error)	
	{
		STB6100_TunerRead(hTuner);

				if(!hTuner->Error)
				{
					STB6100_TunerRead(hTuner);
					locked = ChipGetFieldImage(hTuner,FSTB6100_TESTR0);
				}
	}

	return locked;
}

TUNER_Error_t STB6100_TunerWrite(STCHIP_Handle_t hTuner)
{
 TUNER_Error_t error = TUNER_NO_ERR;
 STCHIP_Error_t chipError=CHIPERR_NO_ERROR;
 SAT_TUNER_Params_t hTunerParams = NULL;
 U8 i;
	hTunerParams =  (SAT_TUNER_Params_t) hTuner->pData;
	
	 /*Fix Coverity Warning*/
 	if(hTuner)
 	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
 	}
 	else
 	{
		error=TUNER_INVALID_HANDLE;	
 	}

 	
	if(hTunerParams && !hTuner->Error)	
 	{
      if(((hTunerParams->DemodModel == DEMOD_STI7111) || (hTunerParams->DemodModel == DEMOD_STIH205)) && (hTuner->RepeaterHost->ChipID <0x20))
      {
         for(i=hTuner->WrStart;i<hTuner->WrSize;i++) /*Write one register at a time for 7111 cut1.0*/
           chipError |= ChipSetRegisters(hTuner,i, 1);
      }
      else
      {
        chipError=ChipSetRegisters(hTuner,hTuner->WrStart,hTuner->WrSize);
      }

 	switch (chipError)
     {
      case CHIPERR_NO_ERROR:
       error = TUNER_NO_ERR;
      break;

      case CHIPERR_INVALID_HANDLE:
       error = TUNER_INVALID_HANDLE;
      break;

      case CHIPERR_INVALID_REG_ID:
       error = TUNER_INVALID_REG_ID;
      break;

      case CHIPERR_INVALID_FIELD_ID:
       error = TUNER_INVALID_FIELD_ID;
      break;

      case CHIPERR_INVALID_FIELD_SIZE:
       error = TUNER_INVALID_FIELD_SIZE;
      break;

      case CHIPERR_I2C_NO_ACK:
      default:
       error = TUNER_I2C_NO_ACK;
      break;

      case CHIPERR_I2C_BURST:
       error = TUNER_I2C_BURST;
      break;
     }

 }

 return error;
}

TUNER_Error_t STB6100_TunerRead(STCHIP_Handle_t hTuner)
{
 TUNER_Error_t error = TUNER_NO_ERR;
 STCHIP_Error_t chipError=CHIPERR_NO_ERROR;
 SAT_TUNER_Params_t hTunerParams = NULL;

           	/*Fix Coverity Warning*/
 	if(hTuner)
 	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
 	}
 	else
 	{
		error=TUNER_INVALID_HANDLE;	
 	}

	if(hTunerParams && !hTuner->Error)	

 {
      if(((hTunerParams->DemodModel == DEMOD_STI7111) || (hTunerParams->DemodModel == DEMOD_STIH205)) && (hTuner->RepeaterHost->ChipID <0x20))
      {
       chipError=ChipGetRegisters(hTuner,hTuner->RdStart,1);/*Only read first register*/
      }
      else
         chipError=ChipGetRegisters(hTuner,hTuner->RdStart,hTuner->RdSize);

      switch (chipError)
      {
       case CHIPERR_NO_ERROR:
        error = TUNER_NO_ERR;
       break;

       case CHIPERR_INVALID_HANDLE:
        error = TUNER_INVALID_HANDLE;
       break;

       case CHIPERR_INVALID_REG_ID:
        error = TUNER_INVALID_REG_ID;
       break;

       case CHIPERR_INVALID_FIELD_ID:
        error = TUNER_INVALID_FIELD_ID;
       break;

       case CHIPERR_INVALID_FIELD_SIZE:
        error = TUNER_INVALID_FIELD_SIZE;
       break;

       case CHIPERR_I2C_NO_ACK:
       default:
        error = TUNER_I2C_NO_ACK;
       break;

       case CHIPERR_I2C_BURST:
        error = TUNER_I2C_BURST;
       break;
      }

   }

 return error;
}

TUNER_Model_t STB6100_TunerGetModel(STCHIP_Handle_t hChip)
{
	TUNER_Model_t model = TUNER_NULL;

	if(hChip && hChip->pData)
	{
		model = ((SAT_TUNER_Params_t) (hChip->pData))->Model;
	}

	return model;
}

void STB6100_TunerSetIQ_Wiring(STCHIP_Handle_t hChip, S8 IQ_Wiring)/*For generic function prototype: change ///AG*/
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->IQ_Wiring = (TUNER_IQ_t)IQ_Wiring;
	}
}

S8 STB6100_TunerGetIQ_Wiring(STCHIP_Handle_t hChip)/*For generic function prototype: change ///AG*/
{
	TUNER_IQ_t wiring = TUNER_IQ_NORMAL;

	if(hChip && hChip->pData)
	{
		wiring = ((SAT_TUNER_Params_t) (hChip->pData))->IQ_Wiring;
	}

	return (S8)wiring;
}

void STB6100_TunerSetReferenceFreq(STCHIP_Handle_t hChip, U32 Reference)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->Reference = Reference;
	}
}

U32 STB6100_TunerGetReferenceFreq(STCHIP_Handle_t hChip)
{
	U32 reference = 0;

	if(hChip && hChip->pData)
	{
		reference = ((SAT_TUNER_Params_t) (hChip->pData))->Reference;
	}

	return reference;
}

void STB6100_TunerSetIF_Freq(STCHIP_Handle_t hChip, U32 IF)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->IF = IF;
	}
}

U32 STB6100_TunerGetIF_Freq(STCHIP_Handle_t hChip)
{
	U32 ifreq = 0;

	if(hChip && hChip->pData)
	{
		ifreq = ((SAT_TUNER_Params_t) (hChip->pData))->IF;
	}

	return ifreq;
}

void STB6100_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->BandSelect = (TUNER_WIDEBandS_t)BandSelect;
	}
}

U8 STB6100_TunerGetBandSelect(STCHIP_Handle_t hChip)
{
	U32 ifreq = 0;

	if(hChip && hChip->pData)
	{
		ifreq = ((SAT_TUNER_Params_t) (hChip->pData))->BandSelect;
	}

	return (U8)ifreq;
}


TUNER_Error_t STB6100_TunerTerm(STCHIP_Handle_t hTuner)
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




