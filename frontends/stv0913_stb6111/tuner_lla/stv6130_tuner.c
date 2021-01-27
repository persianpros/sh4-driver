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
 @File   stv6130_tuner.c
 @brief



*/
#ifndef ST_OSLINUX
	#include "string.h"
	#include "stdlib.h"
#endif

#include "globaldefs.h"
#include "fe_sat_tuner.h"
#include "stv6130_tuner.h"


	/* STV6130 tuner definition */

/*CTRL1*/
#define RSTV6130_CTRL1  0x0000
#define FSTV6130_K  0x000000f8
#define FSTV6130_CODIV  0x00000006
#define FSTV6130_MCLKDIV  0x00000001

/*CTRL2*/
#define RSTV6130_CTRL2  0x0001
#define FSTV6130_DCLOOPOFF_1  0x00010080
#define FSTV6130_ICP_1  0x00010040
#define FSTV6130_SYN_1  0x00010020
#define FSTV6130_REFOUTSEL_1  0x00010010
#define FSTV6130_BBGAIN_1  0x0001000f

/*CTRL3*/
#define RSTV6130_CTRL3  0x0002
#define FSTV6130_NDIV_1_LSB  0x000200ff

/*CTRL4*/
#define RSTV6130_CTRL4  0x0003
#define FSTV6130_PRESC32ON_1  0x00030040
#define FSTV6130_PDIV_1  0x00030030
#define FSTV6130_NDIV_1_MSB  0x0003000f

/*CTRL5*/
#define RSTV6130_CTRL5  0x0004
#define FSTV6130_RCCLFOFF_1  0x00040080
#define FSTV6130_RDIV_1  0x00040060
#define FSTV6130_CF_1  0x0004001f

/*STAT1*/
#define RSTV6130_STAT1  0x0005
#define FSTV6130_CALVCOSTRT_1  0x00050004
#define FSTV6130_CALRCSTRT_1  0x00050002
#define FSTV6130_LOCK_1  0x00050001

/*CTRL6*/
#define RSTV6130_CTRL6  0x0006
#define FSTV6130_PATHON_1  0x00060040
#define FSTV6130_RFB_HGAIN  0x00060020
#define FSTV6130_RFA_HGAIN  0x00060010
#define FSTV6130_RFB_ON  0x00060008
#define FSTV6130_RFA_ON  0x00060004
#define FSTV6130_RFA_1  0x00060001

/*CTRL7*/
#define RSTV6130_CTRL7  0x0007
#define FSTV6130_TEST1  0x000700ff

/*CTRL8*/
#define RSTV6130_CTRL8  0x0008
#define FSTV6130_TEST2  0x000800ff

/*CTRL9*/
#define RSTV6130_CTRL9  0x0009
#define FSTV6130_TEST3  0x000900ff

/*CTRL10*/
#define RSTV6130_CTRL10  0x000a
#define FSTV6130_TEST4  0x000a00ff

/*STAT2*/
#define RSTV6130_STAT2  0x000b
#define FSTV6130_TEST5  0x000b00ff

/*STAT3*/
#define RSTV6130_STAT3  0x000c
#define FSTV6130_TEST6  0x000c00ff

/*STAT4*/
#define RSTV6130_STAT4  0x000d
#define FSTV6130_TEST7  0x000d00ff



TUNER_Error_t STV6130_TunerInit(void *pTunerInit_v,STCHIP_Handle_t *TunerHandle)
{
	SAT_TUNER_Params_t	hTunerParams = NULL;
	STCHIP_Info_t	ChipInfo;
	STCHIP_Handle_t hTuner = NULL;
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t pTunerInit = (SAT_TUNER_Params_t)pTunerInit_v; /*Changed to retain common function prototype for all Tuners: Change ///AG*/

	 STCHIP_Register_t		DefSTV6130Val[STV6130_NBREGS]=
	 {
	 	{RSTV6130_CTRL1		,0x71	},
	 	{RSTV6130_CTRL2		,0x33	},
	 	{RSTV6130_CTRL3		,0x40	},
	 	{RSTV6130_CTRL4		,0x46	},
	 	{RSTV6130_CTRL5		,0x7f	},
	 	{RSTV6130_STAT1		,0x00	},
	 	{RSTV6130_CTRL6		,0x7c	},
	 	{RSTV6130_CTRL7		,0x0c	},
	 	{RSTV6130_CTRL8		,0x8f	},
	 	{RSTV6130_CTRL9		,0x4d	},
	 	{RSTV6130_CTRL10	,0xfb	},
	 	{RSTV6130_STAT2		,0xdb	},
	 	{RSTV6130_STAT3		,0xea	},
	 	{RSTV6130_STAT4		,0x08	}
	 };

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
		/*fix Coverity CID 15049 */		
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

    	ChipInfo.NbRegs   = STV6130_NBREGS;
		ChipInfo.NbFields = STV6130_NBFIELDS;
   		ChipInfo.ChipMode = STCHIP_MODE_SUBADR_8;
    	ChipInfo.WrStart  = RSTV6130_CTRL1;
    	ChipInfo.WrSize   = 14;
    	ChipInfo.RdStart  = RSTV6130_CTRL1;
    	ChipInfo.RdSize   = 14;

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

		if(hTuner != NULL)
		{
			/*******************************
			**   CHIP REGISTER MAP IMAGE INITIALIZATION
			**     ----------------------
			********************************/
			ChipUpdateDefaultValues(hTuner,DefSTV6130Val);

		 	/*******************************
			 **   REGISTER CONFIGURATION
			 **     ----------------------
			  ********************************/

			/*Update the clock divider before registers initialization */
			/*Allowed values 1,2,4 and 8*/
			switch(pTunerInit->OutputClockDivider)
			{
				case 1:
				default:
					ChipSetFieldImage(hTuner,FSTV6130_CODIV,0);
				break;

				case 2:
					ChipSetFieldImage(hTuner,FSTV6130_CODIV,1);
				break;

				case 4:
					ChipSetFieldImage(hTuner,FSTV6130_CODIV,2);
				break;

				case 8:
				case 0:/*Tuner output clock not used then divide by 8 (the STV6130 can not stop completely the out clock)*/
					ChipSetFieldImage(hTuner,FSTV6130_CODIV,3);
				break;

			}

	 		/*I2C registers initialization*/
	 		error=STV6130_TunerWrite(hTuner);
		}

	}
	else
		error=TUNER_INVALID_HANDLE;

	return error;
}

TUNER_Error_t STV6130_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	/*Fix Coverity Warning*/
	if(hTuner)
	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
	}
	if(hTunerParams && !hTuner->Error)	
	{
		if(StandbyOn) /*Power down ON*/
		{
			ChipSetFieldImage(hTuner,FSTV6130_PATHON_1,0);
			ChipSetFieldImage(hTuner,FSTV6130_RFB_ON,0);
			ChipSetFieldImage(hTuner,FSTV6130_RFA_ON,0);
			ChipSetFieldImage(hTuner,FSTV6130_SYN_1,0);
		}
		else	/*Power down OFF*/
		{
			ChipSetFieldImage(hTuner,FSTV6130_PATHON_1,1);
			ChipSetFieldImage(hTuner,FSTV6130_RFB_ON,1);
			ChipSetFieldImage(hTuner,FSTV6130_RFA_ON,1);
			ChipSetFieldImage(hTuner,FSTV6130_SYN_1,1);
		}
		error = STV6130_TunerWrite(hTuner);

	}
	return error;
}


TUNER_Error_t STV6130_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

	U32 divider;
	U32 p,
		rDiv;

	S32 rDivOpt=0;


	U32 Fcomp, Fstep, FstepOpt=0;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams =  (SAT_TUNER_Params_t) hTuner->pData;

	if(hTunerParams && !hTuner->Error)	
	{
		ChipSetField(hTuner,FSTV6130_K,((hTunerParams->Reference/1000000)-16));
		if (Frequency<=750000) /* Low band [250MHz - 750MHz] */
		{
			ChipSetField(hTuner,FSTV6130_RFA_1,~hTunerParams->BandSelect);
		}
		else		/*high Band [950MHz, 2150MHz]*/
		{
			ChipSetField(hTuner,FSTV6130_RFA_1,hTunerParams->BandSelect);
		}


		if (Frequency<=325000) /* in kHz */
		{
			p=3;
		}
		else if (Frequency<=650000)
		{
			p=2;
		}
		else if (Frequency<=1300000)
		{
			p=1;
		}
		else
		{
			p=0;
		}
		ChipSetField(hTuner,FSTV6130_PDIV_1,p);
		/*
		Determine rDiv value which must satisfy
		1.4MHz <= Fcomp <= 4MHz
		and if there are several values of rDiv,
		choose the one which gives the smaller abs(Fstep-1.4MHz).
		*/
		for(rDiv=0;rDiv<=3;rDiv++)
	  	{
			Fcomp=(hTunerParams->Reference/1000)/PowOf2(rDiv+1);
			if((Fcomp>1400)&&(Fcomp<4000))
			{
				Fstep = (hTunerParams->Reference/1000)/(PowOf2(rDiv+1)*PowOf2(p+1));
				if ( ABS ((S32)((S32)1400-(S32)Fstep)) < ABS ((S32)((S32)1400-(S32)FstepOpt))  )
				{
					FstepOpt=Fstep;
					rDivOpt=rDiv;
				}
			}
		}
		divider= ((Frequency)*PowOf2(rDivOpt+1)*PowOf2(p+1))/(hTunerParams->Reference/1000); /*NDiv*/
		if (divider<1024)
		{
			ChipSetField(hTuner,FSTV6130_PRESC32ON_1,0);
		} else
		{
			ChipSetField(hTuner,FSTV6130_PRESC32ON_1,1);
		}
		ChipSetField(hTuner,FSTV6130_RDIV_1,rDivOpt);
		ChipSetField(hTuner,FSTV6130_NDIV_1_MSB,MSB(divider));
		ChipSetField(hTuner,FSTV6130_NDIV_1_LSB,LSB(divider));
		ChipSetField(hTuner,FSTV6130_CALVCOSTRT_1,1);		/* VCO Auto Calibration */
	}

	return error;
}

U32 STV6130_TunerGetFrequency(STCHIP_Handle_t hTuner)
{
	SAT_TUNER_Params_t hTunerParams = NULL;
	U32 frequency = 0;
	U32 nbsteps;
	U32 divider = 0;
	U32 psd2;


             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTunerParams && !hTuner->Error)	
	{
		divider=MAKEWORD(ChipGetField(hTuner,FSTV6130_NDIV_1_MSB),ChipGetField(hTuner,FSTV6130_NDIV_1_LSB));	/*N*/
		psd2=PowOf2(ChipGetField(hTuner,FSTV6130_PDIV_1));	/*P=2^(p+1)*/
		nbsteps=PowOf2(ChipGetField(hTuner,FSTV6130_RDIV_1)); /*R=2^(r+1)*/

		frequency=(divider*(hTunerParams->Reference/1000))/(psd2*nbsteps); /*FLO*/
		frequency /= 4; /*as P=2^(p+1) and R=2^(r+1) divide by 4 */
	}
	return frequency;
}


TUNER_Error_t STV6130_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	U8 u8;
	S32 i=0;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTunerParams && !hTuner->Error)	
	{
		if((Bandwidth/2) > 36000000)   /*F[4:0] BW/2 max =31+5=36 mhz for F=31*/
			u8 = 31;
		else if((Bandwidth/2) < 5000000) /* BW/2 min = 5Mhz for F=0 */
			u8 = 0;
		else							 /*if 5 < BW/2 < 36*/
			u8 = (Bandwidth/2)/1000000 - 5;

		ChipSetField(hTuner,FSTV6130_CF_1,u8);		 /* Set the LPF value */
		ChipSetField(hTuner,FSTV6130_CALRCSTRT_1,1); /* Start LPF auto calibration*/

		i=0;
		while((i<10) && (ChipGetField(hTuner,FSTV6130_CALRCSTRT_1)!=0))
		{
			ChipWaitOrAbort(hTuner,1);	/* wait for LPF auto calibration */
			i++;
		}
	}

	return error;
}

U32 STV6130_TunerGetBandwidth(STCHIP_Handle_t hTuner)
{
	SAT_TUNER_Params_t hTunerParams = NULL;
	U32 bandwidth = 0;
	U8 u8 = 0;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTunerParams && !hTuner->Error)	
	{

		u8 = ChipGetField(hTuner,FSTV6130_CF_1);

		bandwidth = (u8+5)*2000000;	/* x2 for ZIF tuner BW/2=F+5 Mhz*/

	}

	return bandwidth;
}

TUNER_Error_t STV6130_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTunerParams && !hTuner->Error)	
	{
		ChipSetField(hTuner,FSTV6130_BBGAIN_1,(Gain/2));
	}

	return error;
}

S32 STV6130_TunerGetGain(STCHIP_Handle_t hTuner)
{
	SAT_TUNER_Params_t hTunerParams = NULL;
	S32 gain=0;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTunerParams && !hTuner->Error)	
	{

		gain=2*ChipGetField(hTuner,FSTV6130_BBGAIN_1);
	}

	return gain;
}

TUNER_Error_t STV6130_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider)
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

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTunerParams && !hTuner->Error)	
	{
		/*Allowed values 1,2,4 and 8*/
		switch(Divider)
		{
			case 1:
			default:
				ChipSetField(hTuner,FSTV6130_CODIV,0);
			break;

			case 2:
				ChipSetField(hTuner,FSTV6130_CODIV,1);
			break;

			case 4:
				ChipSetField(hTuner,FSTV6130_CODIV,2);
			break;

			case 8:
			case 0:/*Tuner output clock not used then divide by 8 (the STV6130 can not stop completely the out clock)*/
				ChipSetField(hTuner,FSTV6130_CODIV,3);
			break;

		}
	}

	return error;
}

TUNER_Error_t STV6130_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	U32 frequency;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTunerParams && !hTuner->Error)	
	{
		switch(hTunerParams->Model)
		{

			case TUNER_STV6130:
				/*only for 6130 RF Attenuator setting on/off*/
				frequency=STV6130_TunerGetFrequency(hTuner);
				if(AttenuatorOn==TRUE)
				{
					if(frequency<=750000)
					{
						if(hTunerParams->BandSelect == LBRFA_HBRFB)
						{
							/*Low band on RFA, High band on RFB*/
							 ChipSetField(hTuner,FSTV6130_RFA_HGAIN,0);
							 ChipSetField(hTuner,FSTV6130_RFB_HGAIN,1);
						}
						else
						{
							/*Low band on RFB, High band on RFA*/
							 ChipSetField(hTuner,FSTV6130_RFA_HGAIN,1);
							 ChipSetField(hTuner,FSTV6130_RFB_HGAIN,0);
						}
					}
					else
					{
						if(hTunerParams->BandSelect == LBRFA_HBRFB)
						{
							/*Low band on RFA, High band on RFB*/
							 ChipSetField(hTuner,FSTV6130_RFA_HGAIN,1);
							 ChipSetField(hTuner,FSTV6130_RFB_HGAIN,0);
						}
						else
						{
							/*Low band on RFB, High band on RFA*/
							 ChipSetField(hTuner,FSTV6130_RFA_HGAIN,0);
							 ChipSetField(hTuner,FSTV6130_RFB_HGAIN,1);
						}
					}
				}
				else
				{
					ChipSetField(hTuner,FSTV6130_RFA_HGAIN,1);
					ChipSetField(hTuner,FSTV6130_RFB_HGAIN,1);
				}

			default:
			break;
		}
	}

	return error;


}
BOOL STV6130_TunerGetAttenuator(STCHIP_Handle_t hTuner)
{
	SAT_TUNER_Params_t hTunerParams = NULL;
	BOOL attenuator = FALSE;
	U32 frequency;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTunerParams && !hTuner->Error)	
	{
		switch(hTunerParams->Model)
		{
			case TUNER_STV6130:
				frequency=STV6130_TunerGetFrequency(hTuner);
				if(frequency<=750000)
				{
					if(hTunerParams->BandSelect == LBRFA_HBRFB)
					{
						/*Low band on RFA, High band on RFB*/
						if(ChipGetField(hTuner,FSTV6130_RFA_HGAIN)==0)
							attenuator=TRUE;
						else
							attenuator=FALSE;
					}
					else
					{
						/*Low band on RFB, High band on RFA*/
						if(ChipGetField(hTuner,FSTV6130_RFB_HGAIN)==0)
							attenuator=TRUE;
						else
							attenuator=FALSE;
					}
				}
				else
				{
					if(hTunerParams->BandSelect == LBRFA_HBRFB)
					{
						/*Low band on RFA, High band on RFB*/
						if(ChipGetField(hTuner,FSTV6130_RFB_HGAIN)==0)
							attenuator=TRUE;
						else
							attenuator=FALSE;
					}
					else
					{
						/*Low band on RFB, High band on RFA*/
						if(ChipGetField(hTuner,FSTV6130_RFA_HGAIN)==0)
							attenuator=TRUE;
						else
							attenuator=FALSE;
					}
				}
			break;

			default:
			break;
		}
	}
	return attenuator;
}




BOOL STV6130_TunerGetStatus(STCHIP_Handle_t hTuner)
{
	SAT_TUNER_Params_t hTunerParams = NULL;
	BOOL locked = FALSE;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTunerParams && !hTuner->Error)	
	{


		if(!hTuner->Error)
		{
			locked = ChipGetField(hTuner,FSTV6130_LOCK_1);
		}
	}

	return locked;
}

TUNER_Error_t STV6130_TunerWrite(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	STCHIP_Error_t chipError=CHIPERR_NO_ERROR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	U8 i=0;
             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams =  (SAT_TUNER_Params_t) hTuner->pData;

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

TUNER_Error_t STV6130_TunerRead(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	STCHIP_Error_t chipError=CHIPERR_NO_ERROR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	U8 i=0;
             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams =  (SAT_TUNER_Params_t) hTuner->pData;

	if(hTunerParams && !hTuner->Error)	
	{
      if(((hTunerParams->DemodModel == DEMOD_STI7111) || (hTunerParams->DemodModel == DEMOD_STIH205)) && (hTuner->RepeaterHost->ChipID <0x20))
      {
       for(i=hTuner->RdStart;i<hTuner->RdSize;i++) /*Read one register at a time for cut1.0*/
          chipError|=ChipGetRegisters(hTuner, i ,1);
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

TUNER_Model_t STV6130_TunerGetModel(STCHIP_Handle_t hChip)
{
	TUNER_Model_t model = TUNER_NULL;

	if(hChip && hChip->pData)
	{
		model = ((SAT_TUNER_Params_t) (hChip->pData))->Model;
	}

	return model;
}

void STV6130_TunerSetIQ_Wiring(STCHIP_Handle_t hChip, S8 IQ_Wiring)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->IQ_Wiring = (TUNER_IQ_t)IQ_Wiring;
	}
}

S8 STV6130_TunerGetIQ_Wiring(STCHIP_Handle_t hChip)
{
	TUNER_IQ_t wiring = TUNER_IQ_NORMAL;

	if(hChip && hChip->pData)
	{
		wiring = ((SAT_TUNER_Params_t) (hChip->pData))->IQ_Wiring;
	}

	return (S8)wiring;
}

void STV6130_TunerSetReferenceFreq(STCHIP_Handle_t hChip, U32 Reference)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->Reference = Reference;
	}
}

U32 STV6130_TunerGetReferenceFreq(STCHIP_Handle_t hChip)
{
	U32 reference = 0;

	if(hChip && hChip->pData)
	{
		reference = ((SAT_TUNER_Params_t) (hChip->pData))->Reference;
	}

	return reference;
}

void STV6130_TunerSetIF_Freq(STCHIP_Handle_t hChip, U32 IF)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->IF = IF;
	}
}

U32 STV6130_TunerGetIF_Freq(STCHIP_Handle_t hChip)
{
	U32 ifreq = 0;

	if(hChip && hChip->pData)
	{
		ifreq = ((SAT_TUNER_Params_t) (hChip->pData))->IF;
	}

	return ifreq;
}

void STV6130_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->BandSelect = (TUNER_WIDEBandS_t)BandSelect;
	}
}

U8 STV6130_TunerGetBandSelect(STCHIP_Handle_t hChip)
{
	U32 ifreq = 0;

	if(hChip && hChip->pData)
	{
		ifreq = ((SAT_TUNER_Params_t) (hChip->pData))->BandSelect;
	}

	return (U8)ifreq;
}


TUNER_Error_t STV6130_TunerTerm(STCHIP_Handle_t hTuner)
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




