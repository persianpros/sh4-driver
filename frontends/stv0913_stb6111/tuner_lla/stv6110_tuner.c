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
 @File   stv6110_tuner.c
 @brief



*/
#ifndef ST_OSLINUX
	#include "string.h"
	#include "stdlib.h"
#endif

#include "globaldefs.h"
#include "fe_sat_tuner.h"
#include "stv6110_tuner.h"


	/* STV6110 tuner definition */
/*CTRL1*/
#define RSTV6110_CTRL1  0x0000
#define FSTV6110_K  0x000000f8
#define FSTV6110_LPT  0x00000004
#define FSTV6110_RX  0x00000002
#define FSTV6110_SYN  0x00000001

/*CTRL2*/
#define RSTV6110_CTRL2  0x0001
#define FSTV6110_CO_DIV  0x000100c0
#define FSTV6110_REFOUTSEL  0x00010010
#define FSTV6110_BBGAIN  0x0001000f

/*TUNING0*/
#define RSTV6110_TUNING0  0x0002
#define FSTV6110_NDIV_LSB  0x000200ff

/*TUNING1*/
#define RSTV6110_TUNING1  0x0003
#define FSTV6110_RDIV  0x000300c0
#define FSTV6110_PRESC32ON  0x00030020
#define FSTV6110_DIV4SEL  0x00030010
#define FSTV6110_NDIV_MSB  0x0003000f

/*CTRL3*/
#define RSTV6110_CTRL3  0x0004
#define FSTV6110_DCLOOP_OFF  0x00040080
#define FSTV6110_RCCLKOFF  0x00040040
#define FSTV6110_ICP  0x00040020
#define FSTV6110_CF  0x0004001f

/*STAT1*/
#define RSTV6110_STAT1  0x0005
#define FSTV6110_TEST1  0x000500f8
#define FSTV6110_CALVCOSTRT  0x00050004
#define FSTV6110_CALRCSTRT  0x00050002
#define FSTV6110_LOCKPLL  0x00050001

/*STAT2*/
#define RSTV6110_STAT2  0x0006
#define FSTV6110_TEST2  0x000600ff

/*STAT3*/
#define RSTV6110_STAT3  0x0007
#define FSTV6110_TEST3  0x000700ff

/*Nbregs and Nbfields moved to stv6110_tuner.h: Change ///AG */

static FE_LLA_LOOKUP_t FE_STV6110_Gain_LookUp =	{		              
															69,       
															{   
																{	6832	,	0	             },  /*	68.32	*/
																{	6732	,	14360	         },  /*	67.32	*/
																{	6636	,	18060	         },  /*	66.36	*/
																{	6536	,	20461	         },  /*	65.36	*/
																{	6436	,	22260	         },  /*	64.36	*/
																{	6336	,	23895	         },  /*	63.36	*/
																{	6236	,	25200	         },  /*	62.36	*/
																{	6136	,	26366	         },  /*	61.36	*/
																{	6036	,	27400	         },  /*	60.36	*/
																{	5936	,	28350	         },  /*	59.36	*/
																{	5836	,	29347	         },  /*	58.36	*/
																{	5736	,	30176	         },  /*	57.36	*/
																{	5636	,	30975	         },  /*	56.36	*/
																{	5536	,	31695	         },  /*	55.36	*/
																{	5436	,	32380	         },  /*	54.36	*/
																{	5336	,	33071	         },  /*	53.36	*/
																{	5236	,	33710	         },  /*	52.36	*/
																{	5136	,	34320	         },  /*	51.36	*/
																{	5036	,	34880	         },  /*	50.36	*/
																{	4936	,	35440	         },  /*	49.36	*/
																{	4836	,	36050	         },  /*	48.36	*/
																{	4736	,	36560	         },  /*	47.36	*/
																{	4636	,	37055	         },  /*	46.36	*/
																{	4536	,	37520	         },  /*	45.36	*/
																{	4436	,	37980	         },  /*	44.36	*/
																{	4336	,	38512	         },  /*	43.36	*/
																{	4236	,	38944	         },  /*	42.36	*/
																{	4136	,	39383	         },  /*	41.36	*/
																{	4036	,	39808	         },  /*	40.36	*/
																{	3936	,	40208	         },  /*	39.36	*/
																{	3836	,	40656	         },  /*	38.36	*/
																{	3736	,	41055	         },  /*	37.36	*/
																{	3636	,	41444	         },  /*	36.36	*/
																{	3536	,	41823	         },  /*	35.36	*/
																{	3436	,	42198	         },  /*	34.36	*/
																{	3336	,	42512	         },  /*	33.36	*/
																{	3236	,	42881	         },  /*	32.36	*/
																{	3136	,	43248	         },  /*	31.36	*/
																{	3036	,	43247	         },  /*	30.36	*/
																{	2936	,	43600	         },  /*	29.36	*/
																{	2836	,	44338	         },  /*	28.36	*/
																{	2736	,	44687	         },  /*	27.36	*/
																{	2636	,	45024	         },  /*	26.36	*/
																{	2536	,	45360	         },  /*	25.36	*/
																{	2436	,	45696	         },  /*	24.36	*/
																{	2336	,	46063	         },  /*	23.36	*/
																{	2236	,	46400	         },  /*	22.36	*/
																{	2136	,	46735	         },  /*	21.36	*/
																{	2036	,	47055	         },  /*	20.36	*/
																{	1936	,	47375	         },  /*	19.36	*/
																{	1836	,	47727	         },  /*	18.36	*/
																{	1736	,	48047	         },  /*	17.36	*/
																{	1636	,	48367	         },  /*	16.36	*/
																{	1536	,	48677	         },  /*	15.36	*/
																{	1436	,	49000	         },  /*	14.36	*/
																{	1336	,	49311	         },  /*	13.36	*/
																{	1236	,	49631	         },  /*	12.36	*/
																{	1136	,	49935	         },  /*	11.36	*/
																{	1036	,	50255	         },  /*	10.36	*/
																{	936		,	50560	         },  /*	9.36	*/
																{	836		,	50880	         },  /*	8.36	*/
																{	736		,	51184	         },  /*	7.36	*/
																{	636		,	51486	         },  /*	6.36	*/
																{	536		,	51775	         },  /*	5.36	*/
																{	436		,	52064	         },  /*	4.36	*/
																{	336		,	52384	         },  /*	3.36	*/
																{	236		,	52687	         },  /*	2.36	*/
																{	136		,	52974	         },  /*	1.36	*/
																{	40		,	53263	         }  /*	0.4	*/
															}
														};

TUNER_Error_t STV6110_TunerInit(void *pTunerInit_v,STCHIP_Handle_t *TunerHandle)
{
	SAT_TUNER_Params_t	hTunerParams = NULL;
	STCHIP_Info_t	ChipInfo;
	STCHIP_Handle_t hTuner = NULL;
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t pTunerInit = (SAT_TUNER_Params_t)pTunerInit_v; /*Changed to retain common function prototype for all Tuners: Change ///AG*/
 	 STCHIP_Register_t		DefSTV6110Val[STV6110_NBREGS]=
 	 {
 	 	{RSTV6110_CTRL1		,0x07	},
 	 	{RSTV6110_CTRL2		,0x11	},
 	 	{RSTV6110_TUNING0	,0xdc	},
 	 	{RSTV6110_TUNING1	,0x85	},
 	 	{RSTV6110_CTRL3		,0x17	},
 	 	{RSTV6110_STAT1		,0x01	},
 	 	{RSTV6110_STAT2		,0xe6	},
 	 	{RSTV6110_STAT3		,0x1e	}
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
	else
	{
		error=TUNER_INVALID_HANDLE;
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

	ChipInfo.NbRegs   = STV6110_NBREGS;
	ChipInfo.NbFields = STV6110_NBFIELDS;
	ChipInfo.ChipMode = STCHIP_MODE_SUBADR_8;
	ChipInfo.WrStart  = RSTV6110_CTRL1;
	ChipInfo.WrSize   = 8;
	ChipInfo.RdStart  = RSTV6110_CTRL1;
	ChipInfo.RdSize   = 8;

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
		ChipUpdateDefaultValues(hTuner,DefSTV6110Val);

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
					ChipSetFieldImage(hTuner,FSTV6110_CO_DIV,0);
				break;

				case 2:
					ChipSetFieldImage(hTuner,FSTV6110_CO_DIV,1);
				break;

				case 4:
					ChipSetFieldImage(hTuner,FSTV6110_CO_DIV,2);
				break;

				case 8:
				case 0: /*Tuner output clock not used then divide by 8 (the 6110 can not stop completely the out clock)*/
					ChipSetFieldImage(hTuner,FSTV6110_CO_DIV,3);
				break;
			}

		/*I2C registers initialization*/
		error=STV6110_TunerWrite(hTuner);
	}

	}
	else
		error=TUNER_INVALID_HANDLE;

	return error;
}

TUNER_Error_t STV6110_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{
		if(StandbyOn) /*Power down ON*/
		{
			ChipSetFieldImage(hTuner,FSTV6110_SYN,0);
			ChipSetFieldImage(hTuner,FSTV6110_RX,0);
			ChipSetFieldImage(hTuner,FSTV6110_LPT,0);
		}
		else	/*Power down OFF*/
		{
			ChipSetFieldImage(hTuner,FSTV6110_SYN,1);
			ChipSetFieldImage(hTuner,FSTV6110_RX,1);
			ChipSetFieldImage(hTuner,FSTV6110_LPT,1);
		}
		error = STV6110_TunerWrite(hTuner);

	}
	return error;
}


TUNER_Error_t STV6110_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

	/*Deleted unused variable 'frequency' to remove warnings: Change ///AG*/
	U32 divider;
	U32 p,
		Presc,
		rDiv,
		r;

	S32 rDivOpt=0,
		pCalcOpt=1000,
		pVal,
		pCalc,
		i;


	/*Coverity CID 15804 fix*/
	if(hTuner)
	{
		hTunerParams =  (SAT_TUNER_Params_t) hTuner->pData;    
	}
	else
	{
		error=TUNER_INVALID_HANDLE;
	}
	if(hTunerParams && !hTuner->Error)	
	{


				ChipSetField(hTuner,FSTV6110_K,((hTunerParams->Reference/1000000)-16));
				if (Frequency<=1023000)
				{
					p=1;
					Presc=0;
				}
				else if (Frequency<=1300000)
				{
					p=1;
					Presc=1;
				}
				else if (Frequency<=2046000)
				{
					p=0;
					Presc=0;
				}
				else
				{
					p=0;
					Presc=1;
				}
				ChipSetField(hTuner,FSTV6110_DIV4SEL,p);
				ChipSetField(hTuner,FSTV6110_PRESC32ON,Presc);

				pVal=(int)PowOf2(p+1)*10;
				for(rDiv=0;rDiv<=3;rDiv++)
				{
					pCalc= (hTunerParams->Reference/100000)/(PowOf2(rDiv+1));

					if((ABS((S32)(pCalc-pVal))) < (ABS((S32)(pCalcOpt-pVal))))
						rDivOpt= rDiv;

					pCalcOpt= (hTunerParams->Reference/100000)/(PowOf2(rDivOpt+1));

				}


				r=PowOf2(rDivOpt+1);
				divider= (Frequency * r * PowOf2(p+1)*10)/(hTunerParams->Reference/1000);
				divider=(divider+5)/10;

				ChipSetField(hTuner,FSTV6110_RDIV,rDivOpt);
				ChipSetField(hTuner,FSTV6110_NDIV_MSB,MSB(divider));
				ChipSetField(hTuner,FSTV6110_NDIV_LSB,LSB(divider));


				ChipSetField(hTuner,FSTV6110_CALVCOSTRT,1);		/* VCO Auto Calibration */

				i=0;
				while((i<10) && (ChipGetField(hTuner,FSTV6110_CALVCOSTRT)!=0))
				{
					ChipWaitOrAbort(hTuner,1);	/* wait for VCO auto calibration */
					i++;
				}

	}

	return error;
}

U32 STV6110_TunerGetFrequency(STCHIP_Handle_t hTuner)
{
	SAT_TUNER_Params_t hTunerParams = NULL;
	U32 frequency = 0;
	/*Deleted unused variable 'stepsize' to remove warnings: Change ///AG*/
	U32 nbsteps;
	U32 divider = 0;
	/*Deleted unused variable 'swallow' to remove warnings: Change ///AG*/
	U32 psd2;


             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{




				divider = MAKEWORD(ChipGetField(hTuner,FSTV6110_NDIV_MSB),ChipGetField(hTuner,FSTV6110_NDIV_LSB));	/*N*/
				nbsteps = ChipGetFieldImage(hTuner,FSTV6110_RDIV);	/*Rdiv*/
				psd2 = ChipGetFieldImage(hTuner,FSTV6110_DIV4SEL);	/*p*/

				frequency = divider * (hTunerParams->Reference/1000);
				divider=PowOf2(nbsteps+psd2);
				if(divider>0)
				{
					frequency = frequency/divider;
				}
				frequency /= 4;

	}
	return frequency;
}


TUNER_Error_t STV6110_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	U8 u8;
	S32 i=0; /*Deleted unused variable 'filter' to remove warnings: Change ///AG*/

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{

				if((Bandwidth/2) > 36000000)   /*F[4:0] BW/2 max =31+5=36 mhz for F=31*/
					u8 = 31;
				else if((Bandwidth/2) < 5000000) /* BW/2 min = 5Mhz for F=0 */
					u8 = 0;
				else							 /*if 5 < BW/2 < 36*/
					u8 = (Bandwidth/2)/1000000 - 5;

				ChipSetField(hTuner,FSTV6110_RCCLKOFF,0);	 /* Activate the calibration Clock */
				ChipSetField(hTuner,FSTV6110_CF,u8);		 /* Set the LPF value */
				ChipSetField(hTuner,FSTV6110_CALRCSTRT,1); /* Start LPF auto calibration*/

				i=0;
				while((i<10) && (ChipGetField(hTuner,FSTV6110_CALRCSTRT)!=0))
				{
					ChipWaitOrAbort(hTuner,1);	/* wait for LPF auto calibration */
					i++;
				}
				ChipSetField(hTuner,FSTV6110_RCCLKOFF,1);	 /* calibration done, desactivate the calibration Clock */

	}

	return error;
}

U32 STV6110_TunerGetBandwidth(STCHIP_Handle_t hTuner)
{
	SAT_TUNER_Params_t hTunerParams = NULL;
	U32 bandwidth = 0;
	U8 u8 = 0;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{

				u8 = ChipGetField(hTuner,FSTV6110_CF);

				bandwidth = (u8+5)*2000000;	/* x2 for ZIF tuner BW/2=F+5 Mhz*/

	}

	return bandwidth;
}

TUNER_Error_t STV6110_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{

				ChipSetField(hTuner,FSTV6110_BBGAIN,(Gain/2));


	}

	return error;
}

S32 STV6110_TunerGetGain(STCHIP_Handle_t hTuner)
{
	SAT_TUNER_Params_t hTunerParams = NULL;
	S32 gain=0;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{

		gain=2*ChipGetField(hTuner,FSTV6110_BBGAIN);
	}

	return gain;
}

TUNER_Error_t STV6110_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider)
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

	if(hTuner && hTunerParams && !hTuner->Error)
	{

				/*Allowed values 1,2,4 and 8*/
				switch(Divider)
				{
					case 1:
					default:
						ChipSetField(hTuner,FSTV6110_CO_DIV,0);
					break;

					case 2:
						ChipSetField(hTuner,FSTV6110_CO_DIV,1);
					break;

					case 4:
						ChipSetField(hTuner,FSTV6110_CO_DIV,2);
					break;

					case 8:
					case 0: /*Tuner output clock not used then divide by 8 (the 6110 can not stop completely the out clock)*/
						ChipSetField(hTuner,FSTV6110_CO_DIV,3);
					break;
				}

	}

	return error;
}

TUNER_Error_t STV6110_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn)
{
	return TUNER_NO_ERR;
}
BOOL STV6110_TunerGetAttenuator(STCHIP_Handle_t hTuner)
{
	return FALSE;
}

S32 STV6110_TunerGetRFGain(STCHIP_Handle_t hTuner,U32 AGCIntegrator,S32 BBGain)
{
	S32 Gain100dB=1,RefBBgain=6,
		Imin,Imax,i;
		
	SAT_TUNER_Params_t hTunerParams = NULL; 
		
	if(hTuner)	
	{
			hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;   
			if(hTunerParams && !hTuner->Error)
			{
				Imin = 0;
				Imax = FE_STV6110_Gain_LookUp.size-1;
		
				if(AGCIntegrator<=FE_STV6110_Gain_LookUp.table[0].regval)
					Gain100dB=FE_STV6110_Gain_LookUp.table[0].realval;
				else if(AGCIntegrator>=FE_STV6110_Gain_LookUp.table[Imax].regval)
					Gain100dB=FE_STV6110_Gain_LookUp.table[Imax].realval;

				else 
				{
					while((Imax-Imin)>1)
					{
						i=(Imax+Imin)>>1;
						/*equivalent to i=(Imax+Imin)/2; */
						if(INRANGE(FE_STV6110_Gain_LookUp.table[Imin].regval,AGCIntegrator,FE_STV6110_Gain_LookUp.table[i].regval))
							Imax = i;
						else
							Imin = i;
					}
			
					Gain100dB =	(((S32)AGCIntegrator - FE_STV6110_Gain_LookUp.table[Imin].regval)
							* (FE_STV6110_Gain_LookUp.table[Imax].realval - FE_STV6110_Gain_LookUp.table[Imin].realval)
							/ (FE_STV6110_Gain_LookUp.table[Imax].regval - FE_STV6110_Gain_LookUp.table[Imin].regval))
							+ FE_STV6110_Gain_LookUp.table[Imin].realval;
				}
		   }
	}
	
	Gain100dB=Gain100dB+100*(BBGain-RefBBgain); 
	return Gain100dB;

}

BOOL STV6110_TunerGetStatus(STCHIP_Handle_t hTuner)
{
	/*Deleted unused variable 'error' to remove warnings: Change ///AG*/
	SAT_TUNER_Params_t hTunerParams = NULL;
	BOOL locked = FALSE;
	/*Deleted unused variable 'u8' to remove warnings: Change ///AG*/

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{



				if(!hTuner->Error)
				{
					locked = ChipGetField(hTuner,FSTV6110_LOCKPLL);
				}

	}

	return locked;
}

TUNER_Error_t STV6110_TunerWrite(STCHIP_Handle_t hTuner)
{
 TUNER_Error_t error = TUNER_NO_ERR;
 STCHIP_Error_t chipError=CHIPERR_NO_ERROR;
 SAT_TUNER_Params_t hTunerParams = NULL;
 U8 i=0;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

 hTunerParams =  (SAT_TUNER_Params_t) hTuner->pData;

 if(hTuner && hTunerParams)
 {
      if((hTunerParams->DemodModel == DEMOD_STI7111)&&(hTuner->RepeaterHost->ChipID <0x20))
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

TUNER_Error_t STV6110_TunerRead(STCHIP_Handle_t hTuner)
{
 TUNER_Error_t error = TUNER_NO_ERR;
 STCHIP_Error_t chipError=CHIPERR_NO_ERROR;
 SAT_TUNER_Params_t hTunerParams = NULL;
   U8 i=0;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

 hTunerParams =  (SAT_TUNER_Params_t) hTuner->pData;

 if(hTuner && hTunerParams)
 {
      if((hTunerParams->DemodModel == DEMOD_STI7111)&&(hTuner->RepeaterHost->ChipID <0x20))
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

TUNER_Model_t STV6110_TunerGetModel(STCHIP_Handle_t hChip)
{
	TUNER_Model_t model = TUNER_NULL;

	if(hChip && hChip->pData)
	{
		model = ((SAT_TUNER_Params_t) (hChip->pData))->Model;
	}

	return model;
}

void STV6110_TunerSetIQ_Wiring(STCHIP_Handle_t hChip, S8 IQ_Wiring) /*To make generic prototypes Change ///AG*/
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->IQ_Wiring = (TUNER_IQ_t)IQ_Wiring;
	}
}

S8 STV6110_TunerGetIQ_Wiring(STCHIP_Handle_t hChip)  /*To make generic prototypes Change ///AG*/
{
	TUNER_IQ_t wiring = TUNER_IQ_NORMAL; /*Change ///AG*/

	if(hChip && hChip->pData)
	{
		wiring = ((SAT_TUNER_Params_t) (hChip->pData))->IQ_Wiring;
	}

	return (S8)wiring;
}

void STV6110_TunerSetReferenceFreq(STCHIP_Handle_t hChip, U32 Reference)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->Reference = Reference;
	}
}

U32 STV6110_TunerGetReferenceFreq(STCHIP_Handle_t hChip)
{
	U32 reference = 0;

	if(hChip && hChip->pData)
	{
		reference = ((SAT_TUNER_Params_t) (hChip->pData))->Reference;
	}

	return reference;
}

void STV6110_TunerSetIF_Freq(STCHIP_Handle_t hChip, U32 IF)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->IF = IF;
	}
}

U32 STV6110_TunerGetIF_Freq(STCHIP_Handle_t hChip)
{
	U32 ifreq = 0;

	if(hChip && hChip->pData)
	{
		ifreq = ((SAT_TUNER_Params_t) (hChip->pData))->IF;
	}

	return ifreq;
}

void STV6110_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->BandSelect = (TUNER_WIDEBandS_t)BandSelect;
	}
}

U8 STV6110_TunerGetBandSelect(STCHIP_Handle_t hChip)
{
	U32 ifreq = 0;

	if(hChip && hChip->pData)
	{
		ifreq = ((SAT_TUNER_Params_t) (hChip->pData))->BandSelect;
	}

	return (U8)ifreq;
}


TUNER_Error_t STV6110_TunerTerm(STCHIP_Handle_t hTuner)
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




