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
 @File   stv6140_tuner.c
 @brief



*/
#ifndef ST_OSLINUX
	#include "string.h"
	#include "stdlib.h"
#endif

#include "globaldefs.h"
#include "fe_sat_tuner.h"
#include "stv6140_tuner.h"


	/* STV6140 tuner definition */
	/*STV6140*/

/*CTRL1*/
#define RSTV6140_CTRL1  0x0000
#define FSTV6140_K  0x000000f8
#define FSTV6140_CODIV  0x00000006
#define FSTV6140_MCLKDIV  0x00000001

/*CTRL2*/
#define RSTV6140_CTRL2  0x0001
#define FSTV6140_DCLOOPOFF_1  0x00010080
#define FSTV6140_ICP_1  0x00010040
#define FSTV6140_SYN_1  0x00010020
#define FSTV6140_REFOUTSEL_1  0x00010010
#define FSTV6140_BBGAIN_1  0x0001000f

/*CTRL3*/
#define RSTV6140_CTRL3  0x0002
#define FSTV6140_NDIV_1_LSB  0x000200ff

/*CTRL4*/
#define RSTV6140_CTRL4  0x0003
#define FSTV6140_IDVCO  0x00030080
#define FSTV6140_PRESC32ON_1  0x00030040
#define FSTV6140_PDIV_1  0x00030030
#define FSTV6140_NDIV_1_MSB  0x0003000f

/*CTRL5*/
#define RSTV6140_CTRL5  0x0004
#define FSTV6140_RCCLFOFF_1  0x00040080
#define FSTV6140_RDIV_1  0x00040060
#define FSTV6140_CF_1  0x0004001f

/*STAT1*/
#define RSTV6140_STAT1  0x0005
#define FSTV6140_DIVTEST_1  0x000500c0
#define FSTV6140_CPTEST_1  0x00050030
#define FSTV6140_CALTIME_1  0x00050008
#define FSTV6140_CALVCOSTRT_1  0x00050004
#define FSTV6140_CALRCSTRT_1  0x00050002
#define FSTV6140_LOCK_1  0x00050001
#define FSTV6140_DIVTEST_2  0x000500c0
#define FSTV6140_CPTEST_2  0x00050030
#define FSTV6140_CALTIME_2  0x00050008

/*CTRL6*/
#define RSTV6140_CTRL6  0x0006
#define FSTV6140_PATHON_2  0x00060080
#define FSTV6140_PATHON_1  0x00060040
#define FSTV6140_RFB_HGAIN  0x00060020
#define FSTV6140_RFA_HGAIN  0x00060010
#define FSTV6140_LNABON  0x00060008
#define FSTV6140_LNAAON  0x00060004
#define FSTV6140_RFA_2  0x00060002
#define FSTV6140_RFA_1  0x00060001

/*CTRL7*/
#define RSTV6140_CTRL7  0x0007
#define FSTV6140_DCLOOPOFF_2  0x00070080
#define FSTV6140_ICP_2  0x00070040
#define FSTV6140_SYN_2  0x00070020
#define FSTV6140_REFOUTSEL_2  0x00070010
#define FSTV6140_BBGAIN_2  0x0007000f

/*CTRL8*/
#define RSTV6140_CTRL8  0x0008
#define FSTV6140_NDIV_2_LSB  0x000800ff

/*CTRL9*/
#define RSTV6140_CTRL9  0x0009
#define FSTV6140_IDSW  0x00090080
#define FSTV6140_PRESC32ON_2  0x00090040
#define FSTV6140_PDIV_2  0x00090030
#define FSTV6140_NDIV_2_MSB  0x0009000f

/*CTRL10*/
#define RSTV6140_CTRL10  0x000a
#define FSTV6140_RCCLFOFF_2  0x000a0080
#define FSTV6140_RDIV_2  0x000a0060
#define FSTV6140_CF_2  0x000a001f

/*STAT2*/
#define RSTV6140_STAT2  0x000b
#define FSTV6140_CALVCOSTRT_2  0x000b0004
#define FSTV6140_CALRCSTRT_2  0x000b0002
#define FSTV6140_LOCK_2  0x000b0001

/*STAT3*/
#define RSTV6140_STAT3  0x000c
#define FSTV6140_COMP_1  0x000c0080
#define FSTV6140_SEL_1  0x000c007e
#define FSTV6140_CALOFF_1  0x000c0001

/*STAT4*/
#define RSTV6140_STAT4  0x000d
#define FSTV6140_RON_1  0x000d0080
#define FSTV6140_RCCALOFF_1  0x000d0040
#define FSTV6140_RC_1  0x000d003f

/*STAT5*/
#define RSTV6140_STAT5  0x000e
#define FSTV6140_COMP_2  0x000e0080
#define FSTV6140_SEL_2  0x000e007e
#define FSTV6140_CALOFF_2  0x000e0001

/*STAT6*/
#define RSTV6140_STAT6  0x000f
#define FSTV6140_RON_2  0x000f0080
#define FSTV6140_RCCALOFF_2  0x000f0040
#define FSTV6140_RC_2  0x000f003f


TUNER_Error_t STV6140_TunerInit(void *pTunerInit_v,STCHIP_Handle_t *TunerHandle)
{
	SAT_TUNER_Params_t	hTunerParams = NULL;
	STCHIP_Info_t	ChipInfo;
	STCHIP_Handle_t hTuner = NULL;
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t pTunerInit = (SAT_TUNER_Params_t)pTunerInit_v;

 	STCHIP_Register_t		DefSTV6140Val[STV6140_NBREGS]=
 	{
    { RSTV6140_CTRL1,    0x58 }    ,/* CTRL1 */
    { RSTV6140_CTRL2,    0x33 }    ,/* CTRL2 */
    { RSTV6140_CTRL3,    0x30 }    ,/* CTRL3 */
    { RSTV6140_CTRL4,    0x07 }    ,/* CTRL4 */
    { RSTV6140_CTRL5,    0x72 }    ,/* CTRL5 */
    { RSTV6140_STAT1,    0x01 }    ,/* STAT1 */
    { RSTV6140_CTRL6,    0xfd }    ,/* CTRL6 */
    { RSTV6140_CTRL7,    0x33 }    ,/* CTRL7 */
    { RSTV6140_CTRL8,    0x30 }    ,/* CTRL8 */
    { RSTV6140_CTRL9,    0x07 }    ,/* CTRL9 */
    { RSTV6140_CTRL10,    0x72 }    ,/* CTRL10 */
    { RSTV6140_STAT2,    0x00 }    ,/* STAT2 */
    { RSTV6140_STAT3,    0xcc }    ,/* STAT3 */
    { RSTV6140_STAT4,    0x0c }    ,/* STAT4 */
    { RSTV6140_STAT5,    0xbe }    ,/* STAT5 */
    { RSTV6140_STAT6,    0x0c }    ,/* STAT6 */
};


    /*
    **   REGISTER CONFIGURATION
    **     ----------------------
    */
    #ifdef HOST_PC
    hTunerParams = calloc (1,sizeof(SAT_TUNER_InitParams_t));	/* Allocation of the chip structure	*/
    #endif

    #ifdef CHIP_STAPI
             if(*TunerHandle==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t)((*TunerHandle)->pData);
	memcpy(&ChipInfo, *TunerHandle,sizeof(STCHIP_Info_t));
    #endif

    if(hTunerParams != NULL)
    {
    	hTunerParams->Model		= pTunerInit->Model;    		/* Tuner model */
    	hTunerParams->Reference = pTunerInit->Reference;		/* Reference Clock in Hz */
    	hTunerParams->IF = pTunerInit->IF; 						/* IF Hz intermediate frequency */
    	hTunerParams->IQ_Wiring = pTunerInit->IQ_Wiring;		/* hardware IQ invertion */
    	hTunerParams->BandSelect = pTunerInit->BandSelect;		/* Wide band tuner (6130 like, band selection) */
    	hTunerParams->BandBorder = pTunerInit->BandBorder;		/* Wide band tuner low band, high band border in KHz (6140 like) */
		hTunerParams->BandSelect2 = pTunerInit->BandSelect2;	/* Dual Wide band tuner (6140 like, band selection) */
		hTunerParams->InputSelect = pTunerInit->InputSelect;	/* Input Selection for dual tuner*/

		strncpy((char*)ChipInfo.Name,(char*)pTunerInit->TunerName,MAXNAMESIZE-1);	/* Tuner name */
    	ChipInfo.RepeaterHost = pTunerInit->RepeaterHost;		/* Repeater host */
		ChipInfo.RepeaterFn = pTunerInit->RepeaterFn;			/* Repeater enable/disable function */
		ChipInfo.Repeater = TRUE;								/* Tuner need to enable repeater */
		ChipInfo.I2cAddr = pTunerInit->TunerI2cAddress;			/* Init tuner I2C address */


    	ChipInfo.pData = hTunerParams;				/* Store tunerparams pointer into Chip structure */


		ChipInfo.NbRegs   = STV6140_NBREGS;
		ChipInfo.NbFields = STV6140_NBFIELDS;
		ChipInfo.ChipMode = STCHIP_MODE_SUBADR_8;
		ChipInfo.WrStart  = RSTV6140_CTRL1;
		ChipInfo.WrSize   = 16;
		ChipInfo.RdStart  = RSTV6140_CTRL1;
		ChipInfo.RdSize   = 16;

		#ifdef HOST_PC
			hTuner = ChipOpen(&ChipInfo);
			(*TunerHandle) = hTuner;
		#endif
		#ifdef CHIP_STAPI
			hTuner = *TunerHandle;
			memcpy(hTuner, &ChipInfo, sizeof(STCHIP_Info_t));
		#endif

		if((*TunerHandle)== NULL)
			error=TUNER_INVALID_HANDLE;

		if(hTuner != NULL)
		{
			/*******************************
			**   CHIP REGISTER MAP IMAGE INITIALIZATION
			**     ----------------------
			********************************/
			ChipUpdateDefaultValues(hTuner,DefSTV6140Val);

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
					ChipSetFieldImage(hTuner,FSTV6140_CODIV,0);
				break;

				case 2:
					ChipSetFieldImage(hTuner,FSTV6140_CODIV,1);
				break;

				case 4:
					ChipSetFieldImage(hTuner,FSTV6140_CODIV,2);
				break;

				case 8:
				case 0: /*Tuner output clock not used then divide by 8 (the 6140 can not stop completely the out clock)*/
					ChipSetFieldImage(hTuner,FSTV6140_CODIV,3);
				break;
			}

			/*update K field according to the Reference clock (quartz)*/
			ChipSetFieldImage(hTuner,FSTV6140_K,((hTunerParams->Reference/1000000)-16));

			/*I2C registers initialization*/
			error=STV6140_TunerWrite(hTuner);
		}

	}
	else
		error=TUNER_INVALID_HANDLE;

	return error;
}

TUNER_Error_t STV6140_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn)
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
			if(hTunerParams->Model==TUNER_STV6140_Tuner1)
			{
				ChipSetFieldImage(hTuner,FSTV6140_SYN_1,0);
				ChipSetFieldImage(hTuner,FSTV6140_PATHON_1,0);

			}
			else if(hTunerParams->Model==TUNER_STV6140_Tuner2)
			{
				ChipSetFieldImage(hTuner,FSTV6140_SYN_2,0);
				ChipSetFieldImage(hTuner,FSTV6140_PATHON_2,0);

			}

		}
		else	/*Power down OFF*/
		{
			if(hTunerParams->Model==TUNER_STV6140_Tuner1)
			{
				ChipSetFieldImage(hTuner,FSTV6140_SYN_1,1);
				ChipSetFieldImage(hTuner,FSTV6140_PATHON_1,1);

			}
			else if(hTunerParams->Model==TUNER_STV6140_Tuner2)
			{
				ChipSetFieldImage(hTuner,FSTV6140_SYN_2,1);
				ChipSetFieldImage(hTuner,FSTV6140_PATHON_2,1);

			}
		}
		error = STV6140_TunerWrite(hTuner);


	}
	return error;
}


TUNER_Error_t STV6140_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency)
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

	if(hTuner && hTunerParams && !hTuner->Error)
	{


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
				if ( ABS((S32)1400-(S32)Fstep) < ABS((S32)1400-(S32)FstepOpt)  )
				{
					FstepOpt=Fstep;
					rDivOpt=rDiv;
				}
			}
		}
		divider= ((Frequency)*PowOf2(rDivOpt+1)*PowOf2(p+1))/(hTunerParams->Reference/1000); /*NDiv*/

		if(hTunerParams->Model==TUNER_STV6140_Tuner1)
		{
			ChipSetFieldImage(hTuner,FSTV6140_PDIV_1,p);
			if (divider<1024)
			{
				ChipSetFieldImage(hTuner,FSTV6140_PRESC32ON_1,0);
			} else
			{
				ChipSetFieldImage(hTuner,FSTV6140_PRESC32ON_1,1);
			}
			ChipSetFieldImage(hTuner,FSTV6140_RDIV_1,rDivOpt);
			ChipSetFieldImage(hTuner,FSTV6140_NDIV_1_MSB,MSB(divider));
			ChipSetFieldImage(hTuner,FSTV6140_NDIV_1_LSB,LSB(divider));
			ChipSetRegisters(hTuner,RSTV6140_CTRL3,4);

		 	ChipSetField(hTuner,FSTV6140_CALVCOSTRT_1,1);		/* VCO Auto Calibration */
		 }
		 else if(hTunerParams->Model==TUNER_STV6140_Tuner2)
		 {
			ChipSetFieldImage(hTuner,FSTV6140_PDIV_2,p);
			if (divider<1024)
			{
				ChipSetFieldImage(hTuner,FSTV6140_PRESC32ON_2,0);
			} else
			{
				ChipSetFieldImage(hTuner,FSTV6140_PRESC32ON_2,1);
			}
			ChipSetFieldImage(hTuner,FSTV6140_RDIV_2,rDivOpt);
			ChipSetFieldImage(hTuner,FSTV6140_NDIV_2_MSB,MSB(divider));
			ChipSetFieldImage(hTuner,FSTV6140_NDIV_2_LSB,LSB(divider));
			ChipSetRegisters(hTuner,RSTV6140_CTRL8,4);

			ChipSetField(hTuner,FSTV6140_CALVCOSTRT_2,1);		/* VCO Auto Calibration */

		 }
	}

	return error;
}

U32 STV6140_TunerGetFrequency(STCHIP_Handle_t hTuner)
{
	SAT_TUNER_Params_t hTunerParams = NULL;
	U32 frequency = 0;
	U32 nbsteps;
	U32 divider = 0;
	U32 psd2;


             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{
 		if(hTunerParams->Model==TUNER_STV6140_Tuner1)
		{
			ChipGetRegisters(hTuner,RSTV6140_CTRL3,3);
			divider=MAKEWORD(ChipGetFieldImage(hTuner,FSTV6140_NDIV_1_MSB),ChipGetFieldImage(hTuner,FSTV6140_NDIV_1_LSB));	/*N*/
			psd2=PowOf2(ChipGetFieldImage(hTuner,FSTV6140_PDIV_1));	/*P=2^(p+1)*/
			nbsteps=PowOf2(ChipGetField(hTuner,FSTV6140_RDIV_1)); /*R=2^(r+1)*/

			frequency=(divider*(hTunerParams->Reference/1000))/(psd2*nbsteps); /*FLO*/
			frequency /= 4; /*as P=2^(p+1) and R=2^(r+1) divide by 4 */
		}
		else if(hTunerParams->Model==TUNER_STV6140_Tuner2)
		{
			ChipGetRegisters(hTuner,RSTV6140_CTRL8,3);
			divider=MAKEWORD(ChipGetFieldImage(hTuner,FSTV6140_NDIV_2_MSB),ChipGetFieldImage(hTuner,FSTV6140_NDIV_2_LSB));	/*N*/
			psd2=PowOf2(ChipGetFieldImage(hTuner,FSTV6140_PDIV_2));	/*P=2^(p+1)*/
			nbsteps=PowOf2(ChipGetFieldImage(hTuner,FSTV6140_RDIV_2)); /*R=2^(r+1)*/

			frequency=(divider*(hTunerParams->Reference/1000))/(psd2*nbsteps); /*FLO*/
			frequency /= 4; /*as P=2^(p+1) and R=2^(r+1) divide by 4 */
		}
	}
	return frequency;
}


TUNER_Error_t STV6140_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	U8 u8;
	S32 i=0;

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

		if(hTunerParams->Model==TUNER_STV6140_Tuner1)
		{
			ChipSetField(hTuner,FSTV6140_CF_1,u8);		 /* Set the LPF value */
			ChipSetField(hTuner,FSTV6140_CALRCSTRT_1,1); /* Start LPF auto calibration*/

			i=0;
			while((i<10) && (ChipGetField(hTuner,FSTV6140_CALRCSTRT_1)!=0))
			{
				ChipWaitOrAbort(hTuner,1);	/* wait for LPF auto calibration */
				i++;
			}
		}
		else if(hTunerParams->Model==TUNER_STV6140_Tuner2)
		{
			ChipSetField(hTuner,FSTV6140_CF_2,u8);		 /* Set the LPF value */
			ChipSetField(hTuner,FSTV6140_CALRCSTRT_2,1); /* Start LPF auto calibration*/

			i=0;
			while((i<10) && (ChipGetField(hTuner,FSTV6140_CALRCSTRT_2)!=0))
			{
				ChipWaitOrAbort(hTuner,1);	/* wait for LPF auto calibration */
				i++;
			}
		}

	}

	return error;
}

U32 STV6140_TunerGetBandwidth(STCHIP_Handle_t hTuner)
{
	SAT_TUNER_Params_t hTunerParams = NULL;
	U32 bandwidth = 0;
	U8 u8 = 0;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{
		if(hTunerParams->Model==TUNER_STV6140_Tuner1)
			u8 = ChipGetField(hTuner,FSTV6140_CF_1);
		else 	if(hTunerParams->Model==TUNER_STV6140_Tuner2)
			u8 = ChipGetField(hTuner,FSTV6140_CF_2);

		bandwidth = (u8+5)*2000000;	/* x2 for ZIF tuner BW/2=F+5 Mhz*/

	}

	return bandwidth;
}

TUNER_Error_t STV6140_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{
 		if(hTunerParams->Model==TUNER_STV6140_Tuner1)
			ChipSetField(hTuner,FSTV6140_BBGAIN_1,(Gain/2));
		else if(hTunerParams->Model==TUNER_STV6140_Tuner2)
			ChipSetField(hTuner,FSTV6140_BBGAIN_2,(Gain/2));
	}

	return error;
}

S32 STV6140_TunerGetGain(STCHIP_Handle_t hTuner)
{
	SAT_TUNER_Params_t hTunerParams = NULL;
	S32 gain=0;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{
		if(hTunerParams->Model==TUNER_STV6140_Tuner1)
			gain=2*ChipGetField(hTuner,FSTV6140_BBGAIN_1);
		else if(hTunerParams->Model==TUNER_STV6140_Tuner2)
			gain=2*ChipGetField(hTuner,FSTV6140_BBGAIN_2);
	}

	return gain;
}

TUNER_Error_t STV6140_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider)
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
				ChipSetField(hTuner,FSTV6140_CODIV,0);
			break;

			case 2:
				ChipSetField(hTuner,FSTV6140_CODIV,1);
			break;

			case 4:
				ChipSetField(hTuner,FSTV6140_CODIV,2);
			break;

			case 8:
			case 0:
				ChipSetField(hTuner,FSTV6140_CODIV,3);
			break;
		}
	}

	return error;
}

TUNER_Error_t STV6140_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn)
{
	return TUNER_NO_ERR;
}

BOOL STV6140_TunerGetAttenuator(STCHIP_Handle_t hTuner)
{
	return FALSE;
}


BOOL STV6140_TunerGetStatus(STCHIP_Handle_t hTuner)
{

	SAT_TUNER_Params_t hTunerParams = NULL;
	BOOL locked = FALSE;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{

		if(hTunerParams->Model==TUNER_STV6140_Tuner1)
			locked = ChipGetField(hTuner,FSTV6140_LOCK_1);
		else if(hTunerParams->Model==TUNER_STV6140_Tuner2)
			locked = ChipGetField(hTuner,FSTV6140_LOCK_2);
	}

	return locked;
}

TUNER_Error_t      STV6140_TunerRFSelect(STCHIP_Handle_t hTuner)

{
	/* Switch the tuner input to input A,B,C or D

	0 ==> input 1
	1 ==> input 2
	2 ==> input 1
	3 ==> input 2

	The selection between A and B when input1 or C and D when input2
	depend on the frequency (>BandBorder or <BandBorder) and made when setting the ffrequency

	*/


	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

             if(hTuner==NULL)
                return TUNER_INVALID_HANDLE;

	hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;

	if(hTuner && hTunerParams && !hTuner->Error)
	{
	#if 0
		switch(RFSource)
		{
			case 0:	/* input A*/
			case 3:
			default:
				hTunerParams->InputSelect=TUNER_INPUT1;
			break;

			case 1: /* input B*/
			case 2:
				hTunerParams->InputSelect=TUNER_INPUT2;
			break;

		}
		#endif
		if(hTunerParams->Model==TUNER_STV6140_Tuner1)
			ChipSetField(hTuner,FSTV6140_RFA_1,(hTunerParams->RF_Source== 0 ? 1 : 0));	/*0->FE_RF_SOURCE_A*/
		else if(hTunerParams->Model==TUNER_STV6140_Tuner2)
			ChipSetField(hTuner,FSTV6140_RFA_2,(hTunerParams->RF_Source == 0 ? 1 : 0));    /*0->FE_RF_SOURCE_A*/

	}
	return error;

}

TUNER_Error_t STV6140_TunerWrite(STCHIP_Handle_t hTuner)
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

TUNER_Error_t STV6140_TunerRead(STCHIP_Handle_t hTuner)
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

TUNER_Model_t STV6140_TunerGetModel(STCHIP_Handle_t hChip)
{
	TUNER_Model_t model = TUNER_NULL;

	if(hChip && hChip->pData)
	{
		model = ((SAT_TUNER_Params_t) (hChip->pData))->Model;
	}

	return model;
}

void STV6140_TunerSetIQ_Wiring(STCHIP_Handle_t hChip, S8 IQ_Wiring)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->IQ_Wiring = (TUNER_IQ_t)IQ_Wiring;
	}
}

S8 STV6140_TunerGetIQ_Wiring(STCHIP_Handle_t hChip)
{
	TUNER_IQ_t wiring = TUNER_IQ_NORMAL;

	if(hChip && hChip->pData)
	{
		wiring = ((SAT_TUNER_Params_t) (hChip->pData))->IQ_Wiring;
	}

	return (S8)wiring;
}

void STV6140_TunerSetReferenceFreq(STCHIP_Handle_t hChip, U32 Reference)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->Reference = Reference;
	}
}

U32 STV6140_TunerGetReferenceFreq(STCHIP_Handle_t hChip)
{
	U32 reference = 0;

	if(hChip && hChip->pData)
	{
		reference = ((SAT_TUNER_Params_t) (hChip->pData))->Reference;
	}

	return reference;
}

void STV6140_TunerSetIF_Freq(STCHIP_Handle_t hChip, U32 IF)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->IF = IF;
	}
}

U32 STV6140_TunerGetIF_Freq(STCHIP_Handle_t hChip)
{
	U32 ifreq = 0;

	if(hChip && hChip->pData)
	{
		ifreq = ((SAT_TUNER_Params_t) (hChip->pData))->IF;
	}

	return ifreq;
}

void STV6140_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect)
{
	if(hChip && hChip->pData)
	{
		if(((SAT_TUNER_Params_t) (hChip->pData))->InputSelect==TUNER_INPUT1)
			((SAT_TUNER_Params_t) (hChip->pData))->BandSelect = (TUNER_WIDEBandS_t)BandSelect;
		else if(((SAT_TUNER_Params_t) (hChip->pData))->InputSelect==TUNER_INPUT2)
			((SAT_TUNER_Params_t) (hChip->pData))->BandSelect2 = (TUNER_WIDEBandS_t)BandSelect;
	}
}

U8 STV6140_TunerGetBandSelect(STCHIP_Handle_t hChip)
{
	U32 ifreq = 0;

	if(hChip && hChip->pData)
	{
		ifreq = ((SAT_TUNER_Params_t) (hChip->pData))->BandSelect;
	}

	return (U8)ifreq;
}


TUNER_Error_t STV6140_TunerTerm(STCHIP_Handle_t hTuner)
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




