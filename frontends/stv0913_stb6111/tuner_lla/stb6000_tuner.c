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
 @File   stb6000_tuner.c
 @brief



*/
#ifndef ST_OSLINUX
	#include "string.h"
	#include "stdlib.h"
#endif

#include "globaldefs.h"
#include "fe_sat_tuner.h"
#include "stb6000_tuner.h"



	/* RFMagic STB6000 tuner definition */

/*LD*/
#define RSTB6000_LD  0x0000
#define FSTB6000_LD  0x00000001

/*ODIV*/
#define RSTB6000_ODIV  0x0001
#define FSTB6000_OSCH  0x00010080
#define FSTB6000_OCK  0x00010060
#define FSTB6000_ODIV  0x00010010
#define FSTB6000_OSM  0x0001000f

/*N*/
#define RSTB6000_N  0x0002
#define FSTB6000_N_MSB  0x000200ff

/*CP_A*/
#define RSTB6000_CP_A  0x0003
#define FSTB6000_N_LSB  0x00030080
#define FSTB6000_A  0x0003001f

/*K_R*/
#define RSTB6000_K_R  0x0004
#define FSTB6000_K  0x000400c0
#define FSTB6000_R  0x0004003f

/*G*/
#define RSTB6000_G  0x0005
#define FSTB6000_G  0x0005000f

/*F*/
#define RSTB6000_F  0x0006
#define FSTB6000_F  0x0006001f

/*FCL*/
#define RSTB6000_FCL  0x0007
#define FSTB6000_DLB  0x00070038
#define FSTB6000_FCL  0x00070007

/*TEST1*/
#define RSTB6000_TEST1  0x0008
#define FSTB6000_TEST1  0x000800ff

/*TEST2*/
#define RSTB6000_TEST2  0x0009
#define FSTB6000_TEST2  0x000900ff

/*LPEN*/
#define RSTB6000_LPEN  0x000a
#define FSTB6000_LPENPD  0x000a0060
#define FSTB6000_LPEN  0x000a0010

/*XOG*/
#define RSTB6000_XOG  0x000b
#define FSTB6000_XOG  0x000b0080
#define FSTB6000_XOGV  0x000b0040
#define FSTB6000_XOGPD  0x000b0006

static U32 STB6K0_LOOKUP[11][3]=		{/* low			high	  osm */
										{	950000,		999999,		0xA	},
										{	1000000,	1075000,	0xC	},
										{	1075001,	1199999,	0x0	},
										{	1200000,	1299999,	0x1	},
										{	1300000,	1369999,	0x2	},
										{	1370000,	1469999,	0x4	},
										{	1470000,	1529999,	0x5	},
										{	1530000,	1649999,	0x6	},
										{	1650000,	1799999,	0x8	},
										{	1800000,	1949999,	0xA	},
										{	1950000,	2150000,	0xC	}
									};


TUNER_Error_t STB6000_TunerInit(void *pTunerInit_v,STCHIP_Handle_t *TunerHandle)
{
	SAT_TUNER_Params_t	hTunerParams = NULL;
	STCHIP_Info_t	ChipInfo;
	STCHIP_Handle_t hTuner = NULL;
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t pTunerInit = (SAT_TUNER_Params_t)pTunerInit_v; /*Changed to retain common function prototype for all Tuners: Change ///AG*/

	STCHIP_Register_t DefSTB6000Val[STB6000_NBREGS]=
	{
	    {	RSTB6000_LD,		0x81    },/* LD */
	    {	RSTB6000_ODIV,		0x7a    },/* ODIV */
	    {	RSTB6000_N,			0x3c    },/* N */
	    {	RSTB6000_CP_A,		0x6c    },/* CP_A */
	    {	RSTB6000_K_R,		0x04    },/* K_R */
	    {	RSTB6000_G,			0x1b    },/* G */
	    {	RSTB6000_F,			0x9e    },/* F */
	    {	RSTB6000_FCL,		0xdf    },/* FCL */
	    {	RSTB6000_TEST1,		0xd0    },/* TEST1 */
	    {	RSTB6000_TEST2,		0x50    },/* TEST2 */
	    {	RSTB6000_LPEN,		0xfb    },/* LPEN */
	    {	RSTB6000_XOG,		0xcf    }/* XOG */
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


		/* fill elements of external chip data structure */
		ChipInfo.NbRegs   = STB6000_NBREGS;
		ChipInfo.NbFields = STB6000_NBFIELDS;
		ChipInfo.ChipMode = STCHIP_MODE_NOSUBADR_RD;/*STCHIP_MODE_NOSUBADR_RD;*///HS
		ChipInfo.WrStart  = RSTB6000_ODIV;
		ChipInfo.WrSize   = 11;
		ChipInfo.RdStart  = RSTB6000_LD;
		ChipInfo.RdSize   = 12;

		#ifdef HOST_PC				/* Change ///AG*/
			hTuner = ChipOpen(&ChipInfo);
			(*TunerHandle) = hTuner;
		#endif

		#ifdef CHIP_STAPI
			hTuner = *TunerHandle;/*obtain hTuner : change ///AG*/
			memcpy(hTuner, &ChipInfo, sizeof(STCHIP_Info_t)); /* Copy the ChipInfo to hTuner: Change ///AG*/
		#endif

		if(*TunerHandle)
		{
		 	error=TUNER_INVALID_HANDLE;
		}		
		if(hTuner != NULL)
		{
			/*******************************
    		**   CHIP REGISTER MAP IMAGE INITIALIZATION
    		**     ----------------------
    		********************************/
			ChipUpdateDefaultValues(hTuner,DefSTB6000Val);

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

					ChipSetFieldImage(hTuner,FSTB6000_K,0);
				break;

				case 2:
					ChipSetFieldImage(hTuner,FSTB6000_K,1);
				break;

				case 4:
					ChipSetFieldImage(hTuner,FSTB6000_K,2);
				break;

				case 8:
				case 0: /*Tuner output clock not used then divide by 8 (the STB6000 can not stop completely the out clock)*/
					ChipSetFieldImage(hTuner,FSTB6000_K,3);
				break;
			}


		    /*I2C registers initialization*/
			error=STB6000_TunerWrite(hTuner);

	/*************R/W test code***************************
	STB6000_TunerRead(hTuner);
	{
		int i=0;
		printf("\n OPEN");
		for(i=0;i<12;i++)
		{
		        printf("\tRegister[%d] =0x%x",hTuner->pRegMapImage[i].Addr,hTuner->pRegMapImage[i].Value);
		}
	}
	***************************************************/
		}

	}
	else
		error=TUNER_INVALID_HANDLE;


	return error;
}

TUNER_Error_t STB6000_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn)
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
 			ChipSetFieldImage(hTuner,FSTB6000_LPENPD,0x00);
			ChipSetFieldImage(hTuner,FSTB6000_XOGPD,0x00);
		}
		else	/*Power down OFF*/
		{
			ChipSetFieldImage(hTuner,FSTB6000_LPENPD,0x03);
			ChipSetFieldImage(hTuner,FSTB6000_XOGPD,0x03);
		}
		error = STB6000_TunerWrite(hTuner);

	}
	return error;
}


TUNER_Error_t STB6000_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

	U32 frequency;
	U32 stepsize;
	U32 nbsteps;
	U32 divider;
	U32 swallow;

	U8 u8;

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
		ChipSetFieldImage(hTuner,FSTB6000_ODIV,Frequency <= 1075000);

		u8=0;

		while(!INRANGE(STB6K0_LOOKUP[u8][0],Frequency,STB6K0_LOOKUP[u8][1]) && (u8<10)) u8++;
		ChipSetFieldImage(hTuner,FSTB6000_OSM,STB6K0_LOOKUP[u8][2]);

		frequency = Frequency<<(ChipGetFieldImage(hTuner,FSTB6000_ODIV)+1);
		stepsize =  2*(hTunerParams->Reference / ChipGetFieldImage(hTuner,FSTB6000_R)); /* 2 x Fr / R */
		nbsteps = (frequency * 100) / (stepsize / 10);
		divider = nbsteps / 16; /* 32 */
		swallow = nbsteps % 16; /* 32 */
		/*printf("\nF=%d stepsize =%d,nbsteps =%d divider =%d swallow =%d",frequency,stepsize,nbsteps,divider,swallow);	*/
		ChipSetFieldImage(hTuner,FSTB6000_N_MSB,(divider>>1) & 0xFF);
		ChipSetFieldImage(hTuner,FSTB6000_N_LSB,divider & 0x01);
		ChipSetFieldImage(hTuner,FSTB6000_A,swallow);
		ChipSetFieldImage(hTuner,FSTB6000_LPEN,0);	/* PLL loop disabled */
		ChipSetFieldImage(hTuner,FSTB6000_OSCH,1);	/* VCO search enabled */
		ChipSetFieldImage(hTuner,FSTB6000_OCK,1);		/* VCO search clock off */
		error = STB6000_TunerWrite(hTuner);


		ChipSetFieldImage(hTuner,FSTB6000_LPEN,1);	/* PLL loop enabled */
		error = STB6000_TunerWrite(hTuner);
		ChipWaitOrAbort(hTuner,20);
		ChipSetFieldImage(hTuner,FSTB6000_OSCH,0);	/* VCO search disabled */
		ChipSetFieldImage(hTuner,FSTB6000_OCK,3);		/* VCO search clock off */
		error = STB6000_TunerWrite(hTuner);

	}
	/************R/W test code****************************
	STB6000_TunerRead(hTuner);
	{
		int i=0;
		printf("\n");
		for(i=0;i<12;i++)
		{
		    printf("Register[%d] =0x%x",hTuner->pRegMapImage[i].Addr,hTuner->pRegMapImage[i].Value);
		}
	}
	*****************************************/
	return error;
}

U32 STB6000_TunerGetFrequency(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	U32 frequency = 0;
	U32 stepsize;
	U32 nbsteps;
	U32 divider = 0;
	U32 swallow;


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
		divider =	(ChipGetFieldImage(hTuner,FSTB6000_N_MSB)<<1)
					+ ChipGetFieldImage(hTuner,FSTB6000_N_LSB);

		swallow = ChipGetFieldImage(hTuner,FSTB6000_A);
		nbsteps = divider*16 + swallow;	/* N x P + A ; P=16*/
		stepsize =  2*(hTunerParams->Reference / ChipGetFieldImage(hTuner,FSTB6000_R)); /* 2 x Fr / R */

		frequency = (nbsteps * (stepsize>>(ChipGetFieldImage(hTuner,FSTB6000_ODIV)+1)))/1000; /* 2 x Fr x (PxN + A)/R */
	}
	return frequency;
}


TUNER_Error_t STB6000_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	U8 u8;

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
		if(Bandwidth == 0)
		{
			ChipSetFieldImage(hTuner,FSTB6000_FCL,0);
			STB6000_TunerWrite(hTuner);

		}
		else
		{
			if((Bandwidth/2) > 36000000)
				u8 = 31;
			else if((Bandwidth/2) < 5000000)
				u8 = 0;
			else
				u8 = (Bandwidth/2)/1000000/* - 5*/; 

			ChipSetFieldImage(hTuner,FSTB6000_FCL,0);
			STB6000_TunerWrite(hTuner);
			ChipSetFieldImage(hTuner,FSTB6000_F,u8);
			error = STB6000_TunerWrite(hTuner);
			ChipWaitOrAbort(hTuner,10);
			ChipSetFieldImage(hTuner,FSTB6000_FCL,7);
			error=STB6000_TunerWrite(hTuner);
		}
	}

	return error;
}

U32 STB6000_TunerGetBandwidth(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR; 
	SAT_TUNER_Params_t hTunerParams = NULL;
	U32 bandwidth = 0;
	U8 u8 = 0;
	
	/*fix Coverity CID: 20115 */
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
		u8 = ChipGetFieldImage(hTuner,FSTB6000_F);

		bandwidth = (u8)*2000000;	/* x2 for ZIF tuner */
	}

	return bandwidth;
}

TUNER_Error_t STB6000_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain)
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
		ChipSetFieldImage(hTuner,FSTB6000_G,(Gain/2)+7);
		error = STB6000_TunerWrite(hTuner);
	}

	return error;
}

S32 STB6000_TunerGetGain(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	S32 gain=0;
	
	/*fix Coverity CID: 20117 */
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
		gain=ChipGetFieldImage(hTuner,FSTB6000_G);
		gain = (gain-7)*2;
	}

	return gain;
}

TUNER_Error_t STB6000_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider)
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
		/*Allowed values 1,2,4 and 8*/
		switch(Divider)
		{
			case 1:
			default:
				ChipSetFieldImage(hTuner,FSTB6000_K,0);
				error = STB6000_TunerWrite(hTuner);
			break;

			case 2:
				ChipSetFieldImage(hTuner,FSTB6000_K,1);
				error = STB6000_TunerWrite(hTuner);
			break;

			case 4:
				ChipSetFieldImage(hTuner,FSTB6000_K,2);
				error = STB6000_TunerWrite(hTuner);
			break;

			case 8:
			case 0: /*Tuner output clock not used then divide by 8 (the STB6000 can not stop completely the out clock)*/
				ChipSetFieldImage(hTuner,FSTB6000_K,3);
				error = STB6000_TunerWrite(hTuner);
			break;
		}
	}

	return error;
}


TUNER_Error_t STB6000_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn)
{
	return TUNER_NO_ERR;
}
BOOL STB6000_TunerGetAttenuator(STCHIP_Handle_t hTuner)
{
	return FALSE;
}

BOOL STB6000_TunerGetStatus(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	BOOL locked = FALSE;
	/*fix Coverity CID: 20119 */
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
		STB6000_TunerRead(hTuner);

		if(!hTuner->Error)
			locked = ChipGetFieldImage(hTuner,FSTB6000_LD);
	}

	return locked;
}

TUNER_Error_t STB6000_TunerWrite(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	STCHIP_Error_t chipError=CHIPERR_NO_ERROR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	U8 i;

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
        if((hTunerParams->DemodModel != DEMOD_STI7111) && (hTunerParams->DemodModel != DEMOD_STIH205))
            chipError=ChipSetRegisters(hTuner,hTuner->WrStart,hTuner->WrSize);
        else
        {
            for(i=0;i<hTuner->WrSize;i++) /*Write one register at a time for 7111*/
                chipError |= ChipSetRegisters(hTuner, hTuner->WrStart + i, 1);
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

TUNER_Error_t STB6000_TunerRead(STCHIP_Handle_t hTuner)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	STCHIP_Error_t chipError=CHIPERR_NO_ERROR;
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
        if((hTunerParams->DemodModel != DEMOD_STI7111) && (hTunerParams->DemodModel != DEMOD_STIH205))
            chipError=ChipGetRegisters(hTuner,hTuner->RdStart,hTuner->RdSize);
        else
        {
            chipError=ChipGetRegisters(hTuner,hTuner->RdStart,1);/*Only read first register*/
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

TUNER_Model_t STB6000_TunerGetModel(STCHIP_Handle_t hChip)
{
	TUNER_Model_t model = TUNER_NULL;

	if(hChip && hChip->pData)
	{
		model = ((SAT_TUNER_Params_t) (hChip->pData))->Model;
	}

	return model;
}

void STB6000_TunerSetIQ_Wiring(STCHIP_Handle_t hChip,  S8 IQ_Wiring)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->IQ_Wiring = (TUNER_IQ_t)IQ_Wiring;
	}
}

S8 STB6000_TunerGetIQ_Wiring(STCHIP_Handle_t hChip)
{
	TUNER_IQ_t wiring = TUNER_IQ_NORMAL;

	if(hChip && hChip->pData)
	{
		wiring = ((SAT_TUNER_Params_t) (hChip->pData))->IQ_Wiring;
	}

	return (S8)wiring;
}

void STB6000_TunerSetReferenceFreq(STCHIP_Handle_t hChip, U32 Reference)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->Reference = Reference;
	}
}

U32 STB6000_TunerGetReferenceFreq(STCHIP_Handle_t hChip)
{
	U32 reference = 0;

	if(hChip && hChip->pData)
	{
		reference = ((SAT_TUNER_Params_t) (hChip->pData))->Reference;
	}

	return reference;
}

void STB6000_TunerSetIF_Freq(STCHIP_Handle_t hChip, U32 IF)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->IF = IF;
	}
}

U32 STB6000_TunerGetIF_Freq(STCHIP_Handle_t hChip)
{
	U32 ifreq = 0;

	if(hChip && hChip->pData)
	{
		ifreq = ((SAT_TUNER_Params_t) (hChip->pData))->IF;
	}

	return ifreq;
}

void STB6000_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->BandSelect = (TUNER_WIDEBandS_t)BandSelect;
	}
}

U8 STB6000_TunerGetBandSelect(STCHIP_Handle_t hChip)
{
	U32 ifreq = 0;

	if(hChip && hChip->pData)
	{
		ifreq = ((SAT_TUNER_Params_t) (hChip->pData))->BandSelect;
	}

	return (U8)ifreq;
}


TUNER_Error_t STB6000_TunerTerm(STCHIP_Handle_t hTuner)
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




