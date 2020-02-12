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
 @File   stv6120_tuner.c
 @brief



*/
#ifndef ST_OSLINUX
	#include "string.h"
	#include "stdlib.h"
#endif

#include "globaldefs.h"
#include "fe_sat_tuner.h"
#include "stv6120_tuner.h"


	/* STV6120 tuner definition */
	/*STV6120*/

/*CTRL1*/
#define RSTV6120_CTRL1  0x0000
#define FSTV6120_K  0x000000f8
#define FSTV6120_RDIV  0x00000004
#define FSTV6120_ODIV  0x00000002
#define FSTV6120_MCLKDIV  0x00000001

/*CTRL2*/
#define RSTV6120_CTRL2  0x0001
#define FSTV6120_DCLOOPOFF_1  0x00010080
#define FSTV6120_SDOFF_1  0x00010040
#define FSTV6120_SYN_1  0x00010020
#define FSTV6120_REFOUTSEL_1  0x00010010
#define FSTV6120_BBGAIN_1  0x0001000f

/*CTRL3*/
#define RSTV6120_CTRL3  0x0002
#define FSTV6120_NDIV_1_LSB  0x000200ff

/*CTRL4*/
#define RSTV6120_CTRL4  0x0003
#define FSTV6120_F_1_L  0x000300fe
#define FSTV6120_NDIV_1_MSB  0x00030001

/*CTRL5*/
#define RSTV6120_CTRL5  0x0004
#define FSTV6120_F_1_M  0x000400ff

/*CTRL6*/
#define RSTV6120_CTRL6  0x0005
#define FSTV6120_ICP_1  0x00050070
#define FSTV6120_VCOILOW_1  0x00050008
#define FSTV6120_F_1_H  0x00050007

/*CTRL7*/
#define RSTV6120_CTRL7  0x0006
#define FSTV6120_RCCLFOFF_1  0x00060080
#define FSTV6120_PDIV_1  0x00060060
#define FSTV6120_CF_1  0x0006001f

/*CTRL8*/
#define RSTV6120_CTRL8  0x0007
#define FSTV6120_TCAL  0x000700c0
#define FSTV6120_CALTIME_1  0x00070020
#define FSTV6120_CFHF_1  0x0007001f

/*STAT1*/
#define RSTV6120_STAT1  0x0008
#define FSTV6120_CALVCOSTRT_1  0x00080004
#define FSTV6120_CALRCSTRT_1  0x00080002
#define FSTV6120_LOCK_1  0x00080001

/*CTRL9*/
#define RSTV6120_CTRL9  0x0009
#define FSTV6120_RFSEL_2  0x0009000c
#define FSTV6120_RFSEL_1  0x00090003

/*CTRL10*/
#define RSTV6120_CTRL10  0x000a
#define FSTV6120_LNADON  0x000a0020
#define FSTV6120_LNACON  0x000a0010
#define FSTV6120_LNABON  0x000a0008
#define FSTV6120_LNAAON  0x000a0004
#define FSTV6120_PATHON_2  0x000a0002
#define FSTV6120_PATHON_1  0x000a0001

/*CTRL11*/
#define RSTV6120_CTRL11  0x000b
#define FSTV6120_DCLOOPOFF_2  0x000b0080
#define FSTV6120_SDOFF_2  0x000b0040
#define FSTV6120_SYN_2  0x000b0020
#define FSTV6120_REFOUTSEL_2  0x000b0010
#define FSTV6120_BBGAIN_2  0x000b000f

/*CTRL12*/
#define RSTV6120_CTRL12  0x000c
#define FSTV6120_NDIV_2_LSB  0x000c00ff

/*CTRL13*/
#define RSTV6120_CTRL13  0x000d
#define FSTV6120_F_2_L  0x000d00fe
#define FSTV6120_NDIV_2_MSB  0x000d0001

/*CTRL14*/
#define RSTV6120_CTRL14  0x000e
#define FSTV6120_F_2_M  0x000e00ff

/*CTRL15*/
#define RSTV6120_CTRL15  0x000f
#define FSTV6120_ICP_2  0x000f0070
#define FSTV6120_VCOILOW_2  0x000f0008
#define FSTV6120_F_2_H  0x000f0007

/*CTRL16*/
#define RSTV6120_CTRL16  0x0010
#define FSTV6120_RCCLFOFF_2  0x00100080
#define FSTV6120_PDIV_2  0x00100060
#define FSTV6120_CF_2  0x0010001f

/*CTRL17*/
#define RSTV6120_CTRL17  0x0011
#define FSTV6120_CALTIME_2  0x00110020
#define FSTV6120_CFHF_2  0x0011001f

/*STAT2*/
#define RSTV6120_STAT2  0x0012
#define FSTV6120_CALVCOSTRT_2  0x00120004
#define FSTV6120_CALRCSTRT_2  0x00120002
#define FSTV6120_LOCK_2  0x00120001

/*CTRL18*/
#define RSTV6120_CTRL18  0x0013
#define FSTV6120_TEST1  0x001300ff

/*CTRL19*/
#define RSTV6120_CTRL19  0x0014
#define FSTV6120_TEST2  0x001400ff

/*CTRL20*/
#define RSTV6120_CTRL20  0x0015
#define FSTV6120_TEST3  0x001500ff

/*CTRL21*/
#define RSTV6120_CTRL21  0x0016
#define FSTV6120_TEST4  0x001600ff

/*CTRL22*/
#define RSTV6120_CTRL22  0x0017
#define FSTV6120_TEST5  0x001700ff

/*CTRL23*/
#define RSTV6120_CTRL23  0x0018
#define FSTV6120_TEST6  0x001800ff


static U32 STB6120_LOOKUP[7][3]=	{/* low			high	    lcp */
										{2300000,	2388000,	0},
										{2389000,	2610000,	1},  
										{2611000,	2919000,	2},  
										{2920000,	3273000,	3},
										{3274000,	3715000,	5},
										{3716000,	4246000,	6},
										{4247000,	4600000,	7}
									};
static FE_LLA_LOOKUP_t FE_STV6120_Gain_LookUp =	{		
															75,
															{
																{	7461   		,		0    },  /*	74.61 dB	*/
																{	7443		,	18711	 },  /*	74.43 dB	*/
																{	7336		,	23432	 },  /*	73.36 dB	*/
																{	7240		,	25123	 },  /*	72.4  dB	*/
																{	7147		,	26305	 },  /*	71.47 dB	*/
																{	7047		,	27100	 },  /*	70.47 dB	*/
																{	6950		,	27741	 },  /*	69.5  dB	*/
																{	6852		,	28271	 },  /*	68.52 dB	*/
																{	6752		,	28737	 },  /*	67.52 dB	*/
																{	6653		,	29120	 },  /*	66.53 dB	*/
																{	6554		,	29504	 },  /*	65.54 dB	*/
																{	6455		,	29857	 },  /*	64.55 dB	*/
																{	6355		,	30180	 },  /*	63.55 dB	*/
																{	6256		,	30490	 },  /*	62.56 dB	*/
																{	6157		,	30815	 },  /*	61.57 dB	*/
																{	6058		,	31088	 },  /*	60.58 dB	*/
																{	5957		,	31345	 },  /*	59.57 dB	*/
																{	5859		,	31600	 },  /*	58.59 dB	*/
																{	5760		,	31840	 },  /*	57.6  dB	*/
																{	5660		,	32096	 },  /*	56.6  dB	*/
																{	5559		,	32320	 },  /*	55.59 dB	*/
																{	5459		,	32544	 },  /*	54.59 dB	*/
																{	5361		,	32752	 },  /*	53.61 dB	*/
																{	5261		,	32960	 },  /*	52.61 dB	*/
																{	5161		,	33184	 },  /*	51.61 dB	*/
																{	5062		,	33392	 },  /*	50.62 dB	*/
																{	4961		,	33584	 },  /*	49.61 dB	*/
																{	4861		,	33775	 },  /*	48.61 dB	*/
																{	4762		,	33967	 },  /*	47.62 dB	*/
																{	4662		,	34160	 },  /*	46.62 dB	*/
																{	4562		,	34352	 },  /*	45.62 dB	*/
																{	4462		,	34543	 },  /*	44.62 dB	*/
																{	4362		,	34719	 },  /*	43.62 dB	*/
																{	4262		,	34910	 },  /*	42.62 dB	*/
																{	4162		,	35103	 },  /*	41.62 dB	*/
																{	4062		,	35295	 },  /*	40.62 dB	*/
																{	3962		,	35488	 },  /*	39.62 dB	*/
																{	3862		,	35680	 },  /*	38.62 dB	*/
																{	3762		,	35870	 },  /*	37.62 dB	*/
																{	3662		,	36095	 },  /*	36.62 dB	*/
																{	3562		,	36289	 },  /*	35.62 dB	*/
																{	3462		,	36500	 },  /*	34.62 dB	*/
																{	3362		,	36704	 },  /*	33.62 dB	*/
																{	3262		,	36912	 },  /*	32.62 dB	*/
																{	3162		,	37152	 },  /*	31.62 dB	*/
																{	3062		,	37375	 },  /*	30.62 dB	*/
																{	2962		,	37600	 },  /*	29.62 dB	*/
																{	2862		,	37823	 },  /*	28.62 dB	*/
																{	2762		,	38048	 },  /*	27.62 dB	*/
																{	2662		,	38240	 },  /*	26.62 dB	*/
																{	2562		,	38479	 },  /*	25.62 dB	*/
																{	2462		,	38720	 },  /*	24.62 dB	*/
																{	2363		,	38976	 },  /*	23.63 dB	*/
																{	2263		,	39226	 },  /*	22.63 dB	*/
																{	2162		,	39520	 },  /*	21.62 dB	*/
																{	2062		,	39792	 },  /*	20.62 dB	*/
																{	1962		,	40064	 },  /*	19.62 dB	*/
																{	1862		,	40351	 },  /*	18.62 dB	*/
																{	1762		,	40640	 },  /*	17.62 dB	*/
																{	1662		,	40976	 },  /*	16.62 dB	*/
																{	1562		,	41295	 },  /*	15.62 dB	*/
																{	1462		,	41631	 },  /*	14.62 dB	*/
																{	1362		,	41934	 },  /*	13.62 dB	*/
																{	1262		,	42354	 },  /*	12.62 dB	*/
																{	1162		,	42815	 },  /*	11.62 dB	*/
																{	1062		,	43263	 },  /*	10.62 dB	*/
																{	962			,	43743    },  /*	9.62  dB	*/
																{	862			,	44288    },  /*	8.62  dB	*/
																{	762			,	44913    },  /*	7.62  dB	*/
																{	662			,	45712    },  /*	6.62  dB	*/
																{	563			,	46720    },  /*	5.63  dB	*/
																{	463			,	48164    },  /*	4.63  dB	*/
																{	363			,	50816    },  /*	3.63  dB	*/
																{	294			,	65534    },  /*	2.94  dB	*/
																{	295			,	65535    }  /*	2.95  dB	*/
															}
														};

TUNER_Error_t STV6120_TunerInit(void *pTunerInit_v,STCHIP_Handle_t *TunerHandle)
{
	SAT_TUNER_Params_t	hTunerParams = NULL;
	STCHIP_Info_t	ChipInfo;
	STCHIP_Handle_t hTuner = NULL;
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t pTunerInit = (SAT_TUNER_Params_t)pTunerInit_v;

 	STCHIP_Register_t		DefSTV6120Val[STV6120_NBREGS]=
	 {
	 	{RSTV6120_CTRL1		,0x75},
	 	{RSTV6120_CTRL2		,0x33},
	 	{RSTV6120_CTRL3		,0xCE},
	 	{RSTV6120_CTRL4		,0x54},
	 	{RSTV6120_CTRL5		,0x55},
	 	{RSTV6120_CTRL6		,0x0D},
	 	{RSTV6120_CTRL7		,0x32},
	 	{RSTV6120_CTRL8		,0x44},
	 	{RSTV6120_STAT1		,0x0E},
	 	{RSTV6120_CTRL9		,0xF9},
	 	{RSTV6120_CTRL10	,0x5B},
	 	{RSTV6120_CTRL11	,0x33},
	 	{RSTV6120_CTRL12	,0xCE},
	 	{RSTV6120_CTRL13	,0x54},
	 	{RSTV6120_CTRL14	,0x55},
	 	{RSTV6120_CTRL15	,0x0D},
	 	{RSTV6120_CTRL16	,0x32},
	 	{RSTV6120_CTRL17	,0x44},
	 	{RSTV6120_STAT2		,0x0E},
	 	{RSTV6120_CTRL18	,0x00},
	 	{RSTV6120_CTRL19	,0x00},
	 	{RSTV6120_CTRL20	,0x4C},
	 	{RSTV6120_CTRL21	,0x00},
	 	{RSTV6120_CTRL22	,0x00},
	 	{RSTV6120_CTRL23	,0x4C}
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
		hTunerParams = (SAT_TUNER_Params_t)((*TunerHandle)->pData);
		memcpy(&ChipInfo, *TunerHandle,sizeof(STCHIP_Info_t));
	}
	else
	{
		error=TUNER_INVALID_HANDLE;	
	}
    #endif

    if(hTunerParams != NULL)
    {
    	hTunerParams->Model		= pTunerInit->Model;    		/* Tuner model */
    	hTunerParams->Reference = pTunerInit->Reference;		/* Reference Clock in Hz */
    	hTunerParams->IF = pTunerInit->IF; 						/* IF Hz intermediate frequency */
    	hTunerParams->IQ_Wiring = pTunerInit->IQ_Wiring;		/* hardware IQ invertion */
    	hTunerParams->BandSelect = pTunerInit->BandSelect;		/* Wide band tuner (6130 like, band selection) */
	//printf("\n6120>>>INIT the BAndSelect 1 is %d",hTunerParams->BandSelect);	
	hTunerParams->BandBorder = pTunerInit->BandBorder;		/* Wide band tuner low band, high band border in KHz (6120 like) */
	/*commented in 900 LLA4.6 Path selection is now based on BandSelect */
	/*hTunerParams->TunerPath  = pTunerInit->TunerPath;*/		/* Used path ,Path1 or Path2 for dual tuner */

	hTunerParams->BandSelect2 = pTunerInit->BandSelect2;	/* Dual Wide band tuner (6120 like, band selection) */
	//printf("\n6120>>>INIT the BAndSelect 2 is %d",hTunerParams->BandSelect2);
	hTunerParams->InputSelect = pTunerInit->InputSelect;	/* Input Selection for dual tuner*/
    	hTunerParams->RF_Source= pTunerInit->RF_Source;

		/*fix Coverity CID 20142 */		
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


		ChipInfo.NbRegs   = STV6120_NBREGS;
		ChipInfo.NbFields = STV6120_NBFIELDS;
		ChipInfo.ChipMode = STCHIP_MODE_SUBADR_8;
		ChipInfo.WrStart  = RSTV6120_CTRL1;
		ChipInfo.WrSize   = 25;
		ChipInfo.RdStart  = RSTV6120_CTRL1;
		ChipInfo.RdSize   = 25;

		#ifdef HOST_PC
			hTuner = ChipOpen(&ChipInfo);
			(*TunerHandle) = hTuner;
		#endif
		#ifdef CHIP_STAPI
			hTuner = *TunerHandle;
			memcpy(hTuner, &ChipInfo, sizeof(STCHIP_Info_t));
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
			ChipUpdateDefaultValues(hTuner,DefSTV6120Val);

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
					ChipSetFieldImage(hTuner,FSTV6120_ODIV,0);
				break;

				case 2:
					ChipSetFieldImage(hTuner,FSTV6120_ODIV,1);
				break;

				case 4:
				case 8:
				case 0: /*Tuner output clock not used then divide by 8 (the 6120 can not stop completely the out clock)*/
				break;
			}

			/*update K field according to the Reference clock (quartz)*/
			ChipSetFieldImage(hTuner,FSTV6120_K,((hTunerParams->Reference/1000000)-16));

			/*update RDIV field according to the Reference clock (quartz)*/
			if(hTunerParams->Reference>=27000000) /*if FXTAL >=27MHz set R to 2 else if FXTAL <27MHz set R to 1 */
				ChipSetFieldImage(hTuner,FSTV6120_RDIV,1);
			else
				ChipSetFieldImage(hTuner,FSTV6120_RDIV,0);

			/*As output A/D are not used they are made 0 for power saving.*/
			ChipSetFieldImage(hTuner,FSTV6120_LNAAON,0);
			ChipSetFieldImage(hTuner,FSTV6120_LNADON,0);	
			
				
			/*I2C registers initialization*/ 
			ChipGetRegisters(hTuner,RSTV6120_CTRL9,1); /* mandatory read when 2 instance of the 6120 are used (one for each path)*/
			
			error=STV6120_TunerWrite(hTuner);
	/*************R/W test code*************************
	STV6120_TunerRead(hTuner);
	{
		int i=0;
		printf("\n TUNER READ");
		for(i=0;i<STV6120_NBREGS+1;i++)
		{
		        printf("\nRegister[%d] =0x%x",hTuner->pRegMapImage[i].Addr,hTuner->pRegMapImage[i].Value);
		}
	}
***************************************************/
			
			hTunerParams->BandBorder=0;  /* when the STV6120 is used as wideband tuner set this value to the upper limit of the low band*/

			switch(hTunerParams->RF_Source)
			{
				case FE_RF_SOURCE_A:
					if(hTunerParams->InputSelect==TUNER_INPUT1)
					{	
						ChipSetField(hTuner,FSTV6120_RFSEL_1,0);
					}
					else
					{
						ChipSetField(hTuner,FSTV6120_RFSEL_2,0);
					}
				break;

				case FE_RF_SOURCE_B:
					if(hTunerParams->InputSelect==TUNER_INPUT1)
					{	
						ChipSetField(hTuner,FSTV6120_RFSEL_1,1);
					}
					else
					{
						ChipSetField(hTuner,FSTV6120_RFSEL_2,1);
					}
				break;

				case FE_RF_SOURCE_C:
					if(hTunerParams->InputSelect==TUNER_INPUT1)
					{	
						ChipSetField(hTuner,FSTV6120_RFSEL_1,2);
					}
					else
					{
						ChipSetField(hTuner,FSTV6120_RFSEL_2,2);
					}
				break;

				case FE_RF_SOURCE_D:
					if(hTunerParams->InputSelect==TUNER_INPUT1)
					{	
						ChipSetField(hTuner,FSTV6120_RFSEL_1,3);
					}
					else
					{
						ChipSetField(hTuner,FSTV6120_RFSEL_2,3);
					}
				break;
				default:
				break;
			}

		//printf("\n<%s><%d> read reg CTRL9 Err =%d..value is 0x%x",__func__,__LINE__,ChipGetRegisters(hTuner,RSTV6120_CTRL9,1),ChipGetOneRegister(hTuner,RSTV6120_CTRL9));
		}

	}
	else
		error=TUNER_INVALID_HANDLE;
	
	return error;
}

TUNER_Error_t STV6120_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn) 
{
	TUNER_Error_t error = TUNER_NO_ERR; 
	SAT_TUNER_Params_t hTunerParams = NULL;

	/*fix Coverity CID: 20138 */
	if(hTuner)
	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
		if(hTunerParams && !hTuner->Error)	
		{
			if(StandbyOn) /*Power down ON*/
			{
				if(hTunerParams->Model==TUNER_STV6120_Tuner1)  
				{
					ChipSetField(hTuner,FSTV6120_SYN_1,0);
					ChipSetField(hTuner,FSTV6120_SDOFF_1,1);
					ChipSetField(hTuner,FSTV6120_PATHON_1,0); 
				}
				else if(hTunerParams->Model==TUNER_STV6120_Tuner2)  
				{
					ChipSetField(hTuner,FSTV6120_SYN_2,0);
					ChipSetField(hTuner,FSTV6120_SDOFF_2,1);
					ChipSetField(hTuner,FSTV6120_PATHON_2,0); 
				}
			}
			else	/*Power down OFF*/
			{
				if(hTunerParams->Model==TUNER_STV6120_Tuner1)  
				{
					ChipSetField(hTuner,FSTV6120_SYN_1,1);
					ChipSetField(hTuner,FSTV6120_SDOFF_1,0);
					ChipSetField(hTuner,FSTV6120_PATHON_1,1); 
				}
				else if(hTunerParams->Model==TUNER_STV6120_Tuner2)  
				{
					ChipSetField(hTuner,FSTV6120_SYN_2,1);
					ChipSetField(hTuner,FSTV6120_SDOFF_2,0);
					ChipSetField(hTuner,FSTV6120_PATHON_2,1); 
				}
			}
		}
	}
	else
	{
		error=TUNER_INVALID_HANDLE;	
	}


	return error;
}


TUNER_Error_t STV6120_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;

	U32 divider,
		swallow,
		p,p2,
		rDiv,
		frequency2,
		fequencyVco,
		fequencyVco2,
		delta;
	
	S32 i;
	U8 u8;
		
	TUNER_WIDEBandS_t BandSelect;
	/*fix Coverity CID: 20135 */
	if(hTuner)
	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
		
	if(hTunerParams && !hTuner->Error)	
	{

		rDiv=ChipGetFieldImage(hTuner,FSTV6120_RDIV);

		if (Frequency<=325000) /* in kHz */
		{
			p=3;									   
		}
		else if (Frequency<=650000)
		{
			p=2;
		}
		else if (Frequency</*1300000*/1151000)  /*6120 T50*/ 
		{
			p=1;
		}
		else 
		{
			p=0;
		}
		//printf("\n%sthe F =%d(from App)",__FUNCTION__,Frequency);
		/* Read second path frequency*/
		if(hTunerParams->Model==TUNER_STV6120_Tuner2)
		{
			
			hTunerParams->Model=TUNER_STV6120_Tuner1;
			frequency2= FE_Sat_TunerGetFrequency(hTuner);                     
			hTunerParams->Model=TUNER_STV6120_Tuner2; 
			p2=ChipGetField(hTuner,FSTV6120_PDIV_1);
		}
		else
		{
			hTunerParams->Model=TUNER_STV6120_Tuner2; 
			frequency2= FE_Sat_TunerGetFrequency(hTuner);                     
			hTunerParams->Model=TUNER_STV6120_Tuner1;  
			p2=ChipGetField(hTuner,FSTV6120_PDIV_2);
		}
		
		/*Path1 Frequency and Path2 Frequency must distant by 2MHz at least */
		/* if Not move wanted Freqency by 2MHz from the other path frequency  */
		/*
		if(ABS ((S32)Frequency - (S32)frequency2)<2000)
		{
			if(Frequency>frequency2)
			{
				Frequency=frequency2+2000;	
			}
			else
			{
				Frequency=frequency2-2000;	
			}

		}
		*/
		fequencyVco=Frequency*PowOf2(p+1);
		fequencyVco2=frequency2*PowOf2(p2+1);
		
		delta=4000*PowOf2(ABS((S32)p2-(S32)p));
		
		if(ABS ((S32)fequencyVco - (S32)fequencyVco2)<delta)  
		{
			if(fequencyVco>fequencyVco2)
			{
				if(p>p2)
					Frequency=(frequency2/(delta/4000))+2000;
				else
					Frequency=(frequency2*(delta/4000))+2000;
			}
			else
			{
				if(p>p2)
					Frequency=(frequency2/(delta/4000))-2000;
				else
					Frequency=(frequency2*(delta/4000))-2000;
			}

		}


		swallow= (Frequency *PowOf2(p+rDiv+1));
		divider= swallow/(hTunerParams->Reference/1000);  /*N = (Frequency *P*R)/Fxtal */
		
		swallow = swallow % (hTunerParams->Reference/1000); /*F = (2^18 *(Frequency *P*R)%Fxtal)/Fxtal */ 
		if(swallow < 0x3FFF) /*to avoid U32 bits saturation*/
			swallow = (0x40000 * swallow)/(hTunerParams->Reference/1000);
		else if (swallow < 0x3FFF0) /*to avoid U32 bits saturation*/
			swallow = (0x8000 * swallow)/(hTunerParams->Reference/8000);
		else			   /*to avoid U32 bits saturation*/
			swallow = (0x2000 * swallow)/(hTunerParams->Reference/32000);
			
		
		switch(hTunerParams->InputSelect)
		{
			case TUNER_INPUT1:
			default:
				BandSelect=hTunerParams->BandSelect;
			break;
				
			case TUNER_INPUT2:
				BandSelect=hTunerParams->BandSelect2; 
			break;
		}
		
		
		if(hTunerParams->Model==TUNER_STV6120_Tuner1)  
		{
#if 0    /* formerly excluded lines */
				if(hTunerParams->BandBorder>0)
				{
			
					/*
						if BandBorder is > 0 the the 6120 is used as a dual wide band tuner
					 	update the input slection A,B,C or D according to the band definition
					*/
			
					/* if the BandBorder is equal to 0 the tuner is used as dual smaple band with 4 input A,B,C or D 
						in this case ise the function "STV6120_TunerSwitchInput" to switch the input to A,B,C or D
					*/
				
					/*
						For dual Wide Band use The function "STV6120_TunerSwitchInput" to switch the input to:
						TUNER_INPUT1 ==> A,B wide band input for this the argument passed to the function must be 0 or 1.
						TUNER_INPUT2 ==> C,D wide band input for this the argument passed to the function must be 2 or 3.
					
					*/

					switch(BandSelect)
					{
						case LBRFA_HBRFB:
						default:
							if(Frequency<hTunerParams->BandBorder)
								ChipSetField(hTuner,FSTV6120_RFSEL_1,0);
							else
								ChipSetField(hTuner,FSTV6120_RFSEL_1,1);	
						break;

						case LBRFB_HBRFA:
							if(Frequency<hTunerParams->BandBorder)
								ChipSetField(hTuner,FSTV6120_RFSEL_1,1);
							else
								ChipSetField(hTuner,FSTV6120_RFSEL_1,0);	
				
						break;
				
						case LBRFC_HBRFD:
							if(Frequency<hTunerParams->BandBorder)
								ChipSetField(hTuner,FSTV6120_RFSEL_1,2);
							else
								ChipSetField(hTuner,FSTV6120_RFSEL_1,3);
				
						break;

						case LBRFD_HBRFC:
							if(Frequency<hTunerParams->BandBorder)
								ChipSetField(hTuner,FSTV6120_RFSEL_1,3);
							else
								ChipSetField(hTuner,FSTV6120_RFSEL_1,2);
				
						break;
					}
				
				}
#endif   /* formerly excluded lines */
			//printf("\n<%s,%d>Input_Select is %d BandSelect =%d..BandBorder =%d",__func__,__LINE__,hTunerParams->InputSelect,BandSelect,hTunerParams->BandBorder);
			u8=0;   /*6120 T50*/
			while(!INRANGE(STB6120_LOOKUP[u8][0],fequencyVco,STB6120_LOOKUP[u8][1]) && (u8<6)) u8++;  /*6120 T50*/
			ChipSetFieldImage(hTuner,FSTV6120_ICP_1,STB6120_LOOKUP[u8][2]);  /*6120 T50*/
			ChipSetRegisters(hTuner,RSTV6120_CTRL6,1);		 /*6120 T50*/
			//printf("\nreading the register CTRL6..set to 6...Err=%d..reg Read is %d",ChipGetRegisters(hTuner,RSTV6120_CTRL6,1),ChipGetOneRegister(hTuner,RSTV6120_CTRL6));
			ChipSetFieldImage(hTuner,FSTV6120_PDIV_1,p);
			ChipSetFieldImage(hTuner,FSTV6120_NDIV_1_LSB,(divider & 0xFF)); 
			ChipSetFieldImage(hTuner,FSTV6120_NDIV_1_MSB,((divider>>8) && 0x01));
			ChipSetFieldImage(hTuner,FSTV6120_F_1_H,((swallow>>15) & 0x07));  
			ChipSetFieldImage(hTuner,FSTV6120_F_1_M,((swallow>>7) & 0xFF));
			ChipSetFieldImage(hTuner,FSTV6120_F_1_L,(swallow & 0x7F));
			
			ChipSetRegisters(hTuner,RSTV6120_CTRL3,5);
			//printf("\nreading the register CTRL3..set to 5...Err = %d..Reg read is %d",ChipGetRegisters(hTuner,RSTV6120_CTRL3,1),ChipGetOneRegister(hTuner,RSTV6120_CTRL3));
			ChipSetField(hTuner,FSTV6120_CALVCOSTRT_1,1);		/* VCO Auto Calibration */
		
			i=0;
			while((i<10) && (ChipGetField(hTuner,FSTV6120_CALVCOSTRT_1)!=0))
			{
				ChipWaitOrAbort(hTuner,1);	/* wait for VCO auto calibration */
				i++;
			}

		}
		else if(hTunerParams->Model==TUNER_STV6120_Tuner2)  
		{
		 
#if 0    /* formerly excluded lines */
				if(hTunerParams->BandBorder>0)
			{
			
				/*
					if BandBorder is > 0 the the 6120 is used as a dual wide band tuner
				 	update the input slection A,B,C or D according to the band definition
				*/
			
				/* if the BandBorder is equal to 0 the tuner is used as dual smaple band with 4 input A,B,C or D 
					in this case ise the function "STV6120_TunerSwitchInput" to switch the input to A,B,C or D
				*/
				
				/*
					For dual Wide Band use The function "STV6120_TunerSwitchInput" to switch the input to:
					TUNER_INPUT1 ==> A,B wide band input for this the argument passed to the function must be 0 or 1.
					TUNER_INPUT2 ==> C,D wide band input for this the argument passed to the function must be 2 or 3.
					
				*/
                         printf("\nTUNER_STV6120_Tuner2   BandSelect  =%d",BandSelect);
				switch(BandSelect)
				{
					case LBRFA_HBRFB:
					default:
						if(Frequency<hTunerParams->BandBorder)
							ChipSetField(hTuner,FSTV6120_RFSEL_2,0);
						else
							ChipSetField(hTuner,FSTV6120_RFSEL_2,1);	
					break;
					

					case LBRFB_HBRFA:
						if(Frequency<hTunerParams->BandBorder)
							ChipSetField(hTuner,FSTV6120_RFSEL_2,1);
						else
							ChipSetField(hTuner,FSTV6120_RFSEL_2,0);	
				
					break;
				
					case LBRFC_HBRFD:
						if(Frequency<hTunerParams->BandBorder)
							ChipSetField(hTuner,FSTV6120_RFSEL_2,2);
						else
							ChipSetField(hTuner,FSTV6120_RFSEL_2,3);
				
					break;

					case LBRFD_HBRFC:
						if(Frequency<hTunerParams->BandBorder)
							ChipSetField(hTuner,FSTV6120_RFSEL_2,3);
						else
							ChipSetField(hTuner,FSTV6120_RFSEL_2,2);
				
					break;
				}
				
			}
#endif   /* formerly excluded lines */
			//printf("\n<%s,%d>*6120_Tuner2_**Input_Select = %d BandSelect =%d..BandBorder =%d",__func__,__LINE__,hTunerParams->InputSelect,hTunerParams->BandSelect,hTunerParams->BandBorder);
			u8=0;   /*6120 T50*/
			while(!INRANGE(STB6120_LOOKUP[u8][0],fequencyVco,STB6120_LOOKUP[u8][1]) && (u8<6)) u8++;  /*6120 T50*/
			ChipSetFieldImage(hTuner,FSTV6120_ICP_2,STB6120_LOOKUP[u8][2]);  /*6120 T50*/
			ChipSetRegisters(hTuner,RSTV6120_CTRL15,1);		 /*6120 T50*/

			ChipSetFieldImage(hTuner,FSTV6120_PDIV_2,p);
			ChipSetFieldImage(hTuner,FSTV6120_NDIV_2_LSB,(divider & 0xFF)); 
			ChipSetFieldImage(hTuner,FSTV6120_NDIV_2_MSB,((divider>>8) && 0x01));
			ChipSetFieldImage(hTuner,FSTV6120_F_2_H,((swallow>>15) & 0x07));  
			ChipSetFieldImage(hTuner,FSTV6120_F_2_M,((swallow>>7) & 0xFF));
			ChipSetFieldImage(hTuner,FSTV6120_F_2_L,(swallow & 0x7F));
			
			ChipSetRegisters(hTuner,RSTV6120_CTRL12,5);
  			
			ChipSetField(hTuner,FSTV6120_CALVCOSTRT_2,1);		/* VCO Auto Calibration */
		
			i=0;
			while((i<10) && (ChipGetField(hTuner,FSTV6120_CALVCOSTRT_2)!=0))
			{
				ChipWaitOrAbort(hTuner,1);	/* wait for VCO auto calibration */
				i++;
			}

		}

	}
	}
	else
	{
		error=TUNER_INVALID_HANDLE;	
	}
		
	return error;
}

U32 STV6120_TunerGetFrequency(STCHIP_Handle_t hTuner)
{

	SAT_TUNER_Params_t hTunerParams = NULL; 
	U32 frequency = 0;
	U32 swallow;
	U32 divider = 0;
	U32 psd2;

	/*fix Coverity CID: 20128 */
	if(hTuner)
	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
		
		if(hTunerParams && !hTuner->Error)	
		{
	 		psd2=ChipGetFieldImage(hTuner,FSTV6120_RDIV);  
	 		if(hTunerParams->Model==TUNER_STV6120_Tuner1)  
	 		{
	 			ChipGetRegisters(hTuner,RSTV6120_CTRL3,5);
	 			divider=MAKEWORD(ChipGetFieldImage(hTuner,FSTV6120_NDIV_1_MSB),ChipGetFieldImage(hTuner,FSTV6120_NDIV_1_LSB));	/*N*/
	 			swallow = ((ChipGetFieldImage(hTuner,FSTV6120_F_1_H) & 0x07)<<15) +
	 					  ((ChipGetFieldImage(hTuner,FSTV6120_F_1_M) & 0xFF)<<7) +
	 					  ( ChipGetFieldImage(hTuner,FSTV6120_F_1_L) & 0x7F );
 			
	 			frequency =(hTunerParams->Reference/1000)/PowOf2(ChipGetFieldImage(hTuner,FSTV6120_RDIV));
	 			frequency=(frequency*divider)+((frequency*swallow)/0x40000);
	 			frequency /= PowOf2(1+ ChipGetFieldImage(hTuner,FSTV6120_PDIV_1));
 			 
	 		}
 		
	 		else if(hTunerParams->Model==TUNER_STV6120_Tuner2)  
	 		{
	 			ChipGetRegisters(hTuner,RSTV6120_CTRL12,5);
	 			divider=MAKEWORD(ChipGetFieldImage(hTuner,FSTV6120_NDIV_2_MSB),ChipGetFieldImage(hTuner,FSTV6120_NDIV_2_LSB));	/*N*/
	 			swallow = ((ChipGetFieldImage(hTuner,FSTV6120_F_2_H) & 0x07)<<15) +
	 					  ((ChipGetFieldImage(hTuner,FSTV6120_F_2_M) & 0xFF)<<7) + 
	 					  ( ChipGetFieldImage(hTuner,FSTV6120_F_2_L) & 0x7F );
 			
	 			frequency =(hTunerParams->Reference/1000)/PowOf2(ChipGetFieldImage(hTuner,FSTV6120_RDIV));
	 			frequency=(frequency*divider)+((frequency*swallow)/0x40000);
	 			frequency /= PowOf2(1+ ChipGetFieldImage(hTuner,FSTV6120_PDIV_2)); 
 			 
	 		}

		}
	}

	return frequency;
} 


TUNER_Error_t STV6120_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth)
{
	TUNER_Error_t error = TUNER_NO_ERR; 
	SAT_TUNER_Params_t hTunerParams = NULL;
	U8 u8;
	S32 i=0; 
	
	/*fix Coverity CID: 20134 */
	if(hTuner)
	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
		
		if(hTunerParams && !hTuner->Error)	
		{

			if((Bandwidth/2) > 36000000)   /*F[4:0] BW/2 max =31+5=36 mhz for F=31*/
				u8 = 31;
			else if((Bandwidth/2) < 5000000) /* BW/2 min = 5Mhz for F=0 */
				u8 = 0;
			else							 /*if 5 < BW/2 < 36*/
				u8 = (Bandwidth/2)/1000000 - 5;
		
			if(hTunerParams->Model==TUNER_STV6120_Tuner1)  
			{
				ChipSetField(hTuner,FSTV6120_CF_1,u8);		 /* Set the LPF value */	
				ChipSetField(hTuner,FSTV6120_CALRCSTRT_1,1); /* Start LPF auto calibration*/
		
				i=0;
				while((i<10) && (ChipGetField(hTuner,FSTV6120_CALRCSTRT_1)!=0))
				{
					ChipWaitOrAbort(hTuner,1);	/* wait for LPF auto calibration */
					i++;
				}
			}
			else if(hTunerParams->Model==TUNER_STV6120_Tuner2)  
			{
				ChipSetField(hTuner,FSTV6120_CF_2,u8);		 /* Set the LPF value */	
				ChipSetField(hTuner,FSTV6120_CALRCSTRT_2,1); /* Start LPF auto calibration*/
		
				i=0;
				while((i<10) && (ChipGetField(hTuner,FSTV6120_CALRCSTRT_2)!=0))
				{
					ChipWaitOrAbort(hTuner,1);	/* wait for LPF auto calibration */
					i++;
				}
			}

		}
	}

	return error;
}

U32 STV6120_TunerGetBandwidth(STCHIP_Handle_t hTuner)
{

	SAT_TUNER_Params_t hTunerParams = NULL;
	U32 bandwidth = 0;
	U8 u8 = 0;
	
	/*fix Coverity CID: 20127 */
	if(hTuner)
	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
	
		if(hTunerParams && !hTuner->Error)	
		{
			if(hTunerParams->Model==TUNER_STV6120_Tuner1)  
				u8 = ChipGetField(hTuner,FSTV6120_CF_1);
			else 	if(hTunerParams->Model==TUNER_STV6120_Tuner2)  
				u8 = ChipGetField(hTuner,FSTV6120_CF_2);
			
			bandwidth = (u8+5)*2000000;	/* x2 for ZIF tuner BW/2=F+5 Mhz*/
			
		}
	}

	return bandwidth;
}

TUNER_Error_t STV6120_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
		
	/*fix Coverity CID: 20136 */
	if(hTuner)
	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
	
		if(hTunerParams && !hTuner->Error)	
		{
	 		if(hTunerParams->Model==TUNER_STV6120_Tuner1)  
				ChipSetField(hTuner,FSTV6120_BBGAIN_1,(Gain/2));
			else if(hTunerParams->Model==TUNER_STV6120_Tuner2)   
				ChipSetField(hTuner,FSTV6120_BBGAIN_2,(Gain/2));
		}

	}
	else
	{
		error=TUNER_INVALID_HANDLE;	
	}

	
	return error;
}

S32 STV6120_TunerGetGain(STCHIP_Handle_t hTuner)
{

	SAT_TUNER_Params_t hTunerParams = NULL;
	S32 gain=0;
	
	/*fix Coverity CID: 20129 */
	if(hTuner)
	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
	
		if(hTunerParams && !hTuner->Error)	
		{
			if(hTunerParams->Model==TUNER_STV6120_Tuner1)  
				gain=2*ChipGetField(hTuner,FSTV6120_BBGAIN_1);
			else if(hTunerParams->Model==TUNER_STV6120_Tuner2)  
				gain=2*ChipGetField(hTuner,FSTV6120_BBGAIN_2);
		}

	}

	return gain;
}

TUNER_Error_t STV6120_TunerSetHMR_Filter(STCHIP_Handle_t hTuner,S32 filterValue) 
{
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
		
	/*fix Coverity SD30*/
	if(hTuner)
	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
		
		if(hTunerParams && !hTuner->Error)	
		{
			if(hTunerParams->Model==TUNER_STV6120_Tuner1) 
				ChipSetField(hTuner,FSTV6120_CFHF_1,filterValue);
			else
				ChipSetField(hTuner,FSTV6120_CFHF_2,filterValue);
		}
	}
	else
	{
		error=TUNER_INVALID_HANDLE;	
	}

	return error;
}

TUNER_Error_t STV6120_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider)
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
	
	/*fix Coverity CID: 20137 */
	if(hTuner)
	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
	
		if(hTunerParams && !hTuner->Error)	
		{

			/*Allowed values 1,2,4 and 8*/
			switch(Divider)
			{
				case 1:
				default:
					ChipSetField(hTuner,FSTV6120_ODIV,0);
				break;
					
				case 2:
				case 4:
				case 8:
				case 0:
					ChipSetField(hTuner,FSTV6120_ODIV,1);
				break;
			}
		}
	}
	else
	{
		error=TUNER_INVALID_HANDLE;	
	}

	
	return error;
}

TUNER_Error_t STV6120_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn)
{
	return TUNER_NO_ERR;
}

BOOL STV6120_TunerGetAttenuator(STCHIP_Handle_t hTuner)
{
	return FALSE;
}


BOOL STV6120_TunerGetStatus(STCHIP_Handle_t hTuner)
{

	SAT_TUNER_Params_t hTunerParams = NULL;
	BOOL locked = FALSE;
	
	/*fix Coverity CID: 20130 */
	if(hTuner)
	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
		
		if(hTunerParams && !hTuner->Error)	
		{
			if(hTunerParams->Model==TUNER_STV6120_Tuner1)  
				locked = ChipGetField(hTuner,FSTV6120_LOCK_1);
			else if(hTunerParams->Model==TUNER_STV6120_Tuner2)  
				locked = ChipGetField(hTuner,FSTV6120_LOCK_2);
		}
	}

	return locked;
}

TUNER_Error_t STV6120_TunerSwitchInput(STCHIP_Handle_t hTuner, U8 Input)
{
	/* Switch the tuner input to input A,B,C or D
	
	when the tuner is used as sample band i.e BandBorder=0 then
	0 ==> input A
	1 ==> input B
	2 ==> input C
	3 ==> input D
	
	when the tuner is used as wide band i.e BandBorder >0 then
	0 ==> input 1 (A,B)  
	1 ==> input 1 (A,B)	
	2 ==> input 2 (C,D)
	3 ==> input 2 (C,D)
	
	The selection between A and B when input1 or C and D when input2 
	depend on the frequency (>BandBorder or <BandBorder) and made when setting the ffrequency 
	
	*/

	
	TUNER_Error_t error = TUNER_NO_ERR;
	SAT_TUNER_Params_t hTunerParams = NULL;
	/*fix Coverity CID: 20139 */
	if(hTuner)
	{
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
		
		if(hTunerParams && !hTuner->Error)	
		{
			switch(Input)
			{
				case 0:	/* input A*/
				default:
					hTunerParams->InputSelect=TUNER_INPUT1;
				break;
			
				case 1: /* input B*/
					hTunerParams->InputSelect=TUNER_INPUT1;
				break;
			
				case 2: /* input C*/
					hTunerParams->InputSelect=TUNER_INPUT2;
				break;
			
				case 3: /* input D*/
					hTunerParams->InputSelect=TUNER_INPUT2;
				break;
			}
		
			if(hTunerParams->Model==TUNER_STV6120_Tuner1)  
				ChipSetFieldImage(hTuner,FSTV6120_RFSEL_1,Input);	
			else if(hTunerParams->Model==TUNER_STV6120_Tuner2)  
				ChipSetFieldImage(hTuner,FSTV6120_RFSEL_2,Input);
			
			ChipSetRegisters(hTuner,RSTV6120_CTRL9,1);
		
		}
	}
	else
	{
		error=TUNER_INVALID_HANDLE;	
	}

	return error; 
	
}

S32 STV6120_TunerGetRFGain(STCHIP_Handle_t hTuner,U32 AGCIntegrator,S32 BBGain)
{
	S32 Gain100dB=1,RefBBgain=12,
		Imin,Imax,i;
		
	SAT_TUNER_Params_t hTunerParams = NULL; 
		
	if(hTuner)	
	{
			hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;   
			if(hTunerParams && !hTuner->Error)
			{
				Imin = 0;
				Imax = FE_STV6120_Gain_LookUp.size-1;
		
				if(AGCIntegrator<=FE_STV6120_Gain_LookUp.table[0].regval)
					Gain100dB=FE_STV6120_Gain_LookUp.table[0].realval;
				else if(AGCIntegrator>=FE_STV6120_Gain_LookUp.table[Imax].regval)
					Gain100dB=FE_STV6120_Gain_LookUp.table[Imax].realval;

				else 
				{
					while((Imax-Imin)>1)
					{
						i=(Imax+Imin)>>1;
						/*equivalent to i=(Imax+Imin)/2; */
						if(INRANGE(FE_STV6120_Gain_LookUp.table[Imin].regval,AGCIntegrator,FE_STV6120_Gain_LookUp.table[i].regval))
							Imax = i;
						else
							Imin = i;
					}
			
					Gain100dB =	(((S32)AGCIntegrator - FE_STV6120_Gain_LookUp.table[Imin].regval)
							* (FE_STV6120_Gain_LookUp.table[Imax].realval - FE_STV6120_Gain_LookUp.table[Imin].realval)
							/ (FE_STV6120_Gain_LookUp.table[Imax].regval - FE_STV6120_Gain_LookUp.table[Imin].regval))
							+ FE_STV6120_Gain_LookUp.table[Imin].realval;
				}
		   }
	}
	
	Gain100dB=Gain100dB+100*(BBGain-RefBBgain); 
	return Gain100dB;

}

TUNER_Error_t STV6120_TunerWrite(STCHIP_Handle_t hTuner)
{
 TUNER_Error_t error = TUNER_NO_ERR;
 STCHIP_Error_t chipError=CHIPERR_NO_ERROR;
 SAT_TUNER_Params_t hTunerParams = NULL;
 U8 i=0;
  
 /*fix Coverity CID: 20131 */
 if(hTuner)
 {
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
		
		 if(hTunerParams && !hTuner->Error)	
		 {
		      if((hTunerParams->DemodModel == DEMOD_STI7111)&&(hTuner->RepeaterHost->ChipID <0x20))
		      {
		         for(i=hTuner->WrStart;i<hTuner->WrSize;i++) /*Write one register at a time for 7111 cut1.0*/
		           chipError |= ChipSetRegisters(hTuner,i, 1);
		      }
		      else
		      {
/************************************
printf("\n TUNER WRITE ");
		for(i=0;i<STV6120_NBREGS+1;i++)
		{
		        printf("\nRegister[%d] =0x%x",hTuner->pRegMapImage[i].Addr,hTuner->pRegMapImage[i].Value);
		}

*********************************/
	  
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
 }
 else
 {
		error=TUNER_INVALID_HANDLE;	
 }

 return error;
}

TUNER_Error_t STV6120_TunerRead(STCHIP_Handle_t hTuner)
{
 TUNER_Error_t error = TUNER_NO_ERR;
 STCHIP_Error_t chipError=CHIPERR_NO_ERROR;
 SAT_TUNER_Params_t hTunerParams = NULL;
 U8 i=0;

 /*fix Coverity CID: 20133 */
 if(hTuner)
 {
		hTunerParams = (SAT_TUNER_Params_t) hTuner->pData;
	
		if(hTunerParams && !hTuner->Error)	
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
 }
 else
 {
	error=TUNER_INVALID_HANDLE;	
 }

 
 
 return error;
}

TUNER_Model_t STV6120_TunerGetModel(STCHIP_Handle_t hChip)
{
	TUNER_Model_t model = TUNER_NULL;

	if(hChip && hChip->pData)
	{
		model = ((SAT_TUNER_Params_t) (hChip->pData))->Model;
	}

	return model;
}

void STV6120_TunerSetIQ_Wiring(STCHIP_Handle_t hChip, S8 IQ_Wiring)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->IQ_Wiring = (TUNER_IQ_t)IQ_Wiring;
	}
}

S8 STV6120_TunerGetIQ_Wiring(STCHIP_Handle_t hChip)
{
	TUNER_IQ_t wiring = TUNER_IQ_NORMAL;

	if(hChip && hChip->pData)
	{
		wiring = ((SAT_TUNER_Params_t) (hChip->pData))->IQ_Wiring;
	}

	return (S8)wiring;
}

void STV6120_TunerSetReferenceFreq(STCHIP_Handle_t hChip, U32 Reference)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->Reference = Reference;
	}
}

U32 STV6120_TunerGetReferenceFreq(STCHIP_Handle_t hChip)
{
	U32 reference = 0;

	if(hChip && hChip->pData)
	{
		reference = ((SAT_TUNER_Params_t) (hChip->pData))->Reference;
	}

	return reference;
}

void STV6120_TunerSetIF_Freq(STCHIP_Handle_t hChip, U32 IF)
{
	if(hChip && hChip->pData)
	{
		((SAT_TUNER_Params_t) (hChip->pData))->IF = IF;
	}
}

U32 STV6120_TunerGetIF_Freq(STCHIP_Handle_t hChip)
{
	U32 ifreq = 0;

	if(hChip && hChip->pData)
	{
		ifreq = ((SAT_TUNER_Params_t) (hChip->pData))->IF;
	}

	return ifreq;
}

void STV6120_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect)
{
	if(hChip )
	{
		if(hChip->pData)
		{
			if(((SAT_TUNER_Params_t) (hChip->pData))->InputSelect==TUNER_INPUT1)
				((SAT_TUNER_Params_t) (hChip->pData))->BandSelect = (TUNER_WIDEBandS_t)BandSelect;
			else if(((SAT_TUNER_Params_t) (hChip->pData))->InputSelect==TUNER_INPUT2) 
				((SAT_TUNER_Params_t) (hChip->pData))->BandSelect2 = (TUNER_WIDEBandS_t)BandSelect;
		}
	}
}

U8 STV6120_TunerGetBandSelect(STCHIP_Handle_t hChip)
{
	U32 ifreq = 0;
	
	if(hChip )
	{
		if(hChip->pData)
		{
			ifreq = ((SAT_TUNER_Params_t) (hChip->pData))->BandSelect;
		}
	}
	
	return (U8)ifreq;
}


TUNER_Error_t STV6120_TunerTerm(STCHIP_Handle_t hTuner)
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







