/*****************************************************************************/
/* COPYRIGHT (C) 2009 STMicroelectronics - All Rights Reserved               */
/* ST makes no warranty express or implied including but not limited to,     */
/* any warranty of                                                           */
/*                                                                           */
/*   (i)  merchantability or fitness for a particular purpose and/or         */
/*   (ii) requirements, for a particular purpose in relation to the LICENSED */
/*        MATERIALS, which is provided 'AS IS', WITH ALL FAULTS. ST does not */
/*        represent or warrant that the LICENSED MATERIALS provided here     */
/*        under is free of infringement of any third party patents,          */
/*        copyrights,trade secrets or other intellectual property rights.    */
/*        ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE       */
/*        EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                    */
/*                                                                           */
/*****************************************************************************/
/**                                                                            
@File   STV0913_Drv.c                                             
@brief                                                                         



*/

/* includes ---------------------------------------------------------------- */
#ifndef ST_OSLINUX  
#include "stdlib.h" 
#include "stdio.h"
#include "string.h"
#endif

#include "stv0913_util.h"
#include "stv0913_drv.h"
#include "globaldefs.h"
#include "stvvglna.h"

#ifdef HOST_PC
#ifndef NO_GUI
#include "STV0913_GUI.h" 
#include "Appl.h"
#include "Pnl_Report.h"
#include <formatio.h>
#include "UserPar.h"  
#endif
#endif


#define STV0913_BLIND_SEARCH_AGC2_TH  700
#define STV0913_BLIND_SEARCH_AGC2_TH_CUT30 1000/*1400*/ /*Blind Optim*/
#define STV0913_BLIND_SEARCH_AGC2BANDWIDTH	40
#define DmdLock_TIMEOUT_LIMIT	550 

#define STV0913_IQPOWER_THRESHOLD	0x30

/****************************************************************************************************************************
*****************************************************************************************************************************/

/****************************************************************
****************************************************************
**************************************************************** 
**															   **
**		***********************************************		   **
**						Static Data					   **
**		***********************************************		   **
**															   **
**															   **
****************************************************************
**************************************************************** 
****************************************************************/


/****************************************************************************************************************************
*****************************************************************************************************************************/

/************************ 
Current LLA revision	
*************************/
static const ST_Revision_t RevisionSTV0913  = "STV0913-LLA_REL_2.0_October_2012";

/************************ 
DVBS1 and DSS C/N Look-Up table	
*************************/

static FE_STV0913_LOOKUP_t FE_STV0913_S2_CN_LookUp ={                                                    
60,                                              
{                                                
{	-30,	13950	},  /*C/N=-2.5dB*/
{	-25,	13580	},  /*C/N=-2.5dB*/
{	-20,	13150	},  /*C/N=-2.0dB*/
{	-15,	12760	},  /*C/N=-1.5dB*/
{	-10,	12345	},  /*C/N=-1.0dB*/
{	-05,	11900	},  /*C/N=-0.5dB*/ 
{	0,		11520	},  /*C/N=0dB*/     
{	05,		11080	},  /*C/N=0.5dB*/    
{	10,		10630	},  /*C/N=1.0dB*/    
{	15,		10210	},  /*C/N=1.5dB*/    
{	20,		9790 	},  /*C/N=2.0dB*/    
{	25,		9390 	},  /*C/N=2.5dB*/    
{	30,		8970 	},  /*C/N=3.0dB*/    
{	35,		8575 	},  /*C/N=3.5dB*/    
{	40,		8180 	},  /*C/N=4.0dB*/    
{	45,		7800 	},  /*C/N=4.5dB*/    
{	50,		7430 	},  /*C/N=5.0dB*/    
{	55,		7080 	},  /*C/N=5.5dB*/    
{	60,		6720 	},  /*C/N=6.0dB*/    
{	65,		6320 	},  /*C/N=6.5dB*/    
{	70,		6060 	},  /*C/N=7.0dB*/    
{	75,		5760 	},  /*C/N=7.5dB*/    
{	80,		5480 	},  /*C/N=8.0dB*/    
{	85,		5200 	},  /*C/N=8.5dB*/    
{	90,		4930 	},  /*C/N=9.0dB*/    
{	95,		4680 	},  /*C/N=9.5dB*/    
{	100,	4425 	},  /*C/N=10.0dB*/   
{	105,	4210 	},  /*C/N=10.5dB*/   
{	110,	3980 	},  /*C/N=11.0dB*/   
{	115,	3765 	},  /*C/N=11.5dB*/   
{	120,	3570 	},  /*C/N=12.0dB*/   
{	125,	3315 	},  /*C/N=12.5dB*/   
{	130,	3140 	},  /*C/N=13.0dB*/   
{	135,	2980 	},  /*C/N=13.5dB*/   
{	140,	2820 	},  /*C/N=14.0dB*/   
{	145,	2670 	},  /*C/N=14.5dB*/   
{	150,	2535 	},  /*C/N=15.0dB*/   
{	160,	2270 	},  /*C/N=16.0dB*/   
{	170,	2035 	},  /*C/N=17.0dB*/   
{	180,	1825 	},  /*C/N=18.0dB*/   
{	190,	1650 	},  /*C/N=19.0dB*/   
{	200,	1485 	},  /*C/N=20.0dB*/   
{	210,	1340 	},  /*C/N=21.0dB*/   
{	220,	1212 	},  /*C/N=22.0dB*/   
{	230,	1100 	},  /*C/N=23.0dB*/   
{	240,	1000 	},  /*C/N=24.0dB*/   
{	250,	910  	},  /*C/N=25.0dB*/   
{	260,	836  	},  /*C/N=26.0dB*/   
{	270,	772  	},  /*C/N=27.0dB*/   
{	280,	718  	},  /*C/N=28.0dB*/   
{	290,	671  	},  /*C/N=29.0dB*/   
{	300,	635  	},  /*C/N=30.0dB*/   
{	310,	602  	},  /*C/N=31.0dB*/   
{	320,	575  	},  /*C/N=32.0dB*/   
{	330,	550  	},  /*C/N=33.0dB*/   
{	350,	517  	},  /*C/N=35.0dB*/   
{	400,	480  	},  /*C/N=40.0dB*/   
{	450,	466  	},  /*C/N=45.0dB*/   
{	500,	464  	},	/*C/N=50.0dB*/   
{	510,	463  	},	/*C/N=51.0dB*/ 

}
};  


/************************ 
DVBS2 C/N Look-Up table	
*************************/
static FE_STV0913_LOOKUP_t FE_STV0913_S1_CN_LookUp ={
54,
{
{	0,		9242	},  /*C/N=-0dB*/								
{	05,		9105	},  /*C/N=0.5dB*/								
{	10,		8950	},  /*C/N=1.0dB*/								
{	15,		8780	},  /*C/N=1.5dB*/								
{	20,		8566	},  /*C/N=2.0dB*/								
{	25,		8366	},  /*C/N=2.5dB*/								
{	30,		8146	},  /*C/N=3.0dB*/								
{	35,		7908	},  /*C/N=3.5dB*/								
{	40,		7666	},  /*C/N=4.0dB*/								
{	45,		7405	},  /*C/N=4.5dB*/								
{	50,		7136	},  /*C/N=5.0dB*/								
{	55,		6861	},  /*C/N=5.5dB*/								
{	60,		6576	},  /*C/N=6.0dB*/								
{	65,		6330	},  /*C/N=6.5dB*/								
{	70,		6048	},  /*C/N=7.0dB*/								
{	75,		5768	},  /*C/N=7.5dB*/								
{	80,		5492	},  /*C/N=8.0dB*/								
{	85,		5224	},  /*C/N=8.5dB*/								
{	90,		4959	},  /*C/N=9.0dB*/								
{	95,		4709	},  /*C/N=9.5dB*/								
{	100,	4467	},  /*C/N=10.0dB*/								
{	105,	4236	},  /*C/N=10.5dB*/								
{	110,	4013	},  /*C/N=11.0dB*/								
{	115,	3800	},  /*C/N=11.5dB*/								
{	120,	3598	},  /*C/N=12.0dB*/								
{	125,	3406	},  /*C/N=12.5dB*/								
{	130,	3225	},  /*C/N=13.0dB*/								
{	135,	3052	},  /*C/N=13.5dB*/								
{	140,	2889	},  /*C/N=14.0dB*/								
{	145,	2733	},  /*C/N=14.5dB*/								
{	150,	2587	},  /*C/N=15.0dB*/								
{	160,	2318	},  /*C/N=16.0dB*/								
{	170,	2077	},  /*C/N=17.0dB*/								
{	180,	1862	},  /*C/N=18.0dB*/								
{	190,	1670	},  /*C/N=19.0dB*/								
{	200,	1499	},  /*C/N=20.0dB*/								
{	210,	1347	},  /*C/N=21.0dB*/								
{	220,	1213	},  /*C/N=22.0dB*/								
{	230,	1095	},  /*C/N=23.0dB*/								
{	240,	992 	},  /*C/N=24.0dB*/								
{	250,	900 	},  /*C/N=25.0dB*/								
{	260,	826 	},  /*C/N=26.0dB*/								
{	270,	758 	},  /*C/N=27.0dB*/								
{	280,	702 	},  /*C/N=28.0dB*/								
{	290,	653 	},  /*C/N=29.0dB*/								
{	300,	613 	},  /*C/N=30.0dB*/								
{	310,	579 	},  /*C/N=31.0dB*/								
{	320,	550 	},  /*C/N=32.0dB*/								
{	330,	526 	},  /*C/N=33.0dB*/								
{	350,	490 	},  /*C/N=33.0dB*/								
{	400,	445 	},  /*C/N=40.0dB*/								
{	450,	430 	},  /*C/N=45.0dB*/								
{	500,	426 	},	/*C/N=50.0dB*/								
{	510,	425 	}		/*C/N=51.0dB*/ 								
}
}; 											

/************************ 
RF level Look-Up table	
*************************/
static FE_STV0913_LOOKUP_t	FE_STV0913_RF_LookUp = 	{
14,
{
{-5, 0xCCA0},   /*-5dBm*/
{-10,0xC6BF},   /*-10dBm*/
{-15,0xC09F},   /*-15dBm*/
{-20,0xBA5F},   /*-20dBm*/
{-25,0xB3DF},   /*-25dBm*/
{-30,0xAD1F},   /*-30dBm*/
{-35,0xA5F0},   /*-35dBm*/
{-40,0x9E40},   /*-40dBm*/
{-45,0x950C},   /*-45dBm*/
{-50,0x8BFF},   /*-50dBm*/
{-55,0x7F5F},   /*-55dBm*/
{-60,0x7170},   /*-60dBm*/
{-65,0x5BDD},   /*-65dBm*/
{-70,0x34C0}	/*-70dBm*/
}
};

static FE_STV0913_LOOKUP_t	FE_STV0913_RF_LookUpLow = 	{
5,
{
{-70,0x94},
{-75,0x55}, 
{-80,0x33}, 
{-85,0x20},
{-90,0x18},
}
};
/*
static FE_STV0913_LOOKUP_t	FE_STV0913_RF_LookUpVGLNA = 	
{
21,
{
{	-33	,42104  },
{	-34	,41775  },
{	-35	,41437  },
{	-36	,41152  },
{	-41	,39727  },
{	-46	,38495  },
{	-51	,37454  },
{	-56	,36463  },
{	-61	,35551  },
{	-66	,34607  },
{	-71	,33567  },
{	-76	,32415  },
{	-81	,31030  },
{	-86	,29392  },
{	-91	,27374  },
{	-96	,25200  },
{	-101,23584  },
{	-106,22651  },
{	-111,22333  },
{	-116,22219  },
{	-120,	0   }
}
};
*/													
typedef struct
{
FE_STV0913_ModCod_t ModCode;
U8 CarLoopPilotsOn_2;
U8 CarLoopPilotsOff_2;
U8 CarLoopPilotsOn_5;
U8 CarLoopPilotsOff_5;
U8 CarLoopPilotsOn_10;
U8 CarLoopPilotsOff_10;
U8 CarLoopPilotsOn_20;
U8 CarLoopPilotsOff_20;
U8 CarLoopPilotsOn_30;
U8 CarLoopPilotsOff_30;

}FE_STV0913_CarLoopOPtim;
/********************************************************************** 
Tracking carrier loop carrier QPSK 1/2 to 8PSK 9/10 long Frame 	
*********************************************************************/
static	FE_STV0913_CarLoopOPtim	FE_STV0913_S2CarLoop[14]	=	
{  /*Modcod			2MPon 		2MPoff		5MPon 		5MPoff		10MPon 		10MPoff		20MPon 		20MPoff		30MPon 		30MPoff	*/
{FE_SAT_QPSK_12,	0x0C,		0x1C,		0x2B,		0x1C,		0x0B,		0x2C,		0x0B,		0x0C,		0x2A, 		0x2B},
{FE_SAT_QPSK_35,	0x1C,		0x1C,		0x2B,		0x1C,		0x0B,		0x2C,		0x0B,		0x0C,		0x2A,		0x2B}, 
{FE_SAT_QPSK_23,	0x2C,		0x2C,		0x2B,		0x1C,		0x0B,		0x2C,		0x0B,		0x0C,		0x2A,		0x2B}, 
{FE_SAT_QPSK_34,	0x3C,		0x2C,		0x3B,		0x2C,		0x1B,		0x1C,		0x1B,		0x3B,		0x3A,		0x1B}, 
{FE_SAT_QPSK_45,	0x0D,		0x3C,		0x3B,		0x2C,		0x1B,		0x1C,		0x1B,		0x3B,		0x3A,		0x1B}, 
{FE_SAT_QPSK_56,	0x1D,		0x3C,		0x0C,		0x2C,		0x2B,		0x1C,		0x1B,		0x3B,		0x0B,		0x1B}, 
{FE_SAT_QPSK_89,	0x3D,		0x0D,		0x0C,		0x2C,		0x2B,		0x0C,		0x2B,		0x2B,		0x0B,		0x0B}, 
{FE_SAT_QPSK_910,	0x1E,		0x0D,		0x1C,		0x2C,		0x3B,		0x0C,		0x2B,		0x2B,		0x1B,		0x0B},
{FE_SAT_8PSK_35,	0x28,		0x09,		0x28,		0x09,		0x28,		0x09,		0x28,		0x08,		0x28,		0x27}, 
{FE_SAT_8PSK_23,	0x19,		0x29,		0x19,		0x29,		0x19,		0x29,		0x38,		0x19,		0x28,		0x09}, 
{FE_SAT_8PSK_34,	0x1A,		0x0B,		0x1A,		0x3A,		0x0A,		0x2A,		0x39,		0x2A,		0x39,		0x1A}, 
{FE_SAT_8PSK_56,	0x2B,		0x2B,		0x1B,		0x1B,		0x0B,		0x1B,		0x1A,		0x0B,		0x1A,		0x1A}, 
{FE_SAT_8PSK_89,	0x0C,		0x0C,		0x3B,		0x3B,		0x1B,		0x1B,		0x2A,		0x0B,		0x2A,		0x2A}, 
{FE_SAT_8PSK_910,	0x0C,		0x1C,		0x0C,		0x3B,		0x2B,		0x1B,		0x3A,		0x0B,		0x2A,		0x2A}
};

/************************************************************************ 
Tracking carrier loop carrier 16APSK 2/3 to 32APSK 9/10 long Frame 	
************************************************************************/
static	FE_STV0913_CarLoopOPtim	FE_STV0913_S2APSKCarLoop[11]	=
{  /*Modcod			    2MPon 		2MPoff		5MPon 		5MPoff		10MPon 		10MPoff		20MPon 		20MPoff		30MPon 		30MPoff	*/
{FE_SAT_16APSK_23,	0x0A,		0x0A,		0x0A,		0x0A,		0x1A,		0x0A,		0x39,		0x0A,		0x29,		0x0A},
{FE_SAT_16APSK_34,	0x0A,		0x0A,		0x0A,		0x0A,		0x0B,		0x0A,		0x2A,		0x0A,		0x1A,		0x0A},
{FE_SAT_16APSK_45,	0x0A,		0x0A,		0x0A,		0x0A,		0x1B,		0x0A,		0x3A,		0x0A,		0x2A,		0x0A},
{FE_SAT_16APSK_56,	0x0A,		0x0A,		0x0A,		0x0A,		0x1B,		0x0A,		0x3A,		0x0A,		0x2A,		0x0A},
{FE_SAT_16APSK_89,	0x0A,		0x0A,		0x0A,		0x0A,		0x2B,		0x0A,		0x0B,		0x0A,		0x3A,		0x0A},
{FE_SAT_16APSK_910,	0x0A,		0x0A,		0x0A,		0x0A,		0x2B,		0x0A,		0x0B,		0x0A,		0x3A,		0x0A},
{FE_SAT_32APSK_34,	0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09},
{FE_SAT_32APSK_45,	0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09},
{FE_SAT_32APSK_56,	0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09},
{FE_SAT_32APSK_89,	0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09},
{FE_SAT_32APSK_910,	0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09,		0x09}
};

/************************************************************************ 
Tracking carrier loop carrier QPSK 1/4 to QPSK 2/5 long Frame 	
************************************************************************/
static	FE_STV0913_CarLoopOPtim	FE_STV0913_S2LowQPCarLoop[3]	=
{  /*Modcod			   2MPon 		2MPoff		5MPon 		5MPoff		10MPon 		10MPoff		20MPon 		20MPoff		30MPon 		30MPoff	*/
{FE_SAT_QPSK_14,	0x0C,		0x3C,		0x0B,		0x3C,		0x2A,		0x2C,		0x2A,		0x1C,		0x3A,		0x3B},
{FE_SAT_QPSK_13,	0x0C,		0x3C,		0x0B,		0x3C,		0x2A,		0x2C,		0x3A,		0x0C,		0x3A,		0x2B},
{FE_SAT_QPSK_25,	0x1C,		0x3C,		0x1B,		0x3C,		0x3A,		0x1C,		0x3A,		0x3B,		0x3A,		0x2B}
};
/****************************************************************************************************************************
*****************************************************************************************************************************/

/****************************************************************
****************************************************************
**************************************************************** 
**															   **
**		***********************************************		   **
**						PRIVATE ROUTINES					   **
**		***********************************************		   **
**															   **
**															   **
****************************************************************
**************************************************************** 
****************************************************************/


/****************************************************************************************************************************
*****************************************************************************************************************************/

/*****************************************************
**FUNCTION	::	GetMclkFreq
**ACTION	::	Set the STV0913 master clock frequency
**PARAMS IN	::  hChip		==>	handle to the chip
**				ExtClk		==>	External clock frequency (Hz)
**PARAMS OUT::	NONE
**RETURN	::	Synthesizer frequency (Hz)
*****************************************************/
U32 FE_STV0913_GetMclkFreq(STCHIP_Handle_t hChip, U32 ExtClk)
{
U32 mclk = 135000000,
ndiv = 1,
idf = 1,
ldf = 1,
odf = 1,
fvco = 1;

odf = ChipGetField(hChip,FSTV0913_ODF);
ndiv = ChipGetOneRegister(hChip,RSTV0913_NCOARSE1);
idf = ChipGetField(hChip,FSTV0913_IDF);

if(ndiv<8)
ndiv = 8;

ldf = 2 * ndiv;

if(idf==0)
idf = 1;
if(odf==0)
odf = 1;

fvco = ((ExtClk/1000000)*ldf)/(idf);	
mclk = (fvco)/(2 * odf);

return mclk * 1000000; 
/*printf("\nMclk =%d",mclk);*/
}

/*****************************************************
**FUNCTION	::	GetErrorCount
**ACTION	::	return the number of errors from a given counter
**PARAMS IN	::  hChip		==>	handle to the chip
**			::	Counter		==>	used counter 1 or 2.
::  
**PARAMS OUT::	NONE
**RETURN	::	Synthesizer frequency (Hz)
*****************************************************/
U32 FE_STV0913_GetErrorCount(STCHIP_Handle_t hChip,FE_STV0913_ERRORCOUNTER Counter)
{
U32 lsb,msb,hsb,errValue;


/*Read the Error value*/
switch(Counter)
{
case FE_STV0913_COUNTER1:
default:
ChipGetRegisters(hChip,RSTV0913_ERRCNT12,3);
hsb = ChipGetFieldImage(hChip,FSTV0913_ERR_CNT12);
msb = ChipGetFieldImage(hChip,FSTV0913_ERR_CNT11);
lsb = ChipGetFieldImage(hChip,FSTV0913_ERR_CNT10);
break;

case FE_STV0913_COUNTER2:
ChipGetRegisters(hChip,RSTV0913_ERRCNT22,3);
hsb = ChipGetFieldImage(hChip,FSTV0913_ERR_CNT22);
msb = ChipGetFieldImage(hChip,FSTV0913_ERR_CNT21);
lsb = ChipGetFieldImage(hChip,FSTV0913_ERR_CNT20);	
break;
}

/*Cupute the Error value 3 bytes (HSB,MSB,LSB)*/
errValue=(hsb<<16)+(msb<<8)+(lsb);
return (errValue);
}

/*****************************************************
**FUNCTION	::	STV0913_RepeaterFn  (First repeater )
**ACTION	::	Set the repeater On or OFF 
**PARAMS IN	::  hChip		==>	handle to the chip
::	State		==> repeater state On/Off.
**PARAMS OUT::	NONE
**RETURN	::	Error (if any)
*****************************************************/
/*
#ifdef CHIP_STAPI
STCHIP_Error_t FE_STV0910_RepeaterFn(STCHIP_Handle_t hChip,BOOL State, U8 *Buffer)
#else
STCHIP_Error_t FE_STV0910_RepeaterFn(STCHIP_Handle_t hChip,BOOL State)
#endif
{
STCHIP_Error_t error = CHIPERR_NO_ERROR;

if(hChip != NULL)
{
if(State == TRUE)
#ifdef CHIP_STAPI
ChipFillRepeaterMessage(hChip,FSTV0910_P1_I2CT_ON,1,Buffer);
#else
ChipSetField(hChip,FSTV0910_P1_I2CT_ON,1);
#endif
}

return error;
}

*/
#ifdef CHIP_STAPI
STCHIP_Error_t FE_STV0913_RepeaterFn(STCHIP_Handle_t hChip,BOOL State, U8 *Buffer)
#else
STCHIP_Error_t FE_STV0913_RepeaterFn(STCHIP_Handle_t hChip,BOOL State)
#endif
{
STCHIP_Error_t error = CHIPERR_NO_ERROR;

if(hChip != NULL)
{
if(State == TRUE)
#ifdef CHIP_STAPI
ChipFillRepeaterMessage(hChip,FSTV0913_I2CT_ON,1,Buffer);
#else
ChipSetField(hChip,FSTV0913_I2CT_ON,1);
#endif
}

return error;
}

/*****************************************************
--FUNCTION	::	CarrierWidth
--ACTION	::	Compute the width of the carrier
--PARAMS IN	::	SymbolRate	->	Symbol rate of the carrier (Kbauds or Mbauds)
--				RollOff		->	Rolloff * 100
--PARAMS OUT::	NONE
--RETURN	::	Width of the carrier (KHz or MHz) 
--***************************************************/
U32 FE_STV0913_CarrierWidth(U32 SymbolRate, FE_STV0913_RollOff_t RollOff)
{
U32 rolloff;

switch(RollOff)
{
case FE_SAT_20:
rolloff=20;
break;

case FE_SAT_25:
rolloff=25;
break;

case FE_SAT_35:
default:
rolloff=35;
break;
}
return (SymbolRate  + (SymbolRate*rolloff)/100);
}


void FE_STV0913_SetTS_Parallel_Serial(STCHIP_Handle_t hChip,FE_STV0913_Clock_t PathTS)
{

switch(PathTS)
{
case FE_TS_SERIAL_PUNCT_CLOCK:
ChipSetFieldImage(hChip,FSTV0913_TSFIFO_SERIAL,0x01);	/*Serial mode*/ 
ChipSetFieldImage(hChip,FSTV0913_TSFIFO_DVBCI,0);	    /* pucntured clock */
break;

case FE_TS_SERIAL_CONT_CLOCK:
ChipSetFieldImage(hChip,FSTV0913_TSFIFO_SERIAL,0x01);	/*Serial mode*/ 
ChipSetFieldImage(hChip,FSTV0913_TSFIFO_DVBCI,1);		/* continues clock */
break;

case FE_TS_PARALLEL_PUNCT_CLOCK:		
ChipSetFieldImage(hChip,FSTV0913_TSFIFO_SERIAL,0x00); /*Parallel mode*/
ChipSetFieldImage(hChip,FSTV0913_TSFIFO_DVBCI,0);	 /* pucntured clock */
break;

case FE_TS_DVBCI_CLOCK:
ChipSetFieldImage(hChip,FSTV0913_TSFIFO_SERIAL,0x00); /*Parallel mode*/
ChipSetFieldImage(hChip,FSTV0913_TSFIFO_DVBCI,1);	 /* continues clock */
break;


default:
break;
}
ChipSetRegisters(hChip,RSTV0913_TSCFGH,1);

}

/*****************************************************
--FUNCTION	::	GetCarrierFrequency
--ACTION	::	Return the carrier frequency offset
--PARAMS IN	::	MasterClock	->	Masterclock frequency (Hz)
::		
--PARAMS OUT::	NONE
--RETURN	::	Frequency offset in Hz. 
--***************************************************/
S32 FE_STV0913_GetCarrierFrequency(STCHIP_Handle_t hChip,U32 MasterClock )
{
S32	derot,
rem1,
rem2,
intval1,
intval2,
carrierFrequency;

/*	Read the carrier frequency regs value	*/

ChipGetRegisters(hChip,RSTV0913_CFR2,3); 
derot=(ChipGetFieldImage(hChip,FSTV0913_CAR_FREQ2)<<16)+
(ChipGetFieldImage(hChip,FSTV0913_CAR_FREQ1)<<8)+
(ChipGetFieldImage(hChip,FSTV0913_CAR_FREQ0));

/*	cumpute the signed value	*/ 
derot=Get2Comp(derot,24);

/*
Formulat:
carrier_frequency = MasterClock * Reg / 2^24
*/
intval1=MasterClock>>12;
intval2=derot>>12;

rem1=MasterClock%0x1000;
rem2=derot%0x1000;
derot=(intval1*intval2)+
((intval1*rem2)>>12)+
((intval2*rem1)>>12);  	/*only for integer calculation */


carrierFrequency=derot;

return carrierFrequency;
}

S32 FE_STV0913_TimingGetOffset(STCHIP_Handle_t hChip,U32 SymbolRate)
{
S32 timingoffset;

/*	Formulat :
SR_Offset = TMGRREG * SR /2^29
TMGREG is 3 bytes registers value 
SR is the current symbol rate
*/
ChipGetRegisters(hChip,RSTV0913_TMGREG2,3);

timingoffset=(ChipGetFieldImage(hChip,FSTV0913_TMGREG2)<<16)+
(ChipGetFieldImage(hChip,FSTV0913_TMGREG1)<<8)+
(ChipGetFieldImage(hChip,FSTV0913_TMGREG0));

timingoffset=Get2Comp(timingoffset,24);


if(timingoffset==0)
timingoffset=1;	
timingoffset=((S32)SymbolRate*10)/((S32)0x1000000/timingoffset);
timingoffset/= 320;

return timingoffset;


}


/*****************************************************
--FUNCTION	::	GetSymbolRate
--ACTION	::	Get the current symbol rate
--PARAMS IN	::	hChip		->	handle to the chip
::	MasterClock	->	Masterclock frequency (Hz)
::	
--PARAMS OUT::	NONE
--RETURN	::	Symbol rate in Symbol/s
*****************************************************/
U32 FE_STV0913_GetSymbolRate(STCHIP_Handle_t hChip,U32 MasterClock)
{
S32	rem1,
rem2,
intval1,
intval2,
symbolRate;

ChipGetRegisters(hChip,RSTV0913_SFR3,4);
symbolRate =(ChipGetFieldImage(hChip,FSTV0913_SYMB_FREQ3)<<24)+
(ChipGetFieldImage(hChip,FSTV0913_SYMB_FREQ2)<<16)+
(ChipGetFieldImage(hChip,FSTV0913_SYMB_FREQ1)<<8)+
(ChipGetFieldImage(hChip,FSTV0913_SYMB_FREQ0));		

/*	Formulat :
Found_SR = Reg * MasterClock /2^32
*/

intval1=(MasterClock)>>16;
intval2=(symbolRate)>>16;

rem1=(MasterClock)%0x10000;
rem2=(symbolRate)%0x10000;

symbolRate	=(intval1*intval2)+
((intval1*rem2)>>16)+
((intval2*rem1)>>16);  	/*only for integer calculation */

return (symbolRate);

}

/*****************************************************
--FUNCTION	::	SetSymbolRate
--ACTION	::	Get the Symbol rate
--PARAMS IN	::	hChip		->	handle to the chip
::	MasterClock	->	Masterclock frequency (Hz)
::	SymbolRate	->	Symbol Rate (Symbol/s)
::	
--PARAMS OUT::	NONE
--RETURN	::	None
*****************************************************/
void FE_STV0913_SetSymbolRate(STCHIP_Handle_t hChip,U32 MasterClock,U32 SymbolRate)
{
U32	symb;

if(SymbolRate>60000000) /*SR >60Msps*/
{

symb=SymbolRate<<4;			/* Formulat :init_symbol_rate_regs = SR * 2^16/MasterClock */
symb/=(MasterClock>>12);

/* 
equivalent to :
symb=(SymbolRate/2000)*65536;			
symb/=(MasterClock/2000);
*/
}
else if(SymbolRate>6000000)
{
symb=SymbolRate<<6;			/* Formulat :init_symbol_rate_regs = SR * 2^16/MasterClock */
symb/=(MasterClock>>10);

/* 
equivalent to :
symb=(SymbolRate/1000)*65536;			
symb/=(MasterClock/1000);*/
}
else
{
symb=SymbolRate<<9;			/* Formulat :init_symbol_rate_regs = SR * 2^16/MasterClock */
symb/=(MasterClock>>7);

/* 
equivalent to :
symb=(SymbolRate/100)*65536;			
symb/=(MasterClock/100);
*/
}


ChipSetOneRegister(hChip,RSTV0913_SFRINIT1,(symb>>8) & 0x7F);	/* update the MSB */
ChipSetOneRegister(hChip,RSTV0913_SFRINIT0,(symb & 0xFF));	/* update the LSB */
}

/*****************************************************
--FUNCTION	::	GetStandard
--ACTION	::	Return the current standrad (DVBS1,DSS or DVBS2
--PARAMS IN	::	hChip		->	handle to the chip
::	
--PARAMS OUT::	NONE
--RETURN	::	standard (DVBS1, DVBS2 or DSS
*****************************************************/
FE_STV0913_TrackingStandard_t	FE_STV0913_GetStandard(STCHIP_Handle_t hChip)
{
FE_STV0913_TrackingStandard_t foundStandard;
S32 state;

state=ChipGetField(hChip,FSTV0913_HEADER_MODE);

if(state==2)
foundStandard=FE_SAT_DVBS2_STANDARD;			/*	The demod Find DVBS2*/

else if(state==3)									/*	The demod Find DVBS1/DSS*/			
{
if(ChipGetField(hChip,FSTV0913_DSS_DVB)==1)
foundStandard=FE_SAT_DSS_STANDARD;				/*	Viterbi Find DSS*/
else
foundStandard=FE_SAT_DVBS1_STANDARD;			/*	Viterbi Find DVBS1*/ 
}
else
foundStandard=FE_SAT_UNKNOWN_STANDARD;				/*	The demod find nothing, unknown standard	*/
return foundStandard; 
}


/*****************************************************
--FUNCTION	::	GetRFLevel
--ACTION	::	Return power of the signal
--PARAMS IN	::	hChip	->	handle to the chip
::	lookup	->	LUT for RF level estimation.
::		
--PARAMS OUT::	NONE
--RETURN	::	Power of the signal (dBm), -100 if no signal 
--***************************************************/
S32 FE_STV0913_GetRFLevel(STCHIP_Handle_t hChip,FE_STV0913_LOOKUP_t *lookup)
{
S32 agcGain = 0,
iqpower = 0,
Imin,
Imax,
i,
rfLevel = 0;

if((lookup != NULL) && lookup->size)
{
/* Read the AGC integrator registers (2 Bytes)	*/
ChipGetRegisters(hChip,RSTV0913_AGCIQIN1,2);
agcGain=MAKEWORD(ChipGetFieldImage(hChip,FSTV0913_AGCIQ_VALUE1),ChipGetFieldImage(hChip,FSTV0913_AGCIQ_VALUE0));
Imin = 0;
Imax = lookup->size-1;

if(agcGain>lookup->table[0].regval)
rfLevel=lookup->table[0].realval;

else if(agcGain>=lookup->table[lookup->size-1].regval)
{
while((Imax-Imin)>1)
{
i=(Imax+Imin)>>1;
/*equivalent to i=(Imax+Imin)/2; */
if(INRANGE(lookup->table[Imin].regval,agcGain,lookup->table[i].regval))
Imax = i;
else
Imin = i;
}

rfLevel =	(((S32)agcGain - lookup->table[Imin].regval)
* (lookup->table[Imax].realval - lookup->table[Imin].realval)
/ (lookup->table[Imax].regval - lookup->table[Imin].regval))
+ lookup->table[Imin].realval;
}

else
{
if(iqpower>FE_STV0913_RF_LookUpLow.table[0].regval)
rfLevel=FE_STV0913_RF_LookUpLow.table[0].regval;
else if(iqpower>=FE_STV0913_RF_LookUpLow.table[lookup->size-1].regval)
{
while((Imax-Imin)>1)
{
i=(Imax+Imin)>>1;
/*equivalent to i=(Imax+Imin)/2; */
if(INRANGE(FE_STV0913_RF_LookUpLow.table[Imin].regval,iqpower,FE_STV0913_RF_LookUpLow.table[i].regval))
Imax = i;
else
Imin = i;
}

rfLevel =	(((S32)iqpower - FE_STV0913_RF_LookUpLow.table[Imin].regval)
* (FE_STV0913_RF_LookUpLow.table[Imax].realval - FE_STV0913_RF_LookUpLow.table[Imin].realval)
/ (FE_STV0913_RF_LookUpLow.table[Imax].regval - FE_STV0913_RF_LookUpLow.table[Imin].regval))
+ FE_STV0913_RF_LookUpLow.table[Imin].realval;
}
else rfLevel=FE_STV0913_RF_LookUpLow.table[lookup->size-1].realval;

}
}
return rfLevel;
}

/*****************************************************
--FUNCTION	::	CarrierGetQuality
--ACTION	::	Return the carrier to noise of the current carrier
--PARAMS IN	::	hChip	->	handle to the chip
::	lookup	->	LUT for CNR level estimation.
--PARAMS OUT::	NONE
--RETURN	::	C/N of the carrier, 0 if no carrier 
--***************************************************/
S32 FE_STV0913_CarrierGetQuality(STCHIP_Handle_t hChip,FE_STV0913_LOOKUP_t *lookup)   
{
S32 c_n = -100,
regval,
Imin,
Imax,
i,
lockFlagField,
noiseField1,
noiseField0,
noiseReg;

lockFlagField =	FSTV0913_LOCK_DEFINITIF;
if(FE_STV0913_GetStandard(hChip)==FE_SAT_DVBS2_STANDARD)
{
/*If DVBS2 use PLH normilized noise indicators*/
noiseField1 =FSTV0913_NOSPLHT_NORMED1;
noiseField0 =FSTV0913_NOSPLHT_NORMED0;
noiseReg= RSTV0913_NNOSPLHT1; 
}
else
{
/*if not DVBS2 use symbol normalized noise indicators*/ 
noiseField1 =FSTV0913_NOSDATAT_NORMED1;
noiseField0 =FSTV0913_NOSDATAT_NORMED0;
noiseReg= RSTV0913_NNOSDATAT1;
}

if(ChipGetField(hChip,lockFlagField))
{
if((lookup != NULL) && lookup->size)
{
regval=0;
ChipWaitOrAbort(hChip,5);
for(i=0;i</*16*/8;i++)
{
ChipGetRegisters(hChip,noiseReg,2);
regval += MAKEWORD(ChipGetFieldImage(hChip,noiseField1),ChipGetFieldImage(hChip,noiseField0));
ChipWaitOrAbort(hChip,1);
}
regval /=/*16*/8;

Imin = 0;
Imax = lookup->size-1;

if(INRANGE(lookup->table[Imin].regval,regval,lookup->table[Imax].regval))
{
while((Imax-Imin)>1)
{
i=(Imax+Imin)>>1;
/*equivalent to i=(Imax+Imin)/2; */
if(INRANGE(lookup->table[Imin].regval,regval,lookup->table[i].regval))
Imax = i;
else
Imin = i;
}

c_n =	((regval - lookup->table[Imin].regval)
* (lookup->table[Imax].realval - lookup->table[Imin].realval)
/ (lookup->table[Imax].regval - lookup->table[Imin].regval))
+ lookup->table[Imin].realval;
}
else if(regval<lookup->table[Imin].regval)
c_n = 1000;
}
}

return c_n; 
}


/*****************************************************
--FUNCTION	::	GetBer
--ACTION	::	Return the Viterbi BER if DVBS1/DSS or the PER if DVBS2
--PARAMS IN	::	hChip	->	handle to the chip
--PARAMS OUT::	NONE
--RETURN	::	ber/per scalled to 1e7 
--***************************************************/
U32 FE_STV0913_GetBer(STCHIP_Handle_t hChip)
{
U32 ber = 10000000,i;
S32 demodState;



demodState=ChipGetField(hChip,FSTV0913_HEADER_MODE);

switch(demodState)
{
case FE_SAT_SEARCH:
case FE_SAT_PLH_DETECTED:
default:
ber=10000000;		/*demod Not locked ber = 1*/
break;

case FE_SAT_DVBS_FOUND:

ber=0;
/* Average 5 ber values */ 
for(i=0;i<5;i++)
{
ChipWaitOrAbort(hChip,5);
ber += FE_STV0913_GetErrorCount(hChip,FE_STV0913_COUNTER1);
}

ber/=5;

if(ChipGetField(hChip,FSTV0913_PRFVIT))	/*	Check for carrier	*/
{
/*	Error Rate	*/
ber *= 9766; 		/*ber = ber * 10^7/2^10 */
ber = ber>>13;	/*  theses two lines => ber = ber * 10^7/2^20	*/
}
break;

case FE_SAT_DVBS2_FOUND:

ber=0;
/* Average 5 ber values */ 
for(i=0;i<5;i++)
{
ChipWaitOrAbort(hChip,5);
ber += FE_STV0913_GetErrorCount(hChip,FE_STV0913_COUNTER1);
}
ber/=5;


if(ChipGetField(hChip,FSTV0913_PKTDELIN_LOCK))	/*	Check for S2 FEC Lock	*/
{
ber *= 9766; 					/*	ber = ber * 10^7/2^10 */
ber=ber>>13;				/*  theses two lines => PER = ber * 10^7/2^23	*/
}

break;
}

return ber;
}

FE_STV0913_Rate_t FE_STV0913_GetPunctureRate(STCHIP_Handle_t hChip)
{
S32 rateField;
FE_STV0913_Rate_t punctureRate;	

rateField=ChipGetField(hChip,FSTV0913_VIT_CURPUN);
switch(rateField)
{
case 13:
punctureRate=FE_SAT_PR_1_2;
break;

case 18:
punctureRate=FE_SAT_PR_2_3;
break;

case 21:
punctureRate=FE_SAT_PR_3_4;
break;

case 24:
punctureRate=FE_SAT_PR_5_6;
break;

case 25:
punctureRate=FE_SAT_PR_6_7;
break;

case 26:
punctureRate=FE_SAT_PR_7_8;
break;

default:
punctureRate=FE_SAT_PR_UNKNOWN;
break;
}

return	punctureRate; 
}


void FE_STV0913_GetLockTimeout(S32 *DemodTimeout,S32 *FecTimeout, S32 SymbolRate, FE_STV0913_SearchAlgo_t Algo)
{
switch (Algo)
{
case FE_SAT_BLIND_SEARCH:

if(SymbolRate<=1500000)  /*10Msps< SR <=15Msps*/
{
(*DemodTimeout)=1000;   /*1500*/
(*FecTimeout)=400;	   /*400*/
}
else if(SymbolRate<=5000000)  /*1.5Msps< SR <=5Msps*/
{
(*DemodTimeout)=1000;	 
(*FecTimeout)=300;
}
else if(SymbolRate<=30000000)  /*5Msps< SR <=30Msps*/
{
(*DemodTimeout)=700;    
(*FecTimeout)=300;
}
else if(SymbolRate<=45000000)  /*30Msps< SR <=45Msps*/
{
(*DemodTimeout)=400;	  
(*FecTimeout)=200;
}
else   /*SR >45Msps*/
{
(*DemodTimeout)=200;
(*FecTimeout)=100;
}

break;

case FE_SAT_COLD_START:
case FE_SAT_WARM_START:
default:
if(SymbolRate<=1000000)  /*SR <=1Msps*/
{
(*DemodTimeout)=3000;
(*FecTimeout)=2000;  /*1700 */  
}
else if(SymbolRate<=2000000)  /*1Msps < SR <=2Msps*/
{
(*DemodTimeout)=2500;
(*FecTimeout)=1300; /*1100 */
}
else if(SymbolRate<=5000000)  /*2Msps< SR <=5Msps*/
{
(*DemodTimeout)=1000;
(*FecTimeout)=650; /* 550 */
}
else if(SymbolRate<=10000000)  /*5Msps< SR <=10Msps*/
{
(*DemodTimeout)=700;
(*FecTimeout)=350; /*250 */
}
else if(SymbolRate<=20000000)  /*10Msps< SR <=20Msps*/
{
(*DemodTimeout)=400;
(*FecTimeout)=200; /* 130 */
}

else   /*SR >20Msps*/
{
(*DemodTimeout)=300;
(*FecTimeout)=200; /* 150 */
}
break;

}
if(Algo==FE_SAT_WARM_START)
{
/*if warm start
demod timeout = coldtimeout/3
fec timeout = same as cold*/
(*DemodTimeout)/=2;
}

}

BOOL FE_STV0913_GetDemodLock(STCHIP_Handle_t hChip,S32 TimeOut)
{
S32 timer=0,
lock=0;
U32 symbolRate ;
U32 TimeOut_SymbRate, SRate_1MSymb_Sec ;

FE_STV0913_SEARCHSTATE_t	demodState;

#ifdef HOST_PC 
#ifndef NO_GUI
double lockTime;
char message[100];
lockTime = Timer();

#endif
#endif

TimeOut_SymbRate = TimeOut ;
SRate_1MSymb_Sec = 0x01e5 ;
while((timer<TimeOut_SymbRate)&&(lock==0))
{

/*Calcul du TimeOut reel en fonction du Symbolrate*/
symbolRate =
(ChipGetField(hChip,FSTV0913_SYMB_FREQ3)<<8)+
(ChipGetField(hChip,FSTV0913_SYMB_FREQ2))
;	
if (TimeOut < DmdLock_TIMEOUT_LIMIT)
TimeOut_SymbRate = TimeOut ;
else
{
/*Securite anti division par 0*/
if (symbolRate < SRate_1MSymb_Sec)
symbolRate = SRate_1MSymb_Sec ;
else if (symbolRate > (5*SRate_1MSymb_Sec))
symbolRate = 5*SRate_1MSymb_Sec ;

TimeOut_SymbRate = TimeOut / (symbolRate / SRate_1MSymb_Sec) ;

/*Securite anti resultat aberrant*/
if (TimeOut_SymbRate < DmdLock_TIMEOUT_LIMIT)
TimeOut_SymbRate = DmdLock_TIMEOUT_LIMIT ;
else if (TimeOut_SymbRate > TimeOut)
TimeOut_SymbRate = TimeOut ;

/*Le resultat se situe maintenant entre 200 ms et TimeOut (=1 sec)*/
}
demodState=ChipGetField(hChip,FSTV0913_HEADER_MODE);
//printf("<%s ,%d>demodState =%d(s1>>3)\n",__func__,__LINE__,demodState);
switch(demodState)
{
case FE_SAT_SEARCH:
case FE_SAT_PLH_DETECTED :	/* no signal*/ 
default :
lock = 0;			
break;

case FE_SAT_DVBS2_FOUND:	/*found a DVBS2 signal */ 
case FE_SAT_DVBS_FOUND:
lock = ChipGetField(hChip,FSTV0913_LOCK_DEFINITIF);		
break;
}
//printf("<%s ,%d>lock =%d\n",__func__,__LINE__,lock);
if(lock==0)
ChipWaitOrAbort(hChip,10);
timer+=10;
}

#ifdef HOST_PC 
#ifndef NO_GUI
lockTime=Timer()-lockTime;
symbolRate =(ChipGetField(hChip,FSTV0913_SYMB_FREQ3)<<8)+
(ChipGetField(hChip,FSTV0913_SYMB_FREQ2));		
Fmt(message,"Demod Lock Time = %f ms (Symbrate=%x,TO=%d)",lockTime*1000.0,symbolRate,TimeOut_SymbRate);
if(lock)
ReportInsertMessage("DEMOD LOCK OK");
else
ReportInsertMessage("DEMOD LOCK FAIL");

ReportInsertMessage(message);
#endif
#endif
//printf("<913_drv.c>_getDemod_LOCK..Symbrate=%d,TO=%d) Lock =%d \n",symbolRate,TimeOut_SymbRate,lock);
return lock;
}

BOOL FE_STV0913_GetFECLock(STCHIP_Handle_t hChip,S32 TimeOut)
{
S32 timer=0,
lock=0;

FE_STV0913_SEARCHSTATE_t	demodState;
#ifdef HOST_PC 
#ifndef NO_GUI
double lockTime;
char message[100];
lockTime = Timer();

#endif
#endif

demodState=ChipGetField(hChip,FSTV0913_HEADER_MODE);
while((timer<TimeOut)&&(lock==0))
{

switch(demodState)
{
case FE_SAT_SEARCH:
case FE_SAT_PLH_DETECTED :	/* no signal*/ 
default :
lock = 0;			
break;

case FE_SAT_DVBS2_FOUND:	/*found a DVBS2 signal */ 
lock = ChipGetField(hChip,FSTV0913_PKTDELIN_LOCK);
break;

case FE_SAT_DVBS_FOUND:
lock = ChipGetField(hChip,FSTV0913_LOCKEDVIT);
break;
}

if(lock==0)
{

ChipWaitOrAbort(hChip,10);
timer+=10;
}
}

#ifdef HOST_PC 
#ifndef NO_GUI
lockTime=Timer()-lockTime;
Fmt(message,"FEC Lock Time = %f ms",lockTime*1000.0);

if(lock)
{	ReportInsertMessage("FEC LOCK OK");
if(demodState==FE_SAT_DVBS2_FOUND)
ReportInsertMessage("DVBS2 found"); 
}
else
ReportInsertMessage("FEC LOCK FAIL");

ReportInsertMessage(message);
#endif
#endif

return lock;
}

/*****************************************************
--FUNCTION	::	WaitForLock
--ACTION	::	Wait until Demod+ FEC locked or timout
--PARAMS IN	::	hChip	->	handle to the chip
::	TimeOut	->	Time out in ms
--PARAMS OUT::	NONE
--RETURN	::	Lock status true or false
--***************************************************/
BOOL FE_STV0913_WaitForLock(STCHIP_Handle_t hChip,S32 DemodTimeOut,S32 FecTimeOut)
{

S32 lock=0;



#ifdef HOST_PC 
#ifndef NO_GUI
double lockTime;
char message[100];
lockTime = Timer();

#endif
#endif




lock=FE_STV0913_GetDemodLock(hChip,DemodTimeOut);
/*printf("\n<%s ,%d>lock =%d",__func__,__LINE__,lock);*/
if(lock)
{
lock=lock && FE_STV0913_GetFECLock(hChip,FecTimeOut);
/*printf("\n<%s ,%d>lock =%d",__func__,__LINE__,lock);		*/
}

#ifdef HOST_PC 
#ifndef NO_GUI
lockTime=Timer()-lockTime;
Fmt(message,"Total Lock Time = %f ms",lockTime*1000.0);
if(lock)
ReportInsertMessage("LOCK OK");
else
ReportInsertMessage("LOCK FAIL");

ReportInsertMessage(message);

#endif
#endif
/*printf("\n<%s ,%d>lock =%d",__func__,__LINE__,lock);*/
if(lock)
return TRUE;
else
return FALSE;
}

void FE_STV0913_SetViterbiStandard(STCHIP_Handle_t hChip,FE_STV0913_SearchStandard_t Standard,FE_STV0913_Rate_t PunctureRate)
{
switch(Standard)
{
case FE_SAT_AUTO_SEARCH:
ChipSetOneRegister(hChip,RSTV0913_FECM,0x00);	/* Enable only DVBS1 ,DSS seach mast be made on demande by giving Std DSS*/
ChipSetOneRegister(hChip,RSTV0913_PRVIT,0x2F);  /* Enable all PR exept 6/7 */
break;

case FE_SAT_SEARCH_DVBS1:
ChipSetOneRegister(hChip,RSTV0913_FECM,0x00);	/* Disable DSS */
switch(PunctureRate)
{
case FE_SAT_PR_UNKNOWN:
default:
ChipSetOneRegister(hChip,RSTV0913_PRVIT,0x2F);   /* Enable All PR */
break;

case FE_SAT_PR_1_2:
ChipSetOneRegister(hChip,RSTV0913_PRVIT,0x01);   /* Enable 1/2 PR only */
break;

case FE_SAT_PR_2_3:
ChipSetOneRegister(hChip,RSTV0913_PRVIT,0x02);   /* Enable 2/3 PR only */
break;

case FE_SAT_PR_3_4:
ChipSetOneRegister(hChip,RSTV0913_PRVIT,0x04);   /* Enable 3/4 PR only */
break;

case FE_SAT_PR_5_6:
ChipSetOneRegister(hChip,RSTV0913_PRVIT,0x08);   /* Enable 5/6 PR only */
break;

case FE_SAT_PR_7_8:
ChipSetOneRegister(hChip,RSTV0913_PRVIT,0x20);   /* Enable 7/8 PR only */
break;
}

break;

case FE_SAT_SEARCH_DSS:
ChipSetOneRegister(hChip,RSTV0913_FECM,0x80);	/* Disable DVBS1 */
switch(PunctureRate)
{
case FE_SAT_PR_UNKNOWN:
default:
ChipSetOneRegister(hChip,RSTV0913_PRVIT,0x13);   /* Enable 1/2, 2/3 and 6/7 PR */
break;

case FE_SAT_PR_1_2:
ChipSetOneRegister(hChip,RSTV0913_PRVIT,0x01);   /* Enable 1/2 PR only */
break;

case FE_SAT_PR_2_3:
ChipSetOneRegister(hChip,RSTV0913_PRVIT,0x02);   /* Enable 2/3 PR only */
break;

case FE_SAT_PR_6_7:
ChipSetOneRegister(hChip,RSTV0913_PRVIT,0x10);   /* Enable 7/8 PR only */
break;
}

break;

default:
break;
}
}
#if 0
S32 FE_STV0913_CheckVGLNAPresent(STCHIP_Handle_t hChipVGLNA)
{
S32 isVGLNA =0;

#if !defined(HOST_PC) || defined(NO_GUI) 
if(hChipVGLNA!=NULL)
{
if(ChipCheckAck(hChipVGLNA)==CHIPERR_I2C_NO_ACK)
isVGLNA=0;
else
isVGLNA=1;
}
#else
if(ChipCheckAck(hChipVGLNA)!=0)
isVGLNA=0;
else
isVGLNA=1;
#endif

return isVGLNA;
}

S32 FE_STV0913_GetRFLevelWithVGLNA(FE_STV0913_InternalParams_t *pParams )
{
S32 Power=0,
vglnagain=0,
BBgain=0,
agcGain;

if(FE_STV0913_CheckVGLNAPresent(pParams->hVGLNA)==0 )
{	
ChipGetRegisters(pParams->hDemod,RSTV0913_AGCIQIN1,2);
agcGain=MAKEWORD(ChipGetFieldImage(pParams->hDemod,FSTV0913_AGCIQ_VALUE1),ChipGetFieldImage(pParams->hDemod,FSTV0913_AGCIQ_VALUE0));
BBgain= FE_Sat_TunerGetGain(pParams->hTuner); 
Power= FE_Sat_TunerGetRFGain(pParams->hTuner,agcGain,BBgain);
if(ChipGetOneRegister(pParams->hDemod,RSTV0913_TSTTNR1)==0x26)
Power=-405-Power;
else
Power=-730-Power; 

Power/=100;

}
else
{
Power 	=  FE_STV0913_GetRFLevel(pParams->hDemod,&FE_STV0913_RF_LookUpVGLNA);
if(STVVGLNA_GetGain(pParams->hVGLNA,&vglnagain)==CHIPERR_NO_ERROR) 
{
BBgain= FE_Sat_TunerGetGain(pParams->hTuner); 
Power 		+=vglnagain;
Power+= ((12-BBgain)+2);
}

}

return Power;
}

#endif
/*****************************************************
--FUNCTION	::	SetSearchStandard
--ACTION	::	Set the Search standard (Auto, DVBS2 only or DVBS1/DSS only)
--PARAMS IN	::	hChip	->	handle to the chip
::	Standard	->	Search standard
--PARAMS OUT::	NONE
--RETURN	::	none
--***************************************************/
void FE_STV0913_SetSearchStandard(FE_STV0913_InternalParams_t *pParams)
{
switch(pParams->DemodSearchStandard)
{
case FE_SAT_SEARCH_DVBS1:
case FE_SAT_SEARCH_DSS: 
/*If DVBS1/DSS only disable DVBS2 search*/
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS1_ENABLE,1);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS2_ENABLE,0);
ChipSetRegisters(pParams->hDemod,RSTV0913_DMDCFGMD,1);

FE_STV0913_SetViterbiStandard(pParams->hDemod,pParams->DemodSearchStandard,pParams->DemodPunctureRate);


break;

case FE_SAT_SEARCH_DVBS2:
/*If DVBS2 only activate the DVBS2 search and stop the VITERBI*/
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS1_ENABLE,0);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS2_ENABLE,1);
ChipSetRegisters(pParams->hDemod,RSTV0913_DMDCFGMD,1);

break;

case FE_SAT_AUTO_SEARCH:
default:
/*If automatic enable both DVBS1/DSS and DVBS2 search*/ 
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS1_ENABLE,1);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS2_ENABLE,1);
ChipSetRegisters(pParams->hDemod,RSTV0913_DMDCFGMD,1);

FE_STV0913_SetViterbiStandard(pParams->hDemod,pParams->DemodSearchStandard,pParams->DemodPunctureRate); 

break;
}

}


/*****************************************************
--FUNCTION	::	StartSearch
--ACTION	::	Trig the Demod to start a new search
--PARAMS IN	::	pParams	->	Pointer FE_STV0913_InternalParams_t structer
--PARAMS OUT::	NONE
--RETURN	::	none
--***************************************************/
void FE_STV0913_StartSearch(FE_STV0913_InternalParams_t *pParams)
{

U32 freq, tmp;
S16 freq_s16;

tmp= ChipGetOneRegister(pParams->hDemod,RSTV0913_TSTRES0);
ChipSetOneRegister(pParams->hDemod,RSTV0913_TSTRES0,tmp|0x08);/*Reset CAR3, bug DVBS2->DVBS1 lock*/
ChipSetOneRegister(pParams->hDemod,RSTV0913_TSTRES0,tmp&0xf7);/*Reset CAR3, bug DVBS2->DVBS1 lock*/
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x1F);	/*Reset the Demod*/


if(pParams->DemodSearchAlgo == FE_SAT_WARM_START)
{
/* if warm start CFR min =-1MHz , CFR max = 1MHz*/
freq = 1000<<16;
freq /= (pParams->MasterClock/1000);
freq_s16 = (S16)freq;
}
else  if  (pParams->DemodSearchAlgo == FE_SAT_COLD_START)
{
/* CFR min =- (SearchRange/2 + margin ) 
CFR max = +(SearchRange/2 + margin)
(80KHz margin if SR <=5Msps else margin =600KHz )*/  

if(pParams->DemodSymbolRate <=5000000)  
freq=(pParams->DemodSearchRange/2000)+80;
else
freq=(pParams->DemodSearchRange/2000)+600;
freq = freq<<16;
freq /= (pParams->MasterClock/1000);
freq_s16= (S16)freq;
}
else 	
{ 
freq=(pParams->DemodSearchRange/2000);
freq = freq<<16;
freq /= (pParams->MasterClock/1000);
freq_s16= (S16)freq;
}
#if 0    /* en attendant que le registre soit visible dans le map */
/*Super FEC bug WA, applicable only for cut1*/
if(pParams->hDemod->ChipID==0x50)
{	
ChipSetOneRegister(pParams->hDemod,RSTV0910_SFDLYSET2,0x00);	/*activate SFEC*/
if(!FE_SAT_BLIND_SEARCH)
{
if (pParams->DemodSymbolRate >= 53500000)
{
ChipSetOneRegister(pParams->hDemod,RSTV0910_SFDLYSET2,0x02);	/*stop SFEC*/
}
}

}
#endif   /* formerly excluded lines */
ChipSetOneRegister(pParams->hDemod,RSTV0913_CARCFG,0x46);		/*no any defined search range*/

ChipSetFieldImage(pParams->hDemod,FSTV0913_CFR_UP1,MSB(freq_s16));
ChipSetFieldImage(pParams->hDemod,FSTV0913_CFR_UP0,LSB(freq_s16));
ChipSetRegisters(pParams->hDemod,RSTV0913_CFRUP1,2); 
freq_s16 *= (-1);
ChipSetFieldImage(pParams->hDemod,FSTV0913_CFR_LOW1,MSB(freq_s16));
ChipSetFieldImage(pParams->hDemod,FSTV0913_CFR_LOW0,LSB(freq_s16));
ChipSetRegisters(pParams->hDemod,RSTV0913_CFRLOW1,2); 

ChipSetFieldImage(pParams->hDemod,FSTV0913_CFR_INIT1,0);
ChipSetFieldImage(pParams->hDemod,FSTV0913_CFR_INIT0,0);
ChipSetRegisters(pParams->hDemod,RSTV0913_CFRINIT1,2);		/*init the demod frequency offset to 0 */



switch(pParams->DemodSearchAlgo)
{
case FE_SAT_WARM_START:/*The symbol rate and the exact carrier Frequency are known */
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x1F);		/*Reset the Demod*/
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x18);		/*Trig an acquisition (start the search)*/
break;

case FE_SAT_COLD_START:/*The symbol rate is known*/
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x1F);		/*Reset the Demod*/
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x15);		/*Trig an acquisition (start the search)*/
break;

case FE_SAT_BLIND_SEARCH:
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x1F);		/*Reset the Demod*/
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x00);		/*Trig an acquisition (start the search)*/
break;

default:
/*Nothing to do in case of blind search, blind  is handled by "FE_STV0913_BlindSearchAlgo" function */  
break;
}
}


U8 FE_STV0913_GetOptimCarrierLoop(S32 SymbolRate,FE_STV0913_ModCod_t ModCode,S32 Pilots,U8 ChipID)
{
U8 aclcValue=0x29;
S32	i;
FE_STV0913_CarLoopOPtim *carLoopS2,
*carLoopLowQpS2;


carLoopS2=FE_STV0913_S2CarLoop;
carLoopLowQpS2=FE_STV0913_S2LowQPCarLoop;


if(ModCode<FE_SAT_QPSK_12)
{
i=0;
while((i<3)&& (ModCode!=carLoopLowQpS2[i].ModCode))i++;
if(i>=3)
i=2;
}
else
{
i=0;
while((i<14)&& (ModCode!=carLoopS2[i].ModCode))i++;

if(i>=14)
{
i=0;
while((i<11)&& (ModCode!=FE_STV0913_S2APSKCarLoop[i].ModCode))i++;
if(i>=11)
i=10;
}
}

if(ModCode<=FE_SAT_QPSK_25)
{
if(Pilots)
{
if(SymbolRate<=3000000)
aclcValue=carLoopLowQpS2[i].CarLoopPilotsOn_2;
else if(SymbolRate<=7000000)
aclcValue=carLoopLowQpS2[i].CarLoopPilotsOn_5;
else if(SymbolRate<=15000000)
aclcValue=carLoopLowQpS2[i].CarLoopPilotsOn_10;
else if(SymbolRate<=25000000)
aclcValue=carLoopLowQpS2[i].CarLoopPilotsOn_20; 
else
aclcValue=carLoopLowQpS2[i].CarLoopPilotsOn_30; 
}
else
{
if(SymbolRate<=3000000)
aclcValue=carLoopLowQpS2[i].CarLoopPilotsOff_2;
else if(SymbolRate<=7000000)
aclcValue=carLoopLowQpS2[i].CarLoopPilotsOff_5;
else if(SymbolRate<=15000000)
aclcValue=carLoopLowQpS2[i].CarLoopPilotsOff_10;
else if(SymbolRate<=25000000)
aclcValue=carLoopLowQpS2[i].CarLoopPilotsOff_20; 
else
aclcValue=carLoopLowQpS2[i].CarLoopPilotsOff_30; 
}


}
else if(ModCode<=FE_SAT_8PSK_910)
{
if(Pilots)
{
if(SymbolRate<=3000000)
aclcValue=carLoopS2[i].CarLoopPilotsOn_2;
else if(SymbolRate<=7000000)
aclcValue=carLoopS2[i].CarLoopPilotsOn_5;
else if(SymbolRate<=15000000)
aclcValue=carLoopS2[i].CarLoopPilotsOn_10;
else if(SymbolRate<=25000000)
aclcValue=carLoopS2[i].CarLoopPilotsOn_20; 
else
aclcValue=carLoopS2[i].CarLoopPilotsOn_30; 
}
else
{
if(SymbolRate<=3000000)
aclcValue=carLoopS2[i].CarLoopPilotsOff_2;
else if(SymbolRate<=7000000)
aclcValue=carLoopS2[i].CarLoopPilotsOff_5;
else if(SymbolRate<=15000000)
aclcValue=carLoopS2[i].CarLoopPilotsOff_10;
else if(SymbolRate<=25000000)
aclcValue=carLoopS2[i].CarLoopPilotsOff_20; 
else
aclcValue=carLoopS2[i].CarLoopPilotsOff_30; 
}
}
else	/*16APSK and 32APSK*/
{

if(SymbolRate<=3000000)
aclcValue=FE_STV0913_S2APSKCarLoop[i].CarLoopPilotsOn_2;
else if(SymbolRate<=7000000)
aclcValue=FE_STV0913_S2APSKCarLoop[i].CarLoopPilotsOn_5;
else if(SymbolRate<=15000000)
aclcValue=FE_STV0913_S2APSKCarLoop[i].CarLoopPilotsOn_10;
else if(SymbolRate<=25000000)
aclcValue=FE_STV0913_S2APSKCarLoop[i].CarLoopPilotsOn_20; 
else
aclcValue=FE_STV0913_S2APSKCarLoop[i].CarLoopPilotsOn_30;
}

return aclcValue;
}

/*****************************************************
--FUNCTION	::	TrackingOptimization
--ACTION	::	Set Optimized parameters for tracking
--PARAMS IN	::	pParams	->	Pointer FE_STV0913_InternalParams_t structer
--PARAMS OUT::	NONE
--RETURN	::	none
--***************************************************/
void FE_STV0913_TrackingOptimization(FE_STV0913_InternalParams_t *pParams)
{
S32 symbolRate,
pilots,
aclc;


FE_STV0913_ModCod_t foundModcod;


symbolRate=FE_STV0913_GetSymbolRate(pParams->hDemod,pParams->MasterClock);		/*	Read the Symbol rate	*/          
symbolRate += FE_STV0913_TimingGetOffset(pParams->hDemod,symbolRate);

switch(pParams->DemodResults.Standard)
{
case FE_SAT_DVBS1_STANDARD:
if( pParams->DemodSearchStandard == FE_SAT_AUTO_SEARCH) 
{
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS1_ENABLE,1);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS2_ENABLE,0);
ChipSetRegisters(pParams->hDemod,RSTV0913_DMDCFGMD,1);
}
/*Set the rolloff to the manual value (given at the initialization)*/
ChipSetFieldImage(pParams->hDemod,FSTV0913_ROLLOFF_CONTROL,pParams->RollOff);
ChipSetFieldImage(pParams->hDemod,FSTV0913_MANUALSX_ROLLOFF,1);
ChipSetRegisters(pParams->hDemod,RSTV0913_DEMOD ,1);

ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL1,0x75);	/* force to viterbi bit error */

break;

case FE_SAT_DSS_STANDARD:
if( pParams->DemodSearchStandard == FE_SAT_AUTO_SEARCH)
{
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS1_ENABLE,1);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS2_ENABLE,0);
ChipSetRegisters(pParams->hDemod,RSTV0913_DMDCFGMD,1);
}
/*Set the rolloff to the manual value (given at the initialization)*/
ChipSetFieldImage(pParams->hDemod,FSTV0913_ROLLOFF_CONTROL,pParams->RollOff);
ChipSetFieldImage(pParams->hDemod,FSTV0913_MANUALSX_ROLLOFF,1);
ChipSetRegisters(pParams->hDemod,RSTV0913_DEMOD ,1);


ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL1,0x75);	/* force to viterbi bit error */
break;

case FE_SAT_DVBS2_STANDARD:

ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS1_ENABLE,0);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS2_ENABLE,1);
ChipSetRegisters(pParams->hDemod,RSTV0913_DMDCFGMD,1);

ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL1,0x67);	/* force to DVBS2 PER  */
if(pParams->DemodResults.FrameLength == FE_SAT_LONG_FRAME)
{
/*Carrier loop setting for lon frame*/
foundModcod=ChipGetField(pParams->hDemod,FSTV0913_DEMOD_MODCOD);
pilots=ChipGetFieldImage(pParams->hDemod,FSTV0913_DEMOD_TYPE) & 0x01;

aclc= FE_STV0913_GetOptimCarrierLoop(symbolRate,foundModcod,pilots,pParams->hDemod->ChipID);
if(foundModcod<=FE_SAT_QPSK_910)
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S2Q,aclc);
else if (foundModcod<=FE_SAT_8PSK_910)
{
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S2Q,0x2a);
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S28,aclc);
}
else
{
#if 0    /* formerly excluded lines */
if(foundModcod<=FE_SAT_16APSK_910)
{
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S2Q,0x2a);
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S216A,aclc);
}
else if(foundModcod<=FE_SAT_32APSK_910)
{
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S2Q,0x2a);
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S232A,aclc);
}
#endif   /* formerly excluded lines */
}
}
break;

case FE_SAT_UNKNOWN_STANDARD:
default:
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS1_ENABLE,1);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS2_ENABLE,1);
ChipSetRegisters(pParams->hDemod,RSTV0913_DMDCFGMD,1);
break;
}

}

/*****************************************************
--FUNCTION	::	TrackingOptimization2
--ACTION	::	Set Optimized parameters for tracking
--PARAMS IN	::	pParams	->	Pointer FE_STV0913_InternalParams_t structer
::	Demod	->	current demod 1 or 2
--PARAMS OUT::	NONE
--RETURN	::	none
--***************************************************/
FE_STV0913_Error_t FE_STV0913_TrackingOptimization_2(FE_STV0913_Handle_t Handle )
{
S32 symbolRate,
pilots,
aclc;

FE_STV0913_ModCod_t foundModcod;

FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t *pParams=NULL;


if(Handle != NULL)
{
pParams = (FE_STV0913_InternalParams_t *) Handle;
}
else
{
error = FE_LLA_INVALID_HANDLE; 	
}

symbolRate=FE_STV0913_GetSymbolRate(pParams->hDemod,pParams->MasterClock);		/*	Read the Symbol rate	*/          
symbolRate += FE_STV0913_TimingGetOffset(pParams->hDemod,symbolRate);
switch(pParams->DemodResults.Standard)
{
case FE_SAT_DVBS1_STANDARD:
if( pParams->DemodSearchStandard == FE_SAT_AUTO_SEARCH) 
{
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS1_ENABLE,1);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS2_ENABLE,0);
ChipSetRegisters(pParams->hDemod,RSTV0913_DMDCFGMD,1);
}
/*Set the rolloff to the manual value (given at the initialization)*/
ChipSetFieldImage(pParams->hDemod,FSTV0913_ROLLOFF_CONTROL,pParams->RollOff);
ChipSetFieldImage(pParams->hDemod,FSTV0913_MANUALSX_ROLLOFF,1);
ChipSetRegisters(pParams->hDemod,RSTV0913_DEMOD ,1);

ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL1,0x75);	/* force to viterbi bit error */

break;

case FE_SAT_DSS_STANDARD:
if( pParams->DemodSearchStandard == FE_SAT_AUTO_SEARCH)
{
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS1_ENABLE,1);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS2_ENABLE,0);
ChipSetRegisters(pParams->hDemod,RSTV0913_DMDCFGMD,1);
}
/*Set the rolloff to the manual value (given at the initialization)*/
ChipSetFieldImage(pParams->hDemod,FSTV0913_ROLLOFF_CONTROL,pParams->RollOff);
ChipSetFieldImage(pParams->hDemod,FSTV0913_MANUALSX_ROLLOFF,1);
ChipSetRegisters(pParams->hDemod,RSTV0913_DEMOD ,1);


ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL1,0x75);	/* force to viterbi bit error */
break;

case FE_SAT_DVBS2_STANDARD:

ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS1_ENABLE,0);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS2_ENABLE,1);
ChipSetRegisters(pParams->hDemod,RSTV0913_DMDCFGMD,1);

ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL1,0x67);	/* force to DVBS2 PER  */

if(pParams->DemodResults.FrameLength == FE_SAT_LONG_FRAME)
{
/*Carrier loop setting for lon frame*/
foundModcod=ChipGetField(pParams->hDemod,FSTV0913_DEMOD_MODCOD);
pilots=ChipGetFieldImage(pParams->hDemod,FSTV0913_DEMOD_TYPE) & 0x01;

aclc= FE_STV0913_GetOptimCarrierLoop(symbolRate,foundModcod,pilots,pParams->hDemod->ChipID);
if(foundModcod<=FE_SAT_QPSK_910)
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S2Q,aclc);
else if (foundModcod<=FE_SAT_8PSK_910)
{
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S2Q,0x2a);
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S28,aclc);
}
#if 0    /* formerly excluded lines */
else
{
if(foundModcod<=FE_SAT_16APSK_910)
{
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S2Q,0x2a);
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S216A,aclc);
}
else if(foundModcod<=FE_SAT_32APSK_910)
{
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S2Q,0x2a);
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S232A,aclc);
}
}
#endif   /* formerly excluded lines */
}
break;

case FE_SAT_UNKNOWN_STANDARD:
default:
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS1_ENABLE,1);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS2_ENABLE,1);
ChipSetRegisters(pParams->hDemod,RSTV0913_DMDCFGMD,1);
break;
}




return error;
}

/*****************************************************
--FUNCTION	::	GetSignalParams
--ACTION	::	Read signal caracteristics
--PARAMS IN	::	pParams	->	Pointer FE_STV0913_InternalParams_t structer
--PARAMS OUT::	NONE
--RETURN	::	RANGE Ok or not 
--***************************************************/
FE_STV0913_SIGNALTYPE_t FE_STV0913_GetSignalParams(
FE_STV0913_InternalParams_t *pParams,
U32						Satellite_Scan) 
{
FE_STV0913_SIGNALTYPE_t	range=FE_SAT_OUTOFRANGE;
S32 offsetFreq,
symbolRateOffset,
i=0;

U8 timing;

#ifdef HOST_PC 
#ifndef NO_GUI
char message[100];
#endif
#endif

ChipWaitOrAbort(pParams->hDemod,5);
if(pParams->DemodSearchAlgo==FE_SAT_BLIND_SEARCH)
{
/*	if Blind search wait for symbol rate offset information transfert from the timing loop to the
demodulator symbol rate
*/
timing=ChipGetOneRegister(pParams->hDemod,RSTV0913_TMGREG2);
i=0;
#if 0    /* formerly excluded lines */
ChipSetOneRegister(pParams->hDemod,RSTV0913_SFRSTEP,0x5c); /* TODO modif suite au tps de log 2 fois plus long en blind qu'en cold*/
#endif   /* formerly excluded lines */
while((i<=50) && (timing!=0) && (timing!=0xFF))
{
timing=ChipGetOneRegister(pParams->hDemod,RSTV0913_TMGREG2);
ChipWaitOrAbort(pParams->hDemod,5);
i+=5;
}
}

pParams->DemodResults.Standard		=	FE_STV0913_GetStandard(pParams->hDemod);

pParams->DemodResults.Frequency	=	FE_Sat_TunerGetFrequency(pParams->hTuner);			
offsetFreq=	FE_STV0913_GetCarrierFrequency(pParams->hDemod,pParams->MasterClock)/1000; 
pParams->DemodResults.Frequency	+= offsetFreq;

pParams->DemodResults.SymbolRate	=	FE_STV0913_GetSymbolRate(pParams->hDemod,pParams->MasterClock);
symbolRateOffset=FE_STV0913_TimingGetOffset(pParams->hDemod,pParams->DemodResults.SymbolRate);
pParams->DemodResults.SymbolRate	+=	symbolRateOffset;
pParams->DemodResults.PunctureRate	=	FE_STV0913_GetPunctureRate(pParams->hDemod); 
pParams->DemodResults.ModCode		=	ChipGetField(pParams->hDemod,FSTV0913_DEMOD_MODCOD);
pParams->DemodResults.Pilots		=	ChipGetFieldImage(pParams->hDemod,FSTV0913_DEMOD_TYPE) & 0x01;
pParams->DemodResults.FrameLength	=	((U32)ChipGetFieldImage(pParams->hDemod,FSTV0913_DEMOD_TYPE))>>1;

pParams->DemodResults.RollOff		=	ChipGetField(pParams->hDemod,FSTV0913_ROLLOFF_STATUS);

switch(pParams->DemodResults.Standard)
{
case FE_SAT_DVBS2_STANDARD:
pParams->DemodResults.Spectrum		=ChipGetField(pParams->hDemod,FSTV0913_SPECINV_DEMOD);

if(pParams->DemodResults.ModCode <= FE_SAT_QPSK_910)
pParams->DemodResults.Modulation=FE_SAT_MOD_QPSK;
else if(pParams->DemodResults.ModCode <= FE_SAT_8PSK_910)
pParams->DemodResults.Modulation=FE_SAT_MOD_8PSK;
else if(pParams->DemodResults.ModCode <= FE_SAT_16APSK_910)
pParams->DemodResults.Modulation=FE_SAT_MOD_16APSK;
else if(pParams->DemodResults.ModCode <= FE_SAT_32APSK_910)
pParams->DemodResults.Modulation=FE_SAT_MOD_32APSK;
else
pParams->DemodResults.Modulation=FE_SAT_MOD_UNKNOWN;

break;

case FE_SAT_DVBS1_STANDARD:
case FE_SAT_DSS_STANDARD:
default:
pParams->DemodResults.Modulation=FE_SAT_MOD_QPSK;
pParams->DemodResults.Spectrum=ChipGetField(pParams->hDemod,FSTV0913_IQINV);  	

break;
}

if((pParams->DemodSearchAlgo==FE_SAT_BLIND_SEARCH) || (pParams->DemodResults.SymbolRate < 10000000)) 
{
/*in case of blind search the tuner freq may has been moven, read the new tuner freq and cumpute the freq offset from the original value*/
offsetFreq=	pParams->DemodResults.Frequency-pParams->TunerFrequency;
pParams->TunerFrequency=FE_Sat_TunerGetFrequency(pParams->hTuner);

if ((pParams->DemodSearchAlgo==FE_SAT_BLIND_SEARCH)&& (Satellite_Scan > 0 ))
{
range=FE_SAT_RANGEOK; /* YMO: No check needed */
}
else 
{
if (ABS(offsetFreq)<= ((pParams->DemodSearchRange/2000)+500))
range=FE_SAT_RANGEOK;
else
{
if (ABS(offsetFreq)<=  (   (FE_STV0913_CarrierWidth(pParams->DemodResults.SymbolRate,pParams->DemodResults.RollOff))/2000) )
range=FE_SAT_RANGEOK;
else
range=FE_SAT_OUTOFRANGE;
}
}
}
else
{
if (ABS(offsetFreq)<= ((pParams->DemodSearchRange/2000)+500))
range=FE_SAT_RANGEOK;
else 
range=FE_SAT_OUTOFRANGE;
}


#ifdef HOST_PC 
#ifndef NO_GUI
if(range==FE_SAT_OUTOFRANGE)
{
Fmt(message,"offsetFreq = %d KHz\n ",offsetFreq);
ReportInsertMessage("Out Of Range");
ReportInsertMessage(message ); 
}
#endif
#endif


return range;
}

BOOL FE_STV0913_GetDemodLockCold(FE_STV0913_InternalParams_t *pParams,S32 DemodTimeout)
{

BOOL lock =FALSE;
S32 
nbSteps,
currentStep,
direction,
tunerFreq,
timeout,
stepsg,
freq;

U32 carrierStep=1200;

if(pParams->DemodSymbolRate>=10000000 )	/*if Symbol rate >= 10Msps check for timing lock */
{

lock = FE_STV0913_GetDemodLock(pParams->hDemod,(DemodTimeout/3)) ;	/* case cold start wait for demod lock */
if(lock==FALSE)
{
//if(FE_STV0913_CheckTimingLock(pParams->hDemod)==TRUE)
{
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x1F);		/*Reset the Demod*/
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x15);		/*Trig an acquisition (start the search)*/

lock=FE_STV0913_GetDemodLock(pParams->hDemod,DemodTimeout) ;
}
/*else
lock = FALSE;
*/
}
}
else	/*Symbol Rate <10Msps do not check timing lock*/
{
lock = FE_STV0913_GetDemodLock(pParams->hDemod,(DemodTimeout/2)) ;	/* case cold start wait for demod lock */   

if(lock==FALSE)
{

carrierStep=pParams->DemodSymbolRate/4000;
timeout=(DemodTimeout*3)/4; 

nbSteps=((pParams->DemodSearchRange/1000)/carrierStep);

if((nbSteps%2)!=0)
nbSteps+=1;

if(nbSteps<=0)	
nbSteps=2;
else if(nbSteps>12)
{
nbSteps =12;
}

currentStep=1;
direction=1;

stepsg=0; 

if(lock==FALSE)
{
tunerFreq=0;

while((currentStep<=nbSteps) && (lock==FALSE))
{

if(direction>0)
tunerFreq +=(currentStep*carrierStep);
else
tunerFreq -= (currentStep*carrierStep);

ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x1C);

/*	Formulat :
CFRINIT_Khz = CFRINIT_Reg * MasterClock_Khz /2^16
*/

freq=(tunerFreq*65536)/(pParams->MasterClock/1000);
ChipSetFieldImage(pParams->hDemod,FSTV0913_CFR_INIT1,MSB(freq) );
ChipSetFieldImage(pParams->hDemod,FSTV0913_CFR_INIT0,LSB(freq));
ChipSetRegisters(pParams->hDemod,RSTV0913_CFRINIT1,2);		/*init the demod frequency offset to 0 */

ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x1F);		
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x05); /* use AEP 0x05 instead of 0x15 to not reset CFRINIT to 0*/		

lock=FE_STV0913_GetDemodLock(pParams->hDemod,timeout) ;

direction*=-1;
currentStep++;
}
}
}
}

return	lock; 
}

U16 FE_STV0913_BlindCheckAGC2BandWidth(FE_STV0913_InternalParams_t *pParams)
{
U32 minagc2level=0xffff,maxagc2level=0x0000,midagc2level,
agc2level,
agc2ratio;
S32	initFreq,freqStep;
U32 tmp1,tmp2,tmp3,tmp4;
U32 asperity=0;
U32 waitforfall=0;
U32 previousmin;
U32 agc2level_step0;
U32 acculevel=0;
U32 div=2;
U32 agc2leveltab[20];

S32 i,j,k,l,nbSteps;


#ifdef HOST_PC 
#ifndef NO_GUI
double lockTime ;
char message[100];
lockTime = Timer() ;   /*SGA*/

#endif
#endif
/*AGC2 Scan*/

/* lockTime = Timer() ;	 */	 

ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x1C);
tmp2=ChipGetOneRegister(pParams->hDemod,RSTV0913_CARCFG);
ChipSetOneRegister(pParams->hDemod,RSTV0913_CARCFG,0x06);

tmp3=ChipGetOneRegister(pParams->hDemod,RSTV0913_BCLC);
ChipSetOneRegister(pParams->hDemod,RSTV0913_BCLC,0x00);
tmp4=ChipGetOneRegister(pParams->hDemod,RSTV0913_CARFREQ);
ChipSetOneRegister(pParams->hDemod,RSTV0913_CARFREQ,0x00);

ChipSetOneRegister(pParams->hDemod,RSTV0913_AGC2REF,0x38);

tmp1=ChipGetOneRegister(pParams->hDemod,RSTV0913_DMDCFGMD);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS1_ENABLE,1);
ChipSetFieldImage(pParams->hDemod,FSTV0913_DVBS2_ENABLE,1);
ChipSetFieldImage(pParams->hDemod,FSTV0913_SCAN_ENABLE,0);  		/*Enable the SR SCAN*/
ChipSetFieldImage(pParams->hDemod,FSTV0913_CFR_AUTOSCAN,0);    /*activate the carrier frequency search loop*/
ChipSetRegisters(pParams->hDemod,RSTV0913_DMDCFGMD,1); 

/*FE_STV0913_SetSymbolRate(pParams->hDemod,pParams->MasterClock,1000000);*/
FE_STV0913_SetSymbolRate(pParams->hDemod,pParams->MasterClock,1000000/div); /*AGC2 bandwidth is 1/div MHz */ 

nbSteps=(pParams->TunerBW/3000000)*div;
if(nbSteps<=0)
nbSteps=1;

freqStep=((1000000<<8)/(pParams->MasterClock>>8))/div;  /* AGC2 step is 1/div MHz */   

initFreq=0;
j=0;	/* index after a rising edge is found */
for(i=0;i<nbSteps;i++)
{
/*Scan on the positive part of the tuner Bw */

ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x1C);
ChipSetOneRegister(pParams->hDemod,RSTV0913_CFRINIT1,(initFreq >>8) & 0xff);
ChipSetOneRegister(pParams->hDemod,RSTV0913_CFRINIT0,initFreq  & 0xff);	   	
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x18);

WAIT_N_MS(5);

agc2level=0;

ChipGetRegisters(pParams->hDemod,RSTV0913_AGC2I1,2);

agc2level = (ChipGetFieldImage(pParams->hDemod,FSTV0913_AGC2_INTEGRATOR1) << 8)
| ChipGetFieldImage(pParams->hDemod,FSTV0913_AGC2_INTEGRATOR0);




if (i == 0)
{
agc2level_step0 = agc2level ;
minagc2level= agc2level;
maxagc2level= agc2level;
midagc2level=agc2level;
#if 1		
for (k=0;k<5*div;k++)
{
agc2leveltab[k]= agc2level;
}
#endif


}

else
{

/* To avoid spurs, it removes anomalies */
/* Rising edge = 2 consecutive mins + ratio> threshold compteurmin*/ /* deleted */
/* If the front comes down to i +1 we consider a spur and ignore */
#if 1		
//k= i-5*div*(i/(5*div));
k= i%(5*div);
agc2leveltab[k]= agc2level;

minagc2level=0xffff;
maxagc2level=0x0000;
acculevel=0;

for (l=0;l<5*div;l++)
{
/* Min and max detection */	


if( agc2leveltab[l]<minagc2level )

{
previousmin= minagc2level;
minagc2level=agc2leveltab[l];

}
else if(agc2leveltab[l] >maxagc2level) 
{
maxagc2level=agc2leveltab[l];

}	

acculevel=acculevel+agc2leveltab[l];

}

midagc2level= acculevel/(5*div);   
#endif

#if 0		
if( agc2level<minagc2level )

{
previousmin= minagc2level; 
minagc2level=agc2level;
}
if(agc2level>maxagc2level) 
{
maxagc2level=agc2level;
}

acculevel=midagc2level*i+agc2level;
midagc2level=acculevel/(i+1);

#endif	


if  (waitforfall==0)
{
agc2ratio = (maxagc2level - minagc2level)*128/midagc2level;
}
else 
{
agc2ratio = (agc2level - minagc2level)*128/midagc2level;
}

if (agc2ratio > 0xffff)
agc2ratio = 0xffff ;		



#ifdef HOST_PC 
#ifndef NO_GUI
Fmt(message,"AGC2Band i=%d m=%d M=%d Ratio=%d",i,minagc2level,maxagc2level,agc2ratio);
ReportInsertMessage(message);
/*Fmt(message,"AGC2Band i   %x %x %d %d",initFreq,ChipGetOneRegister(pParams->hDemod,RSTV0913_CFR2)<<8,agc2level,agc2ratio);*/
/*ReportInsertMessage(message);*/
#endif
#endif

if ((agc2ratio > STV0913_BLIND_SEARCH_AGC2BANDWIDTH) && (agc2level==minagc2level)) 	 /* rising edge */
{


asperity=1;		  /* The first edge is rising */
waitforfall=1;

for (l=0;l<5*div;l++)										
{
agc2leveltab[l]= agc2level;
}

}

else if ((agc2ratio > STV0913_BLIND_SEARCH_AGC2BANDWIDTH))

{	
/* Front descendant */
if (waitforfall==0)
{
asperity=2; /* the first edge is falling */
}
else
{
asperity=1;
}
if (j==1)
{
/* minagc2level= previousmin;*/  /* it was a spur at previous step  */ 

for (l=0;l<5*div;l++)										
{
agc2leveltab[l]= agc2level;
}

j=0;				  /* All reset */
waitforfall=0;
asperity=0;
}
else
{
break ;
}
}			



if ((waitforfall==1) && j==(5*div)) 

{
break;
}

if (waitforfall==1)
{
j+=1;
}	

} 		
initFreq = initFreq + freqStep;

}

ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDCFGMD,tmp1);
ChipSetOneRegister(pParams->hDemod,RSTV0913_CARCFG,tmp2);
ChipSetOneRegister(pParams->hDemod,RSTV0913_BCLC,tmp3);
ChipSetOneRegister(pParams->hDemod,RSTV0913_CARFREQ,tmp4);
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x1C);
ChipSetOneRegister(pParams->hDemod,RSTV0913_CFRINIT1,0);
ChipSetOneRegister(pParams->hDemod,RSTV0913_CFRINIT0,0);	   	
/*ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDT0M,tmp2);*/


if (asperity ==1)  /* rising edge followed by a constant level or a falling edge */
{
pParams->TunerIndexJump= (1000/div)*(i-(j+2)/2);

}

else
{
pParams->TunerIndexJump=(1000/div)*i; /* falling edge */   

}
return (U16)asperity;
}

BOOL FE_STV0913_BlindSearchAlgo(
FE_STV0913_InternalParams_t *pParams,
S32 						demodTimeout,
U32						    Satellite_Scan)
{



BOOL lock = TRUE;
/* U16 agc2Integrateur;*/	   /* YMO: Plus utilisé */


#ifdef HOST_PC 
#ifndef NO_GUI
U32 agc2Threshold;
double lockTime ;
char message[100];
lockTime = Timer();
#endif
#endif

/* lockTime = Timer() ;*/
if (Satellite_Scan ==0)
{
pParams->DemodSearchRange=2*36000000;
}
else
{
pParams->DemodSearchRange=24000000 + pParams->TunerIndexJump*1000;
if ( pParams->DemodSearchRange >40000000) 
{
pParams->DemodSearchRange=40000000;
}

}





#ifdef HOST_PC 
#ifndef NO_GUI
ReportInsertMessage("YES (FE_STV0913_BlindSearchAlgo)");
#endif
#endif

ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x5C); /* Demod Stop*/
FE_STV0913_SetSymbolRate(pParams->hDemod,pParams->MasterClock,pParams->DemodSymbolRate);
FE_STV0913_StartSearch(pParams);
lock=FE_STV0913_GetDemodLock(pParams->hDemod,demodTimeout);
//printf("YES (FE_STV0913_BlindSearchAlgo) lock =%d\n",lock);
return lock; 
}

/*****************************************************
--FUNCTION	::	FE_STV0913_Algo
--ACTION	::	Start a search for a valid DVBS1/DVBS2 or DSS transponder
--PARAMS IN	::	pParams	->	Pointer FE_STV0913_InternalParams_t structer
--PARAMS OUT::	NONE
--RETURN	::	SYGNAL TYPE 
--***************************************************/
FE_STV0913_SIGNALTYPE_t	FE_STV0913_Algo(
FE_STV0913_InternalParams_t *pParams,
U32 						Satellite_Scan)
{
S32 demodTimeout=500,
fecTimeout=50,
iqPower,
agc1Power,
i,
powerThreshold=STV0913_IQPOWER_THRESHOLD,
FinalFreq ;
U32 asperity =0;		  /* YMO */


BOOL lock = FALSE;
FE_STV0913_SIGNALTYPE_t signalType=FE_SAT_NOCARRIER;


#ifdef HOST_PC 
#ifndef NO_GUI
double lockTime ;
char message[100];
lockTime = Timer() ; 	 /* SGA */
#endif
#endif

ChipSetField(pParams->hDemod,FSTV0913_ALGOSWRST,0); /*release reset DVBS2 packet delin*/

ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x5C); /* Demod Stop*/
/* 	lockTime = Timer() ;	 SGA  */ 

/*Get the demod and FEC timeout recomended value depending of the symbol rate and the search algo*/
FE_STV0913_GetLockTimeout(&demodTimeout,&fecTimeout, pParams->DemodSymbolRate, pParams->DemodSearchAlgo);

if(pParams->DemodSearchAlgo==FE_SAT_BLIND_SEARCH)
{	/* If the Symbolrate is unknown  set the BW to the maximum */
pParams->TunerBW=2*36000000;
/* If Blind search set the init symbol rate to 1Msps*/
FE_STV0913_SetSymbolRate(pParams->hDemod,pParams->MasterClock,1000000);
}
else
{
pParams->TunerBW=(FE_STV0913_CarrierWidth(pParams->DemodSymbolRate,pParams->RollOff) + pParams->DemodSearchRange);
pParams->TunerBW=pParams->TunerBW*15/10;
/* Set the Init Symbol rate*/ 
FE_STV0913_SetSymbolRate(pParams->hDemod,pParams->MasterClock,pParams->DemodSymbolRate);
}
#ifdef HOST_PC 
#ifndef NO_GUI
Fmt(message,"FE_STV0913_Algo 0: %f ms",(Timer()-lockTime)*1000);
ReportInsertMessage(message);
#endif
#endif

FE_Sat_TunerSetFrequency(pParams->hTuner,pParams->TunerFrequency);
/*	Set tuner BW by SW (SW tuner API)	*/ 
FE_Sat_TunerSetBandwidth(pParams->hTuner,pParams->TunerBW);
//printf("<913_drv.c>Set Tuner Freq =%d and BW =%d\n",pParams->TunerFrequency,pParams->TunerBW);
#ifdef HOST_PC 
#ifndef NO_GUI
Fmt(message,"Tuner prog: %f ms",(Timer()-lockTime)*1000);
ReportInsertMessage(message);
#endif
#endif
/*	NO signal Detection */
/*Read PowerI and PowerQ To check the signal Presence*/
ChipWaitOrAbort(pParams->hDemod,10);
ChipGetRegisters(pParams->hDemod,RSTV0913_AGCIQIN1,2);
agc1Power=MAKEWORD(ChipGetFieldImage(pParams->hDemod,FSTV0913_AGCIQ_VALUE1),ChipGetFieldImage(pParams->hDemod,FSTV0913_AGCIQ_VALUE0));  
iqPower=0;
if(agc1Power==0)
{
/*if AGC1 integrator value is 0 then read POWERI, POWERQ registers*/
/*Read the IQ power value*/
for(i=0;i<5;i++)
{
ChipGetRegisters(pParams->hDemod,RSTV0913_POWERI,2);
iqPower+=(ChipGetFieldImage(pParams->hDemod,FSTV0913_POWER_I)+ChipGetFieldImage(pParams->hDemod,FSTV0913_POWER_Q))/2;	
}
iqPower/=5;
}
#ifdef HOST_PC 
#ifndef NO_GUI
/*if PC GUI read the IQ power threshold form the GUI user parameters*/
UsrRdInt("NOSignalThreshold",&powerThreshold);
#endif
#endif
#ifdef HOST_PC 
#ifndef NO_GUI
Fmt(message,"AGC1 status: %f ms",(Timer()-lockTime)*1000);
ReportInsertMessage(message);
#endif
#endif
//printf("<913_drv.c>Iqpower=%d NOSignalThreshold =%d Directblind =%d\n",iqPower,powerThreshold,Satellite_Scan);
FinalFreq = 0 ;

if (   ((agc1Power != 0) || (iqPower >= powerThreshold))
&& (pParams->DemodSearchAlgo == FE_SAT_BLIND_SEARCH)
&& (Satellite_Scan ==1)  /* AGC2 scan is launched */
)
{		
asperity = FE_STV0913_BlindCheckAGC2BandWidth(pParams) ;
//printf("asperity =%d\n",asperity);
}


if ((agc1Power == 0) && (iqPower < powerThreshold)) 
{
/*If (AGC1=0 and iqPower<IQThreshold)  then no signal  */
pParams->DemodResults.Locked=FALSE; /*if AGC1 integrator ==0 and iqPower < Threshold then NO signal*/
/* Jump of distance of the bandwith tuner */
signalType = FE_SAT_TUNER_NOSIGNAL ;
#ifdef HOST_PC 
#ifndef NO_GUI
ReportInsertMessage("NO AGC1 signal, NO POWERI, POWERQ Signal ");
ReportInsertMessage("---------------------------------------------"); 
#endif
#endif
//printf("NO AGC1 signal, NO POWERI, POWERQ Signal \n");
}
else if ( 	(pParams->DemodSearchAlgo == FE_SAT_BLIND_SEARCH)
&& (Satellite_Scan ==1)
&& (asperity==0))
{
iqPower = 0 ;
agc1Power = 0 ;
pParams->DemodResults.Locked=FALSE; /*if AGC1 integrator ==0 and iqPower < Threshold then NO signal*/
/*To indicate that it must jump wide receiver*/
signalType = FE_SAT_TUNER_NOSIGNAL ;

#ifdef HOST_PC 
#ifndef NO_GUI
ReportInsertMessage("Nothing in the band, jump BW / 2 ");
ReportInsertMessage("---------------------------------"); 
#endif
#endif
//printf("<913_drv.c>Nothing in the band, jump BW/ 2\n");

}
else if ( 	(pParams->DemodSearchAlgo == FE_SAT_BLIND_SEARCH)
&& (Satellite_Scan ==1)
&& (asperity==1)   /* YMO: First rising edge detected: We will make the blind tuner next step*/ /* YMO */
)
{
iqPower = 0 ;
agc1Power = 0 ;
pParams->DemodResults.Locked=FALSE; /*if AGC1 integrator ==0 and iqPower < Threshold then NO signal*/
/*To indicate that it must jump to a width of 4, 8 or 16 steps*/

/*Saut tuner*/

signalType=FE_SAT_TUNER_JUMP;

#ifdef HOST_PC 
#ifndef NO_GUI
ReportInsertMessage("Rising edge detected - No Blind - Skipping steps i"); 
#endif
#endif
//printf("<stv0913_drv.c>SignalType= TUNER_JUMP >>Rising edge detected - No Blind - Skipping steps i\n");
}
else	 /* YMO:  First falling edge detected or detected asked a blind direct: It is the blind */
{

/*Set the IQ inversion search mode*/
#if 0    /* formerly excluded lines */
ChipSetFieldImage(pParams->hDemod,FSTV0913_SPECINV_CONTROL,pParams->DemodSearch_IQ_Inv);
#endif   /* formerly excluded lines */
ChipSetFieldImage(pParams->hDemod,FSTV0913_SPECINV_CONTROL,FE_SAT_IQ_AUTO);/*bug IQ inversion: set IQ inversion always in auto*/
ChipSetRegisters(pParams->hDemod,RSTV0913_DEMOD,1);
FE_STV0913_SetSearchStandard(pParams);

#ifdef HOST_PC 
#ifndef NO_GUI
Fmt(message,"SearchStandard: %f ms",(Timer()-lockTime)*1000);
ReportInsertMessage(message);
#endif
#endif

/* 8PSK 3/5, 8PSK 2/3 Poff tracking optimization */
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S2Q,0x0B);
ChipSetOneRegister(pParams->hDemod,RSTV0913_ACLC2S28,0x0A);
ChipSetOneRegister(pParams->hDemod,RSTV0913_BCLC2S2Q,0x84);
ChipSetOneRegister(pParams->hDemod,RSTV0913_BCLC2S28,0x84);
ChipSetOneRegister(pParams->hDemod,RSTV0913_CARHDR,0x1c);  /*0x1C*/ /*YMO*/ 
if(pParams->DemodSearchAlgo!=FE_SAT_BLIND_SEARCH)
FE_STV0913_StartSearch(pParams);

if(pParams->DemodSearchAlgo==FE_SAT_BLIND_SEARCH)
{
lock = FE_STV0913_BlindSearchAlgo(pParams,demodTimeout,Satellite_Scan); /* suppresion parameter agc2level_step0*/ 
}
else
{
lock = FE_STV0913_GetDemodLock(pParams->hDemod,demodTimeout) ;	/* case warm or cold start wait for demod lock */
}

if (lock == TRUE)
{
signalType=FE_STV0913_GetSignalParams(pParams,Satellite_Scan);	/* Read signal caracteristics and check the lock range */
}
#ifdef HOST_PC 
#ifndef NO_GUI
Fmt(message,"GetSignal: %f ms %d %d",(Timer()-lockTime)*1000,lock,signalType);
ReportInsertMessage(message);
#endif
#endif
if ((lock == TRUE) && (signalType==FE_SAT_RANGEOK))
{
/*The tracking optimization and the FEC lock check are perfomed only if:
demod is locked and signal type is RANGEOK i.e a TP found  within the given search range 
*/

#if 0    /* formerly excluded lines */
FE_STV0913_TrackingOptimization(pParams);			/* Optimization setting for tracking */
#endif   /* formerly excluded lines */

ChipSetField(pParams->hDemod,FSTV0913_RST_HWARE,0);	/*Release stream merger reset */ 
ChipWaitOrAbort(pParams->hDemod,3);
ChipSetField(pParams->hDemod,FSTV0913_RST_HWARE,1);	/* Stream merger reset */  
ChipSetField(pParams->hDemod,FSTV0913_RST_HWARE,0);	/* Release stream merger reset */


#ifdef HOST_PC 
#ifndef NO_GUI
Fmt(message,"Bef WaitForLock: %f ms",(Timer()-lockTime)*1000);
ReportInsertMessage(message);
#endif
#endif

if(FE_STV0913_WaitForLock(pParams->hDemod,fecTimeout,fecTimeout)==TRUE)
{
lock=TRUE;
pParams->DemodResults.Locked=TRUE;
FE_Sat_TunerSetBandwidth(pParams->hTuner,pParams->TunerBW*10/15);

if(pParams->DemodResults.Standard==FE_SAT_DVBS2_STANDARD)
{
ChipSetFieldImage(pParams->hDemod,FSTV0913_RESET_UPKO_COUNT,1);/*reset DVBS2 packet delinator error counter */ 
ChipSetRegisters(pParams->hDemod,RSTV0913_PDELCTRL2,1);

ChipSetFieldImage(pParams->hDemod,FSTV0913_RESET_UPKO_COUNT,0);/*reset DVBS2 packet delinator error counter */ 
ChipSetRegisters(pParams->hDemod,RSTV0913_PDELCTRL2,1);	

ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL1,0x67);	/* reset the error counter to  DVBS2 packet error rate */
}
else
{
ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL1,0x75);	/* reset the viterbi bit error rate */ 
}
ChipSetOneRegister(pParams->hDemod,RSTV0913_FBERCPT4,0);    /*Reset the Total packet counter */
ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL2,0xc1); /*Reset the packet Error counter2 (and Set it to infinit error count mode )*/

#ifdef HOST_PC 
#ifndef NO_GUI
Fmt(message,"Aft WaitForLock: %f ms",(Timer()-lockTime)*1000);
ReportInsertMessage(message);
#endif
#endif
}
else
{
lock=FALSE;
signalType=FE_SAT_NODATA;	/*if the demod is locked and not the FEC signal type is no DATA*/
pParams->DemodResults.Locked=FALSE;
}

}

}
#ifdef HOST_PC 
#ifndef NO_GUI
/* Fmt(message,"AGC2 MIN  = %d",agc2Integrateur); */
Fmt(message,"FE_SAT_xxx : %d %f ms",signalType,(Timer()-lockTime)*1000);
ReportInsertMessage(message);

#endif
#endif

return signalType;
}
void FE_STV0913_SetHMRFilter(STCHIP_Handle_t hChip,TUNER_Handle_t hTuner)
{
S32 hmrVal,
agc1InitVal,
agc1PowerVal2,
agc1Reg =0,
agc1Field_msb=0,
agc1Field_lsb=0;

U32 tunerFrequency;


/* HMR filter calibration , specific 6120*/
if((((SAT_TUNER_Params_t) (hTuner->pData))->Model ==TUNER_STV6120_Tuner1) || (((SAT_TUNER_Params_t) (hTuner->pData))->Model ==TUNER_STV6120_Tuner2))  
{

{

agc1Reg= RSTV0913_AGCIQIN1;
agc1Field_msb=FSTV0913_AGCIQ_VALUE1;
agc1Field_lsb=FSTV0913_AGCIQ_VALUE0;

}

FE_Sat_TunerSetGain(hTuner,6);
FE_Sat_TunerSetHMR_Filter(hTuner,0);

tunerFrequency=FE_Sat_TunerGetFrequency(hTuner);
if(tunerFrequency<1500000)
{
ChipWaitOrAbort(hChip,5);

ChipGetRegisters(hChip,agc1Reg,2);
agc1InitVal=MAKEWORD(ChipGetFieldImage(hChip,agc1Field_msb),ChipGetFieldImage(hChip,agc1Field_lsb));  


FE_Sat_TunerSetGain(hTuner,8);
ChipWaitOrAbort(hChip,5);

ChipGetRegisters(hChip,agc1Reg,2);
agc1PowerVal2=MAKEWORD(ChipGetFieldImage(hChip,agc1Field_msb),ChipGetFieldImage(hChip,agc1Field_lsb));  
hmrVal=0;

while((agc1PowerVal2>agc1InitVal) &&(hmrVal<29))
{
hmrVal+=2;
FE_Sat_TunerSetHMR_Filter(hTuner,hmrVal);

ChipWaitOrAbort(hChip,5);
ChipGetRegisters(hChip,agc1Reg,2);
agc1PowerVal2=MAKEWORD(ChipGetFieldImage(hChip,agc1Field_msb),ChipGetFieldImage(hChip,agc1Field_lsb));  
}
}
}

}

/****************************************************************************************************************************
*****************************************************************************************************************************/

/****************************************************************
****************************************************************
**************************************************************** 
**															   **
**		***********************************************		   **
**						PUBLIC ROUTINES						   **
**		***********************************************		   **
**															   **
****************************************************************
**************************************************************** 
****************************************************************/


/****************************************************************************************************************************
*****************************************************************************************************************************/

/*****************************************************
--FUNCTION	::	FE_STV0913_GetRevision
--ACTION	::	Return current LLA version
--PARAMS IN	::	NONE
--PARAMS OUT::	NONE
--RETURN	::	Revision ==> Text string containing LLA version
--***************************************************/
ST_Revision_t FE_STV0913_GetRevision(void)
{
return (RevisionSTV0913);
}

/*****************************************************
--FUNCTION	::	FE_STV0913_Init
--ACTION	::	Initialisation of the STV0913 chip
--PARAMS IN	::	Handle	==>	Front End Handle
--PARAMS OUT::	NONE
--RETURN	::	Handle to STV0913
--***************************************************/
FE_STV0913_Error_t	FE_STV0913_Init(FE_STV0913_InitParams_t		*pInit,FE_STV0913_Handle_t *Handle)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR; 
FE_STV0913_InternalParams_t		*pParams = NULL;

TUNER_Error_t	tunerError = TUNER_NO_ERR;

STCHIP_Error_t  demodError = CHIPERR_NO_ERROR; 
Demod_InitParams_t		DemodInitParams;	/* Demodulator chip initialisation parameters */
SAT_TUNER_InitParams_t		TunerInitParams;

/* Internal params structure allocation */
#ifdef HOST_PC
STCHIP_Info_t			DemodChip;
pParams = calloc(1,sizeof(FE_STV0913_InternalParams_t));
(*Handle) = (FE_STV0913_Handle_t) pParams;
#endif
#ifdef CHIP_STAPI
pParams = (FE_STV0913_InternalParams_t *)(*Handle); 
#endif


if(pParams != NULL)
{
/* Chip initialisation */
#if defined(HOST_PC) && !defined(NO_GUI) 

pParams->hDemod  = DEMOD;
pParams->hTuner =  TUNER_A;
pParams->Quartz  = EXTCLK;

ChipSetField(pParams->hDemod,FSTV0913_TUN_IQSWAP,pInit->TunerIQ_Inversion);
ChipSetField(pParams->hDemod,FSTV0913_ROLLOFF_CONTROL,pInit->RollOff);

#else

#ifdef CHIP_STAPI
DemodInitParams.Chip = (pParams->hDemod); 
#else
DemodInitParams.Chip = &DemodChip;
#endif

/* Demodulator (STV0913) */
DemodInitParams.NbDefVal = STV0913_NBREGS;
DemodInitParams.Chip->RepeaterHost = NULL;
DemodInitParams.Chip->RepeaterFn = NULL;
DemodInitParams.Chip->Repeater = FALSE;
DemodInitParams.Chip->I2cAddr = pInit->DemodI2cAddr;
strcpy((char *)DemodInitParams.Chip->Name,pInit->DemodName);

demodError=STV0913_Init(&DemodInitParams,&pParams->hDemod);

if(pInit->TunerModel != TUNER_NULL)

TunerInitParams.Model=pInit->TunerModel;
TunerInitParams.TunerName=pInit->TunerName;
strcpy((char *)TunerInitParams.TunerName,pInit->TunerName);
TunerInitParams.TunerI2cAddress=pInit->Tuner_I2cAddr;
TunerInitParams.Reference=pInit->TunerRefClock;
TunerInitParams.IF=0;
if(pInit->TunerIQ_Inversion==FE_SAT_IQ_NORMAL)
TunerInitParams.IQ_Wiring= TUNER_IQ_NORMAL; 
else
TunerInitParams.IQ_Wiring= TUNER_IQ_INVERT;

#if 0    /* formerly excluded lines */
TunerInitParams.BandSelect=pInit->TunerBandSelect; 
#endif   /* formerly excluded lines */
TunerInitParams.RepeaterHost=pParams->hDemod;
TunerInitParams.RepeaterFn=FE_STV0913_RepeaterFn;
TunerInitParams.DemodModel=DEMOD_STV0913;
TunerInitParams.OutputClockDivider=pInit->TunerOutClkDivider;
/*TunerInitParams.Lna_agc_mode = pInit->TunerLnaAgcMode;
TunerInitParams.Lna_agc_ref = pInit->TunerLnaAgcRef; 
*/
TunerInitParams.RF_Source = pInit->TunerRF_Source;
TunerInitParams.InputSelect = pInit->TunerInputSelect;
/*printf("\n<%s,%d>TunerInitParams.BAndSelect is %d..RFSource =%d Inputselect=%d",__func__,__LINE__,TunerInitParams.BandSelect,TunerInitParams.RF_Source,TunerInitParams.InputSelect);*/


tunerError=FE_Sat_TunerInit( (&TunerInitParams),&pParams->hTuner);

if(demodError == CHIPERR_NO_ERROR) /*Check the demod error first*/
{
/*If no Error on the demod the check the Tuners*/
if(tunerError == TUNER_NO_ERR) 
error = FE_LLA_NO_ERROR;  /*if no error on the tuner*/
else if(tunerError == TUNER_INVALID_HANDLE)
error=FE_LLA_INVALID_HANDLE;   
else if(tunerError == TUNER_TYPE_ERR)
error=FE_LLA_BAD_PARAMETER;  /*if tuner type error*/
else 
error=FE_LLA_I2C_ERROR;
}else
{
if(demodError == CHIPERR_INVALID_HANDLE)
error=FE_LLA_INVALID_HANDLE;
else
error=FE_LLA_I2C_ERROR; 	
}


#endif

if(pParams->hDemod != NULL) 
{
if(error==FE_LLA_NO_ERROR)
{
/*Read IC cut ID*/
pParams->hDemod->ChipID=ChipGetOneRegister(pParams->hDemod,RSTV0913_MID);

/*Tuner parameters*/
pParams->TunerType=pInit->TunerHWControlType;
pParams->TunerGlobal_IQ_Inv=pInit->TunerIQ_Inversion;

pParams->RollOff=pInit->RollOff;
#if defined(CHIP_STAPI) || defined(NO_GUI)
pParams->Quartz = pInit->DemodRefClock;	/*Ext clock in Hz*/

ChipSetField(pParams->hDemod,FSTV0913_ROLLOFF_CONTROL,pInit->RollOff);

FE_STV0913_SetTS_Parallel_Serial(pParams->hDemod,pInit->PathTSClock);  /*Set TS1 and TS2 to serial or parallel mode */ 

switch (pInit->TunerHWControlType)
{
case FE_SAT_SW_TUNER:
default:
FE_Sat_TunerSetReferenceFreq(pParams->hTuner, pInit->TunerRefClock);   /* Set the tuner ref clock*/ 

FE_Sat_TunerSetGain(pParams->hTuner,pInit->TunerBasebandGain);   /*Set the tuner BBgain*/

/*Set the ADC's range according to the tuner Model*/
switch(pInit->TunerModel)
{
case TUNER_STB6000:
case TUNER_STB6100:
ChipSetOneRegister(pParams->hDemod,RSTV0913_TSTTNR1,0x27); /*Set the ADC range to 2Vpp*/ 
break;

case TUNER_STV6110:
case TUNER_STV6130:
case TUNER_STV6111: 
ChipSetOneRegister(pParams->hDemod,RSTV0913_TSTTNR1,0x26); /*Set the ADC range to 1Vpp*/ 	
break;

default:
break;

}

break;

}

ChipSetField(pParams->hDemod,FSTV0913_TUN_IQSWAP,pInit->TunerIQ_Inversion);  /*IQSWAP setting mast be after FE_STV0913_SetTunerType*/

FE_STV0913_SetMclk((FE_STV0913_Handle_t)pParams,135000000,pParams->Quartz); /*Set the Mclk value to 135MHz*/ 
ChipWaitOrAbort(pParams->hDemod,3); /*wait for PLL lock*/
ChipSetOneRegister(pParams->hDemod,RSTV0913_SYNTCTRL,0x22);		/* switch the 900 to the PLL*/

#endif

pParams->MasterClock = FE_STV0913_GetMclkFreq(pParams->hDemod,pParams->Quartz);	/*Read the cuurent Mclk*/

if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) /*Check the error at the end of the Init*/
error=FE_LLA_I2C_ERROR;
}
}  
else
{
error=FE_LLA_INVALID_HANDLE;
}
}

return error;
}

FE_STV0913_Error_t	FE_STV0913_SetTSoutput(FE_STV0913_Handle_t	Handle,FE_STV0913_TSConfig_t *PathpTSConfig)
{

FE_STV0913_Error_t error = FE_LLA_NO_ERROR;      
FE_STV0913_InternalParams_t	*pParams = NULL;


pParams = (FE_STV0913_InternalParams_t	*) Handle;
if(pParams != NULL)
{
if (pParams->hDemod->Error )
error=FE_LLA_I2C_ERROR; 
else
{
if(error == FE_LLA_NO_ERROR)
{

switch(PathpTSConfig->TSMode)
{
case FE_TS_PARALLEL_PUNCT_CLOCK:
#if 0    /* formerly excluded lines */
ChipSetField(pParams->hDemod,FSTV0913_TS_OUTSER_HZ,0x01);
#endif   /* formerly excluded lines */
ChipSetField(pParams->hDemod,FSTV0913_TS1_OUTPAR_HZ,0x00);
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_SERIAL,0x00); /* parallel mode   */
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_DVBCI,0x00);
break;

case FE_TS_DVBCI_CLOCK:
#if 0    /* formerly excluded lines */
ChipSetField(pParams->hDemod,FSTV0913_TS_OUTSER_HZ,0x01);
#endif   /* formerly excluded lines */
ChipSetField(pParams->hDemod,FSTV0913_TS1_OUTPAR_HZ,0x00);
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_SERIAL,0x00); /* parallel mode   */
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_DVBCI,0x01);
break;

case FE_TS_SERIAL_PUNCT_CLOCK:
ChipSetField(pParams->hDemod,FSTV0913_TS1_OUTSER_HZ,0x00);
ChipSetField(pParams->hDemod,FSTV0913_TS1_OUTPAR_HZ,0x01);
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_SERIAL,0x01);	/* serial mode   */
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_DVBCI,0x00);
break;

case FE_TS_SERIAL_CONT_CLOCK:
ChipSetField(pParams->hDemod,FSTV0913_TS1_OUTSER_HZ,0x00);
ChipSetField(pParams->hDemod,FSTV0913_TS1_OUTPAR_HZ,0x01);
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_SERIAL,0x01);	/* serial mode   */
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_DVBCI,0x01);
break;

default:
break;
}
ChipSetField(pParams->hDemod,FSTV0913_RST_HWARE,1);/*Reset stream merger*/
ChipSetField(pParams->hDemod,FSTV0913_RST_HWARE,0);

}
}
}
return error;
}
/*****************************************************
--FUNCTION	::	FE_STV0913_SetTSConfig
--ACTION	::	TS configuration 
--PARAMS IN	::	Handle		==>	Front End Handle
::	Path1pTSConfig	==> path1 TS config parameters 	
::	Path2pTSConfig	==> path2 TS config parameters 
--PARAMS OUT::	Path1TSSpeed_Hz	==> path1 Current TS speed in Hz.
Path2TSSpeed_Hz	==> path2 Current TS speed in Hz. 
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_SetTSConfig(	FE_STV0913_Handle_t Handle ,
FE_STV0913_TSConfig_t *PathpTSConfig,
U32 *PathTSSpeed_Hz
)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;      
FE_STV0913_InternalParams_t	*pParams = NULL;

U32 tsspeed;

pParams = (FE_STV0913_InternalParams_t	*) Handle;
if(pParams != NULL)
{
if (pParams->hDemod->Error )
error=FE_LLA_I2C_ERROR; 
else
{

ChipSetOneRegister(pParams->hDemod,RSTV0913_TSGENERAL,0);


if(PathpTSConfig->TSSyncByteEnable == FE_TS_SYNCBYTE_OFF)	  /* enable/Disable SYNC byte */
ChipSetField(pParams->hDemod,FSTV0913_TSDEL_SYNCBYTE,1);
else
ChipSetField(pParams->hDemod,FSTV0913_TSDEL_SYNCBYTE,0);

if(PathpTSConfig->TSParityBytes== FE_TS_PARITYBYTES_ON)	/*DVBS1 Data parity bytes enabling/disabling*/
ChipSetField(pParams->hDemod,FSTV0913_TSINSDEL_RSPARITY,1);
else
ChipSetField(pParams->hDemod,FSTV0913_TSINSDEL_RSPARITY,0);


if(PathpTSConfig->TSClockPolarity == FE_TS_RISINGEDGE_CLOCK)	/*TS clock Polarity setting : rising edge/falling edge*/
ChipSetField(pParams->hDemod,FSTV0913_TS1_CLOCKOUT_XOR,0);
else
ChipSetField(pParams->hDemod,FSTV0913_TS1_CLOCKOUT_XOR,1);

if(PathpTSConfig->TSSpeedControl== FE_TS_MANUAL_SPEED)
{
/*path 2 TS speed setting*/
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_MANSPEED,3);
switch(PathpTSConfig->TSMode)
{
case FE_TS_PARALLEL_PUNCT_CLOCK:
case FE_TS_DVBCI_CLOCK:
/*Formulat :
TS_Speed_Hz = 4 * Msterclock_Hz / TSSPEED_REG
*/
tsspeed= (4 * pParams->MasterClock)/ PathpTSConfig->TSClockRate;
if(tsspeed<=16)
tsspeed=16; /*in parallel clock the TS speed is limited < MasterClock/4*/ 

ChipSetOneRegister(pParams->hDemod,RSTV0913_TSSPEED,tsspeed); 

break;

case FE_TS_SERIAL_PUNCT_CLOCK:
case FE_TS_SERIAL_CONT_CLOCK:
case FE_TS_OUTPUTMODE_DEFAULT: 
default:

/*Formulat :
TS_Speed_Hz = 32 * Msterclock_Hz / TSSPEED_REG
*/
tsspeed= (16 * pParams->MasterClock)/ (PathpTSConfig->TSClockRate/2);
if(tsspeed<=32)
tsspeed=32; /*in serial clock the TS speed is limited <= MasterClock*/

ChipSetOneRegister(pParams->hDemod,RSTV0913_TSSPEED,tsspeed); 
break;
}

}
if(PathpTSConfig->TSMode==FE_TS_DVBCI_CLOCK)
{

ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_MANSPEED,1);
/*Formulat :
TS_Speed_Hz = 32 * Msterclock_Hz / TSSPEED_REG
*/
tsspeed= (16 * pParams->MasterClock)/ 9000000; /*if DVBCI set TS clock to 9MHz*/
if(tsspeed<=32)
tsspeed=32; /*in serial clock the TS speed is limited <= MasterClock*/

ChipSetOneRegister(pParams->hDemod,RSTV0913_TSSPEED,tsspeed); 

}
else
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_MANSPEED,0);   	


switch(PathpTSConfig->TSMode)	/*D7/D0 permute if serial*/
{
case FE_TS_SERIAL_PUNCT_CLOCK:
case FE_TS_SERIAL_CONT_CLOCK:
case FE_TS_OUTPUTMODE_DEFAULT: 
default:
if(PathpTSConfig->TSSwap==FE_TS_SWAP_ON)
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_PERMDATA,1);
else
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_PERMDATA,0);  
break;

case FE_TS_PARALLEL_PUNCT_CLOCK:
case FE_TS_DVBCI_CLOCK:
ChipSetField(pParams->hDemod,FSTV0913_TSFIFO_PERMDATA,0);
break;
}



if(ChipGetField(pParams->hDemod,FSTV0913_TSFIFO_SERIAL)==0) /*Parallel mode*/
*PathTSSpeed_Hz= (4 * pParams->MasterClock)/ChipGetOneRegister(pParams->hDemod,RSTV0913_TSSPEED);
else	/*serial mode*/ 
{
*PathTSSpeed_Hz= (16 * pParams->MasterClock)/ChipGetOneRegister(pParams->hDemod,RSTV0913_TSSPEED);
(*PathTSSpeed_Hz)*=2;
}

if ( pParams->hDemod->Error )  /*Check the error at the end of the function*/ 
error=FE_LLA_I2C_ERROR; 
}
}
else
error=FE_LLA_INVALID_HANDLE; 


return error;
}
/*****************************************************
--FUNCTION	::	FE_STV0913_GetSignalInfo
--ACTION	::	Return informations on the locked transponder
--PARAMS IN	::	Handle	==>	Front End Handle
--PARAMS OUT::	pInfo	==> Informations (BER,C/N,power ...)
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t	FE_STV0913_GetSignalInfo(FE_STV0913_Handle_t Handle,FE_STV0913_SignalInfo_t	*pInfo)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = NULL;
FE_STV0913_SEARCHSTATE_t	demodState;

S32 symbolRateOffset;
//int i=0;
pParams = (FE_STV0913_InternalParams_t	*) Handle;

if(pParams != NULL)
{		
if ((pParams->hDemod->Error) || (pParams->hTuner->Error) )
error=FE_LLA_I2C_ERROR; 
else
{
demodState=ChipGetField(pParams->hDemod,FSTV0913_HEADER_MODE);
switch(demodState)
{

case FE_SAT_SEARCH:
case FE_SAT_PLH_DETECTED :
default :
pInfo->Locked = FALSE;
break;
case FE_SAT_DVBS2_FOUND:
pInfo->Locked = ChipGetField(pParams->hDemod,FSTV0913_LOCK_DEFINITIF) &&
ChipGetField(pParams->hDemod,FSTV0913_PKTDELIN_LOCK) &&
ChipGetField(pParams->hDemod,FSTV0913_TSFIFO_LINEOK); 

break;

case FE_SAT_DVBS_FOUND:
pInfo->Locked = ChipGetField(pParams->hDemod,FSTV0913_LOCK_DEFINITIF) &&
ChipGetField(pParams->hDemod,FSTV0913_LOCKEDVIT) &&
ChipGetField(pParams->hDemod,FSTV0913_TSFIFO_LINEOK);  

break;
}

{
/* transponder_frequency = tuner +  demod carrier frequency */
pInfo->Frequency	=  FE_Sat_TunerGetFrequency(pParams->hTuner);

pInfo->Frequency	+= FE_STV0913_GetCarrierFrequency(pParams->hDemod,pParams->MasterClock)/1000;

pInfo->SymbolRate	=  FE_STV0913_GetSymbolRate(pParams->hDemod,pParams->MasterClock);	/* Get symbol rate */
symbolRateOffset=FE_STV0913_TimingGetOffset(pParams->hDemod,pInfo->SymbolRate);
pInfo->SymbolRate	+= symbolRateOffset;	/* Get timing loop offset */

pInfo->Standard		=  FE_STV0913_GetStandard(pParams->hDemod); 

pInfo->PunctureRate	=  FE_STV0913_GetPunctureRate(pParams->hDemod);
pInfo->ModCode		=  ChipGetField(pParams->hDemod,FSTV0913_DEMOD_MODCOD);
pInfo->Pilots 		=  ChipGetFieldImage(pParams->hDemod,FSTV0913_DEMOD_TYPE) & 0x01;
pInfo->FrameLength	=  ((U32)ChipGetFieldImage(pParams->hDemod,FSTV0913_DEMOD_TYPE))>>1;
pInfo->RollOff		=  ChipGetField(pParams->hDemod,FSTV0913_ROLLOFF_STATUS);


pInfo->Power 		=  FE_STV0913_GetRFLevel(pParams->hDemod,&FE_STV0913_RF_LookUp);

//pInfo->Power=FE_STV0913_GetRFLevelWithVGLNA(pParams);		
/*Update BER ; only if ! BlindSearch*/
if(pParams->DemodSearchAlgo != FE_SAT_BLIND_SEARCH)
{
pInfo->BER 			=  FE_STV0913_GetBer(pParams->hDemod);

}
else
pInfo->BER 			= 0;
//printk("<%s>**.pInfo->Frequency =%d..pInfo->SR =%d Standard =%d LOCK=%d\n",__FUNCTION__,pInfo->Frequency,pInfo->SymbolRate,pInfo->Standard,pInfo->Locked);

if(pInfo->Standard==FE_SAT_DVBS2_STANDARD)
{
if(pParams->DemodSearchAlgo != FE_SAT_BLIND_SEARCH)
pInfo->C_N 			=  FE_STV0913_CarrierGetQuality(pParams->hDemod,&FE_STV0913_S2_CN_LookUp);
else
pInfo->C_N   = 100 ; //deafult good value

pInfo->Spectrum=ChipGetField(pParams->hDemod,FSTV0913_SPECINV_DEMOD);

if(pInfo->ModCode <= FE_SAT_QPSK_910)
pInfo->Modulation=FE_SAT_MOD_QPSK;
else if(pInfo->ModCode <= FE_SAT_8PSK_910)
pInfo->Modulation=FE_SAT_MOD_8PSK;
else if(pInfo->ModCode <= FE_SAT_16APSK_910)
pInfo->Modulation=FE_SAT_MOD_16APSK;
else if(pInfo->ModCode <= FE_SAT_32APSK_910)
pInfo->Modulation=FE_SAT_MOD_32APSK;
else
pInfo->Modulation=FE_SAT_MOD_UNKNOWN;

/*reset The error counter to PER*/
ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL1,0x67);	/* reset the error counter to  DVBS2 packet error rate */ 

}
else /*DVBS1/DSS*/
{
if(pParams->DemodSearchAlgo != FE_SAT_BLIND_SEARCH)
pInfo->C_N 			=  FE_STV0913_CarrierGetQuality(pParams->hDemod,&FE_STV0913_S1_CN_LookUp);
else
pInfo->C_N  = 100 ; //default good value

pInfo->Spectrum=ChipGetField(pParams->hDemod,FSTV0913_IQINV);

pInfo->Modulation=FE_SAT_MOD_QPSK;

}

}

if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) /*Check the error at the end of the function*/
error=FE_LLA_I2C_ERROR; 
}
}
else
error = FE_LLA_INVALID_HANDLE; 

return error; 
}

/*****************************************************
--FUNCTION	::	FE_STV0913_GetSignalInfoLite
--ACTION	::	Return C/N, ModCode, Puncturate,Pilot
--PARAMS IN	::	Handle	==>	Front End Handle
::	Demod	==>	Cuurent demod 1 or 2
--PARAMS OUT::	pInfo	==> Informations (BER,C/N,power ...)
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t	FE_STV0913_GetSignalInfoLite(FE_STV0913_Handle_t Handle,FE_STV0913_SignalInfo_t	*pInfo)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = NULL;
//FE_STV0913_SEARCHSTATE_t	demodState;


pParams = (FE_STV0913_InternalParams_t	*) Handle;

if(pParams != NULL)
{		
if ( (pParams->hDemod->Error ) || (pParams->hTuner->Error)) 
error=FE_LLA_I2C_ERROR; 
else
{
#if 0    /* formerly excluded lines */
demodState=ChipGetField(pParams->hDemod,FSTV0913_HEADER_MODE);
switch(demodState)
{

case FE_SAT_SEARCH:
case FE_SAT_PLH_DETECTED :
default :
pInfo->Locked = FALSE;
break;
case FE_SAT_DVBS2_FOUND:
pInfo->Locked = ChipGetField(pParams->hDemod,FSTV0913_LOCK_DEFINITIF) &&
ChipGetField(pParams->hDemod,FSTV0913_PKTDELIN_LOCK) &&
ChipGetField(pParams->hDemod,FSTV0913_TSFIFO_LINEOK); 
break;

case FE_SAT_DVBS_FOUND:
pInfo->Locked = ChipGetField(pParams->hDemod,FSTV0913_LOCK_DEFINITIF) &&
ChipGetField(pParams->hDemod,FSTV0913_LOCKEDVIT) &&
ChipGetField(pParams->hDemod,FSTV0913_TSFIFO_LINEOK);  
break;
}
#endif   /* formerly excluded lines */

pInfo->Standard		=  FE_STV0913_GetStandard(pParams->hDemod); 
if(pInfo->Standard==FE_SAT_DVBS2_STANDARD)
{
pInfo->C_N 			=  FE_STV0913_CarrierGetQuality(pParams->hDemod,&FE_STV0913_S2_CN_LookUp);
}

if ( (pParams->hDemod->Error ) || (pParams->hTuner->Error)) 
error=FE_LLA_I2C_ERROR; 
}

}
else
error = FE_LLA_INVALID_HANDLE; 

return error; 
}
/*****************************************************
--FUNCTION	::	FE_STV0913_Tracking
--ACTION	::	Return Tracking informations : lock status, RF level, C/N and BER.
--PARAMS IN	::	Handle	==>	Front End Handle
::  
--PARAMS OUT::	pTrackingInfo	==> pointer to FE_Sat_TrackingInfo_t struct.
::  
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_Tracking(FE_STV0913_Handle_t Handle, FE_Sat_TrackingInfo_t *pTrackingInfo)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = NULL;
FE_STV0913_SEARCHSTATE_t	demodState;

pParams = (FE_STV0913_InternalParams_t	*) Handle;

if(pParams != NULL)
{		

if ( (pParams->hDemod->Error ) || (pParams->hTuner->Error)) 
error=FE_LLA_I2C_ERROR; 
else
{
pTrackingInfo->Power =  FE_STV0913_GetRFLevel(pParams->hDemod,&FE_STV0913_RF_LookUp); 
//pTrackingInfo->Power=FE_STV0913_GetRFLevelWithVGLNA(pParams);
pTrackingInfo->BER 			=  FE_STV0913_GetBer(pParams->hDemod);
demodState=ChipGetField(pParams->hDemod,FSTV0913_HEADER_MODE);
switch(demodState)
{
case FE_SAT_SEARCH:
case FE_SAT_PLH_DETECTED :
default :
pTrackingInfo->Locked = FALSE;
pTrackingInfo->C_N 	  = 0;
break;
case FE_SAT_DVBS2_FOUND:
pTrackingInfo->Locked = ChipGetField(pParams->hDemod,FSTV0913_LOCK_DEFINITIF) &&
ChipGetField(pParams->hDemod,FSTV0913_PKTDELIN_LOCK) &&
ChipGetField(pParams->hDemod,FSTV0913_TSFIFO_LINEOK);

pTrackingInfo->C_N 			=  FE_STV0913_CarrierGetQuality(pParams->hDemod,&FE_STV0913_S2_CN_LookUp);
/*reset The error counter to PER*/
ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL1,0x67);	/* reset the error counter to  DVBS2 packet error rate */ 

break;

case FE_SAT_DVBS_FOUND:
pTrackingInfo->Locked = ChipGetField(pParams->hDemod,FSTV0913_LOCK_DEFINITIF) &&
ChipGetField(pParams->hDemod,FSTV0913_LOCKEDVIT) &&
ChipGetField(pParams->hDemod,FSTV0913_TSFIFO_LINEOK);

pTrackingInfo->C_N 			=  FE_STV0913_CarrierGetQuality(pParams->hDemod,&FE_STV0913_S1_CN_LookUp); 
break;
}

pTrackingInfo->Frequency_IF =  FE_Sat_TunerGetFrequency(pParams->hTuner);
pTrackingInfo->Frequency_IF += FE_STV0913_GetCarrierFrequency(pParams->hDemod,pParams->MasterClock)/1000;

if ( (pParams->hDemod->Error ) || (pParams->hTuner->Error)) 
error=FE_LLA_I2C_ERROR; 
}

}
else
error = FE_LLA_INVALID_HANDLE;

return error;
}


/*****************************************************
--FUNCTION	::	FE_STV913_GetPacketErrorRate
--ACTION	::	Return informations the number of error packet and the window packet count
--PARAMS IN	::	Handle	==>	Front End Handle
--PARAMS OUT::	PacketsErrorCount	==> Number of packet error, max is 2^23 packet.
::  TotalPacketsCount   ==> total window packets , max is 2^24 packet.
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV913_GetPacketErrorRate(FE_STV0913_Handle_t Handle, U32 *PacketsErrorCount,U32 *TotalPacketsCount)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = NULL;

U8 packetsCount4=0,
packetsCount3=0,
packetsCount2=0,
packetsCount1=0,
packetsCount0=0;


pParams = (FE_STV0913_InternalParams_t	*) Handle;

if(pParams != NULL)
{		
if ((pParams->hDemod->Error) || (pParams->hTuner->Error) )
error=FE_LLA_I2C_ERROR; 
else
{

if(FE_STV0913_Status(Handle)==FALSE) /*if Demod+FEC not locked */
{
*PacketsErrorCount=1<<23;		 /*Packet error count is set to the maximum value*/	
*TotalPacketsCount=1<<24;		 /*Total Packet count is set to the maximum value*/
}
else
{
/*Read the error counter 2 (23 bits) */
*PacketsErrorCount= FE_STV0913_GetErrorCount(pParams->hDemod,FE_STV0913_COUNTER2)&0x7FFFFF;

/*Read the total packet counter 40 bits, read 5 bytes is mondatory */
packetsCount4=ChipGetOneRegister(pParams->hDemod,RSTV0913_FBERCPT4);    /*Read the Total packet counter byte 5*/
packetsCount3=ChipGetOneRegister(pParams->hDemod,RSTV0913_FBERCPT3);    /*Read the Total packet counter byte 4*/  
packetsCount2=ChipGetOneRegister(pParams->hDemod,RSTV0913_FBERCPT2);    /*Read the Total packet counter byte 3*/  
packetsCount1=ChipGetOneRegister(pParams->hDemod,RSTV0913_FBERCPT1);    /*Read the Total packet counter byte 2*/  
packetsCount0=ChipGetOneRegister(pParams->hDemod,RSTV0913_FBERCPT0);    /*Read the Total packet counter byte 1*/

if( (packetsCount4 ==0) && (packetsCount3 ==0) )/*Use the counter for a maximum of 2^24 packets*/  
{
*TotalPacketsCount=((packetsCount2 & 0xFF) <<16)+((packetsCount1 & 0xFF) <<8)+(packetsCount0 & 0xFF);
}
else
{
*TotalPacketsCount=1<<24;
}

if(*TotalPacketsCount==0)
{
/*if the packets count doesn't start yet the packet error =1 and total packets =1 */
/* if the packet counter doesn't start there is an FEC error in the */
*TotalPacketsCount=1;
*PacketsErrorCount=1;
}
}

ChipSetOneRegister(pParams->hDemod,RSTV0913_FBERCPT4,0);    /*Reset the Total packet counter */ 
ChipSetField(pParams->hDemod,FSTV0913_BERMETER_RESET,1);
ChipSetField(pParams->hDemod,FSTV0913_BERMETER_RESET,0);
ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL2,0xc1); /*Reset the packet Error counter2 (and Set it to infinit error count mode )*/

if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) /*Check the error at the end of the function*/
error=FE_LLA_I2C_ERROR; 
}
}
else
error = FE_LLA_INVALID_HANDLE;

return error;
}

/*****************************************************
--FUNCTION	::	FE_STV913_GetPreViterbiBER
--ACTION	::	Return DEMOD BER (Pre-VITERBI BER) for DVB-S1 only
--PARAMS IN	::	Handle	==>	Front End Handle
--PARAMS OUT::  Pre-VITERBI BER x 10^7	
--RETURN	::	Error (if any)  
--***************************************************/
FE_STV0913_Error_t FE_STV913_GetPreViterbiBER(FE_STV0913_Handle_t Handle, U32 *PreVirebiBER)
{

/* warning, when using this function the error counter number 1 is set to pre-Viterbi BER, 
the post viterbi BER info (given in FE_STV0900_GetSignalInfo function or FE_STV0900_Tracking is not significant any more
*/

FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = NULL;

U32 errorCount=10000000,
frameSize;
FE_STV0913_Rate_t punctureRate;

pParams = (FE_STV0913_InternalParams_t	*) Handle;  

if(pParams != NULL)
{		
if (pParams->hDemod->Error) 
error=FE_LLA_I2C_ERROR; 
else
{

if(ChipGetField(pParams->hDemod,FSTV0913_LOCKEDVIT)) /* check for VITERBI lock*/
{
ChipSetOneRegister(pParams->hDemod,RSTV0913_ERRCTRL1,0x13); /* Set and reset the error counter1 to Pre-Viterbi BER with observation window =2^6 frames*/
WAIT_N_MS(50); /* wait for error accumulation*/

punctureRate=FE_STV0913_GetPunctureRate(pParams->hDemod) ; /* Read the puncture rate*/

/* Read the error counter*/
errorCount=((ChipGetOneRegister(pParams->hDemod,RSTV0913_ERRCNT12)& 0x7F)<<16)+
((ChipGetOneRegister(pParams->hDemod,RSTV0913_ERRCNT11)& 0xFF)<<8)+
((ChipGetOneRegister(pParams->hDemod,RSTV0913_ERRCNT10)& 0xFF));

switch(punctureRate)
{
/*
compute the frame size
frame size = 2688*2*PR;
*/
case FE_SAT_PR_1_2:
frameSize=(5376)/2;
break;

case FE_SAT_PR_2_3:
frameSize=(5376*2)/3;
break;

case FE_SAT_PR_3_4:
frameSize=(5376*3)/4;
break;

case FE_SAT_PR_5_6:
frameSize=(5376*5)/6;
break;

case FE_SAT_PR_6_7:
frameSize=(5376*6)/7;
break;

case FE_SAT_PR_7_8:
frameSize=(5376*7)/8;
break;

default:
errorCount=1;
frameSize=1;
break;
}

if(frameSize>1000)
{
frameSize*=64;/*total window size = frameSize*2^6 */
errorCount=(errorCount*1000)/(frameSize/1000);
errorCount*=10;
}
else
{
errorCount=10000000; /* if PR is unknown BER=1 */
}
}
else
errorCount=10000000; /*if VITERBI is not locked BER=1*/ 

if (pParams->hDemod->Error) 
error=FE_LLA_I2C_ERROR; 
}
}
*PreVirebiBER=errorCount;

return error;
}



/*****************************************************
--FUNCTION	::	FE_STV0913_Status
--ACTION	::	Return locked status
--PARAMS IN	::	Handle	==>	Front End Handle
--PARAMS OUT::	NONE
--RETURN	::	Bool (locked or not)
--***************************************************/
BOOL	FE_STV0913_Status(	FE_STV0913_Handle_t	Handle)
{

FE_STV0913_InternalParams_t	*pParams = NULL;
BOOL Locked=FALSE;
FE_STV0913_SEARCHSTATE_t	demodState;
FE_STV0913_Error_t error = FE_LLA_NO_ERROR; 

pParams = (FE_STV0913_InternalParams_t	*) Handle;

if(pParams != NULL)
{
if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) 
{
error=FE_LLA_I2C_ERROR;
Locked=FALSE;
}
else
{
demodState=ChipGetField(pParams->hDemod,FSTV0913_HEADER_MODE);
switch(demodState)
{
case FE_SAT_SEARCH:
case FE_SAT_PLH_DETECTED :
default :
Locked = FALSE;
break;

case FE_SAT_DVBS2_FOUND:
Locked = ChipGetField(pParams->hDemod,FSTV0913_LOCK_DEFINITIF) &&
ChipGetField(pParams->hDemod,FSTV0913_PKTDELIN_LOCK) &&
ChipGetField(pParams->hDemod,FSTV0913_TSFIFO_LINEOK); 
break;

case FE_SAT_DVBS_FOUND:
Locked = ChipGetField(pParams->hDemod,FSTV0913_LOCK_DEFINITIF) &&
ChipGetField(pParams->hDemod,FSTV0913_LOCKEDVIT) &&
ChipGetField(pParams->hDemod,FSTV0913_TSFIFO_LINEOK); 
break;
}

if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) 
error=FE_LLA_I2C_ERROR; 
}
}
else
Locked=FALSE;

return Locked;

}

/*****************************************************
--FUNCTION	::	FE_STV0913_Unlock
--ACTION	::	Unlock the demodulator , set the demod to idle state  
--PARAMS IN	::	Handle	==>	Front End Handle

-PARAMS OUT::	NONE.
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_Unlock (FE_STV0913_Handle_t Handle)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = (FE_STV0913_InternalParams_t	*)Handle;

if(pParams != NULL)
{
if ((pParams->hDemod->Error) || (pParams->hTuner->Error) )
error=FE_LLA_I2C_ERROR; 
else
{
ChipSetOneRegister(pParams->hDemod,RSTV0913_DMDISTATE,0x5C); /* Demod Stop*/

if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) 
error=FE_LLA_I2C_ERROR; 
}
}
else
error=FE_LLA_INVALID_HANDLE;

return(error);
}

/*****************************************************
--FUNCTION	::	FE_STV0913_ResetDevicesErrors
--ACTION	::	Reset Devices I2C error status  
--PARAMS IN	::	Handle	==>	Front End Handle
::	
-PARAMS OUT::	NONE.
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_ResetDevicesErrors (FE_STV0913_Handle_t Handle)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = (FE_STV0913_InternalParams_t	*)Handle;

if(pParams != NULL)
{
if(pParams->hDemod != NULL)
pParams->hDemod->Error = CHIPERR_NO_ERROR;
else
error = FE_LLA_INVALID_HANDLE;

if(pParams->hTuner != NULL)
pParams->hTuner->Error = CHIPERR_NO_ERROR;
else 
error = FE_LLA_INVALID_HANDLE;
}
else
error=FE_LLA_INVALID_HANDLE;

return(error);
}
void FE_STV0913_SetChargePump(FE_STV0913_Handle_t Handle, U32 n_div)
{
U32 cp;
FE_STV0913_InternalParams_t	*pParams = (FE_STV0913_InternalParams_t	*)Handle;

if(n_div >= 7 && n_div <= 71)
cp = 7;
else if(n_div >= 72 && n_div <= 79)
cp = 8;
else if(n_div >= 80 && n_div <= 87) 
cp = 9;
else if(n_div >= 88 && n_div <= 95) 
cp = 10;
else if(n_div >= 96 && n_div <= 103)
cp = 11;
else if(n_div >= 104 && n_div <= 111) 
cp = 12;
else if(n_div >= 112 && n_div <= 119)
cp = 13;
else if(n_div >= 120 && n_div <= 127)
cp = 14;
else if(n_div >= 128 && n_div <= 135) 
cp = 15;
else if(n_div >= 136 && n_div <= 143) 
cp = 16;
else if(n_div >= 144 && n_div <= 151)
cp = 17;
else if(n_div >= 152 && n_div <= 159) 
cp = 18;
else if(n_div >= 160 && n_div <= 167)
cp = 19;
else if(n_div >= 168 && n_div <= 175)
cp = 20;
else if(n_div >= 176 && n_div <= 183) 
cp = 21;
else if(n_div >= 184 && n_div <= 191) 
cp = 22;
else if(n_div >= 192 && n_div <= 199)
cp = 23;
else if(n_div >= 200 && n_div <= 207) 
cp = 24;
else if(n_div >= 208 && n_div <= 215)
cp = 25;
else if(n_div >= 216 && n_div <= 223)
cp = 26;
else if(n_div >= 224 && n_div <= 225) 
cp = 27;
else{ cp = 7;}

ChipSetField(pParams->hDemod, FSTV0913_CP, cp);
}

/*****************************************************
--FUNCTION	::	FE_STV0913_STV6120_HMRFilter
--ACTION	::	use this function after the acquisition to calibrate the STV6120 tuner HMR filter 
--PARAMS IN	::	Handle	==>	Front End Handle
::	Demod	==>	Current demod 1 or 2 
-PARAMS OUT::	NONE.
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_STV6120_HMRFilter (FE_STV0913_Handle_t Handle)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = (FE_STV0913_InternalParams_t	*)Handle;

if(pParams != NULL)
{

if ( (pParams->hDemod->Error ) || (pParams->hTuner->Error) )
error=FE_LLA_I2C_ERROR;
else
{
FE_STV0913_SetHMRFilter(pParams->hDemod,pParams->hTuner); 
if ( (pParams->hDemod->Error ) || (pParams->hTuner->Error) )  /*Check the error at the end of the function*/
error=FE_LLA_I2C_ERROR;

}		
}
else
error=FE_LLA_INVALID_HANDLE;

return(error);
}  

/*****************************************************
--FUNCTION	::	FE_STV0913_SetMclk
--ACTION	::	Set demod Master Clock  
--PARAMS IN	::	Handle	==>	Front End Handle
::	Mclk 	==> demod master clock
::	ExtClk 	==>	external Quartz
--PARAMS OUT::	NONE.
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_SetMclk(FE_STV0913_Handle_t Handle, U32 Mclk,U32 ExtClk)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = (FE_STV0913_InternalParams_t	*)Handle;


U32 ndiv, quartz, idf, odf, Fphi;

if(pParams == NULL)
error=FE_LLA_INVALID_HANDLE;
else
{
if ( pParams->hDemod->Error ) 
error=FE_LLA_I2C_ERROR; 
else
{

/* 800MHz < Fvco < 1800MHz  							*/
/* Fvco = (ExtClk * 2 * NDIV) / IDF 					*/
/* (400 * IDF) / ExtClk < NDIV < (900 * IDF) / ExtClk 	*/

/* ODF forced to 4 otherwise desynchronization of digital and analog clock which result to a bad calculated symbolrate */
odf = 4;
/* IDF forced to 1 : Optimal value */
idf = 1;

ChipSetField(pParams->hDemod,FSTV0913_ODF, odf);
ChipSetField(pParams->hDemod,FSTV0913_IDF, idf);

quartz = ExtClk/1000000;
Fphi = Mclk/1000000;

ndiv = (Fphi * odf * idf) / quartz;

ChipSetField(pParams->hDemod,FSTV0913_N_DIV,ndiv);

/* Set CP according to NDIV */
FE_STV0913_SetChargePump(Handle, ndiv);


if ( pParams->hDemod->Error ) /*Check the error at the end of the function*/ 
error=FE_LLA_I2C_ERROR; 
}
}

return(error);

}

/*****************************************************
--FUNCTION	::	FE_STV0913_SetStandby
--ACTION	::	Set demod STANDBAY mode On/Off  
--PARAMS IN	::	Handle	==>	Front End Handle

-PARAMS OUT::	NONE.
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_SetStandby(FE_STV0913_Handle_t Handle, U8 StandbyOn)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = (FE_STV0913_InternalParams_t	*)Handle;

if(pParams != NULL)
{
if ((pParams->hDemod->Error) || (pParams->hTuner->Error) )
error=FE_LLA_I2C_ERROR; 
else
{
if(StandbyOn)
{	

ChipSetField(pParams->hDemod,FSTV0913_FSK_PON,0);   /*FSK power off*/ 
ChipSetField(pParams->hDemod,FSTV0913_ADC1_PON,0); /*ADC1 power off*/
ChipSetField(pParams->hDemod,FSTV0913_I2C_DISEQC_PON,0); /*DiseqC 1 power off*/

/*Stop Path1 Clocks*/
ChipSetField(pParams->hDemod,FSTV0913_STOP_DEMOD,1);   /*Stop Sampling clock*/ 
ChipSetField(pParams->hDemod,FSTV0913_STOP_DVBS1FEC,1);   /*Stop VITERBI 1 clock*/  
ChipSetField(pParams->hDemod,FSTV0913_STOP_DVBS2FEC,1);   /*Stop VITERBI 1 clock*/  

ChipSetField(pParams->hDemod,FSTV0913_STANDBY,1); /* general power OFF*/

/* Set tuner1 to standby On*/
FE_Sat_TunerSetStandby(pParams->hTuner, 1); 

}
else
{
ChipSetField(pParams->hDemod,FSTV0913_STANDBY,0); /* general power ON*/

ChipSetField(pParams->hDemod,FSTV0913_FSK_PON,1);   /*FSK power off*/ 
ChipSetField(pParams->hDemod,FSTV0913_ADC1_PON,1); /*ADC1 power off*/
ChipSetField(pParams->hDemod,FSTV0913_I2C_DISEQC_PON,1); /*DiseqC 1 power off*/

/*Stop Path1 Clocks*/
ChipSetField(pParams->hDemod,FSTV0913_STOP_DEMOD,0);   /*Stop Sampling clock*/ 
ChipSetField(pParams->hDemod,FSTV0913_STOP_DVBS1FEC,0);   /*Stop VITERBI 1 clock*/  
ChipSetField(pParams->hDemod,FSTV0913_STOP_DVBS2FEC,0);   /*Stop VITERBI 1 clock*/  

/* Set tuner1 to standby OFF*/
FE_Sat_TunerSetStandby(pParams->hTuner,0); 
}

FE_Sat_TunerSetStandby(pParams->hTuner, StandbyOn); /*Set the Tuner to Standby*/ 

if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) /*Check the error at the end of the function*/
error=FE_LLA_I2C_ERROR; 
}
}
else
error=FE_LLA_INVALID_HANDLE;

return(error);
}

/*****************************************************
--FUNCTION	::	FE_STV0913_SetAbortFlag
--ACTION	::	Set Abort flag On/Off  
--PARAMS IN	::	Handle	==>	Front End Handle

-PARAMS OUT::	NONE.
--RETURN	::	Error (if any)

--***************************************************/
FE_STV0913_Error_t FE_STV0913_SetAbortFlag(FE_STV0913_Handle_t Handle, BOOL Abort)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = (FE_STV0913_InternalParams_t	*)Handle;

if(pParams != NULL)
{
if ( (pParams->hDemod->Error ) || (pParams->hTuner->Error)) 
error=FE_LLA_I2C_ERROR; 
else
{
ChipAbort(pParams->hTuner,Abort);                                   
ChipAbort(pParams->hDemod,Abort);

if ((pParams->hDemod->Error )||(pParams->hTuner->Error)) /*Check the error at the end of the function*/ 
error=FE_LLA_I2C_ERROR; 
}
}
else
error=FE_LLA_INVALID_HANDLE;
return(error);
}


/*****************************************************
--FUNCTION	::	FE_STV0913_Search
--ACTION	::	Search for a valid transponder
--PARAMS IN	::	Handle	==>	Front End Handle
pSearch ==> Search parameters
pResult ==> Result of the search
--PARAMS OUT::	NONE
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t	FE_STV0913_Search(
FE_STV0913_Handle_t			Handle,
FE_STV0913_SearchParams_t	*pSearch,
FE_STV0913_SearchResult_t	*pResult,
U32					   		Satellite_Scan)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t *pParams =NULL;
FE_STV0913_SIGNALTYPE_t signalType=FE_SAT_NOCARRIER;


#ifdef HOST_PC 
#ifndef NO_GUI
double lockTime ;
char message[200];
lockTime = Timer();     /*SGA*/
#endif
#endif

if(Handle != NULL)
{
pParams = (FE_STV0913_InternalParams_t *) Handle; 

if ((pParams->hDemod->Error) || (pParams->hTuner->Error) )
error=FE_LLA_I2C_ERROR; 
else
{
if(	(INRANGE(100000,pSearch->SymbolRate,70000000)) &&
(INRANGE(500000,pSearch->SearchRange,70000000))
)
{
pParams->DemodSearchStandard= pSearch->Standard; 
pParams->DemodSymbolRate= pSearch->SymbolRate;
pParams->DemodSearchRange = pSearch->SearchRange;
pParams->TunerFrequency=pSearch->Frequency; 
pParams->DemodSearchAlgo=pSearch->SearchAlgo;
pParams->DemodSearch_IQ_Inv=pSearch->IQ_Inversion;
pParams->DemodPunctureRate=pSearch->PunctureRate;
pParams->DemodModulation=pSearch->Modulation;
pParams->DemodModcode=pSearch->Modcode; 
//printk("<913_drv.c>Calling Algo: Freq=%d \n",pParams->TunerFrequency);
signalType = FE_STV0913_Algo(pParams,Satellite_Scan) ;
#ifdef HOST_PC 
#ifndef NO_GUI
Fmt(message,"FE_STV0913_Algo: %f ms",(Timer()-lockTime)*1000);
ReportInsertMessage(message);
#endif
#endif
if (signalType==FE_SAT_TUNER_JUMP)
{			  
error=FE_LLA_TUNER_JUMP;
}

else if (signalType == FE_SAT_TUNER_NOSIGNAL)
{
/* half of the tuner bandwith jump */
error = FE_LLA_TUNER_NOSIGNAL ;
}
else if(((signalType == FE_SAT_RANGEOK) || ((Satellite_Scan > 0 ) && (signalType == FE_SAT_NODATA)))&& (pParams->hDemod->Error == CHIPERR_NO_ERROR))
{
if ((Satellite_Scan > 0 ) && (signalType == FE_SAT_NODATA))
{
error = FE_LLA_NODATA ;
#ifdef HOST_PC 
#ifndef NO_GUI
Fmt(message,"Nodata Detect");
ReportInsertMessage(message);
#endif
#endif
}
else
{
error = FE_LLA_NO_ERROR ;
}
pResult->Locked = pParams->DemodResults.Locked;

/* update results */
pResult->Standard=pParams->DemodResults.Standard;
pResult->Frequency = pParams->DemodResults.Frequency;			
pResult->SymbolRate = pParams->DemodResults.SymbolRate;										
pResult->PunctureRate = pParams->DemodResults.PunctureRate;
pResult->ModCode=pParams->DemodResults.ModCode;
pResult->Pilots=pParams->DemodResults.Pilots;
pResult->FrameLength=pParams->DemodResults.FrameLength;
pResult->Spectrum=pParams->DemodResults.Spectrum;
pResult->RollOff= pParams->DemodResults.RollOff;
pResult->Modulation=pParams->DemodResults.Modulation;

FE_STV0913_TrackingOptimization(pParams);		/* Optimization setting for tracking */
if((pResult->Pilots==0)&&((pResult->ModCode==FE_SAT_8PSK_23)||(pResult->ModCode==FE_SAT_8PSK_35))) /*8PSK 35 23 pilot off adaptation*/
{
FE_STV0913_8PSKCarrierAdaptation(pParams);
}
}
else
{
pResult->Locked = FALSE;

switch(pParams->DemodError)
{
case FE_LLA_I2C_ERROR:					/*I2C error*/	
error = FE_LLA_I2C_ERROR;
break;

case FE_LLA_NO_ERROR:
default:
error = FE_LLA_SEARCH_FAILED;
break;
}
}

if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) /*Check the error at the end of the function*/
error=FE_LLA_I2C_ERROR; 
}
else
error = FE_LLA_BAD_PARAMETER;
}
}
else
error=FE_LLA_INVALID_HANDLE;


#ifdef HOST_PC 
#ifndef NO_GUI
Fmt(message,"FE_LLA_xxx: %d, %f ms",error,(Timer()-lockTime)*1000);
ReportInsertMessage(message);
#endif
#endif
//Added HS
pSearch->TunerIndexJump = pParams->TunerIndexJump;

return error;


}

/*****************************************************
--FUNCTION	::	FE_STV0913_DVBS2_SetGoldCode
--ACTION	::	Set the DVBS2 Gold Code
--PARAMS IN	::	Handle	==>	Front End Handle 
U32		==>	cold code value (18bits)
--PARAMS OUT::	NONE
--RETURN	::	Error (if any )
--***************************************************/
FE_STV0913_Error_t FE_STV0913_DVBS2_SetGoldCodeX(FE_STV0913_Handle_t	Handle,U32 GoldCode)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t *pParams;


if((void *)Handle != NULL)
{
pParams = (FE_STV0913_InternalParams_t *) Handle; 

if(pParams != NULL)
{
if ((pParams->hDemod->Error) || (pParams->hTuner->Error) )
error=FE_LLA_I2C_ERROR; 
else
{
/*Gold code X setting*/
/* bit[3:4] of register PLROOT2
3..2 : P1_PLSCRAMB_MODE[1:0] |URW|
entry mode p1_plscramb_root
00: p1_plscramb_root is the root of PRBS X.
01: p1_plscramb_root is the DVBS2 gold code.
*/

ChipSetOneRegister(pParams->hDemod,RSTV0913_PLROOT2,0x04| ((GoldCode >>16) & 0x3) ); 
ChipSetOneRegister(pParams->hDemod,RSTV0913_PLROOT1,(GoldCode >>8) & 0xff ); 
ChipSetOneRegister(pParams->hDemod,RSTV0913_PLROOT0,(GoldCode) & 0xff );

if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) /*Check the error at the end of the function*/
error=FE_LLA_I2C_ERROR; 
}
}
else
error = FE_LLA_INVALID_HANDLE;
}
else
error = FE_LLA_INVALID_HANDLE; 

return error;
}


/*****************************************************
--FUNCTION	::	FE_STV0913_DiseqcTxInit
--ACTION	::	Set the diseqC Tx mode 
--PARAMS IN	::	Handle	==>	Front End Handle
::	DiseqCMode	==> diseqc Mode : continues tone, 2/3 PWM, 3/3 PWM ,2/3 envelop or 3/3 envelop.
--PARAMS OUT::None
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_DiseqcInit(	FE_STV0913_Handle_t Handle ,FE_STV0913_DiseqC_TxMode DiseqCMode)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;      
FE_STV0913_InternalParams_t	*pParams = NULL;

pParams = (FE_STV0913_InternalParams_t	*) Handle;
if(pParams != NULL)
{
if ((pParams->hDemod->Error) || (pParams->hTuner->Error) )
error=FE_LLA_I2C_ERROR; 
else
{
ChipSetField(pParams->hDemod,FSTV0913_DISEQC_MODE,DiseqCMode);

ChipSetField(pParams->hDemod,FSTV0913_DISTX_RESET,1);
ChipSetField(pParams->hDemod,FSTV0913_DISTX_RESET,0);

if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) /*Check the error at the end of the function*/
error=FE_LLA_I2C_ERROR; 
}
}
else
error=FE_LLA_INVALID_HANDLE; 


return error;
}


/*****************************************************
--FUNCTION	::	FE_STV0913_DiseqcSend
--ACTION	::	Read receved bytes from DiseqC FIFO 
--PARAMS IN	::	Handle	==>	Front End Handle
::	Data	==> Table of bytes to send.
::	NbData	==> Number of bytes to send.
--PARAMS OUT::None
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_DiseqcSend(	FE_STV0913_Handle_t Handle ,U8 *Data ,U32 NbData)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;      
FE_STV0913_InternalParams_t	*pParams = NULL;
S32 i=0; 

pParams = (FE_STV0913_InternalParams_t	*) Handle;
if(pParams != NULL)
{

if ( (pParams->hDemod->Error ) || (pParams->hTuner->Error)) 
error=FE_LLA_I2C_ERROR;
else
{
ChipSetField(pParams->hDemod,FSTV0913_DIS_PRECHARGE,1);
while(i<NbData)
{ 
while(ChipGetField(pParams->hDemod,FSTV0913_TX_FIFO_FULL));		/* wait for FIFO empty	*/

ChipSetOneRegister(pParams->hDemod,RSTV0913_DISTXFIFO,Data[i]);	/* send byte to FIFO :: WARNING don't use set field	!! */
i++;
}
ChipSetField(pParams->hDemod,FSTV0913_DIS_PRECHARGE,0);

i=0; 
while((ChipGetField( pParams->hDemod,FSTV0913_TX_IDLE)!=1) && (i<10))
{
/*wait until the end of diseqc send operation */
ChipWaitOrAbort(pParams->hDemod,10);
i++;     
}

if ( (pParams->hDemod->Error ) || (pParams->hTuner->Error)) /*Check the error at the end of the function*/ 
error=FE_LLA_I2C_ERROR;
}
}
else
error=FE_LLA_INVALID_HANDLE; 


return error;
}

/*****************************************************
--FUNCTION	::	FE_STV0913_DiseqcReceive
--ACTION	::	Read receved bytes from DiseqC FIFO 
--PARAMS IN	::	Handle	==>	Front End Handle
--PARAMS OUT::	Data	==> Table of received bytes.
::	NbData	==> Number of received bytes.
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_DiseqcReceive(	FE_STV0913_Handle_t Handle ,U8 *Data ,U32 *NbData)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;      
FE_STV0913_InternalParams_t	*pParams = NULL;
S32 i=0;

pParams = (FE_STV0913_InternalParams_t	*) Handle;
if(pParams != NULL)
{
if ( (pParams->hDemod->Error ) || (pParams->hTuner->Error)) 
error=FE_LLA_I2C_ERROR;
else
{
*NbData=0;
while((ChipGetField(pParams->hDemod,FSTV0913_RXEND)!=1)&&(i<10))
{
ChipWaitOrAbort(pParams->hDemod,10);
i++;
}
if(ChipGetField(pParams->hDemod,FSTV0913_RXEND))
{
*NbData=ChipGetField(pParams->hDemod,FSTV0913_RXFIFO_BYTES);
for(i=0;i<(*NbData);i++)
Data[i]=ChipGetOneRegister(pParams->hDemod,RSTV0913_DISRXFIFO);
}

if ( (pParams->hDemod->Error ) || (pParams->hTuner->Error)) /*Check the error at the end of the function*/ 
error=FE_LLA_I2C_ERROR;

}
}
else
error=FE_LLA_INVALID_HANDLE; 

return error;
}

/*****************************************************
--FUNCTION	::	FE_STV0913_Set22KHzContinues
--ACTION	::	Set the diseqC Tx mode 
--PARAMS IN	::	Handle	==>	Front End Handle
--PARAMS OUT::None
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_Set22KHzContinues(	FE_STV0913_Handle_t Handle ,BOOL Enable)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;      
FE_STV0913_InternalParams_t	*pParams = NULL;

pParams = (FE_STV0913_InternalParams_t	*) Handle;
if(pParams != NULL)
{
if ((pParams->hDemod->Error) || (pParams->hTuner->Error) )
error=FE_LLA_I2C_ERROR; 
else
{
if(Enable==TRUE)
{
}
else
{
}
if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) /*Check the error at the end of the function*/
error=FE_LLA_I2C_ERROR; 
}
}
else
error=FE_LLA_INVALID_HANDLE; 


return error;
}



/***********************************************************************
**FUNCTION	::	FE_STV0913_SetupFSK
**ACTION	::	Setup FSK
**PARAMS IN	::	hChip		      -> handle to the chip
**				fskt_carrier	  -> FSK modulator carrier frequency  (Hz)
**				fskt_deltaf       -> FSK frequency deviation		  (Hz)
**				fskr_carrier	  -> FSK demodulator carrier frequency	(Hz)
**PARAMS OUT::	NONE
**RETURN	::	Symbol frequency
***********************************************************************/
FE_STV0913_Error_t FE_STV0913_SetupFSK(FE_STV0913_Handle_t Handle,U32 TransmitCarrier,U32 ReceiveCarrier,U32 Deltaf)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t *pParams;
U32 fskt_carrier,fskt_deltaf,fskr_carrier;

if((void *)Handle != NULL)
{
pParams = (FE_STV0913_InternalParams_t *) Handle; 

if(pParams != NULL)
{
if ((pParams->hDemod->Error) || (pParams->hTuner->Error) )
error=FE_LLA_I2C_ERROR; 
else
{
ChipSetField(pParams->hDemod,FSTV0913_FSKT_KMOD,0x23);

/*
Formulat:
FSKT_CAR =  2^20*transmit_carrier/MasterClock;

*/
fskt_carrier= ( TransmitCarrier << 8 ) / (pParams->MasterClock >> 12); 

ChipSetField(pParams->hDemod,FSTV0913_FSKT_CAR2,((fskt_carrier >> 16) & 0x03));
ChipSetField(pParams->hDemod,FSTV0913_FSKT_CAR1,((fskt_carrier >> 8) & 0xFF));
ChipSetField(pParams->hDemod,FSTV0913_FSKT_CAR0,(fskt_carrier & 0xFF));


/*
Formulat:
FSKT_DELTAF =  2^20*fskt_deltaf/MasterClock;

*/

fskt_deltaf=(Deltaf<<20)/pParams->MasterClock;		 /* 2^20=1048576 */ 

ChipSetField(pParams->hDemod,FSTV0913_FSKT_DELTAF1,((fskt_deltaf >> 8) & 0x0F));
ChipSetField(pParams->hDemod,FSTV0913_FSKT_DELTAF0,(fskt_deltaf & 0xFF));				

ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKTCTRL,0x04);

ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRFC2,0x10);
/*
Formulat:
FSKR_CAR =  2^20*receive_carrier/MasterClock;

*/

fskr_carrier= ( ReceiveCarrier << 8 ) / (pParams->MasterClock >> 12);

ChipSetField(pParams->hDemod,FSTV0913_FSKR_CAR2,((fskr_carrier >> 16) & 0x03));
ChipSetField(pParams->hDemod,FSTV0913_FSKR_CAR1,((fskr_carrier >> 8) & 0xFF));
ChipSetField(pParams->hDemod,FSTV0913_FSKR_CAR0,(fskr_carrier & 0xFF));

ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRK1,0x53);
ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRK2,0x94);

ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRAGCR,0x28); 
ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRAGC,0x5F);

ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRALPHA,0x13);
ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRPLTH1,0x90);
ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRPLTH0,0x45);


ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRDF1,0x9f);
/*
Formulat:
FSKR_DELTAF =  2^20*fskt_deltaf/MasterClock;

*/



ChipSetField(pParams->hDemod,FSTV0913_FSKR_DELTAF1,((fskt_deltaf >> 8) & 0x0F));
ChipSetField(pParams->hDemod,FSTV0913_FSKR_DELTAF0,(fskt_deltaf & 0xFF));				


ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRSTEPP,0x02);
ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRSTEPM,0x4A);

ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRDET1,0);
ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRDET0,0x2F);

ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRDTH1,0);
ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRDTH0,0x55);

ChipSetOneRegister(pParams->hDemod,RSTV0913_FSKRLOSS,0x1F);

if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) /*Check the error at the end of the function*/
error=FE_LLA_I2C_ERROR; 
}
}
else
error = FE_LLA_INVALID_HANDLE;
}
else
error = FE_LLA_INVALID_HANDLE; 

return error;
}

/***********************************************************
**FUNCTION	::	FE_STV0913_EnableFSK
**ACTION	::	Enable - Disable FSK modulator /Demodulator
**PARAMS IN	::	hChip	 -> handle to the chip
**				mod_en	 -> FSK modulator on/off
**				demod_en -> FSK demodulator on/off	
**PARAMS OUT::	NONE
**RETURN	::	Error if any
***********************************************************/
FE_STV0913_Error_t FE_STV0913_EnableFSK(FE_STV0913_Handle_t Handle,BOOL EnableModulation,BOOL EnableDemodulation)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t *pParams;

if((void *)Handle != NULL)
{
pParams = (FE_STV0913_InternalParams_t *) Handle; 

if(pParams != NULL)
{
if ((pParams->hDemod->Error) || (pParams->hTuner->Error) )
error=FE_LLA_I2C_ERROR; 
else
{
if(EnableDemodulation==TRUE) 
ChipSetField(pParams->hDemod,FSTV0913_FSKT_MOD_EN,1);
else
ChipSetField(pParams->hDemod,FSTV0913_FSKT_MOD_EN,0);

if (EnableDemodulation==FALSE)
{
ChipSetField(pParams->hDemod,FSTV0913_FSK_PON,0x01);
//ChipSetField(pParams->hDemod,FSTV0913_SEL_FSK,0x00);
}
else
{
ChipSetField(pParams->hDemod,FSTV0913_FSK_PON,0x00);
//ChipSetField(pParams->hDemod,FSTV0913_SEL_FSK,0x01);
}

if ((pParams->hDemod->Error) || (pParams->hTuner->Error) ) /*Check the error at the end of the function*/
error=FE_LLA_I2C_ERROR; 

}
}
else
error = FE_LLA_INVALID_HANDLE;
}
else
error = FE_LLA_INVALID_HANDLE; 

return error;
}

/*****************************************************
--FUNCTION	::	FE_STV0913_Term
--ACTION	::	Terminate STV0913 chip connection
--PARAMS IN	::	Handle	==>	Front End Handle
--PARAMS OUT::	NONE
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t	FE_STV0913_Term(FE_STV0913_Handle_t	Handle)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = NULL;  

pParams = (FE_STV0913_InternalParams_t	*) Handle;

if(pParams != NULL)
{
#ifdef HOST_PC
#ifdef NO_GUI
ChipClose(pParams->hDemod);
FE_Sat_TunerTerm(pParams->hTuner);
#endif

if(Handle)
free(pParams);
#endif

}
else
error = FE_LLA_INVALID_HANDLE; 	

return error;
}

/*****************************************************
--FUNCTION	::	FE_STV0913_SetReg
--ACTION	::	write one register  
--PARAMS IN	::	Handle	==>	Front End Handle
::	Reg		==> register Index in the register Map
::	Val 	==> Val to be writen
--PARAMS OUT::	NONE.
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_SetReg(FE_STV0913_Handle_t Handle, U16 Reg, U32 Val)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = (FE_STV0913_InternalParams_t	*)Handle;

if(pParams == NULL)
error=FE_LLA_INVALID_HANDLE;
else
{
ChipSetOneRegister(pParams->hDemod,Reg,Val);
if (pParams->hDemod->Error)
error=FE_LLA_I2C_ERROR; 
}
return(error);
}

/*****************************************************
--FUNCTION	::	FE_STV0913_GetReg
--ACTION	::	write one register  
--PARAMS IN	::	Handle	==>	Front End Handle
::	Reg		==> register Index in the register Map
--PARAMS OUT::	Val		==> Read value.
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_GetReg(FE_STV0913_Handle_t Handle, U16 Reg, U32 *Val)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = (FE_STV0913_InternalParams_t	*)Handle;

if((pParams == NULL)||(Val==NULL))
error=FE_LLA_INVALID_HANDLE;
else
{
*Val=ChipGetOneRegister(pParams->hDemod,Reg);
if (pParams->hDemod->Error)
error=FE_LLA_I2C_ERROR; 
}
return(error);
}
/*****************************************************
--FUNCTION	::	FE_STV0913_SetField
--ACTION	::	write a value to a Field  
--PARAMS IN	::	Handle	==>	Front End Handle
::	Reg		==> Field Index in the register Map
::	Val 	==> Val to be writen
--PARAMS OUT::	NONE.
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_SetField(FE_STV0913_Handle_t Handle, U32 Field, S32 Val)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = (FE_STV0913_InternalParams_t	*)Handle;

if(pParams == NULL)
error=FE_LLA_INVALID_HANDLE;
else
{
ChipSetField(pParams->hDemod,Field,Val);
if (pParams->hDemod->Error)
error=FE_LLA_I2C_ERROR; 
}
return(error);
}

/*****************************************************
--FUNCTION	::	FE_STV0913_GetField
--ACTION	::	Read A Field  
--PARAMS IN	::	Handle	==>	Front End Handle
::	Field	==> Field Index in the register Map
--PARAMS OUT::	Val		==> Read value.
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_GetField(FE_STV0913_Handle_t Handle, U32 Field, S32 *Val)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = (FE_STV0913_InternalParams_t	*)Handle;

if((pParams == NULL)||(Val==NULL))
error=FE_LLA_INVALID_HANDLE;
else
{
*Val=ChipGetField(pParams->hDemod,Field);
if (pParams->hDemod->Error)
error=FE_LLA_I2C_ERROR; 
}
return(error);
}
FE_STV0913_Error_t FE_STV0913_SetContinous_Tone(FE_STV0913_Handle_t Handle,STFRONTEND_LNBToneState_t tonestate)
{
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
FE_STV0913_InternalParams_t	*pParams = NULL;
pParams = (FE_STV0913_InternalParams_t	*) Handle;

if(pParams != NULL){

if(tonestate!=STFRONTEND_LNB_TONE_OFF)
ChipSetField(pParams->hDemod,FSTV0913_DISEQC_MODE,0x0);
else
ChipSetField(pParams->hDemod,FSTV0913_DISEQC_MODE,0x2);	

}
return(error);
}

/*****************************************************
--FUNCTION	::	FE_STV0913_8PSKCarrierAdaptation
--ACTION	::	workaround for frequency shifter with high jitter and phase noise
--PARAMS IN	::	Handle	==>	Front End Handle

--PARAMS OUT::	NONE
--RETURN	::	Error (if any)
--***************************************************/
FE_STV0913_Error_t FE_STV0913_8PSKCarrierAdaptation(FE_STV0913_InternalParams_t *pParams)
{
/* 

Before acquisition:
Px_CARHDR=0x1C
Px_BCLC2S28=0x84

After FEC lock:

If  ((8PSK 3/5 or  2/3 Poff) && CNR<8dB)
Px_CARHDR=0x04
Px_BCLC2S28=0x31

*/
FE_STV0913_SignalInfo_t pInfo;
FE_STV0913_Error_t error = FE_LLA_NO_ERROR;
//FE_STV0913_InternalParams_t *pParams;	

//	pParams = (FE_STV0913_InternalParams_t *) Handle; 

if (pParams->hDemod->Error)
error=FE_LLA_I2C_ERROR; 
else
{
FE_STV0913_GetSignalInfoLite(pParams,&pInfo);

if(pInfo.C_N<80)
{
ChipSetOneRegister(pParams->hDemod,RSTV0913_CARHDR,0x04);
ChipSetOneRegister(pParams->hDemod,RSTV0913_BCLC2S28,0x31);
}

if (pParams->hDemod->Error )  
error=FE_LLA_I2C_ERROR; 
}

/*else
error = FE_LLA_INVALID_HANDLE;
*/
return error;
}



/******************************************/
