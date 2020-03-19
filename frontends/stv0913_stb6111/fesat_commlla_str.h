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
 @File   fesat_commlla_str.h
 @brief



*/
#ifndef FESAT_COMMLLA_STR_H
#define FESAT_COMMLLA_STR_H

/*************************************************************
       INCLUDES
*************************************************************/
//#include "stddefs.h"    /* Standard definitions */
//#include "fe_sat_tuner.h"

/****************************************************************
     COMMON STRUCTURES AND TYPEDEF
 ****************************************************************/
#define TUNER_BBGAIN(tun) (tun == ((TUNER_STB6000)||(TUNER_STB6100)) ? 8 : 6) 

typedef enum
{
	FE_SAT_SEARCH = 0,
	FE_SAT_PLH_DETECTED,
	FE_SAT_DVBS2_FOUND,
	FE_SAT_DVBS_FOUND
} FE_Sat_SEARCHSTATE_t;/*Current demod search state*/

typedef enum
{
	FE_SAT_NOAGC1 = 0,
	FE_SAT_AGC1OK,
	FE_SAT_NOTIMING,
	FE_SAT_ANALOGCARRIER,
	FE_SAT_TIMINGOK,
	FE_SAT_NOAGC2,
	FE_SAT_AGC2OK,
	FE_SAT_NOCARRIER,
	FE_SAT_CARRIEROK,
	FE_SAT_NODATA,
	FE_SAT_DATAOK,
	FE_SAT_OUTOFRANGE,
	FE_SAT_RANGEOK,
	FE_SAT_TUNER_NOSIGNAL,
	FE_SAT_TUNER_JUMP
} FE_Sat_SIGNALTYPE_t;

typedef enum
{
	FE_SAT_DEMOD_1,
	FE_SAT_DEMOD_2
} FE_Sat_DemodPath_t;      /*Current Demod*/

typedef enum
{
	FE_SAT_LNB_1,
	FE_SAT_LNB_2
} FE_Sat_LNBPath_t;      /*Current LNB*/

typedef enum
{
	FE_SAT_DEMOD_NUM_0,
	FE_SAT_DEMOD_NUM_1,
	FE_SAT_DEMOD_NUM_2,
	FE_SAT_DEMOD_NUM_3,
	FE_SAT_DEMOD_NUM_4,
	FE_SAT_DEMOD_NUM_5,
	FE_SAT_DEMOD_NUM_6,
	FE_SAT_DEMOD_NUM_7
} FE_Sat_DemodNumber_t;      /*Current Demod Number in use*/

typedef enum
{
	FE_SAT_SW_TUNER,   /* Tuner Control suported by user SW */
	FE_SAT_AUTO_STB6000,     /* Auto control of the tuner throw the demodulator */
	FE_SAT_AUTO_STB6100,  /* Auto control of the tuner throw the demodulator */
	FE_SAT_AUTO_STV6110,  /* Auto control of the tuner throw the demodulator */
	FE_SAT_AUTO_STV6111,  /* Auto control of the tuner throw the demodulator */
	FE_SAT_AUTO_STV6140_Tuner1,  /* Auto control of the tuner throw the demodulator */
	FE_SAT_AUTO_STV6140_Tuner2,  /* Auto control of the tuner throw the demodulator */
	FE_SAT_AUTO_STV6120 ,  /* Auto control of the tuner throw the demodulator */
	FE_SAT_AUTO_STV6120_Tuner1,   /* Auto control of the tuner throw the demodulator */
	FE_SAT_AUTO_STV6120_Tuner2   /* Auto control of the tuner throw the demodulator */
} FE_Sat_Tuner_t;

typedef enum
{
	FE_SAT_DVBS1_STANDARD,     /* Found Standard*/
	FE_SAT_DVBS2_STANDARD,
	FE_SAT_DSS_STANDARD,
	FE_SAT_TURBOCODE_STANDARD,
	FE_SAT_UNKNOWN_STANDARD
} FE_Sat_TrackingStandard_t;

typedef enum
{
	FE_SAT_AUTO_SEARCH,
	FE_SAT_SEARCH_DVBS1,    /* Search Standard*/
	FE_SAT_SEARCH_DVBS2,
	FE_SAT_SEARCH_DSS,
	FE_SAT_SEARCH_TURBOCODE
} FE_Sat_SearchStandard_t;

typedef enum
{
	FE_SAT_BLIND_SEARCH,     /* offset freq and SR are Unknown */
	FE_SAT_COLD_START,      /* only the SR is known */
	FE_SAT_WARM_START      /* offset freq and SR are known */
} FE_Sat_SearchAlgo_t;

typedef enum
{
	FE_SAT_MOD_UNKNOWN =0,
	FE_SAT_MOD_QPSK =1,
	FE_SAT_MOD_8PSK =2,
	FE_SAT_MOD_16APSK = 513,
	FE_SAT_MOD_32APSK   
} FE_Sat_Modulation_t; /* sat modulation type*/

#if 0
typedef enum
{
	FE_SAT_DUMMY_PLF,
	FE_SAT_QPSK_14,
	FE_SAT_QPSK_13,
	FE_SAT_QPSK_25,
	FE_SAT_QPSK_12,
	FE_SAT_QPSK_35,
	FE_SAT_QPSK_23,
	FE_SAT_QPSK_34,
	FE_SAT_QPSK_45,
	FE_SAT_QPSK_56,
	FE_SAT_QPSK_89,
	FE_SAT_QPSK_910,
	FE_SAT_8PSK_35,
	FE_SAT_8PSK_23,
	FE_SAT_8PSK_34,
	FE_SAT_8PSK_56,
	FE_SAT_8PSK_89,
	FE_SAT_8PSK_910,
	FE_SAT_16APSK_23,
	FE_SAT_16APSK_34,
	FE_SAT_16APSK_45,
	FE_SAT_16APSK_56,
	FE_SAT_16APSK_89,
	FE_SAT_16APSK_910,
	FE_SAT_32APSK_34,
	FE_SAT_32APSK_45,
	FE_SAT_32APSK_56,
	FE_SAT_32APSK_89,
	FE_SAT_32APSK_910,
	FE_SAT_MODCODE_UNKNOWN
} FE_Sat_ModCod_t;      /*ModCod for DVBS2*/
#endif

typedef enum   /*DVBS1, DSS and turbo code puncture rate*/
{
	FE_SAT_PR_1_2 =0,
	FE_SAT_PR_2_3,
	FE_SAT_PR_3_4,
	FE_SAT_PR_4_5,  /*for turbo code only*/
	FE_SAT_PR_5_6,
	FE_SAT_PR_6_7,  /*for DSS only   */
	FE_SAT_PR_7_8,
	FE_SAT_PR_8_9,  /*for turbo code only*/
	FE_SAT_PR_UNKNOWN
} FE_Sat_Rate_t;

typedef enum
{
	FE_SAT_LONG_FRAME,
	FE_SAT_SHORT_FRAME
} FE_Sat_FRAME_t;      /*Frame Type*/

typedef enum
{
	FE_SAT_PILOTS_OFF,
	FE_SAT_PILOTS_ON
} FE_Sat_PILOTS_t;

typedef enum
{
	FE_SAT_35,
	FE_SAT_25,
	FE_SAT_20
} FE_Sat_RollOff_t;

typedef enum
{
	FE_SAT_IQ_AUTO,
	FE_SAT_IQ_AUTO_NORMAL_FIRST,
	FE_SAT_IQ_FORCE_NORMAL,
	FE_SAT_IQ_FORCE_SWAPPED
} FE_Sat_Search_IQ_Inv_t;

typedef enum
{
	FE_SAT_IQ_NORMAL,
	FE_SAT_IQ_SWAPPED
} FE_Sat_IQ_Inversion_t;

/*DiseqC configuration*/
typedef enum
{
	FE_SAT_22KHZ_Continues = 0,
	FE_SAT_DISEQC_2_3_PWM = 2,
	FE_SAT_DISEQC_3_3_PWM = 3,
	FE_SAT_DISEQC_2_3_ENVELOP = 4,
	FE_SAT_DISEQC_3_3_ENVELOP = 5
} FE_Sat_DiseqC_TxMode;

typedef enum
{
	FE_SAT_SINGLE = 0,
	FE_SAT_DUAL
} FE_Sat_DemodType;

typedef enum
{
	FE_POL_HORIZONTAL = 1,
	FE_POL_VERTICAL  = 1<<1
} FE_Sat_Polarization_t;

typedef enum
{
	FE_PARITY_ON,
	FE_PARITY_OFF
} FE_Sat_DataParity_t;

typedef enum
{
	FE_PARALLEL_CLOCK,
	FE_PARALLEL_FIFO_CLOCK,
	FE_SERIAL_MASTER_CLOCK,
	FE_SERIAL_FIFO_CLOCK
} FE_Sat_Clock_t;

/****************************************************************
     INIT STRUCTURES
  structure passed to the FE_STVXXXX_Init() function
 ****************************************************************/

typedef struct
{
	char       				DemodName[20];   /* Demod name */
	U8       				DemodI2cAddr;   /* Demod I2c address */
	U32       				DemodRefClock;                            /* Refrence,Input clock for the demodulator in Hz (27MHz,30MHz,....)*/
	FE_Sat_DemodType  	DemodType;    /* Demodulator Type (single demod or dual demod) */
	FE_Sat_RollOff_t   	 	RollOff;    /* NYQUIST Filter value (used for DVBS1,DSS, DVBS2 is automatic)*/

	FE_Sat_Tuner_t    		TunerHWControlType;  /* Tuner1 HW controller Type */
	FE_TS_OutputMode_t     PathTSClock;      /* Path1 TS settings  */

	TUNER_Model_t   		TunerModel;    /* Tuner 1 model (SW control) */
	char       				TunerName[20];    /* Tuner 1 name */
	U8          				Tuner_I2cAddr;   /* Tuner 1 I2c address */
	U32       				TunerRefClock;   /* Tuner 1 Refrence,Input clock for the Tuner in Hz (27MHz,30MHz,16MHz....)*/
	S32       				TunerOutClkDivider;  /* Tuner 1 Clock Divider, for 0 to stop the tuner output clock, 1,2,4 or 8 */
	FE_Sat_IQ_Inversion_t  TunerIQ_Inversion;  /* Tuner 1 Global IQ setting (coonction of IQ from the tuner1 to the demod */
	S32       				TunerBasebandGain;  /* Tuner 1 base band gain */
	TUNER_WIDEBandS_t  	TunerBandSelect;  /* For wide band tuner (6130 and 6120 like)hardware band lowband/highband wiring */

	FE_Sat_Tuner_t    		Tuner2HWControlType; /* Tuner2 HW controller Type */
	FE_TS_OutputMode_t     Path2TSClock;      /* Path2 TS settings   */

	TUNER_Model_t   		Tuner2Model;   /* Tuner 2 model (SW control) */
	char       				Tuner2Name[20];    /* Tuner 2 name */
	U8       				Tuner2_I2cAddr;   /* Tuner 2 I2c address */
	U32       				Tuner2RefClock;   /* Tuner 2 Refrence,Input clock for the Tuner in Hz (27MHz,30MHz,16MHz....)*/
	S32       				Tuner2OutClkDivider; /* Tuner 2 Clock Divider, 0 to stop the tuner output clock, 1,2,4 or 8 */
	FE_Sat_IQ_Inversion_t  Tuner2IQ_Inversion;  /* Tuner 2 Global IQ setting (coonction of IQ from the tuner1 to the demod */
	S32       				Tuner2BasebandGain;  /* Tuner 2 base band gain */
	TUNER_WIDEBandS_t  	Tuner2BandSelect;  /* For wide band tuner (6130 and 6120 like)hardware band lowband/highband wiring */

#ifdef CHIP_STAPI
	ST_Partition_t     *MemoryPartition;
#endif
	FE_Sat_SearchStandard_t  	Standard;
	FE_Sat_Clock_t    			Clock;
	FE_Sat_DataParity_t   		Parity;
	TUNER_RFSource_t 		TunerRF_Source;
	TUNER_DualInput_t		TunerInputSelect;		/* dual tuner input select*/
	TUNER_RFSource_t 		Tuner2RF_Source;
	TUNER_DualInput_t		Tuner2InputSelect;		/* dual tuner input select*/
	U8 						Lna_agc_mode; /* Applicable only for 6111 tuner */
	U8 						Lna_agc_ref;  /* Applicable only for 6111 tuner */
	//U8 						Lna_agc_mode2; /* Applicable only for 6111 tuner */
	//U8 						Lna_agc_ref2;  /* Applicable only for 6111 tuner */
} FE_Sat_InitParams_t;

/****************************************************************
     SEARCH STRUCTURES
 ****************************************************************/

typedef struct
{
	FE_Sat_DemodPath_t   Path;      /* Path Used demod1 or 2          */
	U32      Frequency;     /* Transponder frequency (in KHz)        */
	U32      SymbolRate;     /* Transponder symbol rate  (in bds)       */
	U32      SearchRange;    /* Range of the search (in Hz)          */
	FE_Sat_SearchStandard_t Standard;     /* DVBS1,DVBS2, DSS or Turbo         */
	FE_Sat_Modulation_t  Modulation;     /* modulation type use FE_MOD_UNKNOWN for auto     */
	FE_Sat_Rate_t   PunctureRate;    /* DVBS1,DSS and turbo puncture rate use FE_PR_UNKNOWN for auto */
	FE_Sat_ModCod_t   Modcode;     /* DVBS2 modcode use FE_MODCODE_UNKNOWN for auto    */
	FE_Sat_Search_IQ_Inv_t IQ_Inversion;    /* IQ spectrum search               */
	FE_Sat_SearchAlgo_t  SearchAlgo;     /* Search Algorithm            */
	U32 Directblind;	 /* 	0: single blind search   1: edge detection   2: launch immediatly a blind without edge detection<91x device blindcsn optimizations>*/ 
	U32 Doublefail; /* pass to 1 if the first blind attempt is failed after a falling edge detected <91x device blindscan optimizations>*/ 
	S32 TunerIndexJump;/*contains the freq index where the rising or falling edge occurs*/
	FE_Sat_DemodNumber_t Demod_Number; /*the number of Demod used;presently for 8 Demod combination*/ 
} FE_Sat_SearchParams_t;

typedef struct
{
	BOOL       Locked;     /* Transponder found      */
	S32       Frequency;    /* Found Frequency       */
	U32       SymbolRate;    /* Found Symbol rate     */
	FE_Sat_TrackingStandard_t Standard;    /* Found Standard DVBS1,DVBS2 or DSS */
	FE_Sat_Rate_t     PunctureRate;   /* Found Puncture rate  For DVBS1   */
	FE_Sat_ModCod_t    ModCode;    /* Found Modcode only for DVBS2   */
	FE_Sat_Modulation_t   Modulation;    /* Found modulation type    */
	FE_Sat_PILOTS_t    Pilots;     /* pilots Found for DVBS2    */
	FE_Sat_FRAME_t    FrameLength;   /* Found frame length for DVBS2   */
	FE_Sat_IQ_Inversion_t  Spectrum;    /* IQ specrum swap setting    */
	FE_Sat_RollOff_t   RollOff;    /* Rolloff factor (0.2, 0.25 or 0.35) */
} FE_Sat_SearchResult_t;

/************************
 INFO STRUCTURE
 ************************/
typedef struct
{
	BOOL       Locked;     /* Transponder locked       */
	U32       Frequency;    /* Transponder frequency (in KHz)    */
	U32       SymbolRate;    /* Transponder symbol rate  (in Mbds)   */
	FE_Sat_TrackingStandard_t Standard;    /* Found Standard DVBS1,DVBS2 or DSS or Turbo */
	FE_Sat_Rate_t     PunctureRate;   /* Puncture rate          */
	FE_Sat_ModCod_t    ModCode;    /* MODCODE Only for DVBS2      */
	FE_Sat_Modulation_t   Modulation;    /* Modulation type        */
	FE_Sat_PILOTS_t    Pilots;     /* Pilots on,off only for DVB-S2    */
	FE_Sat_FRAME_t    FrameLength;   /* Found frame length only for DVB-S2   */
	FE_Sat_RollOff_t   RollOff;    /* Rolloff factor (0.2, 0.25 or 0.35)   */
	S32       Power;     /* Power of the RF signal (dBm)     */
	S32       C_N;     /* Carrier to noise ratio (dB x10)    */
	U32       BER;     /* Bit error rate (x10^7)      */
	FE_Sat_IQ_Inversion_t  Spectrum;    /* IQ specrum swap setting      */
} FE_Sat_SignalInfo_t;

typedef struct
{
	BOOL       Locked;  /* Transponder locked       */
	S32       Power;  /* Power of the RF signal (dBm)     */
	S32       C_N;  /* Carrier to noise ratio (dB x10)    */
	U32       BER;  /* Bit error rate (x10^7)      */
	S32 Frequency_IF;
} FE_Sat_TrackingInfo_t;

#endif
// vim:ts=4
