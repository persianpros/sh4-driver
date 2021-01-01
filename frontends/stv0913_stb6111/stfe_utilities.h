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
 @File   stfe_utilities.h
 @brief



*/
#ifndef H_STFE_UTILITIES
 #define H_STFE_UTILITIES

// #include "stddefs.h"
// #if defined (CHIP_STAPI)
   #include "stfrontend.h"
//   #include "chip.h"
//  #endif

typedef void *InternalParamsPtr;  /*Handle to the FE */

#define FE_LLA_MAXLOOKUPSIZE 500
/* One point of the lookup table */
typedef struct
{
	S32 realval; /* real value */
	S32 regval;  /* binary value */
} FE_LLA_LOOKPOINT_t;

/* Lookup table definition */
typedef struct
{
	S32 size;  /* Size of the lookup table */
	FE_LLA_LOOKPOINT_t table[FE_LLA_MAXLOOKUPSIZE];  /* Lookup table */
} FE_LLA_LOOKUP_t;

typedef enum
{
	FE_LLA_NO_ERROR = ST_NO_ERROR,
	FE_LLA_INVALID_HANDLE =ST_ERROR_INVALID_HANDLE ,
	FE_LLA_ALLOCATION =ST_ERROR_NO_MEMORY ,
	FE_LLA_BAD_PARAMETER = ST_ERROR_BAD_PARAMETER ,
	FE_LLA_ALREADY_INITIALIZED = ST_ERROR_BAD_PARAMETER,
	FE_LLA_I2C_ERROR = STFRONTEND_ERROR_I2C,
	FE_LLA_SEARCH_FAILED,
	FE_LLA_TRACKING_FAILED,
	FE_LLA_NODATA,
	FE_LLA_TUNER_NOSIGNAL,
	FE_LLA_TUNER_JUMP,
	FE_LLA_TUNER_4_STEP,
	FE_LLA_TUNER_8_STEP,
	FE_LLA_TUNER_16_STEP,
	FE_LLA_TERM_FAILED,
	FE_LLA_DISEQC_FAILED
} FE_LLA_Error_t;  /*Error Type*/

typedef enum
{
	TUNER_IQ_NORMAL = 1,
	TUNER_IQ_INVERT = -1
}  TUNER_IQ_t;

typedef enum
{
	/* tuner address                                  set by MT2068_Open() */
	Tuner_IC_ADDR,
	/* max number of MT2068 tuners     set by MT_TUNER_CNT in mt_userdef.h */
	Tuner_MAX_OPEN,
	/*  current number of open MT2068 tuners           set by MT2068_Open() */
	Tuner_NUM_OPEN,
	/*  crystal frequency                            (default: 16000000 Hz) */
	Tuner_SRO_FREQ,
	/*  min tuning step size                            (default: 50000 Hz) */
	Tuner_STEPSIZE,
	/*  input center frequency                         set by MT2068_Tune() */
	Tuner_INPUT_FREQ,
	/*  LO1 Frequency                                  set by MT2068_Tune() */
	Tuner_LO1_FREQ,
	/*  LO1 minimum step size                          (default: 250000 Hz) */
	Tuner_LO1_STEPSIZE,
	/*  LO1 FracN keep-out region                      (default: 999999 Hz) */
	Tuner_LO1_FRACN_AVOID,
	/*  Current 1st IF in use                          set by MT2068_Tune() */
	Tuner_IF1_ACTUAL,
	/*  Requested 1st IF                               set by MT2068_Tune() */
	Tuner_IF1_REQUEST,
	/*  Center of 1st IF SAW filter               (default: tuner-specific) */
	Tuner_IF1_CENTER,
	/*  Bandwidth of 1st IF SAW filter               (default: 20000000 Hz) */
	Tuner_IF1_BW,
	/*  zero-IF bandwidth                             (default: 2000000 Hz) */
	Tuner_ZIF_BW,
	/*  LO2 Frequency                                  set by MT2068_Tune() */
	Tuner_LO2_FREQ,
	/*  LO2 minimum step size                           (default: 50000 Hz) */
	Tuner_LO2_STEPSIZE,
	/*  LO2 FracN keep-out region                      (default: 374999 Hz) */
	Tuner_LO2_FRACN_AVOID,
	/*  output center frequency                        set by MT2068_Tune() */
	Tuner_OUTPUT_FREQ,
	/*  output bandwidth                               set by MT2068_Tune() */
	Tuner_OUTPUT_BW,
	/*  min inter-tuner LO separation                 (default: 1000000 Hz) */
	Tuner_LO_SEPARATION,
	/*  ID of avoid-spurs algorithm in use            compile-time constant */
	Tuner_AS_ALG,
	/*  max # of intra-tuner harmonics                       (default: 15)  */
	Tuner_MAX_HARM1,
	/*  max # of inter-tuner harmonics                        (default: 7)  */
	Tuner_MAX_HARM2,
	/*  # of 1st IF exclusion zones used               set by MT2068_Tune() */
	Tuner_EXCL_ZONES,
	/*  # of spurs found/avoided                       set by MT2068_Tune() */
	Tuner_NUM_SPURS,
	/*  >0 spurs avoided                               set by MT2068_Tune() */
	Tuner_SPUR_AVOIDED,
	/*  >0 spurs in output (mathematically)             set by Tuner_Tune() */
	Tuner_SPUR_PRESENT,
	/* GCU Mode: 1 - Automatic, 0- Manual                                   */
	Tuner_GCU_AUTO,
	/* Receiver Mode for some parameters. 1 is DVB-T                        */
	Tuner_RCVR_MODE,
	/* Set target power level at PD1.  Paremeter is value to set.           */
	Tuner_PD1_TGT,
	/* Get RF attenuator code                                               */
	Tuner_ACRF,
	/* Set target power level at PD2.  Paremeter is value to set.           */
	Tuner_PD2_TGT,
	/* Get FIF attenuator code                                              */
	Tuner_ACFIF,
	/*  CapTrim code                                                        */
	Tuner_CTRIM,
	/*  ClearTune filter number in use                 set by MT2068_Tune() */
	Tuner_CTFILT,
	/*  Flag for 1=automatic/0=manual RF AGC                                */
	Tuner_AUTOAGC,
	/*  Set the limit values for FIF Atten Code and RF Atten Code           */
	Tuner_FORCEAGC,
	/*  Flag to 1=freeze/0=run RF AGC                                       */
	Tuner_FREEZEAGC,
	/* Control setting to avoid DECT freqs      (default: MT_AVOID_US_DECT) */
	Tuner_AVOID_DECT,
	/* Set the VGA Gain Control value                (default: 0     40dB ) */
	Tuner_VGAGC,
	/*  last entry in enumerated list                                       */
	Tuner_EOP,
	/*  Dummy Entry                                                         */
	Tuner_STANDBY_WITH_VCO,
	Tuner_AGC_LOOP_BW,  /*for Mxl201 */
	Tuner_LOCK_STATUS   /*for Mxl201*/
} Tuner_Param_t;

typedef enum
{
	TUNER_LOCKED,
	TUNER_NOTLOCKED
} TUNER_Lock_t;

typedef enum
{
	TUNER_INPUT1 = 0,  /* input 1 (A,B for 6120) */
	TUNER_INPUT2 = 1,  /* input 2 (C,D for 6120) */
} TUNER_DualInput_t;	/*Dual Tuner (6120 like) input selection */

typedef enum
{
	FE_RF_SOURCE_A = 0,
	FE_RF_SOURCE_B = 1,
	FE_RF_SOURCE_C = 2,   /* No applicable for 6140 */
	FE_RF_SOURCE_D = 3    /* No applicable for 6140 */
} TUNER_RFSource_t ;

typedef enum
{
	LBRFA_HBRFB = 0,	/*Low band on RFA, High band on RFB*/
	LBRFB_HBRFA = 1,	/*Low band on RFB, High band on RFA*/
	LBRFC_HBRFD = 2,	/*Low band on RFC, High band on RFD*/
	LBRFD_HBRFC = 3		/*Low band on RFD, High band on RFC*/
} TUNER_WIDEBandS_t;		/* for wide band tuner 6130 and 6120 */

typedef enum			/* Path selection for dual tuner */
{
	TUNER_PATH1=0,		/* Path1 i.e tuner1  */
	TUNER_PATH2=1,		/* Path1 i.e tuner2  */
} TUNER_Dual_t;	/*Dual Tuner (6120 like) path selection */

typedef enum
{
	Band_VHF3 = 2,
	Band_UHF = 8
} TUNER_Band_t;


/* PJ/TA 12/2009: this enum must be in line with STCHIP_Error_t in chip.h */
typedef enum
{
	TUNER_NO_ERR = ST_NO_ERROR,
	TUNER_INVALID_HANDLE,
	TUNER_INVALID_REG_ID,     /* Using of an invalid register */
	TUNER_INVALID_FIELD_ID,   /* Using of an Invalid field */
	TUNER_INVALID_FIELD_SIZE, /* Using of a field with an invalid size */
	TUNER_I2C_NO_ACK,         /* No acknowledge from the chip */
	TUNER_I2C_BURST,          /* Two many registers accessed in burst mode */
	TUNER_TYPE_ERR
} TUNER_Error_t;

typedef enum
{
	FE_BER_Algo_Default = STFRONTEND_BER_ALGO_DEFAULT,
	FE_BER_Algo_Task_Berwindow =STFRONTEND_BER_ALGO_TASK_BERWINDOW,
	FE_BER_Algo_Appli_Berwindow = STFRONTEND_BER_ALGO_APPLI_BERWINDOW
} FE_Sat_BER_Algo_t;

typedef enum
{
	FE_FLAG_IF1_AGC1_USED = STFRONTEND_FLAG_IF1_AGC1_USED,  /* IF_OUT and AGC_IN are used for both digital and analog */
	FE_FLAG_IF2_AGC2_USED = STFRONTEND_FLAG_IF2_AGC2_USED,  /* IF_OUT and AGC_IN are used for both digital and analog */
	FE_FLAG_TUNER_XTAL_SHARED = STFRONTEND_FLAG_TUNER_XTAL_SHARED,  /* Tuner Xtal is shared with another tuner */
	FE_FLAG_TUNER_CLK_EXT_REF = STFRONTEND_FLAG_TUNER_CLK_EXT_REF, /* External reference clock is used i. e if a 4MHz Xtal is shared with demodulator */
	FE_FLAG_TUNER_CLK_XTAL_IN_OUT_ON = STFRONTEND_FLAG_TUNER_CLK_XTAL_IN_OUT_ON, /* XTAL is directly connected to the tuner; CLK_OUT is enabled */
	FE_FLAG_TUNER_CLK_IN_OUT_ON = STFRONTEND_FLAG_TUNER_CLK_IN_OUT_ON, /* Tuner is driven by CLK_IN and CLK_OUT is enabled */
	FE_FLAG_TUNER_CLK_IN_ON_OUT_OFF = STFRONTEND_FLAG_TUNER_CLK_IN_ON_OUT_OFF ,/* Tuner is driven by CLK_IN and CLK_OUT is disabled */
} FE_AdditionalParams_Flags_t;

/****************************************************************
					ST FE Demodulators List
 ****************************************************************/

typedef enum
{
	FE_DVB_T = 0,
	FE_DVB_T2,
	FE_DVB_C ,
	FE_DVB_C2,
	FE_DVB_S ,
	FE_DVB_S2,
	FE_ISDB_T,
	FE_ATSC,
	FE_J83B,
	FE_TransmissionStandard_Unknown = 0xFF
} FE_TransmissionStandard_t;

typedef enum
{
	DEMOD_STV0297E,
	DEMOD_STV0297D,
	DEMOD_STV0297J,
	DEMOD_STV0900,
	DEMOD_STV0903,
	DEMOD_STI7111,
	DEMOD_STIH205,
	DEMOD_STV0288,
	DEMOD_STV0289,
	DEMOD_STBSTC1,
	DEMOD_STB0899,
	DEMOD_STBADV1,
	DEMOD_STV0299,
	DEMOD_STV0399,
	DEMOD_STV0367DVBT,
	DEMOD_STV0367DVBC,
	DEMOD_STV0362,
	DEMOD_STV0360,
	DEMOD_STV0361,
	DEMOD_STV0372,
	DEMOD_STV0374ATSC,
	DEMOD_STV0374J83B,
	DEMOD_STV0373,
	DEMOD_STV0371ATSC,
	DEMOD_STV0371J83B,
	DEMOD_STV0370ATSC,
	DEMOD_STV0370J83B,
	DEMOD_STI7141,
	DEMOD_STI5197,
	DEMOD_STV0368DVBC,
	DEMOD_STV0368DVBS,
	DEMOD_STV0368DVBT,
	DEMOD_STV0368DVBT2,
	DEMOD_STID333,
	DEMOD_STV0910,
	DEMOD_STV0913,
	DEMOD_STV5189,
	DEMOD_STV5289,
	DEMOD_CUSTOM_DEMOD_01,
	DEMOD_ISDBT_02,
	DEMOD_SONY2820R
} FE_DEMOD_Model_t;

typedef enum
{
	TUNER_NULL ,		/* unknown tuner */
	TUNER_DTT73002,
	TUNER_TDQD3,
	TUNER_MT2266,
	TUNER_FQD1216ME,
	TUNER_DTT7546X,
	TUNER_STV4100,
	TUNER_EDT3022,
	TUNER_DTT73200A,
	TUNER_TDAE,
	TUNER_DNOS40,
	TUNER_DTT75300,
	TUNER_EN4020TER,
	TUNER_HC401Z,
	TUNER_FT2117,
	TUNER_DTOS403,
	TUNER_FT3114,
	TUNER_DNOS40AS,
	TUNER_DCT7045,
	TUNER_TDQE3,
	TUNER_DCF8783,
	TUNER_MT2060,
	TUNER_AIT1051,
	TUNER_DCT70700,
	TUNER_TDCHG,
	TUNER_TDCJG,
	TUNER_TDCGG,
	TUNER_TDCCG,
	TUNER_CD1616LF,
	TUNER_RF4800,
	TUNER_ET55DHR,
	TUNER_DCF87900,
	TUNER_DTT7546,
	TUNER_MT2062,
	TUNER_MT2011,
	TUNER_AIT1042,
	TUNER_EN4020,
	TUNER_MT2066,
	TUNER_MXL201RF,
	TUNER_MT2068,
	TUNER_DTT768XX,
	TUNER_DTT7600,
	TUNER_MXL203RF,
	TUNER_FQD1116,
	TUNER_TDA9898,
	TUNER_ENG37E03KF,
	TUNER_DCT7040,
	TUNER_CD1100,
	TUNER_STB6000,
	TUNER_STB6100,		/* RF magic STB6100 tuner */
	TUNER_STV6110,		/* 6110 tuner */
	TUNER_STV6111,		/* 6111 tuner */
	TUNER_STV6130,		/* 6130 tuner */
	UNER_STV6120,		/* 6120 tuner */
	TUNER_STV6120_Tuner1,		/* 6120 tuner */
	TUNER_STV6120_Tuner2,		/* 6120 tuner */
	TUNER_STV6140_Tuner1,		/* 6140 tuner */
	TUNER_STV6140_Tuner2,		/* 6140 tuner */
	TUNER_ASCOT2S,
	TUNER_ASCOT2D,
	TUNER_TDA18250,
	TUNER_MT2082,
	TUNER_TDAE3,
	TUNER_TDA18212,
	TUNER_TDA18260,
	TUNER_TDA18219,
	TUNER_FJ222,
	TUNER_CXD2831WR,
	TUNER_MXL603,
	TUNER_TDA18250A
} TUNER_Model_t;

/****************************************************************
      TS CONFIG STRUCTURE
 ****************************************************************/
 /*TS configuration enum*/

typedef enum
{
	FE_TS_OUTPUTMODE_DEFAULT = STFRONTEND_TS_MODE_DEFAULT,/*Use default register setting*/
	FE_TS_SERIAL_PUNCT_CLOCK = STFRONTEND_TS_SERIAL_PUNCT_CLOCK,  /*Serial punctured clock : serial STBackend  */
	FE_TS_SERIAL_CONT_CLOCK = STFRONTEND_TS_SERIAL_CONT_CLOCK,  /*Serial continues clock       */
	FE_TS_PARALLEL_PUNCT_CLOCK= STFRONTEND_TS_PARALLEL_PUNCT_CLOCK,  /*Parallel punctured clock : Parallel STBackend */
	FE_TS_DVBCI_CLOCK = STFRONTEND_TS_DVBCI_CLOCK/*Parallel continues clock : DVBCI    */
} FE_TS_OutputMode_t;

typedef enum
{
	FE_TS_DATARATECONTROL_DEFAULT = STFRONTEND_TS_DATARATECONTROL_DEFAULT,
	FE_TS_MANUAL_SPEED= STFRONTEND_TS_MANUAL_SPEED,    /* TS Speed manual */
	FE_TS_AUTO_SPEED = STFRONTEND_TS_AUTO_SPEED/* TS Speed automatic */
} FE_TS_DataRateControl_t;

typedef enum
{
	FE_TS_CLOCKPOLARITY_DEFAULT = STFRONTEND_TS_CLOCKPOLARITY_DEFAULT,
	FE_TS_RISINGEDGE_CLOCK= STFRONTEND_TS_RISINGEDGE_CLOCK,   /* TS clock rising edge  */
	FE_TS_FALLINGEDGE_CLOCK = STFRONTEND_TS_FALLINGEDGE_CLOCK/* TS clock falling edge */
} FE_TS_ClockPolarity_t;

typedef enum
{
	FE_TS_SYNCBYTE_DEFAULT = STFRONTEND_TS_SYNCBYTE_DEFAULT,
	FE_TS_SYNCBYTE_ON = STFRONTEND_TS_SYNCBYTE_ON,    /* TS synchro byte  ON    */
	FE_TS_SYNCBYTE_OFF  = STFRONTEND_TS_SYNCBYTE_OFF/* delete TS synchro byte */
} FE_TS_SyncByteEnable_t;


typedef enum
{
	FE_TS_PARTITYBYTES_DEFAULT = STFRONTEND_TS_PARTITYBYTES_DEFAULT,
	FE_TS_PARITYBYTES_ON = STFRONTEND_TS_PARITYBYTES_ON,   /* insert TS parity bytes */
	FE_TS_PARITYBYTES_OFF = STFRONTEND_TS_PARITYBYTES_OFF/* delete TS parity bytes */
} FE_TS_ParityBytes_t;

typedef enum
{
	FE_TS_SWAP_DEFAULT = STFRONTEND_TS_SWAP_DEFAULT,
	FE_TS_SWAP_ON = STFRONTEND_TS_SWAP_ON,     /* swap TS bits LSB <-> MSB */
	FE_TS_SWAP_OFF = STFRONTEND_TS_SWAP_OFF/* don't swap TS bits LSB <-> MSB */
} FE_TS_Swap_t;

typedef enum
{
	FE_TS_SMOOTHER_DEFAULT = STFRONTEND_TS_SMOOTHER_DEFAULT,
	FE_TS_SMOOTHER_ON = STFRONTEND_TS_SMOOTHER_ON,    /* enable TS smoother or fifo */
	FE_TS_SMOOTHER_OFF = STFRONTEND_TS_SMOOTHER_OFF    /* disable TS smoother or fifo */
} FE_TS_Smoother_t;

typedef enum
{
	FE_TS_OUTPUT1,    /*Demod selects TS1(demod's TS)*/
	FE_TS_OUTPUT2,    /*Demod selects TS2(demod's TS)*/
	FE_TS_OUTPUT3,    /*Demod selects TS3(demod's TS)*/
	FE_TS_MUX/*Demod selects MUX TS (demod's TS)*/
} FE_TS_SelectOutput_t;

typedef struct
{
	FE_TS_OutputMode_t   	TSMode;   		 /* TS Serial pucntured, serial continues, parallel punctured or DVBSI      */
	FE_TS_DataRateControl_t   TSSpeedControl;  /* TS speed control : manual or automatic   */
	FE_TS_ClockPolarity_t  	TSClockPolarity;  /* TS clock polarity : rising edge/falling edge */
	U32       				TSClockRate;  	/* TS clock rate in Hz if manual mode   */
	U32 					TSDataRate;		/* TS Data rate in Hz if manual mode			*/
	FE_TS_SyncByteEnable_t    TSSyncByteEnable; /* TS sync byte : enable/disabe     */
	FE_TS_ParityBytes_t   		TSParityBytes;    /* TS parity bytes Enable/Disable    */
	FE_TS_Swap_t    			TSSwap;    	      /* TS bits swap/ unswap       */
	FE_TS_Smoother_t   		TSSmoother;      /* TS smoother enable/disable     */
	FE_TS_SelectOutput_t		TSOutput;
} FE_TS_Config_t;

#define PowOf2(number) (1<<(number))				/* was: S32 PowOf2(S32 number); */
#define PowOf4(number) (1<<((number)<<1))			       /* was: S32 PowOf2(S32 number); */

U32 GetPowOf2(S32 number);
U32 BinaryFloatDiv(U32 n1, U32 n2, S32 precision);
S32 Get2Comp(S32 a, S32 width);
S32 XtoPowerY(S32 Number, U32 Power);
S32 RoundToNextHighestInteger(S32 Number,U32 Digits);
U32 Log2Int(S32 number);
S32 SqrtInt(S32 Sq);
//#ifdef ST_OSLINUX
unsigned int Log10Int(U32 value);
//#endif
U32 Gcdivisor(U32 m, U32 n);
#endif
/* End of stfe_utilities.h */
// vim:ts=4
