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
 @File   stfrontend.h
 @brief



*/
#ifndef __STFRONTEND_H
#define __STFRONTEND_H

/* Includes ---------------------------------------------------------------- */
#ifdef __cplusplus
extern "C"
{
#endif                          /* __cplusplus */

//#include "stddefs.h"    /* Standard definitions */
#include "stcommon.h"
//#include "stpio.h"
//#include "stos.h"
//#include "sttbx.h" /*For STTBX report and print functionalities*/

#ifndef ST_OSLINUX
/*For String functions such as strcmp*/
//#include <string.h>
#endif

/* Exported Types ---------------------------------------------------------- */
/* Exported Constants ------------------------------------------------------ */
/* STAPI Driver ID */
#define STFRONTEND_DRIVER_ID 6

/* STFRONTEND specific Error code base value */
#define STFRONTEND_DRIVER_BASE (STFRONTEND_DRIVER_ID << 16)

/* Maximum timeout (generic) */
#define STFRONTEND_MAX_TIMEOUT ((U32)-1)

/* number of (STEVT) events STFRONTEND posts per instance */
#define STFRONTEND_NUMBER_EVENTS 5

#ifdef STFRONTEND_NUM_MAX_HANDLES
#define STFRONTEND_MAX_HANDLES  STFRONTEND_NUM_MAX_HANDLES
#else
#define STFRONTEND_MAX_HANDLES 8
#endif

/* parameter passed to STFRONTEND_GetLLARevision() for retrieving LLA versions of all the demods in the current build */
#define STFRONTEND_ALL_IN_CURRENT_BUILD 0xFF

/* common */
#define STFRONTEND_DEMOD_BASE   10 /* to 50  */
#define STFRONTEND_TUNER_BASE   60 /* to 100 */

/* satellite */
#define STFRONTEND_LNB_BASE    210 /* to 250 */
#define STFRONTEND_SCR_BASE    300 /* to 350 */

#define STFRONTEND_BAND_START_C 3000000
#define STFRONTEND_BAND_END_C 4200000

#define STFRONTEND_BAND_START_KU_LOW 10600000
#define STFRONTEND_BAND_END_KU_HIGH 13400000

#define STFRONTEND_BAND_START_L 950000
#define STFRONTEND_BAND_END_L 2150000

#define STFRONTEND_DVBT2_L1_TUNE	0x20102010
#define STFRONTEND_DVBT2_FULL_TUNE	    0x21102111
/* top level handle */
 typedef U32 STFRONTEND_Handle_t; /* index into driver device allocation database (common) */


/* error return values (common) */
    typedef enum STFRONTEND_Error_e
    {
        STFRONTEND_ERROR_UNSPECIFIED = STFRONTEND_DRIVER_BASE,
        STFRONTEND_ERROR_HWFAIL,       /* hardware failure eg. LNB fail for a satellite device */
        STFRONTEND_ERROR_EVT_REGISTER, /* could not register tuner events with STEVT */
        STFRONTEND_ERROR_ID,           /* incorrect device/driver ID */
        STFRONTEND_ERROR_POINTER,      /* pointer was null or did not point to valid memory (RAM) */
        STFRONTEND_ERROR_DISEQC,  /*Diseqc error*/
        STFRONTEND_ERROR_I2C, /*Reports I2C Errors to the driver*/
	STFRONTEND_ERROR_REMOTE_NOT_READY  /*remote hw not ready to acknowledge*/
      } STFRONTEND_Error_t;

   /* event codes (common) */
    typedef enum STFRONTEND_EventType_e
    {
        STFRONTEND_EV_NO_OPERATION = STFRONTEND_DRIVER_BASE,  /* No current scan */
        STFRONTEND_EV_LOCKED,                              /* tuner locked    */
        STFRONTEND_EV_UNLOCKED,                            /* tuner lost lock */
        STFRONTEND_EV_SEARCH_FAILED,                         /* scan failed     */
        STFRONTEND_EV_TIMEOUT,                             /* scan timed out  */
        STFRONTEND_EV_HW_FAILURE    /* incase of LNB failure due to current
               overloading or over heating*/
    } STFRONTEND_EventType_t;


/* device category (common) */
    typedef enum STFRONTEND_Media_e
    {
        STFRONTEND_DEVICE_NONE      = 0,
        STFRONTEND_DEVICE_SATELLITE = 1,  /* Satellite Devices(DVB-S, DVB-S2)     */

        STFRONTEND_DEVICE_CABLE     = 2,  /* Cable Devices(DVB-C/ J83) */
        STFRONTEND_DEVICE_TERR      = 3,   /*   Terrestrial Devices (DVB-T) */
        STFRONTEND_DEVICE_TERR_VSB  =4     /*   Terrestrial Devices 8-VSB */
    } STFRONTEND_Media_t;
typedef STFRONTEND_Media_t STFRONTEND_Device_t;/*For backward compatibility*/

/* demod device-driver (common) */
#define MAX_DEMOD_NUM 40
    typedef enum STFRONTEND_DemodType_e
    {
          STFRONTEND_DEMOD_NONE = STFRONTEND_DEMOD_BASE, /* common: null demodulation driver use with e.g. STV0399 demod driver*/

          STFRONTEND_DEMOD_STV0900_Demod1,       /*Satellite     */
          STFRONTEND_DEMOD_STV0900_Demod2,       /*Satellite     */
          STFRONTEND_DEMOD_STX7111_Demod1,       /*Satellite     */
          STFRONTEND_DEMOD_STX7111_Demod2,       /*Satellite     */
          STFRONTEND_DEMOD_STXH205_Demod1,       /*Satellite     */
          STFRONTEND_DEMOD_STXH205_Demod2,       /*Satellite     */
          STFRONTEND_DEMOD_STV0289,   		/*Satellite     */
		  STFRONTEND_DEMOD_STV0910_Demod1,       /*Satellite     */
          STFRONTEND_DEMOD_STV0910_Demod2, 
	      STFRONTEND_DEMOD_STV0913,                       /*Satellite     */
          STFRONTEND_DEMOD_STV0903,                       /*Satellite     */
          STFRONTEND_DEMOD_STV0362,                   /* terrestrial-DVB-T */
          STFRONTEND_DEMOD_STI7167TER,                   /* terrestrial-DVB-T */
          STFRONTEND_DEMOD_SONY2820R,                   /* terrestrial-DVB-T,T2 */
          STFRONTEND_DEMOD_STV0297E,   /*Cable     */
          STFRONTEND_DEMOD_STI7141,   /*Cable     */
          STFRONTEND_DEMOD_STI5197,   /*Cable     */
          STFRONTEND_DEMOD_STV5189,   /*Satellite     */
          STFRONTEND_DEMOD_STV5289,   /*Satellite     */
          STFRONTEND_DEMOD_STI7167CAB,/* cable-DVB-C */
          STFRONTEND_DEMOD_STV0374QAM, /*cable */
          STFRONTEND_DEMOD_STV0297J,   /*Cable     */
	      STFRONTEND_DEMOD_SONY2820RCAB,/* cable -DVB-C */
          STFRONTEND_DEMOD_ISDBT_01,	/* ISDB-T */
          STFRONTEND_DEMOD_ISDBT_02,	/* ISDB-T */
          STFRONTEND_DEMOD_ISDBT_03,
          STFRONTEND_DEMOD_SAT_01_Demod0,   /* */
          STFRONTEND_DEMOD_SAT_01_Demod1,   /* */  
	  STFRONTEND_DEMOD_SAT_01_Demod2,   /* */
          STFRONTEND_DEMOD_SAT_01_Demod3,   /* */  
          STFRONTEND_DEMOD_SAT_01_Demod4,   /* */
          STFRONTEND_DEMOD_SAT_01_Demod5,   /* */  
	  STFRONTEND_DEMOD_SAT_01_Demod6,   /* */
          STFRONTEND_DEMOD_SAT_01_Demod7,   /* */  
		  STFRONTEND_DEMOD_REMOTE,
		  STFRONTEND_DEMOD_STV0368TER,
		  STFRONTEND_DEMOD_STV0368CAB,
          STFRONTEND_DEMOD_STV0373,                   /* terrestrial -VSB*/
          STFRONTEND_DEMOD_STV0374VSB=STFRONTEND_DEMOD_STV0373,  /* terrestrial -VSB*/
          STFRONTEND_DEMOD_CUSTOM_01,  /* Enum for Custom Demods - Customer/3rd party
                                                          demods , which are not part of stfrontend source code*/
          STFRONTEND_DEMOD_CUSTOM_02,  /* Enum for Custom Demods*/
          STFRONTEND_DEMOD_CUSTOM_03,  /* Enum for Custom Demods*/
          STFRONTEND_DEMOD_CUSTOM_04   /* Enum for Custom Demods*/
           

    } STFRONTEND_DemodType_t;

/* tuner device-driver (common) */
#define MAX_TUNER_NUM 100
    typedef enum STFRONTEND_TunerType_e
    {
       STFRONTEND_TUNER_NONE = STFRONTEND_TUNER_BASE, /* null tuner driver e.g. use with 399 which has no external tuner driver */
       STFRONTEND_TUNER_DTT7300X,    /* terrestrial : THOMSON */
       STFRONTEND_TUNER_DTT7546X,    /* terrestrial : THOMSON */
       STFRONTEND_TUNER_DTT7546=STFRONTEND_TUNER_DTT7546X,/* Cable : Tuner */
       STFRONTEND_TUNER_STV4100,    /* terrestrial */
       STFRONTEND_TUNER_MT2266,    /* terrestrial VSB: Microtune */
       STFRONTEND_TUNER_DTOS403,    /* terrestrial */
       STFRONTEND_TUNER_DTT73200A,    /* terrestrial */
       STFRONTEND_TUNER_DTT75300,    /* terrestrial */
       STFRONTEND_TUNER_EDT3022,    /* terrestrial */
       STFRONTEND_TUNER_FQD1216ME,    /* terrestrial */
       STFRONTEND_TUNER_HC401Z,    /* terrestrial */
       STFRONTEND_TUNER_TDAE,    /* terrestrial */
       STFRONTEND_TUNER_TDQD3,    /* terrestrial */
       STFRONTEND_TUNER_FT3114,    /* terrestrial */
       STFRONTEND_TUNER_MT2060,     /*  Tuner MICROTUNE cable/terrestrial */
       STFRONTEND_TUNER_MT2060_CAB=STFRONTEND_TUNER_MT2060,     /* Cable : Tuner MICROTUNE J83 A */
       STFRONTEND_TUNER_MT2060_ALTERNATEGPIO, /* Cable/terrestrial : Tuner Mictrotune MT2060 but with different GPIO config to be used with QAMi5107 board*/
       STFRONTEND_TUNER_MT2060_ALTERNATEGPIO_CAB=STFRONTEND_TUNER_MT2060_ALTERNATEGPIO, /* Cable : Tuner Mictrotune MT2060 but with different GPIO config to be used with QAMi5107 board*/
       STFRONTEND_TUNER_RF4800,     /* Cable : Tuner RFMAGIC */
       STFRONTEND_TUNER_DCT7040,    /* Cable : Tuner THOMSON J83 A */
       STFRONTEND_TUNER_DCT7045,    /* Cable : Tuner THOMSON */
       STFRONTEND_TUNER_TDQE3,      /* Cable : Tuner ALPS */
       STFRONTEND_TUNER_DCT7045EVAL ,/* Cable : Tuner THOMSON 7045 on Eval board*/
       STFRONTEND_TUNER_MT2011, /* Cable : Tuner MICROTUNE */
       STFRONTEND_TUNER_DCT70700,   /* Cable : Tuner THOMSON */
       STFRONTEND_TUNER_DCF87900, /* Cable : Tuner */
       STFRONTEND_TUNER_DCF8783, /* Cable : Tuner  */
       STFRONTEND_TUNER_AIT1051, /* Cable : Tuner */
       STFRONTEND_TUNER_DNOS40AS,/* Cable,terrestrial : Tuner */
       STFRONTEND_TUNER_DNOS40=STFRONTEND_TUNER_DNOS40AS,    /* terrestrial */
       STFRONTEND_TUNER_ET55DHR,/* Cable : Tuner */
       STFRONTEND_TUNER_TDCCG,/* Cable : Tuner */
       STFRONTEND_TUNER_TDCGG,/* Cable : Tuner */
       STFRONTEND_TUNER_TDCHG,/* Cable : Tuner */
       STFRONTEND_TUNER_TDCJG,/* Cable : Tuner */
       STFRONTEND_TUNER_CD1616LF, /* Cable : Tuner Philips */
       STFRONTEND_TUNER_MT2066, /* Cable : Microtune */
       STFRONTEND_TUNER_MXL201, /* Cable : Tuner */
       STFRONTEND_TUNER_AIT1042,/* Cable : Tuner */
       STFRONTEND_TUNER_EN4020,/* Cable : Tuner Entropic */
       STFRONTEND_TUNER_MT2068, /* Cable : Tuner MICROTUNE */
       STFRONTEND_TUNER_MT2062, /* Cable : Tuner MICROTUNE */
       STFRONTEND_TUNER_MXL203, /* Cable : Tuner */
       STFRONTEND_TUNER_TDA9898, /* Cable : Tuner */
       STFRONTEND_TUNER_FQD1116, /* terrestrial -Cable : Tuner */
       STFRONTEND_TUNER_CD1100, /* Cable : Tuner */
       STFRONTEND_TUNER_ENG37E03KF, /* TER/Cable : Tuner */
       STFRONTEND_TUNER_ASCOT2S, /* TER : Tuner */
       STFRONTEND_TUNER_ASCOT2D, /* TER : Tuner */
       STFRONTEND_TUNER_TDA18250,/* Cable : Tuner */
       STFRONTEND_TUNER_MT2082, /* Cable : Tuner MICROTUNE */
       STFRONTEND_TUNER_TDAE3,  /* Cable : Tuner */
       STFRONTEND_TUNER_TDA18212,/*cable +terr. tuner */
       STFRONTEND_TUNER_TDA18260,/*cable  tuner */
       STFRONTEND_TUNER_TDA18219,/*cable +terr. tuner */
       STFRONTEND_TUNER_FJ222,
	   STFRONTEND_TUNER_REMOTE,
	   STFRONTEND_TUNER_CXD2831WR, /*ISdbt tuner ascot2d*/
	   STFRONTEND_TUNER_MXL603,
   	   STFRONTEND_TUNER_TDA18250A,
       STFRONTEND_TUNER_STB6000,  /* satellite */
       STFRONTEND_TUNER_STV6110,  /* satellite */
       STFRONTEND_TUNER_STB6100,  /* satellite */
       STFRONTEND_TUNER_STV6130, /*satellite*/
       STFRONTEND_TUNER_STV6120, /*satellite*/
       STFRONTEND_TUNER_STV6120_Tuner1,/*satellite*/
       STFRONTEND_TUNER_STV6120_Tuner2,/*satellite*/
       STFRONTEND_TUNER_STV6140_Tuner1,/*satellite*/
       STFRONTEND_TUNER_STV6140_Tuner2,/*satellite*/
       STFRONTEND_TUNER_STV6111,  /* satellite */
       STFRONTEND_AUTO_STB6000,     /* Auto control of the tuner through the STV0900/903 */
       STFRONTEND_AUTO_STB6100,  /* Auto control of the tuner through the STV0900/903 */
       STFRONTEND_AUTO_STV6110,  /* Auto control of the tuner through the STV0900/903 */
       STFRONTEND_AUTO_STV6120,  /* Auto control of the tuner through the STV0900/903 */
       STFRONTEND_AUTO_STV6120_Tuner1,/* Auto control of the tuner through the Demod */
       STFRONTEND_AUTO_STV6120_Tuner2,/* Auto control of the tuner through the Demod */
       STFRONTEND_AUTO_STV6140_Tuner1,
       STFRONTEND_AUTO_STV6140_Tuner2,
       STFRONTEND_AUTO_STV6111,  /* Auto control of the tuner through the STV0900/903 */
       STFRONTEND_TUNER_DTT768XX,  /*cable/terrestrial QAM/VSB:THOMSON*/
       STFRONTEND_TUNER_CUSTOM_01,  /* Enum for Custom Tuners - Customer/3rd party
                                                          tuners , which are not part of stfrontend source code*/
       STFRONTEND_TUNER_CUSTOM_02,  /* Enum for Custom Tuners*/
       STFRONTEND_TUNER_CUSTOM_03,  /* Enum for Custom Tuners*/
       STFRONTEND_TUNER_CUSTOM_04,  /* Enum for Custom Tuners*/
       STFRONTEND_TUNER_CUSTOM_05,  /* Enum for Custom Tuners*/
       STFRONTEND_TUNER_CUSTOM_06,  /* Enum for Custom Tuners*/
       STFRONTEND_TUNER_CUSTOM_07,  /* Enum for Custom Tuners*/
       STFRONTEND_TUNER_CUSTOM_08,  /* Enum for Custom Tuners*/
       STFRONTEND_TUNER_CUSTOM_09,  /* Enum for Custom Tuners*/
       STFRONTEND_TUNER_CUSTOM_10   /* Enum for Custom Tuners*/
      
    } STFRONTEND_TunerType_t;

/* lnb device-driver (sat) */
#define MAX_LNB_NUM 40
typedef enum STFRONTEND_LnbType_e
    {
        STFRONTEND_LNB_NONE = STFRONTEND_LNB_BASE,    /* null LNB driver */
        STFRONTEND_LNB_LNBH21,   /* I2C controlled LNBPH21 -> used for tone & DiSEqC*/
        STFRONTEND_LNB_LNBH23,    /* I2C controlled LNBPH23 -> used for tone & DiSEqC*/
        STFRONTEND_LNB_LNBH24,     /* I2C controlled LNBPH24 -> used for tone & DiSEqC*/
        STFRONTEND_LNB_LNBH25,     /* I2C controlled LNBPH25 -> used for tone & DiSEqC*/
        STFRONTEND_LNB_LNBH26_Lnb1,     /* I2C controlled LNBPH26 -> used for tone & DiSEqC*/
        STFRONTEND_LNB_LNBH26_Lnb2,     /* I2C controlled LNBPH26 -> used for tone & DiSEqC*/
        STFRONTEND_LNB_DEMODIO,               /* To support LNB such as LNBP8/9 using Demod GPIO -> used for tone & DiSEqC*/
        STFRONTEND_LNB_BACKENDGPIO,        /* To control LNB through Backend GPIO -> used for tone & DiSEqC*/
        STFRONTEND_LNB_CUSTOM_01,  /*Enums for custom lnb- 3rd party lnb drivers,
                                                       not part of stfrontend code*/
        STFRONTEND_LNB_CUSTOM_02,  /*Enums for custom lnb*/
        STFRONTEND_LNB_CUSTOM_03, /*Enums for custom lnb*/
        STFRONTEND_LNB_CUSTOM_04 /*Enums for custom lnb*/
} STFRONTEND_LnbType_t;


#if 0
/* architecture specifics for I/O mode (common)*/
    typedef enum STFRONTEND_IORoute_e
    {
        STFRONTEND_IO_DIRECT,      /* open IO address for this instance of device, Used for Demod & Lnb*/
        STFRONTEND_IO_REPEATER    /* as above but send IO via Repeater of another device such as demod
               Used For Tuners*/
    } STFRONTEND_IORoute_t;
#endif

/* architecture specific VGLNA i2c connection to demod repeater */
    typedef enum STFRONTEND_Repeater_e
    {
        STFRONTEND_REPEATER_1,      /* VGLNA connection to the Repeater of Demod 1*/
        STFRONTEND_REPEATER_2    /* VGLNA connection to the Repeater of Demod 2*/
    } STFRONTEND_Repeater_t;

#if 0
/* select I/O target (common) */
    typedef enum STFRONTEND_IODriver_e
    {
        STFRONTEND_IODRV_NULL,     /* null driver, not used currently */
        STFRONTEND_IODRV_I2C,      /* use I2C drivers */
        STFRONTEND_IODRV_MEM      /* memory-mapped drivers */
    } STFRONTEND_IODriver_t;
#endif

/*For 6140 so that dynamically switching between 2RF sources is made possible*/
typedef enum  STFRONTEND_RFSource_e {
        STFRONTEND_RF_SOURCE_A = 0,
        STFRONTEND_RF_SOURCE_B = 1,
        STFRONTEND_RF_SOURCE_C = 2,   /* No applicable for 6140 */
        STFRONTEND_RF_SOURCE_D = 3    /* No applicable for 6140 */
}STFRONTEND_RFSource_t;


/* device I/O selection (common) */
    typedef struct STFRONTEND_IOParam_s
    {
        STFRONTEND_IORoute_t   Route;          /* where IO will go: Direct/Repeater*/
        STFRONTEND_IODriver_t  Driver;         /* which IO driver to use IO will go */
        U32          BaudRate;/*Changes speed during runtime, see STI2C API doc*/
        ST_DeviceName_t     DriverName;     /* STI2C driver name (for Init/Open) if applicable */
        U32                 Address;        /* I2C Address*/
    } STFRONTEND_IOParam_t;



/* StandByModes, will be replaced by new APIs  with STPOWER*/
   typedef enum STFRONTEND_StandByMode_e
   {
      STFRONTEND_NORMAL_POWER_MODE ,
      STFRONTEND_STANDBY_POWER_MODE
   } STFRONTEND_StandByMode_t;


#define MAX_SCR_NUM 8
/*****************************************************************
    IOCTL related enums & structures
*****************************************************************/

 /* any function index below STFRONTEND_IOCTL_BASE are
  reserved and should not be used */
 #define STFRONTEND_IOCTL_BASE 0x00010000

 /*ID for software function calls via ioctl (not direct to device drivers) */
 #define STFRONTEND_SOFT_ID  1

 typedef enum{
  /* Do direct register I/O using the I/O handle of the device-driver (type) selected */
   STFRONTEND_IOCTL_RAWIO = STFRONTEND_IOCTL_BASE,
  /* read a chip register (can be used without opening a tuner instance) */
   STFRONTEND_IOCTL_PROBE
 }STFRONTEND_IOCTL_FunctionID_t;
 /*Currently added support for IOCTL_PROBE only*/

 /* parameters for tuner ioctl call 'STFRONTEND_IOCTL_PROBE' */
typedef struct STFRONTEND_ProbeParams_s
{
   ST_DeviceName_t      DriverName;
   U32                  Address;
   U32                  SubAddress;
   U32                  Value;
   U32                  XferSize;
   U32                  TimeOut;
   U32                 BaudRate;
} STFRONTEND_ProbeParams_t;



typedef struct STFRONTEND_TunerProbeParams_s
{
  ST_DeviceName_t               DemodI2CBus;
  U32                           DemodAddress;
  U32                           DemodRepeaterAddress;/* currently not to be used in application*/
  U32                           TunerProbeAddress;
  U32                           XferSize;
  U32                           TimeOut;
  U32                           BaudRate;
  U8                            Value;
  U32                           TunerSubAddress;
  STFRONTEND_DemodType_t        DemodType;
}STFRONTEND_TunerProbeParams_t;

typedef enum
{
  STFRONTEND_CTL_PROBE_DEVICE,
  STFRONTEND_CTL_REGISTER_READ_WRITE,
  STFRONTEND_CTL_GET_REGISTER_DUMP,
  STFRONTEND_CTL_LNB_CONFIGURATION,
  STFRONTEND_CTL_SET_DATA_ROUTE,
  STFRONTEND_CTL_GET_DATA_ROUTE,
  STFRONTEND_CTL_SET_DISEQC_ENVELOPE_MODE,
  STFRONTEND_CTL_SELECT_TSOUT_PORT,
  STFRONTEND_CTL_8PSK_HIGH_SR_OPTIMISED,
  STFRONTEND_CTL_ADDON_VGLNA,
  STFRONTEND_CTL_SEL_BER_AlGO,
  STFRONTEND_CTL_GET_RF_LEVEL,       /*added to report rf level for isdbt Demod_02*/
  STFRONTEND_CTL_GET_T2_DATA_PLP,		/*added to report data and common PLPs for sony2820 DVBT2*/
  STFRONTEND_CTL_SET_NUMBER_LOST_BEFORE_UNLOCK,  /*For Unlock reporting try once if demod is LOCKED*/
  STFRONTEND_CTL_PID_FILTER, /*PID Filter by demod*/
  STFRONTEND_CTL_TS_MUX_SELECTION  /*To select the Muxed mode in  */
}STFRONTEND_Control_t;

typedef enum{
   STFRONTEND_BER_ALGO_DEFAULT ,    /* STFRONTEND_BER_ALGO_INTERNAL_BERWINDOW */
   STFRONTEND_BER_ALGO_TASK_BERWINDOW, /*Corresponds to legacy BER calc  */
   STFRONTEND_BER_ALGO_APPLI_BERWINDOW/*Corresponds to BER calc. based on API call STFxx_GetTunerInfo */
}STFRONTEND_BER_Algo_t;

typedef enum STFRONTEND_DeviceOperation_s
{
  STFRONTEND_DEVICE_READ,
   STFRONTEND_DEVICE_WRITE
} STFRONTEND_DeviceIOperation_t;

typedef enum STFRONTEND_LnbLlcState_t
{
 STFRONTEND_LNB_LLC_DISABLE,
 STFRONTEND_LNB_LLC_ENABLE
} STFRONTEND_LnbLlcState_t;

typedef struct STRONTEND_Register_s
{
 U32     SubAddress;
 U8       Value;
}STFRONTEND_Register_t;

typedef enum STFRONTEND_LNBShortCircuitProtectionMode_s
{
 STFRONTEND_LNB_PROTECTION_DYNAMIC=0,/* LNB output will toggle at short circuit*/
 STFRONTEND_LNB_PROTECTION_STATIC
} STFRONTEND_LNBShortCircuitProtectionMode_t;

typedef enum STFRONTEND_LnbControl_s
{
 STFRONTEND_LNB_LLC=0,
 STFRONTEND_LNB_SHORTCIRCUITPROTECTION,
 STFRONTEND_LNB_GET_STATUS
} STFRONTEND_LnbControl_t;

typedef struct STFRONTEND_DeviceAccessParams_s
{
   STFRONTEND_DeviceIOperation_t Operation;
   STFRONTEND_Register_t   *RegInfo ;
   U32    ArraySize;
   U32   Device;
} STFRONTEND_DeviceAccessParams_t;

typedef enum
{
STFRONTEND_TS_PORT0,
STFRONTEND_TS_PORT1,
STFRONTEND_TS_PORT2,
STFRONTEND_TS_PORT3,
STFRONTEND_TS_MUXTS,
STFRONTEND_TS_DEFAULT
} STFRONTEND_TSPort_t;

typedef enum
{
STFRONTEND_ADC_A,
STFRONTEND_ADC_B,
STFRONTEND_ADC_C,
STFRONTEND_ADC_DEFAULT
} STFRONTEND_ADC_t;

typedef struct STFRONTEND_DataRoute_s
{
STFRONTEND_ADC_t ADCIn;
STFRONTEND_TSPort_t TSOutPort;
} STFRONTEND_DataRoute_t;

typedef struct STRONTEND_Vglna_InitParams_s
{
STFRONTEND_IOParam_t VglnaIO;
STFRONTEND_Repeater_t Vglna_Repeater;
}STFRONTEND_Vglna_InitParams_t;
/* For DVB-T2 data PLP's */
typedef struct STRONTEND_GetT2_DataPLP_s
{
U8 		DataPLPid[256];	/* An array of atleast 256bytes in length */
U8		NumDataPLP;		/* The number of data PLPs detected(signalled in L1-post) */
U8		CmmnPLPid[200];	/* An array of atleast 256bytes in length */
U8		NumCmnPLP;		/* The number of data PLPs detected(signalled in L1-post) */
}STRONTEND_GetT2_DataPLP_t;

/*****************************************************************
 TS formatting related structures & enums
*****************************************************************/
/* transport stream output mode (common) */
typedef enum
 {
  STFRONTEND_TS_MODE_DEFAULT,   /*No Action/demod uses default register settings*/
  STFRONTEND_TS_SERIAL_PUNCT_CLOCK,  /*Serial punctured clock : serial STBackend  */
  STFRONTEND_TS_SERIAL_CONT_CLOCK,  /*Serial continues clock       */
  STFRONTEND_TS_PARALLEL_PUNCT_CLOCK,  /*Parallel punctured clock : Parallel STBackend */
  STFRONTEND_TS_DVBCI_CLOCK    /*Parallel continues clock : DVBCI    */
 } STFRONTEND_TS_OutputMode_t;

/*TS SPEED/data rate*/ /*DataRate Control Supported in 900/903*/
 typedef enum
 {
  STFRONTEND_TS_DATARATECONTROL_DEFAULT,/*No Action/demod uses default register settings*/
  STFRONTEND_TS_MANUAL_SPEED,    /* TS Speed manual */
  STFRONTEND_TS_AUTO_SPEED    /* TS Speed automatic */
 } STFRONTEND_TS_DataRateControl_t;

/*TS Clock Polarity*/
 typedef enum
 {
  STFRONTEND_TS_CLOCKPOLARITY_DEFAULT,/*No Action/demod uses default register settings*/
  STFRONTEND_TS_RISINGEDGE_CLOCK,   /* TS clock rising edge  */
  STFRONTEND_TS_FALLINGEDGE_CLOCK   /* TS clock falling edge */
 } STFRONTEND_TS_ClockPolarity_t;

 /*TS Sync byte enable/disable*/
 typedef enum
 {
  STFRONTEND_TS_SYNCBYTE_DEFAULT,/*No Action/demod uses default register settings*/
  STFRONTEND_TS_SYNCBYTE_ON,    /* sync bytes are not stripped */
  STFRONTEND_TS_SYNCBYTE_OFF    /* delete TS sync byte */
 } STFRONTEND_TS_SyncByteEnable_t;


 typedef enum
 {
  STFRONTEND_TS_PARTITYBYTES_DEFAULT,/*No Action/demod uses default register settings*/
  STFRONTEND_TS_PARITYBYTES_ON,   /* All MPEG/TS bits are output*/
  STFRONTEND_TS_PARITYBYTES_OFF   /*delete TS parity bytes */
 } STFRONTEND_TS_ParityBytes_t;

/*Supported in 297E/7141/5197  only*/
 typedef enum
 {
  STFRONTEND_TS_SWAP_DEFAULT,/*No Action/demod uses default register settings*/
  STFRONTEND_TS_SWAP_ON,     /* swap TS bits LSB <-> MSB */
  STFRONTEND_TS_SWAP_OFF     /* don't swap TS bits LSB <-> MSB */
 } STFRONTEND_TS_Swap_t;

 /*Supported in 297E/7141/5197 only*/
 typedef enum
 {
  STFRONTEND_TS_SMOOTHER_DEFAULT,/*No Action/demod uses default register settings*/
  STFRONTEND_TS_SMOOTHER_ON,    /* enable TS smoother or fifo for serial ouput only*/
  STFRONTEND_TS_SMOOTHER_OFF    /* disable TS smoother or fifo for serial output only*/
 } STFRONTEND_TS_Smoother_t;


/* enumerated configuration get/set ---------------------------------------- */

/*  note: all devices do not support all configuration options in the types defined below.
Moreover,   note that the values assigned are not to be taken as a direct mapping of the low level
hardware registers (individual device drivers may translate these values) */

/* type of modulation (common) */
    typedef enum STFRONTEND_Modulation_e
    {
        STFRONTEND_MOD_NONE   = 0x00,  /* Modulation unknown */
        STFRONTEND_MOD_ALL    = 0x3FF, /* Logical OR of all MODs */
        STFRONTEND_MOD_QPSK   = 1,
        STFRONTEND_MOD_8PSK   = (1 << 1),
        STFRONTEND_MOD_QAM    = (1 << 2),
        STFRONTEND_MOD_4QAM   = (1 << 3),
        STFRONTEND_MOD_16QAM  = (1 << 4),
        STFRONTEND_MOD_32QAM  = (1 << 5),
        STFRONTEND_MOD_64QAM  = (1 << 6),
        STFRONTEND_MOD_128QAM = (1 << 7),
        STFRONTEND_MOD_256QAM = (1 << 8),
        STFRONTEND_MOD_BPSK   = (1 << 9),
        STFRONTEND_MOD_16APSK,
        STFRONTEND_MOD_32APSK,
        STFRONTEND_MOD_8VSB  =  (1 << 10),
        STFRONTEND_MOD_DQPSK
    } STFRONTEND_Modulation_t;

/*cab, terr & satellite*/
    typedef enum STFRONTEND_Spectrum_e
 {
        STFRONTEND_INVERSION_NONE = 0, /*IQ Spectrum normal*/
        STFRONTEND_INVERSION      = 1,  /*IQ Spectrum Inverted*/
        STFRONTEND_INVERSION_AUTO = 2, /*Automatic search*/
        STFRONTEND_INVERSION_UNK  = 4 /*DVB-T only*/
    } STFRONTEND_Spectrum_t;

/*IQ wiring :formerly STFRONTEND_IQMode_t*/
typedef enum STFRONTEND_IQWiring_e
{
        STFRONTEND_IQWiring_NORMAL       = 0,/* indicates NORMAL wiring connection between tuner and demod IQ*/
        STFRONTEND_IQWiring_SWAPPED     = 1  /* indicates INVERTED wiring connection between tuner and demod IQ*/
} STFRONTEND_IQWiring_t;

/* tuner status (common) */
    typedef enum STFRONTEND_Status_e
    {
        STFRONTEND_STATUS_IDLE,    /* driver state is IDLE, Ready for Acquisition*/
        STFRONTEND_STATUS_ACQUISITION,   /*Acquisition is in progress*/
        STFRONTEND_STATUS_LOCKED,   /*Device locked to frequency and driver is in tracking mode*/
        STFRONTEND_STATUS_UNLOCKED,  /*Device has lost lock*/
        STFRONTEND_STATUS_STANDBY   /*Low power mode*/
    } STFRONTEND_Status_t;

/*to allow/disable the pids */
typedef struct
{ 
U16 originalPid;         // pid from TS
  U16 remappedPid;         // remapped pid
  BOOL enable;          // enable or disable pid
  BOOL allowOrDrop;     // allow or drop pid
  BOOL enablePidRemap;  // enable or disable pid remap
} STFRONTEND_Pid_t;

/*to allow/disable All pids at once */
  typedef enum
  {
    STFRONTEND_PIDS_ALLOW_ALL = 0,     // Allow all pids
    STFRONTEND_PIDS_DROP_ALL,          // Drop all pids
    STFRONTEND_INVALIDATE_PID_FILTER,   // Invalidate PID filter table
    STFRONTEND_PIDS_DEFAULT
  } STFRONTEND_Pid_Flt_Ctrl_t;

typedef struct
{
 STFRONTEND_Pid_t pid_ts[6];
  U8 numpids;            //Nr of pids enabled or disabled
  U8 Demod_num;
  STFRONTEND_Pid_Flt_Ctrl_t Flt_Ctrl;
}STFRONTEND_TS_Pid_Filter_t;


typedef enum
{
  STFRONTEND_TS_MUX_DISABLE = 0,  // No Mux ( 1 TSIF to 1 TSIF)
  STFRONTEND_TS_MUX_2_TO_1,       // Mux 2 TSIF to 1 TSIF
  STFRONTEND_TS_MUX_4_TO_1,       // Mux 4 TSIF to 1 TSIF
} STFRONTEND_TS_MUX_TYPE_t;

typedef enum
{
  STFRONTEND_TS_GROUP_0_3 = 0,     // TS group 0 to 3 (TS0, TS1, TS2 & TS3)
  STFRONTEND_TS_GROUP_4_7,         // TS group 0 to 3 (TS4, TS5, TS6 & TS7)
} STFRONTEND_TS_GROUP_t;


typedef struct STFRONTEND_TS_MUX_Setting_e
{
   STFRONTEND_TS_GROUP_t       Mux_Group;
   STFRONTEND_TS_MUX_TYPE_t  Mux_Type;
    
}STFRONTEND_TS_MUX_Setting_t;


/****************************************************************


     Cable Structures

****************************************************************/
/*For reporting of packet counters, corrected block, uncorrected block*/
  typedef struct STFRONTEND_PacketCounter_e
  {
  	U32  TotalBlocks;
  	U32  TotalBlocksOffset;
  	U32  TotalCB;
  	U32  TotalCBOffset;
  	U32  TotalUCB;
  	U32  TotalUCBOffset;
  }
  STFRONTEND_PacketCounter_t;

/****************************************************************

     Satellite Structures

****************************************************************/

/* downlink frequency bands (sat) */
    typedef enum STFRONTEND_DownLink_e
    {
       STFRONTEND_DOWNLINK_Ku = 1,   /* Used for Ku Band : both high and low */
       STFRONTEND_DOWNLINK_C    ,     /* Used for C Band  */
       STFRONTEND_DOWNLINK_L	      /*Used for L band*/
    }
    STFRONTEND_DownLink_t;

	typedef  enum STFRONTEND_ScrLNB_Index_e
	{
	STFRONTEND_LNB0, /* LNB index 0 for RF Input*/
	STFRONTEND_LNB1, /* LNB index 1 for RF Input*/
	STFRONTEND_LNB2, /* LNB index 2 for RF Input*/
	STFRONTEND_LNB3  /* LNB index 3 for RF Input*/
	}
	STFRONTEND_ScrLNB_Index_t;
	typedef enum STFRONTEND_ScrBPF_e
	{
	STFRONTEND_ScrBPF0,  /* output BPF index 0*/
	STFRONTEND_ScrBPF1,  /* output BPF index 1*/
	STFRONTEND_ScrBPF2,  /* output BPF index 2*/
	STFRONTEND_ScrBPF3   /* output BPF index 3*/
	}
	STFRONTEND_ScrBPF_t ;

	typedef enum STFRONTEND_Scr_Mode_e
	{
  	STFRONTEND_Scr_MANUAL, /* user will set all lnb parameters (LO, LPF)*/
  	STFRONTEND_Scr_AUTO /* a search tone will detect free Scr and set LNB parameters*/

	}
	STFRONTEND_Scr_Mode_t;

	typedef enum STFRONTEND_ODUType_e
	{
	STFRONTEND_ODU_NONE =STFRONTEND_SCR_BASE,
  	STFRONTEND_SATCR, /* user will set  if wants to use SATCR*/
  	STFRONTEND_DISEQC_TO_FSK_DONGLE   /* If the user selects the MDU support*/
	 }
	STFRONTEND_ODUType_t;

	typedef  enum STFRONTEND_Scr_AppType_e
	{
	 /* Put all the 7 applications supported by ST7LNB1*/
 	STFRONTEND_Scr_APPLICATION_0,
 	STFRONTEND_Scr_APPLICATION_1,
 	STFRONTEND_Scr_APPLICATION_2,
 	STFRONTEND_Scr_APPLICATION_3,
 	STFRONTEND_Scr_APPLICATION_4,
 	STFRONTEND_Scr_APPLICATION_5,
 	STFRONTEND_Scr_APPLICATION_6,
 	STFRONTEND_Scr_APPLICATION_7,
 	STFRONTEND_Scr_APPLICATION_NOT_SUPPORTED
	}
	STFRONTEND_Scr_AppType_t ;

     typedef enum STFRONTEND_SAT_Tone_BE_or_FE_e
	 {
	  STFRONTEND_TONE_THRU_BACKEND=0, /*determines LNB type-> lnbh24 /lnbh21 /lnb_none etc*/
	  STFRONTEND_TONE_THRU_FRONTEND
	 }STFRONTEND_SAT_Tone_BE_or_FE_t;
/*For Backend LNB via GPIOs controll*/

  typedef struct STFRONTEND_LNB_Via_PIO_s
 {
        ST_DeviceName_t PIODeviceName;
        U8 VSEL_PIOPort; /* PIO port number for Voltage selection(Ref : Backend GPIO) */
        U8 VSEL_PIOPin; /* PIO Pin number for Voltage selection(Ref : Backend GPIO) */
        U8 VSEL_PIOBit; /* PIO Bit number for Voltage selection(Ref : Backend GPIO) */
        U8 VEN_PIOPort; /* PIO port number for Voltage Enable(Ref : Backend GPIO) */
        U8 VEN_PIOPin; /* PIO Pin number for Voltage Enable(Ref : Backend GPIO)*/
        U8 VEN_PIOBit; /* PIO bit number for Voltage Enable(Ref : Backend GPIO) */
        U8 TEN_PIOPort; /* PIO port number for Tone Enable(Ref : Backend GPIO)*/
        U8 TEN_PIOPin; /* PIO Pin number for Tone Enable(Ref : Backend GPIO)*/
        U8 TEN_PIOBit; /* PIO bit number for Tone Enable(Ref : Backend GPIO)*/
       STFRONTEND_SAT_Tone_BE_or_FE_t  Select_Tone_Thru_BE_or_FE;/*This f;ag will help in determining if BE or FE PIO is used  0->BE;1->FE.*/
 }
STFRONTEND_LNB_Via_PIO_t;



 typedef struct STFRONTEND_SAT_LNBParams_s
 {
  STFRONTEND_LnbType_t                LnbType; /*determines LNB type-> lnbh24 /lnbh21 /lnb_none etc*/
  STFRONTEND_IOParam_t                LnbIO;    /*I/O Parameters*/
  STFRONTEND_LNB_Via_PIO_t       LnbIOPort;  /*used for backend GPIO*/
 }STFRONTEND_SAT_LNBParams_t;

  typedef struct STFRONTEND_Sat_ScrParams_s
 {

  STFRONTEND_Scr_AppType_t           Scr_App_Type;    /*Scr type*/
  STFRONTEND_Scr_Mode_t          Scr_Mode;  /* Scr Installation -> Manual or Auto Mode */
  STFRONTEND_ScrBPF_t            ScrBPF ;    /* Scr indexing***/
  U8 NbLnb;
  U16   ScrLNB_LO_Frequencies[8]; /* In MHz*/
  U8 NbScr;
  U16   ScrBPFFrequencies[12];  /* In MHz*/
  STFRONTEND_ODUType_t	ODUType; /* For selecting the SATCR or MDU support */
 }STFRONTEND_Sat_ScrParams_t;

typedef struct STFRONTEND_SCRSearchParams_s
 {

  BOOL                          	ODU_Enable;    /*fsu addition*/
  STFRONTEND_ScrLNB_Index_t    LNBIndex ;   /* From which LNB to get RF signal*/
  STFRONTEND_ScrBPF_t            ScrBPF ;    /* Scr indexing***/

 }STFRONTEND_SCRSearchParams_t;
  /*Satellite Specific Device Parameters*/
    typedef struct STFRONTEND_SatDevice_InitParams_s
    {
     STFRONTEND_SAT_LNBParams_t          LNBParams;

     STFRONTEND_LNB_Via_PIO_t 			PIOParams;

 #ifdef STFRONTEND_DRV_SAT_DISEQC_THROUGH_BACKEND
      STFRONTEND_SAT_DiSeqCParams_t      DiSeqCParams;
     #endif
      STFRONTEND_Sat_ScrParams_t             ScrParams;

    }
    STFRONTEND_SatDevice_InitParams_t;

/* LNB tone state (sat) */
typedef enum STFRONTEND_LNBToneState_e
{
    STFRONTEND_LNB_TONE_DEFAULT,   /* Default (current) LNB state */
    STFRONTEND_LNB_TONE_OFF,       /* LNB disabled */
    STFRONTEND_LNB_TONE_22KHZ,      /* LNB set to 22kHz */
    STFRONTEND_LNB_TONE_AUTO      /* Applicable for Ku band STFRONTEND_Scan() only to switch LO at KU_SWITCH */
} STFRONTEND_LNBToneState_t;

/*Configure LNB for Diseqc Recieve (Rx)/ Diseqc Transmission(Tx)*/
typedef enum STFRONTEND_LnbTTxMode_e
{
   STFRONTEND_LNB_RX = 0, /*receiving mode: should be used in Diseqc*/
   STFRONTEND_LNB_TX,      /*transmitting mode: should be used in Diseqc*/
   STFRONTEND_LNB_DEFAULT /*This mode is for LNBH26/25 Only*/
}STFRONTEND_LnbTTxMode_t;


typedef enum STFRONTEND_LNBCurrentThresholdSelection_e
{
  STFRONTEND_LNB_CURRENT_THRESHOLD_HIGH = 0,/*LNBHxx configuration, see datasheet for more details*/
  STFRONTEND_LNB_CURRENT_THRESHOLD_LOW
} STFRONTEND_LNBCurrentThresholdSelection_t;

typedef BOOL STFRONTEND_CoaxCableLossCompensation_t;  /*TRUE corresponds to 13/18 + 1 V */

/*DiSEqC Version*/
typedef enum STFRONTEND_DISEQC_Version_e
{
   STFRONTEND_DISEQC_VER_1_2 = 1, /* version 1.2*/
   STFRONTEND_DISEQC_VER_2_0 /* version 2.0*/
}STFRONTEND_DISEQC_Version_t;

typedef enum STFRONTEND_DiSEqCCommand_e
  {
   STFRONTEND_DiSEqC_TONE_BURST_OFF,     /* Generic */
   STFRONTEND_DiSEqC_TONE_BURST_OFF_F22_HIGH,   /* f22 pin high after tone off */
   STFRONTEND_DiSEqC_TONE_BURST_OFF_F22_LOW,    /* f22 pin low after tone off*/
   STFRONTEND_DiSEqC_CONTINUOUS_TONE_BURST_ON,    /* unmodulated */
   STFRONTEND_DiSEqC_TONE_BURST_SEND_0_UNMODULATED,          /* send of 0 for 12.5 ms ;continuous tone*/
   STFRONTEND_DiSEqC_TONE_BURST_SEND_0_MODULATED,            /* 0-2/3 duty cycle tone*/
   STFRONTEND_DiSEqC_TONE_BURST_SEND_1,     /* 1-1/3 duty cycle tone*/
   STFRONTEND_DiSEqC_COMMAND       /* DiSEqC (1.2/2)command   */
  } STFRONTEND_DiSEqCCommand_t;

typedef struct
{
 STFRONTEND_DiSEqCCommand_t DiSEqCCommandType;   /* Command type */
 unsigned char  TotalNoOfBytes ;   /* No of Data bytes to be sent including framing and address bytes*/
 unsigned char  *pFrmAddCmdData ;   /* Pointer to data to be sent; Data has to be sequentially placed*/
 unsigned char  msecBeforeNextCommand; /* Time gap required in milliseconds (ms) */
} STFRONTEND_DiSEqCSendPacket_t;


typedef struct
{
 unsigned char  TotalBytesReceived ;   /* No of Data bytes received including framing  byte*/
 unsigned char  *ReceivedFrmAddData ;   /* Pointer to data received; Data has to be sequentially placed*/
 unsigned char  ExpectedNoOfBytes;   /* Higher limit of expected bytes from peripheral*/
} STFRONTEND_DiSEqCResponsePacket_t;

/* Outdoor units: FSK & SCR(not added yet)*/
    typedef enum STFRONTEND_OutdoorDevID_e
    {
           STFRONTEND_OUTDOORDEVICE_FSK /*FSK*/
    } STFRONTEND_OutdoorDevID_t;

/*Outdoor unit control options*/
    typedef enum STFRONTEND_OutdoorDevCtrlID_e
    {
            STFRONTEND_FSK_ModulatorCtrl, /*FSK*/
            STFRONTEND_FSK_DiseqcModeSelect /*FSK*/

    } STFRONTEND_OutdoorDevCtrlID_t;

/*Configuration parameters for FSK*/
typedef struct STFRONTEND_FSK_ModeParams_e
{
      U32 TransmitCarrierFrequency;
      U32 ReceiveCarrierFrequency;
      S32 TransmitterFreqDeviation;
      S32 ReceiverFreqDeviation;
}STFRONTEND_FSK_ModeParams_t;

/* FEC Rates (sat & terr)*/
    typedef enum STFRONTEND_FECRate_e
    {
        STFRONTEND_FEC_NONE = 0x00,    /* no FEC rate specified */
        STFRONTEND_FEC_ALL = 0xFFFF,     /* Logical OR of all FECs */
        STFRONTEND_FEC_1_2 = 1,
        STFRONTEND_FEC_2_3 = (1 << 1),
        STFRONTEND_FEC_3_4 = (1 << 2),
        STFRONTEND_FEC_4_5 = (1 << 3),
        STFRONTEND_FEC_5_6 = (1 << 4),
        STFRONTEND_FEC_6_7 = (1 << 5),
        STFRONTEND_FEC_7_8 = (1 << 6),
        STFRONTEND_FEC_8_9 = (1 << 7),
        STFRONTEND_FEC_1_4 = (1 << 8),
        STFRONTEND_FEC_1_3 = (1 << 9),
        STFRONTEND_FEC_2_5 = (1 << 10),
        STFRONTEND_FEC_3_5 = (1 << 11),
        STFRONTEND_FEC_9_10= (1 << 12)
    } STFRONTEND_FECRate_t;

/* Search Algo parameter used in Satellite Devices such as 900/903/STC*/
typedef enum
{
  STFRONTEND_BLIND_SEARCH,  /* offset freq and SR are Unknown */
  STFRONTEND_COLD_START,  /* only the SR is known */
  STFRONTEND_WARM_START  /* offset freq and SR are known */

} STFRONTEND_SearchAlgo_t;

/*Roll Off Factor, reported in 900/903*/
typedef enum
{
 STFRONTEND_RollOff_35,/*rolloff factor = 0.35*/
 STFRONTEND_RollOff_25,/*roll off factor =0.25*/
 STFRONTEND_RollOff_20,/*roll off factor =0.20*/
 STFRONTEND_RollOff_UNKNOWN/*Unknown RollOff factor*/
}STFRONTEND_RollOff_t;

/* Used for DVB-S2 : Indicates Modulation + Coding : Extracted from physical layer */
typedef enum STFRONTEND_ModeCode_e
 {
  STFRONTEND_MODECODE_DUMMY_PLF,
  STFRONTEND_MODECODE_QPSK_14,
  STFRONTEND_MODECODE_QPSK_13,
  STFRONTEND_MODECODE_QPSK_25,
  STFRONTEND_MODECODE_QPSK_12,
  STFRONTEND_MODECODE_QPSK_35,
  STFRONTEND_MODECODE_QPSK_23,
  STFRONTEND_MODECODE_QPSK_34,
  STFRONTEND_MODECODE_QPSK_45,
  STFRONTEND_MODECODE_QPSK_56,
  STFRONTEND_MODECODE_QPSK_89,
  STFRONTEND_MODECODE_QPSK_910,
  STFRONTEND_MODECODE_8PSK_35,
  STFRONTEND_MODECODE_8PSK_23,
  STFRONTEND_MODECODE_8PSK_34,
  STFRONTEND_MODECODE_8PSK_56,
  STFRONTEND_MODECODE_8PSK_89,
  STFRONTEND_MODECODE_8PSK_910,
  STFRONTEND_MODECODE_16APSK_23,
  STFRONTEND_MODECODE_16APSK_34,
  STFRONTEND_MODECODE_16APSK_45,
  STFRONTEND_MODECODE_16APSK_56,
  STFRONTEND_MODECODE_16APSK_89,
  STFRONTEND_MODECODE_16APSK_910,
  STFRONTEND_MODECODE_32APSK_34,
  STFRONTEND_MODECODE_32APSK_45,
  STFRONTEND_MODECODE_32APSK_56,
  STFRONTEND_MODECODE_32APSK_89,
  STFRONTEND_MODECODE_32APSK_910,
  STFRONTEND_MODECODE_UNKNOWN
 }
 STFRONTEND_ModeCode_t;

/*Used for Satellite devices for LNB control */
typedef enum STFRONTEND_Polarization_e
    {
     STFRONTEND_PLR_ALL        = 0x07, /* Logical OR of all PLRs */
     STFRONTEND_PLR_HORIZONTAL = 1,
     STFRONTEND_PLR_VERTICAL   = (1 << 1),
     STFRONTEND_LNB_OFF        = (1 << 2)  /* Add for LNB off-> in case tuner is fed from modulator */
    } STFRONTEND_Polarization_t;

/*DVB-S2*/
typedef enum STFRONTEND_Framelength_e
    {
        STFRONTEND_LONG_FRAME, /*Used for DVBS2 : Frame size : 64800*/
        STFRONTEND_SHORT_FRAME /*Used for DVBS2 : Frame size : 16400*/
    } STFRONTEND_Framelength_t;

/*Search Standard*/
 typedef enum STFRONTEND_FECType_e
 {
  STFRONTEND_FEC_AUTO_SEARCH ,  /*Used for 900/903 for locking DVBS1/DVBS2 automatically for ADV*/
  STFRONTEND_FEC_DVBS1_STANDARD,         /*DVB-S1 standard*/
  STFRONTEND_FEC_DVBS2_STANDARD , /*DVB-S2 standard*/
  STFRONTEND_FEC_DSS_STANDARD,
  STFRONTEND_FEC_ADCM_STANDARD,/*for use with  TURBO codes*/
  STFRONTEND_FEC_ADCM_QPSK_STANDARD,  /*for use with  TURBO codes*/
  STFRONTEND_FEC_ADCM_8PSK_STANDARD,  /*for use with  TURBO codes*/
  STFRONTEND_FEC_UNKNOWN_STANDARD /*Unknown FECType (corresponds to Unknown Standard used in LLAs) */
 } STFRONTEND_FECType_t;

typedef struct STFRONTEND_LnbConfigureParams_s
{
   STFRONTEND_LnbControl_t  LnbControl;
   STFRONTEND_LnbLlcState_t   LnbLlcState;
   STFRONTEND_LNBShortCircuitProtectionMode_t LnbShortCircuitProtectionMode;
   STFRONTEND_LNBToneState_t LnbToneState;
   STFRONTEND_Polarization_t LnbPolarization;
   BOOL IsLNBPowerOn;
   BOOL IsLNBShortCircuit;
} STFRONTEND_LnbConfigureParams_t;





typedef struct STFRONTEND_BlindScan_Optimization_Params_s
{
 BOOL Optimize_On ;       /*To set the optimisations ON/OFF*/
 U32 Directblind;	 	 /* 	0: single blind search  	1: edge detection   2: launch immediatly a blind without edge detection*/ 
 U32 Doublefail;	         /* pass to 1 if the first blind attempt is failed after a falling edge detected */ 
 S32 TunerIndexJump;   /*contains the freq index where the rising or falling edge occurs*/
 }STFRONTEND_BlindScan_Optimization_Params_t;

typedef enum STFRONTEND_ASP_Version_e
{
  STFRONTEND_ASP_SAT_VER1=1,
  STFRONTEND_ASP_TER_VER1,
  STFRONTEND_ASP_CAB_VER1,
  STFRONTEND_ASP_VSB_VER1,
} STFRONTEND_ASP_Version_t;

typedef struct {
        STFRONTEND_RFSource_t  RFSource;
        U32 GoldCode;
	U32 Associate_Lnb_at_i2c_Address; /*For Dynamic configuration of LNB ;the LNB i2c address will be associated with a handle */
	ST_DeviceName_t STI2C_Bus;
	STFRONTEND_BlindScan_Optimization_Params_t  Blind_Optimize;	
}STFRONTEND_Additional_SearchParams_SatVer1_t;

typedef struct {
      STFRONTEND_ASP_Version_t   Ver;
      union{
        U32 Reserved[15];          /* union max size */
        STFRONTEND_Additional_SearchParams_SatVer1_t SatVer1;
       /* STFRONTEND_Additional_SearchParams_TerVer1_t TerVer1;
        STFRONTEND_Additional_SearchParams_CabVer1_t CabVer1;
	 STFRONTEND_Additional_SearchParams_VsbVer1_t VsbVer1; */  /* For Future use */
      } Addnl;
}STFRONTEND_Additional_SearchParams_t;

typedef struct {
 STFRONTEND_Modulation_t   Modulation; /* example : qpsk , 8psk etc*/
 U32                 SymbolRate;
 STFRONTEND_Spectrum_t     Spectrum; /* IQ spectrum */

 STFRONTEND_Polarization_t           Polarization; /* H/V/ALL */
 STFRONTEND_LNBToneState_t        LNBToneState;    /* 22kHz On/Off */
 STFRONTEND_FECType_t               FECType;  /* S1/S2/DSS */

 STFRONTEND_SearchAlgo_t                   SearchAlgo; /*used to determine blind / cold / warm search */
 STFRONTEND_FECRate_t             FECRates;
 U32       LO;  /*Local Oscillator frequency(in MHz) */
 STFRONTEND_DownLink_t Downlink;    /* Decides Downlink band of transponder Ku/C */
STFRONTEND_SCRSearchParams_t          SCRSearchParams;
 void  * addtnlSearchParams;
}STFRONTEND_Sat_SearchParams_t;


 typedef struct{
  STFRONTEND_Modulation_t    Modulation;
  U32                 SymbolRate;
  STFRONTEND_Spectrum_t      Spectrum;
  S32       SignalQuality;
  U32       BitErrorRate;
  S32       RFLevel;
  STFRONTEND_FECType_t            FECType;/*Sends the standard found(DVB-S/DVB-S2/etc..) on the locked transponder*/
  STFRONTEND_RollOff_t            RollOff;

  STFRONTEND_Polarization_t  Polarization; /*SAT*/
  STFRONTEND_LNBToneState_t     LNBToneState;    /*SAT*/
  STFRONTEND_FECRate_t        FECRate; /*SAT*/
  STFRONTEND_DownLink_t  Band; /*SAT*/
  STFRONTEND_Framelength_t Framelength ;   /*DVBS2 */
  BOOL     Pilots;                            /*DVBS2*/
  STFRONTEND_ModeCode_t ModCode ;  /*DVBS2*/
  U32     PacketErrorRate; /*DVBS2 */
 }STFRONTEND_SAT_SearchResult_t;

/****************************************************************

     Cable Structures

****************************************************************/
/* A number of standards for the transmission of digital TV signals in broadband cable networks are listed in the ITU-T J.83 standard. */
typedef enum STFRONTEND_J83_e
    {
        STFRONTEND_J83_NONE   = 0x00,
        STFRONTEND_J83_ALL    = 0xFF,
        STFRONTEND_J83_A      = 1,           /*ITU-T J.83/A directly corresponds to the DVB-C methods developed in Europe.*/
        STFRONTEND_J83_B      = (1 << 1), /*ITU-T J.83/B describes the broadband TV cable transmission method used in North America.*/
        STFRONTEND_J83_C      = (1 << 2) /*ITU-T J.83/C is basically nothing more than the adaptation of DVB-C to the 6 MHz world used in Japan.*/
    } STFRONTEND_J83_t;

  typedef struct STFRONTEND_CabDevice_InitParams_s
    {
     /*Reserved for future use*/
      U32 TunerIF; /* Tuner output IF frequency default 36.125MHz*/
     void *Dummy1; /*to remove compilation error in OS20*/

    }STFRONTEND_CabDevice_InitParams_t;


typedef struct {
 STFRONTEND_Modulation_t    Modulation;
 U32                 SymbolRate;
 STFRONTEND_Spectrum_t      Spectrum;
 STFRONTEND_J83_t             J83;
void  * addtnlSearchParams;
}STFRONTEND_Cab_SearchParams_t;

typedef struct {
 STFRONTEND_Modulation_t    Modulation;
 U32                 SymbolRate;
 STFRONTEND_Spectrum_t      Spectrum;
 U32       SignalQuality;
 U32       BitErrorRate;
 S32       RFLevel;
 STFRONTEND_PacketCounter_t PacketCounter;
 U32      SNR_10dB;
 U32      PacketErrorRate;
 }STFRONTEND_CAB_SearchResult_t;

/****************************************************************

     Terrestrial Structures

****************************************************************/
/* FFT Mode : The number of carriers : 6817 (8k) or 1705 (2k) carriers */
typedef enum STFRONTEND_Mode_e
{
        STFRONTEND_MODE_2K, /*FFT Mode 2K*/
        STFRONTEND_MODE_8K, /*FFT Mode 8K*/
        STFRONTEND_MODE_4K,  /*FFT Mode 4K*/
        STFRONTEND_MODE_1K, /*FFT Mode 1K - DVB-T2*/
        STFRONTEND_MODE_16K, /*FFT Mode 16K- DVB-T2*/
        STFRONTEND_MODE_32K  /*FFT Mode 32K- DVB-T2*/
} STFRONTEND_Mode_t;


  /* Guard intervals : Used to ensure that distinct transmissions do not interfere with one another.  */
typedef enum STFRONTEND_Guard_e
{
        STFRONTEND_GUARD_1_32,               /* Guard interval = 1/32 */
        STFRONTEND_GUARD_1_16,               /* Guard interval = 1/16 */
        STFRONTEND_GUARD_1_8,                /* Guard interval = 1/8  */
        STFRONTEND_GUARD_1_4,                 /* Guard interval = 1/4  */
        STFRONTEND_GUARD_1_128,               /* Guard interval = 1/128 - DVB-T2*/
        STFRONTEND_GUARD_19_128,                /* Guard interval = 19/128 - DVB-T2 */
        STFRONTEND_GUARD_19_256                 /* Guard interval = 19/256 - DVB-T2 */
} STFRONTEND_Guard_t;


/*Hierarchical modulation : The separation of the RF channel in two virtual circuits , each having a specific bit rate capacity , a specific roughness . */
typedef enum STFRONTEND_Hierarchy_e
    {
        STFRONTEND_HIER_NONE,  /*Hierarchy None*/
        STFRONTEND_HIER_LOW_PRIO,   /*Hierarchy : Low Priority*/
        STFRONTEND_HIER_HIGH_PRIO,     /*Hierarchy : High Priority*/
        STFRONTEND_HIER_PRIO_ANY    /*Hierarchy  :Any*/
    } STFRONTEND_Hierarchy_t;

  /* Alpha value  during Hierarchical Modulation */
typedef enum STFRONTEND_Hierarchy_Alpha_e
    {
       STFRONTEND_HIER_ALPHA_NONE,  /*Alpha None*/
       STFRONTEND_HIER_ALPHA_1,  /*Alpha  : 1*/
       STFRONTEND_HIER_ALPHA_2,  /*Alpha  : 2*/
       STFRONTEND_HIER_ALPHA_4  /*Alpha  : 4*/
    } STFRONTEND_Hierarchy_Alpha_t;

typedef enum STFRONTEND_Force_e
    {
        STFRONTEND_FORCE_NONE  = 0,
        STFRONTEND_FORCE_M_G   = 1
    } STFRONTEND_Force_t;

/*Channel Bandwidth*/
typedef enum STFRONTEND_ChannelBW_e
{
        STFRONTEND_CHAN_BW_NONE =0x00, /*BW : None*/
        STFRONTEND_CHAN_BW_6M  = 6, /*BW : 6MHz*/
        STFRONTEND_CHAN_BW_7M  = 7, /*BW : 7MHz*/
        STFRONTEND_CHAN_BW_8M  = 8,  /*BW : 8MHz*/
        STFRONTEND_CHAN_BW_5M  = 5, /*BW : 5MHz - DVB-T2*/
        STFRONTEND_CHAN_BW_17M  = 17, /*BW : 1.7MHz- DVB-T2*/
        STFRONTEND_CHAN_BW_10M  = 10  /*BW : 10MHz- DVB-T2*/
} STFRONTEND_ChannelBW_t;



typedef enum STFRONTEND_DVB_Type_e
{
    STFRONTEND_DVBT, /*To select DVB-T*/
    STFRONTEND_DVBT2 /*To select DVB-T2*/
} STFRONTEND_DVB_Type_t;

typedef enum STFRONTEND_AdditionalParams_Id_e
{
  STFRONTEND_ADDITIONALPARAMS_FLAGS
}STFRONTEND_AdditionalParams_Id_t;


typedef enum STFRONTEND_Additional_Flags_e
{
  STFRONTEND_FLAG_IF1_AGC1_USED = (1<<0),  /* IF_OUT1 and AGC_IN1 are used for both digital and analog */
  STFRONTEND_FLAG_IF2_AGC2_USED = (1 << 1),  /* IF_OUT2 and AGC_IN 2are used for both digital and analog */
  STFRONTEND_FLAG_TUNER_XTAL_SHARED = (1 << 2),  /* Tuner Xtal is shared with another tuner */
  STFRONTEND_FLAG_TUNER_CLK_EXT_REF = (1 << 3) ,/* External reference clock is used i. e if a 4MHz Xtal is shared with demodulator */
  STFRONTEND_FLAG_TUNER_CLK_XTAL_IN_OUT_ON = (1 << 4),/* XTAL is directly connected to the tuner; CLK_OUT is enabled */
  STFRONTEND_FLAG_TUNER_CLK_IN_OUT_ON = (1 << 5),/* Tuner is driven by CLK_IN and CLK_OUT is enabled */
  STFRONTEND_FLAG_TUNER_CLK_IN_ON_OUT_OFF = (1 << 6) /* Tuner is driven by CLK_IN and CLK_OUT is disabled */

}STFRONTEND_Additional_Flags_t;

typedef struct STFRONTEND_TerDevice_InitParams_s
{
   /*Reserved for future use*/
   void *Dummy; /*to remove compilation error in OS20*/
}STFRONTEND_TerDevice_InitParams_t;

/*tuner IF IQ wiring for terrestrial tuners*/
typedef enum
 {
  STFRONTEND_NORMAL_IF_TUNER, /*most of the tuners fall in this category*/
  STFRONTEND_NORMAL_LONGPATH_IF_TUNER,
  STFRONTEND_LONGPATH_IQ_TUNER

 }STFRONTEND_IF_IQ_Mode_t;
/*Structure for search parameters in terrestrial*/
 typedef struct {
    STFRONTEND_Modulation_t    Modulation;  /*Modulation standards : QPSK,16QAM,64QAM,8-VSB*/
    STFRONTEND_Spectrum_t      Spectrum; /*Inverted , Noninverted or Auto*/
    STFRONTEND_Guard_t         Guard; /*2k or 8k*/
    STFRONTEND_Mode_t          Mode;
    STFRONTEND_Force_t         Force; /**/
    STFRONTEND_Hierarchy_t     Hierarchy; /*Low ,High ,None or Any priority*/
    S32                      EchoPos;
    STFRONTEND_ChannelBW_t     ChannelBW;   /*6,7or8MHz*/
    STFRONTEND_FECRate_t       FECRates;      /*1_2,2_3,3_4,5_6or7_8*/
    STFRONTEND_IF_IQ_Mode_t TunerMode;    /*Normal,IF-IQ or longpath*/
    U32                      DataPLPId;  /*Data PLP id to selct in acquisition*/
    STFRONTEND_DVB_Type_t  DVB_Type;   /*To select DVB-T or DVB-T2*/
     void  * addtnlSearchParams;
 }STFRONTEND_Ter_SearchParams_t;

 typedef struct {
  STFRONTEND_Modulation_t    Modulation;
  STFRONTEND_Spectrum_t      Spectrum;
  U32       SignalQuality;
  U32       BitErrorRate;
  S32       RFLevel;
  STFRONTEND_Guard_t       Guard; /*2k or 8k*/
  STFRONTEND_Mode_t        Mode;
  STFRONTEND_ChannelBW_t   ChannelBW;   /*6,7or8MHz*/
  STFRONTEND_FECRate_t     FECRates;      /*1_2,2_3,3_4,5_6or7_8*/
  STFRONTEND_Hierarchy_t   Hierarchy; /*Low ,High ,None or Any priority*/
  STFRONTEND_Hierarchy_Alpha_t Hierarchy_Alpha;/*TER*/
  S32 ResidualOffset ;/*TER */
  U32	FrameErrorRate;
  U8    PLPid;    /*Active PLP id */
  STFRONTEND_PacketCounter_t PacketCounter;
  U32     PacketErrorRate; /*added to report pER for ISDBT*/
  U8 SSI;    /*signal strength indicator as per nordig spec */
  U32 SNR_10dB;  /* SNR  in (SNR dB * 10 )*/
  U32 SQI;
  U32 LDPC_Iterations;
 }STFRONTEND_TER_SearchResult_t;


 typedef struct STFRONTEND_VSBDevice_InitParams_s
{
   /*Reserved for future use*/
   void *Dummy; /*to remove compilation error in OS20*/
}STFRONTEND_VSBDevice_InitParams_t;


  typedef struct {
    STFRONTEND_Modulation_t    Modulation;  /*Modulation standards : QPSK,16QAM,64QAM,8-VSB*/
    U32                     SymbolRate;     /* useful for 8-VSB :10762000 Sym/s*/
    STFRONTEND_Spectrum_t      Spectrum; /*Inverted , Noninverted or Auto*/
    void  * addtnlSearchParams;

 }STFRONTEND_VSB_SearchParams_t;

typedef struct {

STFRONTEND_Modulation_t    Modulation;
  U32                     SymbolRate;
  STFRONTEND_Spectrum_t      Spectrum;
  U32       SignalQuality;
  U32       BitErrorRate;
  S32       RFLevel;
  S32 ResidualOffset ;/*TER */
}STFRONTEND_VSB_SearchResult_t;

/***************************************************************
  Common Structures for API use
***************************************************************/

typedef struct STFRONTEND_TSModeParams_s
{
     STFRONTEND_TS_ClockPolarity_t      ClockPolarity;   /*Configuration of clock rising/falling edge*/
     STFRONTEND_TS_OutputMode_t       TSOutputMode;  /*TS output Serial/Parallel/DVB-CI */
     STFRONTEND_TS_SyncByteEnable_t  SyncByteEnable; /*MPEG Sync Bytes passed/blocked */
     STFRONTEND_TS_ParityBytes_t         ParityBytesEnable;/*Parity Bytes passed/blocked*/
     STFRONTEND_TS_Swap_t                 Swap; /*Applicable to Serial TS Output: swap MSB and LSB or not, on TS datas output*/

     /*DataRate Control Supported in 900/903*/
     STFRONTEND_TS_DataRateControl_t  DataRateControl; /*TS SPEED/data rate- manual/automatic*/
     U32      TSClockRate;  /* TS clock rate in Hz, valid only if DataRateControl is manual mode */
     /*Smoother programmable only in 297e, 7141, 5197*/
     STFRONTEND_TS_Smoother_t          Smoother;/*the smoother is used/bypass for the serial output*/
 } STFRONTEND_TSModeParams_t;



/*For future use*/
typedef struct STFRONTEND_AddtionalParams_s
    {
      U32             Id;   /*A unique Id specified by driver for making decision on the use of
                                  additional  parameter*/
      void *addtionalparam; /*pointer to the additional parameter*/
    } STFRONTEND_AddtionalParams_t;

typedef struct STFRONTEND_InitParams_s
{
	STFRONTEND_Media_t      Device;/*Cab/Sat/Terr */
	ST_Partition_t      *DriverPartition; /*Memory Partition*/
	STFRONTEND_DemodType_t  DemodType;
	STFRONTEND_IOParam_t    DemodIO;  /*Demod I/O parameters*/
	STFRONTEND_TunerType_t  TunerType;
	STFRONTEND_IOParam_t    TunerIO; /*Tuner I/O parameters*/
	U32                  DemodClock; /*Input clock fed to demodulator XTAL input pins in Hz*/
	U32                  TunerClock;/*Input clock fed to tuner XTAL input in Hz */
	U32                  TunerOutClkDiv;/*If the demodulator clock is fed from tuner output clock then TunerOutClkDiv =
                              TunerClock/DemodClock, else TunerOutClkDiv = 0*/
	STFRONTEND_IQWiring_t   Tuner_IQwiring; /*IQ wiring connection between Demod  and Tuner*/
	/* TunerClock - Value of the XTAL connected to the Tuner
	DemodClock - Value of the XTAL connected to Demod or desired input clock
                           value, whichever applicable*/
	union
	{
		STFRONTEND_SatDevice_InitParams_t  SatParams;        /*Parameters for other sat devices such as LNB, DISEQC, etc.(added new)*/
		STFRONTEND_TerDevice_InitParams_t  TerParams ;
		STFRONTEND_CabDevice_InitParams_t  CabParams ;
		STFRONTEND_VSBDevice_InitParams_t  VSBParams ;
	} DeviceParams;
	/* A new union of Technology Specific devices added to configure devices like
	SCR,LNB,DiSeqC etc.*/

	STFRONTEND_TSModeParams_t   TSModeParams; /*Parameters related to TS*/

	ST_DeviceName_t EVT_DeviceName;       /* name required to Init() STEVT for STFRONTEND events */
	ST_DeviceName_t EVT_RegistrantName;  /* name given to opened instance of EVT_DeviceName by STFRONTEND */

	/*A new structure added to confugure Transport Stream related parameters*/
	STFRONTEND_AddtionalParams_t AdditionalParams;  /* added new contains a void*
	and  field ID   A new structure type added to provide flexibility for the future use */

} STFRONTEND_InitParams_t;

/*Acquisition/Scan Parameters*/
typedef struct STFRONTEND_SearchParams_s
{
	union
	{
		STFRONTEND_Sat_SearchParams_t  Sat_SearchParams;
		STFRONTEND_Cab_SearchParams_t         Cab_SearchParams;
		STFRONTEND_Ter_SearchParams_t  Ter_SearchParams;
		STFRONTEND_VSB_SearchParams_t  VSB_SearchParams;
	} Media_SearchParams;
} STFRONTEND_SearchParams_t;

typedef struct STFRONTEND_DVBT2_Additional_SearchResult_s
{
	U32 BCH_Ber;
} STFRONTEND_Additional_SearchResult_t;

/*The following structure contains the Acquisition Search Results + some other driver state machine info
used in GetTunerInfo() API*/
typedef struct STFRONTEND_TunerInfo_s
{
	STFRONTEND_Status_t     Status;
	U32       StandBy_Check;
	U32      LockTime;
	U32       Frequency; /*Frequency in kHz*/

	union
	{
		STFRONTEND_SAT_SearchResult_t SatSearchRes;
		STFRONTEND_CAB_SearchResult_t CabSearchRes;
		STFRONTEND_TER_SearchResult_t TerSearchRes;
		STFRONTEND_VSB_SearchResult_t VSBSearchRes;
	} STFRONTEND_SearchResult;/*end of  union*/

	void *addnlResults; /*additional results,if any*/
} STFRONTEND_TunerInfo_t;

/* instance open parameters (common) */
typedef struct STFRONTEND_OpenParams_s
{
	/*For Event Notification using STEVT driver*/
	void (*NotifyFunction)(STFRONTEND_Handle_t Handle, STFRONTEND_EventType_t EventType, ST_ErrorCode_t Error);
} STFRONTEND_OpenParams_t;

/* termination parameters (common) */
typedef struct STFRONTEND_TermParams_s
{
	BOOL ForceTerminate;    /* force close() of open handles if TRUE */
} STFRONTEND_TermParams_t;

/* Exported Variables ------------------------------------------------------ */
/* Exported Macros -------------------------------------------------------- */
/* Exported Functions ------------------------------------------------------ */
ST_Revision_t STFRONTEND_GetRevision(void);
ST_ErrorCode_t STFRONTEND_Init(ST_DeviceName_t DeviceName, STFRONTEND_InitParams_t *InitParams);
ST_ErrorCode_t STFRONTEND_Open(const ST_DeviceName_t DeviceName, const STFRONTEND_OpenParams_t *OpenParams_p, STFRONTEND_Handle_t *Handle_p);
ST_ErrorCode_t STFRONTEND_Close(STFRONTEND_Handle_t Handle);
ST_ErrorCode_t STFRONTEND_Term(ST_DeviceName_t DeviceName, const STFRONTEND_TermParams_t *TermParams_p);
ST_ErrorCode_t STFRONTEND_SetFrequency(STFRONTEND_Handle_t Handle, U32 Frequency, STFRONTEND_SearchParams_t *SearchParams_p, U32 Timeout);
ST_ErrorCode_t STFRONTEND_Unlock(STFRONTEND_Handle_t Handle);
ST_ErrorCode_t STFRONTEND_GetTunerInfo(STFRONTEND_Handle_t Handle, STFRONTEND_TunerInfo_t *TunerInfo_p);
ST_ErrorCode_t STFRONTEND_GetStatus(STFRONTEND_Handle_t Handle, STFRONTEND_Status_t *Status_p);
ST_ErrorCode_t STFRONTEND_Scan(STFRONTEND_Handle_t Handle, U32 FreqFrom, U32 FreqTo, U32 FreqStep, STFRONTEND_SearchParams_t *ScanParams_p, U32 Timeout);
ST_ErrorCode_t STFRONTEND_ScanContinue(STFRONTEND_Handle_t Handle);
ST_ErrorCode_t STFRONTEND_DiSEqCSendReceive(STFRONTEND_Handle_t Handle,STFRONTEND_DiSEqCSendPacket_t *pDiSEqCSendPacket,STFRONTEND_DiSEqCResponsePacket_t *pDiSEqCResponsePacket, STFRONTEND_DISEQC_Version_t DiSEqCVersion);
ST_ErrorCode_t STFRONTEND_SetLNBConfig(STFRONTEND_Handle_t Handle, STFRONTEND_LNBToneState_t LNBToneState, STFRONTEND_Polarization_t LNBVoltage);
ST_ErrorCode_t STFRONTEND_OutdoorUnitConfigure(STFRONTEND_Handle_t Handle, STFRONTEND_OutdoorDevID_t OutdoorDeviceID,   void *configurationParams, void  *returnParams);
ST_ErrorCode_t STFRONTEND_OutdoorUnitControl(STFRONTEND_Handle_t Handle , STFRONTEND_OutdoorDevID_t OutdoorDeviceID, STFRONTEND_OutdoorDevCtrlID_t  ControlFunctionID,  void *ControlParams,  void *returnParams);
ST_ErrorCode_t STFRONTEND_CustomiseControl(STFRONTEND_Handle_t Handle, STFRONTEND_Control_t ControlFunction, void *InParams, void *OutParams);
/*Following APIs not tested*/
ST_ErrorCode_t STFRONTEND_StandByMode(STFRONTEND_Handle_t Handle,STFRONTEND_StandByMode_t PowerMode);

#ifdef __cplusplus
}
#endif                          /* __cplusplus */

#endif                          /* __STFRONTEND_H */

