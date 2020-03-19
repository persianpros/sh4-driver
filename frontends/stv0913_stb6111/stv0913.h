/*
	STV0913 Multistandard Broadcast Frontend driver
	Copyright (C) Manu Abraham <abraham.manu@gmail.com>

	Copyright (C) ST Microelectronics

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifndef __STV0913_H
#define __STV0913_H

#if defined(FOREVER_3434HD) \
 || defined(FOREVER_NANOSMART) \
 || defined(FOREVER_9898HD) \
 || defined(DP7001) \
 || defined(FOREVER_2424HD) \
 || defined(EP8000) \
 || defined(EPP8000) \
 || defined(GPV8000)
#define ST_H205
#endif

/* Standard definitions */
/* Common unsigned types */
#ifndef DEFINED_U8
#define DEFINED_U8
typedef unsigned char  U8;
#endif

#ifndef DEFINED_U16
#define DEFINED_U16
typedef unsigned short U16;
#endif

#ifndef DEFINED_U32
#define DEFINED_U32
typedef unsigned int   U32;
#endif


/* Common signed types */
#ifndef DEFINED_S8
#define DEFINED_S8
typedef signed char  S8;
#endif

#ifndef DEFINED_S16
#define DEFINED_S16
typedef signed short S16;
#endif

#ifndef DEFINED_S32
#define DEFINED_S32
typedef signed int   S32;
#endif

/* Boolean type (values should be among TRUE and FALSE constants only) */
#ifndef DEFINED_BOOL
#define DEFINED_BOOL
typedef int BOOL;
#endif

#ifndef PARTITION_T
#define PARTITION_T
typedef int partition_t;
#endif
#ifndef ST_PARTITION_T
#define ST_PARTITION_T
typedef partition_t ST_Partition_t;
#endif

/* Revision structure */
typedef const char * ST_Revision_t;

/* Function return error code */
typedef U32 ST_ErrorCode_t;

/* Device name type */
#define ST_MAX_DEVICE_NAME 16  /* 15 characters plus '\0' */
typedef char ST_DeviceName_t[ST_MAX_DEVICE_NAME];

/* error codes */
typedef enum
{
    CHIPERR_NO_ERROR = 0,       /* No error encountered */
    CHIPERR_INVALID_HANDLE,     /* Using of an invalid chip handle */
    CHIPERR_INVALID_REG_ID,     /* Using of an invalid register */
    CHIPERR_INVALID_FIELD_ID,   /* Using of an Invalid field */
    CHIPERR_INVALID_FIELD_SIZE, /* Using of a field with an invalid size */
    CHIPERR_I2C_NO_ACK,         /* No acknowledge from the chip */
    CHIPERR_I2C_BURST,          /* Two many registers accessed in burst mode */
    CHIPERR_I2C_OTHER_ERROR     /* Other Errors reported  from I2C driver, added in stfrontend driver*/
} STCHIP_Error_t;

/* how to access I2C bus */
typedef enum
{
	STCHIP_MODE_SUBADR_8,       /* <addr><reg8><data><data>        (e.g. demod chip) */
	STCHIP_MODE_SUBADR_16,      /* <addr><reg8><data><data>        (e.g. demod chip) */
	STCHIP_MODE_NOSUBADR,        /* <addr><data>|<data><data><data> (e.g. tuner chip) */
	STCHIP_MODE_NOSUBADR_RD,
	STCHIP_MODE_SUBADR_8_NS_MICROTUNE,
	STCHIP_MODE_SUBADR_8_NS,
	STCHIP_MODE_MXL,
	STCHIP_MODE_NOSUBADR_SR,    /*start repeat*/
	STCHIP_MODE_SUBADR_8_SR,
	STCHIP_MODE_STV0368		/* 0368: specific use of PAGESHIFT register */
} STCHIP_Mode_t;

/* register information */
typedef struct
{
	U16 Addr;     /* Address */
	U8  Value;    /* Current value */
}
STCHIP_Register_t; /*changed to be in sync with LLA :april 09*/

typedef U32 STI2C_Handle_t;

/* maximum number of chars per chip/tuner name */
#define MAXNAMESIZE 30

/* architecture specifics for I/O mode (common)*/
typedef enum STFRONTEND_IORoute_e
{
	STFRONTEND_IO_DIRECT,      /* open IO address for this instance of device, Used for Demod & Lnb*/
	STFRONTEND_IO_REPEATER    /* as above but send IO via Repeater of another device such as demod
            Used For Tuners*/
} STFRONTEND_IORoute_t;

/* select I/O target (common) */
typedef enum STFRONTEND_IODriver_e
{
	STFRONTEND_IODRV_NULL,     /* null driver, not used currently */
	STFRONTEND_IODRV_I2C,      /* use I2C drivers */
	STFRONTEND_IODRV_MEM      /* memory-mapped drivers */
} STFRONTEND_IODriver_t;

typedef struct stchip_Info_t
{
	ST_DeviceName_t DriverName;
	STFRONTEND_IODriver_t  IODriver;
	STFRONTEND_IORoute_t   IORoute;

	U32                 I2C_BaudRate;
	U32                 I2C_TimeOut;
	STI2C_Handle_t  I2C_Handle;
	semaphore_t *sem_deviceAccess;/*Semaphore to protect against simultaneous access
                                                          to same tuner/demod from two or more threads*/
	S32                  taskID;/*contains taskID of the task in which tuner acquires sempahore*/

/*following part copied directly from LLA*/
	U32    I2cAddr;          /* Chip I2C address */
	char                Name[MAXNAMESIZE];         /* Name of the chip */
	U32                 NbRegs;           /* Number of registers in the chip */
	U32                NbFields;         /* Number of fields in the chip */
	STCHIP_Register_t  *pRegMapImage;     /* Pointer to register map */
	STCHIP_Error_t      Error;            /* Error state */
	STCHIP_Mode_t       ChipMode;         /* Access bus in demod (SubAdr) or tuner (NoSubAdr) mode */
	U8     ChipID;           /* Chip cut ID */
	BOOL                Repeater;         /* Is repeater enabled or not ? */
	struct stchip_Info_t *RepeaterHost;   /* Owner of the repeater */
#ifdef CHIP_STAPI
    STCHIP_Error_t   (*RepeaterFn)(struct stchip_Info_t *hChip,BOOL State, U8 *Buffer);    /* Pointer to repeater routine */
#else
    STCHIP_Error_t   (*RepeaterFn)(struct stchip_Info_t *hChip,BOOL State);    /* Pointer to repeater routine */
#endif
	BOOL IsAutoRepeaterOffEnable;
	U32					XtalFreq;         /* added for dll purpose*/
/* Parameters needed for non sub address devices */
	U32     WrStart;    /* Id of the first writable register */
	U32     WrSize;           /* Number of writable registers */
	U32     RdStart;    /* Id of the first readable register */
	U32     RdSize;     /* Number of readable registers */
	BOOL   Abort;			   /* Abort flag when set to on no register access and no wait  are done  */
	BOOL				IFmode;			  /* IF mode 1; IQ mode 0   */
	void    *pData;     /* pointer to chip data */
} STCHIP_Info_t;

typedef STCHIP_Info_t *STCHIP_Handle_t;

/* Driver specific defines */
/*Used in LLA chip, placed here as LLA includes chip.h */
typedef struct
{
	STCHIP_Info_t *Chip;      /* pointer to parameters to pass to the CHIP API */
	U32            NbDefVal;  /* number of default values (must match number of STC registers) */
}
Demod_InitParams_t;


#define FE_STV0913_MAXLOOKUPSIZE 500
 /* One point of the lookup table */
typedef struct
{
	int realval; /* real value */
	int regval;  /* binary value */
} FE_STV0913_LOOKPOINT_t;

 /* Lookup table definition */
typedef struct
{
	int size;  /* Size of the lookup table */
	FE_STV0913_LOOKPOINT_t table[FE_STV0913_MAXLOOKUPSIZE];  /* Lookup table */
} FE_STV0913_LOOKUP_t;

#if 0
typedef enum
{
	FE_SAT_SEARCH = 0,
	FE_SAT_PLH_DETECTED,
	FE_SAT_DVBS2_FOUND,
	FE_SAT_DVBS_FOUND
} FE_Sat_SEARCHSTATE_t;  /*Current demod search state*/

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
} FE_Sat_DemodPath_t;  /*Current Demod*/

typedef enum
{
	FE_SAT_LNB_1,
	FE_SAT_LNB_2
} FE_Sat_LNBPath_t;  /*Current LNB*/

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
} FE_Sat_DemodNumber_t;  /*Current Demod Number in use*/

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
	FE_SAT_MOD_UNKNOWN = 0,
	FE_SAT_MOD_QPSK = 1,
	FE_SAT_MOD_8PSK = 2,
	FE_SAT_MOD_16APSK = 513,
	FE_SAT_MOD_32APSK  
} FE_Sat_Modulation_t;  /* sat modulation type*/
#endif

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
} FE_STV0913_ModCod_t;  /* ModCod for DVBS2 */

#if 0
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
} FE_Sat_FRAME_t;   /* Frame Type */

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

 /*DiSEqC configuration*/
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
	FE_SAT_SINGLE=0,
	FE_SAT_DUAL
} FE_Sat_DemodType;

typedef enum
{
	FE_POL_HORIZONTAL = 1,
	FE_POL_VERTICAL  = 1 << 1
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
#endif  // if 0

#endif // __STV0913_H
// vim:ts=4
