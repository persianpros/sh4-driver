/****************************************************************************
 *
 * Spark7162 frontend driver
 * 
 * Original code Copyright (C), 2011-2016, AV Frontier Tech. Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ***************************************************************************/
#ifndef __TUNER_INTERFACE_H
#define __TUNER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/* common */
#define TUNER_DEMOD_BASE 10  /* to 50 */
#define TUNER_TUNER_BASE 60  /* to 100 */

/* satellite */
#define TUNER_LNB_BASE 210  /* to 250 */
#define TUNER_MAX_HANDLES 6

/* cable */

/* terrestrial */

/* demod device-driver (common) */
typedef enum TUNER_DemodType_e
{
	TUNER_DEMOD_NONE = TUNER_DEMOD_BASE,  /* common: null demodulation driver use with e.g. STV0399 demod driver*/
	TUNER_DEMOD_MN88472, /* terrestrial */
	TUNER_DEMOD_M3501,
	TUNER_DEMOD_STI7167TER,
	TUNER_DEMOD_STI7167CAB
} TUNER_DemodType_T;

/* tuner device-driver (common) */
typedef enum TUNER_TunerType_e
{
	TUNER_TUNER_NONE = TUNER_TUNER_BASE,  /* null tuner driver e.g. use with 399 which has no external tuner driver */
	TUNER_TUNER_SHARP7306,
	TUNER_TUNER_SHARP7903,
	TUNER_TUNER_MXL301,  /* terr 2012.2.13 jhy */
	TUNER_TUNER_MXL603,
	TUNER_TUNER_SN761640  /* cab/ter 2021.03.03.audioniek */
} TUNER_TunerType_T;

#define TUNER_TunerTypeIsMXL(TunerType) ((TunerType == TUNER_TUNER_MXL301) || \
										 (TunerType == TUNER_TUNER_MXL603))

/* enumerated configuration get/set ---------------------------------------- */

/* note: not all devices/devices support all configuration options, also with enumerations
   note that the values assigned are not to be taken as a direct mapping of the low level
   hardware registers (individual device drivers may translate these values) */

/* mode of OFDM signal (ter) */
typedef enum TUNER_Mode_e
{
	TUNER_MODE_2K,
	TUNER_MODE_8K
} TUNER_Mode_T;

/* guard of OFDM signal (ter) */
typedef enum TUNER_Guard_e
{
	TUNER_GUARD_1_32,  // Guard interval = 1/32
	TUNER_GUARD_1_16,  // Guard interval = 1/16
	TUNER_GUARD_1_8,   // Guard interval = 1/8
	TUNER_GUARD_1_4    // Guard interval = 1/4
} TUNER_Guard_T;

/* hierarchy (ter) */
typedef enum TUNER_Hierarchy_e
{
	TUNER_HIER_NONE, /* Regular modulation */
	TUNER_HIER_1, /* Hierarchical modulation a = 1 */
	TUNER_HIER_2, /* Hierarchical modulation a = 2 */
	TUNER_HIER_4 /* Hierarchical modulation a = 4 */
} TUNER_Hierarchy_T;

/* priority (ter) */
typedef enum TUNER_Priority_e
{
	TUNER_PRIORITY_H,
	TUNER_PRIORITY_L
} TUNER_Priority_T;

/* (ter & cable) */
typedef enum TUNER_Spectrum_e
{
	TUNER_INVERSION_NONE = 0,
	TUNER_INVERSION = 1,
	TUNER_INVERSION_AUTO = 2,
	TUNER_INVERSION_UNK = 4
} TUNER_Spectrum_T;

typedef enum TUNER_J83_e
{
	TUNER_J83_NONE = 0x00,
	TUNER_J83_ALL = 0xFF,
	TUNER_J83_A = 1,
	TUNER_J83_B = (1 << 1),
	TUNER_J83_C = (1 << 2)
} TUNER_J83_T;

/* (ter) */
typedef enum TUNER_FreqOff_e
{
	TUNER_OFFSET_NONE = 0,
	TUNER_OFFSET = 1,
	TUNER_OFFSET_POSITIVE = 2,
	TUNER_OFFSET_NEGATIVE = 3
} TUNER_FreqOff_T;

/* (ter) */
typedef enum TUNER_Force_e
{
	TUNER_FORCE_NONE = 0,
	TUNER_FORCE_M_G = 1
} TUNER_Force_T;

/* FEC Modes (sat & terr) */
typedef enum TUNER_FECMode_e
{
	TUNER_FEC_MODE_DEFAULT, /* Use default FEC mode */
	TUNER_FEC_MODE_DVB, /* DVB mode */
	TUNER_FEC_MODE_DIRECTV /* DirecTV mode */
} TUNER_FECMode_T;

/* tuner status (common) */
typedef enum TUNER_Status_e
{
	TUNER_INNER_STATUS_UNLOCKED,
	TUNER_INNER_STATUS_SCANNING,
	TUNER_INNER_STATUS_LOCKED,
	TUNER_INNER_STATUS_NOT_FOUND,
	TUNER_INNER_STATUS_SLEEP,
	TUNER_INNER_STATUS_STANDBY
} TUNER_Status_T;

/* transport stream output mode (common) */
typedef enum TUNER_TSOutputMode_e
{
	TUNER_TS_MODE_DEFAULT, /* TS output not changeable */
	TUNER_TS_MODE_SERIAL, /* TS output serial */
	TUNER_TS_MODE_PARALLEL, /* TS output parallel */
	TUNER_TS_MODE_DVBCI /* TS output DVB-CI */
} TUNER_TSOutputMode_T;

typedef enum TUNER_TuneType_e
{
	TUNER_SET_NONE,
	TUNER_SET_LNB,
	TUNER_SET_FREQ,
	TUNER_SET_ALL
} TUNER_TuneType_t;

typedef enum TUNER_DataClockPolarity_e
{
	TUNER_DATA_CLOCK_POLARITY_DEFAULT = 0x00, /*Clock polarity default*/
	TUNER_DATA_CLOCK_POLARITY_FALLING, /*Clock polarity in rising edge*/
	TUNER_DATA_CLOCK_POLARITY_RISING, /*Clock polarity in falling edge*/
	TUNER_DATA_CLOCK_NONINVERTED, /*Non inverted*/
	TUNER_DATA_CLOCK_INVERTED /*inverted*/
} TUNER_DataClockPolarity_t;

/*tuner */
typedef struct TUNER_IOParam_s
{
	TUNER_IORoute_t Route;
	TUNER_IODriver_t Driver;

	TUNER_DeviceName_t DriverName;
	u32 Address;
} TUNER_IOParam_T;

typedef struct TUNER_TunerIdentifyDbase_s
{
	YWTUNER_DeliverType_T DeviceType;
	TUNER_TunerType_T TunerType;
	u8 ucAddr;
	u32(*Tuner_identify)(u32 IOHandle, TUNER_DemodType_T DemodType);
} TUNER_TunerIdentifyDbase_T;

/* ter tuner */
typedef struct TUNER_TerTunerConfig_s
{
	u32 Frequency;
	u32 TunerStep;
	u32 IF;
	u32 Bandwidth;
	u32 Reference;

	u8 IOBuffer[5];

	u8 CalValue;  // lwj add
	u8 CLPF_CalValue;  // lwj add
} TUNER_TerTunerConfig_T;

/* ter parameters */
typedef struct TUNER_TerParam_s
{
	u32 TuneType;
	u32 FreqKHz;
	TUNER_Mode_T Mode;
	TUNER_Guard_T Guard;
	TUNER_Force_T Force;
	TUNER_Priority_T Hierarchy;
	TUNER_Spectrum_T Spectrum;
	TUNER_FreqOff_T FreqOff;
	YWTUNER_TERBandwidth_T ChannelBW;
	s32 EchoPos;
	BOOL bAntenna;
	u8 TuneMode;  // jhy add for panasonic mn88472, 0 = unkown, 1 = c; 2 = T; 3 = T2
	u8 T2PlpID;   // Mutiplp
	u8 T2PlpNum;
} TUNER_TerParam_T;

/* ter demod */
typedef struct TUNER_TerDemodDriver_s
{
	u32(*Demod_standy)(u8 Index);

	u32(*Demod_reset)(u8 Index);
	u32(*Demod_repeat)(u32 DemodIOHandle, u32 TunerIOHandle, TUNER_IOARCH_Operation_t Operation,
		  u16 SubAddr, u8 *Data, u32 TransferSize, u32 Timeout);
	u32(*Demod_GetSignalInfo)(u8 Index, u32 *Quality, u32 *Intensity, u32 *Ber);
	u32(*Demod_IsLocked)(u8 Index, BOOL *IsLocked);
	u32(*Demod_ScanFreq)(u8 Index);
	u32(*Demod_GetTransponder)(u8 Index, void *Deliver);
	u32(*Demod_GetTransponderNum)(u8 Index, u32 *Num);
} TUNER_TerDemodDriver_T;

/* ter tuner */
typedef struct TUNER_TerTunerDriver_s
{
	u32(*tuner_SetFreq)(u8 Index, u32 Frequency, u32 BandWidth, u32 *NewFrequency);
	u32(*tuner_IsLocked)(u8 Index, BOOL *IsLocked);
	u32(*tuner_GetFreq)(u8 Index, u32 *Frequency);
	u32(*tuner_SetBandWith)(u8 Index, u32 BandWidth);
	u32(*tuner_StartAndCalibrate)(u8 Index);
	u32(*tuner_SetLna)(u8 Index);
	u32(*tuner_AdjustRfPower)(u8 Index, s32 AGC2VAL2);
	u32(*tuner_SetDVBC)(u8 Index);
	u32(*tuner_SetDVBT)(u8 Index);
	u32(*tuner_SetStandby)(u8 Index, u8 StandbyOn);
	s32(*tuner_GetRF_Level)(u8 Index, u16 RFAGC, u16 IFAGC);
} TUNER_TerTunerDriver_T;

/* ter lnb */
typedef struct TUNER_TerLnbDriver_s
{
	u32(*lnb_SetConfig)(u8 Index, TUNER_TerParam_T *TuneParam);
} TUNER_TerLnbDriver_T;

typedef struct TUNER_TerDriverParam_s
{
	TUNER_TerParam_T Param;
	/*TUNER_TerParam_T Result;*/

	TUNER_DemodType_T DemodType;
	TUNER_IOREG_DeviceMap_t Demod_DeviceMap;

	TUNER_TerDemodDriver_T DemodDriver;
	u32 DemodIOHandle;

	TUNER_TunerType_T TunerType;
	TUNER_IOREG_DeviceMap_t Tuner_DeviceMap;

	TUNER_TerTunerDriver_T TunerDriver;
	u32 TunerIOHandle;
	TUNER_TerTunerConfig_T TunerConfig;

	TUNER_TerLnbDriver_T LnbDriver;
} TUNER_TerDriverParam_T;

/*cab*/
/*tuner*/

#define TCDRV_IOBUFFER_MAX 21  /* 21 registers for MT2030/40 */
#define TCDRV_IOBUFFER_SIZE 4  /* 4 registers */
typedef struct TUNER_CabTunerConfig_ss
{
	u32 Frequency;
	u32 TunerStep;
	s32 IQSense;
	u32 IF;
	u32 FreqFactor;
	u32 BandWidth;
	u32 Reference;
	u8 CalValue;  // lwj add
	u8 CLPF_CalValue;  // lwj add

	u8 IOBuffer[TCDRV_IOBUFFER_MAX]; /* buffer for ioarch I/O */
} TUNER_CabTunerConfig_T;

typedef struct TUNER_CabParam_s
{
	u32 TuneType;
	u32 FreqKHz;
	u32 SymbolRateB;

	TUNER_J83_T J83;  /* (cab) */
	TUNER_Spectrum_T Spectrum;  /* (cable) */
	YWTUNER_Modulation_T Modulation;
} TUNER_CabParam_T;

typedef struct TUNER_CabDemodDriver_s
{
	u32(*Demod_standy)(u8 Index);
	u32(*Demod_reset)(u8 Index);
	u32(*Demod_repeat)(u32 DemodIOHandle, u32 TunerIOHandle, TUNER_IOARCH_Operation_t Operation, u16 SubAddr, u8 *Data, u32 TransferSize, u32 Timeout);
	u32(*Demod_GetSignalInfo)(u8 Index, u32 *Quality, u32 *Intensity, u32 *Ber);
	u32(*Demod_IsLocked)(u8 Index, BOOL *IsLocked);
	u32(*Demod_ScanFreq)(u8 Index);
} TUNER_CabDemodDriver_T;

typedef struct TUNER_CabTunerDriver_s
{
	u32(*tuner_SetFreq)(u8 Index, u32 Frequency, u32 *NewFrequency);u32(*tuner_IsLocked)(u8 Index, BOOL *IsLocked);
	u32(*tuner_GetFreq)(u8 Index, u32 *Frequency);
	u32(*tuner_SetStandby)(u8 Index, u8 StandbyOn);
} TUNER_CabTunerDriver_T;

typedef struct TUNER_CabDriverParam_s
{
	TUNER_CabParam_T Param;
	/*TUNER_CabParam_T Result;*/

	TUNER_DemodType_T DemodType;
	TUNER_IOREG_DeviceMap_t Demod_DeviceMap;
	TUNER_CabDemodDriver_T DemodDriver;
	u32 DemodIOHandle;

	TUNER_TunerType_T TunerType;
	TUNER_IOREG_DeviceMap_t Tuner_DeviceMap;
	TUNER_CabTunerDriver_T TunerDriver;
	u32 TunerIOHandle;
	TUNER_CabTunerConfig_T TunerConfig;
} TUNER_CabDriverParam_T;

/* initialization parameters, expand to incorporate cable & terrestrial parameters as needed (common) */
typedef struct TUNER_OpenParams_s
{
	/* (common) */
	u8 TunerIndex;
	YWTUNER_DeliverType_T Device;

//	TUNER_DemodType_T DemodType;
//	TUNER_IOParam_T DemodIO;

//	TUNER_TunerType_T TunerType;
	TUNER_IOParam_T TunerIO;

	u32 ExternalClock;
//	TUNER_TSOutputMode_T TSOutputMode;
//	TUNER_FECMode_T FECMode;

	u8 I2CBusNo;

//	/* (sat) */
//	BOOL IsOpened;//HandleIsValid;
//	YWTUNER_Diseqc10Port_T Committed10;
//	YWTUNER_Diseqc11Port_T Committed11;
} TUNER_OpenParams_T;

#define SCAN_TASK_STACK_SIZE (4096)
#define SCAN_TASK_PRIORITY 5

#if 0
typedef struct TUNER_ScanTask_s
{
	u8 Index;
	u8 ScanTaskStack[SCAN_TASK_STACK_SIZE];
	YWOS_ThreadID_T ScanTaskID;
	YWOS_SemaphoreID_T GuardSemaphore;
	YWOS_SemaphoreID_T TimeoutSemaphore;

	char TaskName[16];
	u32 TimeoutDelayMs;
	BOOL DeleteTask;
} TUNER_ScanTask_T;
#endif

typedef struct TUNER_TuneParams_s
{
	YWTUNER_DeliverType_T Device;

	union
	{
		/* (terrestrial) */
		TUNER_TerParam_T Ter;
		/* (cable) */
		TUNER_CabParam_T Cab;
	} Params;
} TUNER_TuneParams_T;

typedef enum STTUNER_OutputRateCompensationMode_e
{
	TUNER_COMPENSATE_DATACLOCK,
	TUNER_COMPENSATE_DATAVALID
} TUNER_OutputRateCompensationMode_T;

typedef struct STTUNER_OutputFIFOConfig_s
{
	int CLOCKPERIOD;
//	TUNER_OutputRateCompensationMode_T OutputRateCompensationMode;
} TUNER_OutputFIFOConfig_T;

typedef struct TUNER_ScanTaskParam_s
{
	/* (common) */
	u8 TunerIndex;
	YWTUNER_DeliverType_T Device;
	TUNER_Status_T Status;

	u32 ExternalClock;
	TUNER_TSOutputMode_T TSOutputMode;
	TUNER_FECMode_T FECMode;
	TUNER_DataClockPolarity_t ClockPolarity;
	TUNER_OutputFIFOConfig_T OutputFIFOConfig;

	union
	{
		/* (terrestrial) */
		TUNER_TerDriverParam_T Ter;
		/* (cable) */
		TUNER_CabDriverParam_T Cab;
	} DriverParam;

	u8 UnlockTimes;
	u8 LockCount;

	BOOL ForceSearchTerm;
	TUNER_OpenParams_T *pOpenParams;
	void *userdata;
} TUNER_ScanTaskParam_T;

/* functions --------------------------------------------------------------- */

u32 TUNER_Open(u8 Index, TUNER_OpenParams_T *OpenParams);
u32 TUNER_Close(u8 Index);

u32 TUNER_Tune(u8 Index, TUNER_TuneParams_T *TuneParams);
u32 TUNER_Sleep(u8 Index);
u32 TUNER_Wakeup(u8 Index);

u32 TUNER_IDENTIFY_IoarchInit(void);
u32 TUNER_IDENTIFY_DemodOpenIOHandle(u32 *IOHandle, u8 ucAddr, u8 I2cIndex);
u32 TUNER_IDENTIFY_TunerOpenIOHandle(u32 *ExtIOHandle, u32 *IOHandle, TUNER_IOARCH_RedirFn_t TargetFunction,
		u8 ucAddr, u8 I2cIndex);
u32 TUNER_IDENTIFY_CloseIOHandle(u32 IOHandle);
u32 TUNER_ScanInfo_Open(u8 Index);
TUNER_ScanTaskParam_T *TUNER_GetScanInfo(u8 Index);
u32 TUNER_Reset(u8 Index);
u32 TUNER_Standby(u8 Index);
u32 TUNER_GetStatus(u8 Index, BOOL *IsLocked);
u32 TUNER_GetTransponderNum(u8 Index, u32 *Num);

#ifdef __cplusplus
}
#endif

#endif  // __TUNER_INTERFACE_H
// vim:ts=4
