/****************************************************************************
 *
 * STM STV0367 DVB-T/C demodulator driver, DVB-T mode
 *
 * Version for Fulan Spark7162
 *
 * Note: actual device is part of the STi7162 SoC
 *
 * Original code:
 * Copyright (C), 2005-2010, AV Frontier Tech. Co., Ltd.
 *
 * Date: 2010-11-12
 *
 * File: stv0367ter_drv.c
 *
 * History:
 *  Date            Athor       Version   Reason
 *  ============    =========   =======   =================
 *  2010-11-12      lwj         1.0       File creation
 *
 ****************************************************************************/
#include <linux/kernel.h>  /* Kernel support */
#include <linux/delay.h>

#include "ywdefs.h"

#include "ywtuner_ext.h"
#include "tuner_def.h"
#include "ioarch.h"
#include "ioreg.h"
#include "tuner_interface.h"
#include "chip_0367ter.h"

/* STV0367 includes */

#include "stv0367ter_init.h"
#include "stv0367ter_drv.h"

/* dcdc #define CPQ_LIMIT 23*/
#define CPQ_LIMIT 23

/* local functions definition */
int FE_367TER_FilterCoeffInit(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u16 CellsCoeffs[2][6][5], u32 DemodXtal);
void FE_367TER_AGC_IIR_LOCK_DETECTOR_SET(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle);
void FE_367TER_AGC_IIR_RESET(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle);
FE_TER_SignalStatus_t FE_367TER_CheckSYR(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle);
FE_TER_SignalStatus_t FE_367TER_CheckCPAMP(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, s32 FFTmode);
void FE_STV0367ter_SetTS_Parallel_Serial(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, FE_TS_OutputMode_t PathTS);
void FE_STV0367ter_SetCLK_Polarity(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, FE_TS_ClockPolarity_t clock);
u32 FE_367ter_GetMclkFreq(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u32 ExtClk_Hz);

u16 CellsCoeffs_8MHz_367cofdm[3][6][5] =
{
	{
		{ 0x10EF, 0xE205, 0x10EF, 0xCE49, 0x6DA7 },  // CELL 1 COEFFS 27M
		{ 0x2151, 0xc557, 0x2151, 0xc705, 0x6f93 },  // CELL 2 COEFFS
		{ 0x2503, 0xc000, 0x2503, 0xc375, 0x7194 },  // CELL 3 COEFFS
		{ 0x20E9, 0xca94, 0x20e9, 0xc153, 0x7194 },  // CELL 4 COEFFS
		{ 0x06EF, 0xF852, 0x06EF, 0xC057, 0x7207 },  // CELL 5 COEFFS
		{ 0x0000, 0x0ECC, 0x0ECC, 0x0000, 0x3647 }   // CELL 6 COEFFS
	 },
	{
		{ 0x10A0, 0xE2AF, 0x10A1, 0xCE76, 0x6D6D },  // CELL 1 COEFFS 25M
		{ 0x20DC, 0xC676, 0x20D9, 0xC80A, 0x6F29 },
		{ 0x2532, 0xC000, 0x251D, 0xC391, 0x706F },
		{ 0x1F7A, 0xCD2B, 0x2032, 0xC15E, 0x711F },
		{ 0x0698, 0xFA5E, 0x0568, 0xC059, 0x7193 },
		{ 0x0000, 0x0918, 0x149C, 0x0000, 0x3642 }   // CELL 6 COEFFS
	 },
	{
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },  // 30M
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }
	}
};

u16 CellsCoeffs_7MHz_367cofdm[3][6][5] =
{
	{
		{ 0x12CA, 0xDDAF, 0x12CA, 0xCCEB, 0x6FB1 },  // CELL 1 COEFFS 27M
		{ 0x2329, 0xC000, 0x2329, 0xC6B0, 0x725F },  // CELL 2 COEFFS
		{ 0x2394, 0xC000, 0x2394, 0xC2C7, 0x7410 },  // CELL 3 COEFFS
		{ 0x251C, 0xC000, 0x251C, 0xC103, 0x74D9 },  // CELL 4 COEFFS
		{ 0x0804, 0xF546, 0x0804, 0xC040, 0x7544 },  // CELL 5 COEFFS
		{ 0x0000, 0x0CD9, 0x0CD9, 0x0000, 0x370A }   // CELL 6 COEFFS
	 },
	{
		{ 0x1285, 0xDE47, 0x1285, 0xCD17, 0x6F76 },  // 25M
		{ 0x234C, 0xC000, 0x2348, 0xC6DA, 0x7206 },
		{ 0x23B4, 0xC000, 0x23AC, 0xC2DB, 0x73B3 },
		{ 0x253D, 0xC000, 0x25B6, 0xC10B, 0x747F },
		{ 0x0721, 0xF79C, 0x065F, 0xC041, 0x74EB },
		{ 0x0000, 0x08FA, 0x1162, 0x0000, 0x36FF }
	 },
	{
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },  // 30M
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }
	}
};

u16 CellsCoeffs_6MHz_367cofdm[3][6][5] =
{
	{
		{ 0x1699, 0xD5B8, 0x1699, 0xCBC3, 0x713B },  // CELL 1 COEFFS 27M
		{ 0x2245, 0xC000, 0x2245, 0xC568, 0x74D5 },  // CELL 2 COEFFS
		{ 0x227F, 0xC000, 0x227F, 0xC1FC, 0x76C6 },  // CELL 3 COEFFS
		{ 0x235E, 0xC000, 0x235E, 0xC0A7, 0x778A },  // CELL 4 COEFFS
		{ 0x0ECB, 0xEA0B, 0x0ECB, 0xC027, 0x77DD },  // CELL 5 COEFFS
		{ 0x0000, 0x0B68, 0x0B68, 0x0000, 0xC89A },  // CELL 6 COEFFS
	},
	{
		{ 0x1655, 0xD64E, 0x1658, 0xCBEF, 0x70FE },  // 25M
		{ 0x225E, 0xC000, 0x2256, 0xC589, 0x7489 },
		{ 0x2293, 0xC000, 0x2295, 0xC209, 0x767E },
		{ 0x2377, 0xC000, 0x23AA, 0xC0AB, 0x7746 },
		{ 0x0DC7, 0xEBC8, 0x0D07, 0xC027, 0x7799 },
		{ 0x0000, 0x0888, 0x0E9C, 0x0000, 0x3757}
	},
	{
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },  // 30M
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
		{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }
	}
};

#define RF_LOOKUP_TABLE_SIZE_0 19
#define RF_LOOKUP_TABLE_SIZE_1 7
#define RF_LOOKUP_TABLE_SIZE_2 7
#define RF_LOOKUP_TABLE_SIZE_3 11
#define IF_LOOKUP_TABLE_SIZE_4 43

u32 FE_TUNER_RF_LookUp3[RF_LOOKUP_TABLE_SIZE_3] =
{
	66, 79, 87, 97, 108, 119, 132, 144, 160, 182, 217
};

/* -44dBm to -86dBm lnagain=3 IF AGC */
u32 FE_TUNER_IF_LookUp4[IF_LOOKUP_TABLE_SIZE_4] =
{
	990, 1020, 1070, 1160, 1220, 1275, 1350, 1410, 1475, 1530, 1600, 1690, 1785, 1805, 1825, 1885, 1910,
	1950, 2000, 2040, 2090, 2130, 2200, 2295, 2335, 2400, 2450, 2550, 2600, 2660, 2730, 2805, 2840, 2905,
	2955, 3060, 3080, 3150, 3180, 3315, 3405, 4065, 4080
};

/*********************************************************
 --FUNCTION  :: FE_367TER_FilterCoeffInit
 --ACTION    :: Apply filter coeffs values

 --PARAMS IN :: Handle to the Chip
                CellsCoeffs[2][6][5]
 --PARAMS OUT:: NONE
 --RETURN    :: 0 error, 1 no error
 --*******************************************************/
int FE_367TER_FilterCoeffInit(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u16 CellsCoeffs[2][6][5], u32 DemodXtal)
{
	int i, j, k, InternalFreq;
	if (DemodDeviceMap)
	{
		InternalFreq = FE_367ter_GetMclkFreq(DemodDeviceMap, DemodIOHandle, DemodXtal);
	if (InternalFreq == 53125000)
		{
			k = 1;  /* equivalent to Xtal 25M on 362 */
		}
		else if (InternalFreq == 54000000)
		{
			k = 0;  /* equivalent to Xtal 27M on 362 */
		}
		else if (InternalFreq == 52500000)
		{
			k = 2;  /* equivalent to Xtal 30M on 362 */
		}
		else
		{
			return 0;
		}
		for (i = 1; i <= 6; i++)
		{
			ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_IIR_CELL_NB, i - 1);
			for (j = 1; j <= 5; j++)
			{
				ChipSetOneRegister_0367ter(DemodDeviceMap, DemodIOHandle, (R367TER_IIRCX_COEFF1_MSB + 2 * (j - 1)), MSB(CellsCoeffs[k][i - 1][j - 1]));
				ChipSetOneRegister_0367ter(DemodDeviceMap, DemodIOHandle, (R367TER_IIRCX_COEFF1_LSB + 2 * (j - 1)), LSB(CellsCoeffs[k][i - 1][j - 1]));
			}
		}
		return 1;
	}
	else
	{
		return 0;
	}
}

/*********************************************************
 --FUNCTION :: FE_367TER_SpeedInit
 --ACTION :: calculate I2C speed (for SystemWait ...)

 --PARAMS IN :: Handle to the Chip
 --PARAMS OUT:: NONE
 --RETURN :: #ms for an I2C reading access ..
 --********************************************************/
#if 0
int FE_367TER_SpeedInit(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle)
{
	unsigned int i, tempo;
	clock_t time1, time2, time_diff;
	time1 = time_now();
	for (i = 0; i < 16; i++) ChipGetField(hChip, F367TER_EPQ1);
	time2 = time_now();
	time_diff = time_minus(time2, time1);
	tempo = (time_diff * 1000) / ST_GetClocksPerSecond() + 4; /* Duration in milliseconds, + 4 for rounded value */
	tempo = tempo << 4;
	return tempo;
}
#endif  // lwj remove

/*****************************************************
 --FUNCTION      ::  FE_367TER_AGC_IIR_LOCK_DETECTOR_SET
 --ACTION        ::  Sets Good values for AGC IIR lock detector
 --PARAMS IN     ::  Handle to the Chip
 --PARAMS OUT    ::  None
 --***************************************************/
void FE_367TER_AGC_IIR_LOCK_DETECTOR_SET(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle)
{
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_LOCK_DETECT_LSB, 0x00);

	/* Lock detect 1 */
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_LOCK_DETECT_CHOICE, 0x00);
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_LOCK_DETECT_MSB, 0x06);
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_AUT_AGC_TARGET_LSB, 0x04);

	/* Lock detect 2 */
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_LOCK_DETECT_CHOICE, 0x01);
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_LOCK_DETECT_MSB, 0x06);
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_AUT_AGC_TARGET_LSB, 0x04);

	/* Lock detect 3 */
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_LOCK_DETECT_CHOICE, 0x02);
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_LOCK_DETECT_MSB, 0x01);
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_AUT_AGC_TARGET_LSB, 0x00);

	/* Lock detect 4 */
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_LOCK_DETECT_CHOICE, 0x03);
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_LOCK_DETECT_MSB, 0x01);
	ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_AUT_AGC_TARGET_LSB, 0x00);
}

/*****************************************************
 --FUNCTION   :: FE_367TER_IIR_FILTER_INIT
 --ACTION     :: Sets Good IIR Filters coefficients
 --PARAMS IN  :: Handle to the Chip selected bandwidth
 --PARAMS OUT :: None
 --***************************************************/
int FE_367TER_IIR_FILTER_INIT(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u8 Bandwidth, u32 DemodXtalValue)
{
	if (DeviceMap != NULL)
	{
		ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_NRST_IIR, 0);
		switch (Bandwidth)
		{
			case 6:
			{
				if (!FE_367TER_FilterCoeffInit(DeviceMap, IOHandle, CellsCoeffs_6MHz_367cofdm, DemodXtalValue))
				{
					return 0;
				}
				break;
			}
			case 7:
			{
				if (!FE_367TER_FilterCoeffInit(DeviceMap, IOHandle, CellsCoeffs_7MHz_367cofdm, DemodXtalValue))
				{
					return 0;
				}
				break;
			}
			case 8:
			{
				if (!FE_367TER_FilterCoeffInit(DeviceMap, IOHandle, CellsCoeffs_8MHz_367cofdm, DemodXtalValue))
				{
					return 0;
				}
				break;
			}
			default:
			{
				return 0;
			}
		}
		ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_NRST_IIR, 1);
		return 1;
	}
	else
	{
		return 0;
	}
}

/*****************************************************
 --FUNCTION      ::  FE_367TER_AGC_IIR_RESET
 --ACTION        ::  AGC reset procedure
 --PARAMS IN     ::  Handle to the Chip
 --PARAMS OUT    ::  None
 --***************************************************/
void FE_367TER_AGC_IIR_RESET(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle)
{
	u8 com_n;
	com_n = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_COM_N);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_COM_N, 0x07);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_COM_SOFT_RSTN, 0x00);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_COM_AGC_ON, 0x00);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_COM_SOFT_RSTN, 0x01);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_COM_AGC_ON, 0x01);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_COM_N, com_n);
}

/*********************************************************
--FUNCTION :: FE_367ter_duration
--ACTION :: return a duration regarding mode
--PARAMS IN :: mode, tempo1,tempo2,tempo3
--PARAMS OUT:: none
--********************************************************/
int FE_367ter_duration(s32 mode, int tempo1, int tempo2, int tempo3)
{
	int local_tempo = 0;

	switch (mode)
	{
		case 0:
		{
			local_tempo = tempo1;
			break;
		}
		case 1:
		{
			local_tempo = tempo2;
			break;
		}
		case 2:
		{
			local_tempo = tempo3;
			break;
		}
		default:
		{
			break;
		}
	}
	/* WAIT_N_MS(local_tempo); */
	return local_tempo;
}

/*********************************************************
--FUNCTION :: FE_367TER_CheckSYR
--ACTION :: Get SYR status
--PARAMS IN :: Handle to the Chip
--PARAMS OUT:: CPAMP status
--********************************************************/
FE_TER_SignalStatus_t FE_367TER_CheckSYR(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle)
{
	int wd = 100;
	unsigned short int SYR_var;
	s32 SYRStatus;

	SYR_var = ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_SYR_LOCK);
	while ((!SYR_var) && (wd > 0))
	{
		msleep(2);
		wd -= 2;
		SYR_var = ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_SYR_LOCK);
	}
	if (!SYR_var)
	{
		SYRStatus = FE_TER_NOSYMBOL;
	}
	else
	{
		SYRStatus = FE_TER_SYMBOLOK;
	}
	return SYRStatus;
}

/*********************************************************
--FUNCTION :: FE_367TER_CheckCPAMP
--ACTION :: Get CPAMP status
--PARAMS IN :: Handle to the Chip
--PARAMS OUT:: CPAMP status
--********************************************************/
FE_TER_SignalStatus_t FE_367TER_CheckCPAMP(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, s32 FFTmode)
{
	s32 CPAMPvalue = 0, CPAMPStatus, CPAMPMin;
	int wd = 0;

	switch (FFTmode)
	{
		case 0: /* 2k mode */
		{
			CPAMPMin = 20;
			wd = 10;
			break;
		}
		case 1: /* 8k mode */
		{
			CPAMPMin = 80;
			wd = 55;
			break;
		}
		case 2: /* 4k mode */
		{
			CPAMPMin = 40;
			wd = 30;
			break;
		}
		default:
		{
			CPAMPMin = 0xffff; /* drives to NOCPAMP */
			break;
		}
	}
	CPAMPvalue = ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_PPM_CPAMP_DIRECT);

	while ((CPAMPvalue < CPAMPMin) && (wd > 0))
	{
		msleep(1);
		wd -= 1;
		CPAMPvalue = ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_PPM_CPAMP_DIRECT);
//		dprintk(70, "%s: CPAMPvalue= %d at wd=%d\n",__func__, CPAMPvalue, wd);
	}
//	dprintk(70, "%s: ******last CPAMPvalue= %d at wd=%d\n",__func__, CPAMPvalue, wd);
	if (CPAMPvalue < CPAMPMin)
	{
		CPAMPStatus = FE_TER_NOCPAMP;
	}
	else
	{
		CPAMPStatus = FE_TER_CPAMPOK;
	}
	return CPAMPStatus;
}

/***************************************************************************************
 --FUNCTION   :: FE_367ter_LockAlgo
 --ACTION     :: Search for Signal, Timing, Carrier and then data at a given Frequency,
 --              in a given range:

 --PARAMS IN  :: NONE
 --PARAMS OUT :: NONE
 --RETURN     :: Type of the founded signal (if any)

 --REMARKS    :: This function is supposed to replace FE_367ter_Algo according
 --              to last findings on SYR block
 --*************************************************************************************/
FE_TER_SignalStatus_t FE_367ter_LockAlgo(TUNER_IOREG_DeviceMap_t *pDemod_DeviceMap, u32 DemodIOHandle,
			FE_367ter_InternalParams_t *pParams)
{
	FE_TER_SignalStatus_t   ret_flag;
	short int               wd, tempo;
	unsigned short int      try, u_var1 = 0, u_var2 = 0, u_var3 = 0, u_var4 = 0, mode, guard;
	u32         IOHandle = DemodIOHandle;
	TUNER_IOREG_DeviceMap_t *DeviceMap = pDemod_DeviceMap;

	try = 0;
	do
	{
		ret_flag = FE_TER_LOCKOK;
		ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_CORE_ACTIVE, 0);
		if (pParams->IF_IQ_Mode != 0)
		{
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_COM_N, 0x07);
		}
		ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_GUARD, 3);  /* suggested mode is 2k 1/4 */
		ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_MODE, 0);
		ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_SYR_TR_DIS, 0);
		ChipWaitOrAbort_0367ter(FALSE, 5);
		ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_CORE_ACTIVE, 1);
		if (FE_367TER_CheckSYR(DeviceMap, IOHandle) == FE_TER_NOSYMBOL)
		{
			return FE_TER_NOSYMBOL;
		}
		else
		{
			/* if chip locked on wrong mode first try, it must lock correctly second try *db*/
			mode = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_SYR_MODE);
			if (FE_367TER_CheckCPAMP(DeviceMap, IOHandle, mode) == FE_TER_NOCPAMP)
			{
				if (try == 0)
				{
					ret_flag = FE_TER_NOCPAMP;
				}
			}
		}
		try++;
	}
	while ((try < 2) && (ret_flag != FE_TER_LOCKOK));

	if ((mode != 0) && (mode != 1) && (mode != 2))
	{
		return FE_TER_SWNOK;
	}
	//guard=ChipGetField(hChip,F367TER_SYR_GUARD);
	//supress EPQ auto for SYR_GARD 1/16 or 1/32 and set channel predictor in automatic
#if 0
	switch (guard)
	{
		case 0:
		case 1:
		{
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_AUTO_LE_EN, 0);
			ChipSetOneRegister(hChip, R367TER_CHC_CTL, 0x01);
			break;
		}
		case 2:
		case 3:
		{
			ChipSetField_0367ter(DeviceMap, IOHandle,F367TER_AUTO_LE_EN, 1);
			ChipSetOneRegister(hChip, R367TER_CHC_CTL, 0x11);
			break;
		}
		default:
		{
#if defined(HOST_PC) && !defined(NO_GUI)
			ReportInsertMessage("SYR_GUARD uncorrectly set");
#endif
			return FE_TER_SWNOK;
		}
	}
#endif
	/* reset fec an reedsolo FOR 367 only */
	ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_RST_SFEC, 1);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_RST_REEDSOLO, 1);
	msleep(1);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_RST_SFEC, 0);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_RST_REEDSOLO, 0);
	u_var1 = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_LK);
	u_var2 = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_PRF);
	u_var3 = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_TPS_LOCK);
	/* u_var4=ChipGetField(hChip,F367TER_TSFIFO_LINEOK); */

	wd = FE_367ter_duration(mode, 150, 600, 300);
	tempo = FE_367ter_duration(mode, 4, 16, 8);

	/*while ( ((!u_var1)||(!u_var2)||(!u_var3)||(!u_var4)) && (wd>=0)) */
	while (((!u_var1) || (!u_var2) || (!u_var3)) && (wd >= 0))
	{
		ChipWaitOrAbort_0367ter(FALSE, tempo);
		wd -= tempo;
		u_var1 = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_LK);
		u_var2 = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_PRF);
		u_var3 = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_TPS_LOCK);
		/* u_var4=ChipGetField(hChip,F367TER_TSFIFO_LINEOK); */
	}
	if (!u_var1)
	{
//		dprintk(150, "FE_TER_NOLOCK ================= \n");
		return FE_TER_NOLOCK;
	}
	if (!u_var2)
	{
//		dprintk(150, "FE_TER_NOPRFOUND ================= \n");
		return FE_TER_NOPRFOUND;
	}
	if (!u_var3)
	{
//		dprintk(150, "FE_TER_NOTPS ==============================\n");
		return FE_TER_NOTPS;
	}
	guard = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_SYR_GUARD);
	ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_CHC_CTL, 0x11);
	switch (guard)
	{
		case 0:
		case 1:
		{
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_AUTO_LE_EN, 0);
			/*ChipSetOneRegister(hChip,R367TER_CHC_CTL, 0x1);*/
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_SYR_FILTER, 0);
			break;
		}
		case 2:
		case 3:
		{
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_AUTO_LE_EN, 1);
			/*ChipSetOneRegister(hChip,R367TER_CHC_CTL, 0x11);*/
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_SYR_FILTER, 1);
			break;
		}
		default:
		{
			return FE_TER_SWNOK;
		}
	}
	/* apply Sfec workaround if 8K 64QAM CR!=1/2*/
	if ((ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_TPS_CONST) == 2)
	&&  (mode == 1)
	&&  (ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_TPS_HPCODE) != 0))
	{
		ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_SFDLYSETH, 0xc0);
		ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_SFDLYSETM, 0x60);
		ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_SFDLYSETL, 0x0);
	}
	else
	{
		ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_SFDLYSETH, 0x0);
	}
	wd = FE_367ter_duration(mode, 125, 500, 250);
	u_var4 = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_TSFIFO_LINEOK);
	while ((!u_var4) && (wd >= 0))
	{
		ChipWaitOrAbort_0367ter(FALSE, tempo);
		wd -= tempo;
		u_var4 = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_TSFIFO_LINEOK);
	}
	if (!u_var4)
	{
		return FE_TER_NOLOCK;
	}

	/* for 367 leave COM_N at 0x7 for IQ_mode*/
#if 0
	if(pParams->IF_IQ_Mode != FE_TER_NORMAL_IF_TUNER)
	{
		tempo = 0;
		while ((ChipGetField(hChip, F367TER_COM_USEGAINTRK) != 1) && (ChipGetField(hChip, F367TER_COM_AGCLOCK) != 1) && (tempo < 100))
		{
			ChipWaitOrAbort(hChip, 1);
			tempo += 1;
		}
#if defined(HOST_PC) && !defined(NO_GUI)
		Fmt(str_tmp, "%s<%s%i%s", "AGC digital locked after ", tempo, " ms");
		ReportInsertMessage(str_tmp);
#endif
		ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_COM_N, 0x17);
	}
#endif
	/* ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_SYR_TR_DIS, 1); */
	return FE_TER_LOCKOK;
}

/*****************************************************
 --FUNCTION  ::  FE_STV0367TER_SetTS_Parallel_Serial
 --ACTION    ::  TSOutput setting
 --PARAMS IN ::  hChip, PathTS
 --PARAMS OUT::  NONE
 --RETURN    ::
 --***************************************************/

void FE_STV0367ter_SetTS_Parallel_Serial(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, FE_TS_OutputMode_t PathTS)
{
	if (DeviceMap != NULL)
	{
		ChipSetField_0367ter(DeviceMap, IOHandle, F367_TS_DIS, 0);
		switch (PathTS)
		{
			default:
			/*for removing warning :default we can assume in parallel mode*/
			case FE_TS_PARALLEL_PUNCT_CLOCK:
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_TSFIFO_SERIAL, 0);
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_TSFIFO_DVBCI, 0);
				break;
			}
			case FE_TS_SERIAL_PUNCT_CLOCK:
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_TSFIFO_SERIAL, 1);
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_TSFIFO_DVBCI, 1);
				break;
			}
		}
	}
}

/*****************************************************
 --FUNCTION  ::  FE_STV0367TER_SetCLK_Polarity
 --ACTION    ::  clock polarity setting
 --PARAMS IN ::  hChip, PathTS
 --PARAMS OUT::  NONE
 --RETURN    ::

--***************************************************/
void FE_STV0367ter_SetCLK_Polarity(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, FE_TS_ClockPolarity_t clock)
{
	if (DemodDeviceMap != NULL)
	{
		switch (clock)
		{
			case FE_TS_RISINGEDGE_CLOCK:
			{
				ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367_TS_BYTE_CLK_INV, 0);
				break;
			}
			case FE_TS_FALLINGEDGE_CLOCK:
			{
				ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367_TS_BYTE_CLK_INV, 1);
				break;
			}
			/* case FE_TER_CLOCK_POLARITY_DEFAULT: */
			default:
			{
				ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367_TS_BYTE_CLK_INV, 0);
				break;
			}
		}
	}
}

/*****************************************************
 --FUNCTION  ::  FE_STV0367TER_SetCLKgen
 --ACTION    ::  PLL divider setting
 --PARAMS IN ::  hChip, PathTS
 --PARAMS OUT::  NONE
 --RETURN    ::

 --***************************************************/
void FE_STV0367TER_SetCLKgen(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u32 DemodXtalFreq)
{
	if (DemodDeviceMap != NULL)
	{
		switch (DemodXtalFreq)
		{
			/* set internal freq to 53.125MHz */
			case 25000000:
			{
				ChipSetOneRegister_0367ter(DemodDeviceMap, DemodIOHandle, R367_PLLMDIV, 0xA);
				ChipSetOneRegister_0367ter(DemodDeviceMap, DemodIOHandle, R367_PLLNDIV, 0x55);
				ChipSetOneRegister_0367ter(DemodDeviceMap, DemodIOHandle, R367_PLLSETUP, 0x18);
				break;
			}
			case 27000000:
			{
				ChipSetOneRegister_0367ter(DemodDeviceMap, DemodIOHandle, R367_PLLMDIV, 0x1);
				ChipSetOneRegister_0367ter(DemodDeviceMap, DemodIOHandle, R367_PLLNDIV, 0x8);
				ChipSetOneRegister_0367ter(DemodDeviceMap, DemodIOHandle, R367_PLLSETUP, 0x18);
				break;
			}
			case 30000000:
			{
				ChipSetOneRegister_0367ter(DemodDeviceMap, DemodIOHandle, R367_PLLMDIV, 0xc);
				ChipSetOneRegister_0367ter(DemodDeviceMap, DemodIOHandle, R367_PLLNDIV, 0x55);
				ChipSetOneRegister_0367ter(DemodDeviceMap, DemodIOHandle, R367_PLLSETUP, 0x18);
				break;
			}
			default:
			{
				break;
			}
		}
		/* ChipSetOneRegister_0367ter(DemodDeviceMap, DemodIOHandle, R367TER_ANACTRL, 0); */
		/* error = hChip->Error;*/
	}
}

int FE_STV0367TER_GetPower(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle)
{
	s32 power = 0;
	//s32 snr = 0;
	u16 if_agc = 0;
	u16 rf_agc = 0;
	{
		u8 i = 0;
		if_agc =  ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_AGC2_VAL_LO)
		       + (ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_AGC2_VAL_HI) << 8);
		rf_agc = ChipGetField_0367ter(DeviceMap, IOHandle, F367_RF_AGC1_LEVEL_HI);

		if (rf_agc < 255)
		{
			for (i = 0; i < RF_LOOKUP_TABLE_SIZE_3; i++)
			{
				if (rf_agc <= FE_TUNER_RF_LookUp3[i])
				{
					/* -33dBm to -43dBm */
					power = -33 + (-1) * i;
					break;
				}
			}
			if (i == RF_LOOKUP_TABLE_SIZE_3)
			{
				power = -44;
			}
		}
		else if (rf_agc == 255)
		{
			for (i = 0; i < IF_LOOKUP_TABLE_SIZE_4; i++)
			{
				if (if_agc <= FE_TUNER_IF_LookUp4[i])
				{
					/* -44dBm to -86dBm */
					power = -44 + (-1) * i;
					break;
				}
			}
			if (i == IF_LOOKUP_TABLE_SIZE_4) power = -88;
		}
		else
		{
			/*should never pass here*/
			power = -90;
		}
	}
	power = power + 100;
	// power = 100 - power;
	if (power > 100)
	{
		power = 100;
	}
	if (power < 0)
	{
		power = 0;
	}
	return power * 255 * 255 / 100;
}

int FE_STV0367TER_GetSnr(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle)
{
	s32 snr = 0;

	/* For quality return 1000*snr to application SS */
	/* /4 for STv0367, /8 for STv0362 */
	snr = (1000 * ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_CHCSNR)) / 8;
	/**pNoise = (snr*10) >> 3;*/
	/** fix done here for the bug GNBvd20972 where pNoise is calculated in right percentage **/
	/*pInfo->CN_dBx10=(((snr*1000)/8)/32)/10; */
	snr = (snr / 32) / 10;
	if (snr > 100)
	{
		snr = 100;
	}
	return snr * 255 * 255 / 100;
}

#if 1
int FE_STV0367TER_GetSignalInfo(TUNER_IOREG_DeviceMap_t *pDemod_DeviceMap, u32 DemodIOHandle,
				u32 *CN_dBx10, u32 *Power_dBmx10, u32 *Ber)
{
	//TUNER_ScanTaskParam_T   *Inst;
	u32         IOHandle;
	TUNER_IOREG_DeviceMap_t *DeviceMap;

	*Power_dBmx10 = 0;
	*CN_dBx10     = 0;
	*Ber          = 0;

	IOHandle = DemodIOHandle;
	DeviceMap = pDemod_DeviceMap;

	/*FE_367TER_GetErrors(pParams, &Errors, &bits, &PackErrRate,&BitErrRate); */
	/*MeasureBER(pParams,0,1,&BitErrRate,&PackErrRate) ;*/
	*Ber = FE_367ter_GetErrors(DeviceMap, IOHandle);
	*CN_dBx10 = FE_STV0367TER_GetSnr(DeviceMap, IOHandle);
	*Power_dBmx10 = FE_STV0367TER_GetPower(DeviceMap, IOHandle);
	return 0;
}

/*****************************************************
 --FUNCTION  ::  FE_367TER_GetSignalInfo
 --ACTION    ::  Return informations on the locked channel
 --PARAMS IN ::  Handle  ==> Front End Handle
 --PARAMS OUT::  pInfo   ==> Informations (BER,C/N,power ...)
 --RETURN    ::  Error (if any)
 --***************************************************/
typedef enum
{
	TUNER_IQ_NORMAL = 1,
	TUNER_IQ_INVERT = -1
} TUNER_TER_IQ_t;

FE_LLA_Error_t FE_367ter_GetSignalInfo(TUNER_IOREG_DeviceMap_t *pDemod_DeviceMap, u32 DemodIOHandle,
					FE_367ter_InternalParams_t *pParams, FE_TER_SignalInfo_t *pInfo)
{
	FE_LLA_Error_t           error = FE_LLA_NO_ERROR;
	int                      offset = 0;
	FE_TER_FECRate_t         CurFECRates;  /* Added for HM */
	FE_TER_Hierarchy_Alpha_t CurHierMode;  /* Added for HM */
	int                      constell = 0, snr = 0, Data = 0;
	u32          IOHandle = DemodIOHandle;
	TUNER_IOREG_DeviceMap_t  *DeviceMap = pDemod_DeviceMap;
	TUNER_TER_IQ_t           SpectrInvert = TUNER_IQ_NORMAL;  // lwj add question

	constell = ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TPS_CONST);
	if (constell == 0)
	{
		pInfo->Modulation = FE_TER_MOD_QPSK;
	}
	else if (constell == 1)
	{
		pInfo->Modulation = FE_TER_MOD_16QAM;
	}
	else
	{
		pInfo->Modulation = FE_TER_MOD_64QAM;
	}

	if (pParams->IF_IQ_Mode == FE_TER_IQ_TUNER)
	{
		pInfo->SpectInversion = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_IQ_INVERT);
	}
	else
	{
		pInfo->SpectInversion = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_INV_SPECTR);
	}
	/* Get the Hierarchical Mode */
	Data = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_TPS_HIERMODE);
	switch (Data)
	{
		case 0:
		{
			CurHierMode = FE_TER_HIER_ALPHA_NONE;
			break;
		}
		case 1:
		{
			CurHierMode = FE_TER_HIER_ALPHA_1;
			break;
		}
		case 2:
		{
			CurHierMode = FE_TER_HIER_ALPHA_2;
			break;
		}
		case 3:
		{
			CurHierMode = FE_TER_HIER_ALPHA_4;
			break;
		}
		default:
		{
			CurHierMode = Data;
			error = YWHAL_ERROR_BAD_PARAMETER;
			break;  /* error */
		}
	}

	/* Get the FEC Rate */
	if (pParams->Hierarchy == FE_TER_HIER_LOW_PRIO)
	{
		Data = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_TPS_LPCODE);
	}
	else
	{
		Data = ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_TPS_HPCODE);
	}

	switch (Data)
	{
		case 0:
		{
			CurFECRates = FE_TER_FEC_1_2;
			break;
		}
		case 1:
		{
			CurFECRates = FE_TER_FEC_2_3;
			break;
		}
		case 2:
		{
			CurFECRates = FE_TER_FEC_3_4;
			break;
		}
		case 3:
		{
			CurFECRates = FE_TER_FEC_5_6;
			break;
		}
		case 4:
		{
			CurFECRates = FE_TER_FEC_7_8;
			break;
		}
		default:
		{
			CurFECRates = Data;
			error = YWHAL_ERROR_BAD_PARAMETER;
			break;  /* error */
		}
	}

	/**** end of HM addition ******/
	pInfo->pr = CurFECRates;
	pInfo->Hierarchy_Alpha = CurHierMode;
	pInfo->Hierarchy = pParams->Hierarchy;
	pInfo->Mode = ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_SYR_MODE);
	pInfo->Guard = ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_SYR_GUARD);
	/* Carrier offset calculation */
	ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_FREEZE, 1);
//	ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_CRL_FREQ1, 3);
	ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_CRL_FREQ1, 1);
	ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_CRL_FREQ2, 1);
	ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_CRL_FREQ3, 1);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_FREEZE, 0);
	offset  = (ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_CRL_FOFFSET_VHI) << 16);
	offset += (ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_CRL_FOFFSET_HI) << 8);
	offset += (ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_CRL_FOFFSET_LO));
	if (offset > 8388607)
	{
		offset -= 16777216;
	}
	offset = offset * 2 / 16384;

	if (pInfo->Mode == FE_TER_MODE_2K)
	{
		offset = (offset * 4464) / 1000; /*** 1 FFT BIN=4.464khz***/
	}
	else if (pInfo->Mode == FE_TER_MODE_4K)
	{
		offset = (offset * 223) / 100; /*** 1 FFT BIN=2.23khz***/
	}
	else if (pInfo->Mode == FE_TER_MODE_8K)
	{
		offset = (offset * 111) / 100; /*** 1 FFT BIN=1.1khz***/
	}
	if (ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_PPM_INVSEL) == 1)  /* inversion hard auto */
	{
		if (((ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_INV_SPECTR) != ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_STATUS_INV_SPECRUM)) == 1))
		{
			/* no inversion nothing to do*/
		}
		else
		{
			offset = offset * -1;
		}
	}
	else /* manual inversion*/
	{
		if (((!pParams->SpectrumInversion) && (SpectrInvert == TUNER_IQ_NORMAL))
		||  ((pParams->SpectrumInversion) && (SpectrInvert == TUNER_IQ_INVERT)))
		{
			offset = offset * -1;
		}
	}
	if (pParams->ChannelBW == 6)
	{
		offset = (offset * 6) / 8;
	}
	else if (pParams->ChannelBW == 7)
	{
		offset = (offset * 7) / 8;
	}
	pInfo->ResidualOffset = offset;
	/*FE_367TER_GetErrors(pParams, &Errors, &bits, &PackErrRate,&BitErrRate); */
	/*MeasureBER(pParams,0,1,&BitErrRate,&PackErrRate) ;*/
	pInfo->BER = FE_367ter_GetErrors(DeviceMap, IOHandle);
	/*For quality return 1000*snr to application SS*/
	/* /4 for STv0367, /8 for STv0362 */
	snr = (1000 * ChipGetField_0367ter(DeviceMap, IOHandle, F367TER_CHCSNR)) / 8;
	/**pNoise = (snr*10) >> 3;*/
	/** fix done here for the bug GNBvd20972 where pNoise is calculated in right percentage **/
	/*pInfo->CN_dBx10=(((snr*1000)/8)/32)/10; */
	pInfo->CN_dBx10 = (snr / 32) / 10 ;
	return error;
}
#endif

#if 0
/*****************************************************
 --FUNCTION  ::  FE_367TER_Term
 --ACTION    ::  Terminate STV0367 chip connection
 --PARAMS IN ::  Handle  ==> Front End Handle
 --PARAMS OUT::  NONE
 --RETURN    ::  Error (if any)
 --***************************************************/
FE_LLA_Error_t  FE_367ofdm_Term(FE_367ofdm_Handle_t Handle)
{
	FE_LLA_Error_t             error = FE_LLA_NO_ERROR;
	FE_367ter_InternalParams_t *pParams = NULL;

	pParams = (FE_367ter_InternalParams_t *)Handle;

	if (pParams != NULL)
	{
#ifdef HOST_PC
#ifdef NO_GUI
		if (pParams->hTuner2 != NULL)
		{
			FE_TunerTerm(pParams->hTuner2);
		}
		if (pParams->hTuner != NULL)
		{
			FE_TunerTerm(pParams->hTuner);
		}
		if (pParams->hDemod != NULL)
		{
			ChipClose(pParams->hDemod);
		}
		FE_TunerTerm(pParams->hTuner);
#endif
		if (Handle)
		{
			free(pParams);
		}
#endif
	}
	else
	{
		error = FE_LLA_INVALID_HANDLE;
	}
	return error;
}
#endif

/*****************************************************
 **FUNCTION :: FE_367qam_GetMclkFreq
 **ACTION :: Set the STV0367QAM master clock frequency
 **PARAMS IN :: hChip ==> handle to the chip
 ** ExtClk_Hz ==> External clock frequency (Hz)
 **PARAMS OUT:: NONE
 **RETURN :: MasterClock frequency (Hz)
 *****************************************************/
u32 FE_367ter_GetMclkFreq(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, u32 ExtClk_Hz)
{
	u32 mclk_Hz = 0;  /* master clock frequency (Hz) */
	u32 M, N, P;
	if (ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367_BYPASS_PLLXN) == 0)
	{
		//ChipGetRegisters_0367ter(DemodDeviceMap,DemodIOHandle,R367_PLLMDIV,3); //lwj change
		ChipGetRegisters_0367ter(DemodDeviceMap, DemodIOHandle, R367_PLLMDIV, 1);
		ChipGetRegisters_0367ter(DemodDeviceMap, DemodIOHandle, R367_PLLNDIV, 1);
		ChipGetRegisters_0367ter(DemodDeviceMap, DemodIOHandle, R367_PLLSETUP, 1);
		N = (u32)ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367_PLL_NDIV);
		if (N == 0)
		{
			N = N + 1;
		}
		M = (u32)ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367_PLL_MDIV);
		if (M == 0)
		{
			M = M + 1;
		}
		P = (u32)ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367_PLL_PDIV);
		if (P > 5)
		{
			P = 5;
		}
		mclk_Hz = ((ExtClk_Hz / 2) * N) / (M * PowOf2(P));
	}
	else
	{
		mclk_Hz = ExtClk_Hz;
	}
	return mclk_Hz;
}

#if 0
/*****************************************************
**FUNCTION  ::  STV367TER_RepeaterFn
**ACTION    ::  Set the I2C repeater
**PARAMS IN ::  handle to the chip
**              state of the repeater
**PARAMS OUT::  Error
**RETURN    ::  NONE
*****************************************************/
#ifndef STFRONTEND_FORCE_STI2C_DEPRECATED
STCHIP_Error_t STV367ofdm_RepeaterFn(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, BOOL State, u8 *Buffer)
#else
STCHIP_Error_t STV367ofdm_RepeaterFn(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle, BOOL State)
#endif
{
	STCHIP_Error_t error = CHIPERR_NO_ERROR;

	if (hChip != NULL)
	{
		/*ChipSetField_0367ter(DeviceMap, IOHandle,F367_STOP_ENABLE,1);*/
		if (State == TRUE)
		{
#ifndef STFRONTEND_FORCE_STI2C_DEPRECATED
			ChipFillRepeaterMessage(hChip, F367_I2CT_ON, 1, Buffer);
#else
			ChipSetField_0367ter(DeviceMap, IOHandle, F367_I2CT_ON, 1);
#endif
		}
		else
		{
#ifndef STFRONTEND_FORCE_STI2C_DEPRECATED
			ChipFillRepeaterMessage(hChip, F367_I2CT_ON, 0, Buffer);
#else
			ChipSetField_0367ter(DeviceMap, IOHandle, F367_I2CT_ON, 0);
#endif
		}
	}
	return error;
}
#endif

/*STCHIP_Error_t STV367ofdm_RepeaterFn(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle,BOOL State)
{
	STCHIP_Error_t error = CHIPERR_NO_ERROR;

    if (hChip != NULL)
    {
        ChipSetField_0367ter(DeviceMap, IOHandle,F367ofdm_STOP_ENABLE,1);
        if(State == TRUE)
        {
            ChipSetField_0367ter(DeviceMap, IOHandle,F367ofdm_I2CT_ON,1);
        }
        else
        {
            ChipSetField_0367ter(DeviceMap, IOHandle,F367ofdm_I2CT_ON,0);
        }
	}
    return error;
} */

BOOL FE_367ter_lock(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle)
{
	BOOL Locked = FALSE;
	Locked = ((ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_TPS_LOCK)
	       |   ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_PRF)
	       |   ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_LK)
	       |   ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_TSFIFO_LINEOK)));
	return Locked;
}

/*****************************************************
 **FUNCTION  ::  GetBerErrors
 **ACTION    ::  calculating ber using fec1
 **PARAMS IN ::  pParams ==> pointer to FE_TER_InternalParams_t structure
 **PARAMS OUT::  NONE
 **RETURN    ::  10e7*Ber
 *****************************************************/
u32 FE_367ter_GetBerErrors(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle) /*FEC1*/
{
	u32 Errors = 0, Ber = 0, temporary = 0;
	int abc = 0, def = 0, cpt = 0, max_count = 0;

	//if(pParams != NULL)
	{
		ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_SFEC_ERR_SOURCE, 0x07);
		abc = ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_SFEC_ERR_SOURCE);
		/* set max timeout regarding num events*/
		def = ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_SFEC_NUM_EVENT);
		if (def == 2)
		{
			max_count = 15;
		}
		else if (def == 3)
		{
			max_count = 100;
		}
		else
		{
			max_count = 500;
		}
		/*wait for counting completion*/
		while (((ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_SFERRC_OLDVALUE) == 1) && (cpt < max_count))
		||     ((Errors == 0) && (cpt < max_count)))
		{
			msleep(1);
			ChipGetRegisters_0367ter(DemodDeviceMap, DemodIOHandle, R367TER_SFERRCTRL, 4);
			Errors = ((u32)ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_SFEC_ERR_CNT) * PowOf2(16))
			       + ((u32)ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_SFEC_ERR_CNT_HI) * PowOf2(8))
			       + ((u32)ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_SFEC_ERR_CNT_LO));
			cpt++;
		}
		if (Errors == 0)
		{
			Ber = 0;
		}
		else if (abc == 0x7)
		{
			if (Errors <= 4)
			{
				temporary = (Errors * 1000000000) / (8 * PowOf2(14));
				temporary = temporary;
			}
			else if (Errors <= 42)
			{
				temporary = (Errors * 100000000) / (8 * PowOf2(14));
				temporary = temporary * 10;
			}
			else if (Errors <= 429)
			{
				temporary = (Errors * 10000000) / (8 * PowOf2(14));
				temporary = temporary * 100;
			}
			else if (Errors <= 4294)
			{
				temporary = (Errors * 1000000) / (8 * PowOf2(14));
				temporary = temporary * 1000;
			}
			else if (Errors <= 42949)
			{
				temporary = (Errors * 100000) / (8 * PowOf2(14));
				temporary = temporary * 10000;
			}
			else if (Errors <= 429496)
			{
				temporary = (Errors * 10000) / (8 * PowOf2(14));
				temporary = temporary * 100000;
			}
			else  /* if (Errors<4294967) 2^22 max error */
			{
				temporary = (Errors * 1000) / (8 * PowOf2(14));
				temporary = temporary * 100000; /* still to *10 */
			}
			/* Byte error*/
			if (def == 2)
			{
				/* Ber = Errors / (8 * pow(2, 14)); */
				Ber = temporary;
			}
			else if (def == 3)
			{
				/* Ber = Errors / (8 * pow(2, 16)); */
				Ber = temporary / 4;
			}
			else if (def == 4)
			{
				/* Ber = Errors / (8 * pow(2, 18)); */
				Ber = temporary / 16;
			}
			else if (def == 5)
			{
				/* Ber = Errors / (8 * pow(2, 20)); */
				Ber = temporary / 64;
			}
			else if (def == 6)
			{
				/* Ber = Errors / (8 * pow(2, 22)); */
				Ber = temporary / 256;
			}
			else
			{
				/* should not pass here */
				Ber = 0;
			}
			if ((Errors < 4294967) && (Errors > 429496))
			{
				Ber /= 10;
			}
			else
			{
				Ber /= 100;
			}
		}
		/* save actual value*/
		//pParams->PreviousBER = Ber;

		/* measurement not completed, load previous value
		else
		{
			Ber = pParams->PreviousBER;
		}*/
	}
	return Ber;
}

/*****************************************************
 **FUNCTION  ::  GetPerErrors
 **ACTION    ::  counting packet errors using fec1
 **PARAMS IN ::  pParams ==> pointer to FE_TER_InternalParams_t structure
 **PARAMS OUT::  NONE
 **RETURN    ::  10e9*Per
 *****************************************************/
u32 FE_367ter_GetPerErrors(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle) /*Fec2*/
{
	u32 Errors = 0, Per = 0, temporary = 0;
	int abc = 0, def = 0, cpt = 0;

	if (DemodDeviceMap != NULL)
	{
		ChipSetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_ERR_SRC1, 0x09);
		abc = ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_ERR_SRC1);
		def = ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_NUM_EVT1);

		while (((ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_ERRCNT1_OLDVALUE) == 1) && (cpt < 400))
		||     ((Errors == 0) && (cpt < 400)))
		{
			ChipGetRegisters_0367ter(DemodDeviceMap, DemodIOHandle, R367TER_ERRCTRL1, 4);
			msleep(1);
			Errors = ((u32)ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_ERR_CNT1) * PowOf2(16))
			       + ((u32)ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_ERR_CNT1_HI) * PowOf2(8))
			       + ((u32)ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_ERR_CNT1_LO));
			cpt++;
		}

		if (Errors == 0)
		{
			Per = 0;
		}
		else if (abc == 0x9)
		{
			if (Errors <= 4)
			{
				temporary = (Errors * 1000000000) / (8 * PowOf2(8));
				temporary = temporary;
			}
			else if (Errors <= 42)
			{
				temporary = (Errors * 100000000) / (8 * PowOf2(8));
				temporary = temporary * 10;
			}
			else if (Errors <= 429)
			{
				temporary = (Errors * 10000000) / (8 * PowOf2(8));
				temporary = temporary * 100;
			}
			else if (Errors <= 4294)
			{
				temporary = (Errors * 1000000) / (8 * PowOf2(8));
				temporary = temporary * 1000;
			}
			else if (Errors <= 42949)
			{
				temporary = (Errors * 100000) / (8 * PowOf2(8));
				temporary = temporary * 10000;
			}
			else /*if(Errors<=429496) 2^16 errors max*/
			{
				temporary = (Errors * 10000) / (8 * PowOf2(8));
				temporary = temporary * 100000;
			}
			/* pkt error*/
			if (def == 2)
			{
				/* Per = Errors / PowOf2(8);*/
				Per = temporary;
			}
			else if (def == 3)
			{
				/* Per = Errors / PowOf2(10);*/
				Per = temporary / 4;
			}
			else if (def == 4)
			{
				/* Per = Errors / PowOf2(12);*/
				Per = temporary / 16;
			}
			else if (def == 5)
			{
				/* Per = Errors / PowOf2(14);*/
				Per = temporary / 64;
			}
			else if (def == 6)
			{
				/* Per = Errors / PowOf2(16);*/
				Per = temporary / 256;
			}
			else
			{
				Per = 0;
			}
			/* divide by 100 to get PER * 10^7 */
			Per /= 100;
		}
		/* save actual value */
		//pParams->PreviousPER = Per;
	}
#if 0
	/* measurement not completed, load previous value */
	else
	{
		Ber = pParams->PreviousBER;
	}
#endif
	return Per;
}

/*****************************************************
 **FUNCTION  ::  GetErrors
 **ACTION    ::  counting packet errors using fec1
 **PARAMS IN ::  pParams ==> pointer to FE_TER_InternalParams_t structure
 **PARAMS OUT::  NONE
 **RETURN    ::  error nb
 *****************************************************/
u32 FE_367ter_GetErrors(TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle) /*Fec2*/
{
	u32 Errors = 0;
	int cpt = 0;

	if (DemodDeviceMap != NULL)
	{
		/*wait for counting completion*/
		while (((ChipGetField_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_ERRCNT1_OLDVALUE) == 1) && (cpt < 400))
		||     ((Errors == 0) && (cpt < 400)))
		{
			//ChipGetRegisters_0367ter(DemodDeviceMap, DemodIOHandle,R367TER_ERRCTRL1,4); //lwj change 否则容易超时，当CPU负担重的时候
			ChipGetRegisters_0367ter(DemodDeviceMap, DemodIOHandle, R367TER_ERRCTRL1, 1);
			ChipGetRegisters_0367ter(DemodDeviceMap, DemodIOHandle, R367TER_ERRCNT1H, 1);
			ChipGetRegisters_0367ter(DemodDeviceMap, DemodIOHandle, R367TER_ERRCNT1M, 1);
			ChipGetRegisters_0367ter(DemodDeviceMap, DemodIOHandle, R367TER_ERRCNT1L, 1);
			msleep(1);
			Errors = ((u32)ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_ERR_CNT1) * PowOf2(16))
			       + ((u32)ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_ERR_CNT1_HI) * PowOf2(8))
			       + ((u32)ChipGetFieldImage_0367ter(DemodDeviceMap, DemodIOHandle, F367TER_ERR_CNT1_LO));
			cpt++;
		}
	}
	return Errors;
}

#if 0 //lwj remove
/*****************************************************
--FUNCTION :: FE_367TER_SetAbortFlag
--ACTION :: Set Abort flag On/Off
--PARAMS IN :: Handle ==> Front End Handle

-PARAMS OUT:: NONE.
--RETURN :: Error (if any)

--***************************************************/
FE_LLA_Error_t FE_367ter_SetAbortFlag(FE_367ter_Handle_t Handle, BOOL Abort)
{
	FE_LLA_Error_t error = FE_LLA_NO_ERROR;
	FE_367ter_InternalParams_t *pParams = (FE_367ter_InternalParams_t *)Handle;
	if (pParams != NULL)
	{
		if ((pParams->hDemod->Error) || (pParams->hTuner->Error))
			error = FE_LLA_I2C_ERROR;
		else
		{
			ChipAbort(pParams->hTuner, Abort);
			ChipAbort(pParams->hDemod, Abort);
			if ((pParams->hDemod->Error) || (pParams->hTuner->Error)) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}
	}
	else
		error = FE_LLA_INVALID_HANDLE;
	return (error);
}

void demod_get_pd(void *dummy_handle, unsigned short *level, TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle)
{
	*level = ChipGetField(hTuner, F367_RF_AGC1_LEVEL_HI);
	*level = *level << 2;
	*level |= (ChipGetField(hTuner, F367_RF_AGC1_LEVEL_LO) & 0x03);
}

void demod_get_agc(void *dummy_handle, u16 *rf_agc, u16 *bb_agc, TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle)
{
	u16 rf_low, rf_high, bb_low, bb_high;
	rf_low = ChipGetOneRegister(hTuner, R367TER_AGC1VAL1);
	rf_high = ChipGetOneRegister(hTuner, R367TER_AGC1VAL2);
	rf_high <<= 8;
	rf_low &= 0x00ff;
	*rf_agc = (rf_high + rf_low) << 4;
	bb_low = ChipGetOneRegister(hTuner, R367TER_AGC2VAL1);
	bb_high = ChipGetOneRegister(hTuner, R367TER_AGC2VAL2);
	bb_high <<= 8;
	bb_low &= 0x00ff;
	*bb_agc = (bb_high + bb_low) << 4;
}

u16 bbagc_min_start = 0xffff;

void demod_set_agclim(void *dummy_handle, u16 dir_up, TUNER_IOREG_DeviceMap_t *DemodDeviceMap, u32 DemodIOHandle)
{
	u8 agc_min = 0;
	agc_min = ChipGetOneRegister(hTuner, R367TER_AGC2MIN);
	if (bbagc_min_start == 0xffff)
		bbagc_min_start = agc_min;
	if (dir_up)
	{
		if ((agc_min >= bbagc_min_start) && (agc_min <= (0xa4 - 0x04)))
			agc_min += 0x04;
	}
	else
	{
		if ((agc_min >= (bbagc_min_start + 0x04)) && (agc_min <= 0xa4))
			agc_min -= 0x04;
	}
	ChipSetOneRegister(hTuner, R367TER_AGC2MIN, agc_min);
}
#endif
// vim:ts=4
