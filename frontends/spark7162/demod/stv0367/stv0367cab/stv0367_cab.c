/****************************************************************************
 *
 * STM STV0367 DVB-T/C demodulator driver, DVB-C mode
 *
 * Version for Fulan Spark7162
 *
 * Note: actual device is part of the STi7162 SoC
 *
 * Original code:
 * Copyright (C), 2005-2010, AV Frontier Tech. Co., Ltd.
 *
 * Date: 2010-12-28
 *
 * File:  stv0376_cab.c
 *
 *     Date            Author      Version   Reason
 *     ============    =========   =======   =================
 *     2010-12-28      lwj         1.0
 *
 **************************************************************************/
#define TUNER_USE_CAB_STI7167CAB

#include <linux/kernel.h>  /* Kernel support */
#include <linux/string.h>

#include "ywdefs.h"

#include "ywtuner_ext.h"
#include "tuner_def.h"

#include "ioarch.h"
#include "ioreg.h"
#include "tuner_interface.h"

#include "stv0367cab_init.h"
#include "chip_0367cab.h"
#include "stv0367cab_drv.h"
#include "stv0367_cab.h"

extern int paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[stv0367_cab] "

STCHIP_Register_t Def367cabVal[STV0367CAB_NBREGS] =
{
	{ R367CAB_ID,                     0x60 },  /* ID                    */
	{ R367CAB_I2CRPT,                 0x22 },  /* I2CRPT                */
	{ R367CAB_TOPCTRL,                0x10 },  /* TOPCTRL               */
	{ R367CAB_IOCFG0,                 0x80 },  /* IOCFG0                */
	{ R367CAB_DAC0R,                  0x00 },  /* DAC0R                 */
	{ R367CAB_IOCFG1,                 0x00 },  /* IOCFG1                */
	{ R367CAB_DAC1R,                  0x00 },  /* DAC1R                 */
	{ R367CAB_IOCFG2,                 0x00 },  /* IOCFG2                */
	{ R367CAB_SDFR,                   0x00 },  /* SDFR                  */
	{ R367CAB_AUX_CLK,                0x00 },  /* AUX_CLK               */
	{ R367CAB_FREESYS1,               0x00 },  /* FREESYS1              */
	{ R367CAB_FREESYS2,               0x00 },  /* FREESYS2              */
	{ R367CAB_FREESYS3,               0x00 },  /* FREESYS3              */
	{ R367CAB_GPIO_CFG,               0x55 },  /* GPIO_CFG              */
	{ R367CAB_GPIO_CMD,               0x01 },  /* GPIO_CMD              */
	{ R367CAB_TSTRES,                 0x00 },  /* TSTRES                */
	{ R367CAB_ANACTRL,                0x00 },  /* ANACTRL               */
	{ R367CAB_TSTBUS,                 0x00 },  /* TSTBUS                */
	{ R367CAB_RF_AGC1,                0xea },  /* RF_AGC1               */
	{ R367CAB_RF_AGC2,                0x82 },  /* RF_AGC2               */
	{ R367CAB_ANADIGCTRL,             0x0b },  /* ANADIGCTRL            */
	{ R367CAB_PLLMDIV,                0x01 },  /* PLLMDIV               */
	{ R367CAB_PLLNDIV,                0x08 },  /* PLLNDIV               */
	{ R367CAB_PLLSETUP,               0x18 },  /* PLLSETUP              */
	{ R367CAB_DUAL_AD12,              0x04 },  /* DUAL_AD12             */
	{ R367CAB_TSTBIST,                0x00 },  /* TSTBIST               */
	{ R367CAB_CTRL_1,                 0x00 },  /* CTRL_1                */
	{ R367CAB_CTRL_2,                 0x03 },  /* CTRL_2                */
	{ R367CAB_IT_STATUS1,             0x2b },  /* IT_STATUS1            */
	{ R367CAB_IT_STATUS2,             0x08 },  /* IT_STATUS2            */
	{ R367CAB_IT_EN1,                 0x00 },  /* IT_EN1                */
	{ R367CAB_IT_EN2,                 0x00 },  /* IT_EN2                */
	{ R367CAB_CTRL_STATUS,            0x04 },  /* CTRL_STATUS           */
	{ R367CAB_TEST_CTL,               0x00 },  /* TEST_CTL              */
	{ R367CAB_AGC_CTL,                0x73 },  /* AGC_CTL               */
	{ R367CAB_AGC_IF_CFG,             0x50 },  /* AGC_IF_CFG            */
	{ R367CAB_AGC_RF_CFG,             0x00 },  /* AGC_RF_CFG            */
	{ R367CAB_AGC_PWM_CFG,            0x03 },  /* AGC_PWM_CFG           */
	{ R367CAB_AGC_PWR_REF_L,          0x5a },  /* AGC_PWR_REF_L         */
	{ R367CAB_AGC_PWR_REF_H,          0x00 },  /* AGC_PWR_REF_H         */
	{ R367CAB_AGC_RF_TH_L,            0xff },  /* AGC_RF_TH_L           */
	{ R367CAB_AGC_RF_TH_H,            0x07 },  /* AGC_RF_TH_H           */
	{ R367CAB_AGC_IF_LTH_L,           0x00 },  /* AGC_IF_LTH_L          */
	{ R367CAB_AGC_IF_LTH_H,           0x08 },  /* AGC_IF_LTH_H          */
	{ R367CAB_AGC_IF_HTH_L,           0xff },  /* AGC_IF_HTH_L          */
	{ R367CAB_AGC_IF_HTH_H,           0x07 },  /* AGC_IF_HTH_H          */
	{ R367CAB_AGC_PWR_RD_L,           0xa0 },  /* AGC_PWR_RD_L          */
	{ R367CAB_AGC_PWR_RD_M,           0xe9 },  /* AGC_PWR_RD_M          */
	{ R367CAB_AGC_PWR_RD_H,           0x03 },  /* AGC_PWR_RD_H          */
	{ R367CAB_AGC_PWM_IFCMD_L,        0xe4 },  /* AGC_PWM_IFCMD_L       */
	{ R367CAB_AGC_PWM_IFCMD_H,        0x00 },  /* AGC_PWM_IFCMD_H       */
	{ R367CAB_AGC_PWM_RFCMD_L,        0xff },  /* AGC_PWM_RFCMD_L       */
	{ R367CAB_AGC_PWM_RFCMD_H,        0x07 },  /* AGC_PWM_RFCMD_H       */
	{ R367CAB_IQDEM_CFG,              0x01 },  /* IQDEM_CFG             */
	{ R367CAB_MIX_NCO_LL,             0x22 },  /* MIX_NCO_LL            */
	{ R367CAB_MIX_NCO_HL,             0x96 },  /* MIX_NCO_HL            */
	{ R367CAB_MIX_NCO_HH,             0x55 },  /* MIX_NCO_HH            */
	{ R367CAB_SRC_NCO_LL,             0xff },  /* SRC_NCO_LL            */
	{ R367CAB_SRC_NCO_LH,             0x0c },  /* SRC_NCO_LH            */
	{ R367CAB_SRC_NCO_HL,             0xf5 },  /* SRC_NCO_HL            */
	{ R367CAB_SRC_NCO_HH,             0x20 },  /* SRC_NCO_HH            */
	{ R367CAB_IQDEM_GAIN_SRC_L,       0x06 },  /* IQDEM_GAIN_SRC_L      */
	{ R367CAB_IQDEM_GAIN_SRC_H,       0x01 },  /* IQDEM_GAIN_SRC_H      */
	{ R367CAB_IQDEM_DCRM_CFG_LL,      0xfe },  /* IQDEM_DCRM_CFG_LL     */
	{ R367CAB_IQDEM_DCRM_CFG_LH,      0xff },  /* IQDEM_DCRM_CFG_LH     */
	{ R367CAB_IQDEM_DCRM_CFG_HL,      0x0f },  /* IQDEM_DCRM_CFG_HL     */
	{ R367CAB_IQDEM_DCRM_CFG_HH,      0x00 },  /* IQDEM_DCRM_CFG_HH     */
	{ R367CAB_IQDEM_ADJ_COEFF0,       0x34 },  /* IQDEM_ADJ_COEFF0      */
	{ R367CAB_IQDEM_ADJ_COEFF1,       0xae },  /* IQDEM_ADJ_COEFF1      */
	{ R367CAB_IQDEM_ADJ_COEFF2,       0x46 },  /* IQDEM_ADJ_COEFF2      */
	{ R367CAB_IQDEM_ADJ_COEFF3,       0x77 },  /* IQDEM_ADJ_COEFF3      */
	{ R367CAB_IQDEM_ADJ_COEFF4,       0x96 },  /* IQDEM_ADJ_COEFF4      */
	{ R367CAB_IQDEM_ADJ_COEFF5,       0x69 },  /* IQDEM_ADJ_COEFF5      */
	{ R367CAB_IQDEM_ADJ_COEFF6,       0xc7 },  /* IQDEM_ADJ_COEFF6      */
	{ R367CAB_IQDEM_ADJ_COEFF7,       0x01 },  /* IQDEM_ADJ_COEFF7      */
	{ R367CAB_IQDEM_ADJ_EN,           0x04 },  /* IQDEM_ADJ_EN          */
	{ R367CAB_IQDEM_ADJ_AGC_REF,      0x94 },  /* IQDEM_ADJ_AGC_REF     */
	{ R367CAB_ALLPASSFILT1,           0xc9 },  /* ALLPASSFILT1          */
	{ R367CAB_ALLPASSFILT2,           0x2d },  /* ALLPASSFILT2          */
	{ R367CAB_ALLPASSFILT3,           0xa3 },  /* ALLPASSFILT3          */
	{ R367CAB_ALLPASSFILT4,           0xfb },  /* ALLPASSFILT4          */
	{ R367CAB_ALLPASSFILT5,           0xf6 },  /* ALLPASSFILT5          */
	{ R367CAB_ALLPASSFILT6,           0x45 },  /* ALLPASSFILT6          */
	{ R367CAB_ALLPASSFILT7,           0x6f },  /* ALLPASSFILT7          */
	{ R367CAB_ALLPASSFILT8,           0x7e },  /* ALLPASSFILT8          */
	{ R367CAB_ALLPASSFILT9,           0x05 },  /* ALLPASSFILT9          */
	{ R367CAB_ALLPASSFILT10,          0x0a },  /* ALLPASSFILT10         */
	{ R367CAB_ALLPASSFILT11,          0x51 },  /* ALLPASSFILT11         */
	{ R367CAB_TRL_AGC_CFG,            0x20 },  /* TRL_AGC_CFG           */
	{ R367CAB_TRL_LPF_CFG,            0x28 },  /* TRL_LPF_CFG           */
	{ R367CAB_TRL_LPF_ACQ_GAIN,       0x44 },  /* TRL_LPF_ACQ_GAIN      */
	{ R367CAB_TRL_LPF_TRK_GAIN,       0x22 },  /* TRL_LPF_TRK_GAIN      */
	{ R367CAB_TRL_LPF_OUT_GAIN,       0x03 },  /* TRL_LPF_OUT_GAIN      */
	{ R367CAB_TRL_LOCKDET_LTH,        0x04 },  /* TRL_LOCKDET_LTH       */
	{ R367CAB_TRL_LOCKDET_HTH,        0x11 },  /* TRL_LOCKDET_HTH       */
	{ R367CAB_TRL_LOCKDET_TRGVAL,     0x20 },  /* TRL_LOCKDET_TRGVAL    */
	{ R367CAB_IQ_QAM,                 0x01},   /* IQ_QAM                */
	{ R367CAB_FSM_STATE,              0xa0 },  /* FSM_STATE             */
	{ R367CAB_FSM_CTL,                0x08 },  /* FSM_CTL               */
	{ R367CAB_FSM_STS,                0x0c },  /* FSM_STS               */
	{ R367CAB_FSM_SNR0_HTH,           0x00 },  /* FSM_SNR0_HTH          */
	{ R367CAB_FSM_SNR1_HTH,           0x00 },  /* FSM_SNR1_HTH          */
	{ R367CAB_FSM_SNR2_HTH,           0x00 },  /* FSM_SNR2_HTH          */
	{ R367CAB_FSM_SNR0_LTH,           0x00 },  /* FSM_SNR0_LTH          */
	{ R367CAB_FSM_SNR1_LTH,           0x00 },  /* FSM_SNR1_LTH          */
	{ R367CAB_FSM_EQA1_HTH,           0x00 },  /* FSM_EQA1_HTH          */
	{ R367CAB_FSM_TEMPO,              0x32 },  /* FSM_TEMPO             */
	{ R367CAB_FSM_CONFIG,             0x03 },  /* FSM_CONFIG            */
	{ R367CAB_EQU_I_TESTTAP_L,        0x11 },  /* EQU_I_TESTTAP_L       */
	{ R367CAB_EQU_I_TESTTAP_M,        0x00 },  /* EQU_I_TESTTAP_M       */
	{ R367CAB_EQU_I_TESTTAP_H,        0x00 },  /* EQU_I_TESTTAP_H       */
	{ R367CAB_EQU_TESTAP_CFG,         0x00 },  /* EQU_TESTAP_CFG        */
	{ R367CAB_EQU_Q_TESTTAP_L,        0xff },  /* EQU_Q_TESTTAP_L       */
	{ R367CAB_EQU_Q_TESTTAP_M,        0x00 },  /* EQU_Q_TESTTAP_M       */
	{ R367CAB_EQU_Q_TESTTAP_H,        0x00 },  /* EQU_Q_TESTTAP_H       */
	{ R367CAB_EQU_TAP_CTRL,           0x00 },  /* EQU_TAP_CTRL          */
	{ R367CAB_EQU_CTR_CRL_CONTROL_L,  0x11 },  /* EQU_CTR_CRL_CONTROL_L */
	{ R367CAB_EQU_CTR_CRL_CONTROL_H,  0x05 },  /* EQU_CTR_CRL_CONTROL_H */
	{ R367CAB_EQU_CTR_HIPOW_L,        0x00 },  /* EQU_CTR_HIPOW_L       */
	{ R367CAB_EQU_CTR_HIPOW_H,        0x00 },  /* EQU_CTR_HIPOW_H       */
	{ R367CAB_EQU_I_EQU_LO,           0xef },  /* EQU_I_EQU_LO          */
	{ R367CAB_EQU_I_EQU_HI,           0x00 },  /* EQU_I_EQU_HI          */
	{ R367CAB_EQU_Q_EQU_LO,           0xee },  /* EQU_Q_EQU_LO          */
	{ R367CAB_EQU_Q_EQU_HI,           0x00 },  /* EQU_Q_EQU_HI          */
	{ R367CAB_EQU_MAPPER,             0xc5 },  /* EQU_MAPPER            */
	{ R367CAB_EQU_SWEEP_RATE,         0x80 },  /* EQU_SWEEP_RATE        */
	{ R367CAB_EQU_SNR_LO,             0x64 },  /* EQU_SNR_LO            */
	{ R367CAB_EQU_SNR_HI,             0x03 },  /* EQU_SNR_HI            */
	{ R367CAB_EQU_GAMMA_LO,           0x00 },  /* EQU_GAMMA_LO          */
	{ R367CAB_EQU_GAMMA_HI,           0x00 },  /* EQU_GAMMA_HI          */
	{ R367CAB_EQU_ERR_GAIN,           0x36 },  /* EQU_ERR_GAIN          */
	{ R367CAB_EQU_RADIUS,             0xaa },  /* EQU_RADIUS            */
	{ R367CAB_EQU_FFE_MAINTAP,        0x00 },  /* EQU_FFE_MAINTAP       */
	{ R367CAB_EQU_FFE_LEAKAGE,        0x63 },  /* EQU_FFE_LEAKAGE       */
	{ R367CAB_EQU_FFE_MAINTAP_POS,    0xdf },  /* EQU_FFE_MAINTAP_POS   */
	{ R367CAB_EQU_GAIN_WIDE,          0x88 },  /* EQU_GAIN_WIDE         */
	{ R367CAB_EQU_GAIN_NARROW,        0x41 },  /* EQU_GAIN_NARROW       */
	{ R367CAB_EQU_CTR_LPF_GAIN,       0xd1 },  /* EQU_CTR_LPF_GAIN      */
	{ R367CAB_EQU_CRL_LPF_GAIN,       0xa7 },  /* EQU_CRL_LPF_GAIN      */
	{ R367CAB_EQU_GLOBAL_GAIN,        0x06 },  /* EQU_GLOBAL_GAIN       */
	{ R367CAB_EQU_CRL_LD_SEN,         0x85 },  /* EQU_CRL_LD_SEN        */
	{ R367CAB_EQU_CRL_LD_VAL,         0xe2 },  /* EQU_CRL_LD_VAL        */
	{ R367CAB_EQU_CRL_TFR,            0x20 },  /* EQU_CRL_TFR           */
	{ R367CAB_EQU_CRL_BISTH_LO,       0x00 },  /* EQU_CRL_BISTH_LO      */
	{ R367CAB_EQU_CRL_BISTH_HI,       0x00 },  /* EQU_CRL_BISTH_HI      */
	{ R367CAB_EQU_SWEEP_RANGE_LO,     0x00 },  /* EQU_SWEEP_RANGE_LO    */
	{ R367CAB_EQU_SWEEP_RANGE_HI,     0x00 },  /* EQU_SWEEP_RANGE_HI    */
	{ R367CAB_EQU_CRL_LIMITER,        0x40 },  /* EQU_CRL_LIMITER       */
	{ R367CAB_EQU_MODULUS_MAP,        0x90 },  /* EQU_MODULUS_MAP       */
	{ R367CAB_EQU_PNT_GAIN,           0xa7 },  /* EQU_PNT_GAIN          */
	{ R367CAB_FEC_AC_CTR_0,           0x16 },  /* FEC_AC_CTR_0          */
	{ R367CAB_FEC_AC_CTR_1,           0x0b },  /* FEC_AC_CTR_1          */
	{ R367CAB_FEC_AC_CTR_2,           0x88 },  /* FEC_AC_CTR_2          */
	{ R367CAB_FEC_AC_CTR_3,           0x02 },  /* FEC_AC_CTR_3          */
	{ R367CAB_FEC_STATUS,             0x12 },  /* FEC_STATUS            */
	{ R367CAB_RS_COUNTER_0,           0x7d },  /* RS_COUNTER_0          */
	{ R367CAB_RS_COUNTER_1,           0xd0 },  /* RS_COUNTER_1          */
	{ R367CAB_RS_COUNTER_2,           0x19 },  /* RS_COUNTER_2          */
	{ R367CAB_RS_COUNTER_3,           0x0b },  /* RS_COUNTER_3          */
	{ R367CAB_RS_COUNTER_4,           0xa3 },  /* RS_COUNTER_4          */
	{ R367CAB_RS_COUNTER_5,           0x00 },  /* RS_COUNTER_5          */
	{ R367CAB_BERT_0,                 0x01 },  /* BERT_0                */
	{ R367CAB_BERT_1,                 0x25 },  /* BERT_1                */
	{ R367CAB_BERT_2,                 0x41 },  /* BERT_2                */
	{ R367CAB_BERT_3,                 0x39 },  /* BERT_3                */
	{ R367CAB_OUTFORMAT_0,            0xc2 },  /* OUTFORMAT_0           */
	{ R367CAB_OUTFORMAT_1,            0x22 },  /* OUTFORMAT_1           */
	{ R367CAB_SMOOTHER_2,             0x28 },  /* SMOOTHER_2            */
	{ R367CAB_TSMF_CTRL_0,            0x01 },  /* TSMF_CTRL_0           */
	{ R367CAB_TSMF_CTRL_1,            0xc6 },  /* TSMF_CTRL_1           */
	{ R367CAB_TSMF_CTRL_3,            0x43 },  /* TSMF_CTRL_3           */
	{ R367CAB_TS_ON_ID_0,             0x00 },  /* TS_ON_ID_0            */
	{ R367CAB_TS_ON_ID_1,             0x00 },  /* TS_ON_ID_1            */
	{ R367CAB_TS_ON_ID_2,             0x00 },  /* TS_ON_ID_2            */
	{ R367CAB_TS_ON_ID_3,             0x00 },  /* TS_ON_ID_3            */
	{ R367CAB_RE_STATUS_0,            0x00 },  /* RE_STATUS_0           */
	{ R367CAB_RE_STATUS_1,            0x00 },  /* RE_STATUS_1           */
	{ R367CAB_RE_STATUS_2,            0x00 },  /* RE_STATUS_2           */
	{ R367CAB_RE_STATUS_3,            0x00 },  /* RE_STATUS_3           */
	{ R367CAB_TS_STATUS_0,            0x00 },  /* TS_STATUS_0           */
	{ R367CAB_TS_STATUS_1,            0x00 },  /* TS_STATUS_1           */
	{ R367CAB_TS_STATUS_2,            0xa0 },  /* TS_STATUS_2           */
	{ R367CAB_TS_STATUS_3,            0x00 },  /* TS_STATUS_3           */
	{ R367CAB_T_O_ID_0,               0x00 },  /* T_O_ID_0              */
	{ R367CAB_T_O_ID_1,               0x00 },  /* T_O_ID_1              */
	{ R367CAB_T_O_ID_2,               0x00 },  /* T_O_ID_2              */
	{ R367CAB_T_O_ID_3,               0x00 },  /* T_O_ID_3              */
};

u32 FirstTimeBER[3] = { 0 };
FE_367cab_SIGNALTYPE_t FE_367cab_Algo(u32 Handle, FE_367cab_InternalParams_t *pIntParams);

#if 0
/***********************************************************************
	Name: demod_stv0367cab_Identify
 ***********************************************************************/
unsigned int demod_stv0367cab_Identify(u32 IOHandle, u8 ucID, u8 *pucActualID)
{
#ifdef TUNER_USE_CAB_STI7167CAB
	return YW_NO_ERROR;
#else
	return YWHAL_ERROR_UNKNOWN_DEVICE;
#endif
}
#endif

/***********************************************************************
	Name: demod_stv0367cab_Repeat
 ***********************************************************************/
unsigned int demod_stv0367cab_Repeat(u32 DemodIOHandle, u32 TunerIOHandle, TUNER_IOARCH_Operation_t Operation,
		unsigned short SubAddr, u8 *Data, u32 TransferSize, u32 Timeout)
{
	return 0;
}

#ifdef TUNER_USE_CAB_STI7167CAB
unsigned int stv0367cab_init(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, TUNER_TunerType_T TunerType)
{
	u16 i, j = 1;

	for (i = 1; i <= DeviceMap->Registers; i++)
	{
		ChipUpdateDefaultValues_0367cab(DeviceMap, IOHandle, Def367cabVal[i - 1].Addr, Def367cabVal[i - 1].Value, i - 1);

		if (i != STV0367CAB_NBREGS)
		{
			if (Def367cabVal[i].Addr == Def367cabVal[i - 1].Addr + 1)
			{
				j++;
			}
			else if (j == 1)
			{
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, Def367cabVal[i - 1].Addr, Def367cabVal[i - 1].Value);
			}
			else
			{
				ChipSetRegisters_0367cab(DeviceMap, IOHandle, Def367cabVal[i - j].Addr, j);
				j = 1;
			}
		}
		else
		{
			if (j == 1)
			{
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, Def367cabVal[i - 1].Addr, Def367cabVal[i - 1].Value);
			}
			else
			{
				ChipSetRegisters_0367cab(DeviceMap, IOHandle, Def367cabVal[i - j].Addr, j);
				j = 1;
			}
		}
	}
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_BERT_ON, 0);  /* restart a new BER count */
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_BERT_ON, 1);  /* restart a new BER count */

	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_OUTFORMAT, 0x00);  // FE_TS_PARALLEL_PUNCT_CLOCK
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_CLK_POLARITY, 0x01);  // FE_TS_RISINGEDGE_CLOCK
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_SYNC_STRIP, 0X00);  // STFRONTEND_TS_SYNCBYTE_ON
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_CT_NBST, 0x00);  // STFRONTEND_TS_PARITYBYTES_OFF
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_TS_SWAP, 0x00);  // STFRONTEND_TS_SWAP_OFF
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_FIFO_BYPASS, 0x01);  // FE_TS_SMOOTHER_DEFAULT, ?? question

	/* Here we make the necessary changes to the demod's registers depending on the tuner */
	if (DeviceMap != NULL)
	{
		switch (TunerType) //important
		{
			case TUNER_TUNER_SN761640:
			{
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_ANACTRL, 0x0D); /* PLL bypassed and disabled */
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_PLLMDIV, 0x01); /* IC runs at 54MHz with a 27MHz crystal */
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_PLLNDIV, 0x08);
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_PLLSETUP, 0x18); /* ADC clock is equal to system clock */
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_ANACTRL, 0x00); /* PLL enabled and used */
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_ANADIGCTRL, 0x0b); /* Buffer Q disabled */
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_DUAL_AD12, 0x04); /* ADCQ disabled */
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_FSM_SNR2_HTH, 0x23); /* Improves the C/N lock limit */
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_IQ_QAM, 0x01); /* ZIF/IF Automatic mode */
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_I2CRPT, 0x22); /* I2C repeater configuration, value changes with I2C master clock */
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_EQU_FFE_LEAKAGE, 0x63);
				ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_IQDEM_ADJ_EN, 0x04); //lwj change 0x05 to 0x04
				break;
			}
			default:
			{
				dprintk(1, "%s: Error: tuner type %d unknown\n", __func__, TunerType);
				break;
			}
		}
	}
	return YW_NO_ERROR;
}

unsigned int stv0367cab_Sleep(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle)
{
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_BYPASS_PLLXN, 0x03);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_STDBY_PLLXN, 0x01);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_STDBY, 1);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_STDBY_CORE, 1);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_EN_BUFFER_I, 0);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_EN_BUFFER_Q, 0);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_POFFQ, 1);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_POFFI, 1);
	return YW_NO_ERROR;
}

unsigned int stv0367cab_Wake(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle)
{
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_STDBY_PLLXN, 0x00);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_BYPASS_PLLXN, 0x00);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_STDBY, 0);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_STDBY_CORE, 0);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_EN_BUFFER_I, 1);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_EN_BUFFER_Q, 1);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_POFFQ, 0);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_POFFI, 0);
	return YW_NO_ERROR;
}

unsigned int stv0367cab_I2C_gate_On(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle)
{
	return ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_I2CT_ON, 1);
}

unsigned int stv0367cab_I2C_gate_Off(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle)
{
	return ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_I2CT_ON, 0);
}

unsigned int demod_stv0367cab_Open_test(u32 IOHandle)
{
	unsigned int          Error = YW_NO_ERROR;
	TUNER_IOREG_DeviceMap_t DeviceMap;

	DeviceMap.Timeout   = IOREG_DEFAULT_TIMEOUT;
	DeviceMap.Registers = STV0367CAB_NBREGS;
	DeviceMap.Fields    = STV0367CAB_NBFIELDS;
	DeviceMap.Mode      = IOREG_MODE_SUBADR_16;
	DeviceMap.RegExtClk = 27000000;  // Demod External Crystal_HZ

	//Error = TUNER_IOREG_Open(&DeviceMap);
	DeviceMap.DefVal = NULL;
	DeviceMap.Error = 0;

	stv0367cab_init(&DeviceMap, IOHandle, TUNER_TUNER_SN761640);
	//pParams->MasterClock_Hz = FE_367cab_GetMclkFreq(pParams->hDemod,pParams->Crystal_Hz);
	//pParams->AdcClock_Hz = FE_367cab_GetADCFreq(pParams->hDemod,pParams->Crystal_Hz); //question
	//Error = TUNER_IOREG_Close(&DeviceMap);
	return Error;
}

/***********************************************************************
    Name:   demod_stv0367cab_Open
 ***********************************************************************/
unsigned int demod_stv0367cab_Open(u8 Handle)
{
	unsigned int Error = YW_NO_ERROR;
	TUNER_ScanTaskParam_T *Inst = NULL;
	u32 IOHandle;
	TUNER_IOREG_DeviceMap_t *DeviceMap;

//	dprintk(100, "%s >\n", __func__);
	Inst = TUNER_GetScanInfo(Handle);
	IOHandle = Inst->DriverParam.Cab.DemodIOHandle;
	DeviceMap = &Inst->DriverParam.Cab.Demod_DeviceMap;

	Inst->DriverParam.Cab.DemodDriver.Demod_GetSignalInfo = demod_stv0367cab_GetSignalInfo;
	Inst->DriverParam.Cab.DemodDriver.Demod_IsLocked      = demod_stv0367cab_IsLocked;
	Inst->DriverParam.Cab.DemodDriver.Demod_repeat        = demod_stv0367cab_Repeat;
	Inst->DriverParam.Cab.DemodDriver.Demod_reset         = demod_stv0367cab_Reset;
	Inst->DriverParam.Cab.DemodDriver.Demod_ScanFreq      = demod_stv0367cab_ScanFreq;
	Inst->DriverParam.Cab.DemodDriver.Demod_standy        = demod_stv0367cab_SetStandby;

	DeviceMap->Timeout = IOREG_DEFAULT_TIMEOUT;
	DeviceMap->Registers = STV0367CAB_NBREGS;
	DeviceMap->Fields = STV0367CAB_NBFIELDS;
	DeviceMap->Mode = IOREG_MODE_SUBADR_16;
	DeviceMap->RegExtClk = Inst->ExternalClock;  // Demod External Crystal_HZ

	//Error = TUNER_IOREG_Open(DeviceMap);
	DeviceMap->DefVal = NULL;
	DeviceMap->Error = 0;

	stv0367cab_init(DeviceMap, IOHandle, Inst->DriverParam.Cab.TunerType);

	//pParams->MasterClock_Hz = FE_367cab_GetMclkFreq(pParams->hDemod,pParams->Crystal_Hz);
	//pParams->AdcClock_Hz = FE_367cab_GetADCFreq(pParams->hDemod,pParams->Crystal_Hz); //question
	return Error;
}

/***********************************************************************
	Name:   demod_stv0367cab_Close
 ***********************************************************************/
unsigned int demod_stv0367cab_Close(u8 Handle)
{
	unsigned int Error = YW_NO_ERROR;
	TUNER_ScanTaskParam_T *Inst = NULL;

	Inst = TUNER_GetScanInfo(Handle);
	Inst->DriverParam.Cab.DemodDriver.Demod_GetSignalInfo = NULL;
	Inst->DriverParam.Cab.DemodDriver.Demod_IsLocked = NULL;
	Inst->DriverParam.Cab.DemodDriver.Demod_repeat = NULL;
	Inst->DriverParam.Cab.DemodDriver.Demod_reset = NULL;
	Inst->DriverParam.Cab.DemodDriver.Demod_ScanFreq = NULL;
	//Error = TUNER_IOREG_Close(&Inst->DriverParam.Cab.Demod_DeviceMap);
	Error |= Inst->DriverParam.Cab.Demod_DeviceMap.Error;
	Inst->DriverParam.Cab.Demod_DeviceMap.Error = YW_NO_ERROR;
	return Error;
}

/***********************************************************************
	Name:   demod_stv0367cab_Reset
 ***********************************************************************/
unsigned int demod_stv0367cab_Reset(u8 Index)
{
	unsigned int Error = YW_NO_ERROR;
	TUNER_ScanTaskParam_T *Inst = NULL;
	u32 IOHandle;
	TUNER_IOREG_DeviceMap_t *DeviceMap;

	Inst = TUNER_GetScanInfo(Index);
	IOHandle = Inst->DriverParam.Cab.DemodIOHandle;
	DeviceMap = &Inst->DriverParam.Cab.Demod_DeviceMap;
	stv0367cab_init(DeviceMap, IOHandle, Inst->DriverParam.Cab.TunerType);
	//pParams->MasterClock_Hz = FE_367cab_GetMclkFreq(pParams->hDemod,pParams->Crystal_Hz);
	//pParams->AdcClock_Hz = FE_367cab_GetADCFreq(pParams->hDemod,pParams->Crystal_Hz); //question
	return Error;
}

/***********************************************************************
	Name:   demod_stv0367cab_GetSignalInfo
 ***********************************************************************/
unsigned int demod_stv0367cab_GetSignalInfo(u8 Index, u32 *Quality, u32 *Intensity, u32 *Ber)
{
	unsigned int Error = YW_NO_ERROR;

	*Quality = 0;
	*Intensity = 0;
	*Ber = 0;

	FE_STV0367QAM_GetSignalInfo(Index, Quality, Intensity, Ber, &(FirstTimeBER[Index]));
	return Error;
}

/***********************************************************************
	Name:   demod_stv0367cab_IsLocked
 ***********************************************************************/
unsigned int demod_stv0367cab_IsLocked(u8 Handle, BOOL *IsLocked)
{
	unsigned int Error = YW_NO_ERROR;
	TUNER_ScanTaskParam_T *Inst;
	u32 IOHandle;
	TUNER_IOREG_DeviceMap_t *DeviceMap;

	Inst = TUNER_GetScanInfo(Handle);
	IOHandle = Inst->DriverParam.Cab.DemodIOHandle;
	DeviceMap = &Inst->DriverParam.Cab.Demod_DeviceMap;

	*IsLocked = FE_367cab_Status(DeviceMap, IOHandle);
	return Error;
}

/*****************************************************
 --FUNCTION  ::  FE_367cab_Algo
 --ACTION    ::  Driver functio of the STV0367QAM chip
 --PARAMS IN ::  *pIntParams ==> Demod handle
 --                              Tuner handle
                                 Search frequency
                                 Symbolrate
                                 QAM Size
                                 DerotOffset
                                 SweepRate
                                 Search Range
                                 Master clock
                                 ADC clock
                                 Structure for result storage
 --PARAMS OUT::  NONE
 --RETURN    ::  Handle to STV0367QAM
 --***************************************************/
FE_367cab_SIGNALTYPE_t FE_367cab_Algo(u32 Handle, FE_367cab_InternalParams_t *pIntParams)
{
	FE_367cab_SIGNALTYPE_t  signalType = FE_367cab_NOAGC;  /* Signal search status initialization */
	u32                     QAMFEC_Lock, QAM_Lock, u32_tmp;
	BOOL                    TunerLock = FALSE;
	u32                     LockTime, TRLTimeOut, AGCTimeOut, CRLSymbols, CRLTimeOut, EQLTimeOut, DemodTimeOut, FECTimeOut;
	u8                      TrackAGCAccum;
	TUNER_ScanTaskParam_T   *Inst = NULL;
	TUNER_IOREG_DeviceMap_t *DeviceMap = NULL;
	u32         IOHandle;
	u32                     FreqResult = 0;

	Inst = TUNER_GetScanInfo(Handle);
	IOHandle = Inst->DriverParam.Cab.DemodIOHandle;
	DeviceMap = &Inst->DriverParam.Cab.Demod_DeviceMap;

	if (Inst->ForceSearchTerm)
	{
		return (FE_LLA_INVALID_HANDLE);
	}

	/* Timeouts calculation */
	/* A max lock time of 25 ms is allowed for delayed AGC */
	AGCTimeOut = 25;
	/* 100000 symbols needed by the TRL as a maximum value */
	TRLTimeOut = 100000000 / pIntParams->SymbolRate_Bds;
	/* CRLSymbols is the needed number of symbols to achieve a lock within [-4%, +4%] of the symbol rate.
	   CRL timeout is calculated for a lock within [-SearchRange_Hz, +SearchRange_Hz].
	   EQL timeout can be changed depending on the micro-reflections we want to handle.
	   A characterization must be performed with these echoes to get new timeout values.
	*/
	switch (pIntParams->Modulation)
	{
		case FE_CAB_MOD_QAM16:
		{
			CRLSymbols = 150000;
			EQLTimeOut = 100;
			break;
		}
		case FE_CAB_MOD_QAM32:
		{
			CRLSymbols = 250000;
			EQLTimeOut = 100;
			break;
		}
		case FE_CAB_MOD_QAM64:
		{
			CRLSymbols = 200000;
			EQLTimeOut = 100;
			break;
		}
		case FE_CAB_MOD_QAM128:
		{
			CRLSymbols = 250000;
			EQLTimeOut = 100;
			break;
		}
		case FE_CAB_MOD_QAM256:
		{
			CRLSymbols = 250000;
			EQLTimeOut = 100;
			break;
		}
		default:
		{
			CRLSymbols = 200000;
			EQLTimeOut = 100;
			break;
		}
	}
	if (pIntParams->SearchRange_Hz < 0)
	{
		CRLTimeOut = (25 * CRLSymbols * (-pIntParams->SearchRange_Hz / 1000)) / (pIntParams->SymbolRate_Bds / 1000);
	}
	else
	{
		CRLTimeOut = (25 * CRLSymbols * (pIntParams->SearchRange_Hz / 1000)) / (pIntParams->SymbolRate_Bds / 1000);
	}
	CRLTimeOut = (1000 * CRLTimeOut) / pIntParams->SymbolRate_Bds;
	/* Timeouts below 50ms are coerced */
	if (CRLTimeOut < 50)
	{
		CRLTimeOut = 50;
	}
	/* A maximum of 100 TS packets is needed to get FEC lock even in case the spectrum inversion needs to be changed.
	   This is equal to 20 ms in case of the lowest symbol rate of 0.87Msps
	*/
	FECTimeOut = 20;
	DemodTimeOut = AGCTimeOut + TRLTimeOut + CRLTimeOut + EQLTimeOut;
	/* Reset the TRL to ensure nothing starts until the
	   AGC is stable which ensures a better lock time
	*/
	ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_CTRL_1, 0x04);
	/* Set AGC accumulation time to minimum and lock threshold to maximum in order to speed up the AGC lock */
	TrackAGCAccum = ChipGetField_0367cab(DeviceMap, IOHandle, F367CAB_AGC_ACCUMRSTSEL);
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_AGC_ACCUMRSTSEL, 0x0);
	/* Modulus Mapper is disabled */
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_MODULUSMAP_EN, 0);
	/* Disable the sweep function */
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_SWEEP_EN, 0);
	/* The sweep function is never used, Sweep rate must be set to 0 */
	/* Set the derotator frequency in Hz */
	//FE_367cab_SetDerotFreq(DeviceMap,IOHandle,pIntParams->AdcClock_Hz,(1000*(int)FE_TunerGetIF_Freq(pIntParams->hTuner)+pIntParams->DerotOffset_Hz));

	if (Inst->DriverParam.Cab.TunerType == TUNER_TUNER_SN761640)
	{
		FE_367cab_SetDerotFreq(DeviceMap, IOHandle, pIntParams->AdcClock_Hz, (1000 * (36125000 / 1000) + pIntParams->DerotOffset_Hz)); //question if freq
	}
	/* Disable the Allpass Filter when the symbol rate is out of range */
	if ((pIntParams->SymbolRate_Bds > 10800000) || (pIntParams->SymbolRate_Bds < 1800000))
	{
		ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_ADJ_EN, 0);
		ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_ALLPASSFILT_EN, 0);
	}
	/* Check if the tuner is locked */
	if (Inst->DriverParam.Cab.TunerDriver.tuner_IsLocked != NULL)
	{
		Inst->DriverParam.Cab.TunerDriver.tuner_IsLocked(Handle, &TunerLock);
	}
	if (TunerLock == 0)
	{
		return FE_367cab_NOTUNER;
	}

	/* Release the TRL to start demodulator acquisition */
	/* Wait for QAM lock */
	LockTime = 0;
	ChipSetOneRegister_0367cab(DeviceMap, IOHandle, R367CAB_CTRL_1, 0x00);
	do
	{
		QAM_Lock = ChipGetField_0367cab(DeviceMap, IOHandle, F367CAB_FSM_STATUS);
		if ((LockTime >= (DemodTimeOut - EQLTimeOut)) && (QAM_Lock == 0x04))
		{
			/* We don't wait longer, the frequency/phase offset must be too big */
			LockTime = DemodTimeOut;
		}
		else if ((LockTime >= (AGCTimeOut + TRLTimeOut)) && (QAM_Lock == 0x02))
		{
			/* We do not wait longer, either there is no signal or it is not the right symbol rate or it is an analog carrier */
			LockTime = DemodTimeOut;
			ChipGetRegisters_0367cab(DeviceMap, IOHandle, R367CAB_AGC_PWR_RD_L, 3);
			u32_tmp = ChipGetFieldImage_0367cab(DeviceMap, IOHandle, F367CAB_AGC_PWR_WORD_LO)
					  + (ChipGetFieldImage_0367cab(DeviceMap, IOHandle, F367CAB_AGC_PWR_WORD_ME) << 8)
					  + (ChipGetFieldImage_0367cab(DeviceMap, IOHandle, F367CAB_AGC_PWR_WORD_HI) << 16);
			if (u32_tmp >= 131072)
			{
				u32_tmp = 262144 - u32_tmp;
			}
			u32_tmp = u32_tmp / (PowOf2(11 - ChipGetField_0367cab(DeviceMap, IOHandle, F367CAB_AGC_IF_BWSEL)));

			if (u32_tmp < ChipGetField_0367cab(DeviceMap, IOHandle, F367CAB_AGC_PWRREF_LO) + 256 * ChipGetField_0367cab(DeviceMap, IOHandle, F367CAB_AGC_PWRREF_HI) - 10)
			{
				QAM_Lock = 0x0f;
			}
		}
		else
		{
			ChipWaitOrAbort_0367cab(Inst->ForceSearchTerm, 10); /* wait 10 ms */
			LockTime += 10;
		}
	}
	while (((QAM_Lock != 0x0c) && (QAM_Lock != 0x0b)) && (LockTime < DemodTimeOut));

	if ((QAM_Lock == 0x0c) || (QAM_Lock == 0x0b))
	{
		/* Wait for FEC lock */
		LockTime = 0;
		do
		{
			ChipWaitOrAbort_0367cab(Inst->ForceSearchTerm, 5); /* wait 5 ms */
			LockTime += 5;
			QAMFEC_Lock = ChipGetField_0367cab(DeviceMap, IOHandle, F367CAB_QAMFEC_LOCK);
		}
		while (!QAMFEC_Lock && (LockTime < FECTimeOut));
	}
	else
	{
		QAMFEC_Lock = 0;
	}

	if (QAMFEC_Lock)
	{
		signalType = FE_367cab_DATAOK;
		pIntParams->DemodResult.Modulation = pIntParams->Modulation;
		pIntParams->DemodResult.SpectInv = ChipGetField_0367cab(DeviceMap, IOHandle, F367CAB_QUAD_INV);
		// lwj add begin
		if (Inst->DriverParam.Cab.TunerDriver.tuner_GetFreq != NULL)  // lwj add
		{
			Inst->DriverParam.Cab.TunerDriver.tuner_GetFreq(Handle, &FreqResult);
		}
		// lwj add end

		//if (FE_TunerGetIF_Freq(pIntParams->hTuner) != 0)
#if 0 // lwj remove
		if (0)
		{
			if (FE_TunerGetIF_Freq(pIntParams->hTuner) > pIntParams->AdcClock_Hz / 1000)
			{
				pIntParams->DemodResult.Frequency_kHz = FreqResult - FE_367cab_GetDerotFreq(DeviceMap, IOHandle, pIntParams->AdcClock_Hz) - pIntParams->AdcClock_Hz / 1000 + FE_TunerGetIF_Freq(pIntParams->hTuner);
			}
			else
			{
				pIntParams->DemodResult.Frequency_kHz = FreqResult - FE_367cab_GetDerotFreq(DeviceMap, IOHandle, pIntParams->AdcClock_Hz) + FE_TunerGetIF_Freq(pIntParams->hTuner);
			}
		}
		else
#endif
		{
			pIntParams->DemodResult.Frequency_kHz = FreqResult + FE_367cab_GetDerotFreq(DeviceMap, IOHandle, pIntParams->AdcClock_Hz) - pIntParams->AdcClock_Hz / 4000;
		}
		pIntParams->DemodResult.SymbolRate_Bds = FE_367cab_GetSymbolRate(DeviceMap, IOHandle, pIntParams->MasterClock_Hz);
		pIntParams->DemodResult.Locked = 1;
		/* ChipSetField(pIntParams->hDemod, F367CAB_AGC_ACCUMRSTSEL ,7); */
	}
	else
	{
		switch (QAM_Lock)
		{
			case 1:
			{
				signalType = FE_367cab_NOAGC;
				break;
			}
			case 2:
			{
				signalType = FE_367cab_NOTIMING;
				break;
			}
			case 3:
			{
				signalType = FE_367cab_TIMINGOK;
				break;
			}
			case 4:
			{
				signalType = FE_367cab_NOCARRIER;
				break;
			}
			case 5:
			{
				signalType = FE_367cab_CARRIEROK;
				break;
			}
			case 7:
			{
				signalType = FE_367cab_NOBLIND;
				break;
			}
			case 8:
			{
				signalType = FE_367cab_BLINDOK;
				break;
			}
			case 10:
			{
				signalType = FE_367cab_NODEMOD;
				break;
			}
			case 11:
			{
				signalType = FE_367cab_DEMODOK;
				break;
			}
			case 12:
			{
				signalType = FE_367cab_DEMODOK;
				break;
			}
			case 13:
			{
				signalType = FE_367cab_NODEMOD;
				break;
			}
			case 14:
			{
				signalType = FE_367cab_NOBLIND;
				break;
			}
			case 15:
			{
				signalType = FE_367cab_NOSIGNAL;
				break;
			}
			default:
			{
				break;
			}
		}
	}
	/* Set the AGC control values to tracking values */
	ChipSetField_0367cab(DeviceMap, IOHandle, F367CAB_AGC_ACCUMRSTSEL, TrackAGCAccum);
	return signalType;
}

/***********************************************************************
	Name:   demod_stv0367cab_ScanFreq
 ***********************************************************************/
unsigned int demod_stv0367cab_ScanFreq(u8 Index)
{
	unsigned int             Error = YW_NO_ERROR;
	TUNER_ScanTaskParam_T      *Inst;
	u32            IOHandle;
	TUNER_IOREG_DeviceMap_t    *DeviceMap;
	FE_367cab_InternalParams_t pParams;
	u32                        ZigzagScan = 0;
	s32                        SearchRange_Hz_Tmp;

	Inst = TUNER_GetScanInfo(Index);
	IOHandle = Inst->DriverParam.Cab.DemodIOHandle;
	DeviceMap = &Inst->DriverParam.Cab.Demod_DeviceMap;

	FirstTimeBER[Index] = 1;

	memset(&pParams, 0, sizeof(FE_367cab_InternalParams_t));
	pParams.State = FE_367cab_NOTUNER;
	/*
	--- Modulation type is set to
	*/
	switch (Inst->DriverParam.Cab.Param.Modulation)
	{
		case YWTUNER_MOD_QAM_16:
		{
			pParams.Modulation = FE_CAB_MOD_QAM16;
			break;
		}
		case YWTUNER_MOD_QAM_32:
		{
			pParams.Modulation = FE_CAB_MOD_QAM32;
			break;
		}
		case YWTUNER_MOD_QAM_64:
		{
			pParams.Modulation = FE_CAB_MOD_QAM64;
			break;
		}
		case YWTUNER_MOD_QAM_128:
		{
			pParams.Modulation = FE_CAB_MOD_QAM128;
			break;
		}
		case YWTUNER_MOD_QAM_256:
		{
			pParams.Modulation = FE_CAB_MOD_QAM256;
			break;
		}
		default:
		{
			pParams.Modulation = FE_CAB_MOD_QAM64;
			break;
		}
	}
	pParams.Crystal_Hz = DeviceMap->RegExtClk;  // 30M
	pParams.SearchRange_Hz = 280000;  /* 280 kHz */
	pParams.SymbolRate_Bds = Inst->DriverParam.Cab.Param.SymbolRateB;
	pParams.Frequency_kHz  = Inst->DriverParam.Cab.Param.FreqKHz;
	pParams.AdcClock_Hz    = FE_367cab_GetADCFreq(DeviceMap, IOHandle, pParams.Crystal_Hz);
	pParams.MasterClock_Hz = FE_367cab_GetMclkFreq(DeviceMap, IOHandle, pParams.Crystal_Hz);
//	dprintk(70, "%s: Frequency ================= %d\n", __func__, pParams.Frequency_kHz);
//	dprintk(70, "%s: SymbolRate_Bds  =========== %d\n", __func__, pParams.SymbolRate_Bds);
//	dprintk(70, "%s: pParams.AdcClock_Hz  ====== %d\n", __func__, pParams.AdcClock_Hz);
//	dprintk(70, "%s: pParams.MasterClock_Hz ==== %d\n", __func__, pParams.MasterClock_Hz);

	if (Inst->DriverParam.Cab.TunerDriver.tuner_SetFreq != NULL)
	{
		Error = (Inst->DriverParam.Cab.TunerDriver.tuner_SetFreq)(Index, pParams.Frequency_kHz, NULL);
	}
	/* Sets the QAM size and all the related parameters */
	FE_367cab_SetQamSize(Inst, DeviceMap, IOHandle, pParams.Frequency_kHz, pParams.SymbolRate_Bds, pParams.Modulation);
	/* Sets the symbol and all the related parameters */
	FE_367cab_SetSymbolRate(DeviceMap, IOHandle, pParams.AdcClock_Hz, pParams.MasterClock_Hz, pParams.SymbolRate_Bds, pParams.Modulation);
	/* Zigzag Algorithm test */
	if (25 * pParams.SearchRange_Hz > pParams.SymbolRate_Bds)
	{
		pParams.SearchRange_Hz = -(int)(pParams.SymbolRate_Bds) / 25;
		ZigzagScan = 1;
	}

	/* Search algorithm launch, [-1.1*RangeOffset, +1.1*RangeOffset] scan */
	pParams.State = FE_367cab_Algo(Index, &pParams);
	SearchRange_Hz_Tmp = pParams.SearchRange_Hz; //lwj add
	if (ZigzagScan && (pParams.State != FE_367cab_DATAOK))
	{
		do
		{
#if 1  // lwj modify
			pParams.SearchRange_Hz = -pParams.SearchRange_Hz;

			if (pParams.SearchRange_Hz > 0)
			{
				pParams.DerotOffset_Hz = -pParams.DerotOffset_Hz + pParams.SearchRange_Hz;
			}
			else
			{
				pParams.DerotOffset_Hz = -pParams.DerotOffset_Hz;
			}
#endif
			/* Search algorithm launch, [-1.1*RangeOffset, +1.1*RangeOffset] scan */
			pParams.State = FE_367cab_Algo(Index, &pParams);
		}
		while (((pParams.DerotOffset_Hz + pParams.SearchRange_Hz) >= -(int)SearchRange_Hz_Tmp) && (pParams.State != FE_367cab_DATAOK));
	}
	/* check results */
	if ((pParams.State == FE_367cab_DATAOK) && (!Error))
	{
		/* update results */
//		dprintk(20, "Tuner is locked\n");
		Inst->Status = TUNER_INNER_STATUS_LOCKED;
#if 0
		pResult->Frequency_kHz = pIntParams->DemodResult.Frequency_kHz;
		pResult->SymbolRate_Bds = pIntParams->DemodResult.SymbolRate_Bds;
		pResult->SpectInv = pIntParams->DemodResult.SpectInv;
		pResult->Modulation = pIntParams->DemodResult.Modulation;
#endif
	}
	else
	{
		Inst->Status = TUNER_INNER_STATUS_UNLOCKED;
	}
	return Error;
}

/*****************************************************
 --FUNCTION  ::  demod_stv0367cab_SetStandby
 --ACTION    ::  Set demod STANDBY mode On/Off
 --PARAMS IN ::  Handle  ==> Front End Handle

 -PARAMS OUT::   NONE.
 --RETURN    ::  Error (if any)
 --***************************************************/
unsigned int demod_stv0367cab_SetStandby(u8 Handle)
{
	FE_LLA_Error_t          error = FE_LLA_NO_ERROR;
	TUNER_ScanTaskParam_T   *Inst = NULL;
	TUNER_IOREG_DeviceMap_t *DeviceMap = NULL;
	u32         IOHandle;
	u8                      StandbyOn = 1;

	Inst = TUNER_GetScanInfo(Handle);
	IOHandle = Inst->DriverParam.Cab.DemodIOHandle;
	DeviceMap = &Inst->DriverParam.Cab.Demod_DeviceMap;

	if (StandbyOn)
	{
		stv0367cab_Sleep(DeviceMap, IOHandle);
	}
	else
	{
		stv0367cab_Wake(DeviceMap, IOHandle);
	}
	return (error);
}
#endif
// vim:ts=4
