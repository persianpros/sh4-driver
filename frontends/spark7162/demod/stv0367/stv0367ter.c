/****************************************************************************
 *
 * STM STV0367 DVB-T/C demodulator driver, DVB-T mode
 *
 * Version for Fulan Spark7162
 *
 * Note: actual device is part of the STi7162 SoC
 *
 * Original code:
 * Copyright (C), ?
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 * GNU General Public License for more details.
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/slab.h>

#include <linux/dvb/version.h>

#include "ywdefs.h"

// for 5V antenna output
#if defined(SPARK7162)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#  include <linux/stpio.h>
#else
#  include <linux/stm/pio.h>
#endif
#include <linux/gpio.h>
#include <linux/stm/gpio.h>
#endif

#include "ywtuner_ext.h"
#include "tuner_def.h"

#include "ioarch.h"
#include "ioreg.h"
#include "tuner_interface.h"

#include "stv0367ter_init.h"
#include "chip_0367ter.h"
#include "stv0367ter_drv.h"
#include "stv0367_ter.h"

#include "dvb_frontend.h"
#include "stv0367.h"

extern int paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[stv0367ter] "

struct dvb_stv0367_fe_ofdm_state
{
	struct i2c_adapter             *i2c;
	struct dvb_frontend            frontend;
	u32                            IOHandle;
	TUNER_IOREG_DeviceMap_t        DeviceMap;
	struct dvb_frontend_parameters *p;
	struct stpio_pin               *fe_ter_5V_on_off;  // for 5V antenna output
};

u32 stv0367ter_ScanFreq(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle);

static int dvb_stv0367_fe_ofdm_read_status(struct dvb_frontend *fe, fe_status_t *status)
{
	struct dvb_stv0367_fe_ofdm_state *state = fe->demodulator_priv;
	//int iTunerLock = 0;
	BOOL bIsLocked;

	dprintk(100, "%s >\n", __func__);

#if 0
	if (fe->ops.tuner_ops.get_status)
	{
		if (fe->ops.tuner_ops.get_status(fe, &iTunerLock) < 0)
		{
			printk("1. Tuner get_status err\n");
		}
	}
	if (iTunerLock)
	{
		dprintk(20, "Tuner phase locked\n");
	}
	else
	{
		dprintk(20, "Tuner unlocked\n");
	}
	if (fe->ops.i2c_gate_ctrl)
	{
		if (fe->ops.i2c_gate_ctrl(fe, 0) < 0)
		{
			goto exit;
		}
	}
#endif /* 0 */
	{
		bIsLocked = FE_367ter_lock(&state->DeviceMap, state->IOHandle);
		dprintk(20, "Frontend is %slocked\n", (bIsLocked ? "" : "not "));
	}
	if (bIsLocked)
	{
		*status = FE_HAS_SIGNAL
		        | FE_HAS_CARRIER
		        | FE_HAS_VITERBI
		        | FE_HAS_SYNC
		        | FE_HAS_LOCK;
	}
	else
	{
		*status = 0;
	}
#if 0
	{
		u32 Quality = 0;
		u32 Intensity = 0;
		u32 Ber = 0;
		FE_STV0367TER_GetSignalInfo(&state->DeviceMap, state->IOHandle, &Quality, &Intensity, &Ber);

		printk("Quality = %d, Intensity = %d, Ber = %d\n", Quality, Intensity, Ber);
	}
#endif /* 0 */
	dprintk(100, "%s <\n", __func__);
	return 0;
#if 0
exit:
	printk(KERN_ERR "%s: dvb_stv0367_fe_ofdm_read_status Error\n", __func__);
	return -1;
#endif /* 0 */
}

static int dvb_stv0367_fe_ofdm_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct dvb_stv0367_fe_ofdm_state *state = fe->demodulator_priv;
	TUNER_IOREG_DeviceMap_t *DeviceMap;
	u32 IOHandle;

	DeviceMap = &state->DeviceMap;
	IOHandle = state->IOHandle;

	//dprintk(100, "%s >\n", __func__);
	*ber = FE_367ter_GetErrors(DeviceMap, IOHandle);
	//dprintk(100, "%s <\n", __func__);
	return 0;
}

static int dvb_stv0367_fe_ofdm_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct dvb_stv0367_fe_ofdm_state *state = fe->demodulator_priv;
	TUNER_IOREG_DeviceMap_t *DeviceMap;
	u32 IOHandle;

//	dprintk(100, "%s >\n", __func__);
	DeviceMap = &state->DeviceMap;
	IOHandle = state->IOHandle;
	*strength = FE_STV0367TER_GetPower(DeviceMap, IOHandle);
//	dprintk(100, "%s <\n", __func__);
	return 0;
}

static int dvb_stv0367_fe_ofdm_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct dvb_stv0367_fe_ofdm_state *state = fe->demodulator_priv;
	TUNER_IOREG_DeviceMap_t *DeviceMap;
	u32 IOHandle;

//	dprintk(100, "%s >\n", __func__);
	DeviceMap = &state->DeviceMap;
	IOHandle = state->IOHandle;

	*snr = FE_STV0367TER_GetSnr(DeviceMap, IOHandle);
//	dprintk(100, "%s <\n", __func__);
	return 0;
}

static int dvb_stv0367_fe_ofdm_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	*ucblocks = 0;
	return 0;
}

static int dvb_stv0367_fe_ofdm_get_frontend(struct dvb_frontend *fe, struct dvb_frontend_parameters *p)
{
	return 0;
}

static int dvb_stv0367_fe_ofdm_set_frontend(struct dvb_frontend *fe, struct dvb_frontend_parameters *p)
{
	struct dvb_stv0367_fe_ofdm_state *state = fe->demodulator_priv;
	TUNER_IOREG_DeviceMap_t *DeviceMap;
	u32 IOHandle;

	DeviceMap = &state->DeviceMap;
	IOHandle = state->IOHandle;

	state->p = p;

	stv0367ter_ScanFreq(DeviceMap, IOHandle);
#if 0
	BOOL bIsLocked;

	bIsLocked = FE_367ter_lock(&state->DeviceMap, state->IOHandle);
	dprintk(70, "bIsLocked = %d\n", bIsLocked);
#endif
	state->p = NULL;
	return 0;
}

static int dvb_stv0367_fe_ofdm_sleep(struct dvb_frontend *fe)
{
	struct dvb_stv0367_fe_ofdm_state *state = fe->demodulator_priv;
	TUNER_IOREG_DeviceMap_t *DeviceMap;
	u32 IOHandle;

	DeviceMap = &state->DeviceMap;
	IOHandle = state->IOHandle;

	ChipSetField_0367ter(DeviceMap, IOHandle, F367_STDBY, 1);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367_STDBY_FEC, 1);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367_STDBY_CORE, 1);
	return 0;
}

static int dvb_stv0367_fe_ofdm_init(struct dvb_frontend *fe)
{
	struct dvb_stv0367_fe_ofdm_state *state = fe->demodulator_priv;
	TUNER_IOREG_DeviceMap_t *DeviceMap;
	u32 IOHandle;

	DeviceMap = &state->DeviceMap;
	IOHandle = state->IOHandle;

	stv0367ter_init(DeviceMap, IOHandle, TUNER_TUNER_SN761640);
#if 0
	stv0367ter_ScanFreq(&state->DeviceMap, state->IOHandle);
	{
		u32 Quality = 0;
		u32 Intensity = 0;
		u32 Ber = 0;
		FE_STV0367TER_GetSignalInfo(&state->DeviceMap, state->IOHandle, &Quality, &Intensity, &Ber);
		dprintk(150, "%s: Quality = %d, Intensity = %d, Ber = %d\n", __func__, Quality, Intensity, Ber);
	}
	{
		BOOL bIsLocked;
		bIsLocked = FE_367ter_lock(&state->DeviceMap, state->IOHandle);
		dprintk(150, "%s: bIsLocked = %d\n", __func__, bIsLocked);
	}
#endif
	return 0;
}

static void dvb_stv0367_fe_ofdm_release(struct dvb_frontend *fe)
{
	struct dvb_stv0367_fe_ofdm_state *state = fe->demodulator_priv;
	TUNER_IOREG_DeviceMap_t *DeviceMap;
	u32 IOHandle;

	DeviceMap = &state->DeviceMap;
	IOHandle = state->IOHandle;

	if (DeviceMap->RegMap)
	{
		kfree(DeviceMap->RegMap);
	}
	kfree(state);
}

#if 1  // device is in SoC
static int dvb_stv0367_fe_ofdm_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct dvb_stv0367_fe_ofdm_state *state = fe->demodulator_priv;
	TUNER_IOREG_DeviceMap_t *DeviceMap;
	u32 IOHandle;

	DeviceMap = &state->DeviceMap;
	IOHandle = state->IOHandle;

	if (enable)
	{
		return ChipSetField_0367ter(DeviceMap, IOHandle, F367_I2CT_ON, 1);
	}
	else
	{
		return ChipSetField_0367ter(DeviceMap, IOHandle, F367_I2CT_ON, 0);
	}
	return 0;
}
#else  // device is external
static int dvb_stv0367_fe_ofdm_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	u16 reg_index = 0xF001;
	u8 data;

	if (enable)
	{
		dvb_stv0367_fe_ofdm_i2c_read(fe, reg_index, &data, 1);
		data |= 0xa2;
		dvb_stv0367_fe_ofdm_i2c_write(fe, reg_index, &data, 1);
	}
	else
	{
		dvb_stv0367_fe_ofdm_i2c_read(fe, reg_index, &data, 1);
		data &= 0x00;
		dvb_stv0367_fe_ofdm_i2c_write(fe, reg_index, &data, 1);
	}
	return 0;
}
#endif

#if (DVB_API_VERSION < 5)
static int dvb_stv0367_fe_ofdm_get_info(struct dvb_frontend *fe, struct dvbfe_info *fe_info)
{
	//struct dvb_stv0367_fe_ofdm_state* state = fe->demodulator_priv;
	/* get delivery system info */
	if (fe_info->delivery == DVBFE_DELSYS_DVBT)
	{
		return 0;
	}
	else
	{
		return -EINVAL;
	}
	return 0;
}
#else
static int dvb_stv0367_fe_ofdm_get_property(struct dvb_frontend *fe, struct dtv_property *tvp)
{
	//struct dvb_stv0367_fe_ofdm_state *state = fe->demodulator_priv;

	/* get delivery system info */
	if (tvp->cmd == DTV_DELIVERY_SYSTEM)
	{
		switch (tvp->u.data)
		{
			case SYS_DVBT:
			case SYS_DVBC_ANNEX_AC:
			{
				break;
			}
			default:
			{
				return -EINVAL;
			}
		}
	}
	return 0;
}
#endif

static int dvb_stv0367_fe_ofdm_set_voltage(struct dvb_frontend *fe, fe_sec_voltage_t voltage)
{
	struct dvb_stv0367_fe_ofdm_state *state = fe->demodulator_priv;

	switch (voltage)
	{
		case SEC_VOLTAGE_OFF:
		{
			dprintk(20, "Switch active antenna 5V supply voltage off\n");
			stpio_set_pin(state->fe_ter_5V_on_off, 0);
			break;
		}
		case SEC_VOLTAGE_13:
		{
			dprintk(20, "Switch active antenna 5V supply voltage on\n");
			stpio_set_pin(state->fe_ter_5V_on_off, 1);
			break;
		}
		default:
		{
			dprintk(1, "%s: Error: unknown active antenna voltage: %d\n", __func__, voltage);
			break;
		}
	}
	return 0;
}

static struct dvb_frontend_ops dvb_stv0367_fe_ofdm_ops =
{
	.info =
	{
		.name               = "STM STV0367 (DVB-T)",
		.type               = FE_OFDM,
		.frequency_min      =  47000000,
		.frequency_max      = 863250000,
		.frequency_stepsize = 62500,
		.caps               = FE_CAN_FEC_1_2
		                    | FE_CAN_FEC_2_3
		                    | FE_CAN_FEC_3_4
		                    | FE_CAN_FEC_4_5
		                    | FE_CAN_FEC_5_6
		                    | FE_CAN_FEC_6_7
		                    | FE_CAN_FEC_7_8
		                    | FE_CAN_FEC_8_9
		                    | FE_CAN_FEC_AUTO
		                    | FE_CAN_QAM_16
		                    | FE_CAN_QAM_64
		                    | FE_CAN_QAM_AUTO
		                    | FE_CAN_TRANSMISSION_MODE_AUTO
		                    | FE_CAN_GUARD_INTERVAL_AUTO
		                    | FE_CAN_HIERARCHY_AUTO
	},
	.release                = dvb_stv0367_fe_ofdm_release,
	.init                   = dvb_stv0367_fe_ofdm_init,
	.sleep                  = dvb_stv0367_fe_ofdm_sleep,
	.set_frontend           = dvb_stv0367_fe_ofdm_set_frontend,
	.get_frontend           = dvb_stv0367_fe_ofdm_get_frontend,
	.read_status            = dvb_stv0367_fe_ofdm_read_status,
	.read_ber               = dvb_stv0367_fe_ofdm_read_ber,
	.read_signal_strength   = dvb_stv0367_fe_ofdm_read_signal_strength,
	.read_snr               = dvb_stv0367_fe_ofdm_read_snr,
	.read_ucblocks          = dvb_stv0367_fe_ofdm_read_ucblocks,
	.i2c_gate_ctrl          = dvb_stv0367_fe_ofdm_i2c_gate_ctrl,
#if (DVB_API_VERSION < 5)
	.get_info               = dvb_stv0367_fe_ofdm_get_info,
#else
	.get_property           = dvb_stv0367_fe_ofdm_get_property,
#endif
	.set_voltage            = dvb_stv0367_fe_ofdm_set_voltage  // for active antenna
};

void stv0367ter_TunerSetFreq(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle)
{
	struct dvb_stv0367_fe_ofdm_state *state = (struct dvb_stv0367_fe_ofdm_state *)DeviceMap->priv;
	struct dvb_frontend_parameters *p = state->p;
	struct dvb_frontend *fe = &state->frontend;

	if (fe->ops.tuner_ops.set_params)
	{
		fe->ops.tuner_ops.set_params(fe, p);
		if (fe->ops.i2c_gate_ctrl)
		{
			fe->ops.i2c_gate_ctrl(fe, 0);
		}
	}
}

FE_LLA_Error_t stv0367ter_Algo(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, FE_367ter_InternalParams_t *pParams, FE_TER_SearchResult_t *pResult)
{
	u32 InternalFreq = 0, temp = 0;
	FE_LLA_Error_t error = FE_LLA_NO_ERROR;

	if (pParams != NULL)
	{
		ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_CCS_ENABLE, 0);
		dprintk(150, "%s: pIntParams->IF_IQ_Mode == %d\n", __func__, pParams->IF_IQ_Mode);
		switch (pParams->IF_IQ_Mode)
		{
			case FE_TER_NORMAL_IF_TUNER: /* Normal IF mode */
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367_TUNER_BB, 0);
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_LONGPATH_IF, 0);
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_DEMUX_SWAP, 0);
				break;
			}
			case FE_TER_LONGPATH_IF_TUNER: /* Long IF mode */
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367_TUNER_BB, 0);
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_LONGPATH_IF, 1);
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_DEMUX_SWAP, 1);
				break;
			}
			case FE_TER_IQ_TUNER: /* IQ mode */
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367_TUNER_BB, 1);
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_PPM_INVSEL, 0);
				/*spectrum inversion hw detection off *db*/
				break;
			}
			default:
			{
				return FE_LLA_SEARCH_FAILED;
			}
		}
//		dprintk(150, "pIntParams->Inv == %d\n",pParams->Inv);
//		dprintk(150, "pIntParams->Sense == %d\n",pParams->Sense);
		if ((pParams->Inv == FE_TER_INVERSION_NONE))
		{
			if (pParams->IF_IQ_Mode == FE_TER_IQ_TUNER)
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_IQ_INVERT, 0);
				pParams->SpectrumInversion = FALSE;
			}
			else
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_INV_SPECTR, 0);
				pParams->SpectrumInversion = FALSE;
			}
		}
		else if (pParams->Inv == FE_TER_INVERSION)
		{
			if (pParams->IF_IQ_Mode == FE_TER_IQ_TUNER)
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_IQ_INVERT, 1);
				pParams->SpectrumInversion = TRUE;
			}
			else
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_INV_SPECTR, 1);
				pParams->SpectrumInversion = TRUE;
			}
		}
		else if ((pParams->Inv == FE_TER_INVERSION_AUTO) || ((pParams->Inv == FE_TER_INVERSION_UNK)/*&& (!pParams->first_lock)*/))
		{
			if (pParams->IF_IQ_Mode == FE_TER_IQ_TUNER)
			{
				if (pParams->Sense == 1)
				{
					ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_IQ_INVERT, 1);
					pParams->SpectrumInversion = TRUE;
				}
				else
				{
					ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_IQ_INVERT, 0);
					pParams->SpectrumInversion = FALSE;
				}
			}
			else
			{
				if (pParams->Sense == 1)
				{
					ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_INV_SPECTR, 1);
					pParams->SpectrumInversion = TRUE;
				}
				else
				{
					ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_INV_SPECTR, 0);
					pParams->SpectrumInversion = FALSE;
				}
			}
		}
//		dprintk(150, "%s: pIntParams->PreviousChannelBW == %d MHz\n", __func__, pParams->PreviousChannelBW);
//		dprintk(150, "%s: pIntParams->Crystal_Hz == %d Hz\n", __func__, pParams->Crystal_Hz);
		if ((pParams->IF_IQ_Mode != FE_TER_NORMAL_IF_TUNER)
		&&  (pParams->PreviousChannelBW != pParams->ChannelBW))
		{
			dprintk(50, "%s: Bandwidth changed from %d MHz to = %d Hz\n", __func__, pParams->PreviousChannelBW, pParams->ChannelBW);
			FE_367TER_AGC_IIR_LOCK_DETECTOR_SET(DeviceMap, IOHandle);
			/* set fine agc target to 180 for LPIF or IQ mode */
			/* set Q_AGCTarget */
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_SEL_IQNTAR, 1);
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_AUT_AGC_TARGET_MSB, 0xB);
			/* ChipSetField_0367ter(DeviceMap, IOHandle,AUT_AGC_TARGET_LSB,0x04); */
			/* set Q_AGCTarget */
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_SEL_IQNTAR, 0);
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_AUT_AGC_TARGET_MSB, 0xB);
			/* ChipSetField_0367ter(DeviceMap, IOHandle,AUT_AGC_TARGET_LSB,0x04); */
			if (!FE_367TER_IIR_FILTER_INIT(DeviceMap, IOHandle, pParams->ChannelBW, pParams->Crystal_Hz))
			{
				return FE_LLA_BAD_PARAMETER;
			}
			/* set IIR filter once for 6, 7 or 8MHz BW */
			pParams->PreviousChannelBW = pParams->ChannelBW;
			FE_367TER_AGC_IIR_RESET(DeviceMap, IOHandle);
		}
//		dprintk(70, "%s: pIntParams->Hierarchy == %d\n", __func__, pParams->Hierarchy);
		/*********Code Added For Hierarchical Modulation****************/
		if (pParams->Hierarchy == FE_TER_HIER_LOW_PRIO)
		{
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_BDI_LPSEL, 0x01);
		}
		else
		{
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_BDI_LPSEL, 0x00);
		}
		/*************************/
		InternalFreq = FE_367ter_GetMclkFreq(DeviceMap, IOHandle, pParams->Crystal_Hz) / 1000;
//		dprintk(150, "%s: InternalFreq = %d kHz\n", __func__, InternalFreq);
		temp = (int)((((pParams->ChannelBW * 64 * PowOf2(15) * 100) / (InternalFreq)) * 10) / 7);
//		dprintk(150, "%s: pParams->ChannelBW = %d MHz\n", __func__, pParams->ChannelBW);
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TRL_NOMRATE_LSB, temp % 2);
		temp = temp / 2;
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TRL_NOMRATE_HI, temp / 256);
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TRL_NOMRATE_LO, temp % 256);
		//ChipSetRegisters_0367ter(DeviceMap, IOHandle,R367TER_TRL_NOMRATE1,2);  // lwj
		ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_TRL_NOMRATE1, 1);
		ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_TRL_NOMRATE2, 1);
		//ChipGetRegisters_0367ter(DeviceMap, IOHandle,R367TER_TRL_CTL,3);//lwj
		ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_TRL_CTL, 1);
		ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_TRL_NOMRATE1, 1);
		ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_TRL_NOMRATE2, 1);
		temp = ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TRL_NOMRATE_HI) * 512
		     + ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TRL_NOMRATE_LO) * 2
		     + ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TRL_NOMRATE_LSB);
		temp = (int)((PowOf2(17) * pParams->ChannelBW * 1000) / (7 * (InternalFreq)));
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_GAIN_SRC_HI, temp / 256);
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_GAIN_SRC_LO, temp % 256);
		//ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_GAIN_SRC1,2);
		ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_GAIN_SRC1, 1);
		ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_GAIN_SRC2, 1);

		//ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_GAIN_SRC1,2);
		ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_GAIN_SRC1, 1);
		ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_GAIN_SRC2, 1);

		temp = ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_GAIN_SRC_HI) * 256
		     + ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_GAIN_SRC_LO);
		//if ( Inst->DriverParam.Ter.TunerType == TUNER_TUNER_SN761640)
		{
			temp = (int)((InternalFreq - 36166667 / 1000) * PowOf2(16) / (InternalFreq));  // IF freq
		}
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_INC_DEROT_HI, temp / 256);
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_INC_DEROT_LO, temp % 256);
		//ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_INC_DEROT1, 2);  // lwj change
		ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_INC_DEROT1, 1);
		ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_INC_DEROT2, 1);
		// pParams->EchoPos = pSearch->EchoPos;
		ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_LONG_ECHO, pParams->EchoPos);
		stv0367ter_TunerSetFreq(DeviceMap, IOHandle);
		/*********************************/
		if (FE_367ter_LockAlgo(DeviceMap, IOHandle, pParams) == FE_TER_LOCKOK)
		{
			pResult->Locked = TRUE;
			dprintk(70, "%s: Locked = Yes\n", __func__);
			pResult->SignalStatus = FE_TER_LOCKOK;
		} /* if(FE_367_Algo(pParams) == FE_TER_LOCKOK) */
		else
		{
			pResult->Locked = FALSE;
			dprintk(70, "%s: Search failed (Locked = No)\n", __func__);
			error = FE_LLA_SEARCH_FAILED;
		}
	}  /*if((void *)Handle != NULL)*/
	else
	{
		error = FE_LLA_BAD_PARAMETER;
	}
	dprintk(100, "%s: <\n", __func__);
	return error;
}

u32 stv0367ter_GetFrequency(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle)
{
	u32 Frequency = 722000;
	struct dvb_stv0367_fe_ofdm_state *state = NULL;
	struct dvb_frontend_parameters *p = NULL;

	IOHandle = IOHandle;

	state = (struct dvb_stv0367_fe_ofdm_state *)DeviceMap->priv;
	if (!state)
	{
		return Frequency;
	}
	p = state->p;
	if (!state)
	{
		return Frequency;
	}
	Frequency = p->frequency / 1000;
	dprintk(99, "%s: < Frequency = %d kHz\n", __func__, Frequency);
	return Frequency;
}

u32 stv0367ter_GetChannelBW(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle)
{
	u32 ChannelBW = 8;
	struct dvb_stv0367_fe_ofdm_state *state = NULL;
	struct dvb_frontend_parameters *p = NULL;

	IOHandle = IOHandle;
	state = (struct dvb_stv0367_fe_ofdm_state *)DeviceMap->priv;
	if (!state)
	{
		return ChannelBW;
	}
	p = state->p;
	if (!state)
	{
		return ChannelBW;
	}
	switch (p->u.ofdm.bandwidth)
	{
		case BANDWIDTH_8_MHZ:
		{
			ChannelBW = 8;
			break;
		}
		case BANDWIDTH_7_MHZ:
		{
			ChannelBW = 7;
			break;
		}
		case BANDWIDTH_6_MHZ:
		{
			ChannelBW = 6;
			break;
		}
		default:
		{
			ChannelBW = 8;
			break;
		}
	}
	dprintk(99, "%s: < Bandwidth = %d MHz\n", __func__, ChannelBW);
	return ChannelBW;
}

u32 stv0367ter_ScanFreq(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle)
{
	u32 Error = YW_NO_ERROR;
	/*u8 trials[2]; */
	s8 num_trials = 1;
	u8 index;
	FE_LLA_Error_t error = FE_LLA_NO_ERROR;
	u8 flag_spec_inv;
	u8 flag;
	u8 SenseTrials[2];
	u8 SenseTrialsAuto[2];
	FE_367ter_InternalParams_t pParams;
	FE_TER_SearchResult_t pResult;

	//question pParams->Inv
	memset(&pParams, 0, sizeof(FE_367ter_InternalParams_t));
	memset(&pResult, 0, sizeof(FE_TER_SearchResult_t));
#if 0 //question
	SenseTrials[0] = INV;
	SenseTrials[1] = NINV;
	SenseTrialsAuto[0] = INV;
	SenseTrialsAuto[1] = NINV;
#else
	SenseTrials[1] = INV;
	SenseTrials[0] = NINV;
	SenseTrialsAuto[1] = INV;
	SenseTrialsAuto[0] = NINV;
#endif
//	dprintk(70, "%s: Index = %d\n", __func__, Index);
	pParams.Frequency = stv0367ter_GetFrequency(DeviceMap, IOHandle);
	pParams.ChannelBW = stv0367ter_GetChannelBW(DeviceMap, IOHandle);
	dprintk(70, "%s: Frequency = %d kHz; ChannelBW = %d MHz\n", __func__, pParams.Frequency, pParams.ChannelBW);
	pParams.Crystal_Hz = DeviceMap->RegExtClk;
	pParams.IF_IQ_Mode = FE_TER_NORMAL_IF_TUNER;//FE_TER_IQ_TUNER; //most tuner is IF mode, stv4100 is I/Q mode
	pParams.Inv = FE_TER_INVERSION_AUTO; //FE_TER_INVERSION_AUTO
	pParams.Hierarchy = FE_TER_HIER_NONE;
	pParams.EchoPos = 0;
	pParams.first_lock = 0;
//	dprintk(70, "%s: in FE_367TER_Search () value of pParams.Hierarchy %d\n", __func__, pParams.Hierarchy );
	flag_spec_inv = 0;
	flag = ((pParams.Inv >> 1) & 1);
	switch (flag) /* sw spectrum inversion for LP_IF &IQ &normal IF * db *  */
	{
		case 0:  /* INV + INV_NONE */
		{
			if ((pParams.Inv == FE_TER_INVERSION_NONE) || (pParams.Inv == FE_TER_INVERSION))
			{
				num_trials = 1;
			}
			else  /* UNK */
			{
				num_trials = 2;
			}
			break;
		}
		case 1:  /* AUTO */
		{
			num_trials = 2;
			if ((pParams.first_lock) && (pParams.Inv == FE_TER_INVERSION_AUTO))
			{
				num_trials = 1;
			}
			break;
		}
		default:
		{
			return FE_TER_NOLOCK;
			break;
		}
	}
	pResult.SignalStatus = FE_TER_NOLOCK;
	index = 0;

	while (((index) < num_trials) && (pResult.SignalStatus != FE_TER_LOCKOK))
	{
		if (!pParams.first_lock)
		{
			if (pParams.Inv == FE_TER_INVERSION_UNK)
			{
				pParams.Sense = SenseTrials[index];
			}
			if (pParams.Inv == FE_TER_INVERSION_AUTO)
			{
				pParams.Sense = SenseTrialsAuto[index];
			}
		}
		error = stv0367ter_Algo(DeviceMap, IOHandle, &pParams, &pResult);
		if ((pResult.SignalStatus == FE_TER_LOCKOK)
		&&  (pParams.Inv == FE_TER_INVERSION_AUTO)
		&&  (index == 1))
		{
			SenseTrialsAuto[index] = SenseTrialsAuto[0];  /* invert spectrum sense */
			SenseTrialsAuto[(index + 1) % 2] = (SenseTrialsAuto[1] + 1) % 2;
		}
		index++;
	}
	return Error;
}

struct dvb_frontend *dvb_stv0367_ter_attach(struct i2c_adapter *i2c)
{
	struct dvb_stv0367_fe_ofdm_state *state = NULL;

	/* allocate memory for the internal state */
	state = kzalloc(sizeof(struct dvb_stv0367_fe_ofdm_state), GFP_KERNEL);
	if (state == NULL)
	{
		goto error;
	}

	/* create dvb_frontend */
	memcpy(&state->frontend.ops, &dvb_stv0367_fe_ofdm_ops, sizeof(struct dvb_frontend_ops));
	state->frontend.demodulator_priv = state;
	state->i2c = i2c;

	state->IOHandle = (u32)i2c;
	state->DeviceMap.Timeout   = IOREG_DEFAULT_TIMEOUT;
	state->DeviceMap.Registers = STV0367TER_NBREGS;
	state->DeviceMap.Fields    = STV0367TER_NBFIELDS;
	state->DeviceMap.Mode      = IOREG_MODE_SUBADR_16;
	state->DeviceMap.RegExtClk = 27000000;  // Demod External Crystal_HZ
	state->DeviceMap.RegMap    = (TUNER_IOREG_Register_t *)kzalloc(state->DeviceMap.Registers * sizeof(TUNER_IOREG_Register_t), GFP_KERNEL);
	state->DeviceMap.priv      = (void *)state;

	// allocate/initialize PIO pin for 5V active antenna
#if defined(SPARK7162)
#if 0
	int i, j;
	struct stpio_pin *test_pin;

	/* Temporary: try and find the pio's */
#define SLEEP 500
	for (i = 2; i < 17; i++)  // port counter
	{
		for (j = 0; j < 8; j++)  // pin counter
		{
			if (! ((i == 2 && j == 2) \
			    || (i == 2 && j == 3) \
			    || (i == 2 && j == 4) \
			    || (i == 2 && j == 5) \
			    || (i == 3 && j == 3) \
			    || (i == 3 && j == 4) \
			    || (i == 3 && j == 5) \
			    || (i == 3 && j == 6) \
			    || (i == 3 && j == 7) \
			    || (i == 4 && j == 0) \
			    || (i == 4 && j == 1))
			   )
			{
				test_pin = stpio_request_pin(i, j, "TEST_PIN", STPIO_OUT);
				if (!test_pin)
				{
					dprintk(1, "%s: Test pin PIO %d.%d request failed\n", __func__, i, j);
//					stpio_free_pin(test_pin);
//					goto esc_loop;
//					break;
				}
				else
				{
					stpio_set_pin(test_pin, 0);
					printk("PIO(%2d, %1d): ", i, j);
					printk("off ");
					mdelay(SLEEP);
					stpio_set_pin(test_pin, 1);
					printk("on ");
					mdelay(SLEEP);
					stpio_set_pin(test_pin, 0);
					printk("off ");
					mdelay(SLEEP);
					stpio_free_pin(test_pin);
					test_pin = stpio_request_pin(i, j, "TEST_PIN", STPIO_IN);
//					if (!test_pin)
//					{
//						dprintk(1, "%s: Test pin PIO %d.%d request failed\n", __func__, i, j);
//						stpio_free_pin(test_pin);
//						goto esc_loop;
//						break;
//					}
					printk("set to input\n");
					mdelay(3 * SLEEP);
				}
			}
			else
			{
				printk("PIO(%2d, %1d) skipped\n", i, j);
			}
		}
		printk("\n");
	}
#endif
	dprintk(20, "Allocating PIO pin for 5V output control\n");
	state->fe_ter_5V_on_off = stpio_request_pin(11, 6, "TER_ANT_5V", STPIO_OUT);
	if (!state->fe_ter_5V_on_off)
	{
		dprintk(1, "%s: 5V TER request pin failed\n", __func__);
		stpio_free_pin(state->fe_ter_5V_on_off);
		kfree(state);
		return NULL;
	}
	// switch 5V output off
	stpio_set_pin(state->fe_ter_5V_on_off, 0);
#endif
	dprintk(20, "Attaching STV0367 demodulator (DVB-T mode)\n");
	return &state->frontend;

error:
	kfree(state);
	return NULL;
}
EXPORT_SYMBOL(dvb_stv0367_ter_attach);

MODULE_DESCRIPTION("DVB STV0367 Frontend (DVB-T)");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");
// vim:ts=4
