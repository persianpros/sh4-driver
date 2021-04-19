/****************************************************************************
 *
 * Panasonic MN88472 DVB-T(2)/C demodulator driver
 *
 * Version for Fulan Spark7162
 *
 * Supports Panasonic MN88472 with MaxLinear MxL301 or
 * MaxLinear MxL603 tuner
 *
 * Original code:
 * Copyright (C), 2013-2018, AV Frontier Tech. Co., Ltd.
 *
 */
#include <linux/kernel.h>  /* Kernel support */
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <linux/dvb/version.h>

#include "ywdefs.h"
#include "ywtuner_ext.h"
#include "tuner_def.h"
#include "ioarch.h"

#include "ioreg.h"
#include "tuner_interface.h"

#include "mxl_common.h"
#include "mxL_user_define.h"
#include "mxl301rf.h"
#include "tun_mxl301.h"

#include "dvb_frontend.h"
#include "mn88472.h"
#include "nim_mn88472_ext.h"

#include "mxl301.h"

// for 5V antenna output
#if defined(SPARK7162)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#  include <linux/stpio.h>
#else
#  include <linux/stm/pio.h>
#endif
#endif

extern int paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[mn88472_dvb] "

struct dvb_mn88472_fe_state
{
	struct nim_device              spark_nimdev;
	struct i2c_adapter             *i2c;
	struct dvb_frontend            frontend;
	TUNER_IOREG_DeviceMap_t        DeviceMap;
	struct dvb_frontend_parameters *p;
	struct stpio_pin               *fe_ter_5V_on_off;  // for 5V antenna output};
};

int mn88472_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	int iRet;
	struct dvb_mn88472_fe_state *state = fe->demodulator_priv;

	iRet = nim_mn88472_get_SNR(&(state->spark_nimdev), (u8 *)snr);
	*snr = *snr * 255 * 255 / 100;
	return iRet;
}

int mn88472_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct dvb_mn88472_fe_state *state = fe->demodulator_priv;

	return nim_mn88472_get_BER(&state->spark_nimdev, ber);
}

int mn88472_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	int iRet;
	u32 Strength;
	u32 *Intensity = &Strength;
	struct dvb_mn88472_fe_state *state = fe->demodulator_priv;

	iRet = nim_mn88472_get_AGC(&state->spark_nimdev, (u8 *)Intensity);

	if (*Intensity > 90)
	{
		*Intensity = 90;
	}
	if (*Intensity < 10)
	{
		*Intensity = *Intensity * 11 / 2;
	}
	else
	{
		*Intensity = *Intensity / 2 + 50;
	}
	if (*Intensity > 90)
	{
		*Intensity = 90;
	}
	*Intensity = *Intensity * 255 * 255 / 100;
//	dprintk(70, "%s: *Intensity = %d\n", __func__, *Intensity);
	*strength = (*Intensity);
	msleep(100);
	return iRet;
}

int mn88472_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	int j, iRet;
	struct dvb_mn88472_fe_state *state = fe->demodulator_priv;
	u8 bIsLocked;

//	dprintk(150, "%s >\n", __func__);
	for (j = 0; j < (MN88472_T2_TUNE_TIMEOUT / 50); j++)
	{
		msleep(50);
		iRet = nim_mn88472_get_lock(&state->spark_nimdev, &bIsLocked);
		if (bIsLocked)
		{
			break;
		}
	}
//	dprintk(70, "%s: bIsLocked = %d\n", __func__, bIsLocked);
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
	return 0;
}

int mn88472_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	*ucblocks = 0;
	return 0;
}

int dvb_mn88472_get_property(struct dvb_frontend *fe, struct dtv_property *tvp)
{
	//struct dvb_d0367_fe_ofdm_state* state = fe->demodulator_priv;

	/* get delivery system info */
	if (tvp->cmd == DTV_DELIVERY_SYSTEM)
	{
		switch (tvp->u.data)
		{
			case SYS_DVBT:
			case SYS_DVBT2:
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

int dvb_mn88472_fe_qam_get_property(struct dvb_frontend *fe, struct dtv_property *tvp)
{
	//struct dvb_d0367_fe_ofdm_state* state = fe->demodulator_priv;

	/* get delivery system info */
	if (tvp->cmd == DTV_DELIVERY_SYSTEM)
	{
		switch (tvp->u.data)
		{
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

int mn88472_set_frontend(struct dvb_frontend *fe, struct dvb_frontend_parameters *p)
{
	struct dvb_mn88472_fe_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *props = &fe->dtv_property_cache;

	dprintk(150, "%s >\n", __func__);
	dprintk(50, "Frequency: %d kHz, Bandwidth %d kHz, System %d, Stream ID 0x%02x\n",
		props->frequency / 1000, props->bandwidth_hz / 1000, props->delivery_system, props->stream_id);

	state->p = p;
	demod_mn88472_ScanFreq(p, &state->spark_nimdev, props->delivery_system, props->stream_id != NO_STREAM_ID_FILTER ? props->stream_id : 0);
	state->p = NULL;
	return 0;
}

int d6158earda_set_frontend(struct dvb_frontend *fe, struct dvb_frontend_parameters *p)
{
	struct dvb_mn88472_fe_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *props = &fe->dtv_property_cache;

	dprintk(150, "%s >\n", __func__);
	dprintk(50, "Frequency: %d kHz, Bandwidth %d kHz, System %d, Stream ID 0x%02x\n",
		props->frequency / 1000, props->bandwidth_hz / 1000, props->delivery_system, props->stream_id);

	state->p = p;
	demod_mn88472earda_ScanFreq(p, &state->spark_nimdev, props->delivery_system, props->stream_id != NO_STREAM_ID_FILTER ? props->stream_id : 0);
	state->p = NULL;
	return 0;
}

static int dvb_mn88472_set_voltage(struct dvb_frontend *fe, fe_sec_voltage_t voltage)
{
	struct dvb_mn88472_fe_state *state = fe->demodulator_priv;

	switch (voltage)
	{
		case SEC_VOLTAGE_OFF:
		{
			dprintk(20, "Set active antenna voltage off\n");
			stpio_set_pin(state->fe_ter_5V_on_off, 0);
			break;
		}
		case SEC_VOLTAGE_13:
		{
			dprintk(20, "Set active antenna voltage on\n");
			stpio_set_pin(state->fe_ter_5V_on_off, 1);
			break;
		}
		default:
		{
			dprintk(1, "%s: Error: unknown active antenna voltage type %d\n", __func__, voltage);
			break;
		}
	}
	return 0;
}

static struct dvb_frontend_ops dvb_mn88472_fe_ofdm_ops =
{
	.info =
	{
		.name               = "Panasonic MN88472 (DVB-T(2))",
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
		                    | FE_CAN_2G_MODULATION
		                    | FE_CAN_MULTISTREAM
	},
	.init                   = NULL,
	.release                = NULL,
	.sleep                  = NULL,
	.set_frontend           = mn88472_set_frontend,
	.get_frontend           = NULL,
	.read_ber               = mn88472_read_ber,
	.read_snr               = mn88472_read_snr,
	.read_signal_strength   = mn88472_read_signal_strength,
	.read_status            = mn88472_read_status,
	.read_ucblocks          = mn88472_read_ucblocks,
	.i2c_gate_ctrl          = NULL,
#if (DVB_API_VERSION < 5)
	.get_info               = NULL,
#else
	.get_property           = dvb_mn88472_get_property,
#endif
	.set_voltage            = dvb_mn88472_set_voltage  // for active antenna
};

static struct dvb_frontend_ops dvb_mn88472_fe_qam_ops =
{

	.info =
	{
		.name               = "Panasonic MN88472 (DVB-C)",
		.type               = FE_QAM,
		.frequency_stepsize = 62500,
		.frequency_min      = 51000000,
		.frequency_max      = 858000000,
		.symbol_rate_min    = (57840000 / 2) / 64,  /* SACLK/64 == (XIN/2)/64 */
		.symbol_rate_max    = (57840000 / 2) / 4,   /* SACLK/4 */
		.caps               = FE_CAN_QAM_16
		                    | FE_CAN_QAM_32
		                    | FE_CAN_QAM_64
		                    | FE_CAN_QAM_128
		                    | FE_CAN_QAM_256
		                    | FE_CAN_FEC_AUTO
		                    | FE_CAN_INVERSION_AUTO
	},
	.init                   = NULL,
	.release                = NULL,
	.sleep                  = NULL,
	.set_frontend           = mn88472_set_frontend,
	.get_frontend           = NULL,
	.read_ber               = mn88472_read_ber,
	.read_snr               = mn88472_read_snr,
	.read_signal_strength   = mn88472_read_signal_strength,
	.read_status            = mn88472_read_status,
	.read_ucblocks          = mn88472_read_ucblocks,
	.i2c_gate_ctrl          = NULL,
#if (DVB_API_VERSION < 5)
	.get_info               = NULL,
#else
	.get_property           = dvb_mn88472_fe_qam_get_property,
#endif
};

// Attach Sharp T2/C frontend
struct dvb_frontend *dvb_mn88472_attach(struct i2c_adapter *i2c, u8 system)
{
	struct dvb_mn88472_fe_state *state = NULL;
	struct nim_mn88472_private *priv;
	int ret;
	struct COFDM_TUNER_CONFIG_API Tuner_API;

//	dprintk(150, "%s >\n", __func__);
	/* allocate memory for the internal state */
	state = kzalloc(sizeof(struct dvb_mn88472_fe_state), GFP_KERNEL);
	if (state == NULL)
	{
		goto error;
	}
	priv = (PNIM_MN88472_PRIVATE)kmalloc(sizeof(struct nim_mn88472_private), GFP_KERNEL);
	if (NULL == priv)
	{
		goto error;
	}
	/* create dvb_frontend */
	if (system == DEMOD_BANK_T2) // dvb-t
	{
		dprintk(20, "Attaching MN88472 demodulator in DVB-T(2) mode\n");
		memcpy(&state->frontend.ops, &dvb_mn88472_fe_ofdm_ops, sizeof(struct dvb_frontend_ops));
	}
	else if (system == DEMOD_BANK_C) //dvb-c
	{
		dprintk(20, "Attaching MN88472 demodulator in DVB-C mode\n");
		memcpy(&state->frontend.ops, &dvb_mn88472_fe_qam_ops, sizeof(struct dvb_frontend_ops));
	}
	state->frontend.demodulator_priv = state;
	state->i2c = i2c;
	state->DeviceMap.Timeout = IOREG_DEFAULT_TIMEOUT;
	state->DeviceMap.Registers = 0;  // STV0367ter_NBREGS;
	state->DeviceMap.Fields = 0;  // STV0367ter_NBFIELDS;
	state->DeviceMap.Mode = IOREG_MODE_SUBADR_16;
	state->DeviceMap.RegExtClk = 27000000;  // Demod External Crystal_HZ
	state->DeviceMap.RegMap = (TUNER_IOREG_Register_t *)kzalloc(state->DeviceMap.Registers * sizeof(TUNER_IOREG_Register_t), GFP_KERNEL);
	state->DeviceMap.priv = (void *)state;
	state->spark_nimdev.priv = priv;
	state->spark_nimdev.base_addr = MN88472_T2_ADDR;
	priv->i2c_adap = i2c;
	priv->i2c_addr[0] = MN88472_T_ADDR;
	priv->i2c_addr[1] = MN88472_T2_ADDR;
	priv->i2c_addr[2] = MN88472_C_ADDR;
	priv->if_freq = DMD_E_IF_5000KHZ;
	priv->flag_id = OSAL_INVALID_ID;
	priv->i2c_mutex_id = OSAL_INVALID_ID;
	priv->system = system;  // T2 C
	priv->first_tune_t2 = 1;
	priv->tuner_id = 2;
	if (tuner_mxl301_identify((u32 *)i2c) != YW_NO_ERROR)
	{
//		dprintk(1, "%s: Cannot identify MxL301 tuner\n", __func__);
		kfree(priv);
		kfree(state);
		return NULL;
	}
	dprintk(20, "Found MaxLinear MxL301 tuner\n");
	msleep(50);
	memset(&Tuner_API, 0, sizeof(struct COFDM_TUNER_CONFIG_API));
	Tuner_API.nim_Tuner_Init = tun_mxl301_init;
	Tuner_API.nim_Tuner_Status = tun_mxl301_status;
	Tuner_API.nim_Tuner_Control = tun_mxl301_control;
	Tuner_API.tune_t2_first = 1;  // when demod_mn88472_ScanFreq, param.priv_param >DEMO_DVBT2, tuner tunes T2 then T
	Tuner_API.tuner_config.demo_type = PANASONIC_DEMODULATOR;
	Tuner_API.tuner_config.cTuner_Base_Addr = 0xC2;
	Tuner_API.tuner_config.i2c_adap = (u32 *)i2c;
	if (NULL != Tuner_API.nim_Tuner_Init)
	{
		if (SUCCESS == Tuner_API.nim_Tuner_Init(&priv->tuner_id, &Tuner_API.tuner_config))
		{
			priv->tc.nim_Tuner_Init = Tuner_API.nim_Tuner_Init;
			priv->tc.nim_Tuner_Control = Tuner_API.nim_Tuner_Control;
			priv->tc.nim_Tuner_Status = Tuner_API.nim_Tuner_Status;
			memcpy(&priv->tc.tuner_config, &Tuner_API.tuner_config, sizeof(struct COFDM_TUNER_CONFIG_EXT));
		}
	}
	state->spark_nimdev.get_lock = nim_mn88472_get_lock;
	state->spark_nimdev.get_AGC = nim_mn88472_get_AGC_301;
	ret = nim_mn88472_open(&state->spark_nimdev);
	// allocate/initialize PIO pin for 5V active antenna
#if defined(SPARK7162)
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
	dprintk(20, "5V active antenna power switched off\n");
#endif
//	dprintk(150, "%s:[%d], open result=%d \n", __func__, __LINE__, ret);
	/* Setup init work mode */
	return &state->frontend;

error:
	kfree(state);
	return NULL;
}

// Attach Eardatek T2/C frontend
struct dvb_frontend *dvb_mn88472earda_attach(struct i2c_adapter *i2c, u8 system)
{
	struct dvb_mn88472_fe_state *state = NULL;
	struct nim_mn88472_private *priv;
	int ret;
	struct COFDM_TUNER_CONFIG_API Tuner_API;

//	dprintk(150, "%s >\n", __func__);
	/* allocate memory for the internal state */
	state = kzalloc(sizeof(struct dvb_mn88472_fe_state), GFP_KERNEL);
	if (state == NULL)
	{
		goto error;
	}
	priv = (PNIM_MN88472_PRIVATE)kmalloc(sizeof(struct nim_mn88472_private), GFP_KERNEL);
	if (NULL == priv)
	{
		goto error;
	}
	memset(priv, 0, sizeof(struct nim_mn88472_private));

	/* create dvb_frontend */
	if (system == DEMOD_BANK_T2)  // dvb-t2
	{
		dprintk(20, "Attaching MN88472 demodulator (Eardatek frontend) in DVB-T(2) mode\n");
		memcpy(&state->frontend.ops, &dvb_mn88472_fe_ofdm_ops, sizeof(struct dvb_frontend_ops));
	}
	else if (system == DEMOD_BANK_C)  // dvb-c
	{
		dprintk(20, "Attaching MN88472 demodulator (Eardatek frontend) in DVB-C mode\n");
		memcpy(&state->frontend.ops, &dvb_mn88472_fe_qam_ops, sizeof(struct dvb_frontend_ops));
	}
	state->frontend.ops.set_frontend = d6158earda_set_frontend;
	state->frontend.demodulator_priv = state;
	state->i2c = i2c;
	state->DeviceMap.Timeout = IOREG_DEFAULT_TIMEOUT;
	state->DeviceMap.Registers = 0;  // STV0367ter_NBREGS;
	state->DeviceMap.Fields = 0;     // STV0367ter_NBFIELDS;
	state->DeviceMap.Mode = IOREG_MODE_SUBADR_16;
	state->DeviceMap.RegExtClk = 27000000;  // Demod External Crystal_HZ
	state->DeviceMap.RegMap = (TUNER_IOREG_Register_t *)kzalloc(state->DeviceMap.Registers * sizeof(TUNER_IOREG_Register_t), GFP_KERNEL);
	state->DeviceMap.priv = (void *)state;
	state->spark_nimdev.priv = priv;
	state->spark_nimdev.base_addr = MN88472_T2_ADDR;
	priv->i2c_adap = i2c;
	priv->i2c_addr[0] = MN88472_T_ADDR;
	priv->i2c_addr[1] = MN88472_T2_ADDR;
	priv->i2c_addr[2] = MN88472_C_ADDR;
	priv->if_freq = DMD_E_IF_5000KHZ;
	priv->flag_id = OSAL_INVALID_ID;
	priv->i2c_mutex_id = OSAL_INVALID_ID;
	priv->system = system;  // T2/C
	priv->first_tune_t2 = 1;
	priv->tuner_id = 2;
	msleep(50);
	nim_config_EARDATEK11658(&Tuner_API, 0, 0);
	Tuner_API.tuner_config.i2c_adap = (u32 *)i2c;
	memcpy((void *) &(priv->tc), (void *)&Tuner_API, sizeof(struct COFDM_TUNER_CONFIG_API));
	if (NULL != Tuner_API.nim_Tuner_Init)
	{
		if (SUCCESS == Tuner_API.nim_Tuner_Init(&priv->tuner_id, &Tuner_API.tuner_config))
		{
			DEM_WRITE_READ_TUNER ThroughMode;
			dprintk(20, "Found MaxLinear MxL603 tuner\n");
			priv->tc.nim_Tuner_Init = Tuner_API.nim_Tuner_Init;
			priv->tc.nim_Tuner_Control = Tuner_API.nim_Tuner_Control;
			priv->tc.nim_Tuner_Status = Tuner_API.nim_Tuner_Status;
			memcpy(&priv->tc.tuner_config, &Tuner_API.tuner_config, sizeof(struct COFDM_TUNER_CONFIG_EXT));
			ThroughMode.nim_dev_priv = state->spark_nimdev.priv;
			ThroughMode.Dem_Write_Read_Tuner = DMD_TCB_WriteRead;
			nim_mn88472_ioctl_earda(&state->spark_nimdev, NIM_TUNER_SET_THROUGH_MODE, (u32)&ThroughMode);
		}
	}
	state->spark_nimdev.get_lock = nim_mn88472_get_lock;
	state->spark_nimdev.get_AGC = nim_mn88472_get_AGC_603;
	ret = nim_mn88472_open(&state->spark_nimdev);

	// allocate/initialize PIO pin for 5V active antenna
#if defined(SPARK7162)
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
	dprintk(20, "5V active antenna power switched off\n");
#endif
//	dprintk(150, "%s:[%d], open result=%d \n", __func__, __LINE__, ret);
	/* Setup init work mode */
//	dprintk(150, "%s < OK\n", __func__);
	return &state->frontend;

error:
	kfree(state);
//	dprintk(150, "%s < Error\n", __func__);
	return NULL;
}

/*****************************************************************
 *
 * MaxLinear tuner stuff
 *
 */
struct MxL301_state
{
	struct dvb_frontend *fe;
	struct i2c_adapter *i2c;
	const struct MxL301_config *config;

	u32 frequency;
	u32 bandwidth;
};

static struct dvb_tuner_ops mxlx0x_ops =
{
	.set_params = NULL,
	.release    = NULL,
};

struct dvb_frontend *mxlx0x_attach(struct dvb_frontend *fe, const struct MxL301_config *config, struct i2c_adapter *i2c)
{
	struct MxL301_state *state = NULL;
	struct dvb_tuner_info *info;

	state = kzalloc(sizeof(struct MxL301_state), GFP_KERNEL);

	if (state == NULL)
	{
		goto exit;
	}
	state->config = config;
	state->i2c = i2c;
	state->fe = fe;
	fe->tuner_priv = state;
	fe->ops.tuner_ops = mxlx0x_ops;
	info = &fe->ops.tuner_ops.info;
	memcpy(info->name, config->name, sizeof(config->name));
	dprintk(20, "Attaching %s tuner\n", info->name);
	return fe;

exit:
	kfree(state);
	info = &fe->ops.tuner_ops.info;
	dprintk(1, "%s: Error attaching %s tuner\n", __func__, info->name);
	return NULL;
}

// add by yabin
/***********************************************************************
  Name: tuner_mxl301_identify
 **********************************************************************/
unsigned int tuner_mxl301_identify(u32 *i2c_adap)
{
	unsigned int ret = YW_NO_ERROR;
	unsigned char ucData = 0;
	u8 value[2] = {0};
	u8 reg = 0xEC;
	u8 mask = 0x7F;
	u8 data = 0x53;
	u8 cmd[3];
	u8 cTuner_Base_Addr = 0xC2;

	// tuner soft reset
	cmd[0] = cTuner_Base_Addr;
	cmd[1] = 0xFF;
	ret |= I2C_ReadWrite(i2c_adap, TUNER_IO_SA_WRITE, 0XF7, cmd, 2, 100);
	msleep(20);
	// reset end

	ret |= I2C_ReadWrite(i2c_adap, TUNER_IO_SA_READ, reg, value, 1, 100);
	value[0] |= mask & data;
	value[0] &= (mask ^ 0xff) | data;
	ret |= I2C_ReadWrite(i2c_adap, TUNER_IO_SA_WRITE, reg, value, 1, 100);
	cmd[0] = 0;
	ret |= I2C_ReadWrite(i2c_adap, TUNER_IO_SA_WRITE, 0XEE, cmd, 1, 100);
	cmd[0] = cTuner_Base_Addr;
	cmd[1] = 0xFB;
	cmd[2] = 0x17;
	ret |= I2C_ReadWrite(i2c_adap, TUNER_IO_SA_WRITE, 0XF7, cmd, 3, 100);
	cmd[0] = cTuner_Base_Addr | 0x1;
	ret |= I2C_ReadWrite(i2c_adap, TUNER_IO_SA_WRITE, 0XF7, cmd, 1, 100);
	ret |= I2C_ReadWrite(i2c_adap, TUNER_IO_READ, 0XF7, cmd, 1, 100);
	ucData = cmd[0];
	if (YW_NO_ERROR != ret || 0x09 != (u8)(ucData & 0x0F))
	{
		return YWHAL_ERROR_UNKNOWN_DEVICE;
	}
	return ret;
}
// vim:ts=4
