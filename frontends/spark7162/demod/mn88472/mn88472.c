/*********************************************************************
 *
 * Panasonic MN88472 DVB-T(2)/C demodulator driver
 *
 * Version for Fulan Spark7162
 *
 * Original code:
 * Copyright (C), 2013-2018, AV Frontier Tech. Co., Ltd.
 *
 */
#include <linux/kernel.h>  /* Kernel support */
#include <linux/delay.h>
#include <linux/i2c.h>

#include <linux/dvb/version.h>

#include "ywdefs.h"
#include "ywtuner_ext.h"
#include "tuner_def.h"
#include "ioarch.h"

#include "ioreg.h"
#include "tuner_interface.h"

// tuner
#include "mxl_common.h"
#include "mxL_user_define.h"
#include "mxl301rf.h"
#include "tun_mxl301.h"
#include "dvb_frontend.h"
#include "mn88472.h"

#include "nim_mn88472_ext.h"

extern int paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[mn88472] "

/********************************************************************/

u32 demod_mn88472_Close(u8 Index)
{
	u32 Error = YW_NO_ERROR;
	TUNER_ScanTaskParam_T *Inst;
	struct nim_device *dev;
//	TUNER_IOARCH_CloseParams_t CloseParams;

	Inst = TUNER_GetScanInfo(Index);
	dev = (struct nim_device *)Inst->userdata;
//	TUNER_IOARCH_Close(dev->DemodIOHandle[2], &CloseParams);
	nim_mn88472_close(dev);
	if (YWTUNER_DELIVER_TER == Inst->Device)
	{
		Inst->DriverParam.Ter.DemodDriver.Demod_GetSignalInfo = NULL;
		Inst->DriverParam.Ter.DemodDriver.Demod_IsLocked = NULL;
		Inst->DriverParam.Ter.DemodDriver.Demod_repeat = NULL;
		Inst->DriverParam.Ter.DemodDriver.Demod_reset = NULL;
		Inst->DriverParam.Ter.DemodDriver.Demod_ScanFreq = NULL;
		Inst->DriverParam.Ter.Demod_DeviceMap.Error = YW_NO_ERROR;
	}
	else
	{
		Inst->DriverParam.Cab.DemodDriver.Demod_GetSignalInfo = NULL;
		Inst->DriverParam.Cab.DemodDriver.Demod_IsLocked = NULL;
		Inst->DriverParam.Cab.DemodDriver.Demod_repeat = NULL;
		Inst->DriverParam.Cab.DemodDriver.Demod_reset = NULL;
		Inst->DriverParam.Cab.DemodDriver.Demod_ScanFreq = NULL;
		Inst->DriverParam.Cab.Demod_DeviceMap.Error = YW_NO_ERROR;
	}
	return (Error);
}

/***********************************************************************
	Name: demod_mn88472_IsLocked
 ***********************************************************************/
u32 demod_mn88472_IsLocked(u8 Handle, BOOL *IsLocked)
{
	struct nim_device *dev = NULL;
	TUNER_ScanTaskParam_T *Inst = NULL;
	u8 lock = 0;

	*IsLocked = FALSE;
	Inst = TUNER_GetScanInfo(Handle);
	if (Inst == NULL)
	{
		return 1;
	}
	dev = (struct nim_device *)Inst->userdata;
	if (dev == NULL)
	{
		return 1;
	}
	nim_mn88472_get_lock(dev, &lock);
	*IsLocked = (BOOL)lock;
	return YW_NO_ERROR;
}

/***********************************************************************
	Name: demod_mn88472_identify
 ***********************************************************************/
int demod_mn88472_identify(struct i2c_adapter *i2c_adap, u8 ucID)
{
	int ret;
	u8 pucActualID = 0;
	u8 b0[] = { DMD_CHIPRD };

	struct i2c_msg msg[] =
	{
		{ .addr = MN88472_T2_ADDR >> 1, .flags = 0, .buf = b0, .len = 1 },
		{ .addr = MN88472_T2_ADDR >> 1, .flags = I2C_M_RD, .buf = &pucActualID, .len = 1}
	};

//	dprintk(100, "%s > (pI2c = 0x%08x)\n", __func__, (int)i2c_adap);
	ret = i2c_transfer(i2c_adap, msg, 2);
	if (ret == 2)
	{
//		dprintk(70, "%s: pucActualID = %d, expected : %d\n", __func__, pucActualID, ucID);
		if (pucActualID != ucID)
		{
			dprintk(1, "MN88472 demodulator not found\n");
			return YWHAL_ERROR_UNKNOWN_DEVICE;
		}
		else
		{
			dprintk(20, "MN88472 demodulator found\n");
			return YW_NO_ERROR;
		}
	}
	else
	{
		dprintk(1, "%s: Cannot find MN88472 demodulator, check your hardware!\n", __func__);
		return YWHAL_ERROR_UNKNOWN_DEVICE;
	}
	return YWHAL_ERROR_UNKNOWN_DEVICE;
}

#if 0
u32 demod_mn88472_Repeat(u32 DemodIOHandle, u32 TunerIOHandle, TUNER_IOARCH_Operation_t Operation,
			unsigned short SubAddr, unsigned char *Data, u32 TransferSize, u32 Timeout)
{
	(void)DemodIOHandle;
	(void)TunerIOHandle;
	(void)Operation;
	(void)SubAddr;
	(void)Data;
	(void)TransferSize;
	(void)Timeout;
	return 0;
}
#endif

u32 demod_mn88472_ScanFreq(struct dvb_frontend_parameters *p, struct nim_device *dev, u8 System, u8 plp_id)
{
	// struct nim_device *dev;
	int ret = 0;
	struct NIM_Channel_Change param;

	memset(&param, 0, sizeof(struct NIM_Channel_Change));
	if (dev == NULL)
	{
		return YWHAL_ERROR_BAD_PARAMETER;
	}
	if (SYS_DVBT2 == System)
	{
		param.priv_param = DEMOD_DVBT2;
		param.freq = p->frequency;
		param.PLP_id = plp_id;
		switch (p->u.ofdm.bandwidth)
		{
			case BANDWIDTH_6_MHZ:
			{
				param.bandwidth = MxL_BW_6MHz;
				break;
			}
			case BANDWIDTH_7_MHZ:
			{
				param.bandwidth = MxL_BW_7MHz;
				break;
			}
			case BANDWIDTH_8_MHZ:
			{
				param.bandwidth = MxL_BW_8MHz;
				break;
			}
			default:
			{
				return YWHAL_ERROR_BAD_PARAMETER;
			}
		}
	}
	else if (SYS_DVBT == System)
	{
		param.priv_param = DEMOD_DVBT;  // T2 T
		param.freq = p->frequency;
		switch (p->u.ofdm.bandwidth)
		{
			case BANDWIDTH_6_MHZ:
			{
				param.bandwidth = MxL_BW_6MHz;
				break;
			}
			case BANDWIDTH_7_MHZ:
			{
				param.bandwidth = MxL_BW_7MHz;
				break;
			}
			case BANDWIDTH_8_MHZ:
			{
				param.bandwidth = MxL_BW_8MHz;
				break;
			}
			default:
			{
				return YWHAL_ERROR_BAD_PARAMETER;
			}
		}
	}
	else if (SYS_DVBC_ANNEX_AC == System)
	{
		param.priv_param = DEMOD_DVBC;
		param.bandwidth = MxL_BW_8MHz;
		param.freq = p->frequency;
		switch (p->u.qam.modulation)
		{
			case QAM_16:
			case QAM_32:
			case QAM_64:
			case QAM_128:
			case QAM_256:
			{
				param.modulation = p->u.qam.modulation;
				break;
			}
			default:
			{
				return YWHAL_ERROR_BAD_PARAMETER;
			}
		}
	}
//	dprintk(150, "%s:[%d], dev = 0x%x\n", __func__, __LINE__, (u32)dev);
	ret = nim_mn88472_channel_change(dev, &param);
	return ret;
}

u32 demod_mn88472earda_ScanFreq(struct dvb_frontend_parameters *p, struct nim_device *dev, u8 System, u8 plp_id)
{
	// struct nim_device *dev;
	int ret = 0;
	struct NIM_Channel_Change param;

	memset(&param, 0, sizeof(struct NIM_Channel_Change));
	if (dev == NULL)
	{
		return YWHAL_ERROR_BAD_PARAMETER;
	}
	if (SYS_DVBT2 == System)
	{
		param.priv_param = DEMOD_DVBT2;
		param.freq = p->frequency;
		param.PLP_id = plp_id;
		switch (p->u.ofdm.bandwidth)
		{
			case BANDWIDTH_6_MHZ:
			{
				param.bandwidth = MxL_BW_6MHz;
				break;
			}
			case BANDWIDTH_7_MHZ:
			{
				param.bandwidth = MxL_BW_7MHz;
				break;
			}
			case BANDWIDTH_8_MHZ:
			{
				param.bandwidth = MxL_BW_8MHz;
				break;
			}
			default:
			{
				return YWHAL_ERROR_BAD_PARAMETER;
			}
		}
	}
	else if (SYS_DVBT == System)
	{
		param.priv_param = DEMOD_DVBT;  // T2 T
		param.freq = p->frequency;
		switch (p->u.ofdm.bandwidth)
		{
			case BANDWIDTH_6_MHZ:
			{
				param.bandwidth = MxL_BW_6MHz;
				break;
			}
			case BANDWIDTH_7_MHZ:
			{
				param.bandwidth = MxL_BW_7MHz;
				break;
			}
			case BANDWIDTH_8_MHZ:
			{
				param.bandwidth = MxL_BW_8MHz;
				break;
			}
			default:
			{
				return YWHAL_ERROR_BAD_PARAMETER;
			}
		}
	}
	else if (SYS_DVBC_ANNEX_AC == System)
	{
		param.priv_param = DEMOD_DVBC;
		param.bandwidth = MxL_BW_8MHz;
		param.freq = p->frequency;
		switch (p->u.qam.modulation)
		{
			case QAM_16:
			case QAM_32:
			case QAM_64:
			case QAM_128:
			case QAM_256:
			{
				param.modulation = p->u.qam.modulation;
				break;
			}
			default:
			{
				return YWHAL_ERROR_BAD_PARAMETER;
			}
		}
	}
//	dprintk(150, "%s:[%d], dev = 0x%x\n", __func__, __LINE__, (u32)dev);
	ret = nim_mn88472_channel_change_earda(dev, &param);
	return ret;
}

u32 demod_mn88472_Reset(u8 Index)
{
	u32 Error = YW_NO_ERROR;

	(void)Index;

#if 0
	TUNER_ScanTaskParam_T *Inst = NULL;
	struct nim_device *dev;

	Inst = TUNER_GetScanInfo(Index);
	dev = (struct nim_device *)Inst->userdata;
#endif
	return Error;
}

u32 demod_mn88472_GetSignalInfo(u8 Handle, u32 *Quality, u32 *Intensity, u32 *Ber)
{
	u32 Error = YW_NO_ERROR;
	TUNER_ScanTaskParam_T *Inst = NULL;
	struct nim_device *dev;

	Inst = TUNER_GetScanInfo(Handle);
	dev = (struct nim_device *)Inst->userdata;
	*Quality = 0;
	*Intensity = 0;
	*Ber = 0;
	/* Read noise estimations for C/N and BER */
	nim_mn88472_get_BER(dev, Ber);
	nim_mn88472_get_AGC(dev, (u8 *)Intensity);  // level
	dprintk(70, "%s: Intensity = %d\n", __func__, *Intensity);
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
	nim_mn88472_get_SNR(dev, (u8 *)Quality);  // quality
	dprintk(70, "%s: QualityValue = %d\n", __func__, *Quality);
	if (*Quality < 30)
	{
		*Quality = *Quality * 7 / 3;
	}
	else
	{
		*Quality = *Quality / 3 + 60;
	}
	if (*Quality > 90)
	{
		*Quality = 90;
	}
	return (Error);
}

void nim_config_EARDATEK11658(struct COFDM_TUNER_CONFIG_API *Tuner_API_T, u32 i2c_id, u8 idx)
{
	memset(Tuner_API_T, 0x0, sizeof(struct COFDM_TUNER_CONFIG_API));

	Tuner_API_T->nim_Tuner_Init = tun_mxl603_init;
	Tuner_API_T->nim_Tuner_Control = tun_mxl603_control;
	Tuner_API_T->nim_Tuner_Status = tun_mxl603_status;
	Tuner_API_T->nim_Tuner_Command = tun_mxl603_command;
	Tuner_API_T->tune_t2_first = 1;
	Tuner_API_T->tuner_config.cTuner_Base_Addr = 0xC0;
	Tuner_API_T->tuner_config.demo_type = PANASONIC_DEMODULATOR;
	Tuner_API_T->tuner_config.cChip = Tuner_Chip_MAXLINEAR;
	Tuner_API_T->tuner_config.demo_addr = 0x38;
	Tuner_API_T->tuner_config.i2c_mutex_id = OSAL_INVALID_ID;
	Tuner_API_T->tuner_config.i2c_type_id = i2c_id;
	Tuner_API_T->ext_dm_config.i2c_type_id = i2c_id;
	Tuner_API_T->ext_dm_config.i2c_base_addr = 0xd8;
	Tuner_API_T->work_mode = NIM_COFDM_ONLY_MODE;
	Tuner_API_T->ts_mode = NIM_COFDM_TS_SSI;
	Tuner_API_T->ext_dm_config.nim_idx = idx;
	Tuner_API_T->config_data.Connection_config = 0x00;  // no I/Q swap.
	Tuner_API_T->config_data.Cofdm_Config = 0x03;  // low-if, direct, if-enable.
	Tuner_API_T->config_data.AGC_REF = 0x63;
	Tuner_API_T->config_data.IF_AGC_MAX = 0xff;
	Tuner_API_T->config_data.IF_AGC_MIN = 0x00;
	Tuner_API_T->tuner_config.cTuner_Crystal = 24;  //16; // !!! Unit:MHz //MN88472 uses MxL603 crystal
	Tuner_API_T->tuner_config.wTuner_IF_Freq = 5000;
	Tuner_API_T->tuner_config.cTuner_AGC_TOP = 252;  // Special for MaxLinear
}

#if 0  // not used
static void nim_config_d61558(struct COFDM_TUNER_CONFIG_API *Tuner_API_T, u32 i2c_id, u8 idx)
{
	(void)i2c_id;
	(void)idx;

	YWLIB_Memset(Tuner_API_T, 0, sizeof(struct COFDM_TUNER_CONFIG_API));

	Tuner_API_T->nim_Tuner_Init = tun_mxl301_init;
	Tuner_API_T->nim_Tuner_Status = tun_mxl301_status;
	Tuner_API_T->nim_Tuner_Control = tun_mxl301_control;
	Tuner_API_T->tune_t2_first = 1;  // when demod_mn88472_ScanFreq, param.priv_param >DEMOD_DVBT2, tuner tune T2 then T
	Tuner_API_T->tuner_config.demo_type = PANASONIC_DEMODULATOR;
	Tuner_API_T->tuner_config.cTuner_Base_Addr = 0xC2;
}
#endif

void osal_task_sleep(u32 ms)
{
	msleep(ms);
}

OSAL_ID osal_mutex_create(void)
{
	return 1;
}
// vim:ts=4
