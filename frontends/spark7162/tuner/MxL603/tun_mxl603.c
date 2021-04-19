/*****************************************************************************************
 *
 * FILE NAME : tun_mxl603.c
 *
 * AUTHOR : Mahendra Kondur
 *
 * DATE CREATED : 12/23/2011
 *
 * DESCRIPTION : This file contains I2C driver and Sleep functions that
 * OEM should implement for MxL603 APIs
 *
 *****************************************************************************************
 * Copyright (c) 2011, MaxLinear, Inc.
 ****************************************************************************************/

#include <types.h>
#include <nim_dev.h>
#include <nim_tuner.h>

#include "MxL603_TunerApi.h"
#include "MxL603_TunerCfg.h"
#include "tun_mxl603.h"

#include "nim_mn88472.h"

extern int paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[tun_mxl603] "


#if ((SYS_TUN_MODULE == MXL603) \
 || (SYS_TUN_MODULE == ANY_TUNER))

#define BURST_SZ 6

#if 1
static MxL_ERR_MSG MxL603_Tuner_RFTune(u32 tuner_id, u32 RF_Freq_Hz, MXL603_BW_E BWMHz);
#else
static MxL_ERR_MSG MxL603_Tuner_RFTune(u32 tuner_id, u32 RF_Freq_Hz, MXL603_BW_E BWMHz, MXL603_SIGNAL_MODE_E mode);
#endif
static MxL_ERR_MSG MxL603_Tuner_Init(u32 tuner_id);
static BOOL run_on_through_mode(u32 tuner_id);

static struct COFDM_TUNER_CONFIG_EXT MxL603_Config[MAX_TUNER_SUPPORT_NUM];
static DEM_WRITE_READ_TUNER m_ThroughMode[MAX_TUNER_SUPPORT_NUM];
static BOOL bMxl_Tuner_Inited[MAX_TUNER_SUPPORT_NUM];
static u32 tuner_cnt = MAX_TUNER_SUPPORT_NUM;

/*----------------------------------------------------------------------------------------
--| FUNCTION NAME : MxLWare603_OEM_WriteRegister
--|
--| AUTHOR : Brenndon Lee
--|
--| DATE CREATED : 7/30/2009
--|
--| DESCRIPTION : This function does I2C write operation.
--|
--| RETURN VALUE : True or False
--|
--|-------------------------------------------------------------------------------------*/

#if 0 //def BOARD_RAINROW_02
MXL_STATUS MxLWare603_OEM_WriteRegister(u32 tuner_id, u8 RegAddr, u8 RegData)
{
	AVLEM61_Chip *pAVL_Chip = NULL;
	u32 status = ERR_FAILURE;
	u8 Cmd[2];
	u16 ucAddrSize = 2;

	Cmd[0] = RegAddr;
	Cmd[1] = RegData;
	if (NULL != g_tuner_config)
	{
		pAVL_Chip = g_tuner_config->m_pAVL_Chip;
	}
	if (run_on_through_mode(tuner_id))
	{
		status = m_ThroughMode[tuner_id].Dem_Write_Read_Tuner(m_ThroughMode[tuner_id].nim_dev_priv, MxL603_Config[tuner_id].cTuner_Base_Addr, Cmd, 2, 0, 0);
	}
	else
	{
		status = AVLEM61_I2CRepeater_WriteData(g_tuner_config->m_uiSlaveAddress, Cmd, ucAddrSize, pAVL_Chip);
		if (status != SUCCESS)
		{
			dprintk(100, "AVLEM61_I2CRepeater_WriteData fail err=%d\n", status);
		}
	}
	return (status == SUCCESS ? MXL_TRUE : MXL_FALSE);
}

MXL_STATUS MxLWare603_OEM_ReadRegister(u32 tuner_id, u8 RegAddr, u8 *DataPtr)
{
	AVLEM61_Chip *pAVL_Chip = NULL;
	u32 status = ERR_FAILURE;
	u8 Read_Cmd[2];
	u8 ucAddrSize = 2;
	u16 uiSize = 1;

	/* read step 1. accroding to mxl301 driver API user guide. */
	Read_Cmd[0] = 0xFB;
	Read_Cmd[1] = RegAddr;
	if (NULL != g_tuner_config)
	{
		pAVL_Chip = g_tuner_config->m_pAVL_Chip;
	}
	if (run_on_through_mode(tuner_id))
	{
		status = m_ThroughMode[tuner_id].Dem_Write_Read_Tuner(m_ThroughMode[tuner_id].nim_dev_priv, MxL603_Config[tuner_id].cTuner_Base_Addr, Read_Cmd, 2, 0, 0);
		if (status != SUCCESS)
		{
			return MXL_FALSE;
		}
		status = m_ThroughMode[tuner_id].Dem_Write_Read_Tuner(m_ThroughMode[tuner_id].nim_dev_priv, MxL603_Config[tuner_id].cTuner_Base_Addr, 0, 0, DataPtr, 1);
		if (status != SUCCESS)
		{
			return MXL_FALSE;
		}
	}
	else
	{
		status = AVLEM61_I2CRepeater_ReadData(g_tuner_config->m_uiSlaveAddress, Read_Cmd, ucAddrSize, DataPtr, uiSize, pAVL_Chip);
		if (status != SUCCESS)
		{
			dprintk(100, "AVLEM61_I2CRepeater_ReadData fail err=%d", status);
			return MXL_FALSE;
		}
	}
	return MXL_TRUE;
//	return status;
}

/*****************************************************************************
* int nim_mxl603_control(u32 freq, u8 bandwidth,u8 AGC_Time_Const,u8 *data,u8 _i2c_cmd)
*
* Tuner write operation
*
* Arguments:
* Parameter1: u32 freq : Synthesiser programmable divider
* Parameter2: u8 bandwidth : channel bandwidth
* Parameter3: u8 AGC_Time_Const : AGC time constant
* Parameter4: u8 *data :
*
* Return Value: int : Result
*****************************************************************************/
int tun_mxl603_control(u32 tuner_id, u32 freq, u8 bandwidth, u8 AGC_Time_Const, u8 *data, u8 DvbMode)
{
	MXL603_BW_E BW;
	MxL_ERR_MSG Status;
	MXL603_SIGNAL_MODE_E mode = 0;

	if (tuner_id >= tuner_cnt)
	{
		return ERR_FAILURE;
	}

	//add by yanbinL
	if (DvbMode == 0)
	{
		mode = MXL603_DIG_DVB_T;
		switch (bandwidth)
		{
			case 6:
			{
				BW = MXL603_TERR_BW_6MHz;
				break;
			}
			case 7:
			{
				BW = MXL603_TERR_BW_7MHz;
				break;
			}
			case 8:
			{
				BW = MXL603_TERR_BW_8MHz;
				break;
			}
			default:
			{
				return ERR_FAILURE;
			}
		}
	}
	else if (DvbMode == 1)
	{
		mode = MXL603_DIG_DVB_C;
		switch (bandwidth)
		{
			case 6:
			{
				BW = MXL603_CABLE_BW_6MHz;
				break;
			}
			case 7:
			{
				BW = MXL603_CABLE_BW_7MHz;
				break;
			}
			case 8:
			{
				BW = MXL603_CABLE_BW_8MHz;
				break;
			}
			default:
			{
				return ERR_FAILURE;
			}
		}
	}
	else
	{
		return ERR_FAILURE;
	}
#if 0
	switch(BW)
	{
		case MXL603_TERR_BW_6MHz:
		{
			Mxl301_TunerConfig[tuner_id].IF_Freq = MxL_IF_4_5_MHZ;
			break;
		}
		case MXL603_TERR_BW_7MHz:
		{
			Mxl301_TunerConfig[tuner_id].IF_Freq = MxL_IF_4_5_MHZ;
			break;
		}
		case MXL603_TERR_BW_8MHz:
		{
			Mxl301_TunerConfig[tuner_id].IF_Freq = MxL_IF_5_MHZ;
			break;
		}
		default:
		{
			return ERR_FAILURE;
		}
	}
#endif
	if (bMxl_Tuner_Inited[tuner_id] != TRUE)
	{
		if (MxL603_Tuner_Init(tuner_id) != MxL_OK)
		{
			dprintk(100, "MxL603_Tuner_Init error\n");
			return ERR_FAILURE;
		}
		bMxl_Tuner_Inited[tuner_id] = TRUE;
	}
	if ((Status = MxL603_Tuner_RFTune(tuner_id, freq, BW, mode)) != MxL_OK)
	{
		dprintk(100, "MxL603_Tuner_RFTune error\n");
		return ERR_FAILURE;
	}
	return SUCCESS;
}

static MxL_ERR_MSG MxL603_Tuner_RFTune(u32 tuner_id, u32 RF_Freq_Hz, MXL603_BW_E BWMHz, MXL603_SIGNAL_MODE_E mode)
{
	MXL_STATUS status;
	MXL603_CHAN_TUNE_CFG_T chanTuneCfg;

	dprintk(20, "freq:%d Hz,BWMHz:%d,mode:%d,Crystal:%d\n", RF_Freq_Hz, BWMHz, mode, MxL603_Config[tuner_id].cTuner_Crystal);

	chanTuneCfg.bandWidth = BWMHz;
	chanTuneCfg.freqInHz = RF_Freq_Hz;
	chanTuneCfg.signalMode = mode;
	chanTuneCfg.xtalFreqSel = MxL603_Config[tuner_id].cTuner_Crystal;;
	chanTuneCfg.startTune = MXL_START_TUNE;
	status = MxLWare603_API_CfgTunerChanTune(tuner_id, chanTuneCfg);
	if (status != MXL_SUCCESS)
	{
		dprintk(1, "%s: Error! MxLWare603_API_CfgTunerChanTune\n", __func__);
		return status;
	}
	// Wait 15 ms
	MxLWare603_OEM_Sleep(15);
	return MxL_OK;
}
#else

#if defined(MODULE)
u32 *tun_getExtDeviceHandle(u32 tuner_id)
{
	struct COFDM_TUNER_CONFIG_EXT *mxl603_ptr = NULL;

	dprintk(150, "%s > MxL603_Config[%d].i2c_adap = 0x%08x\n", __func__, tuner_id, (int)MxL603_Config[tuner_id].i2c_adap);
	mxl603_ptr = &MxL603_Config[tuner_id];
	return mxl603_ptr->i2c_adap;
}
#else
u32 *tun_getExtDeviceHandle(u32 tuner_id)
{
	TUNER_ScanTaskParam_T *Inst = NULL;
	u32 IOHandle = 0;

	Inst = TUNER_GetScanInfo(tuner_id);
	if (Inst->Device == YWTUNER_DELIVER_TER)
	{
		IOHandle = Inst->DriverParam.Ter.TunerIOHandle;
	}
	else if (Inst->Device == YWTUNER_DELIVER_CAB)
	{
		IOHandle = Inst->DriverParam.Cab.TunerIOHandle;
	}
	dprintk(70, "%s: IOHandle = %d\n", __func__, IOHandle);
	dprintk(100, "%s: IOARCH_Handle[%d].ExtDeviceHandle = %d\n", __func__, IOHandle, IOARCH_Handle[IOHandle].ExtDeviceHandle);

	if (0 == IOHandle)
	{
		dprintk(5, "%s:[%d] Warning IOHandle may have failed\n", __func__, __LINE__);
	}
	return &IOARCH_Handle[IOHandle].ExtDeviceHandle;
}
#endif

int tun_mxl603_mask_write(u32 tuner_id, u8 addr, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 value[2];
	u32 *pExtDeviceHandle = tun_getExtDeviceHandle(tuner_id);
	(void)addr;

	ret = I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_SA_READ, reg, value, 1, 100);
	dprintk(100, "%s value[0] = %d\n", __func__, value[0]);
	value[0] |= mask & data;
	value[0] &= (mask ^ 0xff) | data;
	ret |= I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_SA_WRITE, reg, value, 1, 100);

	if (SUCCESS != ret)
	{
		dprintk(1, "%s: write i2c failed\n", __func__);
	}
	return ret;
}

int tun_mxl603_i2c_write(u32 tuner_id, u8 *pArray, u32 count)
{
	int result = SUCCESS;
	int i, j, cycle;
	u8 tmp[BURST_SZ + 2];
	struct COFDM_TUNER_CONFIG_EXT *mxl603_ptr = NULL;
	u32 *pExtDeviceHandle = tun_getExtDeviceHandle(tuner_id);

//	dprintk(150, "%s > tuner_id = %d, count = %d\n", __func__, tuner_id, count);
#if 0
	if(tuner_id >= mxl301_tuner_cnt)
	{
		return ERR_FAILURE;
	}
#endif
	cycle = count / BURST_SZ;
	mxl603_ptr = &MxL603_Config[tuner_id];
	osal_mutex_lock(mxl603_ptr->i2c_mutex_id, OSAL_WAIT_FOREVER_TIME);
	tun_mxl603_mask_write(tuner_id, MN88472_T2_ADDR, DMD_TCBSET, 0x7f, 0x53);
	tmp[0] = 0;
	result |= I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_SA_WRITE, DMD_TCBADR, tmp, 1, 100);
//	dprintk(20, "result = %d\n", result);
	for (i = 0; i < cycle; i++)
	{
		memcpy(&tmp[1], &pArray[BURST_SZ * i], BURST_SZ);
		tmp[0] = 0xc0;
		result = I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_SA_WRITE, DMD_TCBCOM, tmp, BURST_SZ + 1, 100);
		for (j = 0; j < (BURST_SZ / 2); j++)
		{
			dprintk(100, "%s: reg[0x%02x], value[0x%02x]\n", __func__, tmp[1 + 2 * j], tmp[1 + 2 * j + 1]);
		}
	}
	if (BURST_SZ * i < (int)count)
	{
		memcpy(&tmp[1], &pArray[BURST_SZ * i], count - BURST_SZ * i);
		tmp[0] = 0xc0;
		result += I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_SA_WRITE, DMD_TCBCOM, tmp, count - BURST_SZ * i + 1, 100);
		for (j = 0; j < (int)((count - BURST_SZ * i) / 2); j++)
		{
			dprintk(70, "%s: reg[0x%02x], value[0x%02x]\n", __func__, tmp[1 + 2 * j], tmp[1 + 2 * j + 1]);
		}
	}
	osal_mutex_unlock(mxl603_ptr->i2c_mutex_id);
	if (SUCCESS != result)
	{
		dprintk(1, "%s Error: write i2c failed\n", __func__);
	}
	return result;
}

int tun_mxl603_i2c_read(u32 tuner_id, u8 Addr, u8 *mData)
{
	int ret = 0;
	u8 cmd[4];
	struct COFDM_TUNER_CONFIG_EXT *mxl603_ptr = NULL;
	u32 *pExtDeviceHandle = tun_getExtDeviceHandle(tuner_id);

	mxl603_ptr = &MxL603_Config[tuner_id];
	osal_mutex_lock(mxl603_ptr->i2c_mutex_id, OSAL_WAIT_FOREVER_TIME);
	tun_mxl603_mask_write(tuner_id, MN88472_T2_ADDR, DMD_TCBSET, 0x7f, 0x53);
	cmd[0] = 0;
	ret |= I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_SA_WRITE, DMD_TCBADR, cmd, 1, 100);

	cmd[0] = 0xc0;
	cmd[1] = 0xFB;
	cmd[2] = Addr;
	ret |= I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_SA_WRITE, DMD_TCBCOM, cmd, 3, 100);

	cmd[0] = 0xc0 | 0x1;
	ret |= I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_SA_WRITE, DMD_TCBCOM, cmd, 1, 100);
	ret |= I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_READ, DMD_TCBCOM, cmd, 1, 100);
	*mData = cmd[0];
	dprintk(70, "%s: cmd[0] = %d\n", __func__, cmd[0]);
	osal_mutex_unlock(mxl603_ptr->i2c_mutex_id);
	if (SUCCESS != ret)
	{
		dprintk(1, "%s Error: read i2c failed\n", __func__);
	}
	return ret;
}

MXL_STATUS MxLWare603_OEM_WriteRegister(u32 tuner_id, u8 RegAddr, u8 RegData)
{
	u32 status = ERR_FAILURE;
	u8 Cmd[2];
	u32 *pExtDeviceHandle = tun_getExtDeviceHandle(tuner_id);
	Cmd[0] = RegAddr;
	Cmd[1] = RegData;
	if (run_on_through_mode(tuner_id))
	{
		// status = m_ThroughMode[tuner_id].Dem_Write_Read_Tuner(m_ThroughMode[tuner_id].nim_dev_priv, MxL603_Config[tuner_id].cTuner_Base_Addr, Cmd, 2, 0, 0);
		status = tun_mxl603_i2c_write(tuner_id, Cmd, 2);
	}
	else
	{
//		dprintk(70, "I2C_ReadWrite\n");
		status = I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_SA_WRITE, Cmd[0], &Cmd[1], 1, 100);
	}
//	dprintk(150, "%s < status = %d\n", __func__, status);
	return (status == SUCCESS ? MXL_TRUE : MXL_FALSE);
}

/*------------------------------------------------------------------------------
--| FUNCTION NAME : MxLWare603_OEM_ReadRegister
--|
--| AUTHOR : Brenndon Lee
--|
--| DATE CREATED : 7/30/2009
--|
--| DESCRIPTION : This function does I2C read operation.
--|
--| RETURN VALUE : True or False
--|
--|---------------------------------------------------------------------------*/

MXL_STATUS MxLWare603_OEM_ReadRegister(u32 tuner_id, u8 RegAddr, u8 *DataPtr)
{
	u32 status = ERR_FAILURE;
	u8 Read_Cmd[2];
	u32 *pExtDeviceHandle = tun_getExtDeviceHandle(tuner_id);

	/* read step 1. accroding to mxl301 driver API user guide. */
	Read_Cmd[0] = 0xFB;
	Read_Cmd[1] = RegAddr;
	if (run_on_through_mode(tuner_id))
	{
		//status = m_ThroughMode[tuner_id].Dem_Write_Read_Tuner(m_ThroughMode[tuner_id].nim_dev_priv, MxL603_Config[tuner_id].cTuner_Base_Addr, Read_Cmd, 2, 0, 0);
		status = tun_mxl603_i2c_write(tuner_id, Read_Cmd, 2);
		if (status != SUCCESS)
		{
			return MXL_FALSE;
		}
		//status = m_ThroughMode[tuner_id].Dem_Write_Read_Tuner(m_ThroughMode[tuner_id].nim_dev_priv, MxL603_Config[tuner_id].cTuner_Base_Addr, 0, 0, DataPtr, 1);
		status = tun_mxl603_i2c_read(tuner_id, RegAddr, DataPtr);
		if (status != SUCCESS)
		{
			return MXL_FALSE;
		}
	}
	else
	{
#if 0
		status = i2c_write(MxL603_Config[tuner_id].i2c_type_id, MxL603_Config[tuner_id].cTuner_Base_Addr, Read_Cmd, 2);
		if (status != SUCCESS)
		{
			return MXL_FALSE;
		}
		status = i2c_read(MxL603_Config[tuner_id].i2c_type_id, MxL603_Config[tuner_id].cTuner_Base_Addr, DataPtr, 1);
		if (status != SUCCESS)
		{
			return MXL_FALSE;
		}
#endif
		status |= I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_SA_WRITE, 0xFB, &Read_Cmd[1], 1, 100);
		Read_Cmd[0] = 0xc0 | 0x1;
		status |= I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_SA_WRITE, DMD_TCBCOM, &Read_Cmd[0], 1, 100);
		status |= I2C_ReadWrite(pExtDeviceHandle, TUNER_IO_READ, DMD_TCBCOM, DataPtr, 1, 100);
	}
	return MXL_TRUE;
//	return status;
}

/*****************************************************************************
* int nim_mxl603_control(u32 freq, u8 bandwidth,u8 AGC_Time_Const,u8 *data,u8 _i2c_cmd)
*
* Tuner write operation
*
* Arguments:
* Parameter1: u32 freq : Synthesizer programmable divider
* Parameter2: u8 bandwidth : channel bandwidth
* Parameter3: u8 AGC_Time_Const : AGC time constant
* Parameter4: u8 *data :
*
* Return Value: int : Result
*****************************************************************************/
int tun_mxl603_control(u32 tuner_id, u32 freq, u8 bandwidth, u8 AGC_Time_Const, u8 *data, u8 _i2c_cmd)
{
	MXL603_BW_E BW;
	MxL_ERR_MSG Status;
	(void)AGC_Time_Const;
	(void)data;
	(void)_i2c_cmd;

	dprintk(150, "%s >\n", __func__);
	dprintk(20, "Tune to %d.%03d MHz (Bandwidth = %d MHz)\n", freq / 1000000, (freq % 1000000) / 1000, bandwidth);
#if 0
	if (tuner_id >= tuner_cnt)
	{
		return ERR_FAILURE;
	}
#endif
	switch (bandwidth)
	{
		case 6:
		{
			BW = MXL603_TERR_BW_6MHz;
			break;
		}
		case 7:
		{
			BW = MXL603_TERR_BW_7MHz;
			break;
		}
		case 8:
		{
			BW = MXL603_TERR_BW_8MHz;
			break;
		}
		default:
		{
			return ERR_FAILURE;
		}
	}

	if (bMxl_Tuner_Inited[tuner_id] != TRUE)
	{
		if (MxL603_Tuner_Init(tuner_id) != MxL_OK)
		{
			return ERR_FAILURE;
		}
		bMxl_Tuner_Inited[tuner_id] = TRUE;
		dprintk(70, "%s: Tuner initialized\n", __func__);
	}
	if ((Status = MxL603_Tuner_RFTune(tuner_id, freq * 1000, BW)) != MxL_OK)
	{
		return ERR_FAILURE;
	}
	return SUCCESS;
}

static MxL_ERR_MSG MxL603_Tuner_RFTune(u32 tuner_id, u32 RF_Freq_Hz, MXL603_BW_E BWMHz)
{
	MXL_STATUS status;
	MXL603_CHAN_TUNE_CFG_T chanTuneCfg;

	chanTuneCfg.bandWidth = BWMHz;
	chanTuneCfg.freqInHz = RF_Freq_Hz;
	chanTuneCfg.signalMode = MXL603_DIG_DVB_T;
	chanTuneCfg.xtalFreqSel = MxL603_Config[tuner_id].cTuner_Crystal;;
	chanTuneCfg.startTune = MXL_START_TUNE;
	status = MxLWare603_API_CfgTunerChanTune(tuner_id, chanTuneCfg);
	if (status != MXL_SUCCESS)
	{
		dprintk(1, "%s: Error! MxLWare603_API_CfgTunerChanTune failed\n", __func__);
		return status;
	}
	// Wait 15 ms
	MxLWare603_OEM_Sleep(15);
	return MxL_OK;
}

int tun_mxl603_set_addr(u32 tuner_idx, u8 addr, u32 i2c_mutex_id)
{
	struct COFDM_TUNER_CONFIG_EXT *mxl603_ptr = NULL;

	if (tuner_idx >= tuner_cnt)
	{
		return ERR_FAILURE;
	}
	mxl603_ptr = &MxL603_Config[tuner_idx];
	mxl603_ptr->demo_addr = addr;
	mxl603_ptr->i2c_mutex_id = i2c_mutex_id;
	return SUCCESS;
}
#endif  //SYS_TUN_MODULE == MXL603... (line 33)

/*------------------------------------------------------------------------------
--| FUNCTION NAME : MxLWare603_OEM_Sleep
--|
--| AUTHOR : Dong Liu
--|
--| DATE CREATED : 01/10/2010
--|
--| DESCRIPTION : This function complete sleep operation. WaitTime is in ms unit
--|
--| RETURN VALUE : None
--|
--|-------------------------------------------------------------------------------------*/
void MxLWare603_OEM_Sleep(u16 DelayTimeInMs)
{
	osal_task_sleep(DelayTimeInMs);
}

static BOOL run_on_through_mode(u32 tuner_id)
{
	return (m_ThroughMode[tuner_id].nim_dev_priv && m_ThroughMode[tuner_id].Dem_Write_Read_Tuner);
}

static int set_through_mode(u32 tuner_id, DEM_WRITE_READ_TUNER *ThroughMode)
{
	if (tuner_id >= tuner_cnt)
	{
		return ERR_FAILURE;
	}
	memcpy((u8 *)(&m_ThroughMode[tuner_id]), (u8 *)ThroughMode, sizeof(DEM_WRITE_READ_TUNER));
	return SUCCESS;
}

/*****************************************************************************
* int tun_mxl603_init(struct COFDM_TUNER_CONFIG_EXT * ptrTuner_Config)
*
* Tuner mxl603 Initialization
*
* Arguments:
* Parameter1: struct COFDM_TUNER_CONFIG_EXT * ptrTuner_Config : pointer for Tuner configuration structure
*
* Return Value: int : Result
*****************************************************************************/
int tun_mxl603_init(u32 *tuner_id, struct COFDM_TUNER_CONFIG_EXT *ptrTuner_Config)
{
	u16 Xtal_Freq;

	/* check Tuner Configuration structure is available or not */
	if ((ptrTuner_Config == NULL))
	{
		return ERR_FAILURE;
	}
	if (*tuner_id >= MAX_TUNER_SUPPORT_NUM)
	{
		return ERR_FAILURE;
	}
	memcpy(&MxL603_Config[*tuner_id], ptrTuner_Config, sizeof(struct COFDM_TUNER_CONFIG_EXT));
	m_ThroughMode[*tuner_id].Dem_Write_Read_Tuner = NULL;
	Xtal_Freq = MxL603_Config[*tuner_id].cTuner_Crystal;
	if (Xtal_Freq >= 1000)  // If it is in kHz, then convert it into MHz.
	{
		Xtal_Freq /= 1000;
	}
	switch (Xtal_Freq)
	{
		case 16:
		{
			MxL603_Config[*tuner_id].cTuner_Crystal = (MXL603_XTAL_FREQ_E)MXL603_XTAL_16MHz;
			break;
		}
		case 24:
		{
			MxL603_Config[*tuner_id].cTuner_Crystal = (MXL603_XTAL_FREQ_E)MXL603_XTAL_24MHz;
			break;
		}
		default:
		{
			return ERR_FAILURE;
		}
	}
	switch (MxL603_Config[*tuner_id].wTuner_IF_Freq)
	{
		case 5000:
		{
			MxL603_Config[*tuner_id].wTuner_IF_Freq = MXL603_IF_5MHz;
			break;
		}
		case 4750:
		{
			MxL603_Config[*tuner_id].wTuner_IF_Freq = MXL603_IF_4_57MHz;
			break;
		}
		case 36000:
		{
			MxL603_Config[*tuner_id].wTuner_IF_Freq = MXL603_IF_36MHz;
			break;
		}
		default:
		{
			return ERR_FAILURE;
		}
	}
	bMxl_Tuner_Inited[*tuner_id] = FALSE;
#if 0
	if (MxL603_Tuner_Init(tuner_cnt) != MxL_OK)
	{
		return ERR_FAILURE;
	}
	bMxl_Tuner_Inited[tuner_cnt] = TRUE;
	if (tuner_id)
	{
		*tuner_id = tuner_cnt; //I2C_TYPE_SCB1
	}
#endif
	tuner_cnt = MAX_TUNER_SUPPORT_NUM;
	return SUCCESS;
}

/*****************************************************************************
* int tun_mxl603_status(u8 *lock)
*
* Tuner read operation
*
* Arguments:
* Parameter1: u8 *lock : Phase lock status
*
* Return Value: int : Result
*****************************************************************************/

int tun_mxl603_status(u32 tuner_id, u8 *lock)
{
	MXL_STATUS status;
	MXL_BOOL refLockPtr = MXL_UNLOCKED;
	MXL_BOOL rfLockPtr = MXL_UNLOCKED;

	if (tuner_id >= tuner_cnt)
	{
		return ERR_FAILURE;
	}
	*lock = FALSE;
	status = MxLWare603_API_ReqTunerLockStatus(tuner_id, &rfLockPtr, &refLockPtr);
	if (status == MXL_TRUE)
	{
		if (MXL_LOCKED == rfLockPtr && MXL_LOCKED == refLockPtr)
		{
			*lock = TRUE;
			dprintk(20, "Tuner locked\n");
		}
	}
	return SUCCESS;
}

int tun_mxl603_rfpower(u32 tuner_id, s16 *rf_power_dbm)
{
	MXL_STATUS status;
	short int RSSI = 0;

	status = MxLWare603_API_ReqTunerRxPower(tuner_id, &RSSI);
	*rf_power_dbm = (s16)RSSI / 100;
	return SUCCESS;  // AVLEM61_EC_OK;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//																		   //
//							Tuner Functions                                //
//																		   //
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
static MxL_ERR_MSG MxL603_Tuner_Init(u32 tuner_id)
{
	MXL_STATUS status;
	MXL_BOOL singleSupply_3_3V;
	MXL603_XTAL_SET_CFG_T xtalCfg;
	MXL603_IF_OUT_CFG_T ifOutCfg;
	MXL603_AGC_CFG_T agcCfg;
	MXL603_TUNER_MODE_CFG_T tunerModeCfg;
	MXL603_CHAN_TUNE_CFG_T chanTuneCfg;
	//MXL603_IF_FREQ_E ifOutFreq;

	//Step 1 : Soft Reset MxL603
	status = MxLWare603_API_CfgDevSoftReset(tuner_id);
	if (status != MXL_SUCCESS)
	{
		dprintk(1, "%s: Error! MxLWare603_API_CfgDevSoftReset\n", __func__);
		return status;
	}

	//Step 2 : Overwrite Default. It should be called after MxLWare603_API_CfgDevSoftReset and MxLWare603_API_CfgDevXtal.
//	singleSupply_3_3V = MXL_DISABLE;
	singleSupply_3_3V = MXL_ENABLE;
	status = MxLWare603_API_CfgDevOverwriteDefaults(tuner_id, singleSupply_3_3V);
	if (status != MXL_SUCCESS)
	{
		dprintk(1, "%s: Error! MxLWare603_API_CfgDevOverwriteDefaults\n", __func__);
		return status;
	}

	//Step 3 : XTAL Setting
	xtalCfg.xtalFreqSel = MxL603_Config[tuner_id].cTuner_Crystal;
	xtalCfg.xtalCap = 25; //20; //12
#if 1
	xtalCfg.clkOutEnable = MXL_ENABLE;
#else
	xtalCfg.clkOutEnable = MXL_DISABLE;  // MXL_ENABLE
#endif
	xtalCfg.clkOutDiv = MXL_DISABLE;
	xtalCfg.clkOutExt = MXL_DISABLE;
//	xtalCfg.singleSupply_3_3V = MXL_DISABLE;
	xtalCfg.singleSupply_3_3V = singleSupply_3_3V;
	xtalCfg.XtalSharingMode = MXL_DISABLE;
	status = MxLWare603_API_CfgDevXtal(tuner_id, xtalCfg);
	if (status != MXL_SUCCESS)
	{
		dprintk(1, "%s: Error! MxLWare603_API_CfgDevXtal\n", __func__);
		return status;
	}

	//Step 4 : IF Out setting
#if 0
	ifOutCfg.ifOutFreq = MXL603_IF_4_57MHz; //MXL603_IF_4_1MHz
	ifOutCfg.ifOutFreq = Mxl301_TunerConfig[tuner_id].IF_Freq;
#endif
#if 1
	ifOutCfg.ifOutFreq = MXL603_IF_5MHz;  // For match to DEM MN88472 IF: DMD_E_IF_5000KHZ.
#else
	ifOutCfg.ifOutFreq = MXL603_IF_36MHz;
#endif
	ifOutCfg.ifInversion = MXL_DISABLE;
	ifOutCfg.gainLevel = 11;
	ifOutCfg.manualFreqSet = MXL_DISABLE;
	ifOutCfg.manualIFOutFreqInKHz = 0;
	status = MxLWare603_API_CfgTunerIFOutParam(tuner_id, ifOutCfg);
	if (status != MXL_SUCCESS)
	{
		dprintk(1, "%s: Error! MxLWare603_API_CfgTunerIFOutParam failed\n", __func__);
		return status;
	}

	//Step 5 : AGC Setting
	agcCfg.agcType = MXL603_AGC_EXTERNAL;
//	agcCfg.agcType = MXL603_AGC_SELF;
	agcCfg.setPoint = 66;
	agcCfg.agcPolarityInverstion = MXL_DISABLE;
	status = MxLWare603_API_CfgTunerAGC(tuner_id, agcCfg);
	if (status != MXL_SUCCESS)
	{
		dprintk(1, "%s: Error! MxLWare603_API_CfgTunerAGC failed\n", __func__);
		return status;
	}

	//Step 6 : Application Mode setting
	tunerModeCfg.signalMode = MXL603_DIG_DVB_T;
#if 1
	tunerModeCfg.ifOutFreqinKHz = 4100;
#else
	tunerModeCfg.ifOutFreqinKHz = 36000;
#endif
	tunerModeCfg.xtalFreqSel = MxL603_Config[tuner_id].cTuner_Crystal;
	tunerModeCfg.ifOutGainLevel = 11;
	status = MxLWare603_API_CfgTunerMode(tuner_id, tunerModeCfg);
	if (status != MXL_SUCCESS)
	{
		dprintk(1, "%s: Error! MxLWare603_API_CfgTunerMode\n", __func__);
		return status;
	}

	//Step 7 : Channel frequency & bandwidth setting
	chanTuneCfg.bandWidth = MXL603_TERR_BW_8MHz;
	chanTuneCfg.freqInHz = 474000000;
	chanTuneCfg.signalMode = MXL603_DIG_DVB_T;
	chanTuneCfg.xtalFreqSel = MxL603_Config[tuner_id].cTuner_Crystal;
	chanTuneCfg.startTune = MXL_START_TUNE;
	status = MxLWare603_API_CfgTunerChanTune(tuner_id, chanTuneCfg);
	if (status != MXL_SUCCESS)
	{
		dprintk(1, "%s: Error! MxLWare603_API_CfgTunerChanTune\n", __func__);
		return status;
	}
	// MxLWare603_API_CfgTunerChanTune():MxLWare603_OEM_WriteRegister(devId, START_TUNE_REG, 0x01) will make singleSupply_3_3V mode become MXL_DISABLE,
	// It is caused by the power becoming 1.2V.
	// So it must be reset singleSupply_3_3V mode.
#if 0
	if (MXL_ENABLE == singleSupply_3_3V)
	{
		status |= MxLWare603_OEM_WriteRegister(tuner_id, MAIN_REG_AMP, 0x14);
	}
#endif
	// Wait 15 ms
	MxLWare603_OEM_Sleep(15);

	status = MxLWare603_API_CfgTunerLoopThrough(tuner_id, MXL_ENABLE);
	if (status != MXL_SUCCESS)
	{
		return status;
	}
	return MxL_OK;
}

int tun_mxl603_powcontrol(u32 tuner_id, u8 stdby)
{
	MXL_STATUS status;

	if (tuner_id >= tuner_cnt)
	{
		return ERR_FAILURE;
	}
	if (stdby)
	{
		//dprintk(20, "start standby mode!\n");
		status = MxLWare603_API_CfgDevPowerMode(tuner_id, MXL603_PWR_MODE_STANDBY);
		if (status != MXL_SUCCESS)
		{
			dprintk(1, "%s: Setting standby mode failed\n", __func__);
			return ERR_FAILURE;
		}
	}
	else
	{
		//dprintk(20, "start wakeup mode!\n");
		status = MxLWare603_API_CfgDevPowerMode(tuner_id, MXL603_PWR_MODE_ACTIVE);
		if (status != MXL_SUCCESS)
		{
			dprintk(1, "%s: Setting wakeup mode failed\n", __func__);
			return ERR_FAILURE;
		}
#if 0
		if (MxL603_Tuner_Init(tuner_id) != MxL_OK)
		{
			return ERR_FAILURE;
		}
#endif
	}
	return SUCCESS;
}

int tun_mxl603_command(u32 tuner_id, int cmd, u32 param)
{
	int ret = SUCCESS;

#if 0
	if (tuner_id >= tuner_cnt)
	{
		return ERR_FAILURE;
	}
	if (bMxl_Tuner_Inited[tuner_id] == FALSE)
	{
		if (MxL603_Tuner_Init(tuner_id) != MxL_OK)
		{
			return ERR_FAILURE;
		}
		bMxl_Tuner_Inited[tuner_id] = TRUE;
	}
#endif
	switch (cmd)
	{
		case NIM_TUNER_SET_THROUGH_MODE:
		{
			ret = set_through_mode(tuner_id, (DEM_WRITE_READ_TUNER *)param);
			break;
		}
		case NIM_TUNER_POWER_CONTROL:
		{
			tun_mxl603_powcontrol(tuner_id, param);
			break;
		}
		default:
		{
			ret = ERR_FAILURE;
			break;
		}
	}
	return ret;
}
#endif
// vim:ts=4
