/*****************************************************************************
 * Copyright (C)2009 fulan Corporation. All Rights Reserved.
 *
 * File: tun_mxl301.c
 *
 * Description: This file contains mxl301 basic function in LLD.
 * History:
 *    Date         Author        Version   Reason
 *    ============ ============= ========= =================
 * 1. 2011/08/26   dmq           Ver 0.1   Create file.
 * 2. 2011/12/12   dmq           Ver 0.2   Add to support Panasonic demodulator
 *
 *****************************************************************************/

#include <types.h>

#include "ywdefs.h"
#include "ywtuner_ext.h"
#include "tuner_def.h"
#include "ioarch.h"

#include "ioreg.h"
#include "tuner_interface.h"

#include "mn88472_inner.h"

#include "mxL_user_define.h"
#include "tun_mxl301.h"
#include "mxl_common.h"
#include "mxl301rf.h"

#define BURST_SZ 6

static u32 mxl301_tuner_cnt = 0;
static struct COFDM_TUNER_CONFIG_EXT *mxl301_cfg[YWTUNERi_MAX_TUNER_NUM] = {NULL};

int tun_mxl301_mask_write(u32 tuner_id, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 value[2];
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
	ret = I2C_ReadWrite(mxl301_cfg[tuner_id]->i2c_adap, TUNER_IO_SA_READ, reg, value, 1, 100);
	value[0] |= mask & data;
	value[0] &= (mask ^ 0xff) | data;
	ret |= I2C_ReadWrite(mxl301_cfg[tuner_id]->i2c_adap, TUNER_IO_SA_WRITE, reg, value, 1, 100);
	return ret;
}

int tun_mxl301_i2c_write(u32 tuner_id, u8 *pArray, u32 count)
{
	int result = SUCCESS;
	int i, cycle;
	u8 tmp[BURST_SZ + 2];
	struct COFDM_TUNER_CONFIG_EXT *mxl301_ptr = NULL;

	cycle = count / BURST_SZ;
	mxl301_ptr = mxl301_cfg[tuner_id];
	if (PANASONIC_DEMODULATOR == mxl301_ptr->demo_type)
	{
		tun_mxl301_mask_write(tuner_id, DMD_TCBSET, 0x7f, 0x53);
		tmp[0] = 0;
		result |= I2C_ReadWrite(mxl301_ptr->i2c_adap, TUNER_IO_SA_WRITE, DMD_TCBADR, tmp, 1, 100);
		for (i = 0; i < cycle; i++)
		{
			memcpy(&tmp[1], &pArray[BURST_SZ * i], BURST_SZ);
			tmp[0] = mxl301_ptr->cTuner_Base_Addr;
			result = I2C_ReadWrite(mxl301_ptr->i2c_adap, TUNER_IO_SA_WRITE, DMD_TCBCOM, tmp, BURST_SZ + 1, 100);
		}
		if (BURST_SZ * i < count)
		{
			memcpy(&tmp[1], &pArray[BURST_SZ * i], count - BURST_SZ * i);
			tmp[0] = mxl301_ptr->cTuner_Base_Addr;
			result += I2C_ReadWrite(mxl301_ptr->i2c_adap, TUNER_IO_SA_WRITE, DMD_TCBCOM, tmp, count - BURST_SZ * i + 1, 100);
		}
	}
	else
	{
		for (i = 0; i < cycle; i++)
		{
			memcpy(&tmp[1], &pArray[BURST_SZ * i], BURST_SZ + 1);
			tmp[0] = mxl301_ptr->cTuner_Base_Addr;
			printk("loop %p\n", mxl301_ptr->i2c_adap);
			result += I2C_ReadWrite(mxl301_ptr->i2c_adap, TUNER_IO_SA_WRITE, 0x09, tmp, BURST_SZ, 100);
		}
		if (BURST_SZ * i < count)
		{
			memcpy(&tmp[1], &pArray[BURST_SZ * i], count - BURST_SZ * i);
			tmp[0] = mxl301_ptr->cTuner_Base_Addr;
			result += I2C_ReadWrite(mxl301_ptr->i2c_adap, TUNER_IO_SA_WRITE, 0x09, tmp, count - BURST_SZ * i + 1, 100);
		}
	}
	return result;
}

int tun_mxl301_i2c_read(u32 tuner_id, u8 Addr, u8 *mData)
{
	int ret = 0;
	u8 cmd[4];
	struct COFDM_TUNER_CONFIG_EXT *mxl301_ptr = NULL;

	mxl301_ptr = mxl301_cfg[tuner_id];
	if (PANASONIC_DEMODULATOR == mxl301_ptr->demo_type)
	{
		tun_mxl301_mask_write(tuner_id, DMD_TCBSET, 0x7f, 0x53);
		cmd[0] = 0;
		ret |= I2C_ReadWrite(mxl301_ptr->i2c_adap, TUNER_IO_SA_WRITE, DMD_TCBADR, cmd, 1, 100);
		cmd[0] = mxl301_ptr->cTuner_Base_Addr;
		cmd[1] = 0xFB;
		cmd[2] = Addr;
		ret |= I2C_ReadWrite(mxl301_ptr->i2c_adap, TUNER_IO_SA_WRITE, DMD_TCBCOM, cmd, 3, 100);
		cmd[0] = mxl301_ptr->cTuner_Base_Addr | 0x1;
		ret |= I2C_ReadWrite(mxl301_ptr->i2c_adap, TUNER_IO_SA_WRITE, DMD_TCBCOM, cmd, 1, 100);
		ret |= I2C_ReadWrite(mxl301_ptr->i2c_adap, TUNER_IO_READ, DMD_TCBCOM, cmd, 1, 100);
		*mData = cmd[0];
	}
	else
	{
		cmd[0] = mxl301_ptr->cTuner_Base_Addr;
		cmd[1] = 0xFB;
		cmd[2] = Addr;
		ret = I2C_ReadWrite(mxl301_ptr->i2c_adap, TUNER_IO_SA_WRITE, 0x09, cmd, 3, 100);
		cmd[0] |= 0x1;
		ret = I2C_ReadWrite(mxl301_ptr->i2c_adap, TUNER_IO_SA_WRITE, 0x09, cmd, 1, 100);
		ret = I2C_ReadWrite(mxl301_ptr->i2c_adap, TUNER_IO_READ, 0x09, cmd, 1, 100);
		*mData = cmd[0];
	}
	return ret;
}

int MxL_Set_Register(u32 tuner_idx, u8 RegAddr, u8 RegData)
{
	int ret = 0;
	u8 pArray[2];

	pArray[0] = RegAddr;
	pArray[1] = RegData;
	ret = tun_mxl301_i2c_write(tuner_idx, pArray, 2);
	return ret;
}

int MxL_Get_Register(u32 tuner_idx, u8 RegAddr, u8 *RegData)
{
	int ret = 0;

	ret = tun_mxl301_i2c_read(tuner_idx, RegAddr, RegData);
	return ret;
}

int MxL_Soft_Reset(u32 tuner_idx)
{
	u8 reg_reset = 0xFF;
	int ret = 0;

	ret = tun_mxl301_i2c_write(tuner_idx, &reg_reset, 1);
	return ret;
}

int MxL_Stand_By(u32 tuner_idx)
{
	u8 pArray[4];  /* an array pointer that stores the addr and data pairs for I2C write */
	int ret = 0;

	pArray[0] = 0x01;
	pArray[1] = 0x0;
	pArray[2] = 0x13;
	pArray[3] = 0x0;
	ret = tun_mxl301_i2c_write(tuner_idx, &pArray[0], 4);
	return ret;
}

int MxL_Wake_Up(u32 tuner_idx, MxLxxxRF_TunerConfigS *myTuner)
{
	u8 pArray[2]; /* an array pointer that stores the addr and data pairs for I2C write */
	(void)myTuner;

	pArray[0] = 0x01;
	pArray[1] = 0x01;
	if (tun_mxl301_i2c_write(tuner_idx, pArray, 2))
	{
		return MxL_ERR_OTHERS;
	}
	return MxL_OK;
}

/*****************************************************************************
 * int tun_mxl301_init(struct COFDM_TUNER_CONFIG_EXT * ptrTuner_Config)
 *
 * Tuner mxl301 Initialization
 *
 * Arguments:
 * Parameter1: struct COFDM_TUNER_CONFIG_EXT *ptrTuner_Config : pointer for Tuner configuration structure
 *
 * Return Value: int : Result
 *****************************************************************************/
int tun_mxl301_init(u32 *tuner_idx, struct COFDM_TUNER_CONFIG_EXT *ptrTuner_Config)
{
	int result;
	u8 data = 1;
	u8 pArray[MAX_ARRAY_SIZE];  /* an array pointer that stores the addr and data pairs for I2C write */
	u32 Array_Size = 0;  /* an integer pointer that stores the number of elements in above array */
	struct COFDM_TUNER_CONFIG_EXT *mxl301_ptr = NULL;
	MxLxxxRF_TunerConfigS *priv_ptr;

	/* check Tuner Configuration structure is available or not */
	if ((ptrTuner_Config == NULL) || mxl301_tuner_cnt >= YWTUNERi_MAX_TUNER_NUM)
	{
		return ERR_FAILURE;
	}
	mxl301_ptr = (struct COFDM_TUNER_CONFIG_EXT *)kmalloc(sizeof(struct COFDM_TUNER_CONFIG_EXT), GFP_KERNEL);
	if (NULL == mxl301_ptr)
	{
		return ERR_FAILURE;
	}
	memcpy(mxl301_ptr, ptrTuner_Config, sizeof(struct COFDM_TUNER_CONFIG_EXT));
	mxl301_ptr->priv = (MxLxxxRF_TunerConfigS *)kmalloc(sizeof(MxLxxxRF_TunerConfigS), GFP_KERNEL);
	if (NULL == mxl301_ptr->priv)
	{
		kfree(mxl301_ptr);
		return ERR_FAILURE;
	}
	mxl301_cfg[*tuner_idx] = mxl301_ptr;  // jhy modified
	mxl301_tuner_cnt++;
	memset(mxl301_ptr->priv, 0, sizeof(MxLxxxRF_TunerConfigS));
	priv_ptr = (MxLxxxRF_TunerConfigS *)mxl301_ptr->priv;
	priv_ptr->I2C_Addr = MxL_I2C_ADDR_97;
	priv_ptr->TunerID = MxL_TunerID_MxL301RF;
	priv_ptr->Mode = MxL_MODE_DVBT;
	priv_ptr->Xtal_Freq = MxL_XTAL_24_MHZ;
	priv_ptr->IF_Freq = MxL_IF_5_MHZ;
	priv_ptr->IF_Spectrum = MxL_NORMAL_IF;
	priv_ptr->ClkOut_Amp = MxL_CLKOUT_AMP_0;
	priv_ptr->Xtal_Cap = MxL_XTAL_CAP_8_PF;
	priv_ptr->AGC = MxL_AGC_SEL1;
	priv_ptr->IF_Path = MxL_IF_PATH1;
	priv_ptr->idac_setting = MxL_IDAC_SETTING_OFF;
	/* '11/10/06 : OKAMOTO Control AGC set point. */
	priv_ptr->AGC_set_point = 0x93;
	/* '11/10/06 : OKAMOTO Select AGC external or internal. */
	priv_ptr->bInternalAgcEnable = TRUE;
	/* Soft reset tuner */
	MxL_Soft_Reset(*tuner_idx);
	result = tun_mxl301_i2c_read(*tuner_idx, 0x17, &data);
	if (SUCCESS != result || 0x09 != (u8)(data & 0x0F))
	{
		kfree(mxl301_ptr->priv);
		kfree(mxl301_ptr);
		return !SUCCESS;
	}
	/* perform initialization calculation */
	MxL301RF_Init(pArray, &Array_Size, (u8)priv_ptr->Mode, (u32)priv_ptr->Xtal_Freq,
				  (u32)priv_ptr->IF_Freq, (u8)priv_ptr->IF_Spectrum, (u8)priv_ptr->ClkOut_Setting, (u8)priv_ptr->ClkOut_Amp,
				  (u8)priv_ptr->Xtal_Cap, (u8)priv_ptr->AGC, (u8)priv_ptr->IF_Path, priv_ptr->bInternalAgcEnable);
	/* perform I2C write here */
	if (SUCCESS != tun_mxl301_i2c_write(*tuner_idx, pArray, Array_Size))
	{
		kfree(mxl301_ptr->priv);
		kfree(mxl301_ptr);
		return !SUCCESS;
	}
	osal_task_sleep(1); /* 1ms delay*/
	return SUCCESS;
}

/*****************************************************************************
 * int tun_mxl301_status(u8 *lock)
 *
 * Tuner read operation
 *
 * Arguments:
 * Parameter1: u8 *lock : Phase lock status
 *
 * Return Value: int : Result
 *****************************************************************************/

int tun_mxl301_status(u32 tuner_idx, u8 *lock)
{
	int result;
	u8 data;

	result = MxL_Get_Register(tuner_idx, 0x16, &data);
	*lock = (0x03 == (data & 0x03));
	return result;
}

/*****************************************************************************
 * int nim_mxl301_control(u32 freq, u8 bandwidth,u8 AGC_Time_Const,u8 *data,u8 _i2c_cmd)
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
int tun_mxl301_control(u32 tuner_idx, u32 freq, u8 bandwidth, u8 AGC_Time_Const, u8 *data, u8 _i2c_cmd)
{
	int ret;
	s16 Data;
	u8 i, Data1, Data2;
	MxLxxxRF_IF_Freq if_freq;
	MxLxxxRF_TunerConfigS *myTuner;
	struct COFDM_TUNER_CONFIG_EXT *mxl301_ptr = NULL;
	u8 pArray[MAX_ARRAY_SIZE]; /* a array pointer that store the addr and data pairs for I2C write */
	u32 Array_Size = 0; /* a integer pointer that store the number of element in above array */
	(void)AGC_Time_Const;
	(void)data;
	(void)_i2c_cmd;

	if (/*tuner_idx >= mxl301_tuner_cnt ||*/ tuner_idx >= YWTUNERi_MAX_TUNER_NUM)
	{
		return ERR_FAILURE;
	}
	mxl301_ptr = mxl301_cfg[tuner_idx];
	myTuner = mxl301_ptr->priv;
	// do tuner init first.
	if (bandwidth == MxL_BW_6MHz)
	{
		if_freq = MxL_IF_4_MHZ;
	}
	else if (bandwidth == MxL_BW_7MHz)
	{
		if_freq = MxL_IF_4_5_MHZ;
	}
	else  // if( bandwidth == MxL_BW_8MHz )
	{
		if_freq = MxL_IF_5_MHZ;
	}
	if (if_freq != myTuner->IF_Freq)
	{
		/*Soft reset tuner */
		MxL_Soft_Reset(tuner_idx);
		/*perform initialization calculation */
		MxL301RF_Init(pArray, &Array_Size, (u8)myTuner->Mode, (u32)myTuner->Xtal_Freq,
					  (u32)myTuner->IF_Freq, (u8)myTuner->IF_Spectrum, (u8)myTuner->ClkOut_Setting, (u8)myTuner->ClkOut_Amp,
					  (u8)myTuner->Xtal_Cap, (u8)myTuner->AGC, (u8)myTuner->IF_Path, myTuner->bInternalAgcEnable);
		/* perform I2C write here */
		if (SUCCESS != tun_mxl301_i2c_write(tuner_idx, pArray, Array_Size))
		{
			//NIM_PRINTF("MxL301RF_Init failed\n");
		}
		osal_task_sleep(10); /* 1ms delay*/
		myTuner->IF_Freq = if_freq;
	}
	// then do tuner tune
	myTuner->RF_Freq_Hz = freq; //modify by yanbinL
	myTuner->BW_MHz = bandwidth;
	/* perform Channel Change calculation */
	ret = MxL301RF_RFTune(pArray, &Array_Size, myTuner->RF_Freq_Hz, myTuner->BW_MHz, myTuner->Mode);
	if (SUCCESS != ret)
	{
		return MxL_ERR_RFTUNE;
	}
	/* perform I2C write here */
	if (SUCCESS != tun_mxl301_i2c_write(tuner_idx, pArray, Array_Size))
	{
		return !SUCCESS;
	}
	MxL_Delay(1); /* Added V9.2.1.0 */
	/* Register read-back based setting for Analog M/N split mode only */
	if (myTuner->TunerID == MxL_TunerID_MxL302RF && myTuner->Mode == MxL_MODE_ANA_MN && myTuner->IF_Split == MxL_IF_SPLIT_ENABLE)
	{
		MxL_Get_Register(tuner_idx, 0xE3, &Data1);
		MxL_Get_Register(tuner_idx, 0xE4, &Data2);
		Data = ((Data2 & 0x03) << 8) + Data1;
		if (Data >= 512)
		{
			Data = Data - 1024;
		}
		if (Data < 20)
		{
			MxL_Set_Register(tuner_idx, 0x85, 0x43);
			MxL_Set_Register(tuner_idx, 0x86, 0x08);
		}
		else if (Data >= 20)
		{
			MxL_Set_Register(tuner_idx, 0x85, 0x9E);
			MxL_Set_Register(tuner_idx, 0x86, 0x0F);
		}
		for (i = 0; i < Array_Size; i += 2)
		{
			if (pArray[i] == 0x11)
			{
				Data1 = pArray[i + 1];
			}
			if (pArray[i] == 0x12)
			{
				Data2 = pArray[i + 1];
			}
		}
		MxL_Set_Register(tuner_idx, 0x11, Data1);
		MxL_Set_Register(tuner_idx, 0x12, Data2);
	}
	if (myTuner->TunerID == MxL_TunerID_MxL302RF)
	{
		MxL_Set_Register(tuner_idx, 0x13, 0x01);
	}
	if (myTuner->TunerID == MxL_TunerID_MxL302RF && myTuner->Mode >= MxL_MODE_ANA_MN && myTuner->IF_Split == MxL_IF_SPLIT_ENABLE)
	{
		if (MxL_Set_Register(tuner_idx, 0x00, 0x01))
		{
			return MxL_ERR_RFTUNE;
		}
	}
	MxL_Delay(30);
	if ((myTuner->Mode == MxL_MODE_DVBT) || (myTuner->Mode >= MxL_MODE_ANA_MN))
	{
		if (MxL_Set_Register(tuner_idx, 0x1A, 0x0D))
		{
			return MxL_ERR_SET_REG;
		}
	}
	if (myTuner->TunerID == MxL_TunerID_MxL302RF && myTuner->Mode >= MxL_MODE_ANA_MN && myTuner->IF_Split == MxL_IF_SPLIT_ENABLE)
	{
		if (MxL_Set_Register(tuner_idx, 0x00, 0x00))
		{
			return MxL_ERR_RFTUNE;
		}
	}
	/* '11/03/16 : OKAMOTO Select IDAC setting in "MxL_Tuner_RFTune". */
	switch (myTuner->idac_setting)
	{
		case MxL_IDAC_SETTING_AUTO:
		{
			u8 Array[] =
			{
				0x0D, 0x00,
				0x0C, 0x67,
				0x6F, 0x89,
				0x70, 0x0C,
				0x6F, 0x8A,
				0x70, 0x0E,
				0x6F, 0x8B,
				0x70, 0x10,
			};
			if ((int)myTuner->idac_hysterisis < 0 || myTuner->idac_hysterisis >= MxL_IDAC_HYSTERISIS_MAX)
			{
				ret = MxL_ERR_OTHERS;
				break;
			}
			else
			{
				u8 ui8_idac_hysterisis;

				ui8_idac_hysterisis = (u8)myTuner->idac_hysterisis;
				Array[15] = Array[15] + ui8_idac_hysterisis;
			}
			ret = tun_mxl301_i2c_write(tuner_idx, Array, sizeof(Array));
			break;
		}
		case MxL_IDAC_SETTING_MANUAL:
		{
			if ((s8)myTuner->dig_idac_code < 0 || myTuner->dig_idac_code >= 63)
			{
				ret = MxL_ERR_OTHERS;
				break;
			}
			else
			{
				u8 Array[] = {0x0D, 0x0};
				Array[1] = 0xc0 + myTuner->dig_idac_code; //DIG_ENIDAC_BYP(0x0D[7])=1, DIG_ENIDAC(0x0D[6])=1
				ret = tun_mxl301_i2c_write(tuner_idx, Array, sizeof(Array));
			}
			break;
		}
		case MxL_IDAC_SETTING_OFF:
		{
			ret = MXL301_register_write_bit_name(tuner_idx, MxL_BIT_NAME_DIG_ENIDAC, 0);//0x0D[6] 0
			break;
		}
		default:
		{
			ret = MxL_ERR_OTHERS;
			break;
		}
	}
	return ret;
}

int tun_mxl301_set_addr(u32 tuner_idx, u8 addr, u32 i2c_mutex_id)
{
	struct COFDM_TUNER_CONFIG_EXT *mxl301_ptr = NULL;

	mxl301_ptr = mxl301_cfg[tuner_idx];
	mxl301_ptr->demo_addr = addr;
	mxl301_ptr->i2c_mutex_id = i2c_mutex_id;
	return SUCCESS;
}

int MxL_Check_RF_Input_Power(u32 tuner_idx, u32 *RF_Input_Level)
{
	u8 RFin1, RFin2, RFOff1, RFOff2;
	u32 RFin, RFoff;
	u32 cal_factor;
	MxLxxxRF_TunerConfigS *myTuner;

	myTuner = mxl301_cfg[tuner_idx]->priv;
	if (MxL_Set_Register(tuner_idx, 0x14, 0x01))
	{
		return MxL_ERR_SET_REG;
	}
	MxL_Delay(1);
	if (MxL_Get_Register(tuner_idx, 0x18, &RFin1)) /* LSBs */
	{
		return MxL_ERR_SET_REG;
	}
	if (MxL_Get_Register(tuner_idx, 0x19, &RFin2)) /* MSBs */
	{
		return MxL_ERR_SET_REG;
	}
	if (MxL_Get_Register(tuner_idx, 0xD6, &RFOff1)) /* LSBs */
	{
		return MxL_ERR_SET_REG;
	}
	if (MxL_Get_Register(tuner_idx, 0xD7, &RFOff2)) /* MSBs */
	{
		return MxL_ERR_SET_REG;
	}
	RFin = (((RFin2 & 0x07) << 5) + ((RFin1 & 0xF8) >> 3) + ((RFin1 & 0x07) << 3));
	RFoff = (((RFOff2 & 0x0F) << 2) + ((RFOff1 & 0xC0) >> 6) + (((RFOff1 & 0x38) >> 3) << 3));
	if (myTuner->Mode == MxL_MODE_DVBT)
	{
		cal_factor = 113;
	}
	else if (myTuner->Mode == MxL_MODE_ATSC)
	{
		cal_factor = 109;
	}
	else if (myTuner->Mode == MxL_MODE_CAB_STD)
	{
		cal_factor = 110;
	}
	else
	{
		cal_factor = 107;
	}
	*RF_Input_Level = (RFin - RFoff - cal_factor);
	return MxL_OK;
}
// vim:ts=4
