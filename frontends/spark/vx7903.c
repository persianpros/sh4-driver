/*****************************************************************************
 *
 * Copyright (C)2008 Ali Corporation. All Rights Reserved.
 *
 * File: vx7903.c
 *
 * Description: This file contains SHARP BS2S7VZ7903 tuner basic function.
 *
 * History:
 *       Date            Author              Version     Reason
 *       ============    =============       =========   =================
 *   1.  2011-10-16      Russell Luo         Ver 0.1     Create file.
 *
 *****************************************************************************/
#include <linux/kernel.h>
#include "dvb_frontend.h"
#include "vx7903.h"

#include "stv090x_reg.h"
#include "stv090x.h"
#include "stv090x_priv.h"

extern short paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[vx7903] "

#define DEF_XTAL_FREQ 16000
#define INIT_DUMMY_RESET 0x0C
#define LPF_CLK_16000kHz 16000

static u32 vx7903_tuner_cnt = 0;

typedef enum _VX7903_INIT_CFG_DATA
{
	VX7903_INILOCAL_FREQ,
	VX7903_INIXTAL_FREQ,
	VX7903_INICHIP_ID,
	VX7903_INILPF_WAIT_TIME,
	VX7903_INIFAST_SEARCH_WAIT_TIME,
	VX7903_ININORMAL_SEARCH_WAIT_TIME,
	VX7903_INI_CFG_MAX
} VX7903_INIT_CFG_DATA, *PVX7903_INIT_CFG_DATA;

typedef enum _VX7903_INIT_REG_DATA
{
	VX7903_REG_00 = 0,
	VX7903_REG_01,
	VX7903_REG_02,
	VX7903_REG_03,
	VX7903_REG_04,
	VX7903_REG_05,
	VX7903_REG_06,
	VX7903_REG_07,
	VX7903_REG_08,
	VX7903_REG_09,
	VX7903_REG_0A,
	VX7903_REG_0B,
	VX7903_REG_0C,
	VX7903_REG_0D,
	VX7903_REG_0E,
	VX7903_REG_0F,
	VX7903_REG_10,
	VX7903_REG_11,
	VX7903_REG_12,
	VX7903_REG_13,
	VX7903_REG_14,
	VX7903_REG_15,
	VX7903_REG_16,
	VX7903_REG_17,
	VX7903_REG_18,
	VX7903_REG_19,
	VX7903_REG_1A,
	VX7903_REG_1B,
	VX7903_REG_1C,
	VX7903_REG_1D,
	VX7903_REG_1E,
	VX7903_REG_1F,
	VX7903_INI_REG_MAX
} VX7903_INIT_REG_DATA, *PVX7903_INIT_REG_DATA;

typedef enum _VX7903_LPF_FC
{
	VX7903_LPF_FC_04MHz = 0,  // 0000:  4MHz
	VX7903_LPF_FC_06MHz,      // 0001:  6MHz
	VX7903_LPF_FC_08MHz,      // 0010:  8MHz
	VX7903_LPF_FC_10MHz,      // 0011: 10MHz
	VX7903_LPF_FC_12MHz,      // 0100: 12MHz
	VX7903_LPF_FC_14MHz,      // 0101: 14MHz
	VX7903_LPF_FC_16MHz,      // 0110: 16MHz
	VX7903_LPF_FC_18MHz,      // 0111: 18MHz
	VX7903_LPF_FC_20MHz,      // 1000: 20MHz
	VX7903_LPF_FC_22MHz,      // 1001: 22MHz
	VX7903_LPF_FC_24MHz,      // 1010: 24MHz
	VX7903_LPF_FC_26MHz,      // 1011: 26MHz
	VX7903_LPF_FC_28MHz,      // 1100: 28MHz
	VX7903_LPF_FC_30MHz,      // 1101: 30MHz
	VX7903_LPF_FC_32MHz,      // 1110: 32MHz
	VX7903_LPF_FC_34MHz,      // 1111: 34MHz
	VX7903_LPF_FC_MAX,
} VX7903_LPF_FC;

typedef struct _VX7903_CONFIG_STRUCT
{
	u32           ui_VX7903_RFChannelkHz;  /* direct channel */
	u32           ui_VX7903_XtalFreqkHz;
	bool          b_VX7903_fast_search_mode;
	bool          b_VX7903_loop_through;
	bool          b_VX7903_tuner_standby;
	bool          b_VX7903_head_amp;
	VX7903_LPF_FC VX7903_lpf;
	u32           ui_VX7903_LpfWaitTime;
	u32           ui_VX7903_FastSearchWaitTime;
	u32           ui_VX7903_NormalSearchWaitTime;
	bool          b_VX7903_iq_output;
} VX7903_CONFIG_STRUCT, *PVX7903_CONFIG_STRUCT;

//=========================================================================
// GLOBAL VARIABLES
//=========================================================================

const unsigned long VX7903_d[VX7903_INI_CFG_MAX] =
{
	0x0017a6b0,  // LOCAL_FREQ
	0x00003e80,  // XTAL_FREQ
	0x00000068,  // CHIP_ID
	0x00000014,  // LPF_WAIT_TIME
	0x00000004,  // FAST_SEARCH_WAIT_TIME
	0x0000000f   // NORMAL_SEARCH_WAIT_TIME
};

// vx7903 initial register values
u8 VX7903_d_reg[VX7903_INI_REG_MAX] =
{
	0x68,  // 0x00
	0x1c,  // 0x01
	0xc0,  // 0x02
	0x10,  // 0x03
	0xbc,  // 0x04
	0xc1,  // 0x05
	0x15,  // 0x06
	0x34,  // 0x07
	0x06,  // 0x08
	0x3e,  // 0x09
	0x00,  // 0x0a
	0x00,  // 0x0b
	0x43,  // 0x0c
	0x00,  // 0x0d
	0x00,  // 0x0e
	0x00,  // 0x0f
	0x00,  // 0x10
	0xff,  // 0x11
	0xf3,  // 0x12
	0x00,  // 0x13
//	0x3f,  // 0x14
	0x3e,  // 0x14
	0x25,  // 0x15
	0x5c,  // 0x16
	0xd6,  // 0x17
	0x55,  // 0x18
//	0xcf,  // 0x19
	0x8f,  // 0x19
	0x95,  // 0x1a
//	0xf6,  // 0x1b
	0xfe,  // 0x1b
	0x36,  // 0x1c
	0xf2,  // 0x1d
	0x09,  // 0x1e
	0x00,  // 0x1f
};

// vx7903 register write flags
const u8 VX7903_d_flg[VX7903_INI_REG_MAX] =  /* 0:R, 1:R/W */
{
	0,  // 0x00
	1,  // 0x01
	1,  // 0x02
	1,  // 0x03
	1,  // 0x04
	1,  // 0x05
	1,  // 0x06
	1,  // 0x07
	1,  // 0x08
	1,  // 0x09
	1,  // 0x0A
	1,  // 0x0B
	1,  // 0x0C
	0,  // 0x0D
	0,  // 0x0E
	0,  // 0x0F
	0,  // 0x10
	1,  // 0x11
	1,  // 0x12
	1,  // 0x13
	1,  // 0x14
	1,  // 0x15
	1,  // 0x16
	1,  // 0x17
	1,  // 0x18
	1,  // 0x19
	1,  // 0x1A
	1,  // 0x1B
	1,  // 0x1C
	1,  // 0x1D
	1,  // 0x1E
	1,  // 0x1F
};

const unsigned long vx7903_local_f[] =  //kHz
{
//	2151000,
	2551000,
	1950000,
	1800000,
	1600000,
	1450000,
	1250000,
	1200000,
	975000,
	950000,
	0,
	0,
	0,
	0,
	0,
	0,
};

const unsigned long vx7903_div2[] =
{
	1,
	1,
	1,
	1,
	1,
	1,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
};

const unsigned long vx7903_vco_band[] =
{
	7,
	6,
	5,
	4,
	3,
	2,
	7,
	6,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
};

const unsigned long vx7903_div45_lband[] =
{
	0,
	0,
	0,
	0,
	0,
	0,
	1,
	1,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
};

// global variables
int                 iCount = 0;
struct vx7903_state *pVz7903_State[4];

/*******************************************
 *
 * I2C routines
 *
 */
int vx7903_i2c_read(struct i2c_adapter *i2c_adap, u8 addr, u8 *pData, u8 bLen)
{
	int ret;
	u8 RegAddr[] = { pData[0] };
	struct i2c_msg msg[] =
	{
		{ .addr = addr, .flags = I2C_M_RD, .buf = pData, .len = bLen}
	};
//	dprintk(70, "%s: len = %d MemAddr = 0x%x addr = 0x%02x\n", __func__, bLen, pData[0], addr);
	ret = i2c_transfer(i2c_adap, msg, 1);

#if 0
	if (paramDebug > 50)
	{
		int i;

		dprintk(1,"%s: I2C message = ");
		for (i = 0; i < bLen; i++)
		{
			printk("[DATA]%x ", pData[i]);
		}
		printk("\n");
	}
#endif
	if (ret != 1)
	{
		if (ret != -ERESTARTSYS)
		{
			printk("%s: error, Reg=[0x%02x], Status=%d\n", __func__, RegAddr[0], ret);
		}
		return ret < 0 ? ret : -EREMOTEIO;
	}
	return 0;
}

int vx7903_i2c_write(struct i2c_adapter *i2c_adap, u8 addr, u8 *pData, u8 bLen)
{
	int ret;
	struct i2c_msg i2c_msg = {.addr = addr, .flags = 0, .buf = pData, .len = bLen };

	//dprintk(70, "%s: pVx7903_State->i2c = 0x%x I2C addr=%x\n", _func__, (int)i2c_adap, addr);

#if 0
	if (paramDebug > 50)
	{
		int i;

		dprintk(1,"%s: I2C message = ");
		for (i = 0; i < bLen; i++)
		{
			printk("%02x ", pData[i]);
		}
		printk("  write Data, Addr: %x\n", addr);
	}
#endif

	ret = i2c_transfer(i2c_adap, &i2c_msg, 1);
	if (ret != 1)
	{
		if (ret != -ERESTARTSYS)
		{
			dprintk(1, "%s: Reg=[0x%04x], Data=[0x%02x ...], Count=%u, Status=%d\n", __func__, pData[0], pData[1], bLen, ret);
		}
		return ret < 0 ? ret : -EREMOTEIO;
	}
	return 0;
}

u32 vx7903_open_repeater(u32 tuner_id)
{
	u32 err = 0;
	u8 data = 0;
	u8 acWriteData[3] = { 0, 0, 0 };

	acWriteData[0] = 0xf1;
	acWriteData[1] = 0x2a;
	err  = vx7903_i2c_write(pVz7903_State[tuner_id]->i2c, pVz7903_State[tuner_id]->config->DemodI2cAddr, acWriteData, 2);
	err |= vx7903_i2c_read(pVz7903_State[tuner_id]->i2c, pVz7903_State[tuner_id]->config->DemodI2cAddr, &data, 1);

	data |= 0x80;
	acWriteData[0] = 0xf1;
	acWriteData[1] = 0x2a;
	acWriteData[2] = data;
	err |= vx7903_i2c_write(pVz7903_State[tuner_id]->i2c, pVz7903_State[tuner_id]->config->DemodI2cAddr, acWriteData, 3);
	return err;
}

/*====================================================*
    vx7903_register_real_write
   --------------------------------------------------
    Description     register write
    Argument        RegAddr (Register Address)
                    RegData (Write data)
    Return Value    bool (TRUE:success, FALSE:error)
 *====================================================*/
bool vx7903_register_real_write(u32 tuner_id, u8 RegAddr, u8 RegData)
{
	u32 err = 0;
	u8 acWriteData[2] = { 0, 0 };

//	dprintk(100, "%s: > pVz7903_State->fe = %x\n", (int)pVz7903_State[tuner_id]->fe, __func__);

	if (pVz7903_State[tuner_id]->fe->ops.i2c_gate_ctrl)
	{
		pVz7903_State[tuner_id]->fe->ops.i2c_gate_ctrl(pVz7903_State[tuner_id]->fe, 0);
		msleep(10);
		pVz7903_State[tuner_id]->fe->ops.i2c_gate_ctrl(pVz7903_State[tuner_id]->fe, 1);
	}
	vx7903_open_repeater(tuner_id);
	acWriteData[0] = RegAddr;
	acWriteData[1] = RegData;
	err = vx7903_i2c_write(pVz7903_State[tuner_id]->i2c, pVz7903_State[tuner_id]->config->addr, acWriteData, 2);
	//dprintk(100, "%s: < err = 0x%x, IOHandle = %d\n", __func__, err, IOHandle);
	return TRUE;
}

/*====================================================*
    vx7903_register_real_read
   --------------------------------------------------
    Description     register read
    Argument        RegAddr (Register Address)
                    apData (Read data)
    Return Value    bool (TRUE:success, FALSE:error)
 *====================================================*/
bool vx7903_register_real_read(u32 tuner_id, u8 RegAddr, u8 *apData)
{
	u32 err = 0;
	u8 i_data = 0;

//	dprintk(100, "%s: > pVz7903_State->fe = %x\n", (int)pVz7903_State[tuner_id]->fe, __func__);
//	dprintk(150, "%s: RegAddr = %x\n", __func__, (int)RegAddr);
	if (pVz7903_State[tuner_id]->fe->ops.i2c_gate_ctrl)
	{
		pVz7903_State[tuner_id]->fe->ops.i2c_gate_ctrl(pVz7903_State[tuner_id]->fe, 0);
		msleep(10);
		pVz7903_State[tuner_id]->fe->ops.i2c_gate_ctrl(pVz7903_State[tuner_id]->fe, 1);
	}
	vx7903_open_repeater(tuner_id);
	i_data = RegAddr;
//	dprintk(70, "%s: pVz7903_State->config = %x\n", __func__, (int)pVz7903_State[tuner_id]->config);
	err |= vx7903_i2c_write(pVz7903_State[tuner_id]->i2c, pVz7903_State[tuner_id]->config->addr, &i_data, 1);
	vx7903_open_repeater(tuner_id);
	err |= vx7903_i2c_read(pVz7903_State[tuner_id]->i2c, pVz7903_State[tuner_id]->config->addr, &i_data, 1);
	*apData = i_data;
//	dprintk(70, "%s:[%d] err = 0x%x, i_data = 0x%02x\n", __func__, __LINE__, err, i_data);
	return TRUE;
}

void vx7903_set_lpf(VX7903_LPF_FC lpf)
{
	VX7903_d_reg[VX7903_REG_08] &= 0xF0;
	VX7903_d_reg[VX7903_REG_08] |= (lpf & 0x0F);
}

bool vx7903_pll_setdata_once(u32 tuner_id, VX7903_INIT_REG_DATA RegAddr, u8 RegData)
{
	bool bRetValue;
	bRetValue = vx7903_register_real_write(tuner_id, RegAddr, RegData);
	return bRetValue;
}

u8 vx7903_pll_getdata_once(u32 tuner_id, VX7903_INIT_REG_DATA RegAddr)
{
	u8 data;

	vx7903_register_real_read(tuner_id, RegAddr, &data);
	return data;
}

void vx7903_get_lock_status(u32 tuner_id, bool *pbLock)
{
	VX7903_d_reg[VX7903_REG_0D] = vx7903_pll_getdata_once(tuner_id, VX7903_REG_0D);
	if (VX7903_d_reg[VX7903_REG_0D] & 0x40)
	{
		*pbLock = TRUE;
	}
	else
	{
		*pbLock = FALSE;
	}
}

bool vx7903_set_searchmode(u32 tuner_id, PVX7903_CONFIG_STRUCT apConfig)
{
	bool bRetValue;

	if (apConfig->b_VX7903_fast_search_mode)
	{
		VX7903_d_reg[VX7903_REG_03] = vx7903_pll_getdata_once(tuner_id, VX7903_REG_03);
		VX7903_d_reg[VX7903_REG_03] |= 0x01;
		bRetValue = vx7903_pll_setdata_once(tuner_id, VX7903_REG_03, VX7903_d_reg[VX7903_REG_03]);
		if (!bRetValue)
		{
			return bRetValue;
		}
	}
	else
	{
		VX7903_d_reg[VX7903_REG_03] = vx7903_pll_getdata_once(tuner_id, VX7903_REG_03);
		VX7903_d_reg[VX7903_REG_03] &= 0xFE;
		bRetValue = vx7903_pll_setdata_once(tuner_id, VX7903_REG_03, VX7903_d_reg[VX7903_REG_03]);
		if (!bRetValue)
		{
			return bRetValue;
		}
	}
	return TRUE;
}

bool vx7903_Set_Operation_Param(u32 tuner_id, PVX7903_CONFIG_STRUCT apConfig)
{
	u8 u8TmpData;
	bool bRetValue;

	VX7903_d_reg[VX7903_REG_01] = vx7903_pll_getdata_once(tuner_id, VX7903_REG_01);  // Volatile
	VX7903_d_reg[VX7903_REG_05] = vx7903_pll_getdata_once(tuner_id, VX7903_REG_05);  // Volatile

	if (apConfig->b_VX7903_loop_through && apConfig->b_VX7903_tuner_standby)
	{
		//LoopThrough = Enable , TunerMode = Standby
		VX7903_d_reg[VX7903_REG_01] |= (0x01 << 3); //BB_REG_enable
		VX7903_d_reg[VX7903_REG_01] |= (0x01 << 2); //HA_LT_enable
		VX7903_d_reg[VX7903_REG_01] |= (0x01 << 1); //LT_enable
		VX7903_d_reg[VX7903_REG_01] |= 0x01;//STDBY
		VX7903_d_reg[VX7903_REG_05] |= (0x01 << 3); //pfd_rst STANDBY
		bRetValue = vx7903_pll_setdata_once(tuner_id, VX7903_REG_05, VX7903_d_reg[VX7903_REG_05]);
		if (!bRetValue)
		{
			return bRetValue;
		}
		bRetValue = vx7903_pll_setdata_once(tuner_id, VX7903_REG_01, VX7903_d_reg[VX7903_REG_01]);
		if (!bRetValue)
		{
			return bRetValue;
		}
	}
	else if (apConfig->b_VX7903_loop_through && !(apConfig->b_VX7903_tuner_standby))
	{
		// LoopThrough = Enable, TunerMode = Normal
		VX7903_d_reg[VX7903_REG_01] |= (0x01 << 3);            // BB_REG_enable
		VX7903_d_reg[VX7903_REG_01] |= (0x01 << 2);            // HA_LT_enable
		VX7903_d_reg[VX7903_REG_01] |= (0x01 << 1);            // LT_enable
		VX7903_d_reg[VX7903_REG_01] &= (~(0x01)) & 0xFF;       // NORMAL
		VX7903_d_reg[VX7903_REG_05] &= (~(0x01 << 3)) & 0xFF;  // pfd_rst NORMAL

		bRetValue = vx7903_pll_setdata_once(tuner_id, VX7903_REG_01, VX7903_d_reg[VX7903_REG_01]);
		if (!bRetValue)
		{
			return bRetValue;
		}
		bRetValue = vx7903_pll_setdata_once(tuner_id, VX7903_REG_05, VX7903_d_reg[VX7903_REG_05]);
		if (!bRetValue)
		{
			return bRetValue;
		}
	}
	else if (!(apConfig->b_VX7903_loop_through) && apConfig->b_VX7903_tuner_standby)
	{
		// LoopThrough = Disable, TunerMode = Standby
		VX7903_d_reg[VX7903_REG_01] &= (~(0x01 << 3)) & 0xFF;  // BB_REG_disable
		VX7903_d_reg[VX7903_REG_01] &= (~(0x01 << 2)) & 0xFF;  // HA_LT_disable
		VX7903_d_reg[VX7903_REG_01] &= (~(0x01 << 1)) & 0xFF;  // LT_disable
		VX7903_d_reg[VX7903_REG_01] |=    0x01;                // STDBY
		VX7903_d_reg[VX7903_REG_05] |=   (0x01 << 3);          // pfd_rst STANDBY
		bRetValue = vx7903_pll_setdata_once(tuner_id, VX7903_REG_05, VX7903_d_reg[VX7903_REG_05]);
		if (!bRetValue)
		{
			return bRetValue;
		}
		bRetValue = vx7903_pll_setdata_once(tuner_id, VX7903_REG_01, VX7903_d_reg[VX7903_REG_01]);
		if (!bRetValue)
		{
			return bRetValue;
		}
	}
	else  // !(iLoopThrough) && !(iTunerMode)
	{
		// LoopThrough = Disable, TunerMode = Normal
		VX7903_d_reg[VX7903_REG_01] |= (0x01 << 3);            // BB_REG_enable
		VX7903_d_reg[VX7903_REG_01] |= (0x01 << 2);            // HA_LT_enable
		VX7903_d_reg[VX7903_REG_01] &= (~(0x01 << 1)) & 0xFF;  // LT_disable
		VX7903_d_reg[VX7903_REG_01] &= (~(0x01)) & 0xFF;       // NORMAL
		VX7903_d_reg[VX7903_REG_05] &= (~(0x01 << 3)) & 0xFF;  // pfd_rst NORMAL
		bRetValue = vx7903_pll_setdata_once(tuner_id, VX7903_REG_01, VX7903_d_reg[VX7903_REG_01]);
		if (!bRetValue)
		{
			return bRetValue;
		}
		bRetValue = vx7903_pll_setdata_once(tuner_id, VX7903_REG_05, VX7903_d_reg[VX7903_REG_05]);
		if (!bRetValue)
		{
			return bRetValue;
		}
	}
	/*Head Amp*/
	u8TmpData = vx7903_pll_getdata_once(tuner_id, VX7903_REG_01);
	if (u8TmpData & 0x04)
	{
		apConfig->b_VX7903_head_amp = TRUE;
	}
	else
	{
		apConfig->b_VX7903_head_amp = FALSE;
	}
	return TRUE;
}

bool vx7903_initialize(u32 tuner_id, PVX7903_CONFIG_STRUCT apConfig)
{
	u8 i_data, i;
	bool bRetValue;

	if (apConfig == NULL)
	{
		return FALSE;
	}
	/* Soft Reset ON */
	vx7903_pll_setdata_once(tuner_id, VX7903_REG_01, INIT_DUMMY_RESET);
	//msleep(100);
	/* Soft Reset OFF */
	i_data = vx7903_pll_getdata_once(tuner_id, VX7903_REG_01);
	i_data |= 0x10;
	vx7903_pll_setdata_once(tuner_id, VX7903_REG_01, i_data);

	/* ID Check */
	i_data = vx7903_pll_getdata_once(tuner_id, VX7903_REG_00);
//	dprintk(70, "%s: i_data = 0x%02x\n", __func__, i_data);

	if (VX7903_d[VX7903_INICHIP_ID] != i_data)
	{
		return FALSE; //"I2C Comm Error", NULL, MB_ICONWARNING);
	}
	/*LPF Tuning On*/
	//msleep(100);
	VX7903_d_reg[VX7903_REG_0C] |= 0x40;
	vx7903_pll_setdata_once(tuner_id, VX7903_REG_0C, VX7903_d_reg[VX7903_REG_0C]);
	msleep(apConfig->ui_VX7903_LpfWaitTime);
	/*LPF_CLK Setting Addr:0x08 b6*/
	if (apConfig->ui_VX7903_XtalFreqkHz == LPF_CLK_16000kHz)
	{
		VX7903_d_reg[VX7903_REG_08] &= ~(0x40);
	}
	else
	{
		VX7903_d_reg[VX7903_REG_08] |= 0x40;
	}

	/* Timeset Setting Addr: 0x12 b[2-0] */
	if (apConfig->ui_VX7903_XtalFreqkHz == LPF_CLK_16000kHz)
	{
		VX7903_d_reg[VX7903_REG_12] &= 0xF8;
		VX7903_d_reg[VX7903_REG_12] |= 0x03;
	}
	else
	{
		VX7903_d_reg[VX7903_REG_12] &= 0xF8;
		VX7903_d_reg[VX7903_REG_12] |= 0x04;
	}

	/* IQ Output Setting Addr: 0x17 b5 , 0x1B b3 */
	if (apConfig->b_VX7903_iq_output)  // Differential
	{
		VX7903_d_reg[VX7903_REG_17] &= 0xDF;
	}
	else  // SinglEnd
	{
		VX7903_d_reg[VX7903_REG_17] |= 0x20;
	}

	for (i = 0; i < VX7903_INI_REG_MAX; i++)
	{
		if (VX7903_d_flg[i] == TRUE)
		{
			vx7903_pll_setdata_once(tuner_id, (VX7903_INIT_REG_DATA)i, VX7903_d_reg[i]);
		}
	}
	bRetValue = vx7903_Set_Operation_Param(tuner_id, apConfig);
	if (!bRetValue)
	{
		return FALSE;
	}
	bRetValue = vx7903_set_searchmode(tuner_id, apConfig);
	if (!bRetValue)
	{
		return FALSE;
	}
	return TRUE;
}

bool vx7903_LocalLpfTuning(u32 tuner_id, PVX7903_CONFIG_STRUCT apConfig)
{
	u8 i_data, i_data1, i;
	u32 COMP_CTRL;

	if (apConfig == NULL)
	{
		return FALSE;
	}

	/* LPF */
	vx7903_set_lpf(apConfig->VX7903_lpf);

	/* div2 / vco_band */
	for (i = 0; i < 15; i++)
	{
		if (vx7903_local_f[i] == 0)
		{
			continue;
		}
		if ((vx7903_local_f[i + 1] <= apConfig->ui_VX7903_RFChannelkHz) && (vx7903_local_f[i] > apConfig->ui_VX7903_RFChannelkHz))
		{
			i_data = vx7903_pll_getdata_once(tuner_id, VX7903_REG_02);
			i_data &= 0x0F;
			i_data |= ((vx7903_div2[i] << 7) & 0x80);
			i_data |= ((vx7903_vco_band[i] << 4) & 0x70);
			vx7903_pll_setdata_once(tuner_id, VX7903_REG_02, i_data);

			i_data = vx7903_pll_getdata_once(tuner_id, VX7903_REG_03);
			i_data &= 0xBF;
			i_data |= ((vx7903_div45_lband[i] << 6) & 0x40);
			vx7903_pll_setdata_once(tuner_id, VX7903_REG_03, i_data);
		}
	}

	/* PLL Counter */
	{
		int F_ref, pll_ref_div, alpha, N, A, sd;
		int M, beta;
		if (apConfig->ui_VX7903_XtalFreqkHz == DEF_XTAL_FREQ)
		{
			F_ref = apConfig->ui_VX7903_XtalFreqkHz;
			pll_ref_div = 0;
		}
		else
		{
			F_ref = (apConfig->ui_VX7903_XtalFreqkHz >> 1);
			pll_ref_div = 1;
		}
		M = ((apConfig->ui_VX7903_RFChannelkHz * 1000) / (F_ref));
		alpha = (int)(M + 500) / 1000 * 1000;
		beta = M - alpha;
		N = (int)((alpha - 12000) / 4000);
		A = alpha / 1000 - 4 * (N + 1) - 5;  // ???
		if (beta >= 0)
		{
			sd = (int)(1048576 * beta / 1000);  // sd = (int)(pow(2.,20.) * beta);
		}
		else
		{
			sd = (int)(0x400000 + 1048576 * beta / 1000);  // sd = (int)(0x400000 + pow(2.,20.) * beta)
		}

		dprintk(70, "%s: beta = %d alpha = %d M = %d A = %d N = %d sd = %x\n", __func__, beta, alpha, M, A, N, sd);
		VX7903_d_reg[VX7903_REG_06] &= 0x40;
		VX7903_d_reg[VX7903_REG_06] |= ((pll_ref_div << 7) | N);
		vx7903_pll_setdata_once(tuner_id, VX7903_REG_06, VX7903_d_reg[VX7903_REG_06]);

		VX7903_d_reg[VX7903_REG_07] &= 0xF0;
		VX7903_d_reg[VX7903_REG_07] |= (A & 0x0F);
		vx7903_pll_setdata_once(tuner_id, VX7903_REG_07, VX7903_d_reg[VX7903_REG_07]);

		/* LPF_CLK, LPF_FC */
		i_data = VX7903_d_reg[VX7903_REG_08] & 0xF0;
		if (apConfig->ui_VX7903_XtalFreqkHz >= 6000 && apConfig->ui_VX7903_XtalFreqkHz < 66000)
		{
			i_data1 = ((u8)((apConfig->ui_VX7903_XtalFreqkHz + 2000) / 4000) * 2 - 4) >> 1;
		}
		else if (apConfig->ui_VX7903_XtalFreqkHz < 6000)
		{
			i_data1 = 0x00;
		}
		else
		{
			i_data1 = 0x0F;
		}
//		dprintk(70, "%s: i_data1 = %x XtalFreqKHz = %d\n", __func__, i_data1, apConfig->ui_VX7903_XtalFreqkHz);
		i_data |= i_data1;
		vx7903_pll_setdata_once(tuner_id, VX7903_REG_08, i_data);

		VX7903_d_reg[VX7903_REG_09] &= 0xC0;
		VX7903_d_reg[VX7903_REG_09] |= ((sd >> 16) & 0x3F);
		VX7903_d_reg[VX7903_REG_0A] = (u8)((sd >> 8) & 0xFF);
		VX7903_d_reg[VX7903_REG_0B] = (u8)(sd & 0xFF);
		vx7903_pll_setdata_once(tuner_id, VX7903_REG_09, VX7903_d_reg[VX7903_REG_09]);
		vx7903_pll_setdata_once(tuner_id, VX7903_REG_0A, VX7903_d_reg[VX7903_REG_0A]);
		vx7903_pll_setdata_once(tuner_id, VX7903_REG_0B, VX7903_d_reg[VX7903_REG_0B]);
	}

	/* COMP_CTRL */
#if 0
	if (apConfig->ui_VX7903_XtalFreqkHz == 17000)
	{
		COMP_CTRL = 0x03;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 18000)
	{
		COMP_CTRL = 0x00;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 19000)
	{
		COMP_CTRL = 0x01;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 20000)
	{
		COMP_CTRL = 0x03;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 21000)
	{
		COMP_CTRL = 0x04;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 22000)
	{
		COMP_CTRL = 0x01;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 23000)
	{
		COMP_CTRL = 0x02;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 24000)
	{
		COMP_CTRL = 0x03;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 25000)
	{
		COMP_CTRL = 0x04;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 26000)
	{
		COMP_CTRL = 0x02;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 27000)
	{
		COMP_CTRL = 0x03;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 28000)
	{
		COMP_CTRL = 0x04;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 29000)
	{
		COMP_CTRL = 0x04;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 30000)
	{
		COMP_CTRL = 0x03;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 31000)
	{
		COMP_CTRL = 0x04;
	}
	else if (apConfig->ui_VX7903_XtalFreqkHz == 32000)
	{
		COMP_CTRL = 0x04;
	}
	else
	{
		COMP_CTRL = 0x02;
	}
#else
	switch (apConfig->ui_VX7903_XtalFreqkHz)
	{
		case 18000:
		{
			COMP_CTRL = 0x00;
			break;
		}
		case 19000:
		case 22000:
		{
			COMP_CTRL = 0x01;
			break;
		}
		case 23000:
		case 26000:
		default:
		{
			COMP_CTRL = 0x02;
			break;
		}
		case 17000:
		case 20000:
		case 24000:
		case 27000:
		case 30000:
		{
			COMP_CTRL = 0x03;
		}
		case 21000:
		case 25000:
		case 28000:
		case 29000:
		case 31000:
		case 32000:
		{
			COMP_CTRL = 0x04;
			break;
		}
	}
#endif
	VX7903_d_reg[VX7903_REG_1D] &= 0xF8;
	VX7903_d_reg[VX7903_REG_1D] |= COMP_CTRL;
	vx7903_pll_setdata_once(tuner_id, VX7903_REG_1D, VX7903_d_reg[VX7903_REG_1D]);

	/* VCO_TM, LPF_TM */
	i_data = vx7903_pll_getdata_once(tuner_id, VX7903_REG_0C);
	i_data &= 0x3F;
	vx7903_pll_setdata_once(tuner_id, VX7903_REG_0C, i_data);
	msleep(100);  // 1024usec

	/* VCO_TM, LPF_TM */
	i_data = vx7903_pll_getdata_once(tuner_id, VX7903_REG_0C);
	i_data |= 0xC0;
	vx7903_pll_setdata_once(tuner_id, VX7903_REG_0C, i_data);

	//msleep(apConfig->ui_VX7903_LpfWaitTime * 1000);  // jhy modified, wait too long
	msleep(100);  // jhy modified, wait too long

	/* LPF_FC */
	vx7903_pll_setdata_once(tuner_id, VX7903_REG_08, VX7903_d_reg[VX7903_REG_08]);

	/* CSEL_Offset */
	VX7903_d_reg[VX7903_REG_13] &= 0x9F;
	vx7903_pll_setdata_once(tuner_id, VX7903_REG_13, VX7903_d_reg[VX7903_REG_13]);

	/* PLL Lock */
	{
		bool bLock;

		vx7903_get_lock_status(tuner_id, &bLock);
//		drintk(70, "%s: bLock = %d\n",__func__, bLock);
		if (bLock != TRUE)
		{
			return FALSE;
		}
	}
	return TRUE;
}

int nim_vx7903_init(u8 *ptuner_id)
{
	struct _VX7903_CONFIG_STRUCT vx7903_cfg;
	bool rval;
	u32 tuner_id = 0;

	if (ptuner_id)
	{
		tuner_id = *ptuner_id;
	}
	if (vx7903_tuner_cnt >= MAX_TUNER_NUM)
	{
		return -9;
	}
	vx7903_tuner_cnt++;

	//vx7903_cfg = (struct _VX7903_CONFIG_STRUCT*)kmalloc(sizeof(struct _VX7903_CONFIG_STRUCT), GFP_KERNEL);
	memset(&vx7903_cfg, 0, sizeof(struct _VX7903_CONFIG_STRUCT));
	vx7903_cfg.ui_VX7903_RFChannelkHz = 1550000;       /* RFChannelkHz */
	vx7903_cfg.ui_VX7903_XtalFreqkHz = DEF_XTAL_FREQ;  /* XtalFreqKHz */
	vx7903_cfg.b_VX7903_fast_search_mode = FALSE;      /* b_fast_search_mode */
	vx7903_cfg.b_VX7903_loop_through = FALSE;          /* b_loop_through */
	vx7903_cfg.b_VX7903_tuner_standby = FALSE;         /* b_tuner_standby */
	vx7903_cfg.b_VX7903_head_amp = TRUE;               /* b_head_amp */
	vx7903_cfg.VX7903_lpf = VX7903_LPF_FC_10MHz;       /* lpf */
	vx7903_cfg.ui_VX7903_LpfWaitTime = 1;              /* VX7903_LpfWaitTime */
	vx7903_cfg.ui_VX7903_FastSearchWaitTime = 4;       /* VX7903_FastSearchWaitTime */  // jhy change: was 20
	vx7903_cfg.ui_VX7903_NormalSearchWaitTime = 15;    /* VX7903_NormalSearchWaitTime */

	//bool b_VX7903_iq_output;

	rval = vx7903_initialize(tuner_id, &vx7903_cfg);
//	dprintk(70, "%s: rval = %d\n", __func__, rval);
	dprintk(20, "VX7903 tuner %d initialized\n", vx7903_tuner_cnt);
//	dprintk(100, "% <\n", __func__);
	return SUCCESS;
}

/*****************************************************************************
 * int nim_vx7903_control(u32 freq, u8 sym, u8 cp)
 *
 * Tuner write operation
 *
 * Arguments:
 * Parameter1: u32 freq : Synthesiser programmable divider(MHz)
 * Parameter2: u32 sym : symbol rate (KHz)
 *
 * Return Value: int : Result
 *****************************************************************************/
int nim_vx7903_control(u8 tuner_id, u32 freq, u32 sym)
{
	u32 Rs, BW;
	u8 LPF = 15;
	bool rval;
	struct _VX7903_CONFIG_STRUCT vx7903_cfg;

	if (tuner_id >= vx7903_tuner_cnt
	||  tuner_id >= MAX_TUNER_NUM)
	{
		return -9;
	}
	//vx7903_cfg = (struct _VX7903_CONFIG_STRUCT*)kmalloc(sizeof(struct _VX7903_CONFIG_STRUCT), GFP_KERNEL);
	memset(&vx7903_cfg, 0, sizeof(struct _VX7903_CONFIG_STRUCT));
	vx7903_cfg.ui_VX7903_RFChannelkHz = 1550000;       /* RFChannelkHz */
	vx7903_cfg.ui_VX7903_XtalFreqkHz = DEF_XTAL_FREQ;  /* XtalFreqKHz */
	vx7903_cfg.b_VX7903_fast_search_mode = FALSE;      /* b_fast_search_mode */
	vx7903_cfg.b_VX7903_loop_through = FALSE;          /* b_loop_through */
	vx7903_cfg.b_VX7903_tuner_standby = FALSE;         /* b_tuner_standby */
	vx7903_cfg.b_VX7903_head_amp = TRUE;               /* b_head_amp */
	vx7903_cfg.VX7903_lpf = VX7903_LPF_FC_10MHz;       /* lpf */
	vx7903_cfg.ui_VX7903_LpfWaitTime = 1;              /* VX7903_LpfWaitTime */  // jhy change was 20
	vx7903_cfg.ui_VX7903_FastSearchWaitTime = 4;       /* VX7903_FastSearchWaitTime */
	vx7903_cfg.ui_VX7903_NormalSearchWaitTime = 15;    /* VX7903_NormalSearchWaitTime */

	/* LPF cut_off */
	Rs = sym;
	if (freq > 2200)
	{
		freq = 2200;
	}
	if (Rs == 0)
	{
		Rs = 45000;
	}
	BW = Rs * 135 / 200;  // rolloff factor is 35%
	if (Rs < 6500)
	{
		BW += 4000;  // add 4M when Rs < 6.5M, since we need shift 4M to avoid DC
	}
	BW += 2000;  // add 2M for LNB frequency shifting
//	ZCY: the cutoff freq of VX7903 is not 3dB point, it more like 0.1dB, so need not 30%
//	BW = BW * 130 / 100;  // 0.1dB to 3dB need about 30% transition band for 7th order LPF
	BW = BW * 108 / 100;  // add 8% margin since fc is not very accurate
	if (BW < 4000)
	{
		BW = 4000;  // Sharp VX7903 LPF can be tuned from 4M to 34M, step is 2.0M
	}
	if (BW > 34000)
	{
		BW = 34000;  // the range can be 6M~~34M actually, 4M is not good
	}

	// determine LPF index
#if 1
	if (BW <= 4000)
	{
		LPF = 0;
	}
	else if (BW <= 6000)
	{
		LPF = 1;
	}
	else if (BW <= 8000)
	{
		LPF = 2;
	}
	else if (BW <= 10000)
	{
		LPF = 3;
	}
	else if (BW <= 12000)
	{
		LPF = 4;
	}
	else if (BW <= 14000)
	{
		LPF = 5;
	}
	else if (BW <= 16000)
	{
		LPF = 6;
	}
	else if (BW <= 18000)
	{
		LPF = 7;
	}
	else if (BW <= 20000)
	{
		LPF = 8;
	}
	else if (BW <= 22000)
	{
		LPF = 9;
	}
	else if (BW <= 24000)
	{
		LPF = 10;
	}
	else if (BW <= 26000)
	{
		LPF = 11;
	}
	else if (BW <= 28000)
	{
		LPF = 12;
	}
	else if (BW <= 30000)
	{
		LPF = 13;
	}
	else if (BW <= 32000)
	{
		LPF = 14;
	}
	else
	{
		LPF = 15;
	}
#else  // alternative: calculate LPF from BW
	LPF = (BW - (BW % 2000) / 2000) - 1;
//	if (LPF != LPFc)
//	{
//		dprintk(1, "%s: Error: LPF = %d, LPFc = %d\n", __func__, LPF, LPFc);
//	}
#endif
	vx7903_cfg.VX7903_lpf = (VX7903_LPF_FC)LPF;
	vx7903_cfg.ui_VX7903_RFChannelkHz = freq * 1000;
	rval = vx7903_LocalLpfTuning(tuner_id, &vx7903_cfg);
//	dprintk(100, "%s < rval = %d\n", __func__, rval);
	return SUCCESS;
}

/*****************************************************************************
 * int nim_vx7903_status(u8 *lock)
 *
 * Tuner read operation
 *
 * Arguments:
 * Parameter1: u8 *lock : Phase lock status
 *
 * Return Value: int : Result
 *****************************************************************************/
int nim_vx7903_status(u8 tuner_id, u8 *lock)
{
	dprintk(100, "%s > tuner_id = %d\n", __func__, tuner_id);

	if (tuner_id >= vx7903_tuner_cnt
	||  tuner_id >= MAX_TUNER_NUM)
	{
		dprintk(1, "%s: Error\n", __func__);
		*lock = 0;
		return -9;
	}
	*lock = 1;
	dprintk(20, "Tuner is locked\n");
	return SUCCESS;
}

/*---add for Dvb-------*/
int vx7903_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct vx7903_state *vx7903_state = fe->tuner_priv;

	*frequency = vx7903_state->frequency;
	dprintk(20, "Frequency = %d kHz\n", vx7903_state->frequency);
	return 0;
}

int vx7903_set_frequency(struct dvb_frontend *fe, u32 frequency)
{
	struct vx7903_state *vx7903_state = fe->tuner_priv;
	struct stv090x_state *state = fe->demodulator_priv;
	int err = 0;

	vx7903_state->frequency = frequency;
	err = nim_vx7903_control(vx7903_state->index, frequency / 1000, state->srate / 1000);
	if (err != 0)
	{
		dprintk(1, "%s: nim_vx7903_control error!\n", __func__);
	}
	dprintk(20, "Frequency = %d MHz, Symbolrate = %d MS/s\n", frequency / 1000, state->srate / 1000);
	return 0;
}

int vx7903_set_bandwidth(struct dvb_frontend *fe, u32 bandwidth)
{
	struct vx7903_state *state = fe->tuner_priv;

	state->bandwidth = bandwidth;
	return 0;
}

int vx7903_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth)
{
	struct vx7903_state *state = fe->tuner_priv;

	*bandwidth = state->bandwidth;
	printk("%s: Bandwidth = %d kHz\n", __func__, state->bandwidth);
	return 0;
}

int vx7903_get_status(struct dvb_frontend *fe, u32 *status)
{
	struct vx7903_state *state = fe->tuner_priv;
	u8 lock;

	nim_vx7903_status(state->index, &lock);
	*status = (u32)lock;
	return 0;
}

static int vx7903_set_params(struct dvb_frontend *fe, struct dvb_frontend_parameters *p)
{
	struct vx7903_state *state = fe->tuner_priv;

	dprintk(20, "Frequency = %d MHz, Symbol rate = %d MS/s\n", p->frequency, p->u.qpsk.symbol_rate);
	state->frequency = p->frequency;
	return nim_vx7903_control(state->index, p->frequency, p->u.qpsk.symbol_rate);
}

static struct dvb_tuner_ops vx7903_ops =
{
	.info =
	{
		.name           = "Sharp BS2S7VZ7903A",
		.frequency_min  =  950000,
		.frequency_max  = 2150000,
		.frequency_step = 0
	},
	.get_status         = vx7903_get_status,
	.set_params         = vx7903_set_params,
	.release            = NULL,
	.get_frequency      = vx7903_get_frequency,
	.get_bandwidth      = vx7903_get_bandwidth,
	.set_frequency      = vx7903_set_frequency,
	.set_bandwidth      = vx7903_set_bandwidth
};

struct dvb_frontend *vx7903_attach(struct dvb_frontend *fe, struct vx7903_config *config, struct i2c_adapter *i2c)
{
	struct vx7903_state *state = NULL;

	if ((state = kzalloc(sizeof(struct vx7903_state), GFP_KERNEL)) == NULL)
	{
		goto exit;
	}
	state->i2c        = i2c;
	state->fe         = fe;
	state->config     = config;
	state->bandwidth  = 125000;
	state->symbolrate = 0;
	state->index      = iCount;
	fe->tuner_priv    = state;
	fe->ops.tuner_ops = vx7903_ops;

	pVz7903_State[iCount] = state;

	nim_vx7903_init((u8 *)&iCount);
	iCount++;
	dprintk(20, "Attaching %s tuner\n", vx7903_ops.info.name);
	return fe;

exit:
	kfree(state);
	return NULL;
}

int tuner_vx7903_identify(struct dvb_frontend *fe, struct vx7903_config *config, struct i2c_adapter *i2c)
{
	u8 data = 0;
	struct vx7903_state state;

	state.fe     = fe;
	state.i2c    = i2c;
	state.config = config;

	pVz7903_State[iCount] = &state;

	vx7903_register_real_read(iCount, 0, &data);
	if (0x68 != data)
	{
		return UNKNOWN_DEVICE;
	}
	return 0;
}
// vim:ts=4
