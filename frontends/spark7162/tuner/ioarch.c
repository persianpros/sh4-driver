/*$Source$*/
/*****************************文件头部注释*************************************/
//
//			Copyright (C), 2012-2017, AV Frontier Tech. Co., Ltd.
//
//
// 文 件 名： $RCSfile$
//
// 创 建 者： D26LF
//
// 创建时间： 2012.09.07
//
// 最后更新： $Date$
//
//				$Author$
//
//				$Revision$
//
//				$State$
//
// 文件描述： ioarch
//
/******************************************************************************/

#include "dvb_frontend.h"
#include "mn88472.h"

#define tuner_i2c_addr 0x1c

extern int paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[ioarch] "

int I2C_Read(struct i2c_adapter *i2c_adap, u8 I2cAddr, u8 *pData, u8 bLen)
{
	int ret;
	u8 RegAddr[] = {pData[0]};
	struct i2c_msg msg[] =
	{
		{ .addr = I2cAddr, .flags = I2C_M_RD, .buf = pData, .len = bLen}
	};
	ret = i2c_transfer(i2c_adap, msg, 1);
#if 0
	if (paramDebug > 50)
	{
		int i;

		dprintk(1, "%s: I2C message: ");
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
			dprintk(1, "%s: Read error, Reg = [0x%02x], Status = %d\n", __func__, RegAddr[0], ret);
		}
		return ret < 0 ? ret : -EREMOTEIO;
	}
	return 0;
}

int I2C_Write(struct i2c_adapter *i2c_adap, u8 I2CAddr, u8 *pData, u8 bLen)
{
	int ret;
	struct i2c_msg i2c_msg = { .addr = I2CAddr, .flags = 0, .buf = pData, .len = bLen };

#if 0
	if (paramDebug > 50)
	{
		int i;

		dprintk(1, "%s: I2C message: ");
		for (i = 0; i < bLen; i++)
		{
			printk("%02x ", pData[i]);
		}
		printk(" write Data, I2C addr: 0x%02x\n", I2CAddr);
	}
#endif
	ret = i2c_transfer(i2c_adap, &i2c_msg, 1);
	if (ret != 1)
	{
#if 0
		if (ret != -ERESTARTSYS)
		{
			dprintk(1, "%s: Error, Reg = [0x%04x], Data = [0x%02x ...], Count=%u, Status=%d\n", __func__, pData[0], pData[1], bLen, ret);
		}
#endif
		return ret < 0 ? ret : -EREMOTEIO;
	}
	return 0;
}

int I2C_ReadWrite(void *I2CHandle, TUNER_IOARCH_Operation_t Operation, unsigned short SubAddr, u8 *Data, u32 TransferSize, u32 Timeout)
{
	int Ret = 0;
	u8 SubAddress, SubAddress16bit[2] = {0};
	u8 nsbuffer[50];
	BOOL ADR16_FLAG = FALSE;
	struct i2c_adapter *i2c_adap = (struct i2c_adapter *)I2CHandle;

	if (SubAddr & 0xFF00)
	{
		ADR16_FLAG = TRUE;
	}
	else
	{
		ADR16_FLAG = FALSE;
	}
	if (ADR16_FLAG == FALSE)
	{
		SubAddress = (u8)(SubAddr & 0xFF);
	}
	else
	{
		SubAddress16bit[0] = (u8)((SubAddr & 0xFF00) >> 8);
		SubAddress16bit[1] = (u8)(SubAddr & 0xFF);
	}
	switch (Operation)
	{
		/* ---------- Read ---------- */
		case TUNER_IO_SA_READ:
		{
			if (ADR16_FLAG == FALSE)
			{
				Ret = I2C_Write(i2c_adap, tuner_i2c_addr, &SubAddress, 1); /* fix for cable (297 chip) */
			}
			else
			{
				Ret = I2C_Write(i2c_adap, tuner_i2c_addr, SubAddress16bit, 2);
			}
			/* fallthrough (no break;) */
		}
		case TUNER_IO_READ:
		{
			Ret = I2C_Read(i2c_adap, tuner_i2c_addr, Data, TransferSize);
			break;
		}
		case TUNER_IO_SA_WRITE:
		{
			if (TransferSize >= 50)
			{
				return (YWHAL_ERROR_NO_MEMORY);
			}
			if (ADR16_FLAG == FALSE)
			{
				nsbuffer[0] = SubAddress;
				memcpy((nsbuffer + 1), Data, TransferSize);
				Ret = I2C_Write(i2c_adap, tuner_i2c_addr, nsbuffer, TransferSize + 1);
			}
			else
			{
				nsbuffer[0] = SubAddress16bit[0];
				nsbuffer[1] = SubAddress16bit[1];
				memcpy((nsbuffer + 2), Data, TransferSize);
				Ret = I2C_Write(i2c_adap, tuner_i2c_addr, nsbuffer, TransferSize + 2);
			}
			break;
		}
		default:
		{
			/* ---------- Error ---------- */
			break;
		}
	}
	return Ret;
}
// vim:ts=4
