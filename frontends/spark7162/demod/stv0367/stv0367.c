/****************************************************************************
 *
 * STM STV0367 DVB-T/C demodulator driver
 *
 * Version for Fulan Spark7162
 *
 * Note: actual device is part of the STi7162 SoC
 *
 * Original code:
 * Copyright (C), 2011-2016, AV Frontier Tech. Co., Ltd.
 *
 * Date 2011.05.09
 */
#include <linux/kernel.h>  /* Kernel support */
#include <linux/delay.h>
#include <linux/i2c.h>

#include "ywdefs.h"
#include "ywtuner_ext.h"
#include "tuner_def.h"
#include "ioarch.h"
#include "ioreg.h"
#include "tuner_interface.h"
#include "chip_0367ter.h"
#include "stv0367ter_init.h"
#include "stv0367_inner.h"

extern int paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[stv0367] "

/*****************************************************
 ** FUNCTION   :: IOREG_GetRegAddress
 ** ACTION     ::
 ** PARAMS IN  ::
 ** PARAMS OUT :: mask
 ** RETURN     :: mask
 *****************************************************/
u16 ChipGetRegAddress(u32 FieldId)
{
	u16 RegAddress;

	RegAddress = (FieldId >> 16) & 0xFFFF;  /* FieldId is [reg address][reg address][sign][mask] --- 4 bytes */
	return RegAddress;
}

/*****************************************************
 ** FUNCTION   :: ChipGetFieldMask
 ** ACTION     ::
 ** PARAMS IN  ::
 ** PARAMS OUT :: mask
 ** RETURN :: mask
 *****************************************************/
int ChipGetFieldMask(u32 FieldId)
{
	int mask;
	mask = FieldId & 0xFF;  /* FieldId is [reg address][reg address][sign][mask] --- 4 bytes */
	return mask;
}

/*****************************************************
 ** FUNCTION   :: ChipGetFieldSign
 ** ACTION     ::
 ** PARAMS IN  ::
 ** PARAMS OUT :: sign
 ** RETURN     :: sign
 *****************************************************/
int ChipGetFieldSign(u32 FieldId)
{
	int sign;
	sign = (FieldId >> 8) & 0x01;  /* FieldId is [reg address][reg address][sign][mask] --- 4 bytes */
	return sign;
}

/*****************************************************
 ** FUNCTION   :: ChipGetFieldPosition
 ** ACTION     ::
 ** PARAMS IN  ::
 ** PARAMS OUT :: position
 ** RETURN     :: position
 *****************************************************/
int ChipGetFieldPosition(u8 Mask)
{
	int position = 0, i = 0;

	while ((position == 0) && (i < 8))
	{
		position = (Mask >> i) & 0x01;
		i++;
	}
	return (i - 1);
}

/*****************************************************
 ** FUNCTION   :: ChipGetFieldBits
 ** ACTION     ::
 ** PARAMS IN  ::
 ** PARAMS OUT :: number of bits
 ** RETURN     :: number of bits
 *****************************************************/
int ChipGetFieldBits(int mask, int Position)
{
	int bits, bit;
	int i = 0;

	bits = mask >> Position;
	bit = bits;
	while ((bit > 0) && (i < 8))
	{
		i++;
		bit = bits >> i;
	}
	return i;
}

/* ----------------------------------------------------------------------------
  Name: ChipGetRegisterIndex

  Description:Get the index of a register from the pRegMapImage table

  Parameters:hChip ==> Handle to the chip
        RegId ==> Id of the register (adress)

  Return Value:Index of the register in the register map image
---------------------------------------------------------------------------- */
s32 ChipGetRegisterIndex(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u16 RegId)
{
	s32 regIndex = -1, reg = 0;
	/* s32 top, bottom, mid; to be used for binary search */

	if (DeviceMap)
	{
		while (reg < DeviceMap->Registers)
		{
			if (DeviceMap->RegMap[reg].Address == RegId)
			{
				regIndex = reg;
				break;
			}
			reg++;
		}
	}
	return regIndex;
}

/*****************************************************
 ** FUNCTION   :: stv0367_write
 ** ACTION     :: write nbdata bytes to STV0367 demodulator
 ** PARAMS IN  ::
 ** PARAMS OUT :: none
 ** RETURN     :: none
 *****************************************************/
void stv0367_write(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, unsigned char *pcData, int nbdata)
{
	struct i2c_adapter *i2c = (struct i2c_adapter *)IOHandle;
	int ret;
	struct i2c_msg msg[] =
	{
		{ .addr = 0x38 >> 1, .flags = 0, .buf = pcData, .len = nbdata },
	};

	ret = i2c_transfer(i2c, &msg[0], 1);
	if (ret != 1)
	{
		if (ret != -ERESTARTSYS)
		{
			dprintk(1, "%s: I2C write error, pcData=[0x%08x], Status=%d\n", __func__, (int)pcData, ret);
		}
	}
}

void stv0367_read(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, unsigned char *pcData, int NbRegs)
{
	int ret;
	struct i2c_adapter *i2c = (struct i2c_adapter *)IOHandle;
	struct i2c_msg msg[] =
	{
		{ .addr = 0x38 >> 1, .flags = I2C_M_RD, .buf = pcData, .len = NbRegs }
	};

	ret = i2c_transfer(i2c, msg, 1);
	if (ret != 1)
	{
		if (ret != -ERESTARTSYS)
		{
			dprintk(1, "%s: I2C read error, Status = %d\n", __func__, ret);
		}
	}
}

/***********************************************************************
	Name: demod_stv0367_identify
 ***********************************************************************/
int demod_stv0367_identify(struct i2c_adapter *i2c, u8 ucID)
{
	int ret;
	u8 pucActualID = 0;
//	u8 b0[] = { R367_ID };
	u8 b0[] = { 0 };
	struct i2c_msg msg[] =
	{
		{ .addr = 0x38 >> 1, .flags = 0, .buf = b0, .len = 1 },
		{ .addr = 0x38 >> 1, .flags = I2C_M_RD, .buf = &pucActualID, .len = 1 }
	};

	ret = i2c_transfer(i2c, msg, 2);
	if (ret == 2)
	{
		if (pucActualID == ucID)
		{
			dprintk(20, "Found STM STV0367 demodulator (I2C bus = %d)\n", i2c->nr);
			return YW_NO_ERROR;
		}
//		else
//		{
			dprintk(1, "STM STV0367 not found (hardware faulty?)\n");  // question
//			return YWHAL_ERROR_UNKNOWN_DEVICE;
//		}
	}
	return YWHAL_ERROR_UNKNOWN_DEVICE;
}
// vim:ts=4
