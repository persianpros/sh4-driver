/*****************************************************************************/
/* COPYRIGHT (C) 2009 STMicroelectronics - All Rights Reserved               */
/* ST makes no warranty express or implied including but not limited to,     */
/* any warranty of                                                           */
/*                                                                           */
/*   (i)  merchantability or fitness for a particular purpose and/or         */
/*   (ii) requirements, for a particular purpose in relation to the LICENSED */
/*        MATERIALS, which is provided "AS IS", WITH ALL FAULTS. ST does not */
/*        represent or warrant that the LICENSED MATERIALS provided here     */
/*        under is free of infringement of any third party patents,          */
/*        copyrights,trade secrets or other intellectual property rights.    */
/*        ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE       */
/*        EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                    */
/*                                                                           */
/*****************************************************************************/
/**
 @File chip_0367ter.c
 @brief

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

extern IOARCH_HandleData_t IOARCH_Handle[TUNER_IOARCH_MAX_HANDLES];

extern int paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[chip_0367ter] "

#define REPEATER_ON    1
#define REPEATER_OFF   0
#define WAITFORLOCK    1

#define I2C_HANDLE(x)  (*((YWI2C_Handle_T *)x))

/* ----------------------------------------------------------------------------
 Name: ChipUpdateDefaultValues_0367ter

 Description:update the default values of the RegMap chip

 Parameters:hChip ==> handle to the chip
    :: pRegMap ==> pointer to

 Return Value:YW_NO_ERROR if ok, YWHAL_ERROR_INVALID_HANDLE otherwise
 ---------------------------------------------------------------------------- */
u32 ChipUpdateDefaultValues_0367ter(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u16 RegAddr, u8 Data, int reg)
{
	u32 error = YW_NO_ERROR;
//	int reg;

	if (DeviceMap != NULL)
	{
//		for (reg = 0; reg < DeviceMap->Registers; reg++)
		{
			DeviceMap->RegMap[reg].Address = RegAddr;
			DeviceMap->RegMap[reg].ResetValue = Data;
			DeviceMap->RegMap[reg].Value = Data;
		}
	}
	else
	{
		error = YWHAL_ERROR_INVALID_HANDLE;
	}
	return error;
}

/* ----------------------------------------------------------------------------
 Name: ChipSetOneRegister_0367ter()

 Description: Writes Value to the register specified by RegAddr

 Parameters:hChip ==> handle to the chip
          RegAddr ==>Address of the register
            Value ==>Value to be written to register
 Return Value: ST_NO_ERROR (SUCCESS)
---------------------------------------------------------------------------- */
u32 ChipSetOneRegister_0367ter(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u16 RegAddr, u8 Data)
{
	s32 regIndex;

	if (DeviceMap)
	{
		regIndex = ChipGetRegisterIndex(DeviceMap, IOHandle, RegAddr);
		if ((regIndex >= 0)
		&&  (regIndex < DeviceMap->Registers))
		{
			DeviceMap->RegMap[regIndex].Value = Data;
			ChipSetRegisters_0367ter(DeviceMap, IOHandle, RegAddr, 1);
		}
	}
	else
	{
		return YWHAL_ERROR_INVALID_HANDLE;
	}
	return DeviceMap->Error;
}

void stv0367ter_write(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u8 *pcData, int nbdata)
{
	stv0367_write(DeviceMap, IOHandle, pcData, nbdata);
}

void stv0367ter_read(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u8 *pcData, int NbRegs)
{
	stv0367_read(DeviceMap, IOHandle, pcData, NbRegs);
}

/***********************************************************************************
 **FUNCTION  ::  ChipSetRegisters_0367ter
 **ACTION    ::  Set values of consecutive's registers (values are taken in RegMap)
 **PARAMS IN ::  hChip       ==> Handle to the chip
 **              FirstReg    ==> Id of the first register
 **              NbRegs      ==> Number of register to write
 **PARAMS OUT::  NONE
 **RETURN    ::  Error
 ***********************************************************************************/
u32 ChipSetRegisters_0367ter(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, int FirstRegAddr, int NbRegs)
{
	u8 data[100], nbdata = 0;
	int i;
	s32 firstRegIndex = -1;

	DeviceMap->Error = 0;
	firstRegIndex = ChipGetRegisterIndex(DeviceMap, IOHandle, FirstRegAddr);
	if (((firstRegIndex >= 0) && ((firstRegIndex + NbRegs - 1) < DeviceMap->Registers)) == FALSE)
	{
//		dprintk(70, "%s: DeviceMap->Registers = %d, __func__, firstRegIndex= %d, NbRegs=%d\n",__func__, DeviceMap->Registers, firstRegIndex, NbRegs);
		DeviceMap->Error = YWHAL_ERROR_BAD_PARAMETER;
		return DeviceMap->Error;
	}

	if (DeviceMap)
	{
		if (NbRegs < 70)
		{
			switch (DeviceMap->Mode)
			{
				case IOREG_MODE_SUBADR_16:
				{
					data[nbdata++] = MSB(FirstRegAddr);  /* 16 bits sub addresses */
				}
				case IOREG_MODE_SUBADR_8:
				{
					data[nbdata++] = LSB(FirstRegAddr);  /* 8 bits sub addresses */
				}
				case IOREG_MODE_NOSUBADR:
				{
					for (i = 0; i < NbRegs; i++)
					{
						data[nbdata++] = DeviceMap->RegMap[firstRegIndex + i].Value;  /* fill data buffer */
					}
					break;
				}
				default:
				{
//					dprintk(1, %s: "Error %d\n", __func__, __LINE__);
					DeviceMap->Error = YWHAL_ERROR_INVALID_HANDLE;
					return YWHAL_ERROR_INVALID_HANDLE;
				}
			}
#if 0
			// lwj add for test
			if (paramDebug > 50)
			{
				int j;

				dprintk(1, "%s: Set register 0x%0x", __func__, FirstRegAddr);
				for (j = 2; j < nbdata; j++)
				{
					printk(", data = 0x%02x", data[j]);
				}
				printk("\n");
			}
			// lwj add for test end
#endif
			stv0367ter_write(DeviceMap, IOHandle, data, nbdata);
		}
		else
		{
			DeviceMap->Error = YWHAL_ERROR_FEATURE_NOT_SUPPORTED;
		}
	}
	else
	{
		return YWHAL_ERROR_INVALID_HANDLE;
	}
	if (paramDebug > 50)
	{
		if (DeviceMap->Error != 0)
		{
			dprintk(1, "%s: DeviceMap->Error=%d, FirstRegAddr=%x\n", __func__, DeviceMap->Error, FirstRegAddr);  // for test
		}
	}
	return DeviceMap->Error;
}

/*****************************************************
 **FUNCTION  ::  ChipSetField_0367ter
 **ACTION    ::  Set value of a field in the chip
 **PARAMS IN ::  hChip   ==> Handle to the chip
 **              FieldId ==> Id of the field
 **              Value   ==> Value to write
 **PARAMS OUT::  NONE
 **RETURN    ::  Error
 *****************************************************/
u32 ChipSetField_0367ter(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FieldId, int Value)
{
	int regValue;
	int mask;
	int Sign;
	int Bits;
	int Pos;

	if (DeviceMap)
	{
		if (FieldId <= 0xffff)
		{
			ChipSetOneRegister_0367ter(DeviceMap, IOHandle, FieldId, Value);
		}
		else
		{
			regValue = ChipGetOneRegister_0367ter(DeviceMap, IOHandle, (FieldId >> 16) & 0xFFFF); /* Read the register */
			Sign = ChipGetFieldSign(FieldId);
			mask = ChipGetFieldMask(FieldId);
			Pos = ChipGetFieldPosition(mask);
			Bits = ChipGetFieldBits(mask, Pos);

			if (Sign == CHIP_SIGNED)
			{
				Value = (Value > 0) ? Value : Value + (Bits);  /* compute signed fieldval */
			}
			Value = mask & (Value << Pos);  /* Shift and mask value */

			regValue = (regValue & (~mask)) + Value;  /* Concat register value and fieldval */
//			dprintk(20, "(FieldId >> 16) & 0xFFFF = %x, regValue = %x\n", (FieldId >> 16) & 0xFFFF, regValue);
			ChipSetOneRegister_0367ter(DeviceMap, IOHandle, (FieldId >> 16) & 0xFFFF, regValue);  /* Write the register */
		}
#if 0
		else
		{
			DeviceMap->Error = YWHAL_ERROR_BAD_PARAMETER;
		}
#endif
	}
	else
	{
		return YWHAL_ERROR_INVALID_HANDLE;
	}
	if (DeviceMap->Error != 0 && paramDebug > 70)
	{
		dprintk(1, "%s: DeviceMap->Error=%d, FirstRegAddr=%x\n", __func__, DeviceMap->Error, FieldId);  // for test
	}
	return DeviceMap->Error;
}

/************************************************************************************
 **FUNCTION  ::  ChipGetRegisters_0367ter
 **ACTION    ::  Get values of consecutive registers (values are writen in RegMap)
 **PARAMS IN ::  hChip       ==> Handle to the chip
 **              FirstReg    ==> Id of the first register
 **              NbRegs      ==> Number of register to read
 **PARAMS OUT::  NONE
 **RETURN    ::  Error
 ************************************************************************************/
u32 ChipGetRegisters_0367ter(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, int FirstRegAddr, int NbRegs)  //, u8 *RegsVal)
{
	u8 data[100], nbdata = 0;
	int i;
	s32 firstRegIndex;

	DeviceMap->Error = 0;
	firstRegIndex = ChipGetRegisterIndex(DeviceMap, IOHandle, FirstRegAddr);

	if (DeviceMap)
	{
		if (NbRegs < 70)
		{
			switch (DeviceMap->Mode)
			{
				case IOREG_MODE_SUBADR_16:
				{
					data[nbdata++] = MSB(FirstRegAddr); /* for 16 bits sub addresses */
				}
				case IOREG_MODE_SUBADR_8:
				{
					data[nbdata++] = LSB(FirstRegAddr);  /* for 8 bits sub addresses */
					stv0367ter_write(DeviceMap, IOHandle, data, nbdata);
				}
				case IOREG_MODE_NOSUBADR:
				{
					stv0367ter_read(DeviceMap, IOHandle, data, NbRegs);
					break;
				}
				default:
				{
					DeviceMap->Error = YWHAL_ERROR_FEATURE_NOT_SUPPORTED;
					return YWHAL_ERROR_FEATURE_NOT_SUPPORTED;
				}
			}
			/* Update RegMap structure */
			for (i = 0; i < NbRegs; i++)
			{
				if (!DeviceMap->Error)
				{
					DeviceMap->RegMap[i + firstRegIndex].Value = data[i];
				}
			}
		}
		else
		{
			DeviceMap->Error = YWHAL_ERROR_FEATURE_NOT_SUPPORTED;
//			dprintk(1, "%s: DeviceMap->Error YWHAL_ERROR_FEATURE_NOT_SUPPORTED ===\n", __func__);
		}
	}
	else
	{
		return YWHAL_ERROR_INVALID_HANDLE;
	}
	if (DeviceMap->Error != 0)
	{
		dprintk(1, "%s: DeviceMap->Error=%d,FirstRegAddr=%x,NbRegs=%d\n", __func__, DeviceMap->Error, FirstRegAddr, NbRegs);  // for test
	}
	return DeviceMap->Error;
}

/*****************************************************
 **FUNCTION  ::  ChipGetOneRegister_0367ter
 **ACTION    ::  Get the value of one register
 **PARAMS IN ::  hChip   ==> Handle to the chip
 **              reg_id  ==> Id of the register
 **PARAMS OUT::  NONE
 **RETURN    ::  Register's value
 *****************************************************/
int ChipGetOneRegister_0367ter(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u16 RegAddr)
{
	//u8 data = 0xFF, dataTab[80];
	u8 data = 0xFF;
	s32 regIndex = -1;

	if (DeviceMap)
	{
		if (ChipGetRegisters_0367ter(DeviceMap, IOHandle, RegAddr, 1) == YW_NO_ERROR)
		{
			regIndex = ChipGetRegisterIndex(DeviceMap, IOHandle, RegAddr);
			data = DeviceMap->RegMap[regIndex].Value;
		}
	}
	else
	{
		DeviceMap->Error = YWHAL_ERROR_INVALID_HANDLE;
	}
	return (data);
}

/* ----------------------------------------------------------------------------
 Name: ChipGetField_0367ter()

 Description:

 Parameters:

 Return Value:
---------------------------------------------------------------------------- */
u8 ChipGetField_0367ter(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FieldId)
{
	s32 value = 0xFF;
	s32 sign, mask, pos, bits;

	if (DeviceMap != NULL)
	{
		if (FieldId <= 0xffff)
		{
			value = ChipGetOneRegister_0367ter(DeviceMap, IOHandle, FieldId);  /* Read the register */
		}
		else
		{
			/* I2C Read : register address set-up */
			sign = ChipGetFieldSign(FieldId);
			mask = ChipGetFieldMask(FieldId);
			pos = ChipGetFieldPosition(mask);
			bits = ChipGetFieldBits(mask, pos);

			value = ChipGetOneRegister_0367ter(DeviceMap, IOHandle, FieldId >> 16); /* Read the register */
			value = (value & mask) >> pos;  /* Extract field */

			if ((sign == CHIP_SIGNED) && (value & (1 << (bits - 1))))
			{
				value = value - (1 << bits);  /* Compute signed value */
			}
		}
	}
	return value;
}

/*****************************************************
 **FUNCTION :: ChipSetFieldImage_0367ter
 **ACTION :: Set value of a field in RegMap
 **PARAMS IN :: hChip ==> Handle to the chip
 **    FieldId ==> Id of the field
 **    Value ==> Value of the field
 **PARAMS OUT:: NONE
 **RETURN :: Error
 *****************************************************/
u32 ChipSetFieldImage_0367ter(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FieldId, s32 Value)
{
	s32 regIndex,
		mask,
		sign,
		bits, regAddress,
		pos;

	if (DeviceMap != NULL)
	{
		regAddress = ChipGetRegAddress(FieldId);
		regIndex = ChipGetRegisterIndex(DeviceMap, IOHandle, regAddress);

		if ((regIndex >= 0) && (regIndex < DeviceMap->Registers))
		{
			sign = ChipGetFieldSign(FieldId);
			mask = ChipGetFieldMask(FieldId);
			pos  = ChipGetFieldPosition(mask);
			bits = ChipGetFieldBits(mask, pos);

			if (sign == CHIP_SIGNED)
			{
				Value = (Value > 0) ? Value : Value + (1 << bits);  /* compute signed fieldval */
			}
			Value = mask & (Value << pos); /* Shift and mask value */
			DeviceMap->RegMap[regIndex].Value = (DeviceMap->RegMap[regIndex].Value & (~mask)) + Value; /* Concat register value and fieldval */
		}
		else
		{
			DeviceMap->Error = YWHAL_ERROR_INVALID_HANDLE;
		}
	}
	else
	{
		return YWHAL_ERROR_INVALID_HANDLE;
	}
	return DeviceMap->Error;
}

/*****************************************************
 **FUNCTION  :: ChipGetFieldImage_0367ter
 **ACTION    :: get the value of a field from RegMap
 **PARAMS IN :: hChip   ==> Handle of the chip
 **             FieldId ==> Id of the field
 **PARAMS OUT:: NONE
 **RETURN    :: field's value
 *****************************************************/
s32 ChipGetFieldImage_0367ter(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FieldId)
{
	s32 value = 0xFF;
	s32 regIndex,
		mask,
		sign,
		bits, regAddress,
		pos;

	if (DeviceMap != NULL)
	{
		regAddress = ChipGetRegAddress(FieldId);
		regIndex = ChipGetRegisterIndex(DeviceMap, IOHandle, regAddress);

		if ((regIndex >= 0) && (regIndex < DeviceMap->Registers))
		{
			sign = ChipGetFieldSign(FieldId);
			mask = ChipGetFieldMask(FieldId);
			pos = ChipGetFieldPosition(mask);
			bits = ChipGetFieldBits(mask, pos);

			if (!DeviceMap->Error)
			{
				value = DeviceMap->RegMap[regIndex].Value;
			}
			value = (value & mask) >> pos;  /* Extract field */

			if ((sign == CHIP_SIGNED) && (value & (1 << (bits - 1))))
			{
				value = value - (1 << bits);  /* Compute signed value */
			}
		}
#if 0
		else
		{
			DeviceMap->Error = YWHAL_ERROR_INVALID_HANDLE;
		}
#endif
	}
	return value;
}

/*****************************************************
 **FUNCTION  ::  ChipWait_Or_Abort
 **ACTION    ::  wait for n ms or abort if requested
 **PARAMS IN ::
 **PARAMS OUT::  NONE
 **RETURN    ::  NONE
 *****************************************************/
void ChipWaitOrAbort_0367ter(BOOL bForceSearchTerm, u32 delay_ms)
{
	//YWOS_TaskSleep(delay_ms);
	u32 i = 0;

	while ((bForceSearchTerm == FALSE) && (i++ < (delay_ms / 10)))
	{
		msleep(10);
	}

	i = 0;
	while ((bForceSearchTerm == FALSE) && (i++ < (delay_ms % 10)))
	{
		msleep(1);
	}
}
// vim:ts=4
