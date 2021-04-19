/****************************************************************
 *
 * Copyright (C), 2005-2010, AV Frontier Tech. Co., Ltd.
 *
 * Date: 12-Oct-2005
 *
 ****************************************************************/
#ifndef __TUNER_IOREG_H
#define __TUNER_IOREG_H

#define FIELD_TYPE_UNSIGNED 0
#define FIELD_TYPE_SIGNED 1

/* default timeout (for all I/O (I2C) operations) */
#define IOREG_DEFAULT_TIMEOUT 100

#define LSB(X) ((X & 0xFF))
#define MSB(Y) ((Y>>8)& 0xFF)

typedef enum
{
	IOREG_NO = 0,
	IOREG_YES = 1
} TUNER_IOREG_Flag_t;

/* register and address width: 8bits x 8addresses, 16x16 or 32x32 */
typedef enum
{
	REGSIZE_8BITS,
	REGSIZE_16BITS,
	REGSIZE_32BITS
} TUNER_IOREG_RegisterSize_t;


/* Addressing mode - 8 bit address, 16 bit address, no address (0) */
typedef enum
{
	IOREG_MODE_SUBADR_8, /* <addr><reg8><data><data>... */
	IOREG_MODE_SUBADR_16, /* <addr><reg8><reg8>data><data>... */
	IOREG_MODE_SUBADR_16_POINTED, /*Used in 899*/
	IOREG_MODE_NOSUBADR, /* <addr><data><data>... */
	IOREG_MODE_NO_R_SUBADR
} TUNER_IOREG_Mode_t;

typedef struct
{
	u32 Address;     // Address
	u32 ResetValue;  // Default (reset) value
	u32 Value;       // Current value
} TUNER_IOREG_Register_t;

typedef struct
{
	s32 Reg;  // Register index
	u32 Pos;  // Bit position
	u32 Bits; // Bit width
	u32 Type; // Signed or unsigned
	u32 Mask; // Mask compute with width and position
} TUNER_IOREG_Field_t;

typedef struct
{
	s32 RegExtClk;  /* external clock value */
	u32 Timeout;    /* I/O timeout */
	u32 Registers;  /* number of registers, e.g. 65 for stv0299 */
	u32 Fields;     /* number of register fields, e.g. 113 for stv0299 */
	TUNER_IOREG_Register_t *RegMap;  /* register map list */
	TUNER_IOREG_Field_t *FieldMap;  /* register field list */
	TUNER_IOREG_Flag_t RegTrigger;  /* trigger scope enable */
	TUNER_IOREG_RegisterSize_t RegSize;  /* width of registers in this list */
	TUNER_IOREG_Mode_t Mode;  /* Addressing mode */
	u8 *ByteStream;  /* storage area for 'ContigousRegisters' operations */
	u32 Error;  /* Added for Error handling in I2C */
	u32 *DefVal;
	u32 MasterClock;
	int ChipId;

//#ifdef STB6110 // added for 6110
	/* Parameters needed for non sub address devices */
	u32 WrStart;  // Id of the first writable register
	u32 WrSize;   // Number of writable registers
	u32 RdStart;  // Id of the first readable register
	u32 RdSize;   // Number of readable registers
//#endif

	void *priv;
} TUNER_IOREG_DeviceMap_t;

/* register information */
typedef struct
{
	u16 Addr; /* Address */
	u8 Value; /* Current value */

} STCHIP_Register_t;  /* changed to be in sync with LLA :april 09 */

/* register field type */
typedef enum
{
	CHIP_UNSIGNED = 0,
	CHIP_SIGNED = 1
}
STCHIP_FieldType_t;

/* functions --------------------------------------------------------------- */
u32 TUNER_IOREG_Open(TUNER_IOREG_DeviceMap_t *DeviceMap);
u32 TUNER_IOREG_Close(TUNER_IOREG_DeviceMap_t *DeviceMap);
u32 TUNER_IOREG_AddReg(TUNER_IOREG_DeviceMap_t *DeviceMap, s32 RegIndex, u32 Address, u32 ResetValue);
u32 TUNER_IOREG_AddField(TUNER_IOREG_DeviceMap_t *DeviceMap, s32 RegIndex, s32 FieldIndex, u32 Pos, u32 Bits, u32 Type);
u32 TUNER_IOREG_Reset(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle);
u32 TUNER_IOREG_SetRegister_Sizeu32(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 RegIndex, u32 Value);
u32 TUNER_IOREG_GetRegister_Sizeu32(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 RegIndex);
/* no I/O performed for the following functions */
void TUNER_IOREG_FieldSetVal_Sizeu32_1(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 FieldIndex, int Value, u8 *DataArr);
u32 TUNER_IOREG_FieldGetVal_Sizeu32_1(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 FieldIndex, u8 *DataArr);
u32 TUNER_IOREG_SetContigousRegisters_Sizeu32_1(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FirstRegAddress, u8 *RegsVal, u32 Number);
u32 TUNER_IOREG_GetContigousRegisters(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, s32 FirstRegIndex, s32 Number);
u32 TUNER_IOREG_SetField(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, s32 FieldIndex, u32 Value);
u32 TUNER_IOREG_GetField(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, s32 FieldIndex);
void TUNER_IOREG_FieldSetVal_Sizeu32(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 FieldIndex, s32 Value, u8 *DataArr);
u32 TUNER_IOREG_SetContigousRegisters_Sizeu32(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FirstRegAddress, u32 *RegsVal, int Number);
u32 TUNER_IOREG_GetContigousRegisters_Sizeu32(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FirstRegAddress, int Number, u32 *RegsVal);
u32 TUNER_IOREG_FieldGetVal_Sizeu32(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 FieldIndex, u8 *DataArr);
int IOREG_GetFieldBits_Sizeu32(int mask, int Position);
int IOREG_GetFieldPosition_Sizeu32(u32 Mask);
int IOREG_GetFieldSign_Sizeu32(u32 FieldInfo);
int TUNER_IOREG_GetField_Sizeu32(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FieldMask, u32 FieldInfo);
u32 TUNER_IOREG_SetField_Sizeu32(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FieldMask, u32 FieldInfo, int Value);
u32 TUNER_IOREG_Reset_Sizeu32(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 *DefaultVal, u32 *Addressarray);
u32 TUNER_IOREG_SetRegister(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, s32 RegIndex, u32 Value);

#endif  // __TUNER_IOREG_H
// vim:ts=4
