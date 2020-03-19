/*****************************************************************************/
/* COPYRIGHT (C) 2009 STMicroelectronics - All Rights Reserved               */
/* ST makes no warranty express or implied including but not limited to,     */
/* any warranty of                                                           */
/*                                                                           */
/*   (i)  merchantability or fitness for a particular purpose and/or         */
/*   (ii) requirements, for a particular purpose in relation to the LICENSED */
/*        MATERIALS, which is provided 'AS IS', WITH ALL FAULTS. ST does not */
/*        represent or warrant that the LICENSED MATERIALS provided here     */
/*        under is free of infringement of any third party patents,          */
/*        copyrights,trade secrets or other intellectual property rights.    */
/*        ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE       */
/*        EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                    */
/*                                                                           */
/*****************************************************************************/
/**
 @File   chip.h
 @brief



*/
#ifndef H_CHIP
#define H_CHIP


/* includes ---------------------------------------------------------------- */

/* STAPI (ST20) requirements */

// #include "stddefs.h"    /* Standard definitions */
 #include "sti2c.h"
 #include "stfrontend.h"
 
/* STV0368 specific */
#define STV0368_PAGESHIFT 				0x004C
#define STV0368_HIGHEST_PROGRAM_ADDR 	0x40FFFF	/* cut 2 */
#define STV0368_MAX_MAILBOX_PAGE_SIZE 	12
#define STV0368_MAX_WAIT_FW_READY_LOOP	1000		/* timeout after issuing a command */
#define MAX_DATA_PACKETS	514 // 258	// max number of r/w data per USB frame
#define FILLSIZE			(MAX_DATA_PACKETS-2)


#ifdef __cplusplus
 extern "C"
 {
#endif                  /* __cplusplus */


#define REPEATER_ON TRUE
#define REPEATER_OFF FALSE

/* maximum number of chips that can be opened */
#define MAXNBCHIP 10

/* maximum number of chars per chip/tuner name */
#define MAXNAMESIZE 30

#define I2cReadWrite(mode, addr, data, nbdata) Chip_I2cReadWrite(hChip, mode, addr, data, nbdata)
typedef enum
{
	I2C_READ = READING,		/* Read from slave */
	I2C_WRITE = WRITING,		/* Write to slave */
	I2C_WRITENOSTOP /* Write to slave without stop */	
} I2C_MODE;

typedef enum
{
	I2C_OK = 0,		/* I2C transmission OK */
	I2C_NOSDA,		/* SDA line is stucked */
	I2C_NOSCL,		/* SCL line is stucked */
	I2C_BUSERROR,	/* SDA and SCL are stucked */
	I2C_NOACK		/* SDA and SCL lines are OK but the device does not acknowledge */
	
} I2C_RESULT;



typedef enum
{
    STCHIP_ACCESS_WR,  /* can be read and written */
    STCHIP_ACCESS_R,   /* only be read from */
    STCHIP_ACCESS_W,   /* only be written to */
    STCHIP_ACCESS_NON  /* cannot be read or written (guarded register, e.g. register skipped by ChipApplyDefaultValues() etc.) */
}
STCHIP_Access_t;


/* register field type */
typedef enum
{
    CHIP_UNSIGNED = 0,
    CHIP_SIGNED = 1
}
STCHIP_FieldType_t;


/* error codes */
typedef enum
{
    CHIPERR_NO_ERROR = 0,       /* No error encountered */
    CHIPERR_INVALID_HANDLE,     /* Using of an invalid chip handle */
    CHIPERR_INVALID_REG_ID,     /* Using of an invalid register */
    CHIPERR_INVALID_FIELD_ID,   /* Using of an Invalid field */
    CHIPERR_INVALID_FIELD_SIZE, /* Using of a field with an invalid size */
    CHIPERR_I2C_NO_ACK,         /* No acknowledge from the chip */
    CHIPERR_I2C_BURST,           /* Two many registers accessed in burst mode */
    CHIPERR_I2C_OTHER_ERROR    /*Other Errors reported  from I2C driver, added in stfrontend driver*/
}
STCHIP_Error_t;


/* how to access I2C bus */
typedef enum
{
    STCHIP_MODE_SUBADR_8,       /* <addr><reg8><data><data>        (e.g. demod chip) */
    STCHIP_MODE_SUBADR_16,      /* <addr><reg8><data><data>        (e.g. demod chip) */
    STCHIP_MODE_NOSUBADR,        /* <addr><data>|<data><data><data> (e.g. tuner chip) */
    STCHIP_MODE_NOSUBADR_RD,
    STCHIP_MODE_SUBADR_8_NS_MICROTUNE,
    STCHIP_MODE_SUBADR_8_NS,
    STCHIP_MODE_MXL,
    STCHIP_MODE_NOSUBADR_SR,    /*start repeat*/
    STCHIP_MODE_SUBADR_8_SR,
    STCHIP_MODE_STV0368		/* 0368: specific use of PAGESHIFT register */
}
STCHIP_Mode_t;

/* structures -------------------------------------------------------------- */


/* register information */
typedef struct
{
    U16    Addr;     /* Address */
    U8    Value;    /* Current value */
}
STCHIP_Register_t; /*changed to be in sync with LLA :april 09*/


/* register field information */
typedef struct
{
    U16                Reg;      /* Register index */
    unsigned char      Pos;      /* Bit position */
    unsigned char      Bits;     /* Bit width */
    unsigned char      Mask;     /* Mask compute with width and position */
    STCHIP_FieldType_t Type;     /* Signed or unsigned */
    char               Name[30]; /* Name */
}
STCHIP_Field_t;


typedef struct stchip_Info_t
{
 ST_DeviceName_t DriverName;
 STFRONTEND_IODriver_t  IODriver;
 STFRONTEND_IORoute_t   IORoute;

 U32                 I2C_BaudRate;
 U32                 I2C_TimeOut;
 STI2C_Handle_t  I2C_Handle;
 semaphore_t *sem_deviceAccess;/*Semaphore to protect against simultaneous access
                                                          to same tuner/demod from two or more threads*/
 S32                  taskID;/*contains taskID of the task in which tuner acquires sempahore*/

/*following part copied directly from LLA*/
 U32    I2cAddr;          /* Chip I2C address */
 char                Name[MAXNAMESIZE];         /* Name of the chip */
 U32                 NbRegs;           /* Number of registers in the chip */
 U32                NbFields;         /* Number of fields in the chip */
 STCHIP_Register_t  *pRegMapImage;     /* Pointer to register map */
 STCHIP_Error_t      Error;            /* Error state */
 STCHIP_Mode_t       ChipMode;         /* Access bus in demod (SubAdr) or tuner (NoSubAdr) mode */
 U8     ChipID;           /* Chip cut ID */

 BOOL                Repeater;         /* Is repeater enabled or not ? */
 struct stchip_Info_t *RepeaterHost;   /* Owner of the repeater */

#ifdef CHIP_STAPI
    STCHIP_Error_t   (*RepeaterFn)(struct stchip_Info_t *hChip,BOOL State, U8 *Buffer);    /* Pointer to repeater routine */
#else
    STCHIP_Error_t   (*RepeaterFn)(struct stchip_Info_t *hChip,BOOL State);    /* Pointer to repeater routine */
#endif
  BOOL IsAutoRepeaterOffEnable;
U32					XtalFreq;         /* added for dll purpose*/
/* Parameters needed for non sub address devices */
 U32     WrStart;    /* Id of the first writable register */
 U32     WrSize;           /* Number of writable registers */
 U32     RdStart;    /* Id of the first readable register */
 U32     RdSize;     /* Number of readable registers */
 BOOL   Abort;			   /* Abort flag when set to on no register access and no wait  are done  */
BOOL				IFmode;			  /* IF mode 1; IQ mode 0   */
 void    *pData;     /* pointer to chip data */

}
STCHIP_Info_t;

/*Used in LLA chip, placed here as LLA includes chip.h */
typedef struct
{
   STCHIP_Info_t *Chip;      /* pointer to parameters to pass to the CHIP API */
   U32            NbDefVal;  /* number of default values (must match number of STC registers) */
}
Demod_InitParams_t;

typedef STCHIP_Info_t *STCHIP_Handle_t;
typedef STCHIP_Info_t *TUNER_Handle_t;

#ifdef CHIP_STAPI
typedef STCHIP_Error_t (*STCHIP_RepeaterFn_t)(STCHIP_Handle_t hChip,BOOL State,U8 *Buffer);    /* Pointer to repeater routine */
#else
typedef STCHIP_Error_t (*STCHIP_RepeaterFn_t)(STCHIP_Handle_t hChip,BOOL State);    /* Pointer to repeater routine */
#endif

/* Access routines */
ST_ErrorCode_t ChipInit(STCHIP_Handle_t *hChip, ST_Partition_t *MemoryPartition);
ST_ErrorCode_t ChipOpen(STCHIP_Info_t *hChip);
ST_ErrorCode_t ChipClose(STCHIP_Handle_t hChip);
ST_ErrorCode_t ChipTerm(STCHIP_Handle_t hChip, ST_Partition_t *MemoryPartition);


ST_ErrorCode_t ChipSetOneRegister(STCHIP_Handle_t hChip,U16 RegAddr, U8 Value);
U8 ChipGetOneRegister(STCHIP_Handle_t hChip, U16 RegAddress);

ST_ErrorCode_t ChipSetRegisters(STCHIP_Handle_t hChip,U32 FirstRegAddress,int Number);
ST_ErrorCode_t ChipGetRegisters(STCHIP_Handle_t hChip,U32 FirstRegAddress,int Number);

ST_ErrorCode_t ChipSetRegistersI2C(STCHIP_Handle_t hChip, U32 FirstReg, int NbRegs);
ST_ErrorCode_t ChipGetRegistersI2C(STCHIP_Handle_t hChip, U32 FirstRegAddress, int NbRegs);


ST_ErrorCode_t ChipSetRegistersIOMem(STCHIP_Handle_t hChip, U32 FirstReg, int NbRegs);
ST_ErrorCode_t ChipGetRegistersIOMem(STCHIP_Handle_t hChip, U32 FirstRegAddress, int NbRegs);

ST_ErrorCode_t ChipSetField(STCHIP_Handle_t hChip, U32 FieldIndex, int Value);
U8             ChipGetField(STCHIP_Handle_t hChip, U32 FieldIndex);

STCHIP_Error_t ChipSetFieldImage(STCHIP_Handle_t hChip,U32 FieldId, S32 Value);
S32 ChipGetFieldImage(STCHIP_Handle_t hChip,U32 FieldId);

void ChipWaitOrAbort(STCHIP_Handle_t hChip,U32 delay_ms);
void ChipAbort(STCHIP_Handle_t hChip,BOOL Abort);

U32  ChipFieldExtractVal(U8 RegisterVal, int FieldIndex);
U16 ChipGetRegAddress(U32 FieldId);
int ChipGetFieldMask(U32 FieldId);
int ChipGetFieldSign(U32 FieldId);
int ChipGetFieldPosition(U8 Mask);
int ChipGetFieldBits(int mask, int Position);
S32 ChipGetRegisterIndex(STCHIP_Handle_t hChip, U16 RegId);
STCHIP_Error_t  ChipUpdateDefaultValues(STCHIP_Handle_t hChip,STCHIP_Register_t  *pRegMap);
ST_ErrorCode_t ChipFillRepeaterMessage(STCHIP_Handle_t hChip, U32 FieldId, int Value, U8 *Buffer);
U32 ChipGetRepeaterMessageLength(STCHIP_Handle_t hChip);
ST_ErrorCode_t PrepareSubAddressHeader(STCHIP_Handle_t hChip,STCHIP_Mode_t Mode, STI2C_Context_t Context, U16 SubAddress, U8 * Buffer,
                                              U32 * SubAddressByteCount, BOOL *UseHostHandle,U32 firstRegIndex);
__inline void FillMsg(STI2C_Message_t * Msg, U32 * MsgIndex, STI2C_Handle_t Handle, U32 BufferLen, U8 * Buffer_p,
                             STI2C_Context_t Context, BOOL UseStop);


ST_ErrorCode_t Chip_I2cReadWrite(STCHIP_Handle_t hChip, I2C_MODE mode, U8 ChipAddress, unsigned char *Data,int NbData);

STCHIP_Error_t Chip_368dvbt2_WaitMailboxComplete(STCHIP_Handle_t hChip);

#ifdef STFRONTEND_DEBUG_THROUGH_IODUMP
enum
{
READ,
WRITE,
WRITE_SA,
WRITE_NS
};
FILE *stfrontend_iodump_fp;

void ChipIODump(IOARCH_Handle_t hChip,unsigned char *Buffer,U32 Size,int OPERATION);
void ChipIODumpMessage(char *buffer);
void ChipIODumpRegMap(IOARCH_Handle_t hChip,U32 Max_Address,char *buffer);
#endif


#ifdef __cplusplus
 }
#endif                  /* __cplusplus */

#endif          /* H_CHIP */

/* End of chip.h */

