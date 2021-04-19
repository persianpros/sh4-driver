
/**********************************文件头部注释***********************************/
//
//
// Copyright (C), 2005-2010, AV Frontier Tech. Co., Ltd.
//
//
// 文件名： ioarch.h
//
// 创建者： chm
//
// 创建时间： 12-Oct-2005
//
// 文件描述：
//
// 修改记录： 日 期 作 者 版本 修定
// --------- --------- ----- -----
//
/*********************************************************************************/
#ifndef __TUNER_IOARCH_H
#define __TUNER_IOARCH_H

#define dprintk(level, x...) do \
{ \
	if ((paramDebug) && ((paramDebug >= level) || level == 0)) \
	{ \
		printk(TAGDEBUG x); \
	} \
} while (0)

/* IO operation required */
typedef enum
{
	TUNER_IO_READ,        // Read only
	TUNER_IO_WRITE,       // Write only

	TUNER_IO_SA_READ,     // Read with prefix subaddress (for devices that support it)
	TUNER_IO_SA_WRITE,    // Write with prefix subaddress

	TUNER_IO_SA_READ_NS,  // Read Sub Add with WriteNoStop Cde
	TUNER_IO_SA_WRITE_NS  //
}
TUNER_IOARCH_Operation_t;

/* architecture specifics for I/O mode (common) */
typedef enum TUNER_IORoute_e
{
	TUNER_IO_DIRECT,    // open IO address for this instance of device
	TUNER_IO_REPEATER,  // as above but send IO via ioaccess function (TUNER_IOARCH_RedirFn_t) of other driver (demod)
	TUNER_IO_PASSTHRU   // do not open address just send IO using other drivers open I/O handle (demod)
} TUNER_IORoute_t;

/* select I/O target (common) */
typedef enum TUNER_IODriver_e
{
	TUNER_IODRV_NULL,  // null driver
	TUNER_IODRV_I2C,   // use I2C drivers
	TUNER_IODRV_DEBUG, // screen and keyboard IO
	TUNER_IODRV_MEM    // ST20 memory driver
} TUNER_IODriver_t;

//typedef u32 IOARCH_Handle_t; /* handle returned by IO driver open */
//typedef u32 IOARCH_ExtHandle_t; /* handle to Instance of function being called */

/* IO repeater/passthru function format */
typedef unsigned int(*TUNER_IOARCH_RedirFn_t)(u32 hInstance, u32 IOHandle, TUNER_IOARCH_Operation_t Operation, unsigned short SubAddr, u8 *Data, u32 TransferSize, u32 Timeout);
typedef unsigned int(*TUNER_IOARCH_RepeaterFn_t)(u32 hInstance, BOOL REPEATER_STATUS);

/* hInstance is the Handle to the Instance of the driver being CALLED (not the caller) from a call to its Open() function
 IOHandle is the CALLERS handle to an already open I/O connection */

/* */
typedef struct
{
	u32 PlaceHolder;
} TUNER_IOARCH_InitParams_t;

/* */
typedef struct
{
	u32 PlaceHolder;
} TUNER_IOARCH_TermParams_t;

#if 0
/* */
typedef struct
{
	u8 I2CIndex;
	TUNER_IORoute_t IORoute; /* where IO will go */
	TUNER_IODriver_t IODriver; /* which IO to use */
	TUNER_DeviceName_t DriverName; /* driver name (for Init/Open) */
	u32 Address; /* and final destination */

	u32 *hInstance; /* pointer to instance of driver being called */
	TUNER_IOARCH_RedirFn_t TargetFunction; /* if IODriver is TUNER_IO_REPEATER or TUNER_IO_PASSTHRU
 then specify function to call instead of TUNER_IOARCH_ReadWrite */
} TUNER_IOARCH_OpenParams_t;

/* */
typedef struct
{
	u32 PlaceHolder;
} TUNER_IOARCH_CloseParams_t;
#endif

typedef struct
{
	BOOL IsFree; /* is handle used */
	TUNER_IODriver_t IODriver; /* IO to use */
	TUNER_IORoute_t IORoute; /* where IO will go */
	u32 Address; /* IO address */
	u32 ExtDeviceHandle; /* external device handle to be stored */

	u32 *hInstance; /* if opened as TUNER_IO_REPEATER / TUNER_IO_PASSTHRU */
	TUNER_IOARCH_RedirFn_t TargetFunction;
} IOARCH_HandleData_t;

#define TUNER_IOARCH_MAX_HANDLES 12

extern IOARCH_HandleData_t IOARCH_Handle[TUNER_IOARCH_MAX_HANDLES];

/* functions --------------------------------------------------------------- */
unsigned int TUNER_IOARCH_Init(TUNER_IOARCH_InitParams_t *InitParams);
//unsigned int TUNER_IOARCH_Open(u32 *Handle, TUNER_IOARCH_OpenParams_t *OpenParams);
//unsigned int TUNER_IOARCH_Close(u32 Handle, TUNER_IOARCH_CloseParams_t *CloseParams);
unsigned int TUNER_IOARCH_Term(const TUNER_IOARCH_TermParams_t *TermParams);
unsigned int TUNER_IOARCH_ChangeTarget(u32 Handle, TUNER_IOARCH_RedirFn_t TargetFunction, u32 *hInstance);
unsigned int TUNER_IOARCH_ChangeRoute(u32 Handle, TUNER_IORoute_t IORoute);
unsigned int TUNER_IOARCH_GetAddr(u32 Handle, u32 *Address);
//unsigned int TUNER_IOARCH_ReadWrite(u32 Handle, TUNER_IOARCH_Operation_t Operation, unsigned short SubAddr, u8 *Data, u32 TransferSize, u32 Timeout);
unsigned int TUNER_IOARCH_ReadWriteNoRep(u32 Handle, TUNER_IOARCH_Operation_t Operation, unsigned short SubAddr, u8 *Data, u32 TransferSize, u32 Timeout);
int I2C_ReadWrite(void *I2CHandle, TUNER_IOARCH_Operation_t Operation, unsigned short SubAddr, u8 *Data, u32 TransferSize, u32 Timeout);//lwj add

#endif  // __TUNER_IOARCH_H
// vim:ts=4
