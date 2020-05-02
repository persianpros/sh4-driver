/*****************************************************************************/
/* COPYRIGHT (C) 2009 STMicroelectronics - All Rights Reserved               */
/* ST makes no warranty express or implied including but not limited to,     */
/* any warranty of                                                           */
/*                                                                           */
/*   (i)  merchantability or fitness for a particular purpose and/or         */
/*   (ii) requirements, for a particular purpose in relation to the LICENSED */
/*        MATERIALS, which is provided .AS IS., WITH ALL FAULTS. ST does not */
/*        represent or warrant that the LICENSED MATERIALS provided here     */
/*        under is free of infringement of any third party patents,          */
/*        copyrights,trade secrets or other intellectual property rights.    */
/*        ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE       */
/*        EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                    */
/*                                                                           */
/*****************************************************************************/
/**
 @File   lxload.h
 @brief



*/
#if !defined __LXLOAD_H
#define __LXLOAD_H

#define LXLOAD_IOCTL_MAJOR         0

/* STFAE - START - Some drivers or applications need to access registers from userspace */
#define STOS_MapRegisters(pAddr, Length, Name)      (void *)STSYS_MapRegisters((unsigned int)pAddr,Length,Name)
#define STOS_UnmapRegisters(vAddr, Length)          STSYS_UnmapRegisters((unsigned int)vAddr,Length)

/* SBC System Configurations Regs (0-99) */
#define SYSCFG_SAPPHIRE_BASE_ADDR  0xfe600000
/* For sys config 100-199 */
#define SYSCFG_CORAL_BASE_ADDR     0xfda50000
/* For sys config 200-399 */
#define SYSCFG_PEARL_BASE_ADDR     0xfd500000
/* For sys config 400-599 */
#define SYSCFG_OPAL_BASE_ADDR      0xfd541000

#define CFG_0_BASE_ADDRESS         SYSCFG_SAPPHIRE_BASE_ADDR /* 0-99 syscfg register    */
#define CFG_1_BASE_ADDRESS         SYSCFG_CORAL_BASE_ADDR    /* 100-199 syscfg register */
#define CFG_2_BASE_ADDRESS         SYSCFG_PEARL_BASE_ADDR    /* 200-399 syscfg register */
#define CFG_3_BASE_ADDRESS         SYSCFG_OPAL_BASE_ADDR     /* 400-599 syscfg register */

/* BOOL type constant values */
#ifndef TRUE
	#define TRUE (1 == 1)
#endif
#ifndef FALSE
	#define FALSE (!TRUE)
#endif

/* Function return error code */
typedef unsigned int ST_ErrorCode_t;

/* Common driver error constants */
#define ST_DRIVER_ID   0
#define ST_DRIVER_BASE (ST_DRIVER_ID << 16)
enum
{
	ST_NO_ERROR = ST_DRIVER_BASE,       /*  0                             */
	ST_ERROR_BAD_PARAMETER,             /*  1 Bad parameter passed        */
	ST_ERROR_NO_MEMORY,                 /*  2 Memory allocation failed    */
	ST_ERROR_UNKNOWN_DEVICE,            /*  3 Unknown device name         */
	ST_ERROR_ALREADY_INITIALIZED,       /*  4 Device already initialized  */
	ST_ERROR_NO_FREE_HANDLES,           /*  5 Cannot open device again    */
	ST_ERROR_OPEN_HANDLE,               /*  6 At least one open handle    */
	ST_ERROR_INVALID_HANDLE,            /*  7 Handle is not valid         */
	ST_ERROR_FEATURE_NOT_SUPPORTED,     /*  8 Feature unavailable         */
	ST_ERROR_INTERRUPT_INSTALL,         /*  9 Interrupt install failed    */
	ST_ERROR_INTERRUPT_UNINSTALL,       /* 10 Interrupt uninstall failed  */
	ST_ERROR_TIMEOUT,                   /* 11 Timeout occured             */
	ST_ERROR_DEVICE_BUSY,               /* 12 Device is currently busy    */
	ST_ERROR_SUSPENDED                  /* 13 Device is in D1 or D2 state */
};



#endif  // __LXLOAD_H
// vim:ts=4

