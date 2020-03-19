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
 @File   sttbx.h                                                               
 @brief                                                                         
                                                                               
                                                                               
                                                                               
*/
#ifndef __STTBX_H
#define __STTBX_H

//#include "stddefs.h"
/* {------------------
 * The following section is needed for some typedef definitions (for example
 * STTBX_TraceTxt_functionpointer_t). It should be a simple #include of the
 * sttbx_traces.h file, but this solution is not possible because due to the
 * fact that sttbx.h is symlinked into the /include directory, we cannot use
 * a relative path to sttbx_traces.h.
 */
#define USE_NEW_MACROS 1
#include "traces/traces_global_define.h"
/* }------------------ */


#include "stuart.h"     /* needed for STUART typedefs used in STTBX_TraceTxt_register_stuart */

/* Determine if STTBX_TRACE (data and relay) is enabled 
   by checking if some shared memory is reserved (/sdk_main/stapp/platform/...)
*/
    
#if defined(STTBX_TRACE_SIZE) && (STTBX_TRACE_SIZE != 0)
    #define STTBX_TRACE 1
#endif

/* Get the number of CPU */
#if !defined(DVD_STIPRC) || defined(ST_HOSTONLY)
    #define STTBX_NbCpu 1
#else
#if defined(STTBX_TRACE_SIZE)
    #define STTBX_NbCpu 2
#else
    #define STTBX_NbCpu 1
#endif
#endif

/* Get the running CPU */
#if defined ST_RT
    #define STTBX_thisCPU 1
#else
    #define STTBX_thisCPU 0
#endif


/* ****************************.*/
/*                                                   */
/* The Trace Data management        */
/*  structure                                    */
/*                                                   */
/* **************************** */

typedef struct STTBX_DataDesc_s
{
    U16 cpu;
    U16 unused_align;
    ST_DeviceName_t name;
    ST_DeviceName_t instance;
} STTBX_DataDesc_t;             /* 36 Bytes */

typedef struct STTBX_DataInfo_s
{
    S64 val;
    S64 min;
    S64 max;
    U32 read;
} STTBX_DataInfo_t; /*  28 Bytes */
typedef struct STTBX_Data_s
{
    STTBX_DataDesc_t DataDesc;  /* 36 B */
    STTBX_DataInfo_t DataInfo;  /* 28 B */
} STTBX_Data_t;               /* 64 B per entry in the sttbx_DataCpu array */

typedef struct STTBX_DataRef_s
{
    U16 cpu;
    U16 idx;
    STTBX_DataInfo_t* dataInfo_p;
} STTBX_DataRef_t;

/* C++ support */
#ifdef __cplusplus
extern "C" {
#endif

/* ****************************.*/
/*                                                   */
/* The Trace Data management        */
/*  functions                                    */
/*                                                   */
/* **************************** */
ST_ErrorCode_t STTBX_DataInit(STTBX_Data_t* Getsttbx_DataCpu[]);
ST_ErrorCode_t STTBX_DataTerm(void);
ST_ErrorCode_t STTBX_GetDataRef (STTBX_DataDesc_t dataDesc, STTBX_DataRef_t* dataRef);
ST_ErrorCode_t STTBX_GetDataInfo (STTBX_DataRef_t dataRef, STTBX_DataInfo_t* dataInfo);
ST_ErrorCode_t STTBX_GetNextData (STTBX_Data_t* data, STTBX_DataRef_t* dataRef);
ST_ErrorCode_t STTBX_SetData (STTBX_Data_t data, STTBX_DataRef_t* dataRef);
ST_ErrorCode_t STTBX_SetDataInfo (STTBX_DataRef_t dataRef, STTBX_DataInfo_t dataInfo);

/*  
    Case of validity of data and relay
    depending on settings :
    
    STTBX_TRACE   |            1    1
    STTBX_NbCpu   | 1    2    1    2
    --------------|------------
    STTBX_Data      |            ok  ok
    STTBX_Relay     |                 ok

*/

#ifdef STTBX_TRACE
    /* Macro to add a variable in the STTBX_Data array and get its reference */
    #define STTBX_SETDATA(VAR, NAME, INSTANCE, MIN, MAX, VAL) \
    {                                               \
        STTBX_Data_t data;                          \
        strncpy(data.DataDesc.name, NAME, ST_MAX_DEVICE_NAME);           \
        strncpy(data.DataDesc.instance, INSTANCE, ST_MAX_DEVICE_NAME);   \
        data.DataDesc.cpu = STTBX_thisCPU;          \
        data.DataInfo.min = MIN;                    \
        data.DataInfo.max = MAX;                    \
        data.DataInfo.val = VAL;                    \
        data.DataInfo.read = 0;                     \
        if (STTBX_SetData(data, &VAR)!= ST_NO_ERROR)\
            STTBX_Report((STTBX_REPORT_LEVEL_ERROR, "STTBX_SETDATA %s %s failed", NAME, INSTANCE)); \
    }
    
    /* Macro to modify the value of a referenced variable in the STTBX_Data array */
    #define STTBX_SETDATAVAL(VAR, VAL)      \
    {                                       \
        if (VAR.dataInfo_p)                 \
        {                                   \
            VAR.dataInfo_p->val = VAL;      \
            VAR.dataInfo_p->read = 0;       \
        }                                   \
        else                                \
            STTBX_Report((STTBX_REPORT_LEVEL_ERROR, "STTBX_SETDATAVAL failed: NULL ref")); \
    }

    /* Macro to get the reference of a defined variable in the STTBX_Data array */
    #define STTBX_GETDATA(VAR, NAME, INSTANCE, CPU)         \
    {                                                       \
        STTBX_DataDesc_t dataDesc;                          \
        strncpy(dataDesc.name, NAME, ST_MAX_DEVICE_NAME);                        \
        strncpy(dataDesc.instance, INSTANCE, ST_MAX_DEVICE_NAME);                \
        dataDesc.cpu = CPU;                                 \
        if (STTBX_GetDataRef (dataDesc, &VAR)!= ST_NO_ERROR)\
            STTBX_Report((STTBX_REPORT_LEVEL_ERROR, "STTBX_GETDATA %s %s failed", NAME, INSTANCE)); \
    }
    
    /* Macro to access the value of a referenced variable in the STTBX_Data array */
    #define STTBX_GETDATAVAL(VAR, VAL)      \
    {                                       \
        if (VAR.dataInfo_p)                 \
        {                                   \
            VAL=VAR.dataInfo_p->val;        \
            VAR.dataInfo_p->read++;         \
        }                                   \
        else                                \
            STTBX_Report((STTBX_REPORT_LEVEL_ERROR, "STTBX_GETDATAVAL failed: NULL ref")); \
    }
    
    #if (STTBX_NbCpu > 1) /* Need the sttbx relay functionality */
        #define STTBX_DATA_SIZE     0x4000 /* 16KB / 64B => 256 entries => 128 data per CPU on dual */
        #ifndef STTRACE_TURN_OFF_MECHANISM
            #define STTRACE_TRACE_SIZE  (STTRACE_ARRAY_LAST_INDEX * 4)
        #else
            #define STTRACE_TRACE_SIZE  0
        #endif
        #define STTBX_RELAY_SIZE    ( STTBX_TRACE_SIZE - STTRACE_TRACE_SIZE - STTBX_DATA_SIZE ) /* The rest */
    #else /* Only one CPU, So no relay necessary */
        #define STTBX_RELAY_SIZE    0
        #ifndef STTRACE_TURN_OFF_MECHANISM
            #define STTRACE_TRACE_SIZE  (STTRACE_CID_OFFSET * STTRACE_LAST_CID * 4)
        #else
            #define STTRACE_TRACE_SIZE  0
        #endif
        #define STTBX_DATA_SIZE     (STTBX_TRACE_SIZE-STTRACE_TRACE_SIZE) /* All of it for the data */
    #endif /* STTBX_NbCpu > 1 */

#else /* No STTBX_TRACE */

    #ifndef STTRACE_TURN_OFF_MECHANISM
        #define STTRACE_TRACE_SIZE  (STTRACE_CID_OFFSET * STTRACE_LAST_CID * 4)
    #else
        #define STTRACE_TRACE_SIZE  0
    #endif

    #define STTBX_SETDATA(VAR, NAME, INSTANCE, MIN, MAX, VAL) 
    #define STTBX_SETDATAVAL(VAR, VAL) 
    #define STTBX_GETDATA(VAR, NAME, INSTANCE, CPU)
    #define STTBX_GETDATAVAL(VAR, VAL)
    #define STTBX_DATA_SIZE     0
    #define STTBX_RELAY_SIZE    0

#endif /* STTBX_TRACE */

#ifndef STTBX_MAX_LINE_SIZE
#define STTBX_MAX_LINE_SIZE 512 /* nb of bytes of print line */
#endif /* STTBX_MAX_LINE_SIZE */

#if defined ST_OSLINUX

/* Exported Constants ------------------------------------------------------- */

#define STTBX_DRIVER_ID       28
#define STTBX_DRIVER_BASE     (STTBX_DRIVER_ID << 16)

/* Under Linux, STTBX functions are defined in STOS vob */
typedef struct STTBX_InitParams_s
{
}STTBX_InitParams_t;
typedef struct STTBX_TermParams_s
{
}STTBX_TermParams_t;
#else

/* The STTBX_MODULE_ID should be defined in the makefile */
/* and set to the module ID ; if not done, zero is taken */
/* and I/O will be done using the default devices.       */
/* It is used by the Toolbox to filter the data.         */
#ifndef STTBX_MODULE_ID
    #define STTBX_MODULE_ID 0
#endif


/* Includes ----------------------------------------------------------------- */

#ifndef STTBX_NO_UART
#include "stuart.h"
#endif /* #ifndef STTBX_NO_UART  */

#if defined(STAPIREF_COMPAT)
#else
#if defined(STSDK)
extern ST_ErrorCode_t TRACE_Print (const char *format,...);
extern ST_ErrorCode_t TRACE_Report(U32 ReportLevel,const char *format,...);
#endif
#endif /* STAPIREF_COMPAT */

/* Exported Constants ------------------------------------------------------- */

#define STTBX_DRIVER_ID       28
#define STTBX_DRIVER_BASE     (STTBX_DRIVER_ID << 16)

enum
{              /* max of 32 chars: | */
    STTBX_ERROR_FUNCTION_NOT_IMPLEMENTED = STTBX_DRIVER_BASE
};


/* Max number of modules that the toolbox can recognise. */
#define STTBX_MAX_NB_OF_MODULES 36

/* Defines the supported input/output devices */
typedef enum STTBX_Device_e
{
    /* Value should have a weight (number of bits set) 1 or 0 */
    STTBX_DEVICE_NULL   =  0,   /* No input device */
    STTBX_DEVICE_DCU    =  1,   /* DCU: input from keyboard and output on screen */
    STTBX_DEVICE_UART   =  2,   /* UART: input and output on UART serial port */
    STTBX_DEVICE_UART2  =  4,   /* UART: input and output on UART serial port */
    STTBX_DEVICE_I1284  =  8,   /* I1284: input and output on I1284 parallel port */

    /* Add new devices above here and update value below...*/
    STTBX_NB_OF_DEVICES =  5,   /* Number of STTBX_DEVICE_XXX elements above */
    STTBX_NO_FILTER     =  3    /* Any value with weight (number of bits set) more than 1 */
} STTBX_Device_t;


/* Defines the levels of the report (type of message) */
typedef enum STTBX_ReportLevel_e
{                                   /* Use the given level to indicate: */
    STTBX_REPORT_LEVEL_FATAL = 0,   /* Imminent non recoverable system failure */
    STTBX_REPORT_LEVEL_ERROR,       /* Serious error, though recoverable */
    STTBX_REPORT_LEVEL_WARNING,     /* Warnings of unusual occurences */
    STTBX_REPORT_LEVEL_ENTER_LEAVE_FN, /* Entrance or exit of a function, and parameter display */
    STTBX_REPORT_LEVEL_INFO,        /* Status and other information - normal though */
    STTBX_REPORT_LEVEL_USER1,       /* User specific */
    STTBX_REPORT_LEVEL_USER2,       /* User specific */
    STTBX_REPORT_LEVEL_SET_TRACE,   /* Used to report SET_TRACE traces from RT
                                       to Host on dual CPU platforms */

    /* Keep as last one (Internal use only) */
    STTBX_NB_OF_REPORT_LEVEL        /* Last element has a value corresponding to the number of elements by default ! */
} STTBX_ReportLevel_t;


/* Defines the various types of inputs that can be filtered */
typedef enum STTBX_InputType_e
{
    STTBX_INPUT_ALL             /* All inputs together */

    /* Keep as last one (Internal use only) */
              /* Last element has a value corresponding to the number of elements by default ! */
} STTBX_InputType_t;


/* Defines the various types of outputs that can be filtered */
typedef enum STTBX_OutputType_e
{
    STTBX_OUTPUT_ALL,           /* All outputs together */
    STTBX_OUTPUT_PRINT,         /* Only STTBX_Print */
    STTBX_OUTPUT_REPORT,        /* Only STTBX_Report (all report levels) */
    STTBX_OUTPUT_REPORT_FATAL,  /* Only STTBX_Report with report level fatal */
    STTBX_OUTPUT_REPORT_ERROR,  /* Only STTBX_Report with report level error */
    STTBX_OUTPUT_REPORT_WARNING, /* Only STTBX_Report with report level warning */
    STTBX_OUTPUT_REPORT_ENTER_LEAVE_FN, /* Only STTBX_Report with report level enter/leave function */
    STTBX_OUTPUT_REPORT_INFO,   /* Only STTBX_Report with report level info */
    STTBX_OUTPUT_REPORT_USER1,  /* Only STTBX_Report with report level user1 */
    STTBX_OUTPUT_REPORT_USER2,  /* Only STTBX_Report with report level user2 */

    /* Keep as last one (Internal use only) */
    STTBX_NB_OF_OUTPUTS         /* Last element has a value corresponding to the number of elements by default ! */
} STTBX_OutputType_t;


/* Exported Types ----------------------------------------------------------- */


typedef U32 STTBX_ModuleID_t;

typedef struct STTBX_Capability_s
{
    U32 NotUsed;
} STTBX_Capability_t;

typedef struct STTBX_InitParams_s
{
    ST_Partition_t *CPUPartition_p;     /* partition for internal memory allocation */
#ifdef STTBX_TRACE
    ST_Partition_t *NCachePartition_p;     /* non cached partition for internal memory allocation */
#endif
    STTBX_Device_t  SupportedDevices;    /* OR of the devices the toolbox should support for this init */
    STTBX_Device_t  DefaultOutputDevice; /* Default output device */
    STTBX_Device_t  DefaultInputDevice;  /* Default input device */
    ST_DeviceName_t UartDeviceName;     /* Name of an initialised UART driver, to use as the resp. device if
                                           its support is required. If not, this parameter is disregarded. */
    ST_DeviceName_t Uart2DeviceName;    /* Name of an initialised UART driver, to use as the resp. device if
                                           its support is required. If not, this parameter is disregarded. */
    ST_DeviceName_t I1284DeviceName;    /* Name of an initialised I1284 driver, to use as the resp. device if
                                           its support is required. If not, this parameter is disregarded. */
} STTBX_InitParams_t;

typedef struct STTBX_TermParams_s
{
    U32 NotUsed;
} STTBX_TermParams_t;


typedef struct STTBX_InputFilter_s
{
    STTBX_ModuleID_t    ModuleID;
    STTBX_InputType_t   InputType;
} STTBX_InputFilter_t;

typedef struct STTBX_OutputFilter_s
{
    STTBX_ModuleID_t    ModuleID;
    STTBX_OutputType_t  OutputType;
} STTBX_OutputFilter_t;

typedef struct STTBX_BuffersConfigParams_s
{
    BOOL Enable;
    BOOL KeepOldestWhenFull;
} STTBX_BuffersConfigParams_t;


/* Exported Variables ------------------------------------------------------- */


/* Functions and variables NOT TO BE USED by users of the API --------------- */
#ifdef STTBX_INPUT
    BOOL sttbx_InputPollChar(const STTBX_ModuleID_t ModuleID, char * const Value_p);
    void sttbx_InputChar(const STTBX_ModuleID_t ModuleID, char * const Value_p);
    U16 sttbx_InputStr(const STTBX_ModuleID_t ModuleID, char * const Buffer_p, const U16 BufferSize);
#endif /* STTBX_INPUT */
#ifdef STTBX_REPORT
    extern semaphore_t * sttbx_LockReport_p;
    void sttbx_ReportFileLine(const STTBX_ModuleID_t ModuleID, const char * const FileName_p, const U32 Line);
    void sttbx_ReportMessage(const STTBX_ReportLevel_t ReportLevel, const char *const Format_p, ...);
#endif /* STTBX_REPORT */
#ifdef STTBX_PRINT
    extern semaphore_t * sttbx_GlobalPrintIdMutex_p;
    extern STTBX_ModuleID_t sttbx_GlobalPrintID;
    void sttbx_Print(const char *const Format_p, ...);
    void sttbx_DirectPrint(const char *const Format_p, ...);
#endif /* STTBX_PRINT */
#ifdef STTBX_FILTER
    void sttbx_SetRedirection(const STTBX_Device_t FromDevice, const STTBX_Device_t ToDevice);
    void sttbx_SetInputFilter(const STTBX_InputFilter_t * const Filter_p, const STTBX_Device_t Device);
    void sttbx_SetOutputFilter(const STTBX_OutputFilter_t * const Filter_p, const STTBX_Device_t Device);
#endif /* STTBX_FILTER */
extern void *sttbx_GlobalCPUPartition_p;
extern void *sttbx_NCachePartition_p;


/* Exported Macros ---------------------------------------------------------- */

#ifdef STTBX_INPUT
    #define STTBX_InputPollChar(x) sttbx_InputPollChar(STTBX_MODULE_ID, (x))
    #define STTBX_InputChar(x) sttbx_InputChar(STTBX_MODULE_ID, (x))
    #define STTBX_InputStr(x, y) sttbx_InputStr(STTBX_MODULE_ID, (x), (y))
#else /* not STTBX_INPUT */
    /* STTBX_InputPollChar should return FALSE when undefined but
       returns 0 instead to avoid a warning when not using the value */
    #define STTBX_InputPollChar(x) 0
    #define STTBX_InputChar(x) { }
    #define STTBX_InputStr(x, y) 0
#endif /* not STTBX_INPUT */

#if defined(ST_RT) && (STTBX_RELAY_SIZE > 0)
    /* In case of dual processor, 
    On the RT side, every STTBX_Report need to take the STTBX relay to be sent to the host side */
    #if defined STTBX_REPORT
        extern int STTBX_RelayReport(const int ReportLevel, const char *const Format_p, ...);
        #define STTBX_Report(x) STTBX_RelayReport x
    #else
        #define STTBX_Report(x)
    #endif
    int STTBX_RelayWrite(const int ReportLevel, char *str);
    #if defined STTBX_PRINT
        extern int STTBX_RelayPrint(const char *const Format_p, ...);
        #define STTBX_Print(x) STTBX_RelayPrint x
        #define STTBX_DirectPrint(x) STTBX_RelayPrint x
    #else
        #define STTBX_Print(x)
        #define STTBX_DirectPrint(x)
    #endif
#elif !defined(STAPIREF_COMPAT)&&defined(STSDK)
/* STFAE - START - Reroute traces */
    #if defined STTBX_PRINT
        #define STTBX_Print(x) TRACE_Print x
        #define STTBX_DirectPrint(x) TRACE_Print x
    #else
        #define STTBX_Print(x) { }
        #define STTBX_DirectPrint(x) { }
    #endif
    
    #if defined STTBX_REPORT
        #define STTBX_Report(x) TRACE_Report x
    #else
        #define STTBX_Report(x)
    #endif
/* STFAE - STOP  - Reroute traces */
#else
    #ifdef STTBX_REPORT
    /*  #ifdef ST_OSWINCE 
        ST-CD: added STTBX_Report support 
      #define STTBX_Report(x)			_WCE_Report_Level x 
     #else*/
        #define STTBX_Report(x)                                                 \
                {                                                               \
                  semaphore_wait(sttbx_LockReport_p);                            \
                  /* sttbx_ReportFileLine has to be called here
                    otherwise FILE and LINE would always be the same */         \
                  sttbx_ReportFileLine(STTBX_MODULE_ID, __FUNCTION__, __LINE__);    \
                  sttbx_ReportMessage x;                                        \
                  semaphore_signal(sttbx_LockReport_p);                          \
                }
    #else /* not STTBX_REPORT */
        #define STTBX_Report(x) { }
    #endif /* not STTBX_REPORT */
    
    
    #ifdef STTBX_PRINT
        #define STTBX_Print(x)                                                  \
                {                                                               \
                  semaphore_wait(sttbx_GlobalPrintIdMutex_p);                    \
                  sttbx_GlobalPrintID = STTBX_MODULE_ID;                        \
                  sttbx_Print x;                                                \
                  semaphore_signal(sttbx_GlobalPrintIdMutex_p);                  \
                }
        #define STTBX_DirectPrint(x) sttbx_DirectPrint x
    #else /* not STTBX_PRINT */
        #define STTBX_Print(x) { }
        #define STTBX_DirectPrint(x) { }
    #endif /* not STTBX_PRINT */
#endif


#ifdef STTBX_FILTER
    #define STTBX_SetRedirection(x, y) sttbx_SetRedirection((x), (y))
    /* Set device to STTBX_NO_FILTER disables the filter */
    #define STTBX_SetInputFilter(x, y) sttbx_SetInputFilter((x), (y))
    /* Set device to STTBX_NO_FILTER disables the filter */
    #define STTBX_SetOutputFilter(x, y) sttbx_SetOutputFilter((x), (y))
#else /* not STTBX_FILTER */
    #define STTBX_SetRedirection(x, y) { }
    #define STTBX_SetInputFilter(x, y) { }
    #define STTBX_SetOutputFilter(x, y) { }
#endif /* not STTBX_FILTER */

/* Exported Functions ------------------------------------------------------- */

ST_Revision_t STTBX_GetRevision(void);
ST_ErrorCode_t STTBX_GetCapability(const ST_DeviceName_t DeviceName, STTBX_Capability_t * const Capability_p);

ST_ErrorCode_t STTBX_SetBuffersConfig(const STTBX_BuffersConfigParams_t * const BuffersConfigParam_p);


void STTBX_WaitMicroseconds(U32 Microseconds);


#endif  /* #ifndef ST_OSLINUX */

ST_ErrorCode_t STTBX_Init(const ST_DeviceName_t DeviceName, const STTBX_InitParams_t * InitParams_p);
ST_ErrorCode_t STTBX_Term(const ST_DeviceName_t DeviceName, const STTBX_TermParams_t * TermParams_p);





/* ===================================== */
/* { public symbols defined in tbx_trc.c */
#define STTBX_TRACETXT_MAXSTRINGSIZE 256


/* malloc() macro (defining our own because we need to avoid dependencies) */
#if defined(ST_OSLINUX) && defined(MODULE)
#   define STTBX_MALLOC(size)   vmalloc(size)
#   define STTBX_FREE(ptr)      vfree(ptr)
#else
#   define STTBX_MALLOC(size)   malloc(size)
#   define STTBX_FREE(ptr)      free(ptr)
#endif



#ifndef STTRACE_TURN_OFF_MECHANISM  /* skip implementation if mechanism off */

/*!
 * Initializes the full trace mechanism.
 *
 * Flushes the buffer containing early traces, initializes the full trace
 * mechanism and switches the function pointer to it.
 */
ST_ErrorCode_t STTBX_FullTrace_Init(void);
/*!
 * Shuts down the full trace mechanism.
 */
ST_ErrorCode_t STTBX_FullTrace_Cleanup(void);

ST_ErrorCode_t STTBX_SetTraceFilter(STTRACE_DriverComponentID component_id, STTRACE_Type  type, STTRACE_Level level,U8 Value);

ST_ErrorCode_t STTBX_PrintTraceFilter(STTRACE_DriverComponentID component_id);

/* } public symbols defined in tbx_trc.c */
/* ===================================== */

/* =========================================== */
/* { public symbols defined in tbx_trc_early.c */
/*!
 * Called at startup, initializes the early trace mechanism (buffer, etc.).
 * \note No error handling because an error here would mean board or buffer
 * size configuration problem, and it should be treated a.s.a.p. Graceful 
 * error handling would only complexify the code.
 */
ST_ErrorCode_t STTBX_EarlyTrace_Init(void);
/*!
 * Cleans up the early trace mechanism.
 */
ST_ErrorCode_t STTBX_EarlyTrace_Cleanup(void);

/*!
 * Function pointer that will be called whenever a trace is to be printed.
 *
 * At startup, it points to the EarlyTrace_TraceTxt function which simply
 * buffers traces.
 */
extern STTBX_TraceTxt_functionpointer_t STTBX_TraceTxt_fptr;

/*!
 * Function pointer for the actual output function
 *
 * At startup, it is 0 => all traces printed with printf()/printk(). After
 * startup it is directed to STUART_Write() for UART output.
 * \see STTBX_TraceTxt_register_stuart()
 */
extern STUART_Write_functionpointer_t STTBX_TraceTxt_stuart_fptr;
/*! handle of the STUART instance used for output */
extern STUART_Handle_t STTBX_TraceTxt_stuart_handle;

/*!
 * Used by STAPP to tell STTBX the coordinates of the STUART instance to use.
 */
void STTBX_TraceTxt_register_stuart(STUART_Write_functionpointer_t fptr, STUART_Handle_t hnd);

/*! size of a buffer entry in bytes */
extern const U32 STTBX_EarlyTrace_buffer_entry_size;
/*! number of buffer entries */
extern volatile U32 STTBX_EarlyTrace_buffer_entries_count;
/*! pointer to the buffer storing traces early on in the system init process */
extern U8 * STTBX_EarlyTrace_buffer;
/*! "semaphore" to tell whether any early traces are in progress */
extern volatile U32 STTBX_EarlyTrace_inprogress;

/* } public symbols defined in tbx_trc_early.c */
/* =========================================== */
#endif /* #ifndef STTRACE_TURN_OFF_MECHANISM */


/* TBXFS part */
typedef struct STTBX_ODD_Context_s STTBX_ODD_Context_t ;

typedef ST_ErrorCode_t (*STTBX_ODD_GenCallbackFn_t)( STTBX_ODD_Context_t *ODDContext, void *LocalData, BOOL *EOF_Indicator );
#define STTBX_ODD_CALLBACK_DECLARE(Fnt) ST_ErrorCode_t Fnt(STTBX_ODD_Context_t *ODDContext, void *LocalData, BOOL *EOF_Indicator )

ST_ErrorCode_t STTBX_ODD_Register( char *Path, U32 MaxDataSize, STTBX_ODD_GenCallbackFn_t GenFn, void *LocalData);
ST_ErrorCode_t STTBX_ODD_Unregister(char *Path);
ST_ErrorCode_t STTBX_ODD_Printf(STTBX_ODD_Context_t *ODDContext, const char *format, ...) __attribute__ ((format (printf, 2, 3)));

#ifndef ST_OSLINUX
void * STTBX_FindTbxContext(char* path);
int STTBX_GetOddData(char *buffer, int size, int *Eof, void *Data);
void * STTBX_FindTbxEntry(char* path);
int STTBX_GetOddPath(char *buffer, int size, void *Data);
#endif

/* C++ support */
#ifdef __cplusplus
}
#endif

#endif /* #ifndef __STTBX_H */

/* End of sttbx.h */

