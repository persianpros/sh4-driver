#ifndef _PTI_H_
#define _PTI_H_

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/errno.h>

#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#include <linux/platform_device.h>
#include <linux/mutex.h>

#include <asm/io.h>

#if defined (CONFIG_KERNELVERSION) || LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#include <linux/bpa2.h>
#else
#include <linux/bigphysarea.h>
#endif
/* doesn't run very stable on ufs910,
 * all other should try this
 */
#undef use_irq

#define USE_ONE_DMA

#define A18 /* stapi a18 */

/* ufs912
 * ! all 7109er architectures should use secure lite 1 but
 * this is not implemented currently.
 */
#if defined(UFS912) || defined(UFS913) || defined(SPARK) || defined(SPARK7162) || defined(ATEVIO7500) || defined(HS7810A) || defined(HS7110)
#define SECURE_LITE2
#endif

/* external declaration */
struct tBuffer;

#define TC_DMA_THRESHOLD_LOW 1
#define TC_DMA_THRESHOLD_HIGH 128

#if defined(UFS912) || defined(UFS913) || defined(SPARK) || defined(SPARK7162) || defined(ATEVIO7500) || defined(HS7810A) || defined(HS7110) || defined(HS7110) || defined(HS7810A) || defined(HS7420) || defined(HS7429) || defined(HS7119) || defined(HS7819) || defined(ATEMIO520) || defined(ATEMIO530) || defined(VITAMIN_HD5000) || defined(SAGEMCOM88)
#define PTI_BASE_ADDRESS 0xfe230000
#elif defined(CONFIG_CPU_SUBTYPE_STB5301)
#define PTI_BASE_ADDRESS 0x20E00000
#else
#define PTI_BASE_ADDRESS 0x19230000
#endif

#define PTI_BASE_SIZE    0x10000
#define PTI_DMAEMPTY_EN  (0x0038)
#define STPTI_MAXCHANNEL 32
#define STPTI_MAXADAPTER 3

#if !defined(SECURE_LITE2)
#define TC_DATA_RAM_SIZE           6656         /* (6.5 * 1024) */
#define TC_CODE_RAM_SIZE           7680         /* (7.5 * 1024) */
#else
#define TC_DATA_RAM_SIZE          13824         /* (13.5 * 1024) */
#define TC_CODE_RAM_SIZE          16384         /* (16 * 1024)   */
#endif

#define TC_NUMBER_OF_HARDWARE_SECTION_FILTERS 32
#define TC_NUMBER_OF_HARDWARE_NOT_FILTERS 32

#define SF_BYTES_PER_CAM       768  /* Fixed value from hardware specs. */
#define SF_FILTER_LENGTH         8  /* Minimum CAM filter length, must be multiple of 8 (8=short section filter mode) */
#define SF_CAMLINE_WIDTH         4  /* Fixed value from hardware specs. */
#define SF_BYTES_PER_MIN_ALLOC ( SF_CAMLINE_WIDTH * (SF_FILTER_LENGTH * 2)) /* N filters of M bytes & masks */

#define SF_NUM_BLOCKS_PER_CAM  ( SF_BYTES_PER_CAM / SF_BYTES_PER_MIN_ALLOC )
#define SF_NUM_FILTERS_PER_BLOCK ( SF_CAMLINE_WIDTH * 2 )       /* block spans the 2 CAMs each holding SF_CAMLINE_WIDTH filters */

/* MAGIC numbers for the CAM_CFG register */
#define CAM_CFG_8_FILTERS       0x0000  /* bits 5:3 (number of filters) */
#define CAM_CFG_16_FILTERS      0x0008
#define CAM_CFG_32_FILTERS      0x0010
#define CAM_CFG_64_FILTERS      0x0018
#define CAM_CFG_96_FILTERS      0x0020

#define INVALID_FILTER_INDEX ((u32)0xffffffff)

#define TC_INVALID_PID             ((unsigned int)0xe000)
#define TC_INVALID_LINK            ((unsigned int)0xf000)
#define SESSION_NOT_ALLOCATED      0xDEAD

#define STATUS_FLAGS_TRANSPORT_ERROR              0x00000001
#define STATUS_FLAGS_SECTION_CRC_ERROR            0x00000002
#define STATUS_FLAGS_INVALID_DESCRAMBLE_KEY       0x00000004
#define STATUS_FLAGS_INVALID_PARAMETER            0x00000008
#define STATUS_FLAGS_INVALID_CC                   0x00000010
#define STATUS_FLAGS_TC_CODE_FAULT                0x00000020
#define STATUS_FLAGS_PACKET_SIGNAL                0x00000040
#define STATUS_FLAGS_PES_ERROR                    0x00000080

#define STATUS_FLAGS_DMA_COMPLETE                 0x00000100
#define STATUS_FLAGS_SUBSTITUTE_COMPLETE          0x00000400
#define STATUS_FLAGS_RECORD_BUFFER_OVERFLOW       0x00000800
#define STATUS_FLAGS_CWP_FLAG                     0x00002000    /* (DSS only) */
#define STATUS_FLAGS_INVALID_LINK                 0x00004000
#define STATUS_FLAGS_BUFFER_OVERFLOW              0x00008000

#define STATUS_FLAGS_ADAPTATION_EXTENSION_FLAG    0x00010000    /* (DVB only) */
#define STATUS_FLAGS_PRIVATE_DATA_FLAG            0x00020000    /* (DVB only) */
#define STATUS_FLAGS_SPLICING_POINT_FLAG          0x00040000    /* (DVB only) */
#define STATUS_FLAGS_OPCR_FLAG                    0x00080000    /* (DVB only) */
#define STATUS_FLAGS_PCR_FLAG                     0x00100000
#define STATUS_FLAGS_PRIORITY_INDICATOR           0x00200000    /* (DVB only) */
#define STATUS_FLAGS_RANDOM_ACCESS_INDICATOR      0x00400000    /* (DVB only) */
#define STATUS_FLAGS_CURRENT_FIELD_FLAG           0x00400000    /* (DSS only) */
#define STATUS_FLAGS_DISCONTINUITY_INDICATOR      0x00800000    /* (DVB only) */
#define STATUS_FLAGS_MODIFIABLE_FLAG              0x00800000    /* (DSS only) */

#define STATUS_FLAGS_START_CODE_FLAG              0x01000000    /* (DVB only) */
#define STATUS_FLAGS_TIME_CODE                    0x01000000    /* (DSS only) */
#define STATUS_FLAGS_PUSI_FLAG                    0x02000000    /* (DVB only) */
#define STATUS_FLAGS_SCRAMBLE_CHANGE              0x04000000
#define STATUS_FLAGS_FIRST_RECORDED_PACKET        0x08000000
#define STATUS_FLAGS_BUNDLE_BOUNDARY              0x10000000    /* (DSS only) */
#define STATUS_FLAGS_AUXILIARY_PACKET             0x20000000    /* (DSS only) */
#define STATUS_FLAGS_SESSION_NUMBER_MASK          0x30000000    /* (DVB only) */
#define STATUS_FLAGS_PACKET_SIGNAL_RECORD_BUFFER  0x40000000
#define STATUS_FLAGS_INDX_TIMEOUT_TICK            0x80000000    /* Only used for STPTI_TIMER_TICK feature */

#define TC_GLOBAL_DATA_DISCARD_SYNC_BYTE_SET      0x0001

/* SessionSectionParams: */

#define TC_SESSION_INFO_FILTER_TYPE_FIELD       0xF000

#define TC_SESSION_INFO_FILTER_TYPE_IREDETO_ECM 0x2000
#define TC_SESSION_INFO_FILTER_TYPE_IREDETO_EMM 0x3000
#define TC_SESSION_INFO_FILTER_TYPE_SHORT       0x4000
#define TC_SESSION_INFO_FILTER_TYPE_LONG        0x5000
#define TC_SESSION_INFO_FILTER_TYPE_MAC         0x6000
#define TC_SESSION_INFO_FILTER_TYPE_NEG_MATCH   0x7000

#define TC_SESSION_INFO_FORCECRCSTATE           0x0001
#define TC_SESSION_INFO_DISCARDONCRCERROR       0x0002

#define TC_SESSION_DVB_PACKET_FORMAT            0x0010

#define TC_DMA_CONFIG_SIGNAL_MODE_FLAGS_SIGNAL_DISABLE    0x1
#define TC_DMA_CONFIG_SIGNAL_MODE_TYPE_MASK               0xe
#define TC_DMA_CONFIG_SIGNAL_MODE_TYPE_NO_SIGNAL          0x0
#define TC_DMA_CONFIG_SIGNAL_MODE_TYPE_QUANTISATION       0x2
#define TC_DMA_CONFIG_SIGNAL_MODE_TYPE_EVERY_TS           0x4
#define TC_DMA_CONFIG_SIGNAL_MODE_SWCDFIFO                0x8
#define TC_DMA_CONFIG_OUTPUT_WITHOUT_META_DATA            0x10
#define TC_DMA_CONFIG_WINDBACK_ON_ERROR                   0x20

#define SESSION_USE_MERGER_FOR_STC              0x8000
#define SESSION_MASK_STC_SOURCE                 0x7FFF

#define SLOT_STATE_INITIAL_SCRAMBLE_STATE       0x4000

#define STPTI_BUFFER_ALIGN_MULTIPLE 0x20
#define STPTI_BUFFER_SIZE_MULTIPLE 0x20

#define DMAScratchAreaSize       (STPTI_BUFFER_SIZE_MULTIPLE+STPTI_BUFFER_ALIGN_MULTIPLE)
#define NO_OF_DVB_STATUS_BLOCKS  2048

#define NO_OF_STATUS_BLOCKS NO_OF_DVB_STATUS_BLOCKS

#define TC_DSRAM_BASE 0x8000

#define TC_SLOT_TYPE_NULL                       0x0000
#define TC_SLOT_TYPE_SECTION                    0x0001
#define TC_SLOT_TYPE_PES                        0x0002
#define TC_SLOT_TYPE_RAW                        0x0003
#define TC_SLOT_TYPE_EMM                        0x0005
#define TC_SLOT_TYPE_ECM                        0x0006

#define TC_MAIN_INFO_PES_STREAM_ID_FILTER_ENABLED            0x0100

#define TC_MAIN_INFO_SLOT_STATE_ODD_SCRAMBLED                 0x0001
#define TC_MAIN_INFO_SLOT_STATE_TRANSPORT_SCRAMBLED           0x0002
#define TC_MAIN_INFO_SLOT_STATE_SCRAMBLED                     0x0004
#define TC_MAIN_INFO_SLOT_STATE_SCRAMBLE_STATE_FIELD          (TC_MAIN_INFO_SLOT_STATE_SCRAMBLED | \
							       TC_MAIN_INFO_SLOT_STATE_TRANSPORT_SCRAMBLED| \
							       TC_MAIN_INFO_SLOT_STATE_ODD_SCRAMBLED)

#define TC_MAIN_INFO_SLOT_STATE_DMA_IN_PROGRESS               0x1000
#define TC_MAIN_INFO_SLOT_STATE_SEEN_PACKET                   0x2000
#define TC_MAIN_INFO_SLOT_STATE_SEEN_TS_SCRAMBLED             0x4000
#define TC_MAIN_INFO_SLOT_STATE_SEEN_PES_SCRAMBLED            0x8000
#define TC_MAIN_INFO_SLOT_STATE_SEEN_FIELD                    (TC_MAIN_INFO_SLOT_STATE_SEEN_PACKET | \
							       TC_MAIN_INFO_SLOT_STATE_SEEN_TS_SCRAMBLED | \
							       TC_MAIN_INFO_SLOT_STATE_SEEN_PES_SCRAMBLED)

#define TC_MAIN_INFO_SLOT_MODE_ALTERNATE_OUTPUT_CLEAR          0x0010
#define TC_MAIN_INFO_SLOT_MODE_ALTERNATE_OUTPUT_DESCRAMBLED    0x0020
#define TC_MAIN_INFO_SLOT_MODE_ALTERNATE_OUTPUT_FIELD          (TC_MAIN_INFO_SLOT_MODE_ALTERNATE_OUTPUT_CLEAR | \
								TC_MAIN_INFO_SLOT_MODE_ALTERNATE_OUTPUT_DESCRAMBLED)

#define TC_MAIN_INFO_SLOT_MODE_IGNORE_SCRAMBLING               0x0040
#define TC_MAIN_INFO_SLOT_MODE_INJECT_SEQ_ERROR_MODE           0x0080
#define TC_MAIN_INFO_SLOT_MODE_SUBSTITUTE_STREAM               0x0100

#define TC_MAIN_INFO_SLOT_MODE_DMA_1                           0x0200
#define TC_MAIN_INFO_SLOT_MODE_DMA_2                           0x0400
#define TC_MAIN_INFO_SLOT_MODE_DMA_3                           0x0600
#define TC_MAIN_INFO_SLOT_MODE_DMA_FIELD                       0x0600

#define TC_MAIN_INFO_SLOT_MODE_DISABLE_CC_CHECK                0x0800

/* For Watch & record reuses Startcode detection word*/
#define TC_MAIN_INFO_REC_BUFFER_MODE_ENABLE                    0x0100
#define TC_MAIN_INFO_REC_BUFFER_MODE_DESCRAMBLE                0x0200

#define TC_MAIN_INFO_STARTCODE_DETECTION_OFFSET_MASK           0x00FF

#define IIF_SYNC_CONFIG_USE_SOP             0x01

typedef enum STPTI_StreamID_s
{
	STPTI_STREAM_ID_TSIN0 = 0x20,
	STPTI_STREAM_ID_TSIN1,
	STPTI_STREAM_ID_TSIN2,
#if defined(UFS912) || defined(ATEVIO7500) /* 7111/7105 arch */
	STPTI_STREAM_ID_TSIN3,
#endif
	STPTI_STREAM_ID_SWTS0,
#ifdef UFS922 /* 7109 arch */
	STPTI_STREAM_ID_SWTS1,
	STPTI_STREAM_ID_SWTS2,
#endif
	STPTI_STREAM_ID_ALTOUT,
	STPTI_STREAM_ID_NOTAGS = 0x80,  /* if tsmerger not configured to emit tag bytes */
	STPTI_STREAM_ID_NONE  /*Use to deactivate a virtual PTI*/

} STPTI_StreamID_t ;

/* BOOL type constant values */
#ifndef TRUE
#define TRUE (1 == 1)
#endif
#ifndef FALSE
#define FALSE (!TRUE)
#endif

typedef volatile u32 *STPTI_DevicePtr_t;

#ifdef A18

typedef volatile struct TCSessionInfo_s
{
	u16 SessionInputPacketCount;
	u16 SessionInputErrorCount;

	u16 SessionCAMFilterStartAddr;   /* PTI4L (ram cam) only */
	u16 SessionCAMConfig;            /* PTI4L (ram cam) only */

	u16 SessionPIDFilterStartAddr;
	u16 SessionPIDFilterLength;

	u16 SessionSectionParams;        /* SF_Config (crc & filter type etc.) */
	u16 SessionTSmergerTag;

	u16 SessionProcessState;
	u16 SessionModeFlags;

	u16 SessionNegativePidSlotIdent;
	u16 SessionNegativePidMatchingEnable;

	u16 SessionUnmatchedSlotMode;
	u16 SessionUnmatchedDMACntrl_p;

	u16 SessionInterruptMask0;
	u16 SessionInterruptMask1;

	u16 SessionSTCWord0;
	u16 SessionSTCWord1;

	u16 SessionSTCWord2;
	u16 SessionDiscardParams;

	u32 SessionPIPFilterBytes;
	u32 SessionPIPFilterMask;

	u32 SessionCAPFilterBytes;
	u32 SessionCAPFilterMask;

	u16 SessionEMMFilterOffset;
	u16 SessionSpare;

	u32 SectionEnables_0_31;

	u32 SectionEnables_32_63;

#if defined(SECURE_LITE2)
	u32 SectionEnables_64_95;

	u32 SectionEnables_96_127;

	u16 SessionLastEvtTick_0;
	u16 SessionLastEvtTick_1;

	u16 SessionTickDMA_p;
	u16 SessionTickDMA_Slot;

	u16 SessionSBoxInfo;
	u16 SessionSpareRegister;
#endif

} TCSessionInfo_t;

#else

typedef volatile struct TCSessionInfo_s
{
	u16 SessionInputPacketCount;
	u16 SessionInputErrorCount;

	u16 SessionCAMFilterStartAddr;   /* PTI4L (ram cam) only */
	u16 SessionCAMConfig;            /* PTI4L (ram cam) only */

	u16 SessionPIDFilterStartAddr;
	u16 SessionPIDFilterLength;

	u16 SessionSectionParams;        /* SF_Config (crc & filter type etc.) */
	u16 SessionTSmergerTag;

	u16 SessionProcessState;
	u16 SessionModeFlags;

	u16 SessionNegativePidSlotIdent;
	u16 SessionNegativePidMatchingEnable;

	u16 SessionUnmatchedSlotMode;
	u16 SessionUnmatchedDMACntrl_p;

	u16 SessionInterruptMask0;
	u16 SessionInterruptMask1;

	u32 SectionEnables_0_31;
	u32 SectionEnables_32_63;

	u16 SessionSTCWord0;
	u16 SessionSTCWord1;

	u16 SessionSTCWord2;
	u16 SessionDiscardParams;

	u32 SessionPIPFilterBytes;
	u32 SessionPIPFilterMask;

	u32 SessionCAPFilterBytes;
	u32 SessionCAPFilterMask;

} TCSessionInfo_t;
#endif

#if defined(A18)
typedef volatile struct TCGlobalInfo_s
{
	u16 GlobalPacketHeader;              /* (TC)   */
	u16 GlobalHeaderDesignator;          /* (TC)   */

	u32 GlobalLastQWrite;                /* (TC)   GlobalLastQWrite_0:16  GlobalLastQWrite_1:16 */

	u16 GlobalQPointsInPacket;           /* (TC)   */
	u16 GlobalProcessFlags;              /* (TC)   */

	u16 GlobalSlotMode;                  /* (TC)   */
	u16 GlobalDMACntrl_p;                /* (TC)   */

	u32 GlobalPktCount;                  /* (TC)   Global Input Packet Count */

	u16 GlobalSignalModeFlags;           /* (TC)   */
	u16 GlobalCAMArbiterIdle;            /* (TC)   For GNBvd18811 */

	u16 GlobalSFTimeouts;                /* (TC)   */
	u16 GlobalDTVPktBufferPayload_p;     /* (TC)   */

	u16 GlobalResidue;                   /* (TC)   */
	u16 GlobalPid;                       /* (TC)   */

	u16 GlobalIIFCtrl;                   /* (TC)   */
	u16 GlobalProfilerCount;             /* (TC)   */

	u16 GlobalRecordDMACntrl_p;          /* (TC)   Holds the address of the DMA structure for the Record Buffer */
	u16 GlobalDescramblingAllowed;       /* (TC)  */

	u16 GlobalCAMArbiterInhibit;         /* (Host) For GNBvd18811 */
	u16 GlobalModeFlags;                 /* (Host) */

	u32 GlobalScratch;                   /* (Host) GlobalScratch_0 GlobalScratch_1 */

	u16 GlobalNegativePidSlotIdent;      /* (Host) */
	u16 GlobalNegativePidMatchingEnable; /* (Host) */

	u16 GlobalSwts_Params;               /* (Host) */
	u16 GlobalSFTimeout;                 /* (Host) */

	u16 GlobalTSDumpDMA_p;               /* (Host) */
	u16 GlobalTSDumpCount;               /* (Host) */

	u16 GlobalICAMVersion;               /* (Host)  Host sets this word to tell TC what is the ICAM version used on this silicon */
	u16 GlobalCAAllowEcmEmm;             /* (TC) */

	u16 GlobalCurrentSessionNo;          /* (TC)   Holds the current packet's session number, currently uses 8bits, when reused add mask to all the users appropriately */
	u16 GlobalSpare1;                    /* FREE VARIABLE, USED FOR ALIGNMENT NOW */

	/* Required for passage/overlay support */
	u16 GlobalShadowMainInfo;             /* (TC)  */
	u16 GlobalLegacyMainInfo;             /* (TC)  */

#if defined(SECURE_LITE2)
	u16 GlobalLastTickSessionNumber;     /* (TC )  */
	u16 GlobalTickEnable;                /* (Host) */

	u16 GlobalTickTemp_0;                /* (TC)   */
	u16 GlobalTickTemp_1;                /* (TC)   */

	u16 GlobalLastTickSlotNumber;        /* (TC )  */
	u16 GlobalSkipSetDiscard;            /* (TC )  */
#endif

} TCGlobalInfo_t;

#else
typedef volatile struct TCGlobalInfo_s
{
	u16 GlobalPacketHeader;              /* (TC)   */
	u16 GlobalHeaderDesignator;          /* (TC)   */

	u32 GlobalLastQWrite;                /* (TC)   GlobalLastQWrite_0:16  GlobalLastQWrite_1:16 */

	u16 GlobalQPointsInPacket;           /* (TC)   */
	u16 GlobalProcessFlags;              /* (TC)   */

	u16 GlobalSlotMode;                  /* (TC)   */
	u16 GlobalDMACntrl_p;                /* (TC)   */

	u32 GlobalPktCount;                  /* (TC)   Global Input Packet Count */

	u16 GlobalSignalModeFlags;           /* (TC)   */
	u16 GlobalCAMArbiterIdle;            /* (TC)   For GNBvd18811 */

	u16 GlobalSFTimeouts;                /* (TC)   */
	u16 GlobalDTVPktBufferPayload_p;     /* (TC)   */

	u16 GlobalResidue;                   /* (TC)   */
	u16 GlobalPid;                       /* (TC)   */

	u16 GlobalIIFCtrl;                   /* (TC)   */
	u16 GlobalProfilerCount;             /* (TC)   */

	u16 GlobalRecordDMACntrl_p;          /* (TC)   Holds the address of the DMA structure for the Record Buffer */
	u16 GlobalSpare1;                    /* (TC)   Filter Address for the Start Code Detector */

	u16 GlobalCAMArbiterInhibit;         /* (Host) For GNBvd18811 */
	u16 GlobalModeFlags;                 /* (Host) */

	u32 GlobalScratch;                   /* (Host) GlobalScratch_0 GlobalScratch_1 */

	u16 GlobalNegativePidSlotIdent;      /* (Host) */
	u16 GlobalNegativePidMatchingEnable; /* (Host) */

	u16 GlobalSwts_Params;               /* (Host) */
	u16 GlobalSFTimeout;                 /* (Host) */

	/* Only used for STPTI_TIMER_TICK feature */
	u16 GlobalTSDumpDMA_p;               /* (Host) */
	u16 GlobalTSDumpCount;               /* (Host) */

	u16 GlobalLastTickSessionNumber;     /* (TC )  */
	u16 GlobalTickEnable;                /* (Host) */

	u16 GlobalTickTemp_0;                /* (TC)   */
	u16 GlobalTickTemp_1;                /* (TC)   */

	u16 GlobalLastTickSlotNumber;        /* (TC )  */
	u16 GlobalSkipSetDiscard;            /* (TC )  */
	/* End of STPTI_TIMER_TICK feature variables */

} TCGlobalInfo_t;
#endif

typedef volatile struct
{
	u32 SFFilterDataLS;
	u32 SFFilterMaskLS;
	u32 SFFilterDataMS;
	u32 SFFilterMaskMS;
} TCSectionFilter_t;

#ifdef FRAG_MICH_KEINER_WARUMS_DIE_STRUKTUR_ZWEIMAL_GIBT
typedef volatile struct
{
	TCSectionFilter_t FilterA  [TC_NUMBER_OF_HARDWARE_SECTION_FILTERS];
	TCSectionFilter_t FilterB  [TC_NUMBER_OF_HARDWARE_SECTION_FILTERS];
	u32               NotFilter[TC_NUMBER_OF_HARDWARE_SECTION_FILTERS];
} TCSectionFilterArrays_t;
#endif

typedef volatile union TCFilterLine_s
{
	struct
	{
		u8 Filter[SF_CAMLINE_WIDTH];
	} Element;

	u32 Word;

} TCFilterLine_t;

typedef volatile struct TCCamEntry_s
{
	TCFilterLine_t Data;
	TCFilterLine_t Mask;
} TCCamEntry_t;

typedef volatile struct TCCamIndex_s
{
	TCCamEntry_t Index[ SF_FILTER_LENGTH ];
} TCCamIndex_t;

typedef volatile struct TCSectionFilterArrays_s
{
	TCCamIndex_t CamA_Block[ SF_NUM_BLOCKS_PER_CAM ];
	u32          ReservedA[256 / 4];

	TCCamIndex_t CamB_Block[ SF_NUM_BLOCKS_PER_CAM ];
	u32          ReservedB[256 / 4];

	u32          NotFilter[ TC_NUMBER_OF_HARDWARE_NOT_FILTERS ];
} TCSectionFilterArrays_t;

typedef volatile struct TCDevice_s
{
	u32 PTIIntStatus0;
	u32 PTIIntStatus1;
	u32 PTIIntStatus2;
	u32 PTIIntStatus3;

	u32 PTIIntEnable0;
	u32 PTIIntEnable1;
	u32 PTIIntEnable2;
	u32 PTIIntEnable3;

	u32 PTIIntAck0;
	u32 PTIIntAck1;
	u32 PTIIntAck2;
	u32 PTIIntAck3;

	u32 TCMode;

	u32 DMAempty_STAT;   /* 3 bits RO */
	u32 DMAempty_EN;     /* 3 bits RW */

	u32 TCPadding_0;

	u32 PTIAudPTS_31_0;
	u32 PTIAudPTS_32;

	u32 PTIVidPTS_31_0;
	u32 PTIVidPTS_32;

	u32 STCTimer0;
	u32 STCTimer1;

	u32 TCPadding_1[(0x1000 - 22 * sizeof(u32)) / sizeof(u32)];

	u32 DMA0Base;
	u32 DMA0Top;
	u32 DMA0Write;
	u32 DMA0Read;
	u32 DMA0Setup;
	u32 DMA0Holdoff;
	u32 DMA0Status;
	u32 DMAEnable;

	u32 DMA1Base;
	u32 DMA1Top;
	u32 DMA1Write;
	u32 DMA1Read;
	u32 DMA1Setup;
	u32 DMA1Holdoff;
	u32 DMA1CDAddr;
	u32 DMASecStart;

	u32 DMA2Base;
	u32 DMA2Top;
	u32 DMA2Write;
	u32 DMA2Read;
	u32 DMA2Setup;
	u32 DMA2Holdoff;
	u32 DMA2CDAddr;
	u32 DMAFlush;

	u32 DMA3Base;
	u32 DMA3Top;
	u32 DMA3Write;
	u32 DMA3Read;
	u32 DMA3Setup;
	u32 DMA3Holdoff;
	u32 DMA3CDAddr;
	u32 DMAPTI3Prog;

	u32 TCPadding_2[(0x2000 - (0x1000 + 32 * sizeof(u32))) / sizeof(u32)];

	u32 TCPadding_4[(0x20e0 - 0x2000) / sizeof(u32)];

	u32 IIFCAMode;

	u32 TCPadding_5[(0x4000 - (0x2000 + 57 * sizeof(u32))) / sizeof(u32)];

	TCSectionFilterArrays_t TC_SectionFilterArrays; /* std or ram cam */

	u32 TCPadding_6[(0x6000 - (0x4000 + sizeof(TCSectionFilterArrays_t))) / sizeof(u32)];

	u32 IIFFIFOCount;
	u32 IIFAltFIFOCount;
	u32 IIFFIFOEnable;
	u32 TCPadding_3[1];
	u32 IIFAltLatency;
	u32 IIFSyncLock;
	u32 IIFSyncDrop;
	u32 IIFSyncConfig;
	u32 IIFSyncPeriod;

	u32 TCPadding_7[(0x7000 - (0x6000 + 9 * sizeof(u32))) / sizeof(u32)];

	u32 TCRegA;
	u32 TCRegB;
	u32 TCRegC;
	u32 TCRegD;
	u32 TCRegP;
	u32 TCRegQ;
	u32 TCRegI;
	u32 TCRegO;
	u32 TCIPtr;
	u32 TCRegE0;
	u32 TCRegE1;
	u32 TCRegE2;
	u32 TCRegE3;
	u32 TCRegE4;
	u32 TCRegE5;
	u32 TCRegE6;
	u32 TCRegE7;

	u32 TCPadding_8[(0x8000 - (0x7000 + 17 * sizeof(u32))) / sizeof(u32)];

	u32 TC_Data[TC_DATA_RAM_SIZE / sizeof(u32)];

	u32 TCPadding_9[(0xC000 - (0x8000 + TC_DATA_RAM_SIZE)) / sizeof(u32)];

	u32 TC_Code[TC_CODE_RAM_SIZE / sizeof(u32)];
} TCDevice_t;

typedef struct
{
	STPTI_DevicePtr_t TC_CodeStart;
	size_t            TC_CodeSize;
	STPTI_DevicePtr_t TC_DataStart;

	STPTI_DevicePtr_t TC_LookupTableStart;
	STPTI_DevicePtr_t TC_GlobalDataStart;
	STPTI_DevicePtr_t TC_StatusBlockStart;
	STPTI_DevicePtr_t TC_MainInfoStart;
	STPTI_DevicePtr_t TC_DMAConfigStart;
	STPTI_DevicePtr_t TC_DescramblerKeysStart;
	STPTI_DevicePtr_t TC_TransportFilterStart;
	STPTI_DevicePtr_t TC_SCDFilterTableStart;
	STPTI_DevicePtr_t TC_PESFilterStart;
	STPTI_DevicePtr_t TC_SubstituteDataStart;
	STPTI_DevicePtr_t TC_SystemKeyStart;    /* Descambler support */
	STPTI_DevicePtr_t TC_SFStatusStart;
	STPTI_DevicePtr_t TC_InterruptDMAConfigStart;
	STPTI_DevicePtr_t TC_EMMStart;
	STPTI_DevicePtr_t TC_ECMStart;              /* Possibly redundant */
	STPTI_DevicePtr_t TC_MatchActionTable;
	STPTI_DevicePtr_t TC_SessionDataStart;
	STPTI_DevicePtr_t TC_VersionID;

	u16               TC_NumberCarousels;
	u16               TC_NumberSystemKeys;    /* Descambler support */
	u16               TC_NumberDMAs;
	u16               TC_NumberDescramblerKeys;
	u16               TC_SizeOfDescramblerKeys;
	u16               TC_NumberIndexs;
	u16               TC_NumberPesFilters;
	u16               TC_NumberSectionFilters;
	u16               TC_NumberSlots;
	u16               TC_NumberTransportFilters;
	u16               TC_NumberEMMFilters;
	u16               TC_SizeOfEMMFilter;
	u16               TC_NumberECMFilters;
	u16               TC_NumberOfSessions;
	u16               TC_NumberSCDFilters;

	int               TC_AutomaticSectionFiltering;
	int               TC_MatchActionSupport;
	int               TC_SignalEveryTransportPacket;

} STPTI_TCParameters_t;

typedef volatile struct TCMainInfo_s
{
	u16 SlotState;
	u16 PacketCount;

	u16 SlotMode;
	u16 DescramblerKeys_p;          /* a.k.a. CAPAccumulatedCount, CAPAccumulatedCount */

	u16 DMACtrl_indices;
	u16 IndexMask;

	u16 SectionPesFilter_p;
	u16 RemainingPESLength;         /* a.k.a. ECMFilterMask */

	u16 PESStuff;                   /* a.k.a. CAPFilterResult, RawCorruptionParams, ECMFilterData */
	u16 StartCodeIndexing_p;        /* a.k.a. RecordBufferMode */

#if defined(SECURE_LITE2)
	u16 Overlay_p;                  /* passage support only */
	u16 MainInfoSpare;              /* SPARE REGISTER - ADDED FOR ALIGNMENT */
#endif
} TCMainInfo_t;

typedef volatile struct TCInterruptDMAConfig_s
{
	u32 DMABase_p;
	u32 DMATop_p;
	u32 DMAWrite_p;
	u32 DMARead_p;
} TCInterruptDMAConfig_t;

#if defined(A18)

#define STPTI_MAX_START_CODES_SUPPORTED 7

typedef struct StartCode_s
{
	u8 Offset;
	u8 Code;
} StartCode_t;

typedef volatile struct TCStatus_s
{
	u32 Flags;

	u32 SlotError: 8;
	u32 SlotNumber: 8;
	u32 Odd_Even: 1;
	u32 PESScrambled: 1;
	u32 Scrambled: 1;
	u32 Padding: 13;

	u32 ArrivalTime0: 16;
	u32 ArrivalTime1: 16;

	u32 ArrivalTime2: 16;
	u32 DMACtrl: 16;

	u32 Pcr0: 16;
	u32 Pcr1: 16;

	u32 Pcr2: 16;
	u32 NumberStartCodes: 8;
	u32 PayloadLength: 8;

	u32 BufferPacketNumber;
	u32 BufferPacketOffset;
	u32 RecordBufferPacketNumber;

	u32 CarouselInfo;

#if defined(SECURE_LITE2)
	u32 StartCodePreviousBPN: 8;
	u32 StartCodePreviousRecordBPN: 8;
	StartCode_t StartCodes[STPTI_MAX_START_CODES_SUPPORTED];
#endif

} TCStatus_t;
#else
typedef volatile struct TCStatus_s
{
	u32 Flags;

	u32 SlotError: 8;
	u32 SlotNumber: 8;
	u32 Odd_Even: 1;
	u32 PESScrambled: 1;
	u32 Scrambled: 1;
	u32 Padding: 13;

	u32 ArrivalTime0: 16;
	u32 ArrivalTime1: 16;

	u32 ArrivalTime2: 16;
	u32 DMACtrl: 16;

	u32 Pcr0: 16;
	u32 Pcr1: 16;

	u32 Pcr2: 16;
	u32 NumberStartCodes: 8;
	u32 PayloadLength: 8;

	u32 BufferPacketNumber;
	u32 BufferPacketOffset;
	u32 RecordBufferPacketNumber;

} TCStatus_t;
#endif

typedef volatile struct TCDMAConfig_s
{
	u32 DMABase_p;
	u32 DMATop_p;
	u32 DMAWrite_p;
	u32 DMARead_p;
	u32 DMAQWrite_p;
	u32 BufferPacketCount;

	u16 SignalModeFlags;
	u16 Threshold;
#if defined(SECURE_LITE2)
	u16 BufferLevelThreshold;
	u16 DMAConfig_Spare;
#endif
} TCDMAConfig_t;

struct TCDMAConfigExt_s
{
	u32 BasePtr_physical;
	u32 TopPtr_physical;
	u8 *pBuf;
	u32 bufSize;
	u32 bufSize_sub_188;
	u32 bufSize_div_188;
	u32 bufSize_div_188_div_2;
};

/* Note that when writing host to TC the key undergoes an endian swap,
this is more complex for the extended AES keys as they are split in 2 sections.
Thus even keys 3:0 are effectively keys 7:4 for AES 16 byte keys... etc
This maintains compatibility with 8 byte keys, and gives the simplest (fastest)
TC access to the extended AES keys */

typedef volatile struct TCKey_s
{
	u16 KeyValidity;
	u16 KeyMode;

	u16 EvenKey3;
	u16 EvenKey2;

	u16 EvenKey1;
	u16 EvenKey0;

	u16 OddKey3;
	u16 OddKey2;

	u16 OddKey1;
	u16 OddKey0;
	/* This is the end of the used portion for non-AES keys */

	u16 EvenKey7;
	u16 EvenKey6;
	u16 EvenKey5;
	u16 EvenKey4;

	u16 EvenIV7;
	u16 EvenIV6;
	u16 EvenIV5;
	u16 EvenIV4;
	u16 EvenIV3;
	u16 EvenIV2;
	u16 EvenIV1;
	u16 EvenIV0;

	u16 OddKey7;
	u16 OddKey6;
	u16 OddKey5;
	u16 OddKey4;

	u16 OddIV7;
	u16 OddIV6;
	u16 OddIV5;
	u16 OddIV4;
	u16 OddIV3;
	u16 OddIV2;
	u16 OddIV1;
	u16 OddIV0;

} TCKey_t;

#define TCKEY_VALIDITY_TS_EVEN     0x0001
#define TCKEY_VALIDITY_TS_ODD      0x0002
#define TCKEY_VALIDITY_PES_EVEN    0x0100
#define TCKEY_VALIDITY_PES_ODD     0x0200

#define TCKEY_ALGORITHM_DVB        0x0000
#define TCKEY_ALGORITHM_DSS        0x1000
#define TCKEY_ALGORITHM_FAST_I     0x2000
#define TCKEY_ALGORITHM_AES        0x3000
#define TCKEY_ALGORITHM_MULTI2     0x4000
#define TCKEY_ALGORITHM_MASK       0xF000

#define TCKEY_CHAIN_ALG_ECB        0x0000
#define TCKEY_CHAIN_ALG_CBC        0x0010
#define TCKEY_CHAIN_ALG_ECB_IV     0x0020
#define TCKEY_CHAIN_ALG_CBC_IV     0x0030
#define TCKEY_CHAIN_ALG_OFB        0x0040
#define TCKEY_CHAIN_ALG_CTS        0x0050
#define TCKEY_CHAIN_ALG_MASK       0x00F0

#define TCKEY_CHAIN_MODE_LR        0x0004

/*
  Use for   TCSectionFilterInfo_t
            TCSessionInfo_t
            TCMainInfo_t
            TCGlobalInfo_t
            TCKey_t
            TCDMAConfig_t

   The TC registers definded in pti4.h that are u32:16 bit fields
   need be read modified and written back.

   A single 16 bit write to ether the high or low part of a u32 is written
   to both the low and high parts.

   So the unchanged part needs to be read and ORed with the changed part and
   written as a u32.

   This function does just that.

   It may not be the best way to do it but it works!!
*/

/* This macro determines whether the write is to the high or low word.
   Reads in the complete 32 bit word writes in the high or low portion and
   then writes the whole word back.
   It get around the TC read modify write problem.
   This macro is a more efficient vertion of the macro below and the function
   below that.        */
#define STSYS_WriteTCReg16LE( reg, u16value ) \
	{ \
		u32 u32value = readl( (void*)((u32)(reg) & 0xfffffffC) );\
		*(u16*)((u32)&u32value + ((u32)(reg) & 0x02)) = (u16value);\
		writel(u32value, (void*)((u32)(reg) & 0xfffffffC) );\
	}

/* Read the register and OR in the value and write it back. */
#define STSYS_SetTCMask16LE( reg, u16value)\
	{\
		STSYS_WriteTCReg16LE(reg, readw(reg) | u16value);\
	}

/* Read the register and AND in the complement(~) of the value and write it back. */
#define STSYS_ClearTCMask16LE( reg, u16value)\
	{\
		STSYS_WriteTCReg16LE(reg, readw(reg) & ~u16value);\
	}

#define GetTCData(ptr, x) {                                                 \
		u32 tmp;                                        \
		STPTI_DevicePtr_t p;                            \
		if(((u32)(ptr) % 4) == 0)                       \
		{                                               \
			p = (STPTI_DevicePtr_t)(ptr);               \
			tmp = readl( (void*)p );          \
		}                                               \
		else                                            \
		{                                               \
			p = (STPTI_DevicePtr_t)((u32)(ptr) & ~0x3); \
			tmp = readl( (void*)p );          \
			tmp >>= 16;                             \
		}                                               \
		tmp &= 0xffff;                                  \
		(x) = tmp;                                      \
	}

#define PutTCData(ptr, x) {                                     \
		u32 tmp;                                        \
		STPTI_DevicePtr_t p;                            \
		if(((u32)(ptr) % 4) == 0)                       \
		{                                               \
			p = (STPTI_DevicePtr_t)(ptr);           \
			tmp = readl( (void*)p );          \
			tmp &= 0xffff0000;                          \
			tmp |= ((x)&0xffff);                        \
		}                                               \
		else                                            \
		{                                               \
			p = (STPTI_DevicePtr_t)((u32)(ptr) & ~0x3); \
			tmp = readl( (void*)p );          \
			tmp &= 0x0000ffff;                          \
			tmp |= ((x)<<16);                           \
		}                                               \
		writel( tmp, (void*)p );              \
	}

extern irqreturn_t pti_interrupt_handler(int irq, void *data,
					 struct pt_regs *regs);

extern int pti_task(void *data);

extern int debug ;
#define dprintk(x...) do { if (debug) printk(KERN_WARNING x); } while (0)

#endif //_PTI_H_
