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
#include <linux/dma-mapping.h>

#include <asm/io.h>

#if defined (CONFIG_KERNELVERSION) || LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#include <linux/bpa2.h>
#else
#include <linux/bigphysarea.h>
#endif

#include <linux/dvb/ca.h>

#include "tc_code.h"
#include "pti.h"
#include <pti_public.h>

#ifdef UFS910
int camRouting = 0;
#endif

int debug = 0;

/* video memory in dvb packets */
#ifdef UFS910
int videoMem = 2048;
#else
int videoMem = 4096;
#endif

/* waiting time in ms for wait queue in pti_prcoess.c */
int waitMS = 20;

#ifdef CONFIG_PRINTK
/* enable statistic output on pti */
int enableStatistic = 0;
#else
int enableSysStatistic = 1;

unsigned long pti_last_time = 0;
unsigned long pti_count = 0;
#endif

static void *stpti_MemoryAlign(void *Memory_p, u32 Alignment)
{
	return (void *)((u32)((u32)Memory_p + Alignment - 1) & ~(Alignment - 1));
}

static size_t stpti_BufferSizeAdjust(size_t Size)
{
	return ((size_t)stpti_MemoryAlign((void *)Size, STPTI_BUFFER_SIZE_MULTIPLE));
}

static void *stpti_BufferBaseAdjust(void *Base_p)
{
	return (stpti_MemoryAlign(Base_p, STPTI_BUFFER_ALIGN_MULTIPLE));
}

/* *****************************
 * Global vars
 */
TCDevice_t *myTC = NULL;
STPTI_TCParameters_t tc_params;

#define STSYS_WriteRegDev32LE(a, b) writel(b,a)
#define STSYS_ReadRegDev32LE(x) readl(x)

static void stopTc(TCDevice_t *TC_Device)
{
	STSYS_WriteRegDev32LE((void *)&TC_Device->IIFFIFOEnable, 0x00); /* Stop the IIF */
	STSYS_WriteRegDev32LE((void *)&TC_Device->TCMode, 0x00);        /* Stop the TC */
	/* --- Perform a software reset --- */
	STSYS_WriteRegDev32LE((void *)&TC_Device->DMAPTI3Prog, 0x01);   /* PTI3 Mode */
	STSYS_WriteRegDev32LE((void *)&TC_Device->DMAFlush, 0x01);      /* Flush DMA 0 */
	/* For PTI4SL, if we do not enable the DMA here, the flush never occurs... */
	STSYS_WriteRegDev32LE((void *)&TC_Device->DMAEnable, 0x01);     /* Enable the DMA */
	while (STSYS_ReadRegDev32LE((void *)&TC_Device->DMAFlush) & 0X01)
		udelay(100 * 64); /* 6400us */
	STSYS_WriteRegDev32LE((void *)&TC_Device->TCMode, 0x08);        /* Full Reset the TC */
	udelay(200 * 64); /* 12800us */             /* Wait */
	STSYS_WriteRegDev32LE((void *)&TC_Device->TCMode, 0x00);        /* Finish Full Reset the TC */
	udelay(10 * 64); /* 640us */
}

/* ************************************
 * From InitializeDevice in basic.c
 * the TC is only for pti4 otherwise
 * another loader should be used
 */
void pti_main_loadtc(struct stpti *pti)
{
	STPTI_DevicePtr_t CodeStart;
	TCDevice_t        *TC_Device;
	u16               IIFSyncPeriod;
	u16               TagBytes;
	u32               PTI4_Base, i;
	TCGlobalInfo_t    *TCGlobalInfo;
	static u8         DmaFifoFlushingTarget[188 + DMAScratchAreaSize];
	int               session;  /* Initialize session info structures */
	u8                *base_p;
	u8                *top_p;
	size_t            DMABufferSize = 0;
	int               block, index;
	TCSectionFilterArrays_t *TC_SectionFilterArrays_p;
	STPTI_TCParameters_t  *TC_Params_p;
	TCInterruptDMAConfig_t *TCInterruptDMAConfig_p;
	myTC = (TCDevice_t *) pti->pti_io;
	CodeStart = &myTC->TC_Code[0];
	PTI4_Base = (u32)((u32)CodeStart - 0XC000);
	TC_Params_p = &tc_params;
	ioremap_nocache((unsigned long)&myTC, sizeof(TCDevice_t));
// normally before start loading tc we should stop tc but in our
// case I hope there isnt running later on we should do this
// ->see stptiHelper_TCInit_Stop
	//stopTc(myTC);
	// *********************************
	//Load TC
	printk("Load real TC Code\n");
	printk("TC_Data = %p, TC_Code = %p\n", myTC->TC_Data, myTC->TC_Code);
	TC_Params_p->TC_CodeStart                  = (STPTI_DevicePtr_t)VERSION;
	TC_Params_p->TC_CodeStart                  = CodeStart;
	TC_Params_p->TC_CodeSize                   = TRANSPORT_CONTROLLER_CODE_SIZE * sizeof(u32);
	TC_Params_p->TC_DataStart                  = (u32 *)(((u32)PTI4_Base) + 0X8000);
#if (defined(UFS912) || defined(UFS913) || defined(SPARK) || defined(SPARK7162) || defined(ATEVIO7500) || defined(HS7810A) || defined(HS7110)) && defined(SECURE_LITE2) && defined(A18)
	TC_Params_p->TC_LookupTableStart           = (u32 *)(((u32)PTI4_Base) + 0X8000);
	TC_Params_p->TC_SystemKeyStart             = (u32 *)(((u32)PTI4_Base) + 0X80C0);
	TC_Params_p->TC_GlobalDataStart            = (u32 *)(((u32)PTI4_Base) + 0X850C);
	TC_Params_p->TC_StatusBlockStart           = (u32 *)(((u32)PTI4_Base) + 0X8560);
	TC_Params_p->TC_MainInfoStart              = (u32 *)(((u32)PTI4_Base) + 0X8598);
	TC_Params_p->TC_DMAConfigStart             = (u32 *)(((u32)PTI4_Base) + 0X8E98);
	TC_Params_p->TC_DescramblerKeysStart       = (u32 *)(((u32)PTI4_Base) + 0X9578);
	TC_Params_p->TC_TransportFilterStart       = (u32 *)(((u32)PTI4_Base) + 0X9C1C);
	TC_Params_p->TC_SCDFilterTableStart        = (u32 *)(((u32)PTI4_Base) + 0X9C1C);
	TC_Params_p->TC_PESFilterStart             = (u32 *)(((u32)PTI4_Base) + 0X9CDC);
	TC_Params_p->TC_SubstituteDataStart        = (u32 *)(((u32)PTI4_Base) + 0X81C8);
	TC_Params_p->TC_SFStatusStart              = (u32 *)(((u32)PTI4_Base) + 0X9CDC);
	TC_Params_p->TC_InterruptDMAConfigStart    = (u32 *)(((u32)PTI4_Base) + 0XB0DC);
	TC_Params_p->TC_SessionDataStart           = (u32 *)(((u32)PTI4_Base) + 0XB0EC);
	TC_Params_p->TC_EMMStart                   = (u32 *)(((u32)PTI4_Base) + 0XB2A4);
	TC_Params_p->TC_ECMStart                   = (u32 *)(((u32)PTI4_Base) + 0XB2A4);
	TC_Params_p->TC_VersionID                  = (u32 *)(((u32)PTI4_Base) + 0XB2E4);
	TC_Params_p->TC_NumberCarousels            = 0X0001;
	TC_Params_p->TC_NumberSystemKeys           = 0X0001;
	TC_Params_p->TC_NumberDMAs                 = 0X0037;
	TC_Params_p->TC_NumberDescramblerKeys      = 0X0019;
	TC_Params_p->TC_SizeOfDescramblerKeys      = 0X0044;
	TC_Params_p->TC_NumberPesFilters           = 0X0000;
	TC_Params_p->TC_NumberSectionFilters       = 0X0080;
	TC_Params_p->TC_NumberSlots                = 0X0060;
	TC_Params_p->TC_NumberOfSessions           = 0X0005;
	TC_Params_p->TC_NumberIndexs               = 0X0060;
	TC_Params_p->TC_NumberTransportFilters     = 0X0000;
	TC_Params_p->TC_NumberSCDFilters           = 0X0018;
	TC_Params_p->TC_SignalEveryTransportPacket = 0X0001;
	TC_Params_p->TC_NumberEMMFilters           = 0X0000;
	TC_Params_p->TC_SizeOfEMMFilter            = 0X001C;
	TC_Params_p->TC_NumberECMFilters           = 0X0060;
	TC_Params_p->TC_AutomaticSectionFiltering  = FALSE;
	{
		STPTI_DevicePtr_t DataStart = (u32 *)(((u32)CodeStart & 0xffff0000) | 0X8000);
		for (i = 0; i < (0X3800 / 4); ++i)
		{
			writel(0x00 , (u32)&DataStart[i]);
		}
		for (i = 0; i < 0X1000; ++i)
		{
			writel(0x00 , (u32)&CodeStart[i]);
		}
		for (i = 0; i < TRANSPORT_CONTROLLER_CODE_SIZE; ++i)
		{
			writel(transport_controller_code[i], (u32)&CodeStart[i]);
		}
		writel(0X5354 | (0X5054 << 16), (u32) & (TC_Params_p->TC_VersionID[0]));
		writel(0X4934 | (0X001F << 16), (u32) & (TC_Params_p->TC_VersionID[1]));
		writel(0X0803 | (0X0000 << 16), (u32) & (TC_Params_p->TC_VersionID[2]));
		printk("Readback TC ... ");
		for (i = 0; i < TRANSPORT_CONTROLLER_CODE_SIZE; ++i)
		{
			unsigned long res = readl((u32)&CodeStart[i]);
			if (res != transport_controller_code[i])
			{
				printk("failed !!!!\n");
				break;
			}
		}
		if (i == TRANSPORT_CONTROLLER_CODE_SIZE)
		{
			printk("successfull\n");
		}
	}
#elif defined(A18)
	TC_Params_p->TC_LookupTableStart           = (u32 *)(((u32)PTI4_Base) + 0X8000);
	TC_Params_p->TC_SystemKeyStart             = (u32 *)(((u32)PTI4_Base) + 0X80C0);
	TC_Params_p->TC_GlobalDataStart            = (u32 *)(((u32)PTI4_Base) + 0X8270);
	TC_Params_p->TC_StatusBlockStart           = (u32 *)(((u32)PTI4_Base) + 0X82B8);
	TC_Params_p->TC_MainInfoStart              = (u32 *)(((u32)PTI4_Base) + 0X82E0);
	TC_Params_p->TC_DMAConfigStart             = (u32 *)(((u32)PTI4_Base) + 0X8A60);
	TC_Params_p->TC_DescramblerKeysStart       = (u32 *)(((u32)PTI4_Base) + 0X902C);
	TC_Params_p->TC_TransportFilterStart       = (u32 *)(((u32)PTI4_Base) + 0X9194);
	TC_Params_p->TC_SCDFilterTableStart        = (u32 *)(((u32)PTI4_Base) + 0X9194);
	TC_Params_p->TC_PESFilterStart             = (u32 *)(((u32)PTI4_Base) + 0X9194);
	TC_Params_p->TC_SubstituteDataStart        = (u32 *)(((u32)PTI4_Base) + 0X81A8);
	TC_Params_p->TC_SFStatusStart              = (u32 *)(((u32)PTI4_Base) + 0X9194);
	TC_Params_p->TC_InterruptDMAConfigStart    = (u32 *)(((u32)PTI4_Base) + 0X9894);
	TC_Params_p->TC_SessionDataStart           = (u32 *)(((u32)PTI4_Base) + 0X98A4);
	TC_Params_p->TC_EMMStart                   = (u32 *)(((u32)PTI4_Base) + 0X9970);
	TC_Params_p->TC_ECMStart                   = (u32 *)(((u32)PTI4_Base) + 0X9970);
	TC_Params_p->TC_VersionID                  = (u32 *)(((u32)PTI4_Base) + 0X9970);
	TC_Params_p->TC_NumberCarousels            = 0X0001;
	TC_Params_p->TC_NumberSystemKeys           = 0X0000;
	TC_Params_p->TC_NumberDMAs                 = 0X0035;
	TC_Params_p->TC_NumberDescramblerKeys      = 0X0012;
	TC_Params_p->TC_SizeOfDescramblerKeys      = 0X0014;
	TC_Params_p->TC_NumberPesFilters           = 0X0000;
	TC_Params_p->TC_NumberSectionFilters       = 0X0040;
	TC_Params_p->TC_NumberSlots                = 0X0060;
	TC_Params_p->TC_NumberOfSessions           = 0X0003;
	TC_Params_p->TC_NumberIndexs               = 0X0060;
	TC_Params_p->TC_NumberTransportFilters     = 0X0000;
	TC_Params_p->TC_NumberSCDFilters           = 0X0000;
	TC_Params_p->TC_SignalEveryTransportPacket = 0X0001;
	TC_Params_p->TC_NumberEMMFilters           = 0X0000;
	TC_Params_p->TC_SizeOfEMMFilter            = 0X001C;
	TC_Params_p->TC_NumberECMFilters           = 0X0060;
	TC_Params_p->TC_AutomaticSectionFiltering  = FALSE;
	{
		STPTI_DevicePtr_t DataStart = (u32 *)(((u32)CodeStart & 0xffff0000) | 0X8000);
		for (i = 0; i < (0X1A00 / 4); ++i)
		{
			writel(0x00 , (u32)&DataStart[i]);
		}
		for (i = 0; i < 0X0780; ++i)
		{
			writel(0x00 , (u32)&CodeStart[i]);
		}
		for (i = 0; i < TRANSPORT_CONTROLLER_CODE_SIZE; ++i)
		{
			writel(transport_controller_code[i], (u32)&CodeStart[i]);
		}
		writel(0X5354 | (0X5054 << 16), (u32) & (TC_Params_p->TC_VersionID[0]));
		writel(0X4934 | (0X0000 << 16), (u32) & (TC_Params_p->TC_VersionID[1]));
		writel(0X0803 | (0X0000 << 16), (u32) & (TC_Params_p->TC_VersionID[2]));
		printk("Readback TC ... ");
		for (i = 0; i < TRANSPORT_CONTROLLER_CODE_SIZE; ++i)
		{
			unsigned long res = readl((u32)&CodeStart[i]);
			if (res != transport_controller_code[i])
			{
				printk("failed !!!!\n");
				break;
			}
		}
		if (i == TRANSPORT_CONTROLLER_CODE_SIZE)
		{
			printk("successfull\n");
		}
	}
#else
	TC_Params_p->TC_LookupTableStart           = (u32 *)(((u32)PTI4_Base) + 0X8000);
	TC_Params_p->TC_SystemKeyStart             = (u32 *)(((u32)PTI4_Base) + 0X80C0);
	TC_Params_p->TC_GlobalDataStart            = (u32 *)(((u32)PTI4_Base) + 0X826C);
	TC_Params_p->TC_StatusBlockStart           = (u32 *)(((u32)PTI4_Base) + 0X82B4);
	TC_Params_p->TC_MainInfoStart              = (u32 *)(((u32)PTI4_Base) + 0X82D8);
	TC_Params_p->TC_DMAConfigStart             = (u32 *)(((u32)PTI4_Base) + 0X8A58);
	TC_Params_p->TC_DescramblerKeysStart       = (u32 *)(((u32)PTI4_Base) + 0X905C);
	TC_Params_p->TC_TransportFilterStart       = (u32 *)(((u32)PTI4_Base) + 0X91C4);
	TC_Params_p->TC_SCDFilterTableStart        = (u32 *)(((u32)PTI4_Base) + 0X91C4);
	TC_Params_p->TC_PESFilterStart             = (u32 *)(((u32)PTI4_Base) + 0X91C4);
	TC_Params_p->TC_SubstituteDataStart        = (u32 *)(((u32)PTI4_Base) + 0X81A8);
	TC_Params_p->TC_SFStatusStart              = (u32 *)(((u32)PTI4_Base) + 0X91C4);
	TC_Params_p->TC_InterruptDMAConfigStart    = (u32 *)(((u32)PTI4_Base) + 0X98C4);
	TC_Params_p->TC_SessionDataStart           = (u32 *)(((u32)PTI4_Base) + 0X98D4);
	TC_Params_p->TC_EMMStart                   = (u32 *)(((u32)PTI4_Base) + 0X9994);
	TC_Params_p->TC_ECMStart                   = (u32 *)(((u32)PTI4_Base) + 0X9994);
	TC_Params_p->TC_VersionID                  = (u32 *)(((u32)PTI4_Base) + 0X99B4);
	TC_Params_p->TC_NumberCarousels            = 0X0001;
	TC_Params_p->TC_NumberSystemKeys           = 0X0000;
	TC_Params_p->TC_NumberDMAs                 = 0X0037;
	TC_Params_p->TC_NumberDescramblerKeys      = 0X0012;
	TC_Params_p->TC_SizeOfDescramblerKeys      = 0X0014;
	TC_Params_p->TC_NumberPesFilters           = 0X0000;
	TC_Params_p->TC_NumberSectionFilters       = 0X0040;
	TC_Params_p->TC_NumberSlots                = 0X0060;
	TC_Params_p->TC_NumberOfSessions           = 0X0003;
	TC_Params_p->TC_NumberIndexs               = 0X0060;
	TC_Params_p->TC_NumberTransportFilters     = 0X0000;
	TC_Params_p->TC_NumberSCDFilters           = 0X0000;
	TC_Params_p->TC_SignalEveryTransportPacket = 0X0001;
	TC_Params_p->TC_NumberEMMFilters           = 0X0000;
	TC_Params_p->TC_NumberECMFilters           = 0X0060;
	TC_Params_p->TC_AutomaticSectionFiltering  = FALSE;
	{
		STPTI_DevicePtr_t DataStart = (u32 *)(((u32)CodeStart & 0xffff0000) | 0X8000);
		for (i = 0; i < (0X1A00 / 4); ++i)
		{
			writel(0x00 , (u32)&DataStart[i]);
		}
		for (i = 0; i < 0X0900; ++i)
		{
			writel(0x00 , (u32)&CodeStart[i]);
		}
		for (i = 0; i < TRANSPORT_CONTROLLER_CODE_SIZE; ++i)
		{
			writel(transport_controller_code[i], (u32)&CodeStart[i]);
		}
		writel(0X5354 | (0X5054 << 16), (u32) & (TC_Params_p->TC_VersionID[0]));
		writel(0X4934 | (0X0000 << 16), (u32) & (TC_Params_p->TC_VersionID[1]));
		writel(0X84D0 | (0X0000 << 16), (u32) & (TC_Params_p->TC_VersionID[2]));
		printk("Readback TC ... ");
		for (i = 0; i < TRANSPORT_CONTROLLER_CODE_SIZE; ++i)
		{
			unsigned long res = readl((u32)&CodeStart[i]);
			if (res != transport_controller_code[i])
			{
				printk("failed !!!!\n");
				break;
			}
		}
		if (i == TRANSPORT_CONTROLLER_CODE_SIZE)
		{
			printk("successfull\n");
		}
	}
#endif
	//stopTc(myTC);
	//Load TC
	// *********************************
	// *********************************
	//Init Hardware (stptiHelper_TCInit_Hardware)
	TC_Device = myTC;
	IIFSyncPeriod = 188 /* DVB_TS_PACKET_LENGTH*/; /* default is 188 for DVB */
	TagBytes = 0; /* default is TSMERGER in bypass mode (or no TSMERGER) */
	/* --- Initialize TC registers --- */
	writel(0 , (void *)&TC_Device->TCMode);
	writel(0xffff, (void *)&TC_Device->PTIIntAck0);
	writel(0xffff, (void *)&TC_Device->PTIIntAck1);
	writel(0xffff, (void *)&TC_Device->PTIIntAck2);
	writel(0xffff, (void *)&TC_Device->PTIIntAck3);
	/* disable all interrupts */
	writel(0 , (void *)&TC_Device->PTIIntEnable0);
	writel(0 , (void *)&TC_Device->PTIIntEnable1);
	writel(0 , (void *)&TC_Device->PTIIntEnable2);
	writel(0 , (void *)&TC_Device->PTIIntEnable3);
	/* Initialise various registers */
	writel(0 , (void *)&TC_Device->STCTimer0);
	writel(0 , (void *)&TC_Device->STCTimer1);
	/* Initialise DMA Registers */
	writel(0 , (void *)&TC_Device->DMAEnable);             /* Disable DMAs */
	writel(1 , (void *)&TC_Device->DMAPTI3Prog);           /* PTI3 Mode */
	writel(0 , (void *)&TC_Device->DMA0Base);
	writel(0 , (void *)&TC_Device->DMA0Top);
	writel(0 , (void *)&TC_Device->DMA0Write);
	writel(0 , (void *)&TC_Device->DMA0Read);
	writel(0 , (void *)&TC_Device->DMA0Setup);
	writel((1 | (1 << 16)), (void *)&TC_Device->DMA0Holdoff);
	writel(0 , (void *)&TC_Device->DMA0Status);
	writel(0 , (void *)&TC_Device->DMA1Base);
	writel(0 , (void *)&TC_Device->DMA1Top);
	writel(0 , (void *)&TC_Device->DMA1Write);
	writel(0 , (void *)&TC_Device->DMA1Read);
	writel(0 , (void *)&TC_Device->DMA1Setup);
	writel((1 | (1 << 16)) , (void *)&TC_Device->DMA1Holdoff);
	writel(0 , (void *)&TC_Device->DMA1CDAddr);
	writel(0 , (void *)&TC_Device->DMASecStart);
	writel(0 , (void *)&TC_Device->DMA2Base);
	writel(0 , (void *)&TC_Device->DMA2Top);
	writel(0 , (void *)&TC_Device->DMA2Write);
	writel(0 , (void *)&TC_Device->DMA2Read);
	writel(0 , (void *)&TC_Device->DMA2Setup);
	writel((1 | (1 << 16)), (void *)&TC_Device->DMA2Holdoff);
	writel(0 , (void *)&TC_Device->DMA2CDAddr);
	writel(0 , (void *)&TC_Device->DMAFlush);
	writel(0 , (void *)&TC_Device->DMA3Base);
	writel(0 , (void *)&TC_Device->DMA3Top);
	writel(0 , (void *)&TC_Device->DMA3Write);
	writel(0 , (void *)&TC_Device->DMA3Read);
	writel(0 , (void *)&TC_Device->DMA3Setup);
	writel((1 | (1 << 16)), (void *)&TC_Device->DMA3Holdoff);
	writel(0 , (void *)&TC_Device->DMA3CDAddr);
	writel(0xf , (void *)&TC_Device->DMAEnable);              /* Enable DMAs */
	/* Initialise IIF Registers */
	writel(0 , (void *)&TC_Device->IIFFIFOEnable);
	writel(1/*Device_p->AlternateOutputLatency*/ , (void *)&TC_Device->IIFAltLatency);
	writel(0/*Device_p->SyncLock*/, (void *)&TC_Device->IIFSyncLock);
	writel(0/*Device_p->SyncDrop*/, (void *)&TC_Device->IIFSyncDrop);
	writel(IIF_SYNC_CONFIG_USE_SOP , (void *)&TC_Device->IIFSyncConfig);
	TagBytes = 6;
	writel(IIFSyncPeriod + TagBytes , (void *)&TC_Device->IIFSyncPeriod);
	writel(1 , (void *)&TC_Device->IIFCAMode);
	//Init Hardware
	// *********************************
	// *********************************************************
	//Init PidSearchEngine (stptiHelper_TCInit_PidSearchEngine)
	for (i = 0; i < TC_Params_p->TC_NumberSlots; i++)
	{
		volatile u16 *Addr_p = (volatile u16 *) TC_Params_p->TC_LookupTableStart;
		PutTCData(&Addr_p[i], TC_INVALID_PID);
	}
	//Init PidSearchEngine (stptiHelper_TCInit_PidSearchEngine)
	// *********************************************************
	// *****************************************
	//Init GlobalInfo (stptiHelper_TCInit_GlobalInfo)
	TCGlobalInfo = (TCGlobalInfo_t *)TC_Params_p->TC_GlobalDataStart;
	/* Set the scratch area so TC can dump DMA0 data (in cdfifo-ish mode). Be paranoid
	 and set the buffer to a DVB packet plus alignment/guard bytes */
	writel((((u32)(DmaFifoFlushingTarget)) + 15) & (~0x0f) , (void *)&TCGlobalInfo->GlobalScratch);
	STSYS_WriteTCReg16LE((void *)&TCGlobalInfo->GlobalSFTimeout, 1429); /* default is 1429 iterations * 7 tc cycles = 10003 tc cycles */
	//Init GlobalInfo (stptiHelper_TCInit_GlobalInfo)
	// *****************************************
	// ******************************************
	//Init MainInfo (stptiHelper_TCInit_MainInfo)
	for (i = 0; i < TC_Params_p->TC_NumberSlots; i++)
	{
		TCMainInfo_t *MainInfo = &((TCMainInfo_t *) TC_Params_p->TC_MainInfoStart)[i];
		STSYS_WriteTCReg16LE((void *)&MainInfo->SlotState, 0);
		STSYS_WriteTCReg16LE((void *)&MainInfo->SlotMode, 0);
		STSYS_WriteTCReg16LE((void *)&MainInfo->DescramblerKeys_p, TC_INVALID_LINK);
		STSYS_WriteTCReg16LE((void *)&MainInfo->DMACtrl_indices, 0xFFFF);
		STSYS_WriteTCReg16LE((void *)&MainInfo->StartCodeIndexing_p, 0);
		STSYS_WriteTCReg16LE((void *)&MainInfo->SectionPesFilter_p, TC_INVALID_LINK);
		STSYS_WriteTCReg16LE((void *)&MainInfo->RemainingPESLength, 0);
		STSYS_WriteTCReg16LE((void *)&MainInfo->PacketCount, 0);
		STSYS_WriteTCReg16LE((void *)&MainInfo->SlotState, 0);
		STSYS_WriteTCReg16LE((void *)&MainInfo->SlotMode, 0);
	}
	//Init MainInfo (stptiHelper_TCInit_MainInfo)
	// ******************************************
	// ******************************************
	//Init SessionInfo (stptiHelper_TCInit_SessionInfo)
	for (session = 0; session < TC_Params_p->TC_NumberOfSessions; session++)
	{
		TCSessionInfo_t *SessionInfo_p = &((TCSessionInfo_t *) TC_Params_p->TC_SessionDataStart)[session];
		STSYS_WriteTCReg16LE((void *)&SessionInfo_p->SessionTSmergerTag, SESSION_NOT_ALLOCATED);
#if defined(A18)
		STSYS_WriteTCReg16LE((void *)&SessionInfo_p->SessionEMMFilterOffset, session * TC_Params_p->TC_SizeOfEMMFilter);
#endif
		/* Set SlotInterrupt in the Interrupt Mask */
		STSYS_SetTCMask16LE((void *)&SessionInfo_p->SessionInterruptMask0, STATUS_FLAGS_PACKET_SIGNAL);
		STSYS_SetTCMask16LE((void *)&SessionInfo_p->SessionInterruptMask0, STATUS_FLAGS_PACKET_SIGNAL_RECORD_BUFFER);
	}
	//Init SessionInfo (stptiHelper_TCInit_SessionInfo)
	// ******************************************
	// ******************************************
	//Init SlotList (stptiHelper_SlotList_Init)
	//fixme: weggelassen, hier holen sie sich speicher fuer die Slots der Sessions; etc
	//->PrivateData
	//Init SlotList (stptiHelper_SlotList_Init)
	// ******************************************
	// ******************************************
	//Init Interrupt DMA (stptiHelper_TCInit_InterruptDMA)
	TCInterruptDMAConfig_p = (TCInterruptDMAConfig_t *) TC_Params_p->TC_InterruptDMAConfigStart;
	/* Adjust the buffer size to make sure it is valid */
	DMABufferSize = stpti_BufferSizeAdjust(sizeof(TCStatus_t) * NO_OF_STATUS_BLOCKS);
	pti->InterruptDMABufferSize = DMABufferSize + DMAScratchAreaSize;
	base_p = (u8 *)dma_alloc_coherent(NULL,
					  pti->InterruptDMABufferSize,
					  (dma_addr_t *) & (pti->InterruptDMABufferInfo),
					  GFP_KERNEL);
	if (base_p == NULL)
	{
		printk("!!!!!!!!!! NO MEMORY !!!!!!!!!!!!!!!\n");
		pti->InterruptBufferStart_p = NULL;
		return;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	__flush_purge_region((void *) base_p, DMABufferSize + DMAScratchAreaSize);
#else
	dma_cache_wback_inv((void *) base_p, DMABufferSize + DMAScratchAreaSize);
#endif
	base_p = stpti_BufferBaseAdjust(base_p);
	pti->InterruptBufferStart_p = base_p;
#if defined (CONFIG_KERNELVERSION) || LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	/* Convert to a physical address for TC. */
	base_p = (u8 *)virt_to_phys(base_p);
#else
	/* Convert to a physical address for TC. */
	base_p = (u8 *)virt_to_bus(base_p);
#endif
	/* Sort out base and top alignment */
	top_p = base_p + DMABufferSize;
	/* set top_p, btm, read_p & write_p ( qwrite_p etc not required ) */
	writel((u32) base_p, (void *)&TCInterruptDMAConfig_p->DMABase_p);
	writel(((u32) top_p - 1) & ~0X0f , (void *)&TCInterruptDMAConfig_p->DMATop_p);
	writel((u32) base_p , (void *)&TCInterruptDMAConfig_p->DMARead_p);
	writel((u32) base_p , (void *)&TCInterruptDMAConfig_p->DMAWrite_p);
	//Init Interrupt DMA (stptiHelper_TCInit_InterruptDMA)
	// ******************************************
	// ******************************************
	//Alloc PrivateData (stptiHelper_TCInit_AllocPrivateData)
	//fixme erstmal wechgelassen (SlotHandle etc werden alloziiert)
	//Alloc PrivateData
	// ******************************************
	// ******************************************
	//TcCam Init (TcCam_Initialize)
	TC_SectionFilterArrays_p = (TCSectionFilterArrays_t *) &TC_Device->TC_SectionFilterArrays;
	//ClearAllCams( TC_SectionFilterArrays_p );
	for (block = 0; block < SF_NUM_BLOCKS_PER_CAM; block++)
	{
		for (index = 0; index < SF_FILTER_LENGTH; index++)
		{
			TC_SectionFilterArrays_p->CamA_Block[block].Index[index].Data.Word = 0xffffffff;
			TC_SectionFilterArrays_p->CamA_Block[block].Index[index].Mask.Word = 0xffffffff;
			TC_SectionFilterArrays_p->CamB_Block[block].Index[index].Data.Word = 0xffffffff;
			TC_SectionFilterArrays_p->CamB_Block[block].Index[index].Mask.Word = 0xffffffff;
		}
	}
	for (index = 0; index < TC_NUMBER_OF_HARDWARE_NOT_FILTERS; index++)
	{
		TC_SectionFilterArrays_p->NotFilter[index] = 0;
	}
	//fixme: auch hier hab ich die ganze interne verwaltung wegoptimiert ;-)
	//TcCam Init (TcCam_Initialize)
	// ******************************************
	// ******************************************
	//Start (stptiHelper_TCInit_Start)
#if 0
	STSYS_WriteRegDev32LE((void *)&TC_Device->TCMode, 0x08);        /* Full Reset the TC  */
	udelay(200 * 64); /* 12800us */             /* Wait */
	STSYS_WriteRegDev32LE((void *)&TC_Device->TCMode, 0x00);        /* Finish Full Reset the TC  */
	udelay(10 * 64); /* 640us */
#endif
	writel(1, (void *)&TC_Device->IIFFIFOEnable);
	writel(2, (void *)&TC_Device->TCMode);
	writel(1, (void *)&TC_Device->TCMode);
}

int __init pti_init(void)
{
	printk("pti loaded (videoMem = %d, waitMS = %d", videoMem, waitMS);
#ifdef UFS910
	printk(", camRouting = %d", camRouting);
#endif
#ifdef CONFIG_PRINTK
	printk(", enableStatistic = %d", enableStatistic);
#else
	printk(", enableSysStatistic = %d", enableSysStatistic);
#endif
	printk(")\n");
	return 0;
}

void __exit pti_exit(void)
{
	printk("pti unloaded\n");
}

module_init(pti_init);
module_exit(pti_exit);

MODULE_DESCRIPTION("PTI driver");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("NO");

#ifdef UFS910
module_param(camRouting, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(camRouting, "ufs910 only! 1=if stream not scrambled do not route it through cimax");
#endif

module_param(videoMem, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(videoMem, "memory for video pid buffer in dvb packets (188 byte). default 2048");

module_param(waitMS, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(waitMS, "waiting time in ms before processing next data (default=20)");

#ifdef CONFIG_PRINTK
module_param(enableStatistic, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(enableStatistic, "enable statistic output on pids arriving pti (default=0=disabled)");
#else
module_param(enableSysStatistic, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(enableSysStatistic, "enable sys statistic output on pids arriving pti (default=1=disabled)");

module_param(pti_last_time, ulong, S_IRUSR | S_IRGRP);
MODULE_PARM_DESC(pti_last_time, "last time pti task called");

module_param(pti_count, ulong, S_IRUSR | S_IRGRP);
MODULE_PARM_DESC(pti_count, "pti package counter");
#endif
