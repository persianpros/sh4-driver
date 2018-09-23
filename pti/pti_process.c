/*
 * Data processing function.
 *
 */

#include <linux/init.h>
#include <linux/wait.h>
#include <linux/module.h>

#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif

#include <asm/io.h>
#include <asm/cacheflush.h>

#include "dmxdev.h"
#include "dvb_demux.h"
#include <linux/dvb/dmx.h>

#include "pti.h"
#include "pti_main.h"
#include "pti_hal.h"
#include "ts_makros.h"

#define U32 u32
#define U16 u16
#define U8 u8

#ifdef UFS910
extern int camRouting;
#endif
extern int waitMS;
#ifdef CONFIG_PRINTK
extern int enableStatistic;
#else
extern int enableSysStatistic;
#endif
extern int max_pti_dma;
extern struct TCDMAConfigExt_s *TCDMAConfigExt_t;

static struct stpti *pti = NULL;
extern void (*demultiplexDvbPackets)(struct dvb_demux *demux, const u8 *buf, int count);

void setDmaThreshold(int a, int b)
{
}

#ifdef CONFIG_PRINTK
int pktCount;
int prevPktCount;
int maxDelta;
int minDelta = 999999999;
int loopCount = 0;
int lastPktCount;
#endif

static wait_queue_head_t bufferHalfFull;

void paceSwtsByPti(void)
{
	if (wait_event_interruptible(bufferHalfFull, 1))
	{
		printk("wait_event_interruptible failed\n");
		return;
	}
}

EXPORT_SYMBOL(paceSwtsByPti);

#ifdef CONFIG_PRINTK
struct pidStatistic_s
{
	int number;
	unsigned long time;

	unsigned long complete;
	unsigned int rate;

	unsigned int min_rate;
	unsigned int max_rate;

	u8 lastCC;
	u8 ccError;
};

struct pidStatistic_s pidS[0xffff];

void ptiStatistic(int num, u8 *pBuf)
{
	int i;
	u8 cc;
	/* walk over the packets, count each pid and remeber the current time */
	for (i = 0; i < num; i++)
	{
		u16 pid = ts_pid(pBuf + (i * 188));
		pidS[pid].number++;
		pidS[pid].complete++;
		if ((pidS[pid].time == 0) || (time_after(jiffies, pidS[pid].time + HZ /* 1 sekunde vorbei ? */)))
		{
			pidS[pid].time = jiffies;
			pidS[pid].rate = pidS[pid].number;
			pidS[pid].number = 0;
			if ((pidS[pid].rate > pidS[pid].max_rate) || (pidS[pid].max_rate == 0))
				pidS[pid].max_rate = pidS[pid].rate;
			if ((pidS[pid].rate < pidS[pid].min_rate) || (pidS[pid].min_rate == 0))
				pidS[pid].min_rate = pidS[pid].rate;
		}
		cc = ts_cc(pBuf + (i * 188));
		if (pidS[pid].lastCC != 255)
		{
			if ((pidS[pid].lastCC + 1) % 0x10 != cc)
			{
				pidS[pid].ccError++;
				printk("cc error last %d exp %d got %d\n", pidS[pid].lastCC, (pidS[pid].lastCC + 1) % 0x10, cc);
			}
		}
		pidS[pid].lastCC = cc;
	}
}

void ptiStatisticClean(void)
{
	int i;
	for (i = 0; i < 0xffff; i++)
	{
		if ((time_after(jiffies, pidS[i].time + (HZ * 5) /* 5 sekunde vorbei ? */)))
		{
			pidS[i].rate = 0;
			pidS[i].min_rate = 0;
			pidS[i].max_rate = 0;
			pidS[i].complete = 0;
			pidS[i].number = 0;
			pidS[i].time = 0;
			pidS[i].lastCC = 255;
			pidS[i].ccError = 0;
		}
	}
}

void ptiStatisticOut(void)
{
	int i;
	for (i = 0; i < 0xffff; i++)
	{
		if (pidS[i].rate != 0)
			printk("pidstatistic: pid 0x%x, rate = %d packets/s (complete %lu) %d Mbps/s, min %d/s - max %d/s, CC error %d\n", i,
			       pidS[i].rate, pidS[i].complete, (pidS[i].rate * 188 * 8) / (1024 * 1024), pidS[i].min_rate, pidS[i].max_rate, pidS[i].ccError);
	}
}
#else
extern unsigned long pti_last_time;
extern unsigned long pti_count;

void ptiStatistic(int num)
{
	pti_last_time = jiffies;
	pti_count += num;
}
#endif

inline static void processDmaChannel(int tc_dma_index,
				     U32 ReadPtr_physical,
				     U32 WritePtr_physical,
				     TCDMAConfig_t *DMAConfig_p)
{
	struct dvb_demux *demux = pti_hal_get_demux_from_dma_index(tc_dma_index);
	/* calculate the write index in the buffer */
	U32 WriteInd = (WritePtr_physical - TCDMAConfigExt_t[tc_dma_index].BasePtr_physical);
	/* calculate the read index in the buffer */
	U32 ReadInd = (ReadPtr_physical - TCDMAConfigExt_t[tc_dma_index].BasePtr_physical);
	/* calculate the read pointer */
	U8 *pBuf_add_ReadInd = (TCDMAConfigExt_t[tc_dma_index].pBuf + ReadInd);
	/* decrease the index by one packet because it might be rolled back or
	 * be incomplete

	 * ***: laut meinen debugs sind nie unvollstaendige Pakete gekommen.
	 *      Ausserdem wuerde dem demuxer nicht vollstaendige Pakete uebergeben werden,
	 *      was dieser dann verwirft, daher auf 188 ausrichten.
	 */
	if (WriteInd >= 188)
		WriteInd -= 188; // normal case
	else
		WriteInd = TCDMAConfigExt_t[tc_dma_index].bufSize_sub_188; //after a wraparound (e.g. WriteInd = 0)
	// align to 188
	WriteInd = (WriteInd / 188) * 188;
	/* ist passiert als ich speicher ueberschrieben hatte, kann nicht schaden
	 * denke ich; update: auch danach ist es aufgetreten; interrupt failed?

	 * ***: ich denke mit diese Abfrage laesst sich kein Bufferueberlauf,
	 *      in einen Ringbuffer feststellen.
	 */
	if (WriteInd > TCDMAConfigExt_t[tc_dma_index].bufSize)
	{
		printk("pti_task: (%x, %x, %x)\n", WriteInd, TCDMAConfigExt_t[tc_dma_index].TopPtr_physical, TCDMAConfigExt_t[tc_dma_index].BasePtr_physical);
		/* clear buffer and bye bye */
		writel(readl((void *)&DMAConfig_p->DMAWrite_p), (void *)&DMAConfig_p->DMARead_p);
		return;
	}
#ifdef CONFIG_PRINTK
	prevPktCount = pktCount;
#endif
	/* validate pointers, even if they are invalid we want to process
	   status blocks */
	if ((TCDMAConfigExt_t[tc_dma_index].pBuf != NULL) && (demux != NULL))
	{
		struct DeviceContext_s *pContext = (struct DeviceContext_s *)demux->priv;
		int num = 0;
		if (WriteInd < ReadInd)
		{
			int bytenum = (TCDMAConfigExt_t[tc_dma_index].bufSize - ReadInd);
			int num1 = bytenum / 188;
			int num2 = WriteInd / 188;
			num = num1 + num2;
#ifdef CONFIG_PRINTK
			pktCount += num;
#endif
			/* zur erklaerung dieser sache. ich hatte eine zeitlang einen ueberschriebenen
			 * speicher und somit konnte dies auftreten. ich denke solche abfragen koennen
			 * nicht schaden von daher hab ich es drinne gelassen.

			 * ***: ich denke mit diese Abfrage laesst sich kein Bufferueberlauf,
			 *      in einen Ringbuffer feststellen.
			 */
			if (num > TCDMAConfigExt_t[tc_dma_index].bufSize_div_188)
			{
				printk("pti_task: (%x, %x, %d, %x, %x, %x)\n", TCDMAConfigExt_t[tc_dma_index].bufSize, ReadInd, num, ReadPtr_physical,
				       TCDMAConfigExt_t[tc_dma_index].BasePtr_physical, TCDMAConfigExt_t[tc_dma_index].TopPtr_physical);
			}
			else if (demultiplexDvbPackets != NULL)
			{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
//				__flush_invalidate_region(pBuf_add_ReadInd, bytenum);
				invalidate_ioremap_region(0, pBuf_add_ReadInd, 0, bytenum);
#else
				dma_cache_inv(pBuf_add_ReadInd, bytenum);
#endif
				demultiplexDvbPackets(demux, pBuf_add_ReadInd, num1); // process packets before wraparound
#ifdef CONFIG_PRINTK
				if (enableStatistic)
					ptiStatistic(num1, pBuf_add_ReadInd);
#else
				if (enableSysStatistic)
					ptiStatistic(num1);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
//				__flush_invalidate_region(TCDMAConfigExt_t[tc_dma_index].pBuf, WriteInd);
				invalidate_ioremap_region(0, TCDMAConfigExt_t[tc_dma_index].pBuf, 0, WriteInd);
#else
				dma_cache_inv(TCDMAConfigExt_t[tc_dma_index].pBuf, WriteInd);
#endif
				demultiplexDvbPackets(demux, TCDMAConfigExt_t[tc_dma_index].pBuf, num2); // process packets after wraparound
#ifdef CONFIG_PRINTK
				if (enableStatistic)
					ptiStatistic(num2, TCDMAConfigExt_t[tc_dma_index].pBuf);
#else
				if (enableSysStatistic)
					ptiStatistic(num2);
#endif
			}
			//printk("+");
		}
		else
		{
			int bytenum = (WriteInd - ReadInd);
			num = bytenum / 188;
#ifdef CONFIG_PRINTK
			pktCount += num;
#endif
			/* ***: ich denke mit diese Abfrage laesst sich kein Bufferueberlauf,
			 *      in einen Ringbuffer feststellen.
			 */
			if (num > TCDMAConfigExt_t[tc_dma_index].bufSize_div_188)
			{
				printk("pti_task: (%x, %x, %x, %d)\n", WriteInd, ReadInd, TCDMAConfigExt_t[tc_dma_index].BasePtr_physical, tc_dma_index);
			}
			else if (demultiplexDvbPackets != NULL)
			{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
//				__flush_invalidate_region(pBuf_add_ReadInd, bytenum);
				invalidate_ioremap_region(0, pBuf_add_ReadInd, 0, bytenum);
#else
				dma_cache_inv(pBuf_add_ReadInd, bytenum);
#endif
				demultiplexDvbPackets(demux, pBuf_add_ReadInd, num);
#ifdef CONFIG_PRINTK
				if (enableStatistic)
					ptiStatistic(num, pBuf_add_ReadInd);
#else
				if (enableSysStatistic)
					ptiStatistic(num);
#endif
			}
			//printk(".");
		}
		/* if the input is the DVR device the data is played
		   back via SWTS to enable passing the playback data
		   though the hardware descrambler */
		if (pContext->pPtiSession->source == DMX_SOURCE_DVR0)
		{
			if (num < TCDMAConfigExt_t[tc_dma_index].bufSize_div_188_div_2)
			{
				/* Buffer is less than half-empty, enable the semaphore.
				   Enforce binary semaphore behavior, otherwise the caller
				   wouldn't block when necessary. */
				wake_up_interruptible(&bufferHalfFull);
			}
			//printk("pti_task: %d\n", num);
		}
	}
	//printk("pti_task: RI %d, BA %x\n", ReadInd, DMAConfig_p->BasePtr_physical);
#ifdef CONFIG_PRINTK
	if ((pktCount - prevPktCount) > maxDelta)
	{
		maxDelta = pktCount - prevPktCount;
	}
	if ((pktCount - prevPktCount) < minDelta)
	{
		minDelta = pktCount - prevPktCount;
	}
	loopCount++;
	if (!(loopCount % 50) && (enableStatistic))
	{
		printk("statistic: pktCount %d last packets %d minDelta %d maxDelta %d\n", pktCount, pktCount - lastPktCount, minDelta, maxDelta);
		ptiStatisticOut();
		ptiStatisticClean();
		lastPktCount = pktCount;
		minDelta = 999999999;
		maxDelta = 0;
	}
#endif
	/*
	 * acknowledge all packets in the block

	 * ***: wird das abschneiden und ausrichten des WriteInd deaktiviert,
	 *      so kann WritePtr_physical verwendet werden.
	 */
	//writel ( WritePtr_physical, ( void * ) &DMAConfig_p->DMARead_p );
	writel(TCDMAConfigExt_t[tc_dma_index].BasePtr_physical + WriteInd, (void *) &DMAConfig_p->DMARead_p);
}

#define SysConfigBaseAddress    0x19001000
#define SYS_CFG0                0x100
#define SYS_CFG1                0x104

#define TSMergerBaseAddress     0x19242000
#define TSM_1394_CFG            0x0810
#define TSM_PTI_SEL             0x0200
#define TSM_1394_DEST           0x0210

unsigned long reg_sys_config = 0;
unsigned long reg_tsm_config = 0;

int pti_task(void *data)
{
	//STPTI_TCParameters_t *TC_Params_p = &tc_params;
	TCDMAConfig_t *DMAConfig_p;
	int vLoopDMAs;
	U32 WritePtr_physical;
	U32 ReadPtr_physical;
	int time_to_wait = msecs_to_jiffies(waitMS);
#ifdef CONFIG_PRINTK
	int count = 0;
#endif
	pti = (struct stpti *) data;
	daemonize("pti_task");
	//set highest prio
	set_user_nice(current, -20);
	printk("pti_task: using %d dma channel\n", max_pti_dma);
	init_waitqueue_head(&pti->queue);
	init_waitqueue_head(&bufferHalfFull);
	while (1)
	{
		wait_event_timeout(pti->queue, 0, time_to_wait);
#ifndef CONFIG_PRINTK
		//update pti time
		if (enableSysStatistic)
			ptiStatistic(0);
#endif
#ifdef CONFIG_PRINTK
		count++;
		if (count > 500)
			{
			printk("\n");
			count = 0;
			pti_hal_output_slot_state();
		}
#endif
		/* pti_hal_output_slot_state(); */
		/* Dagobert: configure the stream routing in accordance to
		 * the scrambled state. route stream only through cimax
		 * if screem is scrambled, otherwise directly to pti
		 */
#ifdef UFS910
		if (camRouting == 1)
		{
			int state;
			if ((state = pti_hal_get_scrambled()) != -1)
			{
				u32 reg;
				if (reg_sys_config == 0)
				{
					reg_sys_config = (unsigned long) ioremap(SysConfigBaseAddress, 0x0900);
					reg_tsm_config = (unsigned long) ioremap(TSMergerBaseAddress, 0x0900);
				}
				reg = ctrl_inl(reg_tsm_config + TSM_1394_CFG);
				if (state == 1)
				{
					ctrl_outl(0x6, reg_sys_config + SYS_CFG0);
					ctrl_outl(reg | 0x20000 , reg_tsm_config + TSM_1394_CFG);
					ctrl_outl(0xe , reg_tsm_config + TSM_PTI_SEL);
					ctrl_outl(0x1 , reg_tsm_config + TSM_1394_DEST);
				}
				else
				{
					ctrl_outl(0x2, reg_sys_config + SYS_CFG0);
					ctrl_outl(reg & ~ 0x20000 , reg_tsm_config + TSM_1394_CFG);
					ctrl_outl(0xf , reg_tsm_config + TSM_PTI_SEL);
					ctrl_outl(0x0 , reg_tsm_config + TSM_1394_DEST);
				}
			}
		}
#endif
		for (vLoopDMAs = 0; vLoopDMAs < /* TC_Params_p->TC_NumberDMAs*/ max_pti_dma; vLoopDMAs++)
		{
			DMAConfig_p = &((TCDMAConfig_t *)tc_params.TC_DMAConfigStart)[vLoopDMAs];
			ReadPtr_physical = readl((void *) &DMAConfig_p->DMARead_p);
			WritePtr_physical = readl((void *) &DMAConfig_p->DMAWrite_p);
			if (ReadPtr_physical != WritePtr_physical)
			{
				processDmaChannel(vLoopDMAs, ReadPtr_physical, WritePtr_physical, DMAConfig_p);
			}
		}
	} /* while true */
	printk("######## PTI task terminated\n");
	// TODO: stop dma
}

