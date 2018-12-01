/*
 * Authors: Dagobert, Phantomias
 * Buffer Pool Handling for the pti. We allocate two pools of buffer.
 * One pool is for BIG buffersizes and one for SMALL (the amount is configurable).
 *
 * The buffer pools are located in the lim_sys.
 *
 * The Addreesing will be done by the constants PTI_POOL_BIG, PTI_POOL_SMALL.
 * The buffer allocation whill be done in ts packet size (188 Bytes) and is
 * alligned to the STPTI_BUFFER_SIZE_MULTIPLE (0x20).
 */

#include <asm/io.h>

#include <linux/version.h>

#if defined (CONFIG_KERNELVERSION) || LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#include <linux/bpa2.h>
#else
#include <linux/bigphysarea.h>
#include <linux/bpa2.h>
#endif

#include "pti.h"
#include "pti_buffer.h"
#include "pti_main.h"

static struct semaphore sem;
static struct tBuffer *bufferArray;
static int poolNumber;

struct TCDMAConfigExt_s *TCDMAConfigExt_t;

typedef struct
{
  int bufSize;
  int freeBuffers;
  struct tBuffer *pList;
} tPool;

static tPool *poolArray;

static size_t adjustSize( size_t size )
{
    return (( size + STPTI_BUFFER_SIZE_MULTIPLE - 1 ) &
                                       ( ~( STPTI_BUFFER_SIZE_MULTIPLE - 1 )));
}

static void * alignAddress( void *pAddress )
{
  return ( void * ) ((( u32 ) pAddress + STPTI_BUFFER_ALIGN_MULTIPLE - 1 ) &
				   ( ~( STPTI_BUFFER_ALIGN_MULTIPLE - 1 )));
}

// the function pushes the buffer on stack
static void freeBuffer(tPool *pPool, struct tBuffer *pBuffer)
{
  if(pBuffer->inUse == 0)
    printk("%s: FREEING UNUSED BUFFER\n", __func__);

  if(down_interruptible (&sem))
  {
    printk("%s(): error taking semaphore\n", __func__);
  }

  pBuffer->inUse = 0;
  pPool->freeBuffers++;

  /* attach the buffer at the list head */
  pBuffer->pNext = pPool->pList;
  pPool->pList = pBuffer;

  printk("%s(): free = %d\n", __func__, pPool->freeBuffers);

  up(&sem);
}

// the function pops a buffer from stack
static struct tBuffer * getBuffer(tPool *pPool)
{
  struct tBuffer *pBuffer;

  if(down_interruptible (&sem))
  {
    printk("%s(): error taking semaphore\n", __func__);
  }

  printk("%s(): free = %d\n", __func__, pPool->freeBuffers);

  /* get the pointer after locking the semaphore */
  pBuffer = pPool->pList;

  if(pPool->pList != NULL)
  {
    // update the buffer and the pool
    if(pBuffer->inUse != 0)
      printk("%s: ALLOCATING USED BUFFER\n", __func__);

    pBuffer->inUse = 1;
    pPool->freeBuffers--;

    pPool->pList = pBuffer->pNext;

    /* invalidate the chain */
    pBuffer->pNext = NULL;
  }

  up(&sem);

  return pBuffer;
}

static void pti_init_dma(int tc_dma_index, u32 BufferPhys_p, int ActualSize)
{
    TCDMAConfig_t * DMAConfig_p = &((TCDMAConfig_t *)tc_params.TC_DMAConfigStart)[tc_dma_index];

    u32 base = (u32)(BufferPhys_p);
    u32 top  = (u32) (base + ActualSize);

    printk("DMA(%d) base = %x, top = %x, buffer = %x, size %d\n", tc_dma_index, base, top, BufferPhys_p, ActualSize);

    TCDMAConfigExt_t[tc_dma_index].BasePtr_physical = base;
    TCDMAConfigExt_t[tc_dma_index].TopPtr_physical = (top - 1) & ~0xf;
    TCDMAConfigExt_t[tc_dma_index].pBuf = ( u8 * ) phys_to_virt ( TCDMAConfigExt_t[tc_dma_index].BasePtr_physical );
    TCDMAConfigExt_t[tc_dma_index].bufSize = ( TCDMAConfigExt_t[tc_dma_index].TopPtr_physical - TCDMAConfigExt_t[tc_dma_index].BasePtr_physical ) + 0x10;
    TCDMAConfigExt_t[tc_dma_index].bufSize_sub_188 = TCDMAConfigExt_t[tc_dma_index].bufSize - 188;
    TCDMAConfigExt_t[tc_dma_index].bufSize_div_188 = TCDMAConfigExt_t[tc_dma_index].bufSize / 188;
    TCDMAConfigExt_t[tc_dma_index].bufSize_div_188_div_2 = TCDMAConfigExt_t[tc_dma_index].bufSize / 188 / 2;

    writel(base, (void*)&DMAConfig_p->DMABase_p );
    writel((top - 1) & ~0xf , (void*)&DMAConfig_p->DMATop_p);
    writel(base, (void*)&DMAConfig_p->DMARead_p );
    writel(base, (void*)&DMAConfig_p->DMAWrite_p );
    writel(base, (void*)&DMAConfig_p->DMAQWrite_p );
    writel(0 , (void*)&DMAConfig_p->BufferPacketCount);

    /* Reset SignalModeFlags as this could have been a previously used DMA and the flags may be in an
     * un-defined state
     */
    STSYS_ClearTCMask16LE((void*)&DMAConfig_p->SignalModeFlags, TC_DMA_CONFIG_SIGNAL_MODE_TYPE_MASK );

    //TC_DMA_CONFIG_SIGNAL_MODE_TYPE_EVERY_TS, TC_DMA_CONFIG_SIGNAL_MODE_SWCDFIFO TC_DMA_CONFIG_SIGNAL_MODE_TYPE_QUANTISATION...
    STSYS_SetTCMask16LE((void*)&DMAConfig_p->SignalModeFlags,
                        TC_DMA_CONFIG_SIGNAL_MODE_TYPE_EVERY_TS | 
                        TC_DMA_CONFIG_OUTPUT_WITHOUT_META_DATA | 
                        TC_DMA_CONFIG_WINDBACK_ON_ERROR);

    STSYS_WriteTCReg16LE((void*)&DMAConfig_p->Threshold, TC_DMA_THRESHOLD_LOW);

    // disable signalling
    STSYS_SetTCMask16LE((void*)&DMAConfig_p->SignalModeFlags, TC_DMA_CONFIG_SIGNAL_MODE_FLAGS_SIGNAL_DISABLE );

}

int pti_buffer_pool_init(int poolNum, int *pSizes, int *pCount)
{
  int i;
  int bufSize;
  u8 *pBuffer;
  int pages;
  int poolInd;
  int totalBuffers = 0;
  int offset = 0;

  /* init the access semaphore */
  sema_init(&sem, 1);

  poolNumber = poolNum;

  for(poolInd = 0; poolInd < poolNum; poolInd++)
    totalBuffers += pCount[poolInd];

  // allocate memory for all buffer control structures
  // (avoids memory fragmentation)
  bufferArray = kmalloc(sizeof(struct tBuffer) * totalBuffers, GFP_KERNEL);
  memset(bufferArray, 0, sizeof(struct tBuffer) * totalBuffers);

  // allocate memory for all pool control structures
  // (avoids memory fragmentation)
  poolArray = kmalloc(sizeof(tPool) * poolNum, GFP_KERNEL);
  memset(poolArray, 0, sizeof(tPool) * poolNum);

  // allocate memory for all Dma extra Bytes for speed up pti_task
  TCDMAConfigExt_t = kmalloc(sizeof(struct TCDMAConfigExt_s) * totalBuffers, GFP_KERNEL);
  memset(TCDMAConfigExt_t, 0, sizeof(struct TCDMAConfigExt_s) * totalBuffers);

  for(poolInd = 0; poolInd < poolNum; poolInd++)
  {
    // init pool data
    poolArray[poolInd].freeBuffers = 0;
    poolArray[poolInd].bufSize = pSizes[poolInd];
    poolArray[poolInd].pList = NULL;

    // compute the buffer size including overhead and the corresponding
    // number of pages
    bufSize = adjustSize(pSizes[poolInd]);
    pages = bufSize / PAGE_SIZE;
    if (bufSize % PAGE_SIZE)
      pages++;

    // initialize buffer control structures for this pool
    for(i = 0; i < pCount[poolInd]; i++)
    {
      int j = i + offset;

      // allocate big buffers at once
      pBuffer = bigphysarea_alloc_pages(pages, 0, GFP_KERNEL | __GFP_DMA);

      printk("pti: %s: allocate %d pages (%lu bytes)\n", __func__, pages, pages * PAGE_SIZE);

      if(pBuffer == NULL)
      {
        printk("%s(): failed to allocate buffer (%d, %d)\n",
                __func__, bufSize, pages);
        break;
      }

      // store the requested buffer size
      bufferArray[j].bufSize = pSizes[poolInd];

      // partition the big buffer area
      bufferArray[j].pBuffer = pBuffer;
#if defined (CONFIG_KERNELVERSION) || LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
      bufferArray[j].pAlignedBuffer = alignAddress((void*)virt_to_phys(pBuffer));
#else
      bufferArray[j].pAlignedBuffer = alignAddress((void*)virt_to_bus(pBuffer));
#endif
      bufferArray[j].pMappedBuffer = ioremap_nocache((int)bufferArray[j].pAlignedBuffer, bufSize);
      /* set the inUse flag before frreing the buffer */
      bufferArray[j].inUse = 1;

      // initialize DMA
      bufferArray[j].dmaIndex = j;
      bufferArray[j].threshold = TC_DMA_THRESHOLD_LOW;
      pti_init_dma(j, (u32) bufferArray[j].pAlignedBuffer, bufSize);

      // put into the pool
      printk("%s: put in pool dmaIndex %d\n", __func__, j);
      freeBuffer(&poolArray[poolInd], &bufferArray[j]);
    }

    // increment the offset within the buffer array
    offset += i;
  }

  return 0;
}

struct tBuffer * pti_buffer_get(int size)
{
  int i;
  struct tBuffer *pBuffer = NULL;

  for(i = 0; i < poolNumber; i++)
  {
    if(poolArray != NULL)
    {
      // check the buffer size
      if(poolArray[i].bufSize == size)
      {
        pBuffer = getBuffer(&poolArray[i]);
        break;
      }
    }
  }

  printk("%s(%d) => %p\n", __func__, size, pBuffer);
  if(pBuffer == NULL)
  {
    printk("%s(): NO FREE BUFFER (%d)\n", __func__, size);
  }
  else
  {
    TCDMAConfig_t * DMAConfig_p = &((TCDMAConfig_t *)tc_params.TC_DMAConfigStart)[pBuffer->dmaIndex];

    printk("%s: dmaIndex %d\n", __func__, pBuffer->dmaIndex);

    u32 base = (u32)pBuffer->pAlignedBuffer;
    u32 top  = (u32) (base + pBuffer->bufSize);

    if((readl((void*)&DMAConfig_p->DMAWrite_p) != base) ||
       (readl((void*)&DMAConfig_p->BufferPacketCount) != 0))
    {
      printk("\n%s(): inconsistent DMA settings %d\n\n", __func__, pBuffer->dmaIndex);
    }

    TCDMAConfigExt_t[pBuffer->dmaIndex].BasePtr_physical = base;
    TCDMAConfigExt_t[pBuffer->dmaIndex].TopPtr_physical = (top - 1) & ~0xf;
    TCDMAConfigExt_t[pBuffer->dmaIndex].pBuf = ( u8 * ) phys_to_virt ( TCDMAConfigExt_t[pBuffer->dmaIndex].BasePtr_physical );
    TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize = ( TCDMAConfigExt_t[pBuffer->dmaIndex].TopPtr_physical - TCDMAConfigExt_t[pBuffer->dmaIndex].BasePtr_physical ) + 0x10;
    TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize_sub_188 = TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize - 188;
    TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize_div_188 = TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize / 188;
    TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize_div_188_div_2 = TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize / 188 / 2;

    writel(base, (void*)&DMAConfig_p->DMABase_p );
    writel((top - 1) & ~0xf , (void*)&DMAConfig_p->DMATop_p);
    writel(base, (void*)&DMAConfig_p->DMARead_p );
    writel(base, (void*)&DMAConfig_p->DMAWrite_p );
    writel(base, (void*)&DMAConfig_p->DMAQWrite_p );
    writel(0 , (void*)&DMAConfig_p->BufferPacketCount);

    /* Reset SignalModeFlags as this could have been a previously used DMA and the flags may be in an
     * un-defined state
     */
    STSYS_ClearTCMask16LE((void*)&DMAConfig_p->SignalModeFlags, TC_DMA_CONFIG_SIGNAL_MODE_TYPE_MASK );

    //TC_DMA_CONFIG_SIGNAL_MODE_TYPE_EVERY_TS, TC_DMA_CONFIG_SIGNAL_MODE_SWCDFIFO TC_DMA_CONFIG_SIGNAL_MODE_TYPE_QUANTISATION...
    STSYS_SetTCMask16LE((void*)&DMAConfig_p->SignalModeFlags,
                        TC_DMA_CONFIG_SIGNAL_MODE_TYPE_EVERY_TS |
                        TC_DMA_CONFIG_OUTPUT_WITHOUT_META_DATA |
                        TC_DMA_CONFIG_WINDBACK_ON_ERROR);

    STSYS_WriteTCReg16LE((void*)&DMAConfig_p->Threshold, TC_DMA_THRESHOLD_LOW);

    // disable signalling
    STSYS_SetTCMask16LE((void*)&DMAConfig_p->SignalModeFlags, TC_DMA_CONFIG_SIGNAL_MODE_FLAGS_SIGNAL_DISABLE );
  }

  return pBuffer;
}

void pti_buffer_free(struct tBuffer *pBuffer)
{
  int i;
  TCDMAConfig_t * DMAConfig_p = &((TCDMAConfig_t *)tc_params.TC_DMAConfigStart)[pBuffer->dmaIndex];

#if defined (CONFIG_KERNELVERSION) /* ST Linux 2.3 */
  printk("%s(%p, %d) - delta 0x%8x, thr 0x%x\n", __func__, pBuffer, pBuffer->bufSize,
         readl((void*)&DMAConfig_p->DMAWrite_p ) - readl((void*)&DMAConfig_p->DMABase_p ), readw((void*)&DMAConfig_p->Threshold) );
#else
  printk("%s(%p, %d) - delta 0x%8lx, thr 0x%lx\n", __func__, pBuffer, pBuffer->bufSize,
         readl((void*)&DMAConfig_p->DMAWrite_p ) - readl((void*)&DMAConfig_p->DMABase_p ), readw((void*)&DMAConfig_p->Threshold) );
#endif

  for(i = 0; i < poolNumber; i++)
  {
    if(poolArray[i].bufSize == pBuffer->bufSize)
    {
      TCDMAConfig_t * DMAConfig_p = &((TCDMAConfig_t *)tc_params.TC_DMAConfigStart)[pBuffer->dmaIndex];

      u32 base = (u32)pBuffer->pAlignedBuffer;
      u32 top  = (u32) (base + pBuffer->bufSize);

      STSYS_WriteTCReg16LE((void*)&DMAConfig_p->SignalModeFlags, TC_DMA_CONFIG_SIGNAL_MODE_FLAGS_SIGNAL_DISABLE);

      TCDMAConfigExt_t[pBuffer->dmaIndex].BasePtr_physical = base;
      TCDMAConfigExt_t[pBuffer->dmaIndex].TopPtr_physical = (top - 1) & ~0xf;
      TCDMAConfigExt_t[pBuffer->dmaIndex].pBuf = ( u8 * ) phys_to_virt ( TCDMAConfigExt_t[pBuffer->dmaIndex].BasePtr_physical );
      TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize = ( TCDMAConfigExt_t[pBuffer->dmaIndex].TopPtr_physical - TCDMAConfigExt_t[pBuffer->dmaIndex].BasePtr_physical ) + 0x10;
      TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize_sub_188 = TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize - 188;
      TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize_div_188 = TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize / 188;
      TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize_div_188_div_2 = TCDMAConfigExt_t[pBuffer->dmaIndex].bufSize / 188 / 2;

      writel(base, (void*)&DMAConfig_p->DMABase_p );
      writel((top - 1) & ~0xf , (void*)&DMAConfig_p->DMATop_p);
      writel(base, (void*)&DMAConfig_p->DMARead_p );
      writel(base, (void*)&DMAConfig_p->DMAWrite_p );
      writel(base, (void*)&DMAConfig_p->DMAQWrite_p );
      writel(0 , (void*)&DMAConfig_p->BufferPacketCount);

      /* Reset SignalModeFlags as this could have been a previously used DMA and the flags may be in an
       * un-defined state
       */


      STSYS_WriteTCReg16LE((void*)&DMAConfig_p->Threshold, TC_DMA_THRESHOLD_LOW);

      freeBuffer(&poolArray[i], pBuffer);

      break;
    }
  }

  if(i == poolNumber)
    printk("%s(): NO POOL FOUND (%p, %d)\n", __func__, pBuffer, pBuffer->bufSize);
}

