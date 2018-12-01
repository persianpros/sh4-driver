#ifndef _PTI_BUFFER_H_
#define _PTI_BUFFER_H_

#include "pti_hal.h"

struct tBuffer
{
  struct tBuffer *pNext;
  int dmaIndex;
  int sessionHandle;
  int inUse;
  int slotCount;
  u8* pBuffer;
  u8* pMappedBuffer;
  u8* pAlignedBuffer;
  int bufSize;
  int threshold;
};

int pti_buffer_pool_init(int poolNum, int *pSizes, int *pCount);

struct tBuffer * pti_buffer_get(int size);
void pti_buffer_free(struct tBuffer *pBuffer);

#endif
