#ifndef __TUNER_DEF_H__
#define __TUNER_DEF_H__

/* TUNER specific Error code base value */
#define TUNER_DRIVER_BASE (TUNER_DRIVER_ID << 16)

typedef enum YWTUNER_DeliverType_e
{
	YWTUNER_DELIVER_SAT = (1 << 0),
	YWTUNER_DELIVER_CAB = (1 << 1),
	YWTUNER_DELIVER_TER = (1 << 2)
} YWTUNER_DeliverType_T;

typedef char TUNER_DeviceName_t[16];

s32 TUNER_Util_PowOf2(s32 number);
s32 TUNER_Util_LongSqrt(s32 Value);
s32 TUNER_Util_BinaryFloatDiv(s32 n1, s32 n2, s32 precision);

BOOL TUNER_WaitTime(u32 Index, s32 TimeOut);
void *TUNER_MemAlloc(s32 size);
void TUNER_MemFree(void *p);

#endif  // __TUNER_DEF_H__
// vim:ts=4
