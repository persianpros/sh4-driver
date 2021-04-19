/*****************************************************************************/
/* COPYRIGHT (C) 2009 STMicroelectronics - All Rights Reserved               */
/* ST makes no warranty express or implied including but not limited to,     */
/* any warranty of                                                           */
/*                                                                           */
/*   (i)  merchantability or fitness for a particular purpose and/or         */
/*   (ii) requirements, for a particular purpose in relation to the LICENSED */
/*        MATERIALS, which is provided "AS IS", WITH ALL FAULTS. ST does not */
/*        represent or warrant that the LICENSED MATERIALS provided here     */
/*        under is free of infringement of any third party patents,          */
/*        copyrights,trade secrets or other intellectual property rights.    */
/*        ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE       */
/*        EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                    */
/*                                                                           */
/*****************************************************************************/
/**
 @File   chip_0367cab.h
 @brief

*/
#ifndef _CHIP_0367CAB_H
#define _CHIP_0367CAB_H

#include "ioarch.h"
#include "ioreg.h"

#ifdef __cplusplus
extern "C"
{
#endif  // __cplusplus

u32 ChipUpdateDefaultValues_0367cab(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u16 RegAddr, u8 Data, s32 reg);
u32 ChipSetOneRegister_0367cab(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u16 RegAddr, u8 Data);
u32 ChipSetField_0367cab(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FieldId, int Value);
s32 ChipGetFieldImage_0367cab(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FieldId);
u32 ChipSetFieldImage_0367cab(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FieldId, s32 Value);
int ChipGetOneRegister_0367cab(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u16 RegAddr);
u8 ChipGetField_0367cab(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, u32 FieldId);
u32 ChipGetRegisters_0367cab(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, int FirstRegAddr, int NbRegs);  //, u8 *RegsVal)
u32 ChipSetRegisters_0367cab(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, int FirstRegAddr, int NbRegs);
void ChipWaitOrAbort_0367cab(BOOL bForceSearchTerm, u32 delay_ms);//lwj

#ifdef __cplusplus
}
#endif   // __cplusplus

#endif  // _CHIP_0367CAB_H
// vim:ts=4
