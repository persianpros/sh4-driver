/******************************************************
 MxL301RF.h
 ----------------------------------------------------
 Rf IC control functions

 <Revision History>
 '11/10/06 : OKAMOTO Select AGC external or internal.
 ----------------------------------------------------
 Copyright(C) 2011 SHARP CORPORATION
 ******************************************************/

#include "mxl_common.h"

/* Initializes the registers of the chip */
u32 MxL301RF_Init(u8 *pArray, /* a array pointer that store the addr and data pairs for I2C write */
					 u32 *Array_Size, /* a integer pointer that store the number of element in above array */
					 u8 Mode, /* Standard */
					 u32 Xtal_Freq_Hz, /* Crystal Frequency in Hz */
					 u32 IF_Freq_Hz, /* IF Frequency in Hz */
					 u8 Invert_IF, /* Inverted IF Spectrum: 1, or Normal IF: 0 */
					 u8 Clk_Out_Enable, /* Enable Crystal Clock out */
					 u8 Clk_Out_Amp, /* Clock out amplitude: 0 min, 15 max */
					 u8 Xtal_Cap, /* Internal Crystal Capacitance */
					 u8 AGC_Sel, /* AGC Selection */
					 u8 IF_Out /* IF1 or IF2 output path */

					 /* '11/10/06 : OKAMOTO Select AGC external or internal. */
					 , BOOL bInternalAgcEnable
					);
/* Sets registers of the tuner based on RF freq, BW, etc. */
u32 MxL301RF_RFTune(u8 *pArray, u32 *Array_Size, u32 RF_Freq, u8 BWMHz, u8 Mode);
// vim:ts=4

