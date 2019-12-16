/***********************************************************************
 *
 * File: soc/stih205/stih205dei.h
 * Copyright (c) 2008 STMicroelectronics Limited.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
\***********************************************************************/
#ifndef _STIh205_DEI_H
#define _STIh205_DEI_H

#include <Gamma/DEIVideoPipeV2.h>
#include <Gamma/DEIReg.h>

class CSTih205DEI: public CDEIVideoPipeV2
{
public:
  CSTih205DEI (stm_plane_id_t   GDPid,
               CGammaVideoPlug *plug,
               ULONG            baseAddr): CDEIVideoPipeV2 (GDPid,
                                                            plug,
                                                            baseAddr)
  {
    /*
     * motion data allocated from system partition, there is only one LMI on
     * stih205.
     */
    m_ulMotionDataFlag = SDAAF_NONE;

    /*
     * (mostly) validation recommended T3I plug settings.
     */
    m_nMBChunkSize          = 0x3; // Validation actually recommend 0xf, this is better for extreme downscales
    m_nRasterOpcodeSize     = 32;
    m_nRasterChunkSize      = 0x7;
    m_nPlanarChunkSize      = 0x3;// Validation do not test this mode so do not have a recommendation

    m_nMotionOpcodeSize     = 32;
    WriteVideoReg (DEI_T3I_MOTION_CTL, (DEI_T3I_CTL_OPCODE_SIZE_32 |
                                        (0x3 << DEI_T3I_CTL_CHUNK_SIZE_SHIFT)));
  }

private:
  CSTih205DEI (const CSTih205DEI &);
  CSTih205DEI& operator= (const CSTih205DEI &);
};

#endif // _STIh205_DEI_H
