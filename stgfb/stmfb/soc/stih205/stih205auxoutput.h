/***********************************************************************
 *
 * File: soc/stih205/stih205auxoutput.h
 * Copyright (c) 2008-2009 STMicroelectronics Limited.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
\***********************************************************************/

#ifndef _STih205AUXOUTPUT_H
#define _STih205AUXOUTPUT_H

#include <HDTVOutFormatter/stmauxtvoutput.h>

class CSTih205Device;

class CSTih205AuxOutput: public CSTmAuxTVOutput
{
public:
  CSTih205AuxOutput(CSTih205Device        *pDev,
                    CSTmSDVTG             *pVTG,
                    CSTmTVOutDENC         *pDENC,
                    CDisplayMixer         *pMixer,
                    CSTmFSynth            *pFSynth,
                    CSTmHDFormatterAWG    *pAWG,
                    CSTmHDFormatterOutput *pHDFOutput,
                    stm_plane_id_t         sharedVideoPlaneID,
                    stm_plane_id_t         sharedGDPPlaneID);

  bool ShowPlane(stm_plane_id_t planeID);

private:
  stm_plane_id_t m_sharedVideoPlaneID;
  stm_plane_id_t m_sharedGDPPlaneID;

  void StartClocks(const stm_mode_line_t*);
  void SetAuxClockToHDFormatter(void);

  void EnableDACs(void);
  void DisableDACs(void);

  void WriteSysCfgReg(ULONG reg, ULONG val) { g_pIOS->WriteRegister(m_pDevReg + ((STih205_SYSCFG_BASE + reg)>>2), val); }
  ULONG ReadSysCfgReg(ULONG reg) { return g_pIOS->ReadRegister(m_pDevReg + ((STih205_SYSCFG_BASE +reg)>>2)); }

  void WriteClkReg(ULONG reg, ULONG val) { g_pIOS->WriteRegister(m_pDevReg + ((STih205_CLKGEN_BASE + reg)>>2), val); }
  ULONG ReadClkReg(ULONG reg) { return g_pIOS->ReadRegister(m_pDevReg + ((STih205_CLKGEN_BASE +reg)>>2)); }

  CSTih205AuxOutput(const CSTih205AuxOutput&);
  CSTih205AuxOutput& operator=(const CSTih205AuxOutput&);
};


#endif //_STih205AUXOUTPUT_H
