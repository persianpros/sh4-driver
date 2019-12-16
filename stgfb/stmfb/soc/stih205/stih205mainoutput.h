/***********************************************************************
 *
 * File: soc/stih205/stih205mainoutput.h
 * Copyright (c) 2008-2009 STMicroelectronics Limited.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
\***********************************************************************/

#ifndef _STih205MAINOUTPUT_H
#define _STih205MAINOUTPUT_H

#include <HDTVOutFormatter/stmhdfoutput.h>

class CDisplayDevice;
class CSTmFSynth;
class CSTmSDVTG;
class CSTmTVOutDENC;
class CSTmHDFormatterAWG;

class CSTih205MainOutput: public CSTmHDFormatterOutput
{
public:
  CSTih205MainOutput(CDisplayDevice *,
                     CSTmSDVTG *,
                     CSTmSDVTG *,
                     CSTmTVOutDENC *,
                     CDisplayMixer *,
                     CSTmFSynth *,
                     CSTmFSynth *,
                     CSTmHDFormatterAWG *,
                     stm_plane_id_t sharedVideoPlaneID,
                     stm_plane_id_t sharedGDPPlaneID);

  virtual ~CSTih205MainOutput();

  bool ShowPlane(stm_plane_id_t planeID);

protected:
  void StartHDClocks(const stm_mode_line_t*);
  void StartSDInterlacedClocks(const stm_mode_line_t*);
  void StartSDProgressiveClocks(const stm_mode_line_t*);
  void SetMainClockToHDFormatter(void);

  void EnableDACs(void);
  void DisableDACs(void);

private:
  CSTmFSynth *m_pFSynthAux;
  stm_plane_id_t m_sharedVideoPlaneID;
  stm_plane_id_t m_sharedGDPPlaneID;

  void WriteSysCfgReg(ULONG reg, ULONG val) { g_pIOS->WriteRegister(m_pDevReg + ((STih205_SYSCFG_BASE + reg)>>2), val); }
  ULONG ReadSysCfgReg(ULONG reg) { return g_pIOS->ReadRegister(m_pDevReg + ((STih205_SYSCFG_BASE +reg)>>2)); }

  void WriteClkReg(ULONG reg, ULONG val) { g_pIOS->WriteRegister(m_pDevReg + ((STih205_CLKGEN_BASE + reg)>>2), val); }
  ULONG ReadClkReg(ULONG reg) { return g_pIOS->ReadRegister(m_pDevReg + ((STih205_CLKGEN_BASE +reg)>>2)); }

  CSTih205MainOutput(const CSTih205MainOutput&);
  CSTih205MainOutput& operator=(const CSTih205MainOutput&);
};


#endif //_STih205MAINOUTPUT_H
