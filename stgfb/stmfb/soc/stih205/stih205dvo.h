/***********************************************************************
 *
 * File: soc/stih205/stih205dvo.h
 * Copyright (c) 2009 STMicroelectronics Limited.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
\***********************************************************************/

#ifndef _STIh205DVO_H
#define _STIh205DVO_H

#include <HDTVOutFormatter/stmhdfdvo.h>

class CSTih205DVO: public CSTmHDFDVO
{
public:
  CSTih205DVO(CDisplayDevice *,COutput *, ULONG ulDVOBase, ULONG ulHDFormatterBase);
  virtual ~CSTih205DVO();

protected:
  ULONG *m_pTVOutRegs;
  ULONG *m_pClkGenBRegs;

  bool SetOutputFormat(ULONG format, const stm_mode_line_t* pModeLine);

  void  WriteTVOutReg  (ULONG reg, ULONG value)  { g_pIOS->WriteRegister (m_pTVOutRegs + (reg>>2), value); }
  ULONG ReadTVOutReg   (ULONG reg)               { return g_pIOS->ReadRegister (m_pTVOutRegs + (reg>>2)); }

  void  WriteClkGenBReg (ULONG reg, ULONG value)  { g_pIOS->WriteRegister (m_pClkGenBRegs + (reg>>2), value); }
  ULONG ReadClkGenBReg  (ULONG reg)               { return g_pIOS->ReadRegister (m_pClkGenBRegs + (reg>>2)); }

private:
  CSTih205DVO(const CSTih205DVO&);
  CSTih205DVO& operator=(const CSTih205DVO&);
};


#endif /* _STIh205DVO_H */
