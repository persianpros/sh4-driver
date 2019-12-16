/***********************************************************************
 *
 * File: soc/stih205/stih205cursor.h
 * Copyright (c) 2009 STMicroelectronics Limited.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
\***********************************************************************/

#ifndef STih205CURSOR_H
#define STih205CURSOR_H

#include <Gamma/GammaCompositorCursor.h>
#include <Gamma/GenericGammaReg.h>

class CSTih205Cursor: public CGammaCompositorCursor
{
public:
  CSTih205Cursor(ULONG baseAddr): CGammaCompositorCursor(baseAddr)
  {
    DENTRY();

    /*
     * Cursor plug registers are at the same offset as GDPs
     */
    WriteCurReg(GDPn_PAS ,6);
    WriteCurReg(GDPn_MAOS,5);
    WriteCurReg(GDPn_MIOS,3);
    WriteCurReg(GDPn_MACS,0);
    WriteCurReg(GDPn_MAMS,3);

    DEXIT();
  }

  ~CSTih205Cursor() {}

private:
  CSTih205Cursor(const CSTih205Cursor &);
  CSTih205Cursor& operator= (const CSTih205Cursor &);
};

#endif // STih205CURSOR_H
