/***********************************************************************
 *
 * File: soc/stih205/stih205gdp.h
 * Copyright (c) 2008 STMicroelectronics Limited.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
\***********************************************************************/

#ifndef _STih205GDP_H
#define _STih205GDP_H

#include <Gamma/GammaCompositorGDP.h>

class CSTih205GDP: public CGammaCompositorGDP
{
public:
  CSTih205GDP(stm_plane_id_t GDPid, ULONG baseAddr);

private:
  CSTih205GDP(const CSTih205GDP&);
  CSTih205GDP& operator=(const CSTih205GDP&);
};

#endif // _STih205GDP_H
