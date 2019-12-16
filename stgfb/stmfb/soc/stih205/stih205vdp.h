/***********************************************************************
 *
 * File: soc/stih205/stih205vdp.h
 * Copyright (c) 2008-2009 STMicroelectronics Limited.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
\***********************************************************************/
#ifndef _STIh205_VDP_H
#define _STIh205_VDP_H

#include <Gamma/DEIVideoPipe.h>

class CSTih205VDP: public CDEIVideoPipe
{
public:
  CSTih205VDP(stm_plane_id_t   planeID,
              CGammaVideoPlug *plug,
              ULONG            vdpBaseAddr): CDEIVideoPipe(planeID,plug,vdpBaseAddr)
  {
    m_bHaveDeinterlacer = false;
    m_keepHistoricBufferForDeinterlacing = false;
  }

private:
  CSTih205VDP (const CSTih205VDP &);
  CSTih205VDP& operator= (const CSTih205VDP &);
};

#endif // _STIh205_VDP_H
