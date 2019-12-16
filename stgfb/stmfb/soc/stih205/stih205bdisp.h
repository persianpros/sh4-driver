/***********************************************************************
 *
 * File: soc/stih205/stih205bdisp.h
 * Copyright (c) 2008 STMicroelectronics Limited.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
\***********************************************************************/

#ifndef STih205BDISP_H
#define STih205BDISP_H

#include <STMCommon/stmbdisp.h>


class CSTih205BDisp: public CSTmBDisp
{
public:
  /* Number of queues:
       - 4 AQs
       - 2 CQs
     Valid draw and copy operations the BDisp on h205 can handle:
       - Disable destination colour keying
       - Enable BD-RLE decode */
  CSTih205BDisp (ULONG *pBDispBaseAddr): CSTmBDisp (pBDispBaseAddr,
                                                    _STM_BLITTER_VERSION_7200c2_7111_7141_7105,
                                                    BDISP_VALID_DRAW_OPS & ~STM_BLITTER_FLAGS_DST_COLOR_KEY,
                                                    ((BDISP_VALID_COPY_OPS & ~STM_BLITTER_FLAGS_DST_COLOR_KEY)
                                                     | STM_BLITTER_FLAGS_RLE_DECODE),
                                                    4,
                                                    2) {}

private:
  void ConfigurePlugs (void);
};


#endif // STih205BDISP_H
