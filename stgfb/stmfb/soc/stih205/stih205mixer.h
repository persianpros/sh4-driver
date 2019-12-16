/***********************************************************************
 *
 * File: soc/stih205/stih205mixer.h
 * Copyright (c) 2008,2009 STMicroelectronics Limited.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
\***********************************************************************/

#ifndef STih205MIXER_H
#define STih205MIXER_H

#include <Gamma/GammaMixer.h>

class CDisplayDevice;

class CSTih205MainMixer: public CGammaMixer
{
public:
  CSTih205MainMixer(CDisplayDevice *pDev, ULONG offset);

private:
  CSTih205MainMixer(const CSTih205MainMixer&);
  CSTih205MainMixer& operator=(const CSTih205MainMixer&);
};


class CSTih205AuxMixer: public CGammaMixer
{
public:
  CSTih205AuxMixer(CDisplayDevice *pDev, ULONG offset);

private:
  CSTih205AuxMixer(const CSTih205AuxMixer&);
  CSTih205AuxMixer& operator=(const CSTih205AuxMixer&);
};

#endif /* STih205MIXER_H */
