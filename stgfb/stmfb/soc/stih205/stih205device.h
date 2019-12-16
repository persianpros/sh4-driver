/***********************************************************************
 *
 * File: soc/stih205/stih205device.h
 * Copyright (c) 2008,2009 STMicroelectronics Limited.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
\***********************************************************************/

#ifndef _STIh205DEVICE_H
#define _STIh205DEVICE_H

#ifdef __cplusplus

#include <Gamma/GenericGammaDevice.h>


class CSTmSDVTG;
class CSTmFSynthType1;
class CSTmTVOutDENC;
class CSTih205MainMixer;
class CSTih205AuxMixer;
class CSTih205BDisp;
class CGammaVideoPlug;
class CSTmHDFormatterAWG;
class CGDPBDispOutput;
class CSTmTVOutTeletext;
class CGammaCompositorPlane;
class CDisplayMixer;

class CSTih205Device : public CGenericGammaDevice
{
public:
  CSTih205Device(void);
  virtual ~CSTih205Device(void);

  bool Create(void);

  CDisplayPlane *GetPlane(stm_plane_id_t planeID) const;

  void UpdateDisplay(COutput *);

protected:
  CSTmFSynthType1        *m_pFSynthHD;
  CSTmFSynthType1        *m_pFSynthSD;
  CSTmSDVTG              *m_pVTG1;
  CSTmSDVTG              *m_pVTG2;
  CSTmTVOutDENC          *m_pDENC;
  CDisplayMixer          *m_pMainMixer;
  CDisplayMixer          *m_pAuxMixer;
  CGammaVideoPlug        *m_pVideoPlug1;
  CGammaVideoPlug        *m_pVideoPlug2;
  CSTmHDFormatterAWG     *m_pAWGAnalog;
  CGDPBDispOutput        *m_pGDPBDispOutput;
  CSTih205BDisp          *m_pBDisp;
  CSTmTVOutTeletext      *m_pTeletext;

  stm_plane_id_t         m_sharedVideoPlaneID;
  stm_plane_id_t         m_sharedGDPPlaneID;

  virtual bool CreateInfrastructure(void);
  virtual bool CreateMixers(void);
  virtual bool CreatePlanes(void);
  virtual bool CreateVideoPlanes(CGammaCompositorPlane **vid1,CGammaCompositorPlane **vid2);
  virtual bool CreateOutputs(void);
  virtual bool CreateHDMIOutput(void);
  virtual bool CreateGraphics(void);

private:
  CSTih205Device(const CSTih205Device&);
  CSTih205Device& operator=(const CSTih205Device&);
};

#endif /* __cplusplus */


#define STih205_OUTPUT_IDX_VDP0_MAIN      ( 0)
#define STih205_OUTPUT_IDX_VDP0_HDMI      ( 2)
#define STih205_OUTPUT_IDX_DVO0           ( 4)
#define STih205_OUTPUT_IDX_VDP1_MAIN      ( 1)
#define STih205_OUTPUT_IDX_GDP            ( 3)

#define STih205_BLITTER_IDX_KERNEL        ( 0)
#define STih205_BLITTER_IDX_VDP0_MAIN     ( 1)
#define STih205_BLITTER_IDX_VDP1_MAIN     ( 2)

#define STih205_BLITTER_AQ_KERNEL         ( 1)
#define STih205_BLITTER_AQ_VDP0_MAIN      ( 2)
#define STih205_BLITTER_AQ_VDP1_MAIN      ( 3)


#endif // _STIh205DEVICE_H
