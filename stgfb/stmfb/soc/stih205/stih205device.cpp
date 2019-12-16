/***********************************************************************
 *
 * File: soc/stih205/stih205device.cpp
 * Copyright (c) 2008,2009 STMicroelectronics Limited.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
\***********************************************************************/
//FIXME: Copied from stih205!!!
#include <stmdisplay.h>

#include <Generic/IOS.h>
#include <Generic/IDebug.h>

#include <STMCommon/stmsdvtg.h>
#include <STMCommon/stmbdispregs.h>
#include <STMCommon/stmvirtplane.h>
#include <STMCommon/stmfsynth.h>

#include <HDTVOutFormatter/stmhdfawg.h>
#include <HDTVOutFormatter/stmtvoutteletext.h>
#include <HDTVOutFormatter/stmtvoutdenc.h>

#include <Gamma/GDPBDispOutput.h>
#include <Gamma/GammaCompositorNULL.h>

#include "stih205reg.h"
#include "stih205device.h"
#include "stih205auxoutput.h"
#include "stih205mainoutput.h"
#include "stih205hdmi.h"
#include "stih205mixer.h"
#include "stih205bdisp.h"
#include "stih205gdp.h"
#include "stih205vdp.h"
#include "stih205dei.h"
#include "stih205dvo.h"
#include "stih205cursor.h"

CSTih205Device::CSTih205Device(void): CGenericGammaDevice()

{
  DENTRY();

  m_pGammaReg = (ULONG*)g_pIOS->MapMemory(STih205_REGISTER_BASE, STih205_REG_ADDR_SIZE);
  ASSERTF(m_pGammaReg,("CSTih205Device::CSTih205Device 'unable to map device registers'\n"));

  /*
   * Global setup of the display clock registers.
   */
  WriteDevReg(STih205_CLKGEN_BASE + CKGB_LCK, CKGB_LCK_UNLOCK);
#if defined(__TDT__) && defined(CONFIG_CPU_SUBTYPE_STXH205) && defined(USE_FS1_FOR_SD)
  DEBUGF2(1,("+++ USING FS1 for SCART OUTPUT +++\n"));
  WriteDevReg(STih205_CLKGEN_BASE + CKGB_CLK_REF_SEL, CKGB_REF_SEL_SYSA_CLKIN);

  // Setup FS1 CHANNEL 3 -> DeltaMu Preprocessor clock, what ever that is
  // Jup we need this, else hd channels will sufer from microfractures
  WriteDevReg(STih205_CLKGEN_BASE + CKGB_FS1_EN3, 0x0);

  WriteDevReg(STih205_CLKGEN_BASE + CKGB_FS1_MD3, 0x19);
  WriteDevReg(STih205_CLKGEN_BASE + CKGB_FS1_PE3, 0x3334);
  WriteDevReg(STih205_CLKGEN_BASE + CKGB_FS1_SDIV3, 0x00);

  WriteDevReg(STih205_CLKGEN_BASE + CKGB_FS1_EN3, 0x1);
  WriteDevReg(STih205_CLKGEN_BASE + CKGB_FS1_EN3, 0x0);


  //WriteDevReg(STih205_CLKGEN_BASE + CKGB_CLK_SRC, 0x07);

#else
  WriteDevReg(STih205_CLKGEN_BASE + CKGB_CLK_REF_SEL, CKGB_REF_SEL_SYSB_CLKIN);
#endif

  m_pFSynthHD       = 0;
  m_pFSynthSD       = 0;
  m_pDENC           = 0;
  m_pVTG1           = 0;
  m_pVTG2           = 0;
  m_pMainMixer      = 0;
  m_pAuxMixer       = 0;
  m_pVideoPlug1     = 0;
  m_pVideoPlug2     = 0;
  m_pAWGAnalog      = 0;
  m_pGDPBDispOutput = 0;
  m_pBDisp          = 0;
  m_pTeletext       = 0;

  /*
   * Default shared video plane is VID2, but for some completely daft reason
   * later chips appear to have changed this to VID1, ahhhhggggg.
   */
  m_sharedVideoPlaneID = OUTPUT_VID2;
  m_sharedGDPPlaneID = OUTPUT_GDP3;

  DEBUGF2(2,("CSTih205Device::CSTih205Device out\n"));
}


CSTih205Device::~CSTih205Device()
{
  DENTRY();

  delete m_pDENC;
  delete m_pVTG1;
  delete m_pVTG2;
  delete m_pMainMixer;
  delete m_pAuxMixer;
  delete m_pVideoPlug1;
  delete m_pVideoPlug2;
  delete m_pFSynthHD;
  delete m_pFSynthSD;
  delete m_pAWGAnalog;
  delete m_pTeletext;

  /*
   * Override the base class destruction of registered graphics accelerators,
   * we need to leave this to the destruction of the BDisp class.
   */
  for(unsigned int i=0;i<N_ELEMENTS(m_graphicsAccelerators);i++)
    m_graphicsAccelerators[i] = 0;
  delete m_pBDisp;

  DEXIT();
}


bool CSTih205Device::Create()
{
  if(!CreateInfrastructure())
    return false;

  if(!CreateMixers())
    return false;

  if(!CreatePlanes())
    return false;

  if(!CreateOutputs())
    return false;

  if(!CreateGraphics())
    return false;

  if(!CGenericGammaDevice::Create())
  {
    DEBUGF(("CSTih205Device::Create - base class Create failed\n"));
    return false;
  }

  DEBUGF2(2,("CSTih205Device::Create out\n"));

  return true;
}


bool CSTih205Device::CreateInfrastructure(void)
{
  if((m_pFSynthHD = new CSTmFSynthType1(this, STih205_CLKGEN_BASE+CKGB_FS0_MD1)) == 0)
  {
    DERROR("failed to create FSynthHD\n");
    return false;
  }

  m_pFSynthHD->SetClockReference(STM_CLOCK_REF_30MHZ,0);

  if((m_pFSynthSD = new CSTmFSynthType1(this, STih205_CLKGEN_BASE+CKGB_FS1_MD1)) == 0)
  {
    DERROR("failed to create FSynthSD\n");
    return false;
  }

  m_pFSynthSD->SetClockReference(STM_CLOCK_REF_30MHZ,0);

  if((m_pVTG1 = new CSTmSDVTG(this, STih205_VTG1_BASE, m_pFSynthHD, false, STVTG_SYNC_POSITIVE)) == 0)
  {
    DERROR("failed to create VTG1\n");
    return false;
  }

  if((m_pVTG2 = new CSTmSDVTG(this, STih205_VTG2_BASE, m_pFSynthSD, false, STVTG_SYNC_POSITIVE)) == 0)
  {
    DERROR("failed to create VTG2\n");
    return false;
  }

  m_pTeletext = new CSTmTVOutTeletext(this, STih205_DENC_BASE,
                                            STih205_AUX_TVOUT_BASE,
                                            STih205_REGISTER_BASE,
                                            SDTP_DENC1_TELETEXT);
  /*
   * Slight difference for Teletext, if we fail to create it then clean up
   * and continue. We may not have been able to claim an FDMA channel and we
   * are not treating that as fatal.
   */
  if(m_pTeletext && !m_pTeletext->Create())
  {
    DERROR("failed to create Teletext");
    delete m_pTeletext;
    m_pTeletext = 0;
  }

  m_pDENC = new CSTmTVOutDENC(this, STih205_DENC_BASE, m_pTeletext);
  if(!m_pDENC || !m_pDENC->Create())
  {
    DERROR("failed to create DENC\n");
    return false;
  }

  if((m_pAWGAnalog = new CSTmHDFormatterAWG(this,STih205_HD_FORMATTER_BASE,STih205_HD_FORMATTER_AWG_SIZE)) == 0)
  {
    DERROR("failed to create analogue sync AWG\n");
    return false;
  }

  return true;
}


bool CSTih205Device::CreateMixers(void)
{
  if((m_pMainMixer = new CSTih205MainMixer(this,STih205_COMPOSITOR_BASE+STih205_MIXER1_OFFSET)) == 0)
  {
    DERROR("failed to create main mixer\n");
    return false;
  }

  if((m_pAuxMixer = new CSTih205AuxMixer(this,STih205_COMPOSITOR_BASE+STih205_MIXER2_OFFSET)) == 0)
  {
    DERROR("failed to create aux mixer\n");
    return false;
  }

  return true;
}


bool CSTih205Device::CreatePlanes(void)
{
  CGammaCompositorGDP   *gdp;
  CSTih205Cursor        *cursor;
  CGammaCompositorPlane *null;

  ULONG ulCompositorBaseAddr = ((ULONG)m_pGammaReg) + STih205_COMPOSITOR_BASE;

  gdp = new CSTih205GDP(OUTPUT_GDP1, ulCompositorBaseAddr+STih205_GDP1_OFFSET);
  if(!gdp || !gdp->Create())
  {
    DERROR("failed to create GDP1\n");
    delete gdp;
    return false;
  }
  m_hwPlanes[0] = gdp;

  gdp = new CSTih205GDP(OUTPUT_GDP2, ulCompositorBaseAddr+STih205_GDP2_OFFSET);
  if(!gdp || !gdp->Create())
  {
    DERROR("failed to create GDP2\n");
    delete gdp;
    return false;
  }
  m_hwPlanes[1] = gdp;

  gdp = new CSTih205GDP(OUTPUT_GDP3, ulCompositorBaseAddr+STih205_GDP3_OFFSET);
  if(!gdp || !gdp->Create())
  {
    DERROR("failed to create GDP3\n");
    delete gdp;
    return false;
  }
  m_hwPlanes[2] = gdp;

  m_pVideoPlug1 = new CGammaVideoPlug(ulCompositorBaseAddr+STih205_VID1_OFFSET, true, true);
  if(!m_pVideoPlug1)
  {
    DERROR("failed to create video plugs\n");
    return false;
  }

  m_pVideoPlug2 = new CGammaVideoPlug(ulCompositorBaseAddr+STih205_VID2_OFFSET, true, true);
  if(!m_pVideoPlug2)
  {
    DERROR("failed to create video plugs\n");
    return false;
  }

  if(!CreateVideoPlanes(&m_hwPlanes[3], &m_hwPlanes[4]))
    return false;

  cursor = new CSTih205Cursor(ulCompositorBaseAddr+STih205_CUR_OFFSET);
  if(!cursor || !cursor->Create())
  {
    DERROR("failed to create CUR\n");
    delete cursor;
    return false;
  }
  m_hwPlanes[5] = cursor;

  null = new CGammaCompositorNULL(OUTPUT_NULL);
  if(!null || !null->Create())
  {
    DERROR("failed to create NULL\n");
    delete null;
    return false;
  }
  m_hwPlanes[6] = null;

  m_numPlanes = 7;
  return true;
}


bool CSTih205Device::CreateVideoPlanes(CGammaCompositorPlane **vid1,CGammaCompositorPlane **vid2)
{
  DENTRY();
  CDEIVideoPipe *disp;

  disp = new CSTih205DEI(OUTPUT_VID1, m_pVideoPlug1, ((ULONG)m_pGammaReg) + STih205_HD_BASE);
  if(!disp || !disp->Create())
  {
    DERROR("failed to create VID1\n");
    delete disp;
    return false;
  }
  *vid1 = disp;

  disp = new CSTih205VDP(OUTPUT_VID2, m_pVideoPlug2, ((ULONG)m_pGammaReg) + STih205_SD_BASE);
  if(!disp || !disp->Create())
  {
    DERROR("failed to create VID2\n");
    delete disp;
    return false;
  }
  *vid2 = disp;

  DEXIT();
  return true;
}


bool CSTih205Device::CreateHDMIOutput(void)
{
  CSTmHDMI *pHDMI;

  if((pHDMI = new CSTih205HDMI(this, m_pOutputs[STih205_OUTPUT_IDX_VDP0_MAIN], m_pVTG1)) == 0)
  {
    DERROR("failed to allocate HDMI output\n");
    return false;
  }

  if(!pHDMI->Create())
  {
    DERROR("failed to create HDMI output\n");
    delete pHDMI;
    return false;
  }

  m_pOutputs[STih205_OUTPUT_IDX_VDP0_HDMI] = pHDMI;
  return true;
}


bool CSTih205Device::CreateOutputs(void)
{
  CSTih205MainOutput *pMainOutput;
  if((pMainOutput = new CSTih205MainOutput(this, m_pVTG1,m_pVTG2, m_pDENC, m_pMainMixer, m_pFSynthHD, m_pFSynthSD, m_pAWGAnalog, m_sharedVideoPlaneID, m_sharedGDPPlaneID)) == 0)
  {
    DERROR("failed to create main output\n");
    return false;
  }
  m_pOutputs[STih205_OUTPUT_IDX_VDP0_MAIN] = pMainOutput;

  if((m_pOutputs[STih205_OUTPUT_IDX_VDP1_MAIN] = new CSTih205AuxOutput(this, m_pVTG2, m_pDENC, m_pAuxMixer, m_pFSynthSD, m_pAWGAnalog, pMainOutput, m_sharedVideoPlaneID, m_sharedGDPPlaneID)) == 0)
  {
    DERROR("failed to create aux output\n");
    return false;
  }

  if(!this->CreateHDMIOutput())
    return false;

  CSTih205DVO *pFDVO = new CSTih205DVO (this, pMainOutput,
                                              STih205_FLEXDVO_BASE,
                                              STih205_HD_FORMATTER_BASE);
  if(!pFDVO)
  {
    DERROR("failed to create FDVO output\n");
    return false;
  }
  m_pOutputs[STih205_OUTPUT_IDX_DVO0] = pFDVO;

  pMainOutput->SetSlaveOutputs(m_pOutputs[STih205_OUTPUT_IDX_DVO0],
                               m_pOutputs[STih205_OUTPUT_IDX_VDP0_HDMI]);

  m_pGDPBDispOutput = new CGDPBDispOutput(this, pMainOutput,
                                          m_pGammaReg + (STih205_BLITTER_BASE>>2),
                                          STih205_REGISTER_BASE+STih205_BLITTER_BASE,
                                          BDISP_CQ1_BASE,1,2);

  if(m_pGDPBDispOutput == 0)
  {
    DERROR("failed to allocate BDisp virtual plane output");
    return false;
  }

  if(!m_pGDPBDispOutput->Create())
  {
    DERROR("failed to create BDisp virtual plane output");
    return false;
  }

  m_pOutputs[STih205_OUTPUT_IDX_GDP] = m_pGDPBDispOutput;
  m_nOutputs = 5;

  return true;
}


bool CSTih205Device::CreateGraphics(void)
{
  m_pBDisp = new CSTih205BDisp(m_pGammaReg + (STih205_BLITTER_BASE>>2));
  if(!m_pBDisp)
  {
    DERROR("failed to allocate BDisp object\n");
    return false;
  }

  if(!m_pBDisp->Create())
  {
    DERROR("failed to create BDisp\n");
    return false;
  }

  m_graphicsAccelerators[STih205_BLITTER_IDX_KERNEL] = m_pBDisp->GetAQ(STih205_BLITTER_AQ_KERNEL);
  m_graphicsAccelerators[STih205_BLITTER_IDX_VDP0_MAIN] = m_pBDisp->GetAQ(STih205_BLITTER_AQ_VDP0_MAIN);
  m_graphicsAccelerators[STih205_BLITTER_IDX_VDP1_MAIN] = m_pBDisp->GetAQ(STih205_BLITTER_AQ_VDP1_MAIN);
  if(!m_graphicsAccelerators[STih205_BLITTER_IDX_KERNEL]
     || !m_graphicsAccelerators[STih205_BLITTER_IDX_VDP0_MAIN]
     || !m_graphicsAccelerators[STih205_BLITTER_IDX_VDP1_MAIN])
  {
    DERROR("failed to get three AQs\n");
    return false;
  }

  m_numAccelerators = 3;
  return true;
}


CDisplayPlane* CSTih205Device::GetPlane(stm_plane_id_t planeID) const
{
  CDisplayPlane *pPlane;

  if((pPlane = m_pGDPBDispOutput->GetVirtualPlane(planeID)) != 0)
    return pPlane;

  return CGenericGammaDevice::GetPlane(planeID);
}


void CSTih205Device::UpdateDisplay(COutput *pOutput)
{
  /*
   * Generic HW processing
   */
  CGenericGammaDevice::UpdateDisplay(pOutput);

  /*
   * Virtual plane output processing related to the main output
   */
  if(m_pGDPBDispOutput->GetTimingID() == pOutput->GetTimingID())
    m_pGDPBDispOutput->UpdateHW();
}
