/*****************************************************************************/
/* COPYRIGHT (C) 2009 STMicroelectronics - All Rights Reserved               */
/* ST makes no warranty express or implied including but not limited to,     */
/* any warranty of                                                           */
/*                                                                           */
/*   (i)  merchantability or fitness for a particular purpose and/or         */
/*   (ii) requirements, for a particular purpose in relation to the LICENSED */
/*        MATERIALS, which is provided AS IS, WITH ALL FAULTS. ST does not   */
/*        represent or warrant that the LICENSED MATERIALS provided here     */
/*        under is free of infringement of any third party patents,          */
/*        copyrights, trade secrets or other intellectual property rights.   */
/*        ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE       */
/*        EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                    */
/*                                                                           */
/*****************************************************************************/

/**
   @file     sthwip.h
   @brief

   Contains the hardware ips present on SOC

*/

#ifndef __STHWIP_H
#define __STHWIP_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/* Public macros                                                              */
/******************************************************************************/

/* HWIP VERSION */
#ifndef HWIP_VERSION
#define HWIP_VERSION(a,b,c)     ((a<<16)|(b<<8)|c)
#endif

#ifndef INVALID_VERSION
#define INVALID_VERSION         HWIP_VERSION(0,0,0)
#endif

#ifndef HWIP_TERMINATE
#define HWIP_TERMINATE         0xffff
#endif

/******************************************************************************/
/* Public types                                                               */
/******************************************************************************/

typedef enum ST_HWIPtype_e
{

    /* Transport & Security */
        STIP_TSMERGER,
        STIP_STFE,
        STIP_TSGDMA,
        STIP_PTI,
        STIP_PDES3,
        STIP_CRYPTOCORE2,
        STIP_CRYPTOCORE3,
        STIP_TKDMA,
        STIP_ICAM,
        STIP_RNG,
        STIP_NVSTORAGE,
        STIP_FDMA,
        STIP_DOCSIS,
        STIP_MCHI,
        STIP_NQAMDMOD,
        STIP_SAFEMEMHV,
        STIP_EBEAM,
        STIP_CCSC,

    /* Graphics & Display */
        STIP_XVP,
        STIP_MAINVDP,
        STIP_SECVDP,
        STIP_AUXVDP,
        STIP_BLITTER,
        STIP_COMPOSITOR,
        STIP_DVP,

    /* Video Decode */
        STIP_DELTA,
        STIP_VDAC,

    /* TV Out */
        STIP_HDTVOUT,
        STIP_VTG,
        STIP_DENC,
        STIP_HDMI,
        STIP_HDFORMAT,
        STIP_AWG_DCS,
        STIP_DVO,
        STIP_HDCP,
        STIP_SDTVOUT,
        STIP_RFMOD,
        STIP_PCMPLAYER,
        STIP_PCMREADER,
        STIP_ADAC,
        STIP_SPDIFPLAYER,

    /* Parallel connectivity */
        STIP_EMISS,
        STIP_PCI,
        STIP_PLI,
        STIP_MES,
        STIP_COMMS,
        STIP_PIOSTDALONE,

    /* Serial connectivity */
        STIP_SATALINK,
        STIP_SATAPHY,
        STIP_SATAHOST,
        STIP_CPGSATA,
        STIP_FTMSATA,
        STIP_HDMISERIALISER,

    /* CPU specific */
        STIP_SH4CPU,
        STIP_C6S4300,
        STIP_ST231,

    /* SOC Specific */
        STIP_FE900,
        STIP_CLKGENA,
        STIP_CLKGENB,
        STIP_PLL800,
        STIP_PLL1600,
        STIP_QFS

} ST_HWIPtype_t;

#ifdef __cplusplus
}
#endif
#endif /*#ifndef __STHWIP_H */

