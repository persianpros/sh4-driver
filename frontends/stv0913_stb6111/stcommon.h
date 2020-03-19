/*****************************************************************************/
/* COPYRIGHT (C) 2009 STMicroelectronics - All Rights Reserved               */
/* ST makes no warranty express or implied including but not limited to,     */
/* any warranty of                                                           */
/*                                                                           */
/*   (i)  merchantability or fitness for a particular purpose and/or         */
/*   (ii) requirements, for a particular purpose in relation to the LICENSED */
/*        MATERIALS, which is provided 'AS IS', WITH ALL FAULTS. ST does not */
/*        represent or warrant that the LICENSED MATERIALS provided here     */
/*        under is free of infringement of any third party patents,          */
/*        copyrights,trade secrets or other intellectual property rights.    */
/*        ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE       */
/*        EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                    */
/*                                                                           */
/*****************************************************************************/
/**
 @File   stcommon.h
 @brief

 Interface to revision reporting and clock info functions

*/

#ifndef _STCOMMON_H_
#define _STCOMMON_H_

/* C++ support */
/* ----------- */
#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
/* -------- */

#include "stddefs.h"
#include "sthwip.h"

#include "linuxcommon.h"

/* Clock infos structure for 5197 */
/* ------------------------------ */
#if defined(ST_5197)
typedef struct ST_ClockInfo_s
{
	U32 plla_clk;      /* PLL A clock                 */
	U32 pllb_clk;      /* PLL B clock                 */
	U32 st40cpu_clk;   /* ST40 cpu clock              */
	U32 st40per_clk;   /* ST40 peripheral clock       */
	U32 st40tick_clk;  /* ST40 timer clock            */
	U32 lmi_clk;       /* LMI sys clock               */
	U32 blitter_clk;   /* Blitter clock               */
	U32 sys_clk;       /* System clock                */
	U32 ic_clk;        /* Interconnect bus clock      */
	U32 com_clk;       /* Communication clock         */
	U32 fdma_clk;      /* FDMA clock                  */
	U32 av_clk;        /* Audio/Video clock           */
	U32 lcmpeg2_clk;   /* MPEG2 decoder clock         */
	U32 audio_clk;     /* Audio clock                 */
	U32 spare_clk;     /* Spare clock                 */
	U32 eth_clk;       /* Ethernet clock              */
	U32 fsa_0_clk;     /* Frequency synth A-0 clock   */
	U32 fsa_1_clk;     /* Frequency synth A-1 clock   */
	U32 fsa_2_clk;     /* Frequency synth A-2 clock   */
	U32 fsa_3_clk;     /* Frequency synth A-3 clock   */
	U32 fsb_0_clk;     /* Frequency synth B-0 clock   */
	U32 fsb_1_clk;     /* Frequency synth B-1 clock   */
	U32 fsb_2_clk;     /* Frequency synth B-2 clock   */
	U32 fsb_3_clk;     /* Frequency synth B-3 clock   */
	U32 fs_spare_clk;  /* FS Spare clock              */
	U32 pcm_clk;       /* PCM clock                   */
	U32 spdif_clk;     /* SPDIF clock                 */
	U32 dss_clk;       /* DSS clock                   */
	U32 pix_clk;       /* Pixel clock                 */
	U32 fdma_fs_clk;   /* FS FDMA clock               */
	U32 aux_clk;       /* Auxiliary clock             */
	U32 usb_clk;       /* USB clock                   */
} ST_ClockInfo_t;
#endif

/* Clock infos structure for 7111 & 7105*/
/* ------------------------------ */
#if defined(ST_7111) || defined (ST_7105)
typedef struct ST_ClockInfo_s
{
	/* Clock gen A */
	struct
	{
		U32 lmi_clk;         /* LMI PLL clock               */
		U32 pll0_clk;        /* PLL0 clock                  */
		U32 pll1_clk;        /* PLL1 clock                  */
		U32 st40cpu_clk;     /* ST40 cpu clock              */
		U32 st40tick_clk;    /* ST40 timer clock            */
		U32 com_clk;         /* Peripherals clock           */
		U32 fdma0_clk;       /* FDMA0 clock                 */
		U32 fdma1_clk;       /* FDMA1 clock                 */
		U32 lxaud_clk;       /* Lx aud cpu clock            */
		U32 lxdmu_clk;       /* Lx dmu cpu clock            */
		U32 bdisp_clk;       /* Blitter display clock       */
		U32 disp_clk;        /* Display clock               */
		U32 ts_clk;          /* Transport bus clock         */
		U32 vdp_clk;         /* Video display pipe clock    */
		U32 blit_clk;        /* Blitter clock               */
		U32 eth_clk;         /* Ethernet clock              */
		U32 pci_clk;         /* PCI clock                   */
		U32 emi_clk;         /* Emi clock                   */
	} ckga;
	/* Clock gen B */
	struct
	{
		U32 fs0_1_clk;    /* Frequency synth0 clock 1    */
		U32 fs0_2_clk;    /* Frequency synth0 clock 2    */
		U32 fs0_3_clk;    /* Frequency synth0 clock 3    */
		U32 fs1_1_clk;    /* Frequency synth1 clock 1    */
		U32 fs1_2_clk;    /* Frequency synth2 clock 2    */
		U32 fs1_3_clk;    /* Frequency synth3 clock 3    */
		U32 fs1_4_clk;    /* Frequency synth4 clock 4    */
		U32 hdmidll_clk;  /* HDMI serializer clock       */
		U32 hdmibch_clk;  /* HDMI bch clock              */
		U32 hdmitmds_clk; /* HDMI tmds clock             */
		U32 hdmipix_clk;  /* HDMI pixel clock            */
		U32 hdpix_clk;    /* HD pixel clock              */
		U32 hddisp_clk;   /* HD display clock            */
		U32 c656_clk;     /* DVO pixel clock             */
		U32 gdp3_clk;     /* GDP3 pixel clock            */
		U32 sdpix_clk;    /* SD pixel clock              */
		U32 sddisp_clk;   /* SD display clock            */
		U32 dvp_clk;      /* Emi clock                   */
		U32 pipe_clk;     /* Video pipeline clock        */
		U32 h264pp_clk;   /* H264 preprocessor clock     */
		U32 ic_clk;       /* Interconnect clock          */
		U32 rtc_clk;      /* Real time clock             */
		U32 lpc_clk;      /* Low power  clock            */
	} ckgb;
	/* CLOCK GEN C */
	struct
	{
		U32 fs0_1_clk;    /* Frequency synth0 clock 1    */
		U32 fs0_2_clk;    /* Frequency synth0 clock 2    */
		U32 fs0_3_clk;    /* Frequency synth0 clock 3    */
		U32 fs0_4_clk;    /* Frequency synth0 clock 4    */
	} ckgc;
	/* CLOCK GEN D */
	struct
	{
		U32 lmi_clk;         /* LMI PLL clock            */
	} ckgd;
} ST_ClockInfo_t;
#endif

/* Clock infos structure for 7141 */
/* ------------------------------ */
#if defined(ST_7141)
typedef struct ST_ClockInfo_s
{
	/* Clock gen A */
	struct
	{
		U32 lmi_clk;         /* LMI PLL clock               */
		U32 pll0_clk;        /* PLL0 clock                  */
		U32 pll1_clk;        /* PLL1 clock                  */
		U32 st40cpu_clk;     /* ST40 cpu clock              */
		U32 st40tick_clk;    /* ST40 timer clock            */
		U32 com_clk;         /* Peripherals clock           */
		U32 fdma0_clk;       /* FDMA0 clock                 */
		U32 fdma1_clk;       /* FDMA1 clock                 */
		U32 fdma2_clk;       /* FDMA2 clock                 */
		U32 lxaud_clk;       /* Lx aud cpu clock            */
		U32 lxdmu_clk;       /* Lx dmu cpu clock            */
		U32 bdisp_clk;       /* Blitter display clock       */
		U32 disp_clk;        /* Display clock               */
		U32 ts_clk;          /* Transport bus clock         */
		U32 vdp_clk;         /* Video display pipe clock    */
		U32 blit_clk;        /* Blitter clock               */
		U32 eth_clk;         /* Ethernet clock              */
		U32 pci_clk;         /* PCI clock                   */
		U32 emi_clk;         /* Emi clock                   */
	} ckga;
	/* Clock gen B */
	struct
	{
		U32 fs0_1_clk;    /* Frequency synth0 clock 1    */
		U32 fs0_2_clk;    /* Frequency synth0 clock 2    */
		U32 fs0_3_clk;    /* Frequency synth0 clock 3    */
		U32 fs1_1_clk;    /* Frequency synth1 clock 1    */
		U32 fs1_2_clk;    /* Frequency synth2 clock 2    */
		U32 fs1_3_clk;    /* Frequency synth3 clock 3    */
		U32 fs1_4_clk;    /* Frequency synth4 clock 4    */
		U32 hdmidll_clk;  /* HDMI serializer clock       */
		U32 hdmibch_clk;  /* HDMI bch clock              */
		U32 hdmitmds_clk; /* HDMI tmds clock             */
		U32 hdmipix_clk;  /* HDMI pixel clock            */
		U32 hdpix_clk;    /* HD pixel clock              */
		U32 hddisp_clk;   /* HD display clock            */
		U32 c656_clk;     /* DVO pixel clock             */
		U32 gdp3_clk;     /* GDP3 pixel clock            */
		U32 sdpix_clk;    /* SD pixel clock              */
		U32 sddisp_clk;   /* SD display clock            */
		U32 dvp_clk;      /* Emi clock                   */
		U32 pipe_clk;     /* Video pipeline clock        */
		U32 h264pp_clk;   /* H264 preprocessor clock     */
		U32 ic_clk;       /* Interconnect clock          */
		U32 rtc_clk;      /* Real time clock             */
		U32 lpc_clk;      /* Low power  clock            */
	} ckgb;
	/* CLOCK GEN C */
	struct
	{
		U32 fs0_1_clk;    /* Frequency synth0 clock 1    */
		U32 fs0_2_clk;    /* Frequency synth0 clock 2    */
		U32 fs0_3_clk;    /* Frequency synth0 clock 3    */
		U32 fs0_4_clk;    /* Frequency synth0 clock 4    */
	} ckgc;
	/* CLOCK GEN D */
	struct
	{
		U32 lmi_clk;         /* LMI PLL clock            */
	} ckgd;
} ST_ClockInfo_t;
#endif

/* Clock infos structure for 5206*/
/* ------------------------------ */
#if defined(ST_5206)
typedef struct ST_ClockInfo_s
{
	/* Clock gen A */
	struct
	{
		U32 pll0hs_clk;      /* PLL0 HS clock                */
		U32 pll0ls_clk;      /* PLL0 LS clock                */
		U32 pll1_clk;        /* PLL1  clock                  */
		U32 st40cpu_clk;     /* ST40 cpu clock               */
		U32 st40tick_clk;    /* ST40 timer clock             */
		U32 com_clk;         /* Peripherals clock            */
		U32 fdma0_clk;       /* FDMA0 clock                  */
		U32 fdma1_clk;       /* FDMA1 clock                  */
		U32 lxaud_clk;       /* Lx aud cpu clock             */
		U32 lxdmu_clk;       /* Lx dmu cpu clock             */
		U32 bdisp_clk;       /* Blitter display clock        */
		U32 disp_clk;        /* Display clock                */
		U32 ts_clk;          /* Transport bus clock          */
		U32 vdp_clk;         /* Video display pipe clock     */
		U32 blit_clk;        /* Blitter clock                */
		U32 eth_clk;         /* Ethernet clock               */
		U32 pci_clk;         /* PCI clock                    */
		U32 emi_clk;         /* Emi clock                    */
		U32 compo_clk;       /* CLKA_IC_COMPO_200            */
		U32 ic_clk;          /* Interconnect clock           */
		U32 mes_clk;         /* MES clock                    */
	} ckga;

	/* Clock gen B */
	struct
	{
		U32 fs0_1_clk;    /* Frequency synth0 clock 1    */
		U32 fs0_2_clk;    /* Frequency synth0 clock 2    */
		U32 fs0_3_clk;    /* Frequency synth0 clock 3    */
		U32 fs0_4_clk;    /* Frequency synth0 clock 4    */
		U32 fs1_1_clk;    /* Frequency synth1 clock 1    */
		U32 fs1_2_clk;    /* Frequency synth2 clock 2    */
		U32 fs1_3_clk;    /* Frequency synth3 clock 3    */
		U32 fs1_4_clk;    /* Frequency synth4 clock 4    */
		U32 fs_spare_clk; /* FS0 Spare clock             */
		U32 sdpix_clk;    /* SD pixel clock              */
		U32 hdpix_clk;    /* HD pixel clock              */
		U32 lpc_clk;      /* Low power  clock            */
	} ckgb;

	/* CLOCK GEN C */
	struct
	{
		U32 fs0_1_clk;    /* Frequency synth0 clock 1    */
		U32 fs0_2_clk;    /* Frequency synth0 clock 2    */
		U32 fs0_3_clk;    /* Frequency synth0 clock 3    */
		U32 fs0_4_clk;    /* Frequency synth0 clock 4    */
	} ckgc;

	/* CLOCK GEN D */
	struct
	{
		U32 lmi_clk;      /* LMI PLL clock CLKD_LMI2X*/
	} ckgd;
} ST_ClockInfo_t;
#endif

/* Clock infos structure for 7108*/
/* ------------------------------ */
#if defined(ST_7108)
typedef struct ST_ClockInfo_s
{
	/* Clock gen A */
	 struct
	{
		U32 pll0hs_l_clk;      /* PLL0 HS Left clock         */
		U32 pll0ls_l_clk;      /* PLL0 LS Left clock         */
		U32 pll1_l_clk;        /* PLL1  Left clock           */
		U32 st40cpu_clk;       /* ST40 Host cpu clock        */
		U32 st40cpu_1_clk;     /* ST40 RT cpu clock          */
		U32 st40tick_clk;      /* ST40 timer clock           */
		U32 com_clk;           /* Peripherals clock          */
		U32 lxaud_clk;         /* Lx aud cpu clock           */
		U32 lxdmu_clk;         /* Lx dmu cpu clock           */
		U32 eth1_clk;          /* Ethernet1 clock            */
		U32 pci_clk;           /* PCI clock                  */
		U32 emi_clk;           /* Emi clock                  */
		U32 pll0hs_r_clk;      /* PLL0 HS Right clock        */
		U32 pll0ls_r_clk;      /* PLL0 LS Right clock        */
		U32 pll1_r_clk;        /* PLL1  Right clock          */
		U32 fdma0_clk;         /* FDMA0 clock                */
		U32 fdma1_clk;         /* FDMA1 clock                */
		U32 fdma2_clk;         /* FDMA1 clock                */
		U32 eth_clk;           /* Ethernet clock             */
		U32 bdisp_clk;         /* Blitter display clock      */
		U32 ts_clk;            /* Transport bus clock        */
		U32 compo_clk;         /* Compositor clock           */
		U32 vdp_clk;           /* Video display pipe clock   */
		U32 blit_clk;          /* Blitter clock              */
	} ckga;
	/* Clock gen B */
	struct
	{
		U32 fs0_1_clk;    /* Frequency synth0 clock 1                 */
		U32 fs0_2_clk;    /* Frequency synth0 clock 2                 */
		U32 fs0_3_clk;    /* Frequency synth0 clock 3                 */
		U32 fs1_1_clk;    /* Frequency synth1 clock 1                 */
		U32 fs1_2_clk;    /* Frequency synth2 clock 2                 */
		U32 fs1_3_clk;    /* Frequency synth3 clock 3                 */
		U32 fs1_4_clk;    /* Frequency synth4 clock 4                 */
		U32 hdpix_vc_clk; /* HD pixel clock to video clock controller */
		U32 sdpix_vc_clk; /* SD pixel clock to video clock controller */
		U32 hdpix_clk;    /* HD pixel clock                           */
		U32 hddisp_clk;   /* HD display clock                         */
		U32 pipdisp_clk;  /* PIP display clock                        */
		U32 sddisp_clk;   /* SD display clock                         */
		U32 c656_clk;     /* DVO pixel clock                          */
		U32 gdp1_clk;     /* GDP1 pixel clock                         */
		U32 gdp2_clk;     /* GDP2 pixel clock                         */
		U32 sdpix_clk;    /* SD pixel clock                           */
		U32 lpc_clk;      /* Low power  clock                         */
		U32 usb_clk;      /* USB clock                                */
		U32 ccsc_clk;     /* CCSC clock                               */
		U32 dss_clk;      /* samrtcard clock                          */
	} ckgb;
	/* Clock gen C */
	struct
	{
		U32 fs0_1_clk;    /* Frequency synth0 clock 1    */
		U32 fs0_2_clk;    /* Frequency synth0 clock 2    */
		U32 fs0_3_clk;    /* Frequency synth0 clock 3    */
		U32 fs0_4_clk;    /* Frequency synth0 clock 4    */
	  } ckgc;
	/* CLOCK GEN D */
	struct
	{
		U32 lmi_clk;      /* LMI PLL clock               */
	} ckgd;
} ST_ClockInfo_t;
#endif

/* Clock infos structure for H205*/
/* ------------------------------ */
#if defined(ST_H205)

 /*Actual Lille code*/

typedef struct ST_ClockInfo_s
{
	/* Clock gen A */
	struct
	{
		/*Clockgen a0*/
		U32 pll0hs_l_clk;      /* PLL0 HS Left clock         */
		U32 pll0ls_l_clk;      /* PLL0 LS Left clock         */
		U32 pll1_l_clk;        /* PLL1 HS Left clock         */
		U32 pll1ls_l_clk;      /* PLL1 LS Left clock         */
		U32 st40tick_clk;      /* ST40 timer clock           */
		U32 st40cpu_clk;       /* ST40 Host cpu clock        */
		U32 com_clk;           /* Peripherals clock*/
		U32 hqvdpproc_clk;     /* HD Display pipeline processing clock*/
		U32 ic_cpu_clk;        /* Interconnect clock         */
		U32 lxdmu_clk;         /* Lx dmu cpu clock           */
		U32 lxaud_clk;         /* Lx aud cpu clock           */
		U32 ic_stnoc_clk;      /* stnoc clock                */
		U32 gdp_proc_clk;      /* gdp processor clock        */
		U32 nand_ctrl_clk;     /* nand ctrl clock            */
		U32 reg_lp_on_clk;     /* reg lp on clock            */
		U32 sec_clk;           /* secure clock               */
		U32 ts_dma_clk;        /* dma interface clock        */
		U32 tsout_clk;         /* tsout clock                */
		U32 reg_lp_off_clk;    /* reg lp off clock           */
		U32 dmu_preproc_clk;   /* DMU preprocessor clock     */
		U32 thns_clk;          /* THNS clock                 */
		U32 sh4_ick_clk;       /* CPU proc and interface clock*/
		U32 pmb_clk;           /* PMB clock                  */
		U32 emi_clk;           /* EMI clock                  */
		U32 master_clk;        /* master clock               */
		U32 priv_tango_clk;    /* TANGO clock                */

		/*Clockgen a1*/
		U32 pll0hs_r_clk;      /* PLL0 HS Right clock        */
		U32 pll0ls_r_clk;      /* PLL0 LS Right clock        */
		U32 pll1_r_clk;        /* PLL1 HS Right clock        */
		U32 pll1ls_r_clk;      /* PLL1 LS Right clock        */
		U32 ic_ddrctrl_clk;    /* DDR CTRL clock             */
		U32 fdma0_clk;         /* FDMA0 clock                */
		U32 fdma1_clk;         /* FDMA1 clock                */
		U32 bdisp_ck_clk;      /* BDISP CK clock             */
		U32 tp_clk;            /* TP Processing clock        */
		U32 compo_clk;         /* COMPO clock                */
		U32 bdisp_clk;         /* BDISP clock                */
		U32 eth_clk;           /* Ethernet clock             */
		U32 dmu_clk;           /* DMU clock                  */
		U32 gmac_clk;          /* GMAC clock                 */
		U32 ptp_ref_clk;       /* PTP REF clock              */
		U32 iqi_clk;           /* HD display IQI clock       */
		U32 card_clk;          /* MMC card clock             */
		U32 vdp_clk;           /* Video display pipe clock   */
		U32 crypto_clk;        /* cryotp clock               */
		U32 sys_mmc_clk;       /* MMC sys clock              */
	} ckga;

	/* Clock gen B */
	struct
	{
		U32 fs0_1_clk;         /* vid_hd_local clk            */
		U32 fs0_2_clk;         /* vid_sd_local clk            */
		U32 fs0_3_clk;         /* pcm0 clock                  */
		U32 fs0_4_clk;         /* STFE clock clk_27_0         */
		U32 fs1_1_clk;         /* vid_sd_rem clk              */
		U32 fs1_2_clk;         /* pcm2 clk                    */
		U32 fs1_3_clk;         /* clk_48                      */
		U32 fs1_4_clk;         /* STFE clock clk_27_1         */
		U32 hdpix_clk;         /* HD pixel clock              */
		U32 hddisp_clk;        /* HD display clock            */
		U32 sddisp_clk;        /* SD display clock            */
		U32 denc_main_clk;     /*denc main clock              */
		U32 gdp1_clk;          /* GDP1 pixel clock            */
		U32 gdp3_clk;          /* GDP3 pixel clock            */
		U32 c656_clk;          /* DVO pixel clock             */
		U32 pixmain_clk;       /* Pix main clock              */
		U32 sdpix_clk;         /* SD pixel clock              */
		U32 gdp4_clk;          /* GDP4 pixel clock            */
		U32 pip_clk;           /* PIP display clock           */
	} ckgvidfs;

	/* Clock gen gpfs */
	struct
	{
		U32 fs0_1_clk;    /* spdif_clk   */
		U32 fs0_2_clk;    /* lpm_clk     */
		U32 fs0_3_clk;    /* DSS_clk     */
		U32 fs0_4_clk;    /* pcm1_clk    */
	} ckggpfs;

} ST_ClockInfo_t;
#endif

/* Legacy Clock info structure common across all SOCs*/
typedef struct ST_LegacyClockInfo_s
{
	U32 STBus;
	U32 CommsBlock;
	U32 VideoMem;
	U32 AudioDSP;
	U32 EMI;
	U32 Flash;
	U32 SDRAM;
	U32 PCM;
	U32 HPTimer;
	U32 LPTimer;
	U32 st40;
	U32 ST40Per;
	U32 PCI;
	U32 Aux;
} ST_LegacyClockInfo_t;

/* Audio frame structure to be used by STHDMI, STAUDLX and STAPP
   RnDHV00030225 can be referred */

typedef struct ST_AudioInfoFrame_s
{
	U32 InfoFrameType : 8; 	/* 0x04 == AudioInfo Frame */
	U32 InfoFrameVersion : 8; 	/* 0x01 */
	U32 LengthOfAudioInfo : 8; /* 10  */
	/* dataByte[1] */
	U32 AudioChannelCount : 3; /* CC0..CC2 : STAUD_AudioChannelCount_e */
	U32 F13 : 1; 				/* F13 = 0 .. reserved. */
	U32 AudioCodingType : 4; 	/* CT0..CT3 : STAUD_AudioFormat_e */
	/* dataByte[2-5] */
	U32 SampleSize : 2; 		/* STAUD_SampleSize_e */
	U32 SamplingFreq : 3;
	U32 F25_27 : 3; 			/* reserved */
	U32 CXT : 5;
	U32 F35_37 : 3; 			/* reserved */
	U32 ChannelAllocation : 8; /* STAUD_AudioCodingMode_e */
	U32 LFE : 2;
	U32 F52 : 1; 				/* reserved */
	U32 LSV : 4;
	U32 DM_INH : 1;
	/* dataByte[6-9] */
	U32 F60_97 : 32; 			/* reserved */
	/* dataByte[10] */
	U32 F100_107 : 8; 			/* reserved */
	U32 Align32 : 24; 			/* not part of AudioInfoFrame but cope with 32bit memory alignment. */
} ST_AudioInfoFrame_t;

/*Jitter distribution to get Jitter profile*/
typedef enum
{
	STCOMMON_JITTER_DISTRIB_TYPE_TUNER,
	STCOMMON_JITTER_DISTRIB_TYPE_IP,
	STCOMMON_JITTER_DISTRIB_TYPE_MAX
} STCOMMON_JitterDistrib_e;

/*
    STCOMMON System Handles
*/

/* Number of system handles (these are statically allocated) */
#define STCOMMON_MAX_SYSTEM_HANDLES 512
#define STCOMMON_INVALID_HANDLE 0xFFFFFFFF

/*
    An STCOMMON System Handle as a bitfield (note: use U32 - no typename is defined) is:
    struct STCOMMON_GlobalHandleStruct_t {
        unsigned int CpuId : 4;
        unsigned int HandleIndex : 12;
        unsigned int DriverId : 16;
    }
*/

/*   Note: #define XXX_MASK (((1 << bit_field_length)-1)<<bit_field_shift) */

#define STCOMMON_HANDLE_CPUID_SHIFT (28)
#define STCOMMON_HANDLE_CPUID_MASK (((1<<4)-1)<<28)

#define STCOMMON_HANDLE_HANDLEINDEX_SHIFT (16)
#define STCOMMON_HANDLE_HANDLEINDEX_MASK (((1<<12)-1)<<16)

#define STCOMMON_HANDLE_DRIVERID_SHIFT (0)
#define STCOMMON_HANDLE_DRIVERID_MASK (((1<<16)-1)<<0)

/*  STCOMMON_HANDLE_GET_FIELD: Accessor macro
    Possible values for Field:
        CPUID
        HANDLEINDEX
        DRIVERID
*/
#define STCOMMON_HANDLE_GET_FIELD(Handle, Field) \
    (((Handle) & STCOMMON_HANDLE_   ##Field##_MASK) >> STCOMMON_HANDLE_##Field##_SHIFT)
/* Prototypes */
/* ---------- */
U32  ST_ConvRevToNum(ST_Revision_t *ptRevision_p);
U32  ST_GetCutRevision(void);
U32  ST_GetClockSpeed(void);
U32  ST_GetClocksPerSecond(void);
U32  ST_GetClocksPerSecondHigh(void);
U32  ST_GetClocksPerSecondLow(void);
U32  ST_GetMajorRevision(ST_Revision_t *ptRevision_p);
U32  ST_GetMinorRevision(ST_Revision_t *ptRevision_p);
U32  ST_GetPatchRevision(ST_Revision_t *ptRevision_p);
#if !defined (ST_5197) && !defined(ST_7108)&& !defined(ST_H205)
U32 ST_GetIpVersion(ST_HWIPtype_t ipname);
#endif
U32 ST_GetSocRevision(void);
BOOL ST_AreStringsEqual(const char *DeviceName1_p,const char *DeviceName2_p);
//ST_ErrorCode_t ST_GetClockInfo(ST_ClockInfo_t *ClockInfo_p);
//ST_ErrorCode_t ST_GetLegacyClockInfo (ST_LegacyClockInfo_t *ClockInfo_p);
//ST_Revision_t STCOMMON_GetRevision(void);
//ST_ErrorCode_t ST_GetJitterProfile(STCOMMON_JitterDistrib_e JitterProfile,U32 *JitterAmplitude);
ST_ErrorCode_t STCOMMON_Open(void);
ST_ErrorCode_t STCOMMON_Close(void);

ST_ErrorCode_t STCOMMON_HandleGetDriverData(U32 SysHandle, void **DriverData);
ST_ErrorCode_t STCOMMON_HandleFromDriverData(U32 *SysHandle, void *DriverData);
ST_ErrorCode_t STCOMMON_HandleAlloc(U32 DriverID, void *DriverData, U32 *SysHandle_p);
ST_ErrorCode_t STCOMMON_HandleFree(U32 SysHandle);

/* C++ support */
/* ----------- */
#ifdef __cplusplus
}
#endif

#endif

