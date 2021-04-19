/****************************************************************************
 *
 * STM STV0367 DVB-T/C demodulator driver, DVB-T mode
 *
 * Version for Fulan Spark7162
 *
 * Note: actual device is part of the STi7162 SoC
 *
 * Original code:
 * Copyright (C), 2005-2010, AV Frontier Tech. Co., Ltd.
 *
 * File: stv0367_ter.c
 *
 * History:
 *  Date            Athor       Version   Reason
 *  ============    =========   =======   =================
 *  2010-11-11      lwj         1.0       File creation
 *
 ****************************************************************************/
#define TUNER_USE_TER_STI7167TER

#include <linux/kernel.h>  /* Kernel support */
#include <linux/string.h>

#include "ywdefs.h"

#include "ywtuner_ext.h"
#include "tuner_def.h"

#include "ioarch.h"
#include "ioreg.h"
#include "tuner_interface.h"

#include "stv0367ter_init.h"  // register definitions
#include "chip_0367ter.h"
#include "stv0367ter_drv.h"
#include "stv0367_ter.h"

extern int paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[stv0367_ter] "

STCHIP_Register_t Def367Val[STV0367TER_NBREGS] =
{
	{ R367_ID,                         0x60 },
	{ R367_I2CRPT,                     0x22 },
	{ R367_TOPCTRL,                    0x02 },
	{ R367_IOCFG0,                     0x40 },
	{ R367_DAC0R,                      0x00 },
	{ R367_IOCFG1,                     0x00 },
	{ R367_DAC1R,                      0x00 },
	{ R367_IOCFG2,                     0x62 },
	{ R367_SDFR,                       0x00 },
	{ R367TER_STATUS,                  0xf8 },
	{ R367_AUX_CLK,                    0x0a },
	{ R367_FREESYS1,                   0x00 },
	{ R367_FREESYS2,                   0x00 },
	{ R367_FREESYS3,                   0x00 },
	{ R367_GPIO_CFG,                   0x55 },
	{ R367_GPIO_CMD,                   0x00 },
	{ R367TER_AGC2MAX,                 0xff },
	{ R367TER_AGC2MIN,                 0x00 },
	{ R367TER_AGC1MAX,                 0xff },
	{ R367TER_AGC1MIN,                 0x00 },
	{ R367TER_AGCR,                    0xbc },
	{ R367TER_AGC2TH,                  0x00 },
	{ R367TER_AGC12C,                  0x00 },
	{ R367TER_AGCCTRL1,                0x85 },
	{ R367TER_AGCCTRL2,                0x1f },
	{ R367TER_AGC1VAL1,                0x00 },
	{ R367TER_AGC1VAL2,                0x00 },
	{ R367TER_AGC2VAL1,                0x6f },
	{ R367TER_AGC2VAL2,                0x05 },
	{ R367TER_AGC2PGA,                 0x00 },
	{ R367TER_OVF_RATE1,               0x00 },
	{ R367TER_OVF_RATE2,               0x00 },
	{ R367TER_GAIN_SRC1,               0x2b },
	{ R367TER_GAIN_SRC2,               0x04 },
	{ R367TER_INC_DEROT1,              0x55 },
	{ R367TER_INC_DEROT2,              0x55 },
	{ R367TER_PPM_CPAMP_DIR,           0x2c },
	{ R367TER_PPM_CPAMP_INV,           0x00 },
	{ R367TER_FREESTFE_1,              0x00 },
	{ R367TER_FREESTFE_2,              0x1c },
	{ R367TER_DCOFFSET,                0x00 },
	{ R367TER_EN_PROCESS,              0x05 },
	{ R367TER_SDI_SMOOTHER,            0x80 },
	{ R367TER_FE_LOOP_OPEN,            0x1c },
	{ R367TER_FREQOFF1,                0x00 },
	{ R367TER_FREQOFF2,                0x00 },
	{ R367TER_FREQOFF3,                0x00 },
	{ R367TER_TIMOFF1,                 0x00 },
	{ R367TER_TIMOFF2,                 0x00 },
	{ R367TER_EPQ,                     0x02 },
	{ R367TER_EPQAUTO,                 0x01 },
	{ R367TER_SYR_UPDATE,              0xf5 },
	{ R367TER_CHPFREE,                 0x00 },
	{ R367TER_PPM_STATE_MAC,           0x23 },
	{ R367TER_INR_THRESHOLD,           0xff },
	{ R367TER_EPQ_TPS_ID_CELL,         0xf9 },
	{ R367TER_EPQ_CFG,                 0x00 },
	{ R367TER_EPQ_STATUS,              0x01 },
	{ R367TER_AUTORELOCK,              0x81 },
	{ R367TER_BER_THR_VMSB,            0x00 },
	{ R367TER_BER_THR_MSB,             0x00 },
	{ R367TER_BER_THR_LSB,             0x00 },
	{ R367TER_CCD,                     0x83 },
	{ R367TER_SPECTR_CFG,              0x00 },
	{ R367TER_CHC_DUMMY,               0x18 },
	{ R367TER_INC_CTL,                 0x88 },
	{ R367TER_INCTHRES_COR1,           0xb4 },
	{ R367TER_INCTHRES_COR2,           0x96 },
	{ R367TER_INCTHRES_DET1,           0x0e },
	{ R367TER_INCTHRES_DET2,           0x11 },
	{ R367TER_IIR_CELLNB,              0x8d },
	{ R367TER_IIRCX_COEFF1_MSB,        0x00 },
	{ R367TER_IIRCX_COEFF1_LSB,        0x00 },
	{ R367TER_IIRCX_COEFF2_MSB,        0x09 },
	{ R367TER_IIRCX_COEFF2_LSB,        0x18 },
	{ R367TER_IIRCX_COEFF3_MSB,        0x14 },
	{ R367TER_IIRCX_COEFF3_LSB,        0x9c },
	{ R367TER_IIRCX_COEFF4_MSB,        0x00 },
	{ R367TER_IIRCX_COEFF4_LSB,        0x00 },
	{ R367TER_IIRCX_COEFF5_MSB,        0x36 },
	{ R367TER_IIRCX_COEFF5_LSB,        0x42 },
	{ R367TER_FEPATH_CFG,              0x00 },
	{ R367TER_PMC1_FUNC,               0x65 },
	{ R367TER_PMC1_FOR,                0x00 },
	{ R367TER_PMC2_FUNC,               0x00 },
	{ R367TER_STATUS_ERR_DA,           0xe0 },
	{ R367TER_DIG_AGC_R,               0xfe },
	{ R367TER_COMAGC_TARMSB,           0x0b },
	{ R367TER_COM_AGC_TAR_ENMODE,      0x41 },
	{ R367TER_COM_AGC_CFG,             0x3e },
	{ R367TER_COM_AGC_GAIN1,           0x39 },
	{ R367TER_AUT_AGC_TARGETMSB,       0x0b },
	{ R367TER_LOCK_DET_MSB,            0x01 },
	{ R367TER_AGCTAR_LOCK_LSBS,        0x40 },
	{ R367TER_AUT_GAIN_EN,             0xf4 },
	{ R367TER_AUT_CFG,                 0xf0 },
	{ R367TER_LOCKN,                   0x23 },
	{ R367TER_INT_X_3,                 0x00 },
	{ R367TER_INT_X_2,                 0x03 },
	{ R367TER_INT_X_1,                 0x8d },
	{ R367TER_INT_X_0,                 0xa0 },
	{ R367TER_MIN_ERRX_MSB,            0x00 },
	{ R367TER_COR_CTL,                 0x23 },
	{ R367TER_COR_STAT,                0xf6 },
	{ R367TER_COR_INTEN,               0x00 },
	{ R367TER_COR_INTSTAT,             0x3f },
	{ R367TER_COR_MODEGUARD,           0x03 },
	{ R367TER_AGC_CTL,                 0x08 },
	{ R367TER_AGC_MANUAL1,             0x00 },
	{ R367TER_AGC_MANUAL2,             0x00 },
	{ R367TER_AGC_TARG,                0x16 },
	{ R367TER_AGC_GAIN1,               0x53 },
	{ R367TER_AGC_GAIN2,               0x1d },
	{ R367TER_RESERVED_1,              0x00 },
	{ R367TER_RESERVED_2,              0x00 },
	{ R367TER_RESERVED_3,              0x00 },
	{ R367TER_CAS_CTL,                 0x44 },
	{ R367TER_CAS_FREQ,                0xb3 },
	{ R367TER_CAS_DAGCGAIN,            0x12 },
	{ R367TER_SYR_CTL,                 0x04 },
	{ R367TER_SYR_STAT,                0x10 },
	{ R367TER_SYR_NCO1,                0x00 },
	{ R367TER_SYR_NCO2,                0x00 },
	{ R367TER_SYR_OFFSET1,             0x00 },
	{ R367TER_SYR_OFFSET2,             0x00 },
	{ R367TER_FFT_CTL,                 0x00 },
	{ R367TER_SCR_CTL,                 0x70 },
	{ R367TER_PPM_CTL1,                0xf8 },
	{ R367TER_TRL_CTL,                 0xac },
	{ R367TER_TRL_NOMRATE1,            0x1e },
	{ R367TER_TRL_NOMRATE2,            0x58 },
	{ R367TER_TRL_TIME1,               0x1d },
	{ R367TER_TRL_TIME2,               0xfc },
	{ R367TER_CRL_CTL,                 0x24 },
	{ R367TER_CRL_FREQ1,               0xad },
	{ R367TER_CRL_FREQ2,               0x9d },
	{ R367TER_CRL_FREQ3,               0xff },
	{ R367TER_CHC_CTL,                 0x01 },
	{ R367TER_CHC_SNR,                 0xf0 },
	{ R367TER_BDI_CTL,                 0x00 },
	{ R367TER_DMP_CTL,                 0x00 },
	{ R367TER_TPS_RCVD1,               0x30 },
	{ R367TER_TPS_RCVD2,               0x02 },
	{ R367TER_TPS_RCVD3,               0x01 },
	{ R367TER_TPS_RCVD4,               0x00 },
	{ R367TER_TPS_ID_CELL1,            0x00 },
	{ R367TER_TPS_ID_CELL2,            0x00 },
	{ R367TER_TPS_RCVD5_SET1,          0x02 },
	{ R367TER_TPS_SET2,                0x02 },
	{ R367TER_TPS_SET3,                0x01 },
	{ R367TER_TPS_CTL,                 0x00 },
	{ R367TER_CTL_FFTOSNUM,            0x34 },
	{ R367TER_TESTSELECT,              0x09 },
	{ R367TER_MSC_REV,                 0x0a },
	{ R367TER_PIR_CTL,                 0x00 },
	{ R367TER_SNR_CARRIER1,            0xa1 },
	{ R367TER_SNR_CARRIER2,            0x9a },
	{ R367TER_PPM_CPAMP,               0x2c },
	{ R367TER_TSM_AP0,                 0x00 },
	{ R367TER_TSM_AP1,                 0x00 },
	{ R367TER_TSM_AP2,                 0x00 },
	{ R367TER_TSM_AP3,                 0x00 },
	{ R367TER_TSM_AP4,                 0x00 },
	{ R367TER_TSM_AP5,                 0x00 },
	{ R367TER_TSM_AP6,                 0x00 },
	{ R367TER_TSM_AP7,                 0x00 },
	{ R367_TSTRES,                     0x00 },
	{ R367_ANACTRL,                    0x0D },  /* caution: PLL stopped, to be restarted at init!!! */
	{ R367_TSTBUS,                     0x00 },
	{ R367TER_TSTRATE,                 0x00 },
	{ R367TER_CONSTMODE,               0x01 },
	{ R367TER_CONSTCARR1,              0x00 },
	{ R367TER_CONSTCARR2,              0x00 },
	{ R367TER_ICONSTEL,                0x0a },
	{ R367TER_QCONSTEL,                0x15 },
	{ R367TER_TSTBISTRES0,             0x00 },
	{ R367TER_TSTBISTRES1,             0x00 },
	{ R367TER_TSTBISTRES2,             0x28 },
	{ R367TER_TSTBISTRES3,             0x00 },
	{ R367_RF_AGC1,                    0xff },
	{ R367_RF_AGC2,                    0x83 },
	{ R367_ANADIGCTRL,                 0x19 },
	{ R367_PLLMDIV,                    0x0c },
	{ R367_PLLNDIV,                    0x55 },
	{ R367_PLLSETUP,                   0x18 },
	{ R367_DUAL_AD12,                  0x00 },
	{ R367_TSTBIST,                    0x00 },
	{ R367TER_PAD_COMP_CTRL,           0x00 },
	{ R367TER_PAD_COMP_WR,             0x00 },
	{ R367TER_PAD_COMP_RD,             0xe0 },
	{ R367TER_SYR_TARGET_FFTADJT_MSB,  0x00 },
	{ R367TER_SYR_TARGET_FFTADJT_LSB,  0x00 },
	{ R367TER_SYR_TARGET_CHCADJT_MSB,  0x00 },
	{ R367TER_SYR_TARGET_CHCADJT_LSB,  0x00 },
	{ R367TER_SYR_FLAG,                0x00 },
	{ R367TER_CRL_TARGET1,             0x00 },
	{ R367TER_CRL_TARGET2,             0x00 },
	{ R367TER_CRL_TARGET3,             0x00 },
	{ R367TER_CRL_TARGET4,             0x00 },
	{ R367TER_CRL_FLAG,                0x00 },
	{ R367TER_TRL_TARGET1,             0x00 },
	{ R367TER_TRL_TARGET2,             0x00 },
	{ R367TER_TRL_CHC,                 0x00 },
	{ R367TER_CHC_SNR_TARG,            0x00 },
	{ R367TER_TOP_TRACK,               0x00 },
	{ R367TER_TRACKER_FREE1,           0x00 },
	{ R367TER_ERROR_CRL1,              0x00 },
	{ R367TER_ERROR_CRL2,              0x00 },
	{ R367TER_ERROR_CRL3,              0x00 },
	{ R367TER_ERROR_CRL4,              0x00 },
	{ R367TER_DEC_NCO1,                0x2c },
	{ R367TER_DEC_NCO2,                0x0f },
	{ R367TER_DEC_NCO3,                0x20 },
	{ R367TER_SNR,                     0xf1 },
	{ R367TER_SYR_FFTADJ1,             0x00 },
	{ R367TER_SYR_FFTADJ2,             0x00 },
	{ R367TER_SYR_CHCADJ1,             0x00 },
	{ R367TER_SYR_CHCADJ2,             0x00 },
	{ R367TER_SYR_OFF,                 0x00 },
	{ R367TER_PPM_OFFSET1,             0x00 },
	{ R367TER_PPM_OFFSET2,             0x03 },
	{ R367TER_TRACKER_FREE2,           0x00 },
	{ R367TER_DEBG_LT10,               0x00 },
	{ R367TER_DEBG_LT11,               0x00 },
	{ R367TER_DEBG_LT12,               0x00 },
	{ R367TER_DEBG_LT13,               0x00 },
	{ R367TER_DEBG_LT14,               0x00 },
	{ R367TER_DEBG_LT15,               0x00 },
	{ R367TER_DEBG_LT16,               0x00 },
	{ R367TER_DEBG_LT17,               0x00 },
	{ R367TER_DEBG_LT18,               0x00 },
	{ R367TER_DEBG_LT19,               0x00 },
	{ R367TER_DEBG_LT1A,               0x00 },
	{ R367TER_DEBG_LT1B,               0x00 },
	{ R367TER_DEBG_LT1C,               0x00 },
	{ R367TER_DEBG_LT1D,               0x00 },
	{ R367TER_DEBG_LT1E,               0x00 },
	{ R367TER_DEBG_LT1F,               0x00 },
	{ R367TER_RCCFGH,                  0x00 },
	{ R367TER_RCCFGM,                  0x00 },
	{ R367TER_RCCFGL,                  0x00 },
	{ R367TER_RCINSDELH,               0x00 },
	{ R367TER_RCINSDELM,               0x00 },
	{ R367TER_RCINSDELL,               0x00 },
	{ R367TER_RCSTATUS,                0x00 },
	{ R367TER_RCSPEED,                 0x6f },
	{ R367TER_RCDEBUGM,                0xe7 },
	{ R367TER_RCDEBUGL,                0x9b },
	{ R367TER_RCOBSCFG,                0x00 },
	{ R367TER_RCOBSM,                  0x00 },
	{ R367TER_RCOBSL,                  0x00 },
	{ R367TER_RCFECSPY,                0x00 },
	{ R367TER_RCFSPYCFG,               0x00 },
	{ R367TER_RCFSPYDATA,              0x00 },
	{ R367TER_RCFSPYOUT,               0x00 },
	{ R367TER_RCFSTATUS,               0x00 },
	{ R367TER_RCFGOODPACK,             0x00 },
	{ R367TER_RCFPACKCNT,              0x00 },
	{ R367TER_RCFSPYMISC,              0x00 },
	{ R367TER_RCFBERCPT4,              0x00 },
	{ R367TER_RCFBERCPT3,              0x00 },
	{ R367TER_RCFBERCPT2,              0x00 },
	{ R367TER_RCFBERCPT1,              0x00 },
	{ R367TER_RCFBERCPT0,              0x00 },
	{ R367TER_RCFBERERR2,              0x00 },
	{ R367TER_RCFBERERR1,              0x00 },
	{ R367TER_RCFBERERR0,              0x00 },
	{ R367TER_RCFSTATESM,              0x00 },
	{ R367TER_RCFSTATESL,              0x00 },
	{ R367TER_RCFSPYBER,               0x00 },
	{ R367TER_RCFSPYDISTM,             0x00 },
	{ R367TER_RCFSPYDISTL,             0x00 },
	{ R367TER_RCFSPYOBS7,              0x00 },
	{ R367TER_RCFSPYOBS6,              0x00 },
	{ R367TER_RCFSPYOBS5,              0x00 },
	{ R367TER_RCFSPYOBS4,              0x00 },
	{ R367TER_RCFSPYOBS3,              0x00 },
	{ R367TER_RCFSPYOBS2,              0x00 },
	{ R367TER_RCFSPYOBS1,              0x00 },
	{ R367TER_RCFSPYOBS0,              0x00 },
	{ R367TER_TSGENERAL,               0x00 },
	{ R367TER_RC1SPEED,                0x6f },
	{ R367TER_TSGSTATUS,               0x18 },
	{ R367TER_FECM,                    0x01 },
	{ R367TER_VTH12,                   0xff },
	{ R367TER_VTH23,                   0xa1 },
	{ R367TER_VTH34,                   0x64 },
	{ R367TER_VTH56,                   0x40 },
	{ R367TER_VTH67,                   0x00 },
	{ R367TER_VTH78,                   0x2c },
	{ R367TER_VITCURPUN,               0x12 },
	{ R367TER_VERROR,                  0x01 },
	{ R367TER_PRVIT,                   0x3f },
	{ R367TER_VAVSRVIT,                0x00 },
	{ R367TER_VSTATUSVIT,              0xbd },
	{ R367TER_VTHINUSE,                0xa1 },
	{ R367TER_KDIV12,                  0x20 },
	{ R367TER_KDIV23,                  0x40 },
	{ R367TER_KDIV34,                  0x20 },
	{ R367TER_KDIV56,                  0x30 },
	{ R367TER_KDIV67,                  0x00 },
	{ R367TER_KDIV78,                  0x30 },
	{ R367TER_SIGPOWER,                0x54 },
	{ R367TER_DEMAPVIT,                0x40 },
	{ R367TER_VITSCALE,                0x00 },
	{ R367TER_FFEC1PRG,                0x00 },
	{ R367TER_FVITCURPUN,              0x12 },
	{ R367TER_FVERROR,                 0x01 },
	{ R367TER_FVSTATUSVIT,             0xbd },
	{ R367TER_DEBUG_LT1,               0x00 },
	{ R367TER_DEBUG_LT2,               0x00 },
	{ R367TER_DEBUG_LT3,               0x00 },
	{ R367TER_TSTSFMET,                0x00 },
	{ R367TER_SELOUT,                  0x00 },
	{ R367TER_TSYNC,                   0x00 },
	{ R367TER_TSTERR,                  0x00 },
	{ R367TER_TSFSYNC,                 0x00 },
	{ R367TER_TSTSFERR,                0x00 },
	{ R367TER_TSTTSSF1,                0x01 },
	{ R367TER_TSTTSSF2,                0x1f },
	{ R367TER_TSTTSSF3,                0x00 },
	{ R367TER_TSTTS1,                  0x00 },
	{ R367TER_TSTTS2,                  0x1f },
	{ R367TER_TSTTS3,                  0x01 },
	{ R367TER_TSTTS4,                  0x00 },
	{ R367TER_TSTTSRC,                 0x00 },
	{ R367TER_TSTTSRS,                 0x00 },
	{ R367TER_TSSTATEM,                0xb0 },
	{ R367TER_TSSTATEL,                0x40 },
	{ R367TER_TSCFGH,                  0x80 },
	{ R367TER_TSCFGM,                  0x00 },
	{ R367TER_TSCFGL,                  0x20 },
	{ R367TER_TSSYNC,                  0x00 },
	{ R367TER_TSINSDELH,               0x00 },
	{ R367TER_TSINSDELM,               0x00 },
	{ R367TER_TSINSDELL,               0x00 },
	{ R367TER_TSDIVN,                  0x03 },
	{ R367TER_TSDIVPM,                 0x00 },
	{ R367TER_TSDIVPL,                 0x00 },
	{ R367TER_TSDIVQM,                 0x00 },
	{ R367TER_TSDIVQL,                 0x00 },
	{ R367TER_TSDILSTKM,               0x00 },
	{ R367TER_TSDILSTKL,               0x00 },
	{ R367TER_TSSPEED,                 0x6f },
	{ R367TER_TSSTATUS,                0x81 },
	{ R367TER_TSSTATUS2,               0x6a },
	{ R367TER_TSBITRATEM,              0x0f },
	{ R367TER_TSBITRATEL,              0xc6 },
	{ R367TER_TSPACKLENM,              0x00 },
	{ R367TER_TSPACKLENL,              0xfc },
	{ R367TER_TSBLOCLENM,              0x0a },
	{ R367TER_TSBLOCLENL,              0x80 },
	{ R367TER_TSDLYH,                  0x90 },
	{ R367TER_TSDLYM,                  0x68 },
	{ R367TER_TSDLYL,                  0x01 },
	{ R367TER_TSNPDAV,                 0x00 },
	{ R367TER_TSBUFSTATH,              0x00 },
	{ R367TER_TSBUFSTATM,              0x00 },
	{ R367TER_TSBUFSTATL,              0x00 },
	{ R367TER_TSDEBUGM,                0xcf },
	{ R367TER_TSDEBUGL,                0x1e },
	{ R367TER_TSDLYSETH,               0x00 },
	{ R367TER_TSDLYSETM,               0x68 },
	{ R367TER_TSDLYSETL,               0x00 },
	{ R367TER_TSOBSCFG,                0x00 },
	{ R367TER_TSOBSM,                  0x47 },
	{ R367TER_TSOBSL,                  0x1f },
	{ R367TER_ERRCTRL1,                0x95 },
	{ R367TER_ERRCNT1H,                0x80 },
	{ R367TER_ERRCNT1M,                0x00 },
	{ R367TER_ERRCNT1L,                0x00 },
	{ R367TER_ERRCTRL2,                0x95 },
	{ R367TER_ERRCNT2H,                0x00 },
	{ R367TER_ERRCNT2M,                0x00 },
	{ R367TER_ERRCNT2L,                0x00 },
	{ R367TER_FECSPY,                  0x88 },
	{ R367TER_FSPYCFG,                 0x2c },
	{ R367TER_FSPYDATA,                0x3a },
	{ R367TER_FSPYOUT,                 0x06 },
	{ R367TER_FSTATUS,                 0x61 },
	{ R367TER_FGOODPACK,               0xff },
	{ R367TER_FPACKCNT,                0xff },
	{ R367TER_FSPYMISC,                0x66 },
	{ R367TER_FBERCPT4,                0x00 },
	{ R367TER_FBERCPT3,                0x00 },
	{ R367TER_FBERCPT2,                0x36 },
	{ R367TER_FBERCPT1,                0x36 },
	{ R367TER_FBERCPT0,                0x14 },
	{ R367TER_FBERERR2,                0x00 },
	{ R367TER_FBERERR1,                0x03 },
	{ R367TER_FBERERR0,                0x28 },
	{ R367TER_FSTATESM,                0x00 },
	{ R367TER_FSTATESL,                0x02 },
	{ R367TER_FSPYBER,                 0x00 },
	{ R367TER_FSPYDISTM,               0x01 },
	{ R367TER_FSPYDISTL,               0x9f },
	{ R367TER_FSPYOBS7,                0xc9 },
	{ R367TER_FSPYOBS6,                0x99 },
	{ R367TER_FSPYOBS5,                0x08 },
	{ R367TER_FSPYOBS4,                0xec },
	{ R367TER_FSPYOBS3,                0x01 },
	{ R367TER_FSPYOBS2,                0x0f },
	{ R367TER_FSPYOBS1,                0xf5 },
	{ R367TER_FSPYOBS0,                0x08 },
	{ R367TER_SFDEMAP,                 0x40 },
	{ R367TER_SFERROR,                 0x00 },
	{ R367TER_SFAVSR,                  0x30 },
	{ R367TER_SFECSTATUS,              0xcc },
	{ R367TER_SFKDIV12,                0x20 },
	{ R367TER_SFKDIV23,                0x40 },
	{ R367TER_SFKDIV34,                0x20 },
	{ R367TER_SFKDIV56,                0x20 },
	{ R367TER_SFKDIV67,                0x00 },
	{ R367TER_SFKDIV78,                0x20 },
	{ R367TER_SFDILSTKM,               0x00 },
	{ R367TER_SFDILSTKL,               0x00 },
	{ R367TER_SFSTATUS,                0xb5 },
	{ R367TER_SFDLYH,                  0x90 },
	{ R367TER_SFDLYM,                  0x60 },
	{ R367TER_SFDLYL,                  0x01 },
	{ R367TER_SFDLYSETH,               0xc0 },
	{ R367TER_SFDLYSETM,               0x60 },
	{ R367TER_SFDLYSETL,               0x00 },
	{ R367TER_SFOBSCFG,                0x00 },
	{ R367TER_SFOBSM,                  0x47 },
	{ R367TER_SFOBSL,                  0x05 },
	{ R367TER_SFECINFO,                0x40 },
	{ R367TER_SFERRCTRL,               0x74 },
	{ R367TER_SFERRCNTH,               0x80 },
	{ R367TER_SFERRCNTM,               0x00 },
	{ R367TER_SFERRCNTL,               0x00 },
	{ R367TER_SYMBRATEM,               0x2f },
	{ R367TER_SYMBRATEL,               0x50 },
	{ R367TER_SYMBSTATUS,              0x7f },
	{ R367TER_SYMBCFG,                 0x00 },
	{ R367TER_SYMBFIFOM,               0xf4 },
	{ R367TER_SYMBFIFOL,               0x0d },
	{ R367TER_SYMBOFFSM,               0xf0 },
	{ R367TER_SYMBOFFSL,               0x2d },
	{ R367TER_DEBUG_LT4,               0x00 },
	{ R367TER_DEBUG_LT5,               0x00 },
	{ R367TER_DEBUG_LT6,               0x00 },
	{ R367TER_DEBUG_LT7,               0x00 },
	{ R367TER_DEBUG_LT8,               0x00 },
	{ R367TER_DEBUG_LT9,               0x00 }
};


FE_LLA_Error_t FE_367ter_Algo(u32 Handle, FE_367ter_InternalParams_t *pParams, FE_TER_SearchResult_t *pResult);

/*****************************************************
 **FUNCTION  ::  PowOf2
 **ACTION    ::  Compute  2^n (where n is an integer)
 **PARAMS IN ::  number -> n
 **PARAMS OUT::  NONE
 **RETURN    ::  2^n
 *****************************************************/
s32 PowOf2(s32 number)
{
	s32 i;
	s32 result = 1;
	for (i = 0; i < number; i++)
	{
		result *= 2;
	}
	return result;
}

/***********************************************************************
    Name:   demod_stv0367ter_Repeat
 ***********************************************************************/
unsigned int demod_stv0367ter_Repeat(u32 DemodIOHandle, u32 TunerIOHandle, TUNER_IOARCH_Operation_t Operation,
				 unsigned short SubAddr, u8 *Data, u32 TransferSize, u32 Timeout)
{
	return 0;
}

#if defined(TUNER_USE_TER_STI7167TER)
void stv0367ter_init(TUNER_IOREG_DeviceMap_t *DeviceMap, u32 IOHandle, TUNER_TunerType_T TunerType)
{
	u16 i;

	for (i = 0; i < DeviceMap->Registers; i++)
	{
		ChipUpdateDefaultValues_0367ter(DeviceMap, IOHandle, Def367Val[i].Addr, Def367Val[i].Value, i);
		ChipSetOneRegister_0367ter(DeviceMap, IOHandle, Def367Val[i].Addr, Def367Val[i].Value);
	}

	if (DeviceMap != NULL)
	{
		switch (TunerType)
		{
			case TUNER_TUNER_SN761640:
			{
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367_ANACTRL, 0x0D);  /* PLL bypassed and disabled */
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367_PLLMDIV, 0x01);  /* IC runs at 54MHz with a 27MHz crystal */
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367_PLLNDIV, 0x08);
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367_PLLSETUP, 0x18);  /* ADC clock is equal to system clock */
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367_TOPCTRL, 0x0);  /* Buffer Q disabled */
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367_DUAL_AD12, 0x04);  /* ADCQ disabled */
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_GAIN_SRC1, 0x2A);
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_GAIN_SRC2, 0xD6);
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_INC_DEROT1, 0x55);
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_INC_DEROT2, 0x55);
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_TRL_CTL, 0x14);
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_TRL_NOMRATE1, 0xAE);
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_TRL_NOMRATE2, 0x56);
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367TER_FEPATH_CFG, 0x0);
				ChipSetOneRegister_0367ter(DeviceMap, IOHandle, R367_ANACTRL, 0x00);
				break;
			}
			default:
			{
				dprintk(1, "%s: Error: tuner type %d unknown\n", __func__, TunerType);
				break;
			}
		}
	}
	FE_STV0367TER_SetCLKgen(DeviceMap, IOHandle, DeviceMap->RegExtClk);
	FE_STV0367ter_SetTS_Parallel_Serial(DeviceMap, IOHandle, FE_TS_PARALLEL_PUNCT_CLOCK);
	FE_STV0367ter_SetCLK_Polarity(DeviceMap, IOHandle, FE_TS_RISINGEDGE_CLOCK);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367_STDBY_ADCGP, 1);
	ChipSetField_0367ter(DeviceMap, IOHandle, F367_STDBY_ADCGP, 0);
}

/***********************************************************************
    Name:   demod_stv0367ter_Open
 ************************************************************************/
unsigned int demod_stv0367ter_Open(u8 Handle)
{
	unsigned int            Error = YW_NO_ERROR;
	TUNER_ScanTaskParam_T   *Inst = NULL;
	u32         IOHandle;
	TUNER_IOREG_DeviceMap_t *DeviceMap;

//	dprintk(100, "%s >\n", __func__);
	Inst = TUNER_GetScanInfo(Handle);
	IOHandle = Inst->DriverParam.Ter.DemodIOHandle;
	DeviceMap = &Inst->DriverParam.Ter.Demod_DeviceMap;

	Inst->DriverParam.Ter.DemodDriver.Demod_GetSignalInfo = demod_stv0367ter_GetSignalInfo;
	Inst->DriverParam.Ter.DemodDriver.Demod_IsLocked      = demod_stv0367ter_IsLocked;
	Inst->DriverParam.Ter.DemodDriver.Demod_repeat        = demod_stv0367ter_Repeat;
	Inst->DriverParam.Ter.DemodDriver.Demod_reset         = demod_stv0367ter_Reset;
	Inst->DriverParam.Ter.DemodDriver.Demod_ScanFreq      = demod_stv0367ter_ScanFreq;
	Inst->DriverParam.Ter.DemodDriver.Demod_standy        = demod_stv0367ter_SetStandby;

	DeviceMap->Timeout   = IOREG_DEFAULT_TIMEOUT;
	DeviceMap->Registers = STV0367TER_NBREGS;
	DeviceMap->Fields    = STV0367TER_NBFIELDS;
	DeviceMap->Mode      = IOREG_MODE_SUBADR_16;
	DeviceMap->RegExtClk = Inst->ExternalClock;  // Demod External Crystal_HZ

	//Error = TUNER_IOREG_Open(DeviceMap);
	DeviceMap->DefVal = NULL;
	DeviceMap->Error = 0;

	stv0367ter_init(DeviceMap, IOHandle, TUNER_TUNER_SN761640);
	return Error;
}

unsigned int demod_stv0367ter_Open_test(u32 IOHandle)
{
	unsigned int Error = YW_NO_ERROR;
	TUNER_IOREG_DeviceMap_t DeviceMap;

//	dprintk(100, "%s >\n", __func__);

	DeviceMap.Timeout   = IOREG_DEFAULT_TIMEOUT;
	DeviceMap.Registers = STV0367TER_NBREGS;
	DeviceMap.Fields    = STV0367TER_NBFIELDS;
	DeviceMap.Mode      = IOREG_MODE_SUBADR_16;
	DeviceMap.RegExtClk = 27000000;  // Demod External Crystal_HZ

	//Error = TUNER_IOREG_Open(&DeviceMap);
	DeviceMap.DefVal = NULL;
	DeviceMap.Error = 0;

	stv0367ter_init(&DeviceMap, IOHandle, TUNER_TUNER_SN761640);
	//Error = TUNER_IOREG_Close(&DeviceMap);
	return Error;
}

/***********************************************************************
    Name:   demod_stv0367ter_Close
 ***********************************************************************/
unsigned int demod_stv0367ter_Close(u8 Handle)
{
	unsigned int Error = YW_NO_ERROR;
	TUNER_ScanTaskParam_T *Inst = NULL;

	Inst = TUNER_GetScanInfo(Handle);
	Inst->DriverParam.Ter.DemodDriver.Demod_GetSignalInfo = NULL;
	Inst->DriverParam.Ter.DemodDriver.Demod_IsLocked = NULL;
	Inst->DriverParam.Ter.DemodDriver.Demod_repeat = NULL;
	Inst->DriverParam.Ter.DemodDriver.Demod_reset = NULL;
	Inst->DriverParam.Ter.DemodDriver.Demod_ScanFreq = NULL;
	//Error = TUNER_IOREG_Close(&Inst->DriverParam.Ter.Demod_DeviceMap);
	Error |= Inst->DriverParam.Ter.Demod_DeviceMap.Error;
	Inst->DriverParam.Ter.Demod_DeviceMap.Error = YW_NO_ERROR;
	return Error;
}

/***********************************************************************
    Name:   demod_stv0367_ter_Reset
 **********************************************************************/
unsigned int demod_stv0367ter_Reset(u8 Index)
{
	unsigned int Error = YW_NO_ERROR;
	TUNER_ScanTaskParam_T *Inst = NULL;
	u32 IOHandle;
	TUNER_IOREG_DeviceMap_t *DeviceMap;

	Inst = TUNER_GetScanInfo(Index);
	IOHandle = Inst->DriverParam.Ter.DemodIOHandle;
	DeviceMap = &Inst->DriverParam.Ter.Demod_DeviceMap;
	stv0367ter_init(DeviceMap, IOHandle, Inst->DriverParam.Ter.TunerType);
	return Error;
}

/***********************************************************************
    Name:   demod_stv0367ter_GetSignalInfo
 **********************************************************************/
unsigned int demod_stv0367ter_GetSignalInfo(u8 Index, u32 *Quality, u32 *Intensity, u32 *Ber)
{
	unsigned int Error = YW_NO_ERROR;

	*Quality = 0;
	*Intensity = 0;
	*Ber = 0;
	//FE_STV0367TER_GetSignalInfo(Index, Quality, Intensity, Ber);
//	dprintk(70, "%s: %d, Quality = %d, Intensity = %d\n",__func__, Index,*Quality,*Intensity);
	return Error;
}

/***********************************************************************
    Name:   demod_stv0367ter_IsLocked
 **********************************************************************/
unsigned int demod_stv0367ter_IsLocked(u8 Handle, BOOL *IsLocked)
{
	unsigned int          Error = YW_NO_ERROR;
	TUNER_ScanTaskParam_T   *Inst;
	u32         IOHandle;
	TUNER_IOREG_DeviceMap_t *DeviceMap;

	Inst = TUNER_GetScanInfo(Handle);
	IOHandle = Inst->DriverParam.Ter.DemodIOHandle;
	DeviceMap = &Inst->DriverParam.Ter.Demod_DeviceMap;

	*IsLocked = FE_367ter_lock(DeviceMap, IOHandle);
	return (Error);
}

/***********************************************************************
    Name:   demod_stv0367ter_ScanFreq
 **********************************************************************/
unsigned int demod_stv0367ter_ScanFreq(u8 Index)
{
	unsigned int          Error = YW_NO_ERROR;
	TUNER_ScanTaskParam_T   *Inst;
	u32         IOHandle;
	TUNER_IOREG_DeviceMap_t *DeviceMap;
	/*u8 trials[2]; */
	s8 num_trials = 1;
	u8 index;
	FE_LLA_Error_t error = FE_LLA_NO_ERROR;
	u8 flag_spec_inv;
	u8 flag;
	u8 SenseTrials[2];
	u8 SenseTrialsAuto[2];
	FE_367ter_InternalParams_t pParams;
	FE_TER_SearchResult_t pResult;

	Inst = TUNER_GetScanInfo(Index);
	IOHandle = Inst->DriverParam.Ter.DemodIOHandle;
	DeviceMap = &Inst->DriverParam.Ter.Demod_DeviceMap;

	memset(&pParams, 0, sizeof(FE_367ter_InternalParams_t));
	memset(&pResult, 0, sizeof(FE_TER_SearchResult_t));
#if 0 //question
	SenseTrials[0] = INV;
	SenseTrials[1] = NINV;
	SenseTrialsAuto[0] = INV;
	SenseTrialsAuto[1] = NINV;
#else
	SenseTrials[1] = INV;
	SenseTrials[0] = NINV;
	SenseTrialsAuto[1] = INV;
	SenseTrialsAuto[0] = NINV;
#endif
	switch (Inst->DriverParam.Ter.Param.ChannelBW)
	{
		case YWTUNER_TER_BANDWIDTH_8_MHZ:
		{
			pParams.ChannelBW = 8;
			break;
		}
		case YWTUNER_TER_BANDWIDTH_7_MHZ:
		{
			pParams.ChannelBW = 7;
			break;
		}
		case YWTUNER_TER_BANDWIDTH_6_MHZ:
		{
			pParams.ChannelBW = 6;
			break;
		}
	}
//	dprintk(70, "%s: Index = %d\n", __func__, Index);
	pParams.Frequency  = Inst->DriverParam.Ter.Param.FreqKHz;
	pParams.Crystal_Hz = DeviceMap->RegExtClk;
	pParams.IF_IQ_Mode = FE_TER_NORMAL_IF_TUNER;  // FE_TER_IQ_TUNER;  // most tuner is IF mode, stv4100 is I/Q mode
	pParams.Inv        = FE_TER_INVERSION_AUTO;   // FE_TER_INVERSION_AUTO
	pParams.Hierarchy  = FE_TER_HIER_NONE;
	pParams.EchoPos    = 0;
	pParams.first_lock = 0;
//	dprintk(150, "%s: in FE_367TER_Search () value of pParams.Hierarchy %d\n", __func__, pParams.Hierarchy );
	flag_spec_inv      = 0;
	flag = ((pParams.Inv >> 1) & 1);

	switch (flag)  /* sw spectrum inversion for LP_IF &IQ &normal IF *db* */
	{
		case 0:  /* INV + INV_NONE */
		{
			if ((pParams.Inv == FE_TER_INVERSION_NONE) || (pParams.Inv == FE_TER_INVERSION))
			{
				num_trials = 1;
			}
			else  /* UNK */
			{
				num_trials = 2;
			}
			break;
		}
		case 1:  /* AUTO */
		{
			num_trials = 2;
			if ((pParams.first_lock)
			&&  (pParams.Inv == FE_TER_INVERSION_AUTO))
			{
				num_trials = 1;
			}
			break;
		}
		default:
		{
			return FE_TER_NOLOCK;
			break;
		}
	}

	pResult.SignalStatus = FE_TER_NOLOCK;
	index = 0;
	while (((index) < num_trials) && (pResult.SignalStatus != FE_TER_LOCKOK))
	{
		if (!pParams.first_lock)
		{
			if (pParams.Inv == FE_TER_INVERSION_UNK)
			{
				pParams.Sense = SenseTrials[index];
			}

			if (pParams.Inv == FE_TER_INVERSION_AUTO)
			{
				pParams.Sense = SenseTrialsAuto[index];
			}
		}
		error = FE_367ter_Algo(Index, &pParams, &pResult);

		if ((pResult.SignalStatus == FE_TER_LOCKOK)
		&&  (pParams.Inv == FE_TER_INVERSION_AUTO)
		&&  (index == 1))
		{
			SenseTrialsAuto[index] = SenseTrialsAuto[0];  /* invert spectrum sense */
			SenseTrialsAuto[(index + 1) % 2] = (SenseTrialsAuto[1] + 1) % 2;
		}
		index++;
	}

	if (!Error)
	{
		// Inst->DriverParam.Ter.Result = Inst->DriverParam.Ter.Param;
		if (pResult.Locked)
		{
			//dprintk(20, "Tuner locked\n");
			Inst->Status = TUNER_INNER_STATUS_LOCKED;
		}
		else
		{
			//dprintk(20, "Tuner not locked");
			Inst->Status = TUNER_INNER_STATUS_UNLOCKED;
		}
	}
	else
	{
		Inst->Status = TUNER_INNER_STATUS_UNLOCKED;
	}
	return Error;
}

/*****************************************************
 --FUNCTION  ::  FE_367TER_Algo
 --ACTION    ::  Search for a valid channel
 --PARAMS IN ::  Handle  ==> Front End Handle
                 pSearch ==> Search parameters
                 pResult ==> Result of the search
 --PARAMS OUT::  NONE
 --RETURN    ::  Error (if any)
 --***************************************************/
FE_LLA_Error_t FE_367ter_Algo(u32 Handle, FE_367ter_InternalParams_t *pParams, FE_TER_SearchResult_t *pResult)
{
	u32                     InternalFreq = 0, temp = 0;
//	u32                     intX = 0;
//	int                     AgcIF = 0;
	FE_LLA_Error_t          error = FE_LLA_NO_ERROR;
	BOOL                    TunerLocked;
	TUNER_ScanTaskParam_T   *Inst = NULL;
	TUNER_IOREG_DeviceMap_t *DeviceMap = NULL;
	u32         IOHandle;

	Inst = TUNER_GetScanInfo(Handle);
	IOHandle = Inst->DriverParam.Ter.DemodIOHandle;
	DeviceMap = &Inst->DriverParam.Ter.Demod_DeviceMap;
	if (Inst->ForceSearchTerm)
	{
		return (FE_LLA_INVALID_HANDLE);
	}
	if (pParams != NULL)
	{
		ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_CCS_ENABLE, 0);
//		dprintk(70, "%s: pIntParams->IF_IQ_Mode ==  %d\n", __func__, pParams->IF_IQ_Mode);
		switch (pParams->IF_IQ_Mode)
		{
			case FE_TER_NORMAL_IF_TUNER:  /* Normal IF mode */
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367_TUNER_BB, 0);
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_LONGPATH_IF, 0);
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_DEMUX_SWAP, 0);
				break;
			}
			case FE_TER_LONGPATH_IF_TUNER:  /* Long IF mode */
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367_TUNER_BB, 0);
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_LONGPATH_IF, 1);
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_DEMUX_SWAP, 1);
				break;
			}
			case FE_TER_IQ_TUNER:  /* IQ mode */
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367_TUNER_BB, 1);
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_PPM_INVSEL, 0); /*spectrum inversion hw detection off *db*/
				break;
			}
			default:
			{
				return FE_LLA_SEARCH_FAILED;
			}
		}
//		dprintk(70, "%s: pIntParams->Inv ==  %d\n", __func__, pParams->Inv);
//		dprintk(70, "%s: pIntParams->Sense ==  %d\n", __func__, pParams->Sense);

		if ((pParams->Inv == FE_TER_INVERSION_NONE))
		{
			if (pParams->IF_IQ_Mode == FE_TER_IQ_TUNER)
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_IQ_INVERT, 0);
				pParams->SpectrumInversion = FALSE;
			}
			else
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_INV_SPECTR, 0);
				pParams->SpectrumInversion = FALSE;
			}
		}
		else if (pParams->Inv == FE_TER_INVERSION)
		{
			if (pParams->IF_IQ_Mode == FE_TER_IQ_TUNER)
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_IQ_INVERT, 1);
				pParams->SpectrumInversion = TRUE;
			}
			else
			{
				ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_INV_SPECTR, 1);
				pParams->SpectrumInversion = TRUE;
			}
		}
		else if ((pParams->Inv == FE_TER_INVERSION_AUTO) || ((pParams->Inv == FE_TER_INVERSION_UNK)/*&& (!pParams->first_lock)*/))
		{
			if (pParams->IF_IQ_Mode == FE_TER_IQ_TUNER)
			{
				if (pParams->Sense == 1)
				{
					ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_IQ_INVERT, 1);
					pParams->SpectrumInversion = TRUE;
				}
				else
				{
					ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_IQ_INVERT, 0);
					pParams->SpectrumInversion = FALSE;
				}
			}
			else
			{
				if (pParams->Sense == 1)
				{
					ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_INV_SPECTR, 1);
					pParams->SpectrumInversion = TRUE;
				}
				else
				{
					ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_INV_SPECTR, 0);
					pParams->SpectrumInversion = FALSE;
				}
			}
		}
//		dprintk(70, "%s: pIntParams->PreviousChannelBW = %d\n", __func__, pParams->PreviousChannelBW);
//		dprintk(70, "%s: pIntParams->Crystal_Hz = %d\n", __func__, pParams->Crystal_Hz);
		if ((pParams->IF_IQ_Mode != FE_TER_NORMAL_IF_TUNER)
		&&  (pParams->PreviousChannelBW != pParams->ChannelBW))
		{
			FE_367TER_AGC_IIR_LOCK_DETECTOR_SET(DeviceMap, IOHandle);
			/* set fine agc target to 180 for LPIF or IQ mode */
			/* set Q_AGCTarget */
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_SEL_IQNTAR, 1);
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_AUT_AGC_TARGET_MSB, 0xB);
			/*ChipSetField_0367ter(DeviceMap, IOHandle,AUT_AGC_TARGET_LSB,0x04); */

			/* set Q_AGCTarget */
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_SEL_IQNTAR, 0);
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_AUT_AGC_TARGET_MSB, 0xB);
			/* ChipSetField_0367ter(DeviceMap, IOHandle,AUT_AGC_TARGET_LSB,0x04); */

			if (!FE_367TER_IIR_FILTER_INIT(DeviceMap, IOHandle, pParams->ChannelBW, pParams->Crystal_Hz))
			{
				return FE_LLA_BAD_PARAMETER;
			}
			/* set IIR filter once for 6, 7 or 8MHz BW */
			pParams->PreviousChannelBW = pParams->ChannelBW;
			FE_367TER_AGC_IIR_RESET(DeviceMap, IOHandle);
		}

//		dprintk(70, "%s: pIntParams->Hierarchy = %d\n", __func__, pParams->Hierarchy);

		/*********Code Added For Hierarchical Modulation****************/
		if (pParams->Hierarchy == FE_TER_HIER_LOW_PRIO)
		{
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_BDI_LPSEL, 0x01);
		}
		else
		{
			ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_BDI_LPSEL, 0x00);
		}
		/*************************/
		InternalFreq = FE_367ter_GetMclkFreq(DeviceMap, IOHandle, pParams->Crystal_Hz) / 1000;
//		dprintk(70, "%s: InternalFreq = %d\n", __func__, InternalFreq);
		temp = (int)((((pParams->ChannelBW * 64 * PowOf2(15) * 100) / (InternalFreq)) * 10) / 7);
//		dprintk(70, "%s: pParams->ChannelBW = %d\n", __func__, pParams->ChannelBW);
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TRL_NOMRATE_LSB, temp % 2);
		temp = temp / 2;
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TRL_NOMRATE_HI, temp / 256);
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TRL_NOMRATE_LO, temp % 256);
		//ChipSetRegisters_0367ter(DeviceMap, IOHandle,R367TER_TRL_NOMRATE1,2);//lwj
		ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_TRL_NOMRATE1, 1);
		ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_TRL_NOMRATE2, 1);
		//ChipGetRegisters_0367ter(DeviceMap, IOHandle,R367TER_TRL_CTL,3);//lwj
		ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_TRL_CTL, 1);
		ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_TRL_NOMRATE1, 1);
		ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_TRL_NOMRATE2, 1);

		temp = ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TRL_NOMRATE_HI) * 512
		     + ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TRL_NOMRATE_LO) * 2
		     + ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_TRL_NOMRATE_LSB);
		temp = (int)((PowOf2(17) * pParams->ChannelBW * 1000) / (7 * (InternalFreq)));
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_GAIN_SRC_HI, temp / 256);
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_GAIN_SRC_LO, temp % 256);
		//ChipSetRegisters_0367ter(DeviceMap, IOHandle,R367TER_GAIN_SRC1,2);
		ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_GAIN_SRC1, 1);
		ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_GAIN_SRC2, 1);

		//ChipGetRegisters_0367ter(DeviceMap, IOHandle,R367TER_GAIN_SRC1,2);
		ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_GAIN_SRC1, 1);
		ChipGetRegisters_0367ter(DeviceMap, IOHandle, R367TER_GAIN_SRC2, 1);
		temp = ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_GAIN_SRC_HI) * 256
		     + ChipGetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_GAIN_SRC_LO);
		if (Inst->DriverParam.Ter.TunerType == TUNER_TUNER_SN761640)
		{
			temp = (int)((InternalFreq - 36166667 / 1000) * PowOf2(16) / (InternalFreq));  // IF freq
		}
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_INC_DEROT_HI, temp / 256);
		ChipSetFieldImage_0367ter(DeviceMap, IOHandle, F367TER_INC_DEROT_LO, temp % 256);
		//ChipSetRegisters_0367ter(DeviceMap, IOHandle,R367TER_INC_DEROT1,2);  // lwj change
		ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_INC_DEROT1, 1);
		ChipSetRegisters_0367ter(DeviceMap, IOHandle, R367TER_INC_DEROT2, 1);
		//pParams->EchoPos = pSearch->EchoPos;
		ChipSetField_0367ter(DeviceMap, IOHandle, F367TER_LONG_ECHO, pParams->EchoPos);

		if (Inst->DriverParam.Ter.TunerType == TUNER_TUNER_SN761640)
		{
			if (Inst->DriverParam.Ter.TunerDriver.tuner_SetFreq != NULL)
			{
				error = (Inst->DriverParam.Ter.TunerDriver.tuner_SetFreq)(Handle, pParams->Frequency, pParams->ChannelBW, NULL);
				if (error != YW_NO_ERROR)
				{
					return error;
				}
			}
			if (Inst->DriverParam.Ter.TunerDriver.tuner_GetFreq != NULL)
			{
				(Inst->DriverParam.Ter.TunerDriver.tuner_GetFreq)(Handle, &(pParams->Frequency));
			}
		}
		else
		{
			dprintk(1, "%s: Error: unsupported tuner type %d\n", __func__, Inst->DriverParam.Ter.TunerType);
		}
		if (Inst->DriverParam.Ter.TunerDriver.tuner_IsLocked != NULL)
		{
			Inst->DriverParam.Ter.TunerDriver.tuner_IsLocked(Handle, &TunerLocked);
		}
		/*********************************/
		if (FE_367ter_LockAlgo(DeviceMap, IOHandle, pParams) == FE_TER_LOCKOK)
		{
			pResult->Locked = TRUE;
			pResult->SignalStatus = FE_TER_LOCKOK;
		}
		else
		{
			pResult->Locked = FALSE;
			error = FE_LLA_SEARCH_FAILED;
		}
	}
	else
	{
		error = FE_LLA_BAD_PARAMETER;
	}
	return error;
}

/*****************************************************
 --FUNCTION  ::  FE_STV0367TER_SetStandby
 --ACTION    ::  Set demod STANDBY mode On/Off
 --PARAMS IN ::  Handle  ==> Front End Handle

 --PARAMS OUT::  NONE.
 --RETURN    ::  Error (if any)
 --***************************************************/
//FE_LLA_Error_t FE_STV0367ter_SetStandby(u32 Handle, u8 StandbyOn)
unsigned int demod_stv0367ter_SetStandby(u8 Handle)
{
	FE_LLA_Error_t          error = FE_LLA_NO_ERROR;
	TUNER_ScanTaskParam_T   *Inst = NULL;
	TUNER_IOREG_DeviceMap_t *DeviceMap = NULL;
	u32                     IOHandle;
	u8                      StandbyOn = 1;

	Inst = TUNER_GetScanInfo(Handle);
	IOHandle = Inst->DriverParam.Ter.DemodIOHandle;
	DeviceMap = &Inst->DriverParam.Ter.Demod_DeviceMap;

	if (StandbyOn)
	{
		ChipSetField_0367ter(DeviceMap, IOHandle, F367_STDBY, 1);
		ChipSetField_0367ter(DeviceMap, IOHandle, F367_STDBY_FEC, 1);
		ChipSetField_0367ter(DeviceMap, IOHandle, F367_STDBY_CORE, 1);
	}
	else
	{
		ChipSetField_0367ter(DeviceMap, IOHandle, F367_STDBY, 0);
		ChipSetField_0367ter(DeviceMap, IOHandle, F367_STDBY_FEC, 0);
		ChipSetField_0367ter(DeviceMap, IOHandle, F367_STDBY_CORE, 0);
	}
	return error;
}
#endif

#define MAX_TUNER_NUM 4
static TUNER_ScanTaskParam_T TUNER_DbaseInst[MAX_TUNER_NUM];
TUNER_ScanTaskParam_T *TUNER_GetScanInfo(u8 Index)
{
	if (Index >= MAX_TUNER_NUM)
	{
		return NULL;
	}
	return &TUNER_DbaseInst[Index];
}
// vim:ts=4
