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
 @File   STV0913_init.c
 @brief
*/
/* -------------------------------------------------------------------------
File Name: STV0913_init.c

Description: STV0913 registers initialization	V4.0 "March/20/2009"

---------------------------------------------------------------------------- */

#include "globaldefs.h"
#include "stv0913_init.h"


STCHIP_Register_t STV0913DefVal[STV0913_NBREGS]=
{
	{ RSTV0913_MID,           0x50 },    /* MID */
	{ RSTV0913_DID,           0x20 },    /* DID */
	{ RSTV0913_DACR1,         0x00 },    /* DACR1 */
	{ RSTV0913_DACR2,         0x00 },    /* DACR2 */
	{ RSTV0913_PADCFG,        0x05 },    /* PADCFG */
	{ RSTV0913_OUTCFG2,       0x00 },    /* OUTCFG2 */
	{ RSTV0913_OUTCFG,        0x7e },    /* OUTCFG */
	{ RSTV0913_IRQSTATUS3,    0x00 },    /* IRQSTATUS3 */
	{ RSTV0913_IRQSTATUS2,    0x00 },    /* IRQSTATUS2 */
	{ RSTV0913_IRQSTATUS1,    0x00 },    /* IRQSTATUS1 */
	{ RSTV0913_IRQSTATUS0,    0x00 },    /* IRQSTATUS0 */
	{ RSTV0913_IRQMASK3,      0x00 },    /* IRQMASK3 */
	{ RSTV0913_IRQMASK2,      0x00 },    /* IRQMASK2 */
	{ RSTV0913_IRQMASK1,      0x00 },    /* IRQMASK1 */
	{ RSTV0913_IRQMASK0,      0x00 },    /* IRQMASK0 */
	{ RSTV0913_I2CCFG,        0x88 },    /* I2CCFG */
	{ RSTV0913_I2CRPT,        0x34 },    /* I2CRPT */
	{ RSTV0913_GPIO0CFG,      0x82 },    /* GPIO0CFG */
	{ RSTV0913_GPIO1CFG,      0x82 },    /* GPIO1CFG */
	{ RSTV0913_GPIO2CFG,      0x82 },    /* GPIO2CFG */
	{ RSTV0913_GPIO3CFG,      0x82 },    /* GPIO3CFG */
	{ RSTV0913_GPIO4CFG,      0x82 },    /* GPIO4CFG */
	{ RSTV0913_GPIO5CFG,      0x82 },    /* GPIO5CFG */
	{ RSTV0913_GPIO6CFG,      0x82 },    /* GPIO6CFG */
	{ RSTV0913_GPIO7CFG,      0x82 },    /* GPIO7CFG */
	{ RSTV0913_GPIO8CFG,      0x82 },    /* GPIO8CFG */
	{ RSTV0913_GPIO9CFG,      0x82 },    /* GPIO9CFG */
	{ RSTV0913_GPIO10CFG,     0x82 },    /* GPIO10CFG */
	{ RSTV0913_GPIO11CFG,     0x82 },    /* GPIO11CFG */
	{ RSTV0913_GPIO12CFG,     0x82 },    /* GPIO12CFG */
	{ RSTV0913_STRSTATUS1,    0x60 },    /* STRSTATUS1 */
	{ RSTV0913_STRSTATUS2,    0x71 },    /* STRSTATUS2 */
	{ RSTV0913_STRSTATUS3,    0x82 },    /* STRSTATUS3 */
	{ RSTV0913_FSKTFC2,       0x8c },    /* FSKTFC2 */
	{ RSTV0913_FSKTFC1,       0x45 },    /* FSKTFC1 */
	{ RSTV0913_FSKTFC0,       0xc9 },    /* FSKTFC0 */
	{ RSTV0913_FSKTDELTAF1,   0x01 },    /* FSKTDELTAF1 */
	{ RSTV0913_FSKTDELTAF0,   0x37 },    /* FSKTDELTAF0 */
	{ RSTV0913_FSKTCTRL,      0x08 },    /* FSKTCTRL */
	{ RSTV0913_FSKRFC2,       0x10 },    /* FSKRFC2 */
	{ RSTV0913_FSKRFC1,       0x45 },    /* FSKRFC1 */
	{ RSTV0913_FSKRFC0,       0xc9 },    /* FSKRFC0 */
	{ RSTV0913_FSKRK1,        0x38 },    /* FSKRK1 */
	{ RSTV0913_FSKRK2,        0x71 },    /* FSKRK2 */
	{ RSTV0913_FSKRAGCR,      0x28 },    /* FSKRAGCR */
	{ RSTV0913_FSKRAGC,       0xff },    /* FSKRAGC */
	{ RSTV0913_FSKRALPHA,     0x13 },    /* FSKRALPHA */
	{ RSTV0913_FSKRPLTH1,     0x90 },    /* FSKRPLTH1 */
	{ RSTV0913_FSKRPLTH0,     0xbe },    /* FSKRPLTH0 */
	{ RSTV0913_FSKRDF1,       0x80 },    /* FSKRDF1 */
	{ RSTV0913_FSKRDF0,       0x00 },    /* FSKRDF0 */
	{ RSTV0913_FSKRSTEPP,     0x58 },    /* FSKRSTEPP */
	{ RSTV0913_FSKRSTEPM,     0x6f },    /* FSKRSTEPM */
	{ RSTV0913_FSKRDET1,      0x00 },    /* FSKRDET1 */
	{ RSTV0913_FSKRDET0,      0x00 },    /* FSKRDET0 */
	{ RSTV0913_FSKRDTH1,      0x00 },    /* FSKRDTH1 */
	{ RSTV0913_FSKRDTH0,      0xe9 },    /* FSKRDTH0 */
	{ RSTV0913_FSKRLOSS,      0x4d },    /* FSKRLOSS */
	{ RSTV0913_NCOARSE,       0x39 },    /* NCOARSE */
	{ RSTV0913_NCOARSE1,      0x12 },    /* NCOARSE1 */
	{ RSTV0913_NCOARSE2,      0x04 },    /* NCOARSE2 */
	{ RSTV0913_SYNTCTRL,      0x02 },    /* SYNTCTRL */
	{ RSTV0913_FILTCTRL,      0x01 },    /* FILTCTRL */
	{ RSTV0913_PLLSTAT,       0x07 },    /* PLLSTAT */
	{ RSTV0913_STOPCLK1,      0x00 },    /* STOPCLK1 */
	{ RSTV0913_STOPCLK2,      0x00 },    /* STOPCLK2 */
	{ RSTV0913_PREGCTL,       0x00 },    /* PREGCTL */
	{ RSTV0913_TSTTNR0,       0x04 },    /* TSTTNR0 */
	{ RSTV0913_TSTTNR1,       0x26 },    /* TSTTNR1 */
	{ RSTV0913_TSTTNR2,       0x6b },    /* TSTTNR2 */
	{ RSTV0913_IQCONST,       0x00 },    /* IQCONST */
	{ RSTV0913_NOSCFG,        0x14 },    /* NOSCFG */
	{ RSTV0913_ISYMB,         0x28 },    /* ISYMB */
	{ RSTV0913_QSYMB,         0xd9 },    /* QSYMB */
	{ RSTV0913_AGC1CFG,       0x54 },    /* AGC1CFG */
	{ RSTV0913_AGC1CN,        0x99 },    /* AGC1CN */
	{ RSTV0913_AGC1REF,       0x58 },    /* AGC1REF */
	{ RSTV0913_IDCCOMP,       0xfb },    /* IDCCOMP */
	{ RSTV0913_QDCCOMP,       0x0d },    /* QDCCOMP */
	{ RSTV0913_POWERI,        0x99 },    /* POWERI */
	{ RSTV0913_POWERQ,        0x97 },    /* POWERQ */
	{ RSTV0913_AGC1AMM,       0x02 },    /* AGC1AMM */
	{ RSTV0913_AGC1QUAD,      0xfc },    /* AGC1QUAD */
	{ RSTV0913_AGCIQIN1,      0x55 },    /* AGCIQIN1 */
	{ RSTV0913_AGCIQIN0,      0xef },    /* AGCIQIN0 */
	{ RSTV0913_DEMOD,         0x04 },    /* DEMOD */
	{ RSTV0913_DMDMODCOD,     0x45 },    /* DMDMODCOD */
	{ RSTV0913_DSTATUS,       0xec },    /* DSTATUS */
	{ RSTV0913_DSTATUS2,      0x80 },    /* DSTATUS2 */
	{ RSTV0913_DMDCFGMD,      0x80 },    /* DMDCFGMD */
	{ RSTV0913_DMDCFG2,       0x3b },    /* DMDCFG2 */
	{ RSTV0913_DMDISTATE,     0x15 },    /* DMDISTATE */
	{ RSTV0913_DMDT0M,        0x40 },    /* DMDT0M */
	{ RSTV0913_DMDSTATE,      0xcc },    /* DMDSTATE */
	{ RSTV0913_DMDFLYW,       0x0f },    /* DMDFLYW */
	{ RSTV0913_DSTATUS3,      0x88 },    /* DSTATUS3 */
	{ RSTV0913_DMDCFG3,       0x08 },    /* DMDCFG3 */
	{ RSTV0913_DMDCFG4,       0x04 },    /* DMDCFG4 */
	{ RSTV0913_CORRELMANT,    0x78 },    /* CORRELMANT */
	{ RSTV0913_CORRELABS,     0x8c },    /* CORRELABS */
	{ RSTV0913_CORRELEXP,     0xaa },    /* CORRELEXP */
	{ RSTV0913_PLHMODCOD,     0x45 },    /* PLHMODCOD */
	{ RSTV0913_DMDREG,        0x01 },    /* DMDREG */
	{ RSTV0913_AGC2O,         0x5b },    /* AGC2O */
	{ RSTV0913_AGC2REF,       0x38 },    /* AGC2REF */
	{ RSTV0913_AGC1ADJ,       0x58 },    /* AGC1ADJ */
	{ RSTV0913_AGC2I1,        0x09 },    /* AGC2I1 */
	{ RSTV0913_AGC2I0,        0xf8 },    /* AGC2I0 */
	{ RSTV0913_CARCFG,        0x46 },    /* CARCFG */
	{ RSTV0913_ACLC,          0x2b },    /* ACLC */
	{ RSTV0913_BCLC,          0x1a },    /* BCLC */
	{ RSTV0913_CARFREQ,       0x79 },    /* CARFREQ */
	{ RSTV0913_CARHDR,        0x1c },    /* CARHDR */
	{ RSTV0913_LDT,           0xd0 },    /* LDT */
	{ RSTV0913_LDT2,          0xb8 },    /* LDT2 */
	{ RSTV0913_CFRICFG,       0xf8 },    /* CFRICFG */
	{ RSTV0913_CFRUP1,        0x0e },    /* CFRUP1 */
	{ RSTV0913_CFRUP0,        0x69 },    /* CFRUP0 */
	{ RSTV0913_CFRLOW1,       0xf1 },    /* CFRLOW1 */
	{ RSTV0913_CFRLOW0,       0x97 },    /* CFRLOW0 */
	{ RSTV0913_CFRINIT1,      0xff },    /* CFRINIT1 */
	{ RSTV0913_CFRINIT0,      0xf9 },    /* CFRINIT0 */
	{ RSTV0913_CFRINC1,       0x03 },    /* CFRINC1 */
	{ RSTV0913_CFRINC0,       0x8e },    /* CFRINC0 */
	{ RSTV0913_CFR2,          0xff },    /* CFR2 */
	{ RSTV0913_CFR1,          0xf9 },    /* CFR1 */
	{ RSTV0913_CFR0,          0x0a },    /* CFR0 */
	{ RSTV0913_LDI,           0x7f },    /* LDI */
	{ RSTV0913_TMGCFG,        0xd3 },    /* TMGCFG */
	{ RSTV0913_RTC,           0x68 },    /* RTC */
	{ RSTV0913_RTCS2,         0x68 },    /* RTCS2 */
	{ RSTV0913_TMGTHRISE,     0x1e },    /* TMGTHRISE */
	{ RSTV0913_TMGTHFALL,     0x08 },    /* TMGTHFALL */
	{ RSTV0913_SFRUPRATIO,    0x20 },    /* SFRUPRATIO */
	{ RSTV0913_SFRLOWRATIO,   0xd0 },    /* SFRLOWRATIO */
	{ RSTV0913_KREFTMG,       0x80 },    /* KREFTMG */
	{ RSTV0913_SFRSTEP,       0x88 },    /* SFRSTEP */
	{ RSTV0913_TMGCFG2,       0x80 },    /* TMGCFG2 */
	{ RSTV0913_KREFTMG2,      0x80 },    /* KREFTMG2 */
	{ RSTV0913_TMGCFG3,       0x06 },    /* TMGCFG3 */
	{ RSTV0913_SFRINIT1,      0x38 },    /* SFRINIT1 */
	{ RSTV0913_SFRINIT0,      0xe3 },    /* SFRINIT0 */
	{ RSTV0913_SFRUP1,        0x39 },    /* SFRUP1 */
	{ RSTV0913_SFRUP0,        0xc7 },    /* SFRUP0 */
	{ RSTV0913_SFRLOW1,       0x38 },    /* SFRLOW1 */
	{ RSTV0913_SFRLOW0,       0x39 },    /* SFRLOW0 */
	{ RSTV0913_SFR3,          0x38 },    /* SFR3 */
	{ RSTV0913_SFR2,          0xe3 },    /* SFR2 */
	{ RSTV0913_SFR1,          0x00 },    /* SFR1 */
	{ RSTV0913_SFR0,          0x00 },    /* SFR0 */
	{ RSTV0913_TMGREG2,       0x00 },    /* TMGREG2 */
	{ RSTV0913_TMGREG1,       0x34 },    /* TMGREG1 */
	{ RSTV0913_TMGREG0,       0xc9 },    /* TMGREG0 */
	{ RSTV0913_TMGLOCK1,      0x3f },    /* TMGLOCK1 */
	{ RSTV0913_TMGLOCK0,      0xae },    /* TMGLOCK0 */
	{ RSTV0913_TMGOBS,        0x10 },    /* TMGOBS */
	{ RSTV0913_EQUALCFG,      0x41 },    /* EQUALCFG */
	{ RSTV0913_EQUAI1,        0x00 },    /* EQUAI1 */
	{ RSTV0913_EQUAQ1,        0x04 },    /* EQUAQ1 */
	{ RSTV0913_EQUAI2,        0x00 },    /* EQUAI2 */
	{ RSTV0913_EQUAQ2,        0xff },    /* EQUAQ2 */
	{ RSTV0913_EQUAI3,        0x00 },    /* EQUAI3 */
	{ RSTV0913_EQUAQ3,        0x00 },    /* EQUAQ3 */
	{ RSTV0913_EQUAI4,        0x00 },    /* EQUAI4 */
	{ RSTV0913_EQUAQ4,        0xff },    /* EQUAQ4 */
	{ RSTV0913_EQUAI5,        0x00 },    /* EQUAI5 */
	{ RSTV0913_EQUAQ5,        0x00 },    /* EQUAQ5 */
	{ RSTV0913_EQUAI6,        0x00 },    /* EQUAI6 */
	{ RSTV0913_EQUAQ6,        0x00 },    /* EQUAQ6 */
	{ RSTV0913_EQUAI7,        0x00 },    /* EQUAI7 */
	{ RSTV0913_EQUAQ7,        0x00 },    /* EQUAQ7 */
	{ RSTV0913_EQUAI8,        0x00 },    /* EQUAI8 */
	{ RSTV0913_EQUAQ8,        0x00 },    /* EQUAQ8 */
	{ RSTV0913_NNOSDATAT1,    0x05 },    /* NNOSDATAT1 */
	{ RSTV0913_NNOSDATAT0,    0x23 },    /* NNOSDATAT0 */
	{ RSTV0913_NNOSDATA1,     0x00 },    /* NNOSDATA1 */
	{ RSTV0913_NNOSDATA0,     0x8e },    /* NNOSDATA0 */
	{ RSTV0913_NNOSPLHT1,     0x05 },    /* NNOSPLHT1 */
	{ RSTV0913_NNOSPLHT0,     0x1b },    /* NNOSPLHT0 */
	{ RSTV0913_NNOSPLH1,      0x00 },    /* NNOSPLH1 */
	{ RSTV0913_NNOSPLH0,      0x8c },    /* NNOSPLH0 */
	{ RSTV0913_NOSDATAT1,     0x04 },    /* NOSDATAT1 */
	{ RSTV0913_NOSDATAT0,     0x81 },    /* NOSDATAT0 */
	{ RSTV0913_NNOSFRAME1,    0x00 },    /* NNOSFRAME1 */
	{ RSTV0913_NNOSFRAME0,    0x8f },    /* NNOSFRAME0 */
	{ RSTV0913_NNOSRAD1,      0x00 },    /* NNOSRAD1 */
	{ RSTV0913_NNOSRAD0,      0x08 },    /* NNOSRAD0 */
	{ RSTV0913_NOSCFGF1,      0x00 },    /* NOSCFGF1 */
	{ RSTV0913_CAR2CFG,       0x06 },    /* CAR2CFG */
	{ RSTV0913_CFR2CFR1,      0xe5 },/*0x25*/    /* CFR2CFR1 */
	{ RSTV0913_CAR3CFG,       0xc0 },    /* CAR3CFG */
	{ RSTV0913_CFR22,         0xff },    /* CFR22 */
	{ RSTV0913_CFR21,         0xff },    /* CFR21 */
	{ RSTV0913_CFR20,         0xf0 },    /* CFR20 */
	{ RSTV0913_ACLC2S2Q,      0x2a },    /* ACLC2S2Q */
	{ RSTV0913_ACLC2S28,      0x2a },    /* ACLC2S28 */
	{ RSTV0913_ACLC2S216A,    0x49 },    /* ACLC2S216A */
	{ RSTV0913_ACLC2S232A,    0x48 },    /* ACLC2S232A */
	{ RSTV0913_BCLC2S2Q,      0x84 },    /* BCLC2S2Q */
	{ RSTV0913_BCLC2S28,      0x84 },    /* BCLC2S28 */
	{ RSTV0913_PLROOT2,       0x40 },    /* PLROOT2 */
	{ RSTV0913_PLROOT1,       0x00 },    /* PLROOT1 */
	{ RSTV0913_PLROOT0,       0x01 },    /* PLROOT0 */
	{ RSTV0913_MODCODLST1,    0xcf },    /* MODCODLST1 */
	{ RSTV0913_MODCODLST2,    0xff },    /* MODCODLST2 */
	{ RSTV0913_MODCODLST3,    0xff },    /* MODCODLST3 */
	{ RSTV0913_MODCODLST4,    0x3f },    /* MODCODLST4 */
	{ RSTV0913_MODCODLST5,    0xff },    /* MODCODLST5 */
	{ RSTV0913_MODCODLST6,    0xff },    /* MODCODLST6 */
	{ RSTV0913_MODCODLST7,    0xcc },    /* MODCODLST7 */
	{ RSTV0913_MODCODLST8,    0xcc },    /* MODCODLST8 */
	{ RSTV0913_MODCODLST9,    0xcc },    /* MODCODLST9 */
	{ RSTV0913_MODCODLSTA,    0xcc },    /* MODCODLSTA */
	{ RSTV0913_MODCODLSTB,    0xcc },    /* MODCODLSTB */
	{ RSTV0913_MODCODLSTC,    0xcc },    /* MODCODLSTC */
	{ RSTV0913_MODCODLSTD,    0xcc },    /* MODCODLSTD */
	{ RSTV0913_MODCODLSTE,    0xff },    /* MODCODLSTE */
	{ RSTV0913_MODCODLSTF,    0xfd },    /* MODCODLSTF */
	{ RSTV0913_GAUSSR0,       0x98 },    /* GAUSSR0 */
	{ RSTV0913_CCIR0,         0x30 },    /* CCIR0 */
	{ RSTV0913_CCIQUANT,      0xac },    /* CCIQUANT */
	{ RSTV0913_CCITHRES,      0x50 },    /* CCITHRES */
	{ RSTV0913_CCIACC,        0x0d },    /* CCIACC */
	{ RSTV0913_DSTATUS4,      0x65 },    /* DSTATUS4 */
	{ RSTV0913_DMDRESCFG,     0x29 },    /* DMDRESCFG */
	{ RSTV0913_DMDRESADR,     0x52 },    /* DMDRESADR */
	{ RSTV0913_DMDRESDATA7,   0x38 },    /* DMDRESDATA7 */
	{ RSTV0913_DMDRESDATA6,   0xe3 },    /* DMDRESDATA6 */
	{ RSTV0913_DMDRESDATA5,   0xff },    /* DMDRESDATA5 */
	{ RSTV0913_DMDRESDATA4,   0xfa },    /* DMDRESDATA4 */
	{ RSTV0913_DMDRESDATA3,   0xa6 },    /* DMDRESDATA3 */
	{ RSTV0913_DMDRESDATA2,   0x58 },    /* DMDRESDATA2 */
	{ RSTV0913_DMDRESDATA1,   0x01 },    /* DMDRESDATA1 */
	{ RSTV0913_DMDRESDATA0,   0xc5 },    /* DMDRESDATA0 */
	{ RSTV0913_FFEI1,         0xf9 },    /* FFEI1 */
	{ RSTV0913_FFEQ1,         0x05 },    /* FFEQ1 */
	{ RSTV0913_FFEI2,         0x02 },    /* FFEI2 */
	{ RSTV0913_FFEQ2,         0xfe },    /* FFEQ2 */
	{ RSTV0913_FFEI3,         0x00 },    /* FFEI3 */
	{ RSTV0913_FFEQ3,         0x01 },    /* FFEQ3 */
	{ RSTV0913_FFEI4,         0x00 },    /* FFEI4 */
	{ RSTV0913_FFEQ4,         0x00 },    /* FFEQ4 */
	{ RSTV0913_FFECFG,        0x71 },    /* FFECFG */
	{ RSTV0913_TNRCFG2,       0x02 },    /* TNRCFG2 */
	{ RSTV0913_SMAPCOEF7,     0x00 },    /* SMAPCOEF7 */
	{ RSTV0913_SMAPCOEF6,     0x00 },    /* SMAPCOEF6 */
	{ RSTV0913_SMAPCOEF5,     0x00 },    /* SMAPCOEF5 */
	{ RSTV0913_NOSTHRES1,     0x60 },    /* NOSTHRES1 */
	{ RSTV0913_NOSTHRES2,     0x69 },    /* NOSTHRES2 */
	{ RSTV0913_NOSDIFF1,      0x4c },    /* NOSDIFF1 */
	{ RSTV0913_RAINFADE,      0x35 },    /* RAINFADE */
	{ RSTV0913_NOSRAMCFG,     0x2c },    /* NOSRAMCFG */
	{ RSTV0913_NOSRAMPOS,     0x04 },    /* NOSRAMPOS */
	{ RSTV0913_NOSRAMVAL,     0xd4 },    /* NOSRAMVAL */
	{ RSTV0913_DMDPLHSTAT,    0x00 },    /* DMDPLHSTAT */
	{ RSTV0913_LOCKTIME3,     0x00 },    /* LOCKTIME3 */
	{ RSTV0913_LOCKTIME2,     0x00 },    /* LOCKTIME2 */
	{ RSTV0913_LOCKTIME1,     0x4d },    /* LOCKTIME1 */
	{ RSTV0913_LOCKTIME0,     0x88 },    /* LOCKTIME0 */
	{ RSTV0913_VITSCALE,      0x00 },    /* VITSCALE */
	{ RSTV0913_FECM,          0x01 },    /* FECM */
	{ RSTV0913_VTH12,         0xd7 },    /* VTH12 */
	{ RSTV0913_VTH23,         0x85 },    /* VTH23 */
	{ RSTV0913_VTH34,         0x58 },    /* VTH34 */
	{ RSTV0913_VTH56,         0x3a },    /* VTH56 */
	{ RSTV0913_VTH67,         0x34 },    /* VTH67 */
	{ RSTV0913_VTH78,         0x28 },    /* VTH78 */
	{ RSTV0913_VITCURPUN,     0x2d },    /* VITCURPUN */
	{ RSTV0913_VERROR,        0x00 },    /* VERROR */
	{ RSTV0913_PRVIT,         0x2f },    /* PRVIT */
	{ RSTV0913_VAVSRVIT,      0x00 },    /* VAVSRVIT */
	{ RSTV0913_VSTATUSVIT,    0x25 },    /* VSTATUSVIT */
	{ RSTV0913_VTHINUSE,      0x80 },    /* VTHINUSE */
	{ RSTV0913_KDIV12,        0x27 },    /* KDIV12 */
	{ RSTV0913_KDIV23,        0x32 },    /* KDIV23 */
	{ RSTV0913_KDIV34,        0x32 },    /* KDIV34 */
	{ RSTV0913_KDIV56,        0x32 },    /* KDIV56 */
	{ RSTV0913_KDIV67,        0x32 },    /* KDIV67 */
	{ RSTV0913_KDIV78,        0x50 },    /* KDIV78 */
	{ RSTV0913_PDELCTRL1,     0x00 },    /* PDELCTRL1 */
	{ RSTV0913_PDELCTRL2,     0x00 },    /* PDELCTRL2 */
	{ RSTV0913_HYSTTHRESH,    0x41 },    /* HYSTTHRESH */
	{ RSTV0913_ISIENTRY,      0x00 },    /* ISIENTRY */
	{ RSTV0913_ISIBITENA,     0x00 },    /* ISIBITENA */
	{ RSTV0913_MATSTR1,       0xf0 },    /* MATSTR1 */
	{ RSTV0913_MATSTR0,       0x00 },    /* MATSTR0 */
	{ RSTV0913_UPLSTR1,       0x05 },    /* UPLSTR1 */
	{ RSTV0913_UPLSTR0,       0xe0 },    /* UPLSTR0 */
	{ RSTV0913_DFLSTR1,       0xe3 },    /* DFLSTR1 */
	{ RSTV0913_DFLSTR0,       0x00 },    /* DFLSTR0 */
	{ RSTV0913_SYNCSTR,       0x47 },    /* SYNCSTR */
	{ RSTV0913_SYNCDSTR1,     0x03 },    /* SYNCDSTR1 */
	{ RSTV0913_SYNCDSTR0,     0xc0 },    /* SYNCDSTR0 */
	{ RSTV0913_PDELSTATUS1,   0x03 },    /* PDELSTATUS1 */
	{ RSTV0913_PDELSTATUS2,   0x45 },    /* PDELSTATUS2 */
	{ RSTV0913_BBFCRCKO1,     0x00 },    /* BBFCRCKO1 */
	{ RSTV0913_BBFCRCKO0,     0x00 },    /* BBFCRCKO0 */
	{ RSTV0913_UPCRCKO1,      0x00 },    /* UPCRCKO1 */
	{ RSTV0913_UPCRCKO0,      0x00 },    /* UPCRCKO0 */
	{ RSTV0913_TSSTATEM,      0xf0 },    /* TSSTATEM */
	{ RSTV0913_TSCFGH,        0x40 },    /* TSCFGH */
	{ RSTV0913_TSCFGM,        0x04 },    /* TSCFGM */
	{ RSTV0913_TSCFGL,        0x20 },    /* TSCFGL */
	{ RSTV0913_TSINSDELH,     0x00 },    /* TSINSDELH */
	{ RSTV0913_TSDIVN,        0x03 },    /* TSDIVN */
	{ RSTV0913_TSCFG4,        0x00 },    /* TSCFG4 */
	{ RSTV0913_TSSPEED,       0x33 },    /* TSSPEED */
	{ RSTV0913_TSSTATUS,      0x81 },    /* TSSTATUS */
	{ RSTV0913_TSSTATUS2,     0x68 },    /* TSSTATUS2 */
	{ RSTV0913_TSBITRATE1,    0x25 },    /* TSBITRATE1 */
	{ RSTV0913_TSBITRATE0,    0x1b },    /* TSBITRATE0 */
	{ RSTV0913_ERRCTRL1,      0x67 },    /* ERRCTRL1 */
	{ RSTV0913_ERRCNT12,      0x00 },    /* ERRCNT12 */
	{ RSTV0913_ERRCNT11,      0x00 },    /* ERRCNT11 */
	{ RSTV0913_ERRCNT10,      0x00 },    /* ERRCNT10 */
	{ RSTV0913_ERRCTRL2,      0xc1 },    /* ERRCTRL2 */
	{ RSTV0913_ERRCNT22,      0x00 },    /* ERRCNT22 */
	{ RSTV0913_ERRCNT21,      0x00 },    /* ERRCNT21 */
	{ RSTV0913_ERRCNT20,      0x00 },    /* ERRCNT20 */
	{ RSTV0913_FECSPY,        0xa8 },    /* FECSPY */
	{ RSTV0913_FSPYCFG,       0x2c },    /* FSPYCFG */
	{ RSTV0913_FSPYDATA,      0x3a },    /* FSPYDATA */
	{ RSTV0913_FSPYOUT,       0x07 },    /* FSPYOUT */
	{ RSTV0913_FSTATUS,       0x61 },    /* FSTATUS */
	{ RSTV0913_FBERCPT4,      0x00 },    /* FBERCPT4 */
	{ RSTV0913_FBERCPT3,      0x00 },    /* FBERCPT3 */
	{ RSTV0913_FBERCPT2,      0x58 },    /* FBERCPT2 */
	{ RSTV0913_FBERCPT1,      0x65 },    /* FBERCPT1 */
	{ RSTV0913_FBERCPT0,      0x6d },    /* FBERCPT0 */
	{ RSTV0913_FBERERR2,      0x00 },    /* FBERERR2 */
	{ RSTV0913_FBERERR1,      0x00 },    /* FBERERR1 */
	{ RSTV0913_FBERERR0,      0x00 },    /* FBERERR0 */
	{ RSTV0913_FSPYBER,       0x11 },    /* FSPYBER */
	{ RSTV0913_SFECSTATUS,    0x44 },    /* SFECSTATUS */
	{ RSTV0913_SFKDIV12,      0x1f },    /* SFKDIV12 */
	{ RSTV0913_SFKDIV23,      0x22 },    /* SFKDIV23 */
	{ RSTV0913_SFKDIV34,      0x24 },    /* SFKDIV34 */
	{ RSTV0913_SFKDIV56,      0x24 },    /* SFKDIV56 */
	{ RSTV0913_SFKDIV67,      0x29 },    /* SFKDIV67 */
	{ RSTV0913_SFKDIV78,      0x2c },    /* SFKDIV78 */
	{ RSTV0913_SFSTATUS,      0x46 },    /* SFSTATUS */
	{ RSTV0913_SFERRCTRL,     0x94 },    /* SFERRCTRL */
	{ RSTV0913_SFERRCNT2,     0x80 },    /* SFERRCNT2 */
	{ RSTV0913_SFERRCNT1,     0x00 },    /* SFERRCNT1 */
	{ RSTV0913_SFERRCNT0,     0x00 },    /* SFERRCNT0 */
	{ RSTV0913_TSGENERAL,     0x00 },    /* TSGENERAL */
	{ RSTV0913_DISIRQCFG,     0x00 },    /* DISIRQCFG */
	{ RSTV0913_DISIRQSTAT,    0x00 },    /* DISIRQSTAT */
	{ RSTV0913_DISTXCFG,      0x02 },    /* DISTXCFG */
	{ RSTV0913_DISTXSTATUS,   0x25 },    /* DISTXSTATUS */
	{ RSTV0913_DISTXBYTES,    0x00 },    /* DISTXBYTES */
	{ RSTV0913_DISTXFIFO,     0x00 },    /* DISTXFIFO */
	{ RSTV0913_DISTXF22,      0xc0 },    /* DISTXF22 */
	{ RSTV0913_DISTIMEOCFG,   0x02 },    /* DISTIMEOCFG */
	{ RSTV0913_DISTIMEOUT,    0x8c },    /* DISTIMEOUT */
	{ RSTV0913_DISRXCFG,      0x04 },    /* DISRXCFG */
	{ RSTV0913_DISRXSTAT1,    0x00 },    /* DISRXSTAT1 */
	{ RSTV0913_DISRXSTAT0,    0x00 },    /* DISRXSTAT0 */
	{ RSTV0913_DISRXBYTES,    0x00 },    /* DISRXBYTES */
	{ RSTV0913_DISRXPARITY1,  0x00 },    /* DISRXPARITY1 */
	{ RSTV0913_DISRXPARITY0,  0x00 },    /* DISRXPARITY0 */
	{ RSTV0913_DISRXFIFO,     0x00 },    /* DISRXFIFO */
	{ RSTV0913_DISRXDC1,      0x03 },    /* DISRXDC1 */
	{ RSTV0913_DISRXDC0,      0xec },    /* DISRXDC0 */
	{ RSTV0913_DISRXF221,     0x01 },    /* DISRXF221 */
	{ RSTV0913_DISRXF220,     0x2b },    /* DISRXF220 */
	{ RSTV0913_DISRXF100,     0xa9 },    /* DISRXF100 */
	{ RSTV0913_DISRXSHORT22K, 0x0f },    /* DISRXSHORT22K */
	{ RSTV0913_ACRPRESC,      0x01 },    /* ACRPRESC */
	{ RSTV0913_ACRDIV,        0x14 },    /* ACRDIV */
	{ RSTV0913_NBITER_NF4,    0x23 },    /* NBITER_NF4 */
	{ RSTV0913_NBITER_NF5,    0x1b },    /* NBITER_NF5 */
	{ RSTV0913_NBITER_NF6,    0x24 },    /* NBITER_NF6 */
	{ RSTV0913_NBITER_NF7,    0x23 },    /* NBITER_NF7 */
	{ RSTV0913_NBITER_NF8,    0x22 },    /* NBITER_NF8 */
	{ RSTV0913_NBITER_NF9,    0x21 },    /* NBITER_NF9 */
	{ RSTV0913_NBITER_NF10,   0x28 },    /* NBITER_NF10 */
	{ RSTV0913_NBITER_NF11,   0x28 },    /* NBITER_NF11 */
	{ RSTV0913_NBITER_NF12,   0x1b },    /* NBITER_NF12 */
	{ RSTV0913_NBITER_NF13,   0x24 },    /* NBITER_NF13 */
	{ RSTV0913_NBITER_NF14,   0x23 },    /* NBITER_NF14 */
	{ RSTV0913_NBITER_NF15,   0x21 },    /* NBITER_NF15 */
	{ RSTV0913_NBITER_NF16,   0x28 },    /* NBITER_NF16 */
	{ RSTV0913_NBITER_NF17,   0x28 },    /* NBITER_NF17 */
	{ RSTV0913_GAINLLR_NF4,   0x20 },    /* GAINLLR_NF4 */
	{ RSTV0913_GAINLLR_NF5,   0x20 },    /* GAINLLR_NF5 */
	{ RSTV0913_GAINLLR_NF6,   0x20 },    /* GAINLLR_NF6 */
	{ RSTV0913_GAINLLR_NF7,   0x20 },    /* GAINLLR_NF7 */
	{ RSTV0913_GAINLLR_NF8,   0x20 },    /* GAINLLR_NF8 */
	{ RSTV0913_GAINLLR_NF9,   0x20 },    /* GAINLLR_NF9 */
	{ RSTV0913_GAINLLR_NF10,  0x20 },    /* GAINLLR_NF10 */
	{ RSTV0913_GAINLLR_NF11,  0x20 },    /* GAINLLR_NF11 */
	{ RSTV0913_GAINLLR_NF12,  0x20 },    /* GAINLLR_NF12 */
	{ RSTV0913_GAINLLR_NF13,  0x20 },    /* GAINLLR_NF13 */
	{ RSTV0913_GAINLLR_NF14,  0x20 },    /* GAINLLR_NF14 */
	{ RSTV0913_GAINLLR_NF15,  0x20 },    /* GAINLLR_NF15 */
	{ RSTV0913_GAINLLR_NF16,  0x20 },    /* GAINLLR_NF16 */
	{ RSTV0913_GAINLLR_NF17,  0x20 },    /* GAINLLR_NF17 */
	{ RSTV0913_CFGEXT,        0x02 },    /* CFGEXT */
	{ RSTV0913_GENCFG,        0x14 },    /* GENCFG */
	{ RSTV0913_LDPCERR1,      0x00 },    /* LDPCERR1 */
	{ RSTV0913_LDPCERR0,      0x06 },    /* LDPCERR0 */
	{ RSTV0913_BCHERR,        0x00 },    /* BCHERR */
	{ RSTV0913_MAXEXTRAITER,  0x07 },    /* MAXEXTRAITER */
	{ RSTV0913_STATUSITER,    0x04 },    /* STATUSITER */
	{ RSTV0913_STATUSMAXITER, 0x2f },    /* STATUSMAXITER */
	{ RSTV0913_TSTRES0,       0x00 },    /* TSTRES0 */
	{ RSTV0913_TCTL4,         0x00 },    /* TCTL4 */
};

STCHIP_Error_t STV0913_Init(Demod_InitParams_t *InitParams,STCHIP_Handle_t *hChipHandle)
{
	STCHIP_Handle_t hChip;

	STCHIP_Error_t error;

   /* fill elements of external chip data structure */
	InitParams->Chip->NbRegs   = STV0913_NBREGS;
	InitParams->Chip->NbFields = STV0913_NBFIELDS;
	InitParams->Chip->ChipMode = STCHIP_MODE_SUBADR_16;
	InitParams->Chip->pData = NULL;

#ifdef HOST_PC
	(*hChipHandle) = ChipOpen(InitParams->Chip);
#endif

	hChip=(*hChipHandle);
	if (hChip != NULL)
	{
		/*******************************
		**   CHIP REGISTER MAP IMAGE INITIALIZATION
		**     ----------------------
		********************************/
		ChipUpdateDefaultValues(hChip,STV0913DefVal);

		/*Read the ID*/
		hChip->ChipID=ChipGetOneRegister(hChip,RSTV0913_MID);

		if (hChip->Error == CHIPERR_NO_ERROR)
		{
			/*******************************
			 **   REGISTER CONFIGURATION
			 **     ----------------------
			 ********************************/

		/* Continue the registers init sequence
			for (i = 0; i <STV0913_NBREGS; i++)
			{
				ChipSetOneRegister(hChip,STV0913DefVal[i].Addr,STV0913DefVal[i].Value);
			}
		*/

			ChipSetOneRegister(hChip,RSTV0913_GENCFG,0x14); /*single LDPC*/
			ChipSetOneRegister(hChip,RSTV0913_I2CRPT,0x34);
			ChipSetOneRegister(hChip,RSTV0913_SYNTCTRL,0x02);
			ChipSetOneRegister(hChip,RSTV0913_OUTCFG,0x00);
			ChipSetOneRegister(hChip,RSTV0913_TMGTHRISE,0x1E);

			ChipSetOneRegister(hChip,RSTV0913_CAR3CFG,0x02);

			/*LDPC reset*/
			ChipSetOneRegister(hChip,RSTV0913_TSTRES0,0x80);
			ChipSetOneRegister(hChip,RSTV0913_TSTRES0,0x00);
		}
		error = hChip->Error;
	}
	else
	{
		error = CHIPERR_INVALID_HANDLE;
	}
	return error;
}
// vim:ts=4
