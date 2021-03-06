/*****************************************************************************
 * Copyright (C)2011 FULAN. All Rights Reserved.
 *
 * File: nim_mn88472.h
 *
 * Description: Source file in LLD.
 *
 * History:
 *
 *    Date         Author        Version   Reason
 *    ============ ============= ========= =================
 * 1. 2011/12/12   DMQ           Ver 0.1   Create file.
 *
 *****************************************************************************/

#ifndef NIM_MN88472_H
#define NIM_MN88472_H

#include <types.h>
#include "nim_dev.h"
#include "nim_tuner.h"

#ifndef dprintk
#define dprintk(level, x...) do \
{ \
	if ((paramDebug) && ((paramDebug >= level) || level == 0)) \
	{ \
		printk(TAGDEBUG x); \
	} \
} while (0)
#endif

#define DMD_PLLSET1     0x00
#define DMD_PLLSET2     0x01
#define DMD_PLLSET      0x02
#define DMD_DTVSET      0x03
#define DMD_SYSSET      0x04
#define DMD_PWDSET      0x05
#define DMD_SPWDSET1    0x06
#define DMD_SPWDSET2    0x07
#define DMD_TSSET1      0x08
#define DMD_TSSET2      0x09
#define DMD_PLDWSET     0x0A
#define DMD_HIZSET1     0x0B
#define DMD_HIZSET2     0x0C
#define DMD_GPIOSET     0x0D
#define DMD_OUTSET1     0x0E
#define DMD_OUTSET2     0x0F
#define DMD_NCOFREQU    0x10
#define DMD_NCOFREQM    0x11
#define DMD_NCOFREQL    0x12
#define DMD_FADU        0x13
#define DMD_FADM        0x14
#define DMD_FADL        0x15
#define DMD_FSFTXU      0x16
#define DMD_FSFTXL      0x17
#define DMD_FSFTYU      0x18
#define DMD_FSFTYL      0x19
#define DMD_SSEQSET     0x1A
#define DMD_SSEQSTSET   0x1B
#define DMD_SSPRESET    0x1C
#define DMD_STTOSET1    0x1D
#define DMD_STTOSET2    0x1E
#define DMD_STTOSET3    0x1F
#define DMD_STTOSET4    0x20
#define DMD_SSEQTAU1    0x21
#define DMD_SSEQTAU2    0x22
#define DMD_UNLCKSET    0x23
#define DMD_FEFSET      0x24
#define DMD_WINSET1     0x25
#define DMD_WINSET2     0x26
#define DMD_WINPOSU     0x27
#define DMD_WINPOSL     0x28
#define DMD_WAFCSET1    0x29
#define DMD_WAFCSET2    0x2A
#define DMD_WAFCSET3    0x2B
#define DMD_CPESET      0x2C
#define DMD_TPSET1      0x2D
#define DMD_TPSET2      0x2E
#define DMD_DETSET      0x2F
#define DMD_FECSET1     0x30
#define DMD_FECSET2     0x31
#define DMD_PLPID       0x32
#define DMD_STMDSET1    0x33
#define DMD_STMDSET2    0x34
#define DMD_STMDSET3    0x35
#define DMD_STENSET1    0x36
#define DMD_STENSET2    0x37
#define DMD_P1SET1      0x38
#define DMD_P1SET2      0x39
#define DMD_P1SET3      0x3A
#define DMD_P1SET4      0x3B
#define DMD_TSYNCSET1   0x3C
#define DMD_TSYNCSET2   0x3D
#define DMD_TSYNCSET3   0x3E
#define DMD_DFCSET1     0x3F
#define DMD_DFCSET2     0x40
#define DMD_DFCSET3     0x41
#define DMD_DFSSET1     0x42
#define DMD_DFSSET2     0x43
#define DMD_DFSSET3     0x44
#define DMD_ANBICSET1   0x45
#define DMD_ANBICSET2   0x46
#define DMD_COFILSET    0x47
#define DMD_PHTHL       0x48
#define DMD_ICISET      0x49
#define DMD_WAFCSET4    0x4A
#define DMD_WAFCSET5    0x4B
#define DMD_ISISET1     0x4C
#define DMD_ISISET2     0x4D
#define DMD_ISISET3     0x4E
#define DMD_DETSET2     0x4F
#define DMD_DETSET3     0x50
#define DMD_DETQCSET    0x51
#define DMD_ADPTSET     0x52
#define DMD_DETIMPSET1  0x53
#define DMD_DETIMPSET2  0x54
#define DMD_DETIMPTH    0x55
#define DMD_DETNSELTH   0x56
#define DMD_DETRMVTH    0x57
#define DMD_ICICWSET    0x58
#define DMD_CSISET      0x59
#define DMD_CPECTSET    0x5A
#define DMD_FFILSET1    0x5B
#define DMD_FFILSET2    0x5C
#define DMD_FFSDTU      0x5D
#define DMD_FFSDTL      0x5E
#define DMD_WPSRCSET1   0x5F
#define DMD_WPSRCSET2   0x60
#define DMD_CFIWAIT     0x61
#define DMD_CFICNTL     0x62
#define DMD_CFIGAIN     0x63
#define DMD_CFIOSET     0x64
#define DMD_CFIOCG      0x65
#define DMD_CFIBLEND    0x66
#define DMD_CFIMONSET1  0x67
#define DMD_CFIMONSET2  0x68
#define DMD_CFIREFMANU  0x69
#define DMD_CFIREFMANL  0x6A
#define DMD_DINTSET1    0x6B
#define DMD_DINTSET2    0x6C
#define DMD_DINTSET3    0x6D
#define DMD_DMPSET1     0x6E
#define DMD_DMPSET2     0x6F
#define DMD_DMPSET3     0x70
#define DMD_CSICOEF1    0x71
#define DMD_CSICOEF2    0x72
#define DMD_LLRSET1     0x73
#define DMD_LLRSET2     0x74
#define DMD_LDPCSET     0x75
#define DMD_LDITMAX1    0x76
#define DMD_LDITMAX2    0x77
#define DMD_LDITMAX3    0x78
#define DMD_LDITMAX4    0x79
#define DMD_TSFSET1     0x7A
#define DMD_TSFSET2     0x7B
#define DMD_TSFSET3     0x7C
#define DMD_TSFSET4     0x7D
#define DMD_TSFCLKSET   0x7E
#define DMD_TSFCLKDT    0x7F
#define DMD_TSFFLGSET   0x80
#define DMD_TSFDBGSET   0x81
#define DMD_TSFDBGRD    0x82
#define DMD_AGCRDSET    0x83
#define DMD_SYN1RDSET   0x84
#define DMD_SYN2RDSET   0x85
#define DMD_ISIRDSET    0x86
#define DMD_DETRDSET1   0x87
#define DMD_DETRDSET2   0x88
#define DMD_CNSET       0x89
#define DMD_BERSET      0x8A
#define DMD_BERLEN      0x8B
#define DMD_NERRFSET    0x8C
#define DMD_AGCFLG      0x8D
#define DMD_AGCRDU      0x8E
#define DMD_AGCRDL      0x8F
#define DMD_DAGCRD      0x90
#define DMD_IMPRD       0x91
#define DMD_SSEQFLG     0x92
#define DMD_SSEQRD1     0x93
#define DMD_SSEQRD2     0x94
#define DMD_P1FLG       0x95
#define DMD_CL1RD       0x96
#define DMD_DL1RD       0x97
#define DMD_L1POSRD     0x98
#define DMD_SYN1FLG     0x99
#define DMD_SYN2FLG     0x9A
#define DMD_DETFLG      0x9B
#define DMD_FECFLG      0x9C
#define DMD_TSFLG       0x9D
#define DMD_TSFFLG      0x9E
#define DMD_TSSTATUS    0x9F
#define DMD_P1RDU       0xA0
#define DMD_P1RDL       0xA1
#define DMD_SYN1RDU     0xA2
#define DMD_SYN1RDL     0xA3
#define DMD_SYN2RDU     0xA4
#define DMD_SYN2RDL     0xA5
#define DMD_ISIRDU      0xA6
#define DMD_ISIRDL      0xA7
#define DMD_DETRDU      0xA8
#define DMD_DETRDM      0xA9
#define DMD_DETRDL      0xAA
#define DMD_CFIMON1RD   0xAB
#define DMD_CFIMON2RD   0xAC
#define DMD_TDIRDU      0xAD
#define DMD_TDIRDL      0xAE
#define DMD_TDISTRDU    0xAF
#define DMD_TDISTRDL    0xB0
#define DMD_CDIRDU      0xB1
#define DMD_CDIRDL      0xB2
#define DMD_LDRD        0xB3
#define DMD_FECCHRD1    0xB4
#define DMD_FECCHRD2    0xB5
#define DMD_TSFRDU      0xB6
#define DMD_TSFRDM      0xB7
#define DMD_TSFRDL      0xB8
#define DMD_DJBRDU      0xB9
#define DMD_DJBRDM      0xBA
#define DMD_DJBRDL      0xBB
#define DMD_CNFLG       0xBC
#define DMD_CNRDU       0xBD
#define DMD_CNRDL       0xBE
#define DMD_BERFLG      0xBF
#define DMD_BERRDU      0xC0
#define DMD_BERRDM      0xC1
#define DMD_BERRDL      0xC2
#define DMD_ERRFLG      0xC3
#define DMD_TPDSET1     0xC4
#define DMD_TPDSET2     0xC5
#define DMD_TPD1        0xC6
#define DMD_TPD2        0xC7
#define DMD_TPD3        0xC8
#define DMD_TPD4        0xC9
#define DMD_TPD5        0xCA
#define DMD_TPD6        0xCB
#define DMD_TPD7        0xCC
#define DMD_TPD8        0xCD
#define DMD_PRESET1     0xCE
#define DMD_PRESET2     0xCF
#define DMD_PRESET3     0xD0
#define DMD_PRESET4     0xD1
#define DMD_PRESET5     0xD2
#define DMD_DOSET1      0xD3
#define DMD_DOSET2      0xD4
#define DMD_DOSET3      0xD5
#define DMD_FLGSET_X    0xD6
#define DMD_FLGSET_T    0xD7
#define DMD_FLGSET_C    0xD8
#define DMD_FLGSET      0xD9
#define DMD_INVSET      0xDA
#define DMD_INTSET      0xDB
#define DMD_INTDEF1_X   0xDC
#define DMD_INTDEF2_X   0xDD
#define DMD_IFLGSET1_X  0xDE
#define DMD_IFLGSET2_X  0xDF
#define DMD_INTDEF1_T   0xE0
#define DMD_INTDEF2_T   0xE1
#define DMD_IFLGSET1_T  0xE2
#define DMD_IFLGSET2_T  0xE3
#define DMD_INTDEF1_C   0xE4
#define DMD_INTDEF2_C   0xE5
#define DMD_IFLGSET_C   0xE6
#define DMD_INTCND      0xE7
#define DMD_INTST       0xE8
#define DMD_GPOSET      0xE9
#define DMD_GPDTU       0xEA
#define DMD_GPDTL       0xEB
#define DMD_TCBSET      0xEC
#define DMD_TCBRT       0xED
#define DMD_TCBADR      0xEE
#define DMD_TCBDT0      0xEF
#define DMD_TCBDT1      0xF0
#define DMD_TCBDT2      0xF1
#define DMD_TCBDT3      0xF2
#define DMD_TCBDT4      0xF3
#define DMD_TCBDT5      0xF4
#define DMD_TCBDT6      0xF5
#define DMD_TCBDT7      0xF6
#define DMD_TCBCOM      0xF7
#define DMD_RSTSET1     0xF8
#define DMD_RSTSET2     0xF9
#define DMD_RSTSET3     0xFA
#define DMD_I2CSET      0xFB
#define DMD_PSEQOP1     0xFC
#define DMD_PSEQOP2     0xFD
#define DMD_PSEQCOM     0xFE
#define DMD_CHIPRD      0xFF

#define DMD_MDSET_T      0x00
#define DMD_MDASET_T     0x01
#define DMD_MDDEF0_T     0x02
#define DMD_MDDEF1_T     0x03
#define DMD_MDDEF2_T     0x04
#define DMD_SSEQSTSET_T  0x05
#define DMD_STTOSET1_T   0x06
#define DMD_STTOSET2_T   0x07
#define DMD_STTOSET3_T   0x08
#define DMD_SSEQTAU1_T   0x09
#define DMD_SSEQTAU2_T   0x0A
#define DMD_SSEQTAU3_T   0x0B
#define DMD_MGSET_T      0x0C
#define DMD_MGDTH_T      0x0D
#define DMD_WINSET1_T    0x0E
#define DMD_WINSET2_T    0x0F
#define DMD_TPSET_T      0x10
#define DMD_FRMSET_T     0x11
#define DMD_SPSYNCSET_T  0x12
#define DMD_DETSET_T     0x13
#define DMD_FECSET1_T    0x14
#define DMD_FECSET2_T    0x15
#define DMD_RSDSET_T     0x16
#define DMD_MFECSET_T    0x17
#define DMD_TSFSET_T     0x18
#define DMD_STMDSET1_T   0x19
#define DMD_STMDSET2_T   0x1A
#define DMD_STENSET1_T   0x1B
#define DMD_STENSET2_T   0x1C
#define DMD_TSYNCSET1_T  0x1D
#define DMD_TSYNCSET2_T  0x1E
#define DMD_TSYNCSET3_T  0x1F
#define DMD_TSYNCSET4_T  0x20
#define DMD_DFCSET1_T    0x21
#define DMD_DFCSET2_T    0x22
#define DMD_DFCSET3_T    0x23
#define DMD_DFCSET4_T    0x24
#define DMD_DFCSET5_T    0x25
#define DMD_DFSSET1_T    0x26
#define DMD_DFSSET2_T    0x27
#define DMD_DFSSET3_T    0x28
#define DMD_DFSSET4_T    0x29
#define DMD_DTSET1_T     0x2A
#define DMD_DTSET2_T     0x2B
#define DMD_DTSET3_T     0x2C
#define DMD_DTSET4_T     0x2D
#define DMD_DTSET5_T     0x2E
#define DMD_ANBICSET1_T  0x2F
#define DMD_ANBICSET2_T  0x30
#define DMD_COFILSET_T   0x31
#define DMD_SYMTCNT_T    0x32
#define DMD_PHTHL_T      0x33
#define DMD_OFLKSET_T    0x34
#define DMD_OFDMTHL_T    0x35
#define DMD_OFDMTHH_T    0x36
#define DMD_JMPTHR1_T    0x37
#define DMD_JMPTHR2_T    0x38
#define DMD_IIRSET_T     0x39
#define DMD_ICISET_T     0x3A
#define DMD_SYMSTSET_T   0x3B
#define DMD_FGAINSET_T   0x3C
#define DMD_CPEQESET_T   0x3D
#define DMD_DETSET2_T    0x3E
#define DMD_DETSET3_T    0x3F
#define DMD_DETQCSET_T   0x40
#define DMD_ADPTSET_T    0x41
#define DMD_CSISET_T     0x42
#define DMD_ICICWSET_T   0x43
#define DMD_ICICWSET2_T  0x44
#define DMD_FFILSET1_T   0x45
#define DMD_FFILSET2_T   0x46
#define DMD_FFSDT_T      0x47
#define DMD_WPSRCSET1_T  0x48
#define DMD_WPSRCSET2_T  0x49
#define DMD_WPCSET_T     0x4A
#define DMD_WPCDT_T      0x4B
#define DMD_CPISET_T     0x4C
#define DMD_MPTHU_T      0x4D
#define DMD_MPTHM_T      0x4E
#define DMD_MPTHL_T      0x4F
#define DMD_CPICLPA_T    0x50
#define DMD_CPICLPB_T    0x51
#define DMD_CPICLPC_T    0x52
#define DMD_CPICLPD_T    0x53
#define DMD_CPICLPE_T    0x54
#define DMD_CPICLPF_T    0x55
#define DMD_CFISET1_T    0x56
#define DMD_CFISET2_T    0x57
#define DMD_CFISET3_T    0x58
#define DMD_CFIOCTHU_T   0x59
#define DMD_CFIOCTHL_T   0x5A
#define DMD_CFIOCGU_T    0x5B
#define DMD_CFIOCGM_T    0x5C
#define DMD_CFIOCGL_T    0x5D
#define DMD_CPSFSET1_T   0x5E
#define DMD_CPSFSET2_T   0x5F
#define DMD_CFICLPVAL_T  0x60
#define DMD_CWDETTH_T    0x61
#define DMD_DMPSET1_T    0x62
#define DMD_DMPSET2_T    0x63
#define DMD_CLPRNK1_T    0x64
#define DMD_CLPRNK2_T    0x65
#define DMD_CLPRNK3_T    0x66
#define DMD_CLPRNK4_T    0x67
#define DMD_CLPRNK5_T    0x68
#define DMD_CLPRNK6_T    0x69
#define DMD_COCHSET1_T   0x6A
#define DMD_COCHSET2_T   0x6B
#define DMD_COCHSET3_T   0x6C
#define DMD_TFECDLY1_T   0x6D
#define DMD_TFECDLY2_T   0x6E
#define DMD_TFECDLY3_T   0x6F
#define DMD_SYN1RDSET_T  0x70
#define DMD_SYN2RDSET_T  0x71
#define DMD_ISIRDSET_T   0x72
#define DMD_DETRDSET1_T  0x73
#define DMD_DETRDSET2_T  0x74
#define DMD_CNSET_T      0x75
#define DMD_CNASET_T     0x76
#define DMD_CNBSET_T     0x77
#define DMD_BERSET1_T    0x78
#define DMD_BERSET2_T    0x79
#define DMD_BERSET3_T    0x7A
#define DMD_BERSTSET_T   0x7B
#define DMD_NERRFSET_T   0x7C
#define DMD_BERRDSET_T   0x7D
#define DMD_MDRD_T       0x7E
#define DMD_SSEQRD_T     0x7F
#define DMD_SYN1FLG_T    0x80
#define DMD_SYN2FLG_T    0x81
#define DMD_DETFLG_T     0x82
#define DMD_FECFLG_T     0x83
#define DMD_SYN1RDU_T    0x84
#define DMD_SYN1RDL_T    0x85
#define DMD_SYN2RDU_T    0x86
#define DMD_SYN2RDL_T    0x87
#define DMD_TMCCD1_T     0x88
#define DMD_TMCCD2_T     0x89
#define DMD_TMCCD3_T     0x8A
#define DMD_TMCCD4_T     0x8B
#define DMD_TMCCD5_T     0x8C
#define DMD_TMCCD6_T     0x8D
#define DMD_TMCCD7_T     0x8E
#define DMD_TMCCD8_T     0x8F
#define DMD_CELLIDU_T    0x90
#define DMD_CELLIDL_T    0x91
#define DMD_DVBH_SIG_T   0x92
#define DMD_ISIRDU_T     0x93
#define DMD_ISIRDL_T     0x94
#define DMD_DETRDU_T     0x95
#define DMD_DETRDM_T     0x96
#define DMD_DETRDL_T     0x97
#define DMD_MPRD_T       0x98
#define DMD_CWRD_T       0x99
#define DMD_CSIRD_T      0x9A
#define DMD_CNFLG_T      0x9B
#define DMD_CNRDU_T      0x9C
#define DMD_CNRDL_T      0x9D
#define DMD_BERFLG_T     0x9E
#define DMD_BERRDU_T     0x9F
#define DMD_BERRDM_T     0xA0
#define DMD_BERRDL_T     0xA1
#define DMD_BERLENRDU_T  0xA2
#define DMD_BERLENRDL_T  0xA3
#define DMD_ERRFLG_T     0xA4
#define DMD_CTSET1_T     0xA5
#define DMD_CTSET2_T     0xA6
#define DMD_CTRDU_T      0xA7
#define DMD_CTRDL_T      0xA8
#define DMD_BERSET1_P_T  0xA9
#define DMD_BERSET2_P_T  0xAA
#define DMD_BERRDSET_P_T 0xAB
#define DMD_BERRD_P_T    0xAC
#define DMD_CLKSET1      0xAE
#define DMD_CLKSET2      0xAF
#define DMD_ADCSET1      0xB0
#define DMD_ADCSET2      0xB1
#define DMD_ADCSET3      0xB2
#define DMD_ADCSET4      0xB3
#define DMD_AGCREF       0xB4
#define DMD_AGCSET1      0xB5
#define DMD_AGCSET2      0xB6
#define DMD_AGCLFGDA     0xB7
#define DMD_AGCLFGIA     0xB8
#define DMD_AGCLFGDS     0xB9
#define DMD_AGCLFGIS     0xBA
#define DMD_AGCLFSET1    0xBB
#define DMD_AGCLFSET2    0xBC
#define DMD_AGCLFSET3    0xBD
#define DMD_AGCWD1       0xBE
#define DMD_AGCWD2       0xBF
#define DMD_AGCFLGSET1   0xC0
#define DMD_AGCFLGSET2   0xC1
#define DMD_UNDERTHR     0xC2
#define DMD_OVERTHR      0xC3
#define DMD_AGCDB1       0xC4
#define DMD_AGCDB2       0xC5
#define DMD_AGCDB3       0xC6
#define DMD_AGCV0RF      0xC7
#define DMD_AGCV0IF      0xC8
#define DMD_AGCV1        0xC9
#define DMD_AGCV2        0xCA
#define DMD_AGCV3        0xCB
#define DMD_DCCSET       0xCC
#define DMD_IMPSET       0xCD
#define DMD_MGAINSET     0xCE
#define DMD_MIPFSET      0xCF
#define DMD_FDLY1        0xD0
#define DMD_FDLY2        0xD1
#define DMD_TDAGCSET     0xD2
#define DMD_TDAGCREF     0xD3
#define DMD_FDAGCSET     0xD4
#define DMD_FDAGCREF     0xD5
#define DMD_TGAINSET     0xD6
#define DMD_FFTSET       0xD7
#define DMD_FFTGSET      0xD8
#define DMD_FIFOSET      0xD9
#define DMD_PKTFSET      0xDA
#define DMD_PKTFRATE     0xDB
#define DMD_PKTFOFS      0xDC
#define DMD_DOFSET       0xDD
#define DMD_PERSET       0xDE
#define DMD_PERLEN       0xDF
#define DMD_PERFLG       0xE0
#define DMD_PERRDU       0xE1
#define DMD_PERRDL       0xE2
#define DMD_PERLENRDU    0xE3
#define DMD_PERLENRDL    0xE4
#define DMD_PKTFRD       0xE5
#define DMD_CTSET        0xE6
#define DMD_CTCARU       0xE7
#define DMD_CTCARL       0xE8
#define DMD_CTRDU        0xE9
#define DMD_CTRDL        0xEA
#define DMD_TESTSET1     0xEC
#define DMD_TESTSET2     0xED
#define DMD_TESTSET3     0xEE
#define DMD_I2CSET_T     0xEF
#define DMD_PSEQCTRL     0xF0
#define DMD_PSEQOP1_T    0xF1
#define DMD_PSEQOP2_T    0xF2
#define DMD_PSEQOP3_T    0xF3
#define DMD_PSEQOP4_T    0xF4
#define DMD_PSEQSET      0xF5
#define DMD_PSEQPRG      0xF6
#define DMD_PSEQDBG      0xF7
#define DMD_PSEQFLG      0xF8
#define DMD_PSEQADRU     0xF9
#define DMD_PSEQADRL     0xFA
#define DMD_PSEQDTSET    0xFB
#define DMD_PSEQDT       0xFC
#define DMD_PSEQTBASE    0xFD
#define DMD_PSEQCOM_T    0xFE
#define DMD_CHIPRD_T     0xFF

#define DMD_CHSRCHSET_C     0x00
#define DMD_SSEQSTSET_C     0x01
#define DMD_SRCHSET1_C      0x02
#define DMD_SRCHSET2_C      0x03
#define DMD_SRCHSET3_C      0x04
#define DMD_SSEQRSTSET1_C   0x05
#define DMD_SSEQRSTSET2_C   0x06
#define DMD_STTOSET_C       0x07
#define DMD_NCOSTEPU_C      0x08
#define DMD_NCOSTEPM_C      0x09
#define DMD_NCOSTEPL_C      0x0A
#define DMD_FRMTOSET0_C     0x0B
#define DMD_FRMTOSET1_C     0x0C
#define DMD_FRMTOSET2_C     0x0D
#define DMD_FRMTOSET3_C     0x0E
#define DMD_FRMTOSET4_C     0x0F
#define DMD_BAUDTHRU_C      0x10
#define DMD_BAUDTHRL_C      0x11
#define DMD_BAUDMAXU_C      0x12
#define DMD_BAUDMAXL_C      0x13
#define DMD_BAUDMINU_C      0x14
#define DMD_BAUDMINL_C      0x15
#define DMD_BAUDGAPU_C      0x16
#define DMD_BAUDGAPL_C      0x17
#define DMD_SRATEIN1_C      0x18
#define DMD_SRATEIN2_C      0x19
#define DMD_SRATEIN3_C      0x1A
#define DMD_SRATEIN4_C      0x1B
#define DMD_RATE1U_C        0x1C
#define DMD_RATE1L_C        0x1D
#define DMD_RATE2U_C        0x1E
#define DMD_RATE2L_C        0x1F
#define DMD_FSFTSET1_C      0x20
#define DMD_FSFTSET2_C      0x21
#define DMD_FSFTSET3_C      0x22
#define DMD_SYNCSET1_C      0x23
#define DMD_SYNCSET2_C      0x24
#define DMD_CAFCTHR_C       0x25
#define DMD_RATE3U_C        0x26
#define DMD_RATE3L_C        0x27
#define DMD_RATE4U_C        0x28
#define DMD_RATE4L_C        0x29
#define DMD_PASB1_C         0x2A
#define DMD_PASB2_C         0x2B
#define DMD_PASB3_C         0x2C
#define DMD_PASB4_C         0x2D
#define DMD_RATESET1_C      0x2E
#define DMD_RATESET2_C      0x2F
#define DMD_RATESET3_C      0x30
#define DMD_RATESET4_C      0x31
#define DMD_RATESET5_C      0x32
#define DMD_DMDPRM1_C       0x33
#define DMD_DMDPRM2_C       0x34
#define DMD_DMDPRM3_C       0x35
#define DMD_DMDPRM4_C       0x36
#define DMD_DMDPRM5_C       0x37
#define DMD_DMDPRM6_C       0x38
#define DMD_DMDPRM7_C       0x39
#define DMD_DMDPRM8_C       0x3A
#define DMD_DMDPRM9_C       0x3B
#define DMD_DMDPRM10_C      0x3C
#define DMD_DMDPRM11_C      0x3D
#define DMD_DMDPRM12_C      0x3E
#define DMD_DMDPRM13_C      0x3F
#define DMD_DMDPRM14_C      0x40
#define DMD_DMDPRM15_C      0x41
#define DMD_DMDPRM16_C      0x42
#define DMD_ERRCNT1_C       0x43
#define DMD_ERRCNT2_C       0x44
#define DMD_ERRCNT3_C       0x45
#define DMD_ERRCNT4_C       0x46
#define DMD_ERRCNT5_C       0x47
#define DMD_ERRCNT6_C       0x48
#define DMD_ERRCNT7_C       0x49
#define DMD_ERRCNT8_C       0x4A
#define DMD_ERRCNT9_C       0x4B
#define DMD_ERRCNT10_C      0x4C
#define DMD_ERRCNT11_C      0x4D
#define DMD_ERRCNT12_C      0x4E
#define DMD_ERRCNT13_C      0x4F
#define DMD_ERRCNT14_C      0x50
#define DMD_QCAFCADD2U_C    0x51
#define DMD_QCAFCADD2L_C    0x52
#define DMD_QCAFCSET1_C     0x53
#define DMD_QCAFCSET2_C     0x54
#define DMD_QCAFCSET3_C     0x55
#define DMD_QCAFCSET4_C     0x56
#define DMD_QCAFCSET5_C     0x57
#define DMD_QCAFCSET6_C     0x58
#define DMD_QCAFCSET7_C     0x59
#define DMD_QCAFCSET8_C     0x5A
#define DMD_GMIN_C          0x5B
#define DMD_QDAGCREF_C      0x5C
#define DMD_DAGCCOQ_C       0x5D
#define DMD_AFCSET1_C       0x5E
#define DMD_AFCCO_C         0x5F
#define DMD_AFCADD_C        0x60
#define DMD_APCSET1_C       0x61
#define DMD_APCSET2_C       0x62
#define DMD_APCSET4_C       0x63
#define DMD_APCSET5_C       0x64
#define DMD_APCCO1_C        0x65
#define DMD_APCCO2_C        0x66
#define DMD_APCCO3_C        0x67
#define DMD_APCCO4_C        0x68
#define DMD_CKRCO1_C        0x69
#define DMD_CKRCO2_C        0x6A
#define DMD_CKRSET1_C       0x6B
#define DMD_CKRSET2_C       0x6C
#define DMD_CKRLDFBN_C      0x6D
#define DMD_CKRLDTH_C       0x6E
#define DMD_CKRLDSET_C      0x6F
#define DMD_PNTSET_C        0x70
#define DMD_PNTCO_C         0x71
#define DMD_QDETSET1_C      0x72
#define DMD_QDETSET2_C      0x73
#define DMD_EQSET_C         0x74
#define DMD_QEQSET_C        0x75
#define DMD_QDETREF_C       0x76
#define DMD_PIRETH3U_C      0x77
#define DMD_PIRETH3L_C      0x78
#define DMD_NRFRSEL_C       0x79
#define DMD_CMFSET10_C      0x7A
#define DMD_DAGCSET_C       0x7B
#define DMD_VDAGCREF_C      0x7C
#define DMD_EQMNSET_C       0x80
#define DMD_CNSET_C         0x81
#define DMD_IBTGSET_C       0x82
#define DMD_SSEQMON1_C      0x83
#define DMD_SSEQMON2_C      0x84
#define DMD_STSMON_C        0x85
#define DMD_RATERDU_C       0x86
#define DMD_RATERDM_C       0x87
#define DMD_RATERDL_C       0x88
#define DMD_DMDSTSMON1_C    0x89
#define DMD_DMDSTSMON2_C    0x8A
#define DMD_AFCMON1_C       0x8B
#define DMD_AFCMON2_C       0x8C
#define DMD_AFCMON3_C       0x8D
#define DMD_AFCMON4_C       0x8E
#define DMD_CKRMON1_C       0x8F
#define DMD_CKRMON2_C       0x90
#define DMD_CKRMON3_C       0x91
#define DMD_APCMON1_C       0x92
#define DMD_APCMON2_C       0x93
#define DMD_APCMON3_C       0x94
#define DMD_APCMON4_C       0x95
#define DMD_PNTMON1_C       0x96
#define DMD_PNTMON2_C       0x97
#define DMD_PNTMON3_C       0x98
#define DMD_PNTMON4_C       0x99
#define DMD_DAGCMON1_C      0x9A
#define DMD_DAGCMON2_C      0x9B
#define DMD_IMON1_C         0x9C
#define DMD_IMON2_C         0x9D
#define DMD_QMON1_C         0x9E
#define DMD_QMON2_C         0x9F
#define DMD_CNFLG_C         0xA0
#define DMD_CNMON1_C        0xA1
#define DMD_CNMON2_C        0xA2
#define DMD_CNMON3_C        0xA3
#define DMD_CNMON4_C        0xA4
#define DMD_RATEMON_C       0xA5
#define DMD_FILMON_C        0xA6
#define DMD_AFCMON_C        0xA7
#define DMD_CKRMON_C        0xA8
#define DMD_APCMON_C        0xA9
#define DMD_PNTMON_C        0xAA
#define DMD_DAGCMON_C       0xAB
#define DMD_IMON_C          0xAC
#define DMD_QMON_C          0xAD
#define DMD_QCAFCCNT_C      0xAE
#define DMD_CKRFEMON_C      0xAF
#define DMD_EQERR1_C        0xB0
#define DMD_EQERR2_C        0xB1
#define DMD_EQCOEF_C        0xB2
#define DMD_APCERR_C        0xB3
#define DMD_PNTERR_C        0xB4
#define DMD_CNFLGP_C        0xB5
#define DMD_CNMON1P_C       0xB6
#define DMD_CNMON2P_C       0xB7
#define DMD_CNMON3P_C       0xB8
#define DMD_CNMON4P_C       0xB9
#define DMD_DAT_TTO_OFFSETU 0xF0
#define DMD_DAT_TTO_OFFSETM 0xF1
#define DMD_DAT_TTO_OFFSETL 0xF2
#define DMD_COM_TTO_OFFSETU 0xF3
#define DMD_COM_TTO_OFFSETM 0xF4
#define DMD_COM_TTO_OFFSETL 0xF5
#define DMD_FECSET3         0xF6
#define DMD_I2CSET          0xFB
#define DMD_PSEQOP1_C       0xFC
#define DMD_PSEQOP2_C       0xFD
#define DMD_PSEQCOM_C       0xFE
#define DMD_CHIPRD_C        0xFF
                              
/* **************************************************** */
/* DMD Information                                      */
/* **************************************************** */
/*! common information enum */
typedef enum
{
	DMD_E_INFO_ALL       =  0,
	DMD_E_INFO_REGREV    =  1,
	DMD_E_INFO_PSEQREV   =  2,
	DMD_E_INFO_SYSTEM    =  3,
	DMD_E_INFO_LOCK      =  4,
	DMD_E_INFO_AGC       =  5,
	DMD_E_INFO_BERRNUM   =  6,
	DMD_E_INFO_BITNUM    =  7,
	DMD_E_INFO_CNR_INT   =  8,
	DMD_E_INFO_CNR_DEC   =  9,
	DMD_E_INFO_PERRNUM   = 10,
	DMD_E_INFO_PACKETNUM = 11,
	DMD_E_INFO_STATUS    = 12,
	DMD_E_INFO_ERRORFREE = 13,
	DMD_E_INFO_COMMON_END_OF_INFORMATION
} DMD_INFO_t;

/*! DVBT information enum */
typedef enum
{
	DMD_E_INFO_DVBT_ALL       =  0,
	DMD_E_INFO_DVBT_REGREV    =  1,
	DMD_E_INFO_DVBT_PSEQRV    =  2,
	DMD_E_INFO_DVBT_SYSTEM    =  3,
	DMD_E_INFO_DVBT_LOCK      =  4,
	DMD_E_INFO_DVBT_AGC       =  5,
	DMD_E_INFO_DVBT_BERRNUM   =  6,
	DMD_E_INFO_DVBT_BITNUM    =  7,
	DMD_E_INFO_DVBT_CNR_INT   =  8,
	DMD_E_INFO_DVBT_CNR_DEC   =  9,
	DMD_E_INFO_DVBT_PERRNUM   = 10,
	DMD_E_INFO_DVBT_PACKETNUM = 11,
	DMD_E_INFO_DVBT_STATUS    = 12,
	DMD_E_INFO_DVBT_ERRORFREE,
	DMD_E_INFO_DVBT_SQI,
	DMD_E_INFO_DVBT_HIERARCHY_SELECT,
	DMD_E_INFO_DVBT_TPS_ALL,
	DMD_E_INFO_DVBT_MODE,
	DMD_E_INFO_DVBT_GI,
	DMD_E_INFO_DVBT_LENGTH_INDICATOR,
	DMD_E_INFO_DVBT_CONSTELLATION,
	DMD_E_INFO_DVBT_HIERARCHY,
	DMD_E_INFO_DVBT_HP_CODERATE,
	DMD_E_INFO_DVBT_LP_CODERATE,
	DMD_E_INFO_DVBT_CELLID,
	DMD_E_INFO_DVBT_END_OF_INFORMATION
} DMD_INFO_DVBT_t;

/*! DVB-T2 information enum */
typedef enum
{
	DMD_E_INFO_DVBT2_ALL       =  0,
	DMD_E_INFO_DVBT2_REGREV    =  1,
	DMD_E_INFO_DVBT2_PSEQRV    =  2,
	DMD_E_INFO_DVBT2_SYSTEM    =  3,
	DMD_E_INFO_DVBT2_LOCK      =  4,
	DMD_E_INFO_DVBT2_AGC       =  5,
	DMD_E_INFO_DVBT2_BERRNUM   =  6,
	DMD_E_INFO_DVBT2_BITNUM    =  7,
	DMD_E_INFO_DVBT2_CNR_INT   =  8,
	DMD_E_INFO_DVBT2_CNR_DEC   =  9,
	DMD_E_INFO_DVBT2_PERRNUM   = 10,
	DMD_E_INFO_DVBT2_PACKETNUM = 11,
	DMD_E_INFO_DVBT2_STATUS    = 12,
	DMD_E_INFO_DVBT2_ERRORFREE,
	DMD_E_INFO_DVBT2_SQI,
	DMD_E_INFO_DVBT2_MODE,
	DMD_E_INFO_DVBT2_GI,
	DMD_E_INFO_DVBT2_BERRNUM_C,
	DMD_E_INFO_DVBT2_BITNUM_C,
	DMD_E_INFO_DVBT2_SELECTED_PLP,
	DMD_E_INFO_DVBT2_L1_ALL,
	DMD_E_INFO_DVBT2_TYPE,
	DMD_E_INFO_DVBT2_BW_EXT,
	DMD_E_INFO_DVBT2_S1,
	DMD_E_INFO_DVBT2_S2,
	DMD_E_INFO_DVBT2_PAPR,
	DMD_E_INFO_DVBT2_L1_MOD,
	DMD_E_INFO_DVBT2_L1_COD,
	DMD_E_INFO_DVBT2_L1_FEC_TYPE,
	DMD_E_INFO_DVBT2_L1_POST_SIZE,
	DMD_E_INFO_DVBT2_L1_POST_INFO_SIZE,
	DMD_E_INFO_DVBT2_PILOT_PATTERN,
	DMD_E_INFO_DVBT2_TX_ID_AVAILABILITY,
	DMD_E_INFO_DVBT2_CELL_ID,
	DMD_E_INFO_DVBT2_NETWORK_ID,
	DMD_E_INFO_DVBT2_T2_SYSTEM_ID,
	DMD_E_INFO_DVBT2_NUM_T2_FRAMES,
	DMD_E_INFO_DVBT2_NUM_DATA_SYMBOLS,
	DMD_E_INFO_DVBT2_REGEN_FLAG,
	DMD_E_INFO_DVBT2_L1_POST_EXTENSION,
	DMD_E_INFO_DVBT2_NUM_RF,
	DMD_E_INFO_DVBT2_CURRENT_RF_IDX,
	DMD_E_INFO_DVBT2_SUB_SLICES_PER_FRAME,
	DMD_E_INFO_DVBT2_SUB_SLICE_INTERVAL,
	DMD_E_INFO_DVBT2_NUM_PLP,
	DMD_E_INFO_DVBT2_NUM_AUX,
	DMD_E_INFO_DVBT2_PLP_MODE,
	DMD_E_INFO_DVBT2_FEF_TYPE,
	DMD_E_INFO_DVBT2_FEF_LENGTH,
	DMD_E_INFO_DVBT2_FEF_INTERVAL,
	DMD_E_INFO_DVBT2_DAT_PLP_ID,
	DMD_E_INFO_DVBT2_DAT_PLP_TYPE,
	DMD_E_INFO_DVBT2_DAT_PLP_PAYLOAD_TYPE,
	DMD_E_INFO_DVBT2_DAT_PLP_GROUP_ID,
	DMD_E_INFO_DVBT2_DAT_PLP_COD,
	DMD_E_INFO_DVBT2_DAT_PLP_MOD,
	DMD_E_INFO_DVBT2_DAT_PLP_ROTATION,
	DMD_E_INFO_DVBT2_DAT_PLP_FEC_TYPE,
	DMD_E_INFO_DVBT2_DAT_PLP_NUM_BLOCKS_MAX,
	DMD_E_INFO_DVBT2_DAT_PLP_FRAME_INTEVAL,
	DMD_E_INFO_DVBT2_DAT_PLP_TIME_IL_LENGTH,
	DMD_E_INFO_DVBT2_DAT_PLP_TIME_IL_TYPE,
	DMD_E_INFO_DVBT2_DAT_FF_FLAG,
	DMD_E_INFO_DVBT2_COM_PLP_ID,
	DMD_E_INFO_DVBT2_COM_PLP_TYPE,
	DMD_E_INFO_DVBT2_COM_PLP_PAYLOAD_TYPE,
	DMD_E_INFO_DVBT2_COM_PLP_GROUP_ID,
	DMD_E_INFO_DVBT2_COM_PLP_COD,
	DMD_E_INFO_DVBT2_COM_PLP_MOD,
	DMD_E_INFO_DVBT2_COM_PLP_ROTATION,
	DMD_E_INFO_DVBT2_COM_PLP_FEC_TYPE,
	DMD_E_INFO_DVBT2_COM_PLP_NUM_BLOCKS_MAX,
	DMD_E_INFO_DVBT2_COM_PLP_FRAME_INTEVAL,
	DMD_E_INFO_DVBT2_COM_PLP_TIME_IL_LENGTH,
	DMD_E_INFO_DVBT2_COM_PLP_TIME_IL_TYPE,
	DMD_E_INFO_DVBT2_COM_FF_FLAG,
	DMD_E_INFO_DVBT2_FRAME_IDX,
	DMD_E_INFO_DVBT2_TYPE_2_START,
	DMD_E_INFO_DVBT2_L1_CHANGE_COUNTER,
	DMD_E_INFO_DVBT2_START_RF_IDX,
	DMD_E_INFO_DVBT2_DAT_FIRST_RF_IDX,
	DMD_E_INFO_DVBT2_DAT_PLP_START,
	DMD_E_INFO_DVBT2_DAT_PLP_NUM_BLOCKS,
	DMD_E_INFO_DVBT2_COM_FIRST_RF_IDX,
	DMD_E_INFO_DVBT2_COM_PLP_START,
	DMD_E_INFO_DVBT2_COM_PLP_NUM_BLOCKS,
	DMD_E_INFO_DVBT2_STATIC_FLAG,
	DMD_E_INFO_DVBT2_STATIC_PADDING_FLAG,
	DMD_E_INFO_DVBT2_IN_BAND_A_FLAG,
	DMD_E_INFO_DVBT2_IN_BAND_B_FLAG,
	DMD_E_INFO_DVBT2_END_OF_INFORMATION
} DMD_INFO_DVBT2_t;

/*! DVB-C information enum */
typedef enum
{
	DMD_E_INFO_DVBC_ALL       =  0,
	DMD_E_INFO_DVBC_REGREV    =  1,
	DMD_E_INFO_DVBC_PSEQRV    =  2,
	DMD_E_INFO_DVBC_SYSTEM    =  3,
	DMD_E_INFO_DVBC_LOCK      =  4,
	DMD_E_INFO_DVBC_AGC       =  5,
	DMD_E_INFO_DVBC_BERRNUM   =  6,
	DMD_E_INFO_DVBC_BITNUM    =  7,
	DMD_E_INFO_DVBC_CNR_INT   =  8,
	DMD_E_INFO_DVBC_CNR_DEC   =  9,
	DMD_E_INFO_DVBC_PERRNUM   = 10,
	DMD_E_INFO_DVBC_PACKETNUM = 11,
	DMD_E_INFO_DVBC_STATUS    = 12,
	DMD_E_INFO_DVBC_ERRORFREE = 13,
	DMD_E_INFO_DVBC_END_OF_INFORMATION
} DMD_INFO_DVBC_t;

/*! ISDB-T information enum */
typedef enum
{
	DMD_E_INFO_ISDBT_ALL       =  0,
	DMD_E_INFO_ISDBT_REGREV    =  1,
	DMD_E_INFO_ISDBT_PSEQRV    =  2,
	DMD_E_INFO_ISDBT_SYSTEM    =  3,
	DMD_E_INFO_ISDBT_LOCK      =  4,
	DMD_E_INFO_ISDBT_AGC       =  5,
	DMD_E_INFO_ISDBT_BERRNUM   =  6,
	DMD_E_INFO_ISDBT_BITNUM    =  7,
	DMD_E_INFO_ISDBT_CNR_INT   =  8,
	DMD_E_INFO_ISDBT_CNR_DEC   =  9,
	DMD_E_INFO_ISDBT_PERRNUM   = 10,
	DMD_E_INFO_ISDBT_PACKETNUM = 11,
	DMD_E_INFO_ISDBT_STATUS    = 12,
	DMD_E_INFO_ISDBT_ERRORFREE = 13,
	DMD_E_INFO_ISDBT_BERRNUM_A,
	DMD_E_INFO_ISDBT_BITNUM_A,
	DMD_E_INFO_ISDBT_BERRNUM_B,
	DMD_E_INFO_ISDBT_BITNUM_B,
	DMD_E_INFO_ISDBT_BERRNUM_C,
	DMD_E_INFO_ISDBT_BITNUM_C,
	DMD_E_INFO_ISDBT_ERRORFREE_A,
	DMD_E_INFO_ISDBT_ERRORFREE_B,
	DMD_E_INFO_ISDBT_ERRORFREE_C,
	DMD_E_INFO_ISDBT_MODE,
	DMD_E_INFO_ISDBT_GI,
	DMD_E_INFO_ISDBT_SYS_TMCC,
	DMD_E_INFO_ISDBT_COUNTDOWN,
	DMD_E_INFO_ISDBT_EMGFLG,
	DMD_E_INFO_ISDBT_PART,
	DMD_E_INFO_ISDBT_MAPA,
	DMD_E_INFO_ISDBT_CRA,
	DMD_E_INFO_ISDBT_INTA,
	DMD_E_INFO_ISDBT_SEGA,
	DMD_E_INFO_ISDBT_MAPB,
	DMD_E_INFO_ISDBT_CRB,
	DMD_E_INFO_ISDBT_INTB,
	DMD_E_INFO_ISDBT_SEGB,
	DMD_E_INFO_ISDBT_MAPC,
	DMD_E_INFO_ISDBT_CRC,
	DMD_E_INFO_ISDBT_INTC,
	DMD_E_INFO_ISDBT_SEGC,
	DMD_E_INFO_ISDBT_PHCOR,
	DMD_E_INFO_ISDBT_END_OF_INFORMATION
} DMD_INFO_ISDBT_t;

/*! ISDB-S information enum */
typedef enum
{
	DMD_E_INFO_ISDBS_ALL       =  0,
	DMD_E_INFO_ISDBS_REGREV    =  1,
	DMD_E_INFO_ISDBS_PSEQRV    =  2,
	DMD_E_INFO_ISDBS_SYSTEM    =  3,
	DMD_E_INFO_ISDBS_LOCK      =  4,
	DMD_E_INFO_ISDBS_AGC       =  5,
	DMD_E_INFO_ISDBS_BERRNUM   =  6,
	DMD_E_INFO_ISDBS_BITNUM    =  7,
	DMD_E_INFO_ISDBS_CNR_INT   =  8,
	DMD_E_INFO_ISDBS_CNR_DEC   =  9,
	DMD_E_INFO_ISDBS_PERRNUM   = 10,
	DMD_E_INFO_ISDBS_PACKETNUM = 11,
	DMD_E_INFO_ISDBS_STATUS    = 12,
	DMD_E_INFO_ISDBS_ERRORFREE = 13,
	DMD_E_INFO_ISDBS_BERRNUM_1,
	DMD_E_INFO_ISDBS_BITNUM_1,
	DMD_E_INFO_ISDBS_BERRNUM_2,
	DMD_E_INFO_ISDBS_BITNUM_2,
	DMD_E_INFO_ISDBS_BERRNUM_3,
	DMD_E_INFO_ISDBS_BITNUM_3,
	DMD_E_INFO_ISDBS_ERRORFREE_0,
	DMD_E_INFO_ISDBS_ERRORFREE_1,
	DMD_E_INFO_ISDBS_ERRORFREE_2,
	DMD_E_INFO_ISDBS_ERRORFREE_3,
	DMD_E_INFO_ISDBS_ERRORFREE_T,
	DMD_E_INFO_ISDBS_TSNO,
	DMD_E_INFO_ISDBS_TSID,
	DMD_E_INFO_ISDBS_MOD,
	DMD_E_INFO_ISDBS_EMGSW,
	DMD_E_INFO_ISDBS_UPLINK,
	DMD_E_INFO_ISDBS_EXON,
	DMD_E_INFO_ISDBS_CHANGE,
	DMD_E_INFO_ISDBS_MOD0,
	DMD_E_INFO_ISDBS_SLOT0,
	DMD_E_INFO_ISDBS_MOD1,
	DMD_E_INFO_ISDBS_SLOT1,
	DMD_E_INFO_ISDBS_MOD2,
	DMD_E_INFO_ISDBS_SLOT2,
	DMD_E_INFO_ISDBS_MOD3,
	DMD_E_INFO_ISDBS_SLOT3,
	DMD_E_INFO_ISDBS_TSID0,
	DMD_E_INFO_ISDBS_TSID1,
	DMD_E_INFO_ISDBS_TSID2,
	DMD_E_INFO_ISDBS_TSID3,
	DMD_E_INFO_ISDBS_TSID4,
	DMD_E_INFO_ISDBS_TSID5,
	DMD_E_INFO_ISDBS_TSID6,
	DMD_E_INFO_ISDBS_TSID7,
	DMD_E_INFO_ISDBS_TSNO01,
	DMD_E_INFO_ISDBS_TSNO02,
	DMD_E_INFO_ISDBS_TSNO03,
	DMD_E_INFO_ISDBS_TSNO04,
	DMD_E_INFO_ISDBS_TSNO05,
	DMD_E_INFO_ISDBS_TSNO06,
	DMD_E_INFO_ISDBS_TSNO07,
	DMD_E_INFO_ISDBS_TSNO08,
	DMD_E_INFO_ISDBS_TSNO09,
	DMD_E_INFO_ISDBS_TSNO10,
	DMD_E_INFO_ISDBS_TSNO11,
	DMD_E_INFO_ISDBS_TSNO12,
	DMD_E_INFO_ISDBS_TSNO13,
	DMD_E_INFO_ISDBS_TSNO14,
	DMD_E_INFO_ISDBS_TSNO15,
	DMD_E_INFO_ISDBS_TSNO16,
	DMD_E_INFO_ISDBS_TSNO17,
	DMD_E_INFO_ISDBS_TSNO18,
	DMD_E_INFO_ISDBS_TSNO19,
	DMD_E_INFO_ISDBS_TSNO20,
	DMD_E_INFO_ISDBS_TSNO21,
	DMD_E_INFO_ISDBS_TSNO22,
	DMD_E_INFO_ISDBS_TSNO23,
	DMD_E_INFO_ISDBS_TSNO24,
	DMD_E_INFO_ISDBS_TSNO25,
	DMD_E_INFO_ISDBS_TSNO26,
	DMD_E_INFO_ISDBS_TSNO27,
	DMD_E_INFO_ISDBS_TSNO28,
	DMD_E_INFO_ISDBS_TSNO29,
	DMD_E_INFO_ISDBS_TSNO30,
	DMD_E_INFO_ISDBS_TSNO31,
	DMD_E_INFO_ISDBS_TSNO32,
	DMD_E_INFO_ISDBS_TSNO33,
	DMD_E_INFO_ISDBS_TSNO34,
	DMD_E_INFO_ISDBS_TSNO35,
	DMD_E_INFO_ISDBS_TSNO36,
	DMD_E_INFO_ISDBS_TSNO37,
	DMD_E_INFO_ISDBS_TSNO38,
	DMD_E_INFO_ISDBS_TSNO39,
	DMD_E_INFO_ISDBS_TSNO40,
	DMD_E_INFO_ISDBS_TSNO41,
	DMD_E_INFO_ISDBS_TSNO42,
	DMD_E_INFO_ISDBS_TSNO43,
	DMD_E_INFO_ISDBS_TSNO44,
	DMD_E_INFO_ISDBS_TSNO45,
	DMD_E_INFO_ISDBS_TSNO46,
	DMD_E_INFO_ISDBS_TSNO47,
	DMD_E_INFO_ISDBS_TSNO48,
	DMD_E_INFO_ISDBS_END_OF_INFORMATION
} DMD_INFO_ISDBS_t;

/*! ATSC information enum */
typedef enum
{
	DMD_E_INFO_ATSC_ALL       =  0,
	DMD_E_INFO_ATSC_REGREV    =  1,
	DMD_E_INFO_ATSC_PSEQRV    =  2,
	DMD_E_INFO_ATSC_SYSTEM    =  3,
	DMD_E_INFO_ATSC_LOCK      =  4,
	DMD_E_INFO_ATSC_AGC       =  5,
	DMD_E_INFO_ATSC_BERRNUM   =  6,
	DMD_E_INFO_ATSC_BITNUM    =  7,
	DMD_E_INFO_ATSC_CNR_INT   =  8,
	DMD_E_INFO_ATSC_CNR_DEC   =  9,
	DMD_E_INFO_ATSC_PERRNUM   = 10,
	DMD_E_INFO_ATSC_PACKETNUM = 11,
	DMD_E_INFO_ATSC_STATUS    = 12,
	DMD_E_INFO_ATSC_ERRORFREE = 13,
	DMD_E_INFO_ATSC_END_OF_INFORMATION
} DMD_INFO_ATSC_t;

/*! QAM information enum */
typedef enum
{
	DMD_E_INFO_QAM_ALL       =  0,
	DMD_E_INFO_QAM_REGREV    =  1,
	DMD_E_INFO_QAM_PSEQRV    =  2,
	DMD_E_INFO_QAM_SYSTEM    =  3,
	DMD_E_INFO_QAM_LOCK      =  4,
	DMD_E_INFO_QAM_AGC       =  5,
	DMD_E_INFO_QAM_BERRNUM   =  6,
	DMD_E_INFO_QAM_BITNUM    =  7,
	DMD_E_INFO_QAM_CNR_INT   =  8,
	DMD_E_INFO_QAM_CNR_DEC   =  9,
	DMD_E_INFO_QAM_PERRNUM   = 10,
	DMD_E_INFO_QAM_PACKETNUM = 11,
	DMD_E_INFO_QAM_STATUS    = 12,
	DMD_E_INFO_QAM_ERRORFREE = 13,
	DMD_E_INFO_QAM_END_OF_INFORMATION
} DMD_INFO_QAM_t;

/*! ANALOG information enum */
typedef enum
{
	DMD_E_INFO_ANALOG_ALL       =  0,
	DMD_E_INFO_ANALOG_REGREV    =  1,
	DMD_E_INFO_ANALOG_PSEQRV    =  2,
	DMD_E_INFO_ANALOG_SYSTEM    =  3,
	DMD_E_INFO_ANALOG_LOCK      =  4,
	DMD_E_INFO_ANALOG_AGC       =  5,
	DMD_E_INFO_ANALOG_BERRNUM   =  6,
	DMD_E_INFO_ANALOG_BITNUM    =  7,
	DMD_E_INFO_ANALOG_CNR_INT   =  8,
	DMD_E_INFO_ANALOG_CNR_DEC   =  9,
	DMD_E_INFO_ANALOG_PERRNUM   = 10,
	DMD_E_INFO_ANALOG_PACKETNUM = 11,
	DMD_E_INFO_ANALOG_STATUS    = 12,
	DMD_E_INFO_ANALOG_ERRORFREE = 13,
	DMD_E_INFO_ANALOG_END_OF_INFORMATION
} DMD_INFO_ANALOG_t;

typedef enum
{
	DMD_E_ERRORFREE_ERROR = 0,
	DMD_E_ERRORFREE_ERRORFREE
} DMD_ERRORFREE_t;

// DVB-T
typedef enum
{
	DMD_E_DVBT_HIER_SEL_LP = 0,
	DMD_E_DVBT_HIER_SEL_HP
} DMD_DVBT_HIER_SEL_t;

typedef enum
{
	DMD_E_DVBT_TPS_OBTAIN_NG = 0,
	DMD_E_DVBT_TPS_OBTAIN_OK
} DMD_DVBT_TPS_OBTAIN_t;

typedef enum
{
	DMD_E_DVBT_MODE_2K = 0,
	DMD_E_DVBT_MODE_8K,
	DMD_E_DVBT_MODE_4K,
	DMD_E_DVBT_MODE_NOT_DEFINED
} DMD_DVBT_MODE_t;

typedef enum
{
	DMD_E_DVBT_GI_1_32 = 0,
	DMD_E_DVBT_GI_1_16,
	DMD_E_DVBT_GI_1_8,
	DMD_E_DVBT_GI_1_4,
	DMD_E_DVBT_GI_NOT_DEFINED
} DMD_DVBT_GI_t;

typedef enum
{
	DMD_E_DVBT_CONST_QPSK = 0,
	DMD_E_DVBT_CONST_16QAM,
	DMD_E_DVBT_CONST_64QAM
} DMD_DVBT_CONST_t;

typedef enum
{
	DMD_E_DVBT_HIERARCHY_NO = 0,
	DMD_E_DVBT_HIERARCHY_ALPHA_1,
	DMD_E_DVBT_HIERARCHY_ALPHA_2,
	DMD_E_DVBT_HIERARCHY_ALPHA_4
} DMD_DVBT_HIERARCHY_t;

typedef enum
{
	DMD_E_DVBT_CR_1_2 = 0,
	DMD_E_DVBT_CR_2_3,
	DMD_E_DVBT_CR_3_4,
	DMD_E_DVBT_CR_5_6,
	DMD_E_DVBT_CR_7_8
} DMD_DVBT_CR_t;

// DVB-T2
typedef enum
{
	DMD_E_DVBT2_MODE_1K = 0,
	DMD_E_DVBT2_MODE_2K,
	DMD_E_DVBT2_MODE_4K,
	DMD_E_DVBT2_MODE_8K,
	DMD_E_DVBT2_MODE_16K,
	DMD_E_DVBT2_MODE_32K
} DMD_DVBT2_MODE_t;

typedef enum
{
	DMD_E_DVBT2_GI_1_32 = 0,
	DMD_E_DVBT2_GI_1_16,
	DMD_E_DVBT2_GI_1_8,
	DMD_E_DVBT2_GI_1_4,
	DMD_E_DVBT2_GI_1_128,
	DMD_E_DVBT2_GI_19_128,
	DMD_E_DVBT2_GI_19_256
} DMD_DVBT2_GI_t;

typedef enum
{
	DMD_E_DVBT2_TYPE_TS = 0,
	DMD_E_DVBT2_TYPE_GS,
	DMD_E_DVBT2_TYPE_TS_GS
} DMD_DVBT2_TYPE_t;

typedef enum
{
	DMD_E_DVBT2_PAPR_NO = 0,
	DMD_E_DVBT2_PAPR_ACE,
	DMD_E_DVBT2_PAPR_TR,
	DMD_E_DVBT2_PAPR_ACE_TR
} DMD_DVBT2_PAPR_t;

typedef enum
{
	DMD_E_DVBT2_L1_MOD_BPSK = 0,
	DMD_E_DVBT2_L1_MOD_QPSK,
	DMD_E_DVBT2_L1_MOD_16QAM,
	DMD_E_DVBT2_L1_MOD_64QAM
} DMD_DVBT2_L1_MOD_t;

typedef enum
{
	DMD_E_DVBT2_CR_1_2 = 0,
	DMD_E_DVBT2_CR_3_5,
	DMD_E_DVBT2_CR_2_3,
	DMD_E_DVBT2_CR_3_4,
	DMD_E_DVBT2_CR_4_5,
	DMD_E_DVBT2_CR_5_6
} DMD_DVBT2_CR_t;

typedef enum
{
	DMD_E_DVBT2_FEC_TYPE_16K = 0,
	DMD_E_DVBT2_FEC_TYPE_64K
} DMD_DVBT2_FEC_TYPE_t;

typedef enum
{
	DMD_E_DVBT2_PP_1 = 0,
	DMD_E_DVBT2_PP_2,
	DMD_E_DVBT2_PP_3,
	DMD_E_DVBT2_PP_4,
	DMD_E_DVBT2_PP_5,
	DMD_E_DVBT2_PP_6,
	DMD_E_DVBT2_PP_7,
	DMD_E_DVBT2_PP_8
} DMD_DVBT2_PP_t;

typedef enum
{
	DMD_E_DVBT2_PLP_MODE_NO = 0,
	DMD_E_DVBT2_PLP_MODE_NORM,
	DMD_E_DVBT2_PLP_MODE_HEM
} DMD_DVBT2_PLP_MODE_t;

typedef enum
{
	DMD_E_DVBT2_PLP_TYPE_COM = 0,
	DMD_E_DVBT2_PLP_TYPE_DAT1,
	DMD_E_DVBT2_PLP_TYPE_DAT2
} DMD_DVBT2_PLP_TYPE_t;

typedef enum
{
	DMD_E_DVBT2_PLP_PAYLOAD_GFPS = 0,
	DMD_E_DVBT2_PLP_PAYLOAD_GCS,
	DMD_E_DVBT2_PLP_PAYLOAD_GSE,
	DMD_E_DVBT2_PLP_PAYLOAD_TS
} DMD_DVBT2_PLP_PAYLOAD_t;

typedef enum
{
	DMD_E_DVBT2_PLP_MOD_QPSK = 0,
	DMD_E_DVBT2_PLP_MOD_16QAM,
	DMD_E_DVBT2_PLP_MOD_64QAM,
	DMD_E_DVBT2_PLP_MOD_256QAM
} DMD_DVBT2_PLP_MOD_t;

/* '11/08/01 : OKAMOTO Correct Error. */
/* **************************************************** */
/* DMD Defines                                          */
/* **************************************************** */
/*! BAND WIDTH */
typedef enum
{
	DMD_E_BW_5MHZ = 5,
	DMD_E_BW_6MHZ,
	DMD_E_BW_7MHZ,
	DMD_E_BW_8MHZ,
	DMD_E_BW_1_7MHZ
} DMD_BANDWIDTH_t;

/* '11/08/12 : OKAMOTO Implement IF 4.5MHz for DVB-T/T2 7MHz. */
/*! IF Frequency */
typedef enum
{
	DMD_E_IF_5000KHZ,
	DMD_E_IF_4500KHZ,
	DMD_E_IF_MAX,
} DMD_IF_FREQ_t;

/* '11/08/29 : OKAMOTO Select TS output. */
typedef enum
{
	DMD_E_TSOUT_PARALLEL_FIXED_CLOCK,
	DMD_E_TSOUT_PARALLEL_VARIABLE_CLOCK,
	DMD_E_TSOUT_SERIAL_VARIABLE_CLOCK,
} DMD_TSOUT;

enum
{
	DEMOD_BANK_T = 0,
	DEMOD_BANK_T2,
	DEMOD_BANK_C,
};

#define DEMO_I2C_MAX_LEN    8
#define DMD_NOT_SUPPORT     0
#define DMD_SYSTEM_MAX      15
#define DMD_TCB_DATA_MAX    256
#define DMD_REGISTER_MAX    2048
#define DMD_INFORMATION_MAX 512

/*! ARRAY FLAG */
typedef enum
{
	DMD_E_END_OF_ARRAY = 0,
	DMD_E_ARRAY
} DMD_ARRAY_FLAG_t;

/*! I2C Register Structure */
typedef struct
{
	u8 slvadr;
	u8 adr;
	u8 data;
	DMD_ARRAY_FLAG_t flag;
} DMD_I2C_Register_t;

typedef struct nim_mn88472_private
{
	struct COFDM_TUNER_CONFIG_API tc;

	DMD_IF_FREQ_t if_freq;
	DMD_BANDWIDTH_t bw;
	u32 frq;//KHz
	u8 system;

	u8 i2c_addr[3];

	u32 tuner_id;
#if defined(MODULE)
	struct i2c_adapter *i2c_adap;  /* i2c bus of the tuner */
#endif

	OSAL_ID flag_id;
	OSAL_ID i2c_mutex_id;

	u32 sym; //for DVB-C

	u8 PLP_num;
	u8 PLP_id;
	u16 scan_stop_flag : 1;
	u16 first_tune_t2 : 1;
	u16 qam : 4; //for DVB-C
	u16 reserved : 10;
} NIM_MN88472_PRIVATE, *PNIM_MN88472_PRIVATE;

/* ************************************************** */
/* Register Setting Array
 **************************************************** */
extern DMD_I2C_Register_t MN88472_REG_COMMON[];
extern DMD_I2C_Register_t MN88472_REG_DVBT2_7MHZ[];
extern DMD_I2C_Register_t MN88472_REG_DVBT_6MHZ[];
extern DMD_I2C_Register_t MN88472_REG_DVBT_7MHZ[];
extern DMD_I2C_Register_t MN88472_REG_DVBT2_6MHZ[];
extern DMD_I2C_Register_t MN88472_REG_DVBT2_8MHZ[];
extern DMD_I2C_Register_t MN88472_REG_DVBT2_5MHZ[];
extern DMD_I2C_Register_t MN88472_REG_DVBT2_1_7MHZ[];
extern DMD_I2C_Register_t MN88472_REG_DVBC[];
extern DMD_I2C_Register_t MN88472_REG_DVBT_8MHZ[];

extern DMD_I2C_Register_t MN88472_REG_24MHZ_COMMON[];
extern DMD_I2C_Register_t MN88472_REG_24MHZ_DVBT2_7MHZ[];
extern DMD_I2C_Register_t MN88472_REG_24MHZ_DVBT_6MHZ[];
extern DMD_I2C_Register_t MN88472_REG_24MHZ_DVBT_7MHZ[];
extern DMD_I2C_Register_t MN88472_REG_24MHZ_DVBT2_6MHZ[];
extern DMD_I2C_Register_t MN88472_REG_24MHZ_DVBT2_8MHZ[];
extern DMD_I2C_Register_t MN88472_REG_24MHZ_DVBT2_5MHZ[];
extern DMD_I2C_Register_t MN88472_REG_24MHZ_DVBT2_1_7MHZ[];
extern DMD_I2C_Register_t MN88472_REG_24MHZ_DVBC[];
extern DMD_I2C_Register_t MN88472_REG_24MHZ_DVBT_8MHZ[];

extern u8 MN88472_REG_AUTOCTRL[];
extern u32 MN88472_REG_AUTOCTRL_SIZE;

extern u32 logtbl[];
extern u32 DMD_DVBT_CNR_P1[3][5];
extern u32 DMD_DVBT2_CNR_P1[4][6];

/* '11/08/12 : OKAMOTO Implement IF 4.5MHz for DVB-T/T2 7MHz. */
extern DMD_I2C_Register_t MN88472_REG_DIFF_DVBT2_7MHZ_IF4500KHZ[];
extern DMD_I2C_Register_t MN88472_REG_DIFF_DVBT_7MHZ_IF4500KHZ[];

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(_array) (sizeof(_array)/sizeof(_array[0]))
#endif
#define MN88472_TUNE_MAX_NUM    2
#define MN88472_PLP_TUNE_NUM    3
#define MN88472_T2_SEARCH_NUM   3
#define MN88472_T2_TUNE_TIMEOUT 1200  //ms
#define MN88472_TUNE_MODE       0
#define MN88472_SEARCH_MODE     1

#define MN88472_T2_ADDR         0x38  // 0x1c
#define MN88472_T_ADDR          0x30  // 0x18
#define MN88472_C_ADDR          0x34  // 0x1a

#endif  // NIM_MN88472_H
// vim:ts=4
