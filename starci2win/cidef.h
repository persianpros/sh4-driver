/*
 * 	Copyright (C) 2010 Duolabs Srl
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef _CIDEF_H
#define _CIDEF_H

#define	UI8		unsigned char
#define	UI16	unsigned short
#define	UI32	unsigned int

#define	I8		char
#define	I16		short
#define	I32		int

#define STARCI2WIN


#define     KO          0x07

#define     NOTUSE      0x00
#define     ATTRIBUTE   0x01
#define     IOACCESS    0x02

/* CIS management */
/* tuples */
#define CISTPL_DEVICE                   0x01
#define CISTPL_DEVICE_A                 0x17
#define CISTPL_DEVICE_0A                0x1d
#define CISTPL_DEVICE_0C                0x1c
#define CISTPL_VERS_1                   0x15
#define CISTPL_MANFID                   0x20
#define CISTPL_CONFIG                   0x1a
#define CISTPL_CFTABLE_ENTRY    0x1b
#define CISTPL_NOLINK                   0x14
#define CISTPL_END                              0xff

/* subtuples */
#define CCST_CIF                                0xc0
#define STCE_EV                                 0xc0
#define STCE_PD                                 0xc1

/* offset in tuple parameters (without the 2 first bytes tuple and link) */
/* CISTPL_VERS_1 */
#define TPLLV1_MAJOR            0
#define TPLLV1_MINOR            1

/* CISTPL_CONFIG */
#define TPCC_SZ                         0
#define TPCC_LAST                       1
#define TPCC_RADR                       2

/* CISTPL_CFTABLE_ENTRY */
#define TPCE_INDX                       0
#define TPCE_IF                         1       /* If field present ! */
#define TPCE_FS                         2       /* If TPCE_IF field present, else 1 */
#define TPCE_PD                         3       /* If TPCE_IF field present, else 2 */

/* bit mask */
/* CISTPL_CONFIG */
#define TPCC_RASZ                       0x03
#define TPCC_RMSZ                       0x3c

/* CISTPL_CFTABLE_ENTRY */
#define TPCE_INDX_MASK          0xc0
#define TPCE_INTFACE            0x80
#define TPCE_IF_DVB_VAL         0x04                    /* Imposed in CENELEC Errata document. */
#define TPCE_FS_DVB_MASK        0x1b                    /* Previously TPCE_FS_MASK set to 0x0b wrong */
#define TPCE_FS_IO_IRQ_MASK     0x18
#define TPCE_FS_PD_MASK         0x03                    /* Previously TPCE_PD_MASK set to 0x01 wrong */
#define TPCE_EXT                        0x80
#define TPCE_TIMING                     0x04
#define TPCE_TIMING_WAIT        0x03
#define TPCE_TIMING_READY       0x1c
#define TPCE_TIMING_RESERV      0xe0
#define TPCE_IO                         0x08
#define TPCE_IO_DVB_VAL         0x22                    /* Imposed in CENELEC Errata document. */
#define TPCE_IO_RANGE           0x80
#define TPCE_IO_SLN                     0xc0
#define TPCE_IO_SLNR            0x06
#define TPCE_IO_SAN                     0x30
#define TPCE_IO_SANR            0x04
#define TPCE_IRQ                        0x10
#define TPCE_IRQ_MASK           0x10
#define TPCE_IR_DVB_VAL         0x20                    /* Imposed in CENELEC Errata document. */
#define TPCE_MEM                        0x60
#define TPCE_MEM_TWO_BYTES      0x20
#define TPCE_MEM_FOUR_BYTES     0x40
#define TPCE_MEM_DESCRIPTOR     0x60
#define TPCE_MEM_SL                     0x18
#define TPCE_MEM_SLR            0x03
#define TPCE_MEM_SCA            0x60
#define TPCE_MEM_SCAR           0x05
#define TPCE_MEM_HPA            0x80
#define TPCE_MISC                       0x80

/* value */
/* CISTPL_VERS_1 */
#define TPLLV1_MAJOR_VAL        0x05
#define TPLLV1_MINOR_VAL        0x00

/* CISTPL_CONFIG */
#define TPCC_RMSK                       0x01
#define NBRBYTES                        0x01
#define MAXADDR                         0xffe

/* CISTPL_CFTABLE_ENTRY */
#define TPCE_IF_VAL                     0x04

/* CISTPL_NOLINK */
#define CISTPL_NOLINKLEN        0x00

/* CCST_CIF */
#define CCST_CIFLEN                     0x0e
#define CCST_CIF1                       0x41
#define CCST_CIF2                       0x02
#define STCI_STR                        "DVB_CI_V"
#define STCI_STRP                       0x2e

/* STCE_EV */
#define STCE_EV_VAL                     "DVB_HOST"

/* STCE_PD */
#define STCE_PD_VAL                     "DVB_CI_MODULE"


#define         CXD1957CNTL     *(volatile UI8 *)0x60000001
#define         CXD1957SO       *(volatile UI8 *)0x60000003
#define         CXD1957MODEA    *(volatile UI8 *)0x60000005
#define         CXD1957GPIOCNTL *(volatile UI8 *)0x60000007
#define         CXD1957MODEB    *(volatile UI8 *)0x60000009

#define OGIO_0_BaseAddress	0xB4000000

#define     CIMODULE_A  *(volatile UI8 *)OGIO_0_BaseAddress
#define     CIMA_DATA   *(volatile UI8 *)OGIO_0_BaseAddress	+ 0
#define     CIMA_CMDST  *(volatile UI8 *)OGIO_0_BaseAddress + 2
#define     CIMA_LS     *(volatile UI8 *)OGIO_0_BaseAddress + 4
#define     CIMA_MS     *(volatile UI8 *)OGIO_0_BaseAddress + 6
#define     CIMA_COR    *(volatile UI8 *)OGIO_0_BaseAddress	+ 0x400

#define     CIMODULE_B  *(volatile UI8 *)OGIO_0_BaseAddress
#define     CIMB_DATA   *(volatile UI8 *)OGIO_0_BaseAddress	+ 0
#define     CIMB_CMDST  *(volatile UI8 *)OGIO_0_BaseAddress	+ 2
#define     CIMB_LS     *(volatile UI8 *)OGIO_0_BaseAddress	+ 4
#define     CIMB_MS     *(volatile UI8 *)OGIO_0_BaseAddress	+ 6
#define     CIMB_COR    *(volatile UI8 *)OGIO_0_BaseAddress	+ 0x400

/* module status information */
#define     MODULE_DA   0x80
#define     MODULE_FR   0x40
#define     MODULE_WE   0x02
#define     MODULE_RE   0x01
/* module command definition */
#define     MODULE_RS   0x08
#define     MODULE_SR   0x04
#define     MODULE_SW   0x02
#define     MODULE_HC   0x01
#define     CONTROL    1//0
#define     STATUS_m   1//0
#define     DATA_m     0//1
#define     SIZE_H     3//2
#define     SIZE_L     2//3
#define     ENABLE     1
#define     DISABLE    0
#define     RESET      5

typedef struct {
    UI16 addr;
    UI16 len;
    UI8 *pbytes;
    UI16 rlen;
} DRV_stMem;

#define  OK				1
#define  NG				0		

#define PCMCIA_ERROR	-1

#define	READ_ERROR		PCMCIA_ERROR
#define	WRITE_ERROR		PCMCIA_ERROR


#define  SLOTA	    0
#define  SLOTB      1

//#ifdef EVAL_HACK
#define  MAX_SLOT	3
//#else
//#define  MAX_SLOT	2
//#endif
/*  CI Main Mode */
#define  DETECT_m       10
#define  PWRCNTL        11
#define  RESET_m        12
#define  READ_CIS       13
#define  NEGOBUFF       14
#define  IDLE_CI      0xff
#define  PWRON_w        16
#define  NO_CIMODULE    17
/*  CI sub mode */
#define  PWRON_m        30
#define  PWROFF_m       31
#define  RES_ON         32
#define  RES_OFF        33
#define  RESET_l			34


struct dvb_entry_bundle
{
   int     Is_TPCE_INDX_Ok;        /* Intface + Default bits in TPCE_INDX if first Entry */
   int     Is_TPCE_IF;             /* If TRUE, the field TPCE_IF is present */
   int     Is_TPCE_IF_Ok;          /* "Errata in EN 50221 from CENELEC..." TPCE_IF imposed. */
   int     IsMasked_TPCE_FS_Ok;    /* TPCE_FS (Features) must include Power + IO + IRQ */
   int     Is_TPCE_PD_Ok;          /* There is at least one Description structure for Vcc */
   int     Is_TPCE_IO_Ok;          /* "Errata in EN 50221 from CENELEC..." TPCE_IO imposed. */
   int     Is_TPCE_IRQ_Ok;         /* "Errata in EN 50221 from CENELEC..." TPCE_IR imposed. */
   int     Is_STCE_EV_Ok;          /* Subtuple of CISTPL_CFTABLE_ENTRY contains "DVB_HOST" */
   int     Is_STCE_PD_Ok;          /* Subtuple of CISTPL_CFTABLE_ENTRY contains "DVB_CI_MODULE" */

   UI8   EntryNumb;              /* Read value of Configuration Entry Number in TPCE_INDX */
   UI8   TPCE_IF_Value;          /* Read value of TPCE_IF field. */
};

#define    DRV_CISLEN  256

#endif //_CIDEF_H
