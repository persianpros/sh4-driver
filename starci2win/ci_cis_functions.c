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

#include <linux/string.h>
#include <linux/ctype.h>
#include "cidef.h"

I32 PHYS_CheckCIS(UI8 *,I32,UI16 *,UI8 *);
I32 PHYS_Tuple_CISTPL_CFTABLE_ENTRY( UI8 *,I32, struct  dvb_entry_bundle *);
I32 PHYS_Tuple(UI8, UI8 **, I32 *, I32 *, I32 *, I32 *, I32 *);


struct dvb_entry_bundle ThisEntry;

/* ======================================================================== */
/* NAME : PHYS_CheckCIS                                                     */
/*                                                                          */
/* DESCRIPTION : Check the CIS                                              */
/*              The following tuples must be found:                         */
/*                      - CISTPL_DEVICE                                     */
/*                      - CISTPL_DEVICE_A                                   */
/*                      - CISTPL_DEVICE_0A                                  */
/*                      - CISTPL_DEVICE_0C                                  */
/*                      - CISTPL_VERS_1                                     */
/*                      - CISTPL_MANFID                                     */
/*                      - CISTPL_CONFIG                                     */
/*                      - CISTPL_CFTABLE_ENTRY                              */
/*                      - CISTPL_NOLINK                                     */
/*                      - CISTPL_END                                        */
/*              Other tuples are ignored                                    */
/*                                                                          */
/* INPUT PARAMETERS :                                                       */
/*              cis : CIS buffer                                            */
/*              len : length of the CIS buffer                              */
/*                                                                          */
/* OUTPUT PARAMETERS :                                                      */
/*              addrcor : COR address                                       */
/*              cor             : COR value                                 */
/*                                                                          */
/* RETURN CODE                                                              */
/*                 0 : OK                                                   */
/*                -1 : error                                                */
/*                                                                          */
/* ======================================================================== */
I32 PHYS_CheckCIS(UI8 *cis, I32 len, UI16 *addrcor, UI8 *cor)
{
    /*volatile*/ UI8 *pcis;        /* pointer to the remainding CIS                     */
    UI8 tuple=0;        /* current tuple type.                               */
    UI8 offset=0;       /* link to next tuple.                               */
    I32 lcis=0;           /* remainding length of the CIS                      */
    I32 first=0;          /* 1 for the first configuration table entry, else 0 */
    I32 addr=0;           /* configuration registers base address              */
    I32 dvb=0;            /* index number of the DVB entry                     */
    I32 last=0;           /* index number of the last entry                    */
    I32 IsAddrCor = 0;  /* Cor Address is found.                             */
    I32 IsCor = 0;      /* Cor is found.                                     */
    I32 Result=0;         /* Return value got from PHYS_Tuple                  */
    I32 iRet=0;           /* return code                                       */

    UI16 cis_index = 0;

	pcis  = cis;
	lcis  = len;
	first = 1;
	dvb   = 0;

	while( * pcis != CISTPL_END )
	{
		tuple  = * pcis;
		offset = *(pcis+1);

		Result = PHYS_Tuple( 0, & pcis, & lcis, & first, & addr, & dvb,  & last );
		if( Result != -1 )		/* Skip error cases */
		{
			switch( tuple )
			{
				case CISTPL_CONFIG:		// 0x1A

					* addrcor = addr;
					IsAddrCor = 1;
					break;

				case CISTPL_DEVICE:		// 0x01
				case CISTPL_DEVICE_A:	// 0x17
				case CISTPL_DEVICE_0A:	// 0x1D
				case CISTPL_DEVICE_0C:	// 0x1C
				case CISTPL_VERS_1:		// 0x15
				case CISTPL_MANFID:		// 0x20
				case CISTPL_NOLINK:		// 0x14
				case CISTPL_END:		// 0xff
				//default:
					break;

				case CISTPL_CFTABLE_ENTRY:	// 0x1B

					if( Result == 1  ||  dvb != 0 )
					{
						* cor = dvb;
						IsCor = 1;
					}
					break;
				default:
					break;

			} /* end of switch */
		} /* end of if */
	} /* end of while */

	if( IsAddrCor == 1  &&  IsCor == 1 )
		iRet = 1;
	else
		iRet = 0;

	return iRet;
}

/* ============================================================================ */
/* NAME : PHYS_Tuple_CISTPL_CFTABLE_ENTRY                                       */
/*                                                                              */
/* DESCRIPTION : Parses CISTPL_CFTABLE_ENTRY tuples data, from current CIS      */
/*                                                                              */
/* INPUT PARAMETERS :                                                           */
/*              pcis :  pointer to the tuple to check                           */
/*              first:  1 for the first CISTPL_CFTABLE_ENTRY tuple, else 0      */
/*              pentry: pointer on a private structure used to collect DVB info */
/*                                                                              */
/* OUTPUT PARAMETERS :                                                          */
/*              first: 0 when an new entry is OK                                */
/*              pentry: pointer on a private structure used to collect DVB info */
/*                                                                              */
/* RETURN CODE                                                                  */
/*                 1 : OK this table entry is a DVB properly filled one.        */
/*                 0 : ERROR DVB table entry not compliant.                     */
/*                -1 : A non-DVB table entry to skip (irrelevant for us).       */
/*                                                                                                                                                      */
/* ============================================================================ */

I32 PHYS_Tuple_CISTPL_CFTABLE_ENTRY( UI8 *pcis,I32 IsFirst,struct dvb_entry_bundle * pentry )
{
     UI8 *pcheck;
     UI32    length = *(pcis+1);
     UI8   Features, ParamSelect, SubTuple;
     I32 Result, SubLen;
     I32 dumb1, dumb2, dumb3, dumb4;

	pcis   += 2;

	/* Make pcheck point to the field TPCE_INDX, remaining length decrease of one byte. */
	pcheck  = pcis + TPCE_INDX;
	length -= 1;

	/* Intface and Default bits must be set in field TPCE_INDX of first table entry */
	if( IsFirst  &&  (*pcheck & TPCE_INDX_MASK) != TPCE_INDX_MASK )
	{
		/* Wrong checking on first CISTPL_CFTABLE_ENTRY tuple, but continue the scan, simply register the failure... */
		pentry->Is_TPCE_INDX_Ok = 0;
	}
	else    /* Even if not first ! */
	{
		pentry->Is_TPCE_INDX_Ok = 1;
	}

	/* Dealing with the field TPCE_INDX. */
	pentry->Is_TPCE_IF = (*pcheck & TPCE_INTFACE) ? 1:0;    /* If TRUE, the field TPCE_IF is present */
	pentry->EntryNumb  =  *pcheck & ~TPCE_INDX_MASK;        /* Configuration Entry Number to write into the COR */

	/* Dealing with the field TPCE_IF if any (New: its value is imposed) */
	pentry->Is_TPCE_IF_Ok = 0;                              /* Preset to FALSE */

	if( pentry->Is_TPCE_IF )
	{
		pcheck  = pcis + TPCE_IF;
		length -= 1;
		pentry->TPCE_IF_Value = *pcheck;                /* Memorize IF field in case of DVB entry */

		/* According to "Errata in EN 50221 and the CENELEC ..." IF value is determined. */
		if( pentry->TPCE_IF_Value == TPCE_IF_DVB_VAL )
		{
			pentry->Is_TPCE_IF_Ok = 1;
		}
	} /* case (see below) it must be equal to TPCE_IF_VAL */

	/* Dealing with the field TPCE_FS which offset depends on the previous field TPCE_IF presence. */
	pcheck  = pcis + ((pentry->Is_TPCE_IF) ? TPCE_FS : (TPCE_FS - 1));
	length -= 1;

	/* Memorize the Feature Selection byte value. */
	Features = *pcheck;

	if( (Features & TPCE_FS_PD_MASK)     != 0  &&
	    (Features & TPCE_FS_IO_IRQ_MASK) == TPCE_FS_IO_IRQ_MASK )
	{
		pentry->IsMasked_TPCE_FS_Ok = 1;
	}
	else
	{
		pentry->IsMasked_TPCE_FS_Ok = 0;
	}

	/* Make pcheck point on the byte following TPCE_FS field (maybe 1st Power Description structures) */
	pcheck = pcis + ((pentry->Is_TPCE_IF) ? TPCE_PD : (TPCE_PD - 1));

	/* Two first LS bits to indicate following Power Description Structures. Compliant to DVB ? */
	pentry->Is_TPCE_PD_Ok = 0;				/* Preset to 0. Set to 1 below if OK. */

	if( Features & TPCE_FS_PD_MASK )
	{
		I32   PdStructNbBytes = 0;
		I32   PdStructCount   = Features & TPCE_FS_PD_MASK;

		pentry->Is_TPCE_PD_Ok = 1;			/* The client must check electrical values */

		while( PdStructCount-- )
		{
			UI8 * pcheckpd;		/* To parse bytes inside each Power Description structure */
			I32             BitPos;			/* To parse bits inside each Param Selection Byte. */

			pcheckpd    = pcheck + PdStructNbBytes; /* Start pointing on Parameter Selection byte */
			ParamSelect = * pcheckpd;		/* Value of the Parameter Selection Byte. */

			++PdStructNbBytes;				/* For the Parameter Selection Byte itself. */

			for( BitPos = 0; BitPos < 8; BitPos++ )
			{
				/* No checking on stored electrical values now... The client is in charge of it. */

				if( ParamSelect & (1 << BitPos) )       /* Nom V, Min V, Max V, Static I, Avg I, Peak I, Pdwn I */
				{
					do						/* At least one byte for (Mantissa+Exponent) */
					{
						++pcheckpd;
						++PdStructNbBytes;
					} while( *pcheckpd & TPCE_EXT );
				}
			}
		}

		pcheck += PdStructNbBytes;			/* PdStructNbBytes may be null here... */
		length -= PdStructNbBytes;
	}

	/* Third LS bit inside TPCE_FS field indicates Timing Information follows. */
	if( Features & TPCE_TIMING )
	{
		UI8 * pcheckti       = pcheck;        /* Points on TPCE_TD field. */
		UI8   TiFieldValue   = * pcheckti;
		I32             TiFieldNbBytes = 0;

		++pcheckti;				/* Points now on WAIT, READY or RESERV value. */
		++TiFieldNbBytes;		/* For Timing Information field itself */

		if( (TiFieldValue & TPCE_TIMING_WAIT) != TPCE_TIMING_WAIT )/* WAIT value first */
		{
			while( *pcheckti & TPCE_EXT )	/* First bytes with TPCE_EXT bit set.  */
			{
				++pcheckti;
				++TiFieldNbBytes;
			}
			++pcheckti;				/* For last byte (maybe the first!)... */
			++TiFieldNbBytes;		/* ending with a TPCE_EXT bit zeroed.  */
		}
		if( (TiFieldValue & TPCE_TIMING_READY) != TPCE_TIMING_READY )           /* READY value */
		{
			while( *pcheckti & TPCE_EXT )	/* First bytes with TPCE_EXT bit set.  */
			{
				++pcheckti;
				++TiFieldNbBytes;
			}
			++pcheckti;				/* For last byte (maybe the first!)... */
			++TiFieldNbBytes;		/* ending with a TPCE_EXT bit zeroed.  */
		}
		if( (TiFieldValue & TPCE_TIMING_RESERV) != TPCE_TIMING_RESERV )         /* READY value */
		{
			while( *pcheckti & TPCE_EXT )                   /* First bytes with TPCE_EXT bit set.  */
			{
				++pcheckti;
				++TiFieldNbBytes;
			}
			++pcheckti;				/* For last byte (maybe the first!)... */
			++TiFieldNbBytes;		/* ending with a TPCE_EXT bit zeroed.  */
		}

		pcheck += TiFieldNbBytes;		/* Points now on next field first byte */
		length -= TiFieldNbBytes;
	}

	/* Fourth LS bit inside TPCE_FS field indicates IO Space Description follows. */
	pentry->Is_TPCE_IO_Ok = 0;		/* Preset to FALSE */

	if( Features & TPCE_IO )
	{
		I32   IOFieldNbBytes = 0;
		I32   RangesCount, LengthSize, AddressSize;

		++IOFieldNbBytes;			/* For IO Space Descript field itself */

		/* Get ride of the DVB compliance indicator about IOs: "Errata in EN 50221..." imposes val */
		if( *pcheck == TPCE_IO_DVB_VAL )
		{
			pentry->Is_TPCE_IO_Ok = 1;
		}

		if( *pcheck & TPCE_IO_RANGE )	/* Else only one byte for IOs. */
		{
			UI8 IORange = * (pcheck + 1);

			++IOFieldNbBytes;			/* For IO Addr Range Desc byte itself */

			RangesCount = (IORange & 0x0f) + 1;             /* The standard specifies '- 1' encoding */

			LengthSize  = (IORange & TPCE_IO_SLN) >> TPCE_IO_SLNR;
			if( LengthSize == 3 )
				LengthSize = 4;

			AddressSize = (IORange & TPCE_IO_SAN) >> TPCE_IO_SANR;
			if( AddressSize == 3 )
				AddressSize = 4;

			IOFieldNbBytes += RangesCount * (LengthSize + AddressSize);
		}

		pcheck += IOFieldNbBytes;		/* Points now on next field first byte */
		length -= IOFieldNbBytes;
	}

	/* Fifth LS bit inside TPCE_FS field indicates Interrupt Request Description follows. */
	pentry->Is_TPCE_IRQ_Ok = 0;			/* Preset to FALSE */

	if( Features & TPCE_IRQ )
	{
		I32   IRFieldNbBytes = 0;

		++IRFieldNbBytes;				/* For the first byte of the field (always) */

		if( *pcheck == TPCE_IR_DVB_VAL )
		{
			pentry->Is_TPCE_IRQ_Ok = 1;
		}

		if( *pcheck & TPCE_IRQ_MASK )		/* Else only one byte (this one) for IRQs. */
		{
			IRFieldNbBytes += 2;
		}

		pcheck += IRFieldNbBytes;
		length -= IRFieldNbBytes;
	}

	/* Sixth and seventh LS bits inside TPCE_FS field indicates Memory Space Description follows. */
	if( Features & TPCE_MEM )
	{
		I32 MEMFieldNbBytes;

		if( Features & TPCE_MEM  ==  TPCE_MEM_TWO_BYTES )
		{
			MEMFieldNbBytes = 2;
		}
		else if( Features & TPCE_MEM  ==  TPCE_MEM_FOUR_BYTES )
		{
			MEMFieldNbBytes = 4;
		}
		else					/* An array of window is present. */
		{
			UI8   MemSpaceDescr = *pcheck;
			I32 WindowCount, LengthSize, AddressSize;

			WindowCount = (MemSpaceDescr & TPCE_MEM_SLR) + 1;
			LengthSize  = (MemSpaceDescr & TPCE_MEM_SL)  >> TPCE_MEM_SLR;
			AddressSize = (MemSpaceDescr & TPCE_MEM_SCA) >> TPCE_MEM_SCAR;

			/* Evaluate first the window array size in several steps. */
			MEMFieldNbBytes = LengthSize + AddressSize;
			if( MemSpaceDescr & TPCE_MEM_HPA )              /* A host Address field is present */
			{												/* of same size as Card Address.   */
				MEMFieldNbBytes += AddressSize;
			}
			MEMFieldNbBytes *= WindowCount;

			/* Here only there is a Memory Space Descriptor Byte (in first position). */
			++MEMFieldNbBytes;
		}

		pcheck += MEMFieldNbBytes;
		length -= MEMFieldNbBytes;
	}

	/* Heighth LS bit inside TPCE_FS field indicates Miscellaneous Features Field follows. */
	if( Features & TPCE_MISC )
	{
		UI8 * pcheckmi	= pcheck;/* Points on TPCE_MI field. */
		I32 MiFieldNbBytes = 0;

		++MiFieldNbBytes;					/* At least one byte for the MISC field */

		while( *pcheckmi & TPCE_EXT )		/* Current byte with TPCE_EXT bit set ? */
		{
			++pcheckmi;
			++MiFieldNbBytes;				/* Because next byte belongs to MISC field */
		}

		pcheck += MiFieldNbBytes;			/* Points now on next field first byte */
		length -= MiFieldNbBytes;
	}

	pentry->Is_STCE_EV_Ok = 0;				/* Preset last flags to 0 */
	pentry->Is_STCE_PD_Ok = 0;

	while( length > 0 )
	{
		UI8 * dummy0;

		dummy0    = pcheck;
		SubTuple  = * pcheck;
		SubLen    = *(pcheck+1) + 2;
		length   -= SubLen;
		pcheck   += SubLen;

		Result = PHYS_Tuple( * (pcis-2), & dummy0, & SubLen, & dumb1, & dumb2, & dumb3, & dumb4 );

		if( Result == 1 )
		{
			if( SubTuple == STCE_EV )
			{
				pentry->Is_STCE_EV_Ok = 1;
			}
			else
			{
				pentry->Is_STCE_PD_Ok = 1;
			}
		}
	}

	if( pentry->Is_STCE_EV_Ok == 1  &&  pentry->Is_STCE_PD_Ok == 1 )
	{
		if( pentry->Is_TPCE_INDX_Ok     == 1  &&
			pentry->Is_TPCE_IF          == 1  &&  pentry->Is_TPCE_IF_Ok == 1  &&
			pentry->IsMasked_TPCE_FS_Ok == 1  &&
			pentry->Is_TPCE_PD_Ok       == 1  &&  pentry->Is_TPCE_IO_Ok == 1  &&  pentry->Is_TPCE_IRQ_Ok == 1 )
		{
			Result = 1;
		}
		else
		{
			Result = 0;
		}
	}
	else
	{
		Result = -1;
	}

	return( Result );
}

/* ================================================================================ */
/* NAME : PHYS_Tuple                                                                */
/*                                                                                  */
/* DESCRIPTION : control the current CIS                                            */
/*                                                                                  */
/* INPUT PARAMETERS :                                                               */
/*              mtuple:main tuple if tuple is a subtuple or 0                       */
/*              tuple: tuple to control                                             */
/*              pcis : pointer to the tuple to control                              */
/*              len  : length of the remainding CIS with the current tuple          */
/*              first: 1 for the first CISTPL_CFTABLE_ENTRY tuple, else 0           */
/*              itpcc_radr : configuration registers base address                   */
/*              last : index number of the last entry                               */
/*                                                                                  */
/* OUTPUT PARAMETERS :                                                              */
/*              pcis : pointer to the next tuple                                    */
/*              len : length of the remainding CIS without the current tuple        */
/*              radr: configuration registers base address (only for CISTPL_CONFIG) */
/*              first: 0 when an new entry is OK                                    */
/*              itcc_radr : configuration registers base address                    */
/*              dvb : index number of the dvb entry                                 */
/*              last: index number of the last entry                                */
/*                                                                                  */
/* RETURN CODE                                                                      */
/*                 1 : OK                                                           */
/*                 0 : one configuration table entry ok ; not the last              */
/*                -1 : error                                                        */
/*                -2 : wrong entry in the configuration Table                       */
/*                                                                                  */
/* ================================================================================ */
I32 PHYS_Tuple(UI8 mtuple, UI8 **pcis, I32 *len, I32 *first, I32 *itpcc_radr, I32 *dvb, I32 *last)
{
    I32 ocis=0;				/* current offset in the CIS */
    I32 link=0;				/* tuple link                            */
    UI8 tuple=0;			/* current tuple                         */
    I32 sublen=0;				/* intermediate length           */
    UI8 *subpcis;			/* pointer to subtuple           */
    I32 nb=0;  				/* intermediate variables        */
    char ctpcc_sz=0;      	// tuple's elementary item
    char lastentry=0;			// 1 if the last entry is found
    char ctpcc_rasz=0;
    char ctpcc_rmsz=0;

	ocis  = 0;
	tuple = **pcis;

//#if 0
	if( tuple == CISTPL_END )
	{
		if( *len == 1 )
		{
			*len = 0;		/* no more tuples            */
			return 1;		/* good CISTPL_END tuple: OK */
		}
	}
//#endif

	link   = *(*pcis+1);
	*pcis += 2;
	ocis  += 2;

	switch( tuple )
	{
		case CISTPL_VERS_1 :

			if (*(*pcis + TPLLV1_MAJOR) != TPLLV1_MAJOR_VAL)        /* Control */
				return -1;      /* wrong TPLLV1_MAJOR */                        /* ======= */

			if (*(*pcis + TPLLV1_MINOR) != TPLLV1_MINOR_VAL)        /* Control */
				return -1;      /* wrong TPLLV1_MINOR */                        /* ======= */

			break;

		case CISTPL_CONFIG :

			ctpcc_sz = *(*pcis + TPCC_SZ);
			ctpcc_rasz = ctpcc_sz & TPCC_RASZ;		/* number of bytes of Configuration Register Base Address */

			if (ctpcc_rasz > NBRBYTES)				/* Control : number = 0 or 1 means 1 or 2 bytes --> address between 0 and 0xfff */
				return -1;							/* ======= wrong Configuration Register Base Address */

			ctpcc_rmsz = ctpcc_sz & TPCC_RMSZ;
			*last  = *(*pcis + TPCC_LAST);			/* index number of last entry in Card configuration Table */

			if (ctpcc_rasz == 0)
				*itpcc_radr = *(*pcis + TPCC_RADR);
			else
				*itpcc_radr = *(*pcis + TPCC_RADR) + (*(*pcis + TPCC_RADR + 1) << 8);

			if (*itpcc_radr > MAXADDR)			/* Control */
				return -1;						/* ======= wrong Configuration Register Base Address */

			if (! ((*(*pcis + TPCC_RADR + ctpcc_rasz + 1)) & TPCC_RMSK))    /* Control */
				return -1;                                                  /* COR not present  ======= */

			nb = TPCC_RADR + ctpcc_rasz + 1 + ctpcc_rmsz + 1;
			subpcis = *pcis + nb;
			sublen = *len - ocis - (link - nb);

			/* control CCST_CIF subtuple */
			if (PHYS_Tuple( tuple, & subpcis, & sublen, first, itpcc_radr, dvb, last ) < 0)
				return -1;              /* Control not found or wrong CCST_CIF */

			break;                          /* =======  */

		case CISTPL_CFTABLE_ENTRY :
			{

				I32  Result;

				Result = PHYS_Tuple_CISTPL_CFTABLE_ENTRY( *pcis - 2, * first, &ThisEntry );

				if( Result >= 0 )
				{
					* dvb = ThisEntry.EntryNumb;
				}

				if( ThisEntry.EntryNumb == *last )
					lastentry = 1;                  /* it's the last entry (as defined in CISTPL_CONFIG) */
				else
				{
					lastentry = 0;                  /* not the last entry */
					* first   = 0;
				}
			}

			break;

		case CISTPL_NOLINK :

			if (link != CISTPL_NOLINKLEN)   /* Control */
				return -1;                  /* ======= wrong CISTPL_NOLINK tuple */

			break;

		/* subtuples */
		case CCST_CIF :     /* if mtuple == CISTPL_CFTABLE_ENTRY then this tuple is STCE_EV */

			if (mtuple == CISTPL_CFTABLE_ENTRY)
			{/* In fact subtuple type is STCE_EV which is of same numerical value as CCST_CIF */

				if (strcmp((char*)*pcis, STCE_EV_VAL ))         // Control
				
					return -1;                  // ======= wrong STCE_EV subtuple
				else
				{
					ocis  += link;
					*len  -= ocis;
					*pcis += ocis;
					return 1;
				}
			}
			else
			{/* Really CCST_CIF */

				if (**pcis != CCST_CIF1)        /* Control */
					return -1;                  /* ======= wrong CCST_CIF subtuple */

				if (*(*pcis + 1) != CCST_CIF2)  /* Control */
					return -1;                  /* ======= wrong CCST_CIF subtuple */

				/* STCI_STR must be "DVB_CI_Vx.xx" where x is digit */

				nb = strlen(STCI_STR);

				if (strncmp((char*)(*pcis + 2), STCI_STR, nb))   // Control
				
					return -1;                          // ======= wrong CCST_CIF subtuple
				if (!isdigit(*(*pcis + 2 + nb)))        // Control
					return -1;                          // ======= wrong CCST_CIF subtuple
				if (*(*pcis + 2 + nb + 1) != STCI_STRP) // Control
					return -1;                          // ======= wrong CCST_CIF subtuple
				if (!isdigit(*(*pcis + 2 + nb + 2)))    // Control
					return -1;                          // ======= wrong CCST_CIF subtuple
				if (!isdigit(*(*pcis + 2 + nb + 3)))    // Control
					return -1;                          // ====== wrong CCST_CIF subtuple

			}

			break;

		case STCE_PD :

			if (strcmp((char*)*pcis, STCE_PD_VAL)) // Control
				return -1;                  // ======= wrong STCE_PD subtuple
		    else
			{
				ocis  += link;
				*len  -= ocis;
				*pcis += ocis;
				return 1;
			}

			break;

		default :

			/* other tuples : no control */
			break;
	}

	/* skip the current tuple */
	ocis += link;

	if (ocis >= *len)       /* Control */
		return -1;              /* ======= tuple not found */

	*pcis = *pcis + link;
	*len -= ocis;

	if (tuple == CISTPL_CFTABLE_ENTRY)
		return lastentry;
	else
		return 1;
}

