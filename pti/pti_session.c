
#include "pti.h"
#include "pti_main.h"
#include "pti_session.h"

static u32 St20ToTcRamAddress(u32 SomeSt20Address, STPTI_TCParameters_t *TC_Params_p)
{
	return ((u32)((u8 *)SomeSt20Address - (u8 *)TC_Params_p->TC_DataStart + (u8 *)TC_DSRAM_BASE));
}

void pti_session_set_source(int index, int tsmerger_tag)
{
	TCSessionInfo_t *TCSessionInfo_p = &((TCSessionInfo_t *)tc_params.TC_SessionDataStart)[index];
	if (tsmerger_tag != STPTI_STREAM_ID_NONE)
	{
		// check the usage of the given stream ID
		// there must be no other session using the same TS input stream
		// otherwise the setting would disturb data arrival for all sessions
		int i;
		for (i = 0; i < tc_params.TC_NumberOfSessions; i++)
		{
			TCSessionInfo_p = &((TCSessionInfo_t *)tc_params.TC_SessionDataStart)[i];
			if (((readw(&TCSessionInfo_p->SessionTSmergerTag) &
					~SESSION_USE_MERGER_FOR_STC) == tsmerger_tag) && (i != index))
			{
				// source already in use
				printk("%s(%d,%x): ERROR STREAM ALREADY IN USE (%d)\n", __func__, index, tsmerger_tag, i);
				return;
			}
		}
	}
	TCSessionInfo_p = &((TCSessionInfo_t *)tc_params.TC_SessionDataStart)[index];
	STSYS_WriteTCReg16LE(&TCSessionInfo_p->SessionTSmergerTag,
			     tsmerger_tag | SESSION_USE_MERGER_FOR_STC);
}

/* **************************************
 * Init Session and return session number
 */
int pti_session_init(int tsmerger_tag, int slotOffset, int numberSlots)
{
	STPTI_TCParameters_t *TC_Params_p = &tc_params;
	TCSessionInfo_t *TCSessionInfo_p;
	u16 SlotsToAllocate;
	u16 BytesToAllocate;
	int session;
	//u8 numfilters;
	//int numblocks, filtermode, filtersperblock=SF_NUM_FILTERS_PER_BLOCK;
	// ******************************************
	//stptiHAL_PeekNextFreeSession(FullHandle_t DeviceHandle)
	for (session = 0; session < TC_Params_p->TC_NumberOfSessions; session++)
	{
		TCSessionInfo_p = &((TCSessionInfo_t *)TC_Params_p->TC_SessionDataStart)[session];
		if (readw((void *)&TCSessionInfo_p->SessionTSmergerTag) == SESSION_NOT_ALLOCATED)
		{
			printk("Free session: %d\n", session);
			break;
		}
	}
	if (session == TC_Params_p->TC_NumberOfSessions)
	{
		printk("No free session !!!!!!!!!!!!!!!!\n");
		return -1;
	}
	else
	{
		printk("Using session %d\n", session);
	}
	// ******************************************************
	//stptiHAL_GetNextFreeSession(FullHandle_t DeviceHandle)
	TCSessionInfo_p = &((TCSessionInfo_t *)TC_Params_p->TC_SessionDataStart)[session];
	pti_session_set_source(session, tsmerger_tag);
	// ********************************************************************
	//stptiHelper_TCInit_SessionInfoModeFlags( FullHandle_t DeviceHandle )
	/* Set DiscardSynchByte bit in the global data structure if STPTI has been initialised with this option */
	/* See DDTS GNBvd08680 PJW */
	//    if ( (Device_p->TCCodes == STPTI_SUPPORTED_TCCODES_SUPPORTS_DVB)
	/* DVB has a 1 byte header (the sync byte) */
	STSYS_SetTCMask16LE((void *)&TCSessionInfo_p->SessionModeFlags, (TC_GLOBAL_DATA_DISCARD_SYNC_BYTE_SET));
	STSYS_WriteTCReg16LE((void *)&TCSessionInfo_p->SessionDiscardParams, 1);
	// ********************************************************
	//stptiHelper_SlotList_Alloc( FullHandle_t DeviceHandle )
	SlotsToAllocate = numberSlots;
	BytesToAllocate = SlotsToAllocate * 2;
	STSYS_WriteTCReg16LE((void *)&TCSessionInfo_p->SessionPIDFilterStartAddr, St20ToTcRamAddress((u32)TC_Params_p->TC_LookupTableStart + slotOffset * 2, TC_Params_p));
	STSYS_WriteTCReg16LE((void *)&TCSessionInfo_p->SessionPIDFilterLength, SlotsToAllocate);
	// *****************************************************************
	//TcCam_FilterAllocateSession( FullHandle_t DeviceHandle )
	// case STPTI_FILTER_OPERATING_MODE_NONE:
	/* invalid filter mode i.e. all bits set - TC_SESSION_INFO_FILTER_TYPE_FIELD */
	STSYS_WriteTCReg16LE((void *)&TCSessionInfo_p->SessionSectionParams,
			     TC_SESSION_INFO_FILTER_TYPE_FIELD);
	STSYS_SetTCMask16LE((void *)&TCSessionInfo_p->SessionSectionParams, TC_SESSION_DVB_PACKET_FORMAT);
	STSYS_SetTCMask16LE((void *)&TCSessionInfo_p->SessionSectionParams, TC_SESSION_INFO_FORCECRCSTATE | TC_SESSION_INFO_DISCARDONCRCERROR);
	/* nothing allocated yet, so nothing enabled from our FilterList_p[n] */
	writel(0, (void *)&TCSessionInfo_p->SectionEnables_0_31);
	writel(0, (void *)&TCSessionInfo_p->SectionEnables_32_63);
#if defined(SECURE_LITE2)
	writel(0, (void *)&TCSessionInfo_p->SectionEnables_64_95);
	writel(0, (void *)&TCSessionInfo_p->SectionEnables_96_127);
#endif
	/* --- setup TC RAM CAM configuration --- */
	//STSYS_WriteTCReg16LE((void*)&TCSessionInfo_p->SessionCAMConfig, CAM_CFG_64_FILTERS );
	/* --- setup TC RAM CAM base address --- */
	/* Note: SessionCAMFilterStartAddr is at bit position 7:1 (7 bits) */
	/* point to just past start of reserved area (0x304) */
	//fixme: dies wird nur beim ersten mal gesetzt (siehe if abfrage im original,
	//ansonsten wird was anderes gemacht!)
	//STSYS_WriteTCReg16LE((void*)&TCSessionInfo_p->SessionCAMFilterStartAddr, (0xC1 << 1) ); /* 0x182 */
	return session;
}
