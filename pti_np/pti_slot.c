#include <dvb_demux.h>

#include "pti.h"
#include "pti_main.h"

/* ************************************
 * Set a pid to a slot
 */
void pti_slot_set_pid(int tc_slot_index, u16 Pid)
{
    STPTI_TCParameters_t* TC_Params_p = &tc_params;
    TCMainInfo_t         *MainInfo_p;
    
    // *****************************************************************
    //stptiHAL_SlotSetPid(FullHandle_t SlotHandle, U16 Pid, U16 MapPid)

    if (tc_slot_index < 0)
    {
    	printk("%s: invalid slot index passed %d\n", __func__, tc_slot_index);
	return;
    }

    MainInfo_p    = &((TCMainInfo_t *)TC_Params_p->TC_MainInfoStart)[tc_slot_index];

    STSYS_SetTCMask16LE((void*)&MainInfo_p->SlotState, (SLOT_STATE_INITIAL_SCRAMBLE_STATE) );

    /* Clear the PID */
    PutTCData( &((u16 *)TC_Params_p->TC_LookupTableStart)[tc_slot_index], 0xe000 /*STPTI_InvalidPid()*/ );

    /* Set the Mapping PID */

    STSYS_WriteTCReg16LE((void*)&MainInfo_p->RemainingPESLength, Pid);

    /* Set the PID */
    PutTCData( &((u16 *)TC_Params_p->TC_LookupTableStart)[tc_slot_index], Pid );

}

void pti_slot_clear_pid(int tc_slot_index, int tc_dma_index, int rewind)
{
#define TC_ONE_PACKET_TIME 75
    STPTI_TCParameters_t* TC_Params_p = &tc_params;
    TCMainInfo_t         *MainInfo_p;

    if (tc_slot_index < 0)
    {
    	printk("%s: invalid slot index passed %d\n", __func__, tc_slot_index);
	return;
    }

    MainInfo_p    = &((TCMainInfo_t *)TC_Params_p->TC_MainInfoStart)[tc_slot_index];

    /* Clear the PID */
    PutTCData( &((u16 *)TC_Params_p->TC_LookupTableStart)[tc_slot_index], 0xe000 /*STPTI_InvalidPid()*/ );

    udelay(TC_ONE_PACKET_TIME);

    /* rewind will be set if clearing the pid, if updating (not implemented)
     * the it will remain false. tc_dma_index kann kleiner null sein!
     */
    if ((rewind == 1) && (tc_dma_index >= 0))
    {
	int vLoop;
	TCDMAConfig_t * DMAConfig_p = &((TCDMAConfig_t *)TC_Params_p->TC_DMAConfigStart)[tc_dma_index];

	/* wait for dma completion */
    	for ( vLoop = 0; vLoop < 16; vLoop++ )
    	{
        	/* if 0 then dma not in progress */
        	if ( !(readw((void*)&MainInfo_p->SlotState) & TC_MAIN_INFO_SLOT_STATE_DMA_IN_PROGRESS) )
		    break;
		    
    		udelay(TC_ONE_PACKET_TIME);
    	}
         
        printk("QWrite = %x\n", DMAConfig_p->DMAQWrite_p);	        
	/* now rewind dma */
	DMAConfig_p->DMAWrite_p = DMAConfig_p->DMAQWrite_p;

        /* Reset Slot Status Word in slot's Main Info */
        STSYS_WriteTCReg16LE((void*)&MainInfo_p->SlotState, 0 );
    }
    
/* fixme: im orig machen sie noch eine Disassoziation des Descramblers 
    STSYS_WriteTCReg16LE((U32)&MainInfo_p->DescramblerKeys_p,TC_INVALID_LINK);
+ des Indexers stpti_SlotDisassociateIndex
*/

}

/* **********************************
 * Get a pid from a slot
 */
u16 pti_slot_get_pid(u32 tc_slot_index)
{
    u16 Pid;
    STPTI_TCParameters_t* TC_Params_p = &tc_params;
    
    if (tc_slot_index < 0)
    {
    	printk("%s: invalid slot index passed %d\n", __func__, tc_slot_index);
	return -1;
    }

    GetTCData( &((u16 *)TC_Params_p->TC_LookupTableStart)[tc_slot_index], Pid )
    
    return Pid;
}

/* ************************************
 * Allocate/Initialise a slot in a session
 */
void pti_slot_allocate(u32 tc_slot_index, int dvb_type, int dvb_pes_type)
{
    STPTI_TCParameters_t* TC_Params_p = &tc_params;
    int type = TC_SLOT_TYPE_NULL;
    
    if (tc_slot_index < 0)
    {
    	printk("%s: invalid slot index passed %d\n", __func__, tc_slot_index);
	return;
    }

    /* Aus den linuxDVB Params type und pes_type den tc type machen */
    switch (dvb_type) 
    {
 	 case DMX_TYPE_TS:
    	    if (dvb_pes_type != DMX_TS_PES_PCR)
	    	type = TC_SLOT_TYPE_RAW;
	    else
	    	type = TC_SLOT_TYPE_NULL;
    	 break;
  	 case DMX_TYPE_SEC:
    	  /*  type = TC_SLOT_TYPE_SECTION;*/
	  /* fixme: erstmal sec filter nicht behandeln, 
	   * da wir die filter daten hier nicht
	   * herbekommen. wir liefern der dvb api
	   * also die packete und sie soll den rest
	   * machen
	   */
	    type = TC_SLOT_TYPE_RAW;
    	 break;
  	 case DMX_TYPE_PES:
    	    type = TC_SLOT_TYPE_PES;
    	 break;
  	 default:
    	    type = TC_SLOT_TYPE_NULL;
	 break;
//FIXME: PCR braucht noch setup des indexers?!
   }

   // ********************************************************
   //SlotAllocate ->stptiHAL_SlotInitialise

   {
        TCMainInfo_t *MainInfo_p = &((TCMainInfo_t *)TC_Params_p->TC_MainInfoStart)[tc_slot_index];

        STSYS_WriteTCReg16LE((void*)&MainInfo_p->SlotState,          0);
        STSYS_WriteTCReg16LE((void*)&MainInfo_p->PacketCount,        0);

        STSYS_WriteTCReg16LE((void*)&MainInfo_p->SlotMode,           0);
        STSYS_WriteTCReg16LE((void*)&MainInfo_p->DescramblerKeys_p,  TC_INVALID_LINK);

        STSYS_WriteTCReg16LE((void*)&MainInfo_p->DMACtrl_indices,    0xffff);
        STSYS_WriteTCReg16LE((void*)&MainInfo_p->IndexMask,           0);

        STSYS_WriteTCReg16LE((void*)&MainInfo_p->SectionPesFilter_p, TC_INVALID_LINK);
        STSYS_WriteTCReg16LE((void*)&MainInfo_p->RemainingPESLength, 0);

        STSYS_WriteTCReg16LE((void*)&MainInfo_p->PESStuff,           0);
        STSYS_WriteTCReg16LE((void*)&MainInfo_p->StartCodeIndexing_p,0);

#if defined(SECURE_LITE2)
        STSYS_WriteTCReg16LE((void*)&MainInfo_p->SecondaryPid_p,    TC_SECONDARY_SLOT_NONE);
#endif

        STSYS_SetTCMask16LE((void*)&MainInfo_p->SlotMode, type);

#if !defined(A18) && !defined(REL23) && !defined(REL29)
        STSYS_SetTCMask16LE((void*)&MainInfo_p->SlotMode, TC_MAIN_INFO_SLOT_MODE_DISABLE_CC_CHECK);
#endif
   }
    
}

void pti_slot_link_to_buffer(u32 tc_slot_index, u32 tc_dma_index)
{
    u16 DMACtrl_indices;
    TCMainInfo_t *MainInfo_p;
    STPTI_TCParameters_t* TC_Params_p = &tc_params;
    TCDMAConfig_t * DMAConfig_p = &((TCDMAConfig_t *)TC_Params_p->TC_DMAConfigStart)[tc_dma_index];

    if (tc_slot_index < 0)
    {
    	printk("%s: invalid slot index passed %d\n", __func__, tc_slot_index);
	return;
    }

    if (tc_dma_index < 0)
    {
    	printk("%s: invalid dma index passed %d\n", __func__, tc_dma_index);
	return;
    }

    //printk("%s (slot %d, dma %d)>\n", __func__, tc_slot_index, tc_dma_index);

    // *******************************************************************************
    //TcHal_MainInfoAssociateDmaWithSlot( TC_Params_p, SlotIdent, DMAConfig_p );
    MainInfo_p = &((TCMainInfo_t *)TC_Params_p->TC_MainInfoStart)[tc_slot_index];
    DMACtrl_indices  = readl((void*)&MainInfo_p->DMACtrl_indices);
    DMACtrl_indices &= 0xFF00;            /* mask off main buffer index */
    DMACtrl_indices |= tc_dma_index & 0x00FF;  /* or in main buffer index */
    STSYS_WriteTCReg16LE((void*)&MainInfo_p->DMACtrl_indices, DMACtrl_indices);

#if defined(SECURE_LITE2)
    STSYS_WriteTCReg16LE((void*)&DMAConfig_p->BufferLevelThreshold, 0 );
#endif
}

void pti_slot_unlink_buffer(u32 tc_slot_index)
{
    STPTI_TCParameters_t* TC_Params_p = &tc_params;
    TCMainInfo_t *MainInfo_p;
    
    /* **************************
     * TcHal_MainInfoUnlinkDmaWithSlot(TC_Params_p, SlotIdent);
     */
    if (tc_slot_index < 0)
    {
    	printk("%s: invalid slot index passed %d\n", __func__, tc_slot_index);
	return;
    }


    MainInfo_p = &((TCMainInfo_t *)TC_Params_p->TC_MainInfoStart)[tc_slot_index];

    STSYS_SetTCMask16LE((void*)&MainInfo_p->DMACtrl_indices, 0xffff);
}

void pti_slot_free(u32 tc_slot_index)
{
    STPTI_TCParameters_t* TC_Params_p = &tc_params;
    int type = TC_SLOT_TYPE_NULL;
    
   /* dont trust user so invalidate pid ;-) */
   PutTCData( &((u16 *)TC_Params_p->TC_LookupTableStart)[tc_slot_index], 0xe000);

   /* stptiHelper_WaitPacketTimeDeschedule */
   udelay(320);
    
   {
    	TCMainInfo_t *MainInfo_p = &((TCMainInfo_t *)TC_Params_p->TC_MainInfoStart)[tc_slot_index];

   	STSYS_WriteTCReg16LE((void*)&MainInfo_p->SlotState,          0);
	STSYS_WriteTCReg16LE((void*)&MainInfo_p->SlotMode,           0);
	STSYS_WriteTCReg16LE((void*)&MainInfo_p->DescramblerKeys_p,  TC_INVALID_LINK);
	STSYS_WriteTCReg16LE((void*)&MainInfo_p->DMACtrl_indices,    0xffff);
	STSYS_WriteTCReg16LE((void*)&MainInfo_p->StartCodeIndexing_p,0);
	STSYS_WriteTCReg16LE((void*)&MainInfo_p->SectionPesFilter_p, TC_INVALID_LINK);
	STSYS_WriteTCReg16LE((void*)&MainInfo_p->RemainingPESLength, 0);
	STSYS_WriteTCReg16LE((void*)&MainInfo_p->PacketCount,        0);

    	STSYS_SetTCMask16LE((void*)&MainInfo_p->SlotMode, type );

   }

}

int pti_slot_get_state(u32 tc_slot_index)
{
    STPTI_TCParameters_t* TC_Params_p = &tc_params;
    
    TCMainInfo_t *MainInfo_p = &((TCMainInfo_t *)TC_Params_p->TC_MainInfoStart)[tc_slot_index];

    return MainInfo_p->SlotState;
}
