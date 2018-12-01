/* 
 * indent -bl -bli0 -cdb -sc -bap -bad -pcs -prs -bls -lp -npsl -bbb
 *
 * Hardware Abstraction Layer for our pti driver
 *
 *		  Session
 *                   |
 *		     |
 *          Slot1 ...... SlotN
 *            | (N:1)      | (N:1)
 *            |            |
 *           DMA      Descrambler
 *         
 * Currently Missing is the presentation of multiple ptis
 * as for 7109 architecture. Must think on this ;-)
 */

/* FIXME: 
 * - Da wir nur noch einen Buffer pro Session benutzen kann Session Loop eigentlich 
 * bei buffer sachen raus
 * - Descrambler sind auch nicht wirklich pro Slot ?!; muss ich noch einbauen
 */
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/errno.h>

#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#include <linux/platform_device.h>
#include <linux/mutex.h>

#include <asm/io.h>
#if defined (CONFIG_KERNELVERSION) || LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#include <linux/bpa2.h> 
#else
#include <linux/bigphysarea.h>
#endif
#include <dvb_demux.h>

/* fixme: this one is still sucking */
#include "../player2/linux/drivers/media/dvb/stm/dvb/dvb_module.h"

#include "pti_hal.h"
#include "pti.h"
#include "pti_main.h"
#include "pti_session.h"
#include "pti_slot.h"
#include "pti_buffer.h"
#include "pti_descrambler.h"

extern void setDmaThreshold(int a, int b);

/* ********************************
 * Definitions
 */

#define SessionHandleStart	10000
#define SlotHandleStart		20000
#define DMAHandleStart		30000	/* DMA Handle = Buffer Handle */
#define DescramblerHandleStart	40000

/* fixme: bringt e2 zum haengen
 * mal mit spin lock probieren?!
 */
#undef use_hal_mutex

/* ********************************
 * Global Vars
 */

static int numberOfAvailableDMAs = 0;
static int numberOfAvailableSlots = 0;
static int numberOfAvailableCarousels = 0;	/* not used */
static int numberOfAvailableSectionFilter = 0;
static int numberOfAvailableSessions = 0;
static int numberOfAvailableSectionIndex = 0;
static int numberOfAvailableDescramblerKeys = 0;

/* fixme: hacky ->think about this */
void (*demultiplexDvbPackets)(struct dvb_demux* demux, const u8 *buf, int count) = NULL;
struct stpti *pti = NULL;

#define AUD_BUF_SIZE (128 * 188)
#define MISC_BUF_SIZE (256 * 188)

#define AUD_BUF_NUM 10
#define MISC_BUF_NUM 3

extern int videoMem;
// see TC_Params_p->TC_NumberDMAs
int max_pti_dma = 0X0037;

static int BufferSizes[] =
{
  0
};

static int BufferCounts[] =
{
  1
};
  

/* ********************************
 * Structures
 */

struct tSlot
{
  int Handle;			/* Our Handle */

  u32 TCIndex;			/* Index in the TC Structure */

  int inUse;

  int SessionHandle;		/* Our Session Handle */
  struct tBuffer *pBuffer;	/* Our Buffer/DMA Handle if linked */
  int DescramblerHandle;	/* Our Descrambler Handle if linked */

  u16 pid;			/* Current set pid */

  int dvb_type;			/* linuxDVB Type */
  int dvb_pes_type;		/* linuxDVB pes Type */

  struct dvb_demux *demux;	/* linuxDVB Demuxer */

  struct StreamContext_s *DemuxStream;
  struct DeviceContext_s *DeviceContext;

};

struct tDescrambler
{
  int Handle;			/* Our Handle */

  u32 TCIndex;			/* Index in the TC Structure */

  int inUse;

  int SlotHandle;		/* Our Slot Handle if linked */

};

struct tSession
{
  int Handle;			/* Our Handle */

  u32 TCIndex;			/* Index in the TC Structure */
  u32 TSMergerTag;
  u32 OldTSMergerTag;

  int inUse;

  struct tSlot **vSlots;
  struct tDescrambler **vDescrambler;

  struct tBuffer *pBufferList;
};

static struct tSession **vSessions = NULL;

#ifdef use_hal_mutex

/* fixme: hatte vor das ganze mal mit spinlocks zu testen
 * deshalb hier mit eigenen funktionen
 */

static struct mutex HALLock;

int hal_mutex_lock ( struct mutex *lock )
{
  mutex_lock ( lock );
}

int hal_mutex_unlock ( struct mutex *lock )
{
  mutex_unlock ( lock );
}

#endif

static int pti_hal_convert_source ( const tInputSource source, int *pTag );

/* **************************************************************
 * Intialize our data structures depening on TC-Code. For this
 * reason we call pti_main_loadtc() here.
 */
void pti_hal_init ( struct stpti *pti , struct dvb_demux* demux, void (*_demultiplexDvbPackets)(struct dvb_demux* demux, const u8 *buf, int count), int numVideoBuffers)
{
  STPTI_TCParameters_t *TC_Params_p = &tc_params;
  int vLoopSession, vLoopSlots, vLoopDescrambler;
  int i;

  dprintk ( "%s >\n", __func__ );

  pti_main_loadtc ( pti );

  numberOfAvailableSessions = TC_Params_p->TC_NumberOfSessions;

  numberOfAvailableCarousels = TC_Params_p->TC_NumberCarousels;
  numberOfAvailableDMAs = TC_Params_p->TC_NumberDMAs;
  numberOfAvailableDescramblerKeys =
    TC_Params_p->TC_NumberDescramblerKeys / TC_Params_p->TC_NumberOfSessions;
  numberOfAvailableSectionFilter = TC_Params_p->TC_NumberSectionFilters;
  numberOfAvailableSlots =
    TC_Params_p->TC_NumberSlots / TC_Params_p->TC_NumberOfSessions;
  numberOfAvailableSectionIndex = TC_Params_p->TC_NumberIndexs;

#ifdef use_hal_mutex
  mutex_init ( &HALLock );
#endif

  BufferCounts[0] = numVideoBuffers;

  /* Dagobert: goHackHack: I would like to make the buffer adjustable,
   * vor first glance I only make the videoBufferSize adjustable 
   * and assume it is on position 0 in this array!
   */
  BufferSizes[0] = videoMem * 188; 

  // calc the max dma buffers for pti_task
  // use of static value (TC_Params_p->TC_NumberDMAs) needs performance
  max_pti_dma = 0;
  for(i=0; i<sizeof(BufferCounts)/sizeof(BufferCounts[0]); i++)
  {
     max_pti_dma += BufferCounts[i];
  }

  printk("using videoMem = %d (packets = %d), max_dma %d\n", videoMem * 188, videoMem, max_pti_dma);

  /* initialize the buffer pools */
  pti_buffer_pool_init(sizeof(BufferSizes)/sizeof(BufferSizes[0]),
                       BufferSizes, BufferCounts);

  /*
   * very memory wasting but I think its ok 
   */

  vSessions =
    kmalloc ( sizeof ( struct tSession * ) * numberOfAvailableSessions,
	      GFP_KERNEL );

  for ( vLoopSession = 0; vLoopSession < numberOfAvailableSessions;
	vLoopSession++ )
  {
    vSessions[vLoopSession] =
      kmalloc ( sizeof ( struct tSession ), GFP_KERNEL );

    memset ( vSessions[vLoopSession], 0, sizeof ( struct tSession ) );

    /*
     * set-up handle 
     */
    vSessions[vLoopSession]->Handle = SessionHandleStart + vLoopSession;

    vSessions[vLoopSession]->TCIndex = vLoopSession;

    vSessions[vLoopSession]->vSlots =
      kmalloc ( sizeof ( struct tSlot * ) * numberOfAvailableSlots,
		GFP_KERNEL );

    for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots; vLoopSlots++ )
    {
      vSessions[vLoopSession]->vSlots[vLoopSlots] =
	kmalloc ( sizeof ( struct tSlot ), GFP_KERNEL );

      memset ( vSessions[vLoopSession]->vSlots[vLoopSlots], 0,
	       sizeof ( struct tSlot ) );

      /*
       * set-up handles 
       */
      vSessions[vLoopSession]->vSlots[vLoopSlots]->Handle =
	SlotHandleStart + vLoopSlots + ( vLoopSession * 1000 );

      /*
       * slots will be count globally from 0 to max 
       */
      vSessions[vLoopSession]->vSlots[vLoopSlots]->TCIndex =
	vLoopSession * numberOfAvailableSlots + vLoopSlots;
    }

    vSessions[vLoopSession]->vDescrambler =
      kmalloc ( sizeof ( struct tDescrambler * ) *
		numberOfAvailableDescramblerKeys, GFP_KERNEL );

    for ( vLoopDescrambler = 0;
	  vLoopDescrambler < numberOfAvailableDescramblerKeys;
	  vLoopDescrambler++ )
    {
      vSessions[vLoopSession]->vDescrambler[vLoopDescrambler] =
	kmalloc ( sizeof ( struct tDescrambler ), GFP_KERNEL );

      memset ( vSessions[vLoopSession]->vDescrambler[vLoopDescrambler], 0,
	       sizeof ( struct tDescrambler ) );

      /*
       * set-up handles 
       */
      vSessions[vLoopSession]->vDescrambler[vLoopDescrambler]->Handle =
	DescramblerHandleStart + vLoopDescrambler + ( vLoopSession * 1000 );

      /*
       * descrambler will be count globally from 0 to max 
       */
      vSessions[vLoopSession]->vDescrambler[vLoopDescrambler]->TCIndex =
	vLoopSession * numberOfAvailableDescramblerKeys + vLoopDescrambler;
    }

  }

  demultiplexDvbPackets = _demultiplexDvbPackets;

  // FIXME: remove
  //pti_session_init ( STPTI_STREAM_ID_TSIN1, vSessions[0]->vSlots[0]->TCIndex,
  //		     numberOfAvailableSlots );
#ifdef use_irq
  dprintk ( "%s !!!!!!!!!!!!! start irq !!!!!!!!!!!!!!!\n", __func__ );

  if ( request_irq
       ( 160, pti_interrupt_handler, SA_INTERRUPT, "PTI", pti ) != 0 )
  {
    dprintk ( "request irq failed\n" );
  }

  //enable the pti interrupt     
  {
  TCDevice_t *myTC = ( TCDevice_t * ) pti->pti_io;

  writel ( 3, ( void * ) &myTC->PTIIntEnable0 );
  //writel ( 0x7, pti->pti_io + PTI_DMAEMPTY_EN );
  }
#endif

  spin_lock_init ( &pti->irq_lock );
  kernel_thread ( pti_task, pti, 0 );

  dprintk ( "%s <\n", __func__ );
}

/* ************************************************************* 
 * Helper Functions for Sessions
 */

/* *********************************
 * Get a new session handle.
 * pti_hal_init must be called before.
 * 
 * O: Errorcode or session handle
 */
int pti_hal_get_new_session_handle ( tInputSource source, struct dvb_demux * demux )
{
  int session_handle = cHALNoFreeSession;
  int vLoopSessions;
  int srcTag = 0;

  dprintk ( "%s >\n", __func__ );

  if ( vSessions == NULL )
    return cHALNotInitialized;

  if ( pti_hal_convert_source ( source, &srcTag ) != 0 )
  {
    return cHALNotInitialized;
  }

#ifdef use_hal_mutex
  hal_mutext_lock ( &( HALLock ) );
#endif

  // FIXME: start with 0
  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->inUse == 0 )
    {
      session_handle = vSessions[vLoopSessions]->Handle;
      vSessions[vLoopSessions]->inUse = 1;
      vSessions[vLoopSessions]->TSMergerTag = STPTI_STREAM_ID_NONE;
      vSessions[vLoopSessions]->OldTSMergerTag = STPTI_STREAM_ID_NONE;

      vSessions[vLoopSessions]->TCIndex =
      pti_session_init ( STPTI_STREAM_ID_NONE, //STPTI_STREAM_ID_TSIN0 + vLoopSessions,
                           vSessions[vLoopSessions]->vSlots[0]->TCIndex,
                           numberOfAvailableSlots );

      /*
       * this should never happen! 
       */
      if ( vSessions[vLoopSessions]->TCIndex == -1 )
      {
	vSessions[vLoopSessions]->inUse = 0;
	session_handle = cHALNoFreeSession;
      }

      break;
    }
  }

#ifdef use_hal_mutex
  hal_mutex_unlock ( &( HALLock ) );
#endif

  dprintk ( "%s (%d) <\n", __func__, session_handle );
  return session_handle;
}

int pti_hal_get_session_handle ( int tc_session_number )
{
  int session_handle = cHALNotInitialized;
  int vLoopSessions;

//      dprintk("%s >\n", __func__);

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( ( vSessions[vLoopSessions]->inUse == 1 )
	 && ( vSessions[vLoopSessions]->TCIndex == tc_session_number ) )
    {
      session_handle = vSessions[vLoopSessions]->Handle;
      break;
    }
  }

//      dprintk("%s (%d)<\n", __func__, session_handle);

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif

  return session_handle;
}

/* **************************************
 * Die Slots sind zwar global definiert
 * aber auf die Sessions aufgeteilt,
 * so koennen wir bestimmen wie die
 * interne session nummer ist.
 */
int pti_hal_get_session_number_from_tc_slot_number ( int tc_slot_number )
{
  int tc_session_number = cHALNotInitialized;
  int vLoopSessions, vLoopSlots;

//      dprintk("%s >\n", __func__);

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->inUse == 1 )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 1 ) &&
	     ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->TCIndex ==
	       tc_slot_number ) )
	{
	  tc_session_number = vSessions[vLoopSessions]->TCIndex;
	  break;
	}
      }
    }
  }

//      dprintk("%s (%d) <\n", __func__, tc_session_number);

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return tc_session_number;
}

struct dvb_demux * pti_hal_get_demux_from_dma_index ( int dmaIndex )
{
  int vLoopSessions;
  int vLoopSlots;
  struct dvb_demux *pDemux = NULL;

  if ( vSessions == NULL )
    return NULL;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->inUse == 1 )
    {
      struct tBuffer *pBuffer = vSessions[vLoopSessions]->pBufferList;

      /* find the buffer assigned to the specified DMA */
      while ( pBuffer != NULL)
      {
	if ( pBuffer->dmaIndex == dmaIndex )
	{
          /* all slots of a session must be assigned to the same demux,
             so take the demux from the first valid slot */
	  for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
		vLoopSlots++ )
	  {
	    if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 1 )
	    {
	      pDemux = vSessions[vLoopSessions]->vSlots[vLoopSlots]->demux;
	      break;
	    }
	  }
	}
	pBuffer = pBuffer->pNext;
      }
      if(pDemux != NULL)
        break;
    }
  }

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return pDemux;
}

int pti_hal_free_session ( int session_handle )
{

/* fixme: free all slots, dmas ... */
  printk ( "FIXME: %s not implemented currently \n", __func__ );
  return -1;
}

/* ************************************************************* 
 *
 * Helper function for setting the session source (frontend)
 *
 *************************************************************/
static int pti_hal_convert_source ( const tInputSource source, int *pTag )
{
  *pTag = source + STPTI_STREAM_ID_TSIN0;

  return 0;
}

int pti_hal_set_source ( int session_handle, const tInputSource source )
{
  int i;
  int tsmerger_tag = 0;

  if ( pti_hal_convert_source ( source, &tsmerger_tag ) != 0 )
  {
    return -EINVAL;
  }

  for ( i = 0; i < numberOfAvailableSessions; i++ )
  {
    if ( vSessions[i]->Handle == session_handle )
    {
      pti_session_set_source ( vSessions[i]->TCIndex, tsmerger_tag );
      vSessions[i]->TSMergerTag = tsmerger_tag;
      vSessions[i]->OldTSMergerTag = tsmerger_tag;
      printk( "%s(): %d, %d\n", __func__, vSessions[i]->TCIndex, tsmerger_tag);
      break;
    }
  }

  if ( i == numberOfAvailableSessions )
  {
    printk ( "%s(): invalid session (%x)\n", __func__, session_handle );
    return -EINVAL;
  }

  return 0;
}

/* ************************************************************* 
 * Helper Functions for Slots
 */

int pti_hal_get_new_slot_handle ( int session_handle, int dvb_type,
				  int dvb_pes_type, struct dvb_demux *demux,
				  struct StreamContext_s *DemuxStream,
				  struct DeviceContext_s *DeviceContext )
{
  int slot_handle = cHALNotInitialized;
  int noFree = 0;
  int vLoopSessions, vLoopSlots;
  int numSlotsInUse = 0;

  dprintk ( "%s >\n", __func__ );

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions; vLoopSessions++)
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
        if (vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse)
            numSlotsInUse++;
      }

      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 0 )
	{
	  slot_handle = vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle;
	  pti_slot_allocate ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->
			      TCIndex, dvb_type, dvb_pes_type );

	  /*
	   * this should never happen 
	   */
	  if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->TCIndex == -1 )
	  {
	    slot_handle = cHALNoFreeSlot;
	  }
	  else
	  {
	    vSessions[vLoopSessions]->vSlots[vLoopSlots]->demux = demux;
	    vSessions[vLoopSessions]->vSlots[vLoopSlots]->DemuxStream =
	      DemuxStream;
	    vSessions[vLoopSessions]->vSlots[vLoopSlots]->DeviceContext =
	      DeviceContext;
	    vSessions[vLoopSessions]->vSlots[vLoopSlots]->dvb_type = dvb_type;
	    vSessions[vLoopSessions]->vSlots[vLoopSlots]->dvb_pes_type =
	      dvb_pes_type;
	    vSessions[vLoopSessions]->vSlots[vLoopSlots]->SessionHandle =
	      vSessions[vLoopSessions]->Handle;
	    vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse = 1;
	  }

      if (numSlotsInUse == 0) /* then this is first again */
      {
         if ((vSessions[vLoopSessions]->OldTSMergerTag != vSessions[vLoopSessions]->TSMergerTag) &&
             (vSessions[vLoopSessions]->OldTSMergerTag != STPTI_STREAM_ID_NONE))
         {
            vSessions[vLoopSessions]->TSMergerTag = vSessions[vLoopSessions]->OldTSMergerTag;
            pti_session_set_source(vSessions[vLoopSessions]->TCIndex, vSessions[vLoopSessions]->OldTSMergerTag);
            printk("*********** automatic set stream id to %d for Session %d-%d\n", vSessions[vLoopSessions]->OldTSMergerTag, vLoopSessions, vSessions[vLoopSessions]->TCIndex);
         }
         //printk("old tag %d, new tag %d numinuse %d\n", vSessions[vLoopSessions]->OldTSMergerTag, vSessions[vLoopSessions]->TSMergerTag, numSlotsInUse);
      }
	  noFree = 1;
	  break;
	}
      }				/* for slots */

      if ( noFree == 0 )
	slot_handle = cHALNoFreeSlot;

    }
  }				/* for sessions */

  dprintk ( "%s (%d) <\n", __func__, slot_handle );

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return slot_handle;
}

int pti_hal_slot_set_pid ( int session_handle, int slot_handle, u16 pid )
{
  int error = cHALNotInitialized;
  int vLoopSessions, vLoopSlots;

  dprintk ( "%s >\n", __func__ );

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
    
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle ==
	     slot_handle )
	{
	  if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->
	       DescramblerHandle != 0 )
	  {
	    int vLoopDescrambler;

	    for ( vLoopDescrambler = 0;
		  vLoopDescrambler < numberOfAvailableDescramblerKeys;
		  vLoopDescrambler++ )
	    {
	      if ( ( vSessions[vLoopSessions]->
		     vDescrambler[vLoopDescrambler]->Handle ==
		     vSessions[vLoopSessions]->vSlots[vLoopSlots]->
		     DescramblerHandle )
		   && ( vSessions[vLoopSessions]->
			vDescrambler[vLoopDescrambler]->inUse == 1 ) )
	      {
		//u8 Data[8] = {0,0,0,0,0,0,0,0};        
		/*
		 * set descrambler invalid 
		 * pti_descrambler_set(vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->TCIndex, 0, &Data[0]);
		 * pti_descrambler_set(vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->TCIndex, 1, &Data[0]);
		 */
	      }
	    }
	  }

	  vSessions[vLoopSessions]->vSlots[vLoopSlots]->pid = pid;
	  pti_slot_set_pid ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->TCIndex, pid );
	  error = cHALNoError;

	  break;
	}
      }				/* for slots */
    }
  }				/* for sessions */

  dprintk ( "%s (%d) <\n", __func__, error );

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return error;
}

int pti_hal_slot_clear_pid ( int session_handle, int slot_handle )
{
  int error = cHALNotInitialized;
  int vLoopSessions, vLoopSlots;

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle ==
	     slot_handle )
	{
	  if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->
	       DescramblerHandle != 0 )
	  {
	    int vLoopDescrambler;

	    for ( vLoopDescrambler = 0;
		  vLoopDescrambler < numberOfAvailableDescramblerKeys;
		  vLoopDescrambler++ )
	    {
	      if ( ( vSessions[vLoopSessions]->
		     vDescrambler[vLoopDescrambler]->Handle ==
		     vSessions[vLoopSessions]->vSlots[vLoopSlots]->
		     DescramblerHandle )
		   && ( vSessions[vLoopSessions]->
			vDescrambler[vLoopDescrambler]->inUse == 1 ) )
	      {
		//u8 Data[8] = {0,0,0,0,0,0,0,0};        
		/*
		 * set descrambler invalid 
		 * pti_descrambler_set(vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->TCIndex, 0, &Data[0]);
		 * pti_descrambler_set(vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->TCIndex, 1, &Data[0]);
		 */
	      }
	    }
	  }
	  vSessions[vLoopSessions]->vSlots[vLoopSlots]->pid = 0;

	  pti_slot_clear_pid ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->
			       TCIndex,
			       pti_hal_get_tc_dma_number ( session_handle,
							   slot_handle ), 1 );

	  error = cHALNoError;

	  break;
	}
      }				/* for slots */
    }
  }				/* for sessions */

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return error;
}

int pti_hal_slot_link_buffer ( int session_handle, int slot_handle,
			       BUFFER_TYPE bufType)
{
  int error = cHALNotInitialized;
  int vLoopSessions, vLoopSlots;
  struct tBuffer *pBuffer = NULL;
  int found = 0;
  int allocated = 0;

  dprintk ( "%s >\n", __func__ );

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle ==
	     slot_handle )
	{
	  if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->pBuffer !=
	       NULL )
	  {
	    error = cHALSlotAlreadyInUse;
	  }
	  else
	  {
            found = 1;
	    break;
	  }
	}
      }	
      if(found)
        break;
    }
  }

  if(found)
  {
#if defined(SEPARATE_MISC_BUFFERS)

    /* SEC and TS types other than video and audio are
       collected in separate buffers */
    if (bufType == MISC_BUFFER)
    {
      pBuffer = pti_buffer_get( MISC_BUF_SIZE);
      allocated = 1;
    }

#endif

#if defined(SEPARATE_AUDIO_BUFFERS)

    /* audio TS packets are collected in separate buffers */
    if (bufType == AUD_BUFFER)
    {
      pBuffer = pti_buffer_get(AUD_BUF_SIZE);
      allocated = 1;
    }

#endif

    if(pBuffer == NULL)
    {
      /* no buffer found so far
         check whether a video buffer is already allocated */
      //if(bufType != VID_BUFFER)
      {
	pBuffer = vSessions[vLoopSessions]->pBufferList;
	while(pBuffer != NULL)
	{
	  if(pBuffer->bufSize == videoMem * 188)
	  {
	    break;
	  }

	  pBuffer = pBuffer->pNext;
	}
      }

      if(pBuffer == NULL)
      {
        /* no video buffer in the list yet */
	pBuffer = pti_buffer_get(videoMem * 188);
	allocated = 1;
      }

    }

    if(pBuffer == NULL)
    {
      return error;
    }
  
    /* prepend the buffer to the list head */
    if(allocated)
    {
      pBuffer->pNext = vSessions[vLoopSessions]->pBufferList;
      vSessions[vLoopSessions]->pBufferList = pBuffer;
    }

    pBuffer->sessionHandle = session_handle;

    if(bufType == VID_BUFFER)
    {
      /* increase the DMA threshold to reduce the interrupt rate caused by
	 the video stream */
      setDmaThreshold(pBuffer->dmaIndex, TC_DMA_THRESHOLD_HIGH);
    }

    pti_slot_link_to_buffer ( vSessions[vLoopSessions]->
			      vSlots[vLoopSlots]->TCIndex,
			      pBuffer->dmaIndex );

    pBuffer->slotCount++;

    vSessions[vLoopSessions]->vSlots[vLoopSlots]->pBuffer = pBuffer;

    error = cHALNoError;
  }

  dprintk ( "%s (%d) <\n", __func__, error );


#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif

  return error;
}

int pti_hal_get_slot_handle ( int session_handle, int tc_slot_number )
{
  int slot_handle = cHALNotInitialized;
  int vLoopSessions, vLoopSlots;

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->TCIndex ==
	       tc_slot_number )
	     && vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 1 )
	{
	  slot_handle = vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle;
	  break;
	}
      }
    }
  }

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return slot_handle;
}

struct dvb_demux *pti_hal_get_demux_from_slot ( int session_handle,
						int slot_handle )
{
  struct dvb_demux *demux = NULL;
  int vLoopSessions, vLoopSlots;

  if ( vSessions == NULL )
    return NULL;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle ==
	       slot_handle )
	     && vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 1 )
	{
	  demux = vSessions[vLoopSessions]->vSlots[vLoopSlots]->demux;
	  break;
	}
      }
    }
  }

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return demux;
}

int pti_hal_get_type_from_slot ( int session_handle, int slot_handle,
				 int *ts_type, int *pes_type )
{
  int error = cHALNotInitialized;
  int vLoopSessions, vLoopSlots;

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle ==
	       slot_handle )
	     && vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 1 )
	{

	  *ts_type = vSessions[vLoopSessions]->vSlots[vLoopSlots]->dvb_type;
	  *pes_type =
	    vSessions[vLoopSessions]->vSlots[vLoopSlots]->dvb_pes_type;
	  error = cHALNoError;
	  break;
	}
      }
    }
  }

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return error;
}

struct StreamContext_s *pti_hal_get_stream_from_slot ( int session_handle,
						       int slot_handle )
{
  struct StreamContext_s *stream = NULL;

  int vLoopSessions, vLoopSlots;

  if ( vSessions == NULL )
    return NULL;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle ==
	       slot_handle )
	     && vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 1 )
	{

	  stream = vSessions[vLoopSessions]->vSlots[vLoopSlots]->DemuxStream;
	  break;
	}
      }
    }
  }

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return stream;
}

struct DeviceContext_s *pti_hal_get_context_from_slot ( int session_handle,
							int slot_handle )
{
  struct DeviceContext_s *context = NULL;

  int vLoopSessions, vLoopSlots;

  if ( vSessions == NULL )
    return NULL;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle ==
	       slot_handle )
	     && vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 1 )
	{

	  context =
	    vSessions[vLoopSessions]->vSlots[vLoopSlots]->DeviceContext;
	  break;
	}
      }
    }
  }

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return context;
}

void pti_hal_get_slots_for_pid ( int session_handle, u16 pid, int **slots,
				 int *number_slots )
{
  int vLoopSessions, vLoopSlots;

  if ( vSessions == NULL )
    return;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  *slots = NULL;
  *number_slots = 0;

  //dprintk("%s >(session_handle %d, pid %d)\n", __func__,session_handle, pid);   

  /*
   * I think in the kernel is no realloc so I first count the slots ;-) 
   */
  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
//                      dprintk("test pid = %d\n", vSessions[vLoopSessions]->vSlots[vLoopSlots]->pid); 
	if ( ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 1 ) &&
	     ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->pid == pid ) )
	{
	  *number_slots += 1;
	}
      }
    }
  }

  if ( *number_slots != 0 )
  {
    int *vSlots;
    int count = 0;

    vSlots = kmalloc ( sizeof ( int ) * *number_slots, GFP_KERNEL );

    for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	  vLoopSessions++ )
    {
      if ( vSessions[vLoopSessions]->Handle == session_handle )
      {
	for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	      vLoopSlots++ )
	{
	  if ( ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 1 ) &&
	       ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->pid == pid ) )
	  {
	    vSlots[count++] =
	      vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle;
	  }
	}
      }
    }

    if ( count != *number_slots )
      printk ( "something went wrong\n" );

    /*
     * remeber to free outside 
     */
    *slots = vSlots;
  }

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
}


int pti_hal_slot_unlink_buffer ( int session_handle, int slot_handle)
{
  int error = cHALNotInitialized;
  int vLoopSessions, vLoopSlots;
  struct tBuffer *pBuffer = NULL;

  dprintk ( "%s > %x, %x, %p\n", __func__, session_handle, slot_handle,
	    pBuffer );

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle ==
	       slot_handle )
	     && vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 1 )
	{
	  /*
	   * unlink buffer / dma from slot in hw 
	   */
	  pti_slot_unlink_buffer ( vSessions[vLoopSessions]->
				   vSlots[vLoopSlots]->TCIndex );

          pBuffer = vSessions[vLoopSessions]->vSlots[vLoopSlots]->pBuffer;
          if(pBuffer != NULL)
          {
	    if( pBuffer->slotCount > 0)
	      pBuffer->slotCount--;
	    else
	      printk("%s(): INVALID SLOT COUNT (%d)\n", __func__,
                                   pBuffer->slotCount);

	    if(pBuffer->slotCount == 0)
	    {
              /* invalidate references to the buffer */
              if(vSessions[vLoopSessions]->pBufferList == pBuffer)
              {
                /* the buffer is the first on the list */
                vSessions[vLoopSessions]->pBufferList =
                               vSessions[vLoopSessions]->pBufferList->pNext;
              }
              else
              {
                struct tBuffer *pTmp = vSessions[vLoopSessions]->pBufferList;
                int loop = 0;

                while(pTmp != NULL)
                {
                  printk(" pTmp = %p\n", pTmp);
                  if(pTmp->pNext == pBuffer)
                  {
                    /* remove the buffer from the list */
                    pTmp->pNext = pBuffer->pNext;
                    break;
                  }
                  if((pTmp == pBuffer->pNext) || (loop > 50))
                  {
                    printk("%s(): LIST ERROR %p, %p, %p\n", __func__, pTmp, pBuffer, pBuffer->pNext);
                    pTmp->pNext = NULL;
                    break;
                  }
                  pTmp = pTmp->pNext;
                  loop++;
                }

                if(pTmp == NULL)
                  printk("%s(): buffer not found in the list\n", __func__);
              }

	      /* no buffer references, release buffer */
	      pti_buffer_free(pBuffer);
	    }
            else
            {
              if(vSessions[vLoopSessions]->vSlots[vLoopSlots]->dvb_pes_type == DMX_TS_PES_VIDEO)
              {
		/* increase the DMA threshold to allow low-bitrate 
		   interrupts to arrive on time */
		setDmaThreshold(pBuffer->dmaIndex, TC_DMA_THRESHOLD_LOW);
              }
            }

	    /*
	     * we have no buffer linkage 
	     */
	    vSessions[vLoopSessions]->vSlots[vLoopSlots]->pBuffer = NULL;

	    error = cHALNoError;
          }
          else
          {
            printk("%s(): no buffer attached\n", __func__);
          }
	  break;
	}
      }
    }
  }

  dprintk ( "%s (%d) <\n", __func__, error );

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return error;
}

int pti_hal_slot_free ( int session_handle, int slot_handle )
{
  int error = cHALNotInitialized;
  int vLoopSessions, vLoopSlots;
  int numSlotsInUse = 0;
  
  dprintk ( "%s >\n", __func__ );

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle ==
	       slot_handle )
	     && vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 1 )
	{
	  /*
	   * clear the pid in hw 
	   */
	  pti_slot_clear_pid ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->
			       TCIndex,
			       pti_hal_get_tc_dma_number ( session_handle,
							   slot_handle ), 1 );

	  /*
	   * unlink from buffer if necessary 
	   */
	  if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->pBuffer !=
	       NULL )
	    pti_hal_slot_unlink_buffer ( session_handle, slot_handle);

	  /*
	   * free the slot in hw 
	   */
	  pti_slot_free ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->
			  TCIndex );

	  /*
	   * init our handling 
	   */
	  vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse = 0;
	  vSessions[vLoopSessions]->vSlots[vLoopSlots]->demux = NULL;
      // DB: commented out to allow resetting the DMA threshold
	  //vSessions[vLoopSessions]->vSlots[vLoopSlots]->dvb_type = 0;
	  //vSessions[vLoopSessions]->vSlots[vLoopSlots]->dvb_pes_type = 0;
	  vSessions[vLoopSessions]->vSlots[vLoopSlots]->pid = 0;
	  vSessions[vLoopSessions]->vSlots[vLoopSlots]->DescramblerHandle = 0;

	  error = cHALNoError;

	  break;
	}
        
      } /* for slots */
      
      /* if no more slots are in use let us set the source for this 
       * session to invalid! there seems to be no legal call from
       * dvb-api and therefor e2 point of view. in some cases, when
       * recording and watching with different tuners this kills 
       * one session completly (ERROR STREAM ALREADY IN USE).
       * this is indeed a fact why we can only collect a stream on one
       * session.
       */
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
        if (vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse)
            numSlotsInUse++;
      }

      if (numSlotsInUse == 0)
      {
         vSessions[vLoopSessions]->OldTSMergerTag = vSessions[vLoopSessions]->TSMergerTag;
         vSessions[vLoopSessions]->TSMergerTag = STPTI_STREAM_ID_NONE;
         pti_session_set_source(vSessions[vLoopSessions]->TCIndex, STPTI_STREAM_ID_NONE);
         printk("*********** STPTI_STREAM_ID_NONE for Session %d-%d\n", vLoopSessions, vSessions[vLoopSessions]->TCIndex);
      }
    }
  }

  dprintk ( "%s (%d) <\n", __func__, slot_handle );

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return slot_handle;
}

/* ************************************************************* 
 * Helper Functions for DMAs
 */

int pti_hal_get_tc_dma_number ( int session_handle, int slot_handle )
{
  int tc_dma_number = cHALNotInitialized;
  int vLoopSessions, vLoopSlots;

//      dprintk("%s >\n", __func__);

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle ==
	       slot_handle )
	     && vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse == 1 )
	{
	  if( vSessions[vLoopSessions]->vSlots[vLoopSlots]->pBuffer  != NULL)
          {
	    tc_dma_number = vSessions[vLoopSessions]->vSlots[vLoopSlots]->pBuffer->dmaIndex;
	    break;
          }
	}
      }

    }				/* if */
  }				/* for sessions */

//      dprintk("%s <\n", __func__);

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return tc_dma_number;
}

#if 0

int pti_hal_buffer_enable ( int session_handle, struct tBuffer *pBuffer )
{
  int error = cHALNotInitialized;

  return 0;
  dprintk ( "%s >\n", __func__ );

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopDMAs = 0; vLoopDMAs < numberOfAvailableDMAs; vLoopDMAs++ )
      {
	if ( ( vDMAs[vLoopDMAs]->Handle == buffer_handle ) &&
	     ( vDMAs[vLoopDMAs]->inUse == 1 ) &&
	     ( vDMAs[vLoopDMAs]->SlotUsage != 0 ) )
	{
	  vDMAs[vLoopDMAs]->EnableCount++;
	  if ( vDMAs[vLoopDMAs]->EnableCount == vDMAs[vLoopDMAs]->SlotUsage )
	  {
	    /*
	     * enable the signalling 
	     */
	    pti_buffer_enable_signalling ( vDMAs[vLoopDMAs]->TCIndex );

	    dprintk ( "enable dma signalling now\n" );
	  }
	  else
	  {
	    dprintk ( "EnableCount = %d, SlotUsage = %d\n",
		      vDMAs[vLoopDMAs]->EnableCount,
		      vDMAs[vLoopDMAs]->SlotUsage );
	  }

	  error = cHALNoError;
	  break;
	}
      }
    }
  }

  dprintk ( "%s (%d) <\n", __func__, error );

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return error;
}

int pti_hal_buffer_disable ( int session_handle, int buffer_handle )
{
  int error = cHALNotInitialized;
  int vLoopSessions, vLoopDMAs;

  dprintk ( "%s >\n", __func__ );

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif
  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopDMAs = 0; vLoopDMAs < numberOfAvailableDMAs; vLoopDMAs++ )
      {
	if ( ( vDMAs[vLoopDMAs]->Handle == buffer_handle ) &&
	     ( vDMAs[vLoopDMAs]->inUse == 1 ) &&
	     ( vDMAs[vLoopDMAs]->SlotUsage != 0 ) )
	{
	  vDMAs[vLoopDMAs]->EnableCount--;

	  if ( vDMAs[vLoopDMAs]->EnableCount != vDMAs[vLoopDMAs]->SlotUsage )
	  {
	    /*
	     * disable the signalling 
	     */
	    pti_buffer_disable_signalling ( vDMAs[vLoopDMAs]->TCIndex );
	  }

	  error = cHALNoError;
	  break;
	}
      }
    }
  }

  dprintk ( "%s <\n", __func__ );

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return error;
}
#endif

/* ***************************************************************
 * Descrambler
 */

int pti_hal_get_new_descrambler ( int session_handle )
{
  int descrambler_handle = cHALNotInitialized;
  int noFree = 0;
  int vLoopSessions, vLoopDescrambler;

  dprintk ( "%s >\n", __func__ );

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopDescrambler = 0;
	    vLoopDescrambler < numberOfAvailableDescramblerKeys;
	    vLoopDescrambler++ )
      {
	if ( vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->
	     inUse == 0 )
	{
	  descrambler_handle =
	    vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->Handle;
	  pti_descrambler_allocate ( vSessions[vLoopSessions]->
				     vDescrambler[vLoopDescrambler]->
				     TCIndex );

	  /*
	   * this should never happen 
	   */
	  if ( vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->
	       TCIndex == -1 )
	  {
	    descrambler_handle = cHALNoFreeDescrambler;
	  }
	  else
	  {
	    vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->inUse =
	      1;
	    noFree = 1;
	    break;
	  }			/* else */
	}			/* if */
      }				/* for descrambler */

      if ( noFree == 0 )
	descrambler_handle = cHALNoFreeDescrambler;
    }				/* if */
  }				/* for sessions */

  dprintk ( "%s (%d) >\n", __func__, descrambler_handle );

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return descrambler_handle;

}

int pti_hal_descrambler_link ( int session_handle, int descrambler_handle,
			       int slot_handle )
{
  int error = cHALNotInitialized;
  int vLoopSessions, vLoopSlots, vLoopDescrambler;

  dprintk ( "%s >\n", __func__ );

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif
  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
	if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle ==
	     slot_handle )
	{
	  if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->
	       DescramblerHandle != 0 )
	  {
	    /*quack: do not retun an error instead unlink the
		  current descrambler and link the new one*/
		/*error = cHALSlotAlreadyInUse;*/
	    for ( vLoopDescrambler = 0;
		  vLoopDescrambler < numberOfAvailableDescramblerKeys;
		  vLoopDescrambler++ )
	    {
	      if ( ( vSessions[vLoopSessions]->
		     vDescrambler[vLoopDescrambler]->Handle ==
		     vSessions[vLoopSessions]->vSlots[vLoopSlots]->DescramblerHandle )
		   && ( vSessions[vLoopSessions]->
			vDescrambler[vLoopDescrambler]->inUse == 1 ))
	    			pti_descrambler_disassociate_from_slot ( vSessions[vLoopSessions]->
						   vDescrambler
						   [vLoopDescrambler]->
						   TCIndex,
						   vSessions[vLoopSessions]->
						   vSlots[vLoopSlots]->
						   TCIndex );
	    }
          }
	  /*else*/
	  {
	    /*
	     * descrambler suchen 
	     */
	    for ( vLoopDescrambler = 0;
		  vLoopDescrambler < numberOfAvailableDescramblerKeys;
		  vLoopDescrambler++ )
	    {
	      if ( ( vSessions[vLoopSessions]->
		     vDescrambler[vLoopDescrambler]->Handle ==
		     descrambler_handle )
		   && ( vSessions[vLoopSessions]->
			vDescrambler[vLoopDescrambler]->inUse == 1 )
		   /*
		    * &&
		    * descrambler können mit mehreren slots verlinkt werden,
		    * * muss ich also irgendwie anders mache
		    * (vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->SlotHandle == 0)  dont link twice 
		    */  )

	      {
		pti_descrambler_associate_with_slot ( vSessions
						      [vLoopSessions]->
						      vDescrambler
						      [vLoopDescrambler]->
						      TCIndex,
						      vSessions
						      [vLoopSessions]->
						      vSlots[vLoopSlots]->
						      TCIndex );

		/*
		 * internal link 
		 */
		vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->
		  SlotHandle = slot_handle;
		vSessions[vLoopSessions]->vSlots[vLoopSlots]->
		  DescramblerHandle = descrambler_handle;

		error = cHALNoError;
		break;
	      }			/* if */
	    }			/* for descrambler */
	  }			/* else */
	}			/* if */
      }				/* for slots */
    }				/* if */
  }				/* for sessions */

  dprintk ( "%s (%d)<\n", __func__, error );

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return error;
}

int pti_hal_descrambler_unlink ( int session_handle, int descrambler_handle )
{
  int error = cHALNotInitialized;
  int vLoopSessions, vLoopSlots, vLoopDescrambler;

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      /*
       * descrambler suchen 
       */
      for ( vLoopDescrambler = 0;
	    vLoopDescrambler < numberOfAvailableDescramblerKeys;
	    vLoopDescrambler++ )
      {
	if ( ( vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->
	       Handle == descrambler_handle )
	     && ( vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->
		  SlotHandle != 0 )
	     && ( vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->
		  inUse == 1 ) )
	{
	  /*
	   * get slot index 
	   */
	  for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
		vLoopSlots++ )
	  {
	    if ( vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle ==
		 vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->
		 SlotHandle )
	      break;
	  }

	  /*
	   * hw unlink 
	   */
	  pti_descrambler_disassociate_from_slot ( vSessions[vLoopSessions]->
						   vDescrambler
						   [vLoopDescrambler]->
						   TCIndex,
						   vSessions[vLoopSessions]->
						   vSlots[vLoopSlots]->
						   TCIndex );

	  /*
	   * internal unlink 
	   */
	  vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->
	    SlotHandle = 0;
	  vSessions[vLoopSessions]->vSlots[vLoopSlots]->DescramblerHandle = 0;

	  error = cHALNoError;
	  break;

	}			/* if */
      }				/* for descrambler */
    }				/* if */
  }				/* for sessions */

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return error;
}

int pti_hal_descrambler_set ( int session_handle, int descrambler_handle,
			      u8 * Data, int parity )
{
  int error = cHALNotInitialized;
  int vLoopSessions, vLoopDescrambler;

  if ( vSessions == NULL )
    return cHALNotInitialized;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif
  dprintk ( "%s session=%d  descrambler=%d\n", __func__, session_handle,descrambler_handle);

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
    if ( vSessions[vLoopSessions]->Handle == session_handle )
    {
      /*
       * descrambler suchen 
       */
      for ( vLoopDescrambler = 0;
	    vLoopDescrambler < numberOfAvailableDescramblerKeys;
	    vLoopDescrambler++ )
      {
	if ( ( vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->
	       Handle == descrambler_handle )
	     && ( vSessions[vLoopSessions]->vDescrambler[vLoopDescrambler]->
		  inUse == 1 ) )
	{
	  pti_descrambler_set ( vSessions[vLoopSessions]->
				vDescrambler[vLoopDescrambler]->TCIndex,
				parity, Data );

	  error = cHALNoError;
	  break;

	}			/* if */
      }				/* for descrambler */
    }				/* if */
  }				/* for sessions */

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return error;
}

int pti_hal_descrambler_clear ( void )
{
  return -1;
}

int pti_hal_descrambler_free ( void )
{
  return -1;
}

/* return:
 * check if a audio / video pid has changed and
 * if the status of the slot is scrambled or not.
 * needed for ufs910 to modify the stream routing ad hoc. 
 *
 * -1 if nothing happens
 * 0  not scrampbled but status changed
 * 1  scrambled and status chaned
 *
 */
int pti_hal_get_scrambled(void)
{
  int vLoopSessions, vLoopSlots;
  int state;
  static u16 lastAudioPid = -1;
  static u16 lastVideoPid = -1;
  static int lastAudioState = -1;
  static int lastVideoState = -1;
  int result = -1;
  
  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
         if (!vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse)
	    continue;
         
	 if (vSessions[vLoopSessions]->vSlots[vLoopSlots]->dvb_type != DMX_TYPE_TS)
	    continue;
	    
	 if (vSessions[vLoopSessions]->vSlots[vLoopSlots]->dvb_pes_type == DMX_TS_PES_VIDEO)
	 {
	     if (vSessions[vLoopSessions]->vSlots[vLoopSlots]->pid == lastVideoPid)
	        continue;
		
             lastVideoPid = vSessions[vLoopSessions]->vSlots[vLoopSlots]->pid;
	     
	     state = pti_slot_get_state(vSessions[vLoopSessions]->vSlots[vLoopSlots]->TCIndex);
	     
	     if (state & (TC_MAIN_INFO_SLOT_STATE_SCRAMBLED | 
	                  TC_MAIN_INFO_SLOT_STATE_TRANSPORT_SCRAMBLED))
             {

                 if (lastVideoState == 1)
		    continue;
		 
		 lastVideoState = 1;
		 return 1;	     
	     } else
             {
                 if (lastVideoState == 0)
		    continue;
		 
		 lastVideoState = 0;
		 return 0;	     
	     }
	 } else
	 if (vSessions[vLoopSessions]->vSlots[vLoopSlots]->dvb_pes_type == DMX_TS_PES_AUDIO)
	 {
	     if (vSessions[vLoopSessions]->vSlots[vLoopSlots]->pid == lastAudioPid)
	        continue;
		
             lastAudioPid = vSessions[vLoopSessions]->vSlots[vLoopSlots]->pid;
	     
	     state = pti_slot_get_state(vSessions[vLoopSessions]->vSlots[vLoopSlots]->TCIndex);
	     
	     if (state & (TC_MAIN_INFO_SLOT_STATE_SCRAMBLED | 
	                  TC_MAIN_INFO_SLOT_STATE_TRANSPORT_SCRAMBLED))
             {
                 if (lastAudioState == 1)
		    continue;
		 
		 lastAudioState = 1;
		 return 1;	     
	     } else
             {
                 if (lastAudioState == 0)
		    continue;
		 
		 lastAudioState = 0;
		 return 0;	     
	     }
	 }
      }				/* for slots */
   }				/* for sessions */
   return result;
}

void pti_hal_output_slot_state(void)
{
  int vLoopSessions, vLoopSlots;

  dprintk ( "%s >\n", __func__ );

  if ( vSessions == NULL )
    return;

#ifdef use_hal_mutex
  mutex_lock ( &( HALLock ) );
#endif

  for ( vLoopSessions = 0; vLoopSessions < numberOfAvailableSessions;
	vLoopSessions++ )
  {
      for ( vLoopSlots = 0; vLoopSlots < numberOfAvailableSlots;
	    vLoopSlots++ )
      {
         if (!vSessions[vLoopSessions]->vSlots[vLoopSlots]->inUse)
	    continue;
          printk("session %d, slot %d, pid %d, state = %d\n", 
	   vSessions[vLoopSessions]->Handle,
	   vSessions[vLoopSessions]->vSlots[vLoopSlots]->Handle,
	   vSessions[vLoopSessions]->vSlots[vLoopSlots]->pid,
	   pti_slot_get_state(vSessions[vLoopSessions]->vSlots[vLoopSlots]->TCIndex)
	   );
      }				/* for slots */
  }				/* for sessions */

  dprintk ( "%s <\n", __func__ );

#ifdef use_hal_mutex
  mutex_unlock ( &( HALLock ) );
#endif
  return;
}

EXPORT_SYMBOL(pti_hal_descrambler_set);
EXPORT_SYMBOL(pti_hal_descrambler_unlink);
EXPORT_SYMBOL(pti_hal_descrambler_link);
EXPORT_SYMBOL(pti_hal_get_new_descrambler);
EXPORT_SYMBOL(pti_hal_slot_free);
EXPORT_SYMBOL(pti_hal_slot_unlink_buffer);
EXPORT_SYMBOL(pti_hal_slot_link_buffer);
EXPORT_SYMBOL(pti_hal_slot_clear_pid);
EXPORT_SYMBOL(pti_hal_slot_set_pid);
EXPORT_SYMBOL(pti_hal_get_new_slot_handle);
EXPORT_SYMBOL(pti_hal_set_source);
EXPORT_SYMBOL(pti_hal_get_session_handle);
EXPORT_SYMBOL(pti_hal_get_new_session_handle);
EXPORT_SYMBOL(pti_hal_init);
