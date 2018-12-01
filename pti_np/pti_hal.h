#ifndef pti_hal_123
#define pti_hal_123

#include <dvb_demux.h>
#include "pti.h"

/* fixme: this one is still sucking */
#include "../player2/linux/drivers/media/dvb/stm/dvb/dvb_module.h"

#define cHALNoError		0
#define cHALAlreadyInUse	-1
#define cHALNotInitialized	-2
#define cHALNoFreeSession	-3
#define cHALNoFreeSlot		-5
#define cHALSlotAlreadyInUse	-6
#define cHALNoFreeBuffer	-7
#define cHALNoFreeDescrambler	-8
#define cHALNotPossible		-9

int pti_hal_get_new_session_handle(tInputSource source,struct dvb_demux *demux);
int pti_hal_set_source(int session_handle, const tInputSource source);
int pti_hal_free_session(int session_handle);
#ifdef PLAYER211
int pti_hal_get_new_slot_handle(int session_handle, int dvb_type, int dvb_pes_type, 
				struct dvb_demux* demux, struct  DvbStreamContext_s* DemuxStream,
		  		struct DeviceContext_s* DeviceContext);
#else
int pti_hal_get_new_slot_handle(int session_handle, int dvb_type, int dvb_pes_type, 
				struct dvb_demux* demux, struct  StreamContext_s* DemuxStream,
		  		struct DeviceContext_s* DeviceContext);
#endif
int pti_hal_slot_set_pid(int session_handle, int slot_handle, u16 pid);
int pti_hal_slot_clear_pid(int session_handle, int slot_handle);
int pti_hal_slot_link_buffer(int session_handle, int slot_handle, BUFFER_TYPE bufType);
int pti_hal_slot_unlink_buffer(int session_handle, int slot_handle);
int pti_hal_slot_free(int session_handle, int slot_handle);

int pti_hal_get_session_handle(int tc_session_number);
int pti_hal_get_session_number_from_tc_slot_number(int tc_slot_number);
int pti_hal_get_slot_handle(int session_handle, int tc_slot_number);
int pti_hal_get_tc_dma_number(int session_handle, int slot_handle);
struct dvb_demux* pti_hal_get_demux_from_slot(int session_handle, int slot_handle);
struct dvb_demux * pti_hal_get_demux_from_dma_index ( int dmaIndex );

int pti_hal_get_new_descrambler(int session_handle);
int pti_hal_descrambler_link(int session_handle, int descrambler_handle, int slot_handle);
int pti_hal_descrambler_unlink(int session_handle, int descrambler_handle);
int pti_hal_descrambler_set(int session_handle, int descrambler_handle, u8* Data, int parity);

//int pti_hal_buffer_enable(int session_handle, int buffer_handle);
//int pti_hal_buffer_disable(int session_handle, int buffer_handle);

int pti_hal_get_type_from_slot(int session_handle, int slot_handle, int* ts_type, int* pes_type);
#ifdef PLAYER211
struct  DvbStreamContext_s* pti_hal_get_stream_from_slot(int session_handle, int slot_handle);
#else
struct  StreamContext_s* pti_hal_get_stream_from_slot(int session_handle, int slot_handle);
#endif

struct  DeviceContext_s* pti_hal_get_context_from_slot(int session_handle, int slot_handle);
void pti_hal_get_slots_for_pid(int session_handle, u16 pid, int** slots, int* number_slots);
#if defined (CONFIG_KERNELVERSION) /* STLinux 2.3 */
void pti_hal_init ( struct stpti *pti , struct dvb_demux* demux, void (*_demultiplexDvbPackets)(struct dvb_demux* demux, const u8 *buf, int count), int numVideoBuffers);
void paceSwtsByPti(void);
int pti_hal_get_scrambled(void);
#endif
#endif
