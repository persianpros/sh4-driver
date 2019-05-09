#ifndef pti_slot_h_123
#define pti_slot_h_123

void pti_slot_allocate(u32 tc_slot_index, int dvb_type, int dvb_pes_type);

u16 pti_slot_get_pid(u32 tc_slot_index, u16 Pid);
void pti_slot_set_pid(u32 tc_slot_index, u16 Pid);
void pti_slot_clear_pid(int tc_slot_index, int tc_dma_index, int rewind);

void pti_slot_link_to_buffer(u32 tc_slot_index, u32 tc_dma_index);
void pti_slot_unlink_buffer(u32 tc_slot_index);

void pti_slot_free(u32 tc_slot_index);
int pti_slot_get_state(u32 tc_slot_index);
#endif
