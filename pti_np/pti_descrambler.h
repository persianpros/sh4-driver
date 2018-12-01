#ifndef pti_descrambler_123
#define pti_descrambler_123

void pti_descrambler_allocate(u32 tc_descrambler_index);
void pti_descrambler_associate_with_slot(u32 tc_descrambler_index, u32 tc_slot_index);
void pti_descrambler_disassociate_from_slot(u32 tc_descrambler_index, u32 tc_slot_index);
void pti_descrambler_set(u32 tc_descrambler_index, int Parity, u8 *Data);

#endif
