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

#include "pti.h"
#include "pti_main.h"

void pti_descrambler_allocate(u32 tc_descrambler_index)
{
	STPTI_TCParameters_t *TC_Params_p = &tc_params;
	TCKey_t *Key_p;
	if (tc_descrambler_index < 0)
	{
		printk("%s: Invalid descrambler index passed %d\n", __func__, tc_descrambler_index);
		return;
	}
	Key_p =
		(TCKey_t *)((u32)(&(TC_Params_p->TC_DescramblerKeysStart)[0])
			    + ((u16) tc_descrambler_index * TC_Params_p->TC_SizeOfDescramblerKeys));
	/* mark as not valid */
	STSYS_SetTCMask16LE((void *)&Key_p->KeyValidity, 0);
}

/* fixme: logisch gesehen auch eher link slot with descrambler ;) */
void pti_descrambler_associate_with_slot(u32 tc_descrambler_index, u32 tc_slot_index)
{
	STPTI_TCParameters_t *TC_Params_p = &tc_params;
	TCKey_t *Key_p;
	TCMainInfo_t *MainInfo_p;
	if (tc_descrambler_index < 0)
	{
		printk("%s: Invalid descrambler index passed %d\n", __func__, tc_descrambler_index);
		return;
	}
	if (tc_slot_index < 0)
	{
		printk("%s: Invalid slot index passed %d\n", __func__, tc_slot_index);
		return;
	}
	Key_p =
		(TCKey_t *)((u32)(&(TC_Params_p->TC_DescramblerKeysStart)[0])
			    + ((u16) tc_descrambler_index * TC_Params_p->TC_SizeOfDescramblerKeys));
	printk("(da %d, %d)", tc_descrambler_index, tc_slot_index);
	MainInfo_p = &((TCMainInfo_t *)TC_Params_p->TC_MainInfoStart)[tc_slot_index];
	/* convert ST20.40 address in TCKey_p to one that the TC understands */
	STSYS_WriteTCReg16LE((void *)&MainInfo_p->DescramblerKeys_p, (u32)((u8 *)Key_p - (u8 *)TC_Params_p->TC_DataStart + (u8 *)TC_DSRAM_BASE));
	/* slot now does not ignore the descrambler */
	STSYS_ClearTCMask16LE((void *)&MainInfo_p->SlotMode, (TC_MAIN_INFO_SLOT_MODE_IGNORE_SCRAMBLING));
}

/* auch eher eine reine slot aktion */
void pti_descrambler_disassociate_from_slot(u32 tc_descrambler_index, u32 tc_slot_index)
{
	STPTI_TCParameters_t *TC_Params_p = &tc_params;
	TCMainInfo_t *MainInfo_p;
	if (tc_slot_index < 0)
	{
		printk("%s: Invalid slot index passed %d\n", __func__, tc_slot_index);
		return;
	}
	MainInfo_p = &((TCMainInfo_t *)TC_Params_p->TC_MainInfoStart)[tc_slot_index];
	STSYS_WriteTCReg16LE((u32)&MainInfo_p->DescramblerKeys_p, TC_INVALID_LINK);
}

/* wenn werte richtig dann, assignment pid - slot in beiden versionen dumpen */
void dumpDescrambler(TCKey_t *Key_p)
{
	int vLoop = 1;
	printk("%d. Validity = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->KeyValidity));
	printk("%d. Mode     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->KeyMode));
	printk("%d. Even0    = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenKey0));
	printk("%d. Even1    = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenKey1));
	printk("%d. Even2    = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenKey2));
	printk("%d. Even3    = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenKey3));
	printk("%d. Even4    = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenKey4));
	printk("%d. Even5    = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenKey5));
	printk("%d. Even6    = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenKey6));
	printk("%d. Even7    = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenKey7));
	printk("%d. Odd0     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddKey0));
	printk("%d. Odd1     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddKey1));
	printk("%d. Odd2     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddKey2));
	printk("%d. Odd3     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddKey3));
	printk("%d. Odd4     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddKey4));
	printk("%d. Odd5     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddKey5));
	printk("%d. Odd6     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddKey6));
	printk("%d. Odd7     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddKey7));
	printk("%d. EvenIV0  = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenIV0));
	printk("%d. EvenIV1  = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenIV1));
	printk("%d. EvenIV2  = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenIV2));
	printk("%d. EvenIV3  = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenIV3));
	printk("%d. EvenIV4  = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenIV4));
	printk("%d. EvenIV5  = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenIV5));
	printk("%d. EvenIV6  = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenIV6));
	printk("%d. EvenIV7  = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenIV7));
	printk("%d. OddIV0   = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddIV0));
	printk("%d. OddIV1   = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddIV1));
	printk("%d. OddIV2   = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddIV2));
	printk("%d. OddIV3   = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddIV3));
	printk("%d. OddIV4   = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddIV4));
	printk("%d. OddIV5   = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddIV5));
	printk("%d. OddIV6   = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddIV6));
	printk("%d. OddIV7   = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddIV7));
}

void pti_descrambler_set(u32 tc_descrambler_index, int Parity, u8 *Data)
{
	STPTI_TCParameters_t *TC_Params_p = &tc_params;
#ifdef xdebug
	int vLoop;
#endif
	TCKey_t *Key_p;
	u16 /*KeyCheck, */KeyValidity = 0;
	if (tc_descrambler_index < 0)
	{
		printk("%s: Invalid descrambler index passed %d\n", __func__, tc_descrambler_index);
		return;
	}
	Key_p =
		(TCKey_t *)((u32)(&(TC_Params_p->TC_DescramblerKeysStart)[0])
			    + ((u16) tc_descrambler_index * TC_Params_p->TC_SizeOfDescramblerKeys));
	dprintk("%s > (%d)\n", __func__, tc_descrambler_index);
#ifdef xdebug
	for (vLoop = 0; vLoop < 8; vLoop++)
		dprintk("cw[%d] = %0.2x\n", vLoop, Data[vLoop]);
#endif
	/* As the key could be in use we update KeyValidity in a single write */
	KeyValidity = readw((void *)&Key_p->KeyValidity);
	/* Mask off the Algorithm, Chaining Mode, and Residue Mode (LeftResidue or RightResidue) */
	KeyValidity &= ~TCKEY_ALGORITHM_MASK;
	KeyValidity &= ~(TCKEY_CHAIN_ALG_MASK | TCKEY_CHAIN_MODE_LR | TCKEY_ALGORITHM_MASK);
	KeyValidity |= TCKEY_ALGORITHM_DVB;
	/* fixme: das hab ich geaendert im gegensatz zum orig */

	/* das habe ich auskommentiert warum diese umständliche Prüfung ?
	wenn das cw alles 0 hat soll es doch in die Register schreiben
	KeyValidity wird doch sowieso überschrieben */
#if 0
	if ((Data[0] == 0) && (Data[1] == 0) && (Data[2] == 0) && (Data[3] == 0) &&
			(Data[4] == 0) && (Data[5] == 0) && (Data[6] == 0) && (Data[7] == 0))
	{
		if (Parity == 0 /* even */)
		{
			STSYS_WriteTCReg16LE((void *)&Key_p->EvenKey0, 0);
			STSYS_WriteTCReg16LE((void *)&Key_p->EvenKey1, 0);
			STSYS_WriteTCReg16LE((void *)&Key_p->EvenKey2, 0);
			STSYS_WriteTCReg16LE((void *)&Key_p->EvenKey3, 0);
		}
		else       /* STPTI_KEY_PARITY_ODD_PARITY */
		{
			STSYS_WriteTCReg16LE((void *)&Key_p->OddKey0, 0);
			STSYS_WriteTCReg16LE((void *)&Key_p->OddKey1, 0);
			STSYS_WriteTCReg16LE((void *)&Key_p->OddKey2, 0);
			STSYS_WriteTCReg16LE((void *)&Key_p->OddKey3, 0);
		}
		KeyCheck  = readw((void *)&Key_p->EvenKey0) |
			    readw((void *)&Key_p->EvenKey1) |
			    readw((void *)&Key_p->EvenKey2) |
			    readw((void *)&Key_p->EvenKey3);
		KeyCheck |= readw((void *)&Key_p->OddKey0)  |
			    readw((void *)&Key_p->OddKey1)  |
			    readw((void *)&Key_p->OddKey2)  |
			    readw((void *)&Key_p->OddKey3);
		if (0 == KeyCheck)
		{
			/* if all keys zero then mark as not valid */
			STSYS_SetTCMask16LE((void *)&Key_p->KeyValidity, 0);
		}
		return;
	}
#endif
	/* --- valid key of some type to process --- */
	if (Parity == 0 /* even */)
	{
		//dprintk("%s seeting even key\n", __func__);
		STSYS_WriteTCReg16LE((void *)&Key_p->EvenKey0, (Data[0] << 8) | Data[1]);
		STSYS_WriteTCReg16LE((void *)&Key_p->EvenKey1, (Data[2] << 8) | Data[3]);
		STSYS_WriteTCReg16LE((void *)&Key_p->EvenKey2, (Data[4] << 8) | Data[5]);
		STSYS_WriteTCReg16LE((void *)&Key_p->EvenKey3, (Data[6] << 8) | Data[7]);
		KeyValidity |= TCKEY_VALIDITY_TS_EVEN;
		//KeyValidity |= TCKEY_VALIDITY_PES_EVEN;
		/* STSYS_SetTCMask16LE((void*)&Key_p->KeyValidity, TCKEY_VALIDITY_TS_EVEN);*/
	}
	else      /* STPTI_KEY_PARITY_ODD_PARITY */
	{
		//dprintk("%s seeting odd key\n", __func__);
		STSYS_WriteTCReg16LE((void *)&Key_p->OddKey0, (Data[0] << 8) | Data[1]);
		STSYS_WriteTCReg16LE((void *)&Key_p->OddKey1, (Data[2] << 8) | Data[3]);
		STSYS_WriteTCReg16LE((void *)&Key_p->OddKey2, (Data[4] << 8) | Data[5]);
		STSYS_WriteTCReg16LE((void *)&Key_p->OddKey3, (Data[6] << 8) | Data[7]);
		KeyValidity |= TCKEY_VALIDITY_TS_ODD;
		//KeyValidity |= TCKEY_VALIDITY_PES_ODD;
		/*STSYS_SetTCMask16LE((void*)&Key_p->KeyValidity, TCKEY_VALIDITY_TS_ODD);*/
	}
	STSYS_WriteTCReg16LE((void *)&Key_p->KeyValidity, KeyValidity);
	//STSYS_SetTCMask16LE((void *)&Key_p->KeyValidity, KeyValidity);
#ifdef CONFIG_PRINTK
	printk("Validity = 0x%.4x Mode = 0x%.4x\n", (unsigned int) readw(&Key_p ->KeyValidity), (unsigned int) readw(&Key_p ->KeyMode));
#endif
	//dumpDescrambler(Key_p);
	dprintk("%s <\n", __func__);
	return;
}

void pti_descrambler_set_aes(u32 tc_descrambler_index, int Parity, u8 *Data, int data_type)
{
	STPTI_TCParameters_t *TC_Params_p = &tc_params;
#ifdef xdebug
	int vLoop;
#endif
	TCKey_t              *Key_p;
	u16 /*KeyCheck, */KeyValidity = 0;
	if (tc_descrambler_index < 0)
	{
		printk("%s: Invalid descrambler index passed %d\n", __func__, tc_descrambler_index);
		return;
	}
	Key_p         =
		(TCKey_t *)((u32)(&(TC_Params_p->TC_DescramblerKeysStart)[0])
			    + ((u16) tc_descrambler_index * TC_Params_p->TC_SizeOfDescramblerKeys));
	printk("TC_NumberDescramblerKeys: %d\n" , TC_Params_p->TC_NumberDescramblerKeys);
	printk("TC_SizeOfDescramblerKeys: %d\n" , TC_Params_p->TC_SizeOfDescramblerKeys);
	dprintk("%s > (%d)\n", __func__, tc_descrambler_index);
#ifdef xdebug
	for (vLoop = 0; vLoop < 8; vLoop++)
		dprintk("cw[%d] = %0.2x\n", vLoop, Data[vLoop]);
#endif
	/* As the key could be in use we update KeyValidity in a single write */
	KeyValidity = readw((void *)&Key_p->KeyValidity);
	/* Mask off the Algorithm, Chaining Mode, and Residue Mode (LeftResidue or RightResidue) */
	KeyValidity &= ~TCKEY_ALGORITHM_MASK;
	KeyValidity &= ~(TCKEY_CHAIN_ALG_MASK | TCKEY_CHAIN_MODE_LR | TCKEY_ALGORITHM_MASK);
	/*STPTI_DESCRAMBLER_TYPE_AES_CBC_DESCRAMBLER*/
	KeyValidity |= TCKEY_ALGORITHM_AES | TCKEY_CHAIN_ALG_CBC;

	/* --- valid key of some type to process --- */
	printk("Type: %d Parity: %d\n", data_type, Parity);

	if (Parity == 0 /* STPTI_KEY_PARITY_EVEN_PARITY */)
	{

		STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey4, (Data[0] << 8) | Data[1]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey5, (Data[2] << 8) | Data[3]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey6, (Data[4] << 8) | Data[5]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey7, (Data[6] << 8) | Data[7]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey0, (Data[8] << 8) | Data[9]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey1, (Data[10] << 8) | Data[11]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey2, (Data[12] << 8) | Data[13]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey3, (Data[14] << 8) | Data[15]);

		STSYS_WriteTCReg16LE((void*)&Key_p->EvenIV0, (Data[16] << 8) | Data[17]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenIV1, (Data[18] << 8) | Data[19]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenIV2, (Data[20] << 8) | Data[21]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenIV3, (Data[22] << 8) | Data[23]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenIV4, (Data[24] << 8) | Data[25]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenIV5, (Data[26] << 8) | Data[27]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenIV6, (Data[28] << 8) | Data[29]);
		STSYS_WriteTCReg16LE((void*)&Key_p->EvenIV7, (Data[30] << 8) | Data[31]);

		KeyValidity |= TCKEY_VALIDITY_TS_EVEN;
	}
	else      /* STPTI_KEY_PARITY_ODD_PARITY */
	{

		STSYS_WriteTCReg16LE((void*)&Key_p->OddKey4, (Data[0] << 8) | Data[1]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddKey5, (Data[2] << 8) | Data[3]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddKey6, (Data[4] << 8) | Data[5]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddKey7, (Data[6] << 8) | Data[7]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddKey0, (Data[8] << 8) | Data[9]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddKey1, (Data[10] << 8) | Data[11]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddKey2, (Data[12] << 8) | Data[13]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddKey3, (Data[14] << 8) | Data[15]);

		STSYS_WriteTCReg16LE((void*)&Key_p->OddIV0, (Data[16] << 8) | Data[17]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddIV1, (Data[18] << 8) | Data[19]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddIV2, (Data[20] << 8) | Data[21]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddIV3, (Data[22] << 8) | Data[23]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddIV4, (Data[24] << 8) | Data[25]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddIV5, (Data[26] << 8) | Data[27]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddIV6, (Data[28] << 8) | Data[29]);
		STSYS_WriteTCReg16LE((void*)&Key_p->OddIV7, (Data[30] << 8) | Data[31]);

		KeyValidity |= TCKEY_VALIDITY_TS_ODD;
	}

	STSYS_WriteTCReg16LE((void *)&Key_p->KeyValidity, KeyValidity);
#ifdef CONFIG_PRINTK
	printk("Validity = 0x%.4x Mode = 0x%.4x\n", (unsigned int) readw(&Key_p ->KeyValidity), (unsigned int) readw(&Key_p ->KeyMode));
#endif
	//dumpDescrambler(Key_p);
	dprintk("%s <\n", __func__);
	return;
}
