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
    STPTI_TCParameters_t* TC_Params_p = &tc_params;
    TCKey_t              *Key_p;

    if (tc_descrambler_index < 0)
    {
        printk("%s: Invalid descrambler index passed %d\n", __func__, tc_descrambler_index);
        return;
    }

    Key_p         =
        (TCKey_t *)((u32)(&(TC_Params_p->TC_DescramblerKeysStart)[0])
                    + ((u16) tc_descrambler_index * TC_Params_p->TC_SizeOfDescramblerKeys));

    /* mark as not valid */
    STSYS_SetTCMask16LE((void*)&Key_p->KeyValidity, 0);

}

/* fixme: logisch gesehen auch eher link slot with descrambler ;) */
void pti_descrambler_associate_with_slot(u32 tc_descrambler_index, u32 tc_slot_index)
{
    STPTI_TCParameters_t* TC_Params_p = &tc_params;
    TCKey_t              *Key_p;
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

    Key_p         =
        (TCKey_t *)((u32)(&(TC_Params_p->TC_DescramblerKeysStart)[0])
                    + ((u16) tc_descrambler_index * TC_Params_p->TC_SizeOfDescramblerKeys));

    printk("(da %d, %d)", tc_descrambler_index, tc_slot_index);

    MainInfo_p = &((TCMainInfo_t *)TC_Params_p->TC_MainInfoStart)[tc_slot_index];

    /* convert ST20.40 address in TCKey_p to one that the TC understands */
    STSYS_WriteTCReg16LE((void*)&MainInfo_p->DescramblerKeys_p,(u32) ( (u8 *)Key_p - (u8 *)TC_Params_p->TC_DataStart + (u8 *)TC_DSRAM_BASE ));

    /* slot now does not ignore the descrambler */
    STSYS_ClearTCMask16LE((void*)&MainInfo_p->SlotMode, (TC_MAIN_INFO_SLOT_MODE_IGNORE_SCRAMBLING) );
}

/* auch eher eine reine slot aktion */
void pti_descrambler_disassociate_from_slot(u32 tc_descrambler_index, u32 tc_slot_index)
{
    STPTI_TCParameters_t* TC_Params_p = &tc_params;
    TCMainInfo_t *MainInfo_p;

    if (tc_slot_index < 0)
    {
        printk("%s: Invalid slot index passed %d\n", __func__, tc_slot_index);
        return;
    }

    MainInfo_p = &((TCMainInfo_t *)TC_Params_p->TC_MainInfoStart)[tc_slot_index];

    STSYS_WriteTCReg16LE((u32)&MainInfo_p->DescramblerKeys_p,TC_INVALID_LINK);
}

/* wenn werte richtig dann, assignment pid - slot in beiden versionen dumpen */
void dumpDescrambler(TCKey_t* Key_p)
{
    int vLoop = 1;
    dprintk("%d. Validity = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->KeyValidity));
    dprintk("%d. Mode     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->KeyMode));
    dprintk("%d. Even0    = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenKey0));
    dprintk("%d. Even1    = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenKey1));
    dprintk("%d. Even2    = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenKey2));
    dprintk("%d. Even3    = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->EvenKey3));
    dprintk("%d. Odd0     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddKey0));
    dprintk("%d. Odd1     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddKey1));
    dprintk("%d. Odd2     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddKey2));
    dprintk("%d. Odd3     = 0x%.4x\n", vLoop, (unsigned int) readw(&Key_p ->OddKey3));
}

void pti_descrambler_set(u32 tc_descrambler_index, int Parity, u8 *Data)
{
    STPTI_TCParameters_t* TC_Params_p = &tc_params;
    TCKey_t              *Key_p;

    u16 KeyCheck, KeyValidity = 0, KeyMode = 0;

    if (tc_descrambler_index < 0)
    {
        printk("%s: Invalid descrambler index passed %d\n", __func__, tc_descrambler_index);
        return;
    }

    Key_p         =
        (TCKey_t *)((u32)(&(TC_Params_p->TC_DescramblerKeysStart)[0])
                    + ((u16) tc_descrambler_index * TC_Params_p->TC_SizeOfDescramblerKeys));

    dprintk("%s > (%d)\n", __func__, tc_descrambler_index);

    /* As the key could be in use we update KeyValidity in a single write */
    KeyValidity = readw((void*)&Key_p->KeyValidity);

    /* Mask off the Algorithm, Chaining Mode, and Residue Mode (LeftResidue or RightResidue) */
    KeyValidity &= ~(TCKEY_ALGORITHM_MASK | TCKEY_CHAIN_ALG_MASK | TCKEY_CHAIN_MODE_LR);
    KeyValidity |= TCKEY_ALGORITHM_DVB;

#if defined(SECURE_LITE2)
    KeyMode = readw((void*)&Key_p->KeyMode);
    KeyMode &= ~(TCKEY_MODE_EAVS|TCKEY_MODE_PERM0|TCKEY_MODE_PERM1);

    STSYS_WriteTCReg16LE((void*)&Key_p->KeyMode, KeyMode);
#endif

    /* fixme: das hab ich geaendert im gegensatz zum orig */
    if ((Data[0] == 0) && (Data[1] == 0) && (Data[2] == 0) && (Data[3] == 0) &&
            (Data[4] == 0) && (Data[5] == 0) && (Data[6] == 0) && (Data[7] == 0))
    {
        if (Parity == 0 /* even */)
        {
            STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey0, 0);
            STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey1, 0);
            STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey2, 0);
            STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey3, 0);

        }
        else       /* STPTI_KEY_PARITY_ODD_PARITY */
        {
            STSYS_WriteTCReg16LE((void*)&Key_p->OddKey0, 0);
            STSYS_WriteTCReg16LE((void*)&Key_p->OddKey1, 0);
            STSYS_WriteTCReg16LE((void*)&Key_p->OddKey2, 0);
            STSYS_WriteTCReg16LE((void*)&Key_p->OddKey3, 0);
        }

        KeyCheck  = readw((void*)&Key_p->EvenKey0) |
                    readw((void*)&Key_p->EvenKey1) |
                    readw((void*)&Key_p->EvenKey2) |
                    readw((void*)&Key_p->EvenKey3);
        KeyCheck |= readw((void*)&Key_p->OddKey0)  |
                    readw((void*)&Key_p->OddKey1)  |
                    readw((void*)&Key_p->OddKey2)  |
                    readw((void*)&Key_p->OddKey3);

        if (0 == KeyCheck)
        {
            /* if all keys zero then mark as not valid */
            /* this makes no sense because we leave in next statement
             * but that's what stapi is doing here ;)
             */
            KeyValidity = 0;
        }

        return;
    }

    /* --- valid key of some type to process --- */

    if (Parity == 0 /* even */)
    {
        //dprintk("%s seeting even key\n", __func__);

        STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey0, (Data[0] << 8) | Data[1]);
        STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey1, (Data[2] << 8) | Data[3]);
        STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey2, (Data[4] << 8) | Data[5]);
        STSYS_WriteTCReg16LE((void*)&Key_p->EvenKey3, (Data[6] << 8) | Data[7]);

        KeyValidity |= TCKEY_VALIDITY_TS_EVEN;
        /* KeyValidity |= TCKEY_VALIDITY_PES_EVEN; */
    }
    else      /* STPTI_KEY_PARITY_ODD_PARITY */
    {
        //dprintk("%s seeting odd key\n", __func__);

        STSYS_WriteTCReg16LE((void*)&Key_p->OddKey0, (Data[0] << 8) | Data[1]);
        STSYS_WriteTCReg16LE((void*)&Key_p->OddKey1, (Data[2] << 8) | Data[3]);
        STSYS_WriteTCReg16LE((void*)&Key_p->OddKey2, (Data[4] << 8) | Data[5]);
        STSYS_WriteTCReg16LE((void*)&Key_p->OddKey3, (Data[6] << 8) | Data[7]);

        KeyValidity |= TCKEY_VALIDITY_TS_ODD;
        /* KeyValidity |= TCKEY_VALIDITY_PES_ODD; */
    }

    STSYS_WriteTCReg16LE((void*)&Key_p->KeyValidity, KeyValidity);

    //dumpDescrambler(Key_p);

    dprintk("%s <\n", __func__);
    return;
}

