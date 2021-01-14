/****************************************************************************
 *
 * opt9600_fp_asc.c
 *
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add irq handling
 * (c) 2020 Audioniek: ported to Opticum HD 9600 (TS)
 *
 * UART setup for communication with the front processor.
 *
 * Largely based on cn_micom, enhanced and ported to Opticum HD 9600 (TS)
 * by Audioniek.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 ****************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------
 * 20201222 Audioniek       Initial version, based on cn_micom.
 * 20201226 Audioniek       Change UART to ASC3.
 *
 ****************************************************************************/

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/termbits.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
#include <linux/stm/pio.h>
#else
#include <linux/stpio.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/poll.h>

#include "opt9600_fp.h"
#include "opt9600_fp_asc.h"

//-------------------------------------

unsigned int InterruptLine   = 120;
unsigned int ASCXBaseAddress = ASC3BaseAddress;

//-------------------------------------

void serial_init(void)
{
	/* Configure the PIO pins */
	dprintk(150, "%s >\n", __func__);
	stpio_request_pin(5, 0,  "ASC_TX", STPIO_ALT_OUT); /* Tx */
	stpio_request_pin(5, 1,  "ASC_RX", STPIO_IN);      /* Rx */

	// Configure the ASC input/output settings
	*(unsigned int *)(ASCXBaseAddress + ASC_INT_EN)   = 0x00000000;  // TODO: Why do we set here the INT_EN again ???
	*(unsigned int *)(ASCXBaseAddress + ASC_CTRL)     = 0x00000589;  // mode 0
	*(unsigned int *)(ASCXBaseAddress + ASC_TIMEOUT)  = 0x00000010;
	*(unsigned int *)(ASCXBaseAddress + ASC_BAUDRATE) = 0x0000028b;  // 9600 baud
	*(unsigned int *)(ASCXBaseAddress + ASC_TX_RST)   = 0;
	*(unsigned int *)(ASCXBaseAddress + ASC_RX_RST)   = 0;
	dprintk(150, "%s <\n", __func__);
}

int serial_putc(char Data)
{
	char          *ASCn_TX_BUFF = (char *)(ASCXBaseAddress + ASC_TX_BUFF);
	unsigned int  *ASCn_INT_STA = (unsigned int *)(ASCXBaseAddress + ASC_INT_STA);
	unsigned long Counter = 200000;

	while (((*ASCn_INT_STA & ASC_INT_STA_THE) == 0) && --Counter)  // baseaddress + 0x14, wait for bit 1 (0x02) set
	{
		mdelay(1);
	}
	if (Counter == 0)
	{
		dprintk(1, "%s Error writing char\n", __func__);
	}
	*ASCn_TX_BUFF = Data;
	return 1;
}
// vim:ts=4
