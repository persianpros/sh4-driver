/****************************************************************************
 *
 * opt9600_fp_asc.h
 *
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add irq handling
 * (c) 2020 Audioniek: ported to Opticum HD 9600 (TS)
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
 * 20201226 Audioniek       Initial version, based on cn_micom.
 *
 ****************************************************************************/
#ifndef _opt9600_fp_asc_h
#define _opt9600_fp_asc_h

/* ******************************************** */
/* Access ASC3; from u-boot; copied from corev  */
/* ******************************************** */

#if defined CONFIG_32BIT
#define ASC0BaseAddress  0xfd030000
#define ASC1BaseAddress  0xfd031000
#define ASC2BaseAddress  0xfd032000
#define ASC3BaseAddress  0xfd033000
#else
#define ASC0BaseAddress  0xb8030000
#define ASC1BaseAddress  0xb8031000
#define ASC2BaseAddress  0xb8032000
#define ASC3BaseAddress  0xb8033000
#endif

#define ASC_BAUDRATE     0x00
#define ASC_TX_BUFF      0x04
#define ASC_RX_BUFF      0x08
#define ASC_CTRL         0x0c
#define ASC_INT_EN       0x10
#define ASC_INT_STA      0x14
#define ASC_GUARDTIME    0x18
#define ASC_TIMEOUT      0x1c
#define ASC_TX_RST       0x20
#define ASC_RX_RST       0x24
#define ASC_RETRIES      0x28

#define ASC_INT_STA_RBF  0x0001
#define ASC_INT_STA_TE   0x0002
#define ASC_INT_STA_THE  0x0004
#define ASC_INT_STA_PE   0x0008
#define ASC_INT_STA_FE   0x0010
#define ASC_INT_STA_OE   0x0020
#define ASC_INT_STA_TONE 0x0040
#define ASC_INT_STA_TOE  0x0080
#define ASC_INT_STA_RHF  0x0100
#define ASC_INT_STA_TF   0x0200
#define ASC_INT_STA_NKD  0x0400

#define ASC_CTRL_FIFO_EN 0x0400


//-------------------------------------

#define PIO5BaseAddress  0xb8025000

#define PIO_CLR_PnC0     0x28
#define PIO_CLR_PnC1     0x38
#define PIO_CLR_PnC2     0x48
#define PIO_CLR_PnCOMP   0x58
#define PIO_CLR_PnMASK   0x68
#define PIO_CLR_PnOUT    0x08
#define PIO_PnC0         0x20
#define PIO_PnC1         0x30
#define PIO_PnC2         0x40
#define PIO_PnCOMP       0x50
#define PIO_PnIN         0x10
#define PIO_PnMASK       0x60
#define PIO_PnOUT        0x00
#define PIO_SET_PnC0     0x24
#define PIO_SET_PnC1     0x34
#define PIO_SET_PnC2     0x44
#define PIO_SET_PnCOMP   0x54
#define PIO_SET_PnMASK   0x64
#define PIO_SET_PnOUT    0x04

int serial_putc(char Data);
void serial_init(void);

extern unsigned int InterruptLine;
extern unsigned int ASCXBaseAddress;

#endif  // _opt9600_fp_asc_h
// vim:ts=4

