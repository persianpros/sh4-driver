/*
 * nuvoton_main.c
 *
 * (c) 2009 Dagobert@teamducktales
 * (c) 2010 Schischu & konfetti: Add irq handling
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
 *
 * Fortis HDBOX / 9200 / HS8200 / HS9510 / HS8200 / HS7XXX Front panel driver.
 *
 * Devices:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *  - /dev/rc  (reading of key events)
 *
 *
 ****************************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * --------------------------------------------------------------------------------------
 * 20130929 Audioniek       Initial version.
 * 20160523 Audioniek       procfs added.
 * 20170115 Audioniek       Response on getFrontInfo added.
 * 20170120 Audioniek       Spinner thread for FS9000/9200 added.
 * 20170128 Audioniek       Spinner thread for HS8200 added.
 * 20170202 Audioniek       Icon thread for HS8200 added.
 * 20170417 Audioniek       GMT offset parameter added, default plus one hour.
 * 20190406 Audioniek       HDbox: spinner had inner circle missing.
 *
 ****************************************************************************************/

/* BUGS:
 * - exiting the rcS leads to no data receiving from ttyAS0 ?!?!?
 * - some Icons are missing (Audioniek: not any more)
 * - buttons (see evremote)
 * - GetWakeUpMode not tested (dont know the meaning of the mode ;) )
 *   Audioniek: tested, returns the reason the frontprocessor was started
 *   (power on, timer rec or wake up from deep standby)
 *
 * Unknown Commands (SOP=0x02, EOP=0x03):
 * SOP 0x40 EOP
 * SOP 0xce 0x10 (no EOP?)
 * SOP 0xce 0x30 (no EOP?)
 * SOP 0x72 (no EOP?)
 * SOP 0x93 (no EOP?, beginning of SetLED)
 * The next two are written by the app every keypress. At first I think
 * this is the visual feedback but doing this manual have no effect.
 * SOP, 0x93, 0x01, 0x00, 0x08, EOP (=red LED off)
 * SOP, 0x93, 0xf2, 0x0a, 0x00, EOP (=blue LED + cross brightness 10)
 *
 * New commands from octagon1008:
 * SOP 0xd0 EOP
 *
 * SOP 0xc4 0x20 0x00 0x00 0x00 EOP (= SetIcon nothing)
 *
 * fixme icons: must be mapped somewhere! (Audioniek: done!)
 * 0x00 dolby
 * 0x01 dts
 * 0x02 video
 * 0x03 audio
 * 0x04 link
 * 0x05 hdd
 * 0x06 disk
 * 0x07 DVB
 * 0x08 DVD
 * fixme: usb icon at the side and some other? (Audioniek: done!)
 */

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/termbits.h>
#include <linux/kthread.h>
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

#include "nuvoton.h"
#include "nuvoton_asc.h"

//----------------------------------------------

#define EVENT_BTN                  0x51
#define EVENT_RC                   0x63
#define EVENT_STANDBY              0x80

#define EVENT_ANSWER_GETTIME       0x15
#define EVENT_ANSWER_WAKEUP_REASON 0x81
#define EVENT_ANSWER_GETIRCODE     0xa5
#define EVENT_ANSWER_GETPORT       0xb3 /* unused */
#define EVENT_ANSWER_FRONTINFO     0xd4
#define EVENT_ANSWER_E9            0xe9
#define EVENT_ANSWER_F9            0xf9

#define DATA_BTN_EVENT             2

//----------------------------------------------

short paramDebug = 10;
int waitTime = 1000;
int dataflag = 0;
static unsigned char expectEventData = 0;
static unsigned char expectEventId   = 1;
char *gmt_offset = "3600";  // GMT offset is plus one hour as default
int rtc_offset;

#define ACK_WAIT_TIME msecs_to_jiffies(500)

#define cPackageSizeRC       7
#define cPackageSizeFP       5
#define cPackageSizeStandby  5

#define cGetTimeSize         9
#define cGetWakeupReasonSize 5
#define cGetGetIrCodeSize   18
#define cGetFrontInfoSize    8
#define cGetE9Size          13
#define cGetF9Size          13

#if defined(ATEVIO7500) \
 || defined(FORTIS_HDBOX) \
 || defined(OCTAGON1008)
#define cMinimumSize         4
#else
#define cMinimumSize         5
#endif

#define BUFFERSIZE 256 //must be 2 ^ n
static unsigned char RCVBuffer [BUFFERSIZE];
static int           RCVBufferStart = 0, RCVBufferEnd = 0;

static unsigned char KeyBuffer [BUFFERSIZE];
static int           KeyBufferStart = 0, KeyBufferEnd = 0;

static unsigned char OutBuffer [BUFFERSIZE];
static int           OutBufferStart = 0, OutBufferEnd = 0;

static wait_queue_head_t wq;
static wait_queue_head_t rx_wq;
static wait_queue_head_t ack_wq;
static int dataReady = 0;

/****************************************************************************************/

/* queue data ->transmission is done in the irq */
void nuvoton_putc(unsigned char data)
{
	unsigned int *ASC_X_INT_EN = (unsigned int *)(ASCXBaseAddress + ASC_INT_EN);
	OutBuffer [OutBufferStart] = data;
	OutBufferStart = (OutBufferStart + 1) % BUFFERSIZE;
	/* if irq is not enabled, enable it */
	if (!(*ASC_X_INT_EN & ASC_INT_STA_THE))
	{
		*ASC_X_INT_EN = *ASC_X_INT_EN | ASC_INT_STA_THE;
	}
}

//----------------------------------------------

void ack_sem_up(void)
{
	dataReady = 1;
	wake_up_interruptible(&ack_wq);
}

int ack_sem_down(void)
{
	int err = 0;

	dataReady = 0;

	err  = wait_event_interruptible_timeout(ack_wq, dataReady == 1, ACK_WAIT_TIME);
	if (err == -ERESTARTSYS)
	{
		dprintk(1, "wait_event_interruptible failed\n");
		return err;
	}
	else if (err == 0)
	{
		dprintk(1, "Timeout waiting on ACK\n");
	}
	else
	{
		dprintk(20, "Command processed - remaining jiffies %d\n", err);
	}
	dataflag = dataReady;
	return 0;
}
EXPORT_SYMBOL(ack_sem_down);

//------------------------------------------------------------------

int getLen(int expectedLen)
{
	int i, j, len;
	i = 0;
	j = RCVBufferEnd;

	while (1)
	{
		if (RCVBuffer[j] == EOP)
		{
			if ((expectedLen == -1) || (i == expectedLen - 1))
			{
				break;
			}
		}
		j++;
		i++;

		if (j >= BUFFERSIZE)
		{
			j = 0;
		}

		if (j == RCVBufferStart)
		{
			i = -1;
			break;
		}
	}
	len = i + 1;
	return len;
}

void getRCData(unsigned char *data, int *len)
{
	int i, j;

	*len = 0;

	while (KeyBufferStart == KeyBufferEnd)
	{
		dprintk(200, "%s %d - %d\n", __func__, KeyBufferStart, KeyBufferEnd);

		if (wait_event_interruptible(wq, KeyBufferStart != KeyBufferEnd))
		{
			dprintk(1, "wait_event_interruptible failed\n");
			return;
		}
	}

	i = (KeyBufferEnd + DATA_BTN_EVENT) % BUFFERSIZE;

	if (KeyBuffer[i] == EVENT_BTN)
	{
		*len = cPackageSizeFP;
	}
	else if (KeyBuffer[i] == EVENT_RC)
	{
		*len = cPackageSizeRC;
	}

	if (*len <= 0)
	{
		*len = 0;
		return;
	}

	i = 0;
	j = KeyBufferEnd;

	while (i != *len)
	{
		data[i] = KeyBuffer[j];
		j++;
		i++;
		if (j >= BUFFERSIZE)
		{
			j = 0;
		}

		if (j == KeyBufferStart)
		{
			break;
		}
	}

	KeyBufferEnd = (KeyBufferEnd + *len) % BUFFERSIZE;
	dprintk(150, " < len %d, End %d\n", *len, KeyBufferEnd);
}

void handleCopyData(int len)
{
	int i, j;

	unsigned char *data = kmalloc(len - 4, GFP_KERNEL);
	i = 0;

	/* filter 0x00 0x02 cmd */
	j = (RCVBufferEnd + 3) % BUFFERSIZE;
	while (i != len - 4)
	{
		data[i] = RCVBuffer[j];
		dprintk(1, "Received data[%02d]=0x%02x\n", i, data[i]);
		j++;
		i++;

		if (j >= BUFFERSIZE)
		{
			j = 0;
		}

		if (j == RCVBufferStart)
		{
			break;
		}
	}
	copyData(data, len - 4);
	kfree(data);
}

void dumpData(void)
{
	int i, j, len;

	if (paramDebug < 150)
	{
		return;
	}
	len = getLen(-1);

	if (len == 0)
	{
		return;
	}
	i = RCVBufferEnd;
	for (j = 0; j < len; j++)
	{
		dprintk(1, "Dump #%02d: 0x%02x\n", i, RCVBuffer[i]);
		i++;

		if (i >= BUFFERSIZE)
		{
			i = 0;
		}

		if (i == RCVBufferStart)
		{
			i = -1;
			break;
		}
	}
	printk("\n");
}

void dumpValues(void)
{
	dprintk(150, "BufferStart %d, BufferEnd %d, len %d\n", RCVBufferStart, RCVBufferEnd, getLen(-1));

	if (RCVBufferStart != RCVBufferEnd)
	{
		if (paramDebug >= 50)
		{
			dumpData();
		}
	}
}

static void processResponse(void)
{
	int len, sizelen, i;

	dumpData();
	len = getLen(-1);

	if (len < cMinimumSize)
	{
		return;
	}
	dumpData();

	if (expectEventId)
	{
		/* DATA_BTN_EVENT can be wrapped to start */
		int index = (RCVBufferEnd + DATA_BTN_EVENT) % BUFFERSIZE;

		expectEventData = RCVBuffer[index];
		expectEventId = 0;
	}

	dprintk(100, "event 0x%02x %d %d\n", expectEventData, RCVBufferStart, RCVBufferEnd);
	if (expectEventData)
	{
		switch (expectEventData)
		{
			case EVENT_BTN:
			{
				/* no longkeypress for frontpanel buttons! */
				len = getLen(cPackageSizeFP);
				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cPackageSizeFP)
				{
					goto out_switch;
				}
				dprintk(1, "EVENT_BTN complete\n");
				if (paramDebug >= 50)
				{
					dumpData();
				}
				/* copy data */
				for (i = 0; i < cPackageSizeFP; i++)
				{
					int from, to;

					from = (RCVBufferEnd + i) % BUFFERSIZE;
					to = KeyBufferStart % BUFFERSIZE;

					KeyBuffer[to] = RCVBuffer[from];
					KeyBufferStart = (KeyBufferStart + 1) % BUFFERSIZE;
				}
				wake_up_interruptible(&wq);
				RCVBufferEnd = (RCVBufferEnd + cPackageSizeFP) % BUFFERSIZE;
				break;
			}
			case EVENT_RC:
			{
				len = getLen(cPackageSizeRC);

				if (len == 0)
				{
					goto out_switch;
				}
				if (len < cPackageSizeRC)
				{
					goto out_switch;
				}
				dprintk(1, "EVENT_RC complete %d %d\n", RCVBufferStart, RCVBufferEnd);
				if (paramDebug >= 50)
				{
					dumpData();
				}
				/* copy data */
				for (i = 0; i < cPackageSizeRC; i++)
				{
					int from, to;

					from = (RCVBufferEnd + i) % BUFFERSIZE;
					to = KeyBufferStart % BUFFERSIZE;

					KeyBuffer[to] = RCVBuffer[from];

					KeyBufferStart = (KeyBufferStart + 1) % BUFFERSIZE;
				}
				wake_up_interruptible(&wq);

				RCVBufferEnd = (RCVBufferEnd + cPackageSizeRC) % BUFFERSIZE;
				break;
			}
			case EVENT_ANSWER_GETTIME:
			{
				sizelen = cGetTimeSize;
				goto rsp_common;
			}
			case EVENT_ANSWER_WAKEUP_REASON:
			{
				sizelen = cGetWakeupReasonSize;
				goto rsp_common;
			}
			case EVENT_ANSWER_FRONTINFO:
			{
				sizelen = cGetFrontInfoSize;
				goto rsp_common;
			}
#if 1
			case EVENT_ANSWER_E9:
			{
				sizelen = cGetE9Size;
				goto rsp_common;
			}
			case EVENT_ANSWER_F9:
			{
				sizelen = cGetF9Size;
				goto rsp_common;
			}
#endif
rsp_common:
			{
				len = getLen(sizelen);

				if (len == 0)
				{
					goto out_switch;
				}

				if (len < sizelen)
				{
					goto out_switch;
				}
				handleCopyData(len);
				dprintk(20, "Pos. response received\n");
				errorOccured = 0;
				ack_sem_up();
				RCVBufferEnd = (RCVBufferEnd + sizelen) % BUFFERSIZE;
				break;
			}
			case EVENT_ANSWER_GETIRCODE:
			case EVENT_ANSWER_GETPORT:
			default: // Ignore Response
			{
				dprintk(1, "Invalid Response %02x\n", expectEventData);
				dprintk(1, "start %d end %d\n",  RCVBufferStart,  RCVBufferEnd);
				dumpData();

				/* discard all data, because this happens currently
				 * sometimes. do not know the problem here.
				 */
				RCVBufferEnd = RCVBufferStart;
				break;
			}
		}
	}
out_switch:
	expectEventId = 1;
	expectEventData = 0;
}

static irqreturn_t FP_interrupt(int irq, void *dev_id)
{
	unsigned int *ASC_X_INT_STA = (unsigned int *)(ASCXBaseAddress + ASC_INT_STA);
	unsigned int *ASC_X_INT_EN  = (unsigned int *)(ASCXBaseAddress + ASC_INT_EN);
	char         *ASC_X_RX_BUFF = (char *)(ASCXBaseAddress + ASC_RX_BUFF);
	char         *ASC_X_TX_BUFF = (char *)(ASCXBaseAddress + ASC_TX_BUFF);
	int          dataArrived = 0;

	if (paramDebug > 100)
	{
		dprintk(1, "i - ");
	}

	while (*ASC_X_INT_STA & ASC_INT_STA_RBF)
	{
		RCVBuffer [RCVBufferStart] = *ASC_X_RX_BUFF;
		RCVBufferStart = (RCVBufferStart + 1) % BUFFERSIZE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		// We are too fast, let's make a break
		udelay(0);
#endif
		dataArrived = 1;

		if (RCVBufferStart == RCVBufferEnd)
		{
			dprintk(1, "FP: RCV buffer overflow!!! (%d - %d)\n", RCVBufferStart, RCVBufferEnd);
		}
	}

	if (dataArrived)
	{
		wake_up_interruptible(&rx_wq);
	}

	while ((*ASC_X_INT_STA & ASC_INT_STA_THE) && (*ASC_X_INT_EN & ASC_INT_STA_THE) && (OutBufferStart != OutBufferEnd))
	{
		*ASC_X_TX_BUFF = OutBuffer[OutBufferEnd];
		OutBufferEnd = (OutBufferEnd + 1) % BUFFERSIZE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		// We are too fast, let's make a break
		udelay(0);
#endif
	}

	/* if all the data is transmitted disable irq, otherwise
	 * system is overflowed with irq's
	 */
	if (OutBufferStart == OutBufferEnd)
	{
		if (*ASC_X_INT_EN & ASC_INT_STA_THE)
		{
			*ASC_X_INT_EN &= ~ASC_INT_STA_THE;
		}
	}
	return IRQ_HANDLED;
}

int nuvotonTask(void *dummy)
{
	daemonize("nuvotonTask");
	allow_signal(SIGTERM);
	while (1)
	{
		int dataAvailable = 0;
		if (wait_event_interruptible(rx_wq, (RCVBufferStart != RCVBufferEnd)))
		{
			dprintk(1, "wait_event_interruptible failed\n");
			continue;
		}

		if (RCVBufferStart != RCVBufferEnd)
		{
			dataAvailable = 1;
		}
		while (dataAvailable)
		{
			processResponse();

			if (RCVBufferStart == RCVBufferEnd)
			{
				dataAvailable = 0;
			}
			dprintk(150, "start %d end %d\n",  RCVBufferStart,  RCVBufferEnd);
		}
	}
	printk("nuvotonTask died!\n");
	return 0;
}

//----------------------------------------------

#if defined(FORTIS_HDBOX)
/*********************************************************************
 *
 * spinner_thread: Thread to display the spinner on FS9000/9200.
 *
 */
static int spinner_thread(void *arg)
{
	int i = 0;
	int res = 0;
	char buffer[9];

	if (spinner_state.status == ICON_THREAD_STATUS_RUNNING)
	{
		return 0;
	}
	dprintk(10, "%s: starting\n", __func__);
	spinner_state.status = ICON_THREAD_STATUS_INIT;

	buffer[0] = SOP;
	buffer[1] = cCommandSetIconII;
	buffer[2] = 0x20;
	buffer[8] = EOP;

	spinner_state.status = ICON_THREAD_STATUS_RUNNING;
	dprintk(10, "%s: started\n", __func__);

	while (!kthread_should_stop())
	{
		if (!down_interruptible(&spinner_state.sem))
		{
			if (kthread_should_stop())
			{
				break;
			}

			while (!down_trylock(&spinner_state.sem));
			{
				dprintk(10, "Start spinner, period = %d ms\n", spinner_state.period);
				regs[0x24] |= 0x01; // Circ0 on

				while ((spinner_state.state) && !kthread_should_stop())
				{
					spinner_state.status == ICON_THREAD_STATUS_RUNNING;
					switch (i) // display a rotating disc in 16 states
					{
						case 0:
						{
							regs[0x20] |= 0x01;  // Circ1 on
							regs[0x21] &= 0xfc;  // Circ3 & Circ8 off
							regs[0x22] &= 0xfc;  // Circ2 & Circ6 off
							regs[0x23] &= 0xfc;  // Circ4 & Circ7 off
							regs[0x24] &= 0xfd;  // Circ5 off
							break;
						}
						case 1:
						{
							regs[0x22] |= 0x01;  // Circ2 on
							break;
						}
						case 2:
						{
							regs[0x21] |= 0x02;  // Circ3 on
							break;
						}
						case 3:
						{
							regs[0x23] |= 0x02;  // Circ4 on
							break;
						}
						case 4:
						{
							regs[0x24] |= 0x02;  // Circ5 on
							break;
						}
						case 5:
						{
							regs[0x22] |= 0x02;  // Circ6 on
							break;
						}
						case 6:
						{
							regs[0x23] |= 0x01;  // Circ7 on
							break;
						}
						case 7:
						{
							regs[0x21] |= 0x01;  // Circ8 on
							break;
						}
						case 8:
						{
							regs[0x20] &= 0xfe;  // Circ1 off
							break;
						}
						case 9:
						{
							regs[0x22] &= 0xfe;  // Circ2 off
							break;
						}
						case 10:
						{
							regs[0x21] &= 0xfd;  // Circ3 off
							break;
						}
						case 11:
						{
							regs[0x23] &= 0xfd;  // Circ4 off
							break;
						}
						case 12:
						{
							regs[0x24] &= 0xfd;  // Circ5 off
							break;
						}
						case 13:
						{
							regs[0x22] &= 0xfd;  // Circ6 off
							break;
						}
						case 14:
						{
							regs[0x23] &= 0xfe;  // Circ7 off
							break;
						}
						case 15:
						{
							regs[0x21] &= 0xfe;  // Circ8 off
							break;
						}
					}
					buffer[3] = regs[0x20];
					buffer[4] = regs[0x21];
					buffer[5] = regs[0x22];
					buffer[6] = regs[0x23];
					buffer[7] = regs[0x24];
					res = nuvotonWriteCommand(buffer, 9, 0);
					i++;
					i %= 16;
					msleep(spinner_state.period);
				}
				buffer[3] = regs[0x20] &= 0xfe;  // Circ1 off
				buffer[4] = regs[0x21] &= 0xfc;  // Circ3 & Circ8 off
				buffer[5] = regs[0x22] &= 0xfc;  // Circ2 & Circ6 off
				buffer[6] = regs[0x23] &= 0xfc;  // Circ4 & Circ7 off
				buffer[7] = regs[0x24] &= 0xfc;  // Circ0 & Circ5 off
				res |= nuvotonWriteCommand(buffer, 9, 0);
				spinner_state.status = ICON_THREAD_STATUS_HALTED;
				dprintk(1, "%s: Spinner stopped\n", __func__);
			}
		}
	}
	dprintk(1, "%s stopped\n", __func__);
	spinner_state.status = ICON_THREAD_STATUS_STOPPED;
	spinner_state.task = 0;
	return res;
}
#endif

#if defined(ATEVIO7500)
/*********************************************************************
 *
 * spinner_thread: Thread to display the spinner on HS8200.
 *
 */
static int spinner_thread(void *arg)
{
	int i;
	int res = 0;
	char buffer[9];

	if (spinner_state.status == ICON_THREAD_STATUS_RUNNING)
	{
		return 0;
	}
	dprintk(100, "%s: starting\n", __func__);
	spinner_state.status = ICON_THREAD_STATUS_INIT;

	buffer[0] = SOP;
	buffer[1] = cCommandSetIconII;
	buffer[2] = 0x27;
	buffer[8] = EOP;

	dprintk(100, "%s: started\n", __func__);
	spinner_state.status = ICON_THREAD_STATUS_RUNNING;

	while (!kthread_should_stop())
	{
		if (!down_interruptible(&spinner_state.sem))
		{
			if (kthread_should_stop())
			{
				goto stop;
			}

			while (!down_trylock(&spinner_state.sem));
			{
				dprintk(10, "%s: Start spinner, period = %d ms\n", __func__, spinner_state.period);
				i = 0;
				while ((spinner_state.state) && !kthread_should_stop())
				{
					spinner_state.status = ICON_THREAD_STATUS_RUNNING;
					switch (i) // display a bar in 16 states
					{
						case 0: 
						{
							regs[0x27] = 0x00;
							regs[0x28] = 0x00;
							regs[0x29] = 0x0e;
							regs[0x2a] = 0x00;
							regs[0x2b] = 0x00;
							break;
						}
						case 1:
						{
							regs[0x2a] |= 0x04;
							regs[0x2b] |= 0x02;
							break;
						}
						case 2:
						{
							regs[0x2a] |= 0x08;
							regs[0x2b] |= 0x08;
							break;
						}
						case 3:
						{
							regs[0x2a] |= 0x10;
							regs[0x2b] |= 0x20;
							break;
						}
						case 4:
						{
							regs[0x29] |= 0x30;
							break;
						}
						case 5:
						{
							regs[0x27] |= 0x20;
							regs[0x28] |= 0x10;
							break;
						}
						case 6:
						{
							regs[0x27] |= 0x08;
							regs[0x28] |= 0x08;
							break;
						}
						case 7: // full 8 segments
						{
							regs[0x27] |= 0x02;
							regs[0x28] |= 0x04;
							break;
						}
						case 8: 
						{
							regs[0x29] &= 0xf8;
							break;
						}
						case 9:
						{
							regs[0x2a] &= 0xfb;
							regs[0x2b] &= 0xfd;
							break;
						}
						case 10:
						{
							regs[0x2a] &= 0xf7;
							regs[0x2b] &= 0xf7;
							break;
						}
						case 11:
						{
							regs[0x2a] &= 0xef;
							regs[0x2b] &= 0xdf;
							break;
						}
						case 12:
						{
							regs[0x29] &= 0xcf;
							break;
						}
						case 13:
						{
							regs[0x27] &= 0xdf;
							regs[0x28] &= 0xef;
							break;
						}
						case 14:
						{
							regs[0x27] &= 0xf7;
							regs[0x28] &= 0xf7;
							break;
						}
						case 15:
						{
							regs[0x27] &= 0xfd;
							regs[0x28] &= 0xfb;
							break;
						}
					}
					buffer[3] = regs[0x27];
					buffer[4] = regs[0x28];
					buffer[5] = regs[0x29];
					buffer[6] = regs[0x2a];
					buffer[7] = regs[0x2b];
					res = nuvotonWriteCommand(buffer, 9, 0);
					i++;
					i %= 16;
					msleep(spinner_state.period);
				}
				spinner_state.status = ICON_THREAD_STATUS_HALTED;
			}
stop:
			buffer[3] = regs[0x27] = 0x00;
			buffer[4] = regs[0x28] = 0x00;
			buffer[5] = regs[0x29] = 0x00;
			buffer[6] = regs[0x2a] = 0x00;
			buffer[7] = regs[0x2b] = 0x00;
			res |= nuvotonWriteCommand(buffer, 9, 0);
			dprintk(100, "%s: Spinner stopped\n", __func__);
		}
	}
	spinner_state.status = ICON_THREAD_STATUS_STOPPED;
	spinner_state.task = 0;
	dprintk(100, "%s: stopped\n", __func__);
	return res;
}

/*********************************************************************
 *
 * icon_thread: Thread to display icons on HS8200.
 * TODO: expand to animated icons
 *
 */
int icon_thread(void *arg)
{
	int i = 0;
	int res = 0;
	char buffer[9];

	if (icon_state.status == ICON_THREAD_STATUS_RUNNING || lastdata.icon_state[ICON_SPINNER])
	{
		return 0;
	}
	dprintk(100, "%s: starting\n", __func__);
	spinner_state.status = ICON_THREAD_STATUS_INIT;

	buffer[0] = SOP;
	buffer[1] = cCommandSetIconII;
	buffer[2] = 0x27;
	buffer[8] = EOP;

	icon_state.status = ICON_THREAD_STATUS_RUNNING;
	dprintk(100, "%s: started\n", __func__);

	while (!kthread_should_stop())
	{
		if (!down_interruptible(&icon_state.sem))
		{
			if (kthread_should_stop())
			{
				goto stop_icon;
			}
			while (!down_trylock(&icon_state.sem));
			{
				while ((icon_state.state) && !kthread_should_stop())
				{
					icon_state.status = ICON_THREAD_STATUS_RUNNING;
					if (lastdata.icon_count > 1)
					{
//						dprintk(1, "Display multiple icons\n");
						for (i = ICON_MIN + 1; i < ICON_MAX; i++)
						{
							if (lastdata.icon_state[i])
							{
								res |= nuvotonSetIcon(i, 1);
								msleep(3000);
							}
							if (!icon_state.state)
							{
								break;
							}
						}
					}
					else if (lastdata.icon_count == 1)
					{
						for (i = ICON_MIN + 1; i < ICON_MAX; i++)
						{
							if (lastdata.icon_state[i])
							{
//								dprintk(1, "Show icon %02d in animation\n", i);
// TODO: animation routine
								res = nuvotonSetIcon(i, 1);
								msleep(1500);
							}			
							if (!icon_state.state)
							{
								break;
							}
						}
					}
				}
				icon_state.status = ICON_THREAD_STATUS_HALTED;
			}
		}
	}
stop_icon:
	icon_state.status = ICON_THREAD_STATUS_STOPPED;
	icon_state.task = 0;
	dprintk(100, "%s stopped\n", __func__);
	return res;
}
#endif

extern void create_proc_fp(void);
extern void remove_proc_fp(void);

static int __init nuvoton_init_module(void)
{
	int i = 0;

	// Address for Interrupt enable/disable
	unsigned int *ASC_X_INT_EN = (unsigned int *)(ASCXBaseAddress + ASC_INT_EN);
	// Address for FIFO enable/disable
	unsigned int *ASC_X_CTRL   = (unsigned int *)(ASCXBaseAddress + ASC_CTRL);
	dprintk(100, "%s >\n", __func__);
	// Disable all ASC 2 interrupts
	*ASC_X_INT_EN = *ASC_X_INT_EN & ~0x000001ff;

	serial_init();

	init_waitqueue_head(&wq);
	init_waitqueue_head(&rx_wq);
	init_waitqueue_head(&ack_wq);

	for (i = 0; i < LASTMINOR; i++)
	{
		sema_init(&FrontPanelOpen[i].sem, 1);
	}

	kernel_thread(nuvotonTask, NULL, 0);

	//Enable the FIFO
	*ASC_X_CTRL = *ASC_X_CTRL | ASC_CTRL_FIFO_EN;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
	i = request_irq(InterruptLine, (void *)FP_interrupt, IRQF_DISABLED, "FP_serial", NULL);
#else
	i = request_irq(InterruptLine, (void *)FP_interrupt, SA_INTERRUPT, "FP_serial", NULL);
#endif

	if (!i)
	{
		*ASC_X_INT_EN = *ASC_X_INT_EN | 0x00000001;
	}
	else
	{
		dprintk(1, "Cannot get irq\n");
	}

	msleep(waitTime);
	nuvoton_init_func();

	if (register_chrdev(VFD_MAJOR, "VFD", &vfd_fops))
	{
		dprintk(1, "Unable to get major %d for VFD/NUVOTON\n", VFD_MAJOR);
	}

#if defined(FORTIS_HDBOX) \
 || defined(ATEVIO7500)
	spinner_state.state = 0;
	spinner_state.period = 0;
	spinner_state.status = ICON_THREAD_STATUS_STOPPED;
	sema_init(&spinner_state.sem, 0);
	spinner_state.task = kthread_run(spinner_thread, (void *) ICON_SPINNER, "spinner_thread");
#if defined(ATEVIO7500)
	icon_state.state = 0;
	icon_state.period = 0;
	icon_state.status = ICON_THREAD_STATUS_STOPPED;
	sema_init(&icon_state.sem, 0);
	icon_state.task = kthread_run(icon_thread, (void *) ICON_MIN, "icon_thread");
#endif
#endif
	create_proc_fp();

	dprintk(100, "%s <\n", __func__);
	return 0;
}

#if 0
static int icon_thread_active(void)
{
	int i;

	for (i = 0; i < ICON_SPINNER + 2; i++)
	{
		if (!icon_state[i].status && icon_state[i].icon_task)
		{
			return -1;
		}
	}
	return 0;
}
#endif

static void __exit nuvoton_cleanup_module(void)
{
	printk("[nuvoton] NUVOTON front processor module unloading\n");
	remove_proc_fp();

#if defined(FORTIS_HDBOX)
	if (!(spinner_state.status == ICON_THREAD_STATUS_STOPPED) && spinner_state.task)
	{
		dprintk(50, "Stopping spinner thread\n");
		up(&spinner_state.sem);
		kthread_stop(spinner_state.task);
	}
#elif defined(ATEVIO7500)
	if (!(spinner_state.status == ICON_THREAD_STATUS_STOPPED) && spinner_state.task)
	{
		dprintk(50, "Stopping spinner thread\n");
		up(&icon_state.sem);
		kthread_stop(spinner_state.task);
	}
	if (!(icon_state.status == ICON_THREAD_STATUS_STOPPED) && icon_state.task)
	{
		dprintk(50, "Stopping icon thread\n");
		up(&icon_state.sem);
		kthread_stop(icon_state.task);
	}
#endif
	unregister_chrdev(VFD_MAJOR, "VFD");
	free_irq(InterruptLine, NULL);
}

//----------------------------------------------

module_init(nuvoton_init_module);
module_exit(nuvoton_cleanup_module);

MODULE_DESCRIPTION("NUVOTON frontcontroller module");
MODULE_AUTHOR("Open Vision developers");
MODULE_LICENSE("GPL");

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

module_param(waitTime, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(waitTime, "Wait before init in ms (default=1000)");

module_param(gmt_offset, charp, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gmt_offset, "GMT offset (default 3600");
// vim:ts=4
