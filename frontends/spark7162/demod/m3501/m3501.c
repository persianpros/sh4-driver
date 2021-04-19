/****************************************************************************
 *
 * Ali M3501 DVB-S(2) demodulator driver
 *
 * Version for Fulan Spark7162
 *
 * Original code:
 * Copyright (C)2004 Ali Corporation. All Rights Reserved.
 *
 *    File:    This file contains S3501 DVBS2 basic function in LLD.
 *
 *    Description:    Header file in LLD.
 *
 *  History:
 *      Date            Author      Version   Reason
 *      ============    =========   =======   =================
 *  1.  05/18/2006      Berg        Ver 0.1   Create file for S3501 DVBS2 project
 *  2.  06/30/2007      Dietel                Add synchronous mechanism support
 *  3.  07/03/2007      Dietel                Add autoscan support ,Clean external macro
 *  4.  07/03/2007      Dietel                Not using VBV_BUFFER
 *
 *  5.  03/18/2008      Carcy Liu             Open EQ, only mask EQ when 1/4, 1/3, 2/5 code rate.
 *                                            -> key word is "EQ_ENABLE".
 *
 *  6.  03/18/2008      Carcy Liu             Add description about I2C through.  Note:
 *                                            M3501 support I2C through function, but it should be
 *                                            careful when open & close I2C through function, otherwise
 *                                            the I2C slave may be "deadlock". Please do it as below.
 *                                             1, open I2C through function.
 *                                             2, wait 50ms at least.
 *                                             3, configure tuner
 *                                             4, wait 50ms at least.
 *                                             5, close I2C through function.
 *                                             6, wait 50ms at least.
 *                                            -> key word is "I2C_THROUGH_EN"
 *
 *  7.  03/21/2008      Carcy Liu             Disable HBCD disable function, HBCD is DVBS2 H8PSK
 *                                            option mode, if enable HBCD, firmware will spend
 *                                            more time to check HBCD when channel change.
 *                                            Suggest that disable HBCD.
 *                                            -> key word is "HBCD_DISABLE"
 *
 *  8.  04/04/2008  Carcy Liu                 Update power control
 *                                                -> key word is "power control"
 *
 *  9.  04/18/2008  Carcy Liu                 Update code rate information.
 *
 *  10. 04/21/2008  Carcy Liu                 Add configure ZL10037's base band gain "G_BB_GAIN"
 *
 *  11. 04/23/2008  Carcy Liu                 Software MUST send a reset pulse to the M3501 reset pin
 *                                            before it perform a channel change. This is because a bug
 *                                            is found in M3501 power saving logic which may cause
 *                                            abnormal large power consumption if it is not reset before
 *                                            a channel change is performed. It has been proven that
 *                                            after reset this bug is cleared.
 *                                            Key word "m3501_reset"
 *
 *  12.  04/28/2008  Carcy Liu                Show how to get LDPC code and other information:
 *                                            (All information show in the function)
 *                                            nim_s3501_reg_get_code_rate(u8* code_rate)
 *                                            nim_s3501_reg_get_map_type(u8*map_type)
 *                                            nim_s3501_reg_get_work_mode(u8*work_mode)
 *                                            nim_s3501_reg_get_roll_off(u8* roll_off)
 *                                            1, use nim_s3501_reg_get_work_mode to get work mode: dvbs or dvbs2
 *                                                    //  Work Mode
 *                                                    //      0x0:    DVB-S
 *                                                    //      0x1:    DVB-S2
 *                                                    //      0x2:    DVB-S2 HBC
 *                                            2, use nim_s3501_reg_get_map_type to get modulate type: QPSK/8SPK/16AP   SK
 *                                                    //      Map type:
 *                                                    //      0x0:    HBCD.
 *                                                    //      0x1:    BPSK
 *                                                    //      0x2:    QPSK
 *                                                    //      0x3:    8PSK
 *                                                    //      0x4:    16APSK
 *                                                    //      0x5:    32APSK
 *                                            3, use nim_s3501_reg_get_code_rate to get LDPC code rate.
 *                                                    //  Code rate list
 *                                                    //  for DVB-S:
 *                                                    //      0x0:    1/2,
 *                                                    //      0x1:    2/3,
 *                                                    //      0x2:    3/4,
 *                                                    //      0x3:    5/6,
 *                                                    //      0x4:    6/7,
 *                                                    //      0x5:    7/8.
 *                                                    //  For DVB-S2:
 *                                                    //      0x0:    1/4,
 *                                                    //      0x1:    1/3,
 *                                                    //      0x2:    2/5,
 *                                                    //      0x3:    1/2,
 *                                                    //      0x4:    3/5,
 *                                                    //      0x5:    2/3,
 *                                                    //      0x6:    3/4,
 *                                                    //      0x7:    4/5,
 *                                                    //      0x8:    5/6,
 *                                                    //      0x9:    8/9,
 *                                                    //      0xa:    9/10
 *                                            Key word "ldpc_code"
 *
 *  13. 04/28/2008  Carcy Liu                 Add "nim_s3501_hw_init()" in m3501_reset().
 *                                            Make sure hardware reset successfully.
 *                                            Key word "nim_s3501_hw_init"
 *
 *  14. 2008-5-27   Carcy Liu                 a.  Add dynamic power control: nim_s3501_dynamic_power()
 *                                                This function should be called every some seconds. In fact,
 *                                                MPEG host chip should get signal quality and intensity every
 *                                                some second, You can add nim_s3501_dynamic_power to it's tail.
 *                                                Every time when MPEG host chip get signal quality, it monitor
 *                                                and control M3501's power at the same time.
 *
 *                                            b.  Add clock control in channel change, if channel lock,
 *                                                readback workmode, if workmode is DVBS, then slow down DVBS2
 *                                                clock to save power.
 *
 *                                            c.  Add description for get signal quality and get signal intensity
 *                                                c1: get signal quality : nim_s3501_get_SNR
 *                                                c2: get signal intensity: nim_s3501_get_AGC
 *
 *                                            d:  nim_s3501_get_BER and nim_s3501_get_PER.
 *                                                For DVBS2, BER is from BCH, it's meaning is very different with
 *                                                from viterbi in DVBS. So we use packet error rate (PER) for
 *                                                nim_s3501_get_BER when work in DVBS2.
 *
 *                                                Key word : "power_ctrl"
 *
 *   15. 2008-12-2  Douglass Yan              a. Delete some local static variable
 *                                            b. Change attach function name
 *                                            c. add variable to private structure to save qpsk addr
 *                                            d. Update local function parameters num
 *
 *
 *   16.2008-12-11 Douglass Yan               a. Delete all shared variable for support Dual S3501
 *
 *   17.2008-12-16 Douglass Yan               a. Delete maco REVERT_POLAR
 *
 *****************************************************************************/
#include <linux/version.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/dvb/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#  include <linux/stpio.h>
#else
#  include <linux/stm/pio.h>
#endif

#include "dvbdev.h"
#include "dmxdev.h"
#include "dvb_frontend.h"

#include "ywdefs.h"

#include "m3501_ext.h"
#include "m3501.h"

extern int paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[m3501] "

struct dvb_m3501_fe_state
{
	struct dvb_frontend frontend;
	struct nim_device   spark_nimdev;
	struct stpio_pin    *fe_lnb_13_18;
	struct stpio_pin    *fe_lnb_on_off;
};

#define I2C_ERROR_BASE       -200

#define ERR_I2C_SCL_LOCK     (I2C_ERROR_BASE - 1)  /* I2C SCL be locked */
#define ERR_I2C_SDA_LOCK     (I2C_ERROR_BASE - 2)  /* I2C SDA be locked */
#define ERR_I2C_NO_ACK       (I2C_ERROR_BASE - 3)  /* I2C slave no ack */

#define S3501_ERR_I2C_NO_ACK ERR_I2C_NO_ACK
#define T_CTSK               OSAL_T_CTSK

extern int FFT_energy_1024[1024];

static const u8 ssi_clock_tab[] =
{
	98, 90, 83, 77, 72, 67, 60, 54, 50
};

static const u8 MAP_BETA_ACTIVE_BUF[32] =
{
	0x00,   //                // index 0, do not use
	0x01,   // 1/4 of QPSK    // 01
	0x01,   // 1/3            // 02
	0x01,   // 2/5            // 03
	0x01,   // 1/2            // 04
	0x01,   // 3/5            // 05
	0x01,   // 2/3            // 06
	0x01,   // 3/4            // 07
	0x01,   // 4/5            // 08
	0x01,   // 5/6            // 09
	0x01,   // 8/9            // 0a
	0x01,   // 9/10           // 0b
	0x01,   // 3/5 of 8PSK    // 0c
	0x01,   // 2/3            // 0d
	0x01,   // 3/4            // 0e
	0x01,   // 5/6            // 0f
	0x01,   // 8/9            // 10
	0x01,   // 9/10           // 11
	0x01,   // 2/3 of 16APSK  // 12
	0x01,   // 3/4            // 13
	0x01,   // 4/5            // 14
	0x01,   // 5/6            // 15
	0x01,   // 8/9            // 16
	0x01,   // 9/10           // 17
	0x01,   // for 32 APSK, do not use
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01
};

static const u8 MAP_BETA_BUF[32] =
{
	188,   //  205,     // index 0, do not use
	188,   //  205,     // 1/4 of QPSK    // 01
	190,   //  230,     // 1/3            // 02
	205,   //  205,     // 2/5            // 03
	205,   //  205,     // 1/2            // 04
	180,   //  180,     // 3/5            // 05
	180,   //  180,     // 2/3            // 06
	180,   //  180,     // 3/4            // 07
	155,   //  180,     // 4/5            // 08
	168,   //  180,     // 5/6            // 09
	150,   //  155,     // 8/9            // 0a
	150,   //  155,     // 9/10           // 0b
	180,   //  180,     // 3/5 of 8PSK    // 0c
	180,   //  180,     // 2/3            // 0d
	170,   //  180,     // 3/4            // 0e
	180,   //  155,     // 5/6            // 0f
	150,   //  155,     // 8/9            // 10
	150,   //  155,     // 9/10           // 11
	180,   //  205,     // 2/3 of 16APSK  // 12
	180,   //  180,     // 3/4            // 13
	180,   //  180,     // 4/5            // 14
	170,   //  155,     // 5/6            // 15
	155,   //  155,     // 8/9            // 16
	155,   //  155,     // 9/10           // 17
	155,   //-------    for 32 APSK,  do not use
	155, 155, 155, 155, 155, 155, 155
};

static const u16 DEMAP_NOISE[32] =
{
	0x0000,  // index 0, do not use
	0x016b,  // 1/4 of QPSK    // 01
	0x01d5,  // 1/3            // 02
	0x0246,  // 2/5            // 03
	0x0311,  // 1/2            // 04
	0x0413,  // 3/5            // 05
	0x04fa,  // 2/3            // 06
	0x062b,  // 3/4            // 07
	0x0729,  // 4/5            // 08
	0x080c,  // 5/6            // 09
	0x0a2a,  // 8/9            // 0a
	0x0ab2,  // 9/10           // 0b
	0x08a9,  // 3/5 of 8PSK    // 0c
	0x0b31,  // 2/3            // 0d
	0x0f1d,  // 3/4            // 0e
	0x1501,  // 5/6            // 0f
	0x1ca5,  // 8/9            // 10
	0x1e91,  // 9/10           // 11
	0x133b,  // 2/3 of 16APSK  // 12
	0x199a,  // 3/4            // 13
	0x1f08,  // 4/5            // 14
	0x234f,  // 5/6            // 15
	0x2fa1,  // 8/9            // 16
	0x3291,  // 9/10           // 17
	0x0000,  // for 32 APSK, do not use
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const u8 snr_tab[177] =
{
	  0,   1,   3,   4,   5,   7,   8,
	  9,  10,  11,  13,  14,  15,  16,  17,
	 18,  19,  20,  21,  23,  24,  25,  26,
	 26,  27,  28,  29,  30,  31,  32,  33,
	 34,  35,  35,  36,  37,  38,  39,  39,
	 40,  41,  42,  42,  43,  44,  45,  45,
	 46,  47,  47,  48,  49,  49,  50,  51,
	 51,  52,  52,  53,  54,  54,  55,  55,
	 56,  57,  57,  58,  58,  59,  59,  60,
	 61,  61,  62,  62,  63,  63,  64,  64,
	 65,  66,  66,  67,  67,  68,  68,  69,
	 69,  70,  70,  71,  72,  72,  73,  73,
	 74,  74,  75,  76,  76,  77,  77,  78,
	 79,  79,  80,  80,  81,  82,  82,  83,
	 84,  84,  85,  86,  86,  87,  88,  89,
	 89,  90,  91,  92,  92,  93,  94,  95,
	 96,  96,  97,  98,  99, 100, 101, 102,
	102, 103, 104, 105, 106, 107, 108, 109,
	110, 111, 113, 114, 115, 116, 117, 118,
	119, 120, 122, 123, 124, 125, 127, 128,
	129, 131, 132, 133, 135, 136, 138, 139,
	141, 142, 144, 145, 147, 148, 150, 152,
	153, 155
};

#define SUCCESS        0
#define ERR_FAILED    -1
#define ERR_TIME_OUT  -3  /* Waiting time out */

int nim_reg_read(struct nim_device *dev, u8 bMemAdr, u8 *pData, u8 bLen)
{
	int ret;
	struct nim_s3501_private *priv_mem;
	u8 b0[] = { bMemAdr };

	struct i2c_msg msg[] =
	{
		{ .addr = dev->base_addr, .flags = 0, .buf = b0, .len = 1 },
		{ .addr = dev->base_addr, .flags = I2C_M_RD, .buf = pData, .len = bLen }
	};

	priv_mem = (struct nim_s3501_private *)dev->priv;

	ret = i2c_transfer(priv_mem->i2c_adap, msg, 2);
	if (ret != 2)
	{
		if (ret != -ERESTARTSYS)
		{
			dprintk(1, "%s: Read error, Reg=[0x%02x], Status=%d\n", __func__, bMemAdr, ret);
		}
		return ret < 0 ? ret : -EREMOTEIO;
	}
	return ret;
}

int nim_reg_write(struct nim_device *dev, u8 bMemAdr, u8 *pData, u8 bLen)
{
	int ret;
	u8 buf[bLen + 1];
	struct nim_s3501_private *priv_mem;
	struct i2c_msg i2c_msg = { .addr = dev->base_addr, .flags = 0, .buf = buf, .len = bLen + 1 };

#if 0
	if (paramDebug > 50)
	{
		int i;

		dprintk(1, "%s I2Cmessage ", __func__);
		for (i = 0; i < bLen + 1; i++)
		{
			printk("%02x ", pData[i]);
		}
		printk("\n");
	}
#endif
	priv_mem = (struct nim_s3501_private *)dev->priv;

	buf[0] = bMemAdr;
	memcpy(&buf[1], pData, bLen);

	ret = i2c_transfer(priv_mem->i2c_adap, &i2c_msg, 1);

	if (ret != 1)
	{
		if (ret != -ERESTARTSYS)
		{
			dprintk(1, "%s: Reg=[0x%04x], Data=[0x%02x ...], Count=%u, Status=%d\n", __func__, bMemAdr, pData[0], bLen, ret);
		}
		return ret < 0 ? ret : -EREMOTEIO;
	}
	return ret;
}

#if defined(NIM_3501_FUNC_EXT)
void reg_read_verification(struct nim_device *dev)
{
	u8 ver_data;
	int i, j, k;

	static u8 m_reg_list[192] =
	{
		0x11, 0x00, 0x00, 0x0f, 0x00, 0x5a, 0x50, 0x2f, 0x48, 0x3f, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x83, 0x00, 0x00, 0xec, 0x17, 0x1e,
		0x17, 0xec, 0x6b, 0x8e, 0xb1, 0x28, 0xba, 0x0c, 0x60, 0x00, 0x03, 0x05, 0x00, 0x20, 0x10, 0x04, 0x00, 0x00, 0xe3, 0x88, 0x4f, 0x2d,
		0x68, 0x06, 0x5a, 0x10, 0x77, 0xe5, 0x24, 0xaa, 0x45, 0x87, 0x51, 0x52, 0xba, 0x46, 0x80, 0x3e, 0x25, 0x14, 0x28, 0x3f, 0xc0, 0x00,
		0x02, 0x83, 0x00, 0x00, 0x00, 0x7f, 0x98, 0x48, 0x38, 0x58, 0xc2, 0x12, 0xd2, 0x00, 0x10, 0x27, 0x60, 0xea, 0x71, 0x00, 0x00, 0x32,
		0x10, 0x08, 0x00, 0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0xb0, 0x30, 0x30, 0x59, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x50, 0x5a, 0x57, 0x0b, 0x45, 0xc6, 0x41, 0x3c, 0x04, 0x20, 0x70,
		0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x35, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xca, 0x7c, 0x21, 0x04, 0xb4, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00
	};

	for (i = 0; i <= 0xc7; i++)
	{
		if ((0x03 == i) ||
				(0x04 == i) ||
				(0x0b == i) ||
				(0x0c == i) ||
				(0x11 == i) ||
				(0x12 == i) ||
				(0x26 == i) ||
				(0x27 == i) ||
				(0x45 == i) ||
				(0x46 == i) ||
				(0x55 == i) ||
				(0x56 == i) ||
				(0x5a == i) ||
				(0x68 == i) ||
				(0x69 == i) ||
				(0x6a == i) ||
				(0x6b == i) ||
				(0x6c == i) ||
				(0x6d == i) ||
				(0x6e == i) ||
				(0x6f == i) ||
				(0x72 == i) ||
				(0x73 == i) ||
				(0x7d == i) ||
				(0x86 == i) ||
				(0x87 == i) ||
				(0x88 == i) ||
				(0x89 == i) ||
				(0x8a == i) ||
				(0x8b == i) ||
				(0x8c == i) ||
				(0x8d == i) ||
				(0x9d == i) ||
				(0xa3 == i) ||
				(0xa4 == i) ||
				(0xa5 == i) ||
				(0xa6 == i) ||
				(0xaa == i) ||
				(0xab == i) ||
				(0xac == i) ||
				(0xb2 == i) ||
				(0xb1 == i) ||
				(0xb4 == i) ||
				(0xba == i) ||
				(0xbb == i) ||
				(0xbc == i) ||
				(0xc0 == i) ||
				(0xc1 == i) ||
				(0xc4 == i) ||
				(0xc2 == i) ||
				(0xc3 == i))
		{
		}
		else
		{
			//for (j = 0; j < 1000; j++)
			{
				nim_reg_read(dev, i, &ver_data, 1);
				/* for (k = 0; k < 1000; k++)
				{
				    j = j;
				}*/
				if (ver_data != m_reg_list[i])
				{
					dprintk(1, " read reg number :0x%x ; ver_data :0x%x ,reg_list_data : 0x%x\n", i, ver_data, m_reg_list[i]);
				}
			}
		}
	}
	dprintk(100, "finish register read test\n");
	return;
}

void reg_write_verification(struct nim_device *dev)
{
	u8 ver_data;
	u8 write_data;
	int i;

	for (i = 0x0; i <= 0xc7; i++)
	{
		if ((0x00 == i)
		||  (0xb1 == i)
		||  (0x04 == i)
		||  (0x0b == i)
		||  (0x0c == i)
		||  (0x11 == i)
		||  (0x12 == i)
		||  (0x26 == i)
		||  (0x27 == i)
		||  (0x45 == i)
		||  (0x46 == i)
		||  (0x55 == i)
		||  (0x56 == i)
		||  (0x5a == i)
		||  (0x68 == i)
		||  (0x69 == i)
		||  (0x6a == i)
		||  (0x6b == i)
		||  (0x6c == i)
		||  (0x6d == i)
		||  (0x6e == i)
		||  (0x6f == i)
		||  (0x72 == i)
		||  (0x73 == i)
		||  (0x7d == i)
		||  (0x86 == i)
		||  (0x87 == i)
		||  (0x88 == i)
		||  (0x89 == i)
		||  (0x8a == i)
		||  (0x8b == i)
		||  (0x8c == i)
		||  (0x8d == i)
		||  (0x9d == i)
		||  (0xa3 == i)
		||  (0xa4 == i)
		||  (0xa5 == i)
		||  (0xa6 == i)
		||  (0xaa == i)
		||  (0xab == i)
		||  (0xac == i)
		||  (0xb2 == i)
		||  (0xb3 == i)
		||  (0xb4 == i)
		||  (0xba == i)
		||  (0xbb == i)
		||  (0xbc == i)
		||  (0xc2 == i)
		||  (0xc3 == i)
		||  (0xc4 == i))
		{
		}
		else
		{
			dprintk(1, " i=%d\n", i);
			write_data = 0x5a;
			nim_reg_write(dev, i, &write_data, 1);
			nim_reg_read(dev, i, &ver_data, 1);

			if (ver_data != write_data)
			{
				dprintk(1, " write reg number :0x%x ; write data : %x; ver_data :0x%x\n", i, write_data, ver_data);
			}
			write_data = 0xa6;
			nim_reg_write(dev, i, &write_data, 1);

			nim_reg_read(dev, i, &ver_data, 1);
			if (ver_data != write_data)
			{
				dprintk(1, " write reg number :0x%x  ; write data : %x; ver_data :0x%x\n", i, write_data, ver_data);
			}
		}
	}
	dprintk(100, "reg_write_verfication all finish\n");
	return;
}
#endif  // NIM_3501_FUNC_EXT

/*****************************************************************************
* int nim_s3501_attach (struct QPSK_TUNER_CONFIG_API * ptrQPSK_Tuner)
* Description: S3501 initialization
*
* Arguments:
*  none
*
* Return Value: int
*****************************************************************************/
//__attribute__((section(".reuse")))
#if 0
int nim_s3501_attach(struct QPSK_TUNER_CONFIG_API *ptrQPSK_Tuner)
{
	struct nim_device *dev;
	struct nim_s3501_private *priv_mem;
	static u8 nim_dev_num = 0;

	if (ptrQPSK_Tuner == NULL)
	{
		dprintk(1, "Tuner Configuration API structure is NULL!/n");
		return ERR_NO_DEV;
	}
	if (nim_dev_num > 2)
	{
		dprintk(1, "Cannot support more than two M3501's!/n");
		return ERR_NO_DEV;
	}
	dev = (struct nim_device *) dev_alloc(nim_s3501_name[nim_dev_num], HLD_DEV_TYPE_NIM, sizeof(struct nim_device));
	if (dev == NULL)
	{
		dprintk(1, "Error: Alloc nim device error!\n");
		return ERR_NO_MEM;
	}

	priv_mem = (struct nim_s3501_private *) comm_malloc(sizeof(struct nim_s3501_private));
	if (priv_mem == NULL)
	{
		dev_free(dev);
		dprintk(1, "Alloc nim device prive memory error!/n");
		return ERR_NO_MEM;
	}
	comm_memset(priv_mem, 0, sizeof(struct nim_s3501_private));
	dev->priv = (void *) priv_mem;
	// DiSEqC state init
	dev->diseqc_info.diseqc_type = 0;
	dev->diseqc_info.diseqc_port = 0;
	dev->diseqc_info.diseqc_k22 = 0;

	if ((ptrQPSK_Tuner->config_data.QPSK_Config & M3501_POLAR_REVERT) == M3501_POLAR_REVERT) //bit4: polarity revert.
	{
		dev->diseqc_info.diseqc_polar = LNB_POL_V;
	}
	else //default usage, not revert.
	{
		dev->diseqc_info.diseqc_polar = LNB_POL_H;
	}
	dev->diseqc_typex = 0;
	dev->diseqc_portx = 0;

	/* Function point init */
	dev->base_addr = ptrQPSK_Tuner->ext_dm_config.i2c_base_addr;
	dev->init = nim_s3501_attach;
	dev->open = nim_s3501_open;
	dev->stop = nim_s3501_close;
	dev->do_ioctl = nim_s3501_ioctl;
	dev->set_polar = nim_s3501_set_polar;
	dev->set_12v = nim_s3501_set_12v;
//	dev->channel_change = nim_s3501_channel_change;
	dev->do_ioctl_ext = nim_s3501_ioctl_ext;
	dev->channel_search = nim_s3501_channel_search;
	dev->diseqc_operate = nim_s3501_diseqc_operate;
	dev->diseqc2X_operate = nim_s3501_diseqc2X_operate;
	dev->get_lock = nim_s3501_get_lock;
	dev->get_freq = nim_s3501_get_freq;
	dev->get_sym = nim_s3501_get_symbol_rate;
	dev->get_FEC = nim_s3501_get_code_rate;
	dev->get_AGC = nim_s3501_get_AGC;
	dev->get_SNR = nim_s3501_get_SNR;
//	dev->get_BER = nim_s3501_get_PER;
	dev->get_BER = nim_s3501_get_BER;
	dev->get_fft_result = nim_s3501_get_fft_result;
	dev->get_ver_infor = NULL;  //nim_s3501_get_ver_infor;
	/* tuner configuration function */
	priv_mem->nim_Tuner_Init = ptrQPSK_Tuner->nim_Tuner_Init;
	priv_mem->nim_Tuner_Control = ptrQPSK_Tuner->nim_Tuner_Control;
	priv_mem->nim_Tuner_Status = ptrQPSK_Tuner->nim_Tuner_Status;
	priv_mem->i2c_type_id = ptrQPSK_Tuner->tuner_config.i2c_type_id;

	priv_mem->Tuner_Config_Data.QPSK_Config = ptrQPSK_Tuner->config_data.QPSK_Config;
	priv_mem->ext_dm_config.i2c_type_id = ptrQPSK_Tuner->ext_dm_config.i2c_type_id;
	priv_mem->ext_dm_config.i2c_base_addr = ptrQPSK_Tuner->ext_dm_config.i2c_base_addr;

	priv_mem->ul_status.m_enable_dvbs2_hbcd_mode = 0;
	priv_mem->ul_status.m_dvbs2_hbcd_enable_value = 0x7f;
	priv_mem->ul_status.nim_s3501_sema = OSAL_INVALID_ID;
	priv_mem->ul_status.s3501_autoscan_stop_flag = 0;
	priv_mem->ul_status.s3501_chanscan_stop_flag = 0;
	priv_mem->ul_status.old_ber = 0;
	priv_mem->ul_status.old_per = 0;
	priv_mem->ul_status.m_hw_timeout_thr = 0;
	priv_mem->ul_status.old_ldpc_ite_num = 0;
	priv_mem->ul_status.c_RS = 0;
	priv_mem->ul_status.phase_err_check_status = 0;
	priv_mem->ul_status.s3501d_lock_status = NIM_LOCK_STUS_NORMAL;
	priv_mem->ul_status.m_s3501_type = 0x00;
	priv_mem->ul_status.m_setting_freq = 123;
	priv_mem->ul_status.m_Err_Cnts = 0x00;
	priv_mem->tsk_status.m_lock_flag = NIM_LOCK_STUS_NORMAL;
//	priv_mem->tsk_status.m_task_id = 0x00;
	priv_mem->t_Param.t_aver_snr = -1;
	priv_mem->t_Param.t_last_iter = -1;
	priv_mem->t_Param.t_last_snr = -1;
	priv_mem->t_Param.t_snr_state = 0;
	priv_mem->t_Param.t_snr_thre1 = 256;
	priv_mem->t_Param.t_snr_thre2 = 256;
	priv_mem->t_Param.t_snr_thre3 = 256;
	priv_mem->t_Param.t_dynamic_power_en = 0;
	priv_mem->t_Param.t_phase_noise_detected = 0;
	priv_mem->t_Param.t_reg_setting_switch = 0x0f;
	priv_mem->t_Param.t_i2c_err_flag = 0x00;
	priv_mem->flag_id = OSAL_INVALID_ID;

	/* Add this device to queue */
	if (dev_register(dev) != SUCCESS)
	{
		dprintk(1, "Error: Register nim device error!\n");
		comm_free(priv_mem);
		dev_free(dev);
		return ERR_NO_DEV;
	}
	nim_dev_num++;
	priv_mem->ul_status.nim_s3501_sema = NIM_MUTEX_CREATE(1);

	if (nim_s3501_i2c_open(dev))
	{
		return S3501_ERR_I2C_NO_ACK;
	}
	// Initial the QPSK Tuner
	if (priv_mem->nim_Tuner_Init != NULL)
	{
		dprintk(20, "Initialize the Tuner\n");
		if (((struct nim_s3501_private *) dev->priv)->nim_Tuner_Init(&priv_mem->tuner_id, &(ptrQPSK_Tuner->tuner_config)) != SUCCESS)
		{
			dprintk(1, "Error: Tuner init failed!\n");

			if (nim_s3501_i2c_close(dev))
			{
				return S3501_ERR_I2C_NO_ACK;
			}
			return ERR_NO_DEV;
		}
	}
	if (nim_s3501_i2c_close(dev))
	{
		return S3501_ERR_I2C_NO_ACK;
	}
	nim_s3501_ext_lnb_config(dev, ptrQPSK_Tuner);
	nim_s3501_get_type(dev);
	if (priv_mem->ul_status.m_s3501_type == NIM_CHIP_ID_M3501A  // Chip 3501A
	&&  (priv_mem->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_2BIT_MODE)  // TS 2bit mode
	{
//		for M3606 + M3501A full nim ssi-2bit patch, auto change to 1bit mode.
		priv_mem->Tuner_Config_Data.QPSK_Config &= 0x3f; // set to TS 1 bit mode
		//libc_printk("M3501A SSI 2bit mode, auto change to 1bit mode\n");
	}
	ptrQPSK_Tuner->device_type = priv_mem->ul_status.m_s3501_type;
	return SUCCESS;
}
#endif //lwj remove

int nim_s3501_hw_check(struct nim_device *dev)
{
	u8 data = 0;

	nim_reg_read(dev, RA3_CHIP_ID + 0x01, &data, 1);
	if (data != 0x35)
	{
		return ERR_FAILED;
	}
	else
	{
		dprintk(20, "Ali M3501 demodulator found at I2C address 0x%02x\n", dev->base_addr);
		return SUCCESS;
	}
}

#if 0 //lwj remove this
static void nim_s3501_set_demap_noise(struct nim_device *dev)
{
	u8 data, noise_index;
	u16 est_noise;
	//int i;
	// for (i=0;i<32;i++){
	// activate noise
	nim_reg_read(dev, RD0_DEMAP_NOISE_RPT + 2, &data, 1);
	data &= 0xfc;
	nim_reg_write(dev, RD0_DEMAP_NOISE_RPT + 2, &data, 1);
	// set noise_index
	noise_index = 0x0c; // 8psk,3/5.
	nim_reg_write(dev, RD0_DEMAP_NOISE_RPT + 1, &noise_index, 1);
	// set noise
	est_noise = DEMAP_NOISE[noise_index];
	data = est_noise & 0xff;
	nim_reg_write(dev, RD0_DEMAP_NOISE_RPT, &data, 1);
	data = (est_noise >> 8) & 0x3f;
	data |= 0xc0;
	nim_reg_write(dev, RD0_DEMAP_NOISE_RPT + 1, &data, 1);
	// }
}
#endif

static int nim_s3501_demod_ctrl(struct nim_device *dev, u8 c_Value)
{
	u8 data = c_Value;

	nim_reg_write(dev, R00_CTRL, &data, 1);
	return SUCCESS;
}

static int nim_s3501_hbcd_timeout(struct nim_device *dev, u8 s_Case)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	u8 data;

	switch (s_Case)
	{
		case NIM_OPTR_CHL_CHANGE:
		case NIM_OPTR_IOCTL:
		{
			if (priv->t_Param.t_reg_setting_switch & NIM_SWITCH_HBCD)
			{
				if (priv->ul_status.m_enable_dvbs2_hbcd_mode)
				{
					data = priv->ul_status.m_dvbs2_hbcd_enable_value;
				}
				else
				{
					data = 0x00;
				}
				nim_reg_write(dev, R47_HBCD_TIMEOUT, &data, 1);
				priv->t_Param.t_reg_setting_switch &= ~NIM_SWITCH_HBCD;
			}
			break;
		}
		case NIM_OPTR_SOFT_SEARCH:
		{
			if (!(priv->t_Param.t_reg_setting_switch & NIM_SWITCH_HBCD))
			{
				data = 0x00;
				nim_reg_write(dev, R47_HBCD_TIMEOUT, &data, 1);
				priv->t_Param.t_reg_setting_switch |= NIM_SWITCH_HBCD;
			}
			break;
		}
		case NIM_OPTR_HW_OPEN:
		{
			nim_reg_read(dev, R47_HBCD_TIMEOUT, &(priv->ul_status.m_dvbs2_hbcd_enable_value), 1);
			break;
		}
	}
	return SUCCESS;
}

static int nim_s3501_interrupt_mask_clean(struct nim_device *dev)
{
	u8 data = 0x00;

	nim_reg_write(dev, R02_IERR, &data, 1);
	data = 0xff;
	nim_reg_write(dev, R03_IMASK, &data, 1);
	return SUCCESS;
}

void nim_s3501_after_reset_set_param(struct nim_device *dev)
{
	u8 data;
	int i;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	nim_s3501_demod_ctrl(dev, NIM_DEMOD_CTRL_0X91);
	nim_s3501_interrupt_mask_clean(dev);
	data = 0x50; ////0x2B; let AGC lock, try to modify tuner's gain
	nim_reg_write(dev, R0A_AGC1_LCK_CMD, &data, 1);
	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		// disable dummy function
		data = 0x00;
		nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
		//----------------------------------------------------
		// Set demap beta
		for (i = 0; i < 32; i++)
		{
			data = i;
			nim_reg_write(dev, R9C_DEMAP_BETA + 0x02, &data, 1);
			data = MAP_BETA_ACTIVE_BUF[i];
			nim_reg_write(dev, R9C_DEMAP_BETA, &data, 1);
			data = 0x04;
			nim_reg_write(dev, R9C_DEMAP_BETA + 0x03, &data, 1);
			data = MAP_BETA_BUF[i];
			nim_reg_write(dev, R9C_DEMAP_BETA, &data, 1);
			data = 0x03;
			nim_reg_write(dev, R9C_DEMAP_BETA + 0x03, &data, 1);
			// printk (" !!-----set map par %d OK\n",i);
		}
	}
	return ;
}
#define OSAL_INVALID_ID 0

static int nim_s3501_reg_get_work_mode(struct nim_device *dev, u8 *work_mode)
{
	u8 data;

//	dprintk(100, "%s >\n", __func__);
	nim_reg_read(dev, R68_WORK_MODE, &data, 1);
//	dprintk(70, "%s: data = 0x%02x\n", __func__, data);
	*work_mode = data & 0x03;
//	dprintk(70, "%s: work_mode = %d\n", __func__, *work_mode);
	return SUCCESS;

	//  Work Mode
	//      0x0:    DVB-S
	//      0x1:    DVB-S2
	//      0x2:    DVB-S2 HBC
}

static int nim_s3501_reg_get_iqswap_flag(struct nim_device *dev, u8 *iqswap_flag)
{
	u8 data;

	//dprintk(100, "%s >\n", __func__);
	nim_reg_read(dev, R6C_RPT_SYM_RATE + 0x02, &data, 1);
	*iqswap_flag = ((data >> 4) & 0x01);
	return SUCCESS;
}

static int nim_s3501_set_acq_workmode(struct nim_device *dev, u8 s_Case)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	u8 data, work_mode = 0x00;

	switch (s_Case)
	{
		case NIM_OPTR_CHL_CHANGE0:
		case NIM_OPTR_DYNAMIC_POW0:
		{
			data = 0x73;
			nim_reg_write(dev, R5B_ACQ_WORK_MODE, &data, 1);
			break;
		}
		case NIM_OPTR_CHL_CHANGE:
		{
//			dprintk(70, "%s: NIM_OPTR_CHL_CHANGE\n", __func__);
			nim_s3501_reg_get_work_mode(dev, &work_mode);
			if (work_mode != M3501_DVBS2_MODE)  // not in DVB-S2 mode, key word: power_ctrl
			{
//				dprintk(70, "%s: DVB-S\n", __func__);
				if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
				{
					priv->ul_status.phase_err_check_status = 1000;
				}
				// slow down S2 clock
				data = 0x77;
				nim_reg_write(dev, R5B_ACQ_WORK_MODE, &data, 1);
			}
			break;
		}
		case NIM_OPTR_SOFT_SEARCH:
		case NIM_OPTR_DYNAMIC_POW:
		{
			data = 0x77;
			nim_reg_write(dev, R5B_ACQ_WORK_MODE, &data, 1);
			break;
		}
		case NIM_OPTR_HW_OPEN:
		{
			if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
			{
				// enable ADC
				data = 0x00;
				nim_reg_write(dev, RA0_RXADC_REG + 0x02, &data, 1);
				// enable S2 clock
				data = 0x73;
				nim_reg_write(dev, R5B_ACQ_WORK_MODE, &data, 1);
			}
			break;
		}
		case NIM_OPTR_HW_CLOSE:
		{
			if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
			{
				// close ADC
				data = 0x07;
				nim_reg_write(dev, RA0_RXADC_REG + 0x02, &data, 1);
				// close S2 clock
				data = 0x3f;
			}
			else
			{
				data = 0x7f;
			}
			nim_reg_write(dev, R5B_ACQ_WORK_MODE, &data, 1);
			break;
		}
	}
	return SUCCESS;
}

static int nim_s3501_set_hw_timeout(struct nim_device *dev, u8 time_thr)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	// AGC1 setting
	if (time_thr != priv->ul_status.m_hw_timeout_thr)
	{
		nim_reg_write(dev, R05_TIMEOUT_TRH, &time_thr, 1);
		priv->ul_status.m_hw_timeout_thr = time_thr;
	}
	return SUCCESS;
}

static int nim_s3501_adc_setting(struct nim_device *dev)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	u8 data, ver_data;

	// ADC setting
	if (priv->Tuner_Config_Data.QPSK_Config & M3501_IQ_AD_SWAP)
	{
		data = 0x4a;
	}
	else
	{
		data = 0xa;
	}
	if (priv->Tuner_Config_Data.QPSK_Config & M3501_EXT_ADC)
	{
		data |= 0x80;
	}
	else
	{
		data &= 0x7f;
	}
	nim_reg_write(dev, R01_ADC, &data, 1);

	nim_reg_read(dev, R01_ADC, &ver_data, 1);
	if (data != ver_data)
	{
		dprintk(1, "%s: Register 0x08 write error\n", __func__);
		return ERR_FAILED;
	}
	return SUCCESS;
}

static int nim_s3501_hw_init(struct nim_device *dev)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	u8 data = 0xc0;

	nim_reg_write(dev, RA7_I2C_ENHANCE, &data, 1);

	nim_reg_read(dev, RCC_STRAP_PIN_CLOCK, &data, 1);
	data = data & 0xfb;  // open IIC_TIME_THR_BPS, IIC will enter IDLE if long time waiting.
	nim_reg_write(dev, RCC_STRAP_PIN_CLOCK, &data, 1);

	data = 0x10;
	nim_reg_write(dev, RB3_PIN_SHARE_CTRL, &data, 1);
	// set TR lock symbol number thr, k unit.
	data = 0x1f; // setting for soft search function
	nim_reg_write(dev, R1B_TR_TIMEOUT_BAND, &data, 1);

	// Carcy change PL time out, for low symbol rate. 2008-03-12
	data = 0x84;
	nim_reg_write(dev, R28_PL_TIMEOUT_BND + 0x01, &data, 1);

	// Set Hardware time out
	//data = 0xff;
	//nim_reg_write(dev,R05_TIMEOUT_TRH, &data, 1);
	nim_s3501_set_hw_timeout(dev, 0xff);

	//----eq demod setting
	// Open EQ controll for QPSK and 8PSK
	data = 0x04;  // set EQ control
	nim_reg_write(dev, R21_BEQ_CTRL, &data, 1);

	data = 0x24;  // set EQ mask mode, mask EQ for 1/4,1/3,2/5 code rate
	nim_reg_write(dev, R25_BEQ_MASK, &data, 1);

	//-----set analog pad driving and first TS gate open.
	if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_1BIT_MODE)
	{
		data = 0x08;
	}
	else
	{
		data = 0x00;
	}
	nim_reg_write(dev, RAF_TSOUT_PAD, &data, 1);

	data = 0x00;
	nim_reg_write(dev, RB1_TSOUT_SMT, &data, 1);

	// Carcy add for 16APSK
	data = 0x6c;
	nim_reg_write(dev, R2A_PL_BND_CTRL + 0x02, &data, 1);
	//----eq demod setting end

	nim_s3501_adc_setting(dev);

	// config EQ
#ifdef DFE_EQ  //question
	data = 0xf0;
	nim_reg_write(dev, RD7_EQ_REG, &data, 1);
	nim_reg_read(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
	data = data | 0x01;
	nim_reg_write(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
#else
	data = 0x00;
	nim_reg_write(dev, RD7_EQ_REG, &data, 1);
	nim_reg_read(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
	data = data | 0x01;
	nim_reg_write(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
#endif
	return SUCCESS;
}

/*****************************************************************************
 * int nim_s3501_open(struct nim_device *dev)
 * Description: S3501 open
 *
 * Arguments:
 * Parameter1: struct nim_device *dev
 *
 * Return Value: int
 *****************************************************************************/
static int nim_s3501_open(struct nim_device *dev)
{
	int ret;
	//struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	dprintk(100, "%s >\n", __func__);
	nim_s3501_set_acq_workmode(dev, NIM_OPTR_HW_OPEN);
	ret = nim_s3501_hw_check(dev);
	if (ret != SUCCESS)
		return ret;
	ret = nim_s3501_hw_init(dev);
	nim_s3501_after_reset_set_param(dev);
	nim_s3501_hbcd_timeout(dev, NIM_OPTR_HW_OPEN);
//	nim_s3501_task_init(dev);//question
#ifdef CHANNEL_CHANGE_ASYNC
	if (priv->flag_id == OSAL_INVALID_ID)
		priv->flag_id = NIM_FLAG_CREATE(0);
#endif
	//printk(" Leave fuction nim_s3501_open\n");
	return SUCCESS;
}

/*****************************************************************************
* int nim_s3501_close(struct nim_device *dev)
* Description: S3501 close
*
* Arguments:
* Parameter1: struct nim_device *dev
*
* Return Value: int
*****************************************************************************/
static int nim_s3501_close(struct nim_device *dev)
{
	//u8 data,ver_data;
	//struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;////
	nim_s3501_demod_ctrl(dev, NIM_DEMOD_CTRL_0X90);
	nim_s3501_set_acq_workmode(dev, NIM_OPTR_HW_CLOSE);
	//NIM_MUTEX_DELETE(priv->ul_status.nim_s3501_sema);//question
#ifdef CHANNEL_CHANGE_ASYNC
	NIM_FLAG_DEL(priv->flag_id);
#endif
	return SUCCESS;
}

/*****************************************************************************
* int nim_s3501_set_polar(struct nim_device *dev, u8 polar)
* Description: S3501 set polarization
*
* Arguments:
* Parameter1: struct nim_device *dev
* Parameter2: u8 pol
*
* Return Value: void
*****************************************************************************/
#if 1 //lwj remove question
//#define REVERT_POLAR
#define NIM_PORLAR_HORIZONTAL 0x00
#define NIM_PORLAR_VERTICAL 0x01

u32 nim_s3501_set_polar(struct nim_device *dev, u8 polar)
{
	u8 data = 0;
	struct nim_s3501_private *priv;
	priv = (struct nim_s3501_private *) dev->priv;
	nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
	if ((priv->Tuner_Config_Data.QPSK_Config & M3501_POLAR_REVERT) == 0x00) //not exist H/V polarity revert.
	{
		if (NIM_PORLAR_HORIZONTAL == polar)
		{
			data &= 0xBF;
			//printk("nim_s3501_set_polar CR5C is 00\n");
		}
		else if (NIM_PORLAR_VERTICAL == polar)
		{
			data |= 0x40;
			//printk("nim_s3501_set_polar CR5C is 40\n");
		}
		else
		{
			//printk("nim_s3501_set_polar error\n");
			return 1;
		}
	}
	else//exist H/V polarity revert.
	{
		if (NIM_PORLAR_HORIZONTAL == polar)
		{
			data |= 0x40;
			//printk("nim_s3501_set_polar CR5C is 40\n");
		}
		else if (NIM_PORLAR_VERTICAL == polar)
		{
			data &= 0xBF;
			//printk("nim_s3501_set_polar CR5C is 00\n");
		}
		else
		{
			//printk("nim_s3501_set_polar error\n");
			return 1;
		}
	}
	nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);
	return SUCCESS;
}

/*****************************************************************************
* int nim_s3501_set_12v(struct nim_device *dev, u8 flag)
* Description: S3501 set LNB votage 12V enable or not
*
* Arguments:
* Parameter1: struct nim_device *dev
* Parameter2: u8 flag
*
* Return Value: SUCCESS
*****************************************************************************/
int nim_s3501_set_12v(struct nim_device *dev, u8 flag)
{
	return SUCCESS;
}
#endif

static int nim_s3501_set_ssi_clk(struct nim_device *dev, u8 bit_rate)
{
	u8 data;
	if (bit_rate >= ssi_clock_tab[0])
	{
		// use 135M SSI debug
		nim_reg_read(dev, RDF_TS_OUT_DVBS2, &data, 1);
		data = (data & 0x0f) | 0xf0;
		nim_reg_write(dev, RDF_TS_OUT_DVBS2, &data, 1);
		nim_reg_read(dev, RCE_TS_FMT_CLK, &data, 1);
		data = (data & 0xf3) | 0x08; //135M SSI CLK.
		nim_reg_write(dev, RCE_TS_FMT_CLK, &data, 1);
	}
	else if (bit_rate > ssi_clock_tab[1])
	{
		// use 98M SSI debug
		data = 0x1d; //
		nim_reg_write(dev, RDF_TS_OUT_DVBS2, &data, 1);
	}
	else if (bit_rate > ssi_clock_tab[2])
	{
		// use 90M SSI debug
		data = 0xfd;
		nim_reg_write(dev, RDF_TS_OUT_DVBS2, &data, 1);
		nim_reg_read(dev, RCE_TS_FMT_CLK, &data, 1);
		data = (data & 0xf3) | 0x04; //90M SSI CLK.
		nim_reg_write(dev, RCE_TS_FMT_CLK, &data, 1);
	}
	else if (bit_rate > ssi_clock_tab[3])
	{
		// use 83M SSI debug
		data = 0x0d; //
		nim_reg_write(dev, RDF_TS_OUT_DVBS2, &data, 1);
	}
	else if (bit_rate > ssi_clock_tab[4])
	{
		// use 77M SSI debug
		data = 0x2d; //
		nim_reg_write(dev, RDF_TS_OUT_DVBS2, &data, 1);
	}
	else if (bit_rate > ssi_clock_tab[5])
	{
		// use 72M SSI debug
		data = 0xfd;
		nim_reg_write(dev, RDF_TS_OUT_DVBS2, &data, 1);
		nim_reg_read(dev, RCE_TS_FMT_CLK, &data, 1);
		data = (data & 0xf3) | 0x0c; //72M SSI CLK.
		nim_reg_write(dev, RCE_TS_FMT_CLK, &data, 1);
	}
	else if (bit_rate > ssi_clock_tab[6])
	{
		// use 67M SSI debug
		data = 0x3d;
		nim_reg_write(dev, RDF_TS_OUT_DVBS2, &data, 1);
	}
	else if (bit_rate > ssi_clock_tab[7])
	{
		// use 60M SSI debug
		data = 0x4d;
		nim_reg_write(dev, RDF_TS_OUT_DVBS2, &data, 1);
	}
	else
	{
		// use 54M SSI debug
		data = 0xfd;
		nim_reg_write(dev, RDF_TS_OUT_DVBS2, &data, 1);
		nim_reg_read(dev, RCE_TS_FMT_CLK, &data, 1);
		data = (data & 0xf3) | 0x00; //54M SSI CLK.
		nim_reg_write(dev, RCE_TS_FMT_CLK, &data, 1);
	}
	////printk("clock setting is: %02x \n", data);
	return SUCCESS;
}

static int nim_set_ts_rs(struct nim_device *dev, u32 Rs)
{
	u8 data;
	u32 temp;
	Rs = ((Rs << 10) + 500) / 1000;
	temp = (Rs * 204 + 94) / 188; // rs *2 *204/188
	temp = temp + 1024; // add 1M symbol rate for range.
	data = temp & 0xff;
	nim_reg_write(dev, RDD_TS_OUT_DVBS, &data, 1);
	data = (temp >> 8) & 0xff;
	nim_reg_write(dev, RDD_TS_OUT_DVBS + 0x01, &data, 1);
	nim_reg_read(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
	data = data | 0x10;
	nim_reg_write(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
	////printk("xxx set rs from register file........... \n");
	return SUCCESS;
}

// TS_SSI_SPI_SEL = reg_crd8[0];
// TS_OUT_MODE = reg_crd8[2:1];
// MOCLK_PHASE_OUT = reg_crd8[3];
// TS_SPI_SEL = reg_crd8[4];
// SSI_DEBUG = reg_crd8[5];
// MOCLK_PHASE_SEL = reg_crd8[6];

#ifdef NIM_3501_FUNC_EXT
static int nim_invert_moclk_phase(struct nim_device *dev)
{
	u8 data;
	nim_reg_read(dev, RD8_TS_OUT_SETTING, &data, 1);
	data = data ^ 0x08;
	nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
	return SUCCESS;
}

static int nim_open_ssi_debug(struct nim_device *dev)
{
	u8 data;
	nim_reg_read(dev, RD8_TS_OUT_SETTING, &data, 1);
	data = data | 0x20;
	nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
	return SUCCESS;
}

static int nim_close_ssi_debug(struct nim_device *dev)
{
	u8 data;
	nim_reg_read(dev, RD8_TS_OUT_SETTING, &data, 1);
	data = data & 0xdf;
	nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
	return SUCCESS;
}

static int nim_open_ts_dummy(struct nim_device *dev)
{
	u8 data;
	nim_reg_read(dev, RD8_TS_OUT_SETTING, &data, 1);
	data = data | 0x10;
	nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
	return SUCCESS;
}
#endif

static int nim_close_ts_dummy(struct nim_device *dev)
{
	u8 data;
	//struct nim_s3501_private* priv = (struct nim_s3501_private *)dev->priv;
	data = 0x00;
	nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
	data = 0x40;
	nim_reg_write(dev, RAD_TSOUT_SYMB + 0x01, &data, 1);
	nim_reg_read(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
	data = data & 0x1f;
	nim_reg_write(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
	data = 0xff; // ssi tx debug close
	nim_reg_write(dev, RDF_TS_OUT_DVBS2, &data, 1);
	return SUCCESS;
}

#ifdef NIM_3501_FUNC_EXT
static int nim_s3501_set_dmy_format(struct nim_device *dev)
{
	//SW config dummy packet format.
	u8 data, tmp;
	//enable ECO_TS_DYM_HEAD_EN = bf[7];
	nim_reg_read(dev, RBF_S2_FEC_DBG, &data, 1);
	data = data | 0x80;
	nim_reg_write(dev, RBF_S2_FEC_DBG, &data, 1);
	//enable ECO_TS_DYM_HEAD_0 = {a2[7:3],a1[7:5]} = 47;
	nim_reg_read(dev, RA0_RXADC_REG + 0x02, &data, 1);
	tmp = TS_DYM_HEAD0 & 0xf8;
	data = (data & 0x07) | tmp;
	nim_reg_write(dev, RA0_RXADC_REG + 0x02, &data, 1);
	nim_reg_read(dev, RA0_RXADC_REG + 0x01, &data, 1);
	tmp = TS_DYM_HEAD0 & 0x07;
	tmp = (tmp << 5) & 0xff;
	data = (data & 0x1f) | tmp;
	nim_reg_write(dev, RA0_RXADC_REG + 0x01, &data, 1);
	//enable ECO_TS_DYM_HEAD_1 = {ce[7:4],a9[1:0],a8[1:0]} = 1f;
	nim_reg_read(dev, RCE_TS_FMT_CLK, &data, 1);
	tmp = TS_DYM_HEAD1 & 0xf0;
	data = (data & 0x0f) | tmp;
	nim_reg_write(dev, RCE_TS_FMT_CLK, &data, 1);
	nim_reg_read(dev, RA9_M180_CLK_DCHAN, &data, 1);
	tmp = TS_DYM_HEAD1 & 0x0c;
	tmp = (tmp >> 2) & 0xff;
	data = (data & 0xfc) | tmp;
	nim_reg_write(dev, RA9_M180_CLK_DCHAN, &data, 1);
	nim_reg_read(dev, RA8_M90_CLK_DCHAN, &data, 1);
	tmp = TS_DYM_HEAD1 & 0x03;
	data = (data & 0xfc) | tmp;
	nim_reg_write(dev, RA8_M90_CLK_DCHAN, &data, 1);
	//enable ECO_TS_DYM_HEAD_2 = {c1[7:2],cc[7:6]} = ff;
	nim_reg_read(dev, RC1_DVBS2_FEC_LDPC, &data, 1);
	tmp = TS_DYM_HEAD2 & 0xfc;
	data = (data & 0x03) | tmp;
	nim_reg_write(dev, RC1_DVBS2_FEC_LDPC, &data, 1);
	nim_reg_read(dev, RCC_STRAP_PIN_CLOCK, &data, 1);
	tmp = TS_DYM_HEAD2 & 0x03;
	tmp = (tmp << 6) & 0xff;
	data = (data & 0x3f) | tmp;
	nim_reg_write(dev, RCC_STRAP_PIN_CLOCK, &data, 1);
	//enable ECO_TS_DYM_HEAD_3 = {cf[7:1],c0[1]} = 10;
	nim_reg_read(dev, RCE_TS_FMT_CLK + 0x01, &data, 1);
	tmp = TS_DYM_HEAD3 & 0xfe;
	data = (data & 0x01) | tmp;
	nim_reg_write(dev, RCE_TS_FMT_CLK + 0x01, &data, 1);
	nim_reg_read(dev, RC0_BIST_LDPC_REG, &data, 1);
	tmp = TS_DYM_HEAD3 & 0x01;
	tmp = (tmp << 1) & 0xff;
	data = (data & 0xfd) | tmp;
	nim_reg_write(dev, RC0_BIST_LDPC_REG, &data, 1);
	//enable ECO_TS_DYM_HEAD_4 = {9f[7:4],92[7:4]} = 00;
	nim_reg_read(dev, R9C_DEMAP_BETA + 0x03, &data, 1);
	tmp = TS_DYM_HEAD4 & 0xf0;
	data = (data & 0x0f) | tmp;
	nim_reg_write(dev, R9C_DEMAP_BETA + 0x03, &data, 1);
	nim_reg_read(dev, R90_DISEQC_CLK_RATIO + 0x02, &data, 1);
	tmp = TS_DYM_HEAD4 & 0x0f;
	tmp = (tmp << 4) & 0xff;
	data = (data & 0x0f) | tmp;
	nim_reg_write(dev, R90_DISEQC_CLK_RATIO + 0x02, &data, 1);
	return SUCCESS;
}
#endif

static int nim_s3501_get_bit_rate(struct nim_device *dev, u8 work_mode, u8 map_type, u8 code_rate, u32 Rs, u8 *bit_rate)
{
	u8 data;
	u32 temp;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	if (work_mode != M3501_DVBS2_MODE) // DVBS mode
	{
		if (code_rate == 0)
		{
			temp = (Rs * 2 * 1 + 1000) / 2000;
		}
		else if (code_rate == 1)
		{
			temp = (Rs * 2 * 2 + 1500) / 3000;
		}
		else if (code_rate == 2)
		{
			temp = (Rs * 2 * 3 + 2000) / 4000;
		}
		else if (code_rate == 3)
		{
			temp = (Rs * 2 * 5 + 3000) / 6000;
		}
		else
		{
			temp = (Rs * 2 * 7 + 4000) / 8000;
		}
		if (temp > 254)
		{
			data = 255;
		}
		else
		{
			data = temp + 1;  // add 1M for margin
		}
		*bit_rate = data;
//		dprintk(70,"%s: dvbs bit_rate is %d Mbps (DVB-S)\n", __func__, data);
//		dprintk(100,"%s <)\n", __func__);
		return SUCCESS;
	}
	else  // DVBS2 mode
	{
		if (code_rate == 0)
		{
			temp = (Rs * 1 + 2000) / 4000;
		}
		else if (code_rate == 1)
		{
			temp = (Rs * 1 + 1500) / 3000;
		}
		else if (code_rate == 2)
		{
			temp = (Rs * 2 + 2500) / 5000;
		}
		else if (code_rate == 3)
		{
			temp = (Rs * 1 + 1000) / 2000;
		}
		else if (code_rate == 4)
		{
			temp = (Rs * 3 + 2500) / 5000;
		}
		else if (code_rate == 5)
		{
			temp = (Rs * 2 + 1500) / 3000;
		}
		else if (code_rate == 6)
		{
			temp = (Rs * 3 + 2000) / 4000;
		}
		else if (code_rate == 7)
		{
			temp = (Rs * 4 + 2500) / 5000;
		}
		else if (code_rate == 8)
		{
			temp = (Rs * 5 + 3000) / 6000;
		}
		else if (code_rate == 9)
		{
			temp = (Rs * 8 + 4500) / 9000;
		}
		else
		{
			temp = (Rs * 9 + 5000) / 10000;
		}
		if (map_type == 2)
		{
			temp = temp * 2;
		}
		else if (map_type == 3)
		{
			temp = temp * 3;
		}
		else if (map_type == 4)
		{
			temp = temp * 4;
		}
		else
		{
			dprintk(1, "%s Map type error: %02x \n", __func__, map_type);
		}
		if ((priv->Tuner_Config_Data.QPSK_Config & M3501_USE_188_MODE) != M3501_USE_188_MODE)
		{
			temp = (temp * 204 + 94) / 188;
		}

		if (temp > 200)
		{
			data = 200;
		}
		else
		{
			data = temp;
		}
		dprintk(70, "%s: Code rate is: %02x\n", __func__, code_rate);
		dprintk(70, "%s: Map type is: %02x\n", __func__, map_type);

		data += 1; // Add 1M
		*bit_rate = data;
		dprintk(100,"%s < (dvbs bit_rate is %d Mbps (DVB-S2))\n", __func__, data);
		return SUCCESS;
	}
}

static int nim_s3501_open_ci_plus(struct nim_device *dev, u8 *ci_plus_flag)
{
	u8 data;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	// For CI plus test.
	if (priv->ul_status.m_s3501_sub_type == NIM_CHIP_SUB_ID_S3501D)
	{
		data = 0x06;
	}
	else
	{
		data = 0x02;  // symbol period from reg, 2 cycle
	}
	nim_reg_write(dev, RAD_TSOUT_SYMB, &data, 1);
	dprintk(70, "%s: open ci plus enable REG_ad = %02x \n", __func__, data);

	nim_reg_read(dev, RAD_TSOUT_SYMB + 0x01, &data, 1);
	data = data | 0x80;  // enable symbol period from reg
	nim_reg_write(dev, RAD_TSOUT_SYMB + 0x01, &data, 1);

	nim_reg_read(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
	data = data | 0xe0;
	nim_reg_write(dev, RDC_EQ_DBG_TS_CFG, &data, 1);

	nim_reg_read(dev, RDF_TS_OUT_DVBS2, &data, 1);
	data = (data & 0xfc) | 0x01;
	nim_reg_write(dev, RDF_TS_OUT_DVBS2, &data, 1);

	*ci_plus_flag = 1;
	return SUCCESS;
}

static int nim_s3501_set_ts_mode(struct nim_device *dev, u8 work_mode, u8 map_type, u8 code_rate, u32 Rs,
				   u8 channel_change_flag)
{
	u8 data;
	u8 bit_rate;

	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	u8 ci_plus_flag = 0;

	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		nim_reg_read(dev, RC0_BIST_LDPC_REG, &data, 1);
		data = data | 0xc0;  //1//enable 1bit mode
		nim_reg_write(dev, RC0_BIST_LDPC_REG, &data, 1);
		bit_rate = 0xff;
		nim_s3501_get_bit_rate(dev, work_mode, map_type, code_rate, Rs, &bit_rate);

		/* TS output config 8-bit mode begin */
		if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_8BIT_MODE) //8bit mode
		{
//			dprintk(20, "%s: xxx S3501D output SPI mode\n", _func__);
//			dprintk(20, "%s: xxx work mode is %02x \n", __func__, work_mode);
//			dprintk(50, "%s: xxx bit rate kkk is %d \n", __func__, bit_rate);
			if (work_mode != M3501_DVBS2_MODE)  // DVBS mode
			{
				if (((bit_rate >= 98) || (bit_rate <= ssi_clock_tab[8])) && channel_change_flag)
				{
					// USE SPI dummy, no SSI debug mode.
					if ((priv->Tuner_Config_Data.QPSK_Config & M3501_USE_188_MODE) == M3501_USE_188_MODE)
					{
						data = 0x10;
					}
					else
					{
						data = 0x16;
					}
					nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
//					dprintk(50, "%s: DVBS USE SPI normal dummy, no SSI debug mode\n", __func__);
				}
				else
				{
					// USE SSI debug mode with dummy enable.
					nim_reg_read(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
					data = data | 0x20;
					nim_reg_write(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
					nim_s3501_set_ssi_clk(dev, bit_rate);
					if ((priv->Tuner_Config_Data.QPSK_Config & M3501_USE_188_MODE) == M3501_USE_188_MODE)
					{
						data = 0x21;
					}
					else
					{
						data = 0x27;
					}
					nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
//					dprintk(50, %s: "DVBS  USE SSI debug mode \n", __func__);
				}
			}
			else  // DVBS2 mode
			{
				//   If >98, M3602 need configure DMX clock phase:
				//    0xb8012000  ==  0x......AB -> 0x......AA
				if (((bit_rate <= 98) || (bit_rate >= ssi_clock_tab[8])) && channel_change_flag)
				{
					// USE normal SPI
					//#ifdef USE_188_MODE
					if ((priv->Tuner_Config_Data.QPSK_Config & M3501_USE_188_MODE) == M3501_USE_188_MODE)
					{
						data = 0x10;
					}
					else
					{
						data = 0x16;  // SPI dummy, no SSI debug mode.
					}
					nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
//					dprintk(50, "%s: DVBS2 Enter normal SPI mode, not using SSI debug.\n", __func__);
				}
				else
				{
					// use ssi debug to output 8-bit spi mode
					nim_s3501_open_ci_plus(dev, &ci_plus_flag);
					nim_s3501_set_ssi_clk(dev, bit_rate);
					//#ifdef USE_188_MODE
					if ((priv->Tuner_Config_Data.QPSK_Config & M3501_USE_188_MODE) == M3501_USE_188_MODE)
					{
						data = 0x21;
					}
					else
					{
						data = 0x27;  // enable SSI debug
					}
					nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
//					dprintk(50, "%s: DVB-S2 Enter SSI debug..\n", __func__);
				}
			}
		}
		/* TS output config 8-bit mode end */

		/* TS output config 1-bit mode begin */
		else if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_1BIT_MODE)  // SSI mode
		{
			//  SSI mode
//			dprintk(20, "%s: xxx S3501D output SSI mode\n",__func__);
//			dprintk(20, "%s: xxx work mode is %02x \n", __func__, work_mode);
			if (priv->ul_status.m_s3501_sub_type == NIM_CHIP_SUB_ID_S3501D)
			{
				data = 0x60;
				nim_reg_write(dev, RAD_TSOUT_SYMB + 0x01, &data, 1);
			}
			nim_s3501_set_ssi_clk(dev, bit_rate);
			if (work_mode != M3501_DVBS2_MODE)
			{
				// DVB-S mode
				nim_reg_read(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
				data = data | 0x20;  // add for ssi_clk change point
				nim_reg_write(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
//				dprintk(50, "%s: DVBS USE SSI mode\n", __func__);
			}
			else
			{
				//  DVB-S2 mode
				///  If >98, M3602 need configure DMX clock phase:
				//    0xb8012000  ==  0x......AB -> 0x......AA
				if (bit_rate < 50)
				{
					// do not need to open ci plus function in low bit rate.
//					dprintk(50, "%s: Low bit rate, Close CI plus\n", __func__);
				}
				else
				{
					nim_s3501_open_ci_plus(dev, &ci_plus_flag);
				}
				dprintk(70, "%s: DVB-S2: USE SSI mode\n", __func__);
			}

			if ((priv->Tuner_Config_Data.QPSK_Config & M3501_USE_188_MODE) == M3501_USE_188_MODE)
			{
				data = 0x01;
			}
			else
			{
				data = 0x07;  // enable SSI debug
			}
			nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
		}
		/* TS output config 1-bit mode end */

		/* TS output config 2-bit mode begin */
		else if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_2BIT_MODE)  // TS 2-bit mode
		{
			// TS 2-bit mode
			// S3501D's 2-bit is very different from M3501B
			if (priv->ul_status.m_s3501_sub_type == NIM_CHIP_SUB_ID_S3501D)
			{
				if ((priv->Tuner_Config_Data.QPSK_Config & M3501_USE_188_MODE) == M3501_USE_188_MODE)
				{
					data = 0x10;
				}
				else
				{
					data = 0x16;
				}
				nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
				data = 0x00;
				nim_reg_write(dev, RAD_TSOUT_SYMB + 0x01, &data, 1);
//				dprintk(70, "%s: S3501D Enter 2-bit Mode\n", __func__);
			}
			else
			{
				// TS 2bit mode
				// ECO_SSI_2B_EN = cr9f[3]
				nim_reg_read(dev, R9C_DEMAP_BETA + 0x03, &data, 1);
				data = data | 0x08;
				nim_reg_write(dev, R9C_DEMAP_BETA + 0x03, &data, 1);

				// ECO_SSI_SEL_2B = crc0[3]
				nim_reg_read(dev, RC0_BIST_LDPC_REG, &data, 1);
				data = data | 0x08; //for 2bit mode
				nim_reg_write(dev, RC0_BIST_LDPC_REG, &data, 1);

				nim_s3501_set_ssi_clk(dev, bit_rate);

				if (work_mode != M3501_DVBS2_MODE)// DVBS mode
				{
					nim_reg_read(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
					data = data | 0x20;  // add for ssi_clk change point
					nim_reg_write(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
					dprintk(70, "%s Enter DVB-S 2-bit mode\n", __func__);
				}
				else  // DVBS2 mode
				{
					// For CI plus test.
					if (bit_rate < 50)
					{
						//do not need to open ci plus function in low bit rate.
//						dprintk(50, "%s: Low bit rate, Close CI plus\n", __func__);
					}
					else
					{
						nim_s3501_open_ci_plus(dev, &ci_plus_flag);
					}
//					dprintk(70, "%s: Enter DVB-S2 2-bit mode\n", __func__);
				}
				// no matter what bit_rate, all use ssi_debug mode
				if ((priv->Tuner_Config_Data.QPSK_Config & M3501_USE_188_MODE) == M3501_USE_188_MODE)
				{
					data = 0x29; //0x21 question
				}
				else
				{
					data = 0x2F;  //0x27 question // enable SSI debug
				}
				nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
			}  // end of M3501B 2-bit
		}
		/* TS output config 2-bit mode end */

		/* TS output config 4-bit mode begin */
		else if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_4BIT_MODE)  // 4-bit mode
		{
			// TS 4-bit mode
			// S3501D's 4-bit is very different from M3501B
			if (priv->ul_status.m_s3501_sub_type == NIM_CHIP_SUB_ID_S3501D)
			{
				if ((priv->Tuner_Config_Data.QPSK_Config & M3501_USE_188_MODE) == M3501_USE_188_MODE)
				{
					data = 0x10;
				}
				else
				{
					data = 0x16; //moclk interv for 4bit.
				}
				nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
				data = 0x20;
				nim_reg_write(dev, RAD_TSOUT_SYMB + 0x01, &data, 1);
			}
			else
			{
				// TS 4-bit mode
				// ECO_SSI_2B_EN = cr9f[3]
				nim_reg_read(dev, R9C_DEMAP_BETA + 0x03, &data, 1);
				data = data | 0x08;
				nim_reg_write(dev, R9C_DEMAP_BETA + 0x03, &data, 1);

				// ECO_SSI_SEL_2B = crc0[3]
				nim_reg_read(dev, RC0_BIST_LDPC_REG, &data, 1);
				data = data & 0xf7;  // for 4-bit mode
				nim_reg_write(dev, RC0_BIST_LDPC_REG, &data, 1);

				nim_s3501_set_ssi_clk(dev, bit_rate);

				if (work_mode != M3501_DVBS2_MODE)  // DVB-S mode
				{
					nim_reg_read(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
					data = data | 0x20;  // add for ssi_clk change point
					nim_reg_write(dev, RDC_EQ_DBG_TS_CFG, &data, 1);
//					dprintk(70, "%s: M3501B Enter DVB-S 4-bit mode\n", __func__);
				}
				else  // DVBS2 mode
				{
					// For CI plus test.
					if (bit_rate < 50)
					{
						// don't need open ci plus function in low bit rate.
//						dprintk(50, "%s Low bit rate, Close CI plus\n", __func__);
					}
					else
					{
						nim_s3501_open_ci_plus(dev, &ci_plus_flag);
					}
//					dprintk(70, "%s: M3501B Enter DVB-S2 4-bit mode\n", __func__);
				}
				// no matter bit_rate all use ssi_debug mode
				if ((priv->Tuner_Config_Data.QPSK_Config & M3501_USE_188_MODE) == M3501_USE_188_MODE)
				{
					data = 0x21;
				}
				else
				{
					data = 0x27;  // enable SSI debug
				}
				nim_reg_write(dev, RD8_TS_OUT_SETTING, &data, 1);
			}  // end if M3501B  4bit
		}
		if (ci_plus_flag)
		{
			// RST fsm
			data = 0x91;
			nim_reg_write(dev, R00_CTRL, &data, 1);
			data = 0x51;
			nim_reg_write(dev, R00_CTRL, &data, 1);
		}
		/* TS output config 4-bit mode end */
		//end M3501B config.
	}
	else
	{
		// M3501A
		if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_1BIT_MODE)
		{
			data = 0x60;
		}
		else if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_2BIT_MODE)
		{
			data = 0x00;
		}
		else if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_4BIT_MODE)
		{
			data = 0x20;
		}
		else if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_8BIT_MODE)
		{
			data = 0x40;
		}
		else
		{
			data = 0x40;
		}
		nim_reg_write(dev, RAD_TSOUT_SYMB + 0x01, &data, 1);
	}
	return SUCCESS;
}

#if defined(NIM_3501_FUNC_EXT)
static int nim_s3501_get_phase_error(struct nim_device *dev, int *phase_error)
{
	u8 rdata = 0;
	u8 data = 0;

	nim_reg_read(dev, RC0_BIST_LDPC_REG + 4, &data, 1);
	if (data & 0x80)
	{
		data &= 0x7f;
		nim_reg_write(dev, RC0_BIST_LDPC_REG + 4, &data, 1);
	}
	nim_reg_read(dev, RC0_BIST_LDPC_REG, &data, 1);
	if ((data & 0x04) == 0)
	{
		nim_reg_read(dev, RC0_BIST_LDPC_REG + 5, &rdata, 1);
		data |= 0x04;
		nim_reg_write(dev, RC0_BIST_LDPC_REG, &data, 1);
		if (rdata & 0x80)
		{
			*phase_error = rdata - 256;
		}
		else
		{
			*phase_error = rdata;  // phase error is signed!!!
		}
		dprintk(20, "%s phase error is %d\n", __func__, *phase_error);
		return SUCCESS;  // means phase error measured
	}
	else
	{
		*phase_error = 0;
		return ERR_FAILUE;  // means that phase error is not ready
	}
}
#endif  // NIM_3501_FUNC_EXT

/*****************************************************************************
 * int nim_s3501_get_LDPC(struct nim_device *dev, u32 *RsUbc)
 * Get LDPC average iteration number
 *
 * Arguments:
 *  Parameter1: struct nim_device *dev
 *  Parameter2: u32 *RsUbc
 *
 * Return Value: int
 *****************************************************************************/
static int nim_s3501_get_LDPC(struct nim_device *dev, u32 *RsUbc)
{
	u8 data;
	//u8   ite_num;
	//struct nim_s3501_private* priv = (struct nim_s3501_private  *)dev->priv;

	// read single LDPC iteration number
	nim_reg_read(dev, RAA_S2_FEC_ITER, &data, 1);
	*RsUbc = (u32) data;
	return SUCCESS;
}

// for SNR use
static u8 nim_s3501_get_SNR_index(struct nim_device *dev)
{
	s16 lpf_out16;
	s16 agc2_ref5;
	int snr_indx = 0;
	u8  data[2];
	u16 tdata[2];

	// CR45
	nim_reg_read(dev, R45_CR_LCK_DETECT, &data[0], 1);
//	dprintk(70, "%s: CR20 is 0x%x\n", __func__, data[0]);
	// CR46
	nim_reg_read(dev, R45_CR_LCK_DETECT + 0x01, &data[1], 1);
//	dprintk(70, "%s: CR21 is 0x%x\n", __func__, data[1]);

	tdata[0] = (u16) data[0];
	tdata[1] = (u16)(data[1] << 8);
	lpf_out16 = (s16)(tdata[0] + tdata[1]);
	lpf_out16 /= (16 * 2);

	// CR07
	nim_reg_read(dev, R07_AGC1_CTRL, &data[0], 1);
//	dprintk(70, "%s: CR13 is 0x%x\n", __func__, data[0]);
	agc2_ref5 = (s16)(data[0] & 0x1F);

	snr_indx = (lpf_out16 * agc2_ref5 / 21) - 27;  //27~0

	if (snr_indx < 0)
	{
		snr_indx = 0;
	}
	else if (snr_indx > 176)
	{
		snr_indx = 176;
	}
//	dprintk(50, "%s: snr_indx is 0x%x\n", __func__, snr_indx);
//	dprintk(70, "%s: snr_tab[%d] is 0x%x\n", __func__, snr_indx, snr_tab[snr_indx] );
	return snr_indx;
}

/*****************************************************************************
 * int nim_s3501_get_lock(struct nim_device *dev, u8 *lock)
 *
 *  Read FEC lock status
 *
 * Arguments:
 *  Parameter1: struct nim_device *dev
 *  Parameter2: BOOL *fec_lock
 *
 * Return Value: int
 *
 ****************************************************************************/
static int nim_s3501_get_lock(struct nim_device *dev, u8 *lock)
{
	u8 data;
	u8 h8psk_lock;

	//comm_delay(150);
	udelay(150);

	nim_reg_read(dev, R04_STATUS, &data, 1);
	if ((data & 0x80) == 0x80)
	{
		h8psk_lock = 1;
	}
	else
	{
		h8psk_lock = 0;
	}

	if (h8psk_lock & ((data & 0x2f) == 0x2f))
	{
		*lock = 1;
	}
	else if ((data & 0x3f) == 0x3f)
	{
		*lock = 1;
	}
	else
	{
		*lock = 0;
	}
//	comm_delay(150);
	udelay(150);
	return YW_NO_ERROR;
}

/*****************************************************************************
* int nim_s3501_get_BER(struct nim_device *dev, u32 *RsUbc)
* Get bit error ratio
*
* Arguments:
*  Parameter1: struct nim_device *dev
*  Parameter2: u32 *RsUbc
*
* Return Value: int
*****************************************************************************/
static int nim_s3501_get_BER(struct nim_device *dev, u32 *RsUbc)
{
	u8  data;
	u8  ber_data[3];
	u32 u32ber_data[3];
	u32 uber_data;

	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	// CR78
	nim_reg_read(dev, R76_BIT_ERR + 0x02, &data, 1);
	if (0x00 == (0x80 & data))
	{
//		dprintk(70, "%s: CR78= %x\n", __func__, data);
		// CR76
		nim_reg_read(dev, R76_BIT_ERR, &ber_data[0], 1);
		u32ber_data[0] = (u32) ber_data[0];
		// CR77
		nim_reg_read(dev, R76_BIT_ERR + 0x01, &ber_data[1], 1);
		u32ber_data[1] = (u32) ber_data[1];
		u32ber_data[1] <<= 8;
		// CR78
		nim_reg_read(dev, R76_BIT_ERR + 0x02, &ber_data[2], 1);
		u32ber_data[2] = (u32) ber_data[2];
		u32ber_data[2] <<= 16;

		uber_data = u32ber_data[2] + u32ber_data[1] + u32ber_data[0];

		uber_data *= 100;
		uber_data /= 1632;
		*RsUbc = uber_data;
		priv->ul_status.old_ber = uber_data;
		// CR78
		data = 0x80;
		nim_reg_write(dev, R76_BIT_ERR + 0x02, &data, 1);
	}
	else
	{
		*RsUbc = priv->ul_status.old_ber;
	}
	// Carcy: add for control power
	return SUCCESS;
}

#if defined(NIM_3501_FUNC_EXT)
/*****************************************************************************
* int nim_s3501_s2_get_BER(struct nim_device *dev, u32 *RsUbc)
* Get bit error ratio
*
* Arguments:
*  Parameter1: struct nim_device *dev
*  Parameter2: u32 *RsUbc
*
* Return Value: int 0
*****************************************************************************/
static int nim_s3501_s2_get_BER(struct nim_device *dev, u32 *RsUbc)
{
	u8 data, rdata;
	u8 ber_data[3];
	u32 u32ber_data[3];
	u32 uber_data;
//	struct nim_s3501_private* priv = (struct nim_s3501_private  *)dev->priv;

	// read single LDPC BER information
	nim_reg_read(dev, RD3_BER_REG, &rdata, 1);
	data = rdata & 0x7b;
	nim_reg_write(dev, RD3_BER_REG, &data, 1);

	nim_reg_read(dev, RD3_BER_REG + 0x01, &ber_data[0], 1);
	u32ber_data[0] = (u32) ber_data[0];
	nim_reg_read(dev, RD3_BER_REG + 0x01, &ber_data[1], 1);
	u32ber_data[1] = (u32) ber_data[1];
	u32ber_data[1] <<= 8;
	uber_data = u32ber_data[1] + u32ber_data[0];
	*RsUbc = uber_data;
	return SUCCESS;
}
#endif  // NIM_3501_FUNC_EXT

/*****************************************************************************
 * int nim_s3501_dynamic_power(struct nim_device *dev, u32 *RsUbc)
 * Get bit error ratio
 *
 * Arguments:
 * Parameter1:
 * Key word: power_ctrl
 * Return Value: int
 *****************************************************************************/
static int nim_s3501_dynamic_power(struct nim_device *dev, u8 snr)
{
	u8  coderate;
	u32 ber;
	static u32 ber_sum = 0;  // store the continuous ber
	static u32 last_ber_sum = 0;
	static u32 cur_ber_sum = 0;
	static u32 ber_thres = 0x180;
	static u8 cur_max_iter = 50; // static variable can not auto reset at channel change???
	static u8 snr_bak = 0;
	static u8 last_max_iter = 50;
	static int cnt3 = 0;

	if (cnt3 >= 3)
	{
//		dprintk(70, "%s: ber_sum = %6x\n", __func__, ber_sum);
		last_ber_sum = cur_ber_sum;
		cur_ber_sum = ber_sum;
		cnt3 = 0;
		ber_sum = 0;
	}
	nim_s3501_get_BER(dev, &ber);
	nim_s3501_reg_get_code_rate(dev, &coderate);
	ber_sum += ber;
	cnt3 ++;
	if (coderate < 0x04)  // 1/4 rate
	{
		ber_thres = 0x120;
	}
	else
	{
		ber_thres = 0x70;
	}
//	dprintk(70, "%s: ber = %6x, snr = %2x, cur_max_iter = %2d, snr_bak = %2x\n", __func__, ber, snr, cur_max_iter, snr_bak);

	if (cur_max_iter == 50)
	{
		if (ber_sum >= ber_thres)
		{
			if (snr > snr_bak)
			{
				snr_bak = snr;
			}
			cur_max_iter -= 15;
		}
	}
	else if (cur_max_iter < 50)
	{
		if (((cur_ber_sum + 0x80) < last_ber_sum) || (snr > (snr_bak + 2)))
		{
			cur_max_iter += 15;
			snr_bak = 0;
			cnt3 = 0;
			ber_sum = 0;
			last_ber_sum = 0;
			cur_ber_sum = 0;
		}
		else if (ber_sum > 3 * ber_thres)
		{
			cur_max_iter -= 15;
			if ((coderate < 0x04) && (cur_max_iter < 35))
			{
				cur_max_iter = 35;
			}
			else if (cur_max_iter < 20)
			{
				cur_max_iter = 20;
			}
		}
	}

	if (cur_max_iter != last_max_iter)
	{
//		dprintk(70, "%s: change cur_max_iter to %d\n", __func__, cur_max_iter);
		nim_reg_write(dev, R57_LDPC_CTRL, &cur_max_iter, 1);
		last_max_iter = cur_max_iter;
	}
	return SUCCESS;
}

/*****************************************************************************
 * int nim_s3501_get_SNR(struct nim_device *dev, u8 *snr)
 *
 * This function returns an approximate estimation of the SNR from the NIM
 *  The Eb No is calculated using the SNR from the NIM, using the formula:
 *     Eb ~     13312- M_SNR_H
 *     -- =    ----------------  dB.
 *     NO           683
 *
 * Arguments:
 *  Parameter1: struct nim_device *dev
 *  Parameter2: u16 *RsUbc
 *
 * Return Value: int
 *****************************************************************************/
static int nim_s3501_get_SNR(struct nim_device *dev, u16 *snr)
{
	u8 lock; //coderate,
	u8 data;
	u32 tdata, iter_num;
	int i, total_iter, sum;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	if (priv->Tuner_Config_Data.QPSK_Config & M3501_SIGNAL_DISPLAY_LIN)
	{
#define TDATA_SUM_LIN 6
		nim_reg_read(dev, R04_STATUS, &data, 1);
		if (data & 0x08)
		{
			// CR lock
			sum = 0;
			total_iter = 0;
			for (i = 0; i < TDATA_SUM_LIN; i++)
			{
				nim_reg_read(dev, R45_CR_LCK_DETECT + 0x01, &data, 1);
				tdata = data << 8;
				nim_reg_read(dev, R45_CR_LCK_DETECT, &data, 1);
				tdata |= data;

				if (tdata & 0x8000)
				{
					tdata = 0x10000 - tdata;
				}
				nim_s3501_get_LDPC(dev, &iter_num);
				total_iter += iter_num;
				tdata >>= 5;
				sum += tdata;
			}
			sum /= TDATA_SUM_LIN;
			total_iter /= TDATA_SUM_LIN;
			sum *= 3;
			sum /= 5;
			if (sum > 255)
			{
				sum = 255;
			}
			if (priv->t_Param.t_last_snr == -1)
			{
				*snr = sum;
				priv->t_Param.t_last_snr = *snr;
			}
			else
			{
				if (total_iter == priv->t_Param.t_last_iter)
				{
					*snr = priv->t_Param.t_last_snr;
					if (sum > priv->t_Param.t_last_snr)
					{
						priv->t_Param.t_last_snr++;
					}
					else if (sum < priv->t_Param.t_last_snr)
					{
						priv->t_Param.t_last_snr--;
					}
				}
				else if ((total_iter > priv->t_Param.t_last_iter) && (sum < priv->t_Param.t_last_snr))
				{
					*snr = sum;
					priv->t_Param.t_last_snr = sum;
				}
				else if ((total_iter < priv->t_Param.t_last_iter) && (sum > priv->t_Param.t_last_snr))
				{
					*snr = sum;
					priv->t_Param.t_last_snr = sum;
				}
				else
				{
					*snr = priv->t_Param.t_last_snr;
				}
			}
			priv->t_Param.t_last_iter = total_iter;
		}
		else
		{
			// CR unlock
			*snr = 0;
			priv->t_Param.t_last_snr = -1;
			priv->t_Param.t_last_iter = -1;
		}
	}
	else
	{
#define TDATA_START 7
#define TDATA_SUM 4
		tdata = 0;
		for (i = 0; i < TDATA_SUM; i++)
		{
			data = snr_tab[nim_s3501_get_SNR_index(dev)];
			tdata += data;
		}
		tdata /= TDATA_SUM;
		// CR04
		nim_reg_read(dev, R04_STATUS, &data, 1);
		if ((0x3F == data) | (0x7f == data) | (0xaf == data))
		{
		}
		else
		{
#if (defined(SIGNAL_INDICATOR) && (SIGNAL_INDICATOR == 3))
			// To have linear signal indicator change between lock and unlock per Hicway's request
			// --Michael Xie 2005/8/17
			tdata = 0;
			if (0x01 == (data & 0x01))  // agc1
			{
				tdata += 2;
			}
			if (0x02 == (data & 0x02))  // agc2
			{
				tdata += 10;
			}
			if (0x04 == (data & 0x04))  // cr
			{
				tdata += 16;
			}
			if (0x08 == (data & 0x08))  // tr
			{
				tdata += 16;
			}
			if (0x10 == (data & 0x10))  // frame sync
			{
				tdata += 20;
			}
			if (0x20 == (data & 0x20))
			{
				tdata += 26;
			}
			if (0x40 == (data & 0x40))
			{
				tdata += 26;
			}
			if (0x80 == (data & 0x80))
			{
				tdata += 26;
			}
#else
			//tdata /= 2;
			tdata = TDATA_START;
			if (0x01 == (data & 0x01))
			{
				tdata += 1;
			}
			if (0x02 == (data & 0x02))
			{
				tdata += 3;
			}
			if (0x04 == (data & 0x04))
			{
				tdata += 3;
			}
			if (0x08 == (data & 0x08))
			{
				tdata += 2;
			}
			if (0x10 == (data & 0x10))
			{
				tdata += 2;
			}
			if (0x20 == (data & 0x20))
			{
				tdata += 2;
			}
			if (0x40 == (data & 0x40))
			{
				tdata += 2;
			}
			if (0x80 == (data & 0x80))
			{
				tdata += 2;
			}
#endif
		}
		*snr = tdata / 2;
	}

	if (priv->t_Param.t_aver_snr == -1)
	{
		priv->t_Param.t_aver_snr = (*snr);
	}
	else
	{
		priv->t_Param.t_aver_snr += (((*snr) - priv->t_Param.t_aver_snr) >> 2);
	}

	// no phase noise and actived after channel lock
	// if moniter_phase_noise default is 0, this function may before set_phase_noise!!!!!
	// the threshods are affected by coderate
	nim_s3501_get_lock(dev, &lock);
	if ((lock) && (priv->t_Param.t_phase_noise_detected == 0) && (priv->t_Param.phase_noise_detect_finish == 1))
	{
		if (priv->t_Param.t_snr_state == 0)
		{
			priv->t_Param.t_snr_state = 1;
			data = 0x52;
			nim_reg_write(dev, R33_CR_CTRL + 0x04, &data, 1);
			data = 0xba;
			nim_reg_write(dev, RB5_CR_PRS_TRA, &data, 1);
//			dprintk(50, "%s: initial snr state = 1, reg37 = 0x52, regb5 = 0xba\n", __func__);

		}
		if ((priv->t_Param.t_snr_state == 1) && (priv->t_Param.t_aver_snr > priv->t_Param.t_snr_thre1))
		{
			data = 0x4e;
			nim_reg_write(dev, R33_CR_CTRL + 0x04, &data, 1);
			data = 0xaa;
			nim_reg_write(dev, RB5_CR_PRS_TRA, &data, 1);
//			dprintk(50, "%s: snr state = 2, reg37 = 0x4e, regb5 = 0xaa;\n", __func__);
			priv->t_Param.t_snr_state = 2;
		}
		else if (priv->t_Param.t_snr_state == 2)
		{
			if (priv->t_Param.t_aver_snr > priv->t_Param.t_snr_thre2)
			{
				data = 0x42;
				nim_reg_write(dev, R33_CR_CTRL + 0x04, &data, 1);
				data = 0x9a;
				nim_reg_write(dev, RB5_CR_PRS_TRA, &data, 1);
//				dprintk(50, "%s: snr state = 3, reg37 = 0x42, regb5 = 0x9a;\n", __func__);
				priv->t_Param.t_snr_state = 3;
			}
			else if (priv->t_Param.t_aver_snr < (priv->t_Param.t_snr_thre1 - 5))
			{
				data = 0x52;
				nim_reg_write(dev, R33_CR_CTRL + 0x04, &data, 1);
				data = 0xba;
				nim_reg_write(dev, RB5_CR_PRS_TRA, &data, 1);
//				dprintk(50, "%s: snr state = 1, reg37 = 0x52, regb5 = 0xba;\n", __func__);

				priv->t_Param.t_snr_state = 1;
			}
		}
		else if (priv->t_Param.t_snr_state == 3)
		{
			if (priv->t_Param.t_aver_snr > priv->t_Param.t_snr_thre3)
			{
				data = 0x3e;
				nim_reg_write(dev, R33_CR_CTRL + 0x04, &data, 1);
				data = 0x8a;
				nim_reg_write(dev, RB5_CR_PRS_TRA, &data, 1);
//				dprintk(50, "%s: snr state = 4, reg37 = 0x3e, regb5 = 0x8a;\n", __func__);

				priv->t_Param.t_snr_state = 4;
			}
			else if (priv->t_Param.t_aver_snr < (priv->t_Param.t_snr_thre2 - 5))
			{
				data = 0x4e;
				nim_reg_write(dev, R33_CR_CTRL + 0x04, &data, 1);
				data = 0xaa;
				nim_reg_write(dev, RB5_CR_PRS_TRA, &data, 1);
//				dprintk(50, "%s: snr state = 2, reg37 = 0x4e, regb5 = 0xaa;\n", __func__);
				priv->t_Param.t_snr_state = 2;
			}
		}
		else if (priv->t_Param.t_snr_state == 4)
		{
			if (priv->t_Param.t_aver_snr < (priv->t_Param.t_snr_thre3 - 5))
			{
				data = 0x42;
				nim_reg_write(dev, R33_CR_CTRL + 0x04, &data, 1);
				data = 0x9a;
				nim_reg_write(dev, RB5_CR_PRS_TRA, &data, 1);
//				dprintk(50, "%s: snr state = 3, reg37 = 0x42, regb5 = 0x9a;\n", __func__);
				priv->t_Param.t_snr_state = 3;
			}
		}
	}
	if (priv->t_Param.t_dynamic_power_en)
	{
		nim_s3501_dynamic_power(dev, (*snr));
	}
	return SUCCESS;
}

static int nim_s3501_get_new_BER(struct nim_device *dev, u32 *ber)
{
	u8 data;
	u32 t_count, myber;

	myber = 0;
	for (t_count = 0; t_count < 200; t_count++)
	{
		nim_reg_read(dev, R76_BIT_ERR + 0x02, &data, 1);
		if ((data & 0x80) == 0)
		{
			myber = data & 0x7f;
			nim_reg_read(dev, R76_BIT_ERR + 0x01, &data, 1);
			myber <<= 8;
			myber += data;
			nim_reg_read(dev, R76_BIT_ERR, &data, 1);
			myber <<= 8;
			myber += data;
			break;
		}
	}
	*ber = myber;
	return SUCCESS;
}

static int nim_s3501_get_new_PER(struct nim_device *dev, u32 *per)
{
	u8  data;
	u32 t_count, myper;

	myper = 0;
	for (t_count = 0; t_count < 200; t_count++)
	{
		nim_reg_read(dev, R79_PKT_ERR + 0x01, &data, 1);
		if ((data & 0x80) == 0)
		{
			myper = data & 0x7f;
			nim_reg_read(dev, R79_PKT_ERR, &data, 1);
			myper <<= 8;
			myper += data;
			break;
		}
	}
	*per = myper;
//	dprintk(70, "%s myPER cost %d time, per = %d\n", __func__, t_count, myper);
	return SUCCESS;
}

static int nim_s3501_set_phase_noise(struct nim_device *dev)
{
	u32 debug_time, debug_time_thre, i;
	u8  data, verdata, sdat;
	u16 snr;
	u32 ber, per;
	u32 min_per, max_per;
	u32 per_buf[4] = { 0, 0, 0, 0 };  // 0~3: per_ba~per_8a
	u32 buf_index = 0;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	dprintk(100, "%s >\n", __func__);
	sdat = 0xba;
	nim_reg_write(dev, RB5_CR_PRS_TRA, &sdat, 1);
	debug_time = 0;
	debug_time_thre = 4;
	sdat = 0xba;

	data = 0x00;
	nim_reg_write(dev, R74_PKT_STA_NUM, &data, 1);
	data = 0x10;  // M3501B needs at least 0x10, or else read 0 only.
	nim_reg_write(dev, R74_PKT_STA_NUM + 0x01, &data, 1);
	for (debug_time = 0; debug_time < debug_time_thre; debug_time++)
	{
		nim_reg_read(dev, R02_IERR, &data, 1);
		if ((data & 0x02) == 0)
		{
			break;
		}
		data = 0x80;
		nim_reg_write(dev, R76_BIT_ERR + 0x02, &data, 1);
		nim_reg_read(dev, R76_BIT_ERR + 0x02, &verdata, 1);
		if (verdata != data)
		{
			dprintk(1, "%s: RESET BER ERROR!\n", __func__);
		}
		data = 0x80;
		nim_reg_write(dev, R79_PKT_ERR + 0x01, &data, 1);
		nim_reg_read(dev, R79_PKT_ERR + 0x01, &verdata, 1);
		if (verdata != data)
		{
			dprintk(1, "%s: RESET PER ERROR!\n", __func__);
		}
		//comm_delay(100);
		udelay(100);

		nim_s3501_get_SNR(dev, &snr);
		nim_s3501_get_new_BER(dev, &ber);
		nim_s3501_get_new_PER(dev, &per);
		dprintk(50, "%s: snr = %d, ber = %d, per = %d\n", __func__, snr, ber, per);

//		if (per > per_buf[buf_index])
		{
			per_buf[buf_index] = per;
		}
		sdat -= 0x10;
		buf_index++;
		nim_reg_write(dev, RB5_CR_PRS_TRA, &sdat, 1);
		if ((per_buf[buf_index - 2] == 0) && (buf_index >= 2))
		{
			break;
		}
	}
	min_per = 0;
	max_per = 0;
	for (i = 0; i < buf_index; i++)
	{
		dprintk(50, "%s: per_buf[%d] = 0x%x\n", __func__, i, per_buf[i]);
		per_buf[i] >>= 4;
		if (per_buf[i] < per_buf[min_per])
		{
			min_per = i;
		}
		if (per_buf[i] > per_buf[max_per])
		{
			max_per = i;
			if (i > 1)
			{
				break;  // if phase noise exists, wider BW should have miner per. At threshod, wider BW may cause unlock, per=0 when unlock
			}
		}
	}
	if (min_per <= max_per)
	{
		priv->t_Param.t_phase_noise_detected = 0;
		dprintk(50, "%s: No phase noise detected\n", __func__);
	}
	else
	{
		priv->t_Param.t_phase_noise_detected = 1;
		dprintk(20, "%s: Phase noise detected\n", __func__);
		data = 0x42;
		nim_reg_write(dev, R33_CR_CTRL + 0x04, &data, 1);
	}
	dprintk(150, "%s: min_per is %d, max_per is %d\n", __func__, min_per, max_per);
	if ((min_per < buf_index - 1)
	&&  (per_buf[min_per] == per_buf[min_per + 1]))
	{
		if ((per_buf[min_per] > 0) || (snr < 0x10) || (priv->t_Param.t_phase_noise_detected == 0))
		{
			sdat = 0xba - min_per * 0x10;
		}
		else
		{
			sdat = 0xaa - min_per * 0x10;
		}
		nim_reg_write(dev, RB5_CR_PRS_TRA, &sdat, 1);
	}
	else
	{
		sdat = 0xba - min_per * 0x10;
		nim_reg_write(dev, RB5_CR_PRS_TRA, &sdat, 1);
	}
	dprintk(100, "%s < (REG_b5 = 0x%x)\n", __func__, sdat);
	data = 0x10;
	nim_reg_write(dev, R74_PKT_STA_NUM, &data, 1);
	data = 0x27;
	nim_reg_write(dev, R74_PKT_STA_NUM + 0x01, &data, 1);
	return SUCCESS;
}

static int nim_s3501_reg_get_roll_off(struct nim_device *dev, u8 *roll_off)
{
	u8 data;

//	dprintk(100, "%s >\n", __func__);
	nim_reg_read(dev, R6C_RPT_SYM_RATE + 0x02, &data, 1);
	*roll_off = ((data >> 5) & 0x03);
	return SUCCESS;

	//  DVBS2 Roll off report
	//      0x0:    0.35
	//      0x1:    0.25
	//      0x2:    0.20
	//      0x3:    Reserved
}

static int nim_s3501_reg_get_map_type(struct nim_device *dev, u8 *map_type)
{
	u8 data;

//	dprintk(100, "%s >\n", __func__);
	nim_reg_read(dev, R69_RPT_CARRIER + 0x02, &data, 1);
	*map_type = ((data >> 5) & 0x07);
	return SUCCESS;

	//  Map type:
	//      0x0:    HBCD
	//      0x1:    BPSK
	//      0x2:    QPSK
	//      0x3:    8PSK
	//      0x4:    16APSK
	//      0x5:    32APSK
}

#if 1
#define sys_ms_cnt (SYS_CPU_CLOCK / 2000)
static int nim_s3501_waiting_channel_lock(/*TUNER_ScanTaskParam_T *Inst,*/struct nim_device *dev, u32 freq, u32 sym)
{
	u32 timeout, locktimes = 200;
	u32 tempFreq;
	u32 Rs;
	u8 work_mode, map_type, iqswap_flag, roll_off;
	u8 intdata;
	u8 code_rate;
	u8 data;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	u8 channel_change_flag = 1;  // bentao add for judge channel change or soft_search in set_ts_mode

	timeout = 0;

	if (sym > 40000)
	{
		locktimes = 204;
	}
	else if (sym < 2000)
	{
		locktimes = 2000;
	}
	else if (sym < 4000)
	{
		locktimes = 1604 - sym / 40;
	}
	else if (sym < 6500)
	{
		locktimes = 1004 - sym / 60;
	}
	else
	{
		locktimes = 604 - sym / 100;
	}
	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		locktimes *= 2; // lwj change *3 to *2
	}
	else
	{
		locktimes *= 2;
	}
	locktimes = locktimes / 5; //lwj add
	while (priv->ul_status.s3501_chanscan_stop_flag == 0)
	{
		timeout++;
		if (locktimes < timeout)  // hardware timeout
		{
			priv->t_Param.phase_noise_detect_finish = 1;
			nim_s3501_clear_int(dev);
			dprintk(1, "%s: timeout \n", __func__);
			priv->ul_status.s3501_chanscan_stop_flag = 0;
			if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
			{
				priv->tsk_status.m_lock_flag = NIM_LOCK_STUS_SETTING;
			}
			return ERR_FAILED;
		}
		nim_reg_read(dev, R02_IERR, &intdata, 1);
		dprintk(150, "%s: R02_IERR intdata = 0x%x\n", __func__, intdata);

		data = 0x02;
		if (0 != (intdata & data))
		{
			tempFreq = freq;
			nim_s3501_reg_get_freq(dev, &tempFreq);
			dprintk(70, "%s: Frequency is %d MHz\n", __func__, (LNB_LOACL_FREQ - tempFreq));
			nim_s3501_reg_get_symbol_rate(dev, &Rs);
			dprintk(70, "%s: Symbol rate is %d MS/s\n", __func__, Rs);
			nim_s3501_reg_get_code_rate(dev, &code_rate);
			dprintk(150, "%s: Code_rate is %d\n", __func__, code_rate);
			nim_s3501_reg_get_work_mode(dev, &work_mode);
			dprintk(150, "%s: work_mode is %d\n", __func__, work_mode);
			nim_s3501_reg_get_roll_off(dev, &roll_off);
			dprintk(100, "%s: roll_off is %d\n", __func__, roll_off);
			nim_s3501_reg_get_iqswap_flag(dev, &iqswap_flag);
			dprintk(100, "%s: iqswap_flag is %d\n", __func__, iqswap_flag);
			nim_s3501_reg_get_map_type(dev, &map_type);
			dprintk(100, "%s: map_type is %d\n", __func__, map_type);
			if ((priv->ul_status.m_enable_dvbs2_hbcd_mode == 0) && ((map_type == 0) || (map_type == 5)))
			{
				dprintk(1, "%s: Demod Error: wrong map_type is %d\n", __func__, map_type);
			}
			else
			{
				dprintk(20, "Channel tuned\n");

				if (work_mode == M3501_DVBS2_MODE)
				{
					data = 0x52;
					nim_reg_write(dev, R33_CR_CTRL + 0x04, &data, 1);
					data = 0xba;
					nim_reg_write(dev, RB5_CR_PRS_TRA, &data, 1);
				}  // amy modify for QPSK 2010-3-2

				if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
				{
					if (work_mode != M3501_DVBS2_MODE)  // not in DVB-S2 mode, key word: power_ctrl
					{
						// slow down S2 clock
						data = 0x77;
						nim_reg_write(dev, R5B_ACQ_WORK_MODE, &data, 1);
						priv->ul_status.phase_err_check_status = 1000;
					}

					priv->tsk_status.m_lock_flag = NIM_LOCK_STUS_CLEAR;
					nim_s3501_set_ts_mode(dev, work_mode, map_type, code_rate, Rs, channel_change_flag);
					nim_reg_read(dev, R9C_DEMAP_BETA + 0x02, &data, 1);
					data = data | 0x80;
					nim_reg_write(dev, R9C_DEMAP_BETA + 0x02, &data, 1);
					/*
					    //open ts
					    nim_reg_read(dev,RAF_TSOUT_PAD,&data,1);
					    data = data & 0xef;    // ts open
					    nim_reg_write(dev,RAF_TSOUT_PAD, &data, 1);
					*/
					nim_reg_read(dev, R02_IERR, &data, 1);
					data = data & 0x02;
					if (data)
					{
						//Inst->Status = TUNER_STATUS_LOCKED;
						priv->bLock = TRUE;
						priv->ul_status.s3501d_lock_status = NIM_LOCK_STUS_NORMAL;  // Carcy: modify for unstable lock
						if ((work_mode == M3501_DVBS2_MODE) && (map_type == 3))
						{
							nim_s3501_set_phase_noise(dev);
						}
					}
					else
					{
						priv->ul_status.s3501d_lock_status = NIM_LOCK_STUS_SETTING;  // Carcy
					}
				}
				else
				{
					// M3501A
					if (work_mode != M3501_DVBS2_MODE)  // not in DVB-S2 mode, key word: power_ctrl
					{
						// slow down S2 clock
						data = 0x77;
						nim_reg_write(dev, R5B_ACQ_WORK_MODE, &data, 1);
					}
					else
					{
						if (map_type == 3)
						{
							// S2, 8PSK
							nim_s3501_set_phase_noise(dev);
						}
					}
				}
				priv->t_Param.phase_noise_detect_finish = 1;

				if ((work_mode == M3501_DVBS2_MODE)
				&&  (map_type == 3)
				&&  (priv->t_Param.t_phase_noise_detected == 0))
				{
					// S2, 8PSK
					if (code_rate == 4)
					{
						// code rate 3/5
						priv->t_Param.t_snr_thre1 = 30;
						priv->t_Param.t_snr_thre2 = 45;
						priv->t_Param.t_snr_thre3 = 85;
					}
					else if ((code_rate == 5) || (code_rate == 6))
					{
						// code rate 2/3, 3/4
						priv->t_Param.t_snr_thre1 = 35;
						priv->t_Param.t_snr_thre2 = 55;
					}
					else if (code_rate == 8)
					{
						// code rate 5/6
						priv->t_Param.t_snr_thre1 = 55;
						priv->t_Param.t_snr_thre2 = 65;
					}
					else if (code_rate == 9)
					{
						// code rate 8/9
						priv->t_Param.t_snr_thre1 = 75;
					}
					else
					{
						// code rate 9/10
						priv->t_Param.t_snr_thre1 = 80;
					}
				}

				if ((work_mode == M3501_DVBS2_MODE) && (map_type <= 3))  // only S2 needs dynamic power
				{
					priv->t_Param.t_dynamic_power_en = 1;
				}
				/* Keep current frequency.*/
				priv->ul_status.m_CurFreq = tempFreq;
				nim_s3501_clear_int(dev);
				priv->ul_status.s3501_chanscan_stop_flag = 0;
				return SUCCESS;
			}
		}
		else
		{
#if 0  // lwj add
			if (Inst->ForceSearchTerm)
			{
			  return SUCCESS;
			}
#endif
			if (priv->ul_status.s3501_chanscan_stop_flag)
			{
				priv->ul_status.s3501_chanscan_stop_flag = 0;
				if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
				{
					priv->tsk_status.m_lock_flag = NIM_LOCK_STUS_SETTING;
				}
				return ERR_FAILED;
			}
			//udelay(200);
			msleep(10);  // lwj change 200us to 10ms
		}
	}
	priv->ul_status.s3501_chanscan_stop_flag = 0;
	return SUCCESS;
}
#endif

void nim_s3501_clear_int(struct nim_device *dev)
{
	u8 data;
	u8 rdata;

	//clear the int
	// CR02
	data = 0x00;
	nim_reg_write(dev, R02_IERR, &data, 1);
	nim_reg_write(dev, R04_STATUS, &data, 1);

	nim_reg_read(dev, R00_CTRL, &rdata, 1);
	data = (rdata | 0x10);
	nim_s3501_demod_ctrl(dev, data);
//	dprintk(100, "%s <\n", __func__);
}

#if !defined(SPARK7162)  // lwj remove DiSEqC operate
/*****************************************************************************
* int nim_s3501_DiSEqC_operate(struct nim_device *dev, u32 mode, u8* cmd, u8 cnt)
*
*  defines DiSEqC operations
*
* Arguments:
*  Parameter1: struct nim_device *dev
*  Parameter2: u32 mode
*  Parameter3: u8* cmd
*  Parameter4: u8 cnt
*
* Return Value: void
*****************************************************************************/
#define NIM_DISEQC_MODE_22KOFF      0   /* 22kHz off */
#define NIM_DISEQC_MODE_22KON       1   /* 22kHz on */
#define NIM_DISEQC_MODE_BURST0      2   /* Burst mode, on for 12.5mS = 0 */
#define NIM_DISEQC_MODE_BURST1      3   /* Burst mode, modulated 1:2 for 12.5mS = 1 */
#define NIM_DISEQC_MODE_BYTES       4   /* Modulated with bytes from DISEQC INSTR */
#define NIM_DISEQC_MODE_ENVELOP_ON  5   /* Envelop enable*/
#define NIM_DISEQC_MODE_ENVELOP_OFF 6   /* Envelop disable, out put 22K wave form*/
#define NIM_DISEQC_MODE_OTHERS      7   /* Undefined mode */
#define NIM_DISEQC_MODE_BYTES_EXT_STEP1     8   /*Split NIM_DISEQC_MODE_BYTES to 2 steps to improve the speed,*/
#define NIM_DISEQC_MODE_BYTES_EXT_STEP2     9   /*(30ms--->17ms) to fit some SPEC */

static int nim_s3501_diseqc_operate(struct nim_device *dev, u32 mode, u8 *cmd, u8 cnt)
{
	u8 data, temp;
	u16 timeout, timer;
	u8 i;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	dprintk(50, "%s: mode = 0x%d\n", __func__, mode);
	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		data = (CRYSTAL_FREQ * 90 / 135);
	}
	else
	{
		data = (CRYSTAL_FREQ * 99 / 135);
	}
	nim_reg_write(dev, R7C_DISEQC_CTRL + 0x14, &data, 1);
	switch (mode)
	{
		case NIM_DISEQC_MODE_22KOFF:
		case NIM_DISEQC_MODE_22KON:
		{
			nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
			data = ((data & 0xF8) | mode);
			nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);
			break;
		}
		case NIM_DISEQC_MODE_BURST0:
		{
			nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
			// tone burst 0
			temp = 0x02;
			data = ((data & 0xF8) | temp);
			nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);
			msleep(16);
			break;
		}
		case NIM_DISEQC_MODE_BURST1:
		{
			nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
			// tone bust 1
			temp = 0x03;
			data = ((data & 0xF8) | temp);
			nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);
			msleep(16);
			break;
		}
		case NIM_DISEQC_MODE_BYTES:
		{
			// DiSEqC init for mode byte every time :test
			nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
			data = ((data & 0xF8) | 0x00);
			nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);

			// write the writed data count
			nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
			temp = cnt - 1;
			data = ((data & 0xC7) | (temp << 3));
			nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);

			// write the data
			for (i = 0; i < cnt; i++)
			{
				nim_reg_write(dev, (i + 0x7E), cmd + i, 1);
			}
			// clear the interupt
			nim_reg_read(dev, R7C_DISEQC_CTRL + 0x01, &data, 1);
			data &= 0xF8;
			nim_reg_write(dev, R7C_DISEQC_CTRL + 0x01, &data, 1);

			// write the control bits
			nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
			temp = 0x04;
			data = ((data & 0xF8) | temp);
			nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);

			// waiting for the send over
			timer = 0;
			timeout = 75 + 13 * cnt;
			while (timer < timeout)
			{
				nim_reg_read(dev, R7C_DISEQC_CTRL + 0x01, &data, 1);
				// change < to != by Joey
				if (0 != (data & 0x07))
				{
					break;
				}
				msleep(10);
				timer += 10;
			}
			if (1 == (data & 0x07))
			{
				// osal_task_sleep(2000);
				return SUCCESS;
			}
			else
			{
				return ERR_FAILED;
			}
			break;
		}
		case NIM_DISEQC_MODE_BYTES_EXT_STEP1:
		{
			// DiSEqC init for mode byte every time :test
			nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
			data = ((data & 0xF8) | 0x00);
			nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);

			// write the writed data count
			nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
			temp = cnt - 1;
			data = ((data & 0xC7) | (temp << 3));
			nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);

			// write the data
			for (i = 0; i < cnt; i++)
			{
				nim_reg_write(dev, (i + 0x7E), cmd + i, 1);
			}

			// clear the interupt
			nim_reg_read(dev, R7C_DISEQC_CTRL + 0x01, &data, 1);
			data &= 0xF8;
			nim_reg_write(dev, R7C_DISEQC_CTRL + 0x01, &data, 1);
			break;
		}
		case NIM_DISEQC_MODE_BYTES_EXT_STEP2:
		{
			// write the control bits
			nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
			temp = 0x04;
			data = ((data & 0xF8) | temp);
			nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);

			// waiting for the send over
			timer = 0;
			timeout = 75 + 13 * cnt;
			while (timer < timeout)
			{
				nim_reg_read(dev, R7C_DISEQC_CTRL + 0x01, &data, 1);
				// change < to != by Joey
				if (0 != (data & 0x07))
				{
					break;
				}
				msleep(10);
				timer += 10;
			}
			if (1 == (data & 0x07))
			{
				// osal_task_sleep(2000);
				return SUCCESS;
			}
			else
			{
				return ERR_FAILED;
			}
			break;
		}
		case NIM_DISEQC_MODE_ENVELOP_ON:
		{
			nim_reg_read(dev, R24_MATCH_FILTER, &data, 1);
			data |= 0x01;
			nim_reg_write(dev, R24_MATCH_FILTER, &data, 1);
			break;
		}
		case NIM_DISEQC_MODE_ENVELOP_OFF:
		{
			nim_reg_read(dev, R24_MATCH_FILTER, &data, 1);
			data &= 0xFE;
			nim_reg_write(dev, R24_MATCH_FILTER, &data, 1);
			break;
		}
		default:
		{
			break;
		}
	}
	return SUCCESS;
}
#endif  // !defined(SPARK7162)

// lwj add begin
#if !defined(SPARK7162)
// lwj add end
/*****************************************************************************
* int nim_s3501_diseqc2X_operate(struct nim_device *dev, u32 mode, u8* cmd,
*                               u8 cnt, u8 *rt_value, u8 *rt_cnt)
*
*  defines DiSEqC 2.X operations
*
* Arguments:
*  Parameter1: struct nim_device *dev
*  Parameter2: u32 mode
*  Parameter3: u8* cmd
*  Parameter4: u8 cnt
*  Parameter5: u8 *rt_value
*  Parameter6: u8 *rt_cnt
*
* Return Value: Operation result.
*****************************************************************************/
#define DISEQC2X_ERR_NO_REPLY       0x01
#define DISEQC2X_ERR_REPLY_PARITY   0x02
#define DISEQC2X_ERR_REPLY_UNKNOWN  0x03
#define DISEQC2X_ERR_REPLY_BUF_FUL  0x04
static int nim_s3501_diseqc2X_operate(struct nim_device *dev, u32 mode, u8 *cmd, u8 cnt, u8 *rt_value, u8 *rt_cnt)
{
	int result;  //,temp1,val_flag,reg_tmp;
	u8 data, temp, rec_22k = 0;//,ret;
	u16 timeout, timer;
	u8 i;

//	cmd[0] = cmd[1] = cmd[2] = 0xe2;
	switch (mode)
	{
		case NIM_DISEQC_MODE_BYTES:
		{
			// set unconti timer
			data = 0x40;
			nim_reg_write(dev, RBA_AGC1_REPORT, &data, 1);

			// set receive timer: 0x88 for default;
			data = 0x88;
			nim_reg_write(dev, R8E_DISEQC_TIME, &data, 1);
			data = 0xff;
			nim_reg_write(dev, R8E_DISEQC_TIME + 0x01, &data, 1);

			// set clock ratio: 90MHz is 0x5a;
			data = 0x64;
			nim_reg_write(dev, R90_DISEQC_CLK_RATIO, &data, 1);
			nim_reg_read(dev, R90_DISEQC_CLK_RATIO, &data, 1);

			// set min pulse width
			nim_reg_read(dev, R90_DISEQC_CLK_RATIO + 0x01, &data, 1);
			data = ((data & 0xf0) | (0x2));
			nim_reg_write(dev, R90_DISEQC_CLK_RATIO + 0x01, &data, 1);
		}
		// write the data to send buffer
		for (i = 0; i < cnt; i++)
		{
			data = cmd[i];
			nim_reg_write(dev, (i + R7C_DISEQC_CTRL + 0x02), &data, 1);
		}

		// set DiSEqC data counter
		temp = cnt - 1;
		nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
		data = ((data & 0x47) | (temp << 3));
		nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);

		// enable DiSEqC interrupt mask event bit.
		nim_reg_read(dev, R03_IMASK, &data, 1);
		data |= 0x80;
		nim_reg_write(dev, R03_IMASK, &data, 1);

		// clear corresponding DiSEqC interrupt event bit
		nim_reg_read(dev, R02_IERR, &data, 1);
		data &= 0x7f;
		nim_reg_write(dev, R02_IERR, &data, 1);

		// write the control bits, need reply
		nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
		temp = 0x84;
		data = ((data & 0x78) | temp);
		nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);

		// waiting for the send over
		timer = 0;
		timeout = 75 + 13 * cnt + 200; //200 for reply time margin.
		data = 0;

		// check DiSEqC interrupt state
		while (timer < timeout)
		{
			nim_reg_read(dev, R02_IERR, &data, 1);
			if (0x80 == (data & 0x80)) //event happen.
			{
				break;
			}
			msleep(10);
			timer += 1;
		}

		// init value for error happens
		result = ERR_FAILED;
		rt_value[0] = DISEQC2X_ERR_NO_REPLY;
		*rt_cnt = 0;
		if (0x80 == (data & 0x80)) //event happen. //else, error occur.
		{
			nim_reg_read(dev, R7C_DISEQC_CTRL + 0x01, &data, 1);

			switch (data & 0x07)
			{
				case 1:
				{
					*rt_cnt = (u8)((data >> 4) & 0x0f);
					if (*rt_cnt > 0)
					{
						for (i = 0; i < *rt_cnt; i++)
						{
							nim_reg_read(dev, (i + R86_DISEQC_RDATA), (rt_value + i), 1);
						}
						result = SUCCESS;
					}
					break;
				}
				case 2:
				{
					rt_value[0] = DISEQC2X_ERR_NO_REPLY;
					break;
				}
				case 3:
				{
					rt_value[0] = DISEQC2X_ERR_REPLY_PARITY;
					break;
				}
				case 4:
				{
					rt_value[0] = DISEQC2X_ERR_REPLY_UNKNOWN;
					break;
				}
				case 5:
				{
					rt_value[0] = DISEQC2X_ERR_REPLY_BUF_FUL;
					break;
				}
				default:
				{
					rt_value[0] = DISEQC2X_ERR_NO_REPLY;
					break;
				}
			}
		}
		if ((rec_22k & 0x07) <= 0x01) // set 22k and polarity by original value; other-values are don't care.
		{
			nim_reg_write(dev, R7C_DISEQC_CTRL, &rec_22k, 1);
		}
		return result;

		default:  // ????
		{
			break;
		}
	}
	nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
	nim_reg_read(dev, R7C_DISEQC_CTRL + 0x01, &data, 1);
	msleep(1000);
	return SUCCESS;
}
#endif  // !defined(SPARK7162)

// not used?
#if 0
/*****************************************************************************
* int nim_s3501_get_freq(struct nim_device *dev, u32 *freq)
* Read S3501 frequence
*
* Arguments:
*  Parameter1: struct nim_device *dev
*  Parameter2: u32 *freq: frequency in kHz
*
* Return Value: int 0
*****************************************************************************/
static int nim_s3501_get_freq(struct nim_device *dev, u32 *freq)
{
	nim_s3501_reg_get_freq(dev, freq);
	return SUCCESS;
}

/*****************************************************************************
* int nim_s3501_get_symbol_rate(struct nim_device *dev, u32 *sym_rate)
* Read S3501 symbol rate
*
* Arguments:
*  Parameter1: struct nim_device *dev
*  Parameter2: u16 *sym_rate: Symbol rate in kHz
*
* Return Value: int 0
*****************************************************************************/
static int nim_s3501_get_symbol_rate(struct nim_device *dev, u32 *sym_rate)
{
	nim_s3501_reg_get_symbol_rate(dev, sym_rate);
	return SUCCESS;
}

/*****************************************************************************
* int nim_s3501_get_code_rate(struct nim_device *dev, u8* code_rate)
* Description: Read S3501 code rate
*
* Arguments:
*  Parameter1: struct nim_device *dev
*  Parameter2: u8* code_rate
*
* Return Value: int 0
*****************************************************************************/
static int nim_s3501_get_code_rate(struct nim_device *dev, u8 *code_rate)
{
	nim_s3501_reg_get_code_rate(dev, code_rate);
	return SUCCESS;
}
#endif  // 0

/*****************************************************************************
* int nim_s3501_get_AGC(struct nim_device *dev, u8 *agc)
*
*  This function will access the NIM to determine the AGC feedback value
*
* Arguments:
*  Parameter1: struct nim_device *dev
*  Parameter2: u16* agc
*
* Return Value: int
*****************************************************************************/
static int nim_s3501_get_AGC(struct nim_device *dev, u16 *agc)
{
	u8 data;
	struct nim_s3501_private *priv = (struct nim_s3501_private *)dev->priv;

	if (priv->Tuner_Config_Data.QPSK_Config & M3501_SIGNAL_DISPLAY_LIN)
	{
		nim_reg_read(dev, R04_STATUS, &data, 1);
		if (data & 0x01)
		{
			// AGC1 lock
			nim_reg_read(dev, R0A_AGC1_LCK_CMD + 0x01, &data, 1);  // read AGC1 gain
			if (data > 0x7f)
			{
				*agc = data - 0x80;
			}
			else
			{
				*agc = data + 0x80;
			}
			return SUCCESS;
		}
		else
		{
			*agc = 0;
		}
	}
	else
	{
		// CR0B
		nim_reg_read(dev, R07_AGC1_CTRL + 0x04, &data, 1);
#if 0 // ????
		data = 255 - data;

		if (0x40 <= data)
		{
			data -= 0x40;
		}
		else if ((0x20 <= data) || (0x40 > data))
		{
			data -= 0x20;
		}
		else
		{
			data -= 0;
		}
		data /= 2;
		data += 16;
#endif // 0
		*agc = (u8) data;
	}
	return SUCCESS;
}

#if defined(NIM_3501_FUNC_EXT)
static int nim_get_symbol(struct nim_device *dev)
{
	u8 data;
	u32 i;

	for (i = 0; i < 5000; i++)
	{
		data = 0xc1;
		nim_reg_write(dev, RC8_BIST_TOLERATOR, &data, 1);
		nim_reg_read(dev, RC9_CR_OUT_IO_RPT, &data, 1);
//		printk("%02x", data);
		nim_reg_read(dev, RC9_CR_OUT_IO_RPT + 0x01, &data, 1);
//		printk("%02x\n", data);
	}
	// set default value
	data = 0xc0;
	nim_reg_write(dev, RC8_BIST_TOLERATOR, &data, 1);
}
#endif  // NIM_3501_FUNC_EXT

int nim_s3501_reg_get_freq(struct nim_device *dev, u32 *freq)
{
	int freq_off;
	u8 data[3];
	u32 tdata;
	u32 temp;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	temp = 0;
//	dprintk(100, "%s >\n", __func__);
	nim_reg_read(dev, R69_RPT_CARRIER + 0x02, &data[2], 1);
	temp = data[2] & 0x01;
	temp = temp << 8;
	nim_reg_read(dev, R69_RPT_CARRIER + 0x01, &data[1], 1);
	temp = temp | (data[1] & 0xff);
	temp = temp << 8;
	nim_reg_read(dev, R69_RPT_CARRIER, &data[0], 1);
	temp = temp | (data[0] & 0xff);

	tdata = temp;

	if ((data[2] & 0x01) == 1)
	{
		freq_off = (0xffff0000 | (tdata & 0xffff));
	}
	else
	{
		freq_off = tdata & 0xffff;
	}
	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		freq_off = (freq_off * (CRYSTAL_FREQ * 90 / 135)) / 90;
	}
	else
	{
		freq_off = (freq_off * (CRYSTAL_FREQ * 99 / 135)) / 90;
	}
	freq_off /= 1024;
	*freq += freq_off;
	return SUCCESS;
}

int nim_s3501_reg_get_code_rate(struct nim_device *dev, u8 *code_rate)
{
	u8 data;

//	dprintk(100, "%s >\n", __func__);
	nim_reg_read(dev, R69_RPT_CARRIER + 0x02, &data, 1);
	*code_rate = ((data >> 1) & 0x0f);
	return SUCCESS;

	//  Code rate list
	//  for DVB-S:
	//      0x0:    1/2,
	//      0x1:    2/3,
	//      0x2:    3/4,
	//      0x3:    5/6,
	//      0x4:    6/7,
	//      0x5:    7/8
	//  For DVB-S2:
	//      0x0:    1/4,
	//      0x1:    1/3,
	//      0x2:    2/5,
	//      0x3:    1/2,
	//      0x4:    3/5,
	//      0x5:    2/3,
	//      0x6:    3/4,
	//      0x7:    4/5,
	//      0x8:    5/6,
	//      0x9:    8/9,
	//      0xa:    9/10
}

// Carcy: add ldpc_code information

int nim_s3501_reg_get_symbol_rate(struct nim_device *dev, u32 *sym_rate)
{
	u8 data[3];//, i;
	u32 temp;
	u32 symrate;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	temp = 0;
	nim_reg_read(dev, R6C_RPT_SYM_RATE + 0x02, &data[2], 1);
	temp = data[2] & 0x01;
	temp = temp << 8;
	nim_reg_read(dev, R6C_RPT_SYM_RATE + 0x01, &data[1], 1);
	temp = temp | (data[1] & 0xff);
	temp = temp << 8;
	nim_reg_read(dev, R6C_RPT_SYM_RATE, &data[0], 1);
	temp = temp | (data[0] & 0xff);
	symrate = temp;

//	dprintk(70, "symbol rate = %d Ms/s\n", symrate);
	symrate = (u32)((temp * 1000) / 2048);
	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		symrate = (symrate * (CRYSTAL_FREQ * 90 / 135)) / 90;
	}
	else
	{
		symrate = (symrate * (CRYSTAL_FREQ * 99 / 135)) / 90;
	}
	*sym_rate = symrate;
	dprintk(50, "Calculated symbol rate is %d Ms/s\n", *sym_rate);
	return SUCCESS;
}

void nim_s3501_set_CodeRate(struct nim_device *dev, u8 coderate)
{
	return;
}

void nim_s3501_set_RS(struct nim_device *dev, u32 rs)
{
	u8 data, ver_data;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	rs <<= 11;
	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		dprintk(70, "%s: Chip version: M3501B\n", __func__);
		rs = rs / (CRYSTAL_FREQ * 90 / 135) * 90;
	}
	else
	{
		rs = rs / (CRYSTAL_FREQ * 99 / 135) * 90;
	}
	rs /= 1000;
	data = 0;
	ver_data = 0;

	// CR3F
	data = (u8)(rs & 0xFF);

	nim_reg_read(dev, RA3_CHIP_ID + 0x01, &ver_data, 1);
//	dprintk(70, "%s: ver_data = 0x%x\n", __func__, ver_data);

	nim_reg_read(dev, R5F_ACQ_SYM_RATE, &ver_data, 1);
//	dprintk(70, "%s: ver_data = 0x%x\n", __func__, ver_data);

	nim_reg_write(dev, R5F_ACQ_SYM_RATE, &data, 1);
	nim_reg_read(dev, R5F_ACQ_SYM_RATE, &ver_data, 1);
	if (data != ver_data)
	{
		dprintk(1, "%s: register 0x5f write error\n", __func__);
	}
//	dprintk(70, "%s: CR3F rs = 0x%x, ver_data = 0x%x\n", __func__, data, ver_data);

	// CR40
	data = (u8)((rs & 0xFF00) >> 8);
	nim_reg_read(dev, R5F_ACQ_SYM_RATE + 1, &ver_data, 1);
//	dprintk(70, "%s: ver_data = 0x%x\n", __func__, ver_data);

	nim_reg_write(dev, R5F_ACQ_SYM_RATE + 0x01, &data, 1);
	nim_reg_read(dev, R5F_ACQ_SYM_RATE + 0x01, &ver_data, 1);
	if (data != ver_data)
	{
		dprintk(1, "%s: register 0x60 write error\n", __func__);
	}
//	dprintk(70, "%s: CR40 rs = 0x%x, ver_data = 0x%x\n", __func__, data, ver_data);

	// CR41
	data = (u8)((rs & 0x10000) >> 16);
	nim_reg_write(dev, R5F_ACQ_SYM_RATE + 0x02, &data, 1);
	nim_reg_read(dev, R5F_ACQ_SYM_RATE + 0x02, &ver_data, 1);
	if (data != ver_data)
	{
		dprintk(1, "%s: register 0x61 write error\n", __func__);
	}
//	dprintk(70, "%s: CR41 rs = 0x%x, ver_data = 0x%x\n", __func__, data, ver_data);
}

void nim_s3501_set_freq_offset(struct nim_device *dev, int delfreq)
{
	u8 data, ver_data;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	delfreq = delfreq * 1024;
	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		delfreq = (delfreq / (CRYSTAL_FREQ * 90 / 135)) * 90;
	}
	else
	{
		delfreq = (delfreq / (CRYSTAL_FREQ * 99 / 135)) * 90;
	}
	delfreq /= 1000;
	// CR5C
	data = (u8)(delfreq & 0xFF);
	nim_reg_write(dev, R5C_ACQ_CARRIER, &data, 1);
	nim_reg_read(dev, R5C_ACQ_CARRIER, &ver_data, 1);
	if (data != ver_data)
	{
		dprintk(1, "%s: register 0x5c write error\n", __func__);
	}
	// CR5D
	data = (u8)((delfreq & 0xFF00) >> 8);
	nim_reg_write(dev, R5C_ACQ_CARRIER + 0x01, &data, 1);
	nim_reg_read(dev, R5C_ACQ_CARRIER + 0x01, &ver_data, 1);
	if (data != ver_data)
	{
		dprintk(1, "%s: register 0x5d write error\n", __func__);
	}
	// CR5E
	data = (u8)((delfreq & 0x10000) >> 16);
	nim_reg_write(dev, R5C_ACQ_CARRIER + 0x02, &data, 1);

	nim_reg_read(dev, R5C_ACQ_CARRIER + 0x02, &ver_data, 1);
	if (data != ver_data)
	{
		dprintk(1, "%s: register 0x5 write error\n", __func__);
	}
}

int nim_s3501_get_bitmode(struct nim_device *dev, u8 *bitMode)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_1BIT_MODE)
	{
		*bitMode = 0x60;
	}
	else if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_2BIT_MODE)
	{
		*bitMode = 0x00;
	}
	else if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_4BIT_MODE)
	{
		*bitMode = 0x20;
	}
#if 0
	else if ((priv->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_8BIT_MODE)
	{
		*bitMode = 0x40;
	}
#endif
	else
	{
		*bitMode = 0x40;
	}
	return SUCCESS;
}

// begin add operation for s3501 optimize
#if 0
static int nim_s3501_set_err(struct nim_device *dev)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *)dev->priv;

	priv->ul_status.m_Err_Cnts++;
	return SUCCESS;
}
#endif

static int nim_s3501_get_err(struct nim_device *dev)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *)dev->priv;

	if (priv->ul_status.m_Err_Cnts > 0x00)
	{
		return ERR_FAILED;
	}
	else
	{
		return SUCCESS;
	}
}

static int nim_s3501_clear_err(struct nim_device *dev)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *)dev->priv;

	priv->ul_status.m_Err_Cnts = 0x00;
	return SUCCESS;
}

static int nim_s3501_get_type(struct nim_device *dev)
{
	u8 temp[4] = { 0x00, 0x00, 0x00, 0x00 };
	u32 m_Value = 0x00;
	struct nim_s3501_private *priv = (struct nim_s3501_private *)dev->priv;

	nim_reg_read(dev, RA3_CHIP_ID, temp, 4);
	m_Value = temp[1];
	m_Value = (m_Value << 8) | temp[0];
	m_Value = (m_Value << 8) | temp[3];
	m_Value = (m_Value << 8) | temp[2];
//	dprintk(70, "%s: temp = 0x%x, m_Value = 0x%x\n", __func__, (int)temp, (int)m_Value);
//	priv->ul_status.m_s3501_type = m_Value; //lwj remove
	nim_reg_read(dev, RC0_BIST_LDPC_REG, temp, 1);
	priv->ul_status.m_s3501_sub_type = temp[0];
	return SUCCESS;
}

static int nim_s3501_i2c_open(struct nim_device *dev)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	nim_s3501_clear_err(dev);

//	dprintk(100, "%s > priv->Tuner_Config_Data.QPSK_Config = 0x%x\n", __func__, priv->Tuner_Config_Data.QPSK_Config);
	if (priv->Tuner_Config_Data.QPSK_Config & M3501_I2C_THROUGH)
	{
		u8 data, ver_data;
		data = 0xd4;
		nim_reg_write(dev, RCB_I2C_CFG, &data, 1);

		nim_reg_read(dev, RB3_PIN_SHARE_CTRL, &ver_data, 1);
		data = ver_data | 0x04;
		nim_reg_write(dev, RB3_PIN_SHARE_CTRL, &data, 1);
		//comm_delay(200);
		udelay(200);

		if (nim_s3501_get_err(dev))
		{
			return ERR_FAILED;
		}
		return SUCCESS;
	}
	return SUCCESS;
}

static int nim_s3501_i2c_close(struct nim_device *dev)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	nim_s3501_clear_err(dev);

	if (priv->Tuner_Config_Data.QPSK_Config & M3501_I2C_THROUGH)
	{
		u8 data, ver_data;
		udelay(200);
		nim_reg_read(dev, RB3_PIN_SHARE_CTRL, &ver_data, 1);
		data = ver_data & 0xfb;
		nim_reg_write(dev, RB3_PIN_SHARE_CTRL, &data, 1);
		data = 0xc4;
		nim_reg_write(dev, RCB_I2C_CFG, &data, 1);
//		dprintk(20, "%s Exit nim through\n", __func__);

		if (nim_s3501_get_err(dev))
		{
			return ERR_FAILED;
		}
		return SUCCESS;
	}
	return SUCCESS;
}

static int nim_s3501_sym_config(struct nim_device *dev, u32 sym)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	if (sym > 40000)
	{
		priv->ul_status.c_RS = 8;
	}
	else if (sym > 30000)
	{
		priv->ul_status.c_RS = 4;
	}
	else if (sym > 20000)
	{
		priv->ul_status.c_RS = 2;
	}
	else if (sym > 10000)
	{
		priv->ul_status.c_RS = 1;
	}
	else
	{
		priv->ul_status.c_RS = 0;
	}
	return SUCCESS;
}

static int nim_s3501_agc1_ctrl(struct nim_device *dev, u8 low_sym, u8 s_Case)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	u8 data;

//	dprintk(100, "%s > priv->Tuner_Config_Data.QPSK_Config = 0x%x\n",priv->Tuner_Config_Data.QPSK_Config);
	// AGC1 setting
	if (priv->Tuner_Config_Data.QPSK_Config & M3501_NEW_AGC1)
	{
		switch (s_Case)
		{
			case NIM_OPTR_CHL_CHANGE:
			{
				if (1 == low_sym)
				{
					data = 0xaf;
				}
				else
				{
					data = 0xb5;
				}
				break;
			}
			case NIM_OPTR_SOFT_SEARCH:
			{
				if (1 == low_sym)
				{
					data = 0xb1;
				}
				else
				{
					data = 0xb9;
				}
				break;
			}
			case NIM_OPTR_FFT_RESULT:
			{
				data = 0xba;
				break;
			}
		}
	}
	else
	{
		switch (s_Case)
		{
			case NIM_OPTR_CHL_CHANGE:
			case NIM_OPTR_SOFT_SEARCH:
			{
				if (1 == low_sym)
				{
					data = 0x3c;
				}
				else
				{
					data = 0x54;
				}
				break;
			}
			case NIM_OPTR_FFT_RESULT:
			{
				data = 0x54;
				break;
			}
		}
	}
	if ((priv->Tuner_Config_Data.QPSK_Config & M3501_AGC_INVERT) == 0x0)  // STV6110's AGC be invert by QinHe
	{
		data = data ^ 0x80;
	}
	nim_reg_write(dev, R07_AGC1_CTRL, &data, 1);
	return SUCCESS;
}

static int nim_s3501_freq_offset_set(struct nim_device *dev, u8 low_sym, u32 *s_Freq)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	if (priv->Tuner_Config_Data.QPSK_Config & M3501_QPSK_FREQ_OFFSET)
	{
		if (1 == low_sym)
		{
			*s_Freq += 3;
		}
	}
	return SUCCESS;
}

static int nim_s3501_freq_offset_reset(struct nim_device *dev, u8 low_sym)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	if (priv->Tuner_Config_Data.QPSK_Config & M3501_QPSK_FREQ_OFFSET)
	{
		if (1 == low_sym)
		{
			nim_s3501_set_freq_offset(dev, -3000);
		}
		else
		{
			nim_s3501_set_freq_offset(dev, 0);
		}
	}
	else
	{
		nim_s3501_set_freq_offset(dev, 0);
	}
	return SUCCESS;
}

static int nim_s3501_cr_setting(struct nim_device *dev, u8 s_Case)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	u8 data;

	switch (s_Case)
	{
		case NIM_OPTR_SOFT_SEARCH:
		{
			// set CR parameter
			data = 0xaa;
			nim_reg_write(dev, R33_CR_CTRL + 0x03, &data, 1);
			data = 0x45;
			nim_reg_write(dev, R33_CR_CTRL + 0x04, &data, 1);
			data = 0x87;
			nim_reg_write(dev, R33_CR_CTRL + 0x05, &data, 1);

			if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
			{
				data = 0x9a;
			}
			else
			{
				data = 0xaa; // S2 CR parameter
			}
			nim_reg_write(dev, RB5_CR_PRS_TRA, &data, 1);
			break;
		}
		case NIM_OPTR_CHL_CHANGE:
		{
			// set CR parameter
			data = 0xaa;
			nim_reg_write(dev, R33_CR_CTRL + 0x03, &data, 1);
			data = 0x45;
			nim_reg_write(dev, R33_CR_CTRL + 0x04, &data, 1);
			data = 0x87;
			nim_reg_write(dev, R33_CR_CTRL + 0x05, &data, 1);

#if 0
			if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
			{
				data = 0xaa;
			}
			else
#endif
			{
				data = 0xaa; // S2 CR parameter
			}
			nim_reg_write(dev, RB5_CR_PRS_TRA, &data, 1);
			break;
		}
	}
	return SUCCESS;
}

static int nim_s3501_ldpc_setting(struct nim_device *dev, u8 s_Case, u8 c_ldpc, u8 c_fec)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	u8 data;

	// LDPC parameter
	switch (s_Case)
	{
		case NIM_OPTR_CHL_CHANGE:
		{
			if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
			{
				u8 temp[3] = { 0x32, 0xc8, 0x08 };
				nim_reg_write(dev, R57_LDPC_CTRL, temp, 3);
			}
			else
			{
				u8 temp[3] = { 0x32, 0x44, 0x04 };

				temp[0] = 0x1e - priv->ul_status.c_RS;  // 30 time iteration
				nim_reg_write(dev, R57_LDPC_CTRL, temp, 3);
			}
			data = 0x01;  // active ldpc avg iter
			nim_reg_write(dev, RC1_DVBS2_FEC_LDPC, &data, 1);
			break;
		}
		case NIM_OPTR_SOFT_SEARCH:
		{
			if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
			{
				u8 temp[3] = { 0x32, 0x48, 0x08 };
				nim_reg_write(dev, R57_LDPC_CTRL, temp, 3);
			}
			else
			{
				u8 temp[3] = { 0x1e, 0x46, 0x06 };
				nim_reg_write(dev, R57_LDPC_CTRL, temp, 3);
			}
			data = 0x01;  // active ldpc avg iter
			nim_reg_write(dev, RC1_DVBS2_FEC_LDPC, &data, 1);
			break;
		}
		case NIM_OPTR_DYNAMIC_POW:
		{
			data = c_ldpc - priv->ul_status.c_RS;
			nim_reg_write(dev, R57_LDPC_CTRL, &data, 1);
			break;
		}
	}
	data = c_fec;
	nim_reg_write(dev, RC1_DVBS2_FEC_LDPC, &data, 1);
	return SUCCESS;
}

static int nim_s3501_set_FC_Search_Range(struct nim_device *dev, u8 s_Case, u32 rs)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	u8 data, ver_data;
	u32 temp;

	switch (s_Case)
	{
		case NIM_OPTR_SOFT_SEARCH:
		{
			if (!(priv->t_Param.t_reg_setting_switch & NIM_SWITCH_FC))
			{
				// CR62, fc search range.
				if (rs > 16000)
				{
					temp = 5 * 16; // (4 * 90 * 16) / (CRYSTAL_FREQ * 99 / 135);
				}
				else if (rs > 5000)
				{
					temp = 4 * 16; // (3 * 90 * 16) / (CRYSTAL_FREQ * 99 / 135);
				}
				else
				{
					temp = 3 * 16; // (2 * 90 * 16) / (CRYSTAL_FREQ * 99/ 135);
				}
				data = temp & 0xff;
				nim_reg_write(dev, R62_FC_SEARCH, &data, 1);
				nim_reg_read(dev, R62_FC_SEARCH, &ver_data, 1);
				if (data != ver_data)
				{
					dprintk(1, "%s: Register 0x62 write error\n", __func__);
				}
				// CR63
				data = (temp >> 8) & 0x3;
				if (rs > 10000)
				{
					data |= 0xa0;
				}
				else if (rs > 3000)
				{
					data |= 0xc0;  // amy change for 91.5E 3814/V/6666
				}
				else
				{
					data |= 0xb0;  // amy change for 91.5E 3629/V/2200
				}
				nim_reg_write(dev, R62_FC_SEARCH + 0x01, &data, 1);
				nim_reg_read(dev, R62_FC_SEARCH + 0x01, &ver_data, 1);
				if (data != ver_data)
				{
					dprintk(1, "%s: Register 0x63 write error\n", __func__);
				}
				priv->t_Param.t_reg_setting_switch |= NIM_SWITCH_FC;
			}
			break;
		}
		case NIM_OPTR_CHL_CHANGE:
		{
			if (priv->t_Param.t_reg_setting_switch & NIM_SWITCH_FC)
			{
				// set sweep range
				if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
				{
					temp = (3 * 90 * 16) / (CRYSTAL_FREQ * 90 / 135);
				}
				else
				{
					temp = (3 * 90 * 16) / (CRYSTAL_FREQ * 99 / 135);
				}
				data = temp & 0xff;
				nim_reg_write(dev, R62_FC_SEARCH, &data, 1);

				data = (temp >> 8) & 0x3;
				data |= 0xb0;
				nim_reg_write(dev, R62_FC_SEARCH + 0x01, &data, 1);
				priv->t_Param.t_reg_setting_switch &= ~NIM_SWITCH_FC;
			}
			break;
		}
	}
	return SUCCESS;
}

static int nim_s3501_RS_Search_Range(struct nim_device *dev, u8 s_Case, u32 rs)
{
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;
	u8 data, ver_data;
	u32 temp;

	switch (s_Case)
	{
		case NIM_OPTR_SOFT_SEARCH:
		{
			if (!(priv->t_Param.t_reg_setting_switch & NIM_SWITCH_RS))
			{
				// CR64
				// temp = (3 * 90 * 16) / (CRYSTAL_FREQ * 99 / 135);
				// data = temp & 0xff;
				if (rs > 16000)
				{
					temp = rs / 4000;
				}
				else if (rs > 5000)
				{
					temp = rs / 3000;
				}
				else
				{
					temp = rs / 2000;
				}
				if (temp < 3)
				{
					temp = 3;
				}
				else if (temp > 11)
				{
					temp = 11;
				}
				temp = temp << 4;
				// temp = temp / (CRYSTAL_FREQ / 135);
				data = temp & 0xff;
				nim_reg_write(dev, R64_RS_SEARCH, &data, 1);
				nim_reg_read(dev, R64_RS_SEARCH, &ver_data, 1);
				if (data != ver_data)
				{
					dprintk(1, "%s: Register 0x64 write error\n", __func__);
				}
				// CR65
				data = (temp >> 8) & 0x3;
				if (rs > 6500)
				{
					data |= 0xb0;
				}
				else
				{
					data |= 0xa0;
				}
				// data |= 0xb0;
				nim_reg_write(dev, R64_RS_SEARCH + 0x01, &data, 1);
				nim_reg_read(dev, R64_RS_SEARCH + 0x01, &ver_data, 1);
				if (data != ver_data)
				{
					dprintk(1, "%s: Register 0x65 write error\n", __func__);
				}
				priv->t_Param.t_reg_setting_switch |= NIM_SWITCH_RS;
			}
			break;
		}
		case NIM_OPTR_CHL_CHANGE:
		{
			if (priv->t_Param.t_reg_setting_switch & NIM_SWITCH_RS)
			{
				if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
				{
					temp = (3 * 90 * 16) / (CRYSTAL_FREQ * 90 / 135);
				}
				else
				{
					temp = (3 * 90 * 16) / (CRYSTAL_FREQ * 99 / 135);
				}
				data = temp & 0xff;
				nim_reg_write(dev, R64_RS_SEARCH, &data, 1);

				data = (temp >> 8) & 0x3;
				data |= 0x30;
				nim_reg_write(dev, R64_RS_SEARCH + 0x01, &data, 1);
				priv->t_Param.t_reg_setting_switch &= ~NIM_SWITCH_RS;
			}
			break;
		}
	}
	return SUCCESS;
}

static int nim_s3501_TR_CR_Setting(struct nim_device *dev, u8 s_Case)
{
	u8 data = 0;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	switch (s_Case)
	{
		case NIM_OPTR_SOFT_SEARCH:
		{
			if (!(priv->t_Param.t_reg_setting_switch & NIM_SWITCH_TR_CR))
			{
				data = 0x4d;
				nim_reg_write(dev, R66_TR_SEARCH, &data, 1);
				data = 0x31;
				nim_reg_write(dev, R67_VB_CR_RETRY, &data, 1);
				priv->t_Param.t_reg_setting_switch |= NIM_SWITCH_TR_CR;
			}
			break;
		}
		case NIM_OPTR_CHL_CHANGE:
		{
			if (priv->t_Param.t_reg_setting_switch & NIM_SWITCH_TR_CR)
			{
				// set reg to default value
				data = 0x59;
				nim_reg_write(dev, R66_TR_SEARCH, &data, 1);
				data = 0x33;
				nim_reg_write(dev, R67_VB_CR_RETRY, &data, 1);
				priv->t_Param.t_reg_setting_switch &= ~NIM_SWITCH_TR_CR;
			}
			break;
		}
	}
	return SUCCESS;
}

#if 0  // we do not need task init
static int nim_s3501_task_init(struct nim_device *dev)
{
	u8 nim_device[3][3] = { 'N', 'M', '0', 'N', 'M', '1', 'N', 'M', '2' };
	//ID nim_task_id ;//= OSAL_INVALID_ID;
	//T_CTSK t_ctsk;
	T_CTSK nim_task_praram;
	static u8 nim_task_num = 0x00;
	struct nim_s3501_private *priv = (struct nim_s3501_private *) dev->priv;

	if (nim_task_num > 1)
	{
		return SUCCESS;
	}
	nim_task_praram.task = nim_s3501_task;//dmx_m3327_record_task;
	nim_task_praram.name[0] = nim_device[nim_task_num][0];
	nim_task_praram.name[1] = nim_device[nim_task_num][1];
	nim_task_praram.name[2] = nim_device[nim_task_num][2];
	nim_task_praram.stksz = 0xc00;
	nim_task_praram.itskpri = OSAL_PRI_NORMAL;
	nim_task_praram.quantum = 10;
	nim_task_praram.para1 = (u32) dev;
	nim_task_praram.para2 = 0 ;//Reserved for future use.
#if 0
	priv->tsk_status.m_task_id = osal_task_create(&nim_task_praram);
	if (OSAL_INVALID_ID == priv->tsk_status.m_task_id)
	{
		//soc_printk("Task create error\n");
		return OSAL_E_FAIL;
	}
#endif  // lwj remove
	nim_task_num++;
	return SUCCESS;
}

static void nim_s3501_task(u32 param1, u32 param2)
{
	struct nim_device *dev = (struct nim_device *) param1;
	struct nim_s3501_private *priv = (struct nim_s3501_private *)dev->priv;

	priv->tsk_status.m_sym_rate = 0x00;
	priv->tsk_status.m_code_rate = 0x00;
	priv->tsk_status.m_map_type = 0x00;
	priv->tsk_status.m_work_mode = 0x00;
	priv->tsk_status.m_info_data = 0x00;
#if defined(CHANNEL_CHANGE_ASYNC)
	u32 flag_ptn;
#endif

	while (1)
	{
#if defined(CHANNEL_CHANGE_ASYNC)
		flag_ptn = 0;
		if (NIM_FLAG_WAIT(&flag_ptn, priv->flag_id, NIM_FLAG_CHN_CHG_START, OSAL_TWF_ANDW | OSAL_TWF_CLR, 0) == OSAL_E_OK)
		{
			//osal_flag_clear(priv->flag_id, NIM_FLAG_CHN_CHG_START);
			NIM_FLAG_SET(priv->flag_id, NIM_FLAG_CHN_CHANGING);
			nim_s3501_waiting_channel_lock(dev, priv->cur_freq, priv->cur_sym);
			NIM_FLAG_CLEAR(priv->flag_id, NIM_FLAG_CHN_CHANGING);
		}
#endif
		if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
		{
			if ((priv->tsk_status.m_lock_flag == NIM_LOCK_STUS_SETTING) && (priv->t_Param.t_i2c_err_flag == 0x00))
			{
				nim_s3501_get_lock(dev, &(priv->tsk_status.m_info_data));
				if (priv->tsk_status.m_info_data && (priv->t_Param.t_i2c_err_flag == 0x00))
				{
					nim_s3501_reg_get_symbol_rate(dev, &(priv->tsk_status.m_sym_rate));
					nim_s3501_reg_get_code_rate(dev, &(priv->tsk_status.m_code_rate));
					nim_s3501_reg_get_work_mode(dev, &(priv->tsk_status.m_work_mode));
					nim_s3501_reg_get_map_type(dev, &(priv->tsk_status.m_map_type));
					if ((priv->ul_status.m_enable_dvbs2_hbcd_mode == 0)
					&&  ((priv->tsk_status.m_map_type == 0) || (priv->tsk_status.m_map_type == 5)))
					{
						dprintk(1, "%s: Demod Error: wrong map_type is %d\n", __func__, priv->tsk_status.m_map_type);
					}
					else
					{
						nim_s3501_set_ts_mode(dev, priv->tsk_status.m_work_mode, priv->tsk_status.m_map_type, priv->tsk_status.m_code_rate,
								      priv->tsk_status.m_sym_rate, 0x1);
						// open TS
						nim_reg_read(dev, R9C_DEMAP_BETA + 0x02, &(priv->tsk_status.m_info_data), 1);
						priv->tsk_status.m_info_data = priv->tsk_status.m_info_data | 0x80;    // ts open  //question
						nim_reg_write(dev, R9C_DEMAP_BETA + 0x02, &(priv->tsk_status.m_info_data), 1);
						priv->tsk_status.m_lock_flag = NIM_LOCK_STUS_CLEAR;
					}
				}
			}
		}
		else
		{
			break;
		}
		msleep(100);
	}
}
#endif

static int m3501_init(struct dvb_frontend *fe)
{
	struct dvb_m3501_fe_state *state = fe->demodulator_priv;

	dprintk(100, "%s >\n", __func__);
	return nim_s3501_open(&state->spark_nimdev);
}

static int m3501_term(struct nim_device *dev)
{
	struct nim_s3501_private *priv_mem;

	priv_mem = (struct nim_s3501_private *)dev->priv;
	i2c_put_adapter(priv_mem->i2c_adap);
	kfree(priv_mem);
	return 0;
}

static void m3501_release(struct dvb_frontend *fe)
{
	struct dvb_m3501_fe_state *state = fe->demodulator_priv;

	nim_s3501_close(&state->spark_nimdev);
	m3501_term(&state->spark_nimdev);
#if 1
	stpio_free_pin(state->fe_lnb_13_18);
	stpio_free_pin(state->fe_lnb_on_off);
#endif  /* 0 */
	kfree(state);
}

static int m3501_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct dvb_m3501_fe_state *state = fe->demodulator_priv;

	return nim_s3501_get_BER(&state->spark_nimdev, ber);
}

static int m3501_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	int iRet;
	struct dvb_m3501_fe_state *state = fe->demodulator_priv;

	iRet = nim_s3501_get_SNR(&state->spark_nimdev, (u16 *)snr); //quality
	if (*snr < 30)
	{
		*snr = *snr * 7 / 3;
	}
	else
	{
		*snr = *snr / 3 + 60;
	}
	if (*snr > 90)
	{
		*snr = 90;
	}
	*snr = *snr * 255 * 255 / 100;
//	dprintk(70, "%s: *snr = %d\n", __func__, *snr);
	return iRet;
}

static int m3501_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	int iRet;
	u16 *Intensity = (u16 *)strength;
	struct dvb_m3501_fe_state *state = fe->demodulator_priv;

	iRet = nim_s3501_get_AGC(&state->spark_nimdev, (u16 *)Intensity);  // level
	// lwj add begin
	if (*Intensity > 100)
	{
		*Intensity = 100;
	}
#if 0
	if (*Intensity < 10)
	{
		*Intensity = *Intensity * 11 / 2;
	}
	else
	{
		*Intensity = *Intensity / 2 + 50;
	}
	if (*Intensity > 90)
	{
		*Intensity = 90;
	}
	dprintk(20, "*Intensity = %d\n", *Intensity);
	*strength = *Intensity;
	dprintk(20, "*strength = %d\n", *strength);
	// lwj add end
#endif // 0
	*Intensity = *Intensity * 255 * 255 / 100;
	*strength = *Intensity;
//	dprintk(70, "%s: *strength = %d\n", __func__, *strength);
	return iRet;
}

static int m3501_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	int iRet;
	u8 lock;
	struct dvb_m3501_fe_state *state = fe->demodulator_priv;

	iRet = nim_s3501_get_lock(&state->spark_nimdev, &lock);
#if defined(NIM_S3501_DEBUG)
	dprintk(150, "%s: lock = %d\n", __func__, lock);
#endif
	if (lock)
	{
		*status = FE_HAS_SIGNAL
		        | FE_HAS_CARRIER
		        | FE_HAS_VITERBI
		        | FE_HAS_SYNC
		        | FE_HAS_LOCK;
	}
	else
	{
		*status = 0;
	}
	return iRet;
}

static enum dvbfe_algo m3501_frontend_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_CUSTOM;
}

static int m3501_set_tuner_params(struct dvb_frontend *fe, u32 freq, u32 sym)
{
	int err = 0;
	struct dvb_frontend_parameters param;
	struct dvb_frontend_ops        *frontend_ops = NULL;
	struct dvb_tuner_ops           *tuner_ops = NULL;

	if (!&fe->ops)
	{
		return 0;
	}
	frontend_ops = &fe->ops;

	if (!&frontend_ops->tuner_ops)
	{
		return 0;
	}
	tuner_ops = &frontend_ops->tuner_ops;

	param.frequency = freq;
	param.u.qpsk.symbol_rate = sym;
	dprintk(70, "%s: Frequency = %d MHz, Symbol rate = %d MS/s\n", __func__, param.frequency, param.u.qpsk.symbol_rate);

	if (!tuner_ops->set_params)
	{
		return 0;
	}
	err = tuner_ops->set_params(fe, &param);
	if (err < 0)
	{
		dprintk(1, "%s: Invalid parameter\n", __func__);
		return -1;
	}
	return 0;
}

#if 1
#if (DVB_API_VERSION < 5)
static enum dvbfe_search m3501_search(struct dvb_frontend *fe, struct dvbfe_params *p)
{
	//u32 IOHandle;
	struct nim_device *dev;
	struct nim_s3501_private *priv;
	//u32 TuneStartTime;
	u32 freq;
	u32 sym;
	//u8 result;
	u8 data = 0x10;
	u8 low_sym;
	int err;
	struct dvb_m3501_fe_state *state = fe->demodulator_priv;

	dev = (struct nim_device *)&state->spark_nimdev;
	priv = (struct nim_s3501_private *) dev->priv;
	printk("p->frequency is %d\n", p->frequency);
	priv->bLock = FALSE;
	//freq = 5150 - p->frequency / 1000;
	freq = p->frequency / 1000;
	sym = p->delsys.dvbs.symbol_rate / 1000;

//	u8 channel_change_flag = 1; // bentao add for judge channel chang or soft_search in set_ts_mode
//	dprintk(" Enter Function nim_s3501_channel_change \n");
	dprintk(50, "%s: Frequency is %d\n", __func__, freq);
	dprintk(50, "%s: Symbol rate is %d\n", __func__, sym);
//	dprintk(50, "%s: fec is %d\n", __func__, fec);
	priv->t_Param.t_phase_noise_detected = 0;
	priv->t_Param.t_dynamic_power_en = 0;
	priv->t_Param.t_last_snr = -1;
	priv->t_Param.t_last_iter = -1;
	priv->t_Param.t_aver_snr = -1;
	priv->t_Param.t_snr_state = 0;
	priv->t_Param.t_snr_thre1 = 256;
	priv->t_Param.t_snr_thre2 = 256;
	priv->t_Param.t_snr_thre3 = 256;
	priv->t_Param.phase_noise_detect_finish = 0x00;
#if 0 //#ifdef CHANNEL_CHANGE_ASYNC
	u32 flag_ptn = 0;
	if (NIM_FLAG_WAIT(&flag_ptn, priv->flag_id, NIM_FLAG_CHN_CHG_START | NIM_FLAG_CHN_CHANGING, OSAL_TWF_ORW, 0) == OSAL_E_OK)
	{
//		channel changing, stop the old changing first.
		priv->ul_status.s3501_chanscan_stop_flag = 1;
		//libc_printf("channel changing already, stop it first\n");
		msleep(2); //sleep 2ms
	}
#endif*/ //lwj remove
	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		dprintk(70, "%s: Chip version is_M3501B\n", __func__);
		priv->ul_status.phase_err_check_status = 0;
		priv->ul_status.s3501d_lock_status = NIM_LOCK_STUS_NORMAL;
	}
	priv->ul_status.m_setting_freq = freq;
	// reset first
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_demod_ctrl(dev, NIM_DEMOD_CTRL_0X91);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	msleep(5);  // sleep 5ms lwj add
	if ((0 == freq) || (0 == sym))
	{
		return DVBFE_ALGO_SEARCH_ERROR;
	}
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_sym_config(dev, sym);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
#if 1
	if (priv->ul_status.s3501_chanscan_stop_flag)
	{
		priv->ul_status.s3501_chanscan_stop_flag = 0;
		return DVBFE_ALGO_SEARCH_FAILED;
	}
#else
	if (Inst->ForceSearchTerm)
	{
		return SUCCESS;
	}
#endif
	// time for channel change and sof search.
	// ttt
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_TR_CR_Setting(dev, NIM_OPTR_CHL_CHANGE);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	low_sym = sym < 6500 ? 1 : 0; /* Symbol rate is less than 10M, low symbol rate */
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_freq_offset_set(dev, low_sym, &freq);
	{
		dprintk(150, "%s:[%d] freq = %d\n", __func__, __LINE__, freq);
	}
	if (nim_s3501_i2c_open(dev))
	{
		return DVBFE_ALGO_SEARCH_FAILED;
	}
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	err = m3501_set_tuner_params(fe, freq, sym);
	if (err < 0)
	{
		return DVBFE_ALGO_SEARCH_FAILED;
	}
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	if (nim_s3501_i2c_close(dev))
	{
		return DVBFE_ALGO_SEARCH_FAILED;
	}
	msleep(1);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	if (nim_s3501_i2c_open(dev))
	{
		return DVBFE_ALGO_SEARCH_FAILED;
	}
	if (fe->ops.tuner_ops.get_status)
	{
		int iTunerLock;

//		dprintk(150, "%s:[%d]\n", __func__, __LINE__);
		if (fe->ops.tuner_ops.get_status(fe, &iTunerLock) < 0)
		{
			dprintk(1, "%s: Tuner get_status failed\n", __func__);
		}
	}
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	if (nim_s3501_i2c_close(dev))
	{
		return DVBFE_ALGO_SEARCH_FAILED;
	}
//	nim_s3501_adc_setting(dev);
	data = 0x10;
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_reg_write(dev, RB3_PIN_SHARE_CTRL, &data, 1);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_adc_setting(dev);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_interrupt_mask_clean(dev);
	// hardware timeout setting
	// ttt
	// data = 0xff;
	// nim_reg_write(dev,R05_TIMEOUT_TRH, &data, 1);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_set_hw_timeout(dev, 0xff);
	// data = 0x1f; // setting for soft search function
	// nim_reg_write(dev, R1B_TR_TIMEOUT_BAND, &data, 1);
	// AGC1 setting
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_agc1_ctrl(dev, low_sym, NIM_OPTR_CHL_CHANGE);
	// Set symbol rate
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_set_RS(dev, sym);
	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		// Only for M3501B
//		dprintk(150, "%s:[%d]\n", __func__, __LINE__);
		nim_set_ts_rs(dev, sym);
	}
	// Set carry offset
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_freq_offset_reset(dev, low_sym);
	// ttt
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_cr_setting(dev, NIM_OPTR_CHL_CHANGE);
	// set workd mode
	// data = 0x34; // dvbs
	// data = 0x71; // dvbs2
	// data = 0x32; // dvbs2-hbcd-s
	// data = 0x52; // dvbs2-hbcd-s2
	// data = 0x73; // auto
	// ttt
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_set_acq_workmode(dev, NIM_OPTR_CHL_CHANGE0);
	// set sweep range
	// ttt
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_set_FC_Search_Range(dev, NIM_OPTR_CHL_CHANGE, 0x00);
	// ttt
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_RS_Search_Range(dev, NIM_OPTR_CHL_CHANGE, 0x00);
	// ttt
	// LDPC parameter
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_ldpc_setting(dev, NIM_OPTR_CHL_CHANGE, 0x00, 0x01);
#if 0

	// use RS to calculate MOCLK
	data = 0x00;
	nim_reg_write(dev, RAD_TSOUT_SYMB, &data, 1);
	// Set IO pad driving
	// ttt
	data = 0x00;
	nim_reg_write(dev, RAF_TSOUT_PAD, &data, 1);
#endif
	// ttt
#if 0
	data = 0x00;
	nim_reg_write(dev,RB1_TSOUT_SMT, &data, 1);
#endif
	// Carcy: disable HBCD check, let time out. 2008-03-12
	// ttt
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_hbcd_timeout(dev, NIM_OPTR_CHL_CHANGE);
	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		// when entering channel change, first close ts dummy.
		nim_close_ts_dummy(dev);
		// ECO_TS_EN = reg_cr9e[7], disable before dmy config successfully.
		nim_reg_read(dev, 0x9e, &data, 1);
		data = data & 0x7f;
		nim_reg_write(dev, 0x9e, &data, 1);
#if 0
		nim_reg_read(dev, 0xaf, &data, 1);
		data = data | 0x10;
		nim_reg_write(dev, 0xaf, &data, 1);
#endif
#if 0
		nim_reg_read(dev,RD8_TS_OUT_SETTING, &data, 1);
		dprintk(70, "When entering channel change reg_crd8 = %02x \n", data);
#endif
//		nim_s3501_set_dmy_format (dev);
	}
	else
	{
#if 0
		M3501A config.
		use RS to caculate MOCLK
		nim_s3501_get_bitmode(dev,&data);
		nim_reg_write(dev,RAD_TSOUT_SYMB+0x01, &data, 1);
#endif
		nim_s3501_set_ts_mode(dev, 0x0, 0x0, 0x0, 0x0, 0X1);
	}
	// comm_delay(10);
	msleep(1);
	if (sym < 3000)
	{
		if (sym < 2000)
		{
			data = 0x08;
		}
		else
		{
			data = 0x0a;
		}
		nim_reg_write(dev, R1B_TR_TIMEOUT_BAND, &data, 1);
	}
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_demod_ctrl(dev, NIM_DEMOD_CTRL_0X51);
#if 0 // #ifdef CHANNEL_CHANGE_ASYNC
	priv->cur_freq = freq;
	priv->cur_sym = sym;
	NIM_FLAG_SET(priv->flag_id, NIM_FLAG_CHN_CHG_START);
#else
	nim_s3501_waiting_channel_lock(/*Inst,*/ dev, freq, sym);
#endif
//	dprintk("Leave Fuction nim_s3501_channel_change\n");
	priv->ul_status.s3501_chanscan_stop_flag = 0;
	if (priv->bLock)
	{
		return DVBFE_ALGO_SEARCH_SUCCESS;
	}
	else
	{
		return DVBFE_ALGO_SEARCH_FAILED;
	}
}
#else
static enum dvbfe_search m3501_search(struct dvb_frontend *fe, struct dvb_frontend_parameters *p)
{
	struct nim_device *dev;
	struct nim_s3501_private *priv;
	u32 freq;
	u32 sym;
	u8 data = 0x10;
	u8 low_sym;
	int err;
	struct dvb_m3501_fe_state *state = fe->demodulator_priv;

	dev = (struct nim_device *)&state->spark_nimdev;
	priv = (struct nim_s3501_private *) dev->priv;

//	dprintk(70, "p->frequency is %d kHz\n", p->frequency);
	priv->bLock = FALSE;
	//freq = 5150 - p->frequency / 1000;
	freq = p->frequency / 1000;
	sym = p->u.qpsk.symbol_rate / 1000;

//	u8 channel_change_flag = 1;  // bentao add for judging channel change or soft_search in set_ts_mode

//	dprintk(100,"%s > nim_s3501_channel_change \n", __func__);
//	dprintk(20, "Frequency is  : %d MHz\n", freq);
//	dprintk(20, "Symbol rate is: %d Ms/s\n", sym);

	priv->t_Param.t_phase_noise_detected = 0;
	priv->t_Param.t_dynamic_power_en = 0;
	priv->t_Param.t_last_snr = -1;
	priv->t_Param.t_last_iter = -1;
	priv->t_Param.t_aver_snr = -1;
	priv->t_Param.t_snr_state = 0;
	priv->t_Param.t_snr_thre1 = 256;
	priv->t_Param.t_snr_thre2 = 256;
	priv->t_Param.t_snr_thre3 = 256;
	priv->t_Param.phase_noise_detect_finish = 0x00;

#if 0
#if defined(CHANNEL_CHANGE_ASYNC)
	 u32 flag_ptn = 0;

	if (NIM_FLAG_WAIT(&flag_ptn, priv->flag_id, NIM_FLAG_CHN_CHG_START | NIM_FLAG_CHN_CHANGING, OSAL_TWF_ORW, 0) == OSAL_E_OK)
	{
		// channel changing, stop the old changing first.
		priv->ul_status.s3501_chanscan_stop_flag = 1;
		//libc_printf("channel changing already, stop it first\n");
		msleep(2); //sleep 2ms
	}
#endif  // CHANNEL_CHANGE_ASYNC
#endif  // lwj remove
	if ((priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B) && (paramDebug >= 150))
	{
		dprintk(1, "%s: Chip version is M3501B\n", __func__);
		priv->ul_status.phase_err_check_status = 0;
		priv->ul_status.s3501d_lock_status = NIM_LOCK_STUS_NORMAL;
	}
	priv->ul_status.m_setting_freq = freq;

	//reset first
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_demod_ctrl(dev, NIM_DEMOD_CTRL_0X91);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	msleep(5); //sleep 5ms lwj add

	if ((0 == freq) || (0 == sym))
	{
		return DVBFE_ALGO_SEARCH_ERROR;
	}
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_sym_config(dev, sym);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
#if 1
	if (priv->ul_status.s3501_chanscan_stop_flag)
	{
	priv->ul_status.s3501_chanscan_stop_flag = 0;
	return DVBFE_ALGO_SEARCH_FAILED;
	}
#else
	if (Inst->ForceSearchTerm)
	{
		return SUCCESS;
	}
#endif

	// time for channel change and soft search.
	// ttt
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_TR_CR_Setting(dev, NIM_OPTR_CHL_CHANGE);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	low_sym = sym < 6500 ? 1 : 0;  /* Symbol rate is less than 10M, low symbol rate */
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_freq_offset_set(dev, low_sym, &freq);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	dprintk(20, "Frequency is = %d MHz\n", freq);

	if (nim_s3501_i2c_open(dev))
	{
		return DVBFE_ALGO_SEARCH_FAILED;
	}
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	err = m3501_set_tuner_params(fe, freq, sym);
	if (err < 0)
	{
		return DVBFE_ALGO_SEARCH_FAILED;
	}
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	if (nim_s3501_i2c_close(dev))
	{
		return DVBFE_ALGO_SEARCH_FAILED;
	}
	msleep(1);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	if (nim_s3501_i2c_open(dev))
	{
		return DVBFE_ALGO_SEARCH_FAILED;
	}
	if (fe->ops.tuner_ops.get_status)
	{
		int iTunerLock;

//		dprintk(150, "%s:[%d]\n", __func__, __LINE__);
		if (fe->ops.tuner_ops.get_status(fe, &iTunerLock) < 0)
		{
			dprintk(1, "%s: Tuner get_status failed\n", __func__);
		}
	}
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	if (nim_s3501_i2c_close(dev))
	{
		return DVBFE_ALGO_SEARCH_FAILED;
	}
	// nim_s3501_adc_setting(dev);
	data = 0x10;
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_reg_write(dev, RB3_PIN_SHARE_CTRL, &data, 1);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_adc_setting(dev);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_interrupt_mask_clean(dev);

	// hardware timeout setting
	// ttt
	// data = 0xff;
	// nim_reg_write(dev,R05_TIMEOUT_TRH, &data, 1);
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_set_hw_timeout(dev, 0xff);
	// data = 0x1f; // setting for soft search function
	// nim_reg_write(dev, R1B_TR_TIMEOUT_BAND, &data, 1);

	// AGC1 setting
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_agc1_ctrl(dev, low_sym, NIM_OPTR_CHL_CHANGE);

	// Set symbol rate
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_set_RS(dev, sym);

	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		// Only for M3501B
//		dprintk(150, "%s:[%d]\n", __func__, __LINE__);
		nim_set_ts_rs(dev, sym);
	}
	// Set carry offset
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_freq_offset_reset(dev, low_sym);
	// ttt
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_cr_setting(dev, NIM_OPTR_CHL_CHANGE);

	// set workd mode
	// data = 0x34;    // dvbs
	// data = 0x71;    // dvbs2
	// data = 0x32;    // dvbs2-hbcd-s
	// data = 0x52;    // dvbs2-hbcd-s2
	// data = 0x73;    // auto
	// ttt
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_set_acq_workmode(dev, NIM_OPTR_CHL_CHANGE0);
	// set sweep range
	// ttt
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_set_FC_Search_Range(dev, NIM_OPTR_CHL_CHANGE, 0x00);
	// ttt
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_RS_Search_Range(dev, NIM_OPTR_CHL_CHANGE, 0x00);
	// ttt
	// LDPC parameter
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_ldpc_setting(dev, NIM_OPTR_CHL_CHANGE, 0x00, 0x01);
#if 0
	// use RS to calculate MOCLK
	data = 0x00;
	nim_reg_write(dev, RAD_TSOUT_SYMB, &data, 1);
	// Set IO pad driving
	// ttt
	data = 0x00;
	nim_reg_write(dev, RAF_TSOUT_PAD, &data, 1);
#endif
	// ttt
#if 0
	data = 0x00;
	nim_reg_write(dev,RB1_TSOUT_SMT, &data, 1);
#endif
	// Carcy: disable HBCD check, let time out. 2008-03-12
	// ttt
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_hbcd_timeout(dev, NIM_OPTR_CHL_CHANGE);

	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		// when entering channel change, first close ts dummy.
		nim_close_ts_dummy(dev);
//		ECO_TS_EN = reg_cr9e[7], disable before dmy config successfully.
		nim_reg_read(dev, 0x9e, &data, 1);
		data = data & 0x7f;
		nim_reg_write(dev, 0x9e, &data, 1);
#if 0
	    nim_reg_read(dev, 0xaf, &data, 1);
	    data = data | 0x10;
	    nim_reg_write(dev, 0xaf, &data, 1);
#endif
#if 0
		nim_reg_read(dev, RD8_TS_OUT_SETTING, &data, 1);
		dprintk(70, "%s: when enter channel change  reg_crd8 = %02x\n", __func__, data);
#endif
		//nim_s3501_set_dmy_format (dev);
	}
	else
	{
#if 0
		M3501A config.
		use RS to caculate MOCLK
		nim_s3501_get_bitmode(dev,&data);
		nim_reg_write(dev,RAD_TSOUT_SYMB+0x01, &data, 1);
#endif
		nim_s3501_set_ts_mode(dev, 0x0, 0x0, 0x0, 0x0, 0X1);
	}
	//comm_delay(10);
	msleep(1);
	if (sym < 3000)
	{
		if (sym < 2000)
		{
			data = 0x08;
		}
		else
		{
			data = 0x0a;
		}
		nim_reg_write(dev, R1B_TR_TIMEOUT_BAND, &data, 1);
	}
//	dprintk(150, "%s:[%d]\n", __func__, __LINE__);
	nim_s3501_demod_ctrl(dev, NIM_DEMOD_CTRL_0X51);
//	dprintk(70, "%s: Tune time: %d ms\n", __func__, YWOS_TimeNow() - TuneStartTime);

#if defined(CHANNEL_CHANGE_ASYNC)
	priv->cur_freq = freq;
	priv->cur_sym = sym;
	NIM_FLAG_SET(priv->flag_id, NIM_FLAG_CHN_CHG_START);
#else
	nim_s3501_waiting_channel_lock(/*Inst,*/ dev, freq, sym);
#endif  // CHANNEL_CHANGE_ASYNC

//	dprintk(100, "%s <\n", __func__);
	priv->ul_status.s3501_chanscan_stop_flag = 0;
	if (priv->bLock)
	{
		return DVBFE_ALGO_SEARCH_SUCCESS;
	}
	else
	{
		return DVBFE_ALGO_SEARCH_FAILED;
	}
}
#endif  //CHANNEL_CHANGE_ASYNC
#endif

static int m3501_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct dvb_m3501_fe_state *state = fe->demodulator_priv;

	if (enable)
	{
		if (nim_s3501_i2c_open(&state->spark_nimdev))
		{
			return S3501_ERR_I2C_NO_ACK;
		}
	}
	else
	{
		if (nim_s3501_i2c_close(&state->spark_nimdev))
		{
			return S3501_ERR_I2C_NO_ACK;
		}
	}
	return 0;
}

#if (DVB_API_VERSION < 5)
static int m3501_get_info(struct dvb_frontend *fe, struct dvbfe_info *fe_info)
{
	//struct dvb_d0367_fe_ofdm_state* state = fe->demodulator_priv;

	/* get delivery system info */
	if (fe_info->delivery == DVBFE_DELSYS_DVBS2)
	{
		return 0;
	}
	else
	{
		return -EINVAL;
	}
	return 0;
}
#else
static int m3501_get_property(struct dvb_frontend *fe, struct dtv_property *tvp)
{
	//struct dvb_d0367_fe_ofdm_state* state = fe->demodulator_priv;

	/* get delivery system info */
	if (tvp->cmd == DTV_DELIVERY_SYSTEM)
	{
		switch (tvp->u.data)
		{
			case SYS_DVBS2:
			case SYS_DVBS:
			case SYS_DSS:
			{
				break;
			}
			default:
			{
				return -EINVAL;
			}
		}
	}
	return 0;
}
#endif

static int m3501_set_tone(struct dvb_frontend *fe, fe_sec_tone_mode_t tone)
{
	u8 data;
	struct dvb_m3501_fe_state *state = fe->demodulator_priv;
	struct nim_device *dev;
	struct nim_s3501_private *priv;

	dev = (struct nim_device *)&state->spark_nimdev;
	priv = (struct nim_s3501_private *) dev->priv;

	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		data = (CRYSTAL_FREQ * 90 / 135);
	}
	else
	{
		data = (CRYSTAL_FREQ * 99 / 135);
	}
	nim_reg_write(dev, R7C_DISEQC_CTRL + 0x14, &data, 1);

	dprintk(20, "Tone is %s\n", (tone == SEC_TONE_ON ? "on" : "off"));
	if (tone == SEC_TONE_ON)  /* Low band -> no 22KHz tone */
	{
		nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
		data = ((data & 0xF8) | 1);
		nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);
	}
	else
	{
		nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
		data = ((data & 0xF8) | 0);
		nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);
	}
	return 0;
}

static int m3501_set_voltage(struct dvb_frontend *fe, fe_sec_voltage_t voltage)
{
	struct dvb_m3501_fe_state *state = fe->demodulator_priv;

	switch (voltage)
	{
		case SEC_VOLTAGE_OFF:
		{
			dprintk(20, "Switch LNB voltage off\n");
			stpio_set_pin(state->fe_lnb_on_off, 0);
			break;
		}
		case SEC_VOLTAGE_13:  /* vertical */
		{
			dprintk(20, "Set LNB voltage 13V (vertical)\n");
			stpio_set_pin(state->fe_lnb_on_off, 1);
			msleep(1);
			stpio_set_pin(state->fe_lnb_13_18, 1);
			msleep(1);
			nim_s3501_set_polar(&state->spark_nimdev, NIM_PORLAR_VERTICAL);
			break;
		}
		case SEC_VOLTAGE_18:  /* horizontal */
		{
			dprintk(20, "Set LNB voltage 18V (horizontal)\n");
			stpio_set_pin(state->fe_lnb_on_off, 1);
			msleep(1);
			stpio_set_pin(state->fe_lnb_13_18, 0);
			msleep(1);
			nim_s3501_set_polar(&state->spark_nimdev, NIM_PORLAR_HORIZONTAL);
			break;
		}
		default:
		{
			dprintk(1, "%s: Error: unknown LNB voltage\n", __func__);
			break;
		}
	}
	return 0;
}

int m3501_send_diseqc_msg(struct dvb_frontend *fe, struct dvb_diseqc_master_cmd *cmd)
{
	struct dvb_m3501_fe_state *state = fe->demodulator_priv;

	u8  *pCurrent_Data = cmd->msg;
	u8  data, temp;
	u16 timeout, timer;
	u8 i;
	struct nim_s3501_private *priv;
	struct nim_device *dev;

	dev = (struct nim_device *)&state->spark_nimdev;
	priv = (struct nim_s3501_private *) dev->priv;

	if (priv->ul_status.m_s3501_type == NIM_CHIP_ID_M3501B)
	{
		data = (CRYSTAL_FREQ * 90 / 135);
	}
	else
	{
		data = (CRYSTAL_FREQ * 99 / 135);
	}
	nim_reg_write(dev, R7C_DISEQC_CTRL + 0x14, &data, 1);

	{
		// DiSEqC init for mode byte every time: test
		nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
		data = ((data & 0xF8) | 0x00);
		nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);

		// write the writed data count
		nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
		temp = cmd->msg_len - 1;
		data = ((data & 0xC7) | (temp << 3));
		nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);

		// write the data
		for (i = 0; i <  cmd->msg_len; i++)
		{
			nim_reg_write(dev, (i + 0x7E), pCurrent_Data + i, 1);
		}

		// clear the interupt
		nim_reg_read(dev, R7C_DISEQC_CTRL + 0x01, &data, 1);
		data &= 0xF8;
		nim_reg_write(dev, R7C_DISEQC_CTRL + 0x01, &data, 1);

		// write the control bits
		nim_reg_read(dev, R7C_DISEQC_CTRL, &data, 1);
		temp = 0x04;
		data = ((data & 0xF8) | temp);
		nim_reg_write(dev, R7C_DISEQC_CTRL, &data, 1);

		// waiting for the send over
		timer = 0;
		timeout = 75 + 13 * cmd->msg_len;
		while (timer < timeout)
		{
			nim_reg_read(dev, R7C_DISEQC_CTRL + 0x01, &data, 1);
			// change < to != by Joey
			if (0 != (data & 0x07))
			{
				break;
			}
			msleep(10);
			timer += 10;
		}
		if (1 == (data & 0x07))
		{
			msleep(100); // dhy 20100416, very important: we add this 100ms delay. some time if no delay the DiSEqC switch not good.
			return 0;
		}
		else
		{
			return -1;
		}
	}
	return (0);
}

static struct dvb_frontend_ops spark_m3501_ops =
{
	.info =
	{
		.name                = "Ali M3501",
		.type                = FE_QPSK,
		.frequency_min       = 950000,
		.frequency_max       = 2150000,
		.frequency_stepsize  = 0,
		.frequency_tolerance = 0,
		.symbol_rate_min     = 1000000,
		.symbol_rate_max     = 45000000,
		.caps                = FE_CAN_INVERSION_AUTO
		                     | FE_CAN_FEC_AUTO
		                     | FE_CAN_QPSK
	},
	.init                    = m3501_init,
	.release                 = m3501_release,
	.read_ber                = m3501_read_ber,
	.read_snr                = m3501_read_snr,
	.read_signal_strength    = m3501_read_signal_strength,
	.read_status             = m3501_read_status,
	.get_frontend_algo       = m3501_frontend_algo,
	.search                  = m3501_search,
	.i2c_gate_ctrl           = m3501_i2c_gate_ctrl,
#if (DVB_API_VERSION < 5)
	.get_info                = m3501_get_info,
#else
	.get_property            = m3501_get_property,
#endif
	.set_tone                = m3501_set_tone,
	.set_voltage             = m3501_set_voltage,
	.diseqc_send_master_cmd  = m3501_send_diseqc_msg,
};

static int m3501_initialization(struct nim_device *dev, struct i2c_adapter *i2c)
{
	struct nim_s3501_private *priv_mem;

	priv_mem = (struct nim_s3501_private *)kzalloc(sizeof(struct nim_s3501_private), GFP_KERNEL);
	if (priv_mem == NULL)
	{
		kfree(dev);
		dprintk(1, "%s: Error allocating nim device private memory\n", __func__);
		return YWHAL_ERROR_NO_MEMORY;
	}
	memset(dev, 0, sizeof(struct nim_device));
	memset(priv_mem, 0, sizeof(struct nim_s3501_private));

	dev->priv = (void *) priv_mem;
	//dev->base_addr = 0x66; /* M3501 i2c base address*/
	dev->base_addr = 0x66 >> 1; /* M3501 i2c base address*/

	priv_mem->i2c_type_id = 0; //question

	priv_mem->i2c_adap = i2c;
	if (!priv_mem->i2c_adap)
	{
		return -1;
	}
//	dprintk(70, "priv_mem->i2c_adap = %0x\n", (int)priv_mem->i2c_adap);

	//priv_mem->tuner_id = Handle; //lwj add important
	priv_mem->ext_dm_config.i2c_type_id = 0;
	priv_mem->ext_dm_config.i2c_base_addr = 0x66; //3501 i2c base addr //0xC0; //7306 i2c base address

	priv_mem->ul_status.m_enable_dvbs2_hbcd_mode = 0;
	priv_mem->ul_status.m_dvbs2_hbcd_enable_value = 0x7f;
	priv_mem->ul_status.nim_s3501_sema = OSAL_INVALID_ID;
	priv_mem->ul_status.s3501_autoscan_stop_flag = 0;
	priv_mem->ul_status.s3501_chanscan_stop_flag = 0;
	priv_mem->ul_status.old_ber = 0;
	priv_mem->ul_status.old_per = 0;
	priv_mem->ul_status.m_hw_timeout_thr = 0;
	priv_mem->ul_status.old_ldpc_ite_num = 0;
	priv_mem->ul_status.c_RS = 0;
	priv_mem->ul_status.phase_err_check_status = 0;
	priv_mem->ul_status.s3501d_lock_status = NIM_LOCK_STUS_NORMAL;
	priv_mem->ul_status.m_s3501_type = 0x00;
	priv_mem->ul_status.m_s3501_type = NIM_CHIP_ID_M3501B;  // lwj add for 3501B
	priv_mem->ul_status.m_setting_freq = 123;
	priv_mem->ul_status.m_Err_Cnts = 0x00;
//	priv_mem->tsk_status.m_lock_flag = NIM_LOCK_STUS_NORMAL;//
//	priv_mem->tsk_status.m_task_id = 0x00;//
	priv_mem->t_Param.t_aver_snr = -1;
	priv_mem->t_Param.t_last_iter = -1;
	priv_mem->t_Param.t_last_snr = -1;
	priv_mem->t_Param.t_snr_state = 0;
	priv_mem->t_Param.t_snr_thre1 = 256;
	priv_mem->t_Param.t_snr_thre2 = 256;
	priv_mem->t_Param.t_snr_thre3 = 256;
	priv_mem->t_Param.t_dynamic_power_en = 0;
	priv_mem->t_Param.t_phase_noise_detected = 0;
	priv_mem->t_Param.t_reg_setting_switch = 0x0f;
	priv_mem->t_Param.t_i2c_err_flag = 0x00;
	priv_mem->flag_id = OSAL_INVALID_ID;

	priv_mem->bLock = FALSE;

	priv_mem->Tuner_Config_Data.QPSK_Config = 0xe9;  // lwj
	//priv_mem->Tuner_Config_Data.QPSK_Config = 0x29;  // lf

	nim_s3501_get_type(dev);
	if (priv_mem->ul_status.m_s3501_type == NIM_CHIP_ID_M3501A  // Chip 3501A
	&&  (priv_mem->Tuner_Config_Data.QPSK_Config & 0xc0) == M3501_2BIT_MODE)  // TS 2bit mode
	{
		// for M3606 + M3501A full nim ssi-2-bit patch, auto change to 1-bit mode.
		priv_mem->Tuner_Config_Data.QPSK_Config &= 0x3f; // set to TS 1-bit mode
		//libc_printf("M3501A SSI 2-bit mode, auto change to 1bit mode\n");
	}
	dprintk(20, "Found Ali M3501%s demodulator\n", (priv_mem->ul_status.m_s3501_type == NIM_CHIP_ID_M3501A ? "A" : "B"));
	return 0;
}

struct dvb_frontend *dvb_m3501_fe_qpsk_attach(const struct m3501_config *config, struct i2c_adapter *i2c)
{
	struct dvb_m3501_fe_state *state = NULL;

	/* allocate memory for the internal state */
	state = kmalloc(sizeof(struct dvb_m3501_fe_state), GFP_KERNEL);
	if (state == NULL)
	{
		goto error;
	}
	memset(state, 0, sizeof(struct dvb_m3501_fe_state));

	/* create dvb_frontend */
	memcpy(&state->frontend.ops, &spark_m3501_ops, sizeof(struct dvb_frontend_ops));
	if (m3501_initialization(&state->spark_nimdev, i2c) < 0)
	{
		kfree(state);
		return NULL;
	}
	memcpy(&state->frontend.ops.info.name, config->name, sizeof(config->name));
	state->frontend.demodulator_priv = state;
#if defined(SPARK7162)
	switch (config->i)
	{
		case 0:
		{
			dprintk(20, "Allocating LNB control PIO pins for frontend %d\n", config->i);
			state->fe_lnb_13_18 = stpio_request_pin(15, 6, "lnb_0 13/18", STPIO_OUT);
			state->fe_lnb_on_off = stpio_request_pin(15, 7, "lnb_0 onoff", STPIO_OUT);
			if (!state->fe_lnb_13_18 || !state->fe_lnb_on_off)
			{
				dprintk(1,"%s: lnb_0 request pin failed\n", __func__);
				if (state->fe_lnb_13_18)
				{
					stpio_free_pin(state->fe_lnb_13_18);
				}
				if (state->fe_lnb_on_off)
				{
					stpio_free_pin(state->fe_lnb_on_off);
				}
				kfree(state);
				return NULL;
			}
			break;
		}
		case 1:
		{
			dprintk(20, "Allocating LNB control PIO pins for frontend %d\n", config->i);
			state->fe_lnb_13_18 = stpio_request_pin(15, 4, "lnb_1 13/18", STPIO_OUT);
			state->fe_lnb_on_off = stpio_request_pin(15, 5, "lnb_1 onoff", STPIO_OUT);
			if (!state->fe_lnb_13_18 || !state->fe_lnb_on_off)
			{
				dprintk(1, "%s: lnb_1 request pin failed\n", __func__);
				if (state->fe_lnb_13_18)
				{
					stpio_free_pin(state->fe_lnb_13_18);
				}
				if (state->fe_lnb_on_off)
				{
					stpio_free_pin(state->fe_lnb_on_off);
				}
				kfree(state);
				return NULL;
			}
			break;
		}
		default:
		{
			dprintk(1, "Error: frontend number %d not valid\n", config->i);
			break;
		}
	}
#endif  // SPARK7162

	{
		int ret = ERR_FAILED;

		ret = nim_s3501_hw_check(&state->spark_nimdev);
		if (ret != SUCCESS)
		{
			m3501_term(&state->spark_nimdev);
			kfree(state);
			return NULL;
		}
	}
	return &state->frontend;

error:
	kfree(state);
	return NULL;
}
// vim:ts=4

