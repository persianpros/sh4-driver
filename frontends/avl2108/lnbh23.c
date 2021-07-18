/*
 * @brief lnbh23.c
 *
 * @author konfetti
 *
 * 	Copyright (C) 2011 duckbox
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "dvb_frontend.h"
#include "avl2108.h"

extern short paramDebug;
#define TAGDEBUG "[lnbh23] "

#define dprintk(level, x...) \
do \
{ \
	if ((paramDebug) && (paramDebug >= level)) \
	{ \
		printk(TAGDEBUG x); \
	} \
} while (0)

struct lnb_state
{
	struct i2c_adapter *i2c;
	u32                lnb[6];
};

/*
 * LNBH23 register description:
 * I2C bus 0 and 1, addr 0x0a (or 0x0b, with ADDR pin = H)
 * seen values: 0xd4 (vertical) 0xdc (horizontal), 0xd0 shutdown
 *
 * Audioniek added:
 * 0xd4 -> 11010100 -> PCL=1, TTX=1, TEN=0, LLC=1, VSEL=0, EN=1 -> Vout=14.4V
 * 0xdc -> 11011100 -> PCL=1, TTX=1, TEN=0, LLC=1, VSEL=1, EN=1 -> Vout=19.5V
 * 0xd0 -> 11010000 -> PCL=1, TTX=1, TEN=0, LLC=1, VSEL=0, EN=0 -> Vout=0V
 * 0xd8 -> 11011000 -> PCL=1, TTX=1, TEN=0, LLC=1, VSEL=1, EN=0 -> Vout=0V
 * therefore: 0xd0 and 0xd8 are both valid for Vout=0
 *
 * From Documentation (can be downloaded on st.com):
 *
 * System register, write
 * MSB                           LSB
 * PCL TTX TEN LLC VSEL EN TEST4 TEST5 Function
 *      0       0    0   1  0     0    Vout=13.4V, Vup=14.5V
 *      0       0    1   1  0     0    Vout=18.4V, Vup=19.5V
 *      0       1    0   1  0     0    Vout=14.4V, Vup=15.5V
 *      0       1    1   1  0     0    Vout=19.5V, VUP=20.6V
 *      0   0            1  0     0    22kHz tone is disabled, EXTM disabled
 *      1   0            1  0     0    22kHz tone is controlled by DSQIN pin
 *      1   1            1  0     0    22kHz tone is ON, DSQIN pin disabled
 *      0                1  0     0    VoRX output is ON, VoTX tone generator is OFF
 *      1                1  0     0    VoRX output is ON, VoTX tone generator is ON
 *  0   X                1  0     0    Pulsed (dynamic) current limiting is selected
 *  1   X                1  0     0    Static current limiting is selected
 *  X   X   X   X    X   0  X     X    Power blocks disabled
 *
 * System register, read
 * MSB                           LSB (Note: S means read back as written)
 * TEST1 TEST2 TEST3 LLC VSEL EN OTF OLF Function
 *                    S   S    S  0   X  Tj < 135 degrees C (normal operation)
 *                    S   S    S  1   X  Tj > 150 degrees C (power blocks disabled)
 *                    S   S    S  X   0  Io < Iomax (normal operation)
 *                    S   S    S  X   1  Io > Iomax (Overload protection triggered)
 *   X    X     X                        These bits must be ignored
 *
 */

static int writereg_lnb_supply(struct lnb_state *state, char data)
{
	int ret = -EREMOTEIO;
	struct i2c_msg msg;
	u8 buf;

	buf = data;

	msg.addr = state->lnb[1];
	msg.flags = 0;
	msg.buf = &buf;
	msg.len = 1;

	dprintk(100, "%s: write 0x%02x to 0x%02x\n", __func__, (int)buf, msg.addr);

#if 0
	if ((ret = i2c_transfer(state->i2c, &msg, 1)) != 1)
	{
		/* ! attempts to write the same data intended for an LNBH23 to one
           half of an LNBH221. These chips are not completely software
           compatible as some bits in the register written have different
           meanings.
        */
		// wohl nicht LNBH23, mal mit LNBH221 versuchen
		msg.addr = state->lnb[2];
		msg.flags = 0;
		msg.buf = &buf;  // ! byte to write is intended for an LNBH23
		msg.len = 1;
		dprintk(20, "%s: 2nd write 0x%02x to 0x%02x\n", __func__, (int)buf, msg.addr);
		if ((ret = i2c_transfer(state->i2c, &msg, 1)) != 1)
		{
			dprintk(1, "%s: error (ret = %i)\n", __func__, ret);
			ret = -EREMOTEIO;
		}
	}
#else
	if ((ret = i2c_transfer(state->i2c, &msg, 1)) != 1)
	{
		dprintk(1, "%s: Error (ret = %i)\n", __func__, ret);
		ret = -EREMOTEIO;
	}
#endif
	return ret;
}

u16 lnbh23_set_voltage(void *_state, struct dvb_frontend *fe, fe_sec_voltage_t voltage)
{
	struct lnb_state *state = (struct lnb_state *) _state;
	u16 ret = 0;

	dprintk(100, "%s(%p, %d) >\n", __func__, fe, voltage);

	switch (voltage)
	{
		case SEC_VOLTAGE_OFF:
		{
			dprintk(20, "Switch LNB voltage off\n");
			writereg_lnb_supply(state, state->lnb[3]);
			break;
		}
		case SEC_VOLTAGE_13: //vertical
		{
			dprintk(20, "Set LNB voltage 13V (vertical)\n");
			writereg_lnb_supply(state, state->lnb[4]);
			break;
		}
		case SEC_VOLTAGE_18: //horizontal
		{
			dprintk(20, "Set LNB voltage 18V (horizontal)\n");
			writereg_lnb_supply(state, state->lnb[5]);
			break;
		}
		default:
		{
			return -EINVAL;
		}
	}
	return ret;
}

void *lnbh23_attach(u32 *lnb, struct avl2108_equipment_s *equipment)
{
	struct lnb_state *state = kmalloc(sizeof(struct lnb_state), GFP_KERNEL);

	memcpy(state->lnb, lnb, sizeof(state->lnb));
//	equipment->lnb_set_voltage = lnbh221_set_voltage;
	equipment->lnb_set_voltage = lnbh23_set_voltage;
	state->i2c = i2c_get_adapter(lnb[0]);
	dprintk(20, "I2C adapter = %p\n", state->i2c);
	return state;
}
// vim:ts=4
