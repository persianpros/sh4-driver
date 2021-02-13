/*
 * @brief lnbp12.c
 *
 * Driver for STM LNBP12 LNB power controller
 *
 * Originally written by konfetti as lnb_pio.c
 *
 * 	Copyright (C) 2011 duckbox
 *            (C) 2020 Audioniek: adapted to STM LNBP12
 *
 *  Driver for STM LNBP12 LNB PIO driven power controller as
 *  used in the Opticum HD (TS) 9600 series
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

#include <linux/version.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
#include <linux/stm/pio.h>
#else
#include <linux/stpio.h>
#endif

#include "dvb_frontend.h"
#include "avl2108.h"

extern short paramDebug;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[lnbp12] "


struct lnb_state
{
	struct stpio_pin *lnb_pin;
	struct stpio_pin *lnb_enable_pin;
	struct stpio_pin *lnb_tone_enable_pin;
	struct stpio_pin *lnb_llc_pin;

/*	lnb is an array holding the following data:
	[0] = PIO port for enable (= on/off)
	[1] = PIO pin for enable (= on/off)
	[2] = polarity (active low/high)
	[3] = PIO port for V/H select
	[4] = PIO pin for V/H select
	[5] = value for vertical (13/14V)
 */
	u32              lnb[6];
};

/*
 * The LNBP12 LNB power controller as used in the Opticum HD 9600 and
 * HD 9600 PRIMA series is connected as follows:
 *
 * Vsel (pin 4): sets the LNB voltage to 13/14V or 18/19V -> PIO2.2
 * EN   (pin 5): switches LNB voltage on (H) or off (L)   -> PIO5.2
 *               when off, LNB voltage is that on the
 *               MI input (pin 10), that is grounded in
 *               the Opticum HD 9600 series; EN acts
 *               therefore as LNB voltage on/off input
 * ENT  (pin 7): switch 22kHz tone on (H) or off (L)      -> PIO2.3
 *               The driver initializes this pin to L
 *               (tone off) and does not bother with
 *               this pin any further
 * LLC  (pin 9): Elevates LNB voltage by 1V when high     -> PIO2.6
 *               The driver initializes this pin to L
 *               (+1V off) and does not bother with
 *               this pin any further
 * MI: (pin 10): connected to ground
 *
 * The LNBP12 does not provide any pins to monitor its status,
 * e.g. LNB overcurrent.
 */
#define LNBP12_ENT_PORT 2
#define LNBP12_ENT_PIN  3
#define LNBP12_LLC_PORT 2
#define LNBP12_LLC_PIN  6

u16 lnbp12_set_high_lnb_voltage(void *_state, struct dvb_frontend *fe, long arg)
{
	struct lnb_state *state = (struct lnb_state *) _state;
	u16 ret = 0;

	dprintk(150, "%s > (%p, %d)\n", __func__, fe, arg);

	stpio_set_pin(state->lnb_llc_pin, (arg ? 1 : 0));
	dprintk(150, "%s < (%d)\n", __func__, ret);
	return ret;
}

u16 lnbp12_set_voltage(void *_state, struct dvb_frontend *fe, fe_sec_voltage_t voltage)
{
	struct lnb_state *state = (struct lnb_state *) _state;
	u16 ret = 0;

	dprintk(150, "%s > (%p, %d)\n", __func__, fe, voltage);

	switch (voltage)
	{
		case SEC_VOLTAGE_OFF:
		{
			if (state->lnb_enable_pin)
			{
				stpio_set_pin(state->lnb_enable_pin, !state->lnb[2]);  // switch LNB voltage off
				dprintk(20, "LNB voltage is off\n");
			}
			break;
		}
		case SEC_VOLTAGE_13:  // vertical
		{
			if (state->lnb_enable_pin)
			{
				stpio_set_pin(state->lnb_enable_pin, state->lnb[2]);  // switch LNB voltage on
			}
			stpio_set_pin(state->lnb_pin, state->lnb[5]);
			dprintk(20, "LNB voltage = 13V (vertical)\n");
			break;
		}
		case SEC_VOLTAGE_18:  // horizontal
		{
			if (state->lnb_enable_pin)
			{
				stpio_set_pin(state->lnb_enable_pin, state->lnb[2]);  // switch LNB voltage on
			}
			stpio_set_pin(state->lnb_pin, !state->lnb[5]);
			dprintk(20, "LNB voltage = 18V (horizontal)\n");
			break;
		}
		default:
		{
			dprintk(1, "%s < Error: invalid voltage value %d)\n", __func__, voltage);
			return -EINVAL;
		}
	}
	dprintk(150, "%s < (%d)\n", __func__, ret);
	return ret;
}

void *lnbp12_attach(u32 *lnb, struct avl2108_equipment_s *equipment)
{
	struct lnb_state *state = kmalloc(sizeof(struct lnb_state), GFP_KERNEL);

	memcpy(state->lnb, lnb, sizeof(state->lnb));

	equipment->lnb_set_voltage = lnbp12_set_voltage;
	equipment->set_high_lnb_voltage = lnbp12_set_high_lnb_voltage;

//	Allocate PIO pins
#if defined(OPT9600)
//	Switch 22kHz tone of LNBP12 off
	dprintk(70, "Initializing PIO %1d.%d (LNBP12_ENT)\n", LNBP12_ENT_PORT, LNBP12_ENT_PIN);
	state->lnb_tone_enable_pin = stpio_request_pin(LNBP12_ENT_PORT, LNBP12_ENT_PIN, "LNBP12_ENT", STPIO_OUT);
	stpio_set_pin(state->lnb_tone_enable_pin, 0);

//	Switch elevated LNB voltages off
	dprintk(70, "Initializing PIO %1d.%d (LNBP12_LLC)\n", LNBP12_LLC_PORT, LNBP12_LLC_PIN);
	state->lnb_llc_pin = stpio_request_pin(LNBP12_LLC_PORT, LNBP12_LLC_PIN, "LNBP12_LLC", STPIO_OUT);
	stpio_set_pin(state->lnb_llc_pin, 0);
#endif

	dprintk(70, "Initializing PIO %1d.%d (LNBP12_EN)\n", lnb[0], lnb[1]);
	state->lnb_enable_pin = stpio_request_pin(lnb[0], lnb[1], "LNBP12_EN", STPIO_OUT);
	stpio_set_pin(state->lnb_enable_pin, lnb[2]);  // switch LNB voltage off

	dprintk(70, "Initializing PIO %1d.%d (LNBP12_VSEL)\n", lnb[3], lnb[4]);
	state->lnb_pin = stpio_request_pin(lnb[3], lnb[4], "LNBP12_VSEL", STPIO_OUT);
	return state;
}
// vim:ts=4
