/*
 * TI SN761640 DVB-T(2)/C tuner driver
 *
 * Version for Fulan Spark7162 (Frontend Sharp ????1ED5469)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef __SN761640_H
#define __SN761640_H

#ifndef dprintk
#define dprintk(level, x...) do \
{ \
	if ((paramDebug) && ((paramDebug >= level) || level == 0)) \
	{ \
		printk(TAGDEBUG x); \
	} \
} while (0)
#endif

struct sn761640_config
{
	char           name[128];

	u8             addr;
//	the following are used in cable version only
	fe_bandwidth_t bandwidth;
	u32            Frequency;
	u32            IF;
	u32            TunerStep;
};
extern struct dvb_frontend *sn761640t_attach(struct dvb_frontend *fe, const struct sn761640_config *config, struct i2c_adapter *i2c);
extern struct dvb_frontend *sn761640c_attach(struct dvb_frontend *fe, const struct sn761640_config *config, struct i2c_adapter *i2c);

#endif  // __SN761640_H
// vim:ts=4
