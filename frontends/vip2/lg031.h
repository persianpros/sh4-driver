/****************************************************************************
 *
 * LG 031 tuner driver
 * Copyright (C) ?
 *
 * Version for:
 * Edision argus VIP (1 pluggable tuner)
 * Edision argus VIP2 (2 pluggable tuners)
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ***************************************************************************/

#ifndef __LG031_H
#define __LG031_H

struct lg031_config
{
	char           name[128];

	u8             addr;
	fe_bandwidth_t bandwidth;
	u32            Frequency;
	u32            IF;
	u32            TunerStep;
};

extern struct dvb_frontend *lg031_attach(struct dvb_frontend *fe, const struct lg031_config *config, struct i2c_adapter *i2c);
#endif /* __LG031_H */
// vim:ts=4
