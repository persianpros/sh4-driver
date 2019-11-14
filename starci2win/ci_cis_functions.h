/*
 * 	Copyright (C) 2010 Duolabs Srl
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
 *
 */

#ifndef __CI_CIS_FUNCTIONS_H__
#define __CI_CIS_FUNCTIONS_H__


#include "cidef.h"
I32 PHYS_CheckCIS(UI8 *,I32,UI16 *,UI8 *);
I32 PHYS_Tuple_CISTPL_CFTABLE_ENTRY( UI8 *,I32, struct  dvb_entry_bundle *);
I32 PHYS_Tuple(UI8, UI8 **, I32 *, I32 *, I32 *, I32 *, I32 *);

#endif ///__CI_CIS_FUNCTIONS_H__



