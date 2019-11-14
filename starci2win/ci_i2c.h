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

#ifndef __CI_I2C_H__
#define __CI_I2C_H__


#define         CiMAX_CNTL      0x1F
#define         AMOD_CNTL       0x00
#define         BMOD_CNTL       0x09
#define         AMOD_IIM        0x06
#define         BMOD_IIM        0x0F
#define         CiMAX_DEST      0x17
#define         CiMAX_PWR       0x18
#define         AMOD_ASMH       0x01
#define         BMOD_ASMH       0x0A
#define         AMOD_ASML       0x02
#define         BMOD_ASML       0x0B

#define			StarCI_Single_TS_CNTL	0x10
#define			StarCI_Twin_TS_CNTL		0x11

#define         CiMAX_ASMHE     0x12
#define         CiMAX_ASMLE     0x13
#define         AMOD_ASPH       0x03
#define         BMOD_ASPH       0x0C
#define         AMOD_ASPL       0x04
#define         BMOD_ASPL       0x0D
#define         AMOD_MACT       0x05
#define         BMOD_MACT       0x0E
#define         CiMAX_IS        0x1A
#define         CiMAX_IMR       0x1B
#define         CiMAX_ICR       0x1C
#define         CiMAX_UCSG1     0x1D
#define         CiMAX_UCSG2     0x1E
#endif ///__CI_I2C_H__


