/****************************************************************************************
 *
 * pace_7241_fp.h
 *
 * (c) 20?? j00zek
 * (c) 2019-2020 Audioniek
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
 * Front panel driver for Pace HDS-7241
 *
 * Devices:
 *  - /dev/vfd (vfd ioctls and read/write function)
 *
 ****************************************************************************************/
#if !defined __PACE_7241_FP_H___
#define __PACE_7241_FP_H___

//extern static short paramDebug = 0;

#define TAGDEBUG "[pace_7241_fp] "
#define dprintk(level, x...) \
do \
{ \
	if ((paramDebug) && (paramDebug >= level) || level == 0) \
	printk(TAGDEBUG x); \
} while (0)

/***************************************************
 *
 * PIO definitions
 *
 */
#define LED1    stm_gpio(4, 0)
#define LED2    stm_gpio(4, 1)

#define KEYPp	stm_gpio(5, 4)
#define KEYPm   stm_gpio(5, 5)

#if 0
#define KEY_R   stm_gpio(5, 6)
#define KEY_L   stm_gpio(5, 7)
#define KEY_PWR stm_gpio(11, 6)
#endif

/***************************************************
 *
 * IOCTL definitions
 *
 */
#define VFDIOC_DCRAMWRITE        0xc0425a00
#define VFDIOC_BRIGHTNESS        0xc0425a03
#define VFDIOC_DISPLAYWRITEONOFF 0xc0425a05
#define VFDIOC_DRIVERINIT        0xc0425a08
#define VFDIOC_ICONDISPLAYONOFF  0xc0425a0a

#define VFD_MAJOR 147

/***************************************************
 *
 * Other definitions
 *
 */
struct vfd_ioctl_data
{
	unsigned char address;
	unsigned char data[64];
	unsigned char length;
};

struct vfd_driver
{
	struct semaphore sem;
	int opencount;
};

spinlock_t mr_lock = SPIN_LOCK_UNLOCKED;
#endif // __PACE_7241_FP_H___
// vim:ts=4
