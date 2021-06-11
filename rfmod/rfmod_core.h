/***************************************************************************
 *
 * rfmod_core.h  Enigma2 compatible RF modulator driver
 *
 * (c) 2021 Audioniek
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
 ***************************************************************************
 *
 * Changes
 *
 * Date     By              Description
 * -------------------------------------------------------------------------
 * 20210326 Audioniek       Initial version.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>

#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <asm/string.h>

extern short paramDebug;
#define dprintk(level, x...) \
do \
{ \
	if ((paramDebug) && (paramDebug >= level) || level == 0) \
	printk(TAGDEBUG x); \
} while (0)

#define I2C_DRIVERID_RFMOD 1
#define RFMOD_MAJOR        150

// IOCTL values sent by Enigma2
#define IOCTL_SET_CHANNEL         0
#define IOCTL_SET_TESTMODE        1
#define IOCTL_SET_SOUNDENABLE     2
#define IOCTL_SET_SOUNDSUBCARRIER 3
#define IOCTL_SET_FINETUNE        4
#define IOCTL_SET_STANDBY         5

// XXX74T1
int xxx_74t1_init(struct i2c_client *client);
int xxx_74t1_ioctl(struct i2c_client *client, unsigned int cmd, void *arg);
int xxx_74t1_ioctl_kernel(struct i2c_client *client, unsigned int cmd, void *arg);
// vim:ts=4
