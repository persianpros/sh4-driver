/*
 * boxtype.c
 * 
 *  captaintrip 12.07.2008
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
 */

/*
 *  Description:
 *
 *  in /proc/boxtype
 *
 *  ufs910 14w returns 1 or 3
 *  ufs910 01w returns 0
 *	adb_box bska returns bska
 *	adb_box bsla returns bsla
 *	adb_box bxzb returns bxzb
 *	adb_box bzzb returns bzzb
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>

#if defined(ADB_BOX)
#include <linux/i2c.h>
#include <linux/delay.h>
#endif

#include "boxtype.h"
#include "pio.h"

#define procfs_name "boxtype"
#define DEVICENAME "boxtype"

int boxtype = 0;
struct proc_dir_entry *BT_Proc_File;

#if defined(ADB_BOX)
#define I2C_ADDR_STB0899_1  (0xd0 >> 1)  //d0 = 0x68 d2 = 69 tuner 1 demod BSLA & BSKA/BXZB
#define I2C_ADDR_STB0899_2  (0xd2 >> 1)  //d0 = 0x68 d2 = 69 tuner 2 demod BSLA only
#define I2C_ADDR_STV090x    (0xd0 >> 1)  //d0 = 0x68 d2 = 69 tuner 1/2 demod BZZB only
#define I2C_BUS             0

#define STB0899_NCOARSE     0xf1b3
#define STB0899_DEV_ID      0xf000
#define STV090x_MID         0xf100

static int _read_reg_boxtype(unsigned int reg, unsigned char nr)
{
	int ret;
	u8 b0[] = { reg >> 8, reg & 0xff };
	u8 buf;

	struct i2c_msg msg[] =
	{
		{
			.addr  = nr,
			.flags = 0,
			.buf   = b0,
			.len   = 2
		},
		{
			.addr  = nr,
			.flags = I2C_M_RD,
			.buf   = &buf,
			.len   = 1
		}
	};

	struct i2c_adapter *i2c_adap = i2c_get_adapter (I2C_BUS);

//	dprintk("[boxtype] stb0899 read reg = %x\n", reg);
	ret = i2c_transfer(i2c_adap, msg, 2);
	if (ret != 2)
	{
		return -1;
	}
	return (int)buf;
}

int stb0899_read_reg_boxtype(unsigned int reg, unsigned char nr)
{
	int result;

	result = _read_reg_boxtype(reg, nr);
	/*
	 * Bug ID 9:
	 * access to 0xf2xx/0xf6xx
	 * must be followed by read from 0xf2ff/0xf6ff.
	 */
	if ((reg != 0xf2ff) 
	&&  (reg != 0xf6ff)
	&&  (((reg & 0xff00) == 0xf200) || ((reg & 0xff00) == 0xf600)))
	{
		_read_reg_boxtype((reg | 0x00ff), nr);
	}
	return result;
}

int stv6412_boxtype(void)
{
	int ret;
	unsigned char buf;
	struct i2c_msg msg = { .addr = 0x4b, I2C_M_RD, .buf = &buf, .len = 1 };
	struct i2c_adapter *i2c_adap = i2c_get_adapter (I2C_BUS);

	ret = i2c_transfer(i2c_adap, &msg, 1);
	return ret;
}
#endif  // defined(ADB_BOX)

/************************************************************************
*
* Unfortunately there is no generic mechanism to unambiguously determine
* STBs from different manufacturers. Since each hardware platform needs
* special I/O pin handling it also requires a different kernel image.
* Setting platform device configuration in the kernel helps to roughly
* determine the STB type. Further identification can be based on reading
* an EPLD or I/O pins.
* To provide a platform identification data add a platform device
* to the board setup.c file as follows:
*
*   static struct platform_device boxtype_device = {
*        .name = "boxtype",
*        .dev.platform_data = (void*)NEW_ID
*   };
*
*   static struct platform_device *plat_devices[] __initdata = {
*           &boxtype_device,
*           &st40_ohci_devices,
*           &st40_ehci_devices,
*           &ssc_device,
*   ...
*
* Where NEW_ID is a unique integer identifying the platform and
* plat_devices is provided to the platform_add_devices() function
* during initialization of hardware resources.
*
************************************************************************/
#if 0
static int boxtype_probe (struct device *dev)
{
	struct platform_device *pdev = to_platform_device (dev);

	if (pdev != NULL)
	{
		boxtype = (int)pdev->dev.platform_data;
	}
	return 0;
}

static int boxtype_remove (struct device *dev)
{
	return 0;
}
#endif

#if 0
static struct device_driver boxtype_driver =
{
	.name   = DEVICENAME,
	.bus    = &platform_bus_type,
	.owner  = THIS_MODULE,
	.probe  = boxtype_probe,
	.remove = boxtype_remove,
};
#endif

int procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;

	if (offset > 0)
	{
		ret  = 0;
	}
	else
	{
#if !defined(ADB_BOX)
		ret = sprintf(buffer, "%d\n", boxtype);
#else
		switch (boxtype)
		{
			case 1:
			{
				ret = sprintf(buffer, "bska\n");
				break;
			}
			case 2:
			{
				break;
				ret = sprintf(buffer, "bsla\n");
			}
			case 3:
			{
				ret = sprintf(buffer, "bxzb\n");
				break;
			}
			case 4:
			{
				ret = sprintf(buffer, "bzzb\n");
				break;
			}
			default:
			{
				ret = sprintf(buffer, "UNKNOWN\n");
				break;
			}
		}
#endif
	}
	return ret;
}

int __init boxtype_init(void)
{
#if defined(ADB_BOX)
	int ret;
#endif
	dprintk("[boxtype] Initializing...\n");

// TODO: FIX THIS
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
	if (driver_register(&boxtype_driver) < 0)
	{
		printk ("%s(): error registering device driver\n", __func__);
	}
#endif
	if (boxtype == 0)
	{
#if !defined(ADB_BOX)
		/* no platform data found, assume ufs910 */
		boxtype = (STPIO_GET_PIN(PIO_PORT(4), 5) << 1) | STPIO_GET_PIN(PIO_PORT(4), 4);
#else // ADB_BOX
//		first, check AVS for a STV6412, if not -> BXZB model 
		ret = stv6412_boxtype();
//		dprintk("[boxtype] %s ret1 = %d\n", __func__, ret);  //ret 1 = ok -> STV6412 present
		if (ret != 1)
		{
			boxtype = 3;	// no STV6412 --> BXZB model
			dprintk("[boxtype] BXZB model detected\n");
		}
		else  // if STV6412 found, check for 2nd demodulator (two tuners)
		{
			// if you can read the register from the second demodulator, then this is a BSLA
			ret = stb0899_read_reg_boxtype(STB0899_NCOARSE, I2C_ADDR_STB0899_2);  // read DIV from demodulator
//			dprintk("[boxtype] %s ret2 = %d\n", __func__, ret);
			if (ret != -1)  // if read OK --> 2nd STV0899 demod present --> must be a BSLA
			{
				boxtype = 2;  // BSLA model
				dprintk("[boxtype] BSLA model detected\n");
			}
			else  // check demodulator type
			{
				// check if demodulator is stb0899 or stv090x
				// read chipid and revision
				ret = stb0899_read_reg_boxtype(STB0899_DEV_ID, I2C_ADDR_STB0899_1);
//				dprintk("boxtype] %s ret3 = %d\n", __func__, ret);  // stb0899 = 130
				if (ret > 0x30)  // if response greater than 0x30 --> STB0899 demodulator
				{
					boxtype = 1; // STB0899 demodulator --> BSKA model
					dprintk("[boxtype] BSKA model detected\n");
				}
				else if (ret > 0)
				{
					boxtype = 4;  // STV090X demodulator --> BZZB model
					dprintk("[boxtype] BZZB model detected\n");
				}
				else
				{
					boxtype = 0;
					dprintk("[boxtype] Unknown ADB model detected\n");
				}
			}
		}
#endif
	}
	dprintk("[boxtype] boxtype = %d\n", boxtype);
	BT_Proc_File = create_proc_read_entry(procfs_name, 0644, NULL, procfile_read, NULL);
	return 0;
}

void __exit boxtype_exit(void)
{
	dprintk("[boxtype] unloading...\n");
// TODO: FIX THIS
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
	driver_unregister (&boxtype_driver);
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
	remove_proc_entry(procfs_name, &proc_root);
#else
	remove_proc_entry(procfs_name, NULL);
#endif
}

module_init(boxtype_init);
module_exit(boxtype_exit);

MODULE_DESCRIPTION("Provides the type of an STB based on STb710x");
MODULE_AUTHOR("Team Ducktales, mod B4Team & freebox");
MODULE_LICENSE("GPL");
// vim:ts=4
