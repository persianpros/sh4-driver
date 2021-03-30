/* *************************************************************************
 *
 * rfmod_core.c  Enigma2 compatible RF modulator driver
 *
 * (c) 2021 Audioniek
 *
 * Largely based on code taken from TDT lnb driver, thank you author(s).
 *
 * Driver currently only supports the XXX74T1 RF modulator chip.
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

#include "rfmod_core.h"

short paramDebug = 0;
#if defined TAGDEBUG
#undef TAGDEBUG
#endif
#define TAGDEBUG "[rfmod] "

enum
{
	MC44BC74T1,
};

static const struct i2c_device_id rfmod_id[] =
{
	{ "74t1", MC44BC74T1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rfmod_id);

static int devType;
static char *type = "74t1";  // default value

/*
 * Addresses to scan
 */
static unsigned short normal_i2c[] =
{
	0x65,  // MC44BC74T1 and compatibles
	I2C_CLIENT_END
};
I2C_CLIENT_INSMOD;

static struct i2c_driver rfmod_i2c_driver;
static struct i2c_client *rfmod_client = NULL;

/*
 * i2c probe
 */
static int rfmod_newprobe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if (rfmod_client)
	{
		dprintk(1, "Failure, client already registered\n");
		return -ENODEV;
	}
	dprintk(20, "I2C device found at address 0x%02x\n", client->addr);

	switch (devType)
	{
		case MC44BC74T1:
		{
			xxx_74t1_init(client);
			break;
		}
		default:
		{
			return -ENODEV;
		}
	}
	rfmod_client = client;
	return 0;
}

static int rfmod_remove(struct i2c_client *client)
{
	rfmod_client = NULL;
	dprintk(20, "Remove driver\n");
	return 0;
}

/*
 * devfs fops
 */
static int rfmod_command_ioctl(struct i2c_client *client, unsigned int cmd, void *arg)
{
	int err = 0;

	dprintk(100, "%s > (IOCTL = %x)\n", __func__, cmd);

	switch (devType)
	{
		case MC44BC74T1:
		{
			err = xxx_74t1_ioctl(client, cmd, arg);
			break;
		}
	}
	return err;
}

static int rfmod_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	dprintk(100, "%s > IOCTL: %02x\n", __func__, cmd);
	return rfmod_command_ioctl(rfmod_client, cmd, (void *)arg);
}

static int rfmod_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int rfmod_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations rfmod_fops =
{
	.owner   = THIS_MODULE,
	.ioctl   = rfmod_ioctl,
	.open    = rfmod_open,
	.release = rfmod_close
};

static int rfmod_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	const char *name = "";

	if (kind == 0)
	{
		name = "74t1";
		kind = MC44BC74T1;
	}
	if (kind < 0)  // if kind negative
	{
		/* detection and identification */
		if (!type || !strcmp("74t1", type))
		{
			kind = MC44BC74T1;
		}
		else
		{
			return -ENODEV;
		}
	}

	switch (kind)
	{
		case MC44BC74T1:
		{
			name = "74t1";
			break;
		}
		default:
		{
			return -ENODEV;
		}
	}
	devType = kind;
	strlcpy(info->type, name, I2C_NAME_SIZE);
	return 0;
}

/*
 * i2c
 */
static struct i2c_driver rfmod_i2c_driver =
{
	.class = I2C_CLASS_TV_DIGITAL,
	.driver =
	{
		.owner = THIS_MODULE,
		.name  = "rfmod_driver",  /* 2.6.30 requires name without spaces */
	},
	.probe        = rfmod_newprobe,
	.detect       = rfmod_detect,
	.remove       = rfmod_remove,
	.id_table     = rfmod_id,
	.address_data = &addr_data,
	.command      = rfmod_command_ioctl
};

/*
 * module init/exit
 */
int __init rfmod_init(void)
{
	int res, err;
	const char *name;

	struct i2c_board_info info;
	err = rfmod_detect(NULL, -1, &info);
	name = info.type;

	if (err)
	{
		dprintk(1, "Unknown RF modulator type\n");
		return err;
	}

	res = i2c_add_driver(&rfmod_i2c_driver);
	if (res)
	{
		dprintk(1, "Adding i2c driver failed\n");
		return res;
	}

	if (!rfmod_client)
	{
		dprintk(1, "Specified RF modulator not found\n");
		i2c_del_driver(&rfmod_i2c_driver);
		return -EIO;
	}

	if (register_chrdev(RFMOD_MAJOR, "RFmod", &rfmod_fops) < 0)
	{
		dprintk(1, "Unable to register device\n");
		i2c_del_driver(&rfmod_i2c_driver);
		return -EIO;
	}
	dprintk(20, "Module RFmod loaded, type: %s\n", name);
	return 0;
}

void __exit rfmod_exit(void)
{
	unregister_chrdev(RFMOD_MAJOR, "RFmod");
	i2c_del_driver(&rfmod_i2c_driver);
	dprintk(20, "Module RFmod unloaded\n");
}

module_init(rfmod_init);
module_exit(rfmod_exit);

module_param(type, charp, 0);
MODULE_PARM_DESC(type, "device type (74t1(default))");

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

MODULE_AUTHOR("Open Vision developers");
MODULE_DESCRIPTION("Enigma2 compatible RF modulator driver");
MODULE_LICENSE("GPL");
// vim:ts=4
