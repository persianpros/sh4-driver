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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>

#include "starci2win_i2c.h"

unsigned short ignore[] = { I2C_CLIENT_END };
unsigned short normal_addr[] = { 0x43, I2C_CLIENT_END };

extern struct i2c_driver starci2win_driver;
extern int debug;

struct starci2win_state {
        u8                  addr;
        int errmode;
        struct i2c_adapter* i2c;
};

static struct i2c_client *new_client = NULL;
static struct starci2win_state * state;

int reg_write (struct starci2win_state *state, u8 reg, u8 data)
{
	int ret;
	u8 buf [] = { reg, data };
	struct i2c_msg msg = { .addr = state->addr, .flags = 0, .buf = buf, .len = 2 };

        dprintk("w: addr = 0x%02x : reg = 0x%02x val = 0x%02x\n", msg.addr, reg, data);
	ret = i2c_transfer (state->i2c, &msg, 1);

	if (ret != 1)
		dprintk("%s: writereg error (reg == 0x%02x, val == 0x%02x, "
			"ret == %i)\n", __FUNCTION__, reg, data, ret);

	return (ret != 1) ? -EREMOTEIO : 0;
}

u8 reg_read (struct starci2win_state *state, u8 reg)
{
	int ret;
	u8 b0 [] = { reg };
	u8 b1 [] = { 0 };
	struct i2c_msg msg [] = { { .addr = state->addr, .flags = 0, .buf = b0, .len = 1 },
			   	  { .addr = state->addr, .flags = I2C_M_RD, .buf = b1, .len = 1 } 
				};

	ret = i2c_transfer (state->i2c, msg, 2);
        dprintk("r: addr = 0x%02x : reg = 0x%02x, val = 0x%02x\n", msg[0].addr, reg, b1[0]);

	if (ret != 2)
		dprintk("%s: readreg error (reg == 0x%02x, ret == %i)\n",
				__FUNCTION__, reg, ret);

	return b1[0];
}

int reg_read_cont (struct starci2win_state* state, u8 reg1, u8 *b, u8 len)
{
	int ret;
	struct i2c_msg msg [] = { { .addr = state->addr, .flags = 0, .buf = &reg1, .len = 1 },
			   { .addr = state->addr, .flags = I2C_M_RD, .buf = b, .len = len } };

	ret = i2c_transfer (state->i2c, msg, 2);

	if (ret != 2)
		dprintk("%s: readreg error (ret == %i)\n", __FUNCTION__, ret);

	return ret == 2 ? 0 : ret;
}

/* This function is used by user to read a register */
unsigned char read_i2c(unsigned short reg_i2c)
{
	return reg_read (state,reg_i2c);
}

/* This function is used by user to write a register */
int write_i2c(unsigned short reg_i2c,unsigned char dat)
{
	return reg_write (state,reg_i2c,dat);
}

struct i2c_client_address_data addr_data = {
        .normal_i2c             = normal_addr,
        .probe                  = ignore,
        .ignore                 = ignore
};

int client_attach(struct i2c_adapter *adap, int addr, int type)
{
    char name[I2C_NAME_SIZE] = "starci2win";

    dprintk("Inside %s()\n", __FUNCTION__);
	
    /* Create and attach the i2c client */
	if ((new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        memset(new_client, 0, sizeof(struct i2c_client));

	memcpy(new_client->name, name, sizeof(char) * I2C_NAME_SIZE);
	new_client->addr = addr;
        new_client->adapter = adap;
        new_client->driver = &starci2win_driver;


	i2c_attach_client(new_client);

	/* Init code here */

	if ((state = kmalloc(sizeof(struct starci2win_state), GFP_KERNEL)) == NULL)
		return -ENOMEM;

	state->addr = new_client->addr;
	state->i2c = new_client->adapter;

    dprintk("%s(): Client attached\n", __FUNCTION__);
	return 0;
}


int attach_adapter(struct i2c_adapter *adap)
{
	dprintk("Inside %s()\n", __FUNCTION__);
	return i2c_probe(adap, &addr_data, client_attach);
}

int detach_client(struct i2c_client *client)
{
	dprintk("%s(): Detaching client\n", __FUNCTION__);
	i2c_detach_client(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}

/* For generic ops */
int starci2win_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
    dprintk("Inside %s()\n", __FUNCTION__);
    switch (cmd) {
		default:
			return -1;
	}
    return 0;
}

