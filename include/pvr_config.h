#ifndef _PVR_CONFIG_H_
#define _PVR_CONFIG_H_

struct plat_tuner_config
{
	int adapter;   /* DVB adapter number */
	int i2c_bus;   /* i2c bus number */
	int i2c_addr;  /* demodulator i2c address */
	/* the following arrays define
	   [0] PIO port number
	   [1] PIO pin number
	   [2] active state of the pin (0 - active-low, 1 - active-high
	 */
	int tuner_enable[3];
	int lnb_enable[3];
	/* the following array defines
	   [0] i2c address of LNB power controller
	   [1] value for LNB power 13V (vertical)
	   [2] value for LNB power 18V (horizontal)
	   [3] value for LNB power off
	 */
	int lnb_vsel[4];
};

struct plat_tuner_data
{
	int num_entries;
	struct plat_tuner_config *tuner_cfg;
};
#endif
// vim:ts=4
