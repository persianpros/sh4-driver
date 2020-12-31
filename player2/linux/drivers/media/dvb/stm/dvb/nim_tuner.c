/*
 * @file nim_tuner.c
 * @brief Driver for autodetecting and initializing NIM tuners
 * 		Autodetect tuners in any of the available i2c slots.
 *		adapter0 will be 'given' to the first tuner found,
 *		adapter1 to the second, ...
 *
 * Copyright (c) 2010 Duolabs spa
 *
 * @author Pedro Aguilar <pedro@duolabs.com>
 *
 * Based on st-tuner.c:
 *
 * Copyright (c) STMicroelectronics 2005
 *
 * Author:Peter Bennett <peter.bennett@st.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/i2c.h>
#include <linux/stm/stm-frontend.h>

#if defined (CONFIG_KERNELVERSION)
#include <linux/stm/pio.h>
#else /* STLinux 2.2 kernel */
#include <linux/stpio.h>
#define IRQF_DISABLED SA_INTERRUPT
#endif

#if defined(QBOXHD) || defined(QBOXHD_MINI)

#if !defined(QBOXHD_MINI)
#define CONFIG_DVB_CX24116
#define CONFIG_DVB_ZL10353
#endif

#define CONFIG_DVB_AVL2108
#define CONFIG_DVB_TDA10023
#define CONFIG_DVB_TDA10048

#ifdef CONFIG_DVB_CX24116
#include "cx24116.h"
#endif

#ifdef CONFIG_DVB_AVL2108
#include "avl2108.h"
#endif

#ifdef CONFIG_DVB_ZL10353
#include "zl10353.h"
#endif

#ifdef CONFIG_DVB_TDA10048
#include "tda10048.h"
#include "tda18218.h"
#endif

#ifdef CONFIG_DVB_TDA10023
#include "tda1002x.h"
#endif

#include "nim_tuner.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activates debugging (default:0)");

#define NIMTUNER_VERSION       "0.0.1"
MODULE_VERSION(NIMTUNER_VERSION);


#define dprintk(args...) 							\
	do { 											\
		if (debug) 									\
			printk(KERN_INFO "nim_tuner: " args); 	\
	} while (0)

#define err_printk(args...)  					\
	do {      								\
		printk("nim_tuner: ERROR: " args);	\
	} while (0)

#define READ_BIT2(array,offset,bitstart,bitsize)	\
	((array[offset] >> bitstart) & ((1<<bitsize)-1))

#define WRITE_BIT2(array,offset,bitstart,bitsize,value) \
	array[offset] = (array[offset] & (~(((1<<bitsize)-1) << bitstart))) | (((value) & ((1<<bitsize)-1) )<< bitstart)

#define READ_BIT(array,pos) READ_BIT2(array,pos)
#define WRITE_BIT(array,pos,value) WRITE_BIT2(array,pos,value)

#if defined(QBOXHD_MINI)

//#define LNBP21
#undef LNBP21

#define LNBA8293
//#undef LNBA8293

/* System register bits for lnbp21*/
#ifdef LNBP21
	#define LNBP21_OLF	0x01
	#define LNBP21_OTF	0x02
	#define LNBP21_EN	0x04
	#define LNBP21_VSEL	0x08
	#define LNBP21_LLC	0x10
	#define LNBP21_TEN	0x20
	#define LNBP21_ISEL	0x40
	#define LNBP21_PCL	0x80
#else //lnba8293
	#define REG_LNB_VOLTAGE	0x00
	#define LNBA8293_VSEL_0	0x01
	#define LNBA8293_VSEL_1	0x02
	#define LNBA8293_VSEL_2	0x04
	#define LNBA8293_VSEL_3	0x08
	#define LNBA8293_ODT	0x10
	#define LNBA8293_EN		0x20

	#define REG_TONE		0xC0
	#define TMODE			0x01
	#define TGATE			0x02
#endif

#endif
/******************************
 * Internal structure for tuners
 ****************************/

struct st_tuner {
	int type;    /**< 0 = None, 1 = i2c, 2 = spi (not supported yet) */
	struct i2c_adapter *i2c_bus;
	u8     i2c_addr_demod;
	u8     i2c_addr_tuner;
	u8     i2c_addr_lnb;

	int srate;
};

struct i2c_detect_results
{
	int 				demod_type;		/**< enum plat_demod_model */
	int 				nim;			/**< NIM in which the demod/tuner is */

	struct i2c_adapter *demod_adapter;	/**< demod's i2c adapter */
	u8                  demod_addr;		/**< demod's i2c address */

	int 				tuner_type;		/**< enum plat_tuner_model */
	u8                  tuner_addr;		/**< tuner's i2c address */

	int lnb_type;						/**< LNB type */
#if defined(QBOXHD_MINI)
  struct i2c_adapter *lnb_adapter;
  u8                  lnb_addr;
#endif
	struct stpio_pin   *lnb_voltage_pio;/**< GPIO for LNB's voltage */
	struct stpio_pin   *lnb_power_pio;	/**< GPIO for LNB's power */

	u8 					attached;		/**< Flag that indicates if the fe is attached of not */
};

static int tsm_channels_in_use[NIMS_NUM_MAX];
static int tuners_num = 0;
static int i2c_bus_count;
static struct plat_frontend_config *i2c_config;
static struct i2c_detect_results results[NIMS_NUM_MAX];
struct nim_info_st nim_info[NIMS_NUM_MAX];


#if defined(QBOXHD_MINI)
#ifdef LNBP21
struct lnbp21 {
	u8			config;
	u8			override_or;
	u8			override_and;
	struct i2c_adapter	*i2c;
};

static int lnbp21_set_voltage(struct dvb_frontend *fe, struct i2c_detect_results *rs, fe_sec_voltage_t voltage)
{

	u8 config=0;
	config &= ~(LNBP21_VSEL | LNBP21_EN);

	switch(voltage) {
	case SEC_VOLTAGE_OFF:
		break;
	case SEC_VOLTAGE_13:
		config |= LNBP21_EN;
		break;
	case SEC_VOLTAGE_18:
		config |= (LNBP21_EN | LNBP21_VSEL);
		break;
	default:
		return -EINVAL;
	};

	u8 data[] = { config };

	struct i2c_msg msg = {	.addr = rs->lnb_addr, .flags = 0,
				.buf = &config,
				.len = sizeof(config) };

	return (i2c_transfer(rs->lnb_adapter, &msg, 1) == 1) ? 0 : -EIO;
}
#else //lnba8293
struct lnba8293 {
	u8			config;
	u8			override_or;
	u8			override_and;
	struct i2c_adapter	*i2c;
};

static int lnb_write_i2c(struct i2c_detect_results *rs, u8 value)
{
	struct i2c_msg msg = {	.addr = rs->lnb_addr, .flags = 0,
				.buf = &value,
				.len = sizeof(value) };

	return (i2c_transfer(rs->lnb_adapter, &msg, 1) == 1) ? 0 : -EIO;
}



static int lnba8293_set_voltage(struct dvb_frontend *fe, struct i2c_detect_results *rs, fe_sec_voltage_t voltage)
{

	u8 config=0;
	config = (REG_LNB_VOLTAGE|LNBA8293_ODT);
	config &= ~(LNBA8293_EN|LNBA8293_VSEL_0|LNBA8293_VSEL_3);

	switch(voltage) {
	case SEC_VOLTAGE_OFF:
		break;
	case SEC_VOLTAGE_13:
		config |= (LNBA8293_VSEL_0 | LNBA8293_EN); // v3=0 v2=0 v1=0 v0=1 -> 13.042V
		break;
	case SEC_VOLTAGE_18:
		config |= (LNBA8293_VSEL_3 | LNBA8293_EN);	// v3=1 v2=0 v1=0 v0=0 -> 18.042V
		break;
	default:
		return -EINVAL;
	};

/*
	u8 data[] = { config };

	struct i2c_msg msg = {	.addr = rs->lnb_addr, .flags = 0,
				.buf = &config,
				.len = sizeof(config) };

	return (i2c_transfer(rs->lnb_adapter, &msg, 1) == 1) ? 0 : -EIO;
*/
	return lnb_write_i2c(rs,config);
}

#endif


static int lnb_i2c_set_voltage(struct dvb_frontend *fe, struct i2c_detect_results *rs, fe_sec_voltage_t voltage)
{
#ifdef LNBP21
	return lnbp21_set_voltage(fe,rs,voltage);
#else	//lnba8293
	return lnba8293_set_voltage(fe,rs,voltage);
#endif
}

#endif


/*******************************
 * LNB
 *****************************/

#ifdef CONFIG_DVB_CX24116
static int cx24116_lnbp20_set_tone(struct dvb_frontend* fe, fe_sec_tone_mode_t tone)
{
	int ret;

	if ((ret = cx24116_set_tone(fe, tone)) < 0) {
		err_printk("%s(): Could not set tone\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int cx24116_lnbp20_set_voltage (struct dvb_frontend* fe, fe_sec_voltage_t voltage)
{
	int n, i2c_nim, tuner_found = 0;

	i2c_nim = cx24116_set_voltage(fe, voltage);
	dprintk("---> %s(): i2c_nim: %d\n", __func__, i2c_nim);
	for (n = 0; n < tuners_num; n++) {
		if (i2c_nim == results[n].nim) {
			tuner_found = 1;
			break;
		}
	}

	if (!tuner_found)
		return 0;

	switch (voltage) {
		case SEC_VOLTAGE_13:
			stpio_set_pin(results[n].lnb_voltage_pio, 1);
			stpio_set_pin(results[n].lnb_power_pio, 0);
			break;
		case SEC_VOLTAGE_18:
			stpio_set_pin(results[n].lnb_voltage_pio, 0);
			stpio_set_pin(results[n].lnb_power_pio, 0);
			break;
		case SEC_VOLTAGE_OFF:
		default:
			stpio_set_pin(results[n].lnb_power_pio, 1);
			break;
	}
	dprintk("%s(): %s\n", __func__,
			voltage == SEC_VOLTAGE_13 ? "SEC_VOLTAGE_13" :
			voltage == SEC_VOLTAGE_18 ? "SEC_VOLTAGE_18" :
			voltage == SEC_VOLTAGE_OFF ? "SEC_VOLTAGE_OFF" : "??");

	return 0;
}
#endif

#ifdef CONFIG_DVB_AVL2108
static int avl2108_lnbp20_set_tone(struct dvb_frontend* fe, fe_sec_tone_mode_t tone)
{
	int ret;

	if ((ret = avl2108_set_tone(fe, tone)) < 0) {
		err_printk("%s(): Could not set tone\n", __func__);
		return -EINVAL;
	}

	return 0;
}

#if defined(QBOXHD_MINI)
static int avl2108_lnb_i2c_set_voltage (struct dvb_frontend* fe, fe_sec_voltage_t voltage)
{
	int n, i2c_nim, tuner_found = 0;
	
	i2c_nim = avl2108_set_voltage(fe, voltage);
	dprintk("%s(): i2c_nim: %d\n", __func__, i2c_nim);
	for (n = 0; n < tuners_num; n++) {
		if (i2c_nim == results[n].nim) {
			tuner_found = 1;
			break;
		}
	}

	if (!tuner_found)
		return 0;

	switch(voltage) {
        case SEC_VOLTAGE_13:
			lnb_i2c_set_voltage(fe,&results[n],SEC_VOLTAGE_13);
            break;
        case SEC_VOLTAGE_18:
			lnb_i2c_set_voltage(fe,&results[n],SEC_VOLTAGE_18);
            break;
        case SEC_VOLTAGE_OFF:
        default:
			lnb_i2c_set_voltage(fe,&results[n],SEC_VOLTAGE_OFF);
            break;
	}
	dprintk("%s(): %s\n", __func__,
		voltage == SEC_VOLTAGE_13 ? "SEC_VOLTAGE_13" : 
		voltage == SEC_VOLTAGE_18 ? "SEC_VOLTAGE_18" : 
		voltage == SEC_VOLTAGE_OFF ? "SEC_VOLTAGE_OFF" : "??");

	return 0;
}
#else
static int avl2108_lnbp20_set_voltage (struct dvb_frontend* fe, fe_sec_voltage_t voltage)
{
	int n, i2c_nim, tuner_found = 0;

	i2c_nim = avl2108_set_voltage(fe, voltage);
	dprintk("---> %s(): i2c_nim: %d\n", __func__, i2c_nim);
	for (n = 0; n < tuners_num; n++) {
		if (i2c_nim == results[n].nim) {
			tuner_found = 1;
			break;
		}
	}

	if (!tuner_found)
		return 0;

	switch (voltage) {
		case SEC_VOLTAGE_13:
			stpio_set_pin(results[n].lnb_voltage_pio, 1);
			stpio_set_pin(results[n].lnb_power_pio, 0);
			break;
		case SEC_VOLTAGE_18:
			stpio_set_pin(results[n].lnb_voltage_pio, 0);
			stpio_set_pin(results[n].lnb_power_pio, 0);
			break;
		case SEC_VOLTAGE_OFF:
		default:
			stpio_set_pin(results[n].lnb_power_pio, 1);
			break;
	}
	dprintk("%s(): %s\n", __func__,
			voltage == SEC_VOLTAGE_13 ? "SEC_VOLTAGE_13" :
			voltage == SEC_VOLTAGE_18 ? "SEC_VOLTAGE_18" :
			voltage == SEC_VOLTAGE_OFF ? "SEC_VOLTAGE_OFF" : "??");

	return 0;
}
#endif
#endif

#ifdef CONFIG_DVB_ZL10353
static int zl10353_lnb_set_voltage (struct dvb_frontend* fe, fe_sec_voltage_t voltage)
{
	int n, i2c_nim, tuner_found = 0;

	i2c_nim = zl10353_set_voltage(fe, voltage);
	dprintk("---> %s(): i2c_nim: %d\n", __func__, i2c_nim);
	for (n = 0; n < tuners_num; n++) {
		if (i2c_nim == results[n].nim) {
			tuner_found = 1;
			break;
		}
	}

	if (!tuner_found)
		return 0;

	switch (voltage) {
		case SEC_VOLTAGE_13:
			stpio_set_pin(results[n].lnb_power_pio, 1);
			break;
		case SEC_VOLTAGE_OFF:
		default:
			stpio_set_pin(results[n].lnb_power_pio, 0);
			break;
	}
	dprintk("%s(): %s\n", __func__,
			voltage == SEC_VOLTAGE_13 ? "SEC_VOLTAGE_13" :
			voltage == SEC_VOLTAGE_OFF ? "SEC_VOLTAGE_OFF" : "??");

	return 0;
}
#endif


/******************************
 * I2C
 *****************************/

static u8 i2c_readreg(struct i2c_adapter *adapter, u8 addr, u8 reg)
{
	u8 b0 [] = { reg };
	u8 b1 [] = { 0 };
	struct i2c_msg msg [] = { { .addr = addr, .flags = 0, .buf = b0, .len = 1 },
		{ .addr = addr, .flags = I2C_M_RD, .buf = b1, .len = 1 } };
	int ret;

	ret = i2c_transfer(adapter, msg, 2);
	if (ret != 2) {
		err_printk(KERN_ERR "DVB: %s: readreg error (reg == 0x%02x, ret == %i)\n",
				__func__, reg, ret);
	}
	return b1[0];
}

#ifdef CONFIG_DVB_TDA10023
static int i2c_writereg(struct i2c_adapter *adapter, u8 addr, u8 reg, u8 data)
{
	u8 buf[] = { reg, data };
	struct i2c_msg msg = { .addr = addr, .flags = 0, .buf = buf, .len = 2 };
	int ret;

	ret = i2c_transfer(adapter, &msg, 1);
	if (ret != 1) {
		err_printk(KERN_ERR "DVB: %s, writereg error (reg == 0x%02x, val == 0x%02x, ret == %i)\n",
				__func__, reg, data, ret);
	}
	return (ret != 1) ? -EREMOTEIO : 0;
}
#endif

static int st_i2c_autodetect (struct i2c_adapter *adapter, u8 i2c_addr, u8 dev_addr)
{
	u8 buf[2] = { 0, 0 };
	struct i2c_msg msg[] = {
		{ .addr = i2c_addr, .flags = 0, .buf = &dev_addr, .len = 1 },
		{ .addr = i2c_addr, .flags = I2C_M_RD, .buf = buf, .len = 1 }
	};
	int b;

	/*printk("[%s] adr %02X, id %02x\n", __func__, i2c_addr, adapter->id);*/
	b = i2c_transfer(adapter,msg,1);
	/*printk("[%s] i2c_transfer=%02X\n", __func__, b);*/
	b |= i2c_transfer(adapter,msg+1,1);
	/*printk("[%s] i2c_transfer=%02X\n", __func__, b);*/

	if (b != 1)
		return -1;

	/*printk("[%s] buf[0]=%02X\n", __func__, buf[0]);*/
	return buf[0];
}

/******************************
 * NIMs funcs
 *****************************/

static void nim_info_init(void)
{
    int i;

    for (i = 0; i < NIMS_NUM_MAX; i++) {
        nim_info[i].type[0] = nim_info[i].name[0] = '\0';
		nim_info[i].nim_pos = -1;
	}
}

static void nim_info_add(const char *type, const char *name, int nim_pos)
{
    int i;

    for (i = 0; i < NIMS_NUM_MAX; i++) {
        if ((!strlen(nim_info[i].type)) || (!strlen(nim_info[i].name))) {
            sprintf(nim_info[i].type, "%s", type);
            sprintf(nim_info[i].name, "%s", name);
            nim_info[i].nim_pos = nim_pos;
            break;
        }
    }
}

int get_nims_info(struct nim_info_st *nims_info)
{
	int i;

    for (i = 0; i < NIMS_NUM_MAX; i++)
		memcpy(nims_info + i, &nim_info[i], sizeof(struct nim_info_st));

	return 0;
}
EXPORT_SYMBOL(get_nims_info);

int get_nims_num_found(int *tuners_found)
{
	*tuners_found = tuners_num;
	return 0;
}
EXPORT_SYMBOL(get_nims_num_found);


int get_nims_config(struct plat_frontend_config * config)
{
	memcpy(config, i2c_config, sizeof(*i2c_config));
	return 0;
}
EXPORT_SYMBOL(get_nims_config);

static int set_tsm_channel(u8 tuners_num, enum plat_frontend_options *option)
{
	int ret = -1;

	switch (tuners_num) {
		case 0:
			*option |= STM_TSM_CHANNEL_0;
			ret = 0;
			break;
		case 1:
			*option |= STM_TSM_CHANNEL_1;
			ret = 1;
			break;
		case 2:
			*option |= STM_TSM_CHANNEL_2;
			ret = 2;
			break;
		case 3:
			*option |= STM_TSM_CHANNEL_3;
			ret = 3;
			break;
		default:
			break;
	}

	return ret;
}

int nim_tuner_attach_frontend(struct dvb_adapter *dvb_adap, int nr_adapters)
{
	int n = 0;

	/*for (n = 0; n < NIMS_NUM_MAX; n++) {*/
	/*printk("results[%d]: demod model: 0x%X, tuner model: 0x%X, nim: %d\n", */
	/*n, results[n].demod_type, results[n].tuner_type, results[n].nim);*/
	/*}*/

	struct dvb_frontend* new_fe = NULL;

	if (results[nr_adapters].attached == 1) {
		dprintk("Frontend is already attached. Skipping...\n");
		return 0;
	}

	if (results[nr_adapters].demod_type == STM_DEMOD_AUTO) {
		dprintk("Empty slot %d\n", n);
		/*continue;*/
		return 0;
	}

#ifdef CONFIG_DVB_CX24116
	if (results[nr_adapters].demod_type == STM_DEMOD_CX24116 && results[nr_adapters].tuner_type == STM_TUNER_CX24116) {
		struct cx24116_config *tmp = (struct cx24116_config*)kmalloc(sizeof(struct cx24116_config),GFP_KERNEL);

		tmp->demod_address = results[nr_adapters].demod_addr;
		tmp->set_ts_params = NULL;
		tmp->reset_device = NULL;

		new_fe = cx24116_attach(tmp,results[nr_adapters].demod_adapter);

		results[nr_adapters].attached = 1;

		if ((results[nr_adapters].lnb_type & STM_LNB_LNBP20) && new_fe ) {
#if defined (CONFIG_KERNELVERSION)
			new_fe->ops.set_tone    = cx24116_lnbp20_set_tone;
			new_fe->ops.set_voltage = cx24116_lnbp20_set_voltage;
#endif
			printk("CX24116: Added LNBP20 support\n");
		}
	}
#endif

#ifdef CONFIG_DVB_AVL2108
	if (results[nr_adapters].demod_type == STM_DEMOD_AVL2108 && results[nr_adapters].tuner_type == STM_TUNER_STV6306) {
		struct avl2108_config *tmp = (struct avl2108_config *)kmalloc(sizeof(struct avl2108_config), GFP_KERNEL);

		tmp->demod_address = results[nr_adapters].demod_addr;
		tmp->tuner_address = 0xC0;
		tmp->demod_freq = pll_conf[5].demod_freq;
		tmp->fec_freq = pll_conf[5].fec_freq;
		tmp->mpeg_freq = pll_conf[5].mpeg_freq;

		new_fe = avl2108_attach(tmp, results[nr_adapters].demod_adapter);

		results[nr_adapters].attached = 1;

		if ((results[nr_adapters].lnb_type & STM_LNB_LNBP20) && new_fe ) {
#if defined (CONFIG_KERNELVERSION)
			new_fe->ops.set_tone    = avl2108_lnbp20_set_tone;
#if defined(QBOXHD_MINI)
					new_fe->ops.set_voltage = avl2108_lnb_i2c_set_voltage;
#else
			new_fe->ops.set_voltage = avl2108_lnbp20_set_voltage;
#endif
#endif 
			printk("AVL2108: Added LNBP20 support\n");
		}
	}
#endif

#ifdef CONFIG_DVB_ZL10353
	if (results[nr_adapters].demod_type == STM_DEMOD_ZL10353 && results[nr_adapters].tuner_type == STM_TUNER_ZL10353) {
		struct zl10353_config *tmp = (struct zl10353_config*)kmalloc(sizeof(struct zl10353_config), GFP_KERNEL);

		tmp->demod_address = results[nr_adapters].demod_addr;
		tmp->adc_clock = 453300;
		tmp->if2 = 361667;
		tmp->no_tuner = 0;
		tmp->parallel_ts = 0;
		tmp->disable_i2c_gate_ctrl = 1;
		tmp->clock_ctl_1 = 0;
		tmp->pll_0 = 0;

		new_fe = zl10353_attach(tmp, results[nr_adapters].demod_adapter);
#if defined (CONFIG_KERNELVERSION)
#if defined(QBOXHD)
		new_fe->ops.set_voltage = zl10353_lnb_set_voltage;
#endif
#endif
		results[nr_adapters].attached = 1;
	}
#endif

#ifdef CONFIG_DVB_TDA10048
	if (results[nr_adapters].demod_type == STM_DEMOD_TDA10048 && results[nr_adapters].tuner_type == STM_TUNER_TDA10048) {
		struct tda10048_config *demod = (struct tda10048_config*)kmalloc(sizeof(struct tda10048_config), GFP_KERNEL);
		struct tda18218_config *tuner = (struct tda18218_config*)kmalloc(sizeof(struct tda18218_config), GFP_KERNEL);

		memset(demod, 0, sizeof(struct tda10048_config));
		demod->demod_address = results[nr_adapters].demod_addr;
		demod->clk_freq_khz = TDA10048_CLK_16000;
		demod->dtv6_if_freq_khz = TDA10048_IF_3000; /* 3000*2 = 6MHz */
		demod->dtv7_if_freq_khz = TDA10048_IF_3500; /* 3500*2 = 7MHz */
		demod->dtv8_if_freq_khz = TDA10048_IF_4000; /* 4000*2 = 8MHz */
		demod->inversion = TDA10048_INVERSION_ON; /* so it locks */
		demod->output_mode = TDA10048_PARALLEL_OUTPUT;
		new_fe = tda10048_attach(demod, results[nr_adapters].demod_adapter);

		memset(tuner, 0, sizeof(struct tda18218_config));
		tuner->i2c_address = 0x60;
		tuner->loop_through = 1;
		tda18218_attach(new_fe, results[nr_adapters].demod_adapter, tuner);

		results[nr_adapters].attached = 1;
	}
#endif

#ifdef CONFIG_DVB_TDA10023
	if (results[nr_adapters].demod_type == STM_DEMOD_TDA10023 && results[nr_adapters].tuner_type == STM_TUNER_CU1216LS) {
		struct tda10023_config *tmp = (struct tda10023_config*)kmalloc(sizeof(struct tda10023_config), GFP_KERNEL);
		//struct tda1002x_config *tmp = (struct tda1002x_config*)kmalloc(sizeof(struct tda1002x_config), GFP_KERNEL);

		tmp->demod_address = results[nr_adapters].demod_addr;
		tmp->xtal = 28920000;
		tmp->pll_m = 8;
		tmp->pll_p = 4;
		tmp->pll_n = 1;

		new_fe = tda10023_attach(tmp, results[nr_adapters].demod_adapter, 0);

		results[nr_adapters].attached = 1;
	}
#endif

	if (new_fe) {
		dprintk("%d: Registering fe 0x%X\n", n, results[nr_adapters].demod_type);
		dvb_register_frontend(dvb_adap, new_fe);
	}

	if (new_fe && results[nr_adapters].tuner_type) {
		struct st_tuner *tuner = (struct st_tuner*)kmalloc(sizeof(struct st_tuner),GFP_KERNEL);

		tuner->type = 1;
		tuner->i2c_addr_demod = results[nr_adapters].demod_addr;
		tuner->i2c_addr_tuner = results[nr_adapters].tuner_addr;
	}

	dprintk ("%s <\n", __func__);

	return 0;
}
EXPORT_SYMBOL(nim_tuner_attach_frontend);


/* Autodetecting tuner's is not as easy as it might sound */
static int nim_tuner_i2c_attach(struct i2c_adapter *adapter)
{
	int n, ret;

	i2c_bus_count++;

	dprintk("\n\n===== i2c bus %d =====\n\n", i2c_bus_count);
	for (n = 0; n < i2c_config->nr_nims; n++) {
		struct plat_frontend_channel *channel = &i2c_config->nims[n];

		dprintk("%d: Searching tuner 0x%X\n", n, channel->demod_model);
		/* The DVB-T CU1216LS and the DVB-S2 AVL2108 have the same i2c address: 0x0C
		 * For recognizing them, we check the chip id of the CU1216LS.
		 * If this matches, we have a CU1216LS, otherwise we have an AVL2108 */
#ifdef CONFIG_DVB_TDA10023
			if (channel->demod_model == STM_DEMOD_TDA10023 && channel->tuner_model == STM_TUNER_CU1216LS) {
				u8 addr = 0x0C;

				if (channel->demod_i2c_address != 0xff)
					addr = channel->demod_i2c_address;

				if (st_i2c_autodetect(adapter, addr, 0x0) != -1 ) {
					/* Wakeup if in standby */
					i2c_writereg (adapter, addr, 0x00, 0x2b);
					/* check if the demod is there */
					if ((i2c_readreg(adapter, addr, 0x1a) & 0xf0) == 0x70) {
						results[tuners_num].demod_type    = STM_DEMOD_TDA10023;
						results[tuners_num].nim    		  = adapter->nr;
						results[tuners_num].demod_adapter = adapter;
						results[tuners_num].demod_addr    = addr;

						results[tuners_num].tuner_type    = STM_TUNER_CU1216LS;
						results[tuners_num].tuner_addr    = 0x0C;

						if ((ret = set_tsm_channel(tuners_num, &channel->option)) < 0) {
							err_printk("Unsupported TSM channel '%d'. Ignoring tuner in NIM#%d",
									ret, i2c_bus_count);
							tuners_num++;
							return 0;
						}
						tsm_channels_in_use[tuners_num] = ret;
						dprintk("Setting TSM CHANNEL %d for STM_DEMOD_TDA10023\n", ret);
						printk("DVB-C CU1216LS found on NIM#%d :)\n", i2c_bus_count);

						nim_info_add("DVB-C", "TDA10023", i2c_bus_count);
						tuners_num++;

						return 0;
					}
				}
			}
#endif

#ifdef CONFIG_DVB_CX24116
		if (channel->demod_model == STM_DEMOD_CX24116 && channel->tuner_model == STM_TUNER_CX24116) {
			u8 addr = 0x0A>>1;
			if (channel->demod_i2c_address != 0xff)
				addr = channel->demod_i2c_address;

			if (st_i2c_autodetect(adapter, addr, 0x0) != -1 ) {
				results[tuners_num].demod_type    = STM_DEMOD_CX24116;
				results[tuners_num].nim    		  = adapter->nr;
				results[tuners_num].demod_adapter = adapter;
				results[tuners_num].demod_addr    = addr;

				results[tuners_num].tuner_type    = STM_TUNER_CX24116;
				results[tuners_num].tuner_addr    = 0xAA>>1;

				results[tuners_num].lnb_type      = channel->option & STM_LNB_LNBP20;

				if ((ret = set_tsm_channel(tuners_num, &channel->option)) < 0) {
					err_printk("Unsupported TSM channel '%d'. Ignoring tuner in NIM#%d",
							ret, i2c_bus_count);
					tuners_num++;
					return 0;
				}
				tsm_channels_in_use[tuners_num] = ret;
				dprintk("Setting TSM CHANNEL %d for STM_DEMOD_CX24116\n", ret);
				printk("DVB-S2 CX24116 found on NIM#%d :)\n", i2c_bus_count);

				/* LNB init */
				// If tuner is in I2C 0 we must use GPIOs 5[0] and 5[2]
				// If tuner is in I2C 1 we must use GPIOs 3[4] and 3[5]
				// If tuner is in I2C 2 we must use GPIOs 1[6] and 3[7]
				// WARNING In QBoxHD hw rev < 0.3 the GPIO 3[7] is used by stmmac: I2C 2 CANNOT be used for DVB-S2
				if (i2c_bus_count == 0)
					results[tuners_num].lnb_power_pio = stpio_request_pin(5, 0, "LNB_POWER_PIN", STPIO_OUT);
				else if (i2c_bus_count == 1)
					results[tuners_num].lnb_power_pio = stpio_request_pin(3, 4, "LNB_POWER_PIN", STPIO_OUT);
				else
					results[tuners_num].lnb_power_pio = stpio_request_pin(1, 6, "LNB_POWER_PIN", STPIO_OUT);

				if (results[tuners_num].lnb_power_pio == NULL)
					err_printk("CX24116: Failed while acquiring LNB power (slot %d)\n", i2c_bus_count);
				else
					printk("CX24116: LNB power acquired (slot %d)\n", i2c_bus_count);


				if (i2c_bus_count == 0)
					results[tuners_num].lnb_voltage_pio = stpio_request_pin(5, 2, "LNB_VOLTAGE_PIN", STPIO_OUT);
				else if (i2c_bus_count == 1)
					results[tuners_num].lnb_voltage_pio = stpio_request_pin(3, 5, "LNB_VOLTAGE_PIN", STPIO_OUT);
				else
					results[tuners_num].lnb_voltage_pio = stpio_request_pin(3, 7, "LNB_VOLTAGE_PIN", STPIO_BIDIR);

				if (results[tuners_num].lnb_voltage_pio == NULL)
					err_printk("CX24116: Failed while acquiring LNB voltage (slot %d)\n", i2c_bus_count);
				else
					printk("CX24116: LNB power voltage (slot %d)\n", i2c_bus_count);

				dprintk("results[%d]: demod model: 0x%X, tuner model: 0x%X, nim: %d\n",
					tuners_num, results[tuners_num].demod_type, results[tuners_num].tuner_type, results[tuners_num].nim);

				nim_info_add("DVB-S2", "CX24116", i2c_bus_count);
				tuners_num++;

				return 0;
			}
		}
#endif

#ifdef CONFIG_DVB_AVL2108
		if (channel->demod_model == STM_DEMOD_AVL2108 && channel->tuner_model == STM_TUNER_STV6306) {
			u8 addr = 0x0C;
			if (channel->demod_i2c_address != 0xff)
				addr = channel->demod_i2c_address;

			if (st_i2c_autodetect(adapter, addr, 0x0) != -1 ) {
				if ((i2c_readreg(adapter, addr, 0x1a) & 0xf0) == 0x70) {
					continue;
				}
				results[tuners_num].demod_type    = STM_DEMOD_AVL2108;
				results[tuners_num].nim    		  = adapter->nr;
				results[tuners_num].demod_adapter = adapter;
				results[tuners_num].demod_addr    = addr;

				results[tuners_num].tuner_type    = STM_TUNER_STV6306;
				results[tuners_num].tuner_addr    = 0x0C;

				results[tuners_num].lnb_type      = channel->option & STM_LNB_LNBP20;

#if defined(QBOXHD_MINI)
				results[tuners_num].lnb_adapter   = adapter;

				if(st_i2c_autodetect(adapter,0x0B,0x0)!=-1)
				{
#ifdef LNBP21
						printk("Found i2c addr of LNBP21\n");		
#else
						printk("Found i2c addr of LNBA8293\n");		
#endif	
						results[tuners_num].lnb_addr      = 0x0B; //channel->lnb_i2c_address;
#ifndef LNBP21
					{
						u8 val=0;
						/* For lnba8293 set the tone in external 22kHz */
						dprintk("LNBP21: Enable 22kHz for lnba8293\n");
						/* Addr 0x11 */
						val=(REG_TONE|TGATE); 	/* Enable the option of TMODE */
						val&=(~TMODE); 			/* Enable the external 22kHz */
						dprintk("LNBP21: Val: 0x%02X\n",val);
						lnb_write_i2c(&results[tuners_num],val);

						/* Addr 0x10 */
						val=0;
						val=(0x80|TGATE); 		/* Enable the option of TMODE */
						val&=(~TMODE); 			/* Enable the external 22kHz */
						dprintk("LNBP21: Val: 0x%02X\n",val);
						lnb_write_i2c(&results[tuners_num],val);
					}
#endif
				}
				else
				{
#ifdef LNBP21
						printk("Not found i2c address of LNBP21\n");
#else
						printk("Not found i2c address of LNBA8293\n");
#endif	
					results[tuners_num].lnb_addr=-1;
				}
#endif

				if ((ret = set_tsm_channel(tuners_num, &channel->option)) < 0) {
					err_printk("Unsupported TSM channel '%d'. Ignoring tuner in NUM#%d",
							ret, i2c_bus_count);
					tuners_num++;
					return 0;
				}
				tsm_channels_in_use[tuners_num] = ret;
				dprintk("Setting TSM CHANNEL %d for STM_DEMOD_AVL2108\n", ret);
				printk("DVB-S2 AVL2108 found on NIM#%d :)\n", i2c_bus_count);

				/* LNB init */
				// If tuner is in I2C 0 we must use GPIOs 5[0] and 5[2]
				// If tuner is in I2C 1 we must use GPIOs 3[4] and 3[5]
				// If tuner is in I2C 2 we must use GPIOs 1[6] and 3[7]
				// WARNING In QBoxHD hw rev < 0.3 the GPIO 3[7] is used by stmmac: I2C 2 CANNOT be used for DVB-S2
#if !defined(QBOXHD_MINI)
				if (i2c_bus_count == 0)
					results[tuners_num].lnb_power_pio = stpio_request_pin(5, 0, "LNB_POWER_PIN", STPIO_OUT);
				else if (i2c_bus_count == 1)
					results[tuners_num].lnb_power_pio = stpio_request_pin(3, 4, "LNB_POWER_PIN", STPIO_OUT);
				else
					results[tuners_num].lnb_power_pio = stpio_request_pin(1, 6, "LNB_POWER_PIN", STPIO_OUT);

				if (results[tuners_num].lnb_power_pio == NULL)
					err_printk("AVL2108: Failed while acquiring LNB power (slot %d)\n", i2c_bus_count);
				else
					printk("AVL2108: LNB power acquired (slot %d)\n", i2c_bus_count);


				if (i2c_bus_count == 0)
					results[tuners_num].lnb_voltage_pio = stpio_request_pin(5, 2, "LNB_VOLTAGE_PIN", STPIO_OUT);
				else if (i2c_bus_count == 1)
					results[tuners_num].lnb_voltage_pio = stpio_request_pin(3, 5, "LNB_VOLTAGE_PIN", STPIO_OUT);
				else
					results[tuners_num].lnb_voltage_pio = stpio_request_pin(3, 7, "LNB_VOLTAGE_PIN", STPIO_OUT);

				if (results[tuners_num].lnb_voltage_pio == NULL)
					err_printk("AVL2108: Failed while acquiring LNB voltage (slot %d)\n", i2c_bus_count);
				else
					printk("AVL2108: LNB voltage acquired (slot %d)\n", i2c_bus_count);
#endif
				nim_info_add("DVB-S2", "AVL2108", i2c_bus_count);
				tuners_num++;
				return 0;
			}
		}
#endif

#ifdef CONFIG_DVB_ZL10353
		if (channel->demod_model == STM_DEMOD_ZL10353 && channel->tuner_model == STM_TUNER_ZL10353) {
			u8 addr = 0x0F;

			if (channel->demod_i2c_address != 0xff)
				addr = channel->demod_i2c_address;

			if (st_i2c_autodetect(adapter, addr, 0x0) != -1 ) {
				results[tuners_num].demod_type    = STM_DEMOD_ZL10353;
				results[tuners_num].nim    		  = adapter->nr;
				results[tuners_num].demod_adapter = adapter;
				results[tuners_num].demod_addr    = addr;

				results[tuners_num].tuner_type    = STM_TUNER_ZL10353;
				results[tuners_num].tuner_addr    = 0xC2;

				if ((ret = set_tsm_channel(tuners_num, &channel->option)) < 0) {
					err_printk("Unsupported TSM channel '%d'. Ignoring tuner in NUM#%d",
							ret, i2c_bus_count);
					tuners_num++;
					return 0;
				}
				tsm_channels_in_use[tuners_num] = ret;
				dprintk("Setting TSM CHANNEL %d for STM_DEMOD_ZL10353\n", ret);
				printk("DVB-T ZL10353 found on NIM#%d :)\n", i2c_bus_count);

				/* LNB init */
				// If tuner is in I2C 0 we must use GPIOs 5[0] and 5[2]
				// If tuner is in I2C 1 we must use GPIOs 3[4] and 3[5]
				// If tuner is in I2C 2 we must use GPIOs 1[6] and 3[7]
				// WARNING In QBoxHD hw rev < 0.3 the GPIO 3[7] is used by stmmac: I2C 2 CANNOT be used for DVB-S2
#if defined(QBOXHD)
				if (i2c_bus_count == 0)
					results[tuners_num].lnb_power_pio = stpio_request_pin(5, 0, "LNB_POWER_PIN", STPIO_OUT);
				else if (i2c_bus_count == 1)
					results[tuners_num].lnb_power_pio = stpio_request_pin(3, 4, "LNB_POWER_PIN", STPIO_OUT);
				else
					results[tuners_num].lnb_power_pio = stpio_request_pin(1, 6, "LNB_POWER_PIN", STPIO_OUT);

				if (results[tuners_num].lnb_power_pio == NULL)
					err_printk("DVB-T ZL10353: Failed while acquiring LNB power (slot %d)\n", i2c_bus_count);
				else
					printk("DVB-T ZL10353: LNB power acquired (slot %d)\n", i2c_bus_count);
#endif			

				nim_info_add("DVB-T", "ZL10353", i2c_bus_count);
				tuners_num++;

				return 0;
			}
		}
#endif

#ifdef CONFIG_DVB_TDA10048
		if (channel->demod_model == STM_DEMOD_TDA10048 && channel->tuner_model == STM_TUNER_TDA10048) {
			u8 addr = 0x08;

			if (channel->demod_i2c_address != 0xff)
				addr = channel->demod_i2c_address;

			if (st_i2c_autodetect(adapter, addr, 0x0) != -1 ) {
				results[tuners_num].demod_type    = STM_DEMOD_TDA10048;
				results[tuners_num].nim    		  = adapter->nr;
				results[tuners_num].demod_adapter = adapter;
				results[tuners_num].demod_addr    = addr;

				results[tuners_num].tuner_type    = STM_TUNER_TDA10048;
				results[tuners_num].tuner_addr    = 0x08;

				if ((ret = set_tsm_channel(tuners_num, &channel->option)) < 0) {
					err_printk("Unsupported TSM channel '%d'. Ignoring tuner in NIM#%d",
							ret, i2c_bus_count);
					tuners_num++;
					return 0;
				}
				tsm_channels_in_use[tuners_num] = ret;
				dprintk("Setting TSM CHANNEL %d for STM_DEMOD_TDA10048?\n", ret);
				printk("DVB-T TDA10048 found on NIM#%d :)\n", i2c_bus_count);

				nim_info_add("DVB-T", "TDA10048", i2c_bus_count);
				tuners_num++;

				return 0;
			}
		}
#endif
	}

	return 0;
}

static int nim_tuner_i2c_detatch(struct i2c_adapter *adapter)
{
	int n;
	for(n=0;n<i2c_config->nr_nims;n++) {
		if (results[n].demod_type && results[n].demod_adapter == adapter) {

			if(results[n].lnb_power_pio){
				stpio_free_pin(results[n].lnb_power_pio);
			}

			if(results[n].lnb_voltage_pio){
				stpio_free_pin(results[n].lnb_voltage_pio);
			}

			memset(&results[n],0,sizeof(struct i2c_detect_results));
		}
	}

	i2c_bus_count--;
	return 0;
}

/******************************
 * Frontend
 *****************************/


struct i2c_driver tuner_i2c_driver = {
	.id = 129,
	.attach_adapter = nim_tuner_i2c_attach,
	.detach_adapter = nim_tuner_i2c_detatch,
	.command        = 0,
	.driver         = {
		.owner      = THIS_MODULE,
		.name       = "i2c_stm_frontend"
	}
};

/**
 * @brief This function only retrieves the configuration data
 */
static int nim_tuner_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device (dev);
	struct plat_frontend_config *config;

	dprintk ("%s >\n", __func__);

	config = (struct plat_frontend_config *)pdev->dev.platform_data;
	if (config == NULL) {
		err_printk ("%s(): no platform device data found\n", __func__);
		return -1;
	}

	i2c_bus_count = -1;
	i2c_config = config;
	memset(results, 0, sizeof(*results));
	nim_info_init();

	dprintk("Number of NIMs supported: %d\n", config->nr_nims);

	i2c_add_driver(&tuner_i2c_driver);

	return 0;
}


static int nim_tuner_remove(struct device *dev)
{
	dprintk ("%s: not implemented yet\n", __func__);
	i2c_del_driver (&tuner_i2c_driver);

	return 0;
}


static struct device_driver nim_tuner_driver = {
  .name = "nimtuner",
  .bus = &platform_bus_type,
  .owner = THIS_MODULE,
  .probe = nim_tuner_probe,
  .remove = nim_tuner_remove,
};

int __init nim_tuner_init(void)
{
	int ret;

	ret = driver_register(&nim_tuner_driver);
	if (ret < 0) {
    	err_printk ("Could not register driver\n");
		return -1;
	}

    dprintk("Driver loaded\n");
    return 0;
}

void __exit nim_tuner_exit(void)
{
	driver_unregister(&nim_tuner_driver);
	dprintk("Drive unloaded\n");
}

module_init(nim_tuner_init);
module_exit(nim_tuner_exit);

MODULE_DESCRIPTION("Driver for autodetecting and initializing DVB NIMs");
MODULE_AUTHOR("Pedro Aguilar");
MODULE_LICENSE("GPL");

#endif
