/*****************************************************************************
 * Copyright (C)2003 Ali Corporation. All Rights Reserved.
 *
 * File: nim_dev.h
 *
 * Description: This file contains NIM device structure define.
 * History:
 *    Date         Author        Version   Reason
 *    ============ ============= ========= =================
 * 1. Feb.16.2003  Justin Wu     Ver 0.1   Create file.
 * 2. Jun.12.2003  George jiang  Ver 0.2   Porting NIM.
 * 3. Aug.22.2003  Justin Wu     Ver 0.3   Update.
 * 4. Sep.29.2009  DMQ           Ver 0.4   Update.
 *
 *****************************************************************************/

#ifndef __NIM_DEV_H__
#define __NIM_DEV_H__

#include "mn88472_inner.h"

#define EXT_QPSK_MODE_SPI 0
#define EXT_QPSK_MODE_SSI 1

#if (defined(_M36F_SINGLE_CPU) || ((STB_BOARD_TYPE == FULAN_3606_220_1CA_1CI) ||(STB_BOARD_TYPE == FULAN_3606_220_SIMPLE)))
#define NIM_MAX_NUM 1  // dmq add
#else
#define NIM_MAX_NUM 2  // dmq add
#endif
#define NIM_CHIP_ID_M3501A 0x350100C0
#define NIM_CHIP_ID_M3501B 0x350100D0
#define NIM_CHIP_SUB_ID_S3501D 0x00
#define NIM_CHIP_SUB_ID_M3501B 0xC0
#define NIM_CHIP_SUB_ID_M3501C 0xCC
#define NIM_FREQ_RETURN_REAL 0
#define NIM_FREQ_RETURN_SET 1
#define FS_MAXNUM 15000
#define TP_MAXNUM 2000
#define CRYSTAL_FREQ 13500  // 13.5M
//#define CRYSTAL_FREQ 16000 // 16M

/* NIM Device I/O control commands */
enum nim_device_ioctrl_command
{
	NIM_DRIVER_READ_TUNER_STATUS,         /* Read tuner lock status */
	NIM_DRIVER_READ_QPSK_STATUS,          /* Read QPSK lock status */
	NIM_DRIVER_READ_FEC_STATUS,           /* Read FEC lcok status */

	NIM_DRIVER_READ_QPSK_BER,             /* Read QPSK Bit Error Rate */
	NIM_DRIVER_READ_VIT_BER,              /* Read Viterbi Bit Error Rate */
	NIM_DRIVER_READ_RSUB,                 /* Read Reed Solomon Uncorrected block */
	NIM_DRIVER_STOP_ATUOSCAN,             /* Stop autoscan */
	NIM_DRIVER_UPDATE_PARAM,              /* Reset current parameters */
	NIM_DRIVER_TS_OUTPUT,                 /* Enable NIM output TS*/
	NIM_DRIVER_FFT,
	NIM_DRIVER_GET_CR_NUM,
	NIM_DRIVER_GET_CUR_FREQ,
	NIM_DRIVER_FFT_PARA,
	NIM_DRIVER_GET_SPECTRUM,
	NIM_FFT_JUMP_STEP,                    /* Get AutoScan FFT frequency jump step */ //huang hao add for m3327 qpsk
	NIM_DRIVER_READ_COFFSET,              /* Read Carrier Offset state */ //joey add for stv0297.
	NIM_DRIVER_SEARCH_1ST_CHANNEL,        /* Search channel spot */  //joey add for stv0297.
	NIM_DRIVER_SET_TS_MODE,               /* Set ts output mode: */
	/*bit0: 1 serial out, 0 parallel out*/
	/*bit1: 1 clk rising, 0 clk falling*/
	/*bit2: 1 valid gap, 0 valid no gap*/
	NIM_DRIVER_SET_PARAMETERS,            /* Set the parameters of nim,add by Roman at 060321 */
	NIM_DRIVER_SET_RESET_CALLBACK,        /* When nim device need to be reset, call an callback to notice app */
	NIM_DRIVER_ENABLE_DVBS2_HBCD,         /* For DVB-S2, enable/disable HBCD mode */
	NIM_DRIVER_STOP_CHANSCAN,             /* Stop channel change because some low symbol rate TP too long to be locked */
	NIM_DRIVER_RESET_PRE_CHCHG,           /* Reset nim device before channel change */

	NIM_ENABLE_IC_SORTING,                /* Enable IC sorting, set IC sorting param */
	NIM_DRIVER_GET_OSD_FREQ_OFFSET,       /*get OSD freq offset. */
	NIM_DRIVER_SET_RF_AD_GAIN,            /* Set RF ad table for RSSI display.*/
	NIM_DRIVER_SET_IF_AD_GAIN,            /* Set IF ad table for RSSI display. */
	NIM_DRIVER_GET_RF_IF_AD_VAL,          /* get RF IF ad value. */
	NIM_DRIVER_GET_REC_PERFORMANCE_INFO,  /* get receiver performance info. */
	NIM_DRIVER_ENABLE_DEBUG_LOG,          /* enable nim driver debug infor out.*/
	NIM_DRIVER_DISABLE_DEBUG_LOG,         /* disable nim driver debug infor out.*/
	NIM_DRIVER_GET_FRONTEND_STATE,        /* Read front end state.*/
	NIM_DRIVER_SET_FRONTEND_LOCK_CHECK,   /* Set front end lock check flag.*/
	NIM_DRIVER_LOG_IFFT_RESULT,
	NIM_DRIVER_CHANGE_TS_GAP,             /* Change ts gap */
	NIM_DRIVER_SET_SSI_CLK,               /* Set SSI Clock */

	NIM_DRIVER_5V_CONTROL,
	//use io control command to instead of function interface for combo soluation
	NIM_DRIVER_SET_POLAR,                 /* DVB-S NIM Device set LNB polarization */
	NIM_DRIVER_SET_12V,                   /* DVB-S NIM Device set LNB votage 12V enable or not */
	NIM_DRIVER_GET_SYM,                   /* Get Current NIM Device Channel Symbol Rate */
	NIM_DRIVER_GET_BER,                   /* Get Current NIM Device Channel Bit-Error Rate */
	NIM_DRIVER_GET_AGC,                   /* Get Current NIM Device Channel AGC Value */
	NIM_DRIVER_DISABLED,                  /* Disable Current NIM Device */
	NIM_DRIVER_GET_GUARD_INTERVAL,
	NIM_DRIVER_GET_FFT_MODE,
	NIM_DRIVER_GET_MODULATION,
	NIM_DRIVER_GET_SPECTRUM_INV,
	//AD2DMA tool
	NIM_DRIVER_ADC2MEM_START,
	NIM_DRIVER_ADC2MEM_STOP,
	NIM_DRIVER_ADC2MEM_SEEK_SET,
	NIM_DRIVER_ADC2MEM_READ_8K,
	// The following code is for tuner io command
	NIM_TUNER_COMMAND = 0x8000,
	NIM_TUNER_POWER_CONTROL,
	NIM_TUNER_GET_AGC,
	NIM_TUNER_GET_RSSI_CAL_VAL,
	NIM_TUNER_RSSI_CAL_ON,                // RSSI calibration on
	NIM_TUNER_RSSI_CAL_OFF,
	NIM_TUNER_RSSI_LNA_CTL,               // RSSI set LNA
	NIM_TUNER_SET_GPIO,
	NIM_TUNER_CHECK,
	NIM_TUNER_LOOPTHROUGH,                // dmq add
	NIM_DRIVER_SET_BLSCAN_MODE,           // Para = 0, NIM_SCAN_FAST; Para = 1, NIM_SCAN_ACCURATE(slower)
	NIM_TURNER_SET_STANDBY,               /*set av2012 standby add by bill 2012.02.21*/
	NIM_DRIVER_STOP_TUNER_FRONTEND,       // add by rxj 2011/06/10.
	NIM_TUNER_SET_THROUGH_MODE
};

/* NIM Device I/O control command extension */
enum nim_device_ioctrl_command_ext
{
	NIM_DRIVER_AUTO_SCAN            = 0x00FC0000,  /* Do AutoScan Procedure */
	NIM_DRIVER_CHANNEL_CHANGE       = 0x00FC0001,  /* Do Channel Change */
	NIM_DRIVER_CHANNEL_SEARCH       = 0x00FC0002,  /* Do Channel Search */
	NIM_DRIVER_GET_RF_LEVEL         = 0x00FC0003,  /* Get RF level */
	NIM_DRIVER_GET_CN_VALUE         = 0x00FC0004,  /* Get CN value */
	NIM_DRIVER_GET_BER_VALUE        = 0x00FC0005,  /* Get BER value */
	NIM_DRIVER_QUICK_CHANNEL_CHANGE = 0x00FC0006,  /* Do Quick Channel Change without waiting lock */
	NIM_DRIVER_GET_ID               = 0x00FC0007,  /* Get 3501 type: M3501A/M3501B */
	NIM_DRIVER_SET_PERF_LEVEL       = 0x00FC0008,  /* Set performance level */
	NIM_DRIVER_START_CAPTURE        = 0x00FC0009,  /* Start capture */
	NIM_DRIVER_GET_I2C_INFO         = 0x00FC000A,  /* Start capture */
	NIM_DRIVER_GET_FFT_RESULT       = 0x00FC000B,  /* Get Current NIM Device Channel FFT spectrum result */
	NIM_DRIVER_DISEQC_OPERATION     = 0x00FC000C,  /* NIM DiSEqC Device Opearation */
	NIM_DRIVER_DISEQC2X_OPERATION   = 0x00FC000D,  /* NIM DiSEqC2X Device Opearation */
	NIM_DRIVER_SET_NIM_MODE         = 0x00FC000E,  /* NIM J83AC/J83B mode setting */
	NIM_DRIVER_SET_QAM_INFO         = 0x00FC000F
};

enum nim_cofdm_work_mode
{
	NIM_COFDM_SOC_MODE = 0,
	NIM_COFDM_ONLY_MODE = 1
};

enum nim_cofdm_ts_mode
{
	NIM_COFDM_TS_SPI_8B = 0,
	NIM_COFDM_TS_SPI_4B = 1,
	NIM_COFDM_TS_SPI_2B = 2,
	NIM_COFDM_TS_SSI    = 3
};

enum
{
	DEMOD_UNKNOWN,
	DEMOD_DVBC,
	DEMOD_DVBT,
	DEMOD_DVBT2,
};

/* Structure for NIM DiSEqC Device Information parameters */
struct t_diseqc_info
{
	u8 sat_or_tp;  /* 0:sat, 1:tp*/
	u8 diseqc_type;
	u8 diseqc_port;
	u8 diseqc11_type;
	u8 diseqc11_port;
	u8 diseqc_k22;
	u8 diseqc_polar;  /* 0: auto,1: H,2: V */
	u8 diseqc_toneburst;  /* 0: off, 1: A, 2: B */

	u8 positioner_type;  /*0-no positioner 1-1.2 positioner support, 2-1.3 USALS*/
	u8 position;  /*use for DiSEqC1.2 only*/
	u16 wxyz;  /*use for USALS only*/
};

/* Structure for Channel Change parameters */
struct NIM_Channel_Change
{
	u32 freq;          /* Channel Center Frequency: in MHz unit */
	u32 sym;           /* Channel Symbol Rate: in KHz unit */
	u8 fec;            /* Channel FEC rate */
	u32 bandwidth;     /* Channel Symbol Rate: same as Channel Symbol Rate ? -- for DVB-T */
	u8 guard_interval; /* Guard Interval -- for DVB-T */
	u8 fft_mode;       /* -- for DVB-T */
	u8 modulation;     /* -- for DVB-T */
	u8 usage_type;     /* -- for DVB-T */
	u8 inverse;        /* -- for DVB-T */
	u8 priority;       /* -- for DVB-T *///dmq add
	u8 op_mode;        // 0:channel change 1:search mode //dmq add
	u8 priv_param;     // dmq add
	u8 dvb_t2_c;       // add by rxj 0:t2,t 1:c
	u8 PLP_id;         // add for t2
	u8 scan_mode;      // add for t2
};

/* Structure for Channel Search parameters */
struct NIM_Channel_Search
{
	u32 freq;           /* Channel Center Frequency: in MHz unit */
	u32 bandwidth;      /* -- for DVB-T */
	u8 guard_interval;  /* -- for DVB-T */
	u8 fft_mode;        /* -- for DVB-T */
	u8 modulation;      /* -- for DVB-T */
	u8 fec;             /* -- for DVB-T */
	u8 usage_type;      /* -- for DVB-T */
	u8 inverse;         /* -- for DVB-T */
	u8 op_mode;         // 0:channel change 1:search mode //dmq add
	u16 freq_offset;    /* -- for DVB-T */
	u16 symbol;         // dmq add

};

#define HLD_MAX_NAME_SIZE 16 /* Max device name length */

/* Structure nim_device, the basic structure between HLD and LLD of demodulator device */
struct nim_device
{
	struct nim_device *next;  /* Next nim device structure */
	u32 type;  /* Interface hardware type */
	s8 name[HLD_MAX_NAME_SIZE];  /* Device name */
	u32 base_addr;  /* Demodulator address */
	u32 Tuner_select;  /* I2C TYPE for TUNER select */
	u16 flags;  /* Interface flags, status and ability */

	u32 DemodIOHandle[3]; //jhy add

	/* Hardware privative structure */
	void *priv;  /* pointer to private data */
	/* Functions of this dem device */
	int(*init)(void);  /* NIM Device Initialization */
	int(*open)(struct nim_device *dev);  /* NIM Device Open */
	int(*stop)(struct nim_device *dev);  /* NIM Device Stop */
	int(*do_ioctl)(struct nim_device *dev, int cmd, u32 param);  /* NIM Device I/O Control */
	int(*do_ioctl_ext)(struct nim_device *dev, int cmd, void *param_list);  /* NIM Device I/O Control Extension */
	int(*get_lock)(struct nim_device *dev, u8 *lock); /* Get Current NIM Device Channel Lock Status */
	int(*get_freq)(struct nim_device *dev, u32 *freq);  /* Get Current NIM Device Channel Frequency */
	int(*get_FEC)(struct nim_device *dev, u8 *fec);  /* Get Current NIM Device Channel FEC Rate */
	int(*get_SNR)(struct nim_device *dev, u8 *snr);  /* Get Current NIM Device Channel SNR Value */

	u8 diseqc_typex;  /* NIM DiSEqC Device Type */
	u8 diseqc_portx;  /* NIM DiSEqC Device Port */
	struct t_diseqc_info diseqc_info;  /* NIM DiSEqC Device Information Structure */
	int(*set_polar)(struct nim_device *dev, u8 polar); /*DVB-S NIM Device set LNB polarization */
	int(*set_12v)(struct nim_device *dev, u8 flag);  /* DVB-S NIM Device set LNB votage 12V enable or not */
	int(*DiSEqC_operate)(struct nim_device *dev, u32 mode, u8 *cmd, u8 cnt);  /* NIM DiSEqC Device Opearation */
	int(*DiSEqC2X_operate)(struct nim_device *dev, u32 mode, u8 *cmd, u8 cnt, u8 *rt_value, u8 *rt_cnt);  /* NIM DiSEqC2X Device Opearation */
	int(*get_sym)(struct nim_device *dev, u32 *sym);  /* Get Current NIM Device Channel Symbol Rate */
	int(*get_BER)(struct nim_device *dev, u32 *ebr);  /* Get Current NIM Device Channel Bit-Error Rate */
	int(*get_PER)(struct nim_device *dev, u32 *per);  /* Get Current NIM Device Channel Bit-Error Rate */
	int(*get_AGC)(struct nim_device *dev, u8 *agc);   /* Get Current NIM Device Channel AGC Value */
	int(*get_fft_result)(struct nim_device *dev, u32 freq, u32 *start_addr);  /* Get Current NIM Device Channel FFT spectrum result */
	int(*channel_search)(struct nim_device *dev, struct NIM_Channel_Search *param);  // should be opened in DVBS

	const char *(*get_ver_infor)(struct nim_device *dev);  /* Get Current NIM Device Version Number */

	int(*channel_change)(struct nim_device *dev, struct NIM_Channel_Change *param);
	int(*disable)(struct nim_device *dev);  /* Disable Current NIM Device */
	int(*get_guard_interval)(struct nim_device *dev, u8 *gi);
	int(*get_fftmode)(struct nim_device *dev, u8 *fftmode);
	int(*get_modulation)(struct nim_device *dev, u8 *modulation);
	int(*get_spectrum_inv)(struct nim_device *dev, u8 *inv);
	int(*get_freq_offset)(struct nim_device *dev, int *freq_offset);
	int(*get_bandwidth)(struct nim_device *dev, int *bandwith);
	int(*get_frontend_type)(struct nim_device *dev, u8 *system);  // dmq 2011/11/28 add
	int(*get_workmode)(struct nim_device *dev, int *qpsk_mode);
	int(*get_PLP_num)(struct nim_device *dev, u8 *plp_num);
	int(*get_PLP_id)(struct nim_device *dev, u8 *plp_id);

	u32 i2c_type_id;  // dmq modify
	u32 mutex_id;     // dmq add
	u16 nim_idx;      // dmq add tuner1/tuner2
};

#endif  // __NIM_DEV_H__
// vim:ts=4
