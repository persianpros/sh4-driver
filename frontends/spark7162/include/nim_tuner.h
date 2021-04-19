/*****************************************************************************
* Ali Corp. All Rights Reserved. 2005 Copyright (C)
*
* File: nim_tuner.h
*
* Description: This file contains QPSK Tuner Configuration AP Functions
*
* History:
* Date Athor Version Reason
* ============ ============= ========= =================
* 1. Aug.25.2005 Jun Zhu Ver 0.1 Create file.
*****************************************************************************/

#ifndef _NIM_TUNER_H_
#define _NIM_TUNER_H_

#define PANASONIC_DEMO_SUPPORT
/*
Nim_xxxx_attach
Function: int nim_xxxx_attach (QPSK_TUNER_CONFIG_API * ptrQPSK_Tuner)
Description: QPSK Driver Attach function
Parameters: QPSK_TUNER_CONFIG_API * ptrQPSK_Tuner, pointer to structure QPSK_TUNER_CONFIG_API
Return: int, operation status code; configuration successful return with SUCCESS

Nim_Tuner _Init
Function: int nim_Tuner_Init (u32* tuner_id,struct QPSK_TUNER_CONFIG_EXT * ptrTuner_Config);
Description: API function for QPSK Tuner Initialization
Parameters: u32* tuner_id, return allocated tuner id value in same tuner type to demod driver.
 struct QPSK_TUNER_CONFIG_EXT * ptrTuner_Config, parameter for tuner config
Return: int, operation status code; configuration successful return with SUCCESS

Nim_Tuner_Control
Function: int nim_Tuner _Control (u32 tuenr_id, u32 freq, u32 sym)
Description: API function for QPSK Tuner's Parameter Configuration
Parameters: u32 tuner_id, tuner device id in same tuner type
 u32 freq, Channel Frequency
 u32 sym, Channel Symbol Rate
Return: int, operation status code; configuration successful return with SUCCESS

Nim_Tuner_Status
Function: int nim_Tuner _Status (u32 tuner_id, u8 *lock)
Description: API function for QPSK Tuner's Parameter Configuration
Parameters: u32 tuner_id, tuner device id in same tuner type
 u8 *lock, pointer to the place to write back the Tuner Current Status
Return: int, operation status code; configuration successful return with SUCCESS
*/
#define MAX_TUNER_SUPPORT_NUM 4
#define FAST_TIMECST_AGC 1
#define SLOW_TIMECST_AGC 0
#define Tuner_Chip_SANYO 9
#define Tuner_Chip_CD1616LF_GIH 8
#define Tuner_Chip_NXP 7
#define Tuner_Chip_MAXLINEAR 6
#define Tuner_Chip_MICROTUNE 5
#define Tuner_Chip_QUANTEK 4
#define Tuner_Chip_RFMAGIC 3
#define Tuner_Chip_ALPS 2 //60120-01Angus
#define Tuner_Chip_PHILIPS 1
#define Tuner_Chip_INFINEON 0
/****************************************************************************/
#define Tuner_Chip_MAXLINEAR 6
#define Tuner_Chip_EN4020 9

#define _1st_i2c_cmd 0
#define _2nd_i2c_cmd 1

/*Front End State*/
#define TUNER_INITIATING 0
#define TUNER_INITIATED 1
#define TUNER_TUNING 2
#define TUNER_TUNED 3

//add by bill for tuner standby function
#define NIM_TUNER_SET_STANDBY_CMD 0xffffffff

typedef int(*INTERFACE_DEM_WRITE_READ_TUNER)(void *nim_dev_priv, u8 tuner_address, u8 *wdata, int wlen, u8 *rdata, int rlen);
typedef struct
{
	void *nim_dev_priv;  // for support dual demodulator.
	// The tuner can not be directly accessed through I2C,
	// tuner driver summit data to dem, dem driver will Write_Read tuner.
	INTERFACE_DEM_WRITE_READ_TUNER Dem_Write_Read_Tuner;
} DEM_WRITE_READ_TUNER;  // Dem control tuner by through mode (it's not by-pass mode).

/* external demodulator config parameter */
struct EXT_DM_CONFIG
{
	u32 i2c_base_addr;
	u32 i2c_type_id;
	u32 dm_crystal;
	u32 dm_clock;
	u32 polar_gpio_num;
	u32 lock_polar_reverse;
	u8 nim_idx;    // dmq add for combo and twin tuner
	u8 ts_mode;    // dmq 2012/03/07 add
	u8 demo_type;  // dmq 2012/03/09 add
};

#define LNB_CMD_BASE 0xf0
#define LNB_CMD_ALLOC_ID (LNB_CMD_BASE+1)
#define LNB_CMD_INIT_CHIP (LNB_CMD_BASE+2)
#define LNB_CMD_SET_POLAR (LNB_CMD_BASE+3)
#define LNB_CMD_POWER_EN (LNB_CMD_BASE+4)
/*external lnb controller config parameter*/
struct EXT_LNB_CTRL_CONFIG
{
	u32 param_check_sum;  // ext_lnb_control+i2c_base_addr+i2c_type_id = param_check_sum
	int(*ext_lnb_control)(u32, u32, u32);
	u32 i2c_base_addr;
	u32 i2c_type_id;
	u8 int_gpio_en;
	u8 int_gpio_polar;
	u8 int_gpio_num;
};

typedef struct COFDM_TUNER_CONFIG_API *PCOFDM_TUNER_CONFIG_API;

struct COFDM_TUNER_CONFIG_DATA
{
	u8 *ptMT352;
	u8 *ptMT353;
	u8 *ptST0360;
	u8 *ptST0361;
	u8 *ptST0362;
	u8 *ptAF9003;
	u8 *ptNXP10048;
	u8 *ptSH1432;
	u16 *ptSH1409;

//for ddk and normal design.
	//for I/Q conncection config. bit2: I/Q swap. bit1: I_Diff swap. bit0: Q_Diff swap.< 0: no, 1: swap>;
	u8 Connection_config;
	//bit0: IF-AGC enable <0: disable, 1: enalbe>;bit1: IF-AGC slop <0: negtive, 1: positive>
	//bit2: RF-AGC enable <0: disable, 1: enalbe>;bit3: RF-AGC slop <0: negtive, 1: positive>
	//bit4: Low-if/Zero-if.<0: Low-if, 1: Zero-if>
	//bit5: RF-RSSI enable <0: disable, 1: enalbe>;bit6: RF-RSSI slop <0: negtive, 1: positive>
	//bit8: fft_gain function <0: disable, 1: enable>
	//bit9: "blank channel" searching function <0: accuate mode, 1: fast mode>
	//bit10~11: frequency offset searching range <0: +-166, 1: +-(166*2), 2: +-(166*3), 3: +-(166*4)>
	//bit12: RSSI monitor <0: disable, 1: enable>
	u16 Cofdm_Config;

	u8 AGC_REF;
	u8 RF_AGC_MAX;
	u8 RF_AGC_MIN;
	u8 IF_AGC_MAX;
	u8 IF_AGC_MIN;
	u32 i2c_type_sel;
	u32 i2c_type_sel_1;  // for I2C_SUPPORT_MUTI_DEMOD
	u8 demod_chip_addr;
	u8 demod_chip_addr1;
	u8 demod_chip_ver;
	u8 tnuer_id;
	u8 cTuner_Tsi_Setting_0;
	u8 cTuner_Tsi_Setting_1;
};

struct COFDM_TUNER_CONFIG_EXT
{
#if defined(MODULE)
	u32 *i2c_adap;
#endif
	u16 cTuner_Crystal;
	u8 cTuner_Base_Addr; /* Tuner BaseAddress for Write Operation: (BaseAddress + 1) for Read */
	u8 cChip;
	u8 cTuner_Ref_DivRatio;
	u16 wTuner_IF_Freq;
	u8 cTuner_AGC_TOP;
	u8 cTuner_support_num; //dmq add
	u16 cTuner_Step_Freq;
	int(*Tuner_Write)(u32 id, u8 slv_addr, u8 *data, int len); /* Write Tuner Program Register */
	int(*Tuner_Read)(u32 id, u8 slv_addr, u8 *data, int len); /* Read Tuner Status Register */
	int(*Tuner_Write_Read)(u32 id, u8 slv_addr, u8 *data, u8 wlen, int len);
	u32 i2c_type_id; /*i2c type and dev id select. bit16~bit31: type, I2C_TYPE_SCB/I2C_TYPE_GPIO. bit0~bit15:dev id, 0/1.*/

	// copy from COFDM_TUNER_CONFIG_DATA struct in order to let tuner knows whether the RF/IF AGC is enable or not.
	// esp for max3580, which uses this info to turn on/off internal power detection circuit. See max3580 user manual for detail.

	//bit0: IF-AGC enable <0: disable, 1: enalbe>;bit1: IF-AGC slop <0: negtive, 1: positive>
	//bit2: RF-AGC enable <0: disable, 1: enalbe>;bit3: RF-AGC slop <0: negtive, 1: positive>
	//bit4: Low-if/Zero-if.<0: Low-if, 1: Zero-if>
	//bit5: RF-RSSI enable <0: disable, 1: enalbe>;bit6: RF-RSSI slop <0: negtive, 1: positive>
	u16 Cofdm_Config;
#ifdef PANASONIC_DEMO_SUPPORT
	u8 demo_type;
	u8 demo_addr;
	u32 i2c_mutex_id;
	void *priv;  // dmq 2011/12/21
#endif

	int if_signal_target_intensity;
};

struct COFDM_TUNER_CONFIG_API
{
	struct COFDM_TUNER_CONFIG_DATA config_data;
	int(*nim_Tuner_Init)(u32 *tuner_id, struct COFDM_TUNER_CONFIG_EXT *ptrTuner_Config);
	int(*nim_Tuner_Control)(u32 tuner_id, u32 freq, u8 bandwidth, u8 AGC_Time_Const, u8 *data, u8 cmd_type);
	int(*nim_Tuner_Status)(u32 tuner_id, u8 *lock);
	union
	{
		int(*nim_Tuner_Cal_Agc)(u32 tuner_id, u8 flag, u16 rf_val, u16 if_val, u8 *data);
		int(*nim_Tuner_Command)(u32 tuner_id, int cmd, u32 param);
	};
	u8(*nim_tuner_select)(struct COFDM_TUNER_CONFIG_API *cfg, u8 idx); //dmq 2009/12/08 add
	void (*nim_lock_cb)(u8 lock);
	struct COFDM_TUNER_CONFIG_EXT tuner_config;
	struct EXT_DM_CONFIG ext_dm_config;

	u32 tuner_type;
	u32 rev_id : 8;
	u32 config_mode : 1;
	u32 work_mode : 1;  // NIM_COFDM_SOC_MODE or NIM_COFDM_ONLY_MODE
	u32 ts_mode : 2;  // enum nim_cofdm_ts_mode, only for NIM_COFDM_ONLY_MODE
#ifdef PANASONIC_DEMO_SUPPORT
	u32 tune_t2_first : 1;  // when manual tune tp, if first to tune t2
	u32 reserved : 19;
#else
	u32 reserved : 20;
#endif
};

typedef enum  // For TSI select
{
	NIM_0_SPI_0 = 0,
	NIM_0_SPI_1,
	NIM_1_SPI_0,
	NIM_1_SPI_1,
	NIM_0_SSI_0,
	NIM_0_SSI_1,
	NIM_1_SSI_0,
	NIM_1_SSI_1
} NIM_TSI_Setting;

extern int tun_mxl603_init(u32 *tuner_id, struct COFDM_TUNER_CONFIG_EXT *ptrTuner_Config);
extern int tun_mxl603_control(u32 tuner_id, u32 freq, u8 bandwidth, u8 AGC_Time_Const, u8 *data, u8 _i2c_cmd);
extern int tun_mxl603_status(u32 tuner_id, u8 *lock);
extern int tun_mxl603_command(u32 tuner_id, int cmd, u32 param);
extern int tun_mxl603_set_addr(u32 tuner_idx, u8 addr, u32 i2c_mutex_id);
extern int tun_mxl603_rfpower(u32 tuner_id, s16 *rf_power_dbm);

struct QPSK_TUNER_CONFIG_DATA
{
	u16 Recv_Freq_Low;
	u16 Recv_Freq_High;
	u16 Ana_Filter_BW;
	u8 Connection_config;
	u8 Reserved_byte;
	u8 AGC_Threshold_1;
	u8 AGC_Threshold_2;
	u16 QPSK_Config;/*bit0:QPSK_FREQ_OFFSET,bit1:EXT_ADC,bit2:IQ_AD_SWAP,bit3:I2C_THROUGH,bit4:polar revert bit5:NEW_AGC1,bit6bit7:QPSK bitmode:
 00:1bit,01:2bit,10:4bit,11:8bit*/
};

struct QPSK_TUNER_CONFIG_EXT
{
	u16 wTuner_Crystal; /* Tuner Used Crystal: in KHz unit */
	u8 cTuner_Base_Addr; /* Tuner BaseAddress for Write Operation: (BaseAddress + 1) for Read */
	u8 cTuner_Out_S_D_Sel; /* Tuner Output mode Select: 1 --> Single end, 0 --> Differential */
	u32 i2c_type_id; /*i2c type and dev id select. bit16~bit31: type, I2C_TYPE_SCB/I2C_TYPE_GPIO. bit0~bit15:dev id, 0/1.*/
	u8 cTuner_support_num; //dmq add
};

struct QPSK_TUNER_CONFIG_API
{
	/* struct for QPSK Configuration */
	struct QPSK_TUNER_CONFIG_DATA config_data;

	/* Tuner Initialization Function */
	unsigned int(*nim_Tuner_Init)(u8 *tuner_id, struct QPSK_TUNER_CONFIG_EXT *ptrTuner_Config);

	/* Tuner Parameter Configuration Function */
	unsigned int(*nim_Tuner_Control)(u8 tuner_id, u32 freq, u32 sym);

	/* Get Tuner Status Function */
	unsigned int(*nim_Tuner_Status)(u8 tuner_id, u8 *lock);

	u8(*nim_tuner_select)(struct QPSK_TUNER_CONFIG_API *cfg, u8 idx);

	/* Extension struct for Tuner Configuration */
	struct QPSK_TUNER_CONFIG_EXT tuner_config;
	struct EXT_DM_CONFIG ext_dm_config;
	struct EXT_LNB_CTRL_CONFIG ext_lnb_config;
	u32 device_type; //current chip type. only used for M3501A
};

struct QAM_TUNER_CONFIG_DATA
{
	u8 RF_AGC_MAX;  // x.y V to xy value, 5.0v to 50v(3.3v to 33v)Qam then use it configue register.
	u8 RF_AGC_MIN;  // x.y V to xy value, 5.0v to 50v(3.3v to 33v)Qam then use it configue register.
	u8 IF_AGC_MAX;  // x.y V to xy value, 5.0v to 50v(3.3v to 33v)Qam then use it configue register.
	u8 IF_AGC_MIN;  // x.y V to xy value, 5.0v to 50v(3.3v to 33v)Qam then use it configue register.
	u8 AGC_REF;     // the average amplitude to full scale of A/D. % percentage rate.
};

struct QAM_TUNER_CONFIG_EXT
{
	u8 cTuner_Crystal;
	u8 cTuner_Base_Addr;  /* Tuner BaseAddress for Write Operation: (BaseAddress + 1) for Read */
	u8 cChip;
	u8 cTuner_special_config;  /* 0x01, RF AGC is disabled */
	u8 cTuner_Ref_DivRatio;
	u16 wTuner_IF_Freq;
	u8 cTuner_AGC_TOP;
	u16 cTuner_Step_Freq;
	int(*Tuner_Write)(u32 id, u8 slv_addr, u8 *data, int len);  /* Write Tuner Program Register */
	int(*Tuner_Read)(u32 id, u8 slv_addr, u8 *data, int len);  /* Read Tuner Status Register */
	u32 i2c_type_id;  /* i2c type and dev id select. bit16~bit31: type, I2C_TYPE_SCB/I2C_TYPE_GPIO. bit0~bit15:dev id, 0/1. */
};

struct QAM_TUNER_CONFIG_API
{
	/* struct for QAM Configuration */
	struct QAM_TUNER_CONFIG_DATA tuner_config_data;

	/* Tuner Initialization Function */
	int(*nim_Tuner_Init)(u32 *ptrTun_id, struct QAM_TUNER_CONFIG_EXT *ptrTuner_Config);

	/* Tuner Parameter Configuration Function */
	int(*nim_Tuner_Control)(u32 Tun_id, u32 freq, u32 sym, u8 param);

	/* Get Tuner Status Function */
	int(*nim_Tuner_Status)(u32 Tun_id, u8 *lock);

	/* Extension struct for Tuner Configuration */
	struct QAM_TUNER_CONFIG_EXT tuner_config_ext;

	struct EXT_DM_CONFIG ext_dem_config;

};

#endif // _NIM_TUNER_H_
// vim:ts=4
