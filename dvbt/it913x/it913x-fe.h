/*
 *  Driver for it913x Frontend
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.=
 */
#ifndef IT913X_FE_H
#define IT913X_FE_H
#include <linux/dvb/frontend.h>

struct ite_config {
	u8 chip_ver;
	u16 chip_type;
	u32 firmware;
	u8 firmware_ver;
	u8 adc_x2;
	u8 tuner_id_0;
	u8 tuner_id_1;
	u8 dual_mode;
	u8 adf;
	/* option to read SIGNAL_LEVEL */
	u8 read_slevel;
};

extern struct dvb_frontend *it913x_fe_attach(struct i2c_adapter *i2c_adap, u8 i2c_addr, struct ite_config *config);

#define I2C_BASE_ADDR        0x10
#define DEV_0                0x0
#define DEV_1                0x10
#define PRO_LINK             0x0
#define PRO_DMOD             0x1
#define DEV_0_DMOD           (PRO_DMOD << 0x7)
#define DEV_1_DMOD           (DEV_0_DMOD | DEV_1)
#define CHIP2_I2C_ADDR       0x3a
#define AFE_MEM0             0xfb24
#define MP2_SW_RST           0xf99d
#define MP2IF2_SW_RST        0xf9a4
#define	PADODPU              0xd827
#define THIRDODPU            0xd828
#define AGC_O_D              0xd829
#define EP0_TX_EN            0xdd11
#define EP0_TX_NAK           0xdd13
#define EP4_TX_LEN_LSB       0xdd88
#define EP4_TX_LEN_MSB       0xdd89
#define EP4_MAX_PKT          0xdd0c
#define EP5_TX_LEN_LSB       0xdd8a
#define EP5_TX_LEN_MSB       0xdd8b
#define EP5_MAX_PKT          0xdd0d
#define IO_MUX_POWER_CLK     0xd800
#define CLK_O_EN             0xd81a
#define I2C_CLK              0xf103
#define I2C_CLK_100          0x7
#define I2C_CLK_400          0x1a
#define D_TPSD_LOCK          0xf5a9
#define MP2IF2_EN            0xf9a3
#define MP2IF_SERIAL         0xf985
#define TSIS_ENABLE          0xf9cd
#define MP2IF2_HALF_PSB      0xf9a5
#define MP2IF_STOP_EN        0xf9b5
#define MPEG_FULL_SPEED      0xf990
#define TOP_HOSTB_SER_MODE   0xd91c
#define PID_RST			     0xf992
#define PID_EN			     0xf993
#define PID_INX_EN		     0xf994
#define PID_INX			     0xf995
#define PID_LSB			     0xf996
#define PID_MSB			     0xf997
#define MP2IF_MPEG_PAR_MODE	 0xf986
#define DCA_UPPER_CHIP       0xf731
#define DCA_LOWER_CHIP       0xf732
#define DCA_PLATCH           0xf730
#define DCA_FPGA_LATCH       0xf778
#define DCA_STAND_ALONE      0xf73c
#define DCA_ENABLE           0xf776
#define DVBT_INTEN           0xf41f
#define DVBT_ENABLE          0xf41a
#define HOSTB_DCA_LOWER      0xd91f
#define HOSTB_MPEG_PAR_MODE  0xd91b
#define HOSTB_MPEG_SER_MODE  0xd91c
#define HOSTB_MPEG_SER_DO7   0xd91d
#define HOSTB_DCA_UPPER      0xd91e
#define PADMISCDR2           0xd830
#define PADMISCDR4           0xd831
#define PADMISCDR8           0xd832
#define PADMISCDRSR          0xd833
#define LOCK3_OUT            0xd8fd
#define GPIOH1_O             0xd8af
#define GPIOH1_EN            0xd8b0
#define GPIOH1_ON            0xd8b1
#define GPIOH3_O             0xd8b3
#define GPIOH3_EN            0xd8b4
#define GPIOH3_ON            0xd8b5
#define GPIOH5_O             0xd8bb
#define GPIOH5_EN            0xd8bc
#define GPIOH5_ON            0xd8bd
#define AFE_MEM0             0xfb24
#define REG_TPSD_TX_MODE     0xf900
#define REG_TPSD_GI          0xf901
#define REG_TPSD_HIER        0xf902
#define REG_TPSD_CONST       0xf903
#define REG_BW               0xf904
#define REG_PRIV             0xf905
#define REG_TPSD_HP_CODE     0xf906
#define REG_TPSD_LP_CODE     0xf907
#define MP2IF_SYNC_LK        0xf999
#define ADC_FREQ             0xf1cd
#define TRIGGER_OFSM         0x0000

/* COEFF Registers start at 0x0001 to 0x0020 */
#define COEFF_1_2048         0x0001
#define XTAL_CLK             0x0025
#define BFS_FCW              0x0029

/* Error Regs */
#define RSD_ABORT_PKT_LSB    0x0032
#define RSD_ABORT_PKT_MSB    0x0033
#define RSD_BIT_ERR_0_7      0x0034
#define RSD_BIT_ERR_8_15     0x0035
#define RSD_BIT_ERR_23_16    0x0036
#define RSD_BIT_COUNT_LSB    0x0037
#define RSD_BIT_COUNT_MSB    0x0038
#define TPSD_LOCK            0x003c
#define TRAINING_MODE        0x0040
#define ADC_X_2              0x0045
#define TUNER_ID             0x0046
#define EMPTY_CHANNEL_STATUS 0x0047
#define SIGNAL_LEVEL         0x0048
#define SIGNAL_QUALITY       0x0049
#define EST_SIGNAL_LEVEL     0x004a
#define FREE_BAND            0x004b
#define SUSPEND_FLAG         0x004c
#define VAR_P_INBAND         0x00f7

/* Built in tuner types */
#define IT9137    0x38
#define IT9135_38 0x38
#define IT9135_51 0x51
#define IT9135_52 0x52
#define IT9135_60 0x60
#define IT9135_61 0x61
#define IT9135_62 0x62

/* Vendor IDs */
#define USB_VID_AVERMEDIA             0x07ca
#define USB_VID_ITETECH               0x048d
#define USB_VID_KWORLD_2              0x1b80

/* Product IDs */
#define USB_PID_ITETECH_IT9135        0x9135
#define USB_PID_ITETECH_IT9135_9005   0x9005
#define USB_PID_ITETECH_IT9135_9006   0x9006
#define USB_PID_KWORLD_UB499_2T_T09   0xe409
#define USB_PID_AVERMEDIA_A835B_1835  0x1835
#define USB_PID_AVERMEDIA_A835B_2835  0x2835
#define USB_PID_AVERMEDIA_A835B_3835  0x3835
#define USB_PID_AVERMEDIA_A835B_4835  0x4835
#define USB_PID_AVERMEDIA_H335        0x0335
#define USB_PID_SVEON_STV22_IT9137    0xe411
#define USB_PID_CTVDIGDUAL_V2         0xe410

enum {
	CMD_DEMOD_READ = 0,
	CMD_DEMOD_WRITE,
	CMD_TUNER_READ,
	CMD_TUNER_WRITE,
	CMD_REG_EEPROM_READ,
	CMD_REG_EEPROM_WRITE,
	CMD_DATA_READ,
	CMD_VAR_READ = 8,
	CMD_VAR_WRITE,
	CMD_PLATFORM_GET,
	CMD_PLATFORM_SET,
	CMD_IP_CACHE,
	CMD_IP_ADD,
	CMD_IP_REMOVE,
	CMD_PID_ADD,
	CMD_PID_REMOVE,
	CMD_SIPSI_GET,
	CMD_SIPSI_MPE_RESET,
	CMD_H_PID_ADD = 0x15,
	CMD_H_PID_REMOVE,
	CMD_ABORT,
	CMD_IR_GET,
	CMD_IR_SET,
	CMD_FW_DOWNLOAD = 0x21,
	CMD_QUERYINFO,
	CMD_BOOT,
	CMD_FW_DOWNLOAD_BEGIN,
	CMD_FW_DOWNLOAD_END,
	CMD_RUN_CODE,
	CMD_SCATTER_READ = 0x28,
	CMD_SCATTER_WRITE,
	CMD_GENERIC_READ,
	CMD_GENERIC_WRITE
};

enum {
	READ_LONG,
	WRITE_LONG,
	READ_SHORT,
	WRITE_SHORT,
	READ_DATA,
	WRITE_DATA,
	WRITE_CMD,
};

enum {
	IT9135_AUTO = 0,
	IT9137_FW,
	IT9135_V1_FW,
	IT9135_V2_FW,
};

#if 0
struct it913x_core_device_t {
	struct list_head entry;

	struct list_head clients;
	struct list_head subclients;
	spinlock_t clientslock;

	struct list_head buffers;
	spinlock_t bufferslock;
	int num_buffers;

	void *common_buffer;
	int common_buffer_size;
	dma_addr_t common_buffer_phys;

	void *context;
	struct device *device;

	char devpath[32];
	unsigned long device_flags;

	setmode_t setmode_handler;
	detectmode_t detectmode_handler;
	sendrequest_t sendrequest_handler;
	preload_t preload_handler;
	postload_t postload_handler;

	int mode, modes_supported;

	/* host <--> device messages */
	struct completion version_ex_done, data_download_done, trigger_done;
	struct completion init_device_done, reload_start_done, resume_done;
	struct completion gpio_configuration_done, gpio_set_level_done;
	struct completion gpio_get_level_done, ir_init_done;

	/* Buffer management */
	wait_queue_head_t buffer_mng_waitq;

	/* GPIO */
	int gpio_get_res;

	/* Target hardware board */
	int board_id;

	/* Firmware */
	u8 *fw_buf;
	u32 fw_buf_size;

	/* Infrared (IR) */
	struct ir_t ir;

	int led_state;
};
#endif

struct PID_STATISTICS_DATA_S {
	struct PID_BURST_S {
		u32 size;
		u32 padding_cols;
		u32 punct_cols;
		u32 duration;
		u32 cycle;
		u32 calc_cycle;
	} burst;

	u32 tot_tbl_cnt;
	u32 invalid_tbl_cnt;
	u32 tot_cor_tbl;
};

struct PID_DATA_S {
	u32 pid;
	u32 num_rows;
	struct PID_STATISTICS_DATA_S pid_statistics;
};

struct TRANSMISSION_STATISTICS_S {
	u32 Frequency;		/* Frequency in Hz */
	u32 Bandwidth;		/* Bandwidth in MHz */
	u32 TransmissionMode;	/* FFT mode carriers in Kilos */
	u32 GuardInterval;	/* Guard Interval from
	SMSHOSTLIB_GUARD_INTERVALS_ET */
	u32 CodeRate;		/* Code Rate from SMSHOSTLIB_CODE_RATE_ET */
	u32 LPCodeRate;		/* Low Priority Code Rate from
	SMSHOSTLIB_CODE_RATE_ET */
	u32 Hierarchy;		/* Hierarchy from SMSHOSTLIB_HIERARCHY_ET */
	u32 Constellation;	/* Constellation from
	SMSHOSTLIB_CONSTELLATION_ET */

	/* DVB-H TPS parameters */
	u32 CellId;		/* TPS Cell ID in bits 15..0, bits 31..16 zero;
	 if set to 0xFFFFFFFF cell_id not yet recovered */
	u32 DvbhSrvIndHP;	/* DVB-H service indication info, bit 1 -
	 Time Slicing indicator, bit 0 - MPE-FEC indicator */
	u32 DvbhSrvIndLP;	/* DVB-H service indication info, bit 1 -
	 Time Slicing indicator, bit 0 - MPE-FEC indicator */
	u32 IsDemodLocked;	/* 0 - not locked, 1 - locked */
};

struct RECEPTION_STATISTICS_S {
	u32 IsRfLocked;		/* 0 - not locked, 1 - locked */
	u32 IsDemodLocked;	/* 0 - not locked, 1 - locked */
	u32 IsExternalLNAOn;	/* 0 - external LNA off, 1 - external LNA on */

	u32 ModemState;		/* from SMSHOSTLIB_DVB_MODEM_STATE_ET */
	s32 SNR;		/* dB */
	u32 BER;		/* Post Viterbi BER [1E-5] */
	u32 BERErrorCount;	/* Number of erronous SYNC bits. */
	u32 BERBitCount;	/* Total number of SYNC bits. */
	u32 TS_PER;		/* Transport stream PER,
	0xFFFFFFFF indicate N/A */
	u32 MFER;		/* DVB-H frame error rate in percentage,
	0xFFFFFFFF indicate N/A, valid only for DVB-H */
	s32 RSSI;		/* dBm */
	s32 InBandPwr;		/* In band power in dBM */
	s32 CarrierOffset;	/* Carrier Offset in bin/1024 */
	u32 ErrorTSPackets;	/* Number of erroneous
	transport-stream packets */
	u32 TotalTSPackets;	/* Total number of transport-stream packets */

	s32 MRC_SNR;		/* dB */
	s32 MRC_RSSI;		/* dBm */
	s32 MRC_InBandPwr;	/* In band power in dBM */
};

/* Statistics information returned as response for
 * SmsHostApiGetStatisticsEx_Req for DVB applications, SMS1100 and up */
struct IT913X_HOSTLIB_STATISTICS_DVB_S {
	/* Reception */
	struct RECEPTION_STATISTICS_S ReceptionData;

	/* Transmission parameters */
	struct TRANSMISSION_STATISTICS_S TransmissionData;

	/* Burst parameters, valid only for DVB-H */
#define	SRVM_MAX_PID_FILTERS 8
	struct PID_DATA_S PidData[SRVM_MAX_PID_FILTERS];
};

struct it913x_dvb_client_t {
	struct list_head               entry;

//	struct it913x_core_device_t    *coredev;
//	struct it913x_core_client_t    *it913x_client;

	struct dvb_adapter             adapter;
	struct dvb_demux               demux;
	struct dmxdev                  dmxdev;
	struct dvb_frontend            frontend;

	fe_status_t                    fe_status;

	struct completion              tune_done;

	/* todo: save freq/band instead whole struct */
	struct dvb_frontend_parameters fe_params;

	struct IT913X_HOSTLIB_STATISTICS_DVB_S it913x_stat_dvb;
	int event_fe_state;
	int event_unc_state;
};



#endif /* IT913X_FE_H */
// vim:ts=4

