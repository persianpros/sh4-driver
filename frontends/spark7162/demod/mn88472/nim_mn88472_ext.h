/****************************************************************************
 *
 * Panasonic MN88472 DVB-T(2)/C demodulator driver
 *
 * Version for Fulan Spark7162
 *
 * Supports Panasonic MN88472 with MaxLinear MxL301 tuner
 *
 * Original code:
 * Copyright (C), 2013-2018, AV Frontier Tech. Co., Ltd.
 *
 * Date: 2013.12.16
 *
 */

#ifndef __NIM_MN88472_EXT_H
#define __NIM_MN88472_EXT_H

#ifdef __cplusplus
extern "C" {
#endif

int nim_mn88472_channel_change(struct nim_device *dev, struct NIM_Channel_Change *param);
int nim_mn88472_open(struct nim_device *dev);
int nim_mn88472_close(struct nim_device *dev);
int nim_mn88472_get_lock(struct nim_device *dev, u8 *lock);
int nim_mn88472_get_SNR(struct nim_device *dev, u8 *snr);
int nim_mn88472_get_BER(struct nim_device *dev, u32 *ber);
int nim_mn88472_get_AGC(struct nim_device *dev, u8 *agc);
int nim_mn88472_get_AGC_301(struct nim_device *dev, u8 *agc);
int nim_mn88472_get_AGC_603(struct nim_device *dev, u8 *agc);
int nim_mn88472_attach(u8 Handle, PCOFDM_TUNER_CONFIG_API pConfig, TUNER_OpenParams_T *OpenParams);
unsigned int tuner_mxl301_identify(u32 *i2c_adap);
int nim_mn88472_channel_change_earda(struct nim_device *dev, struct NIM_Channel_Change *param);
int nim_mn88472_ioctl_earda(struct nim_device *dev, int cmd, u32 param);
int nim_mn88472_attach_earda(u8 Handle, PCOFDM_TUNER_CONFIG_API pConfig, TUNER_OpenParams_T *OpenParams);

int DMD_TCB_WriteRead(void *nim_dev_priv, u8 tuner_address, u8 *wdata, int wlen, u8 *rdata, int rlen);

#ifdef __cplusplus
}
#endif

#endif  // __NIM_MN88472_EXT_H
// vim:ts=4
