/**********************************文件头部注释***********************************
 *
 *  Copyright (C), 2005-2010, AV Frontier Tech. Co., Ltd.
 *
 *  Date: 2008.03.26
 *
 *
 *
 *
 * --------- --------- ----- -----
 * 2008.03.26 CS 0.01 新建
 *
 */
#ifndef __YWTUNER_EXT_H
#define __YWTUNER_EXT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Modulation */
typedef enum YWTUNER_Modulation_e
{
	YWTUNER_MOD_NONE    = (1 << 0),
	YWTUNER_MOD_QAM_16  = (1 << 1),
	YWTUNER_MOD_QAM_32  = (1 << 2),
	YWTUNER_MOD_QAM_64  = (1 << 3),
	YWTUNER_MOD_QAM_128 = (1 << 4),
	YWTUNER_MOD_QAM_256 = (1 << 5),
	YWTUNER_MOD_QPSK    = (1 << 6),
	YWTUNER_MOD_QPSK_S2 = (1 << 7),
	YWTUNER_MOD_8PSK    = (1 << 8),
	YWTUNER_MOD_16APSK  = (1 << 9),
	YWTUNER_MOD_32APSK  = (1 << 10),

	YWTUNER_MOD_BPSK    = (1 << 11),
	YWTUNER_MOD_8VSB    = (1 << 12),
	YWTUNER_MOD_AUTO    = (1 << 13)
} YWTUNER_Modulation_T;

/* Bandwidth */
typedef enum YWTUNER_TERBandwidth_e
{
	YWTUNER_TER_BANDWIDTH_8_MHZ = (1 << 0),
	YWTUNER_TER_BANDWIDTH_7_MHZ = (1 << 1),
	YWTUNER_TER_BANDWIDTH_6_MHZ = (1 << 2)
} YWTUNER_TERBandwidth_T;

#ifdef __cplusplus
}
#endif

#endif
// vim:ts=4
