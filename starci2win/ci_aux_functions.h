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

#ifndef __CI_AUX_FUNCTIONS_H__
#define __CI_AUX_FUNCTIONS_H__

/************************************/
#define	T_SB				0x80
#define	T_RCV				0x81
#define	T_CREATE			0x82
#define	T_CREATE_REPLY		0x83
#define	T_DELETE			0x84
#define	T_DELTE_REPLY		0x85
#define	T_REQUEST			0x86
#define	T_NEW				0x87
#define	T_ERROR				0x88
#define	T_DATA_LAST			0xA0
#define	T_DATA_MORE			0xA1

#define SINGLE_PKT			0x01
#define MULTI_PKT			0x00


typedef enum 
{
	FREE=0,
	IN_READ,
	DATA_AVAILABLE
} CI_READ_STATUS;


typedef struct
{
	unsigned char * data_slot;	
	size_t len_slot;
	CI_READ_STATUS status;
}Read_Data_t;

typedef struct
{
	Read_Data_t cams[2/*CI_NUMBER_OF_CONTROLLERS*/];
	unsigned char id_in_read;
}Read_Action_t;

typedef struct
{
	unsigned short NgBuf;
	unsigned char linkID;
	unsigned char SessionProfile;
	unsigned char in_write;
	unsigned char to_read;
}CI_Info_Control_t;



int CI_DRV_Init(void);
char MessageSend(unsigned char * TxBuff, unsigned int len_data, unsigned char SlotID);
char MessageRead(unsigned char * RxBuff,unsigned int * len_data,unsigned char SlotID);
unsigned char ModifyBuffSize(unsigned short *NegoBuff,unsigned char SlotID);
int q_read (unsigned char device_id, unsigned char *buf, int size);
int q_write (unsigned char device_id, unsigned char *buf, int size);
void polling_ci(unsigned char id);
int q_read_after_polling (unsigned char device_id, unsigned char *buf, int size);

/* Transport Protocol Layer */
void sendCreateTC(unsigned char * buff, unsigned int * TxLen, unsigned char id);
void sendPolling(unsigned char * buff, unsigned int * TxLen, unsigned char id);
int create_tpl(unsigned char slotid);
void sendT_recv(unsigned char * buff, unsigned int * TxLen);
void sendT_send_last(unsigned char * buff, unsigned int * TxLen, unsigned char single);
void sendT_send_more(unsigned char * buff, unsigned int * TxLen);

#endif ///__CI_AUX_FUNCTIONS_H__






