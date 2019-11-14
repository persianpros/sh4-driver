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

#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "ci_aux_functions.h"
#include "ci_cis_functions.h"
#include "starci2win.h"
#include "starci2win_i2c.h"
#include "cidef.h"
#include "ci_i2c.h"

#define CI_TIMER_TIMEOUT 20
#define ONE_SECOND     1000/CI_TIMER_TIMEOUT
#define TIME_OUT 10

#define WAIT_TIMEOUT		300000 /* Counter */

#define MSG_SEND_TIMEOUT	WAIT_TIMEOUT
#define MSG_READ_TIMEOUT	WAIT_TIMEOUT


/**************/
//#define DDD 1
//#define	PRINT_WRITE_LOW	1

#ifdef DDD
	#define D_DEBUG(args...) \
		do { \
			printk("starci2win: " args); \
		} while (0)
#else
	#define D_DEBUG(args...) \
		do { \
		} while (0)
#endif

#define D_ERROR(fmt, args...)    printk(KERN_ERR "%s: " fmt, __FUNCTION__, ##args)
/***************/
extern unsigned long base_address;
extern State_ci_t ci_device[CI_NUMBER_OF_CONTROLLERS];
CI_Info_Control_t ci_info[CI_NUMBER_OF_CONTROLLERS];
Read_Action_t read_op;


/* For negotiate buffer */
UI16 BuffSize[MAX_SLOT];
UI16 MoreIndex[MAX_SLOT];
UI8  MoreFlag[MAX_SLOT];

/* misc variables */
DRV_stMem mem;
UI8 CmdReg;
UI32 ci_timer = 0;
UI16 CamAccessTime = 0;

/* Extern functions (in ci_cis_functions.c) */
extern I32 PHYS_CheckCIS(UI8 *,I32,UI16 *,UI8 *);
extern I32 PHYS_Tuple(UI8, UI8 **, I32 *, I32 *, I32 *, I32 *, I32 *);

/* Prototype of the functions */
unsigned char DRV_phywrite(unsigned char CMD,unsigned char EnDi,unsigned char Data,unsigned char SlotID);
unsigned char DRV_phyread(unsigned char CMD, unsigned char SlotID, unsigned char *val);
unsigned char ConfigAccessMode(unsigned char,unsigned char);
UI8 PHYS_CIS(UI16 *,UI8 *,UI8);
UI8 ReadCIS(UI8 SlotID);
void ci_delay(void);

/*******************/
/*    Functions    */
/*******************/
void CiRegWrite(unsigned char cireg,unsigned char  cidata, unsigned char SlotId)
{
	write_i2c(cireg,cidata);
}

unsigned char CiRegRead(unsigned char cireg, unsigned char SlotId)
{
	unsigned char CiData;
	CiData=read_i2c(cireg);
	return(CiData);
}

UI8 ReadCIS(UI8 SlotID)
{
	UI16 CorAddr=0;
	UI8  CorVal=0;
	UI32 BaseCor=0;

	/* select the module */
	if(SlotID==SLOTA)
	{
		write_i2c(DEST_MODULE,DEST_MOD_A);
	}
	else if(SlotID==SLOTB)
	{
		write_i2c(DEST_MODULE,DEST_MOD_B);
	}
	else
		return(NG);

	ConfigAccessMode(ATTRIBUTE, SlotID);
	ci_timer = 1;
	/* If find the tuples, then write the cor */
	if ( PHYS_CIS(&CorAddr, &CorVal, SlotID) == 1 )
	{
		BaseCor += CorAddr;

		ctrl_outb(CorVal, base_address+(BaseCor*2));
		ci_timer = 1;
		ci_delay();
		ConfigAccessMode(IOACCESS, SlotID);
		return(OK);
	}
	else
	{
		return(NG);
	}
}

UI8 PHYS_CIS(UI16 *addr, UI8 *corval, UI8 SlotID)
{
	UI8 cor=0;				/* COR value */             
	UI16 addrcor=0;			/* COR Address */          
	volatile UI8 *mbase_a;
	UI16 index=0;
	I32		ret_phys=0;
	Module_t module;

	int ii=0,cnt=0,cnt_1=0,counter=0;
	unsigned char cis_tmp[256];

	mem.addr = 0;
	mem.len  = DRV_CISLEN;	/* 256 */

	for(ii=0;ii<256;ii++)
		cis_tmp[ii]=0;

	mdelay(100);
	do{
		mdelay(11);
		cis_tmp[0]=(unsigned char)ctrl_inb(base_address+0);
		counter++;
	}while( (cis_tmp[0]!=0x1D) && (counter<181) );	/* ~ 2 seconds */
	if(counter>=181)
	{
		for(cnt=0;cnt<CIS_LEN;cnt++)
			cis_tmp[cnt]=0xFF;	/* return a cis with all bytes at 0xFF */
	}
	else
	{
		for(cnt=0;cnt<CIS_LEN;cnt++)
		{
			mdelay(3);
			cis_tmp[cnt]=(unsigned char)ctrl_inb(base_address+cnt_1);
			cnt_1+=4;	/* Only even bytes */
		}
	}
	/* Check the CIS and find the COR Address */ 
	ret_phys = PHYS_CheckCIS(&cis_tmp[0], mem.len, &addrcor, &cor);
	if (ret_phys < 0)
	{
		return KO;	/* 0x07 , wrong CIS */
	}
	/* the CIS is OK, addrcor contains the COR Address and cor the COR value */
	mem.addr   = addrcor;
	mem.len    = 1;			/* one byte */ 
	mem.pbytes = &cor;
	*addr      = addrcor;
	*corval    = cor;

	return ret_phys;
}

void ci_delay(void)
{
	udelay(10);//10us
}

unsigned char DRV_phywrite(unsigned char CMD,unsigned char EnDi,unsigned char Data,unsigned char SlotID)
{
	unsigned char res=0;

	/* select the module */
	if(SlotID==SLOTA)
	{
		write_i2c(DEST_MODULE,DEST_MOD_A);
	}
	else if(SlotID==SLOTB)
	{
		write_i2c(DEST_MODULE,DEST_MOD_B);
	}
	else
	  return(NG);

	if(CMD==CONTROL)
	{
		res = 0;
		if (EnDi==ENABLE)
			 res |= Data;
		else
			res=0;
	}
	else
		res = Data;

	/* Write */
	if( (CMD==CONTROL) || (CMD==DATA_m) ||
		(CMD==SIZE_L) || (CMD==SIZE_H) ||
		(CMD==RESET) )
	{
		ctrl_outb(res, base_address+(CMD*2));
	}
	ci_delay();
	write_i2c(DEST_MODULE,0x00);

	return(OK);
}

unsigned char DRV_phyread(unsigned char CMD, unsigned char SlotID, unsigned char *val)
{
	/* select the module */
	if(SlotID==SLOTA)
	{
		write_i2c(DEST_MODULE,DEST_MOD_A);
	}
	else if(SlotID==SLOTB)
	{
		write_i2c(DEST_MODULE,DEST_MOD_B);
	}
	else
		return(NG);

	/* Read */
	if( (CMD==CONTROL) || (CMD==DATA_m) ||
		(CMD==SIZE_L) || (CMD==SIZE_H) ||
		(CMD==RESET) )
	{
		*val=ctrl_inb(base_address+(CMD*2));
	}
	ci_delay();
	write_i2c(DEST_MODULE,0x00);

	return(OK);
}

char MessageSend(unsigned char * TxBuff, unsigned int len_data, unsigned char SlotID)
{
	unsigned short i;
	unsigned char  value;
	unsigned char  CI_status;
	int cct=0;	

	(void)DRV_phywrite(CONTROL,ENABLE,MODULE_HC,SlotID);
	DRV_phyread(STATUS_m,SlotID,&CI_status);

	while( (CI_status & MODULE_FR) != MODULE_FR)
	{
		/* Check if there is the module */
		if(ci_device[SlotID].status==MODULE_NOT_PRESENT)
			return (WRITE_ERROR);
		/****/
		if ( !(cct % 10) )
		{		
			(void)DRV_phywrite(CONTROL, DISABLE, MODULE_HC, SlotID);
			ci_delay();
			(void)DRV_phywrite(CONTROL,ENABLE,MODULE_HC,SlotID);
			DRV_phyread(STATUS_m,SlotID,&CI_status);
		}
		cct++;
		if( cct > MSG_SEND_TIMEOUT ) 
	    {
			printk("MessageSend: cct > 300000\n");
	        break;
       	}
		ci_delay();
	}
	
	if(cct > MSG_SEND_TIMEOUT)	/* 3seconds + 1second for the re-call DRV_* */
	{
		printk("---------------->NO WRITE, status 0x%02X\n",CI_status);
		return(WRITE_ERROR);
	}

	value = (UI8)(len_data & 0xff);
	(void)DRV_phywrite(SIZE_L,ENABLE,value,SlotID);
	ci_delay();
	value = (len_data >> 8) & 0xff;
	(void)DRV_phywrite(SIZE_H,ENABLE,value,SlotID);
	ci_delay();

	for(i = 0; i < len_data; i++ )
	{
		/* Check if there is the module */
		if(ci_device[SlotID].status==MODULE_NOT_PRESENT)
		{
			return (WRITE_ERROR);
		}
		/****/
		value = TxBuff[i];
		ci_delay();
		(void)DRV_phywrite(DATA_m,DISABLE,value,SlotID);
		
	}
	/* No check the WE_BIT */
	ci_delay();
	(void)DRV_phywrite(CONTROL,DISABLE,MODULE_HC,SlotID);
	ci_delay();

	return(OK);
}

char MessageRead(unsigned char * RxBuff,unsigned int * len_data,unsigned char SlotID)
{
	unsigned short i,rxsize,j;
	unsigned char ls,ms,value,datamore,value1;
	unsigned char CI_status;
	unsigned int cc_time,more_count;
	unsigned int total_rxsize=0;

	CmdReg = 0x00;
	datamore = 0;
	i = 0;
	more_count = 0;
	CamAccessTime = 1;

//MORE_READ:
	cc_time = 1;
	CI_status = 0;
	do
	{
		/* Check if there is the module */
		if(ci_device[SlotID].status==MODULE_NOT_PRESENT)
		{
			printk("Module is not present\n");
			*len_data=0;
			return (READ_ERROR);
		}
		/****/
		cc_time++;
		if ( !(cc_time % 10) )
		{
			DRV_phyread(STATUS_m,SlotID,&CI_status);
		}
		if( cc_time > MSG_READ_TIMEOUT ) break;

		ci_delay();
	} while( (CI_status & MODULE_DA) != MODULE_DA );

	if(cc_time > MSG_READ_TIMEOUT)	/* 3seconds + 0.3 second for re-call DRV_* */
	{	
		printk("-------->No read, status_reg:0x%02X, SLOTID: %d\n",CI_status,SlotID);
		*len_data=0;
		return(READ_ERROR);
	}
	if( (CI_status & MODULE_DA) == MODULE_DA )
	{

		(void)DRV_phyread(SIZE_L,SlotID,&ls);
		(void)DRV_phyread(SIZE_H,SlotID,&ms);
		rxsize = ls + ms * 256;
		if ( rxsize == 0 )
		{
			rxsize=*len_data; /* If data are available but the cam doesn't give the len, we use the 'total' len */
	//		return(NG);
		}
		else
		{
		//	CamReset[SlotID] = 0;
		}

		(void)DRV_phyread(DATA_m,SlotID,&value);
		(void)DRV_phyread(DATA_m,SlotID,&value1);
		if( MoreFlag[SlotID] == 0 )
		{
			RxBuff[0] = value;
			RxBuff[1] = value1;
			total_rxsize+=rxsize;
			i = 2;
			MoreIndex[SlotID] = 0;
		}
		else
		{
			total_rxsize+=rxsize-2;
			i = MoreIndex[SlotID];
		}
		//FIXME????????
		*len_data=total_rxsize;

		datamore = value1 & 0x80;
		rxsize -= 2;

		for( j = 0; j < rxsize; j++)
		{
			/* Check if there is the module */
			if(ci_device[SlotID].status==MODULE_NOT_PRESENT)
			{
				*len_data=0;
				return (READ_ERROR);
			}
			/****/
			ci_delay();
			(void)DRV_phyread(DATA_m,SlotID,&value);
			RxBuff[i] = value;
			i++;
		}
/* Read ONLY a time, the multiple packet are manage in up level */
	/*	if( datamore == 0x80 )
		{
			MoreIndex[SlotID] = i;
			MoreFlag[SlotID] = 1;
			more_count++;
			if ( more_count > 10 ) 
			{
				return (NG);
			}
			goto  MORE_READ;
		
		}
		else */
		{
			MoreIndex[SlotID] = 0;
			MoreFlag[SlotID] = 0;
		}
/* These commands aren't necessary ... and in some cams these commands generate a bloc of cam */ 
		value = 0;
//		(void)DRV_phywrite(SIZE_L,ENABLE,value,SlotID);
//		(void)DRV_phywrite(SIZE_H,ENABLE,value,SlotID);
	}

	mdelay(1);

	if( CamAccessTime >= 50*ONE_SECOND )
	{
		CamAccessTime = 0;
		return(NG);
	}

	return(OK);
}

/************************/
/*	Make the hw reset	*/
/************************/
void hw_reset(unsigned char id)
{
	Register_t reg;
	if(id==0)
		reg.offset_addr=MOD_A_CTRL;
	else if(id==1)
		reg.offset_addr=MOD_B_CTRL;
	else
	{
		return;
	}
	reg.value=0xFF;
	reg.value=read_i2c(reg.offset_addr);
	reg.value|=0x80;	/* HW RESET:reset pin up:-> Now cam is in reset */
	write_i2c(reg.offset_addr,reg.value);
	mdelay(100);
	reg.value&=(~0x80);	/* HW RESET:reset pin down:-> Now cam starts */
	write_i2c(reg.offset_addr,reg.value);
	/* End reset hardware */
}

void set_mode_module(Module_t module)
{
	unsigned char res=0;
	if(module.module==MODULE_A)
	{
		res=read_i2c(MOD_A_CTRL);
		if((res&MODULE_PRESENT)!=0x01)
		{
			/* Only for test */
			write_i2c(DEST_MODULE,0x02);	/* Module A */
		}
		else
		{
			write_i2c(STARCI2WIN_CTRL,0x00);
			write_i2c(DEST_MODULE,0x00);
			write_i2c(STARCI2WIN_CTRL,0x01);
			if(module.configuration==CIS)
				write_i2c(MOD_A_CTRL,0x01);
			else
				write_i2c(MOD_A_CTRL,0x05);
			write_i2c(DEST_MODULE,0x02);	/* Module A */
			write_i2c(MP_INTERFACE_CONFIG,0x00);
			write_i2c(POWER_CTRL,0x01);
			write_i2c(MOD_A_ACCESS_TIME,0x44);	/* Set write time like read time: 600ns */
		}
	}
	else if(module.module==MODULE_B)
	{
		res=read_i2c(MOD_B_CTRL);
		if((res&MODULE_PRESENT)!=0x01)
		{
			/* Only for test */
			write_i2c(DEST_MODULE,0x04);	/* Module B */
		}
		else
		{
			write_i2c(STARCI2WIN_CTRL,0x00);
			write_i2c(DEST_MODULE,0x00);
			write_i2c(STARCI2WIN_CTRL,0x01);
			if(module.configuration==CIS)
				write_i2c(MOD_B_CTRL,0x01);
			else
				write_i2c(MOD_B_CTRL,0x05);
			write_i2c(DEST_MODULE,0x04);	/* Module B */
			write_i2c(MP_INTERFACE_CONFIG,0x00);
			write_i2c(POWER_CTRL,0x01);
			write_i2c(MOD_B_ACCESS_TIME,0x44);	/* Set write time like read time: 600ns */
		}
	}
}

unsigned char ModifyBuffSize(unsigned short *NegoBuff,unsigned char  SlotID)
{
	unsigned char Size_Ml, Size_Mh;
	unsigned short i, Size_M;
	unsigned char CI_status=0, value, buff[4];
	unsigned short ci_time1;
	Module_t module;

	ci_info[SlotID].NgBuf=0;
	ci_info[SlotID].linkID=0;
	ci_info[SlotID].SessionProfile=0;

	DRV_phyread(STATUS_m, SlotID, &CI_status);

	module.module=SlotID;
	module.configuration=IO_SPACE;
	set_mode_module(module);

	hw_reset(SlotID);
	mdelay(100);	/* 100ms */
	ReadCIS(SlotID);
	mdelay(100);	/* 100ms */

	ConfigAccessMode(IOACCESS, SlotID);
	memset(buff,0,sizeof(buff));
	CmdReg = 0x00;

	DRV_phywrite(CONTROL, ENABLE,  MODULE_RS, SlotID);
	mdelay(TIME_OUT);
	DRV_phywrite(CONTROL, DISABLE, MODULE_RS, SlotID);
	mdelay(TIME_OUT);

	ci_timer = 1;
	ci_time1 = 0;
	while(1)
	{
		/* Check if there is the module */
		if(ci_device[SlotID].status==MODULE_NOT_PRESENT) goto CI_err;
		/****/
		
		if( !(ci_time1 % 10) )
		{
			DRV_phyread(STATUS_m, SlotID, &CI_status);
			if( (CI_status&MODULE_FR) == MODULE_FR)
			{
				break;
			}
		}
		if ( ci_time1 > 1000 )	/* old is 200,some cams need more loops */
		{
			printk("The bit 'MODULE_FR' is not set \n");
			goto CI_err;
		}
		ci_time1++;
		mdelay(TIME_OUT);
	}
	printk("ModifyBuffSize MODULE_FR #1 : CI_status = 0x%x\r\n", CI_status);
	mdelay(5);
	DRV_phywrite(CONTROL, ENABLE, MODULE_SR, SlotID);
	CI_status=0;
	ci_timer = 1;
	ci_time1 = 0;
	CI_status = 0;
	while(1)
	{
		/* Check if there is the module */
		if(ci_device[SlotID].status==MODULE_NOT_PRESENT) goto CI_err;
		/****/
		
		if( !(ci_time1 % 10) )
		{
			CI_status=0;
			DRV_phyread(STATUS_m, SlotID, &CI_status);
			if( (CI_status & MODULE_DA) == MODULE_DA)
			{
				break;
			}
			if( (ci_time1%500)==0)
				DRV_phywrite(CONTROL, ENABLE, MODULE_SR, SlotID);
		}
		if ( ci_time1 > 4000 )	/* old is 200,some cams need more loops */
		{
			printk("The bit 'MODULE_DA' is not set \n");
			goto CI_err;
		}
		mdelay(TIME_OUT);
		ci_time1++;
	}
	printk("MODULE_FR #2 : CI_status = 0x%x\r\n", CI_status);
	DRV_phyread(SIZE_H, SlotID, &Size_Mh);
	DRV_phyread(SIZE_L, SlotID, &Size_Ml);

	Size_M = (UI16)(Size_Mh * 256 + Size_Ml);
	printk("Size_M Size = 0x%x, Mh%x Ml%x\r\n", Size_M, Size_Mh, Size_Ml);
	if(Size_M>2)
	{
		printk("Size to read the Buffer size is wrong (> 2) \n");
		goto CI_err;
	}
	i = 0;
	while(1)
	{
		DRV_phyread(DATA_m,SlotID,&value);
		buff[i] = value;
		if( i >= 2 )
		{
			break;
		}
		i++;
	}

	BuffSize[SlotID] = (UI16)(buff[0] * 256 + buff[1]);
	printk("BuffSize  = 0x%x, 0x%x 0x%x\r\n",  BuffSize[SlotID], buff[0], buff[1]);
	DRV_phywrite(CONTROL,DISABLE,MODULE_SR,SlotID);
	mdelay(TIME_OUT);

	DRV_phywrite(CONTROL,ENABLE,MODULE_SW,SlotID);
	DRV_phywrite(CONTROL,ENABLE,MODULE_HC,SlotID);
	ci_time1 = 0;
	ci_timer = 1;

	mdelay(100);
	while(1)
	{
		/* Check if there is the module */
		if(ci_device[SlotID].status==MODULE_NOT_PRESENT) goto CI_err;
		/****/
		
		if(!(ci_time1 % 10) )
		{
			DRV_phyread(STATUS_m,SlotID,&CI_status);
			if( (CI_status & MODULE_FR) == MODULE_FR ) break;
		}
		if(ci_time1 > 200 )
		{
			DRV_phywrite(CONTROL,DISABLE,MODULE_HC,SlotID);
			DRV_phywrite(CONTROL,DISABLE,MODULE_SW,SlotID);
			printk("The bit 'MODULE_FR' is not set after the read of the buffer size \n");
			goto CI_err;
		}
		ci_time1++;
		mdelay(TIME_OUT);
	}
	DRV_phywrite(SIZE_L,ENABLE,(Size_M & 0xff),SlotID);
	DRV_phywrite(SIZE_H,ENABLE,(UI8)((Size_M >> 8) & 0x00ff),SlotID);
	for( i = 0; i < Size_M; i++ )
	{
		DRV_phywrite(DATA_m,ENABLE,buff[i],SlotID);
	}

	DRV_phywrite(CONTROL,DISABLE,MODULE_HC,SlotID);
	DRV_phywrite(CONTROL,DISABLE,MODULE_SW,SlotID);
	ci_timer = 1;
	ci_time1 = 0;
	mdelay(100);

	while(1)
	{
		/* Check if there is the module */
		if(ci_device[SlotID].status==MODULE_NOT_PRESENT) goto CI_err;
		/****/
		if( !(ci_time1 % 10) )
		{
			DRV_phyread(STATUS_m,SlotID,&CI_status);
			if ( (CI_status & MODULE_FR) ) break;
		}
		if(ci_time1 > 4000 ) 
		{
			printk("The bit 'MODULE_FR' is not set after buffer size negotation \n");
			goto CI_err;
		}
		ci_time1++;
		mdelay(TIME_OUT);
	}

	CmdReg = 0;
	DRV_phyread(STATUS_m,SlotID,&CI_status);
	printk("CI Buffnego[%c] OK!!ci_time1=%d\r\n", 65+SlotID, ci_time1);

	*NegoBuff=BuffSize[SlotID];
	ci_info[SlotID].NgBuf=*NegoBuff;
	ci_info[SlotID].linkID=0;
	ci_info[SlotID].SessionProfile=0;
	if(create_tpl(SlotID)<0)
	{
		printk("Error: Doesn't create Transport Session\n");
		ci_info[SlotID].NgBuf=*NegoBuff=0;
		return (NG);
	}

	return(OK);

CI_err:
	printk("CI Buffnego[%c] Fail !!\r\n", 65+SlotID);
	*NegoBuff=0;
	return(NG);
}

unsigned char ConfigAccessMode(unsigned char AccMode,unsigned char SlotID)
{
	unsigned char tmprd;

	tmprd = CiRegRead(CiMAX_DEST,SlotID);
	tmprd &= 0xF9;

	if ( SlotID == SLOTA ) tmprd |= 0x02;
	else                   tmprd |= 0x04;

	CiRegWrite(CiMAX_DEST,tmprd,SlotID);
	if ( AccMode == IOACCESS )
	{
		if( SlotID == SLOTA )
		{
			tmprd = CiRegRead(AMOD_CNTL,SlotID);
			tmprd &= 0xF3; //0xFB; 
			tmprd |= 0x04;
			CiRegWrite(AMOD_CNTL,tmprd,SlotID);
		}
		else
		{
			tmprd = CiRegRead(BMOD_CNTL,SlotID);
			tmprd &= 0xF3; //0xFB;
			tmprd |= 0x04;
			CiRegWrite(BMOD_CNTL,tmprd,SlotID);
		}
	}
	else
	{
		if( SlotID == SLOTA )
		{
			tmprd = CiRegRead(AMOD_CNTL,SlotID);
			tmprd &= 0xF3; //0xFB;
			CiRegWrite(AMOD_CNTL,tmprd,SlotID);
		}
		else
		{
			tmprd = CiRegRead(BMOD_CNTL,SlotID);
			tmprd &= 0xF3; //0xFB;
			CiRegWrite(BMOD_CNTL,tmprd,SlotID);
		}
	}
	return(OK);
}

int create_tpl(unsigned char slotid)
{
	unsigned char buf[32];
	unsigned int len=0;
	Module_t mod;
	
	memset(buf,0,32);

	mod.module=slotid;
	mod.configuration=0xFF;
	mod.configuration=detect_module(slotid);
	
	if(mod.configuration==MODULE_NOT_PRESENT)
	{
		printk("ERROR: Module is not present\n");
		return (-1);
	}
	else if(mod.configuration==MODULE_PRESENT)
	{
		sendCreateTC(buf,&len,slotid);
		MessageSend(buf,len,slotid);
		MessageRead(buf,&len,slotid);
		if( (buf[0]==0x01) &&
			(buf[1]==0x00) &&
			(buf[2]==T_CREATE_REPLY) &&
			(buf[3]==0x01) &&
			(buf[4]==0x01) )
		{
			printk("Create Transport Protocol OK\n");
			return 0;
		}
		else
		{
			printk("ERROR: No answer to Transport Protocol create\n");
			return (-1);
		}
	}
	else
	{
		printk("ERROR: Unknown module status\n");
		return (-1);
	}
}

/* Read function */
int q_read (unsigned char device_id, unsigned char *buf, int size)
{
	unsigned int l1=0;
	unsigned int bytes_r=0;
	unsigned char id=0;
	unsigned int total_len_data_available=0;

	//int read_cnt=0; /* for debug, if you need */
	
	if(size==256)
		return 0;
	
	unsigned char tmp_buf[ci_info[device_id].NgBuf];

	id=device_id;

	if(ci_info[id].NgBuf==0)
	{
		D_DEBUG("Error: q_read NgBuf=0\n");
		return 0;
	}
	sendPolling(buf,&l1,id);
	if(MessageSend(buf,l1,id)==NG)
		return (-1);
	l1=32;
	if(MessageRead((unsigned char*)&tmp_buf[0],&l1,id)==NG)
	{
		return (-1);
	}
	if( ((tmp_buf[5]&0x80)==0x80) && (tmp_buf[2]==T_SB) )
	{
		sendT_recv(tmp_buf,&l1);
		if(MessageSend(tmp_buf,l1,id)==NG)
			return (-1);
		l1=ci_info[id].NgBuf;
		if(MessageRead((unsigned char*)&tmp_buf[0],&l1,id)==NG)
			return (-1);
		if(l1>0)
		{
			if( (tmp_buf[1]&0x80)!=0x80)				/* a single packet */
			{
				if( (tmp_buf[3]&0x80)!=0x80)			/* a single byte for len */
				{
					memcpy(buf,&tmp_buf[5],l1-5/*-4*/);	/* l1-5-5: -5->5bytes for header, -4->close sessiontp FIXME */
					bytes_r+=l1-5;
				}
				else /* (tmp_buf[3]&0x7F)-> number of bytes for the length */
				{
					memcpy(buf,&tmp_buf[5+(tmp_buf[3]&0x7F)],l1-5-(tmp_buf[3]&0x7F)/*-4*/);
					bytes_r+=l1-5-(tmp_buf[3]&0x7F)/*-4*/;
				}
			}
			else	/* multiple packets */
			{
				/* the first packet */
				if( (tmp_buf[3]&0x80)!=0x80)
				{
					total_len_data_available=(tmp_buf[3]&0x7F);

					memcpy(buf,&tmp_buf[5],l1-5/*-4*/);	/* l1-5-4: -5->5bytes for header, -4->close sessiontp FIXME */
					bytes_r+=l1-5;
				}
				else	/* (tmp_buf[3]&0x7F)-> number of bytes for the length */
				{
					/* Create the total len of available data */					
					total_len_data_available=0;
					switch((tmp_buf[3]&0x7F))//number of bytes that create the len field
					{
						case 1:
							total_len_data_available=tmp_buf[4];
							break;
						case 2:
							total_len_data_available=tmp_buf[4];
							total_len_data_available=(total_len_data_available<<8)|tmp_buf[5];
							break;
						case 3:
							total_len_data_available=tmp_buf[4];
							total_len_data_available=(total_len_data_available<<8)|tmp_buf[5];
							total_len_data_available=(total_len_data_available<<16)|tmp_buf[6];
							break;
						default:
							total_len_data_available=tmp_buf[4];
							total_len_data_available=(total_len_data_available<<8)|tmp_buf[5];
							total_len_data_available=(total_len_data_available<<16)|tmp_buf[6];
							total_len_data_available=(total_len_data_available<<24)|tmp_buf[7];
							break;

					}

					memcpy(buf,&tmp_buf[5+(tmp_buf[3]&0x7F)],l1-5-(tmp_buf[3]&0x7F)/*-4*/);
					bytes_r=l1-5-(tmp_buf[3]&0x7F)/*-4*/;
				}
				
				do{
					/* header bytes + end session bytes (2+4) that it isn't considerated when the CAM give to the Host the total len of available data */
					/* this is used if the cam doesn't give me the len of sigle multiple-packet in SIZE_H and SIZE_L */
					if( (total_len_data_available-bytes_r+6) > ci_info[id].NgBuf) 
						l1=ci_info[id].NgBuf;
					else
						l1=(total_len_data_available-bytes_r+6);

					if(MessageRead((unsigned char*)&tmp_buf[0],&l1,id)==NG)
					{
						return (-1);
					}
					memcpy(&buf[bytes_r],&tmp_buf[2],l1-2/*-4*/);	/*2 bytes of header,4 bytes for close session */
					bytes_r+=l1-2/*-4*/;
				}while( (tmp_buf[1]&0x80)==0x80);
			}
			return (bytes_r);
		}
		else
			return 0;
	}
	else
		return 0;
}

/* write function */
int q_write (unsigned char device_id, unsigned char *buf, int size)
{
	unsigned int l1,l2;
	unsigned int bytes_w=0;
	unsigned char buf1[size+4+5];	/* 4:header 5:max len for the moment */
	unsigned char res0=0;
	unsigned char id=0;
#ifdef PRINT_WRITE_LOW
	unsigned int debugppp=0;
#endif

	id=device_id;

	if(ci_info[id].NgBuf<9)	/* checks value to prevent impossible values too */
	{
		D_DEBUG("Error: q_write NgBuf=0\n");
		return 0;
	}

	l1=size;
	sendT_send_last(buf1,&l1,SINGLE_PKT);
	D_DEBUG("\n");
	D_DEBUG("Data to write with the header: %d\n",l1);
	D_DEBUG("\n");

	if(size<=(ci_info[id].NgBuf-l1)) //FIXME: ????? Approximate...
	{
		l1=size;
		sendT_send_last(buf1,&l1,SINGLE_PKT);
		memcpy(&buf1[l1],buf,size);

#ifdef PRINT_WRITE_LOW
		printk("\n");
		printk("Header bytes: %d\n",l1);
		printk("DATA TO WRITE SINGLE ...%d bytes, %d total\n",size,size+l1);
		for(debugppp=0;debugppp<size;debugppp++)
			printk("[%d]0x%02X, ",debugppp,(unsigned char)buf1[debugppp]);
		printk("\n");
		debugppp=0;
#endif

		res0=MessageSend((unsigned char*)buf1,l1+size,id);
		if(res0!=OK)
		{
			D_DEBUG("Error in write data (first MessageSend)\n");
			return 0;	/* No write data */
		}
		l2=32;
		MessageRead((unsigned char*)&buf1[0],&l2,id);
		D_DEBUG("Write  %d bytes\n",size); 
		return size;
	}	
	else
	{
#ifdef PRINT_WRITE_LOW
		unsigned int pkt=0; 
#endif
		l1=size;		
		sendT_send_last(buf1,&l1,MULTI_PKT);
		memcpy(&buf1[l1],buf,(ci_info[id].NgBuf-l1/*9*/));

#ifdef PRINT_WRITE_LOW
		printk("\n");
		printk("Header bytes: %d\n",l1);
		printk(" ...bytes_w=%d \n",bytes_w);
		printk("DATA TO WRITE M 0  ...%d bytes\n",(ci_info[id].NgBuf-l1/*9*/));
		for(debugppp=0;debugppp<(ci_info[id].NgBuf);debugppp++)
			printk("[%d]0x%02X, ",debugppp,(unsigned char)buf1[debugppp]);
		printk("\n");
		debugppp=0;
#endif

		res0=MessageSend((unsigned char*)buf1,ci_info[id].NgBuf,id);
		if(res0!=OK)
		{
			D_DEBUG("Error in write data (second MessageSend)\n");
			return 0;	/* No write data */
		}		
		bytes_w+=(ci_info[id].NgBuf-l1/*9*/);
        D_DEBUG("bytes_w=%d\n",bytes_w);

		while( (size-bytes_w) > (ci_info[id].NgBuf-2) )
		{
			buf1[0]=0x01;
			buf1[1]=0x80;
			memcpy(&buf1[2],&buf[bytes_w],(ci_info[id].NgBuf-2));

#ifdef PRINT_WRITE_LOW
			printk("\n");
			printk("Header bytes: 2\n");
			printk(" ...bytes_w=%d \n",bytes_w);
			printk("DATA TO WRITE M %d  ...%d bytes\n",++pkt,(ci_info[id].NgBuf-2));
			for(debugppp=0;debugppp<(ci_info[id].NgBuf);debugppp++)
				printk("[%d]0x%02X, ",debugppp,(unsigned char)buf1[debugppp]);
			printk("\n");
			debugppp=0;		
#endif

			res0=MessageSend((unsigned char*)buf1,ci_info[id].NgBuf,id);
			if(res0!=OK)
			{
				D_DEBUG("Error in write data (third MessageSend)\n");
				return 0;	/* No write data */
			}			
			bytes_w+=(ci_info[id].NgBuf-2);
        };

		/* Last packet */
		buf1[0]=0x01;
		buf1[1]=0x00;	

		memcpy(&buf1[2],&buf[bytes_w],(size-bytes_w));

#ifdef PRINT_WRITE_LOW
		printk("\n");
		printk("Header byte: 2\n");
		printk("DATA TO WRITE M %d [last]  ...%d bytes\n",++pkt,(size-bytes_w));
		for(debugppp=0;debugppp<(size-bytes_w+2);debugppp++)
			printk("[%d]0x%02X, ",debugppp,(unsigned char)buf1[debugppp]);
		printk("\n");
		debugppp=0;		
#endif	

		res0=MessageSend((unsigned char*)buf1,size-bytes_w+2,id);
		if(res0!=OK)
		{
			D_DEBUG("Error in write data (third MessageSend)\n");
			return 0;	/* No write data */
		}			
		bytes_w+=(size-bytes_w);

l2=32;
MessageRead((unsigned char*)&buf1[0],&l2,id);

		D_DEBUG("Write  %d bytes\n",bytes_w);
		return bytes_w;
	}
	return 0;
}

void sendCreateTC(unsigned char * buff, unsigned int * TxLen, unsigned char id)
{
	if ( ci_info[id].linkID == 0 ) ci_info[id].linkID = 1;
	buff[0] = ci_info[id].linkID;
	buff[1] = 0x0;
	buff[2] = T_CREATE;
	buff[3] = 0x1;
	buff[4] = 0x1;
	*TxLen = 5;
}    

void sendPolling(unsigned char * buff, unsigned int * TxLen, unsigned char id)
{
	buff[0] = ci_info[id].linkID;
	buff[1] = 0x0;
	buff[2] = T_DATA_LAST;
	buff[3] = 0x01; //FIXME: ??? Here there was 'SessionProfile' variable
	buff[4] = 0x1;
	*TxLen = 5;
}    

void sendT_recv(unsigned char * buff, unsigned int * TxLen)
{
	buff[0] = 0x1;
	buff[1] = 0x0;
	buff[2] = T_RCV;
	buff[3] = 0x01;
	buff[4] = 0x1;
	*TxLen = 5;
} 
  
void sendT_send_last(unsigned char * buff, unsigned int * TxLen, unsigned char single)
{
	unsigned char num_byte_len=0;
	unsigned int len=*TxLen;
	unsigned char i=0;

	buff[0] = 0x1;
	if(single==SINGLE_PKT)
		buff[1] = 0x00;
	else
		buff[1] = 0x80;

	buff[2] = T_DATA_LAST;
	if(len<=126)	/* 126 and no 127 because there is the byte "0x01" to add */
	{
		buff[3] = len+0x01;
		buff[4] = 0x1;
		*TxLen = 5;
	}
	else
	{
		len+=0x01;	/* for add the byte "0x01" of the header TP */
		if( ((len>>24)&0xFF) != 0)  num_byte_len=4;
		else if( ((len>>16)&0xFF) != 0)  num_byte_len=3;
		else if( ((len>>8)&0xFF) != 0)  num_byte_len=2;
		else if( (len&0xFF) != 0)  num_byte_len=1;

		buff[3] = num_byte_len|0x80;
		for(i=0;i<num_byte_len;i++)
			buff[4+i] = (len>>((num_byte_len-1-i)*8));
		buff[4+num_byte_len] = 0x01;
		*TxLen = 4+num_byte_len+1;
	}
} 

void sendT_send_more(unsigned char * buff, unsigned int * TxLen)
{
	buff[0] = 0x1;
	buff[1] = 0x0;
	buff[2] = T_DATA_MORE;
	buff[3] = 0x01;
	buff[4] = 0x1;
	*TxLen = 5;
} 

