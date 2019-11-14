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

/*****************************
 * INCLUDES
 *****************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
#include <linux/devfs_fs_kernel.h>
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 19)
#include <linux/page-flags.h>
#include <linux/mm.h>
#endif

#include <linux/poll.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/ioctl.h>
#include <linux/cdev.h>

#include <linux/device.h>
#include <linux/delay.h>

#include "lcd.h"
#include "lcd_private.h"


#if defined CONFIG_SH_QBOXHD_MINI_1_0
extern unsigned char lpc_set_bgr(unsigned char val);
unsigned int lcd_id_m=0;
#endif

/**************************************************************************
 * MACROS, GLOBALS & FUNCS DECLARATIONS
 **************************************************************************/


#ifdef CONFIG_SH_QBOXHD_1_0
#define MOD               "-HD"
#elif  CONFIG_SH_QBOXHD_MINI_1_0
#define MOD               "-Mini"
#else
#define MOD               ""
#endif

#define LCD_VERSION       "0.0.12"MOD

/* Struct to write data to the display */
#define	UNDEFINED	0xFF
#define	INVALID		0x00
#define	VALID		0x01
#define	RESTART		0x02

#define	LEN			DISPLAY_WIDTH*DISPLAY_BYPP* DISPLAY_HEIGHT


INT32 debug;                    /**< Turn on/off LCD debugging */
#define dprintk(args...) 					\
    do { 									\
        if (debug) printk("lcd: " args); 	\
    } while (0)

static struct cdev lcd_cdev;    /**< LCD character device */

LCD_CONTROL_BLOCK lcd_cb;       /**< LCD Control Block */

#if !defined CONFIG_SH_QBOXHD_MINI_1_0
enum lcd_model {
	LCD_YM220T,
	LCD_YLM682A
};

enum lcd_model lcd_id = LCD_YM220T;		/**< LCD ID for recognizing different models */
#endif

typedef struct
{
	unsigned char valid_data;
	size_t len_data;
	unsigned char *write_buf;
} Write_Data_t;

static Write_Data_t w_b;

/**************************/
/*        For mmap        */

#include <linux/vmalloc.h>

static char *buf;
static int bsize = 38*PAGE_SIZE;

static struct page *kmisc_vm_nopage(struct vm_area_struct *vma,
unsigned long address, int *type);

static struct vm_operations_struct kmisc_vm_operations = {
	.nopage = kmisc_vm_nopage,
};
/*************************/


static void write_data_function(void);
struct workqueue_struct *queue_write;
DECLARE_WORK(write_fun, write_data_function);

void display_rgbw(unsigned char, unsigned char);
void ctrl_brightness(unsigned int pwr);
/**************************************************************************
 * Register ops
 **************************************************************************/

/**
 * @brief  Write a word to the FPGA
 * @param  data Data
 * @param  offset Offset from FPGA the base address
 */
static void write_reg_fpga(unsigned short data, unsigned short offset)
{
	ctrl_outw(data, lcd_cb.fpga_base_address+(offset*2));
}

/**
 * @brief  Write a WO register
 * @param  base_address The GPIO base address
 * @param  reg The GPIO's register(offset) starting from the base address
 * @param  val 16-bit data
 */
static void reg_writeonly_l(UINT32 base_address, UINT32 reg, unsigned int val)
{
    UINT32 reg_address;
    
    reg_address = base_address + reg;
    ctrl_outl(val, reg_address);
}

/**
 * @brief Set the Register Select. RS input is used to judge serial input data <br>
 *        as display data when RS = H (1) the data is display data <br>
 *        and when RS = L (0) the data is command data.
 * @param rs Register Select
 */
static void reg_set_rs(UINT32 rs)
{
    /* Command data */
    if (!rs)
        reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnOUT, 0x20);
    /* Display data */
    else
        reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnOUT, 0x20);
}

#if !defined CONFIG_SH_QBOXHD_MINI_1_0
/*****************************
 * YM220T
 *****************************/

/**
 * @brief  Write a 16-bit command
 * @param  addr First 8-bit data
 * @param  val Second 8-bit data
 */
static void reg_write_command_ym220t(UINT8 addr, UINT8 val) 
{
    ctrl_outw((addr << 8 | val), lcd_cb.base_address_emi);
}

/**
 * @brief  Write a 16-bit data
 * @param  data_1 First 8-bit data
 * @param  data_2 Second 8-bit data
 */
static void reg_write_data_ym220t(unsigned char data_1,unsigned char data_2)
{ 
    ctrl_outw((data_1 << 8 | data_2), lcd_cb.base_address_emi);
}

/*****************************
 * YLM682a
 *****************************/

/**
 * @brief LCD YLM682a write address
 * @param index Address
 */
static void reg_write_command_ylm682a(UINT16 index)
{
    reg_set_rs(0);
	ctrl_outw((index), lcd_cb.base_address_emi);
    reg_set_rs(1);
}

/**
 * @brief LCD YLM682a write data
 * @param instruction Data
 */
static void reg_write_data_ylm682a(UINT16 instruction)
{
	ctrl_outw((instruction), lcd_cb.base_address_emi);
}

/**************************************************************************
 * Init/Uninit funcs
 **************************************************************************/

 /**
 * @brief Read the LCD ID and save it in the global var lcd_id
 * 		  The YM220T does not support this op
 */
static void lcd_readid(void)
{
	UINT16 res;

    reg_set_rs(0);
	ctrl_outw((0x0), lcd_cb.base_address_emi);
    reg_set_rs(1);
	res = ctrl_inw(lcd_cb.base_address_emi);
	if (res == 0x9325)
		lcd_id = LCD_YLM682A;
	else
		lcd_id = LCD_YM220T;

	switch (lcd_id) {
		case LCD_YLM682A:
			printk("LCD detected: LCD_YLM682A\n");
			break;
		case LCD_YM220T:
		default:
			printk("LCD detected: LCD_YM220T\n");
			break;
	}
}

/*****************************
 * YLM682a
 *****************************/

/**
 * @brief LCD YLM682a write address and data
 * @param index Address
 * @param instruction Data
 */
static void Init_data(UINT16 index, UINT16 instruction)
{
	reg_write_command_ylm682a(index);
    reg_write_data_ylm682a(instruction);
}

/**
 * @brief Set LCD YLM682a initial configuration
 */
static void init_lcd_ylm682a(void)
{
	Init_data(0x00F0,0x0001);
	Init_data(0x001A,0x288A);
	Init_data(0x00F0,0x0000);
	mdelay(50);
	Init_data(0x00E3, 0x3008);	/* Set internal timing */
	Init_data(0x00E7, 0x0012); 	/* Set internal timing */
	Init_data(0x00EF, 0x1231); 	/* Set internal timing */

	//Init_data(0x0001, 0x0100); 	/* set SS and SM bit */
	Init_data(0x0001, 0x0000); 	/* set SS and SM bit */

	Init_data(0x0002, 0x0700); 	/* set 1 line inversion */

	//Init_data(0x0003, 0x1030);	/*0x1030); // set GRAM write direction and BGR=1. */
	Init_data(0x0003, 0x1038);	/*0x1030); // set GRAM write direction and BGR=1. */


	//Init_data(0x0004, 0x0000); /* Resize register */
	//Init_data(0x0008, 0x0207); /*0x0202); // set the back porch and front porch */
	Init_data(0x0008, 0x0202); 	/*0x0202); // set the back porch and front porch */
	Init_data(0x0009, 0x0000); 	/* set non-display area refresh cycle ISC[3:0] */
	//Init_data(0x000A, 0x0000); /* FMARK function */
	//Init_data(0x000C, 0x0000); /* RGB interface setting */
	//Init_data(0x000D, 0x0000); /* Frame marker Position */
	//Init_data(0x000F, 0x0000); /* RGB interface polarity */

	/* Power On sequence */
	Init_data(0x0010, 0x0000);	/* SAP, BT[3:0], AP, DSTB, SLP, STB */
	Init_data(0x0011, 0x0007);	/* DC1[2:0], DC0[2:0], VC[2:0] */
	Init_data(0x0012, 0x0000);	/* VREG1OUT voltage */
	Init_data(0x0013, 0x0000);	/* VDV[4:0] for VCOM amplitude */
	Init_data(0x0007, 0x0001);
	mdelay(200); /* Dis-charge capacitor power voltage */
	//Init_data(0x0010, 0x1490);  /*0x1290); // SAP, BT[3:0], AP, DSTB, SLP, STB */
	Init_data(0x0010, 0x1290);  /*0x1290); // SAP, BT[3:0], AP, DSTB, SLP, STB */
	Init_data(0x0011, 0x0227); 	/* Set DC1[2:0], DC0[2:0], VC[2:0] */
	mdelay(50); 				/* Delay 50ms */
	//Init_data(0x0012, 0x0019); /*0x001a);  //0x001d); // External reference voltage= Vci; */
	Init_data(0x0012, 0x001a); 	/*0x001a);  //0x001d); // External reference voltage= Vci; */
	mdelay(50); 				/* Delay 50ms */
	//Init_data(0x0013, 0x1600); /*0x1400); //0x1000); // Set VDV[4:0] for VCOM amplitude */
	Init_data(0x0013, 0x1800); 	/*0x1400); //0x1000); // Set VDV[4:0] for VCOM amplitude */
	Init_data(0x0029, 0x002A); 	/*//0x002E); // 0x0019); //0x0008); // SetVCM[5:0] for VCOMH    */
	Init_data(0x002B, 0x000c); 	/*0x000d); // Set Frame Rate    //test by xen  */
	mdelay(50); 				/* Delay 50ms */
	Init_data(0x0020, 0x0000); 	/* GRAM horizontal Address */
	Init_data(0x0021, 0x0000); 	/* GRAM Vertical Address */

	/* Adjust the Gamma Curve */
	Init_data(0x0030, 0x0004);
	Init_data(0x0031, 0x0607);
	Init_data(0x0032, 0x0006);
	Init_data(0x0035, 0x0302);
	Init_data(0x0036, 0x0004);
	Init_data(0x0037, 0x0107);
	Init_data(0x0038, 0x0001);
	Init_data(0x0039, 0x0307);
	Init_data(0x003C, 0x0203);
	Init_data(0x003D, 0x0004);

	/* Set GRAM area */
	Init_data(0x0050, 0x0000); 	/* Horizontal GRAM Start Address */
	Init_data(0x0051, 0x00EF); 	/* Horizontal GRAM End Address */
	Init_data(0x0052, 0x0000); 	/* Vertical GRAM Start Address */
	Init_data(0x0053, 0x013F); 	/* Vertical GRAM Start Address */
	Init_data(0x0060, 0xa700); 	/* Gate Scan Line */
	Init_data(0x0061, 0x0001); 	/* NDL,VLE, REV */
	Init_data(0x006A, 0x0000); 	/* set scrolling line */

	/* Partial Display Control */
	Init_data(0x0080, 0x0000);
	Init_data(0x0081, 0x0000);
	Init_data(0x0082, 0x0000);
	Init_data(0x0083, 0x0000);
	Init_data(0x0084, 0x0000);
	Init_data(0x0085, 0x0000);

	/* Panel Control */
	Init_data(0x0090, 0x0010);
	Init_data(0x0092, 0x0600);
	Init_data(0x0007, 0x0133); 	/* 262K color and display ON */
}

/**
 * @brief LCD YLM682a RGB565 test
 */
static void lcd_ylm682a_test(void)
{
	UINT16 i, j, color = 0xFFFF;

	Init_data(0x0020,0x0000);
	Init_data(0x0021,0x0000);//AC
	Init_data(0x0050,0x0000);
	Init_data(0x0051,0x00EF);//H address
	Init_data(0x0052,0x0000);
	Init_data(0x0053,0x013F);//V address	
	reg_write_command_ylm682a(0x0022);	

	for (i = 0; i < 240; i++) {
		for(j = 0; j < 320; j++)
			reg_write_data_ylm682a(color);
		color -= 0x0111;
	}
}

/*****************************
 * YM220T
 *****************************/

/**
 * @brief Set LCD initial configuration
 */
static void init_lcd_ym220t(void)
{
    reg_set_rs(0);
    
    reg_write_command_ym220t(0x02,0x00);
    reg_write_command_ym220t(0x03,0x01);   /* Reset */
    reg_write_command_ym220t(0x62,0x05);
    mdelay(20);
  
    reg_write_command_ym220t(0x00,0x40);   /* Output all data as 0 */
    reg_write_command_ym220t(0x4e,0x20);
    mdelay(10);

    reg_write_command_ym220t(0x4e,0x60);
    reg_write_command_ym220t(0x01,0x00);
    mdelay(5);
    
    reg_write_command_ym220t(0x22,0x01);
    mdelay(5);
    
    reg_write_command_ym220t(0x22,0x00);
    mdelay(15);
    reg_write_command_ym220t(0x01,0x02);
    reg_write_command_ym220t(0x27,0x57);
	mdelay(25);
    reg_write_command_ym220t(0x28,0x60);
    mdelay(25);
    reg_write_command_ym220t(0x29,0x69);
    mdelay(25);
    reg_write_command_ym220t(0x2a,0x00);
    mdelay(25);
    reg_write_command_ym220t(0x3c,0x00);
    mdelay(25);
    reg_write_command_ym220t(0x3d,0x01);
    mdelay(25);
    reg_write_command_ym220t(0x3e,0x5c);
    mdelay(25);
    reg_write_command_ym220t(0x3f,0x87);
    mdelay(25);
    reg_write_command_ym220t(0x40,0x2f);
    mdelay(25);

	reg_write_command_ym220t(0x4b,0x03);
	reg_write_command_ym220t(0x4c,0x01);
	reg_write_command_ym220t(0x4f,0x04);
	reg_write_command_ym220t(0x50,0x26);
	reg_write_command_ym220t(0x53,0x19);
	reg_write_command_ym220t(0x54,0x22);
	reg_write_command_ym220t(0x55,0x0f);
	reg_write_command_ym220t(0x56,0x18);
	reg_write_command_ym220t(0x57,0x05);
	reg_write_command_ym220t(0x58,0x0e);

	reg_write_command_ym220t(0x34,0x70);
	reg_write_command_ym220t(0x30,0x02);
	reg_write_command_ym220t(0x35,0x00);
	reg_write_command_ym220t(0x31,0x00);
	reg_write_command_ym220t(0x33,0x57);
	reg_write_command_ym220t(0x32,0x07);
	reg_write_command_ym220t(0x37,0x60);
	reg_write_command_ym220t(0x36,0x60);	
	reg_write_command_ym220t(0x05,0x00);

	reg_write_command_ym220t(0x06,0x00);
    reg_write_command_ym220t(0x00,0x00);

// 	reg_write_command_ym220t(0x07,0x00);

	reg_write_command_ym220t(0x4d,0x10);
	reg_write_command_ym220t(0x26,0xef);
	mdelay(10);

	reg_write_command_ym220t(0x4d,0x00);
	reg_write_command_ym220t(0x5d,0x18);
	mdelay(15);

	reg_write_command_ym220t(0x01,0x12);
	reg_write_command_ym220t(0x3d,0x05);
	mdelay(25);

	reg_write_command_ym220t(0x4d,0x28);
	reg_write_command_ym220t(0x5d,0xf8);
    mdelay(25);
    
    reg_write_command_ym220t(0x00,0x00);
    mdelay(10);
    
	reg_write_command_ym220t(0x08,0x00);  /* Set the min values of X addr */
	reg_write_command_ym220t(0x00,0x00);
	reg_write_command_ym220t(0x09,0x00);  /* Set the max values of X addr */
	reg_write_command_ym220t(0x00,0xef);
	reg_write_command_ym220t(0x0a,0x00);  /* Set the min values of Y addr */
	reg_write_command_ym220t(0x00,0x00);
	reg_write_command_ym220t(0x0b,0x00);  /* Set the max values of Y addr */
	reg_write_command_ym220t(0x01,0x3f);
	
//     reg_write_command_ym220t(0x01,0x80);    /* Invert X addr, ADX = 1 */
    reg_write_command_ym220t(0x01,0x40);  /* Invert Y addr, ADR = 1 */
    
	reg_write_command_ym220t(0x06,0x00);  /* Set X addr */
	reg_write_command_ym220t(0x00,0x00);    
	reg_write_command_ym220t(0x07,0x00);  /* Set Y addr */
	reg_write_command_ym220t(0x00,0x00);
	
	reg_write_command_ym220t(0x05,0x10);
    reg_write_command_ym220t(0x05,0x04);  /* Increment Y addr: rotate display */
}
#else
/* If the bus EMI is inverted  (D7->D0, D0->D7) */
//#define SWAP_BIT(x)	((x<<7)&0x80) | ((x<<5)&0x40) |  ((x<<3)&0x20) |  ((x<<1)&0x10) |  ((x>>7)&0x01) |  ((x>>5)&0x02) |  ((x>>3)&0x04) |  ((x>>1)&0x08)
#define SWAP_BIT(x)	(x)
void WriteCOM(unsigned char a)
{
	unsigned char data=0;
	data=a;
    reg_set_rs(0);
	ctrl_outb((data), lcd_cb.base_address_emi);
    reg_set_rs(1);
}

void WriteDAT(unsigned char b)
{
	unsigned char data=0;
	data=b;
    reg_set_rs(1);
	ctrl_outb((data), lcd_cb.base_address_emi);
}
/****************************/
/*	Read id for QBoxHD Mini */
/****************************/
static unsigned short read_DA_id_reg(void)
{
	unsigned short res_m=0;
	unsigned short res_da=0;

    /* Reset LCD */
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnOUT, 0x10);
    mdelay(100);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnOUT, 0x10);
    mdelay(100);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnOUT, 0x10);
    mdelay(120);
	WriteCOM(0x11); //Exit Sleep
	mdelay(100);

	/* DA */
	WriteCOM(0xDA);	//manufacturer ID
	res_da=res_m=ctrl_inw(lcd_cb.base_address_emi);
	lcd_id_m|=((res_m>>8)&0xFF);
	dprintk("Value of DA register (manufacturer ID): 0x%04X\n",(res_m>>8));
	lcd_id_m=(lcd_id_m<<8);

	/* DB */
	WriteCOM(0xDB);
	res_m=0;
	res_m=ctrl_inw(lcd_cb.base_address_emi);
	lcd_id_m|=((res_m>>8)&0xFF);
	dprintk("Value of DB register: 0x%04X\n",res_m);
	lcd_id_m=(lcd_id_m<<8);

	/* DC */
	WriteCOM(0xDC);
	res_m=0;
	res_m=ctrl_inw(lcd_cb.base_address_emi);
	lcd_id_m|=((res_m>>8)&0xFF);
	dprintk("Value of DC register: 0x%04X\n",res_m);
	printk("Value of LCD id: 0x%08X\n", lcd_id_m);

	return res_da;
}

/*****************************
 * FDG177
 ****************************/
static void init_lcd_fdg177(void)
{
	printk("\n");
	printk("Init of fdg177 display\n");
	printk("\n");

    /* Reset LCD */
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnOUT, 0x10);
    mdelay(100);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnOUT, 0x10);
    mdelay(100);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnOUT, 0x10);

    mdelay(120);

	WriteCOM(0x11); //SLEEP OUT&BOOST ON

	mdelay(50);
	WriteCOM(0xff);	//Vcom 4  Level  control
	WriteDAT(0x40);
	WriteDAT(0x01);	//01
	WriteDAT(0x1a);	

	WriteCOM(0xd9);	//close IDLE  EEPROM Control Status
	WriteDAT(0x60);
	WriteCOM(0xc7);	//EEPROM Control Status
	WriteDAT(0x90);
	mdelay(100);

	WriteCOM(0xB1);	//Frame Rate Control normal mode/full color
	WriteDAT(0x00);
	WriteDAT(0x06);
	WriteDAT(0x03);

	WriteCOM(0xB2);	//Frame Rate Control 8-colors
	WriteDAT(0x00);
	WriteDAT(0x06);
	WriteDAT(0x03);

	WriteCOM(0xB3);	//Frame Rate Control partial mode/full colors
	WriteDAT(0x00);
	WriteDAT(0x06);
	WriteDAT(0x03);
	WriteDAT(0x00);
	WriteDAT(0x06);
	WriteDAT(0x03);

	WriteCOM(0xB4);	//Display Inversion Control
	WriteDAT(0x02);
	
	WriteCOM(0xB6);	//Display Function set 5
	WriteDAT(0x15);
	WriteDAT(0x02);

	WriteCOM(0xC0);	//Power Control 1
	WriteDAT(0x02);
	WriteDAT(0x70);

	WriteCOM(0xC1);	//Power Control 2
	WriteDAT(0x05);

	WriteCOM(0xC2);	//Power Control 3
	WriteDAT(0x01);
	WriteDAT(0x02);

	WriteCOM(0xC3);	//Power Control 4
	WriteDAT(0x02);
	WriteDAT(0x07);

	WriteCOM(0xC4);	//Power Control 5
	WriteDAT(0x01);
	WriteDAT(0x02);

	WriteCOM(0xFC);	//
	WriteDAT(0x11);
	WriteDAT(0x15);

	WriteCOM(0xC5);	//VCOM Control 1
	WriteDAT(0x39);
	WriteDAT(0x46);

	WriteCOM(0xe0); //GAMCTRP1
	WriteDAT(0x09);
	WriteDAT(0x16);
	WriteDAT(0x09);
	WriteDAT(0x20); //27
	WriteDAT(0x21); //2E
	WriteDAT(0x1B); //25
	WriteDAT(0x13); //1C
	WriteDAT(0x19); //20
	WriteDAT(0x17); //1E
	WriteDAT(0x15); //1A
	WriteDAT(0x1E); //24
	WriteDAT(0x2B); //2D
	WriteDAT(0x04);
	WriteDAT(0x05);
	WriteDAT(0x02);
	WriteDAT(0x0e);

	WriteCOM(0xe1);   //GAMCTRN1
	WriteDAT(0x0b);
	WriteDAT(0x14);
	WriteDAT(0x08);  //09
	WriteDAT(0x1E);  //26
	WriteDAT(0x22);  //27
	WriteDAT(0x1D);  //22
	WriteDAT(0x18);  //1C
	WriteDAT(0x1E);  //20
	WriteDAT(0x1B);  //1D
	WriteDAT(0x1A);  //1A
	WriteDAT(0x24);  //25
	WriteDAT(0x2B);  //2D
	WriteDAT(0x06);
	WriteDAT(0x06);
	WriteDAT(0x02);
	WriteDAT(0x0f);

	WriteCOM(0x36);   //Memory data access control MY MX MV ML RGB MH - -
	WriteDAT(0x60);//(0xc0);   //C8 1=BGR 0=RGB

	WriteCOM(0x2A);   //CASET (2Ah): Column Address Set
	WriteDAT(0x00);
	WriteDAT(0x00);
	WriteDAT(0x00);
	//WriteDAT(0x7f);
	WriteDAT(0x9f);

	WriteCOM(0x2B);    //RASET (2Bh): Row Address Set
	WriteDAT(0x00);
	WriteDAT(0x00);
	WriteDAT(0x00);
	//WriteDAT(0x9f);
	WriteDAT(0x7f);

	WriteCOM(0x3a);   //Interface pixel format
	WriteDAT(0x05);   //011-12-bit/pixel  101-16bit   110-18bit

	WriteCOM(0x29);	// Display on
	//delayms(100);
	mdelay(100);
	WriteCOM(0x2C);	//Write memory
}

/*****************************
 * ILI9163b
 ****************************/
static void init_lcd_ili9163(void)
{
	printk("\n");
	printk("Init of ili9163 display\n");
	printk("\n");

    /* Reset LCD */
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnOUT, 0x10);
    mdelay(100);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnOUT, 0x10);
    mdelay(100);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnOUT, 0x10);

    mdelay(120);

	WriteCOM(0x11); //Exit Sleep
	mdelay(20);

	WriteCOM(0x26); //Set Default Gamma
	WriteDAT(0x04);

	WriteCOM(0xB1);//Set Frame Rate
	WriteDAT(0x0e); 
	WriteDAT(0x14);

	WriteCOM(0xC0); //Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
	WriteDAT(0x08);//08  4.5V
	WriteDAT(0x05);//00//2.75V

	WriteCOM(0xC1); //Set BT[2:0] for AVDD & VCL & VGH & VGL
	WriteDAT(0x05);///6* 3*

	WriteCOM(0xC5); //Set VMH[6:0] & VML[6:0] for VOMH & VCOML
	WriteDAT(35); //38   3.9V
	WriteDAT(90);  //40//3d   -0.9V

	WriteCOM(0xC7); //Set VMH[6:0] & VML[6:0] for VOMH & VCOML
	WriteDAT(0xb8); //38

	WriteCOM(0x36); 
	WriteDAT(0x68);// rotate 90° and RGB //(0x60);//(0x08);  //08
	//WriteCOM(0xb7); 
	//WriteDAT(0x01);  //08
	
	WriteCOM(0x3a); //Set Color Format
	WriteDAT(0x05);

	WriteCOM(0x2A); //Set Column Address
	WriteDAT(0x00);
	WriteDAT(0x00);
	WriteDAT(0x00);
//	WriteDAT(0x7F);
	WriteDAT(0x9F);

	WriteCOM(0x2B); //Set Page Address
	WriteDAT(0x00);
	WriteDAT(0x00);
	WriteDAT(0x00);
//	WriteDAT(0x9F);
	WriteDAT(0x7F);
	//WriteCOM(0xEc);
	//WriteDAT(0x0c);//p1

	WriteCOM(0xB4); 
	WriteDAT(0x00);

	WriteCOM(0xf2); //Enable Gamma bit
	WriteDAT(0x01);

	WriteCOM(0xE0);
	WriteDAT(0x36);//p1
	WriteDAT(0x29);//p2
	WriteDAT(0x12);//p3
	WriteDAT(0x22);//p4
	WriteDAT(0x1C);//p5
	WriteDAT(0x15);//p6
	WriteDAT(0x42);//p7
	WriteDAT(0xB7);//p8
	WriteDAT(0x2F);//p9
	WriteDAT(0x13);//p10
	WriteDAT(0x12);//p11
	WriteDAT(0x0A);//p12
	WriteDAT(0x11);//p13
	WriteDAT(0x0B);//p14
	WriteDAT(0x06);//p15

	WriteCOM(0xE1);
	WriteDAT(0x09);//p1
	WriteDAT(0x16);//p2
	WriteDAT(0x2D);//p3
	WriteDAT(0x0D);//p4
	WriteDAT(0x13);//p5
	WriteDAT(0x15);//p6
	WriteDAT(0x40);//p7
	WriteDAT(0x48);//p8
	WriteDAT(0x53);//p9
	WriteDAT(0x0C);//p10
	WriteDAT(0x1D);//p11
	WriteDAT(0x25);//p12
	WriteDAT(0x2E);//p13
	WriteDAT(0x34);//p14
	WriteDAT(0x39);//p15

	WriteCOM(0x29); // Display On
	WriteCOM(0x2c); // Display On
}
#endif
 
/**
 * @brief Map registers, configure GPIOs and initialize LCD
 */
static int device_init(void)
{
    lcd_cb.base_address_emi  = (unsigned long)ioremap(EMI_BASE_ADDRESS, EMI_IO_SIZE);
    lcd_cb.base_address_pio5 = (unsigned long)ioremap(PIO5_BASE_ADDRESS, PIO5_IO_SIZE);
    lcd_cb.fpga_base_address = (unsigned long)ioremap(FPGA_BASE_ADDRESS, FPGA_IO_SIZE);

    /* Configure LCD reset. Set GPIO 5[4]. Datasheet page 213-217 */
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnC0, 0x10);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnC1, 0x10);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnC2, 0x10);
    
    /* Configure LCD reset. Set GPIO 5[7] -> output */
//    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnC0, 0x80);
//    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnC1, 0x80);
//    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnC2, 0x80);
    
    /* Configure LCD brightness. Set GPIO 5[7] -> input No use it but PWM of FPGA */
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnC0, 0x80);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnC1, 0x80);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnC2, 0x80);
    
	/* Remove the brg */
    ctrl_brightness(0);

    /* Reset LCD */
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnOUT, 0x10);
    mdelay(150);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnOUT, 0x10);
    mdelay(150);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnOUT, 0x10);
    mdelay(150);

    /* Configure Register Select (RS). Set GPIO 5[5]. Datasheet page 213-217 */
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnC0, 0x20);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnC1, 0x20);
    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnC2, 0x20);

#if !defined CONFIG_SH_QBOXHD_MINI_1_0
    /* Init LCD */
	lcd_readid();
	switch (lcd_id) {
		case LCD_YLM682A: /* New LCD */
			init_lcd_ylm682a();
			mdelay(20);
			display_rgbw(0x00,0x00);
			//lcd_ylm682a_test();
			break;
		case LCD_YM220T:
		default:
    		init_lcd_ym220t();
			/* Display a black screen */
			display_rgbw(0x00,0x00);
			break;
	}
#else
	unsigned short id=0;
	id=read_DA_id_reg();
	if( (id>>8) == 0x54)
		init_lcd_ili9163();
	else	//0x5Cxxxx
		init_lcd_fdg177();
#endif
    
	/* Default: brightness ON */
	/* If you use PWM, no set it */
//    reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnOUT, 0x80);
	write_reg_fpga(MAX_BRG,REG_BRIGHTNESS);

    return LCD_NO_ERROR;
}

/**
 * @brief Unmap registers
 */
static void device_uninit(void)
{
    iounmap((void *)lcd_cb.base_address_emi);
    iounmap((void *)lcd_cb.base_address_pio5);
    iounmap((void *)lcd_cb.fpga_base_address);
}

#if !defined CONFIG_SH_QBOXHD_MINI_1_0
/**************************************************************************
 * Functions needed by the Driver File Operations
 **************************************************************************/
void Enter_standby_ylm682a(void)
{
	Init_data(0x0007, 0x0131); // Set D1=0, D0=1
	mdelay(10);
	Init_data(0x0007, 0x0130); // Set D1=0, D0=0
	mdelay(10);
	Init_data(0x0007, 0x0000); // display OFF
	//************* Power OFF sequence **************//
	Init_data(0x0010, 0x0080); // SAP, BT[3:0], APE, AP, DSTB, SLP
	Init_data(0x0011, 0x0000); // DC1[2:0], DC0[2:0], VC[2:0]
	Init_data(0x0012, 0x0000); // VREG1OUT voltage
	Init_data(0x0013, 0x0000); // VDV[4:0] for VCOM amplitude
	mdelay(200); // Dis-charge capacitor power voltage
	Init_data(0x0010, 0x0082); // SAP, BT[3:0], APE, AP, DSTB, SLP
}

void Exit_standby_ylm682a(void)
{
	//*************Power On sequence ******************//
	Init_data(0x0010, 0x0080); // SAP, BT[3:0], AP, DSTB, SLP
	Init_data(0x0011, 0x0000); // DC1[2:0], DC0[2:0], VC[2:0]
	Init_data(0x0012, 0x0000); // VREG1OUT voltage
	Init_data(0x0013, 0x0000); // VDV[4:0] for VCOM amplitude
	Init_data(0x0007, 0x0001);
	mdelay(200); // Dis-charge capacitor power voltage
	Init_data(0x0010, 0x1290); // SAP, BT[3:0], AP, DSTB, SLP, STB

	Init_data(0x0011, 0x0227); // R11h=0x0221 at VCI=3.3V DC1[2:0], DC0[2:0], VC[2:0]
	mdelay(50); // Delay 50ms
	Init_data(0x0012, 0x001A); // External reference voltage =Vci;
	mdelay(50); // Delay 50ms
	Init_data(0x0013, 0x1800); // R13h=0x1200 when R12=009D VDV[4:0] for VCOM amplitude
	Init_data(0x0029, 0x002A); // R29h=0x000C when R12=009D VCM[5:0] for VCOMH
	mdelay(50); // Delay 50ms
	Init_data(0x0007, 0x0133); // 262K color and display ON
	
}
/**
 * @brief Enter/Leave the stand-by mode
 * @param stdby 1: Enter, 0: Leave
 */

/******************************************************************/
/* NOTE: This function and relative ioctl were used only in test! */
/******************************************************************/
static void standby(UINT16 stdby)
{
    reg_set_rs(0);

	switch (lcd_id) {
		case LCD_YLM682A:
			if (!stdby)
			{
				Exit_standby_ylm682a();
				display_rgbw(0x00,0x00);
			}
			else if (stdby == 1)
			{
				Enter_standby_ylm682a();
			}
			break;
		case LCD_YM220T:
		default:
			/* Leave stand-by mode */
			if (!stdby)
			{
				/* Init LCD */
				init_lcd_ym220t();
				/* Display a black screen */
				display_rgbw(0x00,0x00);
			}
			/* Enter stand-by mode */
			else if (stdby == 1)
			{
				reg_write_command_ym220t(0x01, 0x00);
				reg_write_command_ym220t(0x01, 0x02);
				reg_write_command_ym220t(0x27, 0x57);
				mdelay(250);
				reg_write_command_ym220t(0x28, 0x60);
				mdelay(250);
				reg_write_command_ym220t(0x29, 0x69);
				mdelay(250);
				reg_write_command_ym220t(0x2A, 0x00);
				mdelay(250);
				reg_write_command_ym220t(0x3C, 0x00);
				mdelay(250);
				reg_write_command_ym220t(0x3D, 0x01);
				mdelay(250);
				reg_write_command_ym220t(0x3E, 0x5C);
				mdelay(250);
				reg_write_command_ym220t(0x3F, 0x87);
				mdelay(250);
				reg_write_command_ym220t(0x40, 0x2F);
				mdelay(250);
				reg_write_command_ym220t(0x5D, 0x20);
				reg_write_command_ym220t(0x4D, 0x00);
				reg_write_command_ym220t(0x4D, 0x20);
				reg_write_command_ym220t(0x4D, 0x28);
				reg_write_command_ym220t(0x5D, 0x60);
				reg_write_command_ym220t(0x5D, 0xF8);
				reg_write_command_ym220t(0x00, 0x40);
				
				reg_write_command_ym220t(0x06, 0x00);
				reg_write_command_ym220t(0x00, 0x00);
				mdelay(100);
				reg_write_command_ym220t(0x4D, 0x00);
				reg_write_command_ym220t(0x5D, 0x18);
				mdelay(150);
				reg_write_command_ym220t(0x01, 0x12);
				reg_write_command_ym220t(0x3D, 0x05);
				mdelay(250);
				reg_write_command_ym220t(0x4D, 0x28);
				reg_write_command_ym220t(0x5D, 0xF8);
				mdelay(100);
			}
			break;
	}
}
#endif

/********************************************************************/
/* 'x' and 'y' are refered to the hw orig. For us they are inverted */
/********************************************************************/
int set_pos(int x,int y)
{
	unsigned int y_pos=0;

	reg_set_rs(0);

#if !defined CONFIG_SH_QBOXHD_MINI_1_0
	switch (lcd_id) {
		case LCD_YLM682A:
			/* Set the X addr. When in qvga this is equivalent to Y (240 pixels) */
			Init_data(0x0020,x);// Horizontal -> for us Vertical 

			/* Set the Y addr. When in qvga this is equivalent to Y (240 pixels) */
			Init_data(0x0021,y);// Vertical -> for us Horizontal

			reg_write_command_ylm682a(0x0022);
			break;
		case LCD_YM220T:
		default:
			/* Set the X addr. When in qvga this is equivalent to Y (240 pixels) */
			reg_write_command_ym220t(0x06,0x00);
			reg_write_command_ym220t(0x00,239 - (UINT8)x);

			/* Set the Y addr. When in qvga this is equivalent to Y (240 pixels) */
			y_pos=y;

			reg_write_command_ym220t(0x07,0x00);
			reg_write_command_ym220t((y_pos>>8),(y_pos&0xFF));
			break;
	}
#else
	WriteCOM(0x2A);   //CASET (2Ah): Column Address Set
	WriteDAT(0x00);
	WriteDAT(y);
	WriteDAT(0x00);
	WriteDAT(0x9f);

	WriteCOM(0x2B);    //RASET (2Bh): Row Address Set
	WriteDAT(0x00);
	WriteDAT(x);
	WriteDAT(0x00);
	WriteDAT(0x7f);
	WriteCOM(0x2C);	//Write memory
	reg_set_rs(1);
#endif
	return 0;
}

#if !defined CONFIG_SH_QBOXHD_MINI_1_0
static void write_data_function(void)
{
    int x, y;
	/* Set to origin */
	set_pos(0,0);

	/* Write data */
	for ( y=0; y<DISPLAY_HEIGHT; y++)
	{
		/* Set the X addr. When in qvga this is equivalent to Y (240 pixels) */
		if (lcd_id != LCD_YLM682A)
		{
			/* NEW LINE */
			reg_set_rs(0);
			reg_write_command_ym220t(0x06,0x00);
			reg_write_command_ym220t(0x00,(DISPLAY_HEIGHT-1)-(UINT8)y);
			/* Send Data */
			reg_set_rs(1);

		}
		/* For LCD_YLM682A, it doesn't need of commands */
		for (x = 0; x < (DISPLAY_WIDTH * DISPLAY_BYPP); x += 4 )
		{
			if (w_b.valid_data == RESTART)
			{
				set_pos(0,0);			
				x = y = 0;
				w_b.valid_data = VALID;
			}
			if(w_b.valid_data == INVALID) break; /* for mmap function */
			ctrl_outl((UINT8)w_b.write_buf[(y * DISPLAY_WIDTH*DISPLAY_BYPP)+x+3] << 24 | 
					(UINT8)w_b.write_buf[(y * DISPLAY_WIDTH*DISPLAY_BYPP)+x+2] << 16 | 
					(UINT8)w_b.write_buf[(y * DISPLAY_WIDTH*DISPLAY_BYPP)+x+1] << 8 | 
					(UINT8)w_b.write_buf[(y * DISPLAY_WIDTH*DISPLAY_BYPP)+x+0], 
					lcd_cb.base_address_emi);
		}
	}
	w_b.valid_data=UNDEFINED;
}

/**
 * @brief Paint the whole LCD with a solid colour
 * @param data1 First 8-bit data
 * @param data2 Second 8-bit data
 */
void display_rgbw(unsigned char data1, unsigned char data2)
{
    UINT32 i,j;
    UINT16 data;

    reg_set_rs(1);

	/* The followed commands are used to set the orig (0,0) and to create the area to draw */
	if (lcd_id == LCD_YLM682A) {
		Init_data(0x0020,0x0000);
		Init_data(0x0021,0x0000);//AC
		Init_data(0x0050,0x0000);
		Init_data(0x0051,0x00EF);//H address
		Init_data(0x0052,0x0000);
		Init_data(0x0053,0x013F);//V address	
		reg_write_command_ylm682a(0x0022);	
	}
	data = data1 << 8 | data2;

	for (i = 0; i < DISPLAY_WIDTH; i++) {
		for (j = 0; j < DISPLAY_HEIGHT; j++) {
			if (lcd_id == LCD_YLM682A)
				reg_write_data_ylm682a(data);
			else
				reg_write_data_ym220t(data1, data2);
		}
	}
}

#else
static void write_data_function(void)
{
    int x, y;
	/* Set to origin */
	set_pos(0,0);
	/* Write data */

	for ( y=0; y<DISPLAY_HEIGHT; y++)
	{
		/* For LCD_YLM682A, it doesn't need of commands */
		for (x = 0; x < (DISPLAY_WIDTH * DISPLAY_BYPP); x+=2 )
		{
			if (w_b.valid_data == RESTART)
			{
				set_pos(0,0);			
				x = y = 0;
				w_b.valid_data = VALID;
			}
			if(w_b.valid_data == INVALID) break; /* for mmap function */
			ctrl_outb(w_b.write_buf[(y * DISPLAY_WIDTH*2)+x+1],lcd_cb.base_address_emi);
			ctrl_outb(w_b.write_buf[(y * DISPLAY_WIDTH*2)+x],lcd_cb.base_address_emi);

		}
	}

	w_b.valid_data=UNDEFINED;
}

void display_rgbw(unsigned char data1, unsigned char data2)
{
    UINT32 i,j;
    UINT16 data;

	WriteCOM(0x2C);	//Write memory
    reg_set_rs(1);
//	data = data1 << 8 | data2;
	for (i = 0; i < DISPLAY_WIDTH; i++)
	{
		for (j = 0; j < DISPLAY_HEIGHT; j++)
		{
			ctrl_outb(data1,lcd_cb.base_address_emi);
			ctrl_outb(data2,lcd_cb.base_address_emi);
		}
	}
}
#endif
 
/**
 * @brief Turn on/off the brightness pin. The FPGA controls this op
 * @param pwr Turn on/off the brigthness
 */
void ctrl_brightness(unsigned int pwr)
{
#if defined CONFIG_SH_QBOXHD_MINI_1_0
	unsigned char ver=0;
	if (pwr == BRIGHTNESS_ON)
	{
		ver=lpc_set_bgr(32);//max backlight
		if(ver<2)
			write_reg_fpga(MAX_BRG, REG_BRIGHTNESS);
	}
	if (pwr == BRIGHTNESS_OFF)
	{
		ver=lpc_set_bgr(0);
		if(ver<2)
			write_reg_fpga(MIN_BRG, REG_BRIGHTNESS);
	}
#else
	if (pwr == BRIGHTNESS_ON)
	//	reg_writeonly_l(lcd_cb.base_address_pio5, PIO_CLR_PnOUT, 0x80);
		write_reg_fpga(MAX_BRG, REG_BRIGHTNESS);
	if (pwr == BRIGHTNESS_OFF)
	//	reg_writeonly_l(lcd_cb.base_address_pio5, PIO_SET_PnOUT, 0x80);
		write_reg_fpga(MIN_BRG, REG_BRIGHTNESS);
#endif
}

/**
 * @brief Regulate the brightness. The FPGA controls this op
 * @param value The brightness value
 */
void reg_brightness(unsigned short value)
{
#if defined CONFIG_SH_QBOXHD_MINI_1_0
	unsigned char ver=0;
	if(value==63)
		ver=lpc_set_bgr(32);//max backlight
	else
		ver=lpc_set_bgr((value/2));
	if(ver<2)
		write_reg_fpga(value, REG_BRIGHTNESS);	
#else
	write_reg_fpga(value, REG_BRIGHTNESS);
#endif
}

/**************************************************************************
 * Driver File Operations
 **************************************************************************/

/**
 * @brief POSIX open
 * @param inode i_rdev contains the dev char number
 * @param file Ptr to struct file
 * @return =0 success <br>
 *         <0 if any error occur
 */
static int lcd_open (struct inode *inode, struct file *file)
{
    int rc = 0;
    
    if (!MINOR(inode->i_rdev)) {
        if (lcd_cb.driver_inuse == 0)
            lcd_cb.driver_inuse++;
        else {
            PERROR("Device is busy\n");
            rc = -EBUSY;
        }
    }
    else {
        PERROR("Could not open device for LCD\n");
        rc = -ENODEV;
    }
    
    return (rc);
}

/**
 * @brief POSIX release
 * @param inode i_rdev contains the dev char number
 * @param file Ptr to struct file
 * @return =0 success <br>
 *         <0 if any error occur
 */
static int lcd_release (struct inode *inode, struct file *file)
{
    int rc = 0;

    PDEBUG("Releasing device\n");
    if (!MINOR(inode->i_rdev)) {
        if (lcd_cb.driver_inuse > 0)
            lcd_cb.driver_inuse--;
        else {
            PERROR("lcd device is not opend: %d\n", lcd_cb.driver_inuse);
            rc = -EINVAL;
        }
    }
    else {
        PERROR("Could not release lcd device\n");
        rc = -ENODEV;
    }

    return rc;
}

/**
 * @brief  POSIX read
 * @param  file Ptr to the file handle
 * @param  buf Ptr to the input data buffer
 * @param  len Number of characters of input buffer
 * @param  offset Offset of the file
 * @return >=0 the number of read characters <br>
 *         <0 if any error occur
 */
static ssize_t
lcd_read (struct file *file, char *buf, size_t len, loff_t *offset)
{
    return -EINVAL;
}

static ssize_t lcd_write(struct file *file, 
                         const char *buf, 
                         size_t len, 
                         loff_t *offset)
{
    if (len != (DISPLAY_WIDTH*DISPLAY_BYPP* DISPLAY_HEIGHT))
        return -EINVAL;
	
#if defined CONFIG_SH_QBOXHD_MINI_1_0
	/* It is a temporarily work around for old enigma in QboxHD_mini */
	msleep(3);
#endif

	memcpy(w_b.write_buf,buf,len);
	w_b.len_data=len;

	if(w_b.valid_data==VALID) {
		dprintk("RESTART\n");
		w_b.valid_data=RESTART;
	}
	else {
		dprintk("NEW WRITE\n");
		w_b.valid_data=VALID;
		queue_work(queue_write, &write_fun);
	}

    return len;
}

/**
 * @brief  POSIX ioctl
 * @param inode i_rdev contains the dev char number
 * @param file Ptr to struct file
 * @param cmd Special operation number of this device
 * @param arg IO parameters of ioctl
 * @return =0 success <br>
 *         <0 if any error occur
 */
static int lcd_ioctl(struct inode *inode, 
                     struct file *file,
                     unsigned int cmd, 
                     unsigned long arg)
{
    int err = 0;

    /* Check the args before the switch */
    if (_IOC_TYPE(cmd) != LCD_IOC_MAGIC)
        return -ENOTTY;
    if (_IOC_NR(cmd) > LCD_IOC_NR)
        return -ENOTTY;

    switch (cmd) {
        case LCD_IOCTL_RS://0
            if (arg == 0 || arg == 1)
                reg_set_rs((UINT32)arg);
            else
                err = -EINVAL;
            break;        
        case LCD_IOCTL_STANDBY: //1 /* for the moment, it is not used */ 
#if defined CONFIG_SH_QBOXHD_MINI_1_0
			printk("Not implemented yet\n");
#else
            if (arg == 0 || arg == 1) {
				if(arg==1)
            		ctrl_brightness((UINT32)0);/* Turn on the brightness */
                standby((UINT32)arg);
				if(arg==0)
                	ctrl_brightness((UINT32)1);/* Turn off the brightness */
            }
            else
                err = -EINVAL;
#endif
            break;
        case LCD_IOCTL_ON_OFF_BRIGHTNESS://3
            if (arg == 0 || arg == 1)
                ctrl_brightness((UINT32)arg);
            else
                err = -EINVAL;
            break;

        case LCD_IOCTL_REG_BRIGHTNESS://4
            if (arg < 0 || arg > 63) {
            	PERROR("Wrong parameter: 0 < pram < 63 (in decimal format)\n");
            	err = -EINVAL;
            }
            else
				reg_brightness((unsigned short) arg);
            break;

        case LCD_IOCTL_MMAP_WRITE://5
            if (arg == LCD_STOP)
			{	
				w_b.valid_data=INVALID;
            }
			else if (arg == LCD_START)
			{
				dprintk("NEW WRITE\n");
				w_b.len_data=LEN;
				w_b.valid_data=VALID;
				queue_work(queue_write, &write_fun);		
			}
            else
            	PERROR("Wrong parameter: LCD_START or LCD_STOP\n");
            	err = -EINVAL;
            break;
		case LCD_IOCTL_READ_ID:
#if defined CONFIG_SH_QBOXHD_MINI_1_0
			{
				unsigned int id_m=0;
#if 0
				lcd_id_m=0;
				read_DA_id_reg();
#endif	
				if((lcd_id_m>>16)==0x5C)//fgd177
					id_m=1;
				else if((lcd_id_m>>16)==0x54)//ili9163
					id_m=2;	
				else		
					id_m=0;

				copy_to_user((void *) arg,
                           (const void *) &id_m,
                           sizeof(unsigned int));
			}
#else
			printk("Not implemented yet\n");
#endif		

			break;
#if 0
        case LCD_IOCTL_NEWLINE: //2
            if (arg >= DISPLAY_HEIGHT)
                err = -EINVAL;
            else {
                reg_set_rs(0);
                // Set the X addr. When in qvga this is equivalent to Y (240 pixels) 
                reg_write_command_ym220t(0x06,0x00);
                reg_write_command_ym220t(0x00,239 - (UINT8)arg);
            }
            break;
#endif
        default:
            err = -ENOSYS;
    }
    return err;
}

static struct page *kmisc_vm_nopage(struct vm_area_struct *vma,
unsigned long address, int *type)
{
	struct page *pg;
	unsigned long pg_base = vma->vm_pgoff<<PAGE_SHIFT;
	unsigned long ofs = pg_base+(address-vma->vm_start);

	ofs &= PAGE_MASK;
	//printk(KERN_INFO "%s: page fault at %08lx [vma %08lx-%08lx pgoff %08lx]\n",
	//__func__, address, vma->vm_start, vma->vm_end,
	//vma->vm_pgoff<<PAGE_SHIFT);
	if (ofs>=bsize)
		return NOPAGE_SIGBUS;
	pg = vmalloc_to_page(buf+ofs);
	if (pg)
		get_page(pg);
	if (type)
		*type = VM_FAULT_MINOR;
	return pg;
}

static int lcd_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long pg_base = vma->vm_pgoff<<PAGE_SHIFT;

	if ((pg_base+size)>bsize)
		return -EINVAL;
	if (!(vma->vm_flags&VM_SHARED))
		return -EINVAL;
	vma->vm_flags |= VM_RESERVED;
	vma->vm_ops = &kmisc_vm_operations;
	return 0;
}

static struct file_operations lcd_fops = {
     .owner   = THIS_MODULE,
     .open    = lcd_open,
     .release = lcd_release,
     .read    = lcd_read,
	 .write	  = lcd_write,
     .ioctl   = lcd_ioctl,
     .mmap    = lcd_mmap
};

/**************************************************************************
 * Module init/exit
 **************************************************************************/

static int __init lcd_init(void)
{
    dev_t dev = MKDEV(LCD_MAJOR_NUM, LCD_MINOR_START);

    cdev_init(&lcd_cdev, &lcd_fops);
    lcd_cdev.owner = THIS_MODULE;
    lcd_cdev.ops = &lcd_fops;
    if (cdev_add(&lcd_cdev, dev, 1) < 0) {
        PERROR("Couldn't register '%s' driver\n", LCD_DEVICE_NAME);
        return -1;
    }
    PINFO("Registering device '%s', major '%d'\n", LCD_DEVICE_NAME, LCD_MAJOR_NUM);

	buf = vmalloc(bsize);

	w_b.write_buf=(unsigned char *)buf;
	w_b.valid_data=UNDEFINED;
	w_b.len_data=0;

//	ctrl_brightness(0);

    if (device_init() != LCD_NO_ERROR) {
        PERROR("Couldn't init device\n");
        return -1;
    }

	queue_write = create_singlethread_workqueue("Write_Data_LCD");

	PINFO("Version of LCD driver: %s\n", LCD_VERSION);
    return 0;
}

static void __exit lcd_exit(void)
{
	destroy_workqueue(queue_write);

	vfree(buf);

    device_uninit();

    cdev_del(&lcd_cdev);
    PINFO("Unregistering device '%s'\n", LCD_DEVICE_NAME);
}

module_init(lcd_init);
module_exit(lcd_exit);

module_param(debug, int, S_IRUGO);
MODULE_PARM_DESC(debug, "Turn on/off LCD debugging (default:off)");

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for the QBoxHD Front Panel LCD");
MODULE_AUTHOR("Pedro Aguilar <pedro@duolabs.com>, Michele Cecchin");
MODULE_VERSION(LCD_VERSION);

