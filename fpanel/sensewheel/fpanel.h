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
/* some define for protocol st-fpga */

//#define DEBUG1
//#define DEBUG2


#ifdef DEBUG1
#define debug(args...) printk(args)
#else
#define debug(args...)
#endif

#ifdef DEBUG2
#define debug2(args...) printk(args)
#else
#define debug2(args...)
#endif


#define MYVERS "0.0.1-HD"  // 25 sept 2009

/*
 * Just to print an hex dump of a buffer,
 * ptr is the address, m is how many word per line
 * and l how many lines
 */
#define PRINT_BUF(ptr, l, m) ({ \
    int _i, _j; \
    unsigned short *_ptr= (unsigned short *)(ptr); \
    for (_i = 0; _i< (int) l; _i++) { \
	printf("Val[%d]=", m * _i); \
	for (_j = 0; _j < (int) m; _j++) { \
	    printf("%x ", *_ptr);\
	    _ptr++;\
	}\
	printf("\n"); \
    }\
})


#define SUCCESS 0

#define FAIL -1


/* Macro to read/write ST registers */

//#define SCRIVI_REG32(ADDR , VAL) __raw_writel(VAL, phys_to_virt(ADDR))
//#define LEGGI_REG32(ADDR) __raw_readl(phys_to_virt(ADDR))

#define FPGA_BASE                   0x3800000
#define REG1                        0x38
#define REG2                        0x39
#define TS_REG                      0x4

#define MAJOR_NR                    175
#define LED_MAGIC                   0xBC
#define NUMBER_OF_DEVICE            2


/* IOCTL */

#define SET_LED_COLOR			_IO(LED_MAGIC, 11)    // payload = 4byte [what led ?][H][S][V]   LED = N,S,E,W,NE,NW,SE,SW,1,2,3,4
#define SET_LED_OFF			_IO(LED_MAGIC, 12)    // payload = 1byte [what led ?]
#define SET_LED_ALL_PANEL_COLOR		_IO(LED_MAGIC, 13)   // payload = 3byte [H][S][V]
#define SET_LED_ALL_PANEL_OFF		_IO(LED_MAGIC, 14)    // no payload !
#define SET_ALL_BOARD_COLOR		_IO(LED_MAGIC, 15)    // payload = 3byte [H][S][V]
#define SET_ALL_BOARD_OFF		_IO(LED_MAGIC, 16)    // no payload !
#define SET_LED_ALL_COLOR		_IO(LED_MAGIC, 17)    // payload = 3byte [H][S][V]
#define SET_LED_ALL_OFF			_IO(LED_MAGIC, 18)    // no payload !

#define SET_DISABLE_SENSE		_IO(LED_MAGIC, 19)
#define SET_ENABLE_SENSE		_IO(LED_MAGIC, 20)

#define WHEEL				_IO(LED_MAGIC, 22)

#define TS_ROUTING			_IO(LED_MAGIC, 21)


// new add 12.01.2008... 

#define STANDBY                         _IO(LED_MAGIC,25)   /* 2param, 2byte struct...  enter exit standby + led mode(on,off) */


/* Standby parameters struct */
struct sw_standby
{
        unsigned char standby_mode;         // 0: Exit standby, 1: Enter standby
        unsigned char led_group_map;        // bit 0: Panel leds, bit 1: Board leds
}standy_param;



/* *********** */


ssize_t fpanel_read( struct file *filp, char __user *buf, size_t count, loff_t *f_ops );
ssize_t fpanel_write( struct file *filp, const char __user *buf, size_t count, loff_t *f_ops );



