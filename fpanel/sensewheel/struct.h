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

struct sense_p
{
	unsigned char status; // pressed or released
	unsigned char code;	  // button "name" 
}sense;

struct sw_hsv 
{
	unsigned short H;
	unsigned char S;
	unsigned char V;
}comando;


/*
struct send_command
{
	unsigned short cmd : 4;		// 16 commands
	unsigned short led : 4;		// 16 leds
	unsigned short  V  : 8;		// range 0-100

	unsigned short  H  : 9;		// range 0-359
	unsigned short  S  : 7;		// range 0-100

}send_cmd;
*/


struct send_command
{
	unsigned short cmd_led_V;	// 4:4:8 bit
	unsigned short H_S;		// 9:7 bit
}send_cmd;

/* Led commands map  

-- CMD --
   0x1		Set led color
   0x2 	    Set all panel led to color
   0x3 		Set all board led to color
   0x4		Set ALL led to color
   0x5		Disable sense
   0x6      Enable sense

   0x7      Set led off 
   0x8 	    Set all panel led off
   0x9 		Set all board led off
   0xA		Set ALL led off
   0xB      Stand-by mode       --> Send 2 byte to ub 

 
-- led -- 
   0x0 		led N 
   0x1		led S
   0x2 		led E
   0x3      led W
   0x4 		led NE
   0x5		led NW
   0x6		led SE
   0x7		led SW
   0x8 		led central

   0x9		led 1 board
   0xA      led 2 board
   0xB	 	led 3 board
   0xC 		led 4 board


   Example:

   cmd= 0x0   led=0x8  H=300 S=100 V=100  ---> set led central to color HSV !!  
   cmd= 0x2   led=X   H=240 S=80 V=80	  ---> sel all board led to color HSV ! , led field is don't care !! 

*/	


