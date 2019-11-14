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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>

#include <sys/ioctl.h>
#include "lpc_i2c.h"
unsigned char num_reg=19;
unsigned char valid_reg[]={0x1,0x2,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x20,0x21,0x22,0x23,0x30,0x31,0x32,0x33,0x34};

int main()
{
	int fd;
	int res=-1;
	int value=0;
	short value_pwm;
	Rrgb_Value_t trgb;
	int t=0;

	fd=open("/dev/lpc_0", O_RDWR|O_NONBLOCK);
	if(fd<0)
	{
		printf("\nError: doesn't open the device 'lpc_0'\n\n");
		return 1;	
	}	

	while(1)
	{	
		printf("Choose 'ioctl'...\n");
		printf("1 - IOCTL_READ_VERSION\n");
		printf("2 - IOCTL_READ_TEMP\n");
		printf("3 - IOCTL_FAN_POWER\n");
		printf("4 - IOCTL_PWM_VALUE\n");
		printf("5 - IOCTL_SET_MAX_MIN_TEMP\n");
		printf("6 - IOCTL_SET_RGB\n");
		printf("7 - IOCTL_WAKEUP_IR_CODE\n");
		printf("8 - READ_ALL_VALUE\n");
		printf("9 - Quit\n");		
		printf("---> : ");
		scanf("%d",&res);
		printf("\n");
		if((unsigned char)res>=9)
		{
			res=-1;
			return 1;
		}
		switch(res)
		{
			case 1:
				ioctl(fd, IOCTL_READ_VERSION, &value);
				printf("Current version: %d\n",value);
				printf("---> : ");
				break;
			case 2:
				ioctl(fd, IOCTL_READ_TEMP, &value);
				printf("Current temperature: %d\n",value);
				printf("---> : ");
				break;
			case 3:
				printf("Choose the value to write..\n");
				printf("---> : ");
				value=-1;
				scanf("%d",&value);
				printf("\n");
				if((value<0) || (value>32))
					printf("Wrong option\n");
				else
					ioctl(fd, IOCTL_FAN_POWER, &value);
				break;
			case 4:
				value_pwm=0;
				printf("Choose ih the pin is input or output:\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("\n");
				if( (res==INPUT_PWM) || (res==OUTPUT_PWM) )
				{
					value_pwm=(res&0xFF);
					printf("Choose the value to write..\n");
					printf("---> : ");
					res=-1;
					scanf("%d",&res);
					printf("\n");
					if( (res<0) || (res>255) )
					{
						printf("Wrong parameter of pwm value\n");
						break;					
					}
					value_pwm=(value_pwm<<8);
					value_pwm|=(res&0xFF);
					ioctl(fd, IOCTL_PWM_VALUE, &value_pwm);
				}
				else
					printf("Wrong option\n");
				break;
			case 5:
				value=0;
				printf("Choose the maximum temperature..\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("Value of res:%d\n",res);
				if( (res<0) || (res>255))
				{
					printf("Wrong MAX temp parameter\n");
					break;
				}
				value|=res;
				value=(value<<8);				
				printf("Choose the minimum temperature..\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("Value of res:%d\n",res);
				if( (res<0) || (res>255))
				{
					printf("Wrong MIN temp parameter\n");
					break;
				}
				value|=res;
				printf("----->Value: 0x%04X\n",value);
				ioctl(fd, IOCTL_SET_MAX_MIN_TEMP,&value);
				break;
			case 6:
				printf("Choose the value RED...\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				trgb.red=(res&0x1F);
				printf("Choose the value GREEN...\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				trgb.green=(res&0x1F);
				printf("Choose the value BLUE...\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				trgb.blue=(res&0x1F);
				ioctl(fd,IOCTL_SET_RGB,&trgb);
				break;
			case 7:
				printf("Choose the IRCODE ...\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("\n\n---------->>res: 0x%08X\n\n\n",res);
				ioctl(fd,IOCTL_WAKEUP_IR_CODE,&res);
				break;
			case 8:
				for(t=0;t<num_reg;t++)
				{
					res=valid_reg[t];
					ioctl(fd,IOCTL_READ_BYTE,&res);
					printf("Read register: 0x%02X Value: %d\n",valid_reg[t],(res&0xFF));
				}	
				break;

			default:
				printf("WRONG!!!!\n");
				break;
		}
		res=-1;
	}
	close(fd);
	return 0;
}

