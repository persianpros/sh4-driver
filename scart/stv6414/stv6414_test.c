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
#include "stv6414_i2c.h"

int main()
{
	int fd;
	Register_t reg;
	int res=-1;

	reg.value=0;
	reg.offset_addr=0;
	
	fd=open("/dev/stv6414_0", O_RDWR|O_NONBLOCK);
	
	while(1)
	{	
		printf("Choose 'ioctl'...\n");
		printf("1 - IOCTL_READ_I2C\n");
		printf("2 - IOCTL_WRITE_I2C\n");
		printf("3 - IOCTL_READ_LAST_WRITE_VALUE_OF_REG\n");
		printf("4 - IOCTL_SET_169_43\n");
		printf("5 - IOCTL_STANDBY_ON_OFF\n");
		printf("6 - Quit\n");		
		printf("---> : ");
		scanf("%d",&res);
		printf("\n");
		if((unsigned char)res==6)
		{
			res=-1;
			return;
		}
		switch(res)
		{
			case 1:
				printf("Choose the register to read..\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("\n");
				reg.offset_addr=res;
				reg.value=0xFF;
				ioctl(fd, IOCTL_READ_I2C, &reg);
				printf("\n");
				printf("Value of register 0x%02X: 0x%02X\n",reg.offset_addr,reg.value);
				printf("\n");
				break;
			case 2:
				printf("Choose the register to write..\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("\n");
				reg.offset_addr=res;
				printf("Choose the value to write..\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("\n");
				reg.value=res;
				ioctl(fd, IOCTL_WRITE_I2C, &reg);
				break;
			case 3:
				printf("Choose the register to read..\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("\n");
				reg.offset_addr=res;
				reg.value=0xFF;
				ioctl(fd, IOCTL_READ_LAST_WRITE_VALUE_OF_REG, &reg);
				printf("\n");
				printf("Value of register 0x%02X: 0x%02X\n",reg.offset_addr,reg.value);
				printf("\n");
				break;
			case 4:
				printf("Choose the format...\n");
				printf("1 - 16 / 9\n");
				printf("2 - 4 / 3\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("\n");
				if(res==1)
				{
					res=TV_16_9;
					ioctl(fd, IOCTL_SET_169_43, &res);
				}
				else if(res==2)
				{
					res=TV_4_3;
					ioctl(fd, IOCTL_SET_169_43, &res);
				}
				else
					printf("WRONG\n");
				break;
			case 5:
				printf("Choose standby mode...\n");
				printf("1 - ON (enter in standby)\n");
				printf("2 - OFF (exit to standby)\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("\n");
				if(res==1)
				{
					res=STANDBY_ON;
					ioctl(fd, IOCTL_STANDBY_ON_OFF, &res);
				}
				else if(res==2)
				{
					res=STANDBY_OFF;
					ioctl(fd, IOCTL_STANDBY_ON_OFF, &res);
				}
				else
					printf("WRONG\n");
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



