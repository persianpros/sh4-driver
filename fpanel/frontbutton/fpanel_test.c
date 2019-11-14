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
#include "fpanel.h"

int main()
{
	int fd;
	int res=-1;

	fd=open("/dev/fpanel_0", O_RDWR|O_NONBLOCK);
       
	while(1)
	{       
		printf("Choose 'ioctl'...\n");
		printf("1 - IOCTL_READ_BUTTON_STATUS\n");
		printf("2 - Quit\n");           
		printf("---> : ");
		scanf("%d",&res);
		printf("\n");
		if((unsigned char)res>=2)
		{
			res=-1;
			printf("Quit from the test!!\n");
			break;
		}
		switch(res)
		{
			case 1:
				ioctl(fd, IOCTL_READ_BUTTON_STATUS, &res);
				printf("\n======> BUTTON STATUS: ");
				if(res==BUTTON_PRESSED)
					printf("BUTTON_PRESSED\n\n");
				else if(res==BUTTON_RELEASED)
					printf("BUTTON_RELEASED\n\n");
				else
					printf("UNKNOWN\n\n");
				res=1;
				break;
			case 2:
				res=2;
				goto stop;
			default:
				printf("WRONG!!!!\n");
				break;
			}
		res=-1;
	}
stop:
	printf("Close the device 'fpanel_0'\n");
	close(fd);
	return 0;
}


