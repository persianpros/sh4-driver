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
#include "scart_mini.h"

int main()
{
	int fd;
	int res=-1;
	Scart_Status_st status;
       
	fd=open("/dev/scart_0", O_RDWR|O_NONBLOCK);
       
	while(1)
	{       
		printf("Choose 'ioctl'...\n");
		printf("1 - IOCTL_SET_ASPECT_RATIO\n");
		printf("2 - IOCTL_FORMAT_COLOR\n");
		printf("3 - IOCTL_ENABLE_SCART\n");
		printf("4 - IOCTL_SCART_STATUS\n");
		printf("5 - Quit\n");           
		printf("---> : ");
		scanf("%d",&res);
		printf("\n");
		if((unsigned char)res>=5)
		{
			res=-1;
			printf("Quit from the test!!\n");
			break;
		}
		switch(res)
		{
			case 1:
				printf("Choose the aspect ratio ...\n");
				printf("0 - 4/3\n");
				printf("1 - 16/9\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("\n");
				if(res==0)
				{
					res=AR_4_3;
					ioctl(fd, IOCTL_SET_ASPECT_RATIO, &res);
				}
				else if(res==1)
				{
					res=AR_16_9;
					ioctl(fd, IOCTL_SET_ASPECT_RATIO, &res);
				}
				else
					printf("Wrong aspect ratio!!\n");
				break;
			case 2:
				printf("Choose the color format ...\n");
				printf("0 - RGB\n");
				printf("1 - CVBS\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("\n");
				if(res==0)
				{
					res=RGB;
					ioctl(fd, IOCTL_FORMAT_COLOR, &res);
				}
				else if(res==1)
				{
					res=CVBS;
					ioctl(fd, IOCTL_FORMAT_COLOR, &res);
				}
				else
					printf("Wrong color format!!\n");
				break;
			case 3:
				printf("Choose the scart condition ...\n");
				printf("0 - Disable\n");
				printf("1 - Enable\n");
				printf("---> : ");
				res=-1;
				scanf("%d",&res);
				printf("\n");
				if(res==0)
				{
					res=DIS_SCART;
					ioctl(fd, IOCTL_ENABLE_SCART, &res);
				}
				else if(res==1)
				{
					res=EN_SCART;
					ioctl(fd, IOCTL_ENABLE_SCART, &res);
				}
				else
					printf("Wrong status for scart!!\n");
				break;
			case 4:
				ioctl(fd, IOCTL_SCART_STATUS, &status);
				printf("\n");
				printf("SCART CONDITION ... ");
				if(status.scart_cond==EN_SCART)
					printf("ENABLE\n");
				else if(status.scart_cond==DIS_SCART)
					printf("DISABLE\n");
				else
					printf("Unknown\n");

				printf("SCART ASPECT RATIO ... ");
				if(status.scart_ar==AR_4_3)
					printf("4/3\n");
				else if(status.scart_ar==AR_16_9)
					printf("16/9\n");
				else
					printf("Unknown\n");
	
				printf("SCART COLOR FORMAT ... ");
				if(status.scart_f==RGB)
					printf("RGB\n");
				else if(status.scart_f==CVBS)
					printf("CVBS\n");
				else
					printf("Unknown\n");

				printf("\n");
				break;
			default:
				printf("WRONG!!!!\n");
				break;
			}
		res=-1;
	}
	printf("Close the device 'scart_0'\n");
	close(fd);
	return 0;
}


