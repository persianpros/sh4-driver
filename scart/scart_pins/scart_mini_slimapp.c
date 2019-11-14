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
#include <stdlib.h>
#include <sys/ioctl.h>

#include "scart_mini.h"

int main(int argc, char **argv)
{
	int fd;

	fd=open("/dev/scart_0", O_RDWR|O_NONBLOCK);
	if(fd<0)
	{
		printf("Error when open the device 'scart_0'\n");
		return 1;	
	}	

	if(argc==2)
	{	
		int enable=strtol(argv[1],NULL,16);
		printf("Enable: %d\n", enable);
		if(enable==1)
		{
			printf("Enable output scart\n");
			ioctl(fd,IOCTL_ENABLE_SCART , &enable);
		}
		else if(enable==0)
		{	
			printf("Disable output scart\n");
			ioctl(fd,IOCTL_ENABLE_SCART , &enable);
		}
	}
	else
	{
		printf("Wrong parameter, only 1 parameter -> 0 disable scart , 1 enable scart\n");	
	}

	close(fd);
	return 0;
}



