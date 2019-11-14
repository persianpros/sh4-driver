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

#include "stv6414_i2c.h"

int main(int argc, char **argv)
{
	int fd;
	Register_t reg;
	int tmp=0;

	reg.value=0;
	reg.offset_addr=0;

	fd=open("/dev/stv6414_0", O_RDWR|O_NONBLOCK);
	
	if(argc==2)
	{	
		int standby=strtol(argv[1],NULL,16);
		printf("Standby: %d\n", standby);
		if(standby==1)
			ioctl(fd, IOCTL_STANDBY_ON_OFF, &standby);
		else if(standby==0)	
			ioctl(fd, IOCTL_STANDBY_ON_OFF, &standby);
	}
	else if(argc==3)
	{
		if(strcmp(argv[1],"TV_OUT")==0)
		{
			if(strcmp(argv[2],"0")==0)//disable tv_out
			{
				tmp=1;				
				ioctl(fd, IOCTL_DISABLE_TV_OUTPUT, &tmp);
			}
			else if(strcmp(argv[2],"1")==0)//enable tv_out
			{
				tmp=0;				
				ioctl(fd, IOCTL_DISABLE_TV_OUTPUT, &tmp);
			}
		}

	}

	close(fd);
	return 0;
}



