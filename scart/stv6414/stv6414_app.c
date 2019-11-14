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

static void help(void)
{
	printf("\n");
	printf("***************************************\n");
	printf("* You must give 1 or 2 parameters\n");
	printf("* 1 parameter:	1->standby ON, 0-> standby OFF \n");
	printf("* 2 parameters...\n");
	printf("*    1st: the register to write\n");
	printf("*    2nd: the value to write\n");
	printf("***************************************\n");
	printf("\n");
}

int main(int argc, char **argv)
{
	int fd;
	Register_t reg;

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
		printf("argc %d,argv[1] %s, argv[2] %s\n",argc,argv[1], argv[2]);
		reg.offset_addr=strtol(argv[1],NULL,16);
		reg.value=strtol(argv[2],NULL,16);	

		printf("Write register: 0x%02X, value: 0x%02X\n",reg.offset_addr,reg.value);
		ioctl(fd, IOCTL_WRITE_I2C, &reg);

	}
	else
		help();

	close(fd);
	return 0;
}



