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
#include <linux/string.h>
#include <linux/kernel.h> 
#include "time_funcs.h"

static const int local_ytab[2][12]= {
			{ 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
			{ 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
        				};

/****************************/
/* Convert 'time_t' in 'tm' */
/****************************/
void local_time_t_to_tm(register const local_time_t *timer, struct local_tm * res)
{
	static struct local_tm br_time;
	register struct local_tm *timep = &br_time;
	local_time_t time = *timer;
	register unsigned long dayclock, dayno;
	int year = EPOCH_YR;
 
	dayclock = (unsigned long)time % SECS_DAY;
	dayno = (unsigned long)time / SECS_DAY;
 
	timep->tm_sec = dayclock % 60;
	timep->tm_min = (dayclock % 3600) / 60;
	timep->tm_hour = dayclock / 3600;
	timep->tm_wday = (dayno + 4) % 7;       /* day 0 was a thursday */
	while (dayno >= YEARSIZE(year))
	{
		dayno -= YEARSIZE(year);
		year++;
	}
	timep->tm_year = year - YEAR0;
	timep->tm_yday = dayno;
	timep->tm_mon = 0;
	while (dayno >= local_ytab[LEAPYEAR(year)][timep->tm_mon])
	{
		dayno -= local_ytab[LEAPYEAR(year)][timep->tm_mon];
		timep->tm_mon++;
	}
	timep->tm_mday = dayno + 1;
	timep->tm_isdst = 0;
	memcpy(res,timep,sizeof(struct local_tm));
}

/******************************/
/* Convert 'tm' in mjd format */
/******************************/
int local_tm_to_mjd(struct local_tm * t, int * mjd)
{
	// conversion valid for 
	// - all dates with modified Julian Day >= 0     i.e. after  1 Jan 1970 0h
	// - all dates with modified Julian Day <  49710 i.e. before 7 Fev 2106
	// as 49710 days of 86400 s are 2^32 seconds
	// for convenience cut anything posterior to year 2105
	// first check limits

	int year=t->tm_year+1900; 
	int month=t->tm_mon+1;  
	int day=t->tm_mday; 

	*mjd=0;  

	if(year < 1970)    return (-1);
	if(year > 2105)    return (-1);
	if(month > 12)    return (-1);
	if(day > 31)      return (-1); 

	if(month==4 || month==6 || month==9 || month==11) 
	{
		if(day > 30) return (-1);
	}
	else if(month==2)
	{
		// Check february days for leap years
		if( year%100 == 0)
		{
			if( year%400 == 0) 
			{
				if(day > 29) return (-1);
			}
			else 
			{
				if(day > 28) return (-1);
			}
		}
		else if( year%4 == 0) 
		{
			if(day > 29) return (-1);
		}
		else if(day > 28) return (-1);
	}
  
	// compute modified Julian day number since 1rst Jan 1970 0h00

	*mjd = (1461*(year+4800-(14-month)/12))/4 +
           +(367*(month+12*((14-month)/12)-2))/12
           -(3*((year+4900-(14-month)/12)/100))/4 +day -32075 -2440588;

	*mjd=(*mjd)+40587;
	return 0;           
}

/**************************/
/* Convert decimal in BCD */
/**************************/
unsigned char convert_decimal_to_bcd(unsigned char ora)
{
	unsigned char dec=0, value=0, tmp=ora;
	while((tmp-10)>=0)
	{
		tmp=(tmp-10);
		dec++;
	}
	
	value=dec;
	value=(value<<4);
	value|=tmp;
	return value;
}

