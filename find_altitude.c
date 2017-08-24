
/*
This program is part of fly_waypoints copyright Jan Panteltje 2016-always.
it generates 5 Hz GPS output from a specified file with latitude, longitude, speed, altitude and command entries.
this is intended to be written to SDcard one GPS blob per sector.
The card can be read by gpss.asm located here:
 http://panteltje.com/panteltje/quadcopter/index.html
gpss is used to control a Hubsan H501S drone in follow me mode from the remote control,
so it has to be in radio range.

This program is free software
you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation
either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY
without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program
if not, write to the Free Software
Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/


/*
find_altitude.c is a program for the Linux operating system to extract altitude information from a Hubsan H501S quadcopter remote control test point AFC_RX for further processing.
This is example code to show how it can be done.
Use at your own risk.

compile like this;
 gcc -Wall -o find_altitude find_altitude.c

Connect via serial to USB adaptor (ebay) to remote AFC_RX testpoint.

run like this:
./find_altitude /dev/ttyUSB0

altitude=-0.2
altitude=-0.2
altitude=-0.2
altitude=-0.1
altitude=0.0
altitude=-0.1
altitude=0.0
altitude=0.0
...
altitude=0.0

Will report drone altitude as displayed on remote screen,
a checksum check is done.
*/



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <pwd.h>
#include <unistd.h>
//#include <termcap.h>
#include <string.h>
#include <termios.h>
#include <sys/time.h>
#include <time.h>
#include <sys/un.h>
#include <sys/file.h>
#include <pwd.h>
#include <signal.h>
#include <errno.h>
#include "getopt.h"
#include <ctype.h>
#include <unistd.h>       
#include <math.h>
#include <stdint.h>

/* not in string.h ? */
char *strcasestr(const char *haystack, const char *needle);

int verbose;



int set_port_parameters(int fd, int baudrate, char *device)
{
struct termios tty;
speed_t baud;

if(verbose)
	{
	fprintf(stderr, "set_port_parameters(): arg fd=%d baudrate=%d device=%s\n\r", fd, baudrate, device);
	}

/* convert real baudrates to 'B' baudrates */
switch(baudrate)
	{
	case 75:
		baud = B75;
		break;
	case 150:
		baud = B150;
		break;
	case 300:
		baud = B300;
		break;
	case 600:
		baud = B600;
		break;	
	case 1200:
		baud = B1200;
		break;
	case 2400:
		baud = B2400;
		break;
	case 4800:
		baud = B4800;
		break;	
	case 9600:
		baud = B9600;
		break;
	case 19200:
		baud = B19200;
		break;
	case 38400:
		baud = B38400;
		break;
	case 57600:
		baud = B57600;
		break;
	case 115200:
		baud = B115200;
		break;
	default:
		fprintf(stderr, "xgpspc: baudrate=%d unsupported, aborting.\n", baudrate);
		return 0;
		break;
	} /* end switch baudrate */

/* set comport parameters */
tcgetattr(fd, &tty);

tty.c_cflag = 0;
tty.c_cflag |= CLOCAL;			/* ignore modem control lines */
tty.c_cflag |= CREAD;			/* enable receiver */
tty.c_cflag |= CS8;				/* use 8 data bits */
//tty.c_cflag |= CSTOPB;		/* 2 stop bits */
//tty.c_cflag |= PARENB;		/* enable parity */
//tty.c_cflag |= PARODD;		/* use odd parity */
//tty.c_cflag &= ~CRTSCTS;		/* do not use RTS and CTS handshake */

tty.c_iflag = 0;
tty.c_iflag |= IGNBRK;
tty.c_iflag |= IGNPAR;
//tty.c_iflag |= PARMRK
//tty.c_iflag |= INPCK;
//tty.c_iflag |= INLCR;
//tty.c_iflag |= ISTRIP
//tty.c_iflag &= ~(IXON | IXOFF | IXANY);

tty.c_oflag = 0;

tty.c_lflag = 0;
//tty.c_lflag |= ICANON;
//tty.c_cc[VMIN] = 1;
//tty.c_cc[VTIME] = 5;
//tty.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);


cfsetospeed(&tty, (speed_t)baud);
cfsetispeed(&tty, (speed_t)baud);

tcsetattr(fd, TCSANOW, &tty);

return 1;
} /* end function set_port_parameters */




int main(int argc, char **argv)
{
int c, i, j;
FILE *fptr;
int altitude_mode;
char temp[256];
double altitude;
uint8_t checksum;

if(argc < 2)
	{
	fprintf(stderr, "Usage: find_altitude filename");
	
	exit(1);
	}

fptr = fopen(argv[1], "r");
if(! fptr)
	{
	fprintf(stderr, "could not open file or device %s for read, aborting.\n", argv[1]); 

	exit(1);
	}

/* if reading from serial USB device set baudrate */
if(strstr(argv[1] , "/dev/ttyUSB") == argv[1])
	{
	fprintf(stderr, "setting baudrate to 57600\n");
	
	//int set_port_parameters(int fd, int baudrate, char *device)
	if(! set_port_parameters( fileno(fptr), 57600, argv[1]) )
		{
		fprintf(stderr, "could not set baudrate 57600 for device %s, aborting.\n", argv[1]);

		exit(1);
		}
	}
	
/*
FF 0C
16 00 03
08 // field length
20 20 20 5E 30 2E 30 6D
         ^  0  .  0  m      altitude m
20   // checksum
         
looking for FF 0C 16 00 03
*/

checksum = 0;
i = 0;
altitude_mode = 0;
while(1)
	{
	c = fgetc(fptr);

	if(c == EOF)
		{
		break;
		}

	if(altitude_mode < 2)
		{
		checksum = 0;
		}
	else if(altitude_mode < 6)
		{
		checksum ^= c;
		}

	if(altitude_mode == 6)
		{
//fprintf(stderr, "c=%02x\n", c);		

		if(c == checksum)
			{
			altitude = atof(temp);
			fprintf(stdout, "altitude=%.1f\n", altitude);
			}
		else
			{
			fprintf(stderr, "CHECKSUM error\n");
			}

		altitude_mode = 0;
//exit(1);
		}

	if(i)
		{
		if( isdigit(c) || (c == '.') || (c == '-') )
			{
			temp[j] = c;
			j++;
			}
		i--;
		if(i == 0)
			{
			temp[j] = 0;
//			fprintf(stderr, "temp=%s\n", temp);

			altitude_mode = 6;
			}

		continue;
		}		

	if(altitude_mode == 5)
		{
		i = c;
		j = 0;
		continue;
		}

	if(altitude_mode == 4)
		{
		if(c == 0x03) altitude_mode++;
		else altitude_mode = 0;
		}
	if(altitude_mode == 3)
		{
		if(c == 0x00) altitude_mode++;
		else altitude_mode = 0;
		}
	if(altitude_mode == 2)
		{
		if(c == 0x16) altitude_mode++;
		else altitude_mode = 0;
		}
	if(altitude_mode == 1)
		{
		if(c == 0x0c) altitude_mode++;
		else altitude_mode = 0;
		}

	if(c == 0xff)
		{
		altitude_mode++;
		}
	} /* end while */

fclose(fptr);

exit(0);
} /* end function main */

