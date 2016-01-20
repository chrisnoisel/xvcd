/*
 * test.c
 *
 *  Created on: Jan 17, 2016
 *      Author: christophe noisel
 */

#include "serial_jtag.h"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
	int jtag;
	jtag = serial_jtag_open("/dev/ttyS0");

	int count = 0;

	for(;;)
	{
		usleep(800);
//		get_TDO(jtag);
//		set_TDI(jtag, count&1);
//		set_TMS(jtag, (count+1)%2);
		//set_TMS_TDI(jtag, count%2, (count+1)%2);

		get_TDO_set_TMS_TDI(jtag, count%2, (count+1)%2);

		pulse_TCK(jtag);

		count++;
	}

	printf("1");

	serial_jtag_close(jtag);


	printf("2");

	return 0;
}
