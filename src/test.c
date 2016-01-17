/*
 * test.c
 *
 *  Created on: Jan 17, 2016
 *      Author: christophe noisel
 */

#include "serial_jtag.h"
#include <stdio.h>

int main(int argc, char **argv)
{
	int jtag;
	jtag = serial_jtag_open("/dev/ttyUSB0");

	for(;;)
	{
		printf("%d\n", get_TDO(jtag));

		set_TDI(jtag, 1);
		set_TMS(jtag, 1);

		pulse_TCK(jtag);

		set_TDI(jtag, 0);
		set_TMS(jtag, 0);
	}

	serial_jtag_close(jtag);
	return 0;
}
