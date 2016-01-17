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
	struct serial_jtag jtag;
	serial_jtag_open(&jtag, "/dev/ttyUSB0");

	for(;;)
	{
		pulse_TCK(&jtag);
	}

	serial_jtag_close(&jtag);
	return 0;
}
