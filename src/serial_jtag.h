/*
 * serial_jtag.h
 *
 *  Created on: Jan 17, 2016
 *      Author: chris
 */

#include <termios.h>

#ifndef SERIAL_JTAG_H_
#define SERIAL_JTAG_H_

struct serial_jtag
{
	int fd;
	struct termios settings;
};

int serial_jtag_open(struct serial_jtag *dev, char *device);
void serial_jtag_close(struct serial_jtag *dev);
int set_TDI(struct serial_jtag *dev, int state);
int set_TMS(struct serial_jtag *dev, int state);
int pulse_TCK(struct serial_jtag *dev);
int get_TDO(struct serial_jtag *dev);

#endif /* SERIAL_JTAG_H_ */
