/*
 * serial_jtag.c
 *
 *  Created on: Jan 17, 2016
 *      Author: christophe noisel
 */

#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include "serial_jtag.h"


#define BAUDRATE B921600
// CP2102's max speed is B921600
// gives ~77kHz signal

#define PULSE 0xf0
// 1 1111 0000

int serial_jtag_open(struct serial_jtag *dev, char *device)
{
	dev->fd = open(device, O_RDWR|O_NOCTTY);

	if(dev->fd != -1)
	{
		struct termios settings;

		tcgetattr(dev->fd, &settings);

		cfsetispeed(&settings, BAUDRATE);
		cfsetospeed(&settings, BAUDRATE);

		settings.c_cflag &= ~PARENB; // no parity bit
		settings.c_cflag &= ~CSTOPB; // 1 stop bit
		settings.c_cflag &= ~CSIZE; /* Clears the Mask       */
		settings.c_cflag |=  CS7;   /* Set the data bits = 8 */
		settings.c_cflag &= ~CRTSCTS;
		settings.c_cflag |= CREAD | CLOCAL;
		settings.c_iflag &= ~(IXON | IXOFF | IXANY);
		settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

		tcsetattr(dev->fd, TCSANOW, &settings);
	}

	return dev->fd;
}

void serial_jtag_close(struct serial_jtag *dev)
{
	close(dev->fd);
}

int set_TDI(struct serial_jtag *dev, int state)
{
	int flag = TIOCM_RTS;
	return ioctl(dev->fd, state ? TIOCMBIS : TIOCMBIC, &flag);
}

int set_TMS(struct serial_jtag *dev, int state)
{
	int flag = TIOCM_DTR;
	return ioctl(dev->fd, state ? TIOCMBIS : TIOCMBIC, &flag);
}

int pulse_TCK(struct serial_jtag *dev)
{
	char write_buffer[] = {PULSE};
	return (int) write(dev->fd, write_buffer, sizeof(write_buffer));
}

int get_TDO(struct serial_jtag *dev)
{
	int bits;
	ioctl(dev->fd, TIOCMGET, &bits);
	return bits & TIOCM_CTS;
}
