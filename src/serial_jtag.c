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
#include <termios.h>
#include "serial_jtag.h"


#define BAUDRATE B921600
// CP2102's max speed is B921600
// gives a ~77kHz signal

#define PULSE 0xf0
// 1 1111 0000

int serial_jtag_open(char *device)
{
	int fd;
	fd = open(device, O_RDWR|O_NOCTTY);

	if(fd != -1)
	{
		struct termios settings;

		tcgetattr(fd, &settings);

		cfsetispeed(&settings, BAUDRATE);
		cfsetospeed(&settings, BAUDRATE);

		settings.c_cflag &= ~PARENB; // no parity bit
		settings.c_cflag &= ~CSTOPB; // 1 stop bit
		settings.c_cflag &= ~CSIZE; /* Clears the Mask       */
		settings.c_cflag |=  CS7;   /* Set the data bits = 7 */
		settings.c_cflag &= ~CRTSCTS;
		settings.c_cflag |= CREAD | CLOCAL;
		settings.c_iflag &= ~(IXON | IXOFF | IXANY);
		settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

		tcsetattr(fd, TCSANOW, &settings);
	}

	return fd;
}

void serial_jtag_close(int fd)
{
	close(fd);
}

int set_TDI(int fd, int state)
{
	int flag = TIOCM_RTS;
	return ioctl(fd, state ? TIOCMBIS : TIOCMBIC, &flag);
}

int set_TMS(int fd, int state)
{
	int flag = TIOCM_DTR;
	return ioctl(fd, state ? TIOCMBIS : TIOCMBIC, &flag);
}

int pulse_TCK(int fd)
{
	char write_buffer[] = {PULSE};
	return (int) write(fd, write_buffer, sizeof(write_buffer));
}

int get_TDO(int fd)
{
	int bits;
	ioctl(fd, TIOCMGET, &bits);
	return bits & TIOCM_CTS;
}
