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


#define BAUDRATE B1152000

#define PULSE 0xfc

int serial_jtag_open(char *device)
{
	int fd;
	fd = open(device, O_WRONLY|O_NOCTTY|O_NONBLOCK);

	if(fd != -1)
	{
		struct termios settings;

		if (tcgetattr(fd, &settings) == -1) {
			perror("serial_jtag_open(): tcgetattr()");
			return 0;
		}

		cfsetispeed(&settings, BAUDRATE);
		cfsetospeed(&settings, BAUDRATE);

		//http://man7.org/linux/man-pages/man3/termios.3.html
		cfmakeraw(&settings);
		//settings.c_cflag &= ~PARENB; // no parity bit

		settings.c_cflag &= ~CSTOPB; // 1 stop bit
		settings.c_cflag &= ~CSIZE; // Clears the Mask
		settings.c_cflag |=  CS5;   // Set the data bits
		settings.c_cflag &= ~CRTSCTS;
		//settings.c_cflag |= CRTSCTS;
		settings.c_cflag |= CREAD | CLOCAL;
		//settings.c_iflag &= ~(IXON | IXOFF | IXANY);
		//settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		//http://pubs.opengroup.org/onlinepubs/7908799/xsh/termios.h.html

		settings.c_cc[VMIN] = 0;
		settings.c_cc[VTIME] = 0;

		if (tcflush(fd, TCIOFLUSH) == -1) {
			perror("serial_jtag_open(): tcflush()");
			return 0;
		}
		if (tcsetattr(fd, TCSANOW, &settings) == -1) {
			perror("serial_jtag_open(): tcsetattr()");
			return 0;
		}
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
	int out;
	out = ioctl(fd, state ? TIOCMBIS : TIOCMBIC, &flag);
	if (out == -1) {
		perror("set_TDI(): ioctl()");
	}
	return out;
}

int set_TMS(int fd, int state)
{
	int flag = TIOCM_DTR;
	int out;
	out = ioctl(fd, state ? TIOCMBIS : TIOCMBIC, &flag);
	if (out == -1) {
		perror("set_TMS(): ioctl()");
	}
	return out;
}

int set_TMS_TDI(int fd, int state_TMS, int state_TDI)
{
	int flag = 0;
	flag |= state_TDI ? TIOCM_RTS : 0;
	flag |= state_TMS ? TIOCM_DTR : 0;

	int out = ioctl(fd, TIOCMSET, &flag);
	if (out == -1) {
		perror("set_TMS_TDI(): ioctl()");
	}
	return out;
}

int get_TDO_set_TMS_TDI(int fd, int state_TMS, int state_TDI)
{

	int bits;
	if(ioctl(fd, TIOCMGET, &bits) == -1)
		perror("get_TDO_set_TMS_TDI(): ioctl() with TIOCMGET");
	int CTS = 0 != (bits & TIOCM_CTS);
	/*
	int flag = 0;
	flag |= state_TDI ? TIOCM_RTS : 0;
	flag |= state_TMS ? TIOCM_DTR : 0;
	if(flag != (bits & (TIOCM_RTS | TIOCM_DTR)))
		ioctl(fd, TIOCMSET, &flag);
	*/
	if(state_TMS != (0 != (bits & TIOCM_DTR))) {
		int flag = TIOCM_DTR;
		if (ioctl(fd, state_TMS ? TIOCMBIS : TIOCMBIC, &flag)) {
			perror("get_TDO_set_TMS_TDI(): ioctl() with TIOCM_DTR");
		}
	}

	if(state_TMS != (0 != (bits & TIOCM_DTR))) {
		int flag = TIOCM_RTS;
		if (ioctl(fd, state_TMS ? TIOCMBIS : TIOCMBIC, &flag)) {
			perror("get_TDO_set_TMS_TDI(): ioctl() with TIOCM_RTS");
		}
	}

	return CTS;
}

int pulse_TCK(int fd)
{
	char write_buffer[] = {PULSE};
	return (int) write(fd, write_buffer, sizeof(write_buffer));
}

int get_TDO(int fd)
{
	int bits;
	if(ioctl(fd, TIOCMGET, &bits) == -1)
		perror("get_TDO(): ioctl() with TIOCMGET");
	return bits & TIOCM_CTS;
}
