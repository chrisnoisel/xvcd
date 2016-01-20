/*
 * serial_jtag.h
 *
 *  Created on: Jan 17, 2016
 *      Author: chris
 */

#ifndef SERIAL_JTAG_H_
#define SERIAL_JTAG_H_

int serial_jtag_open(char *device);
void serial_jtag_close(int fd);
int set_TDI(int fd, int state);
int set_TMS(int fd, int state);
int set_TMS_TDI(int fd, int state_TMS, int state_TDI);
int get_TDO_set_TMS_TDI(int fd, int state_TMS, int state_TDI);
int pulse_TCK(int fd);
int get_TDO(int fd);

#endif /* SERIAL_JTAG_H_ */
