#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>

int setDevice(char *devicename);
void setBaud(speed_t baud);
int openPort(void);
void closePort(void);

//int send(char *buf, uint8_t size);

#endif //SERIAL_H