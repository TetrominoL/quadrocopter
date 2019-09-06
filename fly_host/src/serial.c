#include "serial.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

struct termios tio;
int tty_fd;

char *deviceName = NULL;
speed_t baudrate = B115200;

int setDevice(char *dev){
    deviceName = (char*) realloc(deviceName,strlen(dev));
    if(!deviceName){
        perror(dev);
        return -errno;
    }
    strcpy(deviceName,dev);
    return 0;
}

void setBaud(speed_t baud){
    baudrate = baud;
}

int openPort(void){
    memset(&tio,0,sizeof(tio));
    tio.c_iflag=0;
    tio.c_oflag=0;
    tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
    tio.c_lflag=0;
    tio.c_cc[VMIN]=1;
    tio.c_cc[VTIME]=5;

    tty_fd=open(deviceName, O_RDWR | O_NONBLOCK);
    if(tty_fd == -1){
        perror(deviceName);
        return -errno;
    }

    cfsetospeed(&tio,baudrate);            // 115200 baud
    cfsetispeed(&tio,baudrate);            // 115200 baud
    tcsetattr(tty_fd,TCSANOW,&tio);

    return tty_fd;
}

void closePort(void){
    close(tty_fd);
}