#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "serial.h"
#include <err.h>
#include <errno.h>
#include "uart_protocoll.h"
#include "ringbuf.h"
#include <sys/time.h> 

#define SERIAL_BUF_SIZE   100

str_ringbuf_t serial_buf;
str_ringbuf_t stdin_buf;
int serial_fd;


static struct termios g_old_kbd_mode;

// did somebody press somthing???
static int kbhit(void){
    struct timeval timeout;
    fd_set read_handles;
    int status;

    // check stdin (fd 0) for activity
    FD_ZERO(&read_handles);
    FD_SET(0, &read_handles);
    timeout.tv_sec = timeout.tv_usec = 1;
    status = select(0 + 1, &read_handles, NULL, NULL, &timeout);
    return status;
} 

// put the things as they were befor leave..!!!
static void old_attr(void){
    tcsetattr(0, TCSANOW, &g_old_kbd_mode);
} 

uint8_t check_for_new_data(int fd, str_ringbuf_t *rbuf){
    char buf[30];
    int errVal;

    errVal = read(fd,buf,30);
    if(errVal > 0){
        write_ringbuf(rbuf,buf,errVal);
        return errVal;
    }
    return 0;
}

int main(int argc,char** argv)
{
    int c;
    int errVal;
    char buf[0xff];
    char ch;
    static char init;
    struct termios new_kbd_mode; 

    char *str_serial_interface_device = "/dev/ttyUSB0";

    if (argc > 1){
        str_serial_interface_device = argv[1];
    }

    buf[0xff-1] = '\0';

    init_ringbuf(&serial_buf, SERIAL_BUF_SIZE);
    init_ringbuf(&stdin_buf, SERIAL_BUF_SIZE);

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    // put keyboard (stdin, actually) in raw, unbuffered mode
    tcgetattr(0, &g_old_kbd_mode);
    memcpy(&new_kbd_mode, &g_old_kbd_mode, sizeof(struct termios));
    
    new_kbd_mode.c_lflag &= ~(ICANON | ECHO);
    new_kbd_mode.c_cc[VTIME] = 0;
    new_kbd_mode.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_kbd_mode);
    atexit(old_attr);

    setBaud(B57600);
    printf("set serial device '%s'\n", str_serial_interface_device);
    if(errVal = setDevice(str_serial_interface_device)){
        err(errVal,"Error while set Device!\n");
        exit(EXIT_FAILURE);
    }
    printf("open port\n");
    serial_fd = openPort();
    printf("ready!\n\n");
    printf("w\tincrease speed\n");
    printf("s\tdecrease speed\n");
    printf("space\tstop all\n");
    printf("k\tcalib\n");
    printf("\n");

    while (c!='q'){
        //Check whether new data have to read
        if(check_for_new_data(serial_fd, &serial_buf)  || get_ringbuf_load(&serial_buf)){
            int s = read_ringbuf(&serial_buf, buf, 0xff-1);
            buf[s] = '\0';
            printf("%s",buf);
        }

        if((c = getchar()) != EOF){
            int size = 0;
            switch (c){
                case 'w':
                    printf("->increase speed\n");
                    size = packPacket(buf, 0xff, INCREASE_SPEED, 2);
                break;
                case 's':
                    printf("->decrease speed\n");
                    size = packPacket(buf, 0xff, DECREASE_SPEED, 2);
                break;
                case 'a':
                break;
                case 'd':
                break;
                case 0x20:
                    printf("->STOP!\n");
                    size = packPacket(buf, 0xff, STOP_ALL, 0);
                break;
                case 'c':
                    printf("CALIB\n");
                    size = packPacket(buf, 0xff, CALIB_ESC, 0);
                break;
                case 'i':
                    printf("->increase pitch\n");
                    size = packPacket(buf, 0xff, INCREASE_PITCH, 2);
                break;
                case 'k':
                    printf("->decrease pitch\n");
                    size = packPacket(buf, 0xff, DECREASE_PITCH, 2);
                break;
                case 'j':
                    printf("->increase roll\n");
                    size = packPacket(buf, 0xff, INCREASE_ROLL, 2);
                break;
                case 'l':
                    printf("->decrease roll\n");
                    size = packPacket(buf, 0xff, DECREASE_ROLL, 2);
                break;
                case 'r':
                    printf("->reset angles\n");
                    size = packPacket(buf, 0xff, RESET_ANGLES, 0);
                break;
                case 'y':
                    printf("->increase p\n");
                    size = packPacket(buf, 0xff, INCREASE_P, 0);
                break;
                case 'x':
                    printf("->decrease p\n");
                    size = packPacket(buf, 0xff, DECREASE_P, 0);
                break;
            }
            //printf("SENDE:%d\n", size);
            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 100;
            select(0,NULL, NULL, NULL, &timeout);
            write(serial_fd, buf, size);
        }
        fflush(stdin);
    }

    closePort();
    exit(EXIT_SUCCESS);
}