#ifndef UART_PROTOCOLL_H
#define UART_PROTOCOLL_H

#include <stdint.h>

#define START_BYTE  0x55
#define MAX_LENGTH  0xff-3

#define E_NOSTARTBYTE   0x01
#define E_WRONGLENGTH   0x02
#define E_WRONGCRC      0x03
#define E_UNKNOWNCMD    0x04

/*
Protokoll:
0x55 | CMD | LENGTH (DATA ink. CRC) | DATA0 | DATA1| ... | DATAN | CRC ()
*/

#pragma pack(push,1)
typedef struct{
    uint8_t start_byte;
    uint8_t cmd;
    uint8_t len;
    uint8_t value;
} str_increase_speed_t, str_decrease_speed_t, str_set_seed_t, str_decrease_roll_t, str_decrease_pitch_t, str_increase_roll_t, str_increase_pitch_t;

typedef struct{
    uint8_t start_byte;
    uint8_t cmd;
    uint8_t len;
} str_get_gyro_t, str_get_acc_t, str_get_motorspeed_t;

typedef struct{
    uint8_t start_byte;
    uint8_t cmd;
    uint8_t len;
} str_begin_t;

typedef struct{
    uint8_t start_byte;
    uint8_t cmd;
    uint8_t len;
    uint8_t response;
} str_ping_t;

#pragma pack(pop)

#define INCREASE_SPEED      0x01
#define DECREASE_SPEED      0x02
#define SET_SPEED           0x03
#define GET_GYRO            0x04
#define GET_ACC             0x05
#define GET_MOTORSPEED      0x06
#define ENABLE_DEBUG        0x07
#define GET_MODE            0x08
#define BEGIN               0x09
#define PING                0x0A
#define SET_MODE            0x0B
#define STOP_ALL            0x0C
#define CALIB_ESC           0x0D
#define INCREASE_ROLL       0x0E
#define DECREASE_ROLL       0x0F
#define INCREASE_PITCH      0x10
#define DECREASE_PITCH      0x11
#define RESET_ANGLES        0x12
#define INCREASE_P          0x13
#define DECREASE_P          0x14
#define SET_KP              0x15
#define SET_KI              0x16
#define SET_KD              0x17

uint8_t calcCrc(char *buf, uint8_t size){
    uint8_t crc = START_BYTE;

    while(size > 0){
        crc -= *buf;
        buf++;
        size--;
    }

    return crc;
}

uint8_t packPacket(uint8_t *buf, uint8_t max_size, uint8_t cmd, uint8_t *value){
    uint8_t size;
    buf[0] = START_BYTE;
    buf[1] = cmd;
    uint8_t i;

    switch (cmd){
        case INCREASE_SPEED:
        case DECREASE_SPEED:
        case INCREASE_PITCH:
        case DECREASE_PITCH:
        case INCREASE_ROLL:
        case DECREASE_ROLL:
        case INCREASE_P:
        case DECREASE_P:
            size = 1+4;
        break;
        case STOP_ALL:
        case RESET_ANGLES:
        case CALIB_ESC:
            size = 0+4;
        break;
        case SET_KP:
        case SET_KI:
        case SET_KD:
            size = 4+4;
        break;
    }
    if(size > max_size)
        return 0;
    
    buf[2] = size -3;

    for(i=3; i< size-1; i++){
        buf[i] = value[i-3];
        printf("B: %d\r\n",buf[i]);
    }

    buf[size -1] = calcCrc(buf, size);

    printf("PKG: ");
    for(i=0; i< size; i++){
        printf("%d ",buf[i]);
    }
    printf("\r\n");
    return size;
}

uint8_t checkPacket(uint8_t *buf){
    uint8_t size = 3+buf[2];

    if(buf[0] != START_BYTE)
        return E_NOSTARTBYTE;
    
    if(buf[2] > MAX_LENGTH)
        return E_WRONGLENGTH;

    if (calcCrc(buf, size) != buf[size-1])
        return E_WRONGCRC;
    
    switch (buf[1]){
        case INCREASE_SPEED:
        case DECREASE_SPEED:
        case SET_SPEED:     
        case GET_GYRO:    
        case GET_ACC:     
        case GET_MOTORSPEED:
        case ENABLE_DEBUG:
        case GET_MODE:
        case BEGIN:
        case PING:
        case SET_MODE:    
        case STOP_ALL:
        case INCREASE_PITCH:
        case DECREASE_PITCH:
        case INCREASE_ROLL:
        case DECREASE_ROLL:
        case RESET_ANGLES:
        case INCREASE_P:
        case DECREASE_P:
        case SET_KP:
        case SET_KI:
        case SET_KD:
            
        break;
        default:
            return E_UNKNOWNCMD;
    }
}

#endif //UART_PROTOCOLL_H