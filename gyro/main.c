
#define DEBUG

#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "mpu6050.h"
#include "usart.h"
#include "twi.h"

//#define debug

int main(void){
    uint8_t ret;
    mpu_init();
    usart_init();

    uint16_t gyro_x,gyro_y,gyro_z;

    while(1){
        if(mpu_update_all_sensor_data()){
            debug("update sensor data failed!\r\n");
        }

        debug("Gyro:\r\n");
        debug(" X=%d\r\n", mpu_gyro_x );
        debug(" Y=%d\r\n",mpu_gyro_y);
        debug(" Z=%d\r\n",mpu_gyro_z);

        debug("ACC:\r\n");
        debug(" X=%d\r\n", mpu_acc_x );
        debug(" Y=%d\r\n",mpu_acc_y);
        debug(" Z=%d\r\n",mpu_acc_z);

        _delay_ms(100);
    }

    return 0;
}


