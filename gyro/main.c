#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "mpu6050.h"
#include "usart.h"
#include "twi.h"
#include "angle_calc.h"
#include <util/atomic.h>

//#define debug
volatile uint16_t glob_wait=0;

int main(void){
    uint8_t ret;
    _delay_ms(100);
    usart_init();
    _delay_ms(100);
    mpu_init();
    _delay_ms(100);
    uint16_t tmp_glob_wait;

    TCCR0B  = 0x00; //Timer stopped
    TCCR0A  = 0x82; //non-inverted mode for OCRA; CTC
    TIMSK0  = 0x02; //enable Timer-CompareA-Interrupt
    TCNT0   = 0x00; //start count value
    OCR0A   = 0x04; //bei 4us => 20us
    TCCR0B  = 0x03; //64-Prescaler => 4us

    angle_init();

    uint16_t gyro_x,gyro_y,gyro_z;
    //debug("ACC_X,ACC_y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z\r\n");
    while(1){
        if(mpu_update_all_sensor_data()){
            debug("update sensor data failed!\r\n");
        }
        angle_update(mpu_gyro_x,mpu_gyro_y,mpu_gyro_z,mpu_acc_x,mpu_acc_y,mpu_acc_z);

        ATOMIC_BLOCK(ATOMIC_FORCEON){
            tmp_glob_wait = glob_wait;
        }

        if(tmp_glob_wait >= 1000){
            /*debug("Gyro:\r\n");
            debug(" X=%d\r\n", mpu_gyro_x);
            debug(" Y=%d\r\n", mpu_gyro_y);
            debug(" Z=%d\r\n", mpu_gyro_z);
*/
            //debug("ACC:\r\n");
            
            
            
            
            /*debug("Angle:\r\n");*/
            //debug(" X=%f\r\n", _angle_x);
            /*debug(" Y=%f\r\n", _angle_y);
            debug(" Z=%f\r\n", _angle_z);*/
            
            //debug("%f %f %f\r\n", (float)(((int16_t)mpu_acc_x)/16384.0),(float)(((int16_t)mpu_acc_y)/16384.0),(float)(((int16_t)mpu_acc_z)/16384.0));
            float acc_x, acc_y, acc_z;
            acc_x = ((int16_t)mpu_acc_x)/16384.0;
            acc_y = ((int16_t)mpu_acc_y)/16384.0;
            acc_z = ((int16_t)mpu_acc_z)/16384.0;


            //debug(" %f\r\n",atan(acc_y/sqrt(acc_x*acc_x+acc_z*acc_z))*(180/3.14159265));

            ATOMIC_BLOCK(ATOMIC_FORCEON){
                glob_wait = 0;
            }
        }
        debug("%d,%d,%d,%d,%d,%d\r\n", mpu_acc_x_raw, mpu_acc_y_raw, mpu_acc_z_raw, mpu_gyro_x_raw, mpu_gyro_y_raw, mpu_gyro_z_raw);
        debug("%d,%d,%d,%d,%d,%d\r\n", mpu_acc_x, mpu_acc_y, mpu_acc_z, mpu_gyro_x, mpu_gyro_y, mpu_gyro_z);
        debug("%f, %f, %f\r\n",_angle_x,_angle_y,_angle_z);
        debug("\r\n");
        _delay_ms(500);
    }

    return 0;
}

ISR (TIMER0_COMPA_vect){	
    static uint16_t cnt=0;
    if(++cnt >= 50){
        glob_wait++;
        cnt=0;
    }
    
}
