/* Author: Simon Lindhorst
* Date: 04.04.2019
*/

#include "esc.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define WAIT_TO(t)  while(0){ \
                    timeout_cnt=t;\
                    while(timeout_cnt);\
                    }

//Max Count Value for ESC-Timing
#define MAX_CNT 1000    // =>20us*1000 = 20ms
#define MIN_SPEED 50    // =>20us*50 = 1ms
#define MAX_SPEED 100   // =>50us*100 = 2ms

#define MOTOR_OUT_PORT  PORTD

typedef struct{
    volatile uint8_t speed;
    uint8_t mask;
}motor_t;

volatile uint16_t cnt;
motor_t motors[4];  // 0->Motor0; 1->Motor1 usw.

volatile uint32_t timeout_cnt;

inline uint8_t _percent_to_speedvalue(uint8_t speed){
    return MIN_SPEED + (uint8_t)((MAX_SPEED-MIN_SPEED)/100.0*speed);
}

void esc_init(){
    motors[0].speed = MIN_SPEED;
    motors[1].speed = MIN_SPEED;
    motors[2].speed = MIN_SPEED;
    motors[3].speed = MIN_SPEED;

    motors[0].mask = (1<<0);
    motors[1].mask = (1<<1);
    motors[2].mask = (1<<2);
    motors[3].mask = (1<<3);
    cnt = 0;

    TCCR0B  = 0x00; //Timer stopped
    TCCR0A  = 0x82; //non-inverted mode for OCRA; CTC
    TIMSK0  = 0x02; //enable Timer-CompareA-Interrupt
    TCNT0   = 0x00; //start count value
    OCR0A   = 0x04; //bei 4us => 20us
    TCCR0B  = 0x03; //64-Prescaler => 4us

    DDRD |= 0x0f;   //PD0 = Motor0; PD1 = Motor1 usw..
    PORTD &= 0xf0;
}

void esc_calib(){
    esc_set_speed_all_abs(MAX_SPEED);
    timeout_cnt = 75000; //=> 1,5sec
    while(timeout_cnt);
    esc_set_speed_all_abs(MIN_SPEED);
    timeout_cnt = 75000; //=> 1,5sec
    while(timeout_cnt);
}

void esc_set_speed_all(uint8_t speed){
    speed = _percent_to_speedvalue(speed);
    motors[0].speed = speed;
    motors[1].speed = speed;
    motors[2].speed = speed;
    motors[3].speed = speed;
}

void esc_set_speed_all_abs(uint8_t speed){
    motors[0].speed = speed;
    motors[1].speed = speed;
    motors[2].speed = speed;
    motors[3].speed = speed;
}

void esc_set_speed(uint8_t motorNo, uint8_t speed){
    speed = _percent_to_speedvalue(speed);
    motors[motorNo].speed = speed;
}

void esc_set_speed_abs(uint8_t motorNo, uint8_t speed){
    motors[motorNo].speed = speed;
}

uint8_t esc_get_speed(uint8_t motorNo){
    return motors[motorNo].speed;
}

ISR (TIMER0_COMPA_vect)
{	
    if(timeout_cnt > 0)
        timeout_cnt--;

	if(++cnt >= MAX_CNT){
        cnt = 0;
        MOTOR_OUT_PORT |= motors[0].mask;
        MOTOR_OUT_PORT |= motors[1].mask;
        MOTOR_OUT_PORT |= motors[2].mask;
        MOTOR_OUT_PORT |= motors[3].mask;
    }
    
    if(cnt >= motors[0].speed){
        MOTOR_OUT_PORT &= ~motors[0].mask;
    }
    if(cnt >= motors[1].speed)
        MOTOR_OUT_PORT &= ~motors[1].mask;
    if(cnt >= motors[2].speed)
        MOTOR_OUT_PORT &= ~motors[2].mask;
    if(cnt >= motors[3].speed)
        MOTOR_OUT_PORT &= ~motors[3].mask;
    
}