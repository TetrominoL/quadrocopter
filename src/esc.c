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
#define MOTOR_OUT_DDR   DDRD
#define MOTOR_BP_1      4
#define MOTOR_BP_2      5
#define MOTOR_BP_3      6
#define MOTOR_BP_4      7

typedef struct{
    volatile uint8_t speed;
    uint8_t mask;
}motor_t;

volatile uint16_t cnt;
motor_t motors[4];  // 0->Motor0; 1->Motor1 usw.

volatile uint32_t timeout_cnt;

inline uint8_t _percent_to_speedvalue(uint8_t speed){
    if(speed > 100)
        speed = 100;
    return MIN_SPEED + (uint8_t)((MAX_SPEED-MIN_SPEED)/100.0*speed);
}

void esc_init(){
    motors[0].speed = MIN_SPEED;
    motors[1].speed = MIN_SPEED;
    motors[2].speed = MIN_SPEED;
    motors[3].speed = MIN_SPEED;

    motors[0].mask = (1<<MOTOR_BP_1);
    motors[1].mask = (1<<MOTOR_BP_2);
    motors[2].mask = (1<<MOTOR_BP_3);
    motors[3].mask = (1<<MOTOR_BP_4);
    cnt = 0;

    /*TCCR0B  = 0x00; //Timer stopped
    TCCR0A  = 0x82; //non-inverted mode for OCRA; CTC
    TIMSK0  = 0x02; //enable Timer-CompareA-Interrupt
    TCNT0   = 0x00; //start count value
    OCR0A   = 0x04; //bei 4us => 20us
    TCCR0B  = 0x03; //64-Prescaler => 4us*/

    MOTOR_OUT_DDR |= motors[3].mask | motors[2].mask | motors[1].mask | motors[0].mask;   //PD0 = Motor0; PD1 = Motor1 usw..
    MOTOR_OUT_PORT &= ~(motors[3].mask | motors[2].mask | motors[1].mask | motors[0].mask);
}

void esc_calib(){
    esc_set_speed_all_abs(MAX_SPEED);
    timeout_cnt = 50000 * 5; //=> 5sec
    while(timeout_cnt);
    esc_set_speed_abs(0,MIN_SPEED);
    timeout_cnt = 50000 * 2; //=> 2sec
    while(timeout_cnt);
    esc_set_speed_abs(1,MIN_SPEED);
    timeout_cnt = 50000 * 2; //=> 2sec
    while(timeout_cnt);
    esc_set_speed_abs(2,MIN_SPEED);
    timeout_cnt = 50000 * 2; //=> 2sec
    while(timeout_cnt);
    esc_set_speed_abs(3,MIN_SPEED);

    timeout_cnt = 50000 * 1; //=> 1sec
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

#define V 10
void esc_update_motor_out(){
    static uint8_t motor0save=MIN_SPEED;
    static uint8_t motor1save=MIN_SPEED;
    static uint8_t motor2save=MIN_SPEED;
    static uint8_t motor3save=MIN_SPEED;

    int16_t diff;

    if(timeout_cnt > 0)
        timeout_cnt--;
    
    // diff = (int16_t)motor0save - (int16_t)motors[0].speed;
    // if(diff > V || diff < -V){
    //     cli();
    //     while(1);
    // }
    // diff = (int16_t)motor1save - (int16_t)motors[1].speed;
    // if(diff > V || diff < -V){
    //     cli();
    //     while(1);
    // }
    // diff = (int16_t)motor2save - (int16_t)motors[2].speed;
    // if(diff > V || diff < -V){
    //     cli();
    //     while(1);
    // }
    // diff = (int16_t)motor3save - (int16_t)motors[3].speed;
    // if(diff > V || diff < -V){
    //     cli();
    //     while(1);
    // }
    motor0save=motors[0].speed;
    motor1save=motors[1].speed;
    motor2save=motors[2].speed;
    motor3save=motors[3].speed;

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