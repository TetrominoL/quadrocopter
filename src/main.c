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
#include "process.h"
#include "timer.h"
#include "esc.h"
#include "uart_protocoll.h"
#include "pid.h"

//ca. 50days, so it will be enough
volatile uint32_t timestamp_ms = 0;

#define MAX_MOTOR_SPEED     100
#define MIN_MOTOR_SPEED     1
#define MAX_ANGLE           2.0

#define M1_KI       0
#define M1_KP       1.3
#define M1_KD       0

#define M3_KI       0
#define M3_KP       1.3
#define M3_KD       0

int16_t update_motor_speed = 1;
int16_t motor_0_speed;
int16_t motor_2_speed;
int16_t motor_3_speed;
int16_t motor_1_speed;

double angle_x_target = 0;
double angle_y_target = 0;

// Structure to strore PID data and pointer to PID structure
pid_t pid_m0;
pid_t pid_m1;
pid_t pid_m2;
pid_t pid_m3;

void wait_ms(uint16_t time){
    uint32_t start_ms = timestamp_ms;
    while(timestamp_ms - start_ms < time);
}

void async_timerstuff(){
    static uint8_t ms_cnt = 0;

    //every 1ms
    if(++ms_cnt >= 50){
        ms_cnt=0;
        timestamp_ms++;
        process_update_counter_all();
    }
    esc_update_motor_out();
}

uint8_t _p_regular = 1.5;
#define P_REGULAR   _p_regular

uint8_t flight_control(process_hdl_t *p){
    
    motor_0_speed = update_motor_speed - (int) (P_REGULAR*(_angle_x - angle_x_target)+0.5);
    motor_2_speed = update_motor_speed + (int) (P_REGULAR*(_angle_x - angle_x_target)+0.5);

    motor_1_speed = update_motor_speed - (int) (P_REGULAR*(_angle_y - angle_y_target)+0.5);
    motor_3_speed = update_motor_speed + P_REGULAR*(_angle_y - angle_y_target)+0.5;

    motor_1_speed = pid_run(&pid_m1, angle_y_target, -_angle_y, update_motor_speed);
    motor_3_speed = pid_run(&pid_m3, angle_y_target, _angle_y, update_motor_speed);

    if(motor_0_speed <= 0)
        motor_0_speed = 1;
    if(motor_1_speed <= 0)
        motor_1_speed = 1;
    if(motor_2_speed <= 0)
        motor_2_speed = 1;
    if(motor_3_speed <= 0)
        motor_3_speed = 1;

    //esc_set_speed(0,motor_0_speed);
    esc_set_speed(1,motor_1_speed);
    //esc_set_speed(2,motor_2_speed);
    esc_set_speed(3,motor_3_speed);

    
    return 0;
}

uint8_t angle_calc_callback(process_hdl_t *p){
    angle_update(mpu_gyro_x,mpu_gyro_y,mpu_gyro_z,mpu_acc_x,mpu_acc_y,mpu_acc_z, timestamp_ms);
    return 0;
}

uint8_t print_angle(process_hdl_t *p){
    debug("X-TARGET: %f\r\n", angle_x_target);
    debug("Y-TARGET: %f\r\n", angle_y_target);
    debug("Y-ANGLE: %f\r\n", _angle_y);
    debug("X-ANGLE: %f\r\n", _angle_x);

    debug("M0: %d\r\n",motor_0_speed);
    debug("M1: %d\r\n",motor_1_speed);
    debug("M2: %d\r\n",motor_2_speed);
    debug("M3: %d\r\n",motor_3_speed);

    debug("KP: %f\r\n", pid_m0.kp);
    debug("KD: %f\r\n", pid_m0.kd);
    debug("KI: %f\r\n", pid_m0.ki);
    return 0;
}

uint32_t debug_time_unbusy = 0;
uint8_t print_debug_time(process_hdl_t *p){
    //debug("TIMING: %lu vs %lu\r\n",debug_time_unbusy,timestamp_ms);
    return 0;
}
#define UART_RX_BUF_SIZE    0xff
uint8_t process_rx_uart(process_hdl_t *p){
    static char rxbuf[UART_RX_BUF_SIZE];
    static uint8_t idx = 0;

    uint8_t rec_complete = 0;
    uint8_t data_available = usart_get_rxdata_load();
    
    if (data_available == 0)
        return 0;

    if (idx == 0){  //No data in local buffer, so new packet musst receive
        *rxbuf = 0x00;
        while(usart_get_rxdata(rxbuf, 1) && *rxbuf != START_BYTE){
            
        }
        if (*rxbuf == START_BYTE){
            idx++;
        }
    }
    
    if (idx > 0){
        if (idx == 1){
            if (usart_get_rxdata(rxbuf+idx,1))
                idx++;
        }
        if (idx == 2){
            if (usart_get_rxdata(rxbuf+idx,1))
                idx++;
        }
        if (idx > 2){
            data_available = usart_get_rxdata_load();
            if (data_available+idx-3 >= rxbuf[2]){
                usart_get_rxdata(rxbuf+idx, rxbuf[2]-(idx-3));
                if (checkPacket(rxbuf)){
                    rec_complete = 1;
                }
                    
                idx = 0;
            }else{
                usart_get_rxdata(rxbuf+idx, data_available);
            }
        }
    }
    if (rec_complete){
        switch (rxbuf[1]){
            case STOP_ALL:
                cli();
                esc_set_speed_all(0);
                while(1);
            break;
            case INCREASE_ROLL:{
                str_increase_roll_t *p = (str_increase_roll_t*) rxbuf;
                double v = p->value * 0.1;
                if (angle_x_target < MAX_ANGLE-v){
                    angle_x_target += v;
                }else{
                    angle_x_target = MAX_ANGLE;
                }
            }break;
            case DECREASE_ROLL:{
                str_decrease_roll_t *p = (str_decrease_roll_t*) rxbuf;
                double v = p->value * 0.1;
                if (angle_x_target > (-MAX_ANGLE)+v){
                    angle_x_target -= v;
                }else{
                    angle_x_target = -MAX_ANGLE;
                }
            }break;
            case INCREASE_PITCH:{
                str_increase_pitch_t *p = (str_increase_pitch_t*) rxbuf;
                double v = p->value * 0.1;
                if (angle_y_target < MAX_ANGLE-v){
                    angle_y_target += v;
                }else{
                    angle_y_target = MAX_ANGLE;
                }
            }break;
            case DECREASE_PITCH:{
                str_decrease_pitch_t *p = (str_decrease_pitch_t*) rxbuf;
                double v = p->value * 0.1;
                if (angle_y_target > -MAX_ANGLE+v){
                    angle_y_target -= v;
                }else{
                    angle_y_target = -MAX_ANGLE;
                }
            }break;
            case INCREASE_SPEED:{
                str_increase_speed_t *p = (str_increase_speed_t*) rxbuf;
                debug("->increase speed", p->value);
                char str[50];
                itoa(p->value,str,10);
                debug(str);
                debug("\r\n");
                if (update_motor_speed < MAX_MOTOR_SPEED){
                    if (MAX_MOTOR_SPEED-update_motor_speed >= p->value){
                        update_motor_speed += p->value;
                    }else{
                        update_motor_speed = MAX_MOTOR_SPEED;
                    }
                    
                }
            }break;
            case DECREASE_SPEED:{
                str_decrease_speed_t *p = (str_decrease_speed_t*) rxbuf;
                debug("->decrease speed", p->value);
                char str[50];
                itoa(p->value,str,10);
                debug(str);
                debug("\r\n");
                if (update_motor_speed > 0){
                    if (update_motor_speed >= p->value){
                        update_motor_speed -= p->value;
                    }else{
                        update_motor_speed = 0;
                    }
                }
            }break;
            case RESET_ANGLES:
                angle_x_target = angle_y_target = 0;
            break;
            case CALIB_ESC:{
                esc_calib();
            }break;
            case INCREASE_P:
                _p_regular += 0.1;
            break;
            case DECREASE_P:
                _p_regular -= 0.1;
            break;
            case SET_KP:{
                double kp = (((uint16_t)(rxbuf[3]))<<8)&0xff00 | rxbuf[4]&0xff;
                kp += ((double)((int16_t)((rxbuf[5]<<8)&0xff00 | rxbuf[6]&0xff)))/10000;
                pid_set_kp(&pid_m0, kp);
                pid_set_kp(&pid_m1, kp);
                pid_set_kp(&pid_m2, kp);
                pid_set_kp(&pid_m3, kp);
            }break;
            case SET_KD:{
                double kd = (((uint16_t)(rxbuf[3]))<<8)&0xff00 | rxbuf[4]&0xff;
                kd += ((double)((int16_t)((rxbuf[5]<<8)&0xff00 | rxbuf[6]&0xff)))/10000;
                pid_set_ki(&pid_m0, kd);
                pid_set_ki(&pid_m1, kd);
                pid_set_ki(&pid_m2, kd);
                pid_set_ki(&pid_m3, kd);
            }break;
            case SET_KI:{
                double ki = (((uint16_t)(rxbuf[3]))<<8)&0xff00 | rxbuf[4]&0xff;
                ki += ((double)((int16_t)((rxbuf[5]<<8)&0xff00 | rxbuf[6]&0xff)))/10000;
                pid_set_ki(&pid_m0, ki);
                pid_set_ki(&pid_m1, ki);
                pid_set_ki(&pid_m2, ki);
                pid_set_ki(&pid_m3, ki);
            }break;
            //default:

        }
    }
}

int main(void){
    timer_t t0;
    char str_buf[512];
    process_hdl_t *p;
    uint32_t time_tmp;

    DDRD |= 0b11000000; 
    PORTD &= ~(1<<7 | 1<<6);

    /*** configure serial interface ***/
    usart_init();
    PORTD |= (1<<7);
    debug("serial interface ready\r\n");

    /*** configre PID ***/
    pid_init(&pid_m1);
    pid_set_ki(&pid_m1, M1_KI);
    pid_set_kp(&pid_m1, M1_KP);
    pid_set_kd(&pid_m1, M1_KD);
    pid_set_out_min(&pid_m1, MIN_MOTOR_SPEED);
    pid_set_out_max(&pid_m1, MAX_MOTOR_SPEED);

    pid_init(&pid_m3);
    pid_set_ki(&pid_m3, M3_KI);
    pid_set_kp(&pid_m3, M3_KP);
    pid_set_kd(&pid_m3, M3_KD);
    pid_set_out_min(&pid_m3, MIN_MOTOR_SPEED);
    pid_set_out_max(&pid_m3, MAX_MOTOR_SPEED);


    /*** configure serial interface END ***/
    esc_init();

    /*** configure timer0 ***/
    timer_init(&t0, TIMER_0);
    timer_stop(&t0);
    timer_conf_interrupt(&t0, \
                        TIMER_INT_DISABLE, \
                        TIMER_INT_ENABLE, \
                        TIMER_INT_DISABLE);
    timer_set_cnt(&t0, 0x00);
    timer_set_mode(&t0, TIMER_MODE_CTC);
    timer_set_int_com_a_callback(&t0, async_timerstuff);
    //CLK=16MHz; PRESC=256; CMPA=0x04 => 80us
    //CLK=16MHz; PRESC=64; CMPA=0x04 => 20us
    timer_set_cmp_a_value(&t0, 0x04);
    //16MHz / 256   => 62,5kHz  => 16us per cycle
    //16MHz / 64    => 250kHz   =>  4us per cycle
    timer_enable(&t0, TIMER_PRESC_64);
    timer_debug(&t0, str_buf, 512);
    
    //debug(str_buf);
    /*** configure timer0 END ***/

    /*** configure mpu6050 and gyrostuff***/
    mpu_init();
    PORTD |= (1<<6);
    angle_init();
    
    /*** configure mpu6050  and gyrostuffEND***/
    //enable global interrupts
    sei();
    /*** configure processes ***/
    process_init();
    /*** configure processes END***/

    /*** Quadrocopter init-phase ***/
    //process_add(PROCESS_PRIOR_HIGH, 2, mpu_update_all_sensor_data);

    /*** Quardocopter init-phase END***/

    process_add(PROCESS_PRIOR_HIGH, 10, mpu_update_all_sensor_data);
    process_add(PROCESS_PRIOR_MED, 10, angle_calc_callback);
    process_add(PROCESS_PRIOR_LOW, 10, flight_control);
    process_add(PROCESS_PRIOR_PETTY, 1000, print_angle);
    process_add(PROCESS_PRIOR_LOW, 100, process_rx_uart);
    process_add(PROCESS_PRIOR_PETTY, 2000, print_debug_time);

    //esc_set_speed(3,50);

    while(1){
        time_tmp = timestamp_ms;
        p = process_get_timouted_hdl();
        while(p){
            cli();
            debug_time_unbusy += timestamp_ms - time_tmp+1;
            sei();
            p->function_hdl(p);
            p = process_get_timouted_hdl();
            cli();
            time_tmp = timestamp_ms;
            sei();
        }
        
    }

    return 0;
}