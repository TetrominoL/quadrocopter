/**
 * Author:  Simon Lindhorst
 * Date:    02.05.2019
 */

/**
 * Diese Bibliotek dient dazu die Timer des ATmega328p anzusprechen
 */
#ifndef _TIMER_H
#define _TIMER_H

#include <stdint.h>
#include <stdio.h>

#define TIMER_INT_ENABLE        1
#define TIMER_INT_UNAFFECTED    0
#define TIMER_INT_DISABLE       2

#define TIMER_PRESC_STOP        ((0<<CS02)|(0<<CS01)|(0<<CS00))
#define TIMER_PRESC_NO          ((0<<CS02)|(0<<CS01)|(1<<CS00))
#define TIMER_PRESC_8           ((0<<CS02)|(1<<CS01)|(0<<CS00))
#define TIMER_PRESC_64          ((0<<CS02)|(1<<CS01)|(1<<CS00))
#define TIMER_PRESC_256         ((1<<CS02)|(0<<CS01)|(0<<CS00))
#define TIMER_PRESC_1024        ((1<<CS02)|(0<<CS01)|(1<<CS00))

#define TIMER_MODE_NORMAL   ((0<<WGM02) | (0<<WGM01) | (0<<WGM00))
#define TIMER_MODE_CTC      ((0<<WGM02) | (1<<WGM01) | (0<<WGM00))

#define TIMER_0 0
#define TIMER_1 1

typedef struct{
    uint8_t *TCCRxB;
    uint8_t *TCCRxA;
    uint8_t *TIMSKx;
    uint8_t *TCNTx;
    uint8_t *OCRxA;
    uint8_t *OCRxB;
    uint8_t *TIFRx;
    uint8_t timer_nr;
} timer_t;

void timer_init(timer_t *t, uint8_t timer_nr);

void timer_enable(timer_t *t, uint8_t prescaler);

void timer_stop(timer_t *t);

void timer_set_cnt(timer_t *t, uint8_t value);

void timer_conf_interrupt(timer_t *t, uint8_t ovf, uint8_t cmp_a, uint8_t cmp_b);

void timer_set_cmp_a_value(timer_t *t, uint8_t cmp_value);

void timer_set_cmp_b_value(timer_t *t, uint8_t cmp_value);

void timer_set_mode(timer_t *t, uint8_t mode);

void timer_set_int_ovf_callback(timer_t *t, void(*callback_func)(void));

void timer_set_int_com_a_callback(timer_t *t, void(*callback_func)(void));

void timer_set_int_com_b_callback(timer_t *t, void(*callback_func)(void));

void timer_debug(timer_t *t, char *buf, uint16_t buf_size);

#endif //#ifndef _TIMER_H