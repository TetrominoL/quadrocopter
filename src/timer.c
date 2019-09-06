#include "timer.h"
#include <avr/interrupt.h>
#include <avr/io.h>

#define TIMER_MASK_PRESCALER    ~(0x07)

#define TIMER_VAL_TOVx  (1<<0)
#define TIMER_VAL_OCFxA (1<<1)
#define TIMER_VAL_OCFxB (1<<2)

void (*_timer0_int_cmpa_callback)(void);
void (*_timer0_int_cmpb_callback)(void);
void (*_timer0_int_ovf_callback)(void);
void (*_timer1_int_cmpa_callback)(void);
void (*_timer1_int_cmpb_callback)(void);
void (*_timer1_int_ovf_callback)(void);

void timer_init(timer_t *t, uint8_t timer_nr){
    switch(timer_nr){
        case 0:
            t->TCCRxA   = (uint8_t*) &TCCR0A;
            t->TCCRxB   = (uint8_t*) &TCCR0B;
            t->TIMSKx   = (uint8_t*) &TIMSK0;
            t->TCNTx    = (uint8_t*) &TCNT0;
            t->OCRxA    = (uint8_t*) &OCR0A;
            t->OCRxB    = (uint8_t*) &OCR0B;
            t->TIFRx    = (uint8_t*) &TIFR0;
            t->timer_nr = timer_nr;
        break;
        case 1:
        break;
    }
}

void timer_enable(timer_t *t, uint8_t prescaler){
    *t->TCCRxB = (*(t->TCCRxB) & TIMER_MASK_PRESCALER) | prescaler;
}

void timer_stop(timer_t *t){
    *t->TCCRxB = (*t->TCCRxB & TIMER_MASK_PRESCALER);
}

void timer_set_cnt(timer_t *t, uint8_t value){
    *t->TCNTx = value;
}

void timer_conf_interrupt(timer_t *t, uint8_t ovf, uint8_t cmp_a, uint8_t cmp_b){
    uint8_t regval = *t->TIMSKx;
    
    if(ovf == TIMER_INT_ENABLE){
        regval |= TIMER_VAL_TOVx;  
    }else if(ovf == TIMER_INT_DISABLE){
        regval &= ~TIMER_VAL_TOVx; 
    }

    if(cmp_a == TIMER_INT_ENABLE){
        regval |= TIMER_VAL_OCFxA;  
    }else if(cmp_a == TIMER_INT_DISABLE){
        regval &= ~TIMER_VAL_OCFxA; 
    }

    if(cmp_b == TIMER_INT_ENABLE){
        regval |= TIMER_VAL_OCFxB;  
    }else if(cmp_b == TIMER_INT_DISABLE){
        regval &= ~TIMER_VAL_OCFxB; 
    }

    *t->TIMSKx = regval;
}

void timer_set_cmp_a_value(timer_t *t, uint8_t cmp_value){
    *t->OCRxA = cmp_value;
}

void timer_set_cmp_b_value(timer_t *t, uint8_t cmp_value){
    *t->OCRxB = cmp_value;
}

void timer_set_mode(timer_t *t, uint8_t mode){
    *t->TCCRxA = (*t->TCCRxA&(~(1<<WGM01))&(~(1<<WGM00))) | (mode & ((1<<WGM01)|(1<<WGM00)));
    *t->TCCRxB = (*t->TCCRxB&(~(1<<WGM02))) | (mode & (1<<WGM02));
}

void timer_set_int_ovf_callback(timer_t *t, void(*callback_func)(void)){
    switch(t->timer_nr){
        case 0:
            _timer0_int_ovf_callback = callback_func;
        break;
        case 1:
            _timer1_int_ovf_callback = callback_func;
        break;
    }
}

void timer_set_int_com_a_callback(timer_t *t, void(*callback_func)(void)){
    switch(t->timer_nr){
        case 0:
            _timer0_int_cmpa_callback = callback_func;
        break;
        case 1:
            _timer1_int_cmpa_callback = callback_func;
        break;
    }
}

void timer_set_int_com_b_callback(timer_t *t, void(*callback_func)(void)){
    switch(t->timer_nr){
        case 0:
            _timer0_int_cmpb_callback = callback_func;
        break;
        case 1:
            _timer1_int_cmpb_callback = callback_func;
        break;
    }
}

void timer_debug(timer_t *t, char *buf, uint16_t buf_size){

/* FÜhrt zu einem Speicherfehler??? WARUM???? */
//     snprintf(buf,buf_size,"\
// TCCRxB: %x\r\n \
// TCCRxA: %x\r\n \
// TIMSKx: %x\r\n \
// TCNTx: %x\r\n \
// OCRxA: %x\r\n \
// OCRxB: %x\r\n \
// TIFRx: %x\r\n \
// timer_nr: %x\r\n"\
//     ,*t->TCCRxB,*t->TCCRxA,*t->TIMSKx,*t->TCNTx,*t->OCRxA,*t->OCRxB,*t->TIFRx,t->timer_nr);
}

ISR (TIMER0_COMPA_vect){
    _timer0_int_cmpa_callback();
}
ISR (TIMER0_OVF_vect){
    _timer0_int_ovf_callback();
}
ISR (TIMER0_COMPB_vect){
    _timer0_int_cmpb_callback();
}
ISR (TIMER1_COMPA_vect){
    _timer1_int_cmpa_callback();
}
ISR (TIMER1_OVF_vect){
    _timer1_int_ovf_callback();
}
ISR (TIMER1_COMPB_vect){
    _timer1_int_cmpb_callback();
}