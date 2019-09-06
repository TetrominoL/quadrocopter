#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "esc.h"

int main(void){
    esc_init();

    sei();
    esc_calib();

    uint8_t i=0;
    esc_set_speed(0,50);
    esc_set_speed_all(50);
    while(1){
        if(++i > 100)
            i=0;
        esc_set_speed_all(i);
        _delay_ms(300);
        
    }

    return 0;
}


