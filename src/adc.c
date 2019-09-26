/* Author: Simon Lindhorst <lindhorst.simon@live.de>
 */

#include "adc.h"
#include <avr/io.h>


void adcVoltageReferenceSelection(uint8_t voltageReferenceSelection){
    ADMUX = (ADMUX & 0x2F) | voltageReferenceSelection;
}

void adcResultAdjust(uint8_t adjustment){
    ADMUX = (ADMUX & 0xCF) | adjustment;
}

void adcSelectChannel(uint8_t channel){
    ADMUX = (ADMUX & 0xE0) | channel;
}

void adcEnable(){
    ADCSRA |= (1<<7);
}

void adcDisable(){
    ADCSRA &= ~(1<<7);
}

void adcStart(){
    adcClearInterruptFlag();
    ADCSRA |= (1<<6); 
}

void adcClearInterruptFlag(){
    ADCSRA |= (1<<4);
}

void adcSetMode(uint8_t mode){
    ADCSRA |= (1<<ADATE);
    ADCSRA = (ADCSRA & 0xf8) | (mode & 0x07);
}

uint8_t adcBusy(){
    return ADCSRA & (1<<4);
}

void adcSelectPrescaler(uint8_t prescaler){
    ADCSRA = (ADCSRA & 0xf8) | prescaler;
}

uint8_t adcGet8BitResult(){
    return ADCH;
}

uint16_t adcGetResult(){
    uint16_t res = ADCL;
    res |= (ADCH<<8);

    return res;
}