#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#define AREF                                ((0<<REFS1) | (0<<REFS0))
#define AVCC                                ((0<<REFS1) | (1<<REFS0))
#define INTERNAL_1_1V                       ((1<<REFS1) | (1<<REFS0))

#define LEFT_ADJUST_RESULT                  1
#define RIGHT_ADJUST_RESULT                 0


enum {
    MODE_FREE_RUNNING = 0x00,
    MODE_ANALOG_COPARATOR,
    MODE_EXTERNAL_INTERRUPT_REQ_0,
    MODE_TIMER0_CMP_MATCH_A,
    MODE_TIMER0_OVF,
    MODE_TIMER1_CMP_MATCH_B,
    MODE_TIMER1_OVF,
    MODE_TIMER1_CAPTURE_EVENT
};

enum{
    CHANNEL_ADC0 = 0x00,
    CHANNEL_ADC1,
    CHANNEL_ADC2,
    CHANNEL_ADC3,
    CHANNEL_ADC4,
    CHANNEL_ADC5,
    CHANNEL_ADC6,
    CHANNEL_ADC7,
    CHANNEL_ADC8,
    CHANNEL_1_1V = 0x0e,
    CHANNEL_0V
};

enum{
    PRESC_2 = 0x01,
    PRESC_4,
    PRESC_8,
    PRESC_16,
    PRESC_32,
    PRESC_64,
    PRESC_128,
};

void adcVoltageReferenceSelection(uint8_t voltageReferenceSelection);

void adcResultAdjust(uint8_t adjustment);

void adcSelectChannel(uint8_t channel);

void adcEnable();

void adcDisable();

void adcStart();

void adcSetMode(uint8_t mode);

void adcClearInterruptFlag();

uint8_t adcBusy();

void adcSelectPrescaler(uint8_t prescaler);

uint8_t adcGet8BitResult();

uint16_t adcGetResult();

#endif //ADC_H