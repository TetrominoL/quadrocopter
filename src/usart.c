
#include "usart.h"
#include <stdarg.h>
#include <stdint.h>
#include <avr/io.h>
#include <stdlib.h>
#include "ringbuf.h"
#include <avr/interrupt.h>

#define RX_BUF_SIZE 0xff

volatile char _usartRxBuf[RX_BUF_SIZE];

void usart_init(){
    stdout = &mystdout;
    _usarRxRingBuf.data = _usartRxBuf;
    if(init_ringbuf(&_usarRxRingBuf, RX_BUF_SIZE)){
        while(1);
    }
    /*UCSR0B=0x18;    //9600-Baud
    UBRR0 = 103;*/
    //UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);

    //Enable TX,RX, RX Complete Interrupt
    UCSR0B=0x98;    //57600-Baud max. for arduino uno ftdi-chip
    UBRR0 = 16;
}

void usart_putc(unsigned char data){
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1<<UDRE0)));
    /* Put data into buffer, sends the data */
    UDR0 = data;
}

void usart_putstr(char *str){
    uint8_t i=0;

    while(str[i]){
        usart_putc(str[i]);
        i++;
    }
}

uint8_t usart_get_rxdata_load(){
    return get_ringbuf_load(&_usarRxRingBuf);
}

uint8_t usart_get_rxdata(char *buf, uint8_t size){
    return read_ringbuf(&_usarRxRingBuf, buf, size);
}

void usart_print(char* str, ...){
   char c[500];
   uint16_t i=0;

   va_list b;
   va_start(b, str);
   vsprintf(c,str,b);
   usart_putstr(c);
   va_end(b);
    return;
    // while (*str != '\0') {
    //     if (*str == '%' && *(str+1) == 'd') {
    //         int d = va_arg(args, int);
    //         itoa(d,c,10);
    //         usart_putstr(c);
    //         str++;
    //     }else if (*str == '%' && *(str+1) == 'u') {
    //         unsigned int d = va_arg(args, unsigned int);
    //         utoa(d,c,10);
    //         usart_putstr(c);
    //         str++;
    //     }else if (*str == '%' && *(str+1) == 'x') {
    //         char d = va_arg(args, char);
    //         //itoa(d,c,16);
    //         //usart_putstr(c);
    //         //str++;
    //     }else if (*str == '%' && *(str+1) == 'f') {
    //         double d = va_arg(args, double);
    //         dtostrf(d, 4, 3, c);
    //         usart_putstr(c);
    //         str++;
    //     }else{
    //         usart_putc(*str);
    //     }
        
    //     str++;
    // }
    // va_end(args);
}

ISR(USART_RX_vect){
    char data = UDR0;
    write_ringbuf(&_usarRxRingBuf, &data, 1);
}