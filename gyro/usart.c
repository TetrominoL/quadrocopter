
#include "usart.h"
#include <stdarg.h>
#include <stdint.h>
#include <avr/io.h>

void usart_init(){
    UCSR0B=0x18;    //9600-Baud
    UBRR0 = 103;
}

void usart_putc(unsigned char data){
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1<<UDRE0)));
    /* Put data into buffer, sends the data */
    UDR0 = data;
}

void usart_putstr(const char *str){
    uint8_t i=0;

    while(str[i]){
        usart_putc(str[i]);
        i++;
    }
}

void usart_print(const char* str, ...){
   char c[500];
    uint16_t i=0;

   va_list args;
    va_start(args, str);

    while (*str != '\0') {
        if (*str == '%' && *(str+1) == 'd') {
            int d = va_arg(args, int);
            itoa(d,c,10);
            usart_putstr(c);
            str++;
        }else if (*str == '%' && *(str+1) == 'u') {
            unsigned int d = va_arg(args, unsigned int);
            itoa(d,c,10);
            usart_putstr(c);
            str++;
        }else if (*str == '%' && *(str+1) == 'x') {
            unsigned int d = va_arg(args, unsigned int);
            itoa(d,c,16);
            usart_putstr(c);
            str++;
        }else{
            usart_putc(*str);
        }
        
        str++;
    }
    va_end(args);
}