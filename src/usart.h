
#ifndef USART_H
#define USART_H

#include <stdint.h>
#include <stdio.h>

typedef struct {
    volatile char        *data;
    uint16_t    size;
    uint16_t    r;
    volatile uint16_t    w;
} str_ringbuf_tt;

volatile str_ringbuf_tt _usarRxRingBuf;

#ifdef DEBUG
#define debug(str, ...) printf(str, ##__VA_ARGS__)
#else
#define debug(str, ...)
#endif

static int uart_putchar(char c, FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void usart_init();

void usart_putc(unsigned char data);

void usart_putstr(char *str);

void usart_print(char* str, ...);

uint8_t usart_get_rxdata_load();

uint8_t usart_get_rxdata(char *buf, uint8_t size);


static int uart_putchar(char c, FILE *stream) {
    
  if (c == '\n') usart_putc('\r');
  usart_putc(c);
  return 0;
}

#endif //USART_H