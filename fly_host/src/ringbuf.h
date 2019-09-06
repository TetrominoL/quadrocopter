#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef AVR
    #include <avr/interrupt.h>
    #define perror(...)
#endif

typedef struct {
    volatile char        *data;
    uint16_t    size;
    uint16_t    r;
    volatile uint16_t    w;
} str_ringbuf_t;

int init_ringbuf(str_ringbuf_t *buf, uint16_t size){
    buf->data = (char*) malloc(size);
    if(!(buf->data)){
        perror("get space for ringbuf");
        return -1;
    }
    buf->size = size;
    buf->r = buf->w;
    return 0;
}

int get_ringbuf_load(str_ringbuf_t *buf){
    uint16_t r = buf->r;
#ifdef ARM
    cli();
#endif    
    uint16_t w = buf->w;
#ifdef ARM
    sei();
#endif
    uint16_t s = buf->size;
    uint16_t res;

    if(r > w){
        res = s-r + w;
    }else{
        res = w-r;
    }
    return res;
}

int write_ringbuf(str_ringbuf_t *buf, char *d, uint16_t size){
    uint16_t i;

    if(buf->size - get_ringbuf_load(buf) < size){
        return -1;
    }
    i=0;

    while(i < size){
        buf->data[buf->w] = d[i];
        if(++(buf->w) >= buf->size){
            buf->w = 0;
        }
        i++;
    }
    return 0;
}

int read_ringbuf(str_ringbuf_t *buf, char *d, uint16_t size){
    uint16_t s;
    uint16_t buf_load = get_ringbuf_load(buf);
    uint16_t i;

    if(buf_load > size){
        s = size;
    }else{
        s = buf_load;
    }

    for(i=0; i<s; i++){
        d[i] = buf->data[buf->r];
        if(++(buf->r) >= buf->size){
            buf->r = 0;
        }
    }
    return s;
}

char peep_readbyte_ringbuf(str_ringbuf_t *buf){
    return buf->data[buf->r];
}


#endif //RINGBUF_H