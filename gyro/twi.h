/****************************************************
in follow to:
    Low Level I2C Data Transreceiveing APIs

    PLEASE SEE WWW.EXTREMEELECTRONICS.CO.IN FOR DETAILED 
    SCHEMATICS,USER GUIDE AND VIDOES.

    COPYRIGHT (C) 2008-2009 EXTREME ELECTRONICS INDIA

****************************************************/

#include <stdint.h>


void twi_init();

void twi_close();

void twi_start();

void twi_stop();

uint8_t twi_write_byte(uint8_t data);

uint8_t twi_read_byte(uint8_t *data,uint8_t ack);