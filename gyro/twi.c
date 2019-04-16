#include "twi.h"
#include <avr/io.h>
#include "usart.h"

void twi_init(){
	//set up TWI module
	TWBR = 2;
	TWSR |=((1<<TWPS1)|(1<<TWPS0));

	//enable the TWI module
	TWCR|=(1<<TWEN);
}

void twi_close(){
	//disable the module
	TWCR&=(~(1<<TWEN));
}


void twi_start(){
	//put start condition on bus
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	//poll till done
	while (!(TWCR & (1<<TWINT)));

}

void twi_stop(){
	//put stop condition on bus
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	//wait for finish
	while(TWCR & (1<<TWSTO));
}

uint8_t twi_write_byte(uint8_t data){
	TWDR = data;	

	//initiate transfer
	TWCR = (1<<TWEN) | (1<<TWINT);

	//wait for done
	while(!(TWCR & (1<<TWINT)));

	//check currend status
	if((TWSR & 0xF8) == 0x18 || (TWSR & 0xF8) == 0x28 || (TWSR & 0xF8) == 0x40){
		//transmitted and ACK received
		return 0;
	}
	else{
		//error occured
		return 1;
	}
}

uint8_t twi_read_byte(uint8_t *data, uint8_t ack){
	if(ack){	//send ACK after reception
		TWCR|=(1<<TWEA);
	}
	else{		//send NACK after reception
		TWCR&=(~(1<<TWEA));
	}

	//enable reception
	TWCR|=(1<<TWINT);

	//wait for done
	while(!(TWCR & (1<<TWINT)));

	//check status
	if((TWSR & 0xF8) == 0x58 || (TWSR & 0xF8) == 0x50){
		//data received
		PORTD |= (1<<1);

		*data=TWDR;
		return 0;
	}
	else{
		//error
		return 1;
	}
	
}
