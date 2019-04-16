CC=avr-gcc
MCU=atmega328p
OBJCPY=avr-objcopy
PROGRAM=avrdude
PROGNAME=gyrotest
ELFFILE=$(basename $(PROGNAME)).elf
BINFILE=$(basename $(PROGNAME)).bin
OPTIMAZATION_FLAGS=-O1

GLOBAL_DEFINES=-DDEBUG=1 -DCPU_FREQ=16000000UL

PROGDEVICE=/dev/ttyUSB0

OBJ=main.o twi.o mpu6050.o usart.o

CFLAGS=-mmcu=${MCU} ${OPTIMAZATION_FLAGS} ${GLOBAL_DEFINES}
LDFLAGS=-Wall

THIS_FILE := $(lastword $(MAKEFILE_LIST))


all: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) -o $(ELFFILE)
	$(OBJCPY) -O ihex $(ELFFILE) $(BINFILE)
	$(MAKE) install

%.o: %.c 
	${CC} ${CFLAGS} -c $<

install: $(OBJ)
	$(PROGRAM) -F -V -c arduino -p ATMEGA328P -P $(PROGDEVICE) -b 115200 -U flash:w:$(BINFILE)

clean:
	rm -f *.o $(ELFFILE) $(BINFILE)
