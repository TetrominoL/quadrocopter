CC=avr-gcc
MCU=atmega328p
OBJCPY=avr-objcopy
PROGRAM=avrdude
PROGNAME=gyrotest
ELFFILE=$(basename $(PROGNAME)).elf
BINFILE=$(basename $(PROGNAME)).bin
OPTIMAZATION_FLAGS=-O2

GLOBAL_DEFINES= -DAVR -DDEBUG=1 -DF_CPU=16000000UL

PROGDEVICE=/dev/ttyUSB0

OBJ=main.o twi.o mpu6050.o usart.o angle_calc.o timer.o process.o esc.o

CFLAGS=-mmcu=${MCU} ${OPTIMAZATION_FLAGS} ${GLOBAL_DEFINES} -std=c99
LDFLAGS=-Wall

THIS_FILE := $(lastword $(MAKEFILE_LIST))


all: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) -o $(ELFFILE)
	$(OBJCPY) -O ihex $(ELFFILE) $(BINFILE)

#$(MAKE) install

%.o: %.c 
	${CC} ${CFLAGS} -c $<

install: $(OBJ)
	#$(PROGRAM) -F -V -c arduino -p ATMEGA328P -P $(PROGDEVICE) -b 115200 -U flash:w:$(BINFILE)
	./transfere_file.sh gyrotest.elf 10.10.10.1 12346

clean:
	rm -f *.o $(ELFFILE) $(BINFILE)
