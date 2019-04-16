CC=avr-gcc
MCU=atmega328p
OBJCPY=avr-objcopy
PROGRAM=avrdude
PROGNAME=escTest
ELFFILE=$(basename $(PROGNAME)).elf
BINFILE=$(basename $(PROGNAME)).bin
OPTIMAZATION_FLAGS=-O1
CPU_FREQ=16000000UL

PROGDEVICE=/dev/ttyUSB0

OBJ=main.o esc.o

CFLAGS=-mmcu=${MCU} ${OPTIMAZATION_FLAGS} -DF_CPU=${CPU_FREQ}
LDFLAGS=-Wall

all: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) -o $(ELFFILE)
	$(OBJCPY) -O ihex $(ELFFILE) $(BINFILE)

%.o: %.c 
	${CC} ${CFLAGS} -c $<

install: $(OBJ)
	$(PROGRAM) -F -V -c arduino -p ATMEGA328P -P $(PROGDEVICE) -b 115200 -U flash:w:$(BINFILE)

clean:
	rm -f *.o $(ELFFILE) $(BINFILE)
