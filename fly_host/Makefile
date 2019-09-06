CC=arm-linux-gnueabihf-gcc
OBJCPY=objcopy
EXECUTABLE=fly_host
ELFFILE=$(basename $(EXECUTABLE)).elf
BINFILE=$(basename $(EXECUTABLE)).bin
BUILDDIR=build
SOURCEDIR=src
SOURCES = $(wildcard $(SOURCEDIR)/*.c)
OBJ= $(patsubst $(SOURCEDIR)/%.c,$(BUILDDIR)/%.o,$(SOURCES))

OPTIMAZATION_FLAGS=
GLOBAL_DEFINES=

CFLAGS=${OPTIMAZATION_FLAGS} ${GLOBAL_DEFINES} -std=c99 -marm -march=armv6
LDFLAGS=-Wall

THIS_FILE := $(lastword $(MAKEFILE_LIST))


all: dir $(BUILDDIR)/$(EXECUTABLE)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) -o $(ELFFILE)

dir:
	mkdir -p $(BUILDDIR)

$(BUILDDIR)/$(EXECUTABLE): $(OBJ)
	$(CC) $^ $(LDFLAGS) -o $@

$(BUILDDIR)/%.o: $(SOURCEDIR)/%.c
	$(CC) -c $(CFLAGS) $< -o $@

install:
	echo "Transfere to 10.10.10.1/12345"
	./transfere_file.sh fly_host.elf 10.10.10.1 12345

clean:
	rm -fr $(BUILDDIR)
