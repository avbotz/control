ARDLIB_DIR = arduino
ARDLIB_SOURCEDIR = $(ARDLIB_DIR)/src
ARDLIB_BUILDDIR = $(ARDLIB_DIR)/build
ARDLIB_INCLUDEDIR = $(ARDLIB_DIR)/include

ARDLIB_SOURCE_FILES = EEPROM.cpp HardwareSerial.cpp Print.cpp WString.cpp wiring_analog.c wiring_digital.c new.cpp wiring.c
ARDLIB_SOURCES = $(patsubst %,$(ARDLIB_SOURCEDIR)/%,$(ARDLIB_SOURCE_FILES))
ARDLIB_OBJECTS = $(patsubst $(ARDLIB_SOURCEDIR)/%,$(ARDLIB_BUILDDIR)/%.o,$(ARDLIB_SOURCES))

SOURCEDIR = src
BUILDDIR = build

EXE = control
CC = g++
CFLAGS  = -ggdb -c -std=c++11
LDFLAGS =
SOURCE_FILES = control.cpp main.cpp pid.cpp io_cpu.cpp
SOURCES = $(patsubst %,$(SOURCEDIR)/%,$(SOURCE_FILES))
OBJECTS = $(patsubst $(SOURCEDIR)/%,$(BUILDDIR)/%.o,$(SOURCES))

EXE_ARD = control.bin
CC_ARD = avr-g++ -mmcu=avr6
CFLAGS_ARD  = -ggdb -c -std=c++11 -I$(ARDLIB_INCLUDEDIR) -DF_CPU=16000000 -D__COMPILING_AVR_LIBC__ -D__AVR_ATmega2560__ -DUBRR0H -DUBRR1H -DUBRR2H -DUBRR3H
LDFLAGS_ARD = -Wl,--defsym=__heap_end=0
SOURCE_FILES_ARD = control.cpp main.cpp pid.cpp io_arduino.cpp
SOURCES_ARD = $(patsubst %,$(SOURCEDIR)/%,$(SOURCE_FILES_ARD))
OBJECTS_ARD = $(patsubst $(SOURCEDIR)/%,$(BUILDDIR)/%_ard.o,$(SOURCES_ARD))

all: arduino

arduino: $(BUILDDIR) $(ARDLIB_BUILDDIR) control.bin

cpu: $(BUILDDIR) control

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(ARDLIB_BUILDDIR):
	mkdir -p $(ARDLIB_BUILDDIR)

$(EXE): $(OBJECTS)
	$(CC) $^ $(LDFLAGS) -o $@

$(BUILDDIR)/%.o: $(SOURCEDIR)/%
	$(CC) $(CFLAGS) $< -o $@

$(EXE_ARD): $(ARDLIB_OBJECTS) $(OBJECTS_ARD)
	$(CC_ARD) -mmcu=avr6 -latmega2560 && $(CC_ARD) $^ $(LDFLAGS_ARD) -latmega2560 -o $@ || /bin/true
	$(CC_ARD) -mmcu=avr6 -latmega2560 || $(CC_ARD) $^ $(LDFLAGS_ARD) -o $@

$(BUILDDIR)/%_ard.o: $(SOURCEDIR)/%
	$(CC_ARD) $(CFLAGS_ARD) $< -o $@

$(ARDLIB_BUILDDIR)/%.o: $(ARDLIB_SOURCEDIR)/%
	$(CC_ARD) $(CFLAGS_ARD) $< -o $@

.PHONY: clean

clean:
	rm -f $(BUILDDIR)/*.o
	rm -f $(EXE)
	rm -f $(ARDLIB_BUILDDIR)/*.o
	rm -f $(EXE_ARD)
	rmdir $(BUILDDIR)
	rmdir $(ARDLIB_BUILDDIR)

