ARDLIB_DIR = arduino
ARDLIB_SOURCEDIR = $(ARDLIB_DIR)/src
ARDLIB_BUILDDIR = $(ARDLIB_DIR)/build
ARDLIB_INCLUDEDIR = $(ARDLIB_DIR)/include

ARDLIB_SOURCE_FILES = abi.cpp CDC.cpp HardwareSerial0.cpp HardwareSerial1.cpp HardwareSerial2.cpp HardwareSerial3.cpp HardwareSerial.cpp HID.cpp hooks.c IPAddress.cpp new.cpp Print.cpp Stream.cpp Tone.cpp USBCore.cpp WInterrupts.c wiring_analog.c wiring.c wiring_digital.c wiring_pulse.c wiring_pulse.S wiring_shift.c WMath.cpp WString.cpp
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
CC_ARD = avr-g++
CFLAGS_ARD  = -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR -I$(ARDLIB_INCLUDEDIR)
SFLAGS_ARD = -c -g -x assembler-with-cpp -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR
LDFLAGS_ARD = -w -Os -Wl,--gc-sections,--relax -mmcu=atmega2560
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
	avr-gcc $(LDFLAGS_ARD) $^ -o $@

$(BUILDDIR)/%_ard.o: $(SOURCEDIR)/%
	$(CC_ARD) $(CFLAGS_ARD) $< -o $@

$(ARDLIB_BUILDDIR)/%.cpp.o: $(ARDLIB_SOURCEDIR)/%.cpp
	$(CC_ARD) $(CFLAGS_ARD) $< -o $@

$(ARDLIB_BUILDDIR)/%.c.o: $(ARDLIB_SOURCEDIR)/%.c
	avr-gcc $(CFLAGS_ARD) $< -o $@

$(ARDLIB_BUILDDIR)/%.S.o: $(ARDLIB_SOURCEDIR)/%.S
	avr-gcc $(SFLAGS_ARD) $< -o $@

flash: control.bin
	avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $<.eep
	avr-objcopy -O ihex -R .eeprom $< $<.hex
	avrdude -Carduino/avrdude.conf -v -patmega2560 -cwiring -P/dev/ttyUSB0 -b115200 -D -Uflash:w:$<.hex:i

.PHONY: clean

clean:
	rm -f $(BUILDDIR)/*
	rm -f $(EXE)
	rm -f $(ARDLIB_BUILDDIR)/*
	rm -f $(EXE_ARD)
	rm -f -d $(BUILDDIR)
	rm -f -d $(ARDLIB_BUILDDIR)

