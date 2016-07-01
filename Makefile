ARDLIB_DIR = arduino
ARDLIB_SOURCEDIR = $(ARDLIB_DIR)/src
ARDLIB_BUILDDIR = $(ARDLIB_DIR)/build
ARDLIB_INCLUDEDIR = $(ARDLIB_DIR)/include

ARDLIB_SOURCE_FILES = abi.cpp CDC.cpp HardwareSerial0.cpp HardwareSerial1.cpp HardwareSerial2.cpp HardwareSerial3.cpp HardwareSerial.cpp HID.cpp hooks.c IPAddress.cpp new.cpp Print.cpp Stream.cpp Tone.cpp USBCore.cpp WInterrupts.c wiring_analog.c wiring.c wiring_digital.c wiring_pulse.c wiring_pulse.S wiring_shift.c WMath.cpp WString.cpp EEPROM.cpp
ARDLIB_SOURCES = $(patsubst %,$(ARDLIB_SOURCEDIR)/%,$(ARDLIB_SOURCE_FILES))
ARDLIB_OBJECTS = $(patsubst $(ARDLIB_SOURCEDIR)/%,$(ARDLIB_BUILDDIR)/%.o,$(ARDLIB_SOURCES))

SOURCEDIR = src
BUILDDIR = build

EXE = control
CXX = g++
CXXFLAGS  = -ggdb -c -std=c++11 -I$(SOURCEDIR) $(CPPFLAGS)
CC = gcc
CFLAGS = -g -c -std=c11 -I$(SOURCEDIR) $(CPPFLAGS)
CPPFLAGS = -DIEEE754
LDFLAGS =
SOURCE_FILES = control.cpp pid.cpp io_cpu.cpp config.cpp rotation.c
SOURCES = $(patsubst %,$(SOURCEDIR)/%,$(SOURCE_FILES))
OBJECTS = $(patsubst $(SOURCEDIR)/%,$(BUILDDIR)/%.o,$(SOURCES))

EXE_PC = control_pc
SOURCE_FILES_PC = control.cpp pid.cpp io.cpp ahrs.c crc_xmodem_generic.c io_ahrs_pc.c io_pc.cpp io_depth_pc.cpp io_m5_pc.c m5.c crc32.c io_relay_pc.c config.cpp rotation.c
SOURCES_PC = $(patsubst %,$(SOURCEDIR)/%,$(SOURCE_FILES_PC))
OBJECTS_PC = $(patsubst $(SOURCEDIR)/%,$(BUILDDIR)/%_pc.o,$(SOURCES_PC))

EXE_ARD = control.bin
CXX_ARD = avr-g++
CXXFLAGS_ARD = -c -std=c++11 -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR -DIEEE754 -DAVR -I$(SOURCEDIR) -I$(ARDLIB_INCLUDEDIR)
CC_ARD = avr-gcc
CFLAGS_ARD = -c -std=c11 -g -Os -w -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR -DIEEE754 -DAVR -DNDEBUG -I$(SOURCEDIR) -I$(ARDLIB_INCLUDEDIR)
SFLAGS_ARD = -c -g -x assembler-with-cpp -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR
LDFLAGS_ARD = -w -Os -Wl,--gc-sections,--relax -mmcu=atmega2560 -Wl,-u,vfprintf -lprintf_flt -lm -Wl,-u,vfscanf -lscanf_flt -lm
SOURCE_FILES_ARD = control.cpp pid.cpp io.cpp ahrs.c crc_xmodem_generic.c io_ahrs_avr.c io_arduino.cpp io_depth_arduino.cpp io_kill_arduino.cpp io_m5_avr.c m5.c crc32.c io_relay_avr.c io_millis_arduino.cpp config.cpp rotation.c
SOURCES_ARD = $(patsubst %,$(SOURCEDIR)/%,$(SOURCE_FILES_ARD))
OBJECTS_ARD = $(patsubst $(SOURCEDIR)/%,$(BUILDDIR)/%_ard.o,$(SOURCES_ARD))

all: arduino

arduino: $(BUILDDIR) $(ARDLIB_BUILDDIR) control.bin

cpu: $(BUILDDIR) control

pc : $(BUILDDIR) $(EXE_PC)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(ARDLIB_BUILDDIR):
	mkdir -p $(ARDLIB_BUILDDIR)

$(EXE): $(OBJECTS)
	$(CXX) $^ $(LDFLAGS) -o $@

$(BUILDDIR)/%.cpp.o: $(SOURCEDIR)/%.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILDDIR)/%.c.o: $(SOURCEDIR)/%.c
	$(CC) $(CFLAGS) $< -o $@

$(BUILDDIR)/%.cpp.o: $(SOURCEDIR)/cpu/%.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILDDIR)/%.c.o: $(SOURCEDIR)/cpu/%.c
	$(CC) $(CFLAGS) $< -o $@


$(EXE_PC): $(OBJECTS_PC)
	$(CXX) $^ $(LDFLAGS) -pthread -o $@

$(BUILDDIR)/%.cpp_pc.o: $(SOURCEDIR)/%.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILDDIR)/%.c_pc.o: $(SOURCEDIR)/%.c
	$(CC) $(CFLAGS) $< -o $@

$(BUILDDIR)/%.cpp_pc.o: $(SOURCEDIR)/pc/%.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILDDIR)/%.c_pc.o: $(SOURCEDIR)/pc/%.c
	$(CC) $(CFLAGS) $< -o $@





$(EXE_ARD): $(ARDLIB_OBJECTS) $(OBJECTS_ARD)
	$(CC_ARD) $(LDFLAGS_ARD) $^ -o $@

$(BUILDDIR)/%.cpp_ard.o: $(SOURCEDIR)/%.cpp
	$(CXX_ARD) $(CXXFLAGS_ARD) $< -o $@

$(BUILDDIR)/%.c_ard.o: $(SOURCEDIR)/%.c
	$(CC_ARD) $(CFLAGS_ARD) $< -o $@

$(BUILDDIR)/%.cpp_ard.o: $(SOURCEDIR)/arduino/%.cpp
	$(CXX_ARD) $(CXXFLAGS_ARD) $< -o $@

$(BUILDDIR)/%.c_ard.o: $(SOURCEDIR)/arduino/%.c
	$(CC_ARD) $(CFLAGS_ARD) $< -o $@


$(ARDLIB_BUILDDIR)/%.cpp.o: $(ARDLIB_SOURCEDIR)/%.cpp
	$(CC_ARD) $(CFLAGS_ARD) $< -o $@

$(ARDLIB_BUILDDIR)/%.c.o: $(ARDLIB_SOURCEDIR)/%.c
	$(CC_ARD) $(CFLAGS_ARD) $< -o $@

$(ARDLIB_BUILDDIR)/%.S.o: $(ARDLIB_SOURCEDIR)/%.S
	$(CC_ARD) $(SFLAGS_ARD) $< -o $@

flash: control.bin
	avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $<.eep
	avr-objcopy -O ihex -R .eeprom $< $<.hex
	avrdude -v -patmega2560 -cwiring -P/dev/ttyUSB1 -b115200 -D -Uflash:w:$<.hex:i

.PHONY: clean

clean:
	rm -f $(BUILDDIR)/*
	rm -f $(EXE)
	rm -f $(ARDLIB_BUILDDIR)/*
	rm -f $(EXE_ARD)
	rm -f $(EXE_PC)
	rm -f -d $(BUILDDIR)
	rm -f -d $(ARDLIB_BUILDDIR)

