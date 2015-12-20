ARDLIB_DIR = arduino
ARDLIB_SOURCEDIR = $(ARDLIB_DIR)/src
ARDLIB_BUILDDIR = $(ARDLIB_DIR)/build
ARDLIB_INCLUDEDIR = $(ARDLIB_DIR)/include

ARDLIB_SOURCES = $(wildcard $(ARDLIB_SOURCEDIR)/*.cpp)
ARDLIB_OBJECTS = $(patsubst $(ARDLIB_SOURCEDIR)/%.cpp,$(ARDLIB_BUILDDIR)/%.o,$(ARDLIB_SOURCES))

SOURCEDIR = src
BUILDDIR = build

EXE = control
CC = g++
CFLAGS  = -ggdb -c -std=c++11
LDFLAGS =
SOURCE_FILES = control main pid io_cpu
SOURCES = $(patsubst %,$(SOURCEDIR)/%.cpp,$(SOURCE_FILES))
OBJECTS = $(patsubst $(SOURCEDIR)/%.cpp,$(BUILDDIR)/%.o,$(SOURCES))

EXE_ARD = control.bin
CC_ARD = avr-g++ -mmcu=atmega2560
CFLAGS_ARD  = -ggdb -c -std=c++11 -I$(ARDLIB_INCLUDEDIR)
LDFLAGS_ARD =
SOURCE_FILES_ARD = control main pid io_arduino
SOURCES_ARD = $(patsubst %,$(SOURCEDIR)/%.cpp,$(SOURCE_FILES_ARD))
OBJECTS_ARD = $(patsubst $(SOURCEDIR)/%.cpp,$(BUILDDIR)/%_ard.o,$(SOURCES_ARD))

all: arduino

arduino: dir control.bin

cpu: dir control

dir:
	mkdir -p $(BUILDDIR)

$(EXE): $(OBJECTS)
	$(CC) $^ $(LDFLAGS) -o $@

$(BUILDDIR)/%.o: $(SOURCEDIR)/%.cpp
	$(CC) $(CFLAGS) $< -o $@

$(EXE_ARD): $(OBJECTS_ARD)
	$(CC_ARD) $^ $(LDFLAGS_ARD) -o $@

$(BUILDDIR)/%_ard.o: $(SOURCEDIR)/%.cpp
	$(CC_ARD) $(CFLAGS_ARD) $< -o $@

$(ARDLIB_BUILDDIR)/%.o: $(ARDLIB_SOURCEDIR)/%.cpp
	$(CC_ARD) $(CFLAGS_ARD) $< -o $@

.PHONY: clean

clean:
	rm -f $(BUILDDIR)/*.o
	rm -f $(EXE)
	rm -f $(ARDLIB_BUILDDIR)/*.o
	rm -f $(EXE_ARD)
	rmdir $(BUILDDIR)
	rmdir $(ARDLIB_BUILDDIR)

