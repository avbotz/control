#include "io.hpp"

#include <string.h>

#include "Arduino.h"

HardwareSerial& cpu_serial = Serial;

static FILE* cpu_in;
static FILE* cpu_out;

int get_cpu(FILE*);
int put_cpu(char, FILE*);

// attempt to copy initialization performed by the Arduino main.cpp
void io_cpu_init()
{
#if defined(USBCON)
	USBDevice.attach();
#endif

	cpu_serial.begin(9600);

	cpu_in = fdevopen(NULL, get_cpu);
	cpu_out = fdevopen(put_cpu, NULL);
}

int put(HardwareSerial&, char, FILE*);
int get(HardwareSerial&, FILE*);

int put_cpu(char c, FILE* f)
{
	return put(cpu_serial, c, f);
}

int get_cpu(FILE* f)
{
	return get(cpu_serial, f);
}

int put(HardwareSerial& serial, char c, FILE* f)
{
	return serial.write(c) == 1 ? 0 : 1;
}

int get(HardwareSerial& serial, FILE* f)
{
	int c = serial.read();
	if (c == -1) return _FDEV_EOF;
	else return c;
}

#include <stdarg.h>

void cscanf(const char* format, ...)
{
	va_list args;
	va_start(args, format);
	vfscanf(cpu_in, format, args);
	va_end(args);
}

void cprintf(const char* format, ...)
{
	va_list args;
	va_start(args, format);
	vfprintf(cpu_out, format, args);
	va_end(args);
}

#include "EEPROM.h"

Config getConfig()
{
	Config config;
	for (uint8_t i = 0; i < numFlags; i++)
	{
		int addr = i * sizeof(bool);
		for (int j = 0; j < sizeof(bool); j++)
			*((char*)&config.flag[i] + j) = EEPROM.read(addr + j);
	}
	for (uint8_t i = 0; i < numSettings; i++)
	{
		int addr = numFlags * sizeof(bool) + i * sizeof(float);
		for (int j = 0; j < sizeof(float); j++)
			*((char*)&config.setting[i] + j) = EEPROM.read(addr + j);
	}
	return config;
}

void setConfig(const Config& config)
{
	for (uint8_t i = 0; i < numFlags; i++)
	{
		int addr = i * sizeof(bool);
		for (int j = 0; j < sizeof(bool); j++)
			EEPROM.write(addr + j, *((char*)&config.flag[i] + j));
	}
	for (uint8_t i = 0; i < numSettings; i++)
	{
		int addr = numFlags * sizeof(bool) + i * sizeof(float);
		for (int j = 0; j < sizeof(float); j++)
			EEPROM.write(addr + j, *((char*)&config.setting[i] + j));
	}
}

