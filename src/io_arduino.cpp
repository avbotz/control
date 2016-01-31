#include "io.hpp"

#include <string.h>

#include "Arduino.h"

HardwareSerial& cpu_serial = Serial;
HardwareSerial& ahrs_serial = Serial3;
HardwareSerial& motor_serial = Serial1;

static FILE* cpu_in;
static FILE* cpu_out;
static FILE* ahrs_in;
static FILE* ahrs_out;
static FILE* motor_in;
static FILE* motor_out;

int get_cpu(FILE*);
int put_cpu(char, FILE*);
int get_ahrs(FILE*);
int put_ahrs(char, FILE*);
int get_motor(FILE*);
int put_motor(char, FILE*);

// attempt to copy initialization performed by the Arduino main.cpp
void init_io()
{
#if defined(USBCON)
	USBDevice.attach();
#endif

	cpu_serial.begin(9600);
	ahrs_serial.begin(38400);
	motor_serial.begin(9600);

	cpu_in = fdevopen(NULL, get_cpu);
	cpu_out = fdevopen(put_cpu, NULL);
	ahrs_in = fdevopen(NULL, get_ahrs);
	ahrs_out = fdevopen(put_ahrs, NULL);
	motor_in = fdevopen(NULL, get_motor);
	motor_out = fdevopen(put_motor, NULL);
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

int put_ahrs(char c, FILE* f)
{
	return put(ahrs_serial, c, f);
}

int get_ahrs(FILE* f)
{
	return get(ahrs_serial, f);
}

int put_motor(char c, FILE* f)
{
	return put(motor_serial, c, f);
}

int get_motor(FILE* f)
{
	return get(motor_serial, f);
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

State getState()
{
	State state;
	// read from ahrs here
	
	// read depth from pressure sensor
	state.property[3] = analogRead(12);

	return state;
}

void setMotor(const Motor& motor)
{
	for (uint8_t i = 0; i < numMotors; i++)
		// send to motor controller here
		cprintf("motor; %i: %f\n", i, motor.thrust[i]);
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

