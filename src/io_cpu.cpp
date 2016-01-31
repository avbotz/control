#include "io.hpp"

#include <stdarg.h>
#include <stdio.h>

static FILE* cpu_in;
static FILE* cpu_out;
static FILE* state_in;
static FILE* motor_out;
static FILE* config_in;
static FILE* config_out;

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
	for (uint8_t i = 0; i < numProperties; i++)
		fscanf(state_in, "%f", &state.property[i]);
	return state;
}

void setMotor(const Motor& motor)
{
	for (uint8_t i = 0; i < numMotors; i++)
		fprintf(motor_out, "%f ", motor.thrust[i]);
	fprintf(motor_out, "\n");
}

Config getConfig()
{
	Config config;
	for (uint8_t i = 0; i < numFlags; i++)
	{
		int flag;
		fscanf(config_in, "%i ", &flag);
		config.flag[i] = flag > 0;
	}
	for (uint8_t i = 0; i < numSettings; i++)
		fscanf(config_in, "%f ", &config.setting[i]);
	return config;
}

void setConfig(const Config& config)
{
	for (uint8_t i = 0; i < numFlags; i++)
		fprintf(config_out, "%i ", config.flag[i] ? 1 : 0);
	for (uint8_t i = 0; i < numSettings; i++)
		fprintf(config_out, "%f ", config.setting[i]);
	fprintf(config_out, "\n");
}

void init_io()
{
	cpu_in = fopen("cpu_in", "r");
	cpu_out = fopen("cpu_out", "w");
	state_in = fopen("state_in", "r");
	motor_out = fopen("motor_out", "w");
	config_in = fopen("config_in", "r");
	config_out = fopen("config_out", "w");
}

