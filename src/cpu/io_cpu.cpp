#include "io.hpp"

#include <stdarg.h>
#include <stdio.h>

#include <time.h>
#include <stdlib.h>

bool alive()
{
	return true;
}

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

int cgetc()
{
	return fgetc(cpu_in);
}

State getState(const State &state)
{
	State newstate = state;
	for (uint8_t i = 0; i < NUM_PROPERTIES; i++)
		fscanf(state_in, " %f", &newstate.property[i]);
	return newstate;
}

void setMotor(const Motor& motor)
{
	for (uint8_t i = 0; i < numMotors; i++)
	{
		fprintf(motor_out, " %f", motor.thrust[i]);
	}
	fprintf(motor_out, "\n");
	fflush(motor_out);
}

Config getConfig()
{
	Config config;
	for (uint8_t i = 0; i < numFlags; i++)
	{
		int flag;
		fscanf(config_in, " %i", &flag);
		config.flag[i] = flag > 0;
	}
	for (uint8_t i = 0; i < numSettings; i++)
		fscanf(config_in, " %f", &config.setting[i]);
	return config;
}

void setConfig(const Config& config)
{
	for (uint8_t i = 0; i < numFlags; i++)
		fprintf(config_out, " %i", config.flag[i] ? 1 : 0);
	for (uint8_t i = 0; i < numSettings; i++)
		fprintf(config_out, " %f", config.setting[i]);
	fprintf(config_out, "\n");
}

void activateRelay(enum Relay)
{
	// Do nothing for now
	return;
}

void deactivateRelay(enum Relay)
{
	// Do nothing for now
	return;
}

unsigned long milliseconds()
{
	struct timespec ts;
	if (clock_gettime(CLOCK_MONOTONIC, &ts))
	{
		perror("Failed getting time.");
		exit(EXIT_FAILURE);
	}
	// convert timespec to milliseconds (truncated)
	return (unsigned long)ts.tv_sec * 1000UL + ts.tv_nsec/1000000UL;
}

void pauseMotorComm()
{
	// Do nothing (we do not communicate with actual thrusters in cpu)
	return;
}

void init_io()
{
	cpu_in = stdin;
	cpu_out = stdout;
	state_in = fopen("state_in", "r");
	motor_out = fopen("motor_out", "w");
	config_in = fopen("config_in", "r");
	config_out = fopen("config_out", "w");
}

