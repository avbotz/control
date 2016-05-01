#include <stdarg.h>
#include <stdio.h>

#include "io.hpp"

static FILE* cpu_in;
static FILE* cpu_out;
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

void io_cpu_init()
{
	cpu_in = stdin;  //fopen("cpu_in", "r");
	cpu_out = stdout;//fopen("cpu_out", "w");
	config_in = fopen("config_in", "r");
	config_out = fopen("config_out", "w");
}
