#include <stdarg.h>
#include <stdio.h>

#include "io.hpp"

static FILE* cpu_in;
static FILE* cpu_out;

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

void io_cpu_init()
{
	cpu_in = stdin;  //fopen("cpu_in", "r");
	cpu_out = stdout;//fopen("cpu_out", "w");
}
