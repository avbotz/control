#include "io.h"

FILE* cpu_in(char* file)
{
	if (file == NULL) return stdin;
	else return fopen(file, "r");
}

FILE* cpu_out(char* file)
{
	if (file == NULL) return stdout;
	else return fopen(file, "w");
}

FILE* sensor_in(char* file)
{
	if (file == NULL) return fopen("sensor_in", "r");
	else return fopen(file, "r");
}

FILE* sensor_out(char* file)
{
	if (file == NULL) return fopen("sensor_out", "w");
	else return fopen(file, "r");
}

FILE* motor_in(char* file)
{
	if (file == NULL) return fopen("motor_in", "r");
	else return fopen(file, "r");
}

FILE* motor_out(char* file)
{
	if (file == NULL) return fopen("motor_out", "w");
	else return fopen(file, "r");
}

FILE* config_in(char* file)
{
	if (file == NULL) return fopen("config_in", "r");
	else return fopen(file, "r");
}

FILE* config_out(char* file)
{
	if (file == NULL) return fopen("config_out", "w");
	else return fopen(file, "r");
}

