#ifndef __IO_H__
#define __IO_H__

#include <stdio.h>

FILE* cpu_in(char*);
FILE* cpu_out(char*);
FILE* sensor_in(char*);
FILE* sensor_out(char*);
FILE* motor_in(char*);
FILE* motor_out(char*);
FILE* config_in(char*);
FILE* config_out(char*);

#endif

