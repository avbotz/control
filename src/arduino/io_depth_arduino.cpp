#include "io_depth.hpp"
#include "Arduino.h"

#define NPIN 12

void io_depth_init(char const *path)
{
	(void)path;
}

float io_depth()
{
	return analogRead(NPIN);
}
