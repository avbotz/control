#include "io_kill.hpp"

#include "io.hpp"

#include "Arduino.h"

#define KILL_PIN 30

void io_kill_init()
{
	pinMode(KILL_PIN, INPUT);
}

bool io_kill()
{
	return digitalRead(KILL_PIN) ? false : true;
}

