#ifndef __IO_HPP__
#define __IO_HPP__

#include "global.hpp"

void cscanf(const char*, ...);
void cprintf(const char*, ...);

State getState();

void setMotor(const Motor&);

Config getConfig();
void setConfig(const Config&);

void init_io();

#endif

