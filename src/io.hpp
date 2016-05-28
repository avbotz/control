#ifndef __IO_HPP__
#define __IO_HPP__

#include "global.hpp"

bool alive();

void cscanf(const char*, ...);
void cprintf(const char*, ...);
int cgetc();

State getState();

void setMotor(const Motor&);

void activateRelay(enum Relay);

void deactivateRelay(enum Relay);

Config getConfig();
void setConfig(const Config&);

void init_io();

#endif
