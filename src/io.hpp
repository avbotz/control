#ifndef __IO_HPP__
#define __IO_HPP__

#include "global.hpp"

bool alive();

void cscanf(const char*, ...);
void cprintf(const char*, ...);
int cgetc();

void setLevelRef(uint_fast8_t dir, float offset);

State getState(const State&);

void setMotor(const Motor&);

void activateRelay(enum Relay);

void deactivateRelay(enum Relay);

void pauseMotorComm();

unsigned long milliseconds();

void init_io();

#endif
