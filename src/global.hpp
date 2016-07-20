#ifndef __GLOBAL_HPP__
#define __GLOBAL_HPP__

#include <stdint.h>


enum Relay
{
	R_DROPPER,
	R_GRABBER_L,
	R_GRABBER_R,
	R_TORPEDO_L,
	R_TORPEDO_R,
	NUM_RELAYS
};

enum
{
	S_X,
	S_Y,
	S_DEPTH,
	S_YAW,
	S_PITCH,
	S_ROLL,
	NUM_PROPERTIES
};

struct State
{
	float property[NUM_PROPERTIES];
};

static const uint8_t numMotors = 8;

struct Motor
{
	float thrust[numMotors];
};

static const uint8_t numSettings = 66;
static const uint8_t numFlags = 0;

struct Config
{
	bool flag[numFlags];
	float setting[numSettings];
};

#endif

