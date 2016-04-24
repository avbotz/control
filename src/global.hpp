#ifndef __GLOBAL_HPP__
#define __GLOBAL_HPP__

#include <stdint.h>

static const uint8_t numProperties = 4;

struct State
{
	float property[numProperties];
};

enum
{
	S_X,
	S_Y,
	S_DEPTH,
	S_YAW,
	S_PITCH,
	S_ROLL,
};

static const uint8_t numMotors = 8;

struct Motor
{
	float thrust[numMotors];
};

static const uint8_t numSettings = 44;
static const uint8_t numFlags = 0;

struct Config
{
	bool flag[numFlags];
	float setting[numSettings];
};

#endif

