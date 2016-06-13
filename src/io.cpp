/**
 * This file is just glue code between the C interfaces of the ahrs and motor
 * code and the cpp interfaces the PID code uses.
 */
#include <math.h>

#include "io.hpp"
#include "io_kill.hpp"
#include "io_ahrs.h"
#include "ahrs.h"
#include "io_depth.hpp"
#include "io_cpu.hpp"
#include "io_m5.h"
#include "m5.h"
#include "io_relay.h"
#include "macrodef.h"

bool alive()
{
	return io_kill();
}

// State.property[S_YAW, S_PITCH, S_ROLL] are non-modular angle values with
// delta physical degrees == delta 1.0 of the value. eg if S_YAW == 0 and there
// is a rotation of 720 degrees, S_YAW will become 1.5 rather than .5.
State getState(const State &current)
{
	ahrs_att_update();
	State newstate = current;
	// order of enum must be S_YAW, S_PITCH, S_ROLL
	for (uint_fast8_t dir = S_YAW; dir <= S_ROLL; ++dir)
	{
		// Scale values to range [-.5, .5]
		newstate.property[dir] = truncf(newstate.property[dir]) +
			(ahrs_att((enum att_axis)(dir - S_YAW)) - ahrs_range[dir - S_YAW][COMPONENT_MIN]) /
			(ahrs_range[dir - S_YAW][COMPONENT_MAX] - ahrs_range[dir - S_YAW][COMPONENT_MIN])
			- .5f;

		// Heuristic to allow non-modular angles. If the angle has changed by
		// more than .5, we assume that is due to angle overflow because it is
		// not likely to rotate that much between calls.
		if (newstate.property[dir] - current.property[dir] < -.5f)
		{
			newstate.property[dir] += 1.f;
		}
		else if (newstate.property[dir] - current.property[dir] > .5f)
		{
			newstate.property[dir] -= 1.f;
		}
	}
	newstate.property[S_DEPTH] = io_depth();
	return newstate;
}

// The actual power values transmitted to the thrusters are truncated to the
// range [-1, 1].
void setMotor(const Motor& motor)
{
	for (uint_fast8_t t = numMotors; t--;)
	{
		// Assumes VERT_FR is the thruster with the lowest Motor ID and all of
		// the following motor IDs are consecutive. This is done so we can skip
		// beginning Motor IDs (ie Motor ID 0 since it caused a bug with our
		// thrusters).
		m5_power((enum thruster)(t + VERT_FR),
				TRUNC(-1.f, motor.thrust[t], 1.f));
	}
	m5_power_offer_resume();

	for (uint8_t i = 0; i < numMotors; i++)
	{
//		cprintf("motor; %i: %f\n", i, TRUNC(-1.f, motor.thrust[i], 1.f));
	}
}

static void setpowers(float vals[NUM_THRUSTERS])
{
	for (uint_fast8_t t = NUM_THRUSTERS; t--;)
	{
		// zero powers
		m5_power((enum thruster)t, vals[t]);
	}
}

void activateRelay(enum Relay r)
{
	io_relay_on((relay)r);
}

void deactivateRelay(enum Relay r)
{
	io_relay_off((relay)r);
}

void init_io()
{
	io_cpu_init();
	io_kill_init();
	io_depth_init("depth_in");
	io_m5_init("/dev/ttyUSB1");
	float powers[NUM_THRUSTERS] = {0.f};
	setpowers(powers); // zero powers
	io_m5_trans_set(m5_power_trans); // Start transmitting data asynchcronously
	io_ahrs_init("/dev/ttyUSB0");
	// May not be strictly necessary, since the data components can be saved to
	// non-volatile memory.
	ahrs_set_datacomp();
	ahrs_cont_start();
	io_ahrs_recv_start(ahrs_att_recv); // Start receiving data asynchronously
	io_relay_init();
	return;
}
