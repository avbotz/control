/**
 * This file is just glue code between the C interfaces of the ahrs and motor
 * code and the cpp interfaces the PID code uses.
 */
#include "io.hpp"
#include "io_kill.hpp"
#include "io_ahrs.h"
#include "ahrs.h"
#include "io_depth.hpp"
#include "io_cpu.hpp"
#include "io_m5.h"
#include "m5.h"
#include "macrodef.h"

bool alive()
{
	return io_kill();
}

State getState()
{
	ahrs_att_update();
	State curr;
	for (uint_fast8_t dir = NUM_ATT_AXES; dir--;)
	{
		// Scale values to range [-.5, .5]
		curr.property[dir] =
			(ahrs_att((enum att_axis)dir) - ahrs_range[dir][COMPONENT_MIN]) /
			(ahrs_range[dir][COMPONENT_MAX] - ahrs_range[dir][COMPONENT_MIN])
			- .5;
	}
	curr.property[3] = io_depth();
	return curr;
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

void init_io()
{
	io_kill_init();
	io_cpu_init();
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
	return;
}
