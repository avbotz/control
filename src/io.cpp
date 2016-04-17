/**
 * This file is just glue code between the C interfaces of the ahrs and motor
 * code and the cpp interfaces the PID code uses.
 */
#include "io.hpp"
#include "io_ahrs.h"
#include "ahrs.h"
#include "io_depth.hpp"
#include "io_cpu.hpp"


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

void setMotor(const Motor& motor)
{
	// TODO
	for (uint8_t i = 0; i < numMotors; i++)
		// send to motor controller here
		cprintf("motor; %i: %f\n", i, motor.thrust[i]);
}

void init_io()
{
	io_cpu_init();
	io_depth_init("depth_in");
	io_ahrs_init("/dev/ttyUSB0");
	// May not be strictly necessary, since the data components can be saved to
	// non-volatile memory.
	ahrs_set_datacomp();
	ahrs_cont_start();
	io_ahrs_recv_start(ahrs_att_recv); // Start receiving data asynchronously
	return;
}
