#include "control.hpp"

#include <stdio.h>
#include <string.h>

#include "pid.hpp"
#include "io.hpp"

int main()
{
	init_io();

	// read config
	Config config = getConfig();
	// the first (3*numProperties) values are PID gains
	float* gains = config.setting;
	// the next (numProperties*numMotors) map pid results to desired thrust (linear transformation)
	float* thrusterMatrix = config.setting + 3*numProperties;

	// create PID filters
	Pid controllers[numProperties];
	for (uint8_t i = 0; i < numProperties; i++)
		controllers[i] = makePid(gains[3*i], gains[3*i + 1], gains[3*i + 2]);

	// the desired state
	State desired = {{0}};

	// stuff for parsing cpu input
	size_t c_idx = 0;
	size_t cbuffer_size = 256;
	char cbuffer[cbuffer_size];

	while (true)
	{
		// process cpu communication
		int c;
		while ((c = cgetc()) != EOF)
		{
			if (c_idx >= cbuffer_size) c_idx = 0;
			cbuffer[c_idx++] = (char)c;

			// get desired state from cpu
			float x, y, depth, yaw, pitch, roll;
			if (sscanf(cbuffer, " c %f %f %f %f %f %f", &x, &y, &depth, &yaw, &pitch, &roll) == 6)
			{
				desired.property[S_X] = x;
				desired.property[S_Y] = y;
				desired.property[S_DEPTH] = depth;
				desired.property[S_YAW] = yaw;
				desired.property[S_PITCH] = pitch;
				desired.property[S_ROLL] = roll;

				c_idx = 0;
				memset(cbuffer, 0, cbuffer_size);
			}

			// send state to cpu
			char space;
			if (sscanf(cbuffer, " s%c", &space) == 1) // I hope space is whitespace...
			{
				cprintf("s %f %f %f %f %f %f\n",
					desired.property[S_X],
					desired.property[S_Y],
					desired.property[S_DEPTH],
					desired.property[S_YAW],
					desired.property[S_PITCH],
					desired.property[S_ROLL]
				);

				c_idx = 0;
				memset(cbuffer, 0, cbuffer_size);
			}
		}

		// read current variable values and send them to PID
		State state = getState();
		float pidValues[numProperties] = {0};
		for (uint8_t i = 0; i < numProperties; i++)
			pidValues[i] = process(&controllers[i], state.property[i] - desired.property[i]);

		// determine desired thrust levels
		Motor motor;
		for (uint8_t i = 0; i < numMotors; i++)
		{
			float thrust = 0;
			for (uint8_t j = 0; j < numProperties; j++)
				thrust += pidValues[j] * thrusterMatrix[i*numProperties + j];
			thrust = (thrust > 100) ? 100 : (thrust < -100) ? -100 : thrust;
			motor.thrust[i] = thrust;
		}
		setMotor(motor);
	}

	return 0;
}

