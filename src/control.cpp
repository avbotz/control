
#include <stdio.h>
#include <string.h>

#include "pid.hpp"
#include "io.hpp"

// Represents distance traveled during one PID iteration at full power in the
// respective direction
#define UPDATE_COEFF_X .01f
#define UPDATE_COEFF_Y .01f

int main()
{
	init_io();

	// read config
	Config config = getConfig();
	// the first (3*NUM_PROPERTIES) values are PID gains
	float* gains = config.setting;
	// the next (NUM_PROPERTIES*numMotors) map pid results to desired thrust (linear transformation)
	float* thrusterMatrix = config.setting + 3*NUM_PROPERTIES;

	// create PID filters
	Pid controllers[NUM_PROPERTIES];
	for (uint8_t i = 0; i < NUM_PROPERTIES; i++)
		controllers[i] = makePid(gains[3*i], gains[3*i + 1], gains[3*i + 2]);

	// the desired state
	State desired = {{0}};
	State state = {{0}};

	// stuff for parsing cpu input
	size_t c_idx = 0;
	size_t cbuffer_size = 256;
	char cbuffer[cbuffer_size];

	bool kill_state = alive();

	while (true)
	{
		// process cpu communication
		int c;
		while ((c = cgetc()) != EOF)
		{
			if (c_idx >= cbuffer_size) c_idx = 0;
			cbuffer[c_idx++] = (char)c;

			// used for scanf error checking
			char space;

			// return kill state
			if (sscanf(cbuffer, " a%c", &space) == 1)
			{
				cprintf("%i\n", kill_state ? 1 : 0);

				c_idx = 0;
				memset(cbuffer, 0, cbuffer_size);
			}

			// get desired state from cpu
			float x, y, depth, yaw, pitch, roll;
			if (sscanf(cbuffer, " s s %f %f %f %f %f %f", &x, &y, &depth, &yaw, &pitch, &roll) == 6)
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
			if (sscanf(cbuffer, " c%c", &space) == 1)
			{
				cprintf("s %f %f %f %f %f %f\n",
					state.property[S_X],
					state.property[S_Y],
					state.property[S_DEPTH],
					state.property[S_YAW],
					state.property[S_PITCH],
					state.property[S_ROLL]
				);

				c_idx = 0;
				memset(cbuffer, 0, cbuffer_size);
			}

			// get a desired configuration value from cpu
			{
				size_t setting;
				float value;
				if (sscanf(cbuffer, " e %zu %f", &setting, &value) == 2)
				{
					if (0 <= setting && setting < numSettings)
					{
						Config tmp = getConfig();
						tmp.setting[setting] = value;
						// change persistently stored setting value (ie EEPROM
						// on avr).
						setConfig(tmp);

						// Change current runtime configuration
						if (setting < 3*NUM_PROPERTIES)
						{
							// set a gain value
							switch (setting % 3)
							{
								case 0:
								{
									 controllers[setting / 3].kp = value;
									 break;
								}
								case 1:
								{
									controllers[setting / 3].ki = value;
									break;
								}
								case 2:
								{
									controllers[setting / 3].kd = value;
									break;
								}
							}
						}
						else
						{
							// set a thruster matrix value
							config.setting[setting] = value;
						}
					}
					 c_idx = 0;
					 memset(cbuffer, 0, cbuffer_size);
				}
			}
		}

		// check kill state
		kill_state = alive();

		// read current variable values and send them to PID
		state = getState(state);
		float pidValues[NUM_PROPERTIES] = {0};
		for (uint8_t i = 0; i < NUM_PROPERTIES; i++)
			pidValues[i] = process(&controllers[i], state.property[i] - desired.property[i]);

		// Desired x and y values are relative. Estimate new x and y state
		// based on their pid values. Assumes velocity is linearly proportional
		// to the PID value.
		// PID values are truncated based on assumed range past which thruster
		// power values would be truncated.
		desired.property[S_X] -=
			UPDATE_COEFF_X *
			((pidValues[S_X] > 1.f) ? 1.f : (pidValues[S_X] < -1.f) ? -1.f : pidValues[S_X]);
		desired.property[S_Y] -=
			UPDATE_COEFF_Y *
			((pidValues[S_Y] > 1.f) ? 1.f : (pidValues[S_Y] < -1.f) ? -1.f : pidValues[S_Y]);

		// determine desired thrust levels
		Motor motor;
		for (uint8_t i = 0; i < numMotors; i++)
		{
			float thrust = 0;
			for (uint8_t j = 0; j < NUM_PROPERTIES; j++)
				thrust += pidValues[j] * thrusterMatrix[i*NUM_PROPERTIES + j];
			thrust = (thrust > 1) ? 1 : (thrust < -1) ? -1 : thrust;
			motor.thrust[i] = thrust;
		}
		setMotor(motor);
	}

	return 0;
}
