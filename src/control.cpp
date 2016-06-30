#include <stdio.h>
#include <string.h>

#include "pid.hpp"
#include "io.hpp"
#include "config.hpp"
#include "rotation.h"

#define PI 3.14159f

// Represents distance traveled during one PID iteration at full power in the
// respective direction
#define UPDATE_COEFF_X (1.f/600.f)
#define UPDATE_COEFF_Y (1.f/600.f)

// How long, in milliseconds, to pause communications with the thrusters when
// starting up and upon being unkilled. This is necessary because the
// communicating with an m5 thruster while it is booting seems to sometimes
// cause it to never begin spinning.
#define PAUSE_TIME 4500UL

int main()
{
	init_io();

	float maxthrust = .8f;

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

	size_t c_idx = 0;
	size_t cbuffer_size = 256;
	char cbuffer[cbuffer_size];

	bool kill_state = false;

	bool paused = false;
	unsigned long pause_until;

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

			// get desired max thrust from cpu
			if (sscanf(cbuffer, " p %f", &maxthrust) == 1)
			{
				c_idx = 0;
				memset(cbuffer, 0, cbuffer_size);
			}

			// get a desired configuration value from cpu
			{
				unsigned int setting;
				float value;
				if (sscanf(cbuffer, " e %u %f", &setting, &value) == 2)
				{
					if (0 <= setting && setting < numSettings)
					{
						config.setting[setting] = value;

						// Change current runtime configuration of pid gain
						if (setting < 3 * NUM_PROPERTIES)
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
					}
					c_idx = 0;
					memset(cbuffer, 0, cbuffer_size);
				}
			}
		}

		// read current variable values and send them to PID
		state = getState(state);

		bool kill_state_prev = kill_state;
		// check kill state
		kill_state = alive();

		if (paused && pause_until <= milliseconds())
		{
			// PAUSE_TIME has been surpassed. Stop pausing.
			paused = false;
		}
		if (kill_state_prev && !kill_state)
		{
			// It's just been killed
			pauseMotorComm();
		}
		if (!kill_state_prev && kill_state)
		{
			// It's just been unkilled. Pause to give thrusters time to start
			// up.
			paused = true;
			pause_until = milliseconds() + PAUSE_TIME;
		}

		// Only run PID when not killed and thrusters have had time to start
		// up. This prevents the issue of thrusters sometimes never starting if
		// they have been communicated with while booting and prevents state
		// values being included into PID while killed.
		if (!paused && kill_state)
		{
			float pidValues[NUM_PROPERTIES] = {0};
			for (uint8_t i = 0; i < NUM_PROPERTIES; i++)
			{
				pidValues[i] = process(&controllers[i], desired.property[i] - state.property[i]);
				// Truncate sum to keep the integral component magnitude from
				// significantly exceeding 1 to prevent extreme underdamping due to
				// the sum building up excessively.
				float i_comp = controllers[i].sum * controllers[i].ki;
				if (i_comp < -1.f)
				{
					controllers[i].sum = -1.f / controllers[i].ki;
				}
				else if (i_comp > 1.f)
				{
					controllers[i].sum = 1.f / controllers[i].ki;
				}
			}

			// Desired x and y values are absolute. Estimate new x and y state
			// based on their pid values. Assumes velocity is linearly proportional
			// to the PID value.
			// PID values are truncated based on assumed range past which thruster
			// power values would be truncated.
			state.property[S_X] +=
				UPDATE_COEFF_X *
				((pidValues[S_X] > 1.f) ? 1.f : (pidValues[S_X] < -1.f) ? -1.f : pidValues[S_X]);
			state.property[S_Y] +=
				UPDATE_COEFF_Y *
				((pidValues[S_Y] > 1.f) ? 1.f : (pidValues[S_Y] < -1.f) ? -1.f : pidValues[S_Y]);

			// Transform the absolute X, Y, and depth values from pid to
			// vectors relative to the sub axes.
			float angles[3];
			for (uint_fast8_t i = 3; i--;)
			{
				angles[i] = 2.f * PI * state.property[S_YAW + i];
			}
			float matrix[3][3];
			rotation(angles, matrix);
			float tmp[3];
			memcpy(tmp, pidValues + S_X, sizeof(tmp));
			for (uint_fast8_t i = 3; i--;)
			{
				pidValues[S_X + i] = 0.f;
				for (uint_fast8_t j = 3; j--;)
				{
					pidValues[S_X + i] += tmp[j] * matrix[i][j];
				}
			}

			// Transform the nautical Yaw, Pitch, and Roll values from pid to
			// vectors relative to the sub
			rotation_angles(angles, matrix);
			memcpy(tmp, pidValues + S_YAW, sizeof(tmp));
			for (uint_fast8_t i = 3; i--;)
			{
				pidValues[S_YAW + i] = 0.f;
				for (uint_fast8_t j = 3; j--;)
				{
					pidValues[S_YAW + i] += tmp[j] * matrix[i][j];
				}
			}

			// determine desired thrust levels
			Motor motor;
			for (uint8_t i = 0; i < numMotors; i++)
			{
				float thrust = 0;
				for (uint8_t j = 0; j < NUM_PROPERTIES; j++)
					thrust += pidValues[j] * thrusterMatrix[i*NUM_PROPERTIES + j];
				thrust = (thrust > maxthrust) ? maxthrust : (thrust < -maxthrust) ? -maxthrust : thrust;
				motor.thrust[i] = thrust;
			}
			setMotor(motor);
		}
	}

	return 0;
}
