#include "control.hpp"

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
	
	while (true)
	{
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

