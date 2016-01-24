#include "control.h"

#include "pid.h"

int control(FILE* cpu_in, FILE* cpu_out, FILE* sensor_in, FILE* sensor_out, FILE* motor_in, FILE* motor_out, FILE* config_in, FILE* config_out)
{
	const int numControllers = 4;
	const int numMotors = 8;

	// read PID filter gains from configuration
	float gains[numControllers * 3]; // yp, yi, yd, pp, pi, ...
	for (uint8_t i = 0; i < numControllers * 3; i++)
		fscanf(config_in, " %f", &gains[i]);

	// create PID filters
	Pid controllers[numControllers];
	for (uint8_t i = 0; i < numControllers; i++)
		controllers[i] = makePid(gains[3*i], gains[3*i + 1], gains[3*i + 2]);

	// linear transformation from pid results and desired thrust
	float thrusterMatrix[numMotors * numControllers];
	for (uint8_t i = 0; i < numMotors; i++)
		for (uint8_t j = 0; j < numControllers; j++)
			fscanf(config_in, " %f", &thrusterMatrix[i*numControllers + j]);

	// the desired state
	float desired[numControllers] = {0};
	
	// get them from config for now
	for (uint8_t i = 0; i < numControllers; i++)
		fscanf(config_in, " %f", &desired[i]);
	
	while (1)
	{
		// read current variable values and send them to PID
		float pidValues[numControllers] = {0};
		for (uint8_t i = 0; i < numControllers; i++)
		{
			float value;
			fscanf(sensor_in, " %f", &value);
			pidValues[i] = process(&controllers[i], value - desired[i]);
		}

		// determine desired thrust levels
		for (uint8_t i = 0; i < numMotors; i++)
		{
			float thrust = 0;
			for (uint8_t j = 0; j < numControllers; j++)
				thrust += pidValues[j] * thrusterMatrix[i*numControllers + j];
			thrust = (thrust > 100) ? 100 : (thrust < -100) ? -100 : thrust;
			fprintf(motor_out, "%i ", (int) thrust);
			fflush(motor_out);
		}
	}

	return 0;
}

