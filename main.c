#include "io.h"
#include "control.h"

int main(int argc, char** argv)
{
	if (argc >= 9)
		return control(
			cpu_in(argv[1]), cpu_out(argv[2]),
			sensor_in(argv[3]), sensor_out(argv[4]),
			motor_in(argv[5]), motor_out(argv[6]),
			config_in(argv[7]), config_out(argv[8])
		);
	else return control(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
}

