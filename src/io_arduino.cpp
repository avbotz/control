#include "io.h"

int put_cpu(char, FILE*);
int get_cpu(FILE*);
int put_sensor(char, FILE*);
int get_sensor(FILE*);
int put_motor(char, FILE*);
int get_motor(FILE*);
int put_config(char, FILE*);
int get_config(FILE*);

/*
FILE cpu_in_file = FDEV_SETUP_STREAM(NULL, get_cpu, _FDEV_SETUP_READ);
FILE cpu_out_file = FDEV_SETUP_STREAM(put_cpu, NULL, _FDEV_SETUP_WRITE);
FILE sensor_in_file = FDEV_SETUP_STREAM(NULL, get_sensor, _FDEV_SETUP_READ);
FILE sensor_out_file = FDEV_SETUP_STREAM(put_sensor, NULL, _FDEV_SETUP_WRITE);
FILE config_in_file = FDEV_SETUP_STREAM(NULL, get_config, _FDEV_SETUP_READ);
FILE config_out_file = FDEV_SETUP_STREAM(put_config, NULL, _FDEV_SETUP_WRITE);
FILE motor_in_file = FDEV_SETUP_STREAM(NULL, get_motor, _FDEV_SETUP_READ);
FILE motor_out_file = FDEV_SETUP_STREAM(put_motor, NULL, _FDEV_SETUP_WRITE);
*/

#include "Arduino.h"

FILE* cpu_in(char* file)
{
	if (!Serial) Serial.begin(9600);
//	return &cpu_in_file;
	return fdevopen(NULL, get_cpu);
}

FILE* cpu_out(char* file)
{
	if (!Serial) Serial.begin(9600);
//	return &cpu_out_file;
	return fdevopen(put_cpu, NULL);
}

FILE* sensor_in(char* file)
{
	if (!Serial3) Serial3.begin(38400);
//	return &sensor_in_file;
	return fdevopen(NULL, get_sensor);
}

FILE* sensor_out(char* file)
{
	if (!Serial3) Serial3.begin(38400);
//	return &sensor_out_file;
	return fdevopen(put_sensor, NULL);
}

FILE* motor_in(char* file)
{
//	return &motor_in_file;
	return fdevopen(NULL, get_cpu);
}

FILE* motor_out(char* file)
{
//	return &motor_out_file;
	return fdevopen(put_sensor, NULL);
}

FILE* config_in(char* file)
{
//	return &config_in_file;
	return fdevopen(NULL, get_cpu);
}

FILE* config_out(char* file)
{
//	return &config_out_file;
	return fdevopen(put_sensor, NULL);
}

int put_cpu(char c, FILE* f)
{
	return Serial.write(c) == 1 ? 0 : 1;
}

int get_cpu(FILE* f)
{
	int c = Serial.read();
	if (c == EOF) return _FDEV_EOF;
	else return c;
}

int put_sensor(char c, FILE* f)
{
	return Serial3.write(c) == 1 ? 0 : 1;
}

struct Frame
{
	float yaw;
	float pitch;
	float roll;
	int depth;

	bool ahrs_complete;
	bool pressure_sensor_complete;
};

struct Frame frame = {0, 0, 0, 0, 0, 0};

char ahrs_buffer[256];
int ahrs_index = 0;
char sensor_buffer[256];
int sensor_index = 0;

bool complete(char* buffer, size_t index)
{
	return true;
}

void ahrs_read(char* buffer, float* yaw, float* pitch, float* roll)
{
}

int get_sensor(FILE* f)
{
	// check ahrs
	while (Serial3.available())
	{
		ahrs_buffer[ahrs_index++] = Serial3.read();

		if (complete(ahrs_buffer, ahrs_index))
		{ // assume we won't get more than 1 complete frame between calls to get_sensor
			ahrs_read(ahrs_buffer, &frame.yaw, &frame.pitch, &frame.roll);
			frame.ahrs_complete = 1;
			ahrs_index = 0;
		}
	}

	// check pressure sensor
	frame.depth = analogRead(12);
	frame.pressure_sensor_complete = 1;

	// assume we aren't receiving frames faster than we can process them
	if (frame.ahrs_complete && frame.pressure_sensor_complete)
		sprintf(sensor_buffer, "%f %f %f %i\n", frame.yaw, frame.pitch, frame.roll, frame.depth);

	// read next byte
	char c = sensor_buffer[sensor_index++];
	if (c == 0)
	{
		sensor_buffer[sensor_index = 0] = 0;
		return _FDEV_EOF;
	}
	else return c;
}

int motorID = -1;

int put_motor(char c, FILE* f)
{
	if (motorID == -1) motorID = c - '0';
	else analogWrite(motorID, c);
	return 0;
}

int get_motor(FILE* f)
{
	return 0;
}

#include "EEPROM.h"

#define EEPROM_SIZE 4096

int eeprom_read_index = 0;
int eeprom_write_index = 0;

int put_config(char c, FILE* f)
{
	EEPROM.write(eeprom_write_index++, c);
	if (eeprom_write_index >= EEPROM_SIZE)
		eeprom_write_index -= EEPROM_SIZE;
	return 0;
}

int get_config(FILE* f)
{
	if (eeprom_read_index >= EEPROM_SIZE) return _FDEV_EOF;
	else return EEPROM.read(eeprom_read_index++);
}

