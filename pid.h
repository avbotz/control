#ifndef __PID_H__
#define __PID_H__

#include <stdio.h>
#include <stdint.h>

typedef struct
{
	float kp;
	float ki;
	float kd;
	float sum;
	float prev;
} Pid;

Pid makePid(float kp, float ki, float kd);

float process(Pid* pid, float error);

#endif

