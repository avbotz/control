#include "pid.hpp"

Pid makePid(float kp, float ki, float kd)
{
	Pid pid = {kp, ki, kd, 0, 0};
	return pid;
}

float process(Pid* pid, float error)
{
	pid->sum += error;
	float diff = error - pid->prev;
	pid->prev = error;
	return pid->kp * error + pid->ki * pid->sum + pid->kd * diff;
}

