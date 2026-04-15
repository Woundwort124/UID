#include "pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float imax)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->imax = imax;
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
}

float pid_update(pid_t *pid, float error, float derivative, float dt)
{
	pid->integral += error * dt;
	if (pid->integral > pid->imax)
		pid->integral = pid->imax;
	if (pid->integral < -pid->imax)
		pid->integral = -pid->imax;

	return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

void pid_reset(pid_t *pid)
{
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
}
