#ifndef PID_H
#define PID_H

typedef struct {
	float kp, ki, kd;
	float imax; // 积分限幅
	float integral;
	float prev_error;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float imax);
float pid_update(pid_t *pid, float error, float derivative, float dt);
void pid_reset(pid_t *pid);

#endif /* PID_H */
