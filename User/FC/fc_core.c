#include "fc_core.h"
#include "madgwick.h"
#include "mixer.h"
#include "pid.h"

static madgwick_t s_mw;
static pid_t s_pid_roll, s_pid_pitch, s_pid_yaw;
static pid_t s_pid_roll_rate, s_pid_pitch_rate, s_pid_yaw_rate;
static attitude_t s_att;

/*******************************************************************************
 * Function Name  : fc_init
 * Description    : 初始化飞控，设置 Madgwick 和 PID 参数
 * Input          : fc_config_t
 * Return         : None
 *******************************************************************************/
void fc_init(const fc_config_t *cfg)
{
	madgwick_init(&s_mw, cfg->madgwick);

	pid_init(&s_pid_roll, cfg->pid_roll_kp, cfg->pid_roll_ki, cfg->pid_roll_kd, 1.0f);
	pid_init(&s_pid_pitch, cfg->pid_pitch_kp, cfg->pid_pitch_ki, cfg->pid_pitch_kd, 1.0f);
	pid_init(&s_pid_yaw, cfg->pid_yaw_kp, cfg->pid_yaw_ki, cfg->pid_yaw_kd, 1.0f);

	pid_init(&s_pid_roll_rate, cfg->pid_rate_kp, cfg->pid_rate_ki, cfg->pid_rate_kd, 1.0f);
	pid_init(&s_pid_pitch_rate, cfg->pid_rate_kp, cfg->pid_rate_ki, cfg->pid_rate_kd, 1.0f);
	pid_init(&s_pid_yaw_rate, cfg->pid_rate_kp, cfg->pid_rate_ki, cfg->pid_rate_kd, 1.0f);
}

void fc_update(const imu_data_t *imu, const mag_data_t *mag, const cmd_t *cmd, float dt, motor_cmd_t *out)
{
	/* 1. 姿态融合 */
	if (mag->updated)
		madgwick_update_9dof(&s_mw, imu->gyro, imu->accel, mag->mag, dt);
	else
		madgwick_update_6dof(&s_mw, imu->gyro, imu->accel, dt);

	madgwick_get_euler(&s_mw, &s_att.roll, &s_att.pitch, &s_att.yaw);
	s_att.q[0] = s_mw.q[0];
	s_att.q[1] = s_mw.q[1];
	s_att.q[2] = s_mw.q[2];
	s_att.q[3] = s_mw.q[3];

	/* 2. 解锁检查 */
	if (!cmd->armed) {
		out->motor[0] = out->motor[1] = out->motor[2] = out->motor[3] = 0;
		pid_reset(&s_pid_roll);
		pid_reset(&s_pid_pitch);
		pid_reset(&s_pid_yaw);
		pid_reset(&s_pid_roll_rate);
		pid_reset(&s_pid_pitch_rate);
		pid_reset(&s_pid_yaw_rate);
		return;
	}

	/* 3. 外环：角度 → 角速度设定点 */
	float roll_rate_sp = pid_update(&s_pid_roll, cmd->roll_sp - s_att.roll, 0.0f, dt);
	float pitch_rate_sp = pid_update(&s_pid_pitch, cmd->pitch_sp - s_att.pitch, 0.0f, dt);
	float yaw_rate_sp = cmd->yaw_rate_sp; /* 偏航直接给角速度设定点 */

	/* 4. 内环：角速度 → 控制量 */
	float roll_out = pid_update(&s_pid_roll_rate, roll_rate_sp - imu->gyro[0], 0.0f, dt);
	float pitch_out = pid_update(&s_pid_pitch_rate, pitch_rate_sp - imu->gyro[1], 0.0f, dt);
	float yaw_out = pid_update(&s_pid_yaw_rate, yaw_rate_sp - imu->gyro[2], 0.0f, dt);

	/* 5. 混控输出 */
	mixer_update(cmd->throttle, roll_out, pitch_out, yaw_out, out);
}

void fc_get_attitude(attitude_t *att)
{
	*att = s_att;
}
