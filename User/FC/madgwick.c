#include "madgwick.h"
#include <math.h>

/*******************************************************************************
 * Function Name  : madgwick_init
 * Description    : 初始化飞控，设置 Madgwick 和 PID 参数
 * Input          : fc_config_t
 * Return         : None
 *******************************************************************************/
void madgwick_init(madgwick_t *mw, float beta)
{
	mw->q[0] = 1.0f;
	mw->q[1] = 0.0f;
	mw->q[2] = 0.0f;
	mw->q[3] = 0.0f;
	mw->beta = beta;
}

static void normalize3(float v[3])
{
	float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (n < 1e-10f)
		return;
	v[0] /= n;
	v[1] /= n;
	v[2] /= n;
}

static void normalize4(float q[4])
{
	float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (n < 1e-10f)
		return;
	q[0] /= n;
	q[1] /= n;
	q[2] /= n;
	q[3] /= n;
}

void madgwick_update_6dof(madgwick_t *mw, const float gyro[3], const float accel[3], float dt)
{
	float q0 = mw->q[0], q1 = mw->q[1], q2 = mw->q[2], q3 = mw->q[3];
	float ax = accel[0], ay = accel[1], az = accel[2];
	float gx = gyro[0], gy = gyro[1], gz = gyro[2];

	float a[3] = {ax, ay, az};
	normalize3(a);
	ax = a[0];
	ay = a[1];
	az = a[2];

	float f0 = 2.0f * (q1 * q3 - q0 * q2) - ax;
	float f1 = 2.0f * (q0 * q1 + q2 * q3) - ay;
	float f2 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

	float j00 = -2.0f * q2, j01 = 2.0f * q3, j02 = -2.0f * q0, j03 = 2.0f * q1;
	float j10 = 2.0f * q1, j11 = 2.0f * q0, j12 = 2.0f * q3, j13 = 2.0f * q2;
	float j20 = 0.0f, j21 = -4.0f * q1, j22 = -4.0f * q2, j23 = 0.0f;

	float s0 = j00 * f0 + j10 * f1 + j20 * f2;
	float s1 = j01 * f0 + j11 * f1 + j21 * f2;
	float s2 = j02 * f0 + j12 * f1 + j22 * f2;
	float s3 = j03 * f0 + j13 * f1 + j23 * f2;

	float sn[4] = {s0, s1, s2, s3};
	normalize4(sn);

	float qdot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - mw->beta * sn[0];
	float qdot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - mw->beta * sn[1];
	float qdot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - mw->beta * sn[2];
	float qdot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - mw->beta * sn[3];

	mw->q[0] = q0 + qdot0 * dt;
	mw->q[1] = q1 + qdot1 * dt;
	mw->q[2] = q2 + qdot2 * dt;
	mw->q[3] = q3 + qdot3 * dt;
	normalize4(mw->q);
}

void madgwick_update_9dof(madgwick_t *mw, const float gyro[3], const float accel[3], const float mag[3], float dt)
{
	float q0 = mw->q[0], q1 = mw->q[1], q2 = mw->q[2], q3 = mw->q[3];
	float ax = accel[0], ay = accel[1], az = accel[2];
	float mx = mag[0], my = mag[1], mz = mag[2];
	float gx = gyro[0], gy = gyro[1], gz = gyro[2];

	float a[3] = {ax, ay, az};
	normalize3(a);
	ax = a[0];
	ay = a[1];
	az = a[2];
	float m[3] = {mx, my, mz};
	normalize3(m);
	mx = m[0];
	my = m[1];
	mz = m[2];

	/* 参考磁场方向 */
	float hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2));
	float hy = 2.0f * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1));
	float bx = sqrtf(hx * hx + hy * hy);
	float bz = 2.0f * (mx * (q1 * q3 - q0 * q2) + my * (q2 * q3 + q0 * q1) + mz * (0.5f - q1 * q1 - q2 * q2));

	float f0 = 2.0f * (q1 * q3 - q0 * q2) - ax;
	float f1 = 2.0f * (q0 * q1 + q2 * q3) - ay;
	float f2 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;
	float f3 = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3) + 2.0f * bz * (q1 * q3 - q0 * q2) - mx;
	float f4 = 2.0f * bx * (q1 * q2 - q0 * q3) + 2.0f * bz * (q0 * q1 + q2 * q3) - my;
	float f5 = 2.0f * bx * (q0 * q2 + q1 * q3) + 2.0f * bz * (0.5f - q1 * q1 - q2 * q2) - mz;
	(void)bz;

	float s0 = -2.0f * q2 * f0 + 2.0f * q1 * f1 + (-2.0f * bz * q2) * f3 + (-2.0f * bx * q3 + 2.0f * bz * q1) * f4 +
		   (2.0f * bx * q2) * f5;
	float s1 = 2.0f * q3 * f0 + 2.0f * q0 * f1 - 4.0f * q1 * f2 + (2.0f * bz * q3) * f3 +
		   (2.0f * bx * q2 + 2.0f * bz * q0) * f4 + (2.0f * bx * q3 - 4.0f * bz * q1) * f5;
	float s2 = -2.0f * q0 * f0 + 2.0f * q3 * f1 - 4.0f * q2 * f2 + (-4.0f * bx * q2 - 2.0f * bz * q0) * f3 +
		   (2.0f * bx * q1 + 2.0f * bz * q3) * f4 + (2.0f * bx * q0 - 4.0f * bz * q2) * f5;
	float s3 = 2.0f * q1 * f0 + 2.0f * q2 * f1 + (-4.0f * bx * q3 + 2.0f * bz * q1) * f3 +
		   (-2.0f * bx * q0 + 2.0f * bz * q2) * f4 + (2.0f * bx * q1) * f5;
	float sn[4] = {s0, s1, s2, s3};
	normalize4(sn);

	float qdot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - mw->beta * sn[0];
	float qdot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - mw->beta * sn[1];
	float qdot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - mw->beta * sn[2];
	float qdot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - mw->beta * sn[3];

	mw->q[0] = q0 + qdot0 * dt;
	mw->q[1] = q1 + qdot1 * dt;
	mw->q[2] = q2 + qdot2 * dt;
	mw->q[3] = q3 + qdot3 * dt;
	normalize4(mw->q);
}

void madgwick_get_euler(const madgwick_t *mw, float *roll, float *pitch, float *yaw)
{
	float q0 = mw->q[0], q1 = mw->q[1], q2 = mw->q[2], q3 = mw->q[3];
	*roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
	*pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
	*yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
