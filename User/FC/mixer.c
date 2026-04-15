#include "mixer.h"

static uint16_t clamp_motor(float v)
{
	if (v < 0.0f)
		return 0;
	if (v > 1000.0f)
		return 1000;
	return (uint16_t)v;
}

void mixer_update(float throttle, float roll, float pitch, float yaw, motor_cmd_t *out)
{
	float t = throttle * 1000.0f;
	float r = roll * 500.0f;
	float p = pitch * 500.0f;
	float y = yaw * 500.0f;

	out->motor[0] = clamp_motor(t - r + p + y); /* M1 左前 */
	out->motor[1] = clamp_motor(t + r + p - y); /* M2 右前 */
	out->motor[2] = clamp_motor(t + r - p + y); /* M3 右后 */
	out->motor[3] = clamp_motor(t - r - p - y); /* M4 左后 */
}