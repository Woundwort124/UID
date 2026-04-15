#ifndef MIXER_H
#define MIXER_H

#include "fc_type.h"

/*
 * X型四旋翼混控
 * throttle: 0.0~1.0
 * roll/pitch/yaw: 归一化控制量，范围 -1.0~1.0
 * out->motor[]: 0~1000
 */
void mixer_update(float throttle, float roll, float pitch, float yaw, motor_cmd_t *out);

#endif /* MIXER_H */ 