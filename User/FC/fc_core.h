#ifndef FC_CORE_H
#define FC_CORE_H

#include "fc_type.h"

void fc_init(const fc_config_t *cfg);
void fc_update(const imu_data_t *imu, const mag_data_t *mag, const cmd_t *cmd, float dt, motor_cmd_t *out);
void fc_get_attidute(attitude_t *att);

#endif // !FC_CORE_H
