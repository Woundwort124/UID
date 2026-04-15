#ifndef __IMU_H__
#define __IMU_H__

#include "debug.h"

/**
 * @brief IMU 寄存器地址映射表
 *
 */
typedef struct __imu_reg_lut__ {
	u8 address;
	u8 ID;	       // 传感器标识
	u8 Acc;	       // 加速度计基址
	u8 Gyro;       // 陀螺仪基址
	u8 Temp;       // 温度
	u8 DataRate;   // 采样率
	u8 Scale_Acc;  // 加速度计量程
	u8 Scale_Gyro; // 陀螺仪量程
} IMU_Reg;

u8 IMU_Init(void); // 初始化IMU

#endif
