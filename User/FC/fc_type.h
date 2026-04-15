#ifndef __FC_TYPE_H__
#define __FC_TYPE_H__

#include <stdint.h>

typedef struct {	// IMU 原始数据
	float gyro[3];	// rad/s, [roll, pitch, yaw]
	float accel[3]; // m/s^2, [x, y, z]
	uint32_t timestamp_us;
} imu_data_t;

typedef struct {	 // 磁力计数据
	float mag[3];	 // uT, [x, y, z]
	uint8_t updated; // 1 = 本帧有新数据
} mag_data_t;

typedef struct {	   // 上位机指令
	float roll_sp;	   // rad
	float pitch_sp;	   // rad
	float yaw_rate_sp; // rad/s
	float throttle;	   // 0.0 ~ 1.0
	uint8_t armed;	   //  0 = 锁定, 1 = 解锁
} cmd_t;

typedef struct {		// 姿态输出
	float roll, pitch, yaw; // 欧拉角, rad
	float q[4];		// 四元数 [w, x, y, z]
} attitude_t;

typedef struct {	   // 电机指令输出
	uint16_t motor[4]; // 归一化值 0 ~ 1000, M1 = 左前, M2 = 右前, M3 = 右后, M4 = 左后
} motor_cmd_t;

typedef struct {	// 配置参数
	float madgwick; // 融合增益, 默认 0.1
	float pid_roll_kp, pid_roll_ki, pid_roll_kd;
	float pid_pitch_kp, pid_pitch_ki, pid_pitch_kd;
	float pid_yaw_kp, pid_yaw_ki, pid_yaw_kd;
	float pid_rate_kp, pid_rate_ki, pid_rate_kd; // 内环三轴共用
} fc_config_t;

#endif // !__FC_TYPE_H__
