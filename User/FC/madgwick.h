#ifndef MADWICK_H
#define MADWICK_H

typedef struct {
	float q[4]; // 四元数 [w, x, y, z]
	float beta; // 融合增益
} madgwick_t;

void madgwick_init(madgwick_t *mw, float beta);

// 六轴融合 (无磁力计)
void madgwick_update_6dof(madgwick_t *mw, const float gyro[3], const float accel[3], float dt);

// 九轴融合
void madgwick_update_9dof(madgwick_t *mw, const float gyro[3], const float accel[3], const float mag[3], float dt);

// 四元数转欧拉角, 单位 rad
void madgwick_get_euler(const madgwick_t *mv, float *roll, float *pitch, float *yaw);

#endif // !MADWICK_H
