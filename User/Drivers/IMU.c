#include "IMU.h"
#include "IIC.h"
#include "debug.h"
#include "qmi8658.h"


const IMU_Reg Reg_QMI8658 = {
	.address = QMI8658_SLAVE_ADDR_H,       // I2C 地址
	.ID = Qmi8658Register_WhoAmI,	       // 传感器标识
	.Acc = Qmi8658Register_Ax_L,	       // 加速度计基址
	.Gyro = Qmi8658Register_Gx_L,	       // 陀螺仪基址
	.Temp = Qmi8658Register_Tempearture_L, // 温度
	.DataRate = 0,			       // 采样率
	.Scale_Acc = 0,			       // 加速度计量程
	.Scale_Gyro = 0			       // 陀螺仪量程
};

// 初始化 IMU
// 返回值:0,成功
//     其他,错误代码
u8 IMU_Init(void)
{
	return 0;
}
