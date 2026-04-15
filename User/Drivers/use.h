/*
 * use.h
 *
 *  Created on: 2026年4月13日
 *      Author: 213516
 */

#ifndef USER_USE_H_
#define USER_USE_H_

#include "AHT_10.h"
#include "AP3216C.h"
#include "FreeRTOS.h"
#include "IIC.h"
#include "MPU6050.h"
#include "QMI8658.h"
#include "debug.h"
#include "fc_core.h"
#include "lcd.h"
#include "queue.h"

#include "semphr.h"

void init1(void);
void get1_task(void *pvParameters);
void vTask_Display(void *pvParameters);
void task1(void *pvParameters);
void DMA_INIT(void);
void USART2_DMA_CFG(void);
void USART2_DMA_Start(uint8_t *data, uint16_t len);
void fc_send_attitude(const attitude_t *att);
extern xQueueHandle xDataQueue;

// 在 main.h 或 sensor.h 中统一定义
typedef struct {
	// 建议按照 4字节对齐 排列，把 float 或 int 放前面
	int16_t mag[3];	 // 磁力计 (6字节)
	int16_t padding; // 补齐2字节，防止对齐问题（可选，视编译器而定）

	int32_t accel[3]; // 加速度 (12字节) - 建议用 int32_t
	int32_t gyro[3];  // 陀螺仪 (12字节)

	int8_t temp;	  // 温度 (1字节)
	uint8_t reserved; // 补齐1字节，让总长度是偶数
} SensorData_t;		  // 传输数据结构体

#endif /* USER_USE_H_ */
