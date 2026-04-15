/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *task1 and task2 alternate printing
 */
#include "AHT_10.h"
#include "AP3216C.h"
#include "FreeRTOS.h"
#include "IIC.h"
#include "QMI8658.h"
#include "debug.h"
#include "lcd.h"
#include "queue.h"
#include "task.h"
#include "use.h"

/* Global define */

/* Global Variable */

TaskHandle_t Task1Task_Handler;
TaskHandle_t Task2Task_Handler;

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);
	DMA_INIT();
	USART2_DMA_CFG();

	// 3. 开启 DMA 发送中断
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	xDataQueue = xQueueCreate(1, sizeof(SensorData_t));
	if (xDataQueue == NULL) {
		printf("Queue Create Failed!\r\n");
		while (1)
			; // 崩溃处理
	}
        
        init1();
	xTaskCreate(get1_task, "Sensor", 512, NULL, 5, NULL);
	xTaskCreate(vTask_Display, "Display", 512, NULL, 4, NULL);

	vTaskStartScheduler();
	while (1) {
		printf("shouldn't run at here!!\n");
	}
}
