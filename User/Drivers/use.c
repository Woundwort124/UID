/*
 * use.c
 *
 *  Created on: 2026年4月13日
 *      Author: 213516
 */
#include "use.h"

#define RXBUF_SIZE 1024
u8 RxBuffer[RXBUF_SIZE] = {0}; // 接收缓冲区
uint16_t rxBufferReadPos = 0;  // 接收读指针

// DMA 发送完成标志位 (volatile 防止被编译器优化)
volatile uint8_t g_dma_tx_complete = 1;

// 串口发送队列

xQueueHandle xDataQueue;

uint8_t i2c_addr = 0X2C;
uint8_t res = 0x80;
u16 ir, als, ps;
void GPIO_Toggle_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void init1() // 初始化加速度计和磁力计
{
	Delay_Init();
	USART_Printf_Init(115200);

	lcd_init();

	lcd_set_color(BLACK, WHITE);
	lcd_show_string(50, 0, 32, "openCH.io");
	lcd_set_color(BLACK, WHITE);
	lcd_show_string(0, 48, 16, "AP3216C");
	while (AP3216C_Init()) // 初始化AP3216C
	{
		lcd_set_color(BLACK, RED);
		lcd_show_string(180, 48, 16, "Error");
		Delay_Ms(200);
		lcd_show_string(180, 48, 16, "     ");
		Delay_Ms(200);
	}
	lcd_set_color(BLACK, GREEN);
	lcd_show_string(180, 48, 16, "OK");

	// QMI8658
	qmi8658_init(); // 初始化 QMI8658，并校准，耗时 2~3 秒
	qmi8658_enable_no_motion(ENABLE, qmi8658_Int_none);

	//  end QMI8658

	// QMC7983
	// 测试 QMC7983 是否存在
	IIC_Init(200000, 0x02);

	res = IIC_WriteLen(i2c_addr, 0x0A, 1, &res);
	if (res == 0) {
		// 初始化 QMC7983
		delay_ms(10);
		res = 0x31;
		res = IIC_WriteLen(i2c_addr, 0x09, 1, &res);
		res = IIC_ReadByte(i2c_addr, 0x0D);

	} else {
		GPIO_Toggle_INIT();
	}
	// end QMC7983

	/* Read samples in polling mode (no int) */
}

void get1_task(void *pvParameters) // 采集任务
{
	float acc_mg[3];
	float gyro_mdps[3];
	float temp_degC;
	int16_t mag_lsb[3];

	// 2. 定义队列发送的数据结构
	SensorData_t xToSend;

	while (1) {
		// --- 开始采集 ---

		// 3. 进入临界区：保护 I2C 读取过程，防止被系统打断
		// 注意：这里只保护读取，不保护 LCD 显示
		taskENTER_CRITICAL();

		// 读取 QMI8658
		qmi8658_read_sensor_data(acc_mg, gyro_mdps);
		temp_degC = qmi8658_readTemp();
		if (acc_mg[0] == 0 && acc_mg[1] == 0 && acc_mg[2] == 0) {
			// 可以在这里翻转一个 LED 看看是不是每次都进这里
			GPIO_Toggle_INIT();
		}
		// 读取 QMC7983
		i2c_addr = 0x2C; // 确保全局变量 i2c_addr 定义正确
		u8 emm_status = IIC_ReadByte(i2c_addr, 0x06);

		if (emm_status & 0x01) {
			IIC_ReadLen(i2c_addr, 0x00, 6, (u8 *)mag_lsb);
		}

		taskEXIT_CRITICAL(); // 4. 退出临界区，允许系统调度

		// --- 处理数据并发送 ---

		// 填充结构体（假设你的结构体定义如下，如果不同请自行调整）
		xToSend.temp = (int)temp_degC;		    // 简单取整
		xToSend.accel[0] = (int)(acc_mg[0] * 1000); // 提前算好转成 int，避免传给 LCD 时算浮点
		xToSend.accel[1] = (int)(acc_mg[1] * 1000);
		xToSend.accel[2] = (int)(acc_mg[2] * 1000);

		xToSend.gyro[0] = (int)(gyro_mdps[0] * 1000);
		xToSend.gyro[1] = (int)(gyro_mdps[1] * 1000);
		xToSend.gyro[2] = (int)(gyro_mdps[2] * 1000);

		xToSend.mag[0] = mag_lsb[0];
		xToSend.mag[1] = mag_lsb[1];
		xToSend.mag[2] = mag_lsb[2];

		// 5. 发送到队列
		// 使用 pdMS_TO_TICKS(10) 防止队列满时卡死太久
		if (xQueueSend(xDataQueue, &xToSend, pdMS_TO_TICKS(10)) != pdPASS) {
			// 发送失败处理
		}

		// 6. 任务延时

		vTaskDelay(pdMS_TO_TICKS(20));
	}
}
void vTask_Display(void *pvParameters) // lcd与串口传输共享一个任务
{
	SensorData_t xReceivedData; // 接收传感器参数

	while (1) {
		// 1. 阻塞等待数据
		if (xQueueReceive(xDataQueue, &xReceivedData, portMAX_DELAY) == pdPASS) {
			// 2. 清屏（使用全屏清除，避免矩形残留）
			// 假设你的 LCD 驱动有 LCD_Clear 函数，如果没有就用你的 rectangle 填充
			lcd_clear(BLACK);

			lcd_set_color(BLACK, WHITE);

			// 3. 显示数据
			// 注意：这里直接打印 int，不要做浮点运算！

			// QMI8658
			lcd_show_string(10, 40, 16, "QMI8658 Temp: %d C", xReceivedData.temp);
			printf("QMI8658  Temp: %d C\n", xReceivedData.temp);
			lcd_set_color(BLACK, GREEN);
			lcd_show_string(10, 60, 16, "Acc X: %d", xReceivedData.accel[0]);
			lcd_show_string(10, 80, 16, "Acc Y: %d", xReceivedData.accel[1]);
			lcd_show_string(10, 100, 16, "Acc Z: %d", xReceivedData.accel[2]);
			printf("Acc X: %d\n", xReceivedData.accel[0]);
			printf("Acc Y: %d\n", xReceivedData.accel[1]);
			printf("Acc Z: %d\n", xReceivedData.accel[2]);
			lcd_show_string(150, 60, 16, "Gyr X: %d", xReceivedData.gyro[0]);
			lcd_show_string(150, 80, 16, "Gyr Y: %d", xReceivedData.gyro[1]);
			lcd_show_string(150, 100, 16, "Gyr Z: %d", xReceivedData.gyro[2]);
			printf("Gyr X: %d\n", xReceivedData.gyro[0]);
			printf("Gyr Y: %d\n", xReceivedData.gyro[1]);
			printf("Gyr Z: %d\n", xReceivedData.gyro[2]);
			// QMC7983
			lcd_set_color(BLACK, WHITE);
			lcd_show_string(10, 140, 16, "QMC7983");
			printf("QMC7983\n");
			lcd_set_color(BLACK, GREEN);
			lcd_show_string(10, 160, 16, "Mag X: %d", xReceivedData.mag[0]);
			lcd_show_string(10, 180, 16, "Mag Y: %d", xReceivedData.mag[1]);
			lcd_show_string(10, 200, 16, "Mag Z: %d", xReceivedData.mag[2]);
			printf("Mag X: %d\n", xReceivedData.mag[0]);
			printf("Mag Y: %d\n", xReceivedData.mag[1]);
			printf("Mag Z: %d\n", xReceivedData.mag[2]);
		}
	}
}

void DMA_INIT(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	// CH32V307: DMA 时钟在 AHB 总线
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	// --- TX DMA (Channel 7) 初始化 ---
	DMA_DeInit(DMA1_Channel7);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DATAR); // 外设地址
	DMA_InitStructure.DMA_MemoryBaseAddr = 0;			  // 内存地址 (发送时动态指定)
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;		  // 内存 -> 外设
	DMA_InitStructure.DMA_BufferSize = 0;				  // 长度 (发送时动态指定)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  // 外设地址不增加
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		  // 内存地址增加
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; // 单次模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);

	// --- RX DMA (Channel 6) 初始化 ---
	DMA_DeInit(DMA1_Channel6);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DATAR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; // 外设 -> 内存
	DMA_InitStructure.DMA_BufferSize = RXBUF_SIZE;	   // 循环缓冲大小
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	   // 循环模式
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);
}

/* --- 2. 串口初始化 (适配 CH32V307) --- */
void USART2_DMA_CFG(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// CH32V307: USART2 在 APB1, GPIOA 在 APB2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// PA2 (TX), PA3 (RX)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(USART2, &USART_InitStructure);

	// 开启 DMA 请求
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

	// 开启串口和 DMA
	USART_Cmd(USART2, ENABLE);
	DMA_Cmd(DMA1_Channel6, ENABLE); // 开启接收 DMA
}

/* --- 3. 发送启动函数 --- */
void USART2_DMA_Start(uint8_t *data, uint16_t len)
{
	// 等待上一次发送完成
	while (g_dma_tx_complete == 0)
		;

	g_dma_tx_complete = 0; // 标记忙

	// 清除 DMA 传输完成标志 (CH32V307 使用 DMA1_FLAG_TC)
	DMA_ClearITPendingBit(DMA1_FLAG_TC7);

	// 配置新的内存地址和长度
	DMA1_Channel7->MADDR = (uint32_t)data;
	DMA1_Channel7->CNTR = len; // 直接操作寄存器设置计数

	DMA_Cmd(DMA1_Channel7, ENABLE); // 启动 DMA
}

/* --- 4. 串口发送任务 --- */

/* --- 5. 中断服务函数 (关键！放在这里或 ch32v30x_it.c) --- */
// 确保函数名与启动文件 (startup_ch32v30x_hd.s) 中的一致
void __attribute__((interrupt("WCH-Interrupt-fast"))) DMA1_Channel7_IRQHandler(void)
{
	// CH32V307 检查 DMA 传输完成中断标志
	if (DMA_GetITStatus(DMA1_FLAG_TC7) != RESET) {
		DMA_ClearITPendingBit(DMA1_FLAG_TC7);
		g_dma_tx_complete = 1; // 发送完成，标志位置 1
	}
}
