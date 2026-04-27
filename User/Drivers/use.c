/*
 * use.c
 *
 *  Created on: 2026��4��13��
 *      Author: 213516
 */
#include "use.h"
#include "dshot.h"
#include "fc_core.h"

/*---------------------------*/
extern cmd_t g_cmd;	     // ��λ��ָ�� (���ڽ�����д��)
extern motor_cmd_t g_motors; // �ɿ����

cmd_t g_cmd = {.roll_sp = 0.0f, .pitch_sp = 0.0f, .yaw_rate_sp = 0.0f, .throttle = 0.3f, .armed = 1};

motor_cmd_t g_motors = {0};
/*---------------------------*/

#define RXBUF_SIZE 1024
u8 RxBuffer[RXBUF_SIZE] = {0}; // ���ջ�����
uint16_t rxBufferReadPos = 0;  // ���ն�ָ��

// DMA ������ɱ�־λ (volatile ��ֹ���������Ż�)
volatile uint8_t g_dma_tx_complete = 1;

// ���ڷ��Ͷ���

xQueueHandle xDataQueue;

uint8_t i2c_addr = 0X2C;
uint8_t res = 0x80;
u16 ir, als, ps;
/*---------------------------*/
static uint8_t crc8(const uint8_t *data, uint8_t len)
{
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++)
			crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
	}
	return crc;
}

static uint8_t tx_buf[17];

void fc_send_attitude(const attitude_t *att)
{
	tx_buf[0] = 0xAA;
	tx_buf[1] = 0x55;
	tx_buf[2] = 12;
	tx_buf[3] = 0x02;
	memcpy(&tx_buf[4], &att->roll, 4);
	memcpy(&tx_buf[8], &att->pitch, 4);
	memcpy(&tx_buf[12], &att->yaw, 4);
	tx_buf[16] = crc8(&tx_buf[2], 14); /* CRC ���� LEN+TYPE+PAYLOAD */
	// USART2_DMA_Start(tx_buf, 17);
}
/*---------------------------*/
void GPIO_Toggle_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void init1() // ��ʼ�����ٶȼƺʹ�����
{
	Delay_Init();
	USART_Printf_Init(115200);

	lcd_init();

	lcd_set_color(BLACK, WHITE);
	lcd_show_string(50, 0, 32, "openCH.io");
	lcd_set_color(BLACK, WHITE);
	lcd_show_string(0, 48, 16, "AP3216C");
	while (AP3216C_Init()) // ��ʼ��AP3216C
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
	qmi8658_init(); // ��ʼ�� QMI8658����У׼����ʱ 2~3 ��
	qmi8658_enable_no_motion(ENABLE, qmi8658_Int_none);

	//  end QMI8658

	// QMC7983
	// ���� QMC7983 �Ƿ����
	IIC_Init(200000, 0x02);

	res = IIC_WriteLen(i2c_addr, 0x0A, 1, &res);
	if (res == 0) {
		// ��ʼ�� QMC7983
		delay_ms(10);
		res = 0x31;
		res = IIC_WriteLen(i2c_addr, 0x09, 1, &res);
		res = IIC_ReadByte(i2c_addr, 0x0D);

	} else {
		GPIO_Toggle_INIT();
	}
	// end QMC7983

	/* Read samples in polling mode (no int) */
	/*---------------------------*/
	fc_config_t fc_fcg = {
		.madgwick = 0.1f,
		.pid_roll_kp = 1.0f,
		.pid_roll_ki = 0.0f,
		.pid_roll_kd = 0.0f,
		.pid_pitch_kp = 1.0f,
		.pid_pitch_ki = 0.0f,
		.pid_pitch_kd = 0.0f,
		.pid_yaw_kp = 1.0f,
		.pid_yaw_ki = 0.0f,
		.pid_yaw_kd = 0.0f,
		.pid_rate_kp = 0.5f,
		.pid_rate_ki = 0.0f,
		.pid_rate_kd = 0.0f,
	};

	fc_init(&fc_fcg);
	/*---------------------------*/
}

void get1_task(void *pvParameters) // �ɼ�����
{
	float acc_mg[3];
	float gyro_mdps[3];
	float temp_degC;
	int16_t mag_lsb[3];

	// 2. ������з��͵����ݽṹ
	SensorData_t xToSend;

	while (1) {
		// --- ��ʼ�ɼ� ---

		// 3. �����ٽ��������� I2C ��ȡ���̣���ֹ��ϵͳ���
		// ע�⣺����ֻ������ȡ�������� LCD ��ʾ
		taskENTER_CRITICAL();

		// ��ȡ QMI8658
		qmi8658_read_sensor_data(acc_mg, gyro_mdps);
		temp_degC = qmi8658_readTemp();
		if (acc_mg[0] == 0 && acc_mg[1] == 0 && acc_mg[2] == 0) {
			// ���������﷭תһ�� LED
			// �����ǲ���ÿ�ζ�������
			GPIO_Toggle_INIT();
		}
		// ��ȡ QMC7983
		i2c_addr = 0x2C; // ȷ��ȫ�ֱ��� i2c_addr ������ȷ
		u8 emm_status = IIC_ReadByte(i2c_addr, 0x06);

		if (emm_status & 0x01) {
			IIC_ReadLen(i2c_addr, 0x00, 6, (u8 *)mag_lsb);
		}

		taskEXIT_CRITICAL(); // 4. �˳��ٽ���������ϵͳ����

		// --- �������ݲ����� ---

		// ���ṹ�壨������Ľṹ�嶨�����£������ͬ�����е�����
		xToSend.temp = (int)temp_degC;
		xToSend.accel[0] = (int)(acc_mg[0] * 1000);
		xToSend.accel[1] = (int)(acc_mg[1] * 1000);
		xToSend.accel[2] = (int)(acc_mg[2] * 1000);

		xToSend.gyro[0] = (int)(gyro_mdps[0] * 1000);
		xToSend.gyro[1] = (int)(gyro_mdps[1] * 1000);
		xToSend.gyro[2] = (int)(gyro_mdps[2] * 1000);

		xToSend.mag[0] = mag_lsb[0];
		xToSend.mag[1] = mag_lsb[1];
		xToSend.mag[2] = mag_lsb[2];
		/*-----------------------------------------------------------------------------------*/
		imu_data_t imu;
		imu.accel[0] = acc_mg[0] * 0.00981f; // mg �� m/s? */
		imu.accel[1] = acc_mg[1] * 0.00981f;
		imu.accel[2] = acc_mg[2] * 0.00981f;
		imu.gyro[0] = gyro_mdps[0] * 0.000017453f; // mdps �� rad/s */
		imu.gyro[1] = gyro_mdps[1] * 0.000017453f;
		imu.gyro[2] = gyro_mdps[2] * 0.000017453f;
		imu.timestamp_us = 0;

		mag_data_t mag;
		mag.mag[0] = (float)mag_lsb[0];
		mag.mag[1] = (float)mag_lsb[1];
		mag.mag[2] = (float)mag_lsb[2];
		mag.updated = (emm_status & 0x01) ? 1 : 0;

		fc_update(&imu, &mag, &g_cmd, 0.02f, &g_motors);
		dshot_write(g_motors.motor);
		attitude_t att;
		fc_get_attitude(&att);
		fc_send_attitude(&att);
		/*-------------------------------------------------------------------------------*/
		// 5. ���͵�����
		// ʹ�� pdMS_TO_TICKS(10) ��ֹ������ʱ����̫��
		if (xQueueSend(xDataQueue, &xToSend, pdMS_TO_TICKS(10)) != pdPASS) {
			// ����ʧ�ܴ���
		}

		// 6. ������ʱ

		vTaskDelay(pdMS_TO_TICKS(20));
	}
}
void vTask_Display(void *pvParameters) // lcd�봮�ڴ��乲��һ������
{
	SensorData_t xReceivedData; // ���մ���������

	while (1) {
		// 1. �����ȴ�����
		if (xQueueReceive(xDataQueue, &xReceivedData, portMAX_DELAY) == pdPASS) {
			// 2. ������ʹ��ȫ�������������β�����
			// ������� LCD ������ LCD_Clear ���������û�о������ rectangle ���
			lcd_clear(BLACK);

			lcd_set_color(BLACK, WHITE);

			// 3. ��ʾ����
			// ע�⣺����ֱ�Ӵ�ӡ int����Ҫ���������㣡

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

	// CH32V307: DMA ʱ���� AHB ����
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	// --- TX DMA (Channel 7) ��ʼ�� ---
	DMA_DeInit(DMA1_Channel7);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DATAR); // �����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = 0;			  // �ڴ��ַ (����ʱ��ָ̬��)
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;		  // �ڴ� -> ����
	DMA_InitStructure.DMA_BufferSize = 0;				  // ���� (����ʱ��ָ̬��)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  // �����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		  // �ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; // ����ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);

	// --- RX DMA (Channel 6) ��ʼ�� ---
	DMA_DeInit(DMA1_Channel6);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DATAR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; // ���� -> �ڴ�
	DMA_InitStructure.DMA_BufferSize = RXBUF_SIZE;	   // ѭ�������С
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	   // ѭ��ģʽ
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);
}

/* --- 2. ���ڳ�ʼ�� (���� CH32V307) --- */
void USART2_DMA_CFG(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// CH32V307: USART2 �� APB1, GPIOA �� APB2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// PA2 (TX), PA3 (RX)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // ��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // ��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(USART2, &USART_InitStructure);

	// ���� DMA ����
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

	// �������ں� DMA
	USART_Cmd(USART2, ENABLE);
	DMA_Cmd(DMA1_Channel6, ENABLE); // �������� DMA
}

/* --- 3. ������������ --- */
void USART2_DMA_Start(uint8_t *data, uint16_t len)
{
	// �ȴ���һ�η������
	while (g_dma_tx_complete == 0)
		;

	g_dma_tx_complete = 0; // ���æ

	// ��� DMA ������ɱ�־ (CH32V307 ʹ�� DMA1_FLAG_TC)
	DMA_ClearITPendingBit(DMA1_FLAG_TC7);

	// �����µ��ڴ��ַ�ͳ���
	DMA1_Channel7->MADDR = (uint32_t)data;
	DMA1_Channel7->CNTR = len; // ֱ�Ӳ����Ĵ������ü���

	DMA_Cmd(DMA1_Channel7, ENABLE); // ���� DMA
}

/* --- 4. ���ڷ������� --- */

/* --- 5. �жϷ����� (�ؼ������������ ch32v30x_it.c) --- */
// ȷ���������������ļ� (startup_ch32v30x_hd.s) �е�һ��
void __attribute__((interrupt("WCH-Interrupt-fast"))) DMA1_Channel7_IRQHandler(void)
{
	// CH32V307 ��� DMA ��������жϱ�־
	if (DMA_GetITStatus(DMA1_FLAG_TC7) != RESET) {
		DMA_ClearITPendingBit(DMA1_FLAG_TC7);
		g_dma_tx_complete = 1; // ������ɣ���־λ�� 1
	}
}
