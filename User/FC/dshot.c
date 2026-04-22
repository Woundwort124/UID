#include "dshot.h"
#include "ch32v30x.h"

/*
 * DShot600: 600 kbps, 1 bit = 1.667 µs
 * TIM1 @ 144 MHz, ARR = 239 → Tpwm = 240/144e6 = 1.667 µs
 *
 * Bit encoding (duty cycle):
 *   logic 1: CCR = 180  (75.0 %)
 *   logic 0: CCR = 90   (37.5 %)
 *
 * Frame: 16 bits (MSB first) + 1 trailing zero (reset/gap)
 *   [10:0]  throttle value (0-2047)
 *   [11]    telemetry request
 *   [15:12] CRC = (~(v ^ (v>>4) ^ (v>>8))) & 0xF
 */

#define DSHOT_ARR 239u
#define DSHOT_T1H 180u	    /* logic 1 high time */
#define DSHOT_T0H 90u	    /* logic 0 high time */
#define DSHOT_FRAME_LEN 17u /* 16 data bits + 1 reset slot */

/* DMA buffers — one per channel, 17 x uint16_t */
static uint16_t dshot_buf[4][DSHOT_FRAME_LEN];

/* Build a 16-bit DShot frame (no telemetry)
 * Frame layout: [15:5]=value, [4]=telem=0, [3:0]=CRC */
static uint16_t dshot_make_frame(uint16_t value)
{
	uint16_t packet = (value << 1) | 0; /* telem = 0 */
	uint16_t crc = (~(packet ^ (packet >> 4) ^ (packet >> 8))) & 0x0Fu;
	return (packet << 4) | crc;
}

/* Fill dshot_buf[ch] from a 16-bit frame word */
static void dshot_encode(uint8_t ch, uint16_t frame)
{
	for (int i = 0; i < 16; i++) {
		dshot_buf[ch][i] = (frame & 0x8000u) ? DSHOT_T1H : DSHOT_T0H;
		frame <<= 1;
	}
	dshot_buf[ch][16] = 0; /* reset / inter-frame gap */
}

static void tim1_gpio_init(void)
{
	GPIO_InitTypeDef g = {0};
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	g.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	g.GPIO_Mode = GPIO_Mode_AF_PP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &g);
}

static void tim1_pwm_init(void)
{
	TIM_TimeBaseInitTypeDef tb = {0};
	TIM_OCInitTypeDef oc = {0};

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	tb.TIM_Prescaler = 0;
	tb.TIM_CounterMode = TIM_CounterMode_Up;
	tb.TIM_Period = DSHOT_ARR;
	tb.TIM_ClockDivision = TIM_CKD_DIV1;
	tb.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &tb);

	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_Pulse = 0;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;

	TIM_OC1Init(TIM1, &oc);
	TIM_OC2Init(TIM1, &oc);
	TIM_OC3Init(TIM1, &oc);
	TIM_OC4Init(TIM1, &oc);

	/* Disable preload so DMA writes take effect immediately */
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Disable);

	TIM_ARRPreloadConfig(TIM1, DISABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
}

/*
 * DMA channel → TIM1 CCR mapping (CH32V307 / STM32F1 compatible):
 *   TIM1_CH1 → DMA1_Channel2  (CCR addr: &TIM1->CH1CVR)
 *   TIM1_CH2 → DMA1_Channel3  (CCR addr: &TIM1->CH2CVR)
 *   TIM1_CH4 → DMA1_Channel4  (CCR addr: &TIM1->CH4CVR)
 *   TIM1_CH3 → DMA1_Channel6  (CCR addr: &TIM1->CH3CVR)
 */
static void dma_channel_init(DMA_Channel_TypeDef *ch, volatile uint16_t *ccr, uint16_t *buf)
{
	DMA_InitTypeDef d = {0};
	DMA_DeInit(ch);
	d.DMA_PeripheralBaseAddr = (uint32_t)ccr;
	d.DMA_MemoryBaseAddr = (uint32_t)buf;
	d.DMA_DIR = DMA_DIR_PeripheralDST;
	d.DMA_BufferSize = DSHOT_FRAME_LEN;
	d.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	d.DMA_MemoryInc = DMA_MemoryInc_Enable;
	d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	d.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	d.DMA_Mode = DMA_Mode_Normal;
	d.DMA_Priority = DMA_Priority_VeryHigh;
	d.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(ch, &d);
	/* Do not enable yet — triggered per-frame in dshot_write() */
}

static void dma_init_all(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	dma_channel_init(DMA1_Channel2, &TIM1->CH1CVR, dshot_buf[0]);
	dma_channel_init(DMA1_Channel3, &TIM1->CH2CVR, dshot_buf[1]);
	dma_channel_init(DMA1_Channel6, &TIM1->CH3CVR, dshot_buf[2]);
	dma_channel_init(DMA1_Channel4, &TIM1->CH4CVR, dshot_buf[3]);

	TIM_DMACmd(TIM1, TIM_DMA_CC1 | TIM_DMA_CC2 | TIM_DMA_CC3 | TIM_DMA_CC4, ENABLE);
}

void dshot_init(void)
{
	tim1_gpio_init();
	tim1_pwm_init();
	dma_init_all();
}

/* Restart a DMA channel for one frame transfer */
static void dma_send(DMA_Channel_TypeDef *ch, uint16_t *buf)
{
	DMA_Cmd(ch, DISABLE);
	ch->MADDR = (uint32_t)buf;
	ch->CNTR = DSHOT_FRAME_LEN;
	DMA_Cmd(ch, ENABLE);
}

void dshot_write(const uint16_t values[4])
{
	for (int i = 0; i < 4; i++) {
		uint16_t v = values[i];
		uint16_t dshot_val;
		if (v == 0) {
			dshot_val = 0;
		} else {
			/* Map 1-1000 → 48-2047 */
			dshot_val = 48 + (uint16_t)((uint32_t)(v - 1) * (2047 - 48) / 999);
		}
		dshot_encode(i, dshot_make_frame(dshot_val));
	}

	dma_send(DMA1_Channel2, dshot_buf[0]);
	dma_send(DMA1_Channel3, dshot_buf[1]);
	dma_send(DMA1_Channel6, dshot_buf[2]);
	dma_send(DMA1_Channel4, dshot_buf[3]);
}
