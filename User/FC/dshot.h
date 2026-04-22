#ifndef DSHOT_H
#define DSHOT_H

#include <stdint.h>

/*
 * Dshot600 Driver - TIM1 CH1-4 (PA8/PA9/PA10/PA11) + DMA
 *
 * Motor mapping:
 *       M1 () -> TIM1_CH1 -> PA8  -> DMA1_Channel2
 *       M2 () -> TIM1_CH2 -> PA9  -> DMA1_Channel3
 *       M3 () -> TIM1_CH3 -> PA10 -> DMA1_Channel6
 *       M4 () -> TIM1_CH4 -> PA11 -> DMA1_Channel4
 *
 * Value ranges: 0 = disarmed/stop, 48-2047 = throttle
 * Accept mixer output 0-1000, mapped to DShot 48-2047.
 */
void dshot_init(void);

/* Send one frame to all 4 motors. values[]: 0-1000 (mixer output).
 * 0 -> DShot 0 (disarm); 1-1000 -> DShot 48-2047. */
void dshot_write(const uint16_t values[4]);

#endif // !DSHOT_H