#ifndef DELAY_US_H
#define DELAY_US_H

#include "stm32h7xx_hal.h"

void DWT_Delay_Init(void);
void delay_us(uint32_t us);

#endif
