#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "stm32f0xx_hal.h"

void delay(uint16_t t); //microseconds
void delay_init(TIM_HandleTypeDef* timer);

#endif /* INC_DELAY_H_ */
