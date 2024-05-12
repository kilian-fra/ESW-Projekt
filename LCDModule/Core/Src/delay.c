#include "delay.h"

//https://controllerstech.com/create-1-microsecond-delay-stm32/

static TIM_HandleTypeDef* pTimerHandle;

void delay_init(TIM_HandleTypeDef* timer) {
	pTimerHandle = timer;
}

//Funktion für Zeitverzögerung in Mikrosekunden (für One-Wire)
void delay(uint16_t t) {
	__HAL_TIM_SET_COUNTER(pTimerHandle, 0);
	while (__HAL_TIM_GET_COUNTER(pTimerHandle) < t);
}
