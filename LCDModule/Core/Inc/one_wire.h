#ifndef INC_ONE_WIRE_H_
#define INC_ONE_WIRE_H_

#include "stm32f0xx_hal.h"

#define ONE_WIRE_Pin GPIO_PIN_1
#define ONE_WIRE_GPIO_Port GPIOA

/*Delays in microseconds*/

#define RESET_DELAY_0 480
#define RESET_DELAY_1 400

#define WRITE_1_DELAY_LOW	1
#define WRITE_1_DELAY_REC	60
#define WRITE_0_DELAY_LOW	60
#define WRITE_0_DELAY_REC	1

#define READ_DELAY_LOW		2
#define READ_DELAY_HIGH		1 //Within 15μs (?)
#define READ_DELAY_REC		60 //Total read duration needs to be 60μs



void one_wire_init();
uint8_t one_wire_Reset();

void one_wire_WriteBit(uint8_t bit);
uint8_t one_wire_ReadBit();

void one_wire_WriteByte(uint8_t byte);
uint8_t one_wire_ReadByte();

void one_wire_WriteBuffer(uint8_t* buffer, size_t size);
void one_wire_ReadBuffer(uint8_t* buffer, size_t size);


#endif /* INC_ONE_WIRE_H_ */
