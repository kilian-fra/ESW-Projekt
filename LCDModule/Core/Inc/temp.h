#ifndef INC_TEMP_H_
#define INC_TEMP_H_

#include "stm32f0xx_hal.h"

#define TEMP_MEASURE_REQUEST 0x44
#define TEMP_SKIP_ROM         0xCC
#define TEMP_READ_REQUEST     0xBE
#define TEMP_SIZE_BYTES         9
#define TEMP_CONV_TIME_MS    750 //max time

#define TEMP_SRATCHPAD_SIZE 9 //bytes
#define COUNT_REMAIN_INDEX 6
#define COUNT_PER_INDEX 7
#define CRC_INDEX 8
#define TEMP_LSB_INDEX 0
#define TEMP_MSB_INDEX 1

typedef struct {
    int16_t preDec;
    int8_t decPlace;
}TEMP_EXTENDED;

int16_t temp_GetDegressCelsius();
void temp_GetDegressCelsiusExtended(TEMP_EXTENDED* pTempData);

#endif /* INC_TEMP_H_ */
