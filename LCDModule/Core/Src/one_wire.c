#include "stm32f0xx_hal.h"
#include "one_wire.h"
#include "delay.h"

/*
 * https://www.analog.com/media/en/technical-documentation/data-sheets/ds18s20.pdf
 * https://pdfserv.maximintegrated.com/en/an/AN126.pdf
 */

//Funktion, zur Konfiguration des Pin's als Ausgabe, um Daten zu übertragen
static void one_wire_SetOutput() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ONE_WIRE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ONE_WIRE_GPIO_Port, &GPIO_InitStruct);
}

/*
 * Funktion, zur Konfiguration des Pin's als Eingabe, um Daten zu lesen,
 * bzw. den hochohmigen Zustand wieder einzunehmen
 */
static void one_wire_SetInput() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ONE_WIRE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ONE_WIRE_GPIO_Port, &GPIO_InitStruct);
}

static inline void one_wire_SetLow() {
	HAL_GPIO_WritePin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin, GPIO_PIN_RESET);
}

static inline void one_wire_SetHigh() {
	HAL_GPIO_WritePin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin, GPIO_PIN_SET);
}

/*
 * Funktion, zum initialiseren der One-Wire-Kommunikation.
 * Dies umfasst die Timer-Initialiserung (für die Mikrosekunden-Verzögerung)
 * und das zurücksetzen des bus.
 */
void one_wire_init(TIM_HandleTypeDef* timer) {
	delay_init(timer);

	one_wire_Reset();
}

uint8_t one_wire_Reset() {
	uint8_t result = 0; //Either 1 (if no presence) or 0 (if presence)

	//Pull low
	one_wire_SetOutput();
	one_wire_SetLow();

	delay(RESET_DELAY_0);

	//Pull high
	one_wire_SetHigh();

	one_wire_SetInput();

	//Check presence
	result = HAL_GPIO_ReadPin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin) ^ 0x01;

	delay(RESET_DELAY_1);

	return result;
}

void one_wire_WriteBit(uint8_t bit) {
	//Configure pin for write
	one_wire_SetOutput();

	//Pull low
	one_wire_SetLow();

	if (bit) {
		delay(WRITE_1_DELAY_LOW);
		//Pull high
		one_wire_SetHigh();
		delay(WRITE_1_DELAY_REC);
	} else {
		delay(WRITE_0_DELAY_LOW);
		//Pull high
		one_wire_SetHigh();
		delay(WRITE_0_DELAY_REC);
	}

	one_wire_SetInput();
}

uint8_t one_wire_ReadBit() {
	uint8_t result = 0;

	one_wire_SetOutput();

	//Pull low
	one_wire_SetLow();
	delay(READ_DELAY_LOW);

	//Pull high
	one_wire_SetHigh();
	delay(READ_DELAY_HIGH);

	one_wire_SetInput();

	//Store read bit
	result = HAL_GPIO_ReadPin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin) & 0x01;

	//Wait recovery
	delay(READ_DELAY_REC);

	return result;
}

void one_wire_WriteByte(uint8_t byte) {
	for (uint8_t i = 0; i < 8; i++) {
		one_wire_WriteBit((byte >> i) & 0x01);
	}
}

uint8_t one_wire_ReadByte() {
	uint8_t result = 0;

	for (uint8_t i = 0; i < 8; i++) {
		//If one set MSB
		if (one_wire_ReadBit()) {
			result |= 1 << i;
		}
	}

	return result;
}

void one_wire_WriteBuffer(uint8_t* buffer, size_t size) {
	for (size_t i = 0; i < size; i++) {
		one_wire_WriteByte(buffer[i]);
	}
}

void one_wire_ReadBuffer(uint8_t* buffer, size_t size) {
	for (size_t i = 0; i < size; i++) {
		buffer[i] = one_wire_ReadByte();
	}
}
