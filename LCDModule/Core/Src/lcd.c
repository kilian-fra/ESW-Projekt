#include "stm32f0xx_hal.h"
#include "lcd.h"

/*
 * https://cdn-reichelt.de/documents/datenblatt/A500/DEM16217SBH-PW-N.pdf
 */

static void setPins(char c) {
	//Lege 1 bzw. 0 an den jeweiligen PINS für die Ausgabe (char c) an
	HAL_GPIO_WritePin(DB4_GPIO_Port, DB4_Pin, c & 0x01);
	HAL_GPIO_WritePin(DB5_GPIO_Port, DB5_Pin, c & 0x02);
	HAL_GPIO_WritePin(DB6_GPIO_Port, DB6_Pin, c & 0x04);
	HAL_GPIO_WritePin(DB7_GPIO_Port, DB7_Pin, c & 0x08);

	//Simuliere eine fallende Flanke am E-Pin
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
}

//Initialiserungsfunktion für LCD
void lcd_init() {
	//Setze E-Pin und RS-Pin initial zurück
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);

	//Wechsel zunächst in den 8-Bit Modus (Function-Set: 0x03)
	setPins(3);
	HAL_Delay(5);
	setPins(3);
	HAL_Delay(2);
	setPins(3);
	HAL_Delay(2);
	//Wechsel nun in den 4-Bit Modus
	setPins(2);
	HAL_Delay(2);

	//Schalte Textmodus mit 2 Zeilen und 5x8 Matrixbuchstaben an
	lcd_send(0x28, 0);
	lcd_send(0x0C, 0);
	HAL_Delay(2);

	// Löschen des Displayinhaltes und abschließende Konfiguration
	lcd_send(0x01, 0);
	//Konfigruation mit Linksanordnung und schieben der Zeichen nach links
	lcd_send(6, 0);
	HAL_Delay(2);
}

//Funktion, zum Senden von Daten oder Kommandos an den LCD (Flag: isData)
void lcd_send(char c, uint8_t isData) {
	if (isData) {
		HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
	}

	setPins(c >> 4);
	setPins(c);
}

//Funktion, zum Senden von Strings
void lcd_sendString(char* c) {
	while(*c != 0) {
		lcd_send(*c++, 1);
	}
}

//Funktion, um Position des Cursors in der ersten bzw. zweiten Zeile zu setzen (Flag: isSecLine)
void lcd_setCursor(char xPos, uint8_t isSecLine) {
	if (isSecLine) lcd_send(0x80 | (xPos + 0x40), 0);
	else lcd_send(0x80 | xPos, 0);
}

//Funktion, zum aktivieren bzw. deaktivieren des cursors
void lcd_toggleCursor(uint8_t isOn) {
	if(isOn) {
		lcd_send(0x0E, 0);
	} else {
		lcd_send(0x0C, 0);
	}
}
