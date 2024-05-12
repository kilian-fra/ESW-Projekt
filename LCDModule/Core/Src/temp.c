#include "temp.h"
#include "delay.h"
#include "one_wire.h"

/*
 * https://www.analog.com/media/en/technical-documentation/data-sheets/ds18s20.pdf
 */

//Funktion, zum auslesen des Sensor-Scratchpads (umfasst insgesamt 9 Bytes)
static void temp_ReadSratchpad(uint8_t* bufferOut) {
    one_wire_Reset();
    one_wire_WriteByte(TEMP_SKIP_ROM);
    one_wire_WriteByte(TEMP_MEASURE_REQUEST);

    //Warten der maximalen Zeit, für eine Temperaturkonvertierung
    HAL_Delay(TEMP_CONV_TIME_MS);

    one_wire_Reset();
    one_wire_WriteByte(TEMP_SKIP_ROM);
    one_wire_WriteByte(TEMP_READ_REQUEST);

    one_wire_ReadBuffer(bufferOut, TEMP_SRATCHPAD_SIZE);
}

//Funktion, zum auslesen der Temperatur in der Einheit Grad Celsius (einfache Auflösung)
int16_t temp_GetDegressCelsius() {
    uint8_t scratchpadBuffer[TEMP_SRATCHPAD_SIZE] = {};
    temp_ReadSratchpad(scratchpadBuffer);

    int16_t temp = (int16_t)scratchpadBuffer[TEMP_MSB_INDEX];
    temp <<= 8;
    temp |= (int16_t)scratchpadBuffer[TEMP_LSB_INDEX];

    return temp >> 1;
}

/*
 * Funktion, zum auslesen der Temperatur in der Einheit Grad Celsius,
 * in erweiterter Auflösung, mit einer Nachkommastelle.
 */
void temp_GetDegressCelsiusExtended(TEMP_EXTENDED* pTempData) {
    uint8_t scratchpadBuffer[TEMP_SRATCHPAD_SIZE] = {0};

    temp_ReadSratchpad(scratchpadBuffer);

    //Lese TEMP_DATA ein (mit LSB und MSB)
    int16_t tempRead = (int16_t)scratchpadBuffer[TEMP_MSB_INDEX];
    tempRead <<= 8;
    tempRead |= (int16_t)scratchpadBuffer[TEMP_LSB_INDEX];
    tempRead >>= 1; //truncate bit 0 (0.5)

    //Wende die Formel aus dem Datenblatt an

    tempRead = (tempRead * 100) - 25;

    uint8_t countRemain = scratchpadBuffer[COUNT_REMAIN_INDEX];
    uint8_t countPer = scratchpadBuffer[COUNT_PER_INDEX];

    tempRead += ((countPer - countRemain) * 100) / countPer;

    pTempData->preDec = tempRead / 100;
    pTempData->decPlace = (tempRead / 10) % 10; //Auslesen der Nachkommastelle
}
