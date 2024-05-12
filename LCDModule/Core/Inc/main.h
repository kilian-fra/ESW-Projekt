/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_15_IRQn
#define ONE_WIRE_Pin GPIO_PIN_1
#define ONE_WIRE_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define DB5_Pin GPIO_PIN_1
#define DB5_GPIO_Port GPIOB
#define DB6_Pin GPIO_PIN_2
#define DB6_GPIO_Port GPIOB
#define DB7_Pin GPIO_PIN_11
#define DB7_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_13
#define RS_GPIO_Port GPIOB
#define E_Pin GPIO_PIN_14
#define E_GPIO_Port GPIOB
#define DB4_Pin GPIO_PIN_15
#define DB4_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define CTRL_CHANGE_MODE_Pin GPIO_PIN_4
#define CTRL_CHANGE_MODE_GPIO_Port GPIOB
#define CTRL_CHANGE_MODE_EXTI_IRQn EXTI4_15_IRQn
#define CTRL_NUM_DOWN_Pin GPIO_PIN_5
#define CTRL_NUM_DOWN_GPIO_Port GPIOB
#define CTRL_NUM_DOWN_EXTI_IRQn EXTI4_15_IRQn
#define CTRL_NUM_UP_Pin GPIO_PIN_6
#define CTRL_NUM_UP_GPIO_Port GPIOB
#define CTRL_NUM_UP_EXTI_IRQn EXTI4_15_IRQn
#define CTRL_R_Pin GPIO_PIN_8
#define CTRL_R_GPIO_Port GPIOB
#define CTRL_R_EXTI_IRQn EXTI4_15_IRQn
#define CTRL_L_Pin GPIO_PIN_9
#define CTRL_L_GPIO_Port GPIOB
#define CTRL_L_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */
#define OUTPUT_BUFFER_SIZE 64
#define MAX_HUMIDTY_VALUE 4095
#define LCD_OUTPUT_PADDING "  "
#define DEGREE_SYMBOL 0xDF
#define CHANGE_MODE_PEROID 10

#define TRUE 1
#define FALSE !TRUE
typedef uint8_t bool_t;

//Datentyp zur Repräsentation der möglichen Modi
typedef uint8_t DisplayMode;
enum {
	TIME,
	TIME_TEMP,
	TEMP_HUMIDITY,
	TIME_CONFIG
};

//Enum, zur Repräsentation der Positionen für die Zeitkonfiguration
typedef uint8_t TimeConfig;
enum {
	HOUR_0,
	HOUR_1,
	MIN_0,
	MIN_1,
	SEC_0,
	SEC_1
};

//Union-Datentyp zur Speicherung aller verwendeten Flags, für die jeweils ein Bit verwendet wird
typedef union {
	struct {
		bool_t lcdUpdate : 1;
		bool_t buttonPressed : 1;
		bool_t isInPeroidMode : 1;
	};

	bool_t flags;
}Flags;

//Union-Datentyp, um die Zeit (Stunden, Minuten & Sekunden) zu repräsentieren
typedef union {
    struct {
        uint32_t minutes : 6; // 6 bits for minutes (0-59)
        uint32_t seconds : 6; // 6 bits for seconds (0-59)
        uint32_t hours : 5;   // 5 bits for hours (0-23)
    };
    uint32_t packed_time; // 32-bit integer to hold the packed time
} Time;

//Struktur zur Speicherung aller benötigten Daten
typedef struct {
	DisplayMode currentMode;
	TimeConfig timeConfig;
	Time time;
	Flags flags;
	char cursorPosition;
	char outputBuffer[OUTPUT_BUFFER_SIZE];
}CONTEXT;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
