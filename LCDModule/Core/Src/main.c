/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lcd.h"
#include <string.h>
#include <stdio.h>
#include "temp.h"
#include "one_wire.h"
#include <inttypes.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM14_Init(void);
static void time2String(char* buffer);
static void updateTime();
/* USER CODE BEGIN PFP */
static void handleLcdUpdate(bool_t isCursorOn, TEMP_EXTENDED temp, uint32_t humidity);
static void handleTimeDigitChange(bool_t isUp);
static void initContext(CONTEXT* pContext);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
static CONTEXT context = {0};

int main(void)
{
  /* USER CODE BEGIN 1 */
  bool_t isCursorOn = FALSE;
  TEMP_EXTENDED temp = {0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_ADC_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim14); //Starte den Timer für Mikrosekunden-Delay (One-Wire)
  lcd_init(); //Initialisierung und Konfiguration des LCD's
  one_wire_init(&htim14); //Initialisierung der One-Wire-Verbindung

  //ADC für das Potentiometer initialisieren
  HAL_ADC_Start(&hadc);
  //Timer6 für die Sekundenzählung (Uhrzeit-Darstellung) starten
  HAL_TIM_Base_Start_IT(&htim6);

  //Initial, die Werte für die simulierte Luftfeuchtigkeit und der Temperatur auslesen
  HAL_ADC_PollForConversion(&hadc, 1000);
  uint32_t humidity = HAL_ADC_GetValue(&hadc);
  temp_GetDegressCelsiusExtended(&temp);

  //Initialisierung der Context-Struktur
  initContext(&context);

  //Initiale LCD-Ausgabe mit Cursor, für die Konfiguration der Zeit
  time2String(context.outputBuffer);
  lcd_send(0x01, FALSE);
  lcd_sendString(context.outputBuffer);
  lcd_setCursor(context.cursorPosition, 0);
  lcd_toggleCursor(TRUE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
	  /*
	   * Überprüfe, ob aktueller Mode TIME_CONFIG ist, falls ja, aktiviere den Cursor, falls dieser noch nicht aktiviert ist
	   * ansonsten deaktivere den Cursor, falls er noch nicht deaktiviert wurde.
	   */
  	  if(context.currentMode == TIME_CONFIG) {
  		  if(!isCursorOn) {
  			  lcd_toggleCursor(TRUE);
  			  isCursorOn = TRUE;
  		  }
  	  } else {
  		  if(isCursorOn) {
  			  lcd_toggleCursor(FALSE);
  			  isCursorOn = FALSE;
  		  }
  	  }

  	  /*
  	   * Aktuallisiere die LCD-Darstellung, in Abhängigkeit davon,
  	   * ob das Flag lcdUpdate gesetzt ist oder nicht
  	   */
  	  if(context.flags.lcdUpdate) {

  		  //Aktuallisiere den erweiterten Temperaturwert
  		  temp_GetDegressCelsiusExtended(&temp);

  		  //Bestimme die Luftfeuchtigkeit (prozentual)
		  HAL_ADC_PollForConversion(&hadc, 1000);
		  humidity = (HAL_ADC_GetValue(&hadc) * 100) / MAX_HUMIDTY_VALUE;

		  //Aktuallisiere die LCD-Darstellung in Abhängigkeit des aktuellen Modus
		  handleLcdUpdate(isCursorOn, temp, humidity);
  	  }

  	  //Verhindern des Prellens der Buttons durch buttonPressed flag und einem 100ms delay
  	  if(context.flags.buttonPressed) {
  		  HAL_Delay(50);
  		  context.flags.buttonPressed = FALSE;
  	  }

  	  HAL_Delay(1);
    }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ONE_WIRE_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DB5_Pin|DB6_Pin|DB7_Pin|RS_Pin
                          |E_Pin|DB4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ONE_WIRE_Pin LD2_Pin */
  GPIO_InitStruct.Pin = ONE_WIRE_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DB5_Pin DB6_Pin DB7_Pin RS_Pin
                           E_Pin DB4_Pin */
  GPIO_InitStruct.Pin = DB5_Pin|DB6_Pin|DB7_Pin|RS_Pin
                          |E_Pin|DB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CTRL_CHANGE_MODE_Pin CTRL_NUM_DOWN_Pin CTRL_NUM_UP_Pin CTRL_R_Pin
                           CTRL_L_Pin */
  GPIO_InitStruct.Pin = CTRL_CHANGE_MODE_Pin|CTRL_NUM_DOWN_Pin|CTRL_NUM_UP_Pin|CTRL_R_Pin
                          |CTRL_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//Initialisierungsfunktion für die Context-Daten
void initContext(CONTEXT* pContext) {
	pContext->currentMode = TIME_CONFIG;
	pContext->flags.lcdUpdate = FALSE;
	pContext->flags.buttonPressed = FALSE;
	pContext->cursorPosition = 0;
	pContext->timeConfig = HOUR_0;
	pContext->time.packed_time = 0;
	memset(pContext->outputBuffer, 0, OUTPUT_BUFFER_SIZE);
}

/*
 * Interrupt-Handler-Funktion für die Steuerung-Buttons
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    //Überprüfe buttonPressed Flag, um Prellen zu verhindern
    if(context.flags.buttonPressed) return;

    switch (GPIO_Pin)
    {
    case CTRL_L_Pin: //Kontroll-Links (Wechsel der Ziffer-Position und der Modi)
        if(context.currentMode == TIME_CONFIG) {
            if(context.timeConfig == HOUR_0){
                //Setze Position der Zeiteinstellung auf Zehnerstelle der Sekunden
                context.timeConfig = SEC_1;
                context.cursorPosition = 7;
            } else {
                context.timeConfig--;
                if(context.cursorPosition == 6
                    || context.cursorPosition == 3) {
                    context.cursorPosition -= 2;
                } else {
                    context.cursorPosition--;
                }
            }
        } else {
        	//Falls alternierenden Anzeigemodus aktiviert ist, deaktivere ihn
            context.flags.isInPeroidMode = FALSE;

            //Manueller Wechsel der Modi
            if(context.currentMode == TIME) {
                context.currentMode = TEMP_HUMIDITY;
                context.flags.lcdUpdate = TRUE;
            } else {
                context.currentMode--;
                context.flags.lcdUpdate = TRUE;
            }
        }
        break;
    case CTRL_R_Pin: //Kontroll-Rechts (Wechsel der Ziffer-Position und der Modi)
        if(context.currentMode == TIME_CONFIG) {
            if(context.timeConfig == SEC_1){
                //Setze Position der Zeiteinstellung auf Zehnerstelle der Stunden
                context.timeConfig = HOUR_0;
                context.cursorPosition = 0;
            } else {
                context.timeConfig++;
                if(context.cursorPosition == 1
                    || context.cursorPosition == 4) {
                    context.cursorPosition += 2;
                } else {
                    context.cursorPosition++;
                }
            }
        } else {
        	//Falls alternierenden Anzeigemodus aktiviert ist, deaktivere ihn
        	 context.flags.isInPeroidMode = FALSE;

        	//Manueller Wechsel der Modi
            if(context.currentMode == TEMP_HUMIDITY) {
                context.currentMode = TIME;
                context.flags.lcdUpdate = TRUE;
            } else {
               context.currentMode++;
               context.flags.lcdUpdate = TRUE;
            }
        }
        break;
    case CTRL_NUM_UP_Pin: //Konroll-Up, zum Inkrementieren der aktuellen Zeitziffer
        if(context.currentMode == TIME_CONFIG) {
            handleTimeDigitChange(TRUE);
        } else {
        	//Flag zur Aktivierung des alternierenden Anzeigemodus
            if(context.flags.isInPeroidMode) {
                context.flags.isInPeroidMode = FALSE;
            } else {
                context.flags.isInPeroidMode = TRUE;
            }
        }
        break;
    case CTRL_NUM_DOWN_Pin: //Konroll-Down, zum Dekrementieren der aktuellen Zeitziffer
        if(context.currentMode == TIME_CONFIG) {
            handleTimeDigitChange(FALSE);
        }
        break;
    case CTRL_CHANGE_MODE_Pin: //Wechsel zwischen dem Modus zur Zeiteinstellung und dem Modus zur Zeitdarstellung
        if(context.currentMode == TIME_CONFIG) {
            context.currentMode = TIME;
            context.cursorPosition = 0;
        } else {
            context.currentMode = TIME_CONFIG;
        }
        break;
    }

    context.flags.lcdUpdate = TRUE; //Flag, zum aktuallisieren des LCD's aus der Main-Funktion
    context.flags.buttonPressed = TRUE;
}

/*
 * Interrupt-Handler-Funktion für den Timer, zur Sekundenzählung
 * und Zeitaktuallisierung sowie zur Interval-Prüfung für den automatischen Modi-Wechsel
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(context.currentMode != TIME_CONFIG) {
		updateTime();
		context.flags.lcdUpdate = TRUE;

		//Falls alternierenden Anzeigemodus aktiviert, prüfe ob CHANGE_MODE_PEROID erreicht wurde
		if(context.flags.isInPeroidMode &&
				context.time.seconds % CHANGE_MODE_PEROID == 0) {
			if(context.currentMode == TEMP_HUMIDITY) {
				context.currentMode = TIME;
			} else {
			    context.currentMode = TEMP_HUMIDITY;
			}
		}
	}
}

/*
 * Funktion, um die Zeit korrekt zu aktualliseren,
 * nachdem die Sekundeninkrementierung stattgefunden hat
 */
static void updateTime() {
	context.time.seconds++;
	if(context.time.seconds >= 60) {
		context.time.seconds = 0;
		context.time.minutes++;
	}
	if(context.time.minutes >= 60) {
		context.time.hours++;
		context.time.minutes = 0;
	}
	if(context.time.hours >= 24) {
		context.time.hours = 0;
	}
}

/*
 * Funktion, zur textuellen Repräsentation der aktuellen Zeit
 */
static void time2String(char* buffer) {
	sprintf(buffer, "%02d:%02d:%02d", context.time.hours, context.time.minutes, context.time.seconds);
}

/*
 * Funktion, die die einstellbare Zeit für
 * die Inkre- und Dekrementierung richtig errechnet und speichert
 *
*/
static void handleTimeDigitChange(bool_t isUp) {
	switch (context.timeConfig)
	{
	case HOUR_0:
		if(isUp) {
			if(context.time.hours >= 14) { //Überlauf bei den Stunden?
				context.time.hours = context.time.hours + 10 - 24;
			} else {
				context.time.hours += 10;
			}
		} else {
			if(context.time.hours <= 9) {
				context.time.hours = context.time.hours + 24 - 10;
			} else {
				context.time.hours -= 10;
			}
		}
		break;
	case HOUR_1:
		if(isUp) {
			if(context.time.hours >= 23) { //Überlauf bei den Stunden?
				context.time.hours = 0;
			} else {
				context.time.hours++;
			}
		} else {
			if(context.time.hours <= 0) {
				context.time.hours = 23;
			} else {
				context.time.hours--;
			}
		}
		break;
	case MIN_0:
		if(isUp) {
			if(context.time.minutes >= 50) { //Überlauf bei der Zehnerstelle der Minuten?
				context.time.minutes = context.time.minutes + 10 - 60;
			} else {
				context.time.minutes += 10;
					}
		} else {
			if(context.time.minutes <= 9) {
				context.time.minutes = context.time.minutes + 60 - 10;
			} else {
				context.time.minutes -= 10;
			}
		}
		break;
	case MIN_1:
		if(isUp) {
			if(context.time.minutes >= 59) { //Überlauf bei der Einerstelle der Minuten?
				context.time.minutes = 0;
			} else {
				context.time.minutes++;
			}
		} else {
			if(context.time.minutes <= 0) {
				context.time.minutes = 59;
			} else {
				context.time.minutes--;
			}
		}
		break;
	case SEC_0:
		if(isUp) {
			if(context.time.seconds >= 50) { //Überlauf bei der Zehnerstelle der Sekunden?
				context.time.seconds = context.time.seconds + 10 - 60;
			} else {
				context.time.seconds += 10;
			}
		} else {
			if(context.time.seconds <= 9) {
				context.time.seconds = context.time.seconds + 60 - 10;
			} else {
				context.time.seconds -= 10;
			}
		}

		break;
	case SEC_1:
		if(isUp) {
			if(context.time.seconds >= 59) {
				context.time.seconds = 0;
			} else {
				context.time.seconds++;
			}
		} else {
			if(context.time.seconds <= 0) {
				context.time.seconds = 59;
			} else {
				context.time.seconds--;
			}
		}
		break;
	}
}

static void handleLcdUpdate(bool_t isCursorOn, TEMP_EXTENDED temp, uint32_t humidity) {
	if(context.currentMode == TIME_CONFIG) {
		time2String(context.outputBuffer);
		lcd_send(0x01, FALSE);
		lcd_sendString(context.outputBuffer);
		if(isCursorOn) {
			lcd_setCursor(context.cursorPosition, 0);
		}
	} else if(context.currentMode == TIME) {
		time2String(context.outputBuffer);
		lcd_send(0x01, FALSE);
		lcd_sendString(context.outputBuffer);
	} else if(context.currentMode == TIME_TEMP) {
		char timeStr[9] = {0};
		time2String(timeStr);
		sprintf(context.outputBuffer, "%s%s%d,%d%cC", timeStr, LCD_OUTPUT_PADDING, temp.preDec, temp.decPlace, DEGREE_SYMBOL);
		lcd_send(0x01, FALSE);
		lcd_sendString(context.outputBuffer);
	} else if(context.currentMode == TEMP_HUMIDITY) {
		sprintf(context.outputBuffer, "%d,%d%cC%s %" PRIu32 "%c", temp.preDec, temp.decPlace, DEGREE_SYMBOL, LCD_OUTPUT_PADDING, humidity, '%');
		lcd_send(0x01, FALSE);
		lcd_sendString(context.outputBuffer);
	}

	context.flags.lcdUpdate = FALSE;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
