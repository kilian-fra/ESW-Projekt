#ifndef INC_LCD_H_
#define INC_LCD_H_

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

void lcd_init();
void lcd_send(char c, uint8_t isData);
void lcd_sendString(char* c);
void lcd_setCursor(char xPos, uint8_t isSecLine);
void lcd_toggleCursor(uint8_t isOn);

#endif /* INC_LCD_H_ */
