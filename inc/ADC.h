/*
 * ADC.h
 *
 *  Created on: Jul 8, 2025
 *      Author: Siddhanta
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32f4xx.h"


#define VREF 3
#define VREF_F 3.0
#define ADC_THRESHOLD ((uint16_t)((2.7 / VREF) * 4095)) //3V Threshold
#define ADC_THRESHOLD_FLOAT ((uint16_t)((2.7f / VREF_F) * 4095.0f))

//ADC initialisation function
void adc_init(void);
uint16_t adc_read(void);

void ADC1_CH8_Init(void);
uint16_t adc_read_CH8(void);

//GPIO Initialisation function
void gpiod_init();
void delay_ms(uint32_t ms);
uint8_t read_button(void);

//USART Initialisation function
void USART2_Init(void);
void USART2_SendChar(char c);
void USART2_SendString(const char *str);
char USART2_ReceiveChar(void);
void USART2_GetString(char *buffer, int maxLength);


#endif /* INC_ADC_H_ */
