/*
 * ADC.c
 *
 *  Created on: Jul 8, 2025
 *      Author: Siddhanta
 */

#include "ADC.h"


void adc_init(void) {

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= (3<<(2*1)); //PA1 for channel 1

	ADC1->SQR3 = 1; //Channel 1
	ADC1->SMPR2 |= (7 << 3); //Max Sample Time
	ADC1->CR2 |= ADC_CR2_ADON; //Enable ADC
}

uint16_t adc_read(void){

	ADC1->CR2 |= ADC_CR2_SWSTART;
	while(!(ADC1->SR & ADC_SR_EOC));
	return ADC1->DR;

}
void ADC1_CH8_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    GPIOB->MODER |= (3 << (0 * 2));  // PB0 analog
    GPIOB->PUPDR &= ~(3 << (0 * 2)); // No pull
    ADC1->CR2 = 0;
    ADC1->SQR3 = 8; // Channel 8 (PB0)
    ADC1->SMPR2 |= (7 << (3 * 8)); // Max sample time
    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t adc_read_CH8(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

void gpiod_init(){

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	//Setting PD 12,13,14,4,5,6 as output
	//PD 4, 5, 6 is used as EN pins for the DG408
	//PD 12, 13, 14 is used as the AD0 AD1 and AD2 pins
	GPIOD->MODER &= ~((3<<(12*2))|(3<<(13*2))|(3<<(14*2))|(3<<(4*2))|(3<<(5*2))|(3<<(6*2)));
	GPIOD->MODER |= ((1<<(12*2))|(1<<(13*2))|(1<<(14*2))|(1<<(4*2))|(1<<(5*2))|(1<<(6*2)));
	//GPIOD->OTYPER &= ~((1<<(12*2))|(1<<(13*2))|(1<<(14*2))|(1<<(4*2))|(1<<(5*2))|(1<<(6*2)));
	GPIOD->PUPDR &= ~((3<<(12*2))|(3<<(13*2))|(3<<(14*2))|(3<<(4*2))|(3<<(5*2))|(3<<(6*2)));
	GPIOD->OTYPER &= ~((1<<12)|(1<<13)|(1<<14)|(1<<4)|(1<<5)|(1<<6));
	//Setting PD0 as input for the button
	GPIOD->MODER &=~(3<<0);
	GPIOD->PUPDR &=~(3<<0);
	GPIOD->PUPDR |= (1<<0);
	//Setting PA0 as input for button
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER &= ~(3 << (0 * 2));
	GPIOA->PUPDR &= ~(3 << (0 * 2));
	//GPIOA->PUPDR |= (2 << (0 * 2)); // Pull-down
	//Setting PD11 as output LED

	GPIOD->MODER |= (1<<(15*2));

}



void delay_ms(uint32_t ms){

	for(uint32_t i=0; i<ms*16000; i++){

	__asm__("nop");

	}

}



uint8_t read_button(void) {

 return (GPIOA->IDR & (1 << 0)) ? 1 : 0;

}



//uint8_t read_button (void){

//	if(GPIOD->IDR && (0<<1) == 1)

//	return 1;

//	else

//	return 0;

//}

// Initialize USART2 for communication
void USART2_Init(void) {
    // Enable clocks for GPIOA and USART2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure PA2 (TX) and PA3 (RX) as alternate function
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3); // Clear mode bits
    GPIOA->MODER |= (2 << GPIO_MODER_MODER2_Pos) | (2 << GPIO_MODER_MODER3_Pos); // Set to AF mode

    GPIOA->AFR[0] &= ~((0xF << (4 * 2)) | (0xF << (4 * 3))); // Clear alternate function bits
    GPIOA->AFR[0] |= (7 << (4 * 2)) | (7 << (4 * 3)); // AF7 (USART2) for PA2 and PA3

    // Configure USART2
    USART2->BRR = 16000000 / 9600; // Assuming APB1 clock is 42 MHz
    USART2->CR1 |= USART_CR1_RE | USART_CR1_TE; // Enable receiver and transmitter
    USART2->CR1 |= USART_CR1_UE; // Enable USART
}

// Send a character via USART2
void USART2_SendChar(char c) {
    while (!(USART2->SR & USART_SR_TXE)); // Wait until transmit buffer is empty
    USART2->DR = c;
}

// Send a null-terminated string via USART2
void USART2_SendString(const char *str) {
    while (*str) {
        USART2_SendChar(*str++);
    }
}

// Receive a character via USART2 (blocking)
char USART2_ReceiveChar(void) {
    while (!(USART2->SR & USART_SR_RXNE)); // Wait until data is received
    return (char)(USART2->DR & 0xFF);
}

// Get a string from USART2 (stops at newline or carriage return)
void USART2_GetString(char *buffer, int maxLength) {
    int i = 0;
    char c;

    while (i < (maxLength - 1)) {
        c = USART2_ReceiveChar();
        USART2_SendChar(c); // Echo back

        if (c == '\r' || c == '\n') {
            break;
        }

        buffer[i++] = c;
    }

    buffer[i] = '\0'; // Null-terminate the string
}



