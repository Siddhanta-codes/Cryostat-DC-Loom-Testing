#include "stm32f4xx.h"



#define VREF 3

#define ADC_THRESHOLD ((uint16_t)((2.5 / VREF) * 4095)) //3V Threshold



void adc_init(void) {

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	GPIOA->MODER |= (3<<(2*1));



	ADC1->SQR3 = 1; //Channel 1

	ADC1->SMPR2 |= (7 << 3); //Max Sample Time

	ADC1->CR2 |= ADC_CR2_ADON; //Enable ADC

}



uint16_t adc_read(void){

	ADC1->CR2 |= ADC_CR2_SWSTART;

	while(!(ADC1->SR & ADC_SR_EOC));

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



void continuity_test(){

	GPIOD->ODR &=~ (1<<15);

	ADC1->CR2 |= ADC_CR2_ADON;

	uint16_t adc_value = adc_read();

	if (adc_value > ADC_THRESHOLD) {

	GPIOD->ODR |= (1 << 15);

	} else {

	GPIOD->ODR &=~ (1 << 15);

	}

	ADC1->CR2 &=~ ADC_CR2_ADON;

}



int main(void){

	adc_init();

	gpiod_init();



	uint8_t state = 1;

	uint8_t last_button = 0;



	while(1){



	uint8_t button = read_button();

	if(button == 1 && last_button == 0){

	switch(state){

	case 1: //000

	GPIOD->ODR &= ~(1<<15); //Turn off PD15

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR &=~ ((1<<12)|(1<<13)|(1<<14));

	continuity_test();



	break;



	case 2: //001

	GPIOD->ODR &= ~(1<<15); //Turn off PD15

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR |= (1<<12);

	GPIOD->ODR &=~((1<<13)|(1<<14));

	continuity_test();



	break;



	case 3: //010

	GPIOD->ODR &= ~(1<<15); //Turn off PD15

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR |= (1<<13);

	GPIOD->ODR &=~((1<<12)|(1<<14));

	continuity_test();

	break;



	case 4: //011

	GPIOD->ODR &= ~(1<<15); //Turn off PD15

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR &=~ (1<<14);

	GPIOD->ODR |=((1<<12)|(1<<13));

	continuity_test();

	break;



	case 5: //100

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR |= (1<<14);

	GPIOD->ODR &=~((1<<12)|(1<<13));

	continuity_test();

	break;



	case 6: //101

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR &=~ (1<<13);

	GPIOD->ODR |= ((1<<14)|(1<<12));

	continuity_test();

	break;



	case 7: //110

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR &=~ (1<<12);

	GPIOD->ODR |= ((1<<13)|(1<<14));

	continuity_test();

	break;



	case 8: //111

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR |= (1<<12);

	GPIOD->ODR |= ((1<<13)|(1<<14));

	continuity_test();

	break;



	case 9:

	GPIOD->ODR |= (1<<5);

	GPIOD->ODR &=~((1<<4)|(1<<6));



	GPIOD->ODR &=~ ((1<<12)|(1<<13)|(1<<14));

	continuity_test();

	break;



	case 10:

	//001

	GPIOD->ODR |= (1<<5);

	GPIOD->ODR &=~((1<<4)|(1<<6));



	GPIOD->ODR |= (1<<12);

	GPIOD->ODR &=~((1<<13)|(1<<14));

	continuity_test();

	break;



	case 11:

	//010

	GPIOD->ODR |= (1<<5);

	GPIOD->ODR &=~((1<<4)|(1<<6));



	GPIOD->ODR |= (1<<13);

	GPIOD->ODR &=~((1<<12)|(1<<14));

	continuity_test();

	break;



	case 12:

	//011

	GPIOD->ODR |= (1<<5);

	GPIOD->ODR &=~((1<<4)|(1<<6));



	GPIOD->ODR &=~ (1<<14);

	GPIOD->ODR |=((1<<12)|(1<<13));

	continuity_test();

	break;



	case 13:

	//100

	GPIOD->ODR |= (1<<5);

	GPIOD->ODR &=~((1<<4)|(1<<6));



	GPIOD->ODR |= (1<<14);

	GPIOD->ODR &=~((1<<12)|(1<<13));

	continuity_test();

	break;



	case 14:

	//101

	GPIOD->ODR |= (1<<5);

	GPIOD->ODR &=~((1<<4)|(1<<6));



	GPIOD->ODR &=~ (1<<13);

	GPIOD->ODR |= ((1<<14)|(1<<12));

	continuity_test();

	break;



	case 15:

	//110

	GPIOD->ODR |= (1<<5);

	GPIOD->ODR &=~((1<<4)|(1<<6));



	GPIOD->ODR &=~ (1<<12);

	GPIOD->ODR |= ((1<<13)|(1<<14));

	continuity_test();

	break;



	case 16:

	//111

	GPIOD->ODR |= (1<<5);

	GPIOD->ODR &=~((1<<4)|(1<<6));



	GPIOD->ODR |= (1<<12);

	GPIOD->ODR |= ((1<<13)|(1<<14));

	continuity_test();

	break;



	case 17:

	//000

	GPIOD->ODR |= (1<<6);

	GPIOD->ODR &=~((1<<5)|(1<<4));



	GPIOD->ODR &=~ ((1<<12)|(1<<13)|(1<<14));

	continuity_test();

	break;



	case 18:

	//001

	GPIOD->ODR |= (1<<6);

	GPIOD->ODR &=~((1<<5)|(1<<4));



	GPIOD->ODR |= (1<<12);

	GPIOD->ODR &=~((1<<13)|(1<<14));

	continuity_test();

	break;



	case 19:

	//010

	GPIOD->ODR |= (1<<6);

	GPIOD->ODR &=~((1<<5)|(1<<4));



	GPIOD->ODR |= (1<<13);

	GPIOD->ODR &=~((1<<12)|(1<<14));

	continuity_test();

	break;



	case 20:

	//011

	GPIOD->ODR |= (1<<6);

	GPIOD->ODR &=~((1<<5)|(1<<4));



	GPIOD->ODR &=~ (1<<14);

	GPIOD->ODR |=((1<<12)|(1<<13));

	continuity_test();

	break;



	case 21: //100

	GPIOD->ODR |= (1<<6);

	GPIOD->ODR &=~((1<<5)|(1<<4));



	GPIOD->ODR |= (1<<14);

	GPIOD->ODR &=~((1<<12)|(1<<13));

	continuity_test();

	break;



	case 22: //101

	GPIOD->ODR |= (1<<6);

	GPIOD->ODR &=~((1<<5)|(1<<4));



	GPIOD->ODR &=~ (1<<13);

	GPIOD->ODR |= ((1<<14)|(1<<12));

	continuity_test();

	break;



	case 23: //110

	GPIOD->ODR |= (1<<6);

	GPIOD->ODR &=~((1<<5)|(1<<4));



	GPIOD->ODR &=~ (1<<12);

	GPIOD->ODR |= ((1<<13)|(1<<14));

	continuity_test();

	break;



	case 24: //111

	GPIOD->ODR |= (1<<6);

	GPIOD->ODR &=~((1<<5)|(1<<4));



	GPIOD->ODR |= (1<<12);

	GPIOD->ODR |= ((1<<13)|(1<<14));

	continuity_test();

	break;



	}

	//GPIOD->ODR &= ~(1<<15);

	if(state == 24)

	state = 1;

	else

	state = state + 1;

	}

	last_button = button;





	}



}
