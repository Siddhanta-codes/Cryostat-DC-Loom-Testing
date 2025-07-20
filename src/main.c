#include "LCD.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "stdlib.h"
#include "ADC.h"


void continuity_test(){
	//lcd_clear();
	char buffer[10];
	GPIOD->ODR &=~ (1<<15);

	ADC1->CR2 |= ADC_CR2_ADON;


	//uint16_t adc_value = adc_read();
    uint16_t adc_value_ch8 = adc_read_CH8();
	itoa(adc_value_ch8, buffer, 10);  // Convert adc_value to ASCII (base 10)
	USART2_SendString("ADC Value: \r\n");
	USART2_SendString(buffer);
	if (adc_value_ch8 > 4000) {

	GPIOD->ODR |= (1 << 15);
	USART2_SendString("Connected \r\n");
	lcd_puts("Closed");
	//lcd_clear();

	} else {

	GPIOD->ODR &=~ (1 << 15);
	USART2_SendString("Not Connected \r\n");
	lcd_puts("Open");
	//lcd_clear();
	}

	ADC1->CR2 &=~ ADC_CR2_ADON;

}

void continuity_resistance_test(){
	//lcd_clear();
		adc_init();
	    ADC1_CH8_Init();
		char buffer[10];
		GPIOD->ODR &=~ (1<<15);

		ADC1->CR2 |= ADC_CR2_ADON;

	    uint16_t adc_value_ch8 = adc_read_CH8();
		itoa(adc_value_ch8, buffer, 10);  // Convert adc_value to ASCII (base 10)
		USART2_SendString("ADC Value: \r\n");
		USART2_SendString(buffer);
		if (adc_value_ch8 > 4000) {

		GPIOD->ODR |= (1 << 15);
		USART2_SendString("Connected \r\n");
		lcd_puts("Closed");
		//lcd_clear();

		//Setting PB0 as a regular output pin set to ground
		GPIOB->MODER &= ~(3 << (0 * 2));  // Clear MODER bits
		GPIOB->MODER |=  (1 << (0 * 2));  // Set PB0 to general purpose output

		// Then, set PB0 output type and pull-down if needed (optional)
		GPIOB->OTYPER &= ~(1 << 0);       // Push-pull output
		GPIOB->PUPDR &= ~(3 << (0 * 2));  // No pull-up/pull-down

		// Drive PB0 LOW
		GPIOB->ODR &= ~(1 << 0);
		ADC1->SQR3 = 1;
		ADC1->SQR1 = 0 << 20;
		uint16_t adc_value = adc_read();
		// Use float for voltage and resistance calculation
		 char buffer_adc[10];
		 itoa(adc_value, buffer_adc, 10);
		 USART2_SendString(buffer_adc);
		 USART2_SendString("\r\n");
		float voltage = (adc_value / 4095.0f) * 3.3f;
		if (voltage <= 0.1) {
		        USART2_SendString("Error: voltage too low\r\n");
		        return;
		    }
		    else{
		    float resistance = 56.0f * (voltage / (3.0f - voltage));

		    // Convert float resistance to string
		    char buffer_resistance[32];
		    sprintf(buffer_resistance, "%.2f ohms\r\n", resistance);
		    //itoa(resistance, buffer_resistance, 32);
		    USART2_SendString(buffer_resistance);
		    lcd_goto(1,0);
		    lcd_puts("R = ");
		    lcd_puts(buffer_resistance);
		    }


		} else {

		GPIOD->ODR &=~ (1 << 15);
		USART2_SendString("Not Connected \r\n");
		lcd_puts("Open");
		//lcd_clear();
		}

		ADC1->CR2 &=~ ADC_CR2_ADON;
}



int main(void){

	adc_init();
    ADC1_CH8_Init();
	gpiod_init();
	USART2_Init();
	I2C1_init();
	lcd_init();


	uint8_t state = 1;
	uint8_t last_button = 0;
	GPIOA->ODR &=~(1<<1);


	while(1){



	uint8_t button = read_button();

	if(button == 1 && last_button == 0){
	lcd_clear();
	char buffer_state[10];
	itoa(state, buffer_state, 10);
	lcd_goto(0,0);
	USART2_SendString("Wire ");
	lcd_puts("Wire");
	USART2_SendString(buffer_state);
	lcd_puts(buffer_state);
	USART2_SendString(" ");
	lcd_puts(" ");
	switch(state){

	case 1: //000

	GPIOD->ODR &= ~(1<<15); //Turn off PD15

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR &=~ ((1<<12)|(1<<13)|(1<<14));

	continuity_resistance_test();
	break;



	case 2: //001

	GPIOD->ODR &= ~(1<<15); //Turn off PD15

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR |= (1<<12);

	GPIOD->ODR &=~((1<<13)|(1<<14));

	continuity_resistance_test();


	break;



	case 3: //010

	GPIOD->ODR &= ~(1<<15); //Turn off PD15

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR |= (1<<13);

	GPIOD->ODR &=~((1<<12)|(1<<14));

	continuity_resistance_test();
	break;



	case 4: //011

	GPIOD->ODR &= ~(1<<15); //Turn off PD15

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR &=~ (1<<14);

	GPIOD->ODR |=((1<<12)|(1<<13));

	continuity_resistance_test();

	break;



	case 5: //100

	GPIOD->ODR |= (1<<4);

	GPIOD->ODR &=~((1<<5)|(1<<6));



	GPIOD->ODR |= (1<<14);

	GPIOD->ODR &=~((1<<12)|(1<<13));

	continuity_resistance_test();

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
