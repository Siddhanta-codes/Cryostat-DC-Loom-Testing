/*
 * LCD.h
 *
 *  Created on: Jun 24, 2025
 *      Author: Siddhanta
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f4xx.h"

#define LCD_ADDR (0x27 << 1)

#define RS (1 << 0)
#define RW (1 << 1)
#define EN (1 << 2)
#define BACKLIGHT (1 << 3)

// LCD functions
void lcd_init(void);
void lcd_putc(char c);
void lcd_puts(const char *str);
void lcd_goto(uint8_t row, uint8_t col);
void lcd_clear(void);

// I2C low-level
void I2C1_init(void);
void I2C1_start(uint8_t addr);
void I2C1_write(uint8_t data);
void I2C1_stop(void);


#endif /* INC_LCD_H_ */
