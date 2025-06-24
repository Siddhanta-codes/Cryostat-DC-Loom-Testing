/*
 * LCD.c
 *
 *  Created on: Jun 19, 2025
 *      Author: Siddhanta
 */

#include <LCD.h>

// Send one byte over I2C
void lcd_write_i2c(uint8_t data) {
    I2C1_start(LCD_ADDR);
    I2C1_write(data);
    I2C1_stop();
}

void lcd_pulse(uint8_t data) {
    lcd_write_i2c(data | EN | BACKLIGHT);
    for (volatile int i = 0; i < 2000; i++);
    lcd_write_i2c((data & ~EN) | BACKLIGHT);
    for (volatile int i = 0; i < 2000; i++);
}

void lcd_send_nibble(uint8_t nibble, uint8_t control) {
    uint8_t data = (nibble << 4) | control;
    lcd_pulse(data);
}

void lcd_send_byte(uint8_t byte, uint8_t control) {
    lcd_send_nibble(byte >> 4, control);
    lcd_send_nibble(byte & 0x0F, control);
}

void lcd_init(void) {
    for (volatile int i = 0; i < 100000; i++);
    lcd_send_nibble(0x03, 0);
    for (volatile int i = 0; i < 50000; i++);
    lcd_send_nibble(0x03, 0);
    for (volatile int i = 0; i < 50000; i++);
    lcd_send_nibble(0x03, 0);
    for (volatile int i = 0; i < 50000; i++);
    lcd_send_nibble(0x02, 0);

    lcd_send_byte(0x28, 0); // 4-bit, 2 line
    lcd_send_byte(0x0C, 0); // Display ON
    lcd_send_byte(0x06, 0); // Entry mode
    lcd_send_byte(0x01, 0); // Clear
}

void lcd_putc(char c) {
    lcd_send_byte(c, RS);
}

void lcd_puts(const char *str) {
    while (*str) lcd_putc(*str++);
}

void lcd_goto(uint8_t row, uint8_t col) {
    lcd_send_byte((row == 0 ? 0x80 : 0xC0) + col, 0);
}

void lcd_clear(void) {
    lcd_send_byte(0x01, 0);  // 0x01 = Clear display command
    for (volatile int i = 0; i < 50000; i++);  // Short delay after clear
}

// I2C1 config & routines
void I2C1_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOB->MODER |= (2 << (6 * 2)) | (2 << (7 * 2));

    GPIOB->OTYPER |= (1 << 6) | (1 << 7);
    GPIOB->OSPEEDR |= (3 << (6 * 2)) | (3 << (7 * 2));
    GPIOB->PUPDR |= (1 << (6 * 2)) | (1 << (7 * 2));

    GPIOB->AFR[0] |= (4 << (6 * 4)) | (4 << (7 * 4));

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_start(uint8_t addr) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    (void)I2C1->SR1;
    I2C1->DR = addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
}

void I2C1_write(uint8_t data) {
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}

void I2C1_stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}


