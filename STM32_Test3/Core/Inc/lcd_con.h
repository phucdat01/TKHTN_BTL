/*
 * MPU6500.c
 *
 *  Created on: Jan 1, 2026
 *      Author: Kamen Rider Remz Esprim Armor
 */
#ifndef __LCD_I2C_H
#define __LCD_I2C_H

#include "stm32f1xx_hal.h"

/* ================= USER CONFIG ================= */

// Địa chỉ PCF8574 (NHỚ << 1)
#define LCD_I2C_ADDR      (0x27 << 1)   // đổi 0x3F nếu module khác

// Bit mapping PCF8574 → LCD
#define LCD_RS            0x01
#define LCD_RW            0x02
#define LCD_EN            0x04
#define LCD_BACKLIGHT     0x08

/* ================= API ================= */

void lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_clear(I2C_HandleTypeDef *hi2c);
void lcd_gotoxy(I2C_HandleTypeDef *hi2c, uint8_t col, uint8_t row);
void lcd_puts(I2C_HandleTypeDef *hi2c, char *str);
void lcd_putchar(I2C_HandleTypeDef *hi2c, char ch);

#endif
