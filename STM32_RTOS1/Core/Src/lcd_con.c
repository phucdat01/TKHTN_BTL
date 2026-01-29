/*
 * lcd_con.c
 *
 *  Created on: Jan 1, 2026
 *      Author: Kamen Rider Remz Technolom Drill
 */
#include "lcd_con.h"
#include <stdio.h>
/* ===== INTERNAL FUNCTIONS ===== */

/*
 * Check địa chỉ I2C được sử dụng
 */
void I2C_Scan(I2C_HandleTypeDef *hi2c, char msg[])
{
    for(uint8_t addr = 1; addr < 128; addr++)
    {
        if(HAL_I2C_IsDeviceReady(hi2c, addr << 1, 2, 10) == HAL_OK)
        {
            sprintf(msg, "Found I2C at 0x%02X\r\n", addr);
        }
    }
}

/*static void lcd_send_4bit(I2C_HandleTypeDef *hi2c, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = data | LCD_EN | LCD_BACKLIGHT;
    buf[1] = data | LCD_BACKLIGHT;
    HAL_I2C_Master_Transmit(hi2c, LCD_I2C_ADDR, buf, 2, 100);
}*/
/*
 * Truyền lệnh qua i2c
 */
static void lcd_send_cmd(I2C_HandleTypeDef *hi2c, uint8_t cmd)
{
    uint8_t data[4];

    data[0] = (cmd & 0xF0) | LCD_EN | LCD_BACKLIGHT;
    data[1] = (cmd & 0xF0) | LCD_BACKLIGHT;
    data[2] = ((cmd << 4) & 0xF0) | LCD_EN | LCD_BACKLIGHT;
    data[3] = ((cmd << 4) & 0xF0) | LCD_BACKLIGHT;

    HAL_I2C_Master_Transmit(hi2c, LCD_I2C_ADDR, data, 4, 100);
    HAL_Delay(20);

}

/*
 * TRuyền dữ liệu hiển thị lên LCD
 */
static void lcd_send_data(I2C_HandleTypeDef *hi2c, uint8_t data)
{
    uint8_t buf[4];

    buf[0] = (data & 0xF0) | LCD_RS | LCD_EN | LCD_BACKLIGHT;
    buf[1] = (data & 0xF0) | LCD_RS | LCD_BACKLIGHT;
    buf[2] = ((data << 4) & 0xF0) | LCD_RS | LCD_EN | LCD_BACKLIGHT;
    buf[3] = ((data << 4) & 0xF0) | LCD_RS | LCD_BACKLIGHT;

    HAL_I2C_Master_Transmit(hi2c, LCD_I2C_ADDR, buf, 4, 100);
    HAL_Delay(20);
}

/* ===== PUBLIC FUNCTIONS ===== */
/*
 * Khởi tạo LCD
 */
void lcd_init(I2C_HandleTypeDef *hi2c)
{
    HAL_Delay(50);

    // Khởi động chuẩn HD44780 (CHỈ GỬI NIBBLE CAO)
    lcd_send_cmd(hi2c, 0x30);
    lcd_send_cmd(hi2c, 0x20);

    // Cấu hình LCD
    lcd_send_cmd(hi2c, 0x28); // 4-bit, 2 dòng
    lcd_send_cmd(hi2c, 0x08); // display off
    lcd_send_cmd(hi2c, 0x01); // clear
    HAL_Delay(5);
    lcd_send_cmd(hi2c, 0x06); // entry mode
    lcd_send_cmd(hi2c, 0x0C); // display ON
}

/*
 * Xóa LCD
 */
void lcd_clear(I2C_HandleTypeDef *hi2c)
{
    lcd_send_cmd(hi2c, 0x01);
    HAL_Delay(2);
}

/*
 * Di chuyển con tro LCD đến vị trí cụ thể
 * 0 là dòng 1
 * 1 là dòng 2
 * Mỗi dòng có thể ghi 16 kí tự
 * nên nó là LCD16x2
 */
void lcd_gotoxy(I2C_HandleTypeDef *hi2c, uint8_t col, uint8_t row)
{
    uint8_t addr;

    switch (row)
    {
        case 0: addr = 0x00 + col; break;
        case 1: addr = 0x40 + col; break;
        default: return;
    }

    lcd_send_cmd(hi2c, 0x80 | addr);
}

/*
 * Hiển thị chuỗi lên LCD chỉ nên là 16 nhiều quá nó bị ngu ko là tao phải chỉnh lại code
 * nên làm thêm biến điếm số kí tự đã hiển thi nó phải sống mãi
 */
void lcd_puts(I2C_HandleTypeDef *hi2c, char *str)
{
    while (*str)
        lcd_send_data(hi2c, *str++);
}

/*
 * Chỉ ghi 1 kí tự
 */
void lcd_putchar(I2C_HandleTypeDef *hi2c, char ch)
{
    lcd_send_data(hi2c, ch);
}
