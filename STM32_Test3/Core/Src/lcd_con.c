/*
 * lcd_con.c
 * OPTIMIZED VERSION - Removed unnecessary blocking delays
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

    // --- ĐÃ XÓA HAL_Delay(20) Ở ĐÂY ---
    // Chip LCD xử lý lệnh thường chỉ mất < 50us, thời gian truyền I2C đã đủ rồi.
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

    // --- ĐÃ XÓA HAL_Delay(20) Ở ĐÂY ---
    // Đây là nguyên nhân chính gây chậm (in chuỗi 10 ký tự mất 200ms)
}

/* ===== PUBLIC FUNCTIONS ===== */
/*
 * Khởi tạo LCD
 */
void lcd_init(I2C_HandleTypeDef *hi2c)
{
    HAL_Delay(50); // Chờ điện áp ổn định (Bắt buộc)

    // Khởi động chuẩn HD44780 (CHỈ GỬI NIBBLE CAO)
    lcd_send_cmd(hi2c, 0x30);
    HAL_Delay(5); // Cần thiết lúc khởi động
    lcd_send_cmd(hi2c, 0x30);
    HAL_Delay(1); // Cần thiết lúc khởi động
    lcd_send_cmd(hi2c, 0x30);
    HAL_Delay(10);
    lcd_send_cmd(hi2c, 0x20); // Chuyển sang 4-bit mode
    HAL_Delay(10);

    // Cấu hình LCD
    lcd_send_cmd(hi2c, 0x28); // 4-bit, 2 dòng
    lcd_send_cmd(hi2c, 0x08); // display off
    lcd_send_cmd(hi2c, 0x01); // clear
    HAL_Delay(2);             // Lệnh Clear cần > 1.52ms
    lcd_send_cmd(hi2c, 0x06); // entry mode
    lcd_send_cmd(hi2c, 0x0C); // display ON
}

/*
 * Xóa LCD
 */
void lcd_clear(I2C_HandleTypeDef *hi2c)
{
    lcd_send_cmd(hi2c, 0x01);
    HAL_Delay(2); // Lệnh Clear màn hình bắt buộc phải chờ > 1.5ms
}

/*
 * Di chuyển con trỏ
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
 * Hiển thị chuỗi
 */
void lcd_puts(I2C_HandleTypeDef *hi2c, char *str)
{
    while (*str)
        lcd_send_data(hi2c, *str++);
}

/*
 * Ghi 1 kí tự
 */
void lcd_putchar(I2C_HandleTypeDef *hi2c, char ch)
{
    lcd_send_data(hi2c, ch);
}
