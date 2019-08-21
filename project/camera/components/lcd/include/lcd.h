#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_PIN_CLK  35
#define LCD_PIN_MOSI 37
#define LCD_PIN_DC   36
#define LCD_PIN_RST  39

#define LCD_PIN_CS0  38
#define LCD_PIN_CS1  41

#define LCD_PIN_BCKL 40

#define LCD0_HORIZONTAL 2
#define LCD1_HORIZONTAL 0

void inline lcd_set_res(uint8_t state)
{
    gpio_set_level(LCD_PIN_RST, state);
}

void inline lcd_set_dc(uint8_t state)
{
    gpio_set_level(LCD_PIN_DC, state);
}

void inline lcd_set_cs0(uint8_t state)
{
    gpio_set_level(LCD_PIN_CS0, state);
}

void inline lcd_set_cs1(uint8_t state)
{
    gpio_set_level(LCD_PIN_CS1, state);
}

void inline lcd_set_blk(uint8_t state)
{
    gpio_set_level(LCD_PIN_BCKL, state);
}


void lcd_rst();

void lcd_init();

void lcd_write_data(uint16_t *data, size_t len);

void lcd_take(uint8_t num);

void lcd_give();

void lcd_set_index(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);

#ifdef __cplusplus
}
#endif