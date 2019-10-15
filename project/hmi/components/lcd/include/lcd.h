#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*   I2S LCD interface configure part  */

#define  WR  GPIO_NUM_34
#define  RS  GPIO_NUM_1
#define  RD  GPIO_NUM_2

#define  D0  GPIO_NUM_35
#define  D1  GPIO_NUM_37
#define  D2  GPIO_NUM_36
#define  D3  GPIO_NUM_39
#define  D4  GPIO_NUM_38
#define  D5  GPIO_NUM_41
#define  D6  GPIO_NUM_40
#define  D7  GPIO_NUM_45

#define  D8   GPIO_NUM_21
#define  D9   GPIO_NUM_18
#define  D10  GPIO_NUM_17
#define  D11  GPIO_NUM_16
#define  D12  GPIO_NUM_15
#define  D13  GPIO_NUM_14
#define  D14  GPIO_NUM_13
#define  D15  GPIO_NUM_12

#define RS_LOW()   gpio_set_level(RS, 0)
#define RS_HIGH()  gpio_set_level(RS, 1)

#define LCD_DELAY_50US(n)   ets_delay_us(50 * n)

void lcd_write_data(uint16_t *data, size_t len);

void lcd_set_index(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end);

void lcd_init(void);

#ifdef __cplusplus
}
#endif

