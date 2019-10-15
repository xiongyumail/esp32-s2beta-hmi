#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*   I2S LCD interface configure part  */

#define  LCD_WR  GPIO_NUM_34
#define  LCD_RS  GPIO_NUM_1
#define  LCD_RD  GPIO_NUM_2

#define  LCD_D0  GPIO_NUM_35
#define  LCD_D1  GPIO_NUM_37
#define  LCD_D2  GPIO_NUM_36
#define  LCD_D3  GPIO_NUM_39
#define  LCD_D4  GPIO_NUM_38
#define  LCD_D5  GPIO_NUM_41
#define  LCD_D6  GPIO_NUM_40
#define  LCD_D7  GPIO_NUM_45

#define  LCD_D8   GPIO_NUM_21
#define  LCD_D9   GPIO_NUM_18
#define  LCD_D10  GPIO_NUM_17
#define  LCD_D11  GPIO_NUM_16
#define  LCD_D12  GPIO_NUM_15
#define  LCD_D13  GPIO_NUM_14
#define  LCD_D14  GPIO_NUM_13
#define  LCD_D15  GPIO_NUM_12

#define RS_LOW()   gpio_set_level(LCD_RS, 0)
#define RS_HIGH()  gpio_set_level(LCD_RS, 1)

#define LCD_DELAY_50US(n)   ets_delay_us(50 * n)

void lcd_write_data(uint16_t *data, size_t len);

void lcd_set_index(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end);

void lcd_cam_init(void);

void cam_xclk_attach(void);
uint8_t* cam_attach(void);
int sccb_slave_prob(void);
void camera_reg_cfg(void);
void cam_start(void);
void take_fram_lock(void);
void give_fram_lock(void);

#ifdef __cplusplus
}
#endif

