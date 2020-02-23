#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_DELAY_50US(n)   ets_delay_us(50 * n)

typedef struct {
    uint8_t lcd_bit_width;
    uint8_t lcd_ws_pin;
    uint8_t lcd_rs_pin;
    uint8_t lcd_rd_pin;
    uint8_t lcd_data_pin[24];
    uint8_t cam_bit_width;
    uint8_t cam_xclk_pin;
    uint8_t cam_pclk_pin;
    uint8_t cam_vsync_pin;
    uint8_t cam_hsync_pin;
    uint8_t cam_data_pin[16];
} lcd_cam_config_t;

void lcd_write_data(uint8_t *data, size_t len);

void lcd_set_index(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end);

void lcd_cam_init(const lcd_cam_config_t *config);

uint8_t* cam_attach(void);
void cam_stop(void);
void cam_start(void);
void take_fram_lock(void);
void give_fram_lock(void);

#ifdef __cplusplus
}
#endif

