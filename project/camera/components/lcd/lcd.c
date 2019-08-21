
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "lcd.h"

static spi_device_handle_t spi;
static SemaphoreHandle_t lcd_write_mux = NULL;
static SemaphoreHandle_t lcd_mux = NULL;
static uint8_t lcd_dc_state = 0;
static uint8_t lcd_num = 0;

static uint16_t *lcd_buffer = NULL;
#define LCD_BUFFER_SIZE (32 * 1024)

#define SPI_BURST_MAX_LEN (LCD_BUFFER_SIZE)  // Maximum pixel data transferred at a time

static void IRAM_ATTR spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    lcd_set_dc(lcd_dc_state);
}

static void spi_write_data(uint16_t *data, size_t len)
{
    if (len <= 0) {
        return;
    }
    int x, y;
    spi_transaction_t t, *rtrans;
    memset(&t, 0, sizeof(t));         //Zero out the transaction      
    t.tx_buffer = lcd_buffer;        
    t.length = 8 * SPI_BURST_MAX_LEN; 
    for (y = 0; y < len / SPI_BURST_MAX_LEN; y++) {
        memcpy(lcd_buffer, data, SPI_BURST_MAX_LEN);
        spi_device_queue_trans(spi, &t, portMAX_DELAY);
        spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        data += SPI_BURST_MAX_LEN / 2;
    }
    if (len % SPI_BURST_MAX_LEN) {
        t.length = 8 * (len % SPI_BURST_MAX_LEN); 
        memcpy(lcd_buffer, data, len % SPI_BURST_MAX_LEN);
        spi_device_queue_trans(spi, &t, portMAX_DELAY);
        spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
    }
}


static void lcd_delay_ms(uint32_t time)
{
    vTaskDelay(time / portTICK_RATE_MS);
}

void lcd_take(uint8_t num)
{
    xSemaphoreTake(lcd_mux, portMAX_DELAY);
    xSemaphoreTake(lcd_write_mux, portMAX_DELAY);
    switch (num) {
        case 0: {
            lcd_set_cs1(1);
            lcd_set_cs0(0);
            lcd_num = 0;
        }
        break;

        case 1: {
            lcd_set_cs0(1);
            lcd_set_cs1(0);
            lcd_num = 1;
        }
        break;
    }
    xSemaphoreGive(lcd_write_mux);
}

void lcd_give()
{
    xSemaphoreGive(lcd_mux);
}

static void lcd_write_cmd(uint8_t data)
{
    xSemaphoreTake(lcd_write_mux, portMAX_DELAY);
    lcd_dc_state = 0;
    spi_write_data(&data, 1);
    xSemaphoreGive(lcd_write_mux);
}

static void lcd_write_byte(uint8_t data)
{
    xSemaphoreTake(lcd_write_mux, portMAX_DELAY);
    lcd_dc_state = 1;
    spi_write_data(&data, 1);
    xSemaphoreGive(lcd_write_mux);
}

void lcd_write_data(uint16_t *data, size_t len)
{
    if (len <= 0) {
        return;
    }
    xSemaphoreTake(lcd_write_mux, portMAX_DELAY);
    lcd_dc_state = 1;
    spi_write_data(data, len);
    xSemaphoreGive(lcd_write_mux);
}

void lcd_rst()
{
    lcd_set_res(0);
    lcd_delay_ms(100);
    lcd_set_res(1);
    lcd_delay_ms(100);
}

static void lcd_240x240_config()
{
    lcd_take(0);

    lcd_write_cmd(0x36); // MADCTL (36h): Memory Data Access Control
#if (LCD0_HORIZONTAL == 0)
    lcd_write_byte(0x00);
#elif (LCD0_HORIZONTAL == 1)
    lcd_write_byte(0xC0);
#elif (LCD0_HORIZONTAL == 2)
    lcd_write_byte(0x70);
#elif (LCD0_HORIZONTAL == 3)
    lcd_write_byte(0xA0);
#endif

    lcd_write_cmd(0x3A);  // COLMOD (3Ah): Interface Pixel Format 
    lcd_write_byte(0x05);

    lcd_write_cmd(0xB2); // PORCTRL (B2h): Porch Setting 
    lcd_write_byte(0x0C);
    lcd_write_byte(0x0C);
    lcd_write_byte(0x00);
    lcd_write_byte(0x33);
    lcd_write_byte(0x33); 

    lcd_write_cmd(0xB7); // GCTRL (B7h): Gate Control 
    lcd_write_byte(0x35);  

    lcd_write_cmd(0xBB); // VCOMS (BBh): VCOM Setting 
    lcd_write_byte(0x19);

    lcd_write_cmd(0xC0); // LCMCTRL (C0h): LCM Control 
    lcd_write_byte(0x2C);

    lcd_write_cmd(0xC2); // VDVVRHEN (C2h): VDV and VRH Command Enable
    lcd_write_byte(0x01);

    lcd_write_cmd(0xC3); // VRHS (C3h): VRH Set
    lcd_write_byte(0x12);   

    lcd_write_cmd(0xC4); // VDVS (C4h): VDV Set 
    lcd_write_byte(0x20);  

    lcd_write_cmd(0xC6); // FRCTRL2 (C6h): Frame Rate Control in Normal Mode 
    lcd_write_byte(0x0F);    

    lcd_write_cmd(0xD0); // PWCTRL1 (D0h): Power Control 1 
    lcd_write_byte(0xA4);
    lcd_write_byte(0xA1);

    lcd_write_cmd(0xE0); // PVGAMCTRL (E0h): Positive Voltage Gamma Control
    lcd_write_byte(0xD0);
    lcd_write_byte(0x04);
    lcd_write_byte(0x0D);
    lcd_write_byte(0x11);
    lcd_write_byte(0x13);
    lcd_write_byte(0x2B);
    lcd_write_byte(0x3F);
    lcd_write_byte(0x54);
    lcd_write_byte(0x4C);
    lcd_write_byte(0x18);
    lcd_write_byte(0x0D);
    lcd_write_byte(0x0B);
    lcd_write_byte(0x1F);
    lcd_write_byte(0x23);

    lcd_write_cmd(0xE1); // NVGAMCTRL (E1h): Negative Voltage Gamma Control
    lcd_write_byte(0xD0);
    lcd_write_byte(0x04);
    lcd_write_byte(0x0C);
    lcd_write_byte(0x11);
    lcd_write_byte(0x13);
    lcd_write_byte(0x2C);
    lcd_write_byte(0x3F);
    lcd_write_byte(0x44);
    lcd_write_byte(0x51);
    lcd_write_byte(0x2F);
    lcd_write_byte(0x1F);
    lcd_write_byte(0x1F);
    lcd_write_byte(0x20);
    lcd_write_byte(0x23);

    lcd_write_cmd(0x21); // INVON (21h): Display Inversion On

    lcd_write_cmd(0x11); // SLPOUT (11h): Sleep Out 

    lcd_write_cmd(0x29); // DISPON (29h): Display On

    lcd_give();
}

static void lcd_240x135_config()
{
    lcd_take(1);

    lcd_write_cmd(0x36); // MADCTL (36h): Memory Data Access Control
#if (LCD1_HORIZONTAL == 0)
    lcd_write_byte(0x00);
#elif (LCD1_HORIZONTAL == 1)
    lcd_write_byte(0xC0);
#elif (LCD1_HORIZONTAL == 2)
    lcd_write_byte(0x70);
#elif (LCD1_HORIZONTAL == 3)
    lcd_write_byte(0xA0);
#endif

    lcd_write_cmd(0x3A);  // COLMOD (3Ah): Interface Pixel Format 
    lcd_write_byte(0x05);

    lcd_write_cmd(0xB2); // PORCTRL (B2h): Porch Setting 
    lcd_write_byte(0x0C);
    lcd_write_byte(0x0C);
    lcd_write_byte(0x00);
    lcd_write_byte(0x33);
    lcd_write_byte(0x33); 

    lcd_write_cmd(0xB7); // GCTRL (B7h): Gate Control 
    lcd_write_byte(0x35);  

    lcd_write_cmd(0xBB); // VCOMS (BBh): VCOM Setting 
    lcd_write_byte(0x19);

    lcd_write_cmd(0xC0); // LCMCTRL (C0h): LCM Control 
    lcd_write_byte(0x2C);

    lcd_write_cmd(0xC2); // VDVVRHEN (C2h): VDV and VRH Command Enable
    lcd_write_byte(0x01);

    lcd_write_cmd(0xC3); // VRHS (C3h): VRH Set
    lcd_write_byte(0x12);   

    lcd_write_cmd(0xC4); // VDVS (C4h): VDV Set 
    lcd_write_byte(0x20);  

    lcd_write_cmd(0xC6); // FRCTRL2 (C6h): Frame Rate Control in Normal Mode 
    lcd_write_byte(0x0F);    

    lcd_write_cmd(0xD0); // PWCTRL1 (D0h): Power Control 1 
    lcd_write_byte(0xA4);
    lcd_write_byte(0xA1);

    lcd_write_cmd(0xE0); // PVGAMCTRL (E0h): Positive Voltage Gamma Control
    lcd_write_byte(0xD0);
    lcd_write_byte(0x04);
    lcd_write_byte(0x0D);
    lcd_write_byte(0x11);
    lcd_write_byte(0x13);
    lcd_write_byte(0x2B);
    lcd_write_byte(0x3F);
    lcd_write_byte(0x54);
    lcd_write_byte(0x4C);
    lcd_write_byte(0x18);
    lcd_write_byte(0x0D);
    lcd_write_byte(0x0B);
    lcd_write_byte(0x1F);
    lcd_write_byte(0x23);

    lcd_write_cmd(0xE1); // NVGAMCTRL (E1h): Negative Voltage Gamma Control
    lcd_write_byte(0xD0);
    lcd_write_byte(0x04);
    lcd_write_byte(0x0C);
    lcd_write_byte(0x11);
    lcd_write_byte(0x13);
    lcd_write_byte(0x2C);
    lcd_write_byte(0x3F);
    lcd_write_byte(0x44);
    lcd_write_byte(0x51);
    lcd_write_byte(0x2F);
    lcd_write_byte(0x1F);
    lcd_write_byte(0x1F);
    lcd_write_byte(0x20);
    lcd_write_byte(0x23);

    lcd_write_cmd(0x21); // INVON (21h): Display Inversion On

    lcd_write_cmd(0x11); // SLPOUT (11h): Sleep Out 

    lcd_write_cmd(0x29); // DISPON (29h): Display On

    lcd_give();
}

void lcd_init()
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = LCD_PIN_MOSI,
        .sclk_io_num = LCD_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_BURST_MAX_LEN
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 80*1000*1000,           //Clock out at 10 MHz
        .mode = 0,                                //SPI mode 0
        .spics_io_num = -1,                       //CS pin
        .queue_size = 1,                          //We want to be able to queue 1 transactions at a time
        .pre_cb = spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
        .flags = SPI_DEVICE_HALFDUPLEX
    };

    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, HSPI_HOST);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

#ifdef LCD_BUFFER_SIZE
    lcd_buffer = (uint16_t *)heap_caps_malloc(LCD_BUFFER_SIZE, MALLOC_CAP_DMA);
#endif

    //Initialize non-SPI GPIOs
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LCD_PIN_DC) | (1ULL << LCD_PIN_RST) | (1ULL << LCD_PIN_CS0) | (1ULL << LCD_PIN_CS1) | (1ULL << LCD_PIN_BCKL);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    lcd_set_cs0(1);
    lcd_set_cs1(1);
    
    lcd_write_mux = xSemaphoreCreateMutex();
    lcd_mux = xSemaphoreCreateMutex();

    lcd_rst();//lcd_rst before LCD Init.
    lcd_delay_ms(100);
    lcd_240x240_config();
    lcd_240x135_config();

    lcd_set_blk(0);
    printf("lcd init ok\n");
}

void lcd_set_index(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    uint16_t start_pos, end_pos;
    switch (lcd_num) {
        case 0: {
            lcd_write_cmd(0x2a);    // CASET (2Ah): Column Address Set 
            // Must write byte than byte
#if (LCD0_HORIZONTAL == 3)
            start_pos = x_start + 80;
            end_pos = x_end + 80;
#else
            start_pos = x_start;
            end_pos = x_end;
#endif
            lcd_write_byte(start_pos >> 8);
            lcd_write_byte(start_pos & 0xFF);
            lcd_write_byte(end_pos >> 8);
            lcd_write_byte(end_pos & 0xFF);

            lcd_write_cmd(0x2b);    // RASET (2Bh): Row Address Set
#if (LCD0_HORIZONTAL == 1)
            start_pos = x_start + 80;
            end_pos = x_end + 80;
#else
            start_pos = y_start;
            end_pos = y_end;
#endif
            start_pos = y_start;
            end_pos = y_end; 
            lcd_write_byte(start_pos >> 8);
            lcd_write_byte(start_pos & 0xFF);
            lcd_write_byte(end_pos >> 8);
            lcd_write_byte(end_pos & 0xFF); 
            lcd_write_cmd(0x2c);    // RAMWR (2Ch): Memory Write 
        }
        break;

        case 1: {
            lcd_write_cmd(0x2a);    // CASET (2Ah): Column Address Set 
            // Must write byte than byte
#if (LCD1_HORIZONTAL == 0)
            start_pos = x_start + 52;
            end_pos = x_end + 52;
#elif (LCD1_HORIZONTAL == 1)
            start_pos = x_start + 53;
            end_pos = x_end + 53;
#elif (LCD1_HORIZONTAL == 2)
            start_pos = x_start + 40;
            end_pos = x_end + 40;
#elif (LCD1_HORIZONTAL == 3)
            start_pos = x_start + 40;
            end_pos = x_end + 40;
#endif
            lcd_write_byte(start_pos >> 8);
            lcd_write_byte(start_pos & 0xFF);
            lcd_write_byte(end_pos >> 8);
            lcd_write_byte(end_pos & 0xFF);

            lcd_write_cmd(0x2b);    // RASET (2Bh): Row Address Set 
#if (LCD1_HORIZONTAL == 0)
            start_pos = y_start + 40;
            end_pos = y_end + 40;
#elif (LCD1_HORIZONTAL == 1)
            start_pos = y_start + 40;
            end_pos = y_end + 40;
#elif (LCD1_HORIZONTAL == 2)
            start_pos = y_start + 53;
            end_pos = y_end + 53;
#elif (LCD1_HORIZONTAL == 3)
            start_pos = y_start + 52;
            end_pos = y_end + 52;
#endif
            lcd_write_byte(start_pos >> 8);
            lcd_write_byte(start_pos & 0xFF);
            lcd_write_byte(end_pos >> 8);
            lcd_write_byte(end_pos & 0xFF);  
            lcd_write_cmd(0x2c);    // RAMWR (2Ch): Memory Write 
        }
        break;
    }

}