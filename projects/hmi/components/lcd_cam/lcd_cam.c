#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"
#include "esp_log.h"
#include "soc/i2s_struct.h"
#include "soc/apb_ctrl_reg.h"
#include "esp32s2/rom/lldesc.h"
#include "esp32s2/rom/cache.h"
#include "soc/dport_access.h"
#include "soc/dport_reg.h"
#include "driver/ledc.h"
#include "lcd_cam.h"
#include "fram_cfg.h"

static const char *TAG = "lcd_cam";

static lldesc_t *lcd_dma = NULL;
static uint8_t *lcd_buffer = NULL;
#define LCD_DMA_SIZE         (4000)
#define LCD_BUFFER_SIZE      (8000)
#define LCD_HALF_BUFFER_SIZE (LCD_BUFFER_SIZE / 2)
#define LCD_NODE_CNT         (LCD_BUFFER_SIZE / LCD_DMA_SIZE)
#define LCD_HALF_NODE_CNT    (LCD_NODE_CNT / 2)

// #define FRAM_BUF_LINE (6)
#define BUF_EOF_CNT  (FRAM_HIGH / FRAM_BUF_LINE)
#define DMA_NODE_CNT  (6)

#define USE_FRAM_LOCK    (1)
#define FRAM_BUFER_DEBUG (0)

typedef struct {
    int fram_id;
    int size;
    uint8_t buf[FRAM_BUF_SIZE];
} camera_fram_t;

typedef struct {
    lcd_cam_config_t config;
    int loop_cnt;
    int fram_id;
    xQueueHandle queue;
    SemaphoreHandle_t disp_mux;
    SemaphoreHandle_t update_mux;
    camera_fram_t *pfram[DMA_NODE_CNT];
    uint8_t *pbuf;
} lcd_cam_obj_t;

static lcd_cam_obj_t *lcd_cam_obj = NULL;

static lldesc_t cam_dma[DMA_NODE_CNT] = {0};

static SemaphoreHandle_t lcd_tx_sem = NULL;

void IRAM_ATTR lcd_cam_isr(void *arg)
{
    typeof(I2S0.int_st) int_st = I2S0.int_st;
    I2S0.int_clr.val = int_st.val;
    BaseType_t HPTaskAwoken = pdFALSE;
    lcd_cam_obj_t* obj = lcd_cam_obj;
    if (int_st.in_suc_eof) {
        obj->pfram[obj->fram_id]->fram_id = obj->loop_cnt;
        obj->pfram[obj->fram_id + 1]->fram_id = obj->loop_cnt + 1;
        xQueueSendFromISR(obj->queue, (void *)&obj->fram_id, &HPTaskAwoken);
        obj->loop_cnt += 2;
        obj->fram_id  += 2;
        if (obj->fram_id == DMA_NODE_CNT) obj->fram_id = 0;
        if (obj->loop_cnt == BUF_EOF_CNT) obj->loop_cnt = 0;
    }

    if (int_st.out_eof) {
        xSemaphoreGiveFromISR(lcd_tx_sem, &HPTaskAwoken);
    }

    if (int_st.out_dscr_err) {
        ets_printf("error\n");
    }

    if(HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void cam_enable(void)
{
    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);
}

static void cam_disable(void)
{
    gpio_matrix_in(0x30, I2S0I_H_ENABLE_IDX, false);
}

void cam_stop(void)
{
    cam_disable();
    I2S0.conf.rx_start = 0;
    I2S0.in_link.stop = 1;
    I2S0.int_ena.in_suc_eof = 0;
    I2S0.conf2.cam_sync_fifo_reset = 1;
    I2S0.conf2.cam_sync_fifo_reset = 0;
    I2S0.int_clr.in_suc_eof = 1;
}

void cam_start(void)
{
    I2S0.int_clr.in_suc_eof = 1;
    I2S0.int_ena.in_suc_eof = 1;
    I2S0.conf.rx_reset = 1;
    I2S0.conf.rx_reset = 0;
    I2S0.conf2.cam_sync_fifo_reset = 1;
    I2S0.conf2.cam_sync_fifo_reset = 0;
    I2S0.in_link.start = 1;
    I2S0.conf.rx_start = 1;
    cam_enable();
}

static void cam_obj_init(lcd_cam_obj_t *obj)
{
    obj->loop_cnt = 0;
    obj->fram_id = 0;
    for (int i = 0; i < DMA_NODE_CNT; i++) {
        obj->pfram[i]->fram_id = 0;
        obj->pfram[i]->size = FRAM_BUF_SIZE;
    }
    xQueueReset(obj->queue);
}

static void lcd_cam_config(void)
{
    //Enable I2S periph
    periph_module_enable(PERIPH_I2S0_MODULE);

    // 配置时钟
    I2S0.clkm_conf.val = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a = 0;
    I2S0.clkm_conf.clk_sel = 2;
    I2S0.clkm_conf.clk_en = 1;

    // 配置采样率
    I2S0.sample_rate_conf.val = 0;
    I2S0.sample_rate_conf.tx_bck_div_num = 2;
    I2S0.sample_rate_conf.tx_bits_mod = lcd_cam_obj->config.lcd_bit_width;
    I2S0.sample_rate_conf.rx_bck_div_num = 1;
    I2S0.sample_rate_conf.rx_bits_mod = lcd_cam_obj->config.cam_bit_width;

    // 配置数据格式
    I2S0.conf.val = 0;
    I2S0.conf.tx_right_first = 1;
    I2S0.conf.tx_msb_right = 1;
    I2S0.conf.tx_dma_equal = 1;
    I2S0.conf.rx_right_first = 1;
    I2S0.conf.rx_msb_right = 1;
    I2S0.conf.rx_dma_equal = 1;

    I2S0.conf1.val = 0;
    I2S0.conf1.tx_pcm_bypass = 1;
    I2S0.conf1.tx_stop_en = 1;
    I2S0.conf1.rx_pcm_bypass = 1;

    I2S0.conf2.val = 0;
    I2S0.conf2.cam_sync_fifo_reset = 1;
    I2S0.conf2.cam_sync_fifo_reset = 0;
    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;
    I2S0.conf2.i_v_sync_filter_en = 1;
    I2S0.conf2.i_v_sync_filter_thres = 1;

    I2S0.conf_chan.val = 0;
    I2S0.conf_chan.tx_chan_mod = 1;
    I2S0.conf_chan.rx_chan_mod = 1;

    I2S0.fifo_conf.val = 0;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.rx_data_num = 32;
    I2S0.fifo_conf.rx_fifo_mod = 2;
    I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.tx_data_num = 32;
    I2S0.fifo_conf.tx_fifo_mod = 2;
    I2S0.fifo_conf.dscr_en = 1;

    I2S0.lc_conf.out_rst  = 1;
    I2S0.lc_conf.out_rst  = 0;
    I2S0.lc_conf.in_rst  = 1;
    I2S0.lc_conf.in_rst  = 0;

    I2S0.timing.val = 0;

    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0;

    I2S0.lc_conf.check_owner = 0;

    I2S0.int_ena.out_eof = 1;
    I2S0.int_ena.out_dscr_err = 1;

    ESP_LOGI(TAG, "--------I2S version  %x\n", I2S0.date);
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, 0, lcd_cam_isr, NULL, NULL);
}

static void cam_xclk_attach(void)
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_1_BIT,
        .freq_hz = 10000000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_2,
        .duty       = 1,
        .gpio_num   = lcd_cam_obj->config.cam_xclk_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_1,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel);
    ESP_LOGI(TAG, "cam_xclk_pin setup\n");
}

static void lcd_cam_set_pin(lcd_cam_config_t *config)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->lcd_ws_pin], PIN_FUNC_GPIO);
    gpio_set_direction(config->lcd_ws_pin, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(config->lcd_ws_pin, GPIO_FLOATING);
    gpio_matrix_out(config->lcd_ws_pin, I2S0O_WS_OUT_IDX, true, 0);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->lcd_rs_pin], PIN_FUNC_GPIO);
    gpio_set_direction(config->lcd_rs_pin, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(config->lcd_rs_pin, GPIO_FLOATING);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->lcd_rd_pin], PIN_FUNC_GPIO);
    gpio_set_direction(config->lcd_rd_pin, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(config->lcd_rd_pin, GPIO_FLOATING);
    gpio_set_level(config->lcd_rd_pin, 1);

    for(int i = 0; i < config->lcd_bit_width; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->lcd_data_pin[i]], PIN_FUNC_GPIO);
        gpio_set_direction(config->lcd_data_pin[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->lcd_data_pin[i], GPIO_FLOATING);
        // 高位对齐，OUT23总是最高位
        // fifo按bit来访问数据，tx_bits_mod为8时，数据需要按8位对齐
        gpio_matrix_out(config->lcd_data_pin[i], I2S0O_DATA_OUT0_IDX + (24 - config->lcd_bit_width) + i, false, 0);
    }

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->cam_pclk_pin], PIN_FUNC_GPIO);
    gpio_set_direction(config->cam_pclk_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->cam_pclk_pin, GPIO_FLOATING);
    gpio_matrix_in(config->cam_pclk_pin, I2S0I_WS_IN_IDX, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->cam_vsync_pin], PIN_FUNC_GPIO);
    gpio_set_direction(config->cam_vsync_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->cam_vsync_pin, GPIO_FLOATING);
    gpio_matrix_in(config->cam_vsync_pin, I2S0I_V_SYNC_IDX, true);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->cam_hsync_pin], PIN_FUNC_GPIO);
    gpio_set_direction(config->cam_hsync_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->cam_hsync_pin, GPIO_FLOATING);
    gpio_matrix_in(config->cam_hsync_pin, I2S0I_H_SYNC_IDX, false);

    for(int i = 0; i < config->cam_bit_width; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->cam_data_pin[i]], PIN_FUNC_GPIO);
        gpio_set_direction(config->cam_data_pin[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(config->cam_data_pin[i], GPIO_FLOATING);
        // 高位对齐，IN16总是最高位
        // fifo按bit来访问数据，rx_bits_mod为8时，数据需要按8位对齐
        gpio_matrix_in(config->cam_data_pin[i], I2S0I_DATA_IN0_IDX + (16 - config->cam_bit_width) + i, false);
    }

    cam_xclk_attach();
}

static inline void i2s_dma_start(uint32_t dma_addr)
{
    I2S0.out_link.addr = dma_addr & 0xfffff;
    I2S0.conf.tx_reset = 1;
    I2S0.conf.tx_reset = 0;
    I2S0.out_link.start = 1;
    I2S0.conf.tx_start = 1;
}

void lcd_write_data(uint8_t *data, size_t len)
{
    int x = 0, cnt = 0, size = 0;
    int end_pos = 0;
    // 生成一段数据DMA链表
    for (x = 0; x < LCD_NODE_CNT; x++) {
        lcd_dma[x].size = LCD_DMA_SIZE;
        lcd_dma[x].length = LCD_DMA_SIZE;
        lcd_dma[x].buf = (lcd_buffer + LCD_DMA_SIZE * x);
        lcd_dma[x].eof = !((x + 1) % LCD_HALF_NODE_CNT);
        lcd_dma[x].empty = &lcd_dma[(x + 1) % LCD_NODE_CNT];
    }
    lcd_dma[LCD_HALF_NODE_CNT - 1].empty = NULL;
    lcd_dma[LCD_NODE_CNT - 1].empty = NULL;
    cnt = len / LCD_HALF_BUFFER_SIZE;
    // 启动信号
    xSemaphoreGive(lcd_tx_sem);
    // 处理完整一段数据， 乒乓操作
    for (x = 0; x < cnt; x++) {
        memcpy(lcd_dma[(x % 2) * LCD_HALF_NODE_CNT].buf, data, LCD_HALF_BUFFER_SIZE);
        data += LCD_HALF_BUFFER_SIZE;
        xSemaphoreTake(lcd_tx_sem , portMAX_DELAY);
        i2s_dma_start(&lcd_dma[(x % 2) * LCD_HALF_NODE_CNT]);
    }
    cnt = len % LCD_HALF_BUFFER_SIZE;
    // 处理剩余非完整段数据
    if (cnt) {
        memcpy(lcd_dma[(x % 2) * LCD_HALF_NODE_CNT].buf, data, cnt);
        // 处理数据长度为 LCD_DMA_SIZE 的整数倍情况
        if (cnt % LCD_DMA_SIZE) {
            end_pos = (x % 2) * LCD_HALF_NODE_CNT + cnt / LCD_DMA_SIZE;
            size = cnt % LCD_DMA_SIZE;
        } else {
            end_pos = (x % 2) * LCD_HALF_NODE_CNT + cnt / LCD_DMA_SIZE - 1;
            size = LCD_DMA_SIZE;
        }
        // 处理尾节点，使其成为 DMA 尾
        lcd_dma[end_pos].size = size;
        lcd_dma[end_pos].length = size;
        lcd_dma[end_pos].eof = 1;
        lcd_dma[end_pos].empty = NULL;
        xSemaphoreTake(lcd_tx_sem , portMAX_DELAY);
        i2s_dma_start(&lcd_dma[(x % 2) * LCD_HALF_NODE_CNT]);
    }
    xSemaphoreTake(lcd_tx_sem , portMAX_DELAY);
}

static void lcd_write_cmd_byte(uint16_t cmd)
{
    uint16_t val;
    if (lcd_cam_obj->config.lcd_bit_width == 8) {
        val = (cmd >> 8) | (cmd << 8);
    } else {
        val = cmd;
        
    }
    gpio_set_level(lcd_cam_obj->config.lcd_rs_pin, 0);
    lcd_write_data(&val, 2);
    gpio_set_level(lcd_cam_obj->config.lcd_rs_pin, 1);
}

static void lcd_write_reg(uint16_t cmd, uint16_t data)
{
    uint16_t val;
    lcd_write_cmd_byte(cmd);

    if (lcd_cam_obj->config.lcd_bit_width == 8) {
        val = (data >> 8) | (data << 8);
    } else {
        val = data;
    }
    lcd_write_data(&val, 2);
}

void lcd_set_index(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end)
{    
    lcd_write_reg(0x2A00, (x_start >> 8));
    lcd_write_reg(0x2A01, (x_start & 0xff));
    lcd_write_reg(0x2A02, (x_end >> 8));
    lcd_write_reg(0x2A03, (x_end & 0xff));

    lcd_write_reg(0x2B00, (y_start >> 8));
    lcd_write_reg(0x2B01, (y_start & 0xff));
    lcd_write_reg(0x2B02, (y_end >> 8));
    lcd_write_reg(0x2B03, (y_end & 0xff));

    lcd_write_cmd_byte(0x2C00);
}

static void nt35510_init(void)
{
    LCD_DELAY_50US(10);
    lcd_write_cmd_byte(0x0100);
    lcd_write_cmd_byte(0x0100);
    LCD_DELAY_50US(100);
    lcd_write_cmd_byte(0x1200);
    lcd_write_reg(0xf000, 0x0055);
    lcd_write_reg(0xf001, 0x00aa);
    lcd_write_reg(0xf002, 0x0052);
    lcd_write_reg(0xf003, 0x0008);
    lcd_write_reg(0xf004, 0x0001);

    lcd_write_reg(0xbc01, 0x0086);
    lcd_write_reg(0xbc02, 0x006a);
    lcd_write_reg(0xbd01, 0x0086);
    lcd_write_reg(0xbd02, 0x006a);
    lcd_write_reg(0xbe01, 0x0067);

    lcd_write_reg(0xd100, 0x0000);
    lcd_write_reg(0xd101, 0x005d);
    lcd_write_reg(0xd102, 0x0000);
    lcd_write_reg(0xd103, 0x006b);
    lcd_write_reg(0xd104, 0x0000);
    lcd_write_reg(0xd105, 0x0084);
    lcd_write_reg(0xd106, 0x0000);
    lcd_write_reg(0xd107, 0x009c);
    lcd_write_reg(0xd108, 0x0000);
    lcd_write_reg(0xd109, 0x00b1);
    lcd_write_reg(0xd10a, 0x0000);
    lcd_write_reg(0xd10b, 0x00d9);
    lcd_write_reg(0xd10c, 0x0000);
    lcd_write_reg(0xd10d, 0x00fd);
    lcd_write_reg(0xd10e, 0x0001);
    lcd_write_reg(0xd10f, 0x0038);
    lcd_write_reg(0xd110, 0x0001);
    lcd_write_reg(0xd111, 0x0068);
    lcd_write_reg(0xd112, 0x0001);
    lcd_write_reg(0xd113, 0x00b9);
    lcd_write_reg(0xd114, 0x0001);
    lcd_write_reg(0xd115, 0x00fb);
    lcd_write_reg(0xd116, 0x0002);
    lcd_write_reg(0xd117, 0x0063);
    lcd_write_reg(0xd118, 0x0002);
    lcd_write_reg(0xd119, 0x00b9);
    lcd_write_reg(0xd11a, 0x0002);
    lcd_write_reg(0xd11b, 0x00bb);
    lcd_write_reg(0xd11c, 0x0003);
    lcd_write_reg(0xd11d, 0x0003);
    lcd_write_reg(0xd11e, 0x0003);
    lcd_write_reg(0xd11f, 0x0046);
    lcd_write_reg(0xd120, 0x0003);
    lcd_write_reg(0xd121, 0x0069);
    lcd_write_reg(0xd122, 0x0003);
    lcd_write_reg(0xd123, 0x008f);
    lcd_write_reg(0xd124, 0x0003);
    lcd_write_reg(0xd125, 0x00a4);
    lcd_write_reg(0xd126, 0x0003);
    lcd_write_reg(0xd127, 0x00b9);
    lcd_write_reg(0xd128, 0x0003);
    lcd_write_reg(0xd129, 0x00c7);
    lcd_write_reg(0xd12a, 0x0003);
    lcd_write_reg(0xd12b, 0x00c9);
    lcd_write_reg(0xd12c, 0x0003);
    lcd_write_reg(0xd12d, 0x00cb);
    lcd_write_reg(0xd12e, 0x0003);
    lcd_write_reg(0xd12f, 0x00cb);
    lcd_write_reg(0xd130, 0x0003);
    lcd_write_reg(0xd131, 0x00cb);
    lcd_write_reg(0xd132, 0x0003);
    lcd_write_reg(0xd133, 0x00cc);

    lcd_write_reg(0xd200, 0x0000);
    lcd_write_reg(0xd201, 0x005d);
    lcd_write_reg(0xd202, 0x0000);
    lcd_write_reg(0xd203, 0x006b);
    lcd_write_reg(0xd204, 0x0000);
    lcd_write_reg(0xd205, 0x0084);
    lcd_write_reg(0xd206, 0x0000);
    lcd_write_reg(0xd207, 0x009c);
    lcd_write_reg(0xd208, 0x0000);
    lcd_write_reg(0xd209, 0x00b1);
    lcd_write_reg(0xd20a, 0x0000);
    lcd_write_reg(0xd20b, 0x00d9);
    lcd_write_reg(0xd20c, 0x0000);
    lcd_write_reg(0xd20d, 0x00fd);
    lcd_write_reg(0xd20e, 0x0001);
    lcd_write_reg(0xd20f, 0x0038);
    lcd_write_reg(0xd210, 0x0001);
    lcd_write_reg(0xd211, 0x0068);
    lcd_write_reg(0xd212, 0x0001);
    lcd_write_reg(0xd213, 0x00b9);
    lcd_write_reg(0xd214, 0x0001);
    lcd_write_reg(0xd215, 0x00fb);
    lcd_write_reg(0xd216, 0x0002);
    lcd_write_reg(0xd217, 0x0063);
    lcd_write_reg(0xd218, 0x0002);
    lcd_write_reg(0xd219, 0x00b9);
    lcd_write_reg(0xd21a, 0x0002);
    lcd_write_reg(0xd21b, 0x00bb);
    lcd_write_reg(0xd21c, 0x0003);
    lcd_write_reg(0xd21d, 0x0003);
    lcd_write_reg(0xd21e, 0x0003);
    lcd_write_reg(0xd21f, 0x0046);
    lcd_write_reg(0xd220, 0x0003);
    lcd_write_reg(0xd221, 0x0069);
    lcd_write_reg(0xd222, 0x0003);
    lcd_write_reg(0xd223, 0x008f);
    lcd_write_reg(0xd224, 0x0003);
    lcd_write_reg(0xd225, 0x00a4);
    lcd_write_reg(0xd226, 0x0003);
    lcd_write_reg(0xd227, 0x00b9);
    lcd_write_reg(0xd228, 0x0003);
    lcd_write_reg(0xd229, 0x00c7);
    lcd_write_reg(0xd22a, 0x0003);
    lcd_write_reg(0xd22b, 0x00c9);
    lcd_write_reg(0xd22c, 0x0003);
    lcd_write_reg(0xd22d, 0x00cb);
    lcd_write_reg(0xd22e, 0x0003);
    lcd_write_reg(0xd22f, 0x00cb);
    lcd_write_reg(0xd230, 0x0003);
    lcd_write_reg(0xd231, 0x00cb);
    lcd_write_reg(0xd232, 0x0003);
    lcd_write_reg(0xd233, 0x00cc);

    lcd_write_reg(0xd300, 0x0000);
    lcd_write_reg(0xd301, 0x005d);
    lcd_write_reg(0xd302, 0x0000);
    lcd_write_reg(0xd303, 0x006b);
    lcd_write_reg(0xd304, 0x0000);
    lcd_write_reg(0xd305, 0x0084);
    lcd_write_reg(0xd306, 0x0000);
    lcd_write_reg(0xd307, 0x009c);
    lcd_write_reg(0xd308, 0x0000);
    lcd_write_reg(0xd309, 0x00b1);
    lcd_write_reg(0xd30a, 0x0000);
    lcd_write_reg(0xd30b, 0x00d9);
    lcd_write_reg(0xd30c, 0x0000);
    lcd_write_reg(0xd30d, 0x00fd);
    lcd_write_reg(0xd30e, 0x0001);
    lcd_write_reg(0xd30f, 0x0038);
    lcd_write_reg(0xd310, 0x0001);
    lcd_write_reg(0xd311, 0x0068);
    lcd_write_reg(0xd312, 0x0001);
    lcd_write_reg(0xd313, 0x00b9);
    lcd_write_reg(0xd314, 0x0001);
    lcd_write_reg(0xd315, 0x00fb);
    lcd_write_reg(0xd316, 0x0002);
    lcd_write_reg(0xd317, 0x0063);
    lcd_write_reg(0xd318, 0x0002);
    lcd_write_reg(0xd319, 0x00b9);
    lcd_write_reg(0xd31a, 0x0002);
    lcd_write_reg(0xd31b, 0x00bb);
    lcd_write_reg(0xd31c, 0x0003);
    lcd_write_reg(0xd31d, 0x0003);
    lcd_write_reg(0xd31e, 0x0003);
    lcd_write_reg(0xd31f, 0x0046);
    lcd_write_reg(0xd320, 0x0003);
    lcd_write_reg(0xd321, 0x0069);
    lcd_write_reg(0xd322, 0x0003);
    lcd_write_reg(0xd323, 0x008f);
    lcd_write_reg(0xd324, 0x0003);
    lcd_write_reg(0xd325, 0x00a4);
    lcd_write_reg(0xd326, 0x0003);
    lcd_write_reg(0xd327, 0x00b9);
    lcd_write_reg(0xd328, 0x0003);
    lcd_write_reg(0xd329, 0x00c7);
    lcd_write_reg(0xd32a, 0x0003);
    lcd_write_reg(0xd32b, 0x00c9);
    lcd_write_reg(0xd32c, 0x0003);
    lcd_write_reg(0xd32d, 0x00cb);
    lcd_write_reg(0xd32e, 0x0003);
    lcd_write_reg(0xd32f, 0x00cb);
    lcd_write_reg(0xd330, 0x0003);
    lcd_write_reg(0xd331, 0x00cb);
    lcd_write_reg(0xd332, 0x0003);
    lcd_write_reg(0xd333, 0x00cc);

    lcd_write_reg(0xd400, 0x0000);
    lcd_write_reg(0xd401, 0x005d);
    lcd_write_reg(0xd402, 0x0000);
    lcd_write_reg(0xd403, 0x006b);
    lcd_write_reg(0xd404, 0x0000);
    lcd_write_reg(0xd405, 0x0084);
    lcd_write_reg(0xd406, 0x0000);
    lcd_write_reg(0xd407, 0x009c);
    lcd_write_reg(0xd408, 0x0000);
    lcd_write_reg(0xd409, 0x00b1);
    lcd_write_reg(0xd40a, 0x0000);
    lcd_write_reg(0xd40b, 0x00d9);
    lcd_write_reg(0xd40c, 0x0000);
    lcd_write_reg(0xd40d, 0x00fd);
    lcd_write_reg(0xd40e, 0x0001);
    lcd_write_reg(0xd40f, 0x0038);
    lcd_write_reg(0xd410, 0x0001);
    lcd_write_reg(0xd411, 0x0068);
    lcd_write_reg(0xd412, 0x0001);
    lcd_write_reg(0xd413, 0x00b9);
    lcd_write_reg(0xd414, 0x0001);
    lcd_write_reg(0xd415, 0x00fb);
    lcd_write_reg(0xd416, 0x0002);
    lcd_write_reg(0xd417, 0x0063);
    lcd_write_reg(0xd418, 0x0002);
    lcd_write_reg(0xd419, 0x00b9);
    lcd_write_reg(0xd41a, 0x0002);
    lcd_write_reg(0xd41b, 0x00bb);
    lcd_write_reg(0xd41c, 0x0003);
    lcd_write_reg(0xd41d, 0x0003);
    lcd_write_reg(0xd41e, 0x0003);
    lcd_write_reg(0xd41f, 0x0046);
    lcd_write_reg(0xd420, 0x0003);
    lcd_write_reg(0xd421, 0x0069);
    lcd_write_reg(0xd422, 0x0003);
    lcd_write_reg(0xd423, 0x008f);
    lcd_write_reg(0xd424, 0x0003);
    lcd_write_reg(0xd425, 0x00a4);
    lcd_write_reg(0xd426, 0x0003);
    lcd_write_reg(0xd427, 0x00b9);
    lcd_write_reg(0xd428, 0x0003);
    lcd_write_reg(0xd429, 0x00c7);
    lcd_write_reg(0xd42a, 0x0003);
    lcd_write_reg(0xd42b, 0x00c9);
    lcd_write_reg(0xd42c, 0x0003);
    lcd_write_reg(0xd42d, 0x00cb);
    lcd_write_reg(0xd42e, 0x0003);
    lcd_write_reg(0xd42f, 0x00cb);
    lcd_write_reg(0xd430, 0x0003);
    lcd_write_reg(0xd431, 0x00cb);
    lcd_write_reg(0xd432, 0x0003);
    lcd_write_reg(0xd433, 0x00cc);

    lcd_write_reg(0xd500, 0x0000);
    lcd_write_reg(0xd501, 0x005d);
    lcd_write_reg(0xd502, 0x0000);
    lcd_write_reg(0xd503, 0x006b);
    lcd_write_reg(0xd504, 0x0000);
    lcd_write_reg(0xd505, 0x0084);
    lcd_write_reg(0xd506, 0x0000);
    lcd_write_reg(0xd507, 0x009c);
    lcd_write_reg(0xd508, 0x0000);
    lcd_write_reg(0xd509, 0x00b1);
    lcd_write_reg(0xd50a, 0x0000);
    lcd_write_reg(0xd50b, 0x00D9);
    lcd_write_reg(0xd50c, 0x0000);
    lcd_write_reg(0xd50d, 0x00fd);
    lcd_write_reg(0xd50e, 0x0001);
    lcd_write_reg(0xd50f, 0x0038);
    lcd_write_reg(0xd510, 0x0001);
    lcd_write_reg(0xd511, 0x0068);
    lcd_write_reg(0xd512, 0x0001);
    lcd_write_reg(0xd513, 0x00b9);
    lcd_write_reg(0xd514, 0x0001);
    lcd_write_reg(0xd515, 0x00fb);
    lcd_write_reg(0xd516, 0x0002);
    lcd_write_reg(0xd517, 0x0063);
    lcd_write_reg(0xd518, 0x0002);
    lcd_write_reg(0xd519, 0x00b9);
    lcd_write_reg(0xd51a, 0x0002);
    lcd_write_reg(0xd51b, 0x00bb);
    lcd_write_reg(0xd51c, 0x0003);
    lcd_write_reg(0xd51d, 0x0003);
    lcd_write_reg(0xd51e, 0x0003);
    lcd_write_reg(0xd51f, 0x0046);
    lcd_write_reg(0xd520, 0x0003);
    lcd_write_reg(0xd521, 0x0069);
    lcd_write_reg(0xd522, 0x0003);
    lcd_write_reg(0xd523, 0x008f);
    lcd_write_reg(0xd524, 0x0003);
    lcd_write_reg(0xd525, 0x00a4);
    lcd_write_reg(0xd526, 0x0003);
    lcd_write_reg(0xd527, 0x00b9);
    lcd_write_reg(0xd528, 0x0003);
    lcd_write_reg(0xd529, 0x00c7);
    lcd_write_reg(0xd52a, 0x0003);
    lcd_write_reg(0xd52b, 0x00c9);
    lcd_write_reg(0xd52c, 0x0003);
    lcd_write_reg(0xd52d, 0x00cb);
    lcd_write_reg(0xd52e, 0x0003);
    lcd_write_reg(0xd52f, 0x00cb);
    lcd_write_reg(0xd530, 0x0003);
    lcd_write_reg(0xd531, 0x00cb);
    lcd_write_reg(0xd532, 0x0003);
    lcd_write_reg(0xd533, 0x00cc);

    lcd_write_reg(0xd600, 0x0000);
    lcd_write_reg(0xd601, 0x005d);
    lcd_write_reg(0xd602, 0x0000);
    lcd_write_reg(0xd603, 0x006b);
    lcd_write_reg(0xd604, 0x0000);
    lcd_write_reg(0xd605, 0x0084);
    lcd_write_reg(0xd606, 0x0000);
    lcd_write_reg(0xd607, 0x009c);
    lcd_write_reg(0xd608, 0x0000);
    lcd_write_reg(0xd609, 0x00b1);
    lcd_write_reg(0xd60a, 0x0000);
    lcd_write_reg(0xd60b, 0x00d9);
    lcd_write_reg(0xd60c, 0x0000);
    lcd_write_reg(0xd60d, 0x00fd);
    lcd_write_reg(0xd60e, 0x0001);
    lcd_write_reg(0xd60f, 0x0038);
    lcd_write_reg(0xd610, 0x0001);
    lcd_write_reg(0xd611, 0x0068);
    lcd_write_reg(0xd612, 0x0001);
    lcd_write_reg(0xd613, 0x00b9);
    lcd_write_reg(0xd614, 0x0001);
    lcd_write_reg(0xd615, 0x00fb);
    lcd_write_reg(0xd616, 0x0002);
    lcd_write_reg(0xd617, 0x0063);
    lcd_write_reg(0xd618, 0x0002);
    lcd_write_reg(0xd619, 0x00b9);
    lcd_write_reg(0xd61a, 0x0002);
    lcd_write_reg(0xd61b, 0x00bb);
    lcd_write_reg(0xd61c, 0x0003);
    lcd_write_reg(0xd61d, 0x0003);
    lcd_write_reg(0xd61e, 0x0003);
    lcd_write_reg(0xd61f, 0x0046);
    lcd_write_reg(0xd620, 0x0003);
    lcd_write_reg(0xd621, 0x0069);
    lcd_write_reg(0xd622, 0x0003);
    lcd_write_reg(0xd623, 0x008f);
    lcd_write_reg(0xd624, 0x0003);
    lcd_write_reg(0xd625, 0x00a4);
    lcd_write_reg(0xd626, 0x0003);
    lcd_write_reg(0xd627, 0x00b9);
    lcd_write_reg(0xd628, 0x0003);
    lcd_write_reg(0xd629, 0x00c7);
    lcd_write_reg(0xd62a, 0x0003);
    lcd_write_reg(0xd62b, 0x00c9);
    lcd_write_reg(0xd62c, 0x0003);
    lcd_write_reg(0xd62d, 0x00cb);
    lcd_write_reg(0xd62e, 0x0003);
    lcd_write_reg(0xd62f, 0x00cb);
    lcd_write_reg(0xd630, 0x0003);
    lcd_write_reg(0xd631, 0x00cb);
    lcd_write_reg(0xd632, 0x0003);
    lcd_write_reg(0xd633, 0x00cc);

    lcd_write_reg(0xba00, 0x0024);
    lcd_write_reg(0xba01, 0x0024);
    lcd_write_reg(0xba02, 0x0024);

    lcd_write_reg(0xb900, 0x0024);
    lcd_write_reg(0xb901, 0x0024);
    lcd_write_reg(0xb902, 0x0024);

    lcd_write_reg(0xf000, 0x0055);
    lcd_write_reg(0xf001, 0x00aa);
    lcd_write_reg(0xf002, 0x0052);
    lcd_write_reg(0xf003, 0x0008);
    lcd_write_reg(0xf004, 0x0000);

    lcd_write_reg(0xb100, 0x00cc);
    lcd_write_reg(0xB500, 0x0050);

    lcd_write_reg(0xbc00, 0x0005);
    lcd_write_reg(0xbc01, 0x0005);
    lcd_write_reg(0xbc02, 0x0005);

    lcd_write_reg(0xb800, 0x0001);
    lcd_write_reg(0xb801, 0x0003);
    lcd_write_reg(0xb802, 0x0003);
    lcd_write_reg(0xb803, 0x0003);

    lcd_write_reg(0xbd02, 0x0007);
    lcd_write_reg(0xbd03, 0x0031);
    lcd_write_reg(0xbe02, 0x0007);
    lcd_write_reg(0xbe03, 0x0031);
    lcd_write_reg(0xbf02, 0x0007);
    lcd_write_reg(0xbf03, 0x0031);

    lcd_write_reg(0xff00, 0x00aa);
    lcd_write_reg(0xff01, 0x0055);
    lcd_write_reg(0xff02, 0x0025);
    lcd_write_reg(0xff03, 0x0001);

    lcd_write_reg(0xf304, 0x0011);
    lcd_write_reg(0xf306, 0x0010);
    lcd_write_reg(0xf308, 0x0000);

    lcd_write_reg(0x3500, 0x0000);
    lcd_write_reg(0x3600, 0x0060);
    
    lcd_write_reg(0x3A00, 0x0005);
    //Display On
    lcd_write_cmd_byte(0x2900);
    // Out sleep
    lcd_write_cmd_byte(0x1100);
    // Write continue
    lcd_write_cmd_byte(0x2C00);
}

static lcd_cam_obj_t* cam_dma_create(void)
{
    for (int i = 0; i < DMA_NODE_CNT; i++) {
        lcd_cam_obj->pfram[i] = (camera_fram_t *)heap_caps_malloc(sizeof(camera_fram_t), MALLOC_CAP_DMA);
        if (!lcd_cam_obj->pfram[i]) {
            ESP_LOGI(TAG, "camera dma node buffer malloc error\n");
            abort();
        }
        memset(lcd_cam_obj->pfram[i], 0, sizeof(camera_fram_t));
    }
    lcd_cam_obj->disp_mux = xSemaphoreCreateBinary();
    if (!lcd_cam_obj->disp_mux) {
        ESP_LOGI(TAG, "disp mux create fail\n");
        abort();
    }
    lcd_cam_obj->update_mux = xSemaphoreCreateBinary();
    if (!lcd_cam_obj->update_mux) {
        ESP_LOGI(TAG, "update mux create fail\n");
        abort();
    }
    xSemaphoreGive(lcd_cam_obj->update_mux);
    lcd_cam_obj->queue = xQueueCreate(10, sizeof(int));
    if (!lcd_cam_obj->queue) {
        ESP_LOGI(TAG, "queue create fail\n");
        abort();
    }
    lcd_cam_obj->pbuf = (uint8_t *)heap_caps_calloc(1, FRAM_WIDTH * FRAM_HIGH * PIX_BYTE, MALLOC_CAP_SPIRAM);
    if (!lcd_cam_obj->pbuf) {
        ESP_LOGI(TAG, "camera fram buffer malloc error\n");
        abort();
    }
    ESP_LOGI(TAG, "fram buf addr: %p\n", lcd_cam_obj->pbuf);
    for (int i = 0; i < FRAM_WIDTH * FRAM_HIGH * PIX_BYTE; i++) {
        lcd_cam_obj->pbuf[i] = ((i + 1) % 32);
    }
    for (int i = 0; i < FRAM_WIDTH * FRAM_HIGH * PIX_BYTE; i++) {
        if (lcd_cam_obj->pbuf[i] != ((i + 1) % 32)) {
            ESP_LOGI(TAG, "item error  %d  %d  %d\n", i, lcd_cam_obj->pbuf[i], ((i + 1) % 32));
        }
    }
    ESP_LOGI(TAG, "fram buf test done\n");
    cam_obj_init(lcd_cam_obj);
    return lcd_cam_obj;
}

#if USE_FRAM_LOCK
typedef enum {
    FRAM_UNLOCK,
    FRAM_LOCK,
    FRAM_IN_PROGRESS,
} fram_sta_t;

fram_sta_t fram_status = FRAM_UNLOCK;
#endif

//Copy fram from DMA buffer to fram buffer
static void cam_fram_copy_task(void *param)
{
    lcd_cam_obj_t* obj = (lcd_cam_obj_t*)param;
    int id = 0;
    while (1) {
        if (xSemaphoreTake(obj->update_mux, (portTickType)10) != pdTRUE) {
            xQueueReceive(obj->queue, (void *)&id, (portTickType)portMAX_DELAY);
        } else {
            while (1) {
                xQueueReceive(obj->queue, (void *)&id, (portTickType)portMAX_DELAY);
                if (fram_status != FRAM_IN_PROGRESS && obj->pfram[id]->fram_id != 0) {
                    continue;
                }
                fram_status = FRAM_IN_PROGRESS;
                // ESP_LOGI(TAG, "%d %d\n", obj->pfram[id]->fram_id, obj->pfram[id + 1]->fram_id);
                memcpy(obj->pbuf + obj->pfram[id]->fram_id * FRAM_BUF_SIZE, obj->pfram[id]->buf, FRAM_BUF_SIZE);
                memcpy(obj->pbuf + obj->pfram[id + 1]->fram_id * FRAM_BUF_SIZE, obj->pfram[id + 1]->buf, FRAM_BUF_SIZE);
                if (obj->pfram[id + 1]->fram_id == BUF_EOF_CNT - 1) {
                    xSemaphoreGive(obj->disp_mux);
                    fram_status = FRAM_LOCK;
                    break;
                }
            }
        }
    }
}

void take_fram_lock(void)
{
    xSemaphoreTake(lcd_cam_obj->disp_mux, (portTickType)portMAX_DELAY);
}

void give_fram_lock(void)
{
    xSemaphoreGive(lcd_cam_obj->update_mux);
}

uint8_t * cam_attach(void)
{
    cam_dma_create();
    for (int i = 0; i < DMA_NODE_CNT; i++) {
        cam_dma[i].size = FRAM_BUF_SIZE;
        cam_dma[i].length = FRAM_BUF_SIZE;
        cam_dma[i].eof = 1;
        cam_dma[i].owner = 1;
        cam_dma[i].buf = lcd_cam_obj->pfram[i]->buf;
        cam_dma[i].empty = &cam_dma[(i + 1) % DMA_NODE_CNT];
    }
    xTaskCreate(cam_fram_copy_task, "cam_fram_copy_task", 1024 * 4, (void *)lcd_cam_obj, 8, NULL);
    I2S0.in_link.addr = ((uint32_t)&cam_dma[0]) & 0xfffff;
    I2S0.rx_eof_num = FRAM_BUF_SIZE * 2;
    return lcd_cam_obj->pbuf;
}

void lcd_cam_init(const lcd_cam_config_t *config)
{
    lcd_cam_obj = (lcd_cam_obj_t *)heap_caps_malloc(sizeof(lcd_cam_obj_t), MALLOC_CAP_DMA);
    if (!lcd_cam_obj) {
        ESP_LOGI(TAG, "camera object malloc error\n");
        abort();
    }
    memset(lcd_cam_obj, 0, sizeof(lcd_cam_obj_t));
    lcd_dma = (lldesc_t *)heap_caps_malloc(sizeof(lldesc_t) * LCD_NODE_CNT, MALLOC_CAP_DMA);
    lcd_buffer = (uint8_t *)heap_caps_malloc(LCD_BUFFER_SIZE, MALLOC_CAP_DMA);
    memcpy(&lcd_cam_obj->config, config, sizeof(lcd_cam_config_t));
    lcd_tx_sem = xSemaphoreCreateBinary();

    lcd_cam_set_pin(&lcd_cam_obj->config);
    lcd_cam_config();
    nt35510_init();
    nt35510_init();
    printf("init_ok\n");
}