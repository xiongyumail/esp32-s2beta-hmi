#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
// #include "driver/i2s.h"
#include "esp_system.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include"soc/apb_ctrl_reg.h"
#include "soc/system_reg.h"
#include "driver/ledc.h"

#include "esp32s2beta/rom/lldesc.h"

#define BIT_MODE   8

#define D0 GPIO_NUM_18
#define D2 GPIO_NUM_17
#define D4 GPIO_NUM_8
#define D6 GPIO_NUM_10

#define D1 GPIO_NUM_21
#define D3 GPIO_NUM_7
#define D5 GPIO_NUM_9
#define D7 GPIO_NUM_11

#define XCLK  GPIO_NUM_0
#define PCLK  GPIO_NUM_12
#define VSYNC GPIO_NUM_14
#define HSYNC GPIO_NUM_13
#define SIG_OFFSET  (8)

#define FRAM_WIDTH    (320)
#define FRAM_HIGH     (240)
#define PIX_SIZE      (2)

#define FRAM_BUF_LINE (6)
#define FRAM_BUF_SIZE (FRAM_WIDTH*PIX_SIZE*FRAM_BUF_LINE)
#define DMA_NODE_CNT  (10)

#define USE_FRAM_LOCK    (0)
#define FRAM_BUFER_DEBUG (0)

typedef struct {
    int fram_id;
    int size;
    uint8_t buf[FRAM_BUF_SIZE];
} camera_fram_t;

typedef struct {
    int loop_cnt;
    int fram_id;
    xQueueHandle queue;
    camera_fram_t *pfram[DMA_NODE_CNT];
    uint8_t *pbuf;
} camera_obj_t;

camera_obj_t *cam_obj = NULL;

lldesc_t _dma[DMA_NODE_CNT] = {0};

static void i2s_cam_enable(void)
{
    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);
}

static void i2s_cam_disable(void)
{
    gpio_matrix_in(0x30, I2S0I_H_ENABLE_IDX, false);
}

static void i2s_rst(void)
{
    I2S0.conf.tx_reset = 1;
    I2S0.conf.tx_reset = 0;
    I2S0.conf.rx_reset = 1;
    I2S0.conf.rx_reset = 0;
    I2S0.conf.tx_fifo_reset = 1;
    I2S0.conf.tx_fifo_reset = 0;
    I2S0.conf.rx_fifo_reset = 1;
    I2S0.conf.rx_fifo_reset = 0;
    while( GET_PERI_REG_MASK(I2S_CONF_REG(0), I2S_TX_FIFO_RESET_ST_M) );
    while( GET_PERI_REG_MASK(I2S_CONF_REG(0), I2S_RX_FIFO_RESET_ST_M) );
}

static void dma_rst(void)
{
    I2S0.lc_conf.in_rst  = 1;
    I2S0.lc_conf.in_rst  = 0;
    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;
}

static void camera_pin_cfg(void)
{
    gpio_matrix_in(0x30, I2S0I_H_ENABLE_IDX, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[HSYNC], PIN_FUNC_GPIO);
    gpio_set_direction(HSYNC, GPIO_MODE_INPUT);
    gpio_matrix_in(HSYNC, I2S0I_H_SYNC_IDX, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[VSYNC], PIN_FUNC_GPIO);
    gpio_set_direction(VSYNC, GPIO_MODE_INPUT);
    gpio_matrix_in(VSYNC, I2S0I_V_SYNC_IDX, true);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PCLK], PIN_FUNC_GPIO);
    gpio_set_direction(PCLK, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PCLK,GPIO_PULLUP_ONLY);
    gpio_matrix_in(PCLK, I2S0I_WS_IN_IDX, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[D0], PIN_FUNC_GPIO);
    gpio_set_direction(D0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(D0,GPIO_PULLUP_ONLY);
    gpio_matrix_in(D0, I2S0I_DATA_IN0_IDX + SIG_OFFSET, false);
    
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[D1], PIN_FUNC_GPIO);
    gpio_set_direction(D1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(D1,GPIO_PULLUP_ONLY);
    gpio_matrix_in(D1, I2S0I_DATA_IN1_IDX + SIG_OFFSET, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[D2], PIN_FUNC_GPIO);
    gpio_set_direction(D2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(D2,GPIO_PULLUP_ONLY);
    gpio_matrix_in(D2, I2S0I_DATA_IN2_IDX + SIG_OFFSET, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[D3], PIN_FUNC_GPIO);
    gpio_set_direction(D3, GPIO_MODE_INPUT);
    gpio_set_pull_mode(D3,GPIO_PULLUP_ONLY);
    gpio_matrix_in(D3, I2S0I_DATA_IN3_IDX + SIG_OFFSET, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[D4], PIN_FUNC_GPIO);
    gpio_set_direction(D4, GPIO_MODE_INPUT);
    gpio_set_pull_mode(D4,GPIO_PULLUP_ONLY);
    gpio_matrix_in(D4, I2S0I_DATA_IN4_IDX + SIG_OFFSET, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[D5], PIN_FUNC_GPIO);
    gpio_set_direction(D5, GPIO_MODE_INPUT);
    gpio_set_pull_mode(D5,GPIO_PULLUP_ONLY);
    gpio_matrix_in(D5, I2S0I_DATA_IN5_IDX + SIG_OFFSET, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[D6], PIN_FUNC_GPIO);
    gpio_set_direction(D6, GPIO_MODE_INPUT);
    gpio_set_pull_mode(D6,GPIO_PULLUP_ONLY);
    gpio_matrix_in(D6, I2S0I_DATA_IN6_IDX + SIG_OFFSET, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[D7], PIN_FUNC_GPIO);
    gpio_set_direction(D7, GPIO_MODE_INPUT);
    gpio_set_pull_mode(D7,GPIO_PULLUP_ONLY);
    gpio_matrix_in(D7, I2S0I_DATA_IN7_IDX + SIG_OFFSET, false);
}

void camera_xclk_attach(void)
{
   ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_2_BIT,
        .freq_hz = 20000000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_2,
        .duty       = 1,
        .gpio_num   = GPIO_NUM_0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_1,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel);
    printf("xclk setup\n");
}

static void i2s_camera_config(void)
{
    //Enable I2S periph
    CLEAR_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2S0_CLK_EN);
    SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2S0_CLK_EN);
    SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2S0_RST);
    CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2S0_RST);
//    REG_WRITE(APB_CTRL_SARADC_CTRL_REG, REG_READ(APB_CTRL_SARADC_CTRL_REG) & (~APB_CTRL_SARADC_DATA_TO_I2S));

    I2S0.conf.val = 0;
    I2S0.conf1.val = 0;
    I2S0.conf1.rx_pcm_bypass = 1;
    I2S0.sample_rate_conf.val = 0;
    I2S0.sample_rate_conf.rx_bck_div_num = 12;
    I2S0.timing.val = 0;
    I2S0.conf2.val = 0;
    I2S0.fifo_conf.val = 0;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.rx_data_num = 32;
    I2S0.conf.rx_right_first = 1;
    I2S0.conf.rx_dma_equal = 1;
 
    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;
    I2S0.conf2.cam_clk_loopback = 0;
    I2S0.conf2.i_v_sync_filter_en = 1;
    I2S0.conf2.i_v_sync_filter_thres = 1;
    I2S0.fifo_conf.rx_data_num = 32;
    I2S0.sample_rate_conf.rx_bits_mod = 8;
    I2S0.fifo_conf.rx_fifo_mod = 1;
    I2S0.conf_chan.rx_chan_mod = 1;

    I2S0.clkm_conf.val = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a =0;
    I2S0.clkm_conf.clk_sel = 2;
    I2S0.clkm_conf.clk_en = 1;
    i2s_rst();
    dma_rst();
    I2S0.conf2.cam_sync_fifo_reset = 1;
    //while(I2S0.conf2.cam_sync_fifo_reset_st);
    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0;
    camera_pin_cfg();
    printf("i2s version  0x%x\n", I2S0.date);
}

void i2s_camera_stop(void)
{
    i2s_cam_disable();
    I2S0.conf.rx_start = 0;
    I2S0.in_link.stop = 1;
    I2S0.fifo_conf.dscr_en = 0;
    I2S0.int_ena.in_suc_eof = 0;
    I2S0.conf2.cam_sync_fifo_reset = 1;
    I2S0.int_clr.in_suc_eof = 1;
}

void i2s_camera_start(void)
{
    I2S0.conf2.cam_sync_fifo_reset = 0;
    I2S0.fifo_conf.dscr_en = 1;
    I2S0.in_link.start = 1;
    I2S0.conf.rx_start = 1;
    I2S0.int_clr.in_suc_eof = 1;
    I2S0.int_ena.in_suc_eof = 1;
    i2s_cam_enable();
}

static void camera_obj_init(camera_obj_t *obj)
{
    obj->loop_cnt = 0;
    obj->fram_id = 0;
    for (int i = 0; i < DMA_NODE_CNT; i++) {
        obj->pfram[i]->fram_id = 0;
        obj->pfram[i]->size = FRAM_BUF_SIZE;
    }
    xQueueReset(obj->queue);
}

static void IRAM_ATTR i2s_cam_isr(void *param)
{
    typeof(I2S0.int_st) int_st = I2S0.int_st;
    I2S0.int_clr.val = int_st.val;
    portBASE_TYPE high_priority_task_awoken = 0;
    camera_obj_t* obj = (camera_obj_t*)param;
    if (int_st.in_suc_eof) {
        obj->pfram[obj->fram_id]->fram_id = obj->loop_cnt;
        obj->pfram[obj->fram_id+1]->fram_id = obj->loop_cnt+1;
        xQueueSendFromISR(obj->queue, (void *)&obj->fram_id, &high_priority_task_awoken);
        obj->loop_cnt += 2;
        obj->fram_id  += 2;
        if(obj->fram_id == DMA_NODE_CNT) obj->fram_id = 0;
        if(obj->loop_cnt == 40) obj->loop_cnt = 0;
    }
    if (high_priority_task_awoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static camera_obj_t* camera_dma_create(void)
{
    camera_obj_t *camera_obj = (camera_obj_t *)malloc(sizeof(camera_obj_t));
    if (!camera_obj){
        printf("camera object malloc error\n");
        abort();
    }
    memset(camera_obj, 0, sizeof(camera_obj_t));
    for (int i = 0; i < DMA_NODE_CNT; i++) {
        camera_obj->pfram[i] = (camera_fram_t *)malloc(sizeof(camera_fram_t));
        if (!camera_obj->pfram[i]) {
            printf("camera dma node buffer malloc error\n");
            abort();
        }
        memset(camera_obj->pfram[i], 0, sizeof(camera_fram_t));
    }
    camera_obj->queue = xQueueCreate(10, sizeof(int));
    if(!camera_obj->queue) {
        printf("queue create fail\n");
        abort();
    }
    camera_obj->pbuf = (uint8_t *)heap_caps_calloc(1, FRAM_WIDTH*FRAM_WIDTH*PIX_SIZE, MALLOC_CAP_SPIRAM);
    if(!camera_obj->pbuf) {
        printf("camera fram buffer malloc error\n");
        abort();
    }
    printf("fram buf addr: %p\n", camera_obj->pbuf);
    for(int i = 0; i < 320*240*2; i++) {
        camera_obj->pbuf[i] = ((i +1) % 32);
    }
    for(int i = 0; i < 320*240*2; i++) {
        if(camera_obj->pbuf[i] != ((i +1) % 32)) {
            printf("item error  %d  %d  %d\n", i, camera_obj->pbuf[i], ((i +1) % 32));
        }
    }
    printf("fram buf test done\n");
    camera_obj_init(camera_obj);
    return camera_obj;
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
static void camera_fram_copy_task(void *param)
{
    camera_obj_t* obj = (camera_obj_t*)param;
    int id = 0; // fram id: 0~240
    while(1) {
        xQueueReceive(obj->queue, (void *)&id, (portTickType)portMAX_DELAY);
#if USE_FRAM_LOCK
        if(fram_status == FRAM_LOCK) {
            continue;
        }
        if (fram_status != FRAM_IN_PROGRESS ) {
            if (obj->pfram[id]->fram_id == 0) {
                fram_status = FRAM_IN_PROGRESS;
            } else {
                fram_status = FRAM_IN_PROGRESS;
                continue;
            }
        }
#endif

#if !FRAM_BUFER_DEBUG
        memcpy(obj->pbuf + obj->pfram[id]->fram_id*FRAM_BUF_SIZE, obj->pfram[id]->buf, FRAM_BUF_SIZE);
        memcpy(obj->pbuf + obj->pfram[id+1]->fram_id*FRAM_BUF_SIZE, obj->pfram[id+1]->buf, FRAM_BUF_SIZE);
#else    
        memset(obj->pbuf + obj->pfram[id]->fram_id*FRAM_BUF_SIZE, 0, FRAM_BUF_SIZE);
        memset(obj->pbuf + obj->pfram[id+1]->fram_id*FRAM_BUF_SIZE, 0x33, FRAM_BUF_SIZE);
#endif

#if USE_FRAM_LOCK
        if(obj->pfram[id+1]->fram_id == 39) {
            fram_status = FRAM_LOCK;
        }
#endif
    }
}

void take_fram_lock(void)
{
#if USE_FRAM_LOCK
    while(fram_status != FRAM_LOCK) vTaskDelay(10/portTICK_PERIOD_MS);
#endif
}

void give_fram_lock(void)
{
#if USE_FRAM_LOCK
    fram_status = FRAM_UNLOCK;
#endif
}

uint8_t * i2s_camera_attach(void)
{
    cam_obj = camera_dma_create();
    for (int i = 0; i < DMA_NODE_CNT; i++) {
        _dma[i].size = FRAM_BUF_SIZE;
        _dma[i].length = FRAM_BUF_SIZE;
        _dma[i].eof = 1;
        _dma[i].owner = 1;
        _dma[i].buf = cam_obj->pfram[i]->buf;
        _dma[i].empty = &_dma[(i + 1)%DMA_NODE_CNT];
    }
    xTaskCreate(camera_fram_copy_task, "camera_fram_copy_task", 1024*4, (void *)cam_obj, 8, NULL);
    i2s_camera_config();
    I2S0.in_link.addr = ((uint32_t)&_dma[0]) & 0xfffff;
    I2S0.rx_eof_num = FRAM_BUF_SIZE * 2;
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, 0, i2s_cam_isr, (void *)cam_obj, NULL);
    return cam_obj->pbuf;
}