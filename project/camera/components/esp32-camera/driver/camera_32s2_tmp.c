// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "time.h"
#include "sys/time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp32s2beta/rom/lldesc.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/periph_ctrl.h"
#include "esp_intr_alloc.h"
#include "sensor.h"
#include "sccb.h"
#include "esp_camera.h"
#include "camera_common.h"
#include "xclk.h"
#if CONFIG_OV2640_SUPPORT
#include "ov2640.h"
#endif
#if CONFIG_OV7725_SUPPORT
#include "ov7725.h"
#endif
#if CONFIG_OV3660_SUPPORT
#include "ov3660.h"
#endif

typedef void (*dma_filter_t)(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);

typedef enum {
	CAMERA_NONE = 0,
	CAMERA_UNKNOWN = 1,
	CAMERA_OV7725 = 7725,
	CAMERA_OV2640 = 2640,
	CAMERA_OV3660 = 3660,
} camera_model_t;

typedef struct camera_fb_s {
    uint8_t * buf;
    size_t len;
    size_t width;
    size_t height;
    pixformat_t format;
    size_t size;
    uint8_t ref;
    uint8_t bad;
    struct camera_fb_s * next;
} camera_fb_int_t;

typedef struct {
	camera_config_t config;
	sensor_t sensor;

	camera_fb_int_t *fb;
	size_t fb_size;
	size_t data_size;

	size_t width;
	size_t height;
	size_t in_bytes_per_pixel;
	size_t fb_bytes_per_pixel;

	size_t dma_received_count;
	size_t dma_filtered_count;
	size_t dma_per_line;
	size_t dma_buf_width;
	size_t dma_sample_count;

	lldesc_t *dma_desc;
	dma_elem_t **dma_buf;
	size_t dma_desc_count;
	size_t dma_desc_cur;

	dma_filter_t dma_filter;
	intr_handle_t i2s_intr_handle;
	QueueHandle_t data_ready;
	QueueHandle_t fb_in;
	QueueHandle_t fb_out;

	TaskHandle_t dma_filter_task;
} camera_state_t;

camera_state_t* s_state = NULL;

typedef union {
    struct {
        uint32_t dma_buf_id: 8;
        uint32_t fram_id:   16;
        uint32_t fram_sta:   8;
    };
    uint32_t va;
} dma_fram_item_t;

#define REG_PID        0x0A
#define REG_VER        0x0B
#define REG_MIDH       0x1C
#define REG_MIDL       0x1D

#define REG16_CHIDH     0x300A
#define REG16_CHIDL     0x300B

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char* TAG = "camera";
#endif

#define DMA_BUF_MAX_SIZE   (4095)
#define DMA_DESC_CNT       (10)

//Invalid dma fram id
#define INVAL_FRAM_ID (0xffff)

static bool is_hs_mode(void)
{
    return s_state->config.xclk_freq_hz > 10000000;
}

static inline void IRAM_ATTR i2s_conf_reset()
{
    I2S0.lc_conf.in_rst  = 1;
    I2S0.lc_conf.in_rst  = 0;
    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;

    I2S0.conf.rx_reset = 1;
    I2S0.conf.rx_reset = 0;
    I2S0.conf.rx_fifo_reset = 1;
    I2S0.conf.rx_fifo_reset = 0;
    while ( GET_PERI_REG_MASK(I2S_CONF_REG(0), I2S_RX_FIFO_RESET_ST_M) );
}


static void IRAM_ATTR i2s_isr(void *param)
{
	static int tagel = 0x1;
	static int  fram_id = 0;
    static int dma_buf_id = 0;
	typeof(I2S0.int_st) int_st = I2S0.int_st;
	I2S0.int_clr.val = int_st.val;
	portBASE_TYPE high_priority_task_awoken = 0;
    dma_fram_item_t fram_item;

	if (int_st.in_suc_eof) {
		// ets_printf("i2s isr\n");
		// gpio_set_level(34, tagel);
		// tagel ^= 0x1;
        fram_item.dma_buf_id = dma_buf_id;
        fram_item.fram_id = fram_id;
        // ets_printf("%d  %d\n", fram_item.dma_buf_id , fram_item.fram_id);
		xQueueSendFromISR(s_state->data_ready, (void *)&fram_item, &high_priority_task_awoken);
        fram_id += 2;
        dma_buf_id += 2;
		if(dma_buf_id == DMA_DESC_CNT) dma_buf_id = 0;
        if(fram_id == 40) fram_id = 0;
	}
	if (high_priority_task_awoken == pdTRUE) {
		portYIELD_FROM_ISR();
	}
}

static void IRAM_ATTR vsync_isr(void* arg)
{
   ets_printf("vsync isr\n");
}

static void IRAM_ATTR vsync_intr_disable(void)
{
    gpio_set_intr_type(s_state->config.pin_vsync, GPIO_INTR_DISABLE);
}

static void vsync_intr_enable(void)
{
    gpio_set_intr_type(s_state->config.pin_vsync, GPIO_INTR_NEGEDGE);
}

static void IRAM_ATTR i2s_start_bus()
{
    s_state->dma_desc_cur = 0;
    s_state->dma_received_count = 0;
    I2S0.int_ena.val = 0;
    //s_state->dma_filtered_count = 0;
    I2S0.fifo_conf.dscr_en = 1;
    I2S0.conf2.cam_sync_fifo_reset = 0;
    I2S0.int_clr.val = I2S0.int_raw.val;
    I2S0.rx_eof_num = s_state->dma_sample_count;
    I2S0.int_ena.in_suc_eof = 1;
    I2S0.in_link.addr = (uint32_t) &s_state->dma_desc[0];
    I2S0.in_link.start = 1;
    I2S0.conf.rx_start = 1;
    if (s_state->config.pixel_format == PIXFORMAT_JPEG) {
        vsync_intr_enable();
    }
}

static int i2s_run(void)
{
	for (int i = 0; i < s_state->dma_desc_count; ++i) {
        lldesc_t* d = &s_state->dma_desc[i];
        ESP_LOGV(TAG, "DMA desc %2d: %u %u %u %u %u %u %p %p",
                 i, d->length, d->size, d->offset, d->eof, d->sosf, d->owner, d->buf, d->qe.stqe_next);
        memset(s_state->dma_buf[i], 0, d->length);
    }
    // wait for frame
    camera_fb_int_t * fb = s_state->fb;
    while (s_state->config.fb_count > 1) {
        while (s_state->fb->ref && s_state->fb->next != fb) {
            s_state->fb = s_state->fb->next;
        }
        if (s_state->fb->ref == 0) {
            break;
        }
    }
    esp_intr_enable(s_state->i2s_intr_handle);
    i2s_conf_reset();
    // gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);
    i2s_start_bus();
    return 0;
}

static void i2s_init(void)
{
    camera_config_t* config = &s_state->config;

    // Configure input GPIOs
    gpio_num_t pins[] = {
        config->pin_d7,
        config->pin_d6,
        config->pin_d5,
        config->pin_d4,
        config->pin_d3,
        config->pin_d2,
        config->pin_d1,
        config->pin_d0,
        config->pin_vsync,
        config->pin_href,
        config->pin_pclk
    };
    gpio_config_t conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    for (int i = 0; i < sizeof(pins) / sizeof(gpio_num_t); ++i) {
        if (rtc_gpio_is_valid_gpio(pins[i])) {
            rtc_gpio_deinit(pins[i]);
        }
        conf.pin_bit_mask |= (1LL << pins[i]);
	}
    
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[34], PIN_FUNC_GPIO);
    gpio_set_direction(34, GPIO_MODE_OUTPUT);

    gpio_config(&conf);

    // Route input GPIOs to I2S peripheral using GPIO matrix
    // gpio_matrix_in(0x30, I2S0I_H_ENABLE_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);
    gpio_matrix_in(config->pin_href, I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(config->pin_vsync, I2S0I_V_SYNC_IDX, true);
    gpio_matrix_in(config->pin_pclk, I2S0I_WS_IN_IDX, false);

    gpio_matrix_in(config->pin_d0, I2S0I_DATA_IN8_IDX, false);
    gpio_matrix_in(config->pin_d1, I2S0I_DATA_IN9_IDX, false);
    gpio_matrix_in(config->pin_d2, I2S0I_DATA_IN10_IDX, false);
    gpio_matrix_in(config->pin_d3, I2S0I_DATA_IN11_IDX, false);
    gpio_matrix_in(config->pin_d4, I2S0I_DATA_IN12_IDX, false);
    gpio_matrix_in(config->pin_d5, I2S0I_DATA_IN13_IDX, false);
    gpio_matrix_in(config->pin_d6, I2S0I_DATA_IN14_IDX, false);
    gpio_matrix_in(config->pin_d7, I2S0I_DATA_IN15_IDX, false);

    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);
    // Toggle some reset bits in LC_CONF register
    // Toggle some reset bits in CONF register

    I2S0.clkm_conf.val = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a = 0;
    I2S0.clkm_conf.clk_sel = 2;
    I2S0.clkm_conf.clk_en = 1;
    I2S0.conf.val = 0;
    I2S0.conf1.val = 0;
    I2S0.conf1.rx_pcm_bypass = 1;
    I2S0.sample_rate_conf.val = 0;
    I2S0.sample_rate_conf.rx_bck_div_num = 2;
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
    I2S0.sample_rate_conf.rx_bits_mod = 8;
    I2S0.fifo_conf.rx_fifo_mod = 1;
    I2S0.conf_chan.rx_chan_mod = 1;
    i2s_conf_reset();
    I2S0.conf2.cam_sync_fifo_reset = 1;
    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0;

    // Allocate I2S interrupt, keep it disabled
   esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_IRAM, i2s_isr, NULL, &s_state->i2s_intr_handle);
}

camera_fb_t* esp_camera_fb_get(void)
{
	if (s_state == NULL) {
		return NULL;
	}
	if (!I2S0.conf.rx_start) {
        camera_fb_int_t * fb_tmp = s_state->fb;
        for (int i = 0; i < s_state->config.fb_count; i++) {
            xQueueSend(s_state->fb_in, &fb_tmp, portMAX_DELAY);
            fb_tmp = fb_tmp->next; 
        }
		if (i2s_run() != 0) {
			return NULL;
		}
	}
	camera_fb_int_t * fb = NULL;
	if (s_state->fb_out) {
		xQueueReceive(s_state->fb_out, &fb, portMAX_DELAY);
	}
	return (camera_fb_t*)fb;
}

void esp_camera_fb_return(camera_fb_t * fb)
{
    if (fb == NULL || s_state == NULL || s_state->fb_in == NULL) {
        return;
    }
    xQueueSend(s_state->fb_in, &fb, portMAX_DELAY);
}

sensor_t * esp_camera_sensor_get()
{
    if (s_state == NULL) {
        return NULL;
    }
    return &s_state->sensor;
}

typedef enum {
    FRAM_BUF_IDLE = 1,
    FRAM_BUF_IN_PROGRESS = 2,
} fram_buf_sta_t;

static void dma_filter_task(void *param)
{
	// Check if it is the expected frame number
	int expect_id = 0;
    dma_fram_item_t fram_item;
	camera_fb_int_t * fb = NULL;
	//frame status
	fram_buf_sta_t fb_sta = FRAM_BUF_IDLE;
	while (1) {
		xQueueReceive(s_state->fb_in, &fb, portMAX_DELAY);
        while(1) {
        	//wait dma fram buf
            xQueueReceive(s_state->data_ready, (void *)&fram_item, portMAX_DELAY);
            //Not the start of a frame
            if( fb_sta == FRAM_BUF_IDLE  && fram_item.fram_id != 0) {
            	continue;
            }
            // If the queue is full, we may lose some frames
            if(expect_id != fram_item.fram_id) {
                ESP_LOGD(TAG, "Bad fram\n");
            	expect_id = 0;
            	fb_sta = FRAM_BUF_IDLE;
            	continue;
            }
            fb_sta = FRAM_BUF_IN_PROGRESS;
            memcpy(fb->buf+fram_item.fram_id*s_state->dma_buf_width, s_state->dma_buf[fram_item.dma_buf_id], s_state->dma_buf_width);
            memcpy(fb->buf+(fram_item.fram_id+1)*s_state->dma_buf_width, s_state->dma_buf[fram_item.dma_buf_id+1], s_state->dma_buf_width);
            expect_id += 2;
            if(fram_item.fram_id == 38) {
            	expect_id = 0;
            	fb_sta = FRAM_BUF_IDLE;
            	xQueueSend(s_state->fb_out, &fb, portMAX_DELAY);
            	break;
            }
        }    
	}
}

static esp_err_t dma_desc_init(void)
{
	size_t line_size = s_state->width * s_state->in_bytes_per_pixel;
	size_t dma_per_line = ((DMA_BUF_MAX_SIZE / line_size) & 0xfe);
	size_t buf_size = line_size * dma_per_line;
	size_t dma_desc_count = DMA_DESC_CNT;
	s_state->dma_buf_width = buf_size;
	s_state->dma_per_line = dma_per_line;
	s_state->dma_desc_count = dma_desc_count;
	ESP_LOGD(TAG, "DMA buffer size: %d, DMA buffers per line: %d", buf_size, dma_per_line);
	ESP_LOGD(TAG, "DMA buffer count: %d", dma_desc_count);
	ESP_LOGD(TAG, "DMA buffer total: %d bytes", buf_size * dma_desc_count);

	s_state->dma_buf = (dma_elem_t**) malloc(sizeof(dma_elem_t*) * dma_desc_count);
	if (s_state->dma_buf == NULL) {
		return ESP_ERR_NO_MEM;
	}
	s_state->dma_desc = (lldesc_t*) malloc(sizeof(lldesc_t) * dma_desc_count);
	if (s_state->dma_desc == NULL) {
		return ESP_ERR_NO_MEM;
	}
	memset(s_state->dma_desc, 0, sizeof(lldesc_t) * dma_desc_count);
	for (int i = 0; i < dma_desc_count; ++i) {
		ESP_LOGD(TAG, "Allocating DMA buffer #%d, size=%d", i, buf_size);
		dma_elem_t* buf = (dma_elem_t*) malloc(buf_size);
		if (buf == NULL) {
			return ESP_ERR_NO_MEM;
		}
		s_state->dma_buf[i] = buf;
		ESP_LOGV(TAG, "dma_buf[%d]=%p", i, buf);
        
        s_state->dma_desc[i].size = buf_size;
        s_state->dma_desc[i].length = buf_size;
		s_state->dma_desc[i].owner = 1;
		s_state->dma_desc[i].buf = (uint8_t*)buf;
		s_state->dma_desc[i].eof = 1;
		s_state->dma_desc[i].empty = &s_state->dma_desc[(i + 1) % dma_desc_count];
	}
	s_state->dma_sample_count = buf_size * 2; //reduce the isr count
	return ESP_OK;
}

static void camera_fb_deinit(void)
{
    camera_fb_int_t * _fb1 = s_state->fb, * _fb2 = NULL;
    while (s_state->fb) {
        _fb2 = s_state->fb;
        s_state->fb = _fb2->next;
        if (_fb2->next == _fb1) {
            s_state->fb = NULL;
        }
        free(_fb2->buf);
        free(_fb2);
    }
}

static void dma_desc_deinit(void)
{
    if (s_state->dma_buf) {
        for (int i = 0; i < s_state->dma_desc_count; ++i) {
            free(s_state->dma_buf[i]);
        }
    }
    free(s_state->dma_buf);
    free(s_state->dma_desc);
}

static esp_err_t camera_fb_init(size_t count)
{
    if (!count) {
        return ESP_ERR_INVALID_ARG;
    }

    camera_fb_deinit();

    ESP_LOGI(TAG, "Allocating %u frame buffers (%d KB total)", count, (s_state->fb_size * count) / 1024);

    camera_fb_int_t * _fb = NULL, * _fb1 = NULL, * _fb2 = NULL;
    for (int i = 0; i < count; i++) {
        _fb2 = (camera_fb_int_t *)malloc(sizeof(camera_fb_int_t));
        if (!_fb2) {
            goto fail;
        }
        memset(_fb2, 0, sizeof(camera_fb_int_t));
        _fb2->size = s_state->fb_size;

        ESP_LOGI(TAG, "Allocating %d KB frame buffer in PSRAM", s_state->fb_size / 1024);
        _fb2->buf = (uint8_t*) heap_caps_calloc(_fb2->size, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!_fb2->buf) {
            free(_fb2);
            ESP_LOGE(TAG, "Allocating %d KB frame buffer Failed", s_state->fb_size / 1024);
            goto fail;
        } else {
            ESP_LOGI(TAG, "Allocating frame buffers-%d  at %p", i, _fb2->buf);    
        }
        memset(_fb2->buf, 0, _fb2->size);
        _fb2->next = _fb;
        _fb = _fb2;
        if (!i) {
            _fb1 = _fb2;
        }
    }
    if (_fb1) {
        _fb1->next = _fb;
    }

    s_state->fb = _fb;//load first buffer

    return ESP_OK;

fail:
    while (_fb) {
        _fb2 = _fb;
        _fb = _fb->next;
        free(_fb2->buf);
        free(_fb2);
    }
    return ESP_ERR_NO_MEM;
}

static esp_err_t camera_init(const camera_config_t* config)
{
    if (!s_state) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state->sensor.id.PID == 0) {
        return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }
    memcpy(&s_state->config, config, sizeof(*config));
    esp_err_t err = ESP_OK;
    framesize_t frame_size = (framesize_t) config->frame_size;
    pixformat_t pix_format = (pixformat_t) config->pixel_format;
    s_state->width = resolution[frame_size][0];
    s_state->height = resolution[frame_size][1];
    if (pix_format == PIXFORMAT_GRAYSCALE) {
        s_state->fb_size = s_state->width * s_state->height;
        if (s_state->sensor.id.PID == OV3660_PID) {
            if (is_hs_mode()) {
                // s_state->sampling_mode = SM_0A00_0B00;
                // s_state->dma_filter = &dma_filter_yuyv_highspeed;
            } else {
                // s_state->sampling_mode = SM_0A0B_0C0D;
                // s_state->dma_filter = &dma_filter_yuyv;
            }
            s_state->in_bytes_per_pixel = 1;       // camera sends Y8
        } else {
            if (is_hs_mode() && s_state->sensor.id.PID != OV7725_PID) {
                // s_state->sampling_mode = SM_0A00_0B00;
                // s_state->dma_filter = &dma_filter_grayscale_highspeed;
            } else {
                // s_state->sampling_mode = SM_0A0B_0C0D;
                // s_state->dma_filter = &dma_filter_grayscale;
            }
            s_state->in_bytes_per_pixel = 2;       // camera sends YU/YV
        }
        s_state->fb_bytes_per_pixel = 1;       // frame buffer stores Y8
    } else if (pix_format == PIXFORMAT_YUV422 || pix_format == PIXFORMAT_RGB565) {
        s_state->fb_size = s_state->width * s_state->height * 2;
        if (is_hs_mode() && s_state->sensor.id.PID != OV7725_PID) {
            // s_state->sampling_mode = SM_0A00_0B00;
            // s_state->dma_filter = &dma_filter_yuyv_highspeed;
        } else {
            // s_state->sampling_mode = SM_0A0B_0C0D;
            // s_state->dma_filter = &dma_filter_yuyv;
        }
        s_state->in_bytes_per_pixel = 2;       // camera sends YU/YV
        s_state->fb_bytes_per_pixel = 2;       // frame buffer stores YU/YV/RGB565
    } else if (pix_format == PIXFORMAT_RGB888) {
        s_state->fb_size = s_state->width * s_state->height * 3;
        if (is_hs_mode()) {
            // s_state->sampling_mode = SM_0A00_0B00;
            // s_state->dma_filter = &dma_filter_rgb888_highspeed;
        } else {
            // s_state->sampling_mode = SM_0A0B_0C0D;
            // s_state->dma_filter = &dma_filter_rgb888;
        }
        s_state->in_bytes_per_pixel = 2;       // camera sends RGB565
        s_state->fb_bytes_per_pixel = 3;       // frame buffer stores RGB888
    } else if (pix_format == PIXFORMAT_JPEG) {
        if (s_state->sensor.id.PID != OV2640_PID && s_state->sensor.id.PID != OV3660_PID) {
            ESP_LOGE(TAG, "JPEG format is only supported for ov2640 and ov3660");
            err = ESP_ERR_NOT_SUPPORTED;
            goto fail;
        }
        int qp = config->jpeg_quality;
        int compression_ratio_bound = 1;
        if (qp > 10) {
            compression_ratio_bound = 16;
        } else if (qp > 5) {
            compression_ratio_bound = 10;
        } else {
            compression_ratio_bound = 4;
        }
        (*s_state->sensor.set_quality)(&s_state->sensor, qp);
        s_state->in_bytes_per_pixel = 2;
        s_state->fb_bytes_per_pixel = 2;
        s_state->fb_size = (s_state->width * s_state->height * s_state->fb_bytes_per_pixel) / compression_ratio_bound;
        // s_state->dma_filter = &dma_filter_jpeg;
        // s_state->sampling_mode = SM_0A00_0B00;
    } else {
        ESP_LOGE(TAG, "Requested format is not supported");
        err = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }

    // ESP_LOGD(TAG, "in_bpp: %d, fb_bpp: %d, fb_size: %d, mode: %d, width: %d height: %d",
    //          s_state->in_bytes_per_pixel, s_state->fb_bytes_per_pixel,
    //          s_state->fb_size, s_state->sampling_mode,
    //          s_state->width, s_state->height);

    i2s_init();

    err = dma_desc_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S and DMA");
        goto fail;
    }

    //s_state->fb_size = 75 * 1024;
    err = camera_fb_init(s_state->config.fb_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer");
        goto fail;
    }
    s_state->data_ready = xQueueCreate(DMA_DESC_CNT, sizeof(dma_fram_item_t));
    if (s_state->data_ready == NULL) {
        ESP_LOGE(TAG, "Failed to dma queue");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    s_state->fb_in = xQueueCreate(s_state->config.fb_count, sizeof(camera_fb_t *));
    s_state->fb_out = xQueueCreate(s_state->config.fb_count, sizeof(camera_fb_t *));
    if (s_state->fb_in == NULL || s_state->fb_out == NULL) {
        ESP_LOGE(TAG, "Failed to fb queues");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    if (!xTaskCreate(&dma_filter_task, "dma_filter", 4096, NULL, 10, &s_state->dma_filter_task))
    {
        ESP_LOGE(TAG, "Failed to create DMA filter task");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    if(pix_format == PIXFORMAT_JPEG) {
	    vsync_intr_disable();
	    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM);
	    err = gpio_isr_handler_add(s_state->config.pin_vsync, &vsync_isr, NULL);
	    if (err != ESP_OK) {
	        ESP_LOGE(TAG, "vsync_isr_handler_add failed (%x)", err);
	        goto fail;
	    }
	    vsync_intr_enable();
    }

    s_state->sensor.status.framesize = frame_size;
    s_state->sensor.pixformat = pix_format;
    ESP_LOGD(TAG, "Setting frame size to %dx%d", s_state->width, s_state->height);
    if (s_state->sensor.set_framesize(&s_state->sensor, frame_size) != 0) {
        ESP_LOGE(TAG, "Failed to set frame size");
        err = ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE;
        goto fail;
    }
    s_state->sensor.set_pixformat(&s_state->sensor, pix_format);

    if (s_state->sensor.id.PID == OV2640_PID) {
        s_state->sensor.set_gainceiling(&s_state->sensor, GAINCEILING_2X);
        s_state->sensor.set_bpc(&s_state->sensor, false);
        s_state->sensor.set_wpc(&s_state->sensor, true);
        s_state->sensor.set_lenc(&s_state->sensor, true);
    }
    //todo: for some reason the first set of the quality does not work.
    if (pix_format == PIXFORMAT_JPEG) {
        (*s_state->sensor.set_quality)(&s_state->sensor, config->jpeg_quality);
    }
    s_state->sensor.init_status(&s_state->sensor);
    return ESP_OK;

fail:
    esp_camera_deinit();
    return err;
}

/*
 * Public Methods
 * */

static esp_err_t camera_probe(const camera_config_t* config, camera_model_t* out_camera_model)
{
    if (s_state != NULL) {
    	ESP_LOGE(TAG, "Maybe esp_camera_deinit should be called first");
        return ESP_ERR_INVALID_STATE;
    }

    s_state = (camera_state_t*) calloc(sizeof(*s_state), 1);
    if (!s_state) {
        return ESP_ERR_NO_MEM;
    }
    memset(s_state, 0, sizeof(*s_state));
    ESP_LOGD(TAG, "Enabling XCLK output");
    camera_enable_out_clock(config);

    ESP_LOGD(TAG, "Initializing SSCB");
    SCCB_Init(config->pin_sscb_sda, config->pin_sscb_scl);

    if (config->pin_pwdn >= 0) {
        ESP_LOGD(TAG, "Resetting camera by power down line");
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << config->pin_pwdn;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        // carefull, logic is inverted compared to reset pin
        gpio_set_level(config->pin_pwdn, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(config->pin_pwdn, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    if (config->pin_reset >= 0) {
        ESP_LOGD(TAG, "Resetting camera");
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << config->pin_reset;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        gpio_set_level(config->pin_reset, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(config->pin_reset, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
#if (CONFIG_OV2640_SUPPORT && !CONFIG_OV3660_SUPPORT)
    } else {
        //reset OV2640
        SCCB_Write(0x30, 0xFF, 0x01);//bank sensor
        SCCB_Write(0x30, 0x12, 0x80);//reset
        vTaskDelay(10 / portTICK_PERIOD_MS);
#endif
    }

    ESP_LOGD(TAG, "Searching for camera address");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uint8_t slv_addr = SCCB_Probe();
    if (slv_addr == 0) {
        *out_camera_model = CAMERA_NONE;
        camera_disable_out_clock();
        return ESP_ERR_CAMERA_NOT_DETECTED;
    }
    s_state->sensor.slv_addr = slv_addr;
    s_state->sensor.xclk_freq_hz = config->xclk_freq_hz;

    //s_state->sensor.slv_addr = 0x30;
    ESP_LOGD(TAG, "Detected camera at address=0x%02x", s_state->sensor.slv_addr);
    sensor_id_t* id = &s_state->sensor.id;

#if (CONFIG_OV2640_SUPPORT && CONFIG_OV3660_SUPPORT)
    if (slv_addr == 0x30) {
        ESP_LOGD(TAG, "Resetting OV2640");
        //camera might be OV2640. try to reset it
        SCCB_Write(0x30, 0xFF, 0x01);//bank sensor
        SCCB_Write(0x30, 0x12, 0x80);//reset
        vTaskDelay(10 / portTICK_PERIOD_MS);
        slv_addr = SCCB_Probe();
    }
#endif

#if CONFIG_OV3660_SUPPORT
    if (s_state->sensor.slv_addr == 0x3c) {
        id->PID = SCCB_Read16(s_state->sensor.slv_addr, REG16_CHIDH);
        id->VER = SCCB_Read16(s_state->sensor.slv_addr, REG16_CHIDL);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_LOGD(TAG, "Camera PID=0x%02x VER=0x%02x", id->PID, id->VER);
    } else {
#endif
        id->PID = SCCB_Read(s_state->sensor.slv_addr, REG_PID);
        id->VER = SCCB_Read(s_state->sensor.slv_addr, REG_VER);
        id->MIDL = SCCB_Read(s_state->sensor.slv_addr, REG_MIDL);
        id->MIDH = SCCB_Read(s_state->sensor.slv_addr, REG_MIDH);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_LOGD(TAG, "Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x",
                 id->PID, id->VER, id->MIDH, id->MIDL);
#if CONFIG_OV3660_SUPPORT
    }
#endif

    switch (id->PID) {
#if CONFIG_OV2640_SUPPORT
        case OV2640_PID:
            *out_camera_model = CAMERA_OV2640;
            ov2640_init(&s_state->sensor);
            break;
#endif
#if CONFIG_OV7725_SUPPORT
        case OV7725_PID:
            *out_camera_model = CAMERA_OV7725;
            ov7725_init(&s_state->sensor);
            break;
#endif
#if CONFIG_OV3660_SUPPORT
        case OV3660_PID:
            *out_camera_model = CAMERA_OV3660;
            ov3660_init(&s_state->sensor);
            break;
#endif
        default:
            id->PID = 0;
            *out_camera_model = CAMERA_UNKNOWN;
            camera_disable_out_clock();
            ESP_LOGE(TAG, "Detected camera not supported.");
            return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }

    ESP_LOGD(TAG, "Doing SW reset of sensor");
    s_state->sensor.reset(&s_state->sensor);

    return ESP_OK;
}

esp_err_t esp_camera_init(const camera_config_t* config)
{
    camera_model_t camera_model = CAMERA_NONE;
    esp_err_t err = camera_probe(config, &camera_model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
        goto fail;
    }
    if (camera_model == CAMERA_OV7725) {
        ESP_LOGD(TAG, "Detected OV7725 camera");
        if (config->pixel_format == PIXFORMAT_JPEG) {
            ESP_LOGE(TAG, "Camera does not support JPEG");
            err = ESP_ERR_CAMERA_NOT_SUPPORTED;
            goto fail;
        }
    } else if (camera_model == CAMERA_OV2640) {
        ESP_LOGD(TAG, "Detected OV2640 camera");
    } else if (camera_model == CAMERA_OV3660) {
        ESP_LOGD(TAG, "Detected OV3660 camera");
    } else {
        ESP_LOGE(TAG, "Camera not supported");
        err = ESP_ERR_CAMERA_NOT_SUPPORTED;
        goto fail;
    }
    err = camera_init(config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }
    return ESP_OK;

fail:
    free(s_state);
    s_state = NULL;
    camera_disable_out_clock();
    return err;
}

esp_err_t esp_camera_deinit(void)
{
    if (s_state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state->dma_filter_task) {
        vTaskDelete(s_state->dma_filter_task);
    }
    if (s_state->data_ready) {
        vQueueDelete(s_state->data_ready);
    }
    if (s_state->fb_in) {
        vQueueDelete(s_state->fb_in);
    }
    if (s_state->fb_out) {
        vQueueDelete(s_state->fb_out);
    }
    gpio_isr_handler_remove(s_state->config.pin_vsync);
    if (s_state->i2s_intr_handle) {
        esp_intr_disable(s_state->i2s_intr_handle);
        esp_intr_free(s_state->i2s_intr_handle);
    }
    dma_desc_deinit();
    camera_fb_deinit();
    free(s_state);
    s_state = NULL;
    camera_disable_out_clock();
    periph_module_disable(PERIPH_I2S0_MODULE);
    return ESP_OK;
}
