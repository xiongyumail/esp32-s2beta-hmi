#if 1
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/apps/sntp.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <math.h>
#include "lvgl.h"
#include "lcd.h"
#include "gui.h"
#include "encoder.h"
#include "WS2812B.h"

static const char *TAG = "main";

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            gui_set_wifi_state(true, 1000);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            gui_set_wifi_state(false, 1000);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {};  
    memcpy(wifi_config.sta.ssid, CONFIG_WIFI_SSID, strlen(CONFIG_WIFI_SSID)+1);
    memcpy(wifi_config.sta.password, CONFIG_WIFI_PASSWORD, strlen(CONFIG_WIFI_PASSWORD)+1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG, "start the WIFI SSID:[%s]", CONFIG_WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Waiting for wifi");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}

static lv_disp_t *disp[2];
static lv_indev_t *indev[1];

bool cam_flag = 0;

static void IRAM_ATTR lv_disp_flush1(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    uint32_t len = (sizeof(uint16_t) * ((area->y2 - area->y1 + 1)*(area->x2 - area->x1 + 1)));

    if (cam_flag == 0) {
        lcd_take(0);
        lcd_set_index(area->x1, area->y1, area->x2, area->y2);
        lcd_write_data((uint16_t *)color_p, len);
        lcd_give();
    }

    lv_disp_flush_ready(disp_drv);
}

static void IRAM_ATTR lv_disp_flush2(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    uint32_t len = (sizeof(uint16_t) * ((area->y2 - area->y1 + 1)*(area->x2 - area->x1 + 1)));

    lcd_take(1);
    lcd_set_index(area->x1, area->y1, area->x2, area->y2);
    lcd_write_data((uint16_t *)color_p, len);
    lcd_give();

    lv_disp_flush_ready(disp_drv);
}

bool lv_encoder_read(lv_indev_drv_t * drv, lv_indev_data_t*data)
{
  data->enc_diff = encoder_get_new_moves();
  if(encoder_get_button_state()) {
      data->state = LV_INDEV_STATE_PR;
  } else {
      data->state = LV_INDEV_STATE_REL;
  }
  
  return false; /*No buffering now so no more data read*/
}

static void lv_memory_monitor(lv_task_t * param)
{
    (void) param; /*Unused*/

    lv_mem_monitor_t mon;
    lv_mem_monitor(&mon);
    printf("used: %6d (%3d %%), frag: %3d %%, biggest free: %6d, system free: %d/%d\n", (int)mon.total_size - mon.free_size,
           mon.used_pct,
           mon.frag_pct,
           (int)mon.free_biggest_size,
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL), esp_get_free_heap_size());
}

static void lv_tick_task(void * arg)
{
    while(1) {
        lv_tick_inc(10);
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

static void gui_task(void *arg)
{
    encoder_init();
    lcd_init();
    
    xTaskCreate(lv_tick_task, "lv_tick_task", 1024, NULL, 5, NULL);

    lv_init();

    /*Create a display buffer*/
    static lv_disp_buf_t disp_buf1;
    static lv_color_t *buf1_1 = NULL;
    buf1_1 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * (240 * 240), MALLOC_CAP_SPIRAM);
    lv_disp_buf_init(&disp_buf1, buf1_1, NULL, 240 * 240);

    /*Create a display*/
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
    disp_drv.buffer = &disp_buf1;
    disp_drv.flush_cb = lv_disp_flush1;    /*Used when `LV_VDB_SIZE != 0` in lv_conf.h (buffered drawing)*/
    disp[0] = lv_disp_drv_register(&disp_drv);

    /*Create an other buffer for double buffering*/
    static lv_disp_buf_t disp_buf2;
    static lv_color_t *buf2_1 = NULL;
    buf2_1 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * (240 * 135), MALLOC_CAP_SPIRAM);
    lv_disp_buf_init(&disp_buf2, buf2_1, NULL, 240 * 135);

    /*Create an other display*/
    lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
    disp_drv.buffer = &disp_buf2;
    disp_drv.flush_cb = lv_disp_flush2;    /*Used when `LV_VDB_SIZE != 0` in lv_conf.h (buffered drawing)*/
    disp_drv.hor_res = 135;
    disp_drv.ver_res = 240;
    disp[1] = lv_disp_drv_register(&disp_drv);

    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);      /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = lv_encoder_read;
    /*Register the driver in LittlevGL and save the created input device object*/
    indev[0] = lv_indev_drv_register(&indev_drv);

    lv_disp_set_default(disp[0]);

    lv_task_create(lv_memory_monitor, 3000, LV_TASK_PRIO_MID, NULL);

    gui_init(disp, indev, lv_theme_material_init(0, NULL));

    while(1) {
        lv_task_handler();
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

uint16_t *lcd_cam_buffer = NULL;

void IRAM_ATTR camera_trans(uint8_t* src, uint32_t fb_size)
{
    int x, y;
    int i = 0;
    for (y = 239; y >= 0; y--) {
        i += 40 * 2;
        for (x = 239; x >= 0; x--) {
            lcd_cam_buffer[y*240 + x] = (src[i+0] << 8) | (src[i+1]);
            i += 2;
        }
        i += 40 * 2;
    }
    lcd_take(0);
    lcd_set_index(0, 0, 239, 239);
    lcd_write_data(lcd_cam_buffer, 240*240*2);
    lcd_give();
    // i = 0;
    // for (y = 239; y >= 0; y--) {
    //     i += 90 * 2;
    //     for (x = 134; x >= 0; x--) {
    //         lcd_cam_buffer[y*135 + x] = (src[i+1] << 8) | (src[i+0]);
    //         i += 2;
    //     }
    //     i += 95 * 2;
    // }
    // lcd_take(1);
    // lcd_set_index(0, 40, 134, 239);
    // lcd_write_data(lcd_cam_buffer, 200*135*2);
    // lcd_give();
}

#include "esp_camera.h"

//WROVER-KIT PIN Map
#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   -1 //software reset will be performed
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    15
#define CAM_PIN_SIOC    16

#define CAM_PIN_D7      11
#define CAM_PIN_D6      10
#define CAM_PIN_D5       9
#define CAM_PIN_D4       8
#define CAM_PIN_D3       7
#define CAM_PIN_D2      17
#define CAM_PIN_D1      21
#define CAM_PIN_D0      18
#define CAM_PIN_VSYNC   14
#define CAM_PIN_HREF    13
#define CAM_PIN_PCLK    12

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_RGB565,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,//QQVGA-QXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1 //if more than one, i2s runs in continuous mode. Use only with JPEG
};

wsRGB_t rgb = {0x0, 0x0, 0x00};

esp_err_t camera_init()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    ESP_LOGI(TAG, "Camera Init Done");
    return ESP_OK;
}

esp_err_t camera_capture()
{
    //acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        cam_flag = 0;
        rgb.r = 0xFF;
        rgb.b = 0x00;
        rgb.g = 0x00;
        WS2812B_setLeds(&rgb, 1);
        return ESP_FAIL;
    }
    cam_flag = 1;
    camera_trans(fb->buf, 320*240*2);
    esp_camera_fb_return(fb);
    return ESP_OK;
}

void app_main()
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
    setenv("TZ", "CST-8", 1);
    tzset();

    WS2812B_init(RMT_CHANNEL_0, GPIO_NUM_4, 1);
    rgb.r = 0x0F;
    WS2812B_setLeds(&rgb, 1);

    camera_init();
    // lcd_init();
    xTaskCreate(gui_task, "gui_task", 4096, NULL, 5, NULL);
    // // xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 5, NULL);

    vTaskDelay(500 / portTICK_PERIOD_MS);
    lcd_set_blk(1);
    // wifi_init();
    

    lcd_cam_buffer = (uint16_t *)heap_caps_malloc(sizeof(uint16_t)*(240 * 240), MALLOC_CAP_SPIRAM);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    rgb.r = 0x0F;
    rgb.b = 0x0F;
    rgb.g = 0x0F;
    WS2812B_setLeds(&rgb, 1);
    while(1) {
        // printf("heap: %d/%d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL), esp_get_free_heap_size());
        camera_capture();
    }
}

#endif