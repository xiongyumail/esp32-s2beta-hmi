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
#include "I2Cbus.hpp"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"
#include "hts221.hpp"
#include "iot_bh1750.h"
#include "WS2812B.h"
#include "lvgl.h"
#include "lcd.h"
#include "ft5x06.h"
#include "gui.h"

static const char *TAG = "main";

static constexpr gpio_num_t SDA = GPIO_NUM_3;
static constexpr gpio_num_t SCL = GPIO_NUM_5;
static constexpr uint32_t CLOCK_SPEED = 400000;  // range from 100 KHz ~ 400Hz

/* MPU configuration */

static constexpr uint16_t kSampleRate      = 10;  // Hz
static constexpr mpud::accel_fs_t kAccelFS = mpud::ACCEL_FS_4G;
static constexpr mpud::gyro_fs_t kGyroFS   = mpud::GYRO_FS_500DPS;
static constexpr mpud::dlpf_t kDLPF        = mpud::DLPF_98HZ;

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
            gui_set_wifi_state(true, 0);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            gui_set_wifi_state(false, 0);
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

static lv_disp_t  * disp;

void IRAM_ATTR disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    uint32_t len = (sizeof(uint16_t) * ((area->y2 - area->y1 + 1)*(area->x2 - area->x1 + 1)));

    lcd_set_index(area->x1, area->y1, area->x2, area->y2);
    lcd_write_data((uint16_t *)color_p, len);

    lv_disp_flush_ready(disp_drv);
}

bool IRAM_ATTR disp_input(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    static uint16_t x = 0, y = 0;
    if (ft5x06_pos_read(&x, &y)) {
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state =  LV_INDEV_STATE_REL;
    }
    // printf("x: %d, y: %d, state: %d\n", x, y, data->state);
    data->point.x = x;   
    data->point.y = y;
    return false; /*No buffering so no more data read*/
}

void memory_monitor(lv_task_t * param)
{
    (void) param; /*Unused*/

    lv_mem_monitor_t mon;
    lv_mem_monitor(&mon);
    printf("used: %6d (%3d %%), frag: %3d %%, biggest free: %6d, system free: %d\n", (int)mon.total_size - mon.free_size,
           mon.used_pct,
           mon.frag_pct,
           (int)mon.free_biggest_size,
           esp_get_free_heap_size());
}

static void gui_tick_task(void * arg)
{
    while(1) {
        lv_tick_inc(10);
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void gui_task(void *arg)
{
    lcd_init();

    xTaskCreate(gui_tick_task, "gui_tick_task", 512, NULL, 10, NULL);

    lv_init();
    /*Create a display buffer*/
    static lv_disp_buf_t disp_buf;
    static lv_color_t *lv_buf1 = NULL;
    static lv_color_t *lv_buf2 = NULL;
    // lv_buf = (lv_color_t *)heap_caps_malloc(sizeof(uint16_t) * LV_HOR_RES_MAX * LV_VER_RES_MAX / 200, MALLOC_CAP_DMA);
    lv_buf1 = (lv_color_t *)heap_caps_malloc(sizeof(uint16_t) * LV_HOR_RES_MAX * LV_VER_RES_MAX * 2, MALLOC_CAP_SPIRAM);
    // lv_buf2 = (lv_color_t *)heap_caps_malloc(sizeof(uint16_t) * LV_HOR_RES_MAX * LV_VER_RES_MAX, MALLOC_CAP_SPIRAM);
    lv_disp_buf_init(&disp_buf, lv_buf1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX * 2);

    /*Create a display*/
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);  
    disp_drv.buffer = &disp_buf;
    disp_drv.flush_cb = disp_flush;  
    disp = lv_disp_drv_register(&disp_drv);

    ft5x06_init();
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);          /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = disp_input;         /*This function will be called periodically (by the library) to get the mouse position and state*/
    lv_indev_t * mouse_indev = lv_indev_drv_register(&indev_drv);

    /*Set a cursor for the mouse*/
    LV_IMG_DECLARE(mouse_cursor_icon);                          /*Declare the image file.*/
    lv_obj_t * cursor_obj =  lv_img_create(lv_scr_act(), NULL); /*Create an image object for the cursor */
    lv_img_set_src(cursor_obj, &mouse_cursor_icon);             /*Set the image source*/
    lv_indev_set_cursor(mouse_indev, cursor_obj);               /*Connect the image  object to the driver*/

    lv_task_create(memory_monitor, 3000, LV_TASK_PRIO_MID, NULL);

    gui_init(lv_theme_material_init(0, NULL));

    while(1) {
        lv_task_handler();
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void camera_task(void *arg)
{
    int x;
    int test = 0;
    lv_color_t *camera_buffer = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(320, 240), MALLOC_CAP_SPIRAM);
    // lv_color_t *camera_buffer2 = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(320, 240), MALLOC_CAP_SPIRAM);
    // for(x = 0; x < 320 * 240; x++) {
    //     camera_buffer1[x] = LV_COLOR_YELLOW;
    //     camera_buffer2[x] = LV_COLOR_RED;
    // }
    while(1) {
        if (GUI_PAGE_CAMERA == gui_get_page()) {
            switch (test) {
                case 0: {
                    for(x = 0; x < 320 * 240; x++) {
                        camera_buffer[x] = LV_COLOR_YELLOW;
                    }
                    gui_set_camera(camera_buffer, 1000);
                    test++;
                }
                break;

                case 1: {
                    for(x = 0; x < 320 * 240; x++) {
                        camera_buffer[x] = LV_COLOR_RED;
                    }
                    gui_set_camera(camera_buffer, 1000);
                    test++;
                }
                break;

                case 2: {
                    for(x = 0; x < 320 * 240; x++) {
                        camera_buffer[x] = LV_COLOR_ORANGE;
                    }
                    gui_set_camera(camera_buffer, 1000);
                    test = 0;
                }
                break;
            }
        }

        // gui_set_camera(camera_buffer, 1000);
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}


static void mpuISR(void*);
static void mpuTask(void*);
static void printTask(void*);
hts221_handle_t hts221;
bh1750_handle_t bh1750;
int16_t temperature;
int16_t humidity;
float light;
float roll{0}, pitch{0}, yaw{0};
uint8_t mac[16];

extern "C" void app_main() 
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

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

    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    // Initialize I2C on port 0 using I2Cbus interface
    i2c0.begin(SDA, SCL, CLOCK_SPEED);
    i2c0.setTimeout(100);
    hts221 = iot_hts221_create();
    bh1750 = iot_bh1750_create(I2C_NUM_0, BH1750_I2C_ADDRESS_DEFAULT);
    iot_bh1750_power_on(bh1750);
    iot_bh1750_set_measure_mode(bh1750, BH1750_CONTINUE_4LX_RES);
    WS2812B_init(RMT_CHANNEL_0, GPIO_NUM_4, 1);
    wsRGB_t rgb = {0x0, 0x0, 0x0};
    WS2812B_setLeds(&rgb, 1);

    // Create a task to setup mpu and read sensor data
    xTaskCreate(mpuTask, "mpuTask", 4 * 1024, nullptr, 6, nullptr);
    // Create a task to print angles
    xTaskCreate(printTask, "printTask", 2 * 1024, nullptr, 5, nullptr);

    xTaskCreate(gui_task, "gui_task", 4096, NULL, 5, NULL);

    xTaskCreate(camera_task, "camera_task", 2 * 1024, NULL, 5, NULL);

    vTaskDelay(1000 /portTICK_RATE_MS);
    wifi_init();
}

/* Tasks */

static MPU_t MPU;

static void mpuTask(void*)
{
    MPU.setBus(i2c0);
    MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);

    // Verify connection
    while (esp_err_t err = MPU.testConnection()) {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    // Initialize
    ESP_ERROR_CHECK(MPU.initialize());

    // Self-Test
    mpud::selftest_t retSelfTest;
    while (esp_err_t err = MPU.selfTest(&retSelfTest)) {
        ESP_LOGE(TAG, "Failed to perform MPU Self-Test, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU Self-Test result: Gyro=%s Accel=%s",  //
             (retSelfTest & mpud::SELF_TEST_GYRO_FAIL ? "FAIL" : "OK"),
             (retSelfTest & mpud::SELF_TEST_ACCEL_FAIL ? "FAIL" : "OK"));

    // Calibrate
    mpud::raw_axes_t accelBias, gyroBias;
    ESP_ERROR_CHECK(MPU.computeOffsets(&accelBias, &gyroBias));
    ESP_ERROR_CHECK(MPU.setAccelOffset(accelBias));
    ESP_ERROR_CHECK(MPU.setGyroOffset(gyroBias));

    // Configure
    ESP_ERROR_CHECK(MPU.setAccelFullScale(kAccelFS));
    ESP_ERROR_CHECK(MPU.setGyroFullScale(kGyroFS));
    ESP_ERROR_CHECK(MPU.setSampleRate(kSampleRate));
    ESP_ERROR_CHECK(MPU.setDigitalLowPassFilter(kDLPF));

    mpud::raw_axes_t rawAccel, rawGyro;
    // Reading Loop
    while (true) {
        MPU.motion(&rawAccel, &rawGyro);  // read both in one shot
        // Calculate tilt angle
        // range: (roll[-180,180]  pitch[-90,90]  yaw[-180,180])
        constexpr double kRadToDeg = 57.2957795131;
        constexpr float kDeltaTime = 1.f / kSampleRate;
        float gyroRoll             = roll + mpud::math::gyroDegPerSec(rawGyro.x, kGyroFS) * kDeltaTime;
        float gyroPitch            = pitch + mpud::math::gyroDegPerSec(rawGyro.y, kGyroFS) * kDeltaTime;
        float gyroYaw              = yaw + mpud::math::gyroDegPerSec(rawGyro.z, kGyroFS) * kDeltaTime;
        float accelRoll            = atan2(-rawAccel.x, rawAccel.z) * kRadToDeg;
        float accelPitch = atan2(rawAccel.y, sqrt(rawAccel.x * rawAccel.x + rawAccel.z * rawAccel.z)) * kRadToDeg;
        // Fusion
        roll  = gyroRoll * 0.95f + accelRoll * 0.05f;
        pitch = gyroPitch * 0.95f + accelPitch * 0.05f;
        yaw   = gyroYaw;
        // correct yaw
        if (yaw > 180.f)
            yaw -= 360.f;
        else if (yaw < -180.f)
            yaw += 360.f;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(nullptr);
}

static void printTask(void*)
{
    float last_temperature = 0, last_humidity = 0;
    uint8_t epaper_data[64];

    time_t now;
    struct tm timeinfo, last_timeinfo;

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    while (true) {
        // printf("Pitch: %+6.1f \t Roll: %+6.1f \t Yaw: %+6.1f \n", pitch, roll, yaw);
        
        iot_hts221_get_temperature(hts221, &temperature);
        iot_hts221_get_humidity(hts221, &humidity);
        iot_bh1750_get_data(bh1750, &light);

        time(&now);
        localtime_r(&now, &timeinfo);
        if (timeinfo.tm_year < (2016 - 1900)) {
            // ESP_LOGE(TAG, "The current date/time error");
        } else {
            if (timeinfo.tm_sec != last_timeinfo.tm_sec) {
                gui_set_time_change(1000);
            }
        }
        last_timeinfo = timeinfo;
        // printf("temperature: %f\n humidity: %f\r\n", (float)(temperature/10), (float)(humidity/10));
        // printf("light: %f\n", light);
        gui_set_sensor((float)(temperature/10), (float)(humidity/10), light, 1000);
        gui_set_motion(pitch, roll, yaw, 1000);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}