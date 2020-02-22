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
#include "lcd_cam.h"
#include "ft5x06.h"
#include "gui.h"
#include "ov2640.h"
#include "fram_cfg.h"
#include "esp_lua.h"
#include "esp_lua_lib.h"

static const char *TAG = "main";

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

#define  CAM_XCLK  GPIO_NUM_0
#define  CAM_PCLK  GPIO_NUM_12
#define  CAM_VSYNC GPIO_NUM_14
#define  CAM_HSYNC GPIO_NUM_13

#define  CAM_D0 GPIO_NUM_18
#define  CAM_D2 GPIO_NUM_17
#define  CAM_D4 GPIO_NUM_8
#define  CAM_D6 GPIO_NUM_10
 
#define  CAM_D1 GPIO_NUM_21
#define  CAM_D3 GPIO_NUM_7
#define  CAM_D5 GPIO_NUM_9
#define  CAM_D7 GPIO_NUM_11

static const lcd_cam_config_t lcd_cam_config = {
    .lcd_bit_width = 8,
    .lcd_ws_pin    = LCD_WR,
    .lcd_rs_pin    = LCD_RS,
    .lcd_rd_pin    = LCD_RD,
    .lcd_data_pin  = {LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7, LCD_D8, LCD_D9, LCD_D10, LCD_D11, LCD_D12, LCD_D13, LCD_D14, LCD_D15},
    .cam_bit_width = 8,
    .cam_xclk_pin  = CAM_XCLK,
    .cam_pclk_pin  = CAM_PCLK,
    .cam_vsync_pin = CAM_VSYNC,
    .cam_hsync_pin = CAM_HSYNC,
    .cam_data_pin  = {CAM_D0, CAM_D1, CAM_D2, CAM_D3, CAM_D4, CAM_D5, CAM_D6, CAM_D7}
};

static const gpio_num_t SDA = GPIO_NUM_3;
static const gpio_num_t SCL = GPIO_NUM_5;
static const uint32_t CLOCK_SPEED = 100000;  // range from 100 KHz ~ 400Hz

/* MPU configuration */

static const uint16_t kSampleRate      = 10;  // Hz
static const mpud::accel_fs_t kAccelFS = mpud::ACCEL_FS_4G;
static const mpud::gyro_fs_t kGyroFS   = mpud::GYRO_FS_500DPS;
static const mpud::dlpf_t kDLPF        = mpud::DLPF_98HZ;

static lv_disp_t  * disp;

void IRAM_ATTR disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    uint32_t len = (sizeof(uint16_t) * ((area->y2 - area->y1 + 1)*(area->x2 - area->x1 + 1)));

    lcd_set_index(area->x1, area->y1, area->x2, area->y2);
    lcd_write_data((uint16_t *)color_p, len);
    // lcd_write(area->x1, area->y1, area->x2, area->y2, (uint16_t *)color_p, len);

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
    data->point.x = x;   
    data->point.y = y;
    return false; /*No buffering so no more data read*/
}

void memory_monitor(lv_task_t * param)
{
    (void) param; /*Unused*/

    lv_mem_monitor_t mon;
    lv_mem_monitor(&mon);
    ESP_LOGI(TAG, "used: %6d (%3d %%), frag: %3d %%, biggest free: %6d, system free: %d/%d\n", (int)mon.total_size - mon.free_size,
           mon.used_pct,
           mon.frag_pct,
           (int)mon.free_biggest_size,
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL), esp_get_free_heap_size());
}

static void gui_tick_task(void * arg)
{
    while(1) {
        lv_tick_inc(10);
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

static QueueHandle_t gui_terminal_queue = NULL;

static void gui_terminal_callback(char *str, int len)
{
    xQueueSend(gui_terminal_queue, &str[0], 100 / portTICK_RATE_MS);
}

static void gui_task(void *arg)
{
    xTaskCreate(gui_tick_task, "gui_tick_task", 512, NULL, 10, NULL);

    lv_init();
    /*Create a display buffer*/
    static lv_disp_buf_t disp_buf;
    static lv_color_t *lv_buf1 = NULL;
    static lv_color_t *lv_buf2 = NULL;
    lv_buf1 = (lv_color_t *)heap_caps_malloc(sizeof(uint16_t) * LV_HOR_RES_MAX * LV_VER_RES_MAX, MALLOC_CAP_SPIRAM);
    lv_disp_buf_init(&disp_buf, lv_buf1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);

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

    gui_terminal_queue = xQueueCreate(16, sizeof(char));
    gui_set_terminal_callback(gui_terminal_callback);
    while(1) {
        lv_task_handler();
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

static void cam_task(void *arg)
{
    if (OV2640_Init(0, 1) == 1) {
        vTaskDelete(NULL);
        return;
    }
	OV2640_RGB565_Mode(false);	//RGB565模式
    OV2640_ImageSize_Set(800, 600);
    OV2640_ImageWin_Set(0, 0, 800, 600);
  	OV2640_OutSize_Set(FRAM_WIDTH, FRAM_HIGH); 
    ESP_LOGI(TAG, "camera init done\n");
    uint8_t *fbuf = cam_attach();
    cam_start();
    while (1) {
        
        if (GUI_PAGE_CAMERA == gui_get_page()) {
            take_fram_lock();
            gui_set_camera(fbuf, FRAM_WIDTH*FRAM_HIGH*2, portMAX_DELAY);
            give_fram_lock();
        } else {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

static size_t esp_lua_input_callback(char *str, size_t len) 
{
    char c[128] = {0};
    size_t ret = 0;
    if ((ret = fread(c, sizeof(char), 128, stdin)) > 0) {
        memcpy(str, c, ret);
    } else if (gui_terminal_queue && xQueueReceive(gui_terminal_queue, &c[0], 0) != pdFAIL) {
        memcpy(str, c, 1);
        ret = 1;
    }
    return ret;
}

static size_t esp_lua_output_callback(char *str, size_t len) 
{
    size_t ret = 0;
    gui_add_terminal_text(str, len, portMAX_DELAY);
    ret = fwrite(str, sizeof(char), len, stdout);
    return ret;
}

static const luaL_Reg mylibs[] = {
    {"sys", esp_lib_sys},
    {"net", esp_lib_net},
    {"web", esp_lib_web},
    {"mqtt", esp_lib_mqtt},
    {"httpd", esp_lib_httpd},
    {NULL, NULL}
};

char LUA_SCRIPT_INIT[] = " \
assert(sys.init()) \
dofile(\'/lua/init.lua\') \
";

void lua_task(void *arg)
{
    char *ESP_LUA_ARGV[5] = {"./lua", "-i", "-e", LUA_SCRIPT_INIT, NULL}; // enter interactive mode after executing 'script'

    // esp_lua_init(esp_lua_input_callback, esp_lua_output_callback, mylibs);
    esp_lua_init(NULL, NULL, mylibs);

    while (1) {
        esp_lua_main(4, ESP_LUA_ARGV);
        printf("lua exit\n");
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

hts221_handle_t hts221;
bh1750_handle_t bh1750;
int16_t temperature;
int16_t humidity;
float light;
float roll{0}, pitch{0}, yaw{0};

static void mpu_task(void*)
{
    MPU_t MPU;
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
        if (GUI_PAGE_MOTION == gui_get_page()) {
            MPU.motion(&rawAccel, &rawGyro);  // read both in one shot
            // Calculate tilt angle
            // range: (roll[-180,180]  pitch[-90,90]  yaw[-180,180])
            const double kRadToDeg = 57.2957795131;
            const float kDeltaTime = 1.f / kSampleRate;
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
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void sensor_task(void*)
{
    time_t now;
    struct tm timeinfo, last_timeinfo;

    hts221 = iot_hts221_create();
    bh1750 = iot_bh1750_create(I2C_NUM_0, BH1750_I2C_ADDRESS_DEFAULT);
    iot_bh1750_power_on(bh1750);
    iot_bh1750_set_measure_mode(bh1750, BH1750_CONTINUE_4LX_RES);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    while (true) {
        if (GUI_PAGE_MONITOR == gui_get_page()) {
            iot_hts221_get_temperature(hts221, &temperature);
            iot_hts221_get_humidity(hts221, &humidity);
            iot_bh1750_get_data(bh1750, &light);
            gui_set_sensor((float)(temperature/10), (float)(humidity/10), light, 1000);
        } else if (GUI_PAGE_MOTION == gui_get_page()) {
            gui_set_motion(pitch, roll, yaw, 1000);
        }

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
        ESP_LOGI(TAG, "temperature: %f\n humidity: %f\r\n", (float)(temperature/10), (float)(humidity/10));
        ESP_LOGI(TAG, "light: %f\n", light);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

#include "driver/gpio.h"

extern "C" void app_main() 
{
    xTaskCreate(lua_task, "lua_task", 10240, NULL, 5, NULL);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Initialize I2C on port 0 using I2Cbus interface
    i2c0.begin(SDA, SCL, CLOCK_SPEED);
    i2c0.setTimeout(100);
    WS2812B_init(RMT_CHANNEL_0, GPIO_NUM_4, 1);
    wsRGB_t rgb = {0x0, 0x0, 0x0};
    WS2812B_setLeds(&rgb, 1);
    lcd_cam_init(&lcd_cam_config);

    xTaskCreate(gui_task, "gui_task", 4096, NULL, 5, NULL);
    xTaskCreate(cam_task, "cam_task", 4096, NULL, 5, NULL);
    xTaskCreate(mpu_task, "mpu_task", 4 * 1024, NULL, 5, NULL);
    xTaskCreate(sensor_task, "sensor_task", 2 * 1024, NULL, 5, NULL);
}