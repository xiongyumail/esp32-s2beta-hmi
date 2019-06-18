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
#include "esp_log.h"
#include "button.h"
#include "I2Cbus.hpp"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"
#include "hts221.hpp"
#include "iot_bh1750.h"
#include "lvgl.h"

static const char *TAG = "APP_BADGE";

static constexpr gpio_num_t SDA = GPIO_NUM_3;
static constexpr gpio_num_t SCL = GPIO_NUM_5;
static constexpr uint32_t CLOCK_SPEED = 100000;  // range from 100 KHz ~ 400Hz

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

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_init(void)
{
    tcpip_adapter_init();
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
    // xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}

void show_task_mem()
{
    char* pbuffer = (char*) malloc(2048);
    memset(pbuffer, 0x0, 2048);

    printf("----------------------------------------------\r\n");
    vTaskList(pbuffer);
    printf("%s", pbuffer);
    printf("----------------------------------------------\r\n");

    free(pbuffer);
}

#define MEMLEAK_DEBUG
static void button_press(uint8_t num, button_press_event_t button_press_event) 
{
    static uint32_t button_press_count = 0, last_button_press_count = 0;
    static uint32_t time_ms = 0, last_time_ms = 0;

    switch (button_press_event) {
        case BUTTON_PRESS: {
            button_press_count++;
            time_ms = system_get_time() / 1000;
            printf("button press count: %d, free heap size %d\n", button_press_count, esp_get_free_heap_size());
            if (time_ms - last_time_ms > 0 && time_ms - last_time_ms <= 5000) {
                if (button_press_count - last_button_press_count >= 5) {
                    #ifdef MEMLEAK_DEBUG
                    printf("\r\n--------DMEMLEAK_DEBUG--------\r\n");
                    show_task_mem();
                    // pvShowMalloc();
                    // heap_caps_dump_all();
                    printf("\r\n------------------------------\r\n");
                    #endif
                    last_time_ms = time_ms;
                    last_button_press_count = button_press_count; 
                }                 
            } else {
                last_time_ms = time_ms;
                last_button_press_count = button_press_count;
            }
        }break;

        case BUTTON_RELEASE: {
            
        }break;

        case BUTTON_PRESS_3S: {
            ESP_LOGI(TAG, "BUTTON_PRESS_3S");
        }break;

        case BUTTON_RELEASE_3S: {

        }break;

        case BUTTON_PRESS_10S: {
            ESP_LOGI(TAG, "BUTTON_PRESS_10S");
        }break; 

        case BUTTON_RELEASE_10S: {

        }break;   

        case BUTTON_PRESS_60S: {

        }break;  

        case BUTTON_RELEASE_60S: {

        }break;  
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

extern "C" void app_main() {
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    button_init(button_press);
    // Initialize I2C on port 0 using I2Cbus interface
    i2c0.begin(SDA, SCL, CLOCK_SPEED);
    hts221 = iot_hts221_create();
    bh1750 = iot_bh1750_create(I2C_NUM_0, BH1750_I2C_ADDRESS_DEFAULT);
    iot_bh1750_power_on(bh1750);
    iot_bh1750_set_measure_mode(bh1750, BH1750_CONTINUE_4LX_RES);
    
    lv_init();

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    nvs_flash_init();
    // wifi_init();

    // Create a task to setup mpu and read sensor data
    xTaskCreate(mpuTask, "mpuTask", 4 * 1024, nullptr, 6, nullptr);
    // Create a task to print angles
    xTaskCreate(printTask, "printTask", 2 * 1024, nullptr, 5, nullptr);
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

static IRAM_ATTR void mpuISR(TaskHandle_t taskHandle)
{
    BaseType_t HPTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
    if (HPTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void printTask(void*)
{
    int ret;
    float last_temperature = 0, last_humidity = 0;
    uint8_t epaper_data[64];

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    while (true) {
        printf("Pitch: %+6.1f \t Roll: %+6.1f \t Yaw: %+6.1f \n", pitch, roll, yaw);
        
        iot_hts221_get_temperature(hts221, &temperature);
        iot_hts221_get_humidity(hts221, &humidity);
        ret = iot_bh1750_get_data(bh1750, &light);
        // if (abs(temperature - last_temperature) > 1*10) {
        //     sprintf((char *)epaper_data, " %.1f ℃", (float)(temperature/10));
        //     // lv_label_set_text(temp_label, (const char *)epaper_data);
        //     last_temperature = temperature;
        // }

        // if (abs(humidity - last_humidity) > 2*10) {
        //     sprintf((char *)epaper_data, " %.1f %%", (float)(humidity/10));
        //     // lv_label_set_text(hum_label, (const char *)epaper_data);
        //     last_humidity = humidity;
        // }
        printf("temperature: %f\n humidity: %f\r\n", (float)(temperature/10), (float)(humidity/10));
        printf("ret: %d, light: %f\n", ret, light);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}