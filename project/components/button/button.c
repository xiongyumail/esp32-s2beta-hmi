#include "button.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "driver/gpio.h"

#define BUTTON_EVENT_LEN BUTTON_NUM * 5

typedef enum {
    BUTTON_PRESS_10MS   = 10,
    BUTTON_RELEASE_10MS = 10,
    BUTTON_TIMEOUT_3S   = 3000,
    BUTTON_TIMEOUT_7S   = 7000,
    BUTTON_TIMEOUT_50S  = 50000
} button_event_enum_t;

typedef struct {
    int obj;
    int event;
    void *arg;
} button_event_t;

button_event_enum_t button_timer_event[BUTTON_NUM];

static TimerHandle_t button_timer[BUTTON_NUM];

static QueueHandle_t button_event = NULL;

button_callback_t button_cb = NULL;


static void button(uint8_t button_num, button_event_enum_t event)
{
    static uint8_t state[BUTTON_NUM] = {0};

    switch (state[button_num]) {    // Update state and update event output
        case 0: {
            if (BUTTON_PRESS_10MS == event) {   
                if (!gpio_get_level(BUTTON_IO)) {
                    state[button_num] = 1;
                    gpio_set_intr_type(BUTTON_IO, GPIO_INTR_POSEDGE);
                }
            } else {
                return;
            }
        }
        break;

        case 1: {
            if (BUTTON_RELEASE_10MS == event) {
                if (gpio_get_level(BUTTON_IO)) {
                    state[button_num] = 0;
                    gpio_set_intr_type(BUTTON_IO, GPIO_INTR_NEGEDGE);
                    if (button_cb && gpio_get_level(BUTTON_IO)) {
                        button_cb(button_num, BUTTON_RELEASE);
                    }
                }
                return;
            } else if (BUTTON_TIMEOUT_3S == event) {
                    state[button_num] = 2;
            } else {
                return;
            }
        }
        break;

        case 2: {
            if (BUTTON_RELEASE_10MS == event) {
                if (gpio_get_level(BUTTON_IO)) {
                    state[button_num] = 0;
                    gpio_set_intr_type(BUTTON_IO, GPIO_INTR_NEGEDGE);
                    if (button_cb && gpio_get_level(BUTTON_IO)) {
                        button_cb(button_num, BUTTON_RELEASE_3S);
                    }
                }
                return;
            } else if (BUTTON_TIMEOUT_7S == event) {
                    state[button_num] = 3;
            } else {
                return;
            }
        }
        break;

        case 3: {
            if (BUTTON_RELEASE_10MS == event) {
                if (gpio_get_level(BUTTON_IO)) {
                    state[button_num] = 0;
                    gpio_set_intr_type(BUTTON_IO, GPIO_INTR_NEGEDGE);
                    if (button_cb && gpio_get_level(BUTTON_IO)) {
                        button_cb(button_num, BUTTON_RELEASE_10S);
                    }
                }
                return;
            } else if (BUTTON_TIMEOUT_50S == event) {
                    state[button_num] = 4;
            } else {
                return;
            }
        }
        break;

        case 4: {
            if (BUTTON_RELEASE_10MS == event) {
                if (gpio_get_level(BUTTON_IO)) {
                    state[button_num] = 0;
                    gpio_set_intr_type(BUTTON_IO, GPIO_INTR_NEGEDGE);
                    if (button_cb && gpio_get_level(BUTTON_IO)) {
                        button_cb(button_num, BUTTON_RELEASE_60S);
                    }
                }
                return;
            } else {
                return;
            }
        }
        break;
    }

    switch (state[button_num]) {        // Update state output
        case 1: {
            button_timer_event[button_num] = BUTTON_TIMEOUT_3S;
            xTimerChangePeriod(button_timer[button_num], BUTTON_TIMEOUT_3S / portTICK_RATE_MS, 0);
            if (button_cb) {
                button_cb(button_num, BUTTON_PRESS);
            }
        }
        break;

        case 2: {
            button_timer_event[button_num] = BUTTON_TIMEOUT_7S;
            xTimerChangePeriod(button_timer[button_num], BUTTON_TIMEOUT_7S / portTICK_RATE_MS, 0);
            if (button_cb) {
                button_cb(button_num, BUTTON_PRESS_3S);
            }
        }
        break;

        case 3: {
            button_timer_event[button_num] = BUTTON_TIMEOUT_50S;
            xTimerChangePeriod(button_timer[button_num], BUTTON_TIMEOUT_50S / portTICK_RATE_MS, 0);
            if (button_cb) {
                button_cb(button_num, BUTTON_PRESS_10S);
            }
        }
        break;

        case 4: {
            if (button_cb) {
                button_cb(button_num, BUTTON_PRESS_60S);
            }
        }
        break;          
    }
}

static void button_task(void *arg)
{
    button_event_t e;
    while (1) {
        xQueueReceive(button_event, &e, portMAX_DELAY);
        button(e.obj, e.event);
    }
}

static void button_timer_callback(TimerHandle_t timer)
{
    button_event_t e;
    // do something no block
    uint32_t button_num = (uint32_t) pvTimerGetTimerID(timer);

    e.obj = button_num;
    e.event = button_timer_event[button_num];
    xQueueSend(button_event, &e, 0);
}

static void IRAM_ATTR button_isr_handle(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if (gpio_num == BUTTON_IO) {
        if (!gpio_get_level(BUTTON_IO)) {
            button_timer_event[0] = BUTTON_PRESS_10MS;
            xTimerChangePeriodFromISR(button_timer[0], BUTTON_PRESS_10MS / portTICK_RATE_MS, &xHigherPriorityTaskWoken);
        } else if (gpio_get_level(BUTTON_IO)) {
            button_timer_event[0] = BUTTON_RELEASE_10MS;
            xTimerChangePeriodFromISR(button_timer[0], BUTTON_RELEASE_10MS / portTICK_RATE_MS, &xHigherPriorityTaskWoken);
        }

        if (xHigherPriorityTaskWoken != pdFALSE) {
            portYIELD_FROM_ISR();
        }
    }
}

void button_init(button_callback_t callback)
{
    if (callback) {
        button_cb = callback;
    } else {
        return;
    }

    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    io_conf.pin_bit_mask = BUTTON_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(BUTTON_IO, button_isr_handle, (void*) BUTTON_IO);

    button_timer[0] = xTimerCreate
                      ("button0_timer",
                       (20) / portTICK_RATE_MS,
                       pdFALSE,
                       (void *) 0,
                       button_timer_callback);
    button_event = xQueueCreate(BUTTON_EVENT_LEN, sizeof(button_event_t));
    xTaskCreate(button_task, "button_task", 2048, NULL, tskIDLE_PRIORITY + 8, NULL);    
}