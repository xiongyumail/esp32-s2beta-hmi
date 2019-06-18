#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define BUTTON_NUM 1
#define BUTTON_IO     0
#define BUTTON_PIN_SEL  ((1ULL<<BUTTON_IO))
#define ESP_INTR_FLAG_DEFAULT 0

typedef enum {
    BUTTON_PRESS = 0,
    BUTTON_RELEASE,
    BUTTON_PRESS_3S,
    BUTTON_RELEASE_3S,
    BUTTON_PRESS_10S,
    BUTTON_RELEASE_10S,
    BUTTON_PRESS_60S,
    BUTTON_RELEASE_60S,
} button_press_event_t;

typedef void (*button_callback_t)(uint8_t button_num, button_press_event_t button_press_event);

void button_init(button_callback_t callback);

#ifdef __cplusplus
}
#endif
