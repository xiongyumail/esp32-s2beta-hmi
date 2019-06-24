#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

typedef enum {
    BATTERY_FULL,
    BATTERY_3,
    BATTERY_2,
    BATTERY_1,
    BATTERY_EMPTY
} gui_battery_value_t;

int gui_set_wifi_state(bool state, int ticks_wait);

int gui_set_time_change(int ticks_wait);

int gui_set_battery_value(gui_battery_value_t value, int ticks_wait);

int gui_set_sensor(float temp, float hum, float light, int ticks_wait);

int gui_set_motion(float pitch, float roll, float yaw, int ticks_wait);

void gui_init(lv_theme_t *th);

#ifdef __cplusplus
} /* extern "C" */
#endif