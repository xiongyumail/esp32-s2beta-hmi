#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

typedef enum {
    GUI_PAGE_LED = 0,
    GUI_PAGE_MONITOR,
    GUI_PAGE_MOTION,
    GUI_PAGE_TOUCH,
    GUI_PAGE_AUDIO,
    GUI_PAGE_CAMERA,
    GUI_PAGE_TERMINAL,
    GUI_PAGE_INFO
} gui_page_t;

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

int gui_set_camera(uint8_t* src, size_t len, int ticks_wait);

gui_page_t gui_get_page();

void gui_init(lv_theme_t *th);

#ifdef __cplusplus
} /* extern "C" */
#endif