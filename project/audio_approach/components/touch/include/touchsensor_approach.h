#pragma once

// #include "touch_pad_vf.h"

#define TOUCH_SINGLE_APPROACH_DEMO  0
#if TOUCH_SINGLE_APPROACH_DEMO
#define TOUCH_PAD_THRESHOLD_PUSH    (395520 + TOUCH_PAD_HYSTERESIS)
#define TOUCH_PAD_THRESHOLD_RELEASE (395520 - TOUCH_PAD_HYSTERESIS)
#endif

#define TOUCH_PAD_HYSTERESIS        (20)
#define TOUCH_PAD_THRESHOLD         (100)
#define APPROACH_PAD_MAIN           (0)
#define APPROACH_PAD_LEFT           (1)
#define APPROACH_PAD_RIGHT          (2)

typedef struct touch_msg {
    touch_pad_intr_mask_t msk;
    uint32_t pads;
    uint32_t data;
    uint32_t intr_msk;
}touch_event_t;

void touchsensor_approach(void *arg);
void touchsensor_approach_wait_event(touch_event_t *evt);
void touchsensor_approach_get_avg_val(uint16_t avg_num, uint8_t app_pad_num, uint32_t *avg_val);
