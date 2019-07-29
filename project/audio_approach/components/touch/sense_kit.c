#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "unity.h"
// #include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include <time.h>
#include <sys/time.h>
#include <sys/time.h>
#include <sys/times.h>

/* User include */
// #include "gpio_vf.h"
// #include "led_driver.h"
// #include "evb_seg_led.h"
#include "touchsensor_approach.h"
// #include "../touch_scope.h"

static const char *TAG = "sense-kit";

#define SCOPE_DEBUG_ENABLE         (0)
#define SCOPE_DEBUG_CHANNEL_MAX    (3)
#define SCOPE_UART_BUADRATE        (256000)
#define SCOPE_DEBUG_FREQ_MS        (30)

#define APPROACH_SLIDE_INTERVAL_MIN    (1)      // ms
#define APPROACH_SLIDE_INTERVAL_MAX    (800)    // ms
#define TOUCH_APPROACH_UP_STEP          1

/* three channel .定义不同感应距离时的阈值 0cm ~ 8cm + 无穷远 */
static const uint32_t approach_threshold[3][10] = {
    {332000*2, 332000, 316500, 313900, 311550, 310500, 310150, 309700, 309150, 308400},
    {295000*2, 295000, 284500, 282550, 281000, 280300, 280000, 279650, 279350, 278850},
    {292000*2, 292000, 282500, 280650, 279300, 278700, 278450, 278200, 277900, 277400}
};
// static const uint32_t approach_diff_threshold[3][10] = {
//     {332000*2, 23600*2, 23600, 8100, 5500, 3150, 2100, 1750, 1300, 750},
//     {295000*2, 16150*2, 16150, 5650, 3700, 2150, 1450, 1150, 800,  500},
//     {292000*2, 14600*2, 14600, 5100, 3250, 1900, 1300, 1050, 800,  500}
// };
/* 修改硬件 + cnt：64 */
// static const uint32_t approach_diff_threshold[3][10] = {
//     {332000*2, 23600*2, 23600, 8100, 5500, 3150, 2100, 1750, 1300, 750},
//     {295000*2, 7670*2,  7670,  4500, 2400, 1820, 1220, 820,  720,  620},
//     {292000*2, 8200*2,  8200,  4300, 2500, 1650, 1150, 700,  550,  550}
// };
/* 修改硬件 + cnt：32 */
// static const uint32_t approach_diff_threshold[3][10] = {
//     {332000*2, 19400*2, 19400, 8400, 4800, 2800, 1900, 1500, 950, 650},
//     {295000*2, 7670*2,  8350,  3350, 1950, 1100, 750,  550,  350, 250},
//     {292000*2, 9100*2,  9100,  3000, 1650, 950,  600,  500,  300, 200}
// };
/* 修改硬件 + cnt：16 */
// static const uint32_t approach_diff_threshold[3][10] = {
//     {332000*2, 7800*2, 7800, 5400, 2500, 1350, 1120, 750, 500, 330},
//     {295000*2, 2830*2,  2830,  1880, 930, 510, 430,  290,  200, 130},
//     {292000*2, 3200*2,  3200,  2020, 900, 470,  360,  240,  180, 110}
// };
static const uint32_t approach_diff_threshold[3][10] = {
    {332000*2, 7800*2, 7800, 5400, 2500, 1350, 1120, 750, 500, 330},
    {295000*2, 5700*2,  5700,  1650, 900, 550, 350,  250,  200, 150},
    {292000*2, 5200*2,  5200,  1600, 850, 500, 300,  200,  150, 100}
};
static const uint32_t approach_distance[3] = {
    6, 4, 4
};

static void approach_distance_display(int diff_val)
{
    for(int i=9; i>0; i--) {
        if(diff_val < approach_diff_threshold[APPROACH_PAD_MAIN][i]) {
            // ch450_write_dig(3, i);
            printf("ch450_write_dig(3, i);\n");
            break; 
        }
    }
}

static void approach_release_pad(touch_event_t *evt)
{
    if(evt->pads == APPROACH_PAD_MAIN) {
        // rgb_red(0);
        // rgb_green(0);
        // rgb_blue(0);
        printf("rgb_(0);\n");
    }
}

static void approach_push_pad(touch_event_t *evt)
{
    static char seg_cnt = 0;
    uint32_t diff_step = 0;
    static uint32_t left_val=0, right_val=0;
    struct timeval left, right;
    if(evt->pads == APPROACH_PAD_LEFT) {
        gettimeofday(&left, NULL);
        left_val = (left.tv_sec) * 1000 + (left.tv_usec) / 1000;
        // printf("a %d\n", left_val);
    } else if(evt->pads == APPROACH_PAD_RIGHT) {
        gettimeofday(&right, NULL);
        right_val = (right.tv_sec) * 1000 + (right.tv_usec) / 1000;
        // printf("b %d\n", right_val);
    } else if(evt->pads == APPROACH_PAD_MAIN) {
        // rgb_red(1);
        // rgb_green(1);
        // rgb_blue(1);
        printf("rgb_(1);\n");
        return;
    } else {
        return;
    }

    if(left_val > right_val) {
        diff_step = left_val - right_val;
        printf("<<< %d\n", diff_step);
        if(diff_step < APPROACH_SLIDE_INTERVAL_MAX && diff_step > APPROACH_SLIDE_INTERVAL_MIN) {
            printf("<<< %d\n", diff_step);
            seg_cnt--;
            if(seg_cnt > 9) {
                seg_cnt = 0;
            } else if(seg_cnt <= 0){
                seg_cnt = 9;
            }
            // ch450_write_dig(5, seg_cnt); 
            printf("ch450_write_dig(5, seg_cnt);\n");
        }
    } else {
        diff_step = right_val - left_val;
        printf(">>> %d\n", diff_step);
        if(diff_step < APPROACH_SLIDE_INTERVAL_MAX && diff_step > APPROACH_SLIDE_INTERVAL_MIN) {
            printf(">>> %d\n", diff_step);
            seg_cnt++;
            if(seg_cnt > 9) {
                seg_cnt = 0;
            } else if(seg_cnt <= 0){
                seg_cnt = 9;
            }
            // ch450_write_dig(5, seg_cnt); 
            printf("ch450_write_dig(5, seg_cnt); \n");
        }
    }
}


static void approach_baseline_reset(uint32_t *baseline, uint32_t new_data, uint32_t noise_thr)
{
    /* 如果新数据小于 baseline ，立刻更新 baseline */
    if(new_data <= *baseline) {
        *baseline = new_data;  
    }
    /* 如果新数据大于 baseline，但小于噪声阈值，baseline 按步长递增 */ 
    else if(new_data - *baseline > 0 && new_data - *baseline < noise_thr) {
        *baseline += TOUCH_APPROACH_UP_STEP;
    }
    /* 如果新数据大于 baseline，且大于噪声阈值，baseline 不改变 */ 
    else {

    }
}

void evb_sense_kit(void *p)
{
    touch_event_t evt;
    uint32_t scope_last_data[3] = {0};  // max approach channel num is 3;
    uint32_t touch_status_old = 0;
    uint32_t touch_status = 0;
    uint32_t touch_avg_val[3];

    // rgb_init();
    // ch450_dev_init();
    // ch450_write_dig(5, 0);  // 5 / 4 / 3
    printf("rgb_init(); \n");
    printf("ch450_dev_init(); \n");
    printf("ch450_write_dig(5, 0); \n");
    touchsensor_approach(NULL);
    touchsensor_approach_get_avg_val(5, 3, touch_avg_val);
    
#if SCOPE_DEBUG_ENABLE
    float scope_temp[10] = {0};  // max scope channel is 10.
    iot_tp_scope_debug_init(0, -1, -1, SCOPE_UART_BUADRATE);
#endif

    while(1) {
        touchsensor_approach_wait_event(&evt); // Block wait approach sensor be touched.
        
        if(evt.data == 0) {
            if(scope_last_data[evt.pads] >= touch_avg_val[evt.pads] + TOUCH_PAD_HYSTERESIS 
                    + approach_diff_threshold[evt.pads][approach_distance[evt.pads]]/2) {
                touch_status |= BIT(evt.pads);
                if(touch_status != touch_status_old) {
                    printf("[%d] PUSH\n", evt.pads);
                    approach_push_pad(&evt);
                }
            } else if(scope_last_data[evt.pads] <= touch_avg_val[evt.pads] - TOUCH_PAD_HYSTERESIS 
                    + approach_diff_threshold[evt.pads][approach_distance[evt.pads]]/2){
                touch_status &= (~BIT(evt.pads));
                if(touch_status != touch_status_old) {
                    printf("[%d] RELEASE\n", evt.pads);
                    approach_release_pad(&evt);
                }
            } else {
            }

            if(evt.pads == APPROACH_PAD_MAIN) {
                approach_distance_display((int)scope_last_data[APPROACH_PAD_MAIN] - touch_avg_val[APPROACH_PAD_MAIN]);
            }
            
#if SCOPE_DEBUG_ENABLE
            scope_temp[evt.pads] = scope_last_data[evt.pads];
            scope_temp[evt.pads+3] = touch_avg_val[evt.pads];
            scope_temp[evt.pads+6] = touch_avg_val[evt.pads] + approach_diff_threshold[evt.pads][approach_distance[evt.pads]];
            if(evt.pads == 2) {
                scope_temp[2] = scope_temp[2] + 0;
                scope_temp[5] = scope_temp[5] + 0;
                scope_temp[8] = scope_temp[8] + 0;
                iot_tp_print_to_scope(scope_temp, 9);
                // printf("val %d %d %d\n", scope_last_data[0], scope_last_data[1], scope_last_data[2]);
                // printf("[%d] 0x%x %d %d raw0x%x\n", evt.pads, evt.intr_msk, approach_threshold[0][4], scope_last_data[0], 
                //     REG_READ(RTC_CNTL_INT_ST_REG) >> (RTC_CNTL_TOUCH_DONE_INT_ST_S) & (TOUCH_PAD_INTR_MASK_ALL));
            }
#endif
            touch_status_old = touch_status;
            approach_baseline_reset(&touch_avg_val[evt.pads], scope_last_data[evt.pads], \
                approach_diff_threshold[evt.pads][approach_distance[evt.pads]]/2);
        }
        scope_last_data[evt.pads] = evt.data;

#if TOUCH_SINGLE_APPROACH_DEMO
        if(evt.data == 0) {
            if(scope_last_data[evt.pads] >= TOUCH_PAD_THRESHOLD_PUSH) {
                touch_status |= BIT(evt.pads);
                if(touch_status != touch_status_old) {
                    printf("[%d] PUSH\n", evt.pads);
                    rgb_red(1);
                }
            } else if(scope_last_data[evt.pads] <= TOUCH_PAD_THRESHOLD_RELEASE){
                touch_status &= (~BIT(evt.pads));
                if(touch_status != touch_status_old) {
                    printf("[%d] RELEASE\n", evt.pads);
                    rgb_red(0);
                }
            } else {
            }
            
            touch_status_old = touch_status;
        }
        scope_last_data[evt.pads] = evt.data;
#endif
    }
}

// TEST_CASE("touch approach demo", "[SENSE KIT][evb]")
// {
//     evb_sense_kit(NULL);
// }
