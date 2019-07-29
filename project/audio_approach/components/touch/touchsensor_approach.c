#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/touch_pad.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/sens_struct.h"
#include "soc/apb_ctrl_reg.h"
// #include "unity.h"

#include "touchsensor_approach.h"

#ifdef TOUCH_PAD_MEASURE_CYCLE_DEFAULT
#undef TOUCH_PAD_MEASURE_CYCLE_DEFAULT
#define TOUCH_PAD_MEASURE_CYCLE_DEFAULT    300  //1 <-> 2us <-> raw_data 30
#endif
#ifdef TOUCH_PAD_SLEEP_CYCLE_DEFAULT
#undef TOUCH_PAD_SLEEP_CYCLE_DEFAULT
#define TOUCH_PAD_SLEEP_CYCLE_DEFAULT      0xf //0x200 0xf  //should keep 1 ~ 2 cycle, for intr read.
#endif

#define TOUCH_APPROACH_PAD_MAIN     TOUCH_PAD_NUM9
#define TOUCH_APPROACH_PAD_LEFT     TOUCH_PAD_NUM8
#define TOUCH_APPROACH_PAD_RIGHT    TOUCH_PAD_NUM11
#define TOUCH_APPROACH_MEAS_CNT     (16) //64

/* single channel. 定义不同感应距离时的阈值 0cm ~ 8cm + 无穷远 */
// static const uint32_t approach_threshold[3][10] = {
//     {347600*2, 347600, 342400, 340500, 336116+1000, 339100, 337350, 337100, 336850, 335750},//339100
//     {292000*2, 292000, 286000, 282600, 281000, 280100, 279850, 279600, 279300, 278820},
//     {289200*2, 289200, 283500, 280500, 279500, 278900, 278300, 278050, 277900, 277400}
// };

/* three channel .定义不同感应距离时的阈值 0cm ~ 8cm + 无穷远 */
// static const uint32_t approach_threshold[3][10] = {
//     {332000*2, 332000, 316500, 313900, 311550, 310500, 310150, 309700, 309150, 308400},
//     {295000*2, 295000, 284500, 282550, 281000, 280300, 280000, 279650, 279350, 278850},
//     {292000*2, 292000, 282500, 280650, 279300, 278700, 278450, 278200, 277900, 277400}
// };

static const char* TAG = "Touch pad";

#define PLATFORM_SELECT            (1)          //0: pxp; 1: chip

static uint16_t touch_intr_cnt = 0;
static QueueHandle_t que_touch = NULL;

/*
  Handle an interrupt triggered when a pad is touched.
  Recognize what pad has been touched and save it in a table.
 */
static void tp_example_rtc_intr(void * arg)
{
    if(que_touch == NULL) {
        return;
    }
    static int cnt = 0;
    uint32_t int_mask = touch_pad_get_int_status_vf();
    uint32_t pad_intr = touch_pad_get_status_vf();
    uint32_t pad_num = touch_pad_get_scan_curr();
    int task_awoken = pdFALSE;
    static touch_event_t evt;

    if(int_mask & TOUCH_PAD_INTR_MASK_ACTIVE) {
        evt.msk = TOUCH_PAD_INTR_MASK_ACTIVE;
        evt.pads = pad_num;
        evt.data = pad_intr;
        xQueueSendFromISR(que_touch, &evt, &task_awoken);
    }
    if(int_mask & TOUCH_PAD_INTR_MASK_INACTIVE) {
        evt.msk = TOUCH_PAD_INTR_MASK_INACTIVE;
        evt.pads = pad_num;
        evt.data = pad_intr;
        xQueueSendFromISR(que_touch, &evt, &task_awoken);
    }
    if(int_mask & TOUCH_PAD_INTR_MASK_DONE) {
        evt.msk = TOUCH_PAD_INTR_MASK_DONE;
        if(pad_num == TOUCH_APPROACH_PAD_MAIN){
            evt.pads = 0;
        } else if(pad_num == TOUCH_APPROACH_PAD_LEFT) {
            evt.pads = 1;
        } else if(pad_num == TOUCH_APPROACH_PAD_RIGHT) {
            evt.pads = 2;
        } else {
            ets_printf("touch intr err!!\n");
        }
        evt.intr_msk = pad_intr;
        touch_pad_filter_baseline_read(pad_num, &evt.data);
        xQueueSendFromISR(que_touch, &evt, &task_awoken);
    }
    if(++cnt > TOUCH_APPROACH_MEAS_CNT) {
        cnt=0;
    }
    // ets_printf("[INT]:en 0x%x, st 0x%x, raw 0x%x. [PAD]:%d, st 0x%x, cnt %d, val %d\r\n", 
    //             REG_READ(RTC_CNTL_INT_ENA_REG),
    //             int_mask, 
    //             REG_READ(RTC_CNTL_INT_RAW_REG),
    //             pad_num,
    //             pad_intr,
    //             cnt,
    //             evt.data);
    if (task_awoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void _testcase_filter_mode(touch_filter_mode_t mode)
{
    /* Filter function */
    touch_filter_config_t filter_info = {
            .mode = mode,    //Test jitter and filter 1/4.
            .debounce_cnt = 1,
            .hysteresis_thr = 3,// 3.1%    //when approach function, this enable.
            .noise_thr = 1,     // 37.5%
            .noise_neg_thr = 1, // 37.5%
            .neg_noise_limit = 5, //16 times cnt.
            .jitter_step = 4,  // use for jitter mode.
    };
    touch_pad_filter_set_config(filter_info);
    touch_pad_filter_start_vf();
    touch_pad_filter_baseline_reset(TOUCH_PAD_MAX);
    ESP_LOGI(TAG, "touch pad filter init %d", mode);
}

void touchsensor_approach_wait_event(touch_event_t *evt)
{
    if (que_touch != NULL) {
        xQueueReceive(que_touch, evt, portMAX_DELAY); // raw data
    }
}

void touchsensor_approach(void *arg)
{
    touch_event_t evt;
    // Initialize touch pad peripheral, it will start a timer to run a filter
    ESP_LOGI(TAG, "Initializing touch pad");
    touch_pad_init_vf();
    touch_pad_intr_disable_vf(TOUCH_PAD_INTR_ALL);
    touch_pad_clear_group_mask_vf(TOUCH_PAD_BIT_MASK_MAX);
    // 0xf-0.4ms; 0x200-5.5ms; 0x400-10ms; 0x1400-50ms; 0x2800-100ms;
    touch_pad_set_meas_time_vf(0x0, TOUCH_PAD_MEASURE_CYCLE_DEFAULT);
    // Set reference voltage for charging/discharging
    touch_pad_set_voltage_vf(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V5);
    touch_pad_set_inactive_connect(TOUCH_PAD_CONN_HIGHZ);

    /* Denoise function */
    touch_pad_denoise_t denoise = {
            .grade = TOUCH_PAD_DENOISE_BIT4,
            .cap_level = TOUCH_PAD_DENOISE_CAP_L7,
    };
    touch_pad_denoise_set_config(denoise);
    touch_pad_denoise_enable();
    touch_pad_set_cnt_mode_vf(0, TOUCH_PAD_SLOPE_7);
    ESP_LOGI(TAG, "Denoise function init");

    /* Waterless function */
    touch_pad_waterless_t waterless = {
            .guard_ring_pad = TOUCH_PAD_NUM0,
            .shield_driver = TOUCH_PAD_SHIELD_DRV_L1,   //40pf
    };
    touch_pad_waterless_set_config(waterless);
    touch_pad_waterless_enable();
    touch_pad_clear_group_mask_vf(BIT(TOUCH_PAD_NUM14));
    touch_pad_io_init_vf(TOUCH_PAD_NUM14);
    ESP_LOGI(TAG, "touch pad waterless init");

    /* Init approach main pad. */
    touch_pad_set_thresh_vf(TOUCH_APPROACH_PAD_MAIN, TOUCH_PAD_THRESHOLD_MAX);
    touch_pad_io_init_vf(TOUCH_APPROACH_PAD_MAIN);
    touch_pad_set_cnt_mode_vf(TOUCH_APPROACH_PAD_MAIN, TOUCH_PAD_SLOPE_7);
    touch_pad_set_group_mask_vf(BIT(TOUCH_APPROACH_PAD_MAIN));

    // /* Init approach left pad. */
    touch_pad_set_thresh_vf(TOUCH_APPROACH_PAD_LEFT, TOUCH_PAD_THRESHOLD_MAX);
    touch_pad_io_init_vf(TOUCH_APPROACH_PAD_LEFT);
    touch_pad_set_cnt_mode_vf(TOUCH_APPROACH_PAD_LEFT, TOUCH_PAD_SLOPE_7);
    touch_pad_set_group_mask_vf(BIT(TOUCH_APPROACH_PAD_LEFT));

    // /* Init approach right pad. */
    touch_pad_set_thresh_vf(TOUCH_APPROACH_PAD_RIGHT, TOUCH_PAD_THRESHOLD_MAX);
    touch_pad_io_init_vf(TOUCH_APPROACH_PAD_RIGHT);
    touch_pad_set_cnt_mode_vf(TOUCH_APPROACH_PAD_RIGHT, TOUCH_PAD_SLOPE_7);
    touch_pad_set_group_mask_vf(BIT(TOUCH_APPROACH_PAD_RIGHT));

    /* Approach function */
    touch_pad_approach_t approach = {
            .select_pad0 = TOUCH_APPROACH_PAD_MAIN,
            .select_pad1 = TOUCH_APPROACH_PAD_LEFT,
            .select_pad2 = TOUCH_APPROACH_PAD_RIGHT,
            .means_num = TOUCH_APPROACH_MEAS_CNT,
    };
    touch_pad_approach_set_config(approach);
    ESP_LOGI(TAG, "touch pad approach init");

    /* Filter function (No use in approach) */
    _testcase_filter_mode(TOUCH_PAD_FILTER_IIR_4);

    // Set thresh hold
    uint32_t touch_value;
    /* 设置感应阈值 - no use */
    // touch_pad_set_thresh_vf(TOUCH_APPROACH_PAD_MAIN,  TOUCH_PAD_THRESHOLD); //8cm
    // touch_pad_set_thresh_vf(TOUCH_APPROACH_PAD_LEFT,  approach_threshold[1][5]); //8cm
    // touch_pad_set_thresh_vf(TOUCH_APPROACH_PAD_RIGHT, approach_threshold[2][5]); //6cm
    // printf("approach_threshold %d, hysteresis_thr %d\n", TOUCH_PAD_THRESHOLD, TOUCH_PAD_THRESHOLD*3/100);
    
    // Register touch interrupt ISR
    touch_pad_isr_register_vf(tp_example_rtc_intr, NULL, TOUCH_PAD_INTR_MASK_ALL);
    touch_pad_intr_enable_vf(TOUCH_PAD_INTR_DONE);  //Check the measure done

    touch_pad_fsm_start_vf(TOUCH_FSM_MODE_TIMER);
    // Wait data stable.
    vTaskDelay(300 / portTICK_RATE_MS);
    if(que_touch == NULL) {
        que_touch = xQueueCreate(TOUCH_PAD_MAX, sizeof(touch_event_t));
    }
    ESP_LOGI(TAG, "touch pad sys init!");
}

void touchsensor_approach_get_avg_val(uint16_t avg_num, uint8_t app_pad_num, uint32_t *avg_val)
{
    int cnt = 0;
    uint64_t scope_temp[3] = {0};
    touch_event_t evt;
    uint32_t scope_last_data[3] = {0};  // max approach channel num is 3;

    while(1) {
        xQueueReceive(que_touch, &evt, portMAX_DELAY); // raw raw_data
        if(evt.pads == (app_pad_num-1)) {
            break;
        }
    }

    while(1) {
        xQueueReceive(que_touch, &evt, portMAX_DELAY); // raw raw_data

        if(evt.data == 0) {
            // printf("pad %d, data %d\n", evt.pads, scope_last_data[evt.pads]);
            scope_temp[evt.pads] += scope_last_data[evt.pads]; // touch threshold
            if(++cnt >= avg_num*app_pad_num) {
                for(uint8_t i=0; i<app_pad_num; i++) {
                    avg_val[i] = (uint32_t)(scope_temp[i] / avg_num);
                    ESP_LOGI(TAG, "pad %d avg val %d", i, avg_val[i]);
                }
                break;
            }
        }
        scope_last_data[evt.pads] = evt.data;
    }
}
