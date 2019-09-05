#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "gui.h"
#include "WS2812B.h"

typedef struct {
    int event;
    void *arg;
} gui_event_t;

#define GUI_WIFI_EVENT 0
#define GUI_BATTERY_EVENT 1
#define GUI_TIME_EVENT 2
#define GUI_SOURCE_EVENT 3

#define MY_TEMP_SYMBOL "\xEF\x8B\x88"  // f2c8
#define MY_MOTION_SYMBOL "\xEF\x8B\x9B"  // f2db
#define MY_LED_SYMBOL "\xEF\x83\xAB"  // f0eb
#define MY_TOUCH_SYMBOL "\xEF\x82\xA6" // f0a6
#define MY_CAMERA_SYMBOL "\xEF\x85\xAD" // f16d
#define MY_ESPRESSIF_SYMBOL "\xEF\x8B\xA1" // f2e1

static lv_style_t style_my_symbol;

static lv_obj_t * header;
static lv_obj_t * time_clock;
static lv_obj_t * wifi;
static lv_obj_t * battery;
static lv_obj_t * body;

static QueueHandle_t gui_event_queue = NULL;

static lv_disp_t **disp;
static lv_indev_t **indev;

static void header_create(void)
{
    header = lv_cont_create(lv_disp_get_scr_act(disp[0]), NULL);
    lv_obj_set_width(header, LV_HOR_RES);
    
    time_clock = lv_label_create(header, NULL);
    lv_obj_align(time_clock, NULL, LV_ALIGN_IN_LEFT_MID, LV_DPI/10, 0);
    lv_label_set_recolor(time_clock, true);
    lv_label_set_text(time_clock, "");

    lv_obj_t * state = lv_label_create(header, NULL);
    lv_label_set_text(state, LV_SYMBOL_SETTINGS);
    lv_obj_align(state, NULL, LV_ALIGN_CENTER, 0, 0);

    wifi = lv_label_create(header, NULL);
    lv_label_set_text(wifi, "");
    lv_obj_align(wifi, NULL, LV_ALIGN_IN_RIGHT_MID, -(LV_DPI/10) * 5, 0);

    battery = lv_label_create(header, NULL);
    lv_label_set_text(battery, "");
    lv_obj_align(battery, NULL, LV_ALIGN_IN_RIGHT_MID, -(LV_DPI/10) * 3, 0);

    lv_cont_set_fit2(header, false, true);   /*Let the height set automatically*/
    lv_obj_set_pos(header, 0, 0);
}

static void time_write()
{
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];
    
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_sec % 2) {
        sprintf(strftime_buf, "%02d#000000 :#%02d", timeinfo.tm_hour, timeinfo.tm_min);
    } else {
        sprintf(strftime_buf, "%02d#ffffff :#%02d", timeinfo.tm_hour, timeinfo.tm_min);
    }
    
    lv_label_set_text(time_clock, strftime_buf);
}

static void gui_task(lv_task_t * arg)
{
    int ret;
    gui_event_t e;
    ret = xQueueReceive(gui_event_queue, &e, 0);
    if (ret == pdFAIL) {
        return;
    }
    switch (e.event) {
        case GUI_WIFI_EVENT: {
            if (e.arg) {
                lv_label_set_text(wifi, LV_SYMBOL_WIFI);
            } else {
                lv_label_set_text(wifi, "");
            }
        }
        break;

        case GUI_BATTERY_EVENT: {
            lv_label_set_text(battery, (const char *)e.arg);
        }
        break;

        case GUI_TIME_EVENT: {
            time_write();
        }
        break;

    }
}

static int gui_event_send(int event, void *arg, int ticks_wait)
{
    int ret;
    if (gui_event_queue == NULL) {
        return -1;
    }

    gui_event_t e;
    e.event = event;
    e.arg = arg;
    ret = xQueueSend(gui_event_queue, &e, ticks_wait);
    if (ret == pdFALSE) {
        return -1;
    }

    return 0;
}

int gui_set_wifi_state(bool state, int ticks_wait)
{
    return gui_event_send(GUI_WIFI_EVENT, (void *)state, ticks_wait);
}

int gui_set_time_change(int ticks_wait)
{
    return gui_event_send(GUI_TIME_EVENT, NULL, ticks_wait);
}

int gui_set_battery_value(gui_battery_value_t value, int ticks_wait)
{   
    int ret;

    switch (value) {
        case BATTERY_FULL: {
            ret = gui_event_send(GUI_BATTERY_EVENT, (void *)LV_SYMBOL_BATTERY_FULL, ticks_wait);
        }
        break;

        case BATTERY_3: {
            ret = gui_event_send(GUI_BATTERY_EVENT, (void *)LV_SYMBOL_BATTERY_3, ticks_wait);
        }
        break;

        case BATTERY_2: {
            ret = gui_event_send(GUI_BATTERY_EVENT, (void *)LV_SYMBOL_BATTERY_2, ticks_wait);
        }
        break;

        case BATTERY_1: {
            ret = gui_event_send(GUI_BATTERY_EVENT, (void *)LV_SYMBOL_BATTERY_1, ticks_wait);
        }
        break;

        case BATTERY_EMPTY: {
            ret = gui_event_send(GUI_BATTERY_EVENT, (void *)LV_SYMBOL_BATTERY_EMPTY, ticks_wait);
        }
        break;

        default: {
            return -1;
        }
        break;
    }
    return ret;
}

static lv_group_t * encoder_group;
static wsRGB_t led_rgb;

static void header1_create(lv_obj_t * parent)
{
    header = lv_cont_create(parent, NULL);
    lv_obj_set_width(header, lv_disp_get_hor_res(disp[1]));
    
    time_clock = lv_label_create(header, NULL);
    lv_obj_align(time_clock, NULL, LV_ALIGN_IN_LEFT_MID, LV_DPI/10, 0);
    lv_label_set_recolor(time_clock, true);
    lv_label_set_text(time_clock, "");

    lv_obj_t * state = lv_label_create(header, NULL);
    lv_label_set_text(state, MY_ESPRESSIF_SYMBOL);
    lv_obj_set_style(state, &style_my_symbol);
    lv_obj_align(state, NULL, LV_ALIGN_CENTER, 0, 0);

    wifi = lv_label_create(header, NULL);
    lv_label_set_text(wifi, LV_SYMBOL_WIFI);
    lv_obj_align(wifi, NULL, LV_ALIGN_IN_LEFT_MID, 10, 0);

    battery = lv_label_create(header, NULL);
    lv_label_set_text(battery, LV_SYMBOL_BATTERY_3);
    lv_obj_align(battery, NULL, LV_ALIGN_IN_RIGHT_MID, -10, 0);

    lv_cont_set_fit2(header, false, true);   /*Let the height set automatically*/
    lv_obj_set_pos(header, 0, 0);
}

#include "esp_camera.h"

sensor_t * sensor = NULL;

lv_obj_t *list_btn[7];

static void disp1_list_event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
        printf("Clicked: %s\n", lv_list_get_btn_text(obj));

        if (sensor) {
            if (obj == list_btn[0]) {
                sensor->set_special_effect(sensor, 0);
                sensor->set_brightness(sensor, 1);
            } else if (obj == list_btn[1]) {
                sensor->set_special_effect(sensor, 1);
            } else if (obj == list_btn[2]) {
                sensor->set_special_effect(sensor, 2);
            } else if (obj == list_btn[3]) {
                sensor->set_special_effect(sensor, 3);
            } else if (obj == list_btn[4]) {
                sensor->set_special_effect(sensor, 4);
            } else if (obj == list_btn[5]) {
                sensor->set_special_effect(sensor, 5);
            } else if (obj == list_btn[6]) {
                sensor->set_special_effect(sensor, 6);
            }
        }
    }
}

void disp1_list(lv_obj_t * parent)
{
    /*Create a list*/
    lv_obj_t * list1 = lv_list_create(parent, NULL);
    lv_obj_set_size(list1, 135, 160);
    lv_obj_align(list1, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_group_add_obj(encoder_group, list1);

    /*Add buttons to the list*/

    // lv_obj_t * list_btn;

    list_btn[0] = lv_list_add_btn(list1, LV_SYMBOL_IMAGE, "Normal");
    lv_obj_set_event_cb(list_btn[0], disp1_list_event_handler);

    list_btn[1] = lv_list_add_btn(list1, LV_SYMBOL_IMAGE, "Negative");
    lv_obj_set_event_cb(list_btn[1], disp1_list_event_handler);

    list_btn[2] = lv_list_add_btn(list1, LV_SYMBOL_IMAGE, "Black");
    lv_obj_set_event_cb(list_btn[2], disp1_list_event_handler);


    list_btn[3] = lv_list_add_btn(list1, LV_SYMBOL_IMAGE, "Reddish");
    lv_obj_set_event_cb(list_btn[3], disp1_list_event_handler);

    list_btn[4] = lv_list_add_btn(list1, LV_SYMBOL_IMAGE, "Greenish");
    lv_obj_set_event_cb(list_btn[4], disp1_list_event_handler);

    list_btn[5] = lv_list_add_btn(list1, LV_SYMBOL_IMAGE, "Blue");
    lv_obj_set_event_cb(list_btn[5], disp1_list_event_handler);

    list_btn[6] = lv_list_add_btn(list1, LV_SYMBOL_IMAGE, "Retro");
    lv_obj_set_event_cb(list_btn[6], disp1_list_event_handler);
}

static void disp1_slider_event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
        printf("Value: %d, %c\n", lv_slider_get_value(obj), (char)obj->user_data);
        switch ((char)obj->user_data) {
            case 'W': {
                led_rgb.r = lv_slider_get_value(obj) * 2.55;
                led_rgb.g = led_rgb.r;
                led_rgb.b = led_rgb.r;
                // if (lv_slider_get_value(obj) % 25 == 0) {
                //     sensor->set_brightness(sensor, 5 - (lv_slider_get_value(obj) / 25));
                // }
            }
            break;
        }
        WS2812B_setLeds(&led_rgb, 1);
    }
}

void disp1_slider(lv_obj_t * parent)
{
    /*Create styles*/
    static lv_style_t style_bg;
    static lv_style_t style_indic;
    static lv_style_t style_knob;

    lv_style_copy(&style_bg, &lv_style_pretty);
    style_bg.body.main_color =  LV_COLOR_BLACK;
    style_bg.body.grad_color =  LV_COLOR_GRAY;
    style_bg.body.radius = LV_RADIUS_CIRCLE;
    style_bg.body.border.color = LV_COLOR_WHITE;

    lv_style_copy(&style_indic, &lv_style_pretty);
    style_indic.body.radius = LV_RADIUS_CIRCLE;
    style_indic.body.shadow.width = 8;
    style_indic.body.shadow.color = style_indic.body.main_color;
    style_indic.body.padding.left = 3;
    style_indic.body.padding.right = 3;
    style_indic.body.padding.top = 3;
    style_indic.body.padding.bottom = 3;

    lv_style_copy(&style_knob, &lv_style_pretty);
    style_knob.body.radius = LV_RADIUS_CIRCLE;
    style_knob.body.opa = LV_OPA_70;
    style_knob.body.padding.top = 10 ;
    style_knob.body.padding.bottom = 10 ;
    
    lv_obj_t * slider = lv_slider_create(parent, NULL);
    slider->user_data = (void *)'W';
    lv_obj_set_width(slider, 100);
    // lv_slider_set_value(slider, 50, LV_ANIM_OFF);
    lv_slider_set_style(slider, LV_SLIDER_STYLE_BG, &style_bg);
    lv_slider_set_style(slider, LV_SLIDER_STYLE_INDIC,&style_indic);
    lv_slider_set_style(slider, LV_SLIDER_STYLE_KNOB, &style_knob);
    lv_obj_align(slider, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
    lv_obj_set_event_cb(slider, disp1_slider_event_handler);
    lv_group_add_obj(encoder_group, slider);
    lv_group_focus_obj(slider);
}

void lv_ex_preload_1(lv_obj_t * parent)
{

    LV_IMG_DECLARE(espressif);
    lv_obj_t * esp_img = lv_img_create(parent, NULL);
    lv_img_set_src(esp_img, &espressif);
    lv_obj_align(esp_img, NULL, LV_ALIGN_CENTER, 0, 0);

    /*Create a style for the Preloader*/
    static lv_style_t style;
    lv_style_copy(&style, &lv_style_plain);
    style.line.width = 10;                         /*10 px thick arc*/
    style.line.color = lv_color_hex(0x3ec2fc);       /*Blueish arc color*/

    style.body.border.color = lv_color_hex3(0xBBB); /*Gray background color*/
    style.body.border.width = 10;
    style.body.border.opa = 200;
    style.body.padding.left = 0;

    /*Create a Preloader object*/
    lv_obj_t * preload = lv_preload_create(parent, NULL);
    lv_obj_set_size(preload, 100, 100);
    lv_obj_align(preload, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_preload_set_style(preload, LV_PRELOAD_STYLE_MAIN, &style);
    // lv_preload_get_style(preload, LV_PRELOAD_TYPE_FILLSPIN_ARC);
    lv_preload_set_spin_time(preload, 500);
}

void gui_init(lv_disp_t **disp_array, lv_indev_t **indev_array, lv_theme_t * th)
{
    disp = disp_array;
    indev = indev_array;
    gui_event_queue = xQueueCreate(5, sizeof(gui_event_t));
    lv_theme_set_current(th);
    th = lv_theme_get_current();    /*If `LV_THEME_LIVE_UPDATE  1` `th` is not used directly so get the real theme after set*/
    lv_cont_set_style(lv_disp_get_scr_act(disp[0]), LV_CONT_STYLE_MAIN, th->style.bg);
    lv_cont_set_style(lv_disp_get_scr_act(disp[1]), LV_CONT_STYLE_MAIN, th->style.bg);

    lv_ex_preload_1(lv_disp_get_scr_act(disp[0]));

    sensor = esp_camera_sensor_get();

    LV_FONT_DECLARE(my_symbol);
    lv_style_copy(&style_my_symbol, &lv_style_scr);
    style_my_symbol.text.font = &my_symbol;

    encoder_group = lv_group_create();
    lv_indev_set_group(indev[0], encoder_group);

    header1_create(lv_disp_get_scr_act(disp[1]));

    disp1_list(lv_disp_get_scr_act(disp[1]));

    disp1_slider(lv_disp_get_scr_act(disp[1]));

}
