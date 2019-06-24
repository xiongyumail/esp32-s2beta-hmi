#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "gui.h"

typedef struct {
    int event;
    void *arg;
} gui_event_t;

#define GUI_WIFI_EVENT 0
#define GUI_BATTERY_EVENT 1
#define GUI_TIME_EVENT 2
#define GUI_SENSOR_EVENT 3
#define GUI_MOTION_EVENT 3

typedef struct {
    float temp;
    float hum;
    float light;
    float pitch;
    float roll;
    float yaw;
} gui_sensor_t;

#define MY_TEMP_SYMBOL "\xEF\x8B\x88"  // f2c8
#define MY_MOTION_SYMBOL "\xEF\x8B\x9B"  // f2db
#define MY_LED_SYMBOL "\xEF\x83\xAB"  // f0eb
#define MY_TOUCH_SYMBOL "\xEF\x82\xA6" // f0a6
#define MY_CAMERA_SYMBOL "\xEF\x85\xAD" // f16d
#define MY_ESPRESSIF_SYMBOL "\xEF\x8B\xA1" // f2e1

static lv_style_t style_my_symbol;

static lv_obj_t * header;
static lv_obj_t * state;
static lv_obj_t * time_clock;
static lv_obj_t * wifi;
static lv_obj_t * battery;

static lv_obj_t * body;
static lv_obj_t * temp;
static lv_obj_t * hum;
static lv_obj_t * light;
static lv_obj_t * motion;

typedef enum {
    GUI_PAGE_MONITOR = 0,
    GUI_PAGE_MOTION,
    GUI_PAGE_LED,
    GUI_PAGE_TOUCH,
    GUI_PAGE_AUDIO,
    GUI_PAGE_CAMERA,
    GUI_PAGE_TERMINAL,
    GUI_PAGE_INFO
} gui_page_t;

static gui_page_t gui_page = GUI_PAGE_MONITOR;

static gui_sensor_t gui_sensor;

static QueueHandle_t gui_event_queue = NULL;

static void time_update()
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

static void sensor_update(gui_sensor_t *sensor) 
{
    char str[64];
    if (gui_page == GUI_PAGE_MONITOR) {
        sprintf(str, "Temp: %.2f", sensor->temp);
        lv_label_set_text(temp, str);
        sprintf(str, "Hum: %.2f", sensor->hum);
        lv_label_set_text(hum, str);
        sprintf(str, "light: %.2f", sensor->light);
        lv_label_set_text(light, str);
    }
    if (gui_page == GUI_PAGE_MOTION) {
        sprintf(str, "Motion: %.2f, %.2f, %.2f", sensor->pitch, sensor->roll, sensor->yaw);
        lv_label_set_text(motion, str);
    }
}

static void header_create(void)
{
    header = lv_cont_create(lv_disp_get_scr_act(NULL), NULL);
    lv_obj_set_width(header, LV_HOR_RES);
    
    time_clock = lv_label_create(header, NULL);
    lv_obj_align(time_clock, NULL, LV_ALIGN_IN_LEFT_MID, LV_DPI/10, 0);
    lv_label_set_recolor(time_clock, true);
    time_update();

    state = lv_label_create(header, NULL);
    lv_obj_align(state, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(state, MY_TEMP_SYMBOL);
    lv_obj_set_style(state, &style_my_symbol);

    wifi = lv_label_create(header, NULL);
    lv_label_set_text(wifi, "");
    lv_obj_align(wifi, NULL, LV_ALIGN_IN_RIGHT_MID, -(LV_DPI/10) * 5, 0);

    battery = lv_label_create(header, NULL);
    lv_label_set_text(battery, "");
    lv_obj_align(battery, NULL, LV_ALIGN_IN_RIGHT_MID, -(LV_DPI/10) * 3, 0);

    lv_cont_set_fit2(header, false, true);   /*Let the height set automatically*/
    lv_obj_set_pos(header, 0, 0);
}

static void body_page_monitor(lv_obj_t * parent)
{
    /*Create a Calendar*/
    lv_obj_t * cal = lv_calendar_create(parent, NULL);
    lv_obj_set_size(cal, 5 * LV_DPI / 2, 5 * LV_DPI / 2);

    time_t now;
    struct tm timeinfo;
    
    time(&now);
    localtime_r(&now, &timeinfo);

    lv_calendar_date_t today;
    today.year = timeinfo.tm_year + 1900;
    today.month = timeinfo.tm_mon + 1;
    today.day = timeinfo.tm_mday;

    lv_calendar_set_today_date(cal, &today);
    lv_calendar_set_showed_date(cal, &today);

    lv_obj_t * h = lv_cont_create(parent, NULL);
    // lv_obj_set_click(h, false);
    lv_cont_set_fit(h, LV_FIT_TIGHT);
    lv_cont_set_layout(h, LV_LAYOUT_COL_L);

    temp = lv_label_create(h, NULL);
    hum = lv_label_create(h, NULL);
    light = lv_label_create(h, NULL);
    sensor_update(&gui_sensor);

    lv_obj_set_pos(cal, 0, 0);
    // lv_obj_align(cal, NULL, LV_ALIGN_IN_LEFT_MID, LV_DPI/10, 0);
}

static void body_page_motion(lv_obj_t * parent)
{
    lv_obj_t * h = lv_cont_create(parent, NULL);
    // lv_obj_set_click(h, false);
    lv_cont_set_fit(h, LV_FIT_TIGHT);
    lv_cont_set_layout(h, LV_LAYOUT_COL_L);

    motion = lv_label_create(h, NULL);
    sensor_update(&gui_sensor);
}

static void body_page_led(lv_obj_t * parent)
{
    /*Create a Window*/
    lv_obj_t * win = lv_win_create(parent, NULL);
    lv_win_add_btn(win, LV_SYMBOL_CLOSE, lv_win_close_event);
    lv_win_add_btn(win, LV_SYMBOL_DOWN, NULL);
    lv_obj_set_size(win, lv_disp_get_hor_res(NULL) / 2, lv_disp_get_ver_res(NULL) / 2);
    lv_obj_set_pos(win, LV_DPI / 20, LV_DPI / 20);
    lv_obj_set_top(win, true);


    /*Create a Label in the Window*/
    lv_obj_t * label = lv_label_create(win, NULL);
    lv_label_set_text(label, "Label in the window");

    /*Create a  Line meter in the Window*/
    lv_obj_t * lmeter = lv_lmeter_create(win, NULL);
    lv_obj_align(lmeter, label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, LV_DPI / 2);
    lv_lmeter_set_value(lmeter, 70);

    /*Create a 2 LEDs in the Window*/
    lv_obj_t * led1 = lv_led_create(win, NULL);
    lv_obj_align(led1, lmeter, LV_ALIGN_OUT_RIGHT_MID, LV_DPI / 2, 0);
    lv_led_on(led1);

    lv_obj_t * led2 = lv_led_create(win, NULL);
    lv_obj_align(led2, led1, LV_ALIGN_OUT_RIGHT_MID, LV_DPI / 2, 0);
    lv_led_off(led2);

    /*Create a Page*/
    lv_obj_t * page = lv_page_create(parent, NULL);
    lv_obj_set_size(page, lv_disp_get_hor_res(NULL) / 3, lv_disp_get_ver_res(NULL) / 2);
    lv_obj_set_top(page, true);
    lv_obj_align(page, win, LV_ALIGN_IN_TOP_RIGHT,  LV_DPI, LV_DPI);

    label = lv_label_create(page, NULL);
    lv_label_set_text(label, "Lorem ipsum dolor sit amet, repudiare voluptatibus pri cu.\n"
                      "Ei mundi pertinax posidonium eum, cum tempor maiorum at,\n"
                      "mea fuisset assentior ad. Usu cu suas civibus iudicabit.\n"
                      "Eum eu congue tempor facilisi. Tale hinc unum te vim.\n"
                      "Te cum populo animal eruditi, labitur inciderint at nec.\n\n"
                      "Eius corpora et quo. Everti voluptaria instructior est id,\n"
                      "vel in falli primis. Mea ei porro essent admodum,\n"
                      "his ei malis quodsi, te quis aeterno his.\n"
                      "Qui tritani recusabo reprehendunt ne,\n"
                      "per duis explicari at. Simul mediocritatem mei et.");

    /*Create a Calendar*/
    lv_obj_t * cal = lv_calendar_create(parent, NULL);
    lv_obj_set_size(cal, 5 * LV_DPI / 2, 5 * LV_DPI / 2);
    lv_obj_align(cal, page, LV_ALIGN_OUT_RIGHT_TOP, -LV_DPI / 2, LV_DPI / 3);
    lv_obj_set_top(cal, true);

    static lv_calendar_date_t highlighted_days[2];
    highlighted_days[0].day = 5;
    highlighted_days[0].month = 5;
    highlighted_days[0].year = 2018;

    highlighted_days[1].day = 8;
    highlighted_days[1].month = 5;
    highlighted_days[1].year = 2018;

    lv_calendar_set_highlighted_dates(cal, highlighted_days, 2);
    lv_calendar_set_today_date(cal, &highlighted_days[0]);
    lv_calendar_set_showed_date(cal, &highlighted_days[0]);

    /*Create a Message box*/
    static const char * mbox_btn_map[] = {" ", "Got it!", " ", ""};
    lv_obj_t * mbox = lv_mbox_create(parent, NULL);
    lv_mbox_set_text(mbox, "Click on the window or the page to bring it to the foreground");
    lv_mbox_add_btns(mbox, mbox_btn_map);
    lv_btnm_set_btn_ctrl(lv_mbox_get_btnm(mbox), 0, LV_BTNM_CTRL_HIDDEN);
    lv_btnm_set_btn_width(lv_mbox_get_btnm(mbox), 1, 7);
    lv_btnm_set_btn_ctrl(lv_mbox_get_btnm(mbox), 2, LV_BTNM_CTRL_HIDDEN);
    lv_obj_set_top(mbox, true);


}

static void body_page_camera(lv_obj_t * parent)
{
    static lv_color_t *canvas_buffer = NULL;
    if (canvas_buffer == NULL) {
        canvas_buffer = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(400, 400), MALLOC_CAP_SPIRAM);
    }

    lv_obj_t * h = lv_cont_create(parent, NULL);
    // lv_obj_set_click(h, false);
    lv_cont_set_fit(h, LV_FIT_TIGHT);
    lv_cont_set_layout(h, LV_LAYOUT_COL_L);
    
    static lv_style_t style;
    lv_style_copy(&style, &lv_style_plain);
    style.text.color = LV_COLOR_RED;
    lv_obj_t * canvas = lv_canvas_create(h, NULL);
    lv_canvas_set_buffer(canvas, canvas_buffer, 400, 400, LV_IMG_CF_TRUE_COLOR);
    lv_canvas_fill_bg(canvas, LV_COLOR_BLACK);
    // lv_canvas_set_px(canvas, 10, 10, LV_COLOR_RED);
    lv_canvas_draw_text(canvas, 0, 200, 400, &style, "NO SIGNAL!", LV_LABEL_ALIGN_CENTER);
}

lv_obj_t * terminal_ta;
lv_obj_t * terminal_kb;

// #undef LV_USE_ANIMATION
#if LV_USE_ANIMATION
static void kb_hide_anim_end(lv_anim_t * a)
{
    lv_obj_del(a->var);
    terminal_kb = NULL;
}
#endif

static void keyboard_event_cb(lv_obj_t * keyboard, lv_event_t event)
{
    (void) keyboard;    /*Unused*/

    lv_kb_def_event_cb(terminal_kb, event);

    if(event == LV_EVENT_APPLY || event == LV_EVENT_CANCEL) {
#if LV_USE_ANIMATION
        lv_anim_t a;
        a.var = terminal_kb;
        a.start = lv_obj_get_y(terminal_kb);
        a.end = LV_VER_RES;
        a.exec_cb = (lv_anim_exec_xcb_t)lv_obj_set_y;
        a.path_cb = lv_anim_path_linear;
        a.ready_cb = kb_hide_anim_end;
        a.act_time = 0;
        a.time = 300;
        a.playback = 0;
        a.playback_pause = 0;
        a.repeat = 0;
        a.repeat_pause = 0;
        a.user_data = NULL;
        lv_anim_create(&a);
#else
        lv_obj_del(terminal_kb);
        terminal_kb = NULL;
#endif
    }
}

static void text_area_event_handler(lv_obj_t * text_area, lv_event_t event)
{
    (void) text_area;    /*Unused*/

    /*Text area is on the scrollable part of the page but we need the page itself*/
    lv_obj_t * parent = lv_obj_get_parent(terminal_ta);

    if(event == LV_EVENT_CLICKED) {
        if(terminal_kb == NULL) {
            printf("test\n");
            terminal_kb = lv_kb_create(parent, NULL);
            lv_obj_set_size(terminal_kb, 520, 180);
            lv_obj_align(terminal_kb, terminal_ta, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, LV_DPI);
            lv_kb_set_ta(terminal_kb, terminal_ta);
            lv_obj_set_event_cb(terminal_kb, keyboard_event_cb);

#if LV_USE_ANIMATION
            lv_anim_t a;
            a.var = terminal_kb;
            a.start = LV_VER_RES;
            a.end = lv_obj_get_y(terminal_kb);
            a.exec_cb = (lv_anim_exec_xcb_t)lv_obj_set_y;
            a.path_cb = lv_anim_path_linear;
            a.ready_cb = NULL;
            a.act_time = 0;
            a.time = 300;
            a.playback = 0;
            a.playback_pause = 0;
            a.repeat = 0;
            a.repeat_pause = 0;
            a.user_data = NULL;
            lv_anim_create(&a);
#endif
        }
    }

}

static void body_page_terminal(lv_obj_t * parent)
{
    lv_obj_t * h = lv_cont_create(parent, NULL);
    lv_obj_set_click(h, true);
    lv_cont_set_fit(h, LV_FIT_TIGHT);
    lv_cont_set_layout(h, LV_LAYOUT_COL_L);
    
    static lv_style_t style;
    lv_style_copy(&style, &lv_style_plain);
    style.text.color = LV_COLOR_GREEN;

    terminal_ta = lv_ta_create(h, NULL);
    lv_obj_set_size(terminal_ta, 520, 200);
    lv_obj_align(terminal_ta, NULL, LV_ALIGN_IN_TOP_RIGHT, -LV_DPI / 10, LV_DPI / 10);
    lv_ta_set_cursor_type(terminal_ta, LV_CURSOR_BLOCK);
    lv_obj_set_event_cb(terminal_ta, text_area_event_handler);
    lv_ta_set_text_sel(terminal_ta, true);
    lv_ta_set_text(terminal_ta, "[esp@localhost ~]$ cd esp\n"
                       "[esp@localhost ~/esp]$ ls\n"
                       "esp-idf xtensa-esp32s2-elf\n"
                       "[esp@localhost ~/esp]$ ");

    terminal_kb = lv_kb_create(h, NULL);
    lv_obj_set_size(terminal_kb, 520, 180);
    lv_obj_align(terminal_kb, terminal_ta, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, LV_DPI);
    lv_kb_set_ta(terminal_kb, terminal_ta);
    lv_obj_set_event_cb(terminal_kb, keyboard_event_cb);
}

static void body_page_info(lv_obj_t * parent)
{
    // lv_obj_t * txt = lv_label_create(parent, NULL);
    // lv_label_set_text(txt, "espressif");
    LV_IMG_DECLARE(espressif);
    lv_obj_t * esp_img = lv_img_create(parent, NULL);
    lv_img_set_src(esp_img, &espressif);
}

static void side_btn_event_callback(lv_obj_t * obj, lv_event_t event)
{
    switch(event) {
        case LV_EVENT_PRESSED:
            printf("Pressed\n");
            break;

        case LV_EVENT_SHORT_CLICKED:
            printf("Short clicked\n");
            break;

        case LV_EVENT_CLICKED:
            printf("Clicked\n");
            break;

        case LV_EVENT_LONG_PRESSED:
            printf("Long press\n");
            break;

        case LV_EVENT_LONG_PRESSED_REPEAT:
            printf("Long press repeat\n");
            break;

        case LV_EVENT_RELEASED:
            printf("Released\n");
            if (gui_page != (gui_page_t)obj->user_data) {
                lv_page_clean(body);
                gui_page = (gui_page_t)obj->user_data;
                switch ((int)gui_page) {
                    case GUI_PAGE_MONITOR: {
                        lv_label_set_text(state, MY_TEMP_SYMBOL);
                        lv_obj_set_style(state, &style_my_symbol);
                        body_page_monitor(body);
                    }
                    break;

                    case GUI_PAGE_MOTION: {
                        lv_label_set_text(state, MY_MOTION_SYMBOL);
                        lv_obj_set_style(state, &style_my_symbol);
                        body_page_motion(body);
                    }
                    break;

                    case GUI_PAGE_LED: {
                        lv_label_set_text(state, MY_LED_SYMBOL);
                        lv_obj_set_style(state, &style_my_symbol);
                        body_page_led(body);
                    }
                    break;

                    case GUI_PAGE_TOUCH: {
                        lv_label_set_text(state, MY_TOUCH_SYMBOL);
                        lv_obj_set_style(state, &style_my_symbol);
                        // body_page_touch(body);
                    }
                    break;

                    case GUI_PAGE_AUDIO: {
                        lv_label_set_text(state, LV_SYMBOL_AUDIO);
                        lv_obj_set_style(state, &lv_style_scr);
                        // body_page_audio(body);
                    }
                    break;

                    case GUI_PAGE_CAMERA: {
                        lv_label_set_text(state, MY_CAMERA_SYMBOL);
                        lv_obj_set_style(state, &style_my_symbol);
                        body_page_camera(body);
                    }
                    break;

                    case GUI_PAGE_TERMINAL: {
                        lv_label_set_text(state, LV_SYMBOL_KEYBOARD);
                        lv_obj_set_style(state, &lv_style_scr);
                        body_page_terminal(body);
                    }
                    break;

                    case GUI_PAGE_INFO: {
                        lv_label_set_text(state, MY_ESPRESSIF_SYMBOL);
                        lv_obj_set_style(state, &style_my_symbol);
                        body_page_info(body);
                    }
                    break;
                }
            }
            
            break;
    }
}

static void side_create(void)
{
    // lv_obj_t * h = lv_cont_create(lv_scr_act(), NULL);
    // lv_cont_set_fit(h, LV_FIT_TIGHT);
    // lv_cont_set_layout(h, LV_LAYOUT_CENTER);

    lv_obj_t * list = lv_list_create(lv_scr_act(), NULL);

    lv_obj_t * label;
    lv_obj_t * list_btn;

    list_btn = lv_list_add(list, MY_TEMP_SYMBOL, "Monitor", NULL);
    list_btn->user_data = GUI_PAGE_MONITOR;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_my_symbol);

    list_btn = lv_list_add(list, MY_MOTION_SYMBOL, "Motion", NULL);
    list_btn->user_data = GUI_PAGE_MOTION;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_my_symbol);

    list_btn = lv_list_add(list, MY_LED_SYMBOL, "LED", NULL);
    list_btn->user_data = GUI_PAGE_LED;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_my_symbol);

    list_btn = lv_list_add(list, MY_TOUCH_SYMBOL, "Touch", NULL);
    list_btn->user_data = GUI_PAGE_TOUCH;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_my_symbol);

    list_btn = lv_list_add(list, LV_SYMBOL_AUDIO, "Audio", NULL);
    list_btn->user_data = GUI_PAGE_AUDIO;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    // label = lv_list_get_btn_img(list_btn);
    // lv_obj_set_style(label, &style_symbol);

    list_btn = lv_list_add(list, MY_CAMERA_SYMBOL, "Camera", NULL);
    list_btn->user_data = GUI_PAGE_CAMERA;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_my_symbol);

    list_btn = lv_list_add(list, LV_SYMBOL_KEYBOARD, "Terminal", NULL);
    list_btn->user_data = GUI_PAGE_TERMINAL;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);

    list_btn = lv_list_add(list, MY_ESPRESSIF_SYMBOL, "Info", NULL);
    list_btn->user_data = GUI_PAGE_INFO;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_my_symbol);

    lv_obj_set_pos(list, LV_HOR_RES - (LV_HOR_RES / 4) * 1, lv_obj_get_height(header));

    lv_obj_t * h = lv_cont_create(lv_scr_act(), NULL);
    // lv_cont_set_fit(h, LV_FIT_FLOOD);
    // lv_cont_set_fit2(h, LV_FIT_FLOOD, LV_FIT_FLOOD);
    lv_cont_set_layout(h, LV_LAYOUT_CENTER);
    lv_obj_t * txt = lv_label_create(h, NULL);
    lv_label_set_text(txt, "ESP32-S2Beta-HMI");
    LV_IMG_DECLARE(QR);
    // lv_obj_t * canvas = lv_canvas_create(h, NULL);
    // lv_canvas_set_buffer(canvas, canvas_buffer, 100, 100, LV_IMG_CF_TRUE_COLOR);
    // lv_canvas_fill_bg(canvas, LV_COLOR_BLUE);
    // lv_canvas_set_px(canvas, 10, 10, LV_COLOR_RED);
    // lv_canvas_draw_img(canvas, 0, 0, &QR, NULL);
    lv_obj_t * qr = lv_img_create(h, NULL);
    lv_img_set_src(qr, &QR);

    lv_obj_set_size(h, (LV_HOR_RES / 4) * 1, LV_VER_RES - (lv_obj_get_height(header) + lv_obj_get_height(list)));
    lv_obj_set_pos(h, LV_HOR_RES - (LV_HOR_RES / 4) * 1, lv_obj_get_height(header) + lv_obj_get_height(list));
}

static void body_create(void)
{
    body = lv_page_create(lv_disp_get_scr_act(NULL), NULL);

    lv_obj_set_width(body, (LV_HOR_RES / 4) * 3);
    lv_obj_set_height(body, LV_VER_RES - lv_obj_get_height(header));
    lv_page_set_sb_mode(body, LV_SB_MODE_DRAG);
    lv_page_set_scrl_layout(body, LV_LAYOUT_ROW_M);

    lv_obj_set_pos(body, 0, lv_obj_get_height(header));
    body_page_monitor(body);
}

static void gui_task(lv_task_t * arg)
{
    int ret;
    gui_event_t e;
    while (1) {
        ret = xQueueReceive(gui_event_queue, &e, 0);
        if (ret == pdFAIL) {
            break;
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
                time_update();
            }
            break;

            case GUI_SENSOR_EVENT: {
                sensor_update((gui_sensor_t *)e.arg);
            }
            break;
        }
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

int gui_set_sensor(float temp, float hum, float light, int ticks_wait) 
{
    gui_sensor.temp = temp;
    gui_sensor.hum = hum;
    gui_sensor.light = light;
    return gui_event_send(GUI_SENSOR_EVENT, (void *)&gui_sensor, ticks_wait);
}

int gui_set_motion(float pitch, float roll, float yaw, int ticks_wait) 
{
    gui_sensor.pitch = pitch;
    gui_sensor.roll = roll;
    gui_sensor.yaw = yaw;
    return gui_event_send(GUI_SENSOR_EVENT, (void *)&gui_sensor, ticks_wait);
}

void gui_init(lv_theme_t * th)
{
    gui_event_queue = xQueueCreate(5, sizeof(gui_event_t));
    lv_theme_set_current(th);
    th = lv_theme_get_current();    /*If `LV_THEME_LIVE_UPDATE  1` `th` is not used directly so get the real theme after set*/
    lv_obj_t * scr = lv_cont_create(NULL, NULL);
    lv_disp_set_scr_act(scr);
    lv_cont_set_style(scr, LV_CONT_STYLE_MAIN, th->style.bg);
    // lv_cont_set_style(scr, th->style.bg);

    LV_FONT_DECLARE(my_symbol);
    lv_style_copy(&style_my_symbol, &lv_style_scr);
    style_my_symbol.text.font = &my_symbol;

    header_create();
    side_create();
    body_create();
    lv_task_create(gui_task, 10, LV_TASK_PRIO_MID, NULL);
}
