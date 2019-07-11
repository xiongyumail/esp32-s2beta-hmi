#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "esp_heap_caps.h"
#include "gui.h"
#include "WS2812B.h"

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

static lv_obj_t * chart = NULL;
static lv_chart_series_t * ser_pitch;
static lv_chart_series_t * ser_roll;
static lv_chart_series_t * ser_yaw;
static lv_obj_t * motion_table;

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

static gui_page_t gui_page = GUI_PAGE_LED;

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
        sprintf(str, "%.2f", sensor->pitch);
        lv_table_set_cell_value(motion_table, 0, 1, str);
        sprintf(str, "%.2f", sensor->roll);
        lv_table_set_cell_value(motion_table, 1, 1, str);
        sprintf(str, "%.2f", sensor->yaw);
        lv_table_set_cell_value(motion_table, 2, 1, str);
        lv_chart_set_next(chart, ser_pitch, (uint16_t)(sensor->pitch + 180));
        lv_chart_set_next(chart, ser_roll, (uint16_t)(sensor->roll + 180));
        lv_chart_set_next(chart, ser_yaw, (uint16_t)(sensor->yaw + 180));
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
    lv_obj_set_click(h, false);
    lv_obj_set_drag(h, true);
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
    lv_obj_t * h1 = lv_cont_create(parent, NULL);
    lv_cont_set_fit(h1, LV_FIT_TIGHT);
    lv_cont_set_layout(h1, LV_LAYOUT_COL_M);

    chart = lv_chart_create(h1, NULL);
    lv_obj_set_size(chart, 550, 250);
    lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/
    lv_chart_set_series_opa(chart, LV_OPA_70);                            /*Opacity of the data series*/
    lv_chart_set_series_width(chart, 4);                                  /*Line width and point radious*/
    lv_chart_set_div_line_count(chart, 10, 10);
    lv_chart_set_point_count(chart, 20);
    lv_chart_set_range(chart, 0, 360);

    const char * list_of_values = "first\nseco\nthird\n";
    lv_chart_set_x_tick_length(chart, LV_CHART_TICK_LENGTH_AUTO, LV_CHART_TICK_LENGTH_AUTO);
    lv_chart_set_x_tick_texts(chart, list_of_values, 3, LV_CHART_AXIS_DRAW_LAST_TICK);

    ser_pitch = lv_chart_add_series(chart, LV_COLOR_RED);
    ser_roll = lv_chart_add_series(chart, LV_COLOR_GREEN);
    ser_yaw = lv_chart_add_series(chart, LV_COLOR_BLUE);

    /*Create a normal cell style*/
    static lv_style_t style_cell1;
    lv_style_copy(&style_cell1, &lv_style_plain);
    style_cell1.body.border.width = 1;
    style_cell1.body.border.color = LV_COLOR_BLACK;
    style_cell1.text.font = &lv_font_roboto_28;
    style_cell1.text.color = LV_COLOR_RED;

    static lv_style_t style_cell2;
    lv_style_copy(&style_cell2, &lv_style_plain);
    style_cell2.body.border.width = 1;
    style_cell2.body.border.color = LV_COLOR_BLACK;
    style_cell2.text.font = &lv_font_roboto_28;
    style_cell2.text.color = LV_COLOR_GREEN;

    static lv_style_t style_cell3;
    lv_style_copy(&style_cell3, &lv_style_plain);
    style_cell3.body.border.width = 1;
    style_cell3.body.border.color = LV_COLOR_BLACK;
    style_cell3.text.font = &lv_font_roboto_28;
    style_cell3.text.color = LV_COLOR_BLUE;

    motion_table = lv_table_create(h1, NULL);
    lv_table_set_style(motion_table, LV_TABLE_STYLE_CELL1, &style_cell1);
    lv_table_set_style(motion_table, LV_TABLE_STYLE_CELL2, &style_cell2);
    lv_table_set_style(motion_table, LV_TABLE_STYLE_CELL3, &style_cell3);
    lv_obj_align(motion_table, chart, LV_ALIGN_OUT_BOTTOM_MID, 0, LV_DPI / 4);

    lv_table_set_col_cnt(motion_table, 2);
    lv_table_set_row_cnt(motion_table, 3);
    lv_table_set_col_width(motion_table, 0, LV_DPI);
    lv_table_set_col_width(motion_table, 1, LV_DPI);

    lv_table_set_cell_type(motion_table, 0, 0, 1);
    lv_table_set_cell_type(motion_table, 0, 1, 1);
    lv_table_set_cell_type(motion_table, 1, 0, 2);
    lv_table_set_cell_type(motion_table, 1, 1, 2);
    lv_table_set_cell_type(motion_table, 2, 0, 3);
    lv_table_set_cell_type(motion_table, 2, 1, 3);

    lv_table_set_cell_value(motion_table, 0, 0, "Pitch:");
    lv_table_set_cell_value(motion_table, 1, 0, "Roll:");
    lv_table_set_cell_value(motion_table, 2, 0, "Yaw:");

    lv_table_set_cell_value(motion_table, 0, 1, "");
    lv_table_set_cell_value(motion_table, 1, 1, "");
    lv_table_set_cell_value(motion_table, 2, 1, "");

    sensor_update(&gui_sensor);
}

static void body_page_touch(lv_obj_t * parent)
{

}

static lv_obj_t * color_picker;
static lv_obj_t * color_picker_led;
static lv_obj_t * color_picker_table;
static lv_color_t *color_picker_buffer = NULL;
static lv_color_hsv_t color_picker_hsv = {360, 100, 100};
static wsRGB_t led_rgb;
static bool led_state = 1;

#define COLOR_PICKER_W 300
#define COLOR_PICKER_H 300
#define COLOR_PICKER_HUE_W 28
#define COLOR_PICKER_HUE_H 300

static void color_picker_copy(lv_color_t *buffer, uint16_t hsv_h)
{
    for (int y=0; y < COLOR_PICKER_H; y++) {
        for (int x = 0; x < COLOR_PICKER_W; x++) {
            buffer[y*COLOR_PICKER_W + x] = lv_color_hsv_to_rgb(hsv_h, (uint8_t)(x * (100.0 / COLOR_PICKER_W)), (uint8_t)(100 - y * (100.0 / COLOR_PICKER_H)));
        }
    }
}

static void color_picker_hue_copy(lv_color_t *buffer)
{
    for (int y=0; y < COLOR_PICKER_HUE_H; y++) {
        for (int x = 0; x < COLOR_PICKER_HUE_W; x++) {
            buffer[y*COLOR_PICKER_HUE_W + x] = lv_color_hsv_to_rgb((uint16_t)(360 - y * (360.0 / COLOR_PICKER_HUE_H)), 100, 100);
        }
    }
}

static void color_picker_table_update(lv_color_hsv_t hsv)
{
    char str[16];

    lv_color_t rgb;
    rgb = lv_color_hsv_to_rgb(hsv.h, hsv.s, hsv.v);
    sprintf(str, "%d", rgb.ch.red << 3);
    lv_table_set_cell_value(color_picker_table, 0, 1, str);
    sprintf(str, "%d", rgb.ch.green << 2);
    lv_table_set_cell_value(color_picker_table, 1, 1, str);
    sprintf(str, "%d", rgb.ch.blue << 3);
    lv_table_set_cell_value(color_picker_table, 2, 1, str);

    sprintf(str, "%d", hsv.h);
    lv_table_set_cell_value(color_picker_table, 0, 3, str);
    sprintf(str, "%d", hsv.s);
    lv_table_set_cell_value(color_picker_table, 1, 3, str);
    sprintf(str, "%d", hsv.v);
    lv_table_set_cell_value(color_picker_table, 2, 3, str);
    sprintf(str, "#%06x", rgb.ch.red << 19 | rgb.ch.green << 10 | rgb.ch.blue << 3);
    lv_table_set_cell_value(color_picker_table, 3, 0, str);
}

static void color_picker_event_cb(lv_obj_t * obj, lv_event_t event)
{
    lv_area_t a;
    lv_point_t p;
    lv_obj_get_coords(obj, &a);
    lv_indev_get_point(lv_indev_get_act(), &p);
    lv_obj_t *child = lv_obj_get_child(obj, NULL);
    switch(event) {
        case LV_EVENT_PRESSED:
        case LV_EVENT_PRESSING:
            if (obj != color_picker) {
                lv_obj_set_y(child, p.y - a.y1 - lv_obj_get_height(child) / 2);
            } else {
                lv_obj_set_pos(child, p.x - a.x1 - lv_obj_get_width(child) / 2, p.y - a.y1 - lv_obj_get_height(child) / 2);
            }
            break;
        case LV_EVENT_RELEASED:
            if (obj != color_picker) {
                if ((p.y - a.y1) >= 0 && (p.y - a.y1) <= COLOR_PICKER_HUE_H) {
                    color_picker_hsv.h = (uint16_t)(360 - (p.y - a.y1) * (360.0 / COLOR_PICKER_HUE_H));
                } else {
                    if (p.y - a.y1 < 0) {
                        color_picker_hsv.h = 360;
                    } else {
                        color_picker_hsv.h = 0;
                    }
                }
                color_picker_copy(color_picker_buffer, color_picker_hsv.h);
                lv_obj_refresh_ext_draw_pad(color_picker);
                lv_obj_set_y(child, (COLOR_PICKER_HUE_H - color_picker_hsv.h * (COLOR_PICKER_HUE_H / 360.0)) - lv_obj_get_height(child) / 2);
            } else {
                if ((p.x - a.x1) >= 0 && (p.x - a.x1) <= COLOR_PICKER_W) {
                    color_picker_hsv.s = (uint8_t)((p.x - a.x1) * (100.0 / COLOR_PICKER_W));
                } else {
                    if (p.x - a.x1 < 0) {
                        color_picker_hsv.s = 0;
                    } else {
                        color_picker_hsv.s = 100;
                    }
                }
                if ((p.y - a.y1) >= 0 && (p.y - a.y1) <= COLOR_PICKER_H) {
                    color_picker_hsv.v = (uint8_t)(100 - (p.y - a.y1) * (100.0 / COLOR_PICKER_H));
                } else {
                    if (p.y - a.y1 < 0) {
                        color_picker_hsv.v = 100;
                    } else {
                        color_picker_hsv.v = 0;
                    }
                }
                lv_obj_set_pos(child, (color_picker_hsv.s * (COLOR_PICKER_W / 100.0)) - lv_obj_get_width(child) / 2, (COLOR_PICKER_H - color_picker_hsv.v * (COLOR_PICKER_H / 100.0)) - lv_obj_get_height(child) / 2);
            }
            lv_style_t *led_style = lv_obj_get_style(color_picker_led);
            led_style->body.main_color = lv_color_hsv_to_rgb(color_picker_hsv.h, color_picker_hsv.s, color_picker_hsv.v);
            lv_obj_refresh_style(color_picker_led);
            color_picker_table_update(color_picker_hsv);
            led_rgb.r = led_style->body.main_color.ch.red << 3;
            led_rgb.g = led_style->body.main_color.ch.green << 2;
            led_rgb.b = led_style->body.main_color.ch.blue << 3;
            if (led_state) {
                WS2812B_setLeds(&led_rgb, 1);
            }

            break;
    }
}

static void led_event_cb(lv_obj_t * obj, lv_event_t event)
{
    wsRGB_t rgb;
    switch(event) {
        case LV_EVENT_PRESSED:
            lv_led_toggle(obj);
            led_state = lv_led_get_bright(obj) == 255 ? 1 : 0;
            if (led_state == 0) {
                rgb.r = 0;
                rgb.g = 0;
                rgb.b = 0;
                WS2812B_setLeds(&rgb, 1);
            } else {
                WS2812B_setLeds(&led_rgb, 1);
            }
            printf("led state: %s\n", (lv_led_get_bright(obj) == 255 ?  "on" : "off" ));
        break;
    }
}

static void body_page_led(lv_obj_t * parent)
{

    lv_obj_t * h1 = lv_cont_create(parent, NULL);
    lv_cont_set_fit(h1, LV_FIT_TIGHT);
    lv_cont_set_layout(h1, LV_LAYOUT_ROW_M);

    static lv_style_t style_thumb;
    lv_style_copy(&style_thumb, &lv_style_pretty);
    style_thumb.body.radius = LV_RADIUS_CIRCLE;
    style_thumb.body.opa = LV_OPA_50;
    style_thumb.body.padding.left = 10 ;

    if (color_picker_buffer == NULL) {
        color_picker_buffer = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(COLOR_PICKER_W, COLOR_PICKER_H), MALLOC_CAP_SPIRAM);
    }

    color_picker = lv_canvas_create(h1, NULL);
    lv_canvas_set_buffer(color_picker, color_picker_buffer, COLOR_PICKER_W, COLOR_PICKER_H, LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_click(color_picker, true);
    lv_obj_set_protect(color_picker, LV_PROTECT_PRESS_LOST);
    lv_obj_set_event_cb(color_picker, color_picker_event_cb);
    color_picker_copy(color_picker_buffer, color_picker_hsv.h);

    lv_obj_t * color_picker_thumb = lv_obj_create(color_picker, NULL);
    lv_obj_set_size(color_picker_thumb, COLOR_PICKER_HUE_W, COLOR_PICKER_HUE_W);
    lv_obj_set_style(color_picker_thumb, &style_thumb);
    lv_obj_set_pos(color_picker_thumb, (color_picker_hsv.s * (COLOR_PICKER_W / 100.0)) - lv_obj_get_width(color_picker_thumb) / 2, (COLOR_PICKER_H - color_picker_hsv.v * (COLOR_PICKER_H / 100.0)) - lv_obj_get_height(color_picker_thumb) / 2);

    static lv_color_t *color_picker_hue_buffer = NULL;
    if (color_picker_hue_buffer == NULL) {
        color_picker_hue_buffer = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(COLOR_PICKER_HUE_W, COLOR_PICKER_HUE_H), MALLOC_CAP_SPIRAM);
    }

    lv_obj_t * color_picker_hue = lv_canvas_create(h1, NULL);
    lv_canvas_set_buffer(color_picker_hue, color_picker_hue_buffer, COLOR_PICKER_HUE_W, COLOR_PICKER_HUE_H, LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_click(color_picker_hue, true);
    lv_obj_set_protect(color_picker_hue, LV_PROTECT_PRESS_LOST);
    lv_obj_set_event_cb(color_picker_hue, color_picker_event_cb);
    color_picker_hue_copy(color_picker_hue_buffer);
    
    lv_obj_t * color_picker_hue_thumb = lv_obj_create(color_picker_hue, NULL);
    lv_obj_set_size(color_picker_hue_thumb, COLOR_PICKER_HUE_W, COLOR_PICKER_HUE_W);
    lv_obj_set_style(color_picker_hue_thumb, &style_thumb);
    lv_obj_set_y(color_picker_hue_thumb, (COLOR_PICKER_HUE_H - color_picker_hsv.h * (COLOR_PICKER_HUE_H / 360.0)) - lv_obj_get_height(color_picker_hue_thumb) / 2);

    lv_obj_t * h2 = lv_cont_create(h1, NULL);
    lv_cont_set_fit(h2, LV_FIT_TIGHT);
    lv_cont_set_layout(h2, LV_LAYOUT_COL_M);

    static lv_style_t led_style; 
    lv_style_copy(&led_style, &lv_style_scr);
    led_style.body.radius = LV_RADIUS_CIRCLE;
    led_style.body.main_color = lv_color_hsv_to_rgb(color_picker_hsv.h, color_picker_hsv.s, color_picker_hsv.v);
    led_style.body.border.width = 5;
    led_style.body.border.opa = LV_OPA_30;
    led_style.body.shadow.width = 8;

    color_picker_led  = lv_led_create(h2, NULL);
    lv_obj_set_style(color_picker_led, &led_style);
    lv_obj_set_click(color_picker_led, true);
    lv_obj_set_event_cb(color_picker_led, led_event_cb);
    lv_obj_set_size(color_picker_led, LV_DPI, LV_DPI);
    
    led_rgb.r = led_style.body.main_color.ch.red << 3;
    led_rgb.g = led_style.body.main_color.ch.green << 2;
    led_rgb.b = led_style.body.main_color.ch.blue << 3;
    if (led_state) {
        lv_led_on(color_picker_led);
        WS2812B_setLeds(&led_rgb, 1);
    } else {
        lv_led_off(color_picker_led);
    }

    color_picker_table = lv_table_create(h2, NULL);
    lv_obj_align(color_picker_table, color_picker_led, LV_ALIGN_OUT_LEFT_TOP, 0, LV_DPI / 4);

    lv_table_set_col_cnt(color_picker_table, 4);
    lv_table_set_row_cnt(color_picker_table, 4);
    lv_table_set_col_width(color_picker_table, 0, LV_DPI / 3);
    lv_table_set_col_width(color_picker_table, 1, LV_DPI / 2);
    lv_table_set_col_width(color_picker_table, 2, LV_DPI / 3);
    lv_table_set_col_width(color_picker_table, 3, LV_DPI / 2);
    lv_table_set_cell_merge_right(color_picker_table, 3, 0, true);
    lv_table_set_cell_merge_right(color_picker_table, 3, 1, true);
    lv_table_set_cell_merge_right(color_picker_table, 3, 2, true);

    lv_table_set_cell_value(color_picker_table, 0, 0, "R:");
    lv_table_set_cell_value(color_picker_table, 1, 0, "G:");
    lv_table_set_cell_value(color_picker_table, 2, 0, "B:");

    lv_table_set_cell_value(color_picker_table, 0, 2, "H:");
    lv_table_set_cell_value(color_picker_table, 1, 2, "S:");
    lv_table_set_cell_value(color_picker_table, 2, 2, "V:");
    color_picker_table_update(color_picker_hsv);
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

int sockfd = -1;
struct sockaddr_in des_addr;
char sendline[128];

static void event_handler(lv_obj_t * obj, lv_event_t event)
{
    lv_obj_t *label;
    label = lv_obj_get_child(obj, NULL);
    switch (event) {
        case LV_EVENT_PRESSED: {
            sprintf(sendline, "{\"audio\": \"%s\"}", lv_label_get_text(label));
        }
        break;

        case LV_EVENT_RELEASED: {
            sprintf(sendline, "{\"audio\": \"%s\"}", "!");
        }
        break;

        default: {
            return;
        }
        break;
    }

    if (sockfd) {
        printf(sendline);
        sendto(sockfd, sendline, strlen(sendline), 0, (struct sockaddr*)&des_addr, sizeof(des_addr));
    }
}
                                  
static void body_page_audio(lv_obj_t * parent)
{
    lv_obj_t * h1 = lv_cont_create(parent, NULL);
    lv_cont_set_fit(h1, LV_FIT_TIGHT);

    lv_obj_t * audio_btn[12];
    lv_obj_t * label;

    audio_btn[0] = lv_btn_create(h1, NULL);
    lv_theme_t * th = lv_theme_get_current();
    lv_btn_set_style(audio_btn[0], LV_BTN_STYLE_REL, th->style.btnm.btn.rel);
    lv_btn_set_style(audio_btn[0], LV_BTN_STYLE_PR, th->style.btnm.btn.pr);
    lv_btn_set_style(audio_btn[0], LV_BTN_STYLE_TGL_REL, th->style.btnm.btn.tgl_rel);
    lv_btn_set_style(audio_btn[0], LV_BTN_STYLE_TGL_PR, th->style.btnm.btn.tgl_pr);
    lv_btn_set_style(audio_btn[0], LV_BTN_STYLE_INA, th->style.btnm.btn.ina);

    lv_obj_set_event_cb(audio_btn[0], event_handler);
    lv_btn_set_layout(audio_btn[0], LV_LAYOUT_OFF);
    lv_obj_set_size(audio_btn[0], 80, 300);
    label = lv_label_create(audio_btn[0], NULL);
    lv_label_set_text(label, "C");
    lv_obj_align(label, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -10);

    audio_btn[1] = lv_btn_create(h1, audio_btn[0]);
    lv_obj_align(audio_btn[1], audio_btn[0], LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
    label = lv_label_create(audio_btn[1], label);
    lv_label_set_text(label, "D");

    audio_btn[2] = lv_btn_create(h1, audio_btn[0]);
    lv_obj_align(audio_btn[2], audio_btn[1], LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
    label = lv_label_create(audio_btn[2], label);
    lv_label_set_text(label, "E");

    audio_btn[3] = lv_btn_create(h1, audio_btn[0]);
    lv_obj_align(audio_btn[3], audio_btn[2], LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
    label = lv_label_create(audio_btn[3], label);
    lv_label_set_text(label, "F");

    audio_btn[4] = lv_btn_create(h1, audio_btn[0]);
    lv_obj_align(audio_btn[4], audio_btn[3], LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
    label = lv_label_create(audio_btn[4], label);
    lv_label_set_text(label, "G");

    audio_btn[5] = lv_btn_create(h1, audio_btn[0]);
    lv_obj_align(audio_btn[5], audio_btn[4], LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
    label = lv_label_create(audio_btn[5], label);
    lv_label_set_text(label, "A");

    audio_btn[6] = lv_btn_create(h1, audio_btn[0]);
    lv_obj_align(audio_btn[6], audio_btn[5], LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
    label = lv_label_create(audio_btn[6], label);
    lv_label_set_text(label, "B");

    static lv_style_t style_rel;

    audio_btn[7] = lv_btn_create(h1, audio_btn[0]);
    lv_style_copy(&style_rel, th->style.btnm.bg);
    style_rel.body.main_color = LV_COLOR_BLACK;
    style_rel.body.grad_color = LV_COLOR_BLACK;
    lv_btn_set_style(audio_btn[7], LV_BTN_STYLE_REL, &style_rel);
    lv_obj_set_size(audio_btn[7], 60, 200);
    lv_obj_align(audio_btn[7], audio_btn[0], LV_ALIGN_OUT_RIGHT_TOP, - 60 / 2, 0);
    label = lv_label_create(audio_btn[7], NULL);
    lv_label_set_text(label, "#C");
    lv_obj_align(label, audio_btn[7], LV_ALIGN_IN_BOTTOM_MID, 0, -10);

    audio_btn[8] = lv_btn_create(h1, audio_btn[7]);
    lv_obj_align(audio_btn[8], audio_btn[1], LV_ALIGN_OUT_RIGHT_TOP, - 60 / 2, 0);
    label = lv_label_create(audio_btn[8], label);
    lv_label_set_text(label, "#D");

    audio_btn[9] = lv_btn_create(h1, audio_btn[7]);
    lv_obj_align(audio_btn[9], audio_btn[3], LV_ALIGN_OUT_RIGHT_TOP, - 60 / 2, 0);
    label = lv_label_create(audio_btn[9], label);
    lv_label_set_text(label, "#F");

    audio_btn[10] = lv_btn_create(h1, audio_btn[7]);
    lv_obj_align(audio_btn[10], audio_btn[4], LV_ALIGN_OUT_RIGHT_TOP, - 60 / 2, 0);
    label = lv_label_create(audio_btn[10], label);
    lv_label_set_text(label, "#G");

    audio_btn[11] = lv_btn_create(h1, audio_btn[7]);
    lv_obj_align(audio_btn[11], audio_btn[5], LV_ALIGN_OUT_RIGHT_TOP, - 60 / 2, 0);
    label = lv_label_create(audio_btn[11], label);
    lv_label_set_text(label, "#A");

    if (sockfd == -1) {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        des_addr.sin_family = AF_INET;
        des_addr.sin_addr.s_addr = inet_addr("192.168.0.255");
        des_addr.sin_port = htons(9999);
    }
}

static void body_page_info(lv_obj_t * parent)
{
    // lv_obj_t * txt = lv_label_create(parent, NULL);
    // lv_label_set_text(txt, "espressif");
    LV_IMG_DECLARE(espressif);
    lv_obj_t * esp_img = lv_img_create(parent, NULL);
    lv_img_set_src(esp_img, &espressif);

    // /*Create a Page*/
    // lv_obj_t * page = lv_page_create(parent, NULL);
    // lv_obj_set_size(page, lv_disp_get_hor_res(NULL) / 3, lv_disp_get_ver_res(NULL) / 2);
    // lv_obj_set_top(page, true);
    // lv_obj_align(page, esp_img, LV_ALIGN_IN_TOP_RIGHT,  LV_DPI, LV_DPI);

    static lv_style_t style;
    lv_style_copy(&style, &lv_style_scr);
    style.text.font = &lv_font_roboto_22;

    lv_obj_t *label = lv_label_create(parent, NULL);
    lv_label_set_style(label, LV_LABEL_STYLE_MAIN, &style);
    lv_label_set_text(label, "Espressif harnesses technology in  \n"
                      "order to create a better world with  \n"
                      "less waste and pollution. By  \n"
                      "leveraging wireless computing, we  \n"
                      "create carefully designed products  \n"
                      "that are more intelligent, adaptable  \n"
                      "and versatile. \n");
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
                    case GUI_PAGE_LED: {
                        lv_label_set_text(state, MY_LED_SYMBOL);
                        lv_obj_set_style(state, &style_my_symbol);
                        body_page_led(body);
                    }
                    break;

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

                    // case GUI_PAGE_TOUCH: {
                    //     lv_label_set_text(state, MY_TOUCH_SYMBOL);
                    //     lv_obj_set_style(state, &style_my_symbol);
                    //     body_page_touch(body);
                    // }
                    // break;

                    case GUI_PAGE_AUDIO: {
                        lv_label_set_text(state, LV_SYMBOL_AUDIO);
                        lv_obj_set_style(state, &lv_style_scr);
                        body_page_audio(body);
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
    lv_obj_t * list = lv_list_create(lv_scr_act(), NULL);

    lv_obj_t * label;
    lv_obj_t * list_btn;

    list_btn = lv_list_add_btn(list, MY_LED_SYMBOL, "LED");
    list_btn->user_data = GUI_PAGE_LED;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_my_symbol);

    list_btn = lv_list_add_btn(list, MY_TEMP_SYMBOL, "Monitor");
    list_btn->user_data = GUI_PAGE_MONITOR;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_my_symbol);

    list_btn = lv_list_add_btn(list, MY_MOTION_SYMBOL, "Motion");
    list_btn->user_data = GUI_PAGE_MOTION;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_my_symbol);

    // list_btn = lv_list_add_btn(list, MY_TOUCH_SYMBOL, "Touch");
    // list_btn->user_data = GUI_PAGE_TOUCH;
    // lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    // label = lv_list_get_btn_img(list_btn);
    // lv_obj_set_style(label, &style_my_symbol);

    list_btn = lv_list_add_btn(list, LV_SYMBOL_AUDIO, "Audio");
    list_btn->user_data = GUI_PAGE_AUDIO;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);

    list_btn = lv_list_add_btn(list, MY_CAMERA_SYMBOL, "Camera");
    list_btn->user_data = GUI_PAGE_CAMERA;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_my_symbol);

    list_btn = lv_list_add_btn(list, LV_SYMBOL_KEYBOARD, "Terminal");
    list_btn->user_data = GUI_PAGE_TERMINAL;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);

    list_btn = lv_list_add_btn(list, MY_ESPRESSIF_SYMBOL, "Info");
    list_btn->user_data = GUI_PAGE_INFO;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_my_symbol);

    lv_obj_set_pos(list, LV_HOR_RES - (LV_HOR_RES / 4) * 1, lv_obj_get_height(header));

    lv_obj_t * h = lv_cont_create(lv_scr_act(), NULL);
    lv_cont_set_layout(h, LV_LAYOUT_CENTER);

    LV_IMG_DECLARE(logo);
    lv_obj_t * img = lv_img_create(h, NULL);
    lv_img_set_src(img, &logo);

    lv_obj_t * txt = lv_label_create(h, NULL);
    lv_label_set_text(txt, "ESP32-S2Beta-HMI");

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
    body_page_led(body);
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
    lv_disp_load_scr(scr);
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
