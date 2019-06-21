#include <stdio.h>
#include <time.h>
#include "gui.h"

static lv_obj_t * header;
static lv_obj_t * body;

typedef struct {
    lv_obj_t *obj;
    float val;
    uint8_t mask;
} source_t;

source_t source[3] = {0};
#define SOURCE_MASK_COLOR "ff3034"

void time_write(lv_task_t * arg)
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
    
    lv_label_set_text((lv_obj_t *)(arg->user_data), strftime_buf);
}

void source_write(source_t source)
{
    char data[6], str[64];

    if (source.val < 10) {
        sprintf(data, "%05.3f", source.val);
    } else if (source.val < 100 && source.val > 10) {
        sprintf(data, "%05.2f", source.val);
    } else {
        sprintf(data, "E R R");
    }

    sprintf(str, "#%s %c##%s %c##%s %c##%s %c##%s %c#", \
                 (source.mask / 16) % 2 ? SOURCE_MASK_COLOR : "000000", data[0], \
                 (source.mask / 8)  % 2 ? SOURCE_MASK_COLOR : "000000", data[1], \
                 (source.mask / 4)  % 2 ? SOURCE_MASK_COLOR : "000000", data[2], \
                 (source.mask / 2)  % 2 ? SOURCE_MASK_COLOR : "000000", data[3], \
                 (source.mask / 1)  % 2 ? SOURCE_MASK_COLOR : "000000", data[4]  );
    
    lv_label_set_text(source.obj, str);
}

static void header_create(void)
{
    header = lv_cont_create(lv_scr_act(), NULL);
    lv_obj_set_width(header, LV_HOR_RES);

    lv_obj_t * clock = lv_label_create(header, NULL);
    lv_obj_align(clock, NULL, LV_ALIGN_IN_LEFT_MID, LV_DPI/10, 0);
    lv_label_set_recolor(clock, true);
    // time_write(clock);
    lv_task_create(time_write, 1000, LV_TASK_PRIO_MID, (void *)(clock));

    lv_obj_t * state = lv_label_create(header, NULL);
    lv_label_set_text(state, LV_SYMBOL_SETTINGS);
    lv_obj_align(state, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t * sym = lv_label_create(header, NULL);
    lv_label_set_text(sym, LV_SYMBOL_WIFI" "LV_SYMBOL_BATTERY_FULL);
    lv_obj_align(sym, NULL, LV_ALIGN_IN_RIGHT_MID, -LV_DPI/10, 0);

    lv_cont_set_fit2(header, false, true);   /*Let the height set automatically*/
    lv_obj_set_pos(header, 0, 0);
}

static void body_page_1(lv_obj_t * parent)
{
    /*Create a Calendar*/
    lv_obj_t * cal = lv_calendar_create(parent, NULL);
    lv_obj_set_size(cal, 5 * LV_DPI / 2, 5 * LV_DPI / 2);
    // lv_obj_align(cal, page, LV_ALIGN_OUT_RIGHT_TOP, -LV_DPI / 2, LV_DPI / 3);
    // lv_obj_set_top(cal, true);

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
}

lv_obj_t * spinbox = NULL;

static void body_page_2(lv_obj_t * parent)
{
    spinbox = lv_spinbox_create(parent, NULL);
    // lv_spinbox_set_style(spinbox, LV_SPINBOX_STYLE_BG, &spinBoxStyle);
    // lv_spinbox_set_style(spinbox, LV_SPINBOX_STYLE_CURSOR, &spinBoxCursorStyle);
    // lv_obj_set_size(spinbox, LV_HOR_RES, 80);
    // lv_obj_align(spinbox, NULL, LV_ALIGN_IN_TOP_LEFT, 4, 0);
    lv_spinbox_set_value(spinbox, 10);
    lv_spinbox_increment(spinbox);
}

static void body_page_3(lv_obj_t * parent)
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

#define MY_TEMP_SYMBOL "\xEF\x8B\x88"  // f2c8
#define MY_MOTION_SYMBOL "\xEF\x8B\x9B"  // f2db
#define MY_LED_SYMBOL "\xEF\x83\xAB"  // f0eb
#define MY_TOUCH_SYMBOL "\xEF\x82\xA6" // f0a6
#define MY_CAMERA_SYMBOL "\xEF\x85\xAD" // f16d
#define MY_ESPRESSIF_SYMBOL "\xEF\x8B\xA1" // f2e1



static void side_btn_event_callback(lv_obj_t * obj, lv_event_t event)
{
    static uint8_t page_num = 0;
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
            if (page_num != obj->user_data) {
                lv_page_clean(body);
                switch ((int)obj->user_data) {
                    case 0: {
                        body_page_1(body);
                    }
                    break;

                    case 1: {
                        body_page_2(body);
                    }
                    break;

                    case 2: {
                        body_page_3(body);
                    }
                    break;
                }
            }
            page_num = obj->user_data;
            break;
    }
}

static void side_create(void)
{
    lv_obj_t * h = lv_cont_create(lv_scr_act(), NULL);


    lv_obj_t * list = lv_list_create(h, NULL);
    LV_FONT_DECLARE(my_symbol);

    static lv_style_t style_symbol;
    lv_style_copy(&style_symbol, &lv_style_plain);
    style_symbol.text.font = &my_symbol;

    lv_obj_t * label;
    lv_obj_t * list_btn;
    list_btn = lv_list_add(list, MY_TEMP_SYMBOL, "Temp. & Hum.", NULL);
    list_btn->user_data = 0;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_symbol);
    list_btn = lv_list_add(list, MY_MOTION_SYMBOL, "Motion", NULL);
    list_btn->user_data = 1;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_symbol);
    list_btn = lv_list_add(list, MY_LED_SYMBOL, "RGB LED", NULL);
    list_btn->user_data = 2;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_symbol);
    list_btn = lv_list_add(list, MY_TOUCH_SYMBOL, "Touch", NULL);
    list_btn->user_data = 3;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_symbol);
    list_btn = lv_list_add(list, LV_SYMBOL_AUDIO, "Audio", NULL);
    list_btn->user_data = 4;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    // label = lv_list_get_btn_img(list_btn);
    // lv_obj_set_style(label, &style_symbol);
    list_btn = lv_list_add(list, MY_CAMERA_SYMBOL, "Camera", NULL);
    list_btn->user_data = 5;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_symbol);
    list_btn = lv_list_add(list, MY_ESPRESSIF_SYMBOL, "Info", NULL);
    list_btn->user_data = 6;
    lv_obj_set_event_cb(list_btn, side_btn_event_callback);
    label = lv_list_get_btn_img(list_btn);
    lv_obj_set_style(label, &style_symbol);

    lv_cont_set_layout(h, LV_LAYOUT_CENTER);
    lv_obj_set_pos(h, LV_HOR_RES - (LV_HOR_RES / 4) * 1, lv_obj_get_height(header));
    lv_obj_set_width(h, (LV_HOR_RES / 4) * 1);
    lv_obj_set_height(h, LV_VER_RES - lv_obj_get_height(header));
}

static void body_create(void)
{
    // body = lv_cont_create(lv_scr_act(), NULL);
    body = lv_page_create(lv_scr_act(), NULL);

    lv_page_set_sb_mode(body, LV_SB_MODE_DRAG);
    // static lv_style_t style_txt;
    // lv_style_copy(&style_txt, &lv_style_plain);
    // LV_FONT_DECLARE(seg_font);
    // style_txt.text.font = &seg_font;
    // // style_txt.text.opa = LV_OPA_100;

    // lv_obj_t *txt;
    // txt = lv_label_create(body, NULL);
    // lv_obj_set_style(txt, &style_txt);
    // lv_label_set_recolor(txt, true);
    // source[0].obj = txt;

    // txt = lv_label_create(body, NULL);
    // lv_obj_set_style(txt, &style_txt);
    // lv_label_set_recolor(txt, true);
    // source[1].obj = txt;

    // txt = lv_label_create(body, NULL);
    // lv_obj_set_style(txt, &style_txt);
    // lv_label_set_recolor(txt, true);
    // source[2].obj = txt;

    // lv_cont_set_layout(body, LV_LAYOUT_CENTER);
    lv_obj_set_pos(body, 0, lv_obj_get_height(header));
    lv_obj_set_width(body, (LV_HOR_RES / 4) * 3);
    lv_obj_set_height(body, LV_VER_RES - lv_obj_get_height(header));
    body_page_1(body);
}

void gui_init(lv_theme_t * th)
{
    lv_theme_set_current(th);
    th = lv_theme_get_current();    /*If `LV_THEME_LIVE_UPDATE  1` `th` is not used directly so get the real theme after set*/
    lv_obj_t * scr = lv_cont_create(NULL, NULL);
    lv_disp_set_scr_act(scr);
    lv_cont_set_style(scr, LV_CONT_STYLE_MAIN, th->style.bg);
    // lv_cont_set_style(scr, th->style.bg);

    header_create();
    side_create();
    body_create();
    // source[0].val = 5.26;
    // source[0].mask = 1;
    // source_write(source[0]);
    // source[1].val = 0.13;
    // source[1].mask = 2;
    // source_write(source[1]);
    // source[2].val = source[0].val * source[1].val;
    // source[2].mask = 4;
    // source_write(source[2]);
}
