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

void time_write(void *arg)
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
    
    lv_label_set_text((lv_obj_t *)(arg), strftime_buf);
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
    time_write(clock);
    lv_task_create(time_write, 1000, LV_TASK_PRIO_MID, (void *)(clock));

    lv_obj_t * state = lv_label_create(header, NULL);
    lv_label_set_text(state, SYMBOL_SETTINGS);
    lv_obj_align(state, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t * sym = lv_label_create(header, NULL);
    lv_label_set_text(sym, SYMBOL_WIFI SYMBOL_BATTERY_FULL);
    lv_obj_align(sym, NULL, LV_ALIGN_IN_RIGHT_MID, -LV_DPI/10, 0);

    lv_cont_set_fit(header, false, true);   /*Let the height set automatically*/
    lv_obj_set_pos(header, 0, 0);
}

static void side_create(void)
{
    lv_obj_t * h = lv_cont_create(lv_scr_act(), NULL);


    lv_obj_t * list = lv_list_create(h, NULL);
    
    // lv_obj_t * label = lv_label_create(list, NULL);
    // char str[16] = {0xf4,0x91,'\0'};
    // lv_label_set_text(label, _SYMBOL_VALUE3(EF,A0,80));
    // // lv_obj_set_style(label, &style_txt);
    // printf("font: %s\n", _SYMBOL_VALUE3(EF,A0,80));

    lv_obj_t * list_btn;
    list_btn = lv_list_add(list, SYMBOL_GPS,  "GPS",  NULL);
    // lv_obj_set_size(list, LV_HOR_RES / 4, LV_VER_RES / 2);
    // lv_btn_set_toggle(list_btn, true);
    lv_list_add(list, SYMBOL_WIFI, "WiFi", NULL);
    lv_list_add(list, SYMBOL_GPS, "GPS", NULL);
    lv_list_add(list, SYMBOL_AUDIO, "Audio", NULL);
    lv_list_add(list, SYMBOL_VIDEO, "Video", NULL);
    lv_list_add(list, SYMBOL_CALL, "Call", NULL);
    lv_list_add(list, SYMBOL_BELL, "Bell", NULL);
    lv_list_add(list, SYMBOL_FILE, "File", NULL);
    lv_list_add(list, SYMBOL_EDIT, "Edit", NULL);
    lv_list_add(list, SYMBOL_CUT,  "Cut",  NULL);
    lv_list_add(list, SYMBOL_COPY, "Copy", NULL);

    lv_cont_set_layout(h, LV_LAYOUT_CENTER);
    lv_obj_set_pos(h, LV_HOR_RES - (LV_HOR_RES / 4) * 1, lv_obj_get_height(header));
    lv_obj_set_width(h, (LV_HOR_RES / 4) * 1);
    lv_obj_set_height(h, LV_VER_RES - lv_obj_get_height(header));
}

static void body_create(void)
{
    body = lv_cont_create(lv_scr_act(), NULL);

    static lv_style_t style_txt;
    lv_style_copy(&style_txt, &lv_style_plain);
    LV_FONT_DECLARE(seg_font);
    style_txt.text.font = &seg_font;
    style_txt.text.opa = LV_OPA_100;

    lv_obj_t *txt;
    txt = lv_label_create(body, NULL);
    lv_obj_set_style(txt, &style_txt);
    lv_label_set_recolor(txt, true);
    source[0].obj = txt;

    txt = lv_label_create(body, NULL);
    lv_obj_set_style(txt, &style_txt);
    lv_label_set_recolor(txt, true);
    source[1].obj = txt;

    txt = lv_label_create(body, NULL);
    lv_obj_set_style(txt, &style_txt);
    lv_label_set_recolor(txt, true);
    source[2].obj = txt;

    lv_cont_set_layout(body, LV_LAYOUT_CENTER);
    lv_obj_set_pos(body, 0, lv_obj_get_height(header));
    lv_obj_set_width(body, (LV_HOR_RES / 4) * 3);
    lv_obj_set_height(body, LV_VER_RES - lv_obj_get_height(header));
}

void gui_init(lv_theme_t * th)
{
    lv_theme_set_current(th);
    th = lv_theme_get_current();    /*If `LV_THEME_LIVE_UPDATE  1` `th` is not used directly so get the real theme after set*/
    lv_obj_t * scr = lv_cont_create(NULL, NULL);
    lv_scr_load(scr);
    lv_cont_set_style(scr, th->bg);

    header_create();
    side_create();
    body_create();
    source[0].val = 5.26;
    source[0].mask = 1;
    source_write(source[0]);
    source[1].val = 0.13;
    source[1].mask = 2;
    source_write(source[1]);
    source[2].val = source[0].val * source[1].val;
    source[2].mask = 4;
    source_write(source[2]);
}
