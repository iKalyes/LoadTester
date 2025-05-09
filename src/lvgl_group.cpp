#include <lvgl_group.h>

lv_group_t* group;

void lvgl_group_init()
{
    group = lv_group_create();
    lv_group_set_default(group);
    lv_indev_set_group(get_encoder_indev(), group);
    lv_group_add_obj(group, ui_RUNSTOP);
    lv_group_add_obj(group, ui_ContMode);
    lv_group_add_obj(group, ui_LCRMode);
    lv_group_add_obj(group, ui_LoadMode);
    lv_group_add_obj(group, ui_LineMode);
    lv_group_add_obj(group, ui_SinductorCalib);
}

