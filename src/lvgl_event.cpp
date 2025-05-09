#include <lvgl_event.h>

void ui_event_ContMode( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      ContMode( e );
      lv_obj_set_style_bg_color(ui_ContMode, lv_color_hex(0x0000FF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextContMode, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_bg_color(ui_LCRMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextLCRMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_bg_color(ui_LoadMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextLoadMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_bg_color(ui_LineMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextLineMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_label_set_text(ui_MeasurementValue, "OPEN");

      lv_label_set_text(ui_Function, "通断测试");
      ContMode();
      MeasureMode = 0;
}
}

void ui_event_LCRMode( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      LCRMode( e );
      lv_obj_set_style_bg_color(ui_ContMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextContMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_bg_color(ui_LCRMode, lv_color_hex(0x0000FF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextLCRMode, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_bg_color(ui_LoadMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextLoadMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_bg_color(ui_LineMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextLineMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_label_set_text(ui_MeasurementValue, "OPEN");

      lv_label_set_text(ui_Function, "元件测量");
      LCRMode();
      MeasureMode = 1;
}
}

void ui_event_LoadMode( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      LoadMode( e );
      lv_obj_set_style_bg_color(ui_ContMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextContMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_bg_color(ui_LCRMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextLCRMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_bg_color(ui_LoadMode, lv_color_hex(0x0000FF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextLoadMode, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_bg_color(ui_LineMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextLineMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_label_set_text(ui_MeasurementValue, "OPEN");

      lv_label_set_text(ui_Function, "负载网络");
      LoadMode();
      MeasureMode = 2;
}
}

void ui_event_LineMode( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      LineMode( e );
      lv_obj_set_style_bg_color(ui_ContMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextContMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_bg_color(ui_LCRMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextLCRMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_bg_color(ui_LoadMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextLoadMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_bg_color(ui_LineMode, lv_color_hex(0x0000FF), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_obj_set_style_text_color(ui_TextLineMode, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_label_set_text(ui_MeasurementValue, "OPEN");

      lv_label_set_text(ui_Function, "故障距离");
      LineMode();
      MeasureMode = 3;
}
}

void ui_event_RUNSTOP( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      _ui_label_set_property(ui_TextIRUNSTOP, _UI_LABEL_PROPERTY_TEXT, "RUN");
      RUNSTOP = false;
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      _ui_label_set_property(ui_TextIRUNSTOP, _UI_LABEL_PROPERTY_TEXT, "STOP");
      RUNSTOP = true;
}
}


