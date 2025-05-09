// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: LoadTester

#include "../ui.h"

void ui_Main_screen_init(void)
{
ui_Main = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Main, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_MeasurementDock = lv_obj_create(ui_Main);
lv_obj_remove_style_all(ui_MeasurementDock);
lv_obj_set_width( ui_MeasurementDock, 316);
lv_obj_set_height( ui_MeasurementDock, 174);
lv_obj_set_x( ui_MeasurementDock, 0 );
lv_obj_set_y( ui_MeasurementDock, 30 );
lv_obj_set_align( ui_MeasurementDock, LV_ALIGN_TOP_MID );
lv_obj_clear_flag( ui_MeasurementDock, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_MeasurementDock, 8, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_MeasurementDock, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_MeasurementDock, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_MeasurementDock, lv_color_hex(0x939CA3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_MeasurementDock, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_MeasurementDock, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_MeasurementDock, LV_BORDER_SIDE_FULL, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_MeasurementCondition = lv_label_create(ui_MeasurementDock);
lv_obj_set_width( ui_MeasurementCondition, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_MeasurementCondition, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_MeasurementCondition, 2 );
lv_obj_set_y( ui_MeasurementCondition, -56 );
lv_obj_set_align( ui_MeasurementCondition, LV_ALIGN_BOTTOM_LEFT );
lv_label_set_text(ui_MeasurementCondition,"Source ~ 1000mV");
lv_obj_set_style_text_font(ui_MeasurementCondition, &ui_font_ASCII18, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_MeasurementVoltage = lv_label_create(ui_MeasurementDock);
lv_obj_set_width( ui_MeasurementVoltage, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_MeasurementVoltage, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_MeasurementVoltage, 2 );
lv_obj_set_y( ui_MeasurementVoltage, -74 );
lv_obj_set_align( ui_MeasurementVoltage, LV_ALIGN_BOTTOM_LEFT );
lv_label_set_text(ui_MeasurementVoltage,"V-DC:0.000000V");
lv_obj_set_style_text_color(ui_MeasurementVoltage, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_MeasurementVoltage, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_MeasurementVoltage, &ui_font_ASCII18, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_MeasurementValue = lv_label_create(ui_MeasurementDock);
lv_obj_set_width( ui_MeasurementValue, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_MeasurementValue, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_MeasurementValue, 0 );
lv_obj_set_y( ui_MeasurementValue, -42 );
lv_obj_set_align( ui_MeasurementValue, LV_ALIGN_CENTER );
lv_label_set_text(ui_MeasurementValue,"OPEN");
lv_obj_set_style_text_color(ui_MeasurementValue, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_MeasurementValue, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_MeasurementValue, &ui_font_ASCII72, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_MeasurementUNIT = lv_label_create(ui_MeasurementDock);
lv_obj_set_width( ui_MeasurementUNIT, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_MeasurementUNIT, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_MeasurementUNIT, -2 );
lv_obj_set_y( ui_MeasurementUNIT, -60 );
lv_obj_set_align( ui_MeasurementUNIT, LV_ALIGN_BOTTOM_RIGHT );
lv_label_set_text(ui_MeasurementUNIT,"  ");
lv_obj_set_style_text_color(ui_MeasurementUNIT, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_MeasurementUNIT, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_MeasurementUNIT, &ui_font_ASCII48, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_VoltageChart = lv_chart_create(ui_MeasurementDock);
lv_obj_set_width( ui_VoltageChart, 249);
lv_obj_set_height( ui_VoltageChart, 56);
lv_obj_set_x( ui_VoltageChart, 2 );
lv_obj_set_y( ui_VoltageChart, -2 );
lv_obj_set_align( ui_VoltageChart, LV_ALIGN_BOTTOM_LEFT );
lv_chart_set_type( ui_VoltageChart, LV_CHART_TYPE_LINE);
lv_chart_set_point_count( ui_VoltageChart, 64);
lv_chart_set_range( ui_VoltageChart, LV_CHART_AXIS_PRIMARY_Y, 0, 2048000);
lv_chart_set_range( ui_VoltageChart, LV_CHART_AXIS_SECONDARY_Y, 0, 0);
lv_chart_set_div_line_count( ui_VoltageChart, 3, 0);
lv_chart_set_axis_tick( ui_VoltageChart, LV_CHART_AXIS_PRIMARY_X, 0, 0, 0, 0, false, 0);
lv_chart_set_axis_tick( ui_VoltageChart, LV_CHART_AXIS_PRIMARY_Y, 0, 0, 0, 0, false, 0);
lv_chart_set_axis_tick( ui_VoltageChart, LV_CHART_AXIS_SECONDARY_Y, 0, 0, 0, 0, false, 50);
lv_obj_set_style_radius(ui_VoltageChart, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_VoltageChart, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_VoltageChart, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_VoltageChart, lv_color_hex(0x939CA3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_VoltageChart, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_VoltageChart, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_line_color(ui_VoltageChart, lv_color_hex(0x939CA3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_line_opa(ui_VoltageChart, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_line_color(ui_VoltageChart, lv_color_hex(0x000000), LV_PART_ITEMS | LV_STATE_DEFAULT );
lv_obj_set_style_line_opa(ui_VoltageChart, 255, LV_PART_ITEMS| LV_STATE_DEFAULT);
lv_obj_set_style_line_width(ui_VoltageChart, 1, LV_PART_ITEMS| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_VoltageChart, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_VoltageChart, 0, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_text_font(ui_VoltageChart, &lv_font_montserrat_8, LV_PART_TICKS| LV_STATE_DEFAULT);

ui_SinductorCalib = lv_spinbox_create(ui_MeasurementDock);
lv_obj_set_width( ui_SinductorCalib, 59);
lv_obj_set_height( ui_SinductorCalib, 56);
lv_obj_set_x( ui_SinductorCalib, 125 );
lv_obj_set_y( ui_SinductorCalib, 56 );
lv_obj_set_align( ui_SinductorCalib, LV_ALIGN_CENTER );
lv_spinbox_set_digit_format( ui_SinductorCalib, 3, 1);
lv_spinbox_set_range( ui_SinductorCalib, 0,500 );
lv_spinbox_set_cursor_pos(ui_SinductorCalib, 3 - 1);
lv_obj_set_style_radius(ui_SinductorCalib, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_SinductorCalib, lv_color_hex(0x939CA3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_SinductorCalib, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_SinductorCalib, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SinductorCalib, &ui_font_ASCII18, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_FunctionDocke = lv_obj_create(ui_Main);
lv_obj_remove_style_all(ui_FunctionDocke);
lv_obj_set_width( ui_FunctionDocke, 105);
lv_obj_set_height( ui_FunctionDocke, 40);
lv_obj_set_x( ui_FunctionDocke, -106 );
lv_obj_set_y( ui_FunctionDocke, 2 );
lv_obj_set_align( ui_FunctionDocke, LV_ALIGN_TOP_MID );
lv_obj_clear_flag( ui_FunctionDocke, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_FunctionDocke, 8, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_FunctionDocke, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_FunctionDocke, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_FunctionDocke, lv_color_hex(0x939CA3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_FunctionDocke, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_FunctionDocke, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_FunctionDocke, LV_BORDER_SIDE_FULL, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Function = lv_label_create(ui_FunctionDocke);
lv_obj_set_width( ui_Function, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Function, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Function, 0 );
lv_obj_set_y( ui_Function, 3 );
lv_obj_set_align( ui_Function, LV_ALIGN_TOP_MID );
lv_label_set_text(ui_Function,"通断测试");
lv_obj_set_style_text_color(ui_Function, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Function, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Function, &ui_font_CHN24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ColorFix = lv_obj_create(ui_Main);
lv_obj_remove_style_all(ui_ColorFix);
lv_obj_set_width( ui_ColorFix, 105);
lv_obj_set_height( ui_ColorFix, 11);
lv_obj_set_x( ui_ColorFix, 2 );
lv_obj_set_y( ui_ColorFix, 31 );
lv_obj_clear_flag( ui_ColorFix, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_ColorFix, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_ColorFix, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ColorFix, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_ColorFix, lv_color_hex(0x939CA3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_ColorFix, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_ColorFix, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_ColorFix, LV_BORDER_SIDE_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ContMode = lv_btn_create(ui_Main);
lv_obj_set_width( ui_ContMode, 76);
lv_obj_set_height( ui_ContMode, 32);
lv_obj_set_x( ui_ContMode, -120 );
lv_obj_set_y( ui_ContMode, -2 );
lv_obj_set_align( ui_ContMode, LV_ALIGN_BOTTOM_MID );
lv_obj_add_flag( ui_ContMode, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_ContMode, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_ContMode, lv_color_hex(0x0000FF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ContMode, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_ContMode, lv_color_hex(0x939CA3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_ContMode, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_ContMode, 1, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_TextContMode = lv_label_create(ui_ContMode);
lv_obj_set_width( ui_TextContMode, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_TextContMode, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_TextContMode, LV_ALIGN_CENTER );
lv_label_set_text(ui_TextContMode,"通断测试");
lv_obj_set_style_text_color(ui_TextContMode, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_TextContMode, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_TextContMode, &ui_font_CHN18, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LCRMode = lv_btn_create(ui_Main);
lv_obj_set_width( ui_LCRMode, 76);
lv_obj_set_height( ui_LCRMode, 32);
lv_obj_set_x( ui_LCRMode, -40 );
lv_obj_set_y( ui_LCRMode, -2 );
lv_obj_set_align( ui_LCRMode, LV_ALIGN_BOTTOM_MID );
lv_obj_add_flag( ui_LCRMode, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_LCRMode, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_LCRMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_LCRMode, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_LCRMode, lv_color_hex(0x939CA3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_LCRMode, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_LCRMode, 1, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_TextLCRMode = lv_label_create(ui_LCRMode);
lv_obj_set_width( ui_TextLCRMode, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_TextLCRMode, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_TextLCRMode, LV_ALIGN_CENTER );
lv_label_set_text(ui_TextLCRMode,"元件测量");
lv_obj_set_style_text_color(ui_TextLCRMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_TextLCRMode, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_TextLCRMode, &ui_font_CHN18, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LoadMode = lv_btn_create(ui_Main);
lv_obj_set_width( ui_LoadMode, 76);
lv_obj_set_height( ui_LoadMode, 32);
lv_obj_set_x( ui_LoadMode, 40 );
lv_obj_set_y( ui_LoadMode, -2 );
lv_obj_set_align( ui_LoadMode, LV_ALIGN_BOTTOM_MID );
lv_obj_add_flag( ui_LoadMode, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_LoadMode, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_LoadMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_LoadMode, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_LoadMode, lv_color_hex(0x939CA3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_LoadMode, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_LoadMode, 1, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_TextLoadMode = lv_label_create(ui_LoadMode);
lv_obj_set_width( ui_TextLoadMode, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_TextLoadMode, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_TextLoadMode, LV_ALIGN_CENTER );
lv_label_set_text(ui_TextLoadMode,"负载网络");
lv_obj_set_style_text_color(ui_TextLoadMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_TextLoadMode, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_TextLoadMode, &ui_font_CHN18, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LineMode = lv_btn_create(ui_Main);
lv_obj_set_width( ui_LineMode, 76);
lv_obj_set_height( ui_LineMode, 32);
lv_obj_set_x( ui_LineMode, 120 );
lv_obj_set_y( ui_LineMode, -2 );
lv_obj_set_align( ui_LineMode, LV_ALIGN_BOTTOM_MID );
lv_obj_add_flag( ui_LineMode, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_LineMode, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_LineMode, lv_color_hex(0x31526B), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_LineMode, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_LineMode, lv_color_hex(0x939CA3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_LineMode, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_LineMode, 1, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_TextLineMode = lv_label_create(ui_LineMode);
lv_obj_set_width( ui_TextLineMode, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_TextLineMode, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_TextLineMode, LV_ALIGN_CENTER );
lv_label_set_text(ui_TextLineMode,"故障距离");
lv_obj_set_style_text_color(ui_TextLineMode, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_TextLineMode, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_TextLineMode, &ui_font_CHN18, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ImageSerial = lv_img_create(ui_Main);
lv_img_set_src(ui_ImageSerial, &ui_img_281677161);
lv_obj_set_width( ui_ImageSerial, LV_SIZE_CONTENT);  /// 32
lv_obj_set_height( ui_ImageSerial, LV_SIZE_CONTENT);   /// 32
lv_obj_set_x( ui_ImageSerial, -22 );
lv_obj_set_y( ui_ImageSerial, -105 );
lv_obj_set_align( ui_ImageSerial, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_ImageSerial, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_ImageSerial, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_SerialStatus = lv_label_create(ui_Main);
lv_obj_set_width( ui_SerialStatus, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SerialStatus, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SerialStatus, 155 );
lv_obj_set_y( ui_SerialStatus, -103 );
lv_obj_set_align( ui_SerialStatus, LV_ALIGN_LEFT_MID );
lv_label_set_text(ui_SerialStatus,"未连接");
lv_obj_set_style_text_color(ui_SerialStatus, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_SerialStatus, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SerialStatus, &ui_font_CHN24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_RUNSTOP = lv_btn_create(ui_Main);
lv_obj_set_width( ui_RUNSTOP, 76);
lv_obj_set_height( ui_RUNSTOP, 27);
lv_obj_set_x( ui_RUNSTOP, 120 );
lv_obj_set_y( ui_RUNSTOP, -212 );
lv_obj_set_align( ui_RUNSTOP, LV_ALIGN_BOTTOM_MID );
lv_obj_add_flag( ui_RUNSTOP, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_RUNSTOP, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_RUNSTOP, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_RUNSTOP, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_RUNSTOP, lv_color_hex(0x939CA3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_RUNSTOP, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_RUNSTOP, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_radius(ui_RUNSTOP, 5, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_bg_color(ui_RUNSTOP, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_RUNSTOP, 255, LV_PART_MAIN| LV_STATE_CHECKED);

ui_TextIRUNSTOP = lv_label_create(ui_RUNSTOP);
lv_obj_set_width( ui_TextIRUNSTOP, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_TextIRUNSTOP, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_TextIRUNSTOP, 0 );
lv_obj_set_y( ui_TextIRUNSTOP, 2 );
lv_obj_set_align( ui_TextIRUNSTOP, LV_ALIGN_CENTER );
lv_label_set_text(ui_TextIRUNSTOP,"STOP");
lv_obj_set_style_text_color(ui_TextIRUNSTOP, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_TextIRUNSTOP, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_TextIRUNSTOP, &ui_font_CHN24, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_SinductorCalib, ui_event_SinductorCalib, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_ContMode, ui_event_ContMode, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_LCRMode, ui_event_LCRMode, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_LoadMode, ui_event_LoadMode, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_LineMode, ui_event_LineMode, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_RUNSTOP, ui_event_RUNSTOP, LV_EVENT_ALL, NULL);

}
