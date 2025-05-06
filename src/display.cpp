#include <display.h>

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */
FT6336U touch_6336;
lv_chart_series_t * VoltageChart;

int voltage_full;
int voltage_int;
int voltage_frac;

int LineMode_Volt_full;
int LineMode_Volt_int;
int LineMode_Volt_frac;

float LineMode_Distance;

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp_drv );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data )
{

  if (touch_6336.available())
  {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touch_6336.touchPoint.tp[0].x;
    data->point.y = touch_6336.touchPoint.tp[0].y;
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }

}

void backlight_init()
{
    pinMode(6, OUTPUT);
    analogWriteFreq(5000);
    analogWriteRange(100);
    analogWrite(6, 100);
}

void display_init()
{
    lv_init();
    tft.begin();          /* TFT init */
    tft.setRotation( 3 ); /* Landscape orientation, flipped */

    /*Set the touchscreen calibration data,
     the actual data for your display can be acquired using
     the Generic -> Touch_calibrate example from the TFT_eSPI library*/
    touch_6336.begin(Wire);
    lv_disp_draw_buf_init( &draw_buf, buf_1, buf_2, screenWidth * screenHeight / 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );
  
    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    ui_init();
    backlight_init();
    style_reset();
}

void style_reset()
{
  static lv_style_t style_pr;
  lv_style_init(&style_pr);
  lv_style_set_transform_width(&style_pr, 0);
  lv_style_set_transform_height(&style_pr, 0);
  lv_obj_add_style(ui_ContMode, &style_pr, LV_STATE_PRESSED);
  lv_obj_add_style(ui_LCRMode, &style_pr, LV_STATE_PRESSED);
  lv_obj_add_style(ui_LoadMode, &style_pr, LV_STATE_PRESSED);
  lv_obj_add_style(ui_LineMode, &style_pr, LV_STATE_PRESSED);

  lv_obj_set_style_outline_color(ui_ContMode, lv_color_hex(0xFFFF00), LV_STATE_FOCUS_KEY);
  lv_obj_set_style_outline_width(ui_ContMode, 1, LV_STATE_FOCUS_KEY);
  lv_obj_set_style_outline_color(ui_LCRMode, lv_color_hex(0xFFFF00), LV_STATE_FOCUS_KEY);
  lv_obj_set_style_outline_width(ui_LCRMode, 1, LV_STATE_FOCUS_KEY);
  lv_obj_set_style_outline_color(ui_LoadMode, lv_color_hex(0xFFFF00), LV_STATE_FOCUS_KEY);
  lv_obj_set_style_outline_width(ui_LoadMode, 1, LV_STATE_FOCUS_KEY);
  lv_obj_set_style_outline_color(ui_LineMode, lv_color_hex(0xFFFF00), LV_STATE_FOCUS_KEY);
  lv_obj_set_style_outline_width(ui_LineMode, 1, LV_STATE_FOCUS_KEY);
}

void lvgl_task_handler()
{
    lv_task_handler();
}

void ADS122C04_DATA_CHART_INIT()
{
  VoltageChart = lv_chart_add_series(ui_VoltageChart, lv_color_hex(0X000000), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_update_mode(ui_VoltageChart, LV_CHART_UPDATE_MODE_SHIFT);
  lv_chart_set_point_count(ui_VoltageChart, 128);
}

void ADS122C04_DATA_REFRESH()
{
    switch (MeasureMode)
    {
    case 0:
    if(RUNSTOP == true)
    {
      voltage_full = round(AIN0_DC_Volt * 1000000);
      voltage_int = voltage_full / 1000000;
      voltage_frac = voltage_full % 1000000;
      lv_label_set_text_fmt(ui_MeasurementVoltage, "V-DC:%01d.%06dV", voltage_int, voltage_frac);
      lv_chart_set_next_value(ui_VoltageChart, VoltageChart, voltage_full);
      if(AIN0_DC_Volt < 0.1)
      {
        lv_label_set_text(ui_MeasurementValue, "SHORT");
      }
      else
      {
        lv_label_set_text(ui_MeasurementValue, "OPEN");
      }
    }
      break;
    case 1:
    if(RUNSTOP == true)
    {
      voltage_full = round(AIN1_AC_Volt * 1000000);
      voltage_int = voltage_full / 1000000;
      voltage_frac = voltage_full % 1000000;
      lv_label_set_text_fmt(ui_MeasurementVoltage, "V-AC:%01d.%06dV", voltage_int, voltage_frac);
      lv_chart_set_next_value(ui_VoltageChart, VoltageChart, voltage_full);
    }
      break;
    case 2:
    if(RUNSTOP == true)
    {
      voltage_full = round(AIN0_DC_Volt * 1000000);
      voltage_int = voltage_full / 1000000;
      voltage_frac = voltage_full % 1000000;
      lv_label_set_text_fmt(ui_MeasurementVoltage, "V-DC:%01d.%06dV", voltage_int, voltage_frac);
    }
      break;
    case 3:
    if(RUNSTOP == true)
    {
      voltage_full = round(AIN2_Line_Volt * 1000000);
      voltage_int = voltage_full / 1000000;
      voltage_frac = voltage_full % 1000000;
      lv_label_set_text_fmt(ui_MeasurementVoltage, "V-LINE:%01d.%06dV", voltage_int, voltage_frac);
      lv_chart_set_next_value(ui_VoltageChart, VoltageChart, voltage_full);

      LineMode_Distance = (LineMode_Volt_Avg - 0.5955) / 0.012;
      
      LineMode_Volt_full = round(LineMode_Distance * 100);
      LineMode_Volt_int = LineMode_Volt_full / 100;
      LineMode_Volt_frac = LineMode_Volt_full % 100;
      lv_label_set_text_fmt(ui_MeasurementValue, "%02d.%02d", LineMode_Volt_int, LineMode_Volt_frac);
    }
      break;
    default:
      break;
    }
}
