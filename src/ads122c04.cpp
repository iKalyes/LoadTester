#include <ads122c04.h>

SFE_ADS122C04 mySensor(SDA_PIN, SCL_PIN, DRDY_PIN, RESET_PIN);

lv_chart_series_t * VoltageChart;
lv_timer_t* DataRefreshTimer;

int voltage_full;
int voltage_int;
int voltage_frac;

int LineMode_Volt_full;
int LineMode_Volt_int;
int LineMode_Volt_frac;

int ContMode_Volt_full;
int ContMode_Volt_int;
int ContMode_Volt_frac;

float LineMode_Distance;

void ADS122C04_init()
{
    //mySensor.enableDebugging(); // 启用调试信息
    // 启用硬件DRDY中断和硬件复位功能
    mySensor.enableHardwareDRDY(true);
    mySensor.enableHardwareReset(true);
  
    mySensor.begin(0x40, Wire1); // 使用地址0x40和Wire1连接到ADS122C04
  
    mySensor.configureADCmode(ADS122C04_SINGLE_ENDED_MODE, ADS122C04_DATA_RATE_330SPS); // 配置为单端模式和20 SPS采样率
  
    mySensor.start();
}

void ADS122C04_task()
{
    static uint8_t currentRate = ADS122C04_DATA_RATE_330SPS; // 跟踪当前采样率
    uint8_t targetRate;
    
    // 根据测量模式选择合适的采样率
    switch (MeasureMode)
    {
    case 0: // 通断测试
    case 3: // 故障距离
        targetRate = ADS122C04_DATA_RATE_330SPS; // 需要更高的采样率
        break;
    case 1: // 元件测量
        targetRate = ADS122C04_DATA_RATE_175SPS; // 中等采样率即可
        break;
    case 2: // 负载网络
        targetRate = ADS122C04_DATA_RATE_175SPS; // 中等采样率即可
        break;
    default:
        targetRate = currentRate; // 保持当前采样率
        break;
    }
    
    // 仅当采样率需要更改时才更新配置
    if(currentRate != targetRate) {
        mySensor.reset();
        mySensor.configureADCmode(ADS122C04_SINGLE_ENDED_MODE, targetRate); // 配置为单端模式和新的采样率
        currentRate = targetRate;
        mySensor.start(); // 重新启动转换
    }
    
    // 检查数据是否就绪
    if(mySensor.isDataReady() == true)
    {
        switch (MeasureMode)
        {
            case 0: // 通断测试
            AIN0_DC_Volt = mySensor.readVoltage(0, targetRate);
            break;
            case 1: // 元件测量
            if(LCRMode_AC_OR_DC_OR_LINE == 0)
            {
                AIN0_DC_Volt = mySensor.readVoltage(0, targetRate);
            }
            else if(LCRMode_AC_OR_DC_OR_LINE == 1)
            {
                AIN1_AC_Volt = mySensor.readVoltage(1, targetRate);
            }
            else if(LCRMode_AC_OR_DC_OR_LINE == 2)
            {
                AIN2_Line_Volt = mySensor.readVoltage(2, targetRate);
            }
            break;
            case 2: // 负载网络
            if(LoadMode_AC_OR_DC == false)
            {
                AIN0_DC_Volt = mySensor.readVoltage(0, targetRate);
            }
            else
            {
                AIN1_AC_Volt = mySensor.readVoltage(1, targetRate);
            }
            break;
            case 3: // 故障距离
            AIN2_Line_Volt = mySensor.readVoltage(2, targetRate);
            break;
            default:
            break;
    }       
    }
}

void ADS122C04_DATA_INIT()
{
  VoltageChart = lv_chart_add_series(ui_VoltageChart, lv_color_hex(0X000000), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_update_mode(ui_VoltageChart, LV_CHART_UPDATE_MODE_SHIFT);
  lv_chart_set_point_count(ui_VoltageChart, 128);
  DataRefreshTimer = lv_timer_create(ADS122C04_DATA_REFRESH, 1, NULL);
}

void ADS122C04_DATA_REFRESH(lv_timer_t *timer)
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
      if(AIN0_DC_Volt < 0.15)
      {
        lv_label_set_text(ui_MeasurementValue, "SHORT");
        lv_label_set_text(ui_MeasurementUNIT, " ");
      }
      else if(AIN0_DC_Volt > 0.15 && AIN0_DC_Volt < 0.95)
      {
        ContMode_Volt_full = round((1.000 - AIN0_DC_Volt) * 1000000);
        ContMode_Volt_int = ContMode_Volt_full / 1000000;
        ContMode_Volt_frac = ContMode_Volt_full % 1000000;
        lv_label_set_text_fmt(ui_MeasurementValue, "%01d.%06d", ContMode_Volt_int, ContMode_Volt_frac);
        lv_label_set_text(ui_MeasurementUNIT, "V");
      }
      else
      {
        lv_label_set_text(ui_MeasurementValue, "OPEN");
        lv_label_set_text(ui_MeasurementUNIT, " ");
      }
    }
      break;
    case 1:
    if(RUNSTOP == true)
    {
        if(LCRMode_AC_OR_DC_OR_LINE == 0)
        {
          voltage_full = round(AIN0_DC_Volt * 1000000);
          voltage_int = voltage_full / 1000000;
          voltage_frac = voltage_full % 1000000;
          lv_label_set_text_fmt(ui_MeasurementVoltage, "V-DC:%01d.%06dV", voltage_int, voltage_frac);
          lv_chart_set_next_value(ui_VoltageChart, VoltageChart, voltage_full);
        }
        else if(LCRMode_AC_OR_DC_OR_LINE == 1)
        {
        voltage_full = round(AIN1_AC_Volt * 1000000);
        voltage_int = voltage_full / 1000000;
        voltage_frac = voltage_full % 1000000;
        lv_label_set_text_fmt(ui_MeasurementVoltage, "V-AC:%01d.%06dV", voltage_int, voltage_frac);
        lv_chart_set_next_value(ui_VoltageChart, VoltageChart, voltage_full);
        }
        else if(LCRMode_AC_OR_DC_OR_LINE == 2)
        {
          voltage_full = round(AIN2_Line_Volt * 1000000);
          voltage_int = voltage_full / 1000000;
          voltage_frac = voltage_full % 1000000;
          lv_label_set_text_fmt(ui_MeasurementVoltage, "V-LINE:%01d.%06dV", voltage_int, voltage_frac);
          lv_chart_set_next_value(ui_VoltageChart, VoltageChart, voltage_full);
        }
        
        static uint8_t lastLCRModeType = 255; // 初始化为不可能的值，确保首次更新
        bool modeChanged = (lastLCRModeType != LCRModeType);
        
        switch(LCRModeType)
        {
          case 0: // 开路
          {
            static bool lastIsOpen = false;
            bool isOpen = true; // 开路状态为固定值
            
            // 增加模式切换检测，强制更新UI
            if(!lastIsOpen || !RUNSTOP || modeChanged) {
              lv_label_set_text(ui_MeasurementValue, "OPEN");
              lv_label_set_text(ui_MeasurementUNIT, " ");
              lv_label_set_text(ui_MeasurementCondition, "Source ~ ");
              lastIsOpen = isOpen;
            }
          }
          break;
          case 1: // 电阻
          {
            static float lastResistance = -1.0f; // 初始化为不可能的值
            
            // 增加模式切换检测，强制更新UI
            if(fabs(LCRMode_Value - lastResistance) > 0.01f || !RUNSTOP || modeChanged) {
              int LCRMode_Value_all = round(LCRMode_Value * 10000);
              int LCRMode_Value_int = LCRMode_Value_all / 10000;
              int LCRMode_Value_frac = LCRMode_Value_all % 100;
              lv_label_set_text_fmt(ui_MeasurementValue, "%04d.%02d", LCRMode_Value_int, LCRMode_Value_frac);
              lv_label_set_text(ui_MeasurementUNIT, "Ω");
              lv_label_set_text(ui_MeasurementCondition, "Source ~ 1000mV");
              
              lastResistance = LCRMode_Value;
            }
          }
          break;
          case 2: // 电容
          {
            static float lastCapacitance = -1.0f; // 初始化为不可能的值
            static String lastCapacitanceUnit = "";
            
            // 增加模式切换检测，强制更新UI
            if(fabs(LCRMode_Value - lastCapacitance) > 0.001f || 
               lastCapacitanceUnit != capacitanceUnit || !RUNSTOP || modeChanged) {
              int LCRMode_Value_all = round(LCRMode_Value * 1000);
              int LCRMode_Value_int = LCRMode_Value_all / 1000;
              int LCRMode_Value_frac = LCRMode_Value_all % 1000;
              lv_label_set_text_fmt(ui_MeasurementValue, "%03d.%03d", LCRMode_Value_int, LCRMode_Value_frac);
              lv_label_set_text(ui_MeasurementUNIT, capacitanceUnit.c_str());
              lv_label_set_text(ui_MeasurementCondition, "Source ~ 87-50000Hz");
              
              lastCapacitance = LCRMode_Value;
              lastCapacitanceUnit = capacitanceUnit;
            }
          }
          break;
          case 3: // 电感
          {
            static float lastInductance = -1.0f; // 初始化为不可能的值
            static String lastInductanceUnit = "";
            
            // 增加模式切换检测，强制更新UI
            if(fabs(LCRMode_Value - lastInductance) > 0.001f || 
               lastInductanceUnit != inductanceUnit || !RUNSTOP || modeChanged) {
              int LCRMode_Value_all = round(LCRMode_Value * 1000);
              int LCRMode_Value_int = LCRMode_Value_all / 1000;
              int LCRMode_Value_frac = LCRMode_Value_all % 1000;
              lv_label_set_text_fmt(ui_MeasurementValue, "%03d.%03d", LCRMode_Value_int, LCRMode_Value_frac);
              lv_label_set_text(ui_MeasurementUNIT, inductanceUnit.c_str());
              lv_label_set_text(ui_MeasurementCondition, "Source ~ 1-200kHz");
              
              lastInductance = LCRMode_Value;
              lastInductanceUnit = inductanceUnit;
            }
          }
          break;
        }
        
        // 更新lastLCRModeType以便下次比较
        lastLCRModeType = LCRModeType;
            }
              break;
      case 2:
      if(RUNSTOP == true)
      {
        if(LoadMode_AC_OR_DC == false)
        {
          voltage_full = round(AIN0_DC_Volt * 1000000);
          voltage_int = voltage_full / 1000000;
          voltage_frac = voltage_full % 1000000;
          lv_label_set_text_fmt(ui_MeasurementVoltage, "V-DC:%01d.%06dV", voltage_int, voltage_frac);
          lv_chart_set_next_value(ui_VoltageChart, VoltageChart, voltage_full);
        }
        else
        {
          voltage_full = round(AIN1_AC_Volt * 1000000);
          voltage_int = voltage_full / 1000000;
          voltage_frac = voltage_full % 1000000;
          lv_label_set_text_fmt(ui_MeasurementVoltage, "V-AC:%01d.%06dV", voltage_int, voltage_frac);
          lv_chart_set_next_value(ui_VoltageChart, VoltageChart, voltage_full);
        }
        
        // 添加静态变量记录上次显示的LoadNetType值
        static uint8_t lastLoadNetType = 255; // 初始化为不可能的值，确保首次会更新
        
        // 只有当负载网络类型变化时才更新显示
        if(lastLoadNetType != LoadNetType)
        {
          // 更新UI显示
          switch(LoadNetType)
          {
              case 0: // 无网络
              lv_label_set_text(ui_MeasurementValue, "OPEN");
              lv_label_set_text(ui_MeasurementCondition, "Source ~ ");
              break;
              case 1: // RC串联
              lv_label_set_text(ui_MeasurementValue, "RC串联");
              lv_label_set_text(ui_MeasurementCondition, "Source ~ 87-1MHz 1kΩ");
              break;
              case 2: // LC串联
              lv_label_set_text(ui_MeasurementValue, "LC串联");
              lv_label_set_text(ui_MeasurementCondition, "Source ~ 87-1MHz 1kΩ");
              break;
              case 3: // RLC串联
              lv_label_set_text(ui_MeasurementValue, "RLC串联");
              lv_label_set_text(ui_MeasurementCondition, "Source ~ 87-1MHz 1kΩ");
              break;
              case 4: // RL串联
              lv_label_set_text(ui_MeasurementValue, "RL串联");
              lv_label_set_text(ui_MeasurementCondition, "Source ~ 87-2MHz 1kΩ");
              break;
              case 5: // RC并联
              lv_label_set_text(ui_MeasurementValue, "RC并联");
              lv_label_set_text(ui_MeasurementCondition, "Source ~ 87-2MHz 1kΩ");
              break;
              case 6: // RL并联
              lv_label_set_text(ui_MeasurementValue, "RL并联");
              lv_label_set_text(ui_MeasurementCondition, "Source ~ 1-100kHz 50Ω");
              break;
              case 7: // LC并联
              lv_label_set_text(ui_MeasurementValue, "LC并联");
              lv_label_set_text(ui_MeasurementCondition, "Source ~ 1-100kHz 50Ω");
              break;
              case 8: // RLC并联
              lv_label_set_text(ui_MeasurementValue, "RLC并联");
              lv_label_set_text(ui_MeasurementCondition, "Source ~ 1-100kHz 50Ω");
              break;
              default:
              lv_label_set_text(ui_MeasurementValue, "OPEN");
              lv_label_set_text(ui_MeasurementCondition, "Source ~ ");
              break;
          }
          
          // 记录当前网络类型，用于下次比较
          lastLoadNetType = LoadNetType;

        }
      }
      else
      {
        // 当RUNSTOP为false时重置lastLoadNetType，确保下次开始测量时会更新显示
        static uint8_t lastLoadNetType = 255;
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

      if(LineMode_Distance < 100)
      {
        LineMode_Volt_full = round(LineMode_Distance * 100);
        LineMode_Volt_int = LineMode_Volt_full / 100;
        LineMode_Volt_frac = LineMode_Volt_full % 100;
        lv_label_set_text_fmt(ui_MeasurementValue, "%02d.%02d", LineMode_Volt_int, LineMode_Volt_frac);
        lv_label_set_text(ui_MeasurementUNIT, "CM");
      }
      else
      {
        lv_label_set_text(ui_MeasurementValue, "OPEN");
        lv_label_set_text(ui_MeasurementUNIT, "");
      }
    }
      break;
    default:
      break;
    }
}
