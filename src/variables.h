#ifndef _VARIABLES_H
#define _VARIABLES_H

#include <Arduino.h>

extern uint8_t MeasureMode;  // 0: 通断测试, 1: 元件测量, 2: 负载网络, 3: 故障距离

extern float AIN0_DC_Volt;
extern float AIN1_AC_Volt;
extern float AIN2_Line_Volt;
extern float AIN3_Sys_Volt;

extern uint16_t DDS_SweepFreqTimems; // 储存从机返回的扫频时间

extern bool RUNSTOP;

extern float LineMode_Volt_Avg; // 平均电压

#endif