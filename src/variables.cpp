#include <variables.h>

uint8_t MeasureMode = 0; // 0: 通断测试, 1: 元件测量, 2: 负载网络, 3: 故障距离

float AIN0_DC_Volt;
float AIN1_AC_Volt;
float AIN2_Line_Volt;
float AIN3_Sys_Volt;

uint16_t DDS_SweepFreqTimems = 0; // 储存从机返回的扫频时间

bool RUNSTOP = true;

float LineMode_Volt_Avg = 0.0; // 平均电压

bool LoadMode_AC_OR_DC = false; // DC
uint8_t LoadNetType = 0;
// 0:无网络
// 1:RL并联 2:LC并联 3:RLC并联
// 4:RL串联 5:RC并联
// 6:RLC串联 7:LC串联 8:RC串联
bool LCRMode_AC_OR_DC = false; // DC