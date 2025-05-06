#include <ADS122C04_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_ADS122C0

#define SDA_PIN 18
#define SCL_PIN 19
#define DRDY_PIN 14
#define RESET_PIN 11

SFE_ADS122C04 mySensor(18, 19, 14, 11);

void setup(void)
{
  Serial.begin(115200);
  
  //mySensor.enableDebugging(); // 启用调试信息
  // 启用硬件DRDY中断和硬件复位功能
  mySensor.enableHardwareDRDY(true);
  mySensor.enableHardwareReset(true);

  mySensor.begin(0x40, Wire1); // 使用地址0x40和Wire1连接到ADS122C04

  mySensor.configureADCmode(ADS122C04_SINGLE_ENDED_MODE, ADS122C04_DATA_RATE_45SPS); // 配置为单端模式和20 SPS采样率

  mySensor.start();
}

void loop(void)
{
  if(mySensor.isDataReady() == true)
  {
  // 读取所有四个通道的电压
  ADS122C04_Voltages voltages = mySensor.readAllVoltages(ADS122C04_DATA_RATE_45SPS); // 使用较高采样率

  // 打印时间戳
  unsigned long currentMillis = millis();
  Serial.print(F("Time (ms): "));
  Serial.print(currentMillis);
  Serial.println(F(" - ADC Readings:"));
  
  // 打印每个通道的电压，保留6位小数
  Serial.print(F("AIN0: "));
  Serial.print(voltages.voltage_AIN0, 6);
  Serial.println(F(" V"));
  
  Serial.print(F("AIN1: "));
  Serial.print(voltages.voltage_AIN1, 6);
  Serial.println(F(" V"));
  
  Serial.print(F("AIN2: "));
  Serial.print(voltages.voltage_AIN2, 6);
  Serial.println(F(" V"));
  
  Serial.print(F("AIN3: "));
  Serial.print(voltages.voltage_AIN3, 6);
  Serial.println(F(" V"));
  
  Serial.println(F("--------------------------------------------"));
  }
}
