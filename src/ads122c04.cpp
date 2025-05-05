#include <ads122c04.h>

SFE_ADS122C04 mySensor(SDA_PIN, SCL_PIN, DRDY_PIN, RESET_PIN);

void ADS122C04_init()
{
    //mySensor.enableDebugging(); // 启用调试信息
    // 启用硬件DRDY中断和硬件复位功能
    mySensor.enableHardwareDRDY(true);
    mySensor.enableHardwareReset(true);
  
    mySensor.begin(0x40, Wire1); // 使用地址0x40和Wire1连接到ADS122C04
  
    mySensor.configureADCmode(ADS122C04_SINGLE_ENDED_MODE, ADS122C04_DATA_RATE_600SPS); // 配置为单端模式和20 SPS采样率
  
    mySensor.start();
}

void ADS122C04_task()
{
    if(mySensor.isDataReady() == true)
    {
        // 读取所有四个通道的电压
        ADS122C04_Voltages voltages = mySensor.readAllVoltages(ADS122C04_DATA_RATE_600SPS); // 使用较高采样率
        AIN0_DC_Volt = voltages.voltage_AIN0;
        AIN1_AC_Volt = voltages.voltage_AIN1;
        AIN2_Line_Volt = voltages.voltage_AIN2;
        AIN3_Sys_Volt = voltages.voltage_AIN3;
    }
}
