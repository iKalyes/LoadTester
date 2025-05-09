#include <ADS122C04_Library.h>

// 静态成员变量初始化
volatile bool SFE_ADS122C04::_dataReady = false;
SFE_ADS122C04* SFE_ADS122C04::_pADS122C04 = nullptr;

// 中断服务程序
void SFE_ADS122C04::dataReadyISR() {
  _dataReady = true;
}

SFE_ADS122C04::SFE_ADS122C04(uint8_t SDA, uint8_t SCL, uint8_t DRDY, uint8_t RESET)
{
  _sda = SDA;
  _scl = SCL;
  _drdy = DRDY;
  _reset = RESET;
}

//Attempt communication with the device and initialise it
//Return true if successful
bool SFE_ADS122C04::begin(uint8_t deviceAddress, TwoWire &wirePort)
{
  _deviceAddress = deviceAddress; //If provided, store the I2C address from user
  _i2cPort = &wirePort; //Grab which port the user wants us to use
  _wireMode = ADS122C04_SINGLE_ENDED_MODE; //Default to using 'safe' settings (disable the IDAC current sources)

  if(_useHardwareDRDY == false)
  {
    pinMode(_drdy, INPUT); // Set the DRDY pin as input
  }
  if(_useHardwareReset == false)
  {
    pinMode(_reset, OUTPUT); // Set the RESET pin as output
    digitalWrite(_reset, HIGH); // Set the RESET pin high
  }

  _i2cPort -> setSDA(_sda); // Set the SDA pin
  _i2cPort -> setSCL(_scl); // Set the SCL pin
  _i2cPort -> begin(); // Start the I2C port

  delay(1); // wait for power-on reset to complete (datasheet says we should do this)

  if (isConnected() == false)
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("begin: isConnected returned false"));
    }
    return (false);
  }

  reset(); // reset the ADS122C04 (datasheet says we should do this)

  return(configureADCmode(ADS122C04_SINGLE_ENDED_MODE)); // Default to using 'safe' settings (disable the IDAC current sources)
}

// Configure the chip for the selected wire mode
bool SFE_ADS122C04::configureADCmode(uint8_t wire_mode, uint8_t rate)
{
  ADS122C04_initParam initParams; // Storage for the chip parameters

  if (wire_mode == ADS122C04_TEMPERATURE_MODE) // Internal temperature mode
  {
    initParams.inputMux = ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_1; // Set the gain to 1
    initParams.pgaBypass = ADS122C04_PGA_DISABLED;
    initParams.dataRate = rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref = ADS122C04_VREF_INTERNAL; // Use the internal 2.048V reference
    initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_ON; // Enable the temperature sensor
    initParams.dataCounterEn = ADS122C04_DCNT_DISABLE; // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent = ADS122C04_IDAC_CURRENT_OFF; // Disable the IDAC current
    initParams.routeIDAC1 = ADS122C04_IDAC1_DISABLED; // Disable IDAC1
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_TEMPERATURE_MODE; // Update the wire mode
  }
  else if (wire_mode == ADS122C04_RAW_MODE) // Raw mode : disable the IDAC and use the internal reference
  {
    initParams.inputMux = ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_1; // Set the gain to 1
    initParams.pgaBypass = ADS122C04_PGA_DISABLED;
    initParams.dataRate = rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref = ADS122C04_VREF_INTERNAL; // Use the internal 2.048V reference
    initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn = ADS122C04_DCNT_DISABLE; // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent = ADS122C04_IDAC_CURRENT_OFF; // Disable the IDAC current
    initParams.routeIDAC1 = ADS122C04_IDAC1_DISABLED; // Disable IDAC1
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_RAW_MODE; // Update the wire mode
  }
  else if (wire_mode == ADS122C04_SINGLE_ENDED_MODE) // 单端ADC模式
  {
    initParams.gainLevel = ADS122C04_GAIN_1; // 设置增益为1
    initParams.pgaBypass = ADS122C04_PGA_DISABLED;
    initParams.dataRate = rate; // 设置采样率
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // 禁用turbo模式
    initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // 使用单次转换模式
    initParams.selectVref = ADS122C04_VREF_INTERNAL; // 使用内部2.048V参考电压
    initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF; // 禁用温度传感器
    initParams.dataCounterEn = ADS122C04_DCNT_DISABLE; // 禁用数据计数器
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // 禁用CRC校验
    initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF; // 禁用烧断电流
    initParams.idacCurrent = ADS122C04_IDAC_CURRENT_OFF; // 禁用IDAC电流
    initParams.routeIDAC1 = ADS122C04_IDAC1_DISABLED; // 禁用IDAC1
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // 禁用IDAC2
    _wireMode = ADS122C04_SINGLE_ENDED_MODE; // 更新线路模式
  }
  else
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("configureADCmode: unknown mode"));
    }
    return(false);
  }
  return(ADS122C04_init(&initParams)); // Configure the chip
}

// 读取所有四个单端ADC通道的电压值
ADS122C04_Voltages SFE_ADS122C04::readAllVoltages(uint8_t rate)
{
  ADS122C04_Voltages voltages = {0.0, 0.0, 0.0, 0.0}; // 初始化结果结构体
  raw_voltage_union raw_v; // 用于转换uint32_t为int32_t的联合体
  unsigned long start_time; // 记录开始时间以便设置超时
  bool drdy = false; // DRDY (1 == 新数据已就绪)
  uint8_t previousWireMode = _wireMode; // 记录先前的线路模式以便恢复
  uint8_t previousRate = ADS122C04_Reg.reg1.bit.DR; // 记录先前的采样率以便恢复
  bool configChanged = (_wireMode != ADS122C04_SINGLE_ENDED_MODE) || (previousRate != rate); // 仅在需要时更改配置
  
  // 配置ADS122C04为单端模式
  if (configChanged)
  {
    if ((configureADCmode(ADS122C04_SINGLE_ENDED_MODE, rate)) == false)
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("readAllVoltages: configureADCmode failed"));
      }
      return voltages;
    }
  }
  
  // 单端多路复用器配置数组，对应AIN0-AVSS到AIN3-AVSS
  uint8_t muxConfigs[4] = {
    ADS122C04_MUX_AIN0_AVSS,
    ADS122C04_MUX_AIN1_AVSS,
    ADS122C04_MUX_AIN2_AVSS,
    ADS122C04_MUX_AIN3_AVSS
  };
  
  // 对每个通道进行读取
  for (uint8_t channel = 0; channel < 4; channel++)
  {
    // 配置多路复用器选择当前通道
    if (setInputMultiplexer(muxConfigs[channel]) == false)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("readAllVoltages: setInputMultiplexer failed for channel "));
        _debugPort->println(channel);
      }
      continue; // 继续下一个通道
    }
    
    // 启动转换
    start();
    
    // 等待DRDY变为有效
    drdy = false;
    start_time = millis();
    while((drdy == false) && (millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT)))
    {
      drdy = checkDataReady();
    }
    
    // 检查是否超时
    if (drdy == false)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("readAllVoltages: checkDataReady timed out for channel "));
        _debugPort->println(channel);
      }
      continue; // 继续下一个通道
    }
    
    // 读取转换结果
    if(ADS122C04_getConversionData(&raw_v.UINT32) == false)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("readAllVoltages: ADS122C04_getConversionData failed for channel "));
        _debugPort->println(channel);
      }
      continue; // 继续下一个通道
    }
    
    // 处理24位有符号数的符号扩展
    if ((raw_v.UINT32 & 0x00800000) == 0x00800000)
      raw_v.UINT32 |= 0xFF000000;
    
    // 转换为电压值 - 使用内部参考电压2.048V且增益为1
    // LSB值 = 2.048V / 2^23 = 0.24414 微伏
    float voltage = (float)raw_v.INT32 * 2.048 / 8388608.0; // 2^23 = 8388608
    
    // 存储结果到对应通道
    switch (channel) {
      case 0: voltages.voltage_AIN0 = voltage; break;
      case 1: voltages.voltage_AIN1 = voltage; break;
      case 2: voltages.voltage_AIN2 = voltage; break;
      case 3: voltages.voltage_AIN3 = voltage; break;
    }
  }
  
  // 恢复之前的线路模式
  if (configChanged)
  {
    if ((configureADCmode(previousWireMode, previousRate)) == false)
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("readAllVoltages: failed to restore previous mode"));
      }
    }
  }
  
  return voltages;
}

// 读取单个指定通道的ADC电压值
float SFE_ADS122C04::readVoltage(uint8_t channel, uint8_t rate)
{
  if (channel > 3) {
    if (_printDebug == true) {
      _debugPort->print(F("readVoltage: Invalid channel ("));
      _debugPort->print(channel);
      _debugPort->println(F("), must be 0-3"));
    }
    return 0.0; // 返回0表示错误
  }

  raw_voltage_union raw_v; // 用于转换uint32_t为int32_t的联合体
  unsigned long start_time; // 记录开始时间以便设置超时
  bool drdy = false; // DRDY (1 == 新数据已就绪)
  uint8_t previousWireMode = _wireMode; // 记录先前的线路模式以便恢复
  uint8_t previousRate = ADS122C04_Reg.reg1.bit.DR; // 记录先前的采样率以便恢复
  bool configChanged = (_wireMode != ADS122C04_SINGLE_ENDED_MODE) || (previousRate != rate); // 仅在需要时更改配置
  float voltage = 0.0; // 返回值
  
  // 配置ADS122C04为单端模式
  if (configChanged)
  {
    if ((configureADCmode(ADS122C04_SINGLE_ENDED_MODE, rate)) == false)
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("readVoltage: configureADCmode failed"));
      }
      return 0.0;
    }
  }
  
  // 单端多路复用器配置，根据通道选择
  uint8_t muxConfig;
  switch (channel) {
    case 0: muxConfig = ADS122C04_MUX_AIN0_AVSS; break;
    case 1: muxConfig = ADS122C04_MUX_AIN1_AVSS; break;
    case 2: muxConfig = ADS122C04_MUX_AIN2_AVSS; break;
    case 3: muxConfig = ADS122C04_MUX_AIN3_AVSS; break;
    default: return 0.0; // 不应该到达这里
  }
  
  // 配置多路复用器选择当前通道
  if (setInputMultiplexer(muxConfig) == false)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("readVoltage: setInputMultiplexer failed for channel "));
      _debugPort->println(channel);
    }
    if (configChanged) {
      configureADCmode(previousWireMode, previousRate); // 尝试恢复先前模式
    }
    return 0.0;
  }
  
  // 启动转换
  start();
  
  // 等待DRDY变为有效
  drdy = false;
  start_time = millis();
  while((drdy == false) && (millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT)))
  {
    drdy = checkDataReady();
  }
  
  // 检查是否超时
  if (drdy == false)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("readVoltage: checkDataReady timed out for channel "));
      _debugPort->println(channel);
    }
    if (configChanged) {
      configureADCmode(previousWireMode, previousRate); // 尝试恢复先前模式
    }
    return 0.0;
  }
  
  // 读取转换结果
  if(ADS122C04_getConversionData(&raw_v.UINT32) == false)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("readVoltage: ADS122C04_getConversionData failed for channel "));
      _debugPort->println(channel);
    }
    if (configChanged) {
      configureADCmode(previousWireMode, previousRate); // 尝试恢复先前模式
    }
    return 0.0;
  }
  
  // 处理24位有符号数的符号扩展
  if ((raw_v.UINT32 & 0x00800000) == 0x00800000)
    raw_v.UINT32 |= 0xFF000000;
  
  // 转换为电压值 - 使用内部参考电压2.048V且增益为1
  // LSB值 = 2.048V / 2^23 = 0.24414 微伏
  voltage = (float)raw_v.INT32 * 2.048 / 8388608.0; // 2^23 = 8388608
  
  // 恢复之前的线路模式
  if (configChanged)
  {
    if ((configureADCmode(previousWireMode, previousRate)) == false)
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("readVoltage: failed to restore previous mode"));
      }
    }
  }
  
  return voltage;
}

//Returns true if device answers on _deviceAddress
bool SFE_ADS122C04::isConnected(void)
{
  _i2cPort->beginTransmission((uint8_t)_deviceAddress);
  return (_i2cPort->endTransmission() == 0);
}

//Enable or disable the printing of debug messages
void SFE_ADS122C04::enableDebugging(Stream &debugPort)
{
  _debugPort = &debugPort; //Grab which port the user wants us to use for debugging
  _printDebug = true; //Should we print the commands we send? Good for debugging
}

void SFE_ADS122C04::disableDebugging(void)
{
  _printDebug = false; //Turn off extra print statements
}

//Safely print messages
void SFE_ADS122C04::debugPrint(char *message)
{
  if (_printDebug == true)
  {
    _debugPort->print(message);
  }
}

//Safely print messages
void SFE_ADS122C04::debugPrintln(char *message)
{
  if (_printDebug == true)
  {
    _debugPort->println(message);
  }
}

// Read the raw signed 24-bit ADC value as int32_t
// The result needs to be multiplied by VREF / GAIN to convert to Volts
int32_t SFE_ADS122C04::readRawVoltage(uint8_t rate)
{
  raw_voltage_union raw_v; // union to convert uint32_t to int32_t
  unsigned long start_time = millis(); // Record the start time so we can timeout
  bool drdy = false; // DRDY (1 == new data is ready)
  uint8_t previousWireMode = _wireMode; // Record the previous wire mode so we can restore it
  uint8_t previousRate = ADS122C04_Reg.reg1.bit.DR; // Record the previous rate so we can restore it
  bool configChanged = (_wireMode != ADS122C04_RAW_MODE) || (previousRate != rate); // Only change the configuration if we need to

  // Configure the ADS122C04 for raw mode
  // Disable the IDAC, use the internal 2.048V reference and set the gain to 1
  if (configChanged)
  {
    if ((configureADCmode(ADS122C04_RAW_MODE, rate)) == false)
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("readRawVoltage: configureADCmode (1) failed"));
      }
      return(0);
    }
  }

  // Start the conversion (assumes we are using single shot mode)
  start();

  // Wait for DRDY to go valid
  while((drdy == false) && (millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT)))
  {
    delay(1); // Don't pound the bus too hard
    drdy = checkDataReady();
  }

  // Check if we timed out
  if (drdy == false)
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("readRawVoltage: checkDataReady timed out"));
    }
    if (configChanged)
      configureADCmode(previousWireMode, previousRate); // Attempt to restore the previous wire mode
    return(0);
  }

  // Read the conversion result
  if(ADS122C04_getConversionData(&raw_v.UINT32) == false)
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("readRawVoltage: ADS122C04_getConversionData failed"));
    }
    if (configChanged)
      configureADCmode(previousWireMode, previousRate); // Attempt to restore the previous wire mode
    return(0);
  }

  // Restore the previous wire mode
  if (configChanged)
  {
    if ((configureADCmode(previousWireMode, previousRate)) == false)
    {
    if (_printDebug == true)
      {
        _debugPort->println(F("readRawVoltage: configureADCmode (2) failed"));
      }
      return(0);
    }
  }

  // The raw voltage is in the bottom 24 bits of raw_temp
  // If we just do a <<8 we will multiply the result by 256
  // Instead pad out the MSB with the MS bit of the 24 bits
  // to preserve the two's complement
  if ((raw_v.UINT32 & 0x00800000) == 0x00800000)
    raw_v.UINT32 |= 0xFF000000;
  return(raw_v.INT32);
}

// Read the raw signed 24-bit ADC value as uint32_t
// The ADC data is returned in the least-significant 24-bits
// Higher functions will need to convert the result to (e.g.) int32_t
uint32_t SFE_ADS122C04::readADC(void)
{
  uint32_t ret_val; // The return value

  // Read the conversion result
  if(ADS122C04_getConversionData(&ret_val) == false)
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("readADC: ADS122C04_getConversionData failed"));
    }
    return(0);
  }

  return(ret_val);
}

// Read the internal temperature
float SFE_ADS122C04::readInternalTemperature(uint8_t rate)
{
  internal_temperature_union int_temp; // union to convert uint16_t to int16_t
  uint32_t raw_temp; // The raw temperature from the ADC
  unsigned long start_time = millis(); // Record the start time so we can timeout
  bool drdy = false; // DRDY (1 == new data is ready)
  float ret_val = 0.0; // The return value
  uint8_t previousWireMode = _wireMode; // Record the previous wire mode so we can restore it
  uint8_t previousRate = ADS122C04_Reg.reg1.bit.DR; // Record the previous rate so we can restore it
  bool configChanged = (_wireMode != ADS122C04_TEMPERATURE_MODE) || (previousRate != rate); // Only change the configuration if we need to

  // Enable the internal temperature sensor
  // Reading the ADC value will return the temperature
  if (configChanged)
  {
    if ((configureADCmode(ADS122C04_TEMPERATURE_MODE, rate)) == false)
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("readInternalTemperature: configureADCmode (1) failed"));
      }
      return(ret_val);
    }
  }

  // Start the conversion
  start();

  // Wait for DRDY to go valid
  while((drdy == false) && (millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT)))
  {
    delay(1); // Don't pound the bus too hard
    drdy = checkDataReady();
  }

  // Check if we timed out
  if (drdy == false)
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("readInternalTemperature: checkDataReady timed out"));
    }
    if (configChanged)
      configureADCmode(previousWireMode, previousRate); // Attempt to restore the previous wire mode
    return(ret_val);
  }

  // Read the conversion result
  if(ADS122C04_getConversionData(&raw_temp) == false)
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("readInternalTemperature: ADS122C04_getConversionData failed"));
    }
    if (configChanged)
      configureADCmode(previousWireMode, previousRate); // Attempt to restore the previous wire mode
    return(ret_val);
  }

  // Restore the previous wire mode
  if (configChanged)
  {
    if ((configureADCmode(previousWireMode, previousRate)) == false)
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("readInternalTemperature: configureADCmode (2) failed"));
      }
      return(ret_val);
    }
  }

  if (_printDebug == true)
  {
    _debugPort->print(F("readInternalTemperature: raw_temp (32-bit) = 0x"));
    _debugPort->println(raw_temp, HEX);
  }

  // The temperature is in the top 14 bits of the bottom 24 bits of raw_temp
  int_temp.UINT16 = (uint16_t)(raw_temp >> 10); // Extract the 14-bit value

  // The signed temperature is now in the bottom 14 bits of int_temp.UINT16
  // If we just do a <<2 we will multiply the result by 4
  // Instead we will pad out the two MS bits with the MS bit of the 14 bits
  // to preserve the two's complement
  if ((int_temp.UINT16 & 0x2000) == 0x2000) // Check if the MS bit is 1
  {
    int_temp.UINT16 |= 0xC000; // Value is negative so pad with 1's
  }
  else
  {
    int_temp.UINT16 &= 0x3FFF;  // Value is positive so make sure the two MS bits are 0
  }

  ret_val = ((float)int_temp.INT16) * TEMPERATURE_SENSOR_RESOLUTION; // Convert to float including the 2 bit shift
  return(ret_val);
}

// Configure the input multiplexer
bool SFE_ADS122C04::setInputMultiplexer(uint8_t mux_config)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) == false)
    return(false);
  ADS122C04_Reg.reg0.bit.MUX = mux_config;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

// Configure the gain
bool SFE_ADS122C04::setGain(uint8_t gain_config)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) == false)
    return(false);
  ADS122C04_Reg.reg0.bit.GAIN = gain_config;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

// Enable/disable the Programmable Gain Amplifier
bool SFE_ADS122C04::enablePGA(uint8_t enable)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) == false)
    return(false);
  ADS122C04_Reg.reg0.bit.PGA_BYPASS = enable;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

// Set the data rate (sample speed)
bool SFE_ADS122C04::setDataRate(uint8_t rate)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.DR = rate;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Configure the operating mode (normal / turbo)
bool SFE_ADS122C04::setOperatingMode(uint8_t mode)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.MODE = mode;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Configure the conversion mode (single-shot / continuous)
bool SFE_ADS122C04::setConversionMode(uint8_t mode)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.CMBIT = mode;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Configure the voltage reference
bool SFE_ADS122C04::setVoltageReference(uint8_t ref)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.VREF = ref;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Enable / disable the internal temperature sensor
bool SFE_ADS122C04::enableInternalTempSensor(uint8_t enable)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.TS = enable;
  if (_printDebug == true)
  {
    _debugPort->print(F("enableInternalTempSensor: ADS122C04_Reg.reg1.bit.TS = 0x"));
    _debugPort->println(ADS122C04_Reg.reg1.bit.TS, HEX);
  }
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Enable / disable the conversion data counter
bool SFE_ADS122C04::setDataCounter(uint8_t enable)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
    return(false);
  ADS122C04_Reg.reg2.bit.DCNT = enable;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Configure the data integrity check
bool SFE_ADS122C04::setDataIntegrityCheck(uint8_t setting)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
    return(false);
  ADS122C04_Reg.reg2.bit.CRCbits = setting;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Enable / disable the 10uA burn-out current source
bool SFE_ADS122C04::setBurnOutCurrent(uint8_t enable)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
    return(false);
  ADS122C04_Reg.reg2.bit.BCS = enable;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Configure the internal programmable current sources
bool SFE_ADS122C04::setIDACcurrent(uint8_t current)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
    return(false);
  ADS122C04_Reg.reg2.bit.IDAC = current;
  if (_printDebug == true)
  {
    _debugPort->print(F("setIDACcurrent: ADS122C04_Reg.reg2.bit.IDAC = 0x"));
    _debugPort->println(ADS122C04_Reg.reg2.bit.IDAC, HEX);
  }
  return(ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Configure the IDAC1 routing
bool SFE_ADS122C04::setIDAC1mux(uint8_t setting)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all)) == false)
    return(false);
  ADS122C04_Reg.reg3.bit.I1MUX = setting;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all));
}

// Configure the IDAC2 routing
bool SFE_ADS122C04::setIDAC2mux(uint8_t setting)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all)) == false)
    return(false);
  ADS122C04_Reg.reg3.bit.I2MUX = setting;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all));
}

// Read Config Reg 2 and check the DRDY bit
// Data is ready when DRDY is high
bool SFE_ADS122C04::checkDataReady(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return(ADS122C04_Reg.reg2.bit.DRDY > 0);
}

void SFE_ADS122C04::enableHardwareDRDY(bool enable) {
  _useHardwareDRDY = enable;
  
  if (enable) {
    // 设置实例指针和中断处理
    _pADS122C04 = this;
    _dataReady = false;
    
    // 配置DRDY引脚为输入，并启用中断
    pinMode(_drdy, INPUT);
    attachInterrupt(digitalPinToInterrupt(_drdy), dataReadyISR, FALLING); // 下降沿触发
    
    if (_printDebug) {
      _debugPort->println(F("Hardware DRDY enabled with interrupt"));
    }
  } else {
    // 禁用中断
    detachInterrupt(digitalPinToInterrupt(_drdy));
    _dataReady = false;
    
    if (_printDebug) {
      _debugPort->println(F("Hardware DRDY disabled"));
    }
  }
}

bool SFE_ADS122C04::isDataReady() {
  if (_useHardwareDRDY) {
    // 使用中断标志检查数据就绪
    bool ready = _dataReady;
    if (ready) {
      _dataReady = false; // 清除标志
    }
    return ready;
  } else {
    // 使用寄存器检查
    return checkDataReady();
  }
}

// Get the input multiplexer configuration
uint8_t SFE_ADS122C04::getInputMultiplexer(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
  return(ADS122C04_Reg.reg0.bit.MUX);
}

// Get the gain setting
uint8_t SFE_ADS122C04::getGain(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
  return(ADS122C04_Reg.reg0.bit.GAIN);
}

// Get the Programmable Gain Amplifier status
uint8_t SFE_ADS122C04::getPGAstatus(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
  return(ADS122C04_Reg.reg0.bit.PGA_BYPASS);
}

// Get the data rate (sample speed)
uint8_t SFE_ADS122C04::getDataRate(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  return(ADS122C04_Reg.reg1.bit.DR);
}

// Get the operating mode (normal / turbo)
uint8_t SFE_ADS122C04::getOperatingMode(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  return(ADS122C04_Reg.reg1.bit.MODE);
}

// Get the conversion mode (single-shot / continuous)
uint8_t SFE_ADS122C04::getConversionMode(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  return(ADS122C04_Reg.reg1.bit.CMBIT);
}

// Get the voltage reference configuration
uint8_t SFE_ADS122C04::getVoltageReference(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  return(ADS122C04_Reg.reg1.bit.VREF);
}

// Get the internal temperature sensor status
uint8_t SFE_ADS122C04::getInternalTempSensorStatus(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  if (_printDebug == true)
  {
    _debugPort->print(F("getInternalTempSensorStatus: ADS122C04_Reg.reg1.bit.TS = 0x"));
    _debugPort->println(ADS122C04_Reg.reg1.bit.TS, HEX);
  }
  return(ADS122C04_Reg.reg1.bit.TS);
}

// Get the data counter status
uint8_t SFE_ADS122C04::getDataCounter(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return(ADS122C04_Reg.reg2.bit.DCNT);
}

// Get the data integrity check configuration
uint8_t SFE_ADS122C04::getDataIntegrityCheck(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return(ADS122C04_Reg.reg2.bit.CRCbits);
}

// Get the burn-out current status
uint8_t SFE_ADS122C04::getBurnOutCurrent(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return(ADS122C04_Reg.reg2.bit.BCS);
}

// Get the IDAC setting
uint8_t SFE_ADS122C04::getIDACcurrent(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  if (_printDebug == true)
  {
    _debugPort->print(F("getIDACcurrent: ADS122C04_Reg.reg2.bit.IDAC = 0x"));
    _debugPort->println(ADS122C04_Reg.reg2.bit.IDAC, HEX);
  }
  return(ADS122C04_Reg.reg2.bit.IDAC);
}

// Get the IDAC1 mux configuration
uint8_t SFE_ADS122C04::getIDAC1mux(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);
  return(ADS122C04_Reg.reg3.bit.I1MUX);
}

// Get the IDAC2 mux configuration
uint8_t SFE_ADS122C04::getIDAC2mux(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);
  return(ADS122C04_Reg.reg3.bit.I2MUX);
}

// Update ADS122C04_Reg and initialise the ADS122C04 using the supplied parameters
bool SFE_ADS122C04::ADS122C04_init(ADS122C04_initParam *param)
{
  ADS122C04_Reg.reg0.all = 0; // Reset all four register values to the default value of 0x00
  ADS122C04_Reg.reg1.all = 0;
  ADS122C04_Reg.reg2.all = 0;
  ADS122C04_Reg.reg3.all = 0;

  ADS122C04_Reg.reg0.bit.MUX = param->inputMux;
  ADS122C04_Reg.reg0.bit.GAIN = param->gainLevel;
  ADS122C04_Reg.reg0.bit.PGA_BYPASS = param->pgaBypass;

  ADS122C04_Reg.reg1.bit.DR = param->dataRate;
  ADS122C04_Reg.reg1.bit.MODE = param->opMode;
  ADS122C04_Reg.reg1.bit.CMBIT = param->convMode;
  ADS122C04_Reg.reg1.bit.VREF = param->selectVref;
  ADS122C04_Reg.reg1.bit.TS = param->tempSensorEn;

  ADS122C04_Reg.reg2.bit.DCNT = param->dataCounterEn;
  ADS122C04_Reg.reg2.bit.CRCbits = param->dataCRCen;
  ADS122C04_Reg.reg2.bit.BCS = param->burnOutEn;
  ADS122C04_Reg.reg2.bit.IDAC = param->idacCurrent;

  ADS122C04_Reg.reg3.bit.I1MUX = param->routeIDAC1;
  ADS122C04_Reg.reg3.bit.I2MUX = param->routeIDAC2;

  bool ret_val = true; // Flag to show if the four writeRegs were successful
  // (If any one writeReg returns false, ret_val will be false)
  ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all);
  ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all);
  ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all);
  ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all);

  // Read and print the new configuration (if enableDebugging has been called)
  printADS122C04config();

  return(ret_val);
}

// Debug print of the ADS122C04 configuration
void SFE_ADS122C04::printADS122C04config(void)
{
  if (_printDebug == true)
  {
    bool successful = true; // Flag to show if the four readRegs were successful
    // (If any one readReg returns false, success will be false)
    successful &= ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
    successful &= ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
    successful &= ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
    successful &= ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);

    if (successful == false)
    {
      _debugPort->println(F("printADS122C04config: readReg failed"));
      return;
    }
    else
    {
      _debugPort->print(F("ConfigReg0: MUX="));
      _debugPort->print(ADS122C04_Reg.reg0.bit.MUX);
      _debugPort->print(F(" GAIN="));
      _debugPort->print(ADS122C04_Reg.reg0.bit.GAIN);
      _debugPort->print(F(" PGA_BYPASS="));
      _debugPort->println(ADS122C04_Reg.reg0.bit.PGA_BYPASS);
      _debugPort->print(F("ConfigReg1: DR="));
      _debugPort->print(ADS122C04_Reg.reg1.bit.DR);
      _debugPort->print(F(" MODE="));
      _debugPort->print(ADS122C04_Reg.reg1.bit.MODE);
      _debugPort->print(F(" CMBIT="));
      _debugPort->print(ADS122C04_Reg.reg1.bit.CMBIT);
      _debugPort->print(F(" VREF="));
      _debugPort->print(ADS122C04_Reg.reg1.bit.VREF);
      _debugPort->print(F(" TS="));
      _debugPort->println(ADS122C04_Reg.reg1.bit.TS);
      _debugPort->print(F("ConfigReg2: DCNT="));
      _debugPort->print(ADS122C04_Reg.reg2.bit.DCNT);
      _debugPort->print(F(" CRC="));
      _debugPort->print(ADS122C04_Reg.reg2.bit.CRCbits);
      _debugPort->print(F(" BCS="));
      _debugPort->print(ADS122C04_Reg.reg2.bit.BCS);
      _debugPort->print(F(" IDAC="));
      _debugPort->println(ADS122C04_Reg.reg2.bit.IDAC);
      _debugPort->print(F("ConfigReg3: I1MUX="));
      _debugPort->print(ADS122C04_Reg.reg3.bit.I1MUX);
      _debugPort->print(F(" I2MUX="));
      _debugPort->println(ADS122C04_Reg.reg3.bit.I2MUX);
    }
  }
}

bool SFE_ADS122C04::reset(void)
{
  if (_useHardwareReset)
  {
    digitalWrite(_reset, LOW);
    delay(1); // 确保复位信号足够长
    digitalWrite(_reset, HIGH);
    delay(1); // 等待芯片稳定
    if (_printDebug) {
      _debugPort->println(F("Hardware RESET performed"));
    }
    return true;
  }
  if (_printDebug) {
    _debugPort->println(F("Hardware RESET not enabled"));
  }
  return(ADS122C04_sendCommand(ADS122C04_RESET_CMD));
}

void SFE_ADS122C04::enableHardwareReset(bool enable) {
  _useHardwareReset = enable;
  
  if (enable) {
    pinMode(_reset, OUTPUT);
    digitalWrite(_reset, HIGH); // 默认高电平
    
    if (_printDebug) {
      _debugPort->println(F("Hardware RESET enabled"));
    }
  }
}

bool SFE_ADS122C04::start(void)
{
  return(ADS122C04_sendCommand(ADS122C04_START_CMD));
}

bool SFE_ADS122C04::powerdown(void)
{
  return(ADS122C04_sendCommand(ADS122C04_POWERDOWN_CMD));
}

bool SFE_ADS122C04::ADS122C04_writeReg(uint8_t reg, uint8_t writeValue)
{
  uint8_t command = 0;
  command = ADS122C04_WRITE_CMD(reg);
  return(ADS122C04_sendCommandWithValue(command, writeValue));
}

bool SFE_ADS122C04::ADS122C04_readReg(uint8_t reg, uint8_t *readValue)
{
  uint8_t command = 0;
  command = ADS122C04_READ_CMD(reg);

  _i2cPort->beginTransmission((uint8_t)_deviceAddress);
  _i2cPort->write(command);

  if (_i2cPort->endTransmission(false) != 0)    //Do not release bus
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("ADS122C04_readReg: sensor did not ACK"));
    }
  return (false); //Sensor did not ACK
  }

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)1); // Request one byte
  if (_i2cPort->available() >= 1)
  {
    *readValue = _i2cPort->read();
    return(true);
  }

  if (_printDebug == true)
  {
    _debugPort->println(F("ADS122C04_readReg: requestFrom returned no data"));
  }
  return(false);
}

bool SFE_ADS122C04::ADS122C04_sendCommand(uint8_t command)
{
  _i2cPort->beginTransmission((uint8_t)_deviceAddress);
  _i2cPort->write(command);
  return (_i2cPort->endTransmission() == 0);
}

bool SFE_ADS122C04::ADS122C04_sendCommandWithValue(uint8_t command, uint8_t value)
{
  _i2cPort->beginTransmission((uint8_t)_deviceAddress);
  _i2cPort->write(command);
  _i2cPort->write(value);
  return (_i2cPort->endTransmission() == 0);
}

// Read the conversion result with count byte.
// The conversion result is 24-bit two's complement (signed)
// and is returned in the 24 lowest bits of the uint32_t conversionData.
// Hence it will always appear positive.
// Higher functions will need to take care of converting it to (e.g.) float or int32_t.
bool SFE_ADS122C04::ADS122C04_getConversionDataWithCount(uint32_t *conversionData, uint8_t *count)
{
  uint8_t RXByte[4] = {0};

  _i2cPort->beginTransmission((uint8_t)_deviceAddress);
  _i2cPort->write(ADS122C04_RDATA_CMD);

  if (_i2cPort->endTransmission(false) != 0)    //Do not release bus
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("ADS122C04_getConversionDataWithCount: sensor did not ACK"));
    }
    return(false); //Sensor did not ACK
  }

  // Note: the next line will need to be changed if data integrity is enabled.
  //       The code will need to request 6 bytes for CRC or 7 bytes for inverted data.
  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)4); // Request four bytes

  if (_printDebug == true)
  {
    if (_i2cPort->available() == 3)
    {
      _debugPort->println(F("ADS122C04_getConversionDataWithCount: only 3 bytes available. Maybe DCNT is disabled?"));
    }
  }

  if (_i2cPort->available() >= 4)
  {
    RXByte[0] = _i2cPort->read(); // Count
    RXByte[1] = _i2cPort->read(); // MSB
    RXByte[2] = _i2cPort->read();
    RXByte[3] = _i2cPort->read(); // LSB
    if (_i2cPort->available() > 0)// Note: this _should_ be redundant
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("ADS122C04_getConversionDataWithCount: excess bytes available. Maybe data integrity is enabled?"));
      }
      while (_i2cPort->available() > 0)
      {
        _i2cPort->read(); // Read and ignore excess bytes (presumably inverted data or CRC)
      }
    }
  }
  else
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("ADS122C04_getConversionDataWithCount: requestFrom failed"));
    }
    return(false);
  }

  *count = RXByte[0];
  *conversionData = ((uint32_t)RXByte[3]) | ((uint32_t)RXByte[2]<<8) | ((uint32_t)RXByte[1]<<16);
  return(true);
}

// Read the conversion result.
// The conversion result is 24-bit two's complement (signed)
// and is returned in the 24 lowest bits of the uint32_t conversionData.
// Hence it will always appear positive.
// Higher functions will need to take care of converting it to (e.g.) float or int32_t.
bool SFE_ADS122C04::ADS122C04_getConversionData(uint32_t *conversionData)
{
  uint8_t RXByte[3] = {0};

  _i2cPort->beginTransmission((uint8_t)_deviceAddress);
  _i2cPort->write(ADS122C04_RDATA_CMD);

  if (_i2cPort->endTransmission(false) != 0)    //Do not release bus
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("ADS122C04_getConversionData: sensor did not ACK"));
    }
    return(false); //Sensor did not ACK
  }

  // Note: the next line will need to be changed if data integrity is enabled.
  //       The code will need to request 5 bytes for CRC or 6 bytes for inverted data.
  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)3); // Request three bytes

  if (_i2cPort->available() >= 3)
  {
    RXByte[0] = _i2cPort->read(); // MSB
    RXByte[1] = _i2cPort->read();
    RXByte[2] = _i2cPort->read(); // LSB
    if (_i2cPort->available() > 0) // Note: this _should_ be redundant
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("ADS122C04_getConversionData: excess bytes available. Maybe data integrity is enabled?"));
      }
      while (_i2cPort->available() > 0)
      {
        _i2cPort->read(); // Read and ignore excess bytes (presumably inverted data or CRC)
      }
    }
  }
  else
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("ADS122C04_getConversionData: requestFrom failed"));
    }
    return(false);
  }

  *conversionData = ((uint32_t)RXByte[2]) | ((uint32_t)RXByte[1]<<8) | ((uint32_t)RXByte[0]<<16);
  return(true);
}
