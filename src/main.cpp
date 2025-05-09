#include <main.h>

static float systemMaxVoltage = 1.7; // 默认值为1.7V，后续会动态更新

static float capacitorCalibLarge = 1.18;   // 800nF以上的校准系数
static float capacitorCalibMedium = 1.12;  // 380nF-800nF的校准系数
static float capacitorCalibSmall = 1.085;  // 380nF以下的校准系数

static float inductorCalib = 1.17;

void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */
    GPIO_init();
    display_init();
    encoder_init();
    lvgl_group_init();

    ADS122C04_init();
    SerialMaster_Init();
    ADS122C04_DATA_INIT();
}

void loop()
{
    lvgl_task_handler();
    SerialMaster_Update();
}

// 全局变量，用于标记是否需要执行ADS122C04_task()
volatile bool ads_task_flag = false;

// 定时器中断处理函数 - 在中断上下文中只设置标志
void timer_callback(uint alarm_num) {
    // 设置标志表示需要执行ADS122C04任务
    ads_task_flag = true;
    
    // 重新设置闹钟，使其每1ms触发一次
    hardware_alarm_set_target(alarm_num, make_timeout_time_ms(1));
}

void setup1()
{
    ADS122C04_init();
    
    // 初始化标志
    ads_task_flag = false;
    
    // 配置定时器中断
    // 声明使用硬件闹钟0
    hardware_alarm_claim(0);
    // 设置闹钟中断回调函数
    hardware_alarm_set_callback(0, timer_callback);
    // 设置首次触发时间为1ms后
    hardware_alarm_set_target(0, make_timeout_time_ms(1));
}

void loop1()
{
    // 检查标志，如果置位则执行任务并清除标志
    if(ads_task_flag) {
        ADS122C04_task();
        ads_task_flag = false;
    }
    

    switch (MeasureMode)
    {
    case 0: // 通断测试
        if (RUNSTOP == true)
        {
            if (AIN0_DC_Volt < 0.15)
            {
                BuzzerOn();
            }
            else
            {
                BuzzerOff();
            }
        }
        else
        {
            BuzzerOff();
        }
        break;

        case 1: // 元件测量
        if (RUNSTOP == true)
        {
            static unsigned long lastActionTime = 0;
            static int commandState = 0; // 0: 执行DC模式并等待前50ms, 1: 采集中间100ms数据, 2: 计算平均值并判断元件类型, 3: 执行相应模式并采集数据, 4: 计算元件值, 5: 等待延迟
            static float ain0Readings[512] = {0}; // 存储AIN0_DC_Volt的采样值
            static float ain1Readings[512] = {0}; // 存储AIN1_AC_Volt的采样值
            static int dcReadingIndex = 0; // AIN0采样索引
            static unsigned long modeStartTime = 0; // DC模式开始时间
            static unsigned long recordStartTime = 0; // 记录开始时间
            static float dcVoltAvg = 0.0; // 存储DC电压平均值
            static int componentType = 0; // 0: 未知, 1: 电阻, 2: 电容, 3: 电感
            static float componentValue = 0.0; // 元件值
            unsigned long currentTime = millis();
            unsigned long currentMicros = micros();

            static float internalResistance = 0.0; // 计算得到的内阻值
    
            switch (commandState) {
                case 0: // 执行DC模式并等待前50ms
                    LCRMode_AC_OR_DC_OR_LINE = 0; // 设置为DC模式
                    LCRMode_DC(); // 执行LCRMode_DC函数
                    modeStartTime = currentTime;
                    dcReadingIndex = 0; // 重置采样索引
                    commandState = 1; // 进入下一状态
                    
                    // 调试信息
                    Serial.println("元件测量：执行LCRMode_DC，等待前50ms...");
                    break;
                    
                case 1: // 采集中间100ms数据
                    if (currentTime - modeStartTime >= 50 && currentTime - modeStartTime < 150) {
                        // 仅在中间100ms采集数据
                        if (dcReadingIndex < 100) {
                            ain0Readings[dcReadingIndex] = AIN0_DC_Volt;
                            dcReadingIndex++;
                        }
                    } else if (currentTime - modeStartTime >= 150) {
                        commandState = 2; // 进入计算平均值状态
                    }
                    break;
    
                case 2: // 计算平均值并判断元件类型
                    // 使用花括号创建局部作用域
                    {
                        // 计算平均值
                        float sum = 0.0;
                        for (int i = 0; i < dcReadingIndex; i++) {
                            sum += ain0Readings[i];
                        }
                        dcVoltAvg = (dcReadingIndex > 0) ? (sum / dcReadingIndex) : 0.0;
                        
                        // 根据电压值判断元件类型
                        if (dcVoltAvg < 0.2) {
                            componentType = 3; // 电感
                            LCRMode_AC_OR_DC_OR_LINE = 1; // 设置为AC模式
                            LCRMode_AC(); // 执行LCRMode_AC函数
                            LCRMode_50(); // 执行LCRMode_50函数
                            Serial.print("检测到电感，平均电压: ");
                        } else if (dcVoltAvg > 0.8) {
                            componentType = 2; // 电容
                            LCRMode_AC_OR_DC_OR_LINE = 1; // 设置为AC模式
                            LCRMode_AC(); // 执行LCRMode_AC函数
                            LCRMode_50(); // 执行LCRMode_50函数
                            Serial.print("检测到电容，平均电压: ");
                        } else {
                            componentType = 1; // 电阻
                            LCRMode_AC_OR_DC_OR_LINE = 0; // 设置为DC模式
                            LCRMode_DC(); // 执行LCRMode_DC函数
                            LCRMode_50(); // 执行LCRMode_50函数
                            Serial.print("检测到电阻，平均电压: ");
                        }
                        Serial.println(dcVoltAvg, 6);
                    }
                    
                    // 记录时间并进入下一状态
                    recordStartTime = currentTime;
                    dcReadingIndex = 0; // 重置采样索引
                    commandState = 3; // 进入采集数据状态
                    Serial.println("等待100ms后开始采集数据...");
                    break;
                    
                case 3: // 等待100ms后采集数据
                    if (currentTime - recordStartTime >= 100) {
                        // 针对电阻进行采集和计算
                        if (componentType == 1) { // 电阻
                            // 在1000ms内采集128个AIN0_DC_Volt的值
                            if (currentTime - recordStartTime < 1100 && dcReadingIndex < 128) {
                                ain0Readings[dcReadingIndex] = AIN0_DC_Volt;
                                dcReadingIndex++;
                            } else {
                                commandState = 4; // 进入计算元件值状态
                            }
                        }
                        else if (componentType == 2) { // 电容
                            static float acVoltReadings[300] = {0}; // 减少到300个样本
                            static float frequencyValues[300] = {0}; // 相应减少频率值数组
                            static unsigned long sweepStartTime = 0; // 记录扫频开始时间
                            static unsigned long freqSetTime = 0; // 记录固定频率设置时间
                            static bool freqCommandSent = false; // 标记是否已发送固定频率命令
                            static bool sweepCommandSent = false; // 标记是否已发送扫频命令
                            static int acSampleIndex = 0; // 电压样本索引
                            
                            if (!freqCommandSent) {
                                // 先发送固定频率命令
                                SerialMaster_SendCommand("SET_FREQ 1 87 1023");
                                freqSetTime = currentTime;
                                freqCommandSent = true;
                                
                                Serial.println("电容测量：发送固定频率命令，等待200ms...");
                                return; // 直接返回，等待下一个循环
                            }
                            
                            // 等待200ms后再发送扫频命令
                            if (freqCommandSent && !sweepCommandSent && (currentTime - freqSetTime >= 200)) {
                                // 发送正确的扫频命令，频率从87Hz到50000Hz
                                SerialMaster_SendCommand("SET_SWEEP 1 87 50000 1023 2000 87");
                                sweepStartTime = currentTime;
                                sweepCommandSent = true;
                                acSampleIndex = 0;
                                
                                Serial.println("电容测量：发送扫频命令，开始采集数据...");
                            }
                            
                            // 在2000ms内采集样本，计算对应的频率值
                            if (sweepCommandSent && (currentTime - sweepStartTime < 2000)) {
                                if (acSampleIndex < 300) { // 减少最大样本数为300
                                    // 采用175SPS的采样率
                                    static unsigned long lastSampleTime = 0;
                                    unsigned long sampleInterval = 1000000 / 175; // 约5714微秒/样本
                                    
                                    if (currentMicros - lastSampleTime >= sampleInterval) {
                                        // 记录电压值
                                        acVoltReadings[acSampleIndex] = AIN1_AC_Volt;
                                        
                                        // 计算当前频率值 - 线性插值
                                        float elapsedRatio = (float)(currentTime - sweepStartTime) / 2000.0f;
                                        frequencyValues[acSampleIndex] = 87.0f + elapsedRatio * 49913.0f; // 87Hz到50000Hz
                                        
                                        acSampleIndex++;
                                        lastSampleTime = currentMicros;
                                    }
                                }
                            } else if (sweepCommandSent && (currentTime - sweepStartTime >= 2000) && acSampleIndex > 0) {
                                // 采样完成，转移数据到静态数组以便后续处理
                                for (int i = 0; i < acSampleIndex && i < 300; i++) {
                                    ain0Readings[i] = acVoltReadings[i];      // 重用ain0Readings存储电压值
                                    ain1Readings[i] = frequencyValues[i];     // 重用ain1Readings存储频率值
                                }
                                dcReadingIndex = acSampleIndex;  // 使用dcReadingIndex记录有效样本数
                                
                                Serial.print("电容测量：完成数据采集，共");
                                Serial.print(acSampleIndex);
                                Serial.println("个样本");
                                
                                // 重置标志，进入计算状态
                                freqCommandSent = false;
                                sweepCommandSent = false;
                                commandState = 4;
                            }
                        }
                        else if (componentType == 3) { // 电感
                            static float acVoltReadings[300] = {0}; // 用于存储电压值
                            static float frequencyValues[300] = {0}; // 用于存储频率值
                            static unsigned long sweepStartTime = 0; // 记录扫频开始时间
                            static unsigned long freqSetTime = 0; // 记录固定频率设置时间
                            static bool freqCommandSent = false; // 标记是否已发送固定频率命令
                            static bool sweepCommandSent = false; // 标记是否已发送扫频命令
                            static int acSampleIndex = 0; // 电压样本索引
                            static bool resistanceMeasured = false; // 标记是否已测量内阻
                            static unsigned long resistanceMeasureStartTime = 0; // 内阻测量开始时间
                            static float resistanceReadings[100] = {0}; // 存储内阻测量的电压值
                            static int resistanceReadingIndex = 0; // 内阻读数索引
                            
                            if (!freqCommandSent) {
                                // 先发送固定频率命令
                                SerialMaster_SendCommand("SET_FREQ 1 1000 1023");
                                freqSetTime = currentTime;
                                freqCommandSent = true;
                                
                                Serial.println("电感测量：发送固定频率命令，等待200ms...");
                                return; // 直接返回，等待下一个循环
                            }
                            
                            // 等待200ms后再发送扫频命令
                            if (freqCommandSent && !sweepCommandSent && (currentTime - freqSetTime >= 200)) {
                                // 发送扫频命令，频率从1000Hz到200000Hz
                                SerialMaster_SendCommand("SET_SWEEP 1 1000 200000 1023 2000 1000");
                                sweepStartTime = currentTime;
                                sweepCommandSent = true;
                                acSampleIndex = 0;
                                
                                Serial.println("电感测量：发送扫频命令，开始采集数据...");
                            }
                            
                            // 在2000ms内采集样本，计算对应的频率值
                            if (sweepCommandSent && !resistanceMeasured && (currentTime - sweepStartTime < 2000)) {
                                if (acSampleIndex < 300) {
                                    // 采用175SPS的采样率
                                    static unsigned long lastSampleTime = 0;
                                    unsigned long sampleInterval = 1000000 / 175; // 约5714微秒/样本
                                    
                                    if (currentMicros - lastSampleTime >= sampleInterval) {
                                        // 记录电压值
                                        acVoltReadings[acSampleIndex] = AIN1_AC_Volt;
                                        
                                        // 计算当前频率值 - 线性插值
                                        float elapsedRatio = (float)(currentTime - sweepStartTime) / 2000.0f;
                                        frequencyValues[acSampleIndex] = 1000.0f + elapsedRatio * 199000.0f; // 1000Hz到200000Hz
                                        
                                        acSampleIndex++;
                                        lastSampleTime = currentMicros;
                                    }
                                }
                            } 
                            else if (sweepCommandSent && !resistanceMeasured && (currentTime - sweepStartTime >= 2000) && acSampleIndex > 0) {
                                // 扫频采样完成，开始测量内阻
                                Serial.print("电感测量：扫频完成，共采集");
                                Serial.print(acSampleIndex);
                                Serial.println("个样本点。开始测量内阻...");
                                LCRMode_AC_OR_DC_OR_LINE = 2; // 设置为LINE模式
                                // 执行LineMode_ON()切换到测内阻模式
                                LineMode_ON();
                                resistanceMeasureStartTime = currentTime;
                                resistanceReadingIndex = 0;
                                resistanceMeasured = true;
                            }
                            
                            // 内阻测量过程
                            if (resistanceMeasured && (currentTime - resistanceMeasureStartTime < 200)) {
                                // 只记录中间100ms的数据
                                if (currentTime - resistanceMeasureStartTime >= 50 && currentTime - resistanceMeasureStartTime < 150) {
                                    if (resistanceReadingIndex < 100) {
                                        resistanceReadings[resistanceReadingIndex] = AIN2_Line_Volt;
                                        resistanceReadingIndex++;
                                    }
                                }
                            }
                            else if (resistanceMeasured && (currentTime - resistanceMeasureStartTime >= 200)) {
                                // 内阻测量完成，执行LineMode_OFF()
                                LineMode_OFF();
                                LCRMode_AC_OR_DC_OR_LINE = 1;
                                // 计算内阻测量的平均值
                                float sum = 0.0;
                                for (int i = 0; i < resistanceReadingIndex; i++) {
                                    sum += resistanceReadings[i];
                                }
                                float avgVoltage = (resistanceReadingIndex > 0) ? (sum / resistanceReadingIndex) : 0.0;
                                
                                // 内阻电压与电阻值的拟合函数
                                // 使用线性拟合: R(mΩ) = a*V + b
                                // 根据提供的数据点拟合曲线:
                                // 10mΩ - 0.592V, 20mΩ - 0.696V, 30mΩ - 0.805V, 
                                // 40mΩ - 0.905V, 50mΩ - 1.002V, 60mΩ - 1.093V
                                const float a = 99.78; // 斜率
                                const float b = -49.07; // 截距
                                
                                // 计算内阻值(毫欧)
                                internalResistance = a * avgVoltage + b;
                                
                                // 确保内阻值在合理范围内
                                if (internalResistance < 0) internalResistance = 0;
                                
                                Serial.print("电感内阻测量完成，平均电压: ");
                                Serial.print(avgVoltage, 4);
                                Serial.print("V，计算得到内阻: ");
                                Serial.print(internalResistance, 2);
                                Serial.println("mΩ");
                                
                                // 转移扫频数据到静态数组以便后续处理
                                for (int i = 0; i < acSampleIndex && i < 300; i++) {
                                    ain0Readings[i] = acVoltReadings[i];      // 重用ain0Readings存储电压值
                                    ain1Readings[i] = frequencyValues[i];     // 重用ain1Readings存储频率值
                                }
                                dcReadingIndex = acSampleIndex;  // 使用dcReadingIndex记录有效样本数
                                
                                // 重置标志，进入计算状态
                                freqCommandSent = false;
                                sweepCommandSent = false;
                                resistanceMeasured = false;
                                commandState = 4; // 进入计算元件值状态
                            }
                        }
                    }
                    break;
                    
                case 4: // 计算元件值
                    if (componentType == 1) { // 电阻
                        // 计算电阻采样的平均值
                        float sum = 0.0;
                        for (int i = 0; i < dcReadingIndex; i++) {
                            sum += ain0Readings[i];
                        }
                        float avgVoltage = (dcReadingIndex > 0) ? (sum / dcReadingIndex) : 0.0;
                        
                        // 通过分压公式计算电阻值: R = R1 * (V2 / (VCC - V2))
                        // 其中R1是串联的1.0千欧姆电阻，V2是测得电压
                        if (avgVoltage > 0.01 && avgVoltage < 1.0) { // 避免除以零或极小的值
                            componentValue = 1000.0 * avgVoltage / (0.997 - avgVoltage); // 1000欧姆是参考电阻值
                            
                            LCRModeType = 1; // 设置为电阻模式
                            LCRMode_Value = componentValue; // 更新LCRMode_Value
                            // 输出计算结果
                            Serial.print("电阻值: ");
                            Serial.print(componentValue, 2);
                            Serial.println(" 欧姆");
                        } else {
                            Serial.println("电压异常，无法准确计算电阻值");
                        }
                    }
                    else if (componentType == 2) { // 电容
                        // 打印采集到的电容数据基本信息
                        Serial.print("电容测量：采集完成，");
                        Serial.print(dcReadingIndex);
                        Serial.println("个采样点已保存");
                        
                        // 计算电压稳定性，检测是否为开路
                        float maxVoltage = 0.0;
                        float minVoltage = systemMaxVoltage;
                        float sumVoltage = 0.0;
                        float voltageVariance = 0.0;
                        
                        // 计算所有采样点的最大值、最小值和平均值
                        for (int i = 0; i < dcReadingIndex; i++) {
                            if (ain0Readings[i] > maxVoltage) {
                                maxVoltage = ain0Readings[i];
                            }
                            if (ain0Readings[i] < minVoltage) {
                                minVoltage = ain0Readings[i];
                            }
                            sumVoltage += ain0Readings[i];
                        }
                        
                        // 计算平均值和方差
                        float avgVoltage = (dcReadingIndex > 0) ? (sumVoltage / dcReadingIndex) : 0.0;
                        for (int i = 0; i < dcReadingIndex; i++) {
                            voltageVariance += pow(ain0Readings[i] - avgVoltage, 2);
                        }
                        voltageVariance /= dcReadingIndex;
                        
                        // 计算电压波动率（最大最小值之差与平均值之比）
                        float voltageFluctuation = (maxVoltage - minVoltage) / avgVoltage;
                        
                        // 输出详细的电压统计信息
                        Serial.print("电压统计：最大=");
                        Serial.print(maxVoltage, 6);
                        Serial.print("V，最小=");
                        Serial.print(minVoltage, 6);
                        Serial.print("V，平均=");
                        Serial.print(avgVoltage, 6);
                        Serial.print("V (系统最大电压的");
                        Serial.print(avgVoltage / systemMaxVoltage * 100);
                        Serial.println("%)");
                        Serial.print("电压方差: ");
                        Serial.print(voltageVariance, 8);
                        Serial.print("，波动率: ");
                        Serial.print(voltageFluctuation * 100, 2);
                        Serial.println("%");
                        
                        // 强化开路判断条件 - 增加多个判断标准
                        bool isVoltageHigh = avgVoltage > systemMaxVoltage * 0.90; // 平均电压接近系统最大电压
                        bool isVarianceLow = voltageVariance < 0.0002; // 放宽方差阈值
                        bool isFluctuationLow = voltageFluctuation < 0.05; // 波动小于5%
                        bool isOpenCircuit = isVoltageHigh && (isVarianceLow || isFluctuationLow);
                        
                        if (isOpenCircuit) {
                            // 设置为开路状态
                            LCRModeType = 0;
                            LCRMode_Value = 0;
                            
                            Serial.println("检测到开路状态，电压稳定在系统最大电压附近");
                            Serial.print("平均电压: ");
                            Serial.print(avgVoltage, 6);
                            Serial.print("V (系统最大电压的");
                            Serial.print(avgVoltage / systemMaxVoltage * 100);
                            Serial.println("%)");
                            Serial.print("电压方差: ");
                            Serial.println(voltageVariance, 8);
                        } else {
                            // 继续计算电容值 - 找到电压等于空载电压的1/√2倍的点
                            const float R3 = 50.0; // 基准电阻值（欧姆）
                            float targetVoltage = systemMaxVoltage / sqrt(2); // 目标电压值
                            float closestDiff = 1000.0; // 初始化为一个较大值
                            float resonanceFreq = 0.0; // 计算用的频率
                            int closestIndex = -1; // 最接近目标值的索引
                            
                            Serial.print("系统最大电压: ");
                            Serial.print(systemMaxVoltage, 6);
                            Serial.print("V，目标电压(1/√2倍): ");
                            Serial.print(targetVoltage, 6);
                            Serial.println("V");
                            
                            // 遍历所有采样点，寻找最接近目标电压的点
                            for (int i = 0; i < dcReadingIndex; i++) {
                                float diff = fabs(ain0Readings[i] - targetVoltage);
                                if (diff < closestDiff) {
                                    closestDiff = diff;
                                    closestIndex = i;
                                    resonanceFreq = ain1Readings[i]; // 对应频率
                                }
                            }
                            
                            // 附加开路检测 - 检查电压点是否始终高于目标值
                            bool allPointsAboveTarget = true;
                            for (int i = 0; i < dcReadingIndex; i++) {
                                if (ain0Readings[i] < targetVoltage * 1.1) { // 允许10%偏差
                                    allPointsAboveTarget = false;
                                    break;
                                }
                            }
                            
                            // 如果最接近点是初始点(索引0或1)且高于目标电压，可能是开路
                            bool firstPointClosest = (closestIndex <= 1) && (ain0Readings[closestIndex] > targetVoltage);
                            
                            if (allPointsAboveTarget || (firstPointClosest && voltageFluctuation < 0.08)) {
                                // 设置为开路状态
                                LCRModeType = 0;
                                LCRMode_Value = 0;
                                
                                Serial.println("检测到可能的开路状态：所有电压点均高于目标值或初始点最接近");
                                Serial.print("最近点电压: ");
                                Serial.print(ain0Readings[closestIndex], 6);
                                Serial.print("V (索引: ");
                                Serial.print(closestIndex);
                                Serial.print(")，与目标差距: ");
                                Serial.print(closestDiff, 6);
                                Serial.println("V");
                            }
                            // 判断找到的点是否足够接近目标电压
                            else if (closestIndex >= 0 && resonanceFreq > 0.0 && closestDiff < targetVoltage * 0.3) { // 允许30%的偏差
                                // 尝试通过线性插值获得更精确的频率值
                                if (closestIndex > 0 && closestIndex < dcReadingIndex - 1) {
                                    // 查找相邻两点，一个低于目标电压，一个高于目标电压
                                    int lowerIndex = -1;
                                    int upperIndex = -1;
                                    
                                    for (int i = 0; i < dcReadingIndex - 1; i++) {
                                        if ((ain0Readings[i] <= targetVoltage && ain0Readings[i+1] >= targetVoltage) ||
                                            (ain0Readings[i] >= targetVoltage && ain0Readings[i+1] <= targetVoltage)) {
                                            lowerIndex = i;
                                            upperIndex = i + 1;
                                            break;
                                        }
                                    }
                                    
                                    // 如果找到合适的两点，执行线性插值
                                    if (lowerIndex >= 0 && upperIndex >= 0) {
                                        float v1 = ain0Readings[lowerIndex];
                                        float v2 = ain0Readings[upperIndex];
                                        float f1 = ain1Readings[lowerIndex];
                                        float f2 = ain1Readings[upperIndex];
                                        
                                        // 通过线性插值计算目标电压对应的频率
                                        if (v2 != v1) {
                                            resonanceFreq = f1 + (f2 - f1) * (targetVoltage - v1) / (v2 - v1);
                                            Serial.println("使用线性插值计算频率");
                                        }
                                    }
                                }
                                 
                                // 计算电容值（法拉）C = 1/(2πfR3)
                                componentValue = 1.0f / (2.0f * PI * resonanceFreq * R3);
                                
                                // 应用分段校准系数
                                float nanoFarads = componentValue * 1e9; // 转换为nF便于判断范围
                                float calibrationFactor = 1.0;
                                
                                // 根据电容值大小选择不同的校准系数
                                if (nanoFarads > 800.0) {
                                    calibrationFactor = capacitorCalibLarge;   // > 800nF
                                    Serial.print("应用大电容校准系数: ");
                                } else if (nanoFarads > 380.0) {
                                    calibrationFactor = capacitorCalibMedium;  // 380nF - 800nF
                                    Serial.print("应用中电容校准系数: ");
                                } else {
                                    calibrationFactor = capacitorCalibSmall;   // ≤ 380nF
                                    Serial.print("应用小电容校准系数: ");
                                }
                                Serial.println(calibrationFactor, 3);
                                
                                // 应用校准系数
                                componentValue *= calibrationFactor;
                                
                                // 转换为更合适的单位显示
                                float displayValue = 0.0;
                                
                                if (componentValue < 1e-9) {
                                    displayValue = componentValue * 1e12;  // 转换为pF
                                    capacitanceUnit = "pF";
                                } else if (componentValue < 1e-6) {
                                    displayValue = componentValue * 1e9;   // 转换为nF
                                    capacitanceUnit = "nF";
                                } else if (componentValue < 1e-3) {
                                    displayValue = componentValue * 1e6;   // 转换为μF
                                    capacitanceUnit = "uF";
                                } else {
                                    displayValue = componentValue * 1e3;   // 转换为mF
                                    capacitanceUnit = "mF";
                                }
                                
                                // 设置电容测量结果
                                LCRModeType = 2;  // 设置为电容模式
                                LCRMode_Value = displayValue;  // 存储转换后的电容值
                                
                                Serial.print("最近点电压: ");
                                Serial.print(ain0Readings[closestIndex], 6);
                                Serial.print("V (索引: ");
                                Serial.print(closestIndex);
                                Serial.print(")，对应频率: ");
                                Serial.print(resonanceFreq);
                                Serial.println("Hz");
                                
                                Serial.print("电容值: ");
                                Serial.print(displayValue, 3);
                                Serial.print(" ");
                                Serial.println(capacitanceUnit);
                                Serial.print("LCRModeType已设置为: ");
                                Serial.print(LCRModeType);
                                Serial.print(", LCRMode_Value已设置为: ");
                                Serial.println(LCRMode_Value, 3);
                            } else {
                                // 未找到合适的点计算电容值，设置为开路状态
                                LCRModeType = 0;
                                LCRMode_Value = 0;
                                
                                Serial.println("未找到有效的测量点或测量点偏差过大，无法计算电容值");
                                Serial.println("设置为开路状态");
                                Serial.print("最近点电压: ");
                                if (closestIndex >= 0) {
                                    Serial.print(ain0Readings[closestIndex], 6);
                                    Serial.print("V (索引: ");
                                    Serial.print(closestIndex);
                                    Serial.print(")，差值: ");
                                    Serial.print(closestDiff, 6);
                                    Serial.print("V，对应频率: ");
                                    Serial.print(resonanceFreq);
                                    Serial.println("Hz");
                                } else {
                                    Serial.println("无有效测量点");
                                }
                            }
                        }
                        
                        // 打印部分采样数据用于分析（前5个和后5个点）
                        Serial.println("采样数据片段 (索引, 频率, 电压):");
                        
                        // 打印前5个点
                        int printCount = min(10, dcReadingIndex);
                        for (int i = 0; i < printCount; i++) {
                            Serial.print(i);
                            Serial.print(", ");
                            Serial.print(ain1Readings[i], 1);
                            Serial.print("Hz, ");
                            Serial.println(ain0Readings[i], 6);
                        }
                        
                        // 打印后5个点
                        if (dcReadingIndex > 10) {
                            int endStart = max(dcReadingIndex - 5, 0);
                            Serial.println("...");
                            for (int i = endStart; i < dcReadingIndex; i++) {
                                Serial.print(i);
                                Serial.print(", ");
                                Serial.print(ain1Readings[i], 1);
                                Serial.print("Hz, ");
                                Serial.println(ain0Readings[i], 6);
                            }
                        }
                    }
                    else if (componentType == 3) { // 电感
                        // 打印采集到的电感数据基本信息
                        Serial.print("电感测量：采集完成，");
                        Serial.print(dcReadingIndex);
                        Serial.println("个采样点已保存");
                        
                        // 计算电压相关统计数据
                        float maxVoltage = 0.0;
                        float minVoltage = systemMaxVoltage;
                        float sumVoltage = 0.0;
                        
                        // 计算所有采样点的最大值、最小值和平均值
                        for (int i = 0; i < dcReadingIndex; i++) {
                            if (ain0Readings[i] > maxVoltage) {
                                maxVoltage = ain0Readings[i];
                            }
                            if (ain0Readings[i] < minVoltage) {
                                minVoltage = ain0Readings[i];
                            }
                            sumVoltage += ain0Readings[i];
                        }
                        
                        // 计算平均值
                        float avgVoltage = (dcReadingIndex > 0) ? (sumVoltage / dcReadingIndex) : 0.0;
                        
                        // 输出基本电压统计信息
                        Serial.print("电压统计：最大=");
                        Serial.print(maxVoltage, 6);
                        Serial.print("V，最小=");
                        Serial.print(minVoltage, 6);
                        Serial.print("V，平均=");
                        Serial.print(avgVoltage, 6);
                        Serial.println("V");
                        
                        // 寻找满足 (电感分压/系统参考电压) = (根号二/4) 的频率
                        float targetRatio = 0.3535534; // 根号二/4 ≈ 0.3535534
                        float targetVoltage = systemMaxVoltage * targetRatio;
                        float closestDiff = 1000.0; // 初始化为一个较大值
                        float resonanceFreq = 0.0; // 找到的频率
                        int closestIndex = -1; // 最接近目标值的索引
                        
                        Serial.print("系统最大电压: ");
                        Serial.print(systemMaxVoltage, 6);
                        Serial.print("V，目标电压(根号二/4倍): ");
                        Serial.print(targetVoltage, 6);
                        Serial.println("V");
                        
                        // 遍历所有采样点，寻找最接近目标电压的点
                        for (int i = 0; i < dcReadingIndex; i++) {
                            float diff = fabs(ain0Readings[i] - targetVoltage);
                            if (diff < closestDiff) {
                                closestDiff = diff;
                                closestIndex = i;
                                resonanceFreq = ain1Readings[i]; // 对应频率
                            }
                        }
                        
                        // 判断找到的点是否足够接近目标电压
                        if (closestIndex >= 0 && resonanceFreq > 0.0 && closestDiff < targetVoltage * 0.3) { // 允许30%的偏差
                            // 尝试通过线性插值获得更精确的频率值
                            if (closestIndex > 0 && closestIndex < dcReadingIndex - 1) {
                                // 查找相邻两点，一个低于目标电压，一个高于目标电压
                                int lowerIndex = -1;
                                int upperIndex = -1;
                                
                                for (int i = 0; i < dcReadingIndex - 1; i++) {
                                    if ((ain0Readings[i] <= targetVoltage && ain0Readings[i+1] >= targetVoltage) ||
                                        (ain0Readings[i] >= targetVoltage && ain0Readings[i+1] <= targetVoltage)) {
                                        lowerIndex = i;
                                        upperIndex = i + 1;
                                        break;
                                    }
                                }
                                
                                // 如果找到合适的两点，执行线性插值
                                if (lowerIndex >= 0 && upperIndex >= 0) {
                                    float v1 = ain0Readings[lowerIndex];
                                    float v2 = ain0Readings[upperIndex];
                                    float f1 = ain1Readings[lowerIndex];
                                    float f2 = ain1Readings[upperIndex];
                                    
                                    // 通过线性插值计算目标电压对应的频率
                                    if (v2 != v1) {
                                        resonanceFreq = f1 + (f2 - f1) * (targetVoltage - v1) / (v2 - v1);
                                        Serial.println("使用线性插值计算频率");
                                    }
                                }
                            }
                            
                            // 将内阻从毫欧转换为欧姆
                            float RS = internalResistance / 1000.0;
                            // 基准电阻
                            const float R3 = 50.0; // 欧姆
                            
                            // 计算电感值（亨利）
                            // L = 根号下[(R3^2 + 2*R3*RS - 7*(RS)^2) / 7*(2*PI*f)^2]
                            double numerator = R3*R3 + 2*R3*RS - 7*RS*RS;
                            double omega = 2 * PI * resonanceFreq;
                            double denominator = 7 * omega * omega;
                            
                            float inductanceValue = 0.0;
                            
                            // 检查分子是否为正数（防止计算复数）
                            if (numerator > 0 && denominator > 0) {
                                inductanceValue = 2 * sqrt(numerator / denominator) * inductorCalib;
                                
                                // 转换为更合适的单位显示
                                float displayValue = 0.0;
                                inductanceUnit = "H";
                                
                                if (inductanceValue < 1e-6) {
                                    displayValue = inductanceValue * 1e9;  // 转换为nH
                                    inductanceUnit = "nH";
                                } else if (inductanceValue < 1e-3) {
                                    displayValue = inductanceValue * 1e6;  // 转换为μH
                                    inductanceUnit = "uH";
                                } else if (inductanceValue < 1) {
                                    displayValue = inductanceValue * 1e3;  // 转换为mH
                                    inductanceUnit = "mH";
                                } else {
                                    displayValue = inductanceValue;        // 保持H单位
                                }
                                
                                // 设置电感测量结果
                                LCRModeType = 3;  // 设置为电感模式
                                LCRMode_Value = displayValue;  // 存储转换后的电感值
                                inductanceUnit = inductanceUnit; // 存储单位（假设全局变量已声明）
                                
                                Serial.print("最近点电压: ");
                                Serial.print(ain0Readings[closestIndex], 6);
                                Serial.print("V (索引: ");
                                Serial.print(closestIndex);
                                Serial.print(")，对应频率: ");
                                Serial.print(resonanceFreq);
                                Serial.println("Hz");
                                
                                Serial.print("内阻值: ");
                                Serial.print(RS, 6);
                                Serial.println("Ω");
                                
                                Serial.print("计算参数: 分子=");
                                Serial.print(numerator, 6);
                                Serial.print(", 分母=");
                                Serial.println(denominator, 6);
                                
                                Serial.print("电感值: ");
                                Serial.print(displayValue, 3);
                                Serial.print(" ");
                                Serial.println(inductanceUnit);
                                
                                Serial.print("LCRModeType已设置为: ");
                                Serial.print(LCRModeType);
                                Serial.print(", LCRMode_Value已设置为: ");
                                Serial.println(LCRMode_Value, 3);
                            } else {
                                // 无法计算（可能分子为负）
                                LCRModeType = 0;  // 设置为开路状态
                                LCRMode_Value = 0;
                                
                                Serial.println("电感计算参数无效，无法计算电感值");
                                Serial.print("计算参数: 分子=");
                                Serial.print(numerator, 6);
                                Serial.print(", 分母=");
                                Serial.println(denominator, 6);
                                Serial.print("内阻值: ");
                                Serial.print(RS, 6);
                                Serial.println("Ω");
                            }
                        } else {
                            // 未找到合适的点计算电感值，设置为开路状态
                            LCRModeType = 0;
                            LCRMode_Value = 0;
                            
                            Serial.println("未找到有效的测量点或测量点偏差过大，无法计算电感值");
                            Serial.println("设置为开路状态");
                        }
                        
                        // 打印部分采样数据用于分析
                        Serial.println("采样数据片段 (索引, 频率, 电压):");
                        
                        // 打印前10个点
                        int printCount = min(10, dcReadingIndex);
                        for (int i = 0; i < printCount; i++) {
                            Serial.print(i);
                            Serial.print(", ");
                            Serial.print(ain1Readings[i], 1);
                            Serial.print("Hz, ");
                            Serial.println(ain0Readings[i], 6);
                        }
                        
                        // 打印后5个点
                        if (dcReadingIndex > 10) {
                            int endStart = max(dcReadingIndex - 5, 0);
                            Serial.println("...");
                            for (int i = endStart; i < dcReadingIndex; i++) {
                                Serial.print(i);
                                Serial.print(", ");
                                Serial.print(ain1Readings[i], 1);
                                Serial.print("Hz, ");
                                Serial.println(ain0Readings[i], 6);
                            }
                        }
                    }
                    
                    // 清空存储的采样数据
                    for (int i = 0; i < 512; i++) {
                        ain0Readings[i] = 0.0;
                    }
                    
                    // 记录时间并进入等待状态
                    lastActionTime = currentTime;
                    commandState = 5; // 进入等待延迟状态
                    Serial.println("等待500毫秒后重新开始...");
                    break;
                    
                case 5: // 延迟等待状态
                    // 等待500毫秒后回到初始状态
                    if (currentTime - lastActionTime >= 500) {
                        commandState = 0; // 回到初始状态，重新执行LCRMode_DC
                        Serial.println("-----等待500毫秒后重新开始-----");
                    }
                    break;
            }
        }
        break;

    case 2: // 负载网络
    if (RUNSTOP == true)
    {
        // 如果连接已建立，可以发送命令
        if (SerialMaster_IsConnected()) {
        static unsigned long lastActionTime = 0;
        static int commandState = 0; // 0: 执行DC模式并等待前50ms, 1: 采集中间100ms数据, 2: 等待后50ms并计算平均值, 3: 记录数据, 4: 打印数据, 5: 等待延迟
        static float ain0Readings[100] = {0}; // 存储AIN0_DC_Volt的采样值
        static float ain1Readings[256] = {0}; // 存储AIN1_AC_Volt的采样值
        static int maxSamples = 256; // 目标采样数
        static int dcReadingIndex = 0; // AIN0采样索引
        static int acReadingIndex = 0; // AIN1采样索引
        static unsigned long modeStartTime = 0; // DC模式开始时间
        static unsigned long recordStartTime = 0; // 记录开始时间
        static unsigned long lastSampleTime = 0; // 上次采样时间
        static int selectedMode = 0; // 选择的运行模式
        static float dcVoltAvg = 0.0; // 存储DC电压平均值
        unsigned long currentTime = millis();
        unsigned long currentMicros = micros();

        switch (commandState) {
        case 0: // 执行DC模式并等待前50ms
            LoadMode_AC_OR_DC = false; // 设置为DC模式
            LoadMode_DC(); // 执行LoadMode_DC函数
            modeStartTime = currentTime;
            dcReadingIndex = 0; // 重置采样索引
            commandState = 1; // 进入下一状态
            
            // 调试信息
            Serial.print("测量的系统参考电压: ");
            Serial.println(systemMaxVoltage, 4);
            Serial.println("执行LoadMode_DC，等待前50ms...");
            break;
            
        case 1: // 采集中间100ms数据
            if (currentTime - modeStartTime >= 50 && currentTime - modeStartTime < 150) {
                // 仅在中间100ms采集数据
                if (dcReadingIndex < 100) {
                    ain0Readings[dcReadingIndex] = AIN0_DC_Volt;
                    dcReadingIndex++;
                }
            } else if (currentTime - modeStartTime >= 150) {
                commandState = 2; // 进入计算平均值状态
            }
            break;

        case 2: //执行LoadMode_AC，计算平均值
            LoadMode_AC_OR_DC = true; // 设置为AC模式
                            
            // 使用花括号创建局部作用域
            {
                // 计算平均值
                float sum = 0.0;
                for (int i = 0; i < dcReadingIndex; i++) {
                    sum += ain0Readings[i];
                }
                dcVoltAvg = (dcReadingIndex > 0) ? (sum / dcReadingIndex) : 0.0;
            }
            LoadMode_AC(); // 执行LoadMode_AC()
            lastActionTime = currentTime; // 记录上次操作时间
            commandState = 3;
            break;
            
            case 3: // 等待300毫秒，选择模式
                if (currentTime - modeStartTime >= 500) { // 完成300ms DC模式
                    static int subState = 0; // 子状态：0-选择模式, 1-等待稳定, 2-发送命令
                    static unsigned long modeSelectTime = 0; // 模式选择的时间点
                    
                    if (subState == 0) {
                        // 根据平均值选择模式并设置参数 - 使用动态参考电压
                        if (dcVoltAvg < systemMaxVoltage * 0.06) { // 约6%的系统参考电压
                            selectedMode = 1;
                            LoadMode_50(); // 执行LoadMode_50
                            Serial.print("进入模式1，平均电压: ");
                        } else if (dcVoltAvg > systemMaxVoltage * 0.56) { // 约65%的系统参考电压
                            selectedMode = 3;
                            LoadMode_1K(); // 执行LoadMode_1K
                            Serial.print("进入模式3，平均电压: ");
                        } else {
                            selectedMode = 2;
                            LoadMode_1K(); // 执行LoadMode_1K
                            Serial.print("进入模式2，平均电压: ");
                        }
                        
                        // 增加比例信息输出，便于调试
                        Serial.println(dcVoltAvg, 4);
                        Serial.print("(相对比例: "); 
                        Serial.print(dcVoltAvg / systemMaxVoltage * 100.0, 1);
                        Serial.println("%)");
                        Serial.print("采样点数: ");
                        Serial.println(dcReadingIndex);
                        
                        // 记录模式选择的时间并进入等待稳定状态
                        modeSelectTime = currentTime;
                        subState = 1;
                        Serial.println("等待200ms系统稳定...");
                    }
                    else if (subState == 1 && (currentTime - modeSelectTime >= 200)) {
                        // 等待200ms后发送扫频命令
                        switch (selectedMode) {
                            case 1:
                                SerialMaster_SendCommand("SET_SWEEP 1 1000 100000 1023 2000 87");
                                Serial.println("发送模式1扫频命令: 1000-100000Hz");
                                break;
                            case 2:
                                SerialMaster_SendCommand("SET_SWEEP 1 87 2000000 1023 1000 87");
                                Serial.println("发送模式2扫频命令: 87-2000000Hz");
                                break;
                            case 3:
                                SerialMaster_SendCommand("SET_SWEEP 1 87 1000000 1023 1000 2000000");
                                Serial.println("发送模式3扫频命令: 87-1000000Hz");
                                break;
                        }
                        
                        // 开始记录AC电压值
                        recordStartTime = currentTime;
                        acReadingIndex = 0;
                        lastSampleTime = currentMicros;
                        commandState = 4; // 进入记录数据状态
                        subState = 0; // 重置子状态以备下次使用
                    }
                }
                break;
            
        case 4: // 记录AIN1_AC_Volt值
            // 时间窗口内持续采样 - 模式1需要2000ms
            if (currentTime - recordStartTime < (selectedMode == 1 ? 2000 : 1000)) {
                // 计算采样间隔(微秒)，保持175SPS的采样率
                unsigned long sampleInterval = 1000000 / 175;
                
                // 每次循环尝试多次采样，提高采样效率
                for (int i = 0; i < 10; i++) {
                    // 为模式2和模式3限制最大采样点数为128
                    int currentMaxSamples = (selectedMode == 1) ? maxSamples : 128;
                    
                    if (acReadingIndex < currentMaxSamples && 
                        (currentMicros - lastSampleTime >= sampleInterval)) {
                        ain1Readings[acReadingIndex] = AIN1_AC_Volt; // 记录电压值
                        acReadingIndex++;
                        lastSampleTime = currentMicros; // 更新上次采样时间
                    }
                    currentMicros = micros(); // 更新当前微秒时间
                }
            } else {
                // 采样时间窗口结束
                commandState = 5;
            }
            break;
            
        case 5: // 打印记录的值
            // 打印表头
            Serial.println("Index, Time(ms), AIN1_AC_Volt");
            
            // 打印所有记录的值及时间戳
            for (int i = 0; i < acReadingIndex; i++) {
                Serial.print(i);
                Serial.print(", ");
                Serial.print(i * 1000.0 / 175.0, 2); // 基于175SPS的理论采样率计算时间戳
                Serial.print(", ");
                Serial.println(ain1Readings[i], 6);
            }
            
            // 根据不同模式判断负载网络类型
            if (acReadingIndex >= 20) { // 确保有足够的数据点
                // 根据selectedMode选择不同的判断逻辑
                switch (selectedMode) {
                    case 1: // 模式1 (dcVoltAvg < 0.1)
                                        {
                                            // 计算基本统计量
                                            float maxVoltage = 0.0;
                                            float maxVoltageIndex = 0;
                                            float sumVoltage = 0.0;
                                            float endToStartRatio = 0.0;
                                            bool hasPeak = false;
                                            
                                            // 查找最大电压值及其位置
                                            for (int i = 0; i < acReadingIndex; i++) {
                                                if (ain1Readings[i] > maxVoltage) {
                                                    maxVoltage = ain1Readings[i];
                                                    maxVoltageIndex = i;
                                                }
                                                sumVoltage += ain1Readings[i];
                                            }
                                            
                                            // 计算各类特征参数
                                            float avgVoltage = sumVoltage / acReadingIndex;
                                            
                                            // 计算初始和末尾电压的比值，用于判断RL并联
                                            float startAvg = 0.0;
                                            float endAvg = 0.0;
                                            int sampleSize = min(20, acReadingIndex/8); // 使用前后20个点或1/8数据点
                                            
                                            for (int i = 0; i < sampleSize; i++) {
                                                startAvg += ain1Readings[i];
                                            }
                                            startAvg /= sampleSize;
                                            
                                            for (int i = acReadingIndex - sampleSize; i < acReadingIndex; i++) {
                                                endAvg += ain1Readings[i];
                                            }
                                            endAvg /= sampleSize;
                                            
                                            endToStartRatio = endAvg / startAvg;
                                            
                                            // 修改判断是否有峰值的条件 - 增强峰值检测能力
                                            hasPeak = (maxVoltageIndex > acReadingIndex * 0.03) && 
                                                     (maxVoltageIndex < acReadingIndex * 0.9) && 
                                                     (maxVoltage > startAvg * 1.25) && 
                                                     (maxVoltage > endAvg * 1.25);
                                            
                                            // 计算频率映射参数 - 在2000ms内从1000Hz线性变化到100000Hz
                                            float timePerSample = 1000.0 / 175.0; // 每个样本的时间间隔(ms)
                                            float freqRatio = 99000.0 / 2000.0; // 每ms频率增长率 (Hz/ms)
                                            
                                            // === 多特征分析代码 ===
                                            
                                            // 1. 计算Q值和峰值分析
                                            float qValue = 0.0;
                                            float resonanceFreq = 0.0;
                                            float peakWidth = 0.0;
                                            float peakArea = 0.0;
                                            float curveAsymmetry = 0.0;
                                            float decayRate = 0.0;
                                            
                                            if (hasPeak) {
                                                // 计算谐振频率（对应最大电压时刻）
                                                resonanceFreq = 1000.0 + (maxVoltageIndex * timePerSample) * freqRatio;
                                                
                                                // 查找左右半功率点
                                                float halfPowerVoltage = maxVoltage * 0.707;
                                                int leftIndex = -1;
                                                int rightIndex = -1;
                                                
                                                for (int i = 0; i < maxVoltageIndex; i++) {
                                                    if (ain1Readings[i] >= halfPowerVoltage) {
                                                        leftIndex = i;
                                                        break;
                                                    }
                                                }
                                                
                                                for (int i = acReadingIndex - 1; i > maxVoltageIndex; i--) {
                                                    if (ain1Readings[i] >= halfPowerVoltage) {
                                                        rightIndex = i;
                                                        break;
                                                    }
                                                }
                                                
                                                // 计算Q值和其他特征
                                                if (leftIndex >= 0 && rightIndex >= 0) {
                                                    float leftFreq = 1000.0 + (leftIndex * timePerSample) * freqRatio;
                                                    float rightFreq = 1000.0 + (rightIndex * timePerSample) * freqRatio;
                                                    float bandWidth = rightFreq - leftFreq;
                                                    
                                                    if (bandWidth > 0) {
                                                        qValue = resonanceFreq / bandWidth;
                                                        peakWidth = rightIndex - leftIndex; // 峰值宽度(采样点数)
                                                        
                                                        // 2. 计算峰值区域面积(近似积分)
                                                        for (int i = leftIndex; i <= rightIndex; i++) {
                                                            peakArea += ain1Readings[i];
                                                        }
                                                        peakArea = peakArea / (rightIndex - leftIndex + 1);
                                                        
                                                        // 3. 计算曲线不对称性(右侧与左侧宽度比)
                                                        float leftWidth = maxVoltageIndex - leftIndex;
                                                        float rightWidth = rightIndex - maxVoltageIndex;
                                                        if (leftWidth > 0) {
                                                            curveAsymmetry = rightWidth / leftWidth;
                                                        }
                                                        
                                                        // 4. 计算峰值后的衰减率
                                                        int decay10Pct = -1;
                                                        for (int i = maxVoltageIndex; i < acReadingIndex; i++) {
                                                            if (ain1Readings[i] < maxVoltage * 0.9) {
                                                                decay10Pct = i;
                                                                break;
                                                            }
                                                        }
                                                        
                                                        if (decay10Pct > 0) {
                                                            decayRate = (maxVoltage * 0.1) / ((decay10Pct - maxVoltageIndex) * timePerSample);
                                                        }
                                                    }
                                                }
                                                
                                                // 输出调试信息
                                                Serial.print("谐振频率(Hz): "); Serial.println(resonanceFreq, 2);
                                                Serial.print("Q值: "); Serial.println(qValue, 2);
                                                Serial.print("峰值宽度: "); Serial.println(peakWidth);
                                                Serial.print("曲线不对称性: "); Serial.println(curveAsymmetry, 3);
                                                Serial.print("衰减率: "); Serial.println(decayRate, 4);
                                            }
                                            
                                            // 5. 计算衰减段的斜率和形状
                                            float decaySlope = 0.0;
                                            float decayVariance = 0.0;
                                            
                                            if (hasPeak && maxVoltageIndex < acReadingIndex - 10) {
                                                int decayStart = maxVoltageIndex;
                                                int decayEnd = min(maxVoltageIndex + 30, acReadingIndex - 1);
                                                
                                                // 简单线性回归计算斜率
                                                float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
                                                int n = decayEnd - decayStart + 1;
                                                
                                                for (int i = decayStart; i <= decayEnd; i++) {
                                                    float x = i - decayStart;
                                                    float y = ain1Readings[i];
                                                    sumX += x;
                                                    sumY += y;
                                                    sumXY += x * y;
                                                    sumX2 += x * x;
                                                }
                                                
                                                if ((n * sumX2 - sumX * sumX) != 0) {
                                                    decaySlope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                                                }
                                                
                                                // 计算离散度/方差
                                                float avgY = sumY / n;
                                                for (int i = decayStart; i <= decayEnd; i++) {
                                                    decayVariance += pow(ain1Readings[i] - avgY, 2);
                                                }
                                                decayVariance /= n;
                                                
                                                Serial.print("衰减斜率: "); Serial.println(decaySlope, 6);
                                                Serial.print("衰减方差: "); Serial.println(decayVariance, 6);
                                            }
                                            
                                            // 新增波形特征分析代码
                                            float peakSharpness = 0;
                                            if (hasPeak && peakWidth > 0) {
                                                peakSharpness = maxVoltage / peakWidth;
                                                Serial.print("峰值尖锐度: "); Serial.println(peakSharpness, 6);
                                            }
                                            
                                            // 计算曲线平滑度 - 通过分析峰值前后波形的二阶差分
                                            float prePeakSmoothness = 0.0;
                                            float postPeakSmoothness = 0.0;
                                            
                                            if (hasPeak && maxVoltageIndex > 5 && maxVoltageIndex < acReadingIndex - 5) {
                                                float prePeakSum = 0.0;
                                                float postPeakSum = 0.0;
                                                
                                                // 计算峰值前的曲率
                                                for (int i = maxVoltageIndex - 5; i < maxVoltageIndex - 1; i++) {
                                                    float d2 = ain1Readings[i+1] - 2*ain1Readings[i] + ain1Readings[i-1];
                                                    prePeakSum += abs(d2);
                                                }
                                                prePeakSmoothness = prePeakSum / 4.0;
                                                
                                                // 计算峰值后的曲率
                                                for (int i = maxVoltageIndex + 2; i < maxVoltageIndex + 6; i++) {
                                                    float d2 = ain1Readings[i+1] - 2*ain1Readings[i] + ain1Readings[i-1];
                                                    postPeakSum += abs(d2);
                                                }
                                                postPeakSmoothness = postPeakSum / 4.0;
                                                
                                                Serial.print("峰值前曲率: "); Serial.println(prePeakSmoothness, 6);
                                                Serial.print("峰值后曲率: "); Serial.println(postPeakSmoothness, 6);
                                            }
                                            
                                            // 计算波形的对称性指数 - LC并联通常更对称
                                            float symmetryIndex = 0.0;
                                            if (hasPeak && curveAsymmetry > 0) {
                                                symmetryIndex = 1.0 / (1.0 + abs(curveAsymmetry - 1.0));
                                                Serial.print("波形对称性指数: "); Serial.println(symmetryIndex, 4);
                                            }
                                            
                                            // 判断峰值下降速度 - LC和RLC的特征差异
                                            float decaySpeedRatio = 0.0;
                                            if (hasPeak) {
                                                int decay70PctIndex = -1;
                                                for (int i = maxVoltageIndex; i < acReadingIndex; i++) {
                                                    if (ain1Readings[i] < maxVoltage * 0.7) {
                                                        decay70PctIndex = i;
                                                        break;
                                                    }
                                                }
                                                
                                                if (decay70PctIndex > 0 && peakWidth > 0) {
                                                    // 衰减需要经过的点数
                                                    float decayPoints = decay70PctIndex - maxVoltageIndex;
                                                    // 计算相对于峰值宽度的比例
                                                    decaySpeedRatio = decayPoints / peakWidth;
                                                    Serial.print("衰减速度比: "); Serial.println(decaySpeedRatio, 4);
                                                }
                                            }
                                            
                                            // 输出诊断信息
                                            Serial.println("\n负载网络诊断信息 (模式1):");
                                            Serial.print("最大电压: "); Serial.println(maxVoltage, 6);
                                            Serial.print("最大电压位置: "); Serial.print(maxVoltageIndex);
                                            Serial.print(" (时间: "); Serial.print(maxVoltageIndex * timePerSample, 2);
                                            Serial.println(" ms)");
                                            Serial.print("末尾/起始电压比: "); Serial.println(endToStartRatio, 6);
                                            Serial.print("峰值特征: "); Serial.println(hasPeak ? "是" : "否");
                                            
                                            // 使用多特征进行负载判断
                                            if (!hasPeak && endToStartRatio > 4.5) { // 降低阈值从6.0到4.5
                                                // RL并联：无峰值，电压从低到高持续上升
                                                LoadNetType = 6; // RL并联
                                                Serial.println("模式1检测到: RL并联 (LoadNetType = 6)");
                                            }
                                            // 新增：如果末尾/起始比不满足但仍有明显上升趋势，也可能是RL并联
                                            else if (!hasPeak && endToStartRatio > 3.0) {
                                                // 检查是否有稳定的上升趋势 - 计算四等分段的平均值
                                                float firstSegment = 0.0, secondSegment = 0.0, thirdSegment = 0.0, fourthSegment = 0.0;
                                                int segmentSize = acReadingIndex / 4;
                            
                                                for (int i = 0; i < segmentSize; i++) {
                                                    firstSegment += ain1Readings[i];
                                                }
                                                    firstSegment /= segmentSize;
                            
                                                for (int i = segmentSize; i < segmentSize * 2; i++) {
                                                    secondSegment += ain1Readings[i];
                                                }
                                                    secondSegment /= segmentSize;
                            
                                                for (int i = segmentSize * 2; i < segmentSize * 3; i++) {
                                                    thirdSegment += ain1Readings[i];
                                                }
                                                    thirdSegment /= segmentSize;
                            
                                                for (int i = segmentSize * 3; i < acReadingIndex; i++) {
                                                    fourthSegment += ain1Readings[i];
                                                }
                                                fourthSegment /= (acReadingIndex - segmentSize * 3);
                                                
                                                // 检查是否呈现持续上升趋势
                                                if (secondSegment > firstSegment * 1.3 && 
                                                    thirdSegment > secondSegment * 1.2 && 
                                                    fourthSegment > thirdSegment * 1.1) {
                                                    LoadNetType = 6; // RL并联
                                                    Serial.println("模式1检测到: RL并联 (稳定上升趋势判断) (LoadNetType = 6)");
                                                    // 输出四段平均值用于调试
                                                    Serial.print("四段平均值: ");
                                                    Serial.print(firstSegment, 4); Serial.print(", ");
                                                    Serial.print(secondSegment, 4); Serial.print(", ");
                                                    Serial.print(thirdSegment, 4); Serial.print(", ");
                                                    Serial.println(fourthSegment, 4);
                                                }
                                                // 判断是否接近末尾时快速上升(这是RL并联的另一个特征)
                                                else if (fourthSegment / firstSegment > 4.0) {
                                                    LoadNetType = 6; // RL并联
                                                    Serial.println("模式1检测到: RL并联 (末段上升判断) (LoadNetType = 6)");
                                                }
                                            }
                                            else if (hasPeak) {
                                                // 检测波形模式特征
                                                bool hasLCPattern = false;
                                                bool hasRLCPattern = false;
                                                
                                                // LC并联典型特征: 快速上升、尖锐峰值、快速下降、高对称性
                                                if (peakSharpness > 0.025 && symmetryIndex > 0.7 && decaySpeedRatio < 0.5) {
                                                    hasLCPattern = true;
                                                    Serial.println("检测到LC并联典型波形特征");
                                                }
                                                
                                                // RLC并联典型特征: 较缓慢上升、平缓峰值、慢速下降、低对称性
                                                if (peakSharpness < 0.020 && symmetryIndex < 0.6 && decaySpeedRatio > 0.7) {
                                                    hasRLCPattern = true;
                                                    Serial.println("检测到RLC并联典型波形特征");
                                                }
                                                
                                                // 创建多特征综合评分
                                                int lcScore = 0;
                                                int rlcScore = 0;
                                                
                                                // ===== 特征评分系统 =====
                                                
                                                // 1. 谐振频率特征(调整频率区间和权重)
                                                if (resonanceFreq < 7000) {
                                                    // 非常低的谐振频率
                                                    if (peakSharpness > 0.03) { // 非常尖锐的峰值是LC特征
                                                        lcScore += 2;
                                                        Serial.println("超低频谐振+尖锐峰值: LC +2");
                                                    } else {
                                                        rlcScore += 2;
                                                        Serial.print("谐振频率特征(超低): "); 
                                                        Serial.print(resonanceFreq); 
                                                        Serial.println("Hz (RLC +2)");
                                                    }
                                                } else if (resonanceFreq >= 7000 && resonanceFreq < 13000) {
                                                    // 对于7-13kHz频率，需要额外验证波形特征
                                                    if (hasLCPattern || peakSharpness > 0.03) {
                                                        lcScore += 2;
                                                        Serial.print("谐振频率特征: "); 
                                                        Serial.print(resonanceFreq); 
                                                        Serial.println("Hz + LC波形特征 (LC +2)");
                                                    } else if (hasRLCPattern) {
                                                        rlcScore += 2;
                                                        Serial.print("谐振频率特征: "); 
                                                        Serial.print(resonanceFreq); 
                                                        Serial.println("Hz + RLC波形特征 (RLC +2)");
                                                    } else if (peakSharpness < 0.02) {
                                                        rlcScore += 1;
                                                        Serial.print("谐振频率特征: "); 
                                                        Serial.print(resonanceFreq); 
                                                        Serial.println("Hz + 平缓峰值 (RLC +1)");
                                                    } else {
                                                        // 基于对称性再做进一步判断
                                                        if (symmetryIndex > 0.8) {
                                                            lcScore += 1;
                                                            Serial.print("谐振频率特征: "); 
                                                            Serial.print(resonanceFreq); 
                                                            Serial.println("Hz + 高对称性 (LC +1)");
                                                        } else {
                                                            rlcScore += 1;
                                                            Serial.print("谐振频率特征: "); 
                                                            Serial.print(resonanceFreq); 
                                                            Serial.println("Hz + 低对称性 (RLC +1)");
                                                        }
                                                    }
                                                } else if (resonanceFreq >= 13000 && resonanceFreq < 18000) {
                                                    // 中频区域(13-18kHz)
                                                    if (hasLCPattern) {
                                                        lcScore += 1;
                                                        Serial.print("谐振频率特征: "); 
                                                        Serial.print(resonanceFreq); 
                                                        Serial.println("Hz + LC波形特征 (LC +1)");
                                                    } else if (hasRLCPattern) {
                                                        rlcScore += 1;
                                                        Serial.print("谐振频率特征: "); 
                                                        Serial.print(resonanceFreq); 
                                                        Serial.println("Hz + RLC波形特征 (RLC +1)");
                                                    } else {
                                                        // 中频区域谐振本身不加分，需要依靠其他特征判断
                                                        Serial.print("谐振频率特征: "); 
                                                        Serial.print(resonanceFreq); 
                                                        Serial.println("Hz (中频区域，不计分)");
                                                    }
                                                } else if (resonanceFreq >= 18000 && resonanceFreq < 25000) {
                                                    lcScore += 1;
                                                    Serial.print("谐振频率特征: "); 
                                                    Serial.print(resonanceFreq); 
                                                    Serial.println("Hz (LC +1)");
                                                } else if (resonanceFreq >= 25000) {
                                                    lcScore += 2;
                                                    Serial.print("谐振频率特征: "); 
                                                    Serial.print(resonanceFreq); 
                                                    Serial.println("Hz (LC +2)");
                                                }
                                                
                                                // 2. 峰值位置特征(降低权重，更关注形状)
                                                float peakTimeRatio = (maxVoltageIndex * timePerSample) / 2000.0;
                                                
                                                if (hasLCPattern && peakTimeRatio < 0.2) {
                                                    // LC模式波形特征明显，即使峰值位置很早也更可能是LC
                                                    lcScore += 2;
                                                    Serial.print("峰值位置特征: "); 
                                                    Serial.print(peakTimeRatio * 2000); 
                                                    Serial.println("ms (LC模式波形优先) (LC +2)");
                                                } else if (hasRLCPattern && peakTimeRatio > 0.3) {
                                                    // RLC模式波形特征明显，即使峰值位置偏后也更可能是RLC
                                                    rlcScore += 2;
                                                    Serial.print("峰值位置特征: "); 
                                                    Serial.print(peakTimeRatio * 2000); 
                                                    Serial.println("ms (RLC模式波形优先) (RLC +2)");
                                                } else if (peakTimeRatio < 0.13) {
                                                    // 早期峰值但不确定模式
                                                    if (peakSharpness > 0.03) {
                                                        lcScore += 2;
                                                        Serial.print("峰值位置特征: 早期尖锐峰值 "); 
                                                        Serial.print(peakTimeRatio * 2000); 
                                                        Serial.println("ms (LC +2)");
                                                    } else {
                                                        rlcScore += 2;
                                                        Serial.print("峰值位置特征: 早期平缓峰值 "); 
                                                        Serial.print(peakTimeRatio * 2000); 
                                                        Serial.println("ms (RLC +2)");
                                                    }
                                                } else if (peakTimeRatio >= 0.13 && peakTimeRatio < 0.25) {
                                                    // 中早期峰值，形状更重要
                                                    if (peakSharpness > 0.025) {
                                                        lcScore += 1;
                                                        Serial.print("峰值位置特征: 中早期尖锐峰值 "); 
                                                        Serial.print(peakTimeRatio * 2000); 
                                                        Serial.println("ms (LC +1)");
                                                    } else {
                                                        rlcScore += 1;
                                                        Serial.print("峰值位置特征: 中早期平缓峰值 "); 
                                                        Serial.print(peakTimeRatio * 2000); 
                                                        Serial.println("ms (RLC +1)");
                                                    }
                                                } else if (peakTimeRatio >= 0.25 && peakTimeRatio < 0.35) {
                                                    // 不加分，除非波形特征明显
                                                    if (hasLCPattern) {
                                                        lcScore += 1;
                                                        Serial.println("峰值位置特征: 中间位置 + LC波形特征 (LC +1)");
                                                    } else if (hasRLCPattern) {
                                                        rlcScore += 1;
                                                        Serial.println("峰值位置特征: 中间位置 + RLC波形特征 (RLC +1)");
                                                    } else {
                                                        Serial.println("峰值位置特征: 中间位置 (不计分)");
                                                    }
                                                } else if (peakTimeRatio >= 0.35) {
                                                    // 晚期峰值更可能是LC
                                                    lcScore += 2;
                                                    Serial.print("峰值位置特征: 晚期峰值 "); 
                                                    Serial.print(peakTimeRatio * 2000); 
                                                    Serial.println("ms (LC +2)");
                                                }
                                                
                                                // 3. 峰值尖锐度 - 新增特征
                                                if (peakSharpness > 0.035) {
                                                    lcScore += 3;
                                                    Serial.println("峰值尖锐度: 极高 (LC +3)");
                                                } else if (peakSharpness > 0.025) {
                                                    lcScore += 2;
                                                    Serial.println("峰值尖锐度: 高 (LC +2)");
                                                } else if (peakSharpness > 0.02) {
                                                    lcScore += 1;
                                                    Serial.println("峰值尖锐度: 中等 (LC +1)");
                                                } else if (peakSharpness < 0.015) {
                                                    rlcScore += 2;
                                                    Serial.println("峰值尖锐度: 低 (RLC +2)");
                                                } else if (peakSharpness < 0.02) {
                                                    rlcScore += 1;
                                                    Serial.println("峰值尖锐度: 中低 (RLC +1)");
                                                }
                                                
                                                // 4. 波形对称性
                                                if (symmetryIndex > 0.85) {
                                                    lcScore += 2;
                                                    Serial.println("波形对称性: 极高 (LC +2)");
                                                } else if (symmetryIndex > 0.7) {
                                                    lcScore += 1;
                                                    Serial.println("波形对称性: 高 (LC +1)");
                                                } else if (symmetryIndex < 0.5) {
                                                    rlcScore += 2;
                                                    Serial.println("波形对称性: 低 (RLC +2)");
                                                } else if (symmetryIndex < 0.6) {
                                                    rlcScore += 1;
                                                    Serial.println("波形对称性: 中低 (RLC +1)");
                                                }
                                                
                                                // 5. 峰值宽度与Q值组合特征
                                                if (peakWidth < 20 && qValue > 1.5) {
                                                    lcScore += 2;
                                                    Serial.print("峰宽Q值组合: 窄峰高Q ("); 
                                                    Serial.print(peakWidth); 
                                                    Serial.print(", "); 
                                                    Serial.print(qValue, 2); 
                                                    Serial.println(") (LC +2)");
                                                } else if (peakWidth < 25) {
                                                    lcScore += 1;
                                                    Serial.print("峰值宽度: 窄 ("); 
                                                    Serial.print(peakWidth); 
                                                    Serial.println(") (LC +1)");
                                                } else if (peakWidth > 40 && qValue < 1.5) {
                                                    rlcScore += 2;
                                                    Serial.print("峰宽Q值组合: 宽峰低Q ("); 
                                                    Serial.print(peakWidth); 
                                                    Serial.print(", "); 
                                                    Serial.print(qValue, 2); 
                                                    Serial.println(") (RLC +2)");
                                                } else if (peakWidth > 35) {
                                                    rlcScore += 1;
                                                    Serial.print("峰值宽度: 宽 ("); 
                                                    Serial.print(peakWidth); 
                                                    Serial.println(") (RLC +1)");
                                                }
                                                
                                                // 6. 衰减速度比特征
                                                if (decaySpeedRatio < 0.4) {
                                                    lcScore += 2;
                                                    Serial.println("衰减速度比: 极快 (LC +2)");
                                                } else if (decaySpeedRatio < 0.6) {
                                                    lcScore += 1;
                                                    Serial.println("衰减速度比: 快 (LC +1)");
                                                } else if (decaySpeedRatio > 0.8) {
                                                    rlcScore += 2;
                                                    Serial.println("衰减速度比: 慢 (RLC +2)");
                                                } else if (decaySpeedRatio > 0.6) {
                                                    rlcScore += 1;
                                                    Serial.println("衰减速度比: 中慢 (RLC +1)");
                                                }
                                                
                                                // 7. 峰值电压/频率比 - 保持但降低权重
                                                float peakFreqRatio = maxVoltage * 10000 / resonanceFreq;
                                                if (peakFreqRatio > 0.8) {
                                                    lcScore += 1;
                                                    Serial.print("峰值/频率比: "); 
                                                    Serial.print(peakFreqRatio, 3); 
                                                    Serial.println(" (LC +1)");
                                                } else if (peakFreqRatio < 0.3) {
                                                    rlcScore += 1;
                                                    Serial.print("峰值/频率比: "); 
                                                    Serial.print(peakFreqRatio, 3); 
                                                    Serial.println(" (RLC +1)");
                                                }
                                                
                                                // 输出评分并做出判断
                                                Serial.print("LC评分: "); Serial.println(lcScore);
                                                Serial.print("RLC评分: "); Serial.println(rlcScore);
                                                
                                                // 增加置信度评估
                                                float confidence = 0.0;
                                                if (lcScore + rlcScore > 0) {
                                                    confidence = abs(lcScore - rlcScore) / (float)(lcScore + rlcScore) * 100.0;
                                                }
                                                Serial.print("判断置信度: "); Serial.print(confidence, 1); Serial.println("%");
                                                
                                                // 改进判断逻辑
                                                if (lcScore > rlcScore) {
                                                    LoadNetType = 7; // LC并联
                                                    Serial.println("模式1检测到: LC并联 (LoadNetType = 7)");
                                                } else if (rlcScore > lcScore) {
                                                    LoadNetType = 8; // RLC并联
                                                    Serial.println("模式1检测到: RLC并联 (LoadNetType = 8)");
                                                } else {
                                                    // 得分相同时，优先查看波形模式特征
                                                    if (hasLCPattern && !hasRLCPattern) {
                                                        LoadNetType = 7; // LC并联
                                                        Serial.println("模式1检测到: LC并联 (波形模式匹配) (LoadNetType = 7)");
                                                    } else if (hasRLCPattern && !hasLCPattern) {
                                                        LoadNetType = 8; // RLC并联 
                                                        Serial.println("模式1检测到: RLC并联 (波形模式匹配) (LoadNetType = 8)");
                                                    } else if (peakSharpness > 0.025) {
                                                        // 尖锐峰值优先判断为LC
                                                        LoadNetType = 7;
                                                        Serial.println("模式1检测到: LC并联 (峰值尖锐度判断) (LoadNetType = 7)");
                                                    } else if (peakTimeRatio < 0.2 && resonanceFreq < 10000) {
                                                        // 早期低频峰值优先判断为RLC
                                                        LoadNetType = 8;
                                                        Serial.println("模式1检测到: RLC并联 (早期低频峰值判断) (LoadNetType = 8)");
                                                    } else {
                                                        // 通过波形形态做最终判断
                                                        if (symmetryIndex > 0.7 || decaySpeedRatio < 0.5) {
                                                            LoadNetType = 7; // LC并联
                                                            Serial.println("模式1检测到: LC并联 (波形形态判断) (LoadNetType = 7)");
                                                        } else {
                                                            LoadNetType = 8; // RLC并联
                                                            Serial.println("模式1检测到: RLC并联 (波形形态判断) (LoadNetType = 8)");
                                                        }
                                                    }
                                                }
                                            }
                                            else {
                                                // 无法判断
                                                LoadNetType = 0; // 默认开路
                                                Serial.println("负载网络判断不明确，默认为开路");
                                            }
                                        }
                                        break;

                        
                    case 2: // 模式2 (0.1 <= dcVoltAvg <= 0.95)
                    {
                        // 识别算法优化
                        
                        // 1. 首先检查是否符合RC并联特征（开始几个点电压较高，然后突然下降到很低且稳定）
                        float maxInitialValue = 0.0;
                        float minLaterValue = 1000.0;
                        float avgVoltage = 0.0;
                        float sumVoltage = 0.0;
                        float variance = 0.0;
                        int highVoltageCount = 0;
                        int lowVoltageCount = 0;
                        int transitionIndex = -1;
                        
                        // 计算总体平均值和方差
                        for (int i = 0; i < acReadingIndex; i++) {
                            sumVoltage += ain1Readings[i];
                        }
                        avgVoltage = sumVoltage / acReadingIndex;
                        
                        // 计算方差
                        for (int i = 0; i < acReadingIndex; i++) {
                            variance += pow(ain1Readings[i] - avgVoltage, 2);
                        }
                        variance /= acReadingIndex;
                        
                        // 计算最大值和最小值
                        float maxVoltage = 0.0;
                        float minVoltage = 10.0;
                        for (int i = 0; i < acReadingIndex; i++) {
                            if (ain1Readings[i] > maxVoltage) maxVoltage = ain1Readings[i];
                            if (ain1Readings[i] < minVoltage) minVoltage = ain1Readings[i];
                        }
                        
                        // 原始代码：找出前15个点中的最大值（增加检测范围）
                        int checkPoints = min(15, acReadingIndex);
                        for (int i = 0; i < checkPoints; i++) {
                            if (ain1Readings[i] > maxInitialValue) {
                                maxInitialValue = ain1Readings[i];
                            }
                            // 计算高于系统参考电压18%的点数
                            if (ain1Readings[i] > systemMaxVoltage * 0.18) {
                                highVoltageCount++;
                            }
                        }
                        
                        // 优化转变点检测，处理更多变化模式
                        for (int i = 1; i < acReadingIndex; i++) {
                            // 常规检测：从高电压到低电压的明显转变
                            if (ain1Readings[i] < systemMaxVoltage * 0.06 && 
                                ain1Readings[i-1] > systemMaxVoltage * 0.18) {
                                transitionIndex = i;
                                break;
                            }
                            // 次级检测：电压明显下降，但可能不满足严格的高->低转变条件
                            else if (i > 2 && ain1Readings[i] < systemMaxVoltage * 0.025 && 
                                    ain1Readings[i-3] > systemMaxVoltage * 0.09 && 
                                    transitionIndex == -1) {
                                transitionIndex = i;
                                break;
                            }
                        }
                        
                        // 即使没有找到转变点，也计算后半部分数据的最小值
                        int startIndex = (transitionIndex > 0) ? transitionIndex : (acReadingIndex / 3);
                        
                        // 计算后半部分的最小值和低电压点数
                        for (int i = startIndex; i < acReadingIndex; i++) {
                            if (ain1Readings[i] < minLaterValue) {
                                minLaterValue = ain1Readings[i];
                            }
                            
                            // 计算后半部分的低电压点数
                            if (ain1Readings[i] < systemMaxVoltage * 0.06) {
                                lowVoltageCount++;
                            }
                        }
                        
                        // 2. 检查是否符合RL串联特征（持续上升趋势）
                        float firstQuarter = 0.0;
                        float lastQuarter = 0.0;
                        bool steadyIncreasing = true;
                        int quarterSize = acReadingIndex / 4;
                        
                        // 计算第一个1/4和最后1/4的平均值
                        for (int i = 0; i < quarterSize; i++) {
                            firstQuarter += ain1Readings[i];
                        }
                        firstQuarter /= quarterSize;
                        
                        for (int i = acReadingIndex - quarterSize; i < acReadingIndex; i++) {
                            lastQuarter += ain1Readings[i];
                        }
                        lastQuarter /= quarterSize;
                        
                        // 计算是否持续上升
                        float prevAvg = 0;
                        for (int i = 0; i < min(5, acReadingIndex); i++) {
                            prevAvg += ain1Readings[i];
                        }
                        prevAvg /= min(5, acReadingIndex);
                        
                        // 检查整体趋势是否上升
                        for (int i = 5; i < acReadingIndex; i += 5) {
                            float currentAvg = 0;
                            int count = 0;
                            for (int j = i; j < min(i+5, acReadingIndex); j++) {
                                currentAvg += ain1Readings[j];
                                count++;
                            }
                            currentAvg /= count;
                            
                            if (currentAvg < prevAvg * 0.95) {
                                steadyIncreasing = false;
                                break;
                            }
                            prevAvg = currentAvg;
                        }
                        
                        // ============= 新增：增强RC并联识别逻辑 =============
                        // 新增特征：低电压稳定型RC并联特性检测
                        bool isLowVoltageStable = false;
                        bool isVeryLowVoltage = false;
                        bool isSmallVariance = false;
                        
                        // 判断总体电压是否很低（低于系统参考电压的6%）
                        isVeryLowVoltage = (avgVoltage < systemMaxVoltage * 0.06);
                        
                        // 判断方差是否很小（高度稳定）- 根据数据样本设定阈值
                        isSmallVariance = (variance < 0.0000001); // 极小的方差表示波动很小
                        
                        // 判断电压稳定性（最大值和最小值的比值接近1）
                        bool isStableVoltage = (maxVoltage / minVoltage < 1.05);
                        
                        // 低电压稳定型RC并联特征
                        isLowVoltageStable = isVeryLowVoltage && (isSmallVariance || isStableVoltage);
                        
                        // ============= 优化RC并联检测逻辑 =============
                        bool isRC = false;
                        
                        // 安全检查，确保minLaterValue有效
                        if (minLaterValue < 900.0) {
                            // 条件1: 典型RC特征 - 高电压开始，后面大部分是低电压
                            bool condition1 = (highVoltageCount > 0) && 
                                            (lowVoltageCount > acReadingIndex/3) && 
                                            (maxInitialValue / minLaterValue > 3.0); // 降低比值门槛从5到3
                                            
                            // 条件2: 起始电压高于后期电压的3倍，并且有大量低电压点
                            bool condition2 = (firstQuarter / minLaterValue > 3.0) && 
                                            (lowVoltageCount > acReadingIndex/3);
                            
                            // 条件3: 新增 - 低电压高度稳定型RC并联
                            bool condition3 = isLowVoltageStable && !steadyIncreasing;
                            
                            isRC = condition1 || condition2 || condition3;
                        }
                        
                        // RL串联检测逻辑保持不变但增加条件
                        bool isRL = (steadyIncreasing || lastQuarter > firstQuarter * 1.2) && 
                                    (firstQuarter > 0.3) &&
                                    (!isRC) && // 确保不是RC网络
                                    (!isVeryLowVoltage); // 确保不是低电压稳定型
                        
                        // ============= 输出详细诊断信息 =============
                        Serial.print("平均电压: ");
                        Serial.print(avgVoltage, 6);
                        Serial.print(", 电压方差: ");
                        Serial.println(variance, 12);
                        Serial.print("电压波动范围: ");
                        Serial.print(minVoltage, 6);
                        Serial.print(" - ");
                        Serial.print(maxVoltage, 6);
                        Serial.print(" (波动率: ");
                        Serial.print((maxVoltage - minVoltage) / avgVoltage * 100.0, 2);
                        Serial.println("%)");
                        Serial.print("低电压稳定特征: ");
                        Serial.println(isLowVoltageStable ? "是" : "否");
                        
                        // 根据特征判断网络类型
                        if (isRC) {
                            LoadNetType = 5; // RC并联
                            if (isLowVoltageStable) {
                                Serial.println("模式2检测到: RC并联 (低电压稳定型) (LoadNetType = 5)");
                            } else {
                                Serial.println("模式2检测到: RC并联 (传统特征) (LoadNetType = 5)");
                                Serial.print("特征: 初始高电压点数=");
                                Serial.print(highVoltageCount);
                                Serial.print(", 转变点=");
                                Serial.print(transitionIndex);
                                Serial.print(", 低电压点数=");
                                Serial.println(lowVoltageCount);
                            }
                        } else if (isRL) {
                            LoadNetType = 4; // RL串联
                            Serial.println("模式2检测到: RL串联 (LoadNetType = 4)");
                            Serial.print("特征: 起始平均值=");
                            Serial.print(firstQuarter, 6);
                            Serial.print(", 结束平均值=");
                            Serial.print(lastQuarter, 6);
                            Serial.print(", 比值=");
                            Serial.println(lastQuarter/firstQuarter, 2);
                        } else {
                            // 添加额外判断分支，针对特殊情况进行处理
                            if (minLaterValue < 0.04 && maxInitialValue > 0.3) {
                                LoadNetType = 5; // 极可能是RC并联
                                Serial.println("模式2检测到: RC并联 (特殊判断) (LoadNetType = 5)");
                            } else {
                                LoadNetType = 0; // 默认开路
                                Serial.println("负载判断不明确，默认为开路");
                            }
                        }
                        
                        // 输出详细诊断信息
                        Serial.print("初始最大值: ");
                        Serial.print(maxInitialValue, 6);
                        Serial.print(", 后续最小值: ");
                        Serial.print(minLaterValue, 6);
                        Serial.print(", 比值: ");
                        Serial.println(maxInitialValue/minLaterValue, 2);
                        Serial.print("持续上升特征: ");
                        Serial.println(steadyIncreasing ? "是" : "否");
                    }
                    break;
                        
                    case 3: // 模式3 (dcVoltAvg > 0.95)
                    {
                        // 计算基本统计量
                        float maxVoltage = 0.0;
                        float minVoltage = 10.0; // 设置一个足够大的初始值
                        float sumVoltage = 0.0;
                        float variance = 0.0;
                        
                        // 找出最大最小值，计算平均值
                        for (int i = 0; i < acReadingIndex; i++) {
                            if (ain1Readings[i] > maxVoltage) {
                                maxVoltage = ain1Readings[i];
                            }
                            if (ain1Readings[i] < minVoltage) {
                                minVoltage = ain1Readings[i];
                            }
                            sumVoltage += ain1Readings[i];
                        }
                        
                        float avgVoltage = sumVoltage / acReadingIndex;
                        
                        // 计算方差（稳定性指标）
                        for (int i = 0; i < acReadingIndex; i++) {
                            variance += pow(ain1Readings[i] - avgVoltage, 2);
                        }
                        variance /= acReadingIndex;
                        
                        // 计算首尾趋势（成长性指标）
                        int quarterSize = acReadingIndex / 4;
                        float firstQuarterAvg = 0.0;
                        float lastQuarterAvg = 0.0;
                        
                        // 计算第一个1/4的平均值
                        for (int i = 0; i < quarterSize; i++) {
                            firstQuarterAvg += ain1Readings[i];
                        }
                        firstQuarterAvg /= quarterSize;
                        
                        // 计算最后1/4的平均值，但忽略可能的异常点
                        int validLastPoints = 0;
                        float lastQuarterSum = 0.0;
                        float normalLastQuarterAvg = 0.0;
                        
                        // 先计算正常的末尾平均值（不包括异常突变点）
                        for (int i = acReadingIndex - quarterSize; i < acReadingIndex - 3; i++) {
                            normalLastQuarterAvg += ain1Readings[i];
                        }
                        normalLastQuarterAvg /= (quarterSize - 3);
                        
                        // 计算最后1/4部分，过滤掉异常值
                        for (int i = acReadingIndex - quarterSize; i < acReadingIndex; i++) {
                            // 过滤掉与平均值相差过大的点（可能是噪声或突变）
                            if (i >= acReadingIndex - 3 && ain1Readings[i] > normalLastQuarterAvg * 1.3) {
                                // 这是异常点，不计入平均
                                continue;
                            }
                            lastQuarterSum += ain1Readings[i];
                            validLastPoints++;
                        }
                        
                        lastQuarterAvg = (validLastPoints > 0) ? (lastQuarterSum / validLastPoints) : normalLastQuarterAvg;
                        
                        // 计算电压的趋势特征 - 新增：分段斜率计算
                        float slopeFirst = 0.0;
                        float slopeLast = 0.0;
                        float overallSlope = 0.0;
                        
                        // 计算整体趋势斜率（简化线性回归）
                        float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
                        for (int i = 0; i < acReadingIndex; i++) {
                            sumX += i;
                            sumY += ain1Readings[i];
                            sumXY += i * ain1Readings[i];
                            sumX2 += i * i;
                        }
                        if (acReadingIndex > 1 && (sumX2 * acReadingIndex - sumX * sumX) != 0) {
                            overallSlope = (acReadingIndex * sumXY - sumX * sumY) / (acReadingIndex * sumX2 - sumX * sumX);
                        }
                        
                        // 计算前半段和后半段的斜率
                        int midPoint = acReadingIndex / 2;
                        sumX = 0; sumY = 0; sumXY = 0; sumX2 = 0;
                        for (int i = 0; i < midPoint; i++) {
                            sumX += i;
                            sumY += ain1Readings[i];
                            sumXY += i * ain1Readings[i];
                            sumX2 += i * i;
                        }
                        if (midPoint > 1 && (sumX2 * midPoint - sumX * sumX) != 0) {
                            slopeFirst = (midPoint * sumXY - sumX * sumY) / (midPoint * sumX2 - sumX * sumX);
                        }
                        
                        sumX = 0; sumY = 0; sumXY = 0; sumX2 = 0;
                        for (int i = midPoint; i < acReadingIndex; i++) {
                            int j = i - midPoint;
                            sumX += j;
                            sumY += ain1Readings[i];
                            sumXY += j * ain1Readings[i];
                            sumX2 += j * j;
                        }
                        if ((acReadingIndex - midPoint) > 1 && (sumX2 * (acReadingIndex - midPoint) - sumX * sumX) != 0) {
                            slopeLast = ((acReadingIndex - midPoint) * sumXY - sumX * sumY) / 
                                       ((acReadingIndex - midPoint) * sumX2 - sumX * sumX);
                        }
                        
                        // 计算电压在前20个点的下降幅度
                        float initialMaxVoltage = 0.0;
                        float lowestVoltageAfterDrop = 10.0;
                        int dropPointIndex = -1;
                        int initialHighCount = 0; // 记录初始阶段高于后期的点数
                        
                        for (int i = 0; i < min(20, acReadingIndex); i++) {
                            if (ain1Readings[i] > initialMaxVoltage) {
                                initialMaxVoltage = ain1Readings[i];
                            }
                            // 统计初始阶段高于平均值的点数
                            if (ain1Readings[i] > avgVoltage * 1.05) {
                                initialHighCount++;
                            }
                        }
                        
                        // 寻找电压下降后的最低点
                        for (int i = 3; i < min(30, acReadingIndex); i++) {
                            if (ain1Readings[i] < lowestVoltageAfterDrop && 
                                ain1Readings[i] < initialMaxVoltage * 0.7) { // 下降到初始值的70%以下
                                lowestVoltageAfterDrop = ain1Readings[i];
                                dropPointIndex = i;
                            }
                        }
                        
                        // 计算后半段的稳定性
                        float laterVariance = 0.0;
                        float laterAvg = 0.0;
                        int laterStartIndex = acReadingIndex / 3; // 从1/3处开始计算后段稳定性
                        
                        for (int i = laterStartIndex; i < acReadingIndex - 3; i++) { // 忽略最后3个可能的异常点
                            laterAvg += ain1Readings[i];
                        }
                        laterAvg /= (acReadingIndex - laterStartIndex - 3);
                        
                        for (int i = laterStartIndex; i < acReadingIndex - 3; i++) {
                            laterVariance += pow(ain1Readings[i] - laterAvg, 2);
                        }
                        laterVariance /= (acReadingIndex - laterStartIndex - 3);
                        
                        // 新增对斜率线性度的计算
                        float risingLinearity = 0.0;
                        bool hasLinearRising = false;
                    
                        // 分析上升部分的线性程度（LC串联的特征）
                        if (minVoltage < 0.1) {
                            // 找到最低电压点的位置
                            int lowestIndex = 0;
                            for (int i = 0; i < acReadingIndex; i++) {
                                if (ain1Readings[i] <= minVoltage * 1.1) {
                                    lowestIndex = i;
                                    break;
                                }
                            }
                            
                            // 计算上升段的线性度
                            if (lowestIndex < acReadingIndex - 10) { // 确保有足够的上升点
                                // 计算线性回归
                                float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
                                int n = acReadingIndex - lowestIndex;
                                
                                for (int i = lowestIndex; i < acReadingIndex; i++) {
                                    float x = i - lowestIndex;
                                    float y = ain1Readings[i];
                                    sumX += x;
                                    sumY += y;
                                    sumXY += x * y;
                                    sumX2 += x * x;
                                }
                                
                                // 计算线性回归系数
                                float slope = 0, intercept = 0;
                                if ((n * sumX2 - sumX * sumX) != 0) {
                                    slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                                    intercept = (sumY - slope * sumX) / n;
                                }
                                
                                // 计算R²值来评估线性度
                                float yMean = sumY / n;
                                float ssTotal = 0, ssResidual = 0;
                                for (int i = lowestIndex; i < acReadingIndex; i++) {
                                    float x = i - lowestIndex;
                                    float y = ain1Readings[i];
                                    float yPredicted = slope * x + intercept;
                                    
                                    ssTotal += pow(y - yMean, 2);
                                    ssResidual += pow(y - yPredicted, 2);
                                }
                                
                                if (ssTotal > 0) {
                                    risingLinearity = 1 - (ssResidual / ssTotal); // R²值，接近1表示线性度高
                                }
                                
                                // 线性上升判断
                                hasLinearRising = risingLinearity > 0.95 && slope > 0;
                                
                                // 输出调试信息
                                Serial.print("上升线性度(R²): "); Serial.println(risingLinearity, 6);
                                Serial.print("上升斜率: "); Serial.println(slope, 6);
                            }
                        }
                        
                        // ===== 新增：RC串联特性的增强检测 =====
                        float steadyDownwardTrend = 0;
                        bool hasConsistentDownwardTrend = false;
                        
                        // 判断是否有一致的缓慢下降趋势(RC串联特征)
                        if (overallSlope < 0 && abs(overallSlope) < 0.0003 && variance < 0.0007) {
                            // 检查是否整体平滑下降
                            bool isSmoothDecline = true;
                            float prevAvg = 0;
                            int windowSize = 10;
                            
                            for (int i = 0; i < acReadingIndex - windowSize; i += windowSize) {
                                float windowAvg = 0;
                                for (int j = 0; j < windowSize && (i+j) < acReadingIndex; j++) {
                                    windowAvg += ain1Readings[i+j];
                                }
                                windowAvg /= windowSize;
                                
                                if (i > 0) {
                                    // 如果出现上升超过0.5%，则不是平滑下降
                                    if (windowAvg > prevAvg * 1.005) {
                                        isSmoothDecline = false;
                                        break;
                                    }
                                }
                                prevAvg = windowAvg;
                            }
                            
                            steadyDownwardTrend = abs(overallSlope);
                            hasConsistentDownwardTrend = isSmoothDecline;
                        }
                        
                        // 基于系统参考电压1.7V重新计算相对电压比例
                        float relativeVoltageRatio = avgVoltage / systemMaxVoltage; // 相对于1.7V的比例
                        
                        // 判断逻辑 - 优化条件判断，考虑实际参考电压
                        bool isStable = variance < 0.0006; // 电压稳定性
                        bool isLaterStable = laterVariance < 0.0001; // 后段高度稳定
                        bool isRising = lastQuarterAvg > firstQuarterAvg * 1.1 && overallSlope > 0; // 上升趋势判断
                        bool hasSignificantDrop = initialMaxVoltage > 0 && 
                                                 (initialMaxVoltage / lowestVoltageAfterDrop) > 1.5;  
                        bool lowMinimum = minVoltage < systemMaxVoltage * 0.12; 
                        bool moderateMinimum = minVoltage >= (systemMaxVoltage * 0.2) && 
                             minVoltage <= (systemMaxVoltage * 0.88); 
                        
                        // 强化开路判断 - 调整阈值
                        bool isVeryCloseToOpen = relativeVoltageRatio > 0.97; // 极接近开路(>97%)
                        bool isCloseToOpen = relativeVoltageRatio > 0.85 && relativeVoltageRatio <= 0.97; // 接近开路(85%-97%)
                        
                        // 检测首尾微弱下降，调整阈值，避免稳定开路中的缓慢下降被误判
                        bool hasSlightDownwardTrend = firstQuarterAvg > lastQuarterAvg &&
                                                     (firstQuarterAvg - lastQuarterAvg) / avgVoltage > 0.01; // 提高阈值至1%
                        
                        // 检测非常微弱的下降 - 专门用于开路判断中的例外情况
                        bool hasVerySlightDownwardTrend = firstQuarterAvg > lastQuarterAvg &&
                                                         (firstQuarterAvg - lastQuarterAvg) / avgVoltage > 0.0005 &&
                                                         (firstQuarterAvg - lastQuarterAvg) / avgVoltage <= 0.01;
                        
                        // 斜率判断更新，提高敏感度
                        bool hasNegativeSlope = overallSlope < -0.00005; // 要求更显著的负斜率
                        bool hasVerySlightNegativeSlope = overallSlope < 0 && overallSlope >= -0.00005; // 微弱负斜率
                        
                        // 标准化电压状态判断，考虑相对于参考电压的比例
                        bool isLowVoltage = relativeVoltageRatio < 0.40;  // 低于40%参考电压
                        bool isMediumVoltage = relativeVoltageRatio >= 0.40 && relativeVoltageRatio <= 0.70; // 40%-70%参考电压
                        bool isHighVoltage = relativeVoltageRatio > 0.70 && relativeVoltageRatio < 0.85; // 70%-85%参考电压
                        
                        // 修改RC串联特征条件，使其更严格
                        bool hasRCCharacteristics = (isStable && !isRising && hasNegativeSlope) || 
                                                    hasConsistentDownwardTrend || 
                                                    hasSlightDownwardTrend;
                        
                        // 开路特征 - 电压极其接近系统最大值且稳定
                        bool hasOpenCircuitCharacteristics = isVeryCloseToOpen && isStable && 
                                                           (maxVoltage - minVoltage) / avgVoltage < 0.15; // 允许最多15%的波动
                        
                        // 输出诊断信息
                        Serial.println("\n负载网络诊断信息:");
                        Serial.print("最大电压: "); Serial.println(maxVoltage, 6);
                        Serial.print("最小电压: "); Serial.println(minVoltage, 6);
                        Serial.print("平均电压: "); Serial.println(avgVoltage, 6);
                        Serial.print("相对电压比例: "); Serial.println(relativeVoltageRatio, 4);
                        Serial.print("电压方差: "); Serial.println(variance, 6);
                        Serial.print("后段方差: "); Serial.println(laterVariance, 6);
                        Serial.print("首尾比值: "); Serial.println(lastQuarterAvg/firstQuarterAvg, 6);
                        Serial.print("首尾差值比: "); Serial.println((firstQuarterAvg-lastQuarterAvg)/avgVoltage, 6);
                        Serial.print("整体斜率: "); Serial.println(overallSlope, 8);
                        Serial.print("前段斜率: "); Serial.println(slopeFirst, 8);
                        Serial.print("后段斜率: "); Serial.println(slopeLast, 8);
                        Serial.print("平稳下降特征: "); Serial.println(hasConsistentDownwardTrend ? "是" : "否");
                        Serial.print("微弱下降趋势(>1%): "); Serial.println(hasSlightDownwardTrend ? "是" : "否");
                        Serial.print("非常微弱下降(0.05%-1%): "); Serial.println(hasVerySlightDownwardTrend ? "是" : "否");
                        Serial.print("下降比值: "); 
                        Serial.println(initialMaxVoltage > 0 ? initialMaxVoltage/lowestVoltageAfterDrop : 0, 6);
                        Serial.print("初始高点数: "); Serial.println(initialHighCount);
                        Serial.print("电压水平: "); 
                        if (isVeryCloseToOpen) Serial.println("极接近开路电压(>97%)");
                        else if (isCloseToOpen) Serial.println("接近开路电压(85%-97%)");
                        else if (isHighVoltage) Serial.println("高电压(70%-85%)");
                        else if (isMediumVoltage) Serial.println("中等电压(40%-70%)");
                        else if (isLowVoltage) Serial.println("低电压(<40%)");
                        
                        // 识别负载网络类型 - 改变判断优先级
                        
                        // 第一优先级：极接近开路电压的判断(>97%)
                        if (isVeryCloseToOpen && isStable && (maxVoltage - minVoltage) / avgVoltage < 0.15) {
                            // 对于极接近开路的情况，即使有微弱下降趋势也视为开路
                            LoadNetType = 0; // 开路
                            Serial.println("模式3检测到: 开路 (极高电压) (LoadNetType = 0)");
                        }
                        // 第二优先级：接近开路电压但不是极接近的情况(85%-97%)
                        else if (isCloseToOpen && isStable && 
                                (maxVoltage - minVoltage) / avgVoltage < 0.01 &&
                                !hasNegativeSlope && !hasSlightDownwardTrend) {
                            // 对于接近但不是极接近的情况，仍然要求没有明显下降趋势
                            LoadNetType = 0; // 开路
                            Serial.println("模式3检测到: 开路 (高电压) (LoadNetType = 0)");
                        } 
                        // 第三优先级：RC串联识别
                        else if (hasRCCharacteristics && !isVeryCloseToOpen) {
                            // 添加!isVeryCloseToOpen条件，避免极高电压被识别为RC串联
                            LoadNetType = 1; // RC串联
                            Serial.println("模式3检测到: RC串联 (特征判断) (LoadNetType = 1)");
                        }
                        // 第四优先级：低电压时的LC串联判断
                        else if (initialHighCount >= 2 && 
                                lowMinimum && // 极低的最小值
                                hasSignificantDrop && // 明显下降
                                (hasLinearRising || (isRising && lastQuarterAvg / firstQuarterAvg > 1.3))) {
                            LoadNetType = 2; // LC串联
                            Serial.println("模式3检测到: LC串联 (LoadNetType = 2)");
                        }
                        // 第五优先级：简化的RC串联判断 - 针对测试数据中的常见情况
                        else if ((isLowVoltage || isMediumVoltage) && isStable && !isRising) {
                            LoadNetType = 1; // RC串联
                            Serial.println("模式3检测到: RC串联 (电压水平判断) (LoadNetType = 1)");
                        }
                        // 第六优先级：RLC串联判断
                        else if ((initialHighCount >= 2 || hasSignificantDrop) && 
                                !lowMinimum && // 不是极低的最小值
                                isRising && !hasConsistentDownwardTrend) { 
                            LoadNetType = 3; // RLC串联
                            Serial.println("模式3检测到: RLC串联 (LoadNetType = 3)");
                        }
                        // 第七优先级：有明显上升趋势且不满足RC特征的情况
                        else if (isRising && overallSlope > 0.0001) {
                            // 区分LC和RLC串联
                            if (lowMinimum) {
                                LoadNetType = 2; // LC串联
                                Serial.println("模式3检测到: LC串联 (上升趋势判断) (LoadNetType = 2)");
                            } else {
                                LoadNetType = 3; // RLC串联
                                Serial.println("模式3检测到: RLC串联 (上升趋势判断) (LoadNetType = 3)");
                            }
                        }
                        // 第八优先级：电压稳定且不接近开路，默认为RC串联
                        else if (isStable && !isVeryCloseToOpen && !isCloseToOpen) {
                            LoadNetType = 1; // RC串联(默认判断)
                            Serial.println("模式3检测到: RC串联(默认稳定状态判断) (LoadNetType = 1)");
                        }
                        // 其他情况 - 基于电压水平进行默认判断
                        else {
                            // 处理边缘情况 - 电压极高但有微弱下降
                            if (isVeryCloseToOpen && hasVerySlightDownwardTrend) {
                                LoadNetType = 0; // 高概率是开路
                                Serial.println("模式3检测到: 开路 (高电压微弱下降) (LoadNetType = 0)");
                            }
                            else if (isVeryCloseToOpen || isCloseToOpen) {
                                LoadNetType = 0; // 高电压默认为开路
                                Serial.println("模式3检测到: 开路 (默认高电压判断) (LoadNetType = 0)");
                            } else {
                                LoadNetType = 1; // 低中电压默认为RC串联
                                Serial.println("模式3检测到: RC串联 (默认判断) (LoadNetType = 1)");
                            }
                        }
                    }
                    break;
                }
            } else {
                Serial.println("数据点不足，无法判断负载网络类型");
            }
            
            // 打印统计信息
            Serial.print("Total samples: ");
            Serial.println(acReadingIndex);
            Serial.print("Target samples: ");
            Serial.println(maxSamples);
            Serial.print("Sampling time: ");
            Serial.print(currentTime - recordStartTime);
            Serial.println(" ms");
            Serial.print("Actual sampling rate: ");
            Serial.print((float)acReadingIndex * 1000.0 / (currentTime - recordStartTime), 2);
            Serial.println(" SPS");
            
            // 清空存储的采样数据
            for (int i = 0; i < maxSamples; i++) {
                ain1Readings[i] = 0.0;  // 将所有采样值重置为0
            }
            acReadingIndex = 0;  // 重置采样索引
            Serial.println("数据已清空，准备下一次采样");
            
            // 记录时间并进入等待状态
            lastActionTime = currentTime;
            commandState = 6;
            Serial.println("等待1000毫秒后重新开始...");
            break;
            
            case 6: // 延迟等待状态
            // 等待1000毫秒后回到初始状态，同时测量系统最高电压作为参考
            if (currentTime - lastActionTime >= 100 && currentTime - lastActionTime < 500) {
                // 发送固定频率命令来测量系统稳定时的电压值
                SerialMaster_SendCommand("SET_FREQ 1 10000 1023");
                // 标记已发送命令
                lastActionTime = currentTime + 400; // 调整时间确保不会重复发送
                Serial.println("发送固定频率命令测量系统参考电压...");
            } 
            else if (currentTime - lastActionTime >= 550 && 
                     currentTime - lastActionTime < 850) {
                // 采集电压值
                static float voltReadings[20] = {0};
                static int voltIndex = 0;
                
                if (voltIndex < 20) {
                    voltReadings[voltIndex] = AIN1_AC_Volt;
                    voltIndex++;
                } else {
                    // 计算平均值作为系统最高电压
                    float sum = 0.0;
                    for (int i = 0; i < 20; i++) {
                        sum += voltReadings[i];
                    }
                    float newMaxVoltage = sum / 20.0;
                    
                    // 更新系统最高电压（使用平滑过渡以避免突变）
                    if (newMaxVoltage > 0.5) { // 确保测量值有效
                        systemMaxVoltage = systemMaxVoltage * 0.3 + newMaxVoltage * 0.7;
                        Serial.print("更新系统参考电压: ");
                        Serial.println(systemMaxVoltage, 4);
                    }
                    
                    // 重置索引，准备下次测量
                    voltIndex = 0;
                    // 调整时间戳，确保只执行一次
                    lastActionTime = currentTime + 100;
                }
            }
            else if (currentTime - lastActionTime >= 1000) {
                commandState = 0; // 回到初始状态，重新执行LoadMode_DC
                switch(selectedMode)
                {
                    case 1: //
                        // 发送命令以设置频率和幅度
                        SerialMaster_SendCommand("SET_FREQ 1 87 1023");
                        break;
                    case 2:
                        // 发送命令以设置频率和幅度
                        SerialMaster_SendCommand("SET_FREQ 1 87 1023");
                        break;
                    case 3:
                        // 发送命令以设置频率和幅度
                        SerialMaster_SendCommand("SET_FREQ 1 2000000 1023");
                        break;
                }
                Serial.println("-----回到初始状态-----");
            }
            break;
        }
        }
    }
    break;

    case 3: // 故障距离
        static unsigned long previousMillis = 0; // 用于记录时间
        static bool lineModeOnExecuted = false; // 标记是否执行了LineMode_ON
        static float voltageReadings[100] = {0}; // 用于存储AIN2_Line_Volt的值
        static int readingIndex = 0; // 当前记录的索引

        if (RUNSTOP == true)
        {
            unsigned long currentMillis = millis();
            if (!lineModeOnExecuted)
            {
                LineMode_ON(); // 执行 LineMode_ON
                lineModeOnExecuted = true;
                previousMillis = currentMillis; // 记录当前时间
                readingIndex = 0; // 重置索引
            }
            else if (lineModeOnExecuted && (currentMillis - previousMillis >= 50) && (currentMillis - previousMillis < 150))
            {
                // 在LineMode_ON的中间100毫秒记录AIN2_Line_Volt的值
                if (readingIndex < 100)
                {
                    voltageReadings[readingIndex] = AIN2_Line_Volt; // 记录电压值
                    readingIndex++;
                }
            }
            else if (lineModeOnExecuted && (currentMillis - previousMillis >= 200))
            {
                LineMode_OFF(); // 执行 LineMode_OFF

                // 在LineMode_OFF期间计算平均值
                if (currentMillis - previousMillis < (200 + 1500))
                {
                    float sum = 0.0;
                    for (int i = 0; i < readingIndex; i++)
                    {
                        sum += voltageReadings[i];
                    }
                    LineMode_Volt_Avg = (readingIndex > 0) ? (sum / readingIndex) : 0.0; // 计算平均值
                }

                if (currentMillis - previousMillis >= (200 + 1500))
                {
                    lineModeOnExecuted = false; // 重置标记，准备下一次执行
                }
            }
        }
        else
        {
            LineMode_OFF();
        }
        break;
    default:
        break;
    }
}