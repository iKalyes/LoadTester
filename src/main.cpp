#include <main.h>

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

static float systemMaxVoltage = 1.7; // 默认值为1.7V，后续会动态更新

void setup1()
{

}

void loop1()
{
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
            static float ain0Readings[128] = {0}; // 存储AIN0_DC_Volt的采样值
            static int dcReadingIndex = 0; // AIN0采样索引
            static unsigned long modeStartTime = 0; // DC模式开始时间
            static unsigned long recordStartTime = 0; // 记录开始时间
            static float dcVoltAvg = 0.0; // 存储DC电压平均值
            static int componentType = 0; // 0: 未知, 1: 电阻, 2: 电容, 3: 电感
            static float componentValue = 0.0; // 元件值
            unsigned long currentTime = millis();
            unsigned long currentMicros = micros();
    
            switch (commandState) {
                case 0: // 执行DC模式并等待前50ms
                    LCRMode_AC_OR_DC = false; // 设置为DC模式
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
                            LCRMode_AC_OR_DC = true; // 设置为AC模式
                            LCRMode_AC(); // 执行LCRMode_AC函数
                            LCRMode_50(); // 执行LCRMode_50函数
                            Serial.print("检测到电感，平均电压: ");
                        } else if (dcVoltAvg > 0.8) {
                            componentType = 2; // 电容
                            LCRMode_AC_OR_DC = true; // 设置为AC模式
                            LCRMode_AC(); // 执行LCRMode_AC函数
                            LCRMode_50(); // 执行LCRMode_50函数
                            Serial.print("检测到电容，平均电压: ");
                        } else {
                            componentType = 1; // 电阻
                            LCRMode_AC_OR_DC = false; // 设置为DC模式
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
                        // 电容和电感的情况（暂不填写具体代码）
                        else if (componentType == 2) { // 电容
                            // 电容测量代码待实现
                            if (currentTime - recordStartTime >= 1000) {
                                commandState = 4; // 进入计算元件值状态
                            }
                        }
                        else if (componentType == 3) { // 电感
                            // 电感测量代码待实现
                            if (currentTime - recordStartTime >= 1000) {
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
                            
                            // 输出计算结果
                            Serial.print("电阻值: ");
                            Serial.print(componentValue, 2);
                            Serial.println(" 欧姆");
                        } else {
                            Serial.println("电压异常，无法准确计算电阻值");
                        }
                    }
                    else if (componentType == 2) { // 电容
                        Serial.println("电容测量功能尚未实现");
                        // 这里将来实现电容测量代码
                    }
                    else if (componentType == 3) { // 电感
                        Serial.println("电感测量功能尚未实现");
                        // 这里将来实现电感测量代码
                    }
                    
                    // 清空存储的采样数据
                    for (int i = 0; i < 128; i++) {
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
                // 根据平均值选择模式并设置参数 - 使用动态参考电压
                if (dcVoltAvg < systemMaxVoltage * 0.06) { // 约6%的系统参考电压
                    selectedMode = 1;
                    LoadMode_50(); // 执行LoadMode_50
                    SerialMaster_SendCommand("SET_SWEEP 1 1000 100000 1023 2000 100");
                    Serial.print("进入模式1，平均电压: ");
                } else if (dcVoltAvg > systemMaxVoltage * 0.56) { // 约65%的系统参考电压
                    selectedMode = 3;
                    LoadMode_1K(); // 执行LoadMode_1K
                    SerialMaster_SendCommand("SET_SWEEP 1 100 1000000 1023 1000 2000000");
                    Serial.print("进入模式3，平均电压: ");
                } else {
                    selectedMode = 2;
                    LoadMode_1K(); // 执行LoadMode_1K
                    SerialMaster_SendCommand("SET_SWEEP 1 10000 2000000 1023 1000 100");
                    Serial.print("进入模式2，平均电压: ");
                }
                
                // 增加比例信息输出，便于调试
                Serial.println(dcVoltAvg, 4);
                Serial.print("(相对比例: "); 
                Serial.print(dcVoltAvg / systemMaxVoltage * 100.0, 1);
                Serial.println("%)");
                Serial.print("采样点数: ");
                Serial.println(dcReadingIndex);
                
                // 开始记录AC电压值
                recordStartTime = currentTime;
                acReadingIndex = 0;
                lastSampleTime = currentMicros;
                commandState = 4; // 进入记录数据状态
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
                        
                        // 判断是否有峰值 - 通过比较峰值与起始/结束电压
                        hasPeak = (maxVoltageIndex > acReadingIndex * 0.1) && 
                                 (maxVoltageIndex < acReadingIndex * 0.9) && 
                                 (maxVoltage > startAvg * 1.5) && 
                                 (maxVoltage > endAvg * 1.5);
                        
                        // 计算频率映射参数 - 在2000ms内从1000Hz线性变化到100000Hz
                        float timePerSample = 1000.0 / 175.0; // 每个样本的时间间隔(ms)
                        float freqRatio = 99000.0 / 2000.0; // 每ms频率增长率 (Hz/ms)
                        
                        // === 新增的多特征分析代码开始 ===
                        
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
                        
                        // === 多特征分析代码结束 ===
                        
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
                            // 创建多特征综合评分
                            int lcScore = 0;
                            int rlcScore = 0;
                            
                            // 1. 峰值位置特征 - 降低权重，调整判断逻辑
                            float peakTimeRatio = (maxVoltageIndex * timePerSample) / 2000.0; // 峰值出现在扫频过程中的相对位置
                            if (peakTimeRatio < 0.15) { // 非常早期峰值
                                rlcScore += 2;  // 降低权重从4分到2分
                                Serial.println("峰值位置特征: 非常早期峰值 (RLC +2)");
                            } else if (peakTimeRatio >= 0.15 && peakTimeRatio < 0.25) {
                                rlcScore += 1;  // 降低权重从3分到1分
                                Serial.println("峰值位置特征: 较早期峰值 (RLC +1)");
                            } else if (peakTimeRatio >= 0.25 && peakTimeRatio < 0.35) {
                                // 这个范围多为LC并联，不加分给RLC
                                Serial.println("峰值位置特征: 中早期峰值 (不计分)");
                            } else if (peakTimeRatio >= 0.35 && peakTimeRatio < 0.40) {
                                lcScore += 1;
                                Serial.println("峰值位置特征: 中间位置峰值 (LC +1)");
                            } else if (peakTimeRatio >= 0.40 && peakTimeRatio < 0.45) {
                                lcScore += 2;
                                Serial.println("峰值位置特征: 较晚峰值 (LC +2)");
                            } else {
                                lcScore += 3;
                                Serial.println("峰值位置特征: 晚期峰值 (LC +3)");
                            }
                            
                            // 2. 谐振频率特征 - 调整判断区间
                            if (resonanceFreq < 10000) { // 降低RLC判断的上限
                                rlcScore += 2;
                                Serial.print("谐振频率特征: "); Serial.print(resonanceFreq); Serial.println(" (RLC +2)");
                            } else if (resonanceFreq >= 10000 && resonanceFreq < 20000) {
                                // 这个范围LC和RLC都有可能，不加分
                                Serial.print("谐振频率特征: "); Serial.print(resonanceFreq); Serial.println(" (中间范围，不计分)");
                            } else if (resonanceFreq >= 20000) { // 提高LC判断的下限
                                lcScore += 2;
                                Serial.print("谐振频率特征: "); Serial.print(resonanceFreq); Serial.println(" (LC +2)");
                            }
                            
                            // 3. Q值特征 - 降低权重和重新调整区间
                            if (qValue < 1.25) {
                                lcScore += 1;  // 降低权重从2到1
                                Serial.print("Q值特征: "); Serial.print(qValue); Serial.println(" (LC +1)");
                            } else if (qValue >= 1.25 && qValue < 1.40) {
                                // 这个范围不明确，不给额外分数
                                Serial.print("Q值特征: "); Serial.print(qValue); Serial.println(" (中间范围，不计分)");
                            } else if (qValue >= 1.40 && qValue < 1.60) {
                                rlcScore += 1;  // 降低权重从2到1
                                Serial.print("Q值特征: "); Serial.print(qValue); Serial.println(" (RLC +1)");
                            } else {
                                lcScore += 1;
                                Serial.print("Q值特征: "); Serial.print(qValue); Serial.println(" (LC +1)");
                            }
                            
                            // 4. 不对称性特征 - 调整判断标准
                            if (curveAsymmetry < 0.8) {
                                rlcScore += 1;
                                Serial.print("不对称性特征: "); Serial.print(curveAsymmetry); Serial.println(" (RLC +1)");
                            } else if (curveAsymmetry > 3.0) { // 提高阈值
                                lcScore += 1;
                                Serial.print("不对称性特征: "); Serial.print(curveAsymmetry); Serial.println(" (LC +1)");
                            } else {
                                // 中间范围不做判断
                                Serial.print("不对称性特征: "); Serial.print(curveAsymmetry); Serial.println(" (中间范围，不计分)");
                            }
                            
                            // 5. 衰减方差特征 - 重新定义阈值
                            if (decayVariance < 0.006) {
                                lcScore += 1;  // 降低权重
                                Serial.print("衰减方差特征: "); Serial.print(decayVariance); Serial.println(" (LC +1)");
                            } else if (decayVariance >= 0.014 && decayVariance < 0.017) {
                                rlcScore += 1;  // 降低权重
                                Serial.print("衰减方差特征: "); Serial.print(decayVariance); Serial.println(" (RLC +1)");
                            } else {
                                // 其他值不做明确判断
                                Serial.print("衰减方差特征: "); Serial.print(decayVariance); Serial.println(" (中间范围，不计分)");
                            }
                            
                            // 6. 峰值宽度特征 - 独立判断，不与位置关联
                            if (peakWidth < 30) {
                                lcScore += 1;  // LC并联可能有较窄峰
                                Serial.print("峰值宽度特征: "); Serial.print(peakWidth); Serial.println(" (LC +1)");
                            } else if (peakWidth > 50) {
                                rlcScore += 1;  // RLC并联可能有较宽峰
                                Serial.print("峰值宽度特征: "); Serial.print(peakWidth); Serial.println(" (RLC +1)");
                            } else {
                                Serial.print("峰值宽度特征: "); Serial.print(peakWidth); Serial.println(" (中间范围，不计分)");
                            }
                            
                            // 7. 增加新特征：峰值电压与频率比值
                            // LC并联通常峰值电压较高但频率较高，RLC并联峰值电压较低但频率也较低
                            float peakFreqRatio = maxVoltage * 10000 / resonanceFreq;
                            if (peakFreqRatio > 0.5) {
                                lcScore += 1;
                                Serial.print("峰值/频率比: "); Serial.print(peakFreqRatio, 3); Serial.println(" (LC +1)");
                            } else if (peakFreqRatio < 0.3) {
                                rlcScore += 1;
                                Serial.print("峰值/频率比: "); Serial.print(peakFreqRatio, 3); Serial.println(" (RLC +1)");
                            } else {
                                Serial.print("峰值/频率比: "); Serial.print(peakFreqRatio, 3); Serial.println(" (中间范围，不计分)");
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
                            
                            // 判断逻辑 - 优先考虑整体评分
                            if (lcScore > rlcScore) {
                                LoadNetType = 7; // LC并联
                                Serial.println("模式1检测到: LC并联 (LoadNetType = 7)");
                            } else if (rlcScore > lcScore) {
                                LoadNetType = 8; // RLC并联
                                Serial.println("模式1检测到: RLC并联 (LoadNetType = 8)");
                            } else {
                                // 得分相同时，基于更可靠的谐振频率特征判断
                                if (resonanceFreq < 10000) {
                                    LoadNetType = 8; // RLC并联
                                    Serial.println("模式1检测到: RLC并联 (基于谐振频率判断) (LoadNetType = 8)");
                                } else if (resonanceFreq > 20000) {
                                    LoadNetType = 7; // LC并联
                                    Serial.println("模式1检测到: LC并联 (基于谐振频率判断) (LoadNetType = 7)");
                                } else {
                                    // 如果谐振频率也在中间范围，使用峰值时间点判断
                                    if (maxVoltageIndex * timePerSample < 160) { // 非常早期峰值(<160ms)更可能是RLC
                                        LoadNetType = 8; // RLC并联
                                        Serial.println("模式1检测到: RLC并联 (基于极早期峰值判断) (LoadNetType = 8)");
                                    } else {
                                        LoadNetType = 7; // LC并联
                                        Serial.println("模式1检测到: LC并联 (默认判断) (LoadNetType = 7)");
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
                            
                            // 计算最后1/4的平均值
                            for (int i = acReadingIndex - quarterSize; i < acReadingIndex; i++) {
                                lastQuarterAvg += ain1Readings[i];
                            }
                            lastQuarterAvg /= quarterSize;
                            
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
                            
                            for (int i = laterStartIndex; i < acReadingIndex; i++) {
                                laterAvg += ain1Readings[i];
                            }
                            laterAvg /= (acReadingIndex - laterStartIndex);
                            
                            for (int i = laterStartIndex; i < acReadingIndex; i++) {
                                laterVariance += pow(ain1Readings[i] - laterAvg, 2);
                            }
                            laterVariance /= (acReadingIndex - laterStartIndex);
                            
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

                            // 判断逻辑 - 更新阈值和条件
                            bool isStable = variance < 0.0005; // 放宽整体稳定性阈值
                            bool isLaterStable = laterVariance < 0.0001; // 后段高度稳定
                            bool isRising = lastQuarterAvg > firstQuarterAvg * 1.1; // 上升趋势判断
                            bool hasSignificantDrop = initialMaxVoltage > 0 && 
                                                     (initialMaxVoltage / lowestVoltageAfterDrop) > 1.5;  
                            bool lowMinimum = minVoltage < systemMaxVoltage * 0.12; // 约为最高电压的12%
                            bool moderateMinimum = minVoltage >= (systemMaxVoltage * 0.2) && 
                                 minVoltage <= (systemMaxVoltage * 0.88); // 使用相对比例
                            bool stableValue = maxVoltage / minVoltage < 1.2; // 放宽最大最小值比值阈值
                            bool isCloseToOpen = avgVoltage > (systemMaxVoltage * 0.94); // 改为接近系统最高电压的94%
                            
                            // RC串联特征 - 初始有波动后趋于稳定
                            bool hasRCCharacteristic = (initialHighCount > 0) && isLaterStable && 
                                                       (avgVoltage > systemMaxVoltage * 0.18 && 
                                                        avgVoltage < systemMaxVoltage * 0.88) &&
                                                        !isRising;
                            
                            // 输出诊断信息
                            Serial.println("\n负载网络诊断信息:");
                            Serial.print("最大电压: "); Serial.println(maxVoltage, 6);
                            Serial.print("最小电压: "); Serial.println(minVoltage, 6);
                            Serial.print("平均电压: "); Serial.println(avgVoltage, 6);
                            Serial.print("电压方差: "); Serial.println(variance, 6);
                            Serial.print("后段方差: "); Serial.println(laterVariance, 6);
                            Serial.print("首尾比值: "); Serial.println(lastQuarterAvg/firstQuarterAvg, 6);
                            Serial.print("下降比值: "); 
                            Serial.println(initialMaxVoltage > 0 ? initialMaxVoltage/lowestVoltageAfterDrop : 0, 6);
                            Serial.print("初始高点数: "); Serial.println(initialHighCount);
                            

                            // 识别负载网络类型
                            if (isStable && isCloseToOpen) {
                                // 开路情况：非常稳定且接近系统最高电压
                                LoadNetType = 0; // 开路
                                Serial.println("模式3检测到: 开路 (LoadNetType = 0)");
                            } 
                            // 新增: LC串联判断条件优先 - 初始高点、极低最低点、后续线性上升
                            else if (initialHighCount >= 3 && 
                                    minVoltage < 0.1 && // 极低的最小值
                                    initialMaxVoltage / minVoltage > 10.0 && // 高比例的下降
                                    (hasLinearRising || (isRising && lastQuarterAvg / firstQuarterAvg > 1.5))) { // 线性上升或强上升趋势
                                // LC串联：典型的高-极低-线性上升特征
                                LoadNetType = 2; // LC串联
                                Serial.println("模式3检测到: LC串联 (LoadNetType = 2)");
                            }
                            else if (hasRCCharacteristic || (isStable && !isCloseToOpen)) {
                                // RC串联：稳定但明显低于开路电压，或符合RC特征曲线
                                LoadNetType = 1; // RC串联
                                Serial.println("模式3检测到: RC串联 (LoadNetType = 1)");
                            }
                            // 修改: RLC串联特征识别添加条件，避免与LC串联混淆
                            else if ((initialHighCount >= 3) && 
                                    (initialMaxVoltage / minVoltage > 1.3 && initialMaxVoltage / minVoltage < 10.0) && // 增加上限
                                    (lastQuarterAvg / firstQuarterAvg > 1.02) &&
                                    minVoltage >= 0.1) { // 确保最低点不是极低
                                // RLC串联：具有初始高点后下降再上升的特征模式，但下降幅度适中
                                LoadNetType = 3; // RLC串联
                                Serial.println("模式3检测到: RLC串联 (特征模式判断) (LoadNetType = 3)");
                            }
                            else if (isRising && lowMinimum) {
                                // LC串联：有明显上升趋势，且最低点接近0
                                LoadNetType = 2; // LC串联
                                Serial.println("模式3检测到: LC串联 (LoadNetType = 2)");
                            }
                            // 放宽RLC串联上升趋势判定条件：从1.1降至1.05
                            else if ((isRising || lastQuarterAvg > firstQuarterAvg * 1.05) && moderateMinimum) {
                                // RLC串联：有上升趋势，且最低点适中
                                LoadNetType = 3; // RLC串联
                                Serial.println("模式3检测到: RLC串联 (LoadNetType = 3)");
                            }
                            else if (isRising) {
                                // 有上升趋势但不符合明确特征的，默认判断为RLC串联
                                LoadNetType = 3; // RLC串联(默认)
                                Serial.println("模式3检测到: RLC串联(默认判断) (LoadNetType = 3)");
                            }
                            else if (stableValue && !isCloseToOpen && avgVoltage > systemMaxVoltage * 0.12) {
                                // 补充条件：电压稳定在中等水平，可能是RC串联
                                LoadNetType = 1; // RC串联(补充判断)
                                Serial.println("模式3检测到: RC串联(补充判断) (LoadNetType = 1)");
                            }
                            // 新增辅助判断：初始下降后略微上升，可能是RLC串联
                            else if (initialHighCount > 0 && 
                                    minVoltage > 0.3 && 
                                    variance > 0.005) {  // 方差大表示波动明显
                                LoadNetType = 3; // RLC串联(辅助判断)
                                Serial.println("模式3检测到: RLC串联(辅助判断) (LoadNetType = 3)");
                            }
                            else {
                                // 无法判断
                                LoadNetType = 0; // 默认开路
                                Serial.println("负载网络判断不明确，默认为开路");
                            }
                        }
                        break;
                                            default:
                        Serial.println("未知模式，无法判断负载网络类型");
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
                        SerialMaster_SendCommand("SET_FREQ 1 100 1023");
                        break;
                    case 2:
                        // 发送命令以设置频率和幅度
                        SerialMaster_SendCommand("SET_FREQ 1 100 1023");
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


