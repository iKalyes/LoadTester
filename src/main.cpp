#include <main.h>

void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */
    GPIO_init();
    display_init();
    encoder_init();
    lvgl_group_init();
    
    ADS122C04_DATA_CHART_INIT();
}

void loop()
{
    lvgl_task_handler();
    ADS122C04_DATA_REFRESH();
    SerialMaster_FRESH();
}

void setup1()
{
    ADS122C04_init();
    SerialMaster_Init();
}

// 可单独调整两个函数的执行时间
uint16_t lineModeOnDuration = 200;  // LineMode_ON 持续时间（毫秒）
uint16_t lineModeOffDuration = 1500; // LineMode_OFF 持续时间（毫秒）

void loop1()
{
    SerialMaster_Update();
    ADS122C04_task();

    switch (MeasureMode)
    {
    case 0: // 通断测试
        if (RUNSTOP == true)
        {
            if (AIN0_DC_Volt < 0.1)
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
            // 如果连接已建立，可以发送命令
            if (SerialMaster_IsConnected()) {
            static unsigned long lastActionTime = 0;
            static int commandState = 0; // 0: 准备发送第一条命令, 1: 等待发送第二条命令
            unsigned long currentTime = millis();
    
            switch (commandState) {
            case 0: // 准备发送第一条命令
                if (currentTime - lastActionTime > DDS_SweepFreqTimems + 1000) {  // 间隔1秒发送第一条命令
                SerialMaster_SendCommand("SET_SWEEP 1 1000 100000");
                lastActionTime = currentTime;
                commandState = 1;
            }   
            break;
        
            case 1: // 检查第一条命令
                if (SerialMaster_IsCommandSuccessful()) {
                lastActionTime = currentTime;
                commandState = 0;
            }
            break;
            }
            }
        }

        else
        {

        }
        break;
    case 2: // 负载网络

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
                if (currentMillis - previousMillis < (200 + lineModeOffDuration))
                {
                    float sum = 0.0;
                    for (int i = 0; i < readingIndex; i++)
                    {
                        sum += voltageReadings[i];
                    }
                    LineMode_Volt_Avg = (readingIndex > 0) ? (sum / readingIndex) : 0.0; // 计算平均值
                }

                if (currentMillis - previousMillis >= (200 + lineModeOffDuration))
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


