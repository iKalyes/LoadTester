#include <serialcode.h>
#include <Arduino.h>

// 主机连接状态枚举
enum HostConnectionState {
    HOST_DISCONNECTED,   // 未连接状态
    HOST_WAIT_CONNECTED, // 已发送连接响应，等待Connected消息
    HOST_CONNECTED       // 连接已建立
};

// 全局变量
HostConnectionState hostState = HOST_DISCONNECTED;
String receivedBuffer = "";
unsigned long lastCommandTime = 0;
bool isCommandPending = false;
String pendingCommandResponse = "";

// 超时设置
const unsigned long CONNECTION_TIMEOUT = 1000;    // 连接超时(毫秒)
const unsigned long COMMAND_TIMEOUT = 1000;       // 命令响应超时(毫秒)

/**
 * 初始化串口通信
 */
void SerialMaster_Init() {
    Serial2.setRX(25); // 设置RX引脚
    Serial2.setTX(24); // 设置TX引脚
    Serial2.begin(115200); // 与从机通信的串口
    Serial.println("串口主机初始化完成");
}

/**
 * 检查连接状态
 * @return bool 是否已连接
 */
bool SerialMaster_IsConnected() {
    return (hostState == HOST_CONNECTED);
}

/**
 * 向从机发送命令
 * @param command 要发送的命令
 * @return bool 是否成功发送
 */
bool SerialMaster_SendCommand(const String &command) {
    if (hostState != HOST_CONNECTED) {
        Serial.println("错误: 尚未连接到从机");
        return false;
    }
    
    Serial.print("发送命令: ");
    Serial.println(command);
    Serial2.print(command + "\n");
    
    isCommandPending = true;
    lastCommandTime = millis();
    
    return true;
}

/**
 * 处理接收到的从机数据
 * @param data 接收到的数据
 */
void SerialMaster_ProcessReceivedData(const String &data) {
    Serial.print("接收: ");
    Serial.println(data);
    
    // 处理连接相关的消息
    if (data == "ConnectionCheck" && hostState == HOST_DISCONNECTED) {
        Serial.println("收到连接请求，发送响应");
        Serial2.print("ConnectionResponse\n");
        hostState = HOST_WAIT_CONNECTED;
    }
    else if (data == "Connected" && hostState == HOST_WAIT_CONNECTED) {
        Serial.println("连接成功建立!");
        hostState = HOST_CONNECTED;
    }
    // 处理断开连接消息
    else if (data == "Disconnected" && hostState == HOST_CONNECTED) {
        Serial.println("从机已断开连接!");
        // 重置状态为未连接
        hostState = HOST_DISCONNECTED;
        isCommandPending = false;
        pendingCommandResponse = "";
    }
    // 处理命令响应
    else if (hostState == HOST_CONNECTED) {
        if (data == "SET_FREQ") {
            Serial.println("频率命令处理成功");
            isCommandPending = false;
            pendingCommandResponse = "ProcessSuccess";
        }
        else if (data.startsWith("SET_SWEEP")) {
            // 处理SET_SWEEP命令，提取扫频时间
            int spaceIndex = data.indexOf(' ');
            if (spaceIndex > 0 && spaceIndex < data.length() - 1) {
                String timeValue = data.substring(spaceIndex + 1);
                DDS_SweepFreqTimems = (uint16_t)timeValue.toInt();
                Serial.print("获取到扫频时间值: ");
                Serial.println(DDS_SweepFreqTimems);
            }
            
            Serial.println("扫频命令处理成功");
            isCommandPending = false;
            pendingCommandResponse = "ProcessSuccess";
        }
        else if (data == "SET_Failed") {
            Serial.println("命令处理失败");
            isCommandPending = false;
            pendingCommandResponse = "ProcessFailed";
        }
    }
}

/**
 * 更新串口通信状态
 * 主循环中需要定期调用此函数
 */
void SerialMaster_Update() {
    unsigned long currentTime = millis();
    
    // 状态机管理
    switch (hostState) {
        case HOST_DISCONNECTED:
            // 等待从机的连接请求
            break;
            
        case HOST_WAIT_CONNECTED:
            // 在实际应用中可能需要超时机制
            break;
            
        case HOST_CONNECTED:
            // 检查命令响应超时
            if (isCommandPending && currentTime - lastCommandTime > COMMAND_TIMEOUT) {
                Serial.println("命令响应超时");
                isCommandPending = false;
            }
            break;
    }
    
    // 处理接收到的数据
    while (Serial2.available()) {
        char c = Serial2.read();
        if (c == '\n') {
            String completeMessage = receivedBuffer;
            completeMessage.trim();
            SerialMaster_ProcessReceivedData(completeMessage);
            receivedBuffer = "";
        } else {
            receivedBuffer += c;
        }
    }
}

/**
 * 断开与从机的连接
 */
void SerialMaster_Disconnect() {
    if (hostState == HOST_CONNECTED) {
        Serial.println("断开连接");
        hostState = HOST_DISCONNECTED;
    }
}

/**
 * 获取上一条命令的响应
 * @return String 命令响应
 */
String SerialMaster_GetLastResponse() {
    String response = pendingCommandResponse;
    pendingCommandResponse = "";
    return response;
}

/**
 * 判断命令是否执行成功
 * @return bool 命令是否成功
 */
bool SerialMaster_IsCommandSuccessful() {
    return (pendingCommandResponse == "ProcessSuccess");
}

/**
 * 获取从机返回的扫频时间
 * @return uint16_t 扫频时间（毫秒）
 */
uint16_t SerialMaster_GetSweepFreqTime() {
    return DDS_SweepFreqTimems;
}

void SerialMaster_FRESH()
{
    if (hostState == HOST_CONNECTED)
    {
        // 更新UI显示
        lv_img_set_src(ui_ImageSerial, &ui_img_2018925606);
        lv_label_set_text(ui_SerialStatus, "已连接");
        lv_obj_set_style_text_color(ui_SerialStatus, lv_color_hex(0x00FF00), 0);
    }
    else if (hostState == HOST_DISCONNECTED)
    {
        // 更新UI显示
        lv_img_set_src(ui_ImageSerial, &ui_img_281677161); // 假设这是断开连接的图标
        lv_label_set_text(ui_SerialStatus, "未连接");
        lv_obj_set_style_text_color(ui_SerialStatus, lv_color_hex(0xFF0000), 0);
    }
}

