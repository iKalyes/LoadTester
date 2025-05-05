#include <encoder.h>

RotaryEncoder *encoder = NULL;

void encoder_tick()  // Interrupt Service Routine for the encoder
{
    encoder->tick();
}

// 跟踪编码器状态
static int last_encoder_dir = 0;
static bool encoder_btn_pressed = false;
static lv_indev_t* encoder_indev = NULL;

// 编码器读取回调函数
static void encoder_read(lv_indev_drv_t* drv, lv_indev_data_t* data)
{
    // 检测旋转
    int current_dir = (int)(encoder->getDirection());
    if(current_dir == -1) {
        data->enc_diff = 1;  // 顺时针
    } else if(current_dir == 1) {
        data->enc_diff = -1; // 逆时针
    } else {
        data->enc_diff = 0;  // 无变化
    }
    last_encoder_dir = current_dir;
    
    // 检测按钮状态 (Pin 4)
    bool btn_state = !digitalRead(15); // 假设低电平为按下
    data->state = btn_state ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    encoder_btn_pressed = btn_state;
}

// 初始化LVGL编码器输入设备
void encoder_init()
{
    // 确保编码器已初始化
    encoder = new RotaryEncoder(17, 16, RotaryEncoder::LatchMode::FOUR3);
    attachInterrupt(digitalPinToInterrupt(17), encoder_tick, CHANGE); // Attach interrupt to pin 16
    attachInterrupt(digitalPinToInterrupt(16), encoder_tick, CHANGE); // Attach interrupt to pin 17
    pinMode(15, INPUT); // Pin 15 for SW
    
    // 注册LVGL输入设备
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = encoder_read;
    
    encoder_indev = lv_indev_drv_register(&indev_drv);
    
    // 初始化位置
    last_encoder_dir = (int)(encoder->getDirection());
}

// 获取编码器输入设备指针
lv_indev_t* get_encoder_indev()
{
    return encoder_indev;
}