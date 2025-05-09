#include <gpio.h>

void GPIO_init()
{
    pinMode(RELAY2, OUTPUT); // RELAY2 - VOUT/I_SOURCE+
    pinMode(RELAY1, OUTPUT); // RELAY1 - VOUT_AC/VOUT_DC
    pinMode(RELAY3, OUTPUT); // RELAY3 - GND/I_SOURCE-
    pinMode(RELAY4, OUTPUT); // RELAY4 - 阻抗切换 50Ω/2kΩ
    
    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, HIGH);
    digitalWrite(RELAY3, HIGH);
    digitalWrite(RELAY4, HIGH);

    pinMode(BUZZER, OUTPUT);
    analogWriteFreq(2700);
    analogWriteRange(100);
    analogWrite(BUZZER, 0);
}

void ContMode()
{
    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, HIGH);
    digitalWrite(RELAY3, HIGH);
    BuzzerOff();
}

void LCRMode()
{
    digitalWrite(RELAY1, HIGH);
    digitalWrite(RELAY2, HIGH);
    digitalWrite(RELAY3, HIGH);
    digitalWrite(RELAY4, HIGH);
    BuzzerOff();
}

void LCRMode_AC()
{
    digitalWrite(RELAY1, HIGH);
}

void LCRMode_DC()
{
    digitalWrite(RELAY1, LOW);
}

void LCRMode_50()
{
    digitalWrite(RELAY4, HIGH);
}

void LoadMode()
{
    digitalWrite(RELAY1, HIGH);
    digitalWrite(RELAY2, HIGH);
    digitalWrite(RELAY3, HIGH);
    BuzzerOff();
}

void LoadMode_AC()
{
    digitalWrite(RELAY1, HIGH);
}

void LoadMode_DC()
{
    digitalWrite(RELAY1, LOW);
}

void LoadMode_50()
{
    digitalWrite(RELAY4, HIGH);
}

void LoadMode_1K()
{
    digitalWrite(RELAY4, LOW);
}

void LineMode()
{
    digitalWrite(RELAY1, HIGH);
    digitalWrite(RELAY2, LOW);
    digitalWrite(RELAY3, LOW);
    digitalWrite(RELAY4, HIGH);
    BuzzerOff();
}

void LineMode_ON()
{
    digitalWrite(RELAY2, LOW);
    digitalWrite(RELAY3, LOW);
}

void LineMode_OFF()
{
    digitalWrite(RELAY2, HIGH);
    digitalWrite(RELAY3, HIGH);
}

void BuzzerOn()
{
    analogWrite(BUZZER, 50);
}

void BuzzerOff()
{
    analogWrite(BUZZER, 0);
}
