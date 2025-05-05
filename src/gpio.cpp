#include <gpio.h>

void GPIO_init()
{
    pinMode(RELAY2, OUTPUT); // RELAY2 - VOUT/I_SOURCE+
    pinMode(RELAY1, OUTPUT); // RELAY1 - VOUT_AC/VOUT_DC
    pinMode(RELAY3, OUTPUT); // RELAY3 - GND/I_SOURCE-
    
    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, HIGH);
    digitalWrite(RELAY3, HIGH);

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
    BuzzerOff();
}

void LoadMode()
{

}

void LineMode()
{
    digitalWrite(RELAY1, HIGH);
    digitalWrite(RELAY2, LOW);
    digitalWrite(RELAY3, LOW);
    BuzzerOff();
}

void BuzzerOn()
{
    analogWrite(BUZZER, 50);
}

void BuzzerOff()
{
    analogWrite(BUZZER, 0);
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