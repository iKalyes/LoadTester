#ifndef _GPIO_H_
#define _GPIO_H_

#include <Arduino.h>

#define RELAY1 21
#define RELAY2 20
#define RELAY3 22
#define BUZZER 26

void GPIO_init();
void ContMode();
void LCRMode();
void LoadMode();
void LineMode();
void BuzzerOn();
void BuzzerOff();
void LineMode_ON();
void LineMode_OFF();

#endif