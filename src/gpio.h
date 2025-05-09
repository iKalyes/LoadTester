#ifndef _GPIO_H_
#define _GPIO_H_

#include <Arduino.h>

#define RELAY1 21
#define RELAY2 20
#define RELAY3 22
#define RELAY4 23
#define BUZZER 26

void GPIO_init();
void ContMode();
void LCRMode();
void LCRMode_AC();
void LCRMode_DC();
void LCRMode_50();
void LoadMode();
void LoadMode_AC();
void LoadMode_DC();
void LoadMode_50();
void LoadMode_1K();
void LineMode();
void LineMode_ON();
void LineMode_OFF();
void BuzzerOn();
void BuzzerOff();



#endif