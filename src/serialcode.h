#ifndef _SERIALCODE_H_
#define _SERIALCODE_H_

#include <Arduino.h>
#include <variables.h>
#include <lvgl.h>
#include "ui/ui.h"

void SerialMaster_Init();
bool SerialMaster_IsConnected();
bool SerialMaster_SendCommand(const String &command);
void SerialMaster_ProcessReceivedData(const String &data);
void SerialMaster_Update();
void SerialMaster_Disconnect();
String SerialMaster_GetLastResponse();
bool SerialMaster_IsCommandSuccessful();
void SerialMaster_FRESH();

#endif