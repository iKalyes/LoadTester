#ifndef _ADS122C04_H
#define _ADS122C04_H

#include <ADS122C04_Library.h>
#include <Arduino.h>
#include <lvgl.h>
#include <Wire.h>

#include <FreeRTOS.h>
#include <task.h>

#include <variables.h>

#define SDA_PIN 18
#define SCL_PIN 19
#define DRDY_PIN 14
#define RESET_PIN 11

void ADS122C04_init();
void ADS122C04_task();

#endif