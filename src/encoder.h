#ifndef _ENCODER_H
#define _ENCODER_H

#include <Arduino.h>
#include <RotaryEncoder.h>
#include <lvgl.h>

void encoder_tick();  // Interrupt Service Routine for the encoder
void encoder_init();
lv_indev_t* get_encoder_indev();

#endif