#pragma once

#include "main.h"
#include "stdint.h" 

typedef union
{
  struct
  {
    uint8_t b;
    uint8_t r;
    uint8_t g;
  } color;
  uint32_t data;
} PixelRGB_t;

extern TIM_HandleTypeDef htim3;

#define NEOPIXEL_0  30
#define NEOPIXEL_1  60

#define NUM_PX      1
#define DMA_BUFF    (NUM_PX*24)+1

#define NEOPIXEL_TIM    htim3
#define NEOPIXEL_CH     TIM_CHANNEL_1

void LED_Set_Color(uint8_t led, uint8_t r, uint8_t g, uint8_t b);
void LED_Update();