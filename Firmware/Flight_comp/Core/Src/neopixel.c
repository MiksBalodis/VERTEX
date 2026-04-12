#include "neopixel.h"

PixelRGB_t pixel[NUM_PX] = {0};
uint32_t dmaBuffer[DMA_BUFF] = {0};


void LED_Set_Color(uint8_t r, uint8_t g, uint8_t b){
  pixel[0].color.r = r;
  pixel[0].color.b = b;
  pixel[0].color.g = g;

  uint32_t *pBuff;
  pBuff = dmaBuffer;
  int8_t j;
  for (j = 23; j >= 0; j--){
    if ((pixel[0].data >> j) & 0x01){
      *pBuff = NEOPIXEL_1;
    }else{
      *pBuff = NEOPIXEL_0;
    }
    pBuff++;
  }
  dmaBuffer[DMA_BUFF - 1] = 0;

  HAL_TIM_PWM_Start_DMA(&NEOPIXEL_TIM, NEOPIXEL_CH, dmaBuffer, DMA_BUFF);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
  HAL_TIM_PWM_Stop_DMA(htim, NEOPIXEL_CH);
}
