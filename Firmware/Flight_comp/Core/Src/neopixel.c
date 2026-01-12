#include "neopixel.h"

PixelRGB_t pixel[NUM_PX] = {0};
uint32_t dmaBuffer[DMA_BUFF] = {0};

void LED_Update(){
    uint32_t *pBuff;
    pBuff = dmaBuffer;
    for (uint8_t i = 0; i < NUM_PX; i++)
    {
       for (uint8_t j = 23; j >= 0; j--)
       {
         if ((pixel[i].data >> j) & 0x01)
         {
           *pBuff = NEOPIXEL_1;
         }
         else
         {
           *pBuff = NEOPIXEL_0;
         }
         pBuff++;
     }
    }
    dmaBuffer[DMA_BUFF - 1] = 0;

    HAL_TIM_PWM_Start_DMA(&NEOPIXEL_TIM, NEOPIXEL_CH, dmaBuffer, DMA_BUFF);
}

void LED_Set_Color(uint8_t led, uint8_t r, uint8_t g, uint8_t b){
    pixel[led].color.r = r;
    pixel[led].color.b = b;
    pixel[led].color.g = g;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, NEOPIXEL_CH);
}