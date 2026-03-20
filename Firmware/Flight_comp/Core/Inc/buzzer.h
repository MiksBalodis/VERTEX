#pragma once

#include "main.h"
#include "stdint.h"

extern TIM_HandleTypeDef htim7;

typedef struct{
    uint32_t ticks;
} Buzzer_Handle;

void BUZZ(Buzzer_Handle *buzz, uint32_t ms){
    HAL_TIM_Base_Start_IT(&htim7);
    buzz->ticks = ms*8;
}


// #define BUZZ_ON  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET)
// #define BUZZ_OFF HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET)