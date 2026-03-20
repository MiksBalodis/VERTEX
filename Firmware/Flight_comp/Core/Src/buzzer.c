#include "buzzer.h"

void BUZZ(Buzzer_Handle *buzz, uint32_t ms){
    HAL_TIM_Base_Start_IT(&htim7);
    buzz->ticks = ms*8;
}