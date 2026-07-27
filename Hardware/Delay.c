#include "Delay.h"

uint8_t delayflag = 0;

void Delay_us(uint32_t xus)
{
    __HAL_TIM_SetCounter(&htim3, xus);
    HAL_TIM_Base_Start_IT(&htim3);
    while(!delayflag);
    delayflag = 0;
}
