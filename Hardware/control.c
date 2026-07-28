#include "control.h"
/**
  * @brief  设置两个电机速度
  * @param  输入pwm
  * @retval 无
  */
void Set_Motor(float speedl, float speedr)
{
	if (speedl > 0)
    {
        // Set the motor to move forward
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, speedl);
        HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        // Set the motor to move backward
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, -speedl);
        HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET);
    }
    
    if (speedr > 0)
    {
        // Set the motor to move forward
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, speedr);
        HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
    }
    else
    {
        // Set the motor to move backward
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, -speedr);
        HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);

    }
}
