#include "L298N.h"

void L298N_GPIO_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	L298N_SetDirection(AllWheel, Wheel_Stop);
	/*Configure GPIO pins */
	GPIO_InitStruct.Pin = L298N_Front_Left_IN1_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(L298N_Front_Left_IN1_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = L298N_Front_Left_IN2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(L298N_Front_Left_IN2_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = L298N_Front_Right_IN3_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(L298N_Front_Right_IN3_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = L298N_Front_Right_IN4_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(L298N_Front_Right_IN4_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = L298N_Rear_Right_IN1_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(L298N_Rear_Right_IN1_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = L298N_Rear_Right_IN2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(L298N_Rear_Right_IN2_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = L298N_Rear_Left_IN3_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(L298N_Rear_Left_IN3_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = L298N_Rear_Left_IN4_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(L298N_Rear_Left_IN4_Port, &GPIO_InitStruct);
}


void L298N_SetDirection(uint8_t wheel, uint8_t dir)
{
	if (wheel & FrontWheel_Left)
	{
		switch (dir)
		{
		case Wheel_Forward:
			{
				L298N_Front_Left_IN1_HIGH;
				L298N_Front_Left_IN2_LOW;
			}
			break;
		case Wheel_Reverse:
			{
				L298N_Front_Left_IN1_LOW;
				L298N_Front_Left_IN2_HIGH;
			}
			break;
		case Wheel_Stop:
			{
				L298N_Front_Left_IN1_LOW;
				L298N_Front_Left_IN2_LOW;
			}
			break;
		default:
			break;
		}
	}
	if (wheel & FrontWheel_Right)
	{
		switch (dir)
		{
		case Wheel_Forward:
			{
				L298N_Front_Right_IN3_HIGH;
				L298N_Front_Right_IN4_LOW;
			}
			break;
		case Wheel_Reverse:
			{
				L298N_Front_Right_IN3_LOW;
				L298N_Front_Right_IN4_HIGH;
			}
			break;
		case Wheel_Stop:
			{
				L298N_Front_Right_IN3_LOW;
				L298N_Front_Right_IN4_LOW;
			}
			break;
		default:
			break;
		}
	}
	if (wheel & RearWheel_Left)
	{
		switch (dir)
		{
		case Wheel_Forward:
			{
				L298N_Rear_Left_IN3_HIGH;
				L298N_Rear_Left_IN4_LOW;
			}
			break;
		case Wheel_Reverse:
			{
				L298N_Rear_Left_IN3_LOW;
				L298N_Rear_Left_IN4_HIGH;
			}
			break;
		case Wheel_Stop:
			{
				L298N_Rear_Left_IN3_LOW;
				L298N_Rear_Left_IN4_LOW;
			}
			break;
		default:
			break;
		}
	}
	if (wheel & RearWheel_Right)
	{
		switch (dir)
		{
		case Wheel_Forward:
			{
				L298N_Rear_Right_IN1_HIGH;
				L298N_Rear_Right_IN2_LOW;
			}
			break;
		case Wheel_Reverse:
			{
				L298N_Rear_Right_IN1_LOW;
				L298N_Rear_Right_IN2_HIGH;
			}
			break;
		case Wheel_Stop:
			{
				L298N_Rear_Right_IN1_LOW;
				L298N_Rear_Right_IN2_LOW;
			}
			break;
		default:
			break;
		}
		
	}
}
