#ifndef _L298N_H_
#define _L298N_H_

#include "stm32f7xx_hal.h"

#define FrontWheel_Left  0x01
#define FrontWheel_Right 0x02
#define RearWheel_Left   0x04
#define RearWheel_Right  0x08
#define AllWheel         0x0F

#define Wheel_Forward 0x01
#define Wheel_Reverse 0x02
#define Wheel_Stop    0x04

#define L298N_Front_Left_IN1_PIN  GPIO_PIN_9
#define L298N_Front_Left_IN1_Port  GPIOG
#define L298N_Front_Left_IN1_HIGH  HAL_GPIO_WritePin(L298N_Front_Left_IN1_Port,L298N_Front_Left_IN1_PIN,GPIO_PIN_SET)
#define L298N_Front_Left_IN1_LOW   HAL_GPIO_WritePin(L298N_Front_Left_IN1_Port,L298N_Front_Left_IN1_PIN,GPIO_PIN_RESET)

#define L298N_Front_Left_IN2_PIN  GPIO_PIN_12
#define L298N_Front_Left_IN2_Port  GPIOG
#define L298N_Front_Left_IN2_HIGH  HAL_GPIO_WritePin(L298N_Front_Left_IN2_Port,L298N_Front_Left_IN2_PIN,GPIO_PIN_SET)
#define L298N_Front_Left_IN2_LOW   HAL_GPIO_WritePin(L298N_Front_Left_IN2_Port,L298N_Front_Left_IN2_PIN,GPIO_PIN_RESET)

#define L298N_Front_Right_IN3_PIN GPIO_PIN_0
#define L298N_Front_Right_IN3_Port GPIOD
#define L298N_Front_Right_IN3_HIGH  HAL_GPIO_WritePin(L298N_Front_Right_IN3_Port,L298N_Front_Right_IN3_PIN,GPIO_PIN_SET)
#define L298N_Front_Right_IN3_LOW   HAL_GPIO_WritePin(L298N_Front_Right_IN3_Port,L298N_Front_Right_IN3_PIN,GPIO_PIN_RESET)

#define L298N_Front_Right_IN4_PIN GPIO_PIN_1
#define L298N_Front_Right_IN4_Port GPIOD
#define L298N_Front_Right_IN4_HIGH  HAL_GPIO_WritePin(L298N_Front_Right_IN4_Port,L298N_Front_Right_IN4_PIN,GPIO_PIN_SET)
#define L298N_Front_Right_IN4_LOW   HAL_GPIO_WritePin(L298N_Front_Right_IN4_Port,L298N_Front_Right_IN4_PIN,GPIO_PIN_RESET)

#define L298N_Rear_Right_IN1_PIN   GPIO_PIN_15
#define L298N_Rear_Right_IN1_Port   GPIOG
#define L298N_Rear_Right_IN1_HIGH  HAL_GPIO_WritePin(L298N_Rear_Right_IN1_Port,L298N_Rear_Right_IN1_PIN,GPIO_PIN_SET)
#define L298N_Rear_Right_IN1_LOW   HAL_GPIO_WritePin(L298N_Rear_Right_IN1_Port,L298N_Rear_Right_IN1_PIN,GPIO_PIN_RESET)

#define L298N_Rear_Right_IN2_PIN   GPIO_PIN_10
#define L298N_Rear_Right_IN2_Port   GPIOG
#define L298N_Rear_Right_IN2_HIGH  HAL_GPIO_WritePin(L298N_Rear_Right_IN2_Port,L298N_Rear_Right_IN2_PIN,GPIO_PIN_SET)
#define L298N_Rear_Right_IN2_LOW   HAL_GPIO_WritePin(L298N_Rear_Right_IN2_Port,L298N_Rear_Right_IN2_PIN,GPIO_PIN_RESET)

#define L298N_Rear_Left_IN3_PIN  GPIO_PIN_13
#define L298N_Rear_Left_IN3_Port  GPIOG
#define L298N_Rear_Left_IN3_HIGH  HAL_GPIO_WritePin(L298N_Rear_Left_IN3_Port,L298N_Rear_Left_IN3_PIN,GPIO_PIN_SET)
#define L298N_Rear_Left_IN3_LOW   HAL_GPIO_WritePin(L298N_Rear_Left_IN3_Port,L298N_Rear_Left_IN3_PIN,GPIO_PIN_RESET)

#define L298N_Rear_Left_IN4_PIN  GPIO_PIN_11
#define L298N_Rear_Left_IN4_Port  GPIOG
#define L298N_Rear_Left_IN4_HIGH  HAL_GPIO_WritePin(L298N_Rear_Left_IN4_Port,L298N_Rear_Left_IN4_PIN,GPIO_PIN_SET)
#define L298N_Rear_Left_IN4_LOW   HAL_GPIO_WritePin(L298N_Rear_Left_IN4_Port,L298N_Rear_Left_IN4_PIN,GPIO_PIN_RESET)

//#define L298N_Front_Left_IN1
//#define L298N_Front_Left_IN2
//#define L298N_Front_Right_IN3 
//#define L298N_Front_Right_IN4
//#define L298N_Rear_Right_IN1
//#define L298N_Rear_Right_IN2
//#define L298N_Rear_Left_IN3 
//#define L298N_Rear_Left_IN4


void L298N_GPIO_INIT(void);
void L298N_SetDirection(uint8_t wheel, uint8_t dir);

#endif // !_L298N_H_
