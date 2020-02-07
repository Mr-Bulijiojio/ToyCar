/**
  ******************************************************************************
  * @file    delay.h
  * @brief   This file includes the declarations of usart3 functions and macro
  *          definition of usart3To make this file compatible with ALIENTEK 
  *          Apollo, usart3 should be changedinto usart1.
  * @author  UESTC Yu XuYao
  ******************************************************************************
  */
#ifndef _USART2_H_
#define _USART2_H_

#include "stm32f767xx.h"
#include "stm32f7xx_hal.h"
#include <stdio.h>

/*Define the maximum number of received bytes*/
#define USART2_REC_LEN  			200
/*Enable or disable usart reception*/
#define EN_USART2_RX 			1

/*RX Buffer*/
extern uint8_t  USART2_RX_BUF[USART2_REC_LEN];
/*Receive status tag*/
extern volatile uint16_t USART2_RX_STA;
/*Handler of USART3*/
extern UART_HandleTypeDef UART2_Handler;

#define USART2_RXBUFFERSIZE   1
/*receive buffer used by HAL library*/
extern uint8_t USART2_aRxBuffer[USART2_RXBUFFERSIZE];

/* Definition for USARTx clock resources */
//#define USARTx                           USART2
#define USART2_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USART2_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART2_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USART2_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USART2_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USART2_TX_PIN                    GPIO_PIN_2
#define USART2_TX_GPIO_PORT              GPIOA
#define USART2_TX_AF                     GPIO_AF7_USART2
#define USART2_RX_PIN                    GPIO_PIN_3
#define USART2_RX_GPIO_PORT              GPIOA
#define USART2_RX_AF                     GPIO_AF7_USART2

/*Initialization function of USART3*/
void USART2_Init(uint32_t bound);
/*UART MSP Initialization.*/
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
#endif
