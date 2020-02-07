/**
  ******************************************************************************
  * @file    delay.h
  * @brief   This file includes the declarations of usart3 functions and macro
  *          definition of usart3To make this file compatible with ALIENTEK 
  *          Apollo, usart3 should be changedinto usart1.
  * @author  UESTC Yu XuYao
  ******************************************************************************
  */
#ifndef _USART3_H_
#define _USART3_H_

#include "stm32f767xx.h"
#include "stm32f7xx_hal.h"
#include <stdio.h>

/*Define the maximum number of received bytes*/
#define USART3_REC_LEN  			200
/*Enable or disable usart reception*/
#define EN_USART3_RX 			1

/*RX Buffer*/
extern uint8_t  USART3_RX_BUF[USART3_REC_LEN];
/*Receive status tag*/
extern volatile uint16_t USART3_RX_STA;
/*Handler of USART3*/
extern UART_HandleTypeDef UART3_Handler;

#define USART3_RXBUFFERSIZE   1
/*receive buffer used by HAL library*/
extern uint8_t USART3_aRxBuffer[USART3_RXBUFFERSIZE];

/* Definition for USART3 clock resources */
//#define USARTx                           USART3
#define USART3_CLK_ENABLE()              __HAL_RCC_USART3_CLK_ENABLE();
#define USART3_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()
#define USART3_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()

#define USART3_FORCE_RESET()             __HAL_RCC_USART3_FORCE_RESET()
#define USART3_RELEASE_RESET()           __HAL_RCC_USART3_RELEASE_RESET()

/* Definition for USART3 Pins */
#define USART3_TX_PIN                    GPIO_PIN_8
#define USART3_TX_GPIO_PORT              GPIOD
#define USART3_TX_AF                     GPIO_AF7_USART3
#define USART3_RX_PIN                    GPIO_PIN_9
#define USART3_RX_GPIO_PORT              GPIOD
#define USART3_RX_AF                     GPIO_AF7_USART3

/*Initialization function of USART3*/
void USART3_Init(uint32_t bound);
/*UART MSP Initialization.*/
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
#endif
