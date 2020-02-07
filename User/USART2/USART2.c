/**
  ******************************************************************************
  * @file    usart2.h
  * @brief   This file includes the initialization functions and Interrupt 
  *	         handlers of usart2 and Retargets the C library printf function to 
  *	         the USART3. this file is compatible with NUCLEO board of stm32f767. 
  *	         To make this file compatible with ALIENTEK Apollo, usart3 should be
  *	         changed into usart1.
  * @author  UESTC Yu XuYao
  ******************************************************************************
  * @attention
  * The sending data of the host computer program is fixed at \r\n
  ******************************************************************************
  */
#include "USART2.h"
#include "USART3.h"

/* UART handler declaration */
UART_HandleTypeDef UART2_Handler;

#if EN_USART2_RX

/*Rx buffer 
 *End with 0x0D ('\n')
 **/
uint8_t USART2_RX_BUF[USART2_REC_LEN];

/**Receive status
  *[15]:  Receive complete(1)
  *[14]:  Receive 0x0d(1)
  *[13:0]:Number of valid bytes received
  */
volatile uint16_t USART2_RX_STA = 0;
/*receive buffer used by HAL library*/
uint8_t USART2_aRxBuffer[USART2_RXBUFFERSIZE];
#endif

/**
  * @brief  Initialization function of USART3.
  * @param  bound: BaudRate of USART3
  * @retval None
  */
void USART2_Init(uint32_t bound)
{
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART configured as follows:
	    - Word Length = 8 Bits : 
						BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
		- Stop Bit    = One Stop bit
		- Parity      = none
		- BaudRate    = 115200 baud
		- Hardware flow control disabled (RTS and CTS signals) */
	UART2_Handler.Instance        = USART2;
	UART2_Handler.Init.BaudRate   = bound;
	UART2_Handler.Init.WordLength = UART_WORDLENGTH_8B;
	UART2_Handler.Init.StopBits   = UART_STOPBITS_1;
	UART2_Handler.Init.Parity     = UART_PARITY_NONE;
	UART2_Handler.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UART2_Handler.Init.Mode       = UART_MODE_TX_RX;
	UART2_Handler.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&UART2_Handler);
	
	HAL_UART_Receive_IT(&UART2_Handler, (uint8_t *)USART2_aRxBuffer, USART2_RXBUFFERSIZE);
}

/**
  * @brief  UART MSP Initialization.
  * @param  huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{

	GPIO_InitTypeDef  GPIO_InitStruct;
	RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

	if (huart->Instance == USART2)
	{
		/* Enable GPIO TX/RX clock */
		USART2_TX_GPIO_CLK_ENABLE();
		USART2_RX_GPIO_CLK_ENABLE();

		/* Select SysClk as source of USART2 clocks */
//		RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
//		RCC_PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
//		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

		/* Enable USART2 clock */
		USART2_CLK_ENABLE();

		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = USART2_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = USART2_TX_AF;

		HAL_GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = USART2_RX_PIN;
		GPIO_InitStruct.Alternate = USART2_RX_AF;

		HAL_GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStruct);
		#if EN_USART2_RX
			/*Enable USART3_IRQn*/
			HAL_NVIC_EnableIRQ(USART2_IRQn);
			/*Preemption priority 3 sub-priority 3*/
			HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
		#endif
	}
	if (huart->Instance == USART3)
	{
		/* Enable GPIO TX/RX clock */
		USART3_TX_GPIO_CLK_ENABLE();
		USART3_RX_GPIO_CLK_ENABLE();

		/* Select SysClk as source of USART3 clocks */
		RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
		RCC_PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

		/* Enable USART3 clock */
		USART3_CLK_ENABLE();

		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = USART3_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = USART3_TX_AF;

		HAL_GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = USART3_RX_PIN;
		GPIO_InitStruct.Alternate = USART3_RX_AF;

		HAL_GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStruct);
		#if EN_USART3_RX
			/*Enable USART3_IRQn*/
			HAL_NVIC_EnableIRQ(USART3_IRQn);
			/*Preemption priority 3 sub-priority 3*/
			HAL_NVIC_SetPriority(USART3_IRQn, 3, 3);
		#endif
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		/*Receive not complete*/
		if ((USART2_RX_STA & 0x8000) == 0)
		{
			/*Receive 0x0D(\r)*/
			if (USART2_RX_STA & 0x4000)
			{
				/*Receive error(\n), start over*/
				if (USART2_aRxBuffer[0] != 0x0a)USART2_RX_STA = 0; //0a
				/*Receive complete*/
				else USART2_RX_STA |= 0x8000;
			}
			else /*Not Receive 0x0D*/
			{
				if (USART2_aRxBuffer[0] == 0x0d)USART2_RX_STA |= 0x4000; //0d
				else
				{
					USART2_RX_BUF[USART2_RX_STA & 0X3FFF] = USART2_aRxBuffer[0];
					USART2_RX_STA++;
					/*Receive data is too long, start receiving again*/
					if (USART2_RX_STA > (USART2_REC_LEN - 1))USART2_RX_STA = 0;  
				}
			}
		}

	}
	if (huart->Instance == USART3)
	{
		/*Receive not complete*/
		if ((USART3_RX_STA & 0x8000) == 0)
		{
			/*Receive 0x0D(\r)*/
			if (USART3_RX_STA & 0x4000)
			{
				/*Receive error(\n), start over*/
				if (USART3_aRxBuffer[0] != 0x0a)USART3_RX_STA = 0; //0a
				/*Receive complete*/
				else USART3_RX_STA |= 0x8000;
			}
			else /*Not Receive 0x0D*/
			{
				if (USART3_aRxBuffer[0] == 0x0d)USART3_RX_STA |= 0x4000; //0d
				else
				{
					USART3_RX_BUF[USART3_RX_STA & 0X3FFF] = USART3_aRxBuffer[0];
					USART3_RX_STA++;
					/*Receive data is too long, start receiving again*/
					if (USART3_RX_STA > (USART3_REC_LEN - 1))USART3_RX_STA = 0;  
				}
			}
		}

	}
}

void USART2_IRQHandler(void)                	
{
	uint32_t  timeout = 0;
	uint32_t maxDelay = 0x1FFFF;
	/*Call HAL library interrupt processing public function*/
	HAL_UART_IRQHandler(&UART2_Handler);
	
	timeout = 0;
	/*Waiting for ready*/
	while (HAL_UART_GetState(&UART2_Handler) != HAL_UART_STATE_READY)
	{
		/*Timeout processing*/
		timeout++;
		if(timeout > maxDelay) break;		
	}
     
	timeout = 0;
	/*After one process is complete, re-enable the interrupt and set RxXferCount to 1*/
	while (HAL_UART_Receive_IT(&UART2_Handler, (uint8_t *)USART2_aRxBuffer, USART2_RXBUFFERSIZE) != HAL_OK)
	{
		/*Timeout processing*/
		timeout++;
		if(timeout > maxDelay) break;	
	}

} 

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		/*##-1- Reset peripherals ##################################################*/
		USART2_FORCE_RESET();
		USART2_RELEASE_RESET();

		/*##-2- Disable peripherals and GPIO Clocks #################################*/
		/* Configure UART Tx as alternate function  */
		HAL_GPIO_DeInit(USART2_TX_GPIO_PORT, USART2_TX_PIN);
		/* Configure UART Rx as alternate function  */
		HAL_GPIO_DeInit(USART2_RX_GPIO_PORT, USART2_RX_PIN);
	}
	if(huart->Instance == USART2)
	{
		/*##-1- Reset peripherals ##################################################*/
		USART3_FORCE_RESET();
		USART3_RELEASE_RESET();

		/*##-2- Disable peripherals and GPIO Clocks #################################*/
		/* Configure UART Tx as alternate function  */
		HAL_GPIO_DeInit(USART3_TX_GPIO_PORT, USART3_TX_PIN);
		/* Configure UART Rx as alternate function  */
		HAL_GPIO_DeInit(USART3_RX_GPIO_PORT, USART3_RX_PIN);
	}
	
}
