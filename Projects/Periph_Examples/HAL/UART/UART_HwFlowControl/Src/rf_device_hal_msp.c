/**
  ******************************************************************************
  * @file    UART/UART_HwFlowControl/Src/rf_device_hal_msp.c
  * @author  RF Application Team
  * @brief   HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "UART_HwFlowControl_main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
 
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* External functions --------------------------------------------------------*/

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USARTx_INSTANCE)
  {
    /* Peripheral clock enable */
    EnableClock_USART();
    
    /* Reset peripherals */
    __HAL_RCC_USART_FORCE_RESET();
    __HAL_RCC_USART_RELEASE_RESET();
    
    EnableClock_USART_TX_PORT();
    EnableClock_USART_RX_PORT();
    EnableClock_USART_CTS_PORT();
    EnableClock_USART_RTS_PORT();
  
    GPIO_InitStruct.Pin = USARTx_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = USARTx_TX_AF;
    HAL_GPIO_Init(USARTx_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USARTx_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = USARTx_RX_AF;
    HAL_GPIO_Init(USARTx_RX_PORT, &GPIO_InitStruct);
 
    GPIO_InitStruct.Pin = USARTx_CTS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = USARTx_CTS_AF;
    HAL_GPIO_Init(USARTx_CTS_PORT, &GPIO_InitStruct);
        
    GPIO_InitStruct.Pin = USARTx_RTS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = USARTx_RTS_AF;
    HAL_GPIO_Init(USARTx_RTS_PORT, &GPIO_InitStruct);
    
    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USARTx_IRQn, IRQ_HIGH_PRIORITY);
    HAL_NVIC_EnableIRQ(USARTx_IRQn);
  }
}

/**
* @brief USART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param husart: USART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {
    /* Reset peripherals */
    __HAL_RCC_USART_FORCE_RESET();
    __HAL_RCC_USART_RELEASE_RESET();

    /* Peripheral clock disable */
    __HAL_RCC_USART_CLK_DISABLE();
  
    HAL_GPIO_DeInit(USARTx_TX_PORT, USARTx_TX_PIN);
    HAL_GPIO_DeInit(USARTx_RX_PORT, USARTx_RX_PIN);
    HAL_GPIO_DeInit(USARTx_CTS_PORT, USARTx_CTS_PIN);
    HAL_GPIO_DeInit(USARTx_RTS_PORT, USARTx_RTS_PIN);

    /* USARTx_INSTANCE interrupt DeInit */
    HAL_NVIC_DisableIRQ(USARTx_IRQn);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



