/**
  ******************************************************************************
  * @file    SPI/SPI_HD_P_IT_S/Src/rf_device_hal_msp.c
  * @author  RF Application Team
  * @brief   HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "SPI_HD_P_IT_S_main.h"

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
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI_SLAVE)
  {
    /* Peripheral clock enable */
    __HAL_RCC_SPI_SLAVE_CLK_ENABLE();
    
   /* SCK */
    GPIO_InitStruct.Pin = GPIO_PIN_SPI_SLAVE_SCK;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF_SPI_SLAVE_SCK;
    HAL_GPIO_Init(GPIO_SLAVE_SCK, &GPIO_InitStruct);

    /* MOSI */
    GPIO_InitStruct.Pin = GPIO_PIN_SPI_SLAVE_MOSI;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF_SPI_SLAVE_MOSI;
    HAL_GPIO_Init(GPIO_SLAVE_MOSI, &GPIO_InitStruct);

    /* MISO */
    GPIO_InitStruct.Pin = GPIO_PIN_SPI_SLAVE_MISO;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF_SPI_SLAVE_MISO;
    HAL_GPIO_Init(GPIO_SLAVE_MISO, &GPIO_InitStruct);

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI_SLAVE_IRQn, IRQ_HIGH_PRIORITY);
    HAL_NVIC_EnableIRQ(SPI_SLAVE_IRQn);
  }
}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI_SLAVE)
  {
    /* Reset peripherals */
    __HAL_RCC_SPI_SLAVE_FORCE_RESET();
    __HAL_RCC_SPI_SLAVE_RELEASE_RESET();
    
    /* Peripheral clock disable */
    __HAL_RCC_SPI_SLAVE_CLK_DISABLE();
    
    HAL_GPIO_DeInit(GPIO_SLAVE_SCK, GPIO_PIN_SPI_SLAVE_SCK);
    HAL_GPIO_DeInit(GPIO_SLAVE_MOSI, GPIO_PIN_SPI_SLAVE_MOSI);
    HAL_GPIO_DeInit(GPIO_SLAVE_MISO, GPIO_PIN_SPI_SLAVE_MISO);
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



