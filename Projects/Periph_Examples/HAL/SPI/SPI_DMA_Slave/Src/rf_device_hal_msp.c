/**
  ******************************************************************************
  * @file    SPI/SPI_DMA_Master_Slave/Src/rf_device_hal_msp.c
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
#include "SPI_DMA_Slave_main.h"


extern DMA_HandleTypeDef hdma_spi_slave_tx;

extern DMA_HandleTypeDef hdma_spi_slave_rx;

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
  GPIO_InitTypeDef GPIO_InitStruct;
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

    /* SPI_SLAVE DMA Init */
    /* SPI_SLAVE_TX Init */
    hdma_spi_slave_tx.Instance = DMA1_Channel3;
    hdma_spi_slave_tx.Init.Request = DMA_REQUEST_SPI_SLAVE_TX;
    hdma_spi_slave_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi_slave_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_slave_tx.Init.MemInc = DMA_MINC_ENABLE;
#if defined( CONFIG_DATASIZE_16BIT ) 
    hdma_spi_slave_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi_slave_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
#elif defined( CONFIG_DATASIZE_8BIT )
    hdma_spi_slave_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi_slave_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
#endif 
    hdma_spi_slave_tx.Init.Mode = DMA_NORMAL;
    hdma_spi_slave_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_spi_slave_tx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(hspi,hdmatx,hdma_spi_slave_tx);

    /* SPI_SLAVE_RX Init */
    hdma_spi_slave_rx.Instance = DMA1_Channel2;
    hdma_spi_slave_rx.Init.Request = DMA_REQUEST_SPI_SLAVE_RX;
    hdma_spi_slave_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi_slave_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_slave_rx.Init.MemInc = DMA_MINC_ENABLE;
#if defined( CONFIG_DATASIZE_16BIT ) 
    hdma_spi_slave_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi_slave_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
#elif defined( CONFIG_DATASIZE_8BIT )
    hdma_spi_slave_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi_slave_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
#endif 
    hdma_spi_slave_rx.Init.Mode = DMA_NORMAL;
    hdma_spi_slave_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_spi_slave_rx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(hspi,hdmarx,hdma_spi_slave_rx);
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
    HAL_GPIO_DeInit(GPIO_SLAVE_MISO, GPIO_PIN_SPI_SLAVE_MISO);
    HAL_GPIO_DeInit(GPIO_SLAVE_MOSI, GPIO_PIN_SPI_SLAVE_MOSI);

    /* SPI_SLAVE DMA DeInit */
    HAL_DMA_DeInit(hspi->hdmatx);
    HAL_DMA_DeInit(hspi->hdmarx);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


