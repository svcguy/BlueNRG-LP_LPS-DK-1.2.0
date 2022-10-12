/**
******************************************************************************
* @file    I2S/I2S_CCA01M1_ComIT/Src/rf_device_hal_msp.c
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
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "cca01m1_audio.h"

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
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s)
{
  GPIO_InitTypeDef  GPIO_InitStruct;  
  
  static DMA_HandleTypeDef hdma_i2sTx_1;   
  
  AUDIO_OUT1_I2S_CLK_ENABLE();
  AUDIO_OUT1_I2S_SD_CLK_ENABLE(); 
  AUDIO_OUT1_I2S_SCK_CLK_ENABLE();
  AUDIO_OUT1_I2S_MCK_CLK_ENABLE();
  AUDIO_OUT1_I2S_WS_CLK_ENABLE();
  AUDIO_OUT1_RST_GPIO_CLK_ENABLE();
  
  /* CODEC_I2S pins configuration: FS, SCK, MCK and SD pins ------------------*/
  GPIO_InitStruct.Pin = AUDIO_OUT1_I2S_SCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = AUDIO_OUT1_I2S_SCK_AF;
  HAL_GPIO_Init(AUDIO_OUT1_I2S_SCK_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Alternate   = AUDIO_OUT1_I2S_SD_AF;
  GPIO_InitStruct.Pin         = AUDIO_OUT1_I2S_SD_PIN ;
  HAL_GPIO_Init(AUDIO_OUT1_I2S_SD_GPIO_PORT, &GPIO_InitStruct);	
  
  GPIO_InitStruct.Alternate   = AUDIO_OUT1_I2S_WS_AF;
  GPIO_InitStruct.Pin         = AUDIO_OUT1_I2S_WS_PIN;
  HAL_GPIO_Init(AUDIO_OUT1_I2S_WS_GPIO_PORT, &GPIO_InitStruct); 
  
  GPIO_InitStruct.Alternate   = AUDIO_OUT1_I2S_MCK_AF;
  GPIO_InitStruct.Pin         = AUDIO_OUT1_I2S_MCK_PIN; 
  HAL_GPIO_Init(AUDIO_OUT1_I2S_MCK_GPIO_PORT, &GPIO_InitStruct);   
  
  /* Configure GPIO PINs to detect Interrupts */ 
  GPIO_InitStruct.Pin = AUDIO_OUT1_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(AUDIO_OUT1_RST_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN, GPIO_PIN_RESET);  
  
  /* Enable the I2S DMA clock */
  AUDIO_OUT1_I2S_DMAx_CLK_ENABLE(); 
  
  if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
  {
    /* Configure the hdma_i2sTx handle parameters */   
    
    hdma_i2sTx_1.Init.Request             = DMA_REQUEST_SPI2_TX;   
    hdma_i2sTx_1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_i2sTx_1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2sTx_1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2sTx_1.Init.PeriphDataAlignment = AUDIO_OUT1_I2S_DMAx_PERIPH_DATA_SIZE;
    hdma_i2sTx_1.Init.MemDataAlignment    = AUDIO_OUT1_I2S_DMAx_MEM_DATA_SIZE;
    hdma_i2sTx_1.Init.Mode                = DMA_CIRCULAR;
    hdma_i2sTx_1.Init.Priority            = DMA_PRIORITY_HIGH;     
    hdma_i2sTx_1.Instance                 = AUDIO_OUT1_I2S_DMAx_STREAM;
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_i2sTx_1);
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hi2s, hdmatx, hdma_i2sTx_1);
    
    /* Configure the DMA Stream */
    if (HAL_DMA_Init(&hdma_i2sTx_1) != HAL_OK)
    {
      Error_Handler();
    }
    
    /* DMA interrupt init */
    /* DMA_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(AUDIO_OUT1_I2S_DMAx_IRQ, 0);
    HAL_NVIC_EnableIRQ(AUDIO_OUT1_I2S_DMAx_IRQ);
  }
}

#ifdef USE_I2S_DEINIT

/**
* @brief  Deinitializes I2S MSP.
* @param  hi2s  I2S handle 
* @retval HAL status
*/
void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s)
{     
  AUDIO_OUT1_I2S_CLK_DISABLE();  
  
  /* I2S pins de initialization: MCK, WS, SCK and SD pins -----------------------------*/
  
  HAL_GPIO_DeInit(AUDIO_OUT1_I2S_SCK_GPIO_PORT, AUDIO_OUT1_I2S_SCK_PIN);  
  HAL_GPIO_DeInit(AUDIO_OUT1_I2S_SD_GPIO_PORT, AUDIO_OUT1_I2S_SD_PIN);    
  HAL_GPIO_DeInit(AUDIO_OUT1_I2S_WS_GPIO_PORT, AUDIO_OUT1_I2S_WS_PIN);
  HAL_GPIO_DeInit(AUDIO_OUT1_I2S_MCK_GPIO_PORT, AUDIO_OUT1_I2S_MCK_PIN);   
  HAL_GPIO_DeInit(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN); 
  
  if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
  {     
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(hi2s->hdmatx); 
    
    /* I2S DMA IRQ Channel configuration */
    HAL_NVIC_DisableIRQ(AUDIO_OUT1_I2S_DMAx_IRQ); 
  }    
}

#endif

/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{  
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==AUDIO_OUT1_I2C_INSTANCE)
  {
    AUDIO_OUT1_I2C_SCL_CLK_ENABLE();
    GPIO_InitStruct.Pin = AUDIO_OUT1_I2C_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = AUDIO_OUT1_I2C_SCL_AF;
    HAL_GPIO_Init(AUDIO_OUT1_I2C_SCL_PORT, &GPIO_InitStruct);
    
    AUDIO_OUT1_I2C_SDA_CLK_ENABLE();
    GPIO_InitStruct.Pin = AUDIO_OUT1_I2C_SDA_PIN;
    //GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    //GPIO_InitStruct.Pull = GPIO_PULLUP;
    //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = AUDIO_OUT1_I2C_SDA_AF;
    HAL_GPIO_Init(AUDIO_OUT1_I2C_SDA_PORT, &GPIO_InitStruct);
    
    /* Peripheral clock enable */
    AUDIO_OUT1_I2C_CLK_ENABLE();
    
    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(AUDIO_OUT1_I2C_IRQn, IRQ_HIGH_PRIORITY);
    HAL_NVIC_EnableIRQ(AUDIO_OUT1_I2C_IRQn);
  }
  
}

/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==AUDIO_OUT1_I2C_INSTANCE)
  {
    /* Peripheral clock disable */
    AUDIO_OUT1_I2C_CLK_DISABLE();
    HAL_GPIO_DeInit(AUDIO_OUT1_I2C_SCL_PORT, AUDIO_OUT1_I2C_SCL_PIN);
    HAL_GPIO_DeInit(AUDIO_OUT1_I2C_SDA_PORT, AUDIO_OUT1_I2C_SDA_PIN);
    
    /* I2Cx interrupt DeInit */
    HAL_NVIC_DisableIRQ(AUDIO_OUT1_I2C_IRQn);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


