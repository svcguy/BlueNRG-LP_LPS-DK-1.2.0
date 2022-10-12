/**
  ******************************************************************************
  * File Name          : TIM/TIM_OnePulse/Src/rf_device_hal_msp.c
  * @author            : RF Application Team
  * Description        : This file provides code for the MSP Initialization 
  * @brief   HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
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
#include "TIM_OnePulse_main.h"

/* Private typedef -----------------------------------------------------------*/
  /* @{
  */

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */
 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */


/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
* @retval None
*/
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  /* Enable the TIMx clock */
  EnableClock_TIMx();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  
  GPIO_InitStruct.Alternate = TIMx_CH1_AF;
  GPIO_InitStruct.Pin = TIMx_CH1_PIN;
  HAL_GPIO_Init(TIMx_CH1_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Alternate = TIMx_CH2_AF;
  GPIO_InitStruct.Pin = TIMx_CH2_PIN;
  HAL_GPIO_Init(TIMx_CH2_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Pin = EXTERNAL_TRIGGER_SIGNAL_PIN;
  HAL_GPIO_Init(EXTERNAL_TRIGGER_SIGNAL_PORT, &GPIO_InitStruct);

}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



