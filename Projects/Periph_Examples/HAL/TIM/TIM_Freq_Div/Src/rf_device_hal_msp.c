/**
  ******************************************************************************
  * File Name          : TIM/TIM_Freq_Div/Src/rf_device_hal_msp.c
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
#include "TIM_Freq_Div_main.h"

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
* @brief TIM_OC MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_oc: TIM_OC handle pointer
* @retval None
*/
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* htim_oc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  if(htim_oc->Instance==TIMx)
  {
    /* Peripheral clock enable */
    EnableClock_TIMx();
    
    GPIO_InitStruct.Pin = TIMx_ETR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = TIMx_ETR_AF;
    HAL_GPIO_Init(TIMx_ETR_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = TIMx_CHx_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = TIMx_CHx_AF;
    HAL_GPIO_Init(TIMx_CHx_PORT, &GPIO_InitStruct);
  }
}

/**
* @brief TIM MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim: TIM handle pointer
* @retval None
*/
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef* htim_oc)
{
  if(htim_oc->Instance==TIMx)
  {
    /* Peripheral clock disable */
    DisableClock_TIMx();
  }
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



