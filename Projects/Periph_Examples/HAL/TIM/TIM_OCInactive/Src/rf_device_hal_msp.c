/**
  ******************************************************************************
  * File Name          : TIM/TIM_OCInactive/Src/rf_device_hal_msp.c
  * @author            : RF Application Team
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
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
#include "TIM_OCInactive_main.h"

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
* @brief TIM_OC MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_oc: TIM_OC handle pointer
* @retval None
*/
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* htim_oc)
{
  if(htim_oc->Instance==TIMx)
  {
    /* Peripheral clock enable */
    EnableClock_TIMx();
    /* TIMx interrupt Init */
    HAL_NVIC_SetPriority(TIMx_IRQn, IRQ_HIGH_PRIORITY);
    HAL_NVIC_EnableIRQ(TIMx_IRQn);
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

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

    /* TIMx interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIMx_IRQn);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


