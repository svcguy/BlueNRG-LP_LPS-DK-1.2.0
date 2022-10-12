/**
  ******************************************************************************
  * @file    GPIO/RTC_Autocalibration/Src/rf_device_hal_msp.c
  * @author  RF Application Team
  * @brief   This file provides code for the MSP Initialization 
  *          and de-Initialization codes.
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
#include "RTC_Autocalibration_main.h"

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
* @brief TIM_IC MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_ic: TIM_IC handle pointer
* @retval None
*/
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_ic->Instance==TIMx)
  {
    /* Peripheral clock enable */
    EnableClock_TIMx();

    GPIO_InitStruct.Pin = TIMx_CH1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = TIMx_CH1_AF;
    HAL_GPIO_Init(TIMx_CH1_PORT, &GPIO_InitStruct);

    /* TIMx interrupt Init */
    HAL_NVIC_SetPriority(TIMx_IRQn, IRQ_HIGH_PRIORITY);
    HAL_NVIC_EnableIRQ(TIMx_IRQn);
  }
}

/**
* @brief TIM MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim: TIM handle pointer
* @retval None
*/
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* htim_ic)
{
  if(htim_ic->Instance==TIMx)
  {
    /* Peripheral clock disable */
    DisableClock_TIMx();
    
    HAL_GPIO_DeInit(TIMx_CH1_PORT, TIMx_CH1_PIN);

    /* TIMx interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIMx_IRQn);
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


