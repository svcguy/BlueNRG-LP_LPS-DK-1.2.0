/** 
  ******************************************************************************
  * @file    TIM/TIM_OCToggle/Src/rf_device_it.c
  * @author  RF Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
#include "TIM_OCToggle_main.h"
#include "rf_device_it.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
 
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;


/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/

/**
  * @brief This function handles System tick timer.
  */
void SysTick_IRQHandler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* Peripheral Interrupt Handlers                                              */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/

/**
  * @brief This function handles TIM1 global interrupt.
  */
void TIMx_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



