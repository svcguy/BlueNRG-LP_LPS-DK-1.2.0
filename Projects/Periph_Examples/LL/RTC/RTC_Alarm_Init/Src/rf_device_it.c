
/**
  ******************************************************************************
  * @file    LL/RTC/RTC_Alarm_Init/Src/rf_device_it.c
  * @author  RF Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
#include "RTC_Alarm_Init_main.h"
#include "rf_device_it.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers             */ 
/******************************************************************************/

/**
  * @brief This function handles Hard fault interrupt.
  * @param  None
  * @retval None
  */
void HardFault_IRQHandler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}


/**
  * @brief This function handles System tick timer.
  */
void SysTick_IRQHandler(void)
{
}

/******************************************************************************/
/*  Peripheral Interrupt Handlers                                             */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file.                                                                     */
/******************************************************************************/

/**
  * @brief  This function handles RTC Alarm interrupt request.
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
  /* Get the Alarm interrupt source enable status */
  if (LL_RTC_IsEnabledIT_ALRA(RTC) != 0)
  {
    /* Get the pending status of the Alarm Interrupt */
    if (LL_RTC_IsActiveFlag_ALRA(RTC) != 0)
    {
      /* Alarm callback */
      Alarm_Callback();

      /* Clear the Alarm interrupt pending bit */
      LL_RTC_ClearFlag_ALRA(RTC);
    }
  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



