/**
  ******************************************************************************
  * @file    TIM/TIM_OnePulse/Src/rf_device_it.c
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
#include "TIM_OnePulse_main.h"
#include "rf_device_it.h"

/** @addtogroup RF_DRIVER_HAL_Examples
  * @{
  */

/** @addtogroup TIM_OnePulse
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
    
/* Private define ------------------------------------------------------------*/
    
/* Private macro -------------------------------------------------------------*/
    
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef TimHandle;
extern EXTI_HandleTypeDef HEXTI_InitStructure;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles Hard Fault exception.
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
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_IRQHandler(void)
{
}

#define DEBOUNCE_CNT  350
volatile uint32_t debounce_count = 0;

/**
* @brief This function handles System tick timer.
*/
void SysTick_IRQHandler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  debounce_count++;
}


/******************************************************************************/
/*  Peripheral Interrupt Handlers                                             */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file.                                                                     */
/******************************************************************************/

/**
* @brief  This function handles external line interrupt request
* @param  None
* @retval None
*/
void USER_BUTTON_IRQHANDLER(void)
{
  static uint32_t debounce_last = 0;
  
  if(HAL_EXTI_GetPending( &HEXTI_InitStructure ))
  {
    if ( (debounce_count - debounce_last) >= DEBOUNCE_CNT )
    {
      /* Add the SW no bounce */
      debounce_last = debounce_count;
      /* Handle user button press in dedicated function */
      HAL_EXTI_IRQHandler( &HEXTI_InitStructure );
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



