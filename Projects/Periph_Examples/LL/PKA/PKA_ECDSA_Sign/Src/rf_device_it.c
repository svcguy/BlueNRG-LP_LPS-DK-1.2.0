/**
  ******************************************************************************
  * @file    LL/PKA/PKA_ECDSA_Sign/Src/rf_device_it.c
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
  *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "PKA_ECDSA_Sign_main.h"
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
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_IRQHandler(void)
{
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
/* Peripheral Interrupt Handlers                                              */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/

/**
  * Brief   This function handles PKA Instance interrupt request.
  * Param   None
  * Retval  None
  */
void PKA_IRQHandler(void)
{
  /* Manage the PKA RAM error flag */
  if(LL_PKA_IsActiveFlag_RAMERR(PKA) == 1)
  {
    LL_PKA_ClearFlag_RAMERR(PKA);
    PKA_ERROR_callback();
  }
  
  /* Manage the Address error flag */
  if(LL_PKA_IsActiveFlag_ADDRERR(PKA) == 1)
  {
    LL_PKA_ClearFlag_ADDRERR(PKA);
    PKA_ERROR_callback();
  }
  
  /* Manage the PKA End of Operation flag */
  if(LL_PKA_IsActiveFlag_PROCEND(PKA) == 1)
  {
    LL_PKA_ClearFlag_PROCEND(PKA);
    PKA_PROCEND_callback();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
