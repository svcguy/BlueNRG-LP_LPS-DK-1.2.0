/**
  ******************************************************************************
  * @file    LL/SPI/SPI_DMA_Slave_Init/Src/rf_device_it.c
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
#include "SPI_DMA_Slave_Init_main.h"
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
* @param  None
* @retval None
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
  * @brief  This function handles DMA1 interrupt request.
  * @param  None
  * @retval None
  */
void DMA_IRQHandler(void)
{

  if (LL_DMA_IsActiveFlag_TC3(DMA1))
  {
    LL_DMA_ClearFlag_GI3(DMA1);
    /* Call function Reception complete Callback */
    DMA_ReceiveComplete_Callback();
  }
  else if (LL_DMA_IsActiveFlag_TE3(DMA1))
  {
    /* Call Error function */
    SPI_SLAVE_TransferError_Callback();
  }

  if (LL_DMA_IsActiveFlag_TC1(DMA1))
  {
    LL_DMA_ClearFlag_GI1(DMA1);
    /* Call function Transmission complete Callback */
    DMA_TransmitComplete_Callback();
  }
  else if (LL_DMA_IsActiveFlag_TE1(DMA1))
  {
    /* Call Error function */
    SPI_SLAVE_TransferError_Callback();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



