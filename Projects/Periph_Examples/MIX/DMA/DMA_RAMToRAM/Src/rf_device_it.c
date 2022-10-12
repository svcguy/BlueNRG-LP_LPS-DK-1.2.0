/**
  ******************************************************************************
  * @file    MIX/DMA/DMA_FLASHToRAM/Src/rf_device_it.c
  * @author  RF Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "DMA_FLASHToRAM_main.h"
#include "rf_device_it.h"

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel1;


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
  * @brief  This function handles DMA channel interrupt request.
  * @param  None
  * @retval None
  */
void DMA_IRQHandler(void)
{
  /* Check the interrupts and clear flags */

  /* Customize process using LL interface to improve performance           */
  /* (exhaustive feature management not handled)                           */
  /* Using LL interface, use :                                             */
  /* - LL_DMA_IsActiveFlag_TC1() to check complete DMA1 Interrupt */
  /* - LL_DMA_IsActiveFlag_TE1() to check error DMA1 Interrupt    */
  /* - LL_DMA_ClearFlag_GI1() to clear all DMA1 Interrupts        */
  if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1)
  {
    LL_DMA_ClearFlag_GI1(DMA1);
    TransferComplete();
  }
  else if(LL_DMA_IsActiveFlag_TE1(DMA1) == 1)
  {
    TransferError();
  }

  /* Using HAL interface, use :                                             */
  /* - HAL_DMA_IRQHandler() to handle all DMA Interrupts (complete, errors) */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



