/**
  ******************************************************************************
  * @file    LL/USART/USART_TxRx_Inverted/Src/rf_device_it.c
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
#include "USART_TxRx_Inverted_main.h"
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

#define DEBOUNCE_CNT  350
volatile uint32_t debounce_count = 0;

/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void SysTick_IRQHandler(void)
{
  debounce_count++;  
}

/******************************************************************************/
/* Peripheral Interrupt Handlers                                              */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/

/**
  * @brief This function handles DMA channel6 and channel7 global interrupt.
  */
void DMA_IRQHandler(void)
{
  if (LL_DMA_IsActiveFlag_TC6(DMA1))
  {
    LL_DMA_ClearFlag_GI6(DMA1);
    /* Call function Reception complete Callback */
    DMA1_ReceiveComplete_Callback();
  }
  else if (LL_DMA_IsActiveFlag_TE6(DMA1))
  {
    /* Call Error function */
    USART_TransferError_Callback();
  }

  if (LL_DMA_IsActiveFlag_TC7(DMA1))
  {
    LL_DMA_ClearFlag_GI7(DMA1);
    /* Call function Transmission complete Callback */
    DMA1_TransmitComplete_Callback();
  }
  else if (LL_DMA_IsActiveFlag_TE7(DMA1))
  {
    /* Call Error function */
    USART_TransferError_Callback();
  }
}

/**
******************************************************************************
* @func   USART1_IRQHandler()
* @brief  handles USART1 global interrupt
* @param  None
* @gvar   
* @gfunc  LL_USART_IsActiveFlag_RXNE(), LL_USART_IsEnabledIT_RXNE(),
*         USART_CharReception_Callback(), Error_Callback
* @retval None
******************************************************************************
*/
void USART1_IRQHandler(void)
{
  if(LL_USART_IsEnabledIT_ERROR(USART1) && LL_USART_IsActiveFlag_NE(USART1))
  {
    /* Call Error function */
    Error_Handler();
  }
  else
  {
    Error_Handler();    /* Call Error function */
  }
}   

/**
* @brief  This function handles GPIOA interrupt request.
* @param  None
* @retval None
*/
void GPIOA_IRQHandler(void)
{
  USER_BUTTON_IRQHandler();
}

/**
* @brief  This function handles user button interrupt request.
* @param  None
* @retval None
*/
void USER_BUTTON_IRQHandler(void)
{
  static uint32_t debounce_last = 0;
  
  if (LL_EXTI_IsInterruptPending(USER_BUTTON_EXTI_LINE) != RESET)
  {
    LL_EXTI_ClearInterrupt(USER_BUTTON_EXTI_LINE);
    
    if ( (debounce_count - debounce_last) >= DEBOUNCE_CNT )
    {
      /* Add the SW no bounce */
      debounce_last = debounce_count;
      /* Handle user button press in dedicated function */
      UserButton_Callback();
    }
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



