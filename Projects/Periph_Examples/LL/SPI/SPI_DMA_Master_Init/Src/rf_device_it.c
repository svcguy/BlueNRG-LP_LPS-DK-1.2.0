/**
  ******************************************************************************
  * @file    LL/SPI/SPI_DMA_Master_Init/Src/rf_device_it.c
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
#include "SPI_DMA_Master_Init_main.h"

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
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/

void USART1_IRQHandler(void)
{  
  BSP_COM_IRQHandler();
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
    SPI_MASTER_TransferError_Callback();
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
    SPI_MASTER_TransferError_Callback();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


