/**
  ******************************************************************************
  * @file    I2S/I2S_Receiver/Src/rf_device_it.c
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
#include "I2S_Receiver_main.h"
#include "rf_device_it.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
extern EXTI_HandleTypeDef HEXTI_InitStructure;

/* Private define ------------------------------------------------------------*/
 
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern I2S_HandleTypeDef hi2s;

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
void HardFault_IRQHandler(void)
{
  while(1);
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
/* Peripheral Interrupt Handlers                                              */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/

/**
* @brief  This function handles external line interrupt request.
* @param  None
* @retval None
*/
void GPIOA_IRQHandler(void)
{  
  static uint32_t debounce_last = 0;
  
  if(HAL_EXTI_GetPending( &HEXTI_InitStructure )){
    if ( (debounce_count - debounce_last) >= DEBOUNCE_CNT )
    { 
      /* Add the SW no bounce */
      debounce_last = debounce_count;
      
      /* Handle user button press in dedicated function */
      HAL_EXTI_IRQHandler( &HEXTI_InitStructure );
    }
    LL_EXTI_ClearInterrupt(HEXTI_InitStructure.Line);
  }
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPIx_IRQHandler(void)
{
  HAL_I2S_IRQHandler(&hi2s);
}

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA_IRQHandler(void)
{
  if( __HAL_DMA_GET_FLAG(DMA1, DMA_FLAG_TC1 ) ){
    /* DMA Stream interrupt request for First Device */
    HAL_DMA_IRQHandler(hi2s.hdmarx);
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



