/**
  ******************************************************************************
  * @file    I2C/I2C_DMA/Src/rf_device_it.c 
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
#include "I2C_DMA_main.h"
#include "rf_device_it.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
 
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2cx_tx;
extern DMA_HandleTypeDef hdma_i2cx_rx;
extern I2C_HandleTypeDef hi2cx;


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
  * @brief This function handles DMA1 channel global interrupt.
  */
void DMA_IRQHandler(void)
{
  if( __HAL_DMA_GET_FLAG(DMA1, DMA_FLAG_TC1 ) ){
    /* DMA1_Channel1_IRQn */
    HAL_DMA_IRQHandler(&hdma_i2cx_tx);
  }
  
  if( __HAL_DMA_GET_FLAG(DMA1, DMA_FLAG_TC2 ) ){
    /* DMA1_Channel2_IRQn */
    HAL_DMA_IRQHandler(&hdma_i2cx_rx);
  }
}

/**
  * @brief This function handles I2Cx event interrupt.
  */
void I2Cx_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2cx);

/* This function handles I2Cx error interrupt.*/
  HAL_I2C_ER_IRQHandler(&hi2cx);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



