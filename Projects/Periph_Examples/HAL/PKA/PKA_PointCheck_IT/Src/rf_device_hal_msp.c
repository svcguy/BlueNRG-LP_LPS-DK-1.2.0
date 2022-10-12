/**
******************************************************************************
* @file    PKA/PKA_PointCheck_IT/Src/rf_device_hal_msp.c
* @author  RF Application Team
* @brief   HAL MSP module.
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
#include "PKA_PointCheck_IT_main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* External functions --------------------------------------------------------*/

/**
* Initializes the Global MSP.
*/
void HAL_MspInit(void)
{ 
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
}

/**
* @brief PKA MSP Initialization
* This function configures the hardware resources used in this example
* @param hpka: PKA handle pointer
* @retval None
*/
void HAL_PKA_MspInit(PKA_HandleTypeDef* hpka)
{
  if(hpka->Instance==PKA)
  {
    /* Peripheral clock enable */
    __HAL_RCC_PKA_CLK_ENABLE();
      
    /* Configure NVIC for PKA interrupts */
    /*   Set priority for PKA_IRQn */
    /*   Enable PKA_IRQn */
    NVIC_SetPriority(PKA_IRQn, IRQ_HIGH_PRIORITY);  
    NVIC_EnableIRQ(PKA_IRQn); 
  }
}

/**
* @brief PKA MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hpka: PKA handle pointer
* @retval None
*/
void HAL_PKA_MspDeInit(PKA_HandleTypeDef* hpka)
{
  if(hpka->Instance==PKA)
  {
    /* Enable PKA reset state */
    __HAL_RCC_PKA_FORCE_RESET();
    /* Release PKA from reset state */
    __HAL_RCC_PKA_RELEASE_RESET();
    /* Peripheral clock disable */
    __HAL_RCC_PKA_CLK_DISABLE();
  }
}

/**
* @brief RNG MSP Initialization
* This function configures the hardware resources used in this example
* @param hrng: RNG handle pointer
* @retval None
*/
void HAL_RNG_MspInit(RNG_HandleTypeDef* hrng)
{
  if(hrng->Instance==RNG)
  {
    /* Peripheral clock enable */
    __HAL_RCC_RNG_CLK_ENABLE();
  }
}

/**
* @brief RNG MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hrng: RNG handle pointer
* @retval None
*/
void HAL_RNG_MspDeInit(RNG_HandleTypeDef* hrng)
{
  if(hrng->Instance==RNG)
  {
    /* Peripheral clock disable */
    __HAL_RCC_RNG_CLK_DISABLE();

    /* Enable RNG reset state */
    __HAL_RCC_RNG_FORCE_RESET();

    /* Release RNG from reset state */
    __HAL_RCC_RNG_RELEASE_RESET();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

