/**
  ******************************************************************************
  * @file    TIM/TIM_InputCapture/Inc/TIM_InputCapture_main.h
  * @author  RF Application Team
  * @brief   Header for TIM_InputCapture_main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "rf_driver_hal.h"

/* Private includes ----------------------------------------------------------*/
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "bluenrg_lp_evb_config.h"
#endif

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

#ifdef STEVAL_IDB011V1
  
#define TIMx                            TIM1
#define TIMx_IRQn                       TIM1_IRQn
#define TIMx_IRQHandler                 TIM1_IRQHandler
#define EnableClock_TIMx()              __HAL_RCC_TIM1_CLK_ENABLE()  
#define DisableClock_TIMx()             __HAL_RCC_TIM1_CLK_DISABLE()    
  
  /**TIM1 GPIO Configuration  
  PA4 / AF4   ------> TIM1_CH1 
  */
#define TIMx_CH1_PIN                    GPIO_PIN_4
#define TIMx_CH1_AF                     GPIO_AF4_TIM1
#define TIMx_CH1_PORT                   GPIOA

#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1

#define TIMx                            TIM2
#define TIMx_IRQn                       TIM2_IRQn
#define TIMx_IRQHandler                 TIM2_IRQHandler
#define EnableClock_TIMx()              __HAL_RCC_TIM2_CLK_ENABLE() 
#define DisableClock_TIMx()             __HAL_RCC_TIM2_CLK_DISABLE()  
  
  /**TIM2 GPIO Configuration  
  PB04/AF4   ------> TIM2_CH1 
  */
#define TIMx_CH1_PIN                    GPIO_PIN_4
#define TIMx_CH1_AF                     GPIO_AF4_TIM2
#define TIMx_CH1_PORT                   GPIOB
 
#endif /* STEVAL_IDB012V1 */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

