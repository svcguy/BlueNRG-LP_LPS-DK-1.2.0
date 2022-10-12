/**
  ******************************************************************************
  * @file    TIM/TIM_Freq_Div/Inc/TIM_Freq_Div_main.h
  * @author  RF Application Team
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "rf_driver_hal.h"
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "bluenrg_lp_evb_config.h"
#endif

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

#ifdef STEVAL_IDB011V1
  
#define TIMx                           TIM1
#define TIMx_IRQn                      TIM1_IRQn 
#define TIMx_IRQHandler                TIM1_IRQHandler
#define EnableClock_TIMx()             __HAL_RCC_TIM1_CLK_ENABLE()
#define DisableClock_TIMx()            __HAL_RCC_TIM1_CLK_DISABLE()

  /**TIM1 GPIO Configuration  
  PB14/AF2      ------> TIM1_ETR  input 
  */
#define TIMx_ETR_PIN                   GPIO_PIN_14
#define TIMx_ETR_AF                    GPIO_AF2_TIM1
#define TIMx_ETR_PORT                  GPIOB

  /**TIM1 GPIO Configuration  
  PA5 / AF4   ------> TIM1_CH2
  */
#define TIMx_CHx_PIN                   GPIO_PIN_5
#define TIMx_CHx_AF                    GPIO_AF4_TIM1
#define TIMx_CHx_PORT                  GPIOA

#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1

#define TIMx                           TIM2
#define TIMx_IRQn                      TIM2_IRQn
#define TIMx_IRQHandler                TIM2_IRQHandler
#define EnableClock_TIMx()             __HAL_RCC_TIM2_CLK_ENABLE()
#define DisableClock_TIMx()            __HAL_RCC_TIM2_CLK_DISABLE()

 /**TIM1 GPIO Configuration  
  PB14/AF2      ------> TIM2_ETR  input 
  */
#define TIMx_ETR_PIN                   GPIO_PIN_14
#define TIMx_ETR_AF                    GPIO_AF2_TIM2
#define TIMx_ETR_PORT                  GPIOB
    
/**TIM2 GPIO Configuration  
  PB05/AF4   ------> TIM2_CH2
  */
#define TIMx_CHx_PIN                   GPIO_PIN_5
#define TIMx_CHx_AF                    GPIO_AF4_TIM2
#define TIMx_CHx_PORT                  GPIOB

#endif /* STEVAL_IDB012V1 */




#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


