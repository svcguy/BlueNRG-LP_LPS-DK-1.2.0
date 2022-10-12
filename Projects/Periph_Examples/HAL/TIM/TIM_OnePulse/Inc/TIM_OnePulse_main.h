/**
  ******************************************************************************
  * @file    TIM/TIM_OnePulse/Inc/TIM_OnePulse_main.h
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
/* Definition for TIMx clock resources */

#ifdef STEVAL_IDB011V1
  
#define TIMx                       TIM1
#define EnableClock_TIMx()         __HAL_RCC_TIM1_CLK_ENABLE()  
#define DisableClock_TIMx()        __HAL_RCC_TIM1_CLK_DISABLE()    
  
  
  /**TIM1 GPIO Configuration  
  PA4 / AF4   ------> TIM1_CH1 
  */
#define TIMx_CH1_PIN    GPIO_PIN_4
#define TIMx_CH1_AF     GPIO_AF4_TIM1
#define TIMx_CH1_PORT   GPIOA

  /**TIM1 GPIO Configuration  
  PA5 / AF4   ------> TIM1_CH2
  */
#define TIMx_CH2_PIN    GPIO_PIN_5
#define TIMx_CH2_AF     GPIO_AF4_TIM1
#define TIMx_CH2_PORT   GPIOA

#define EXTERNAL_TRIGGER_SIGNAL_PIN             GPIO_PIN_8          
#define EXTERNAL_TRIGGER_SIGNAL_PORT            GPIOB

#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1

#define TIMx                                            TIM2
#define EnableClock_TIMx()                              __HAL_RCC_TIM2_CLK_ENABLE() 
#define DisableClock_TIMx()                             __HAL_RCC_TIM2_CLK_DISABLE()  
  
  /**TIM2 GPIO Configuration  
  PB06/AF4   ------> TIM2_CH1 
  */
#define TIMx_CH1_PIN                                    GPIO_PIN_6
#define TIMx_CH1_AF                                     GPIO_AF4_TIM2
#define TIMx_CH1_PORT                                   GPIOB

  /**TIM2 GPIO Configuration  
  PB05/AF4   ------> TIM2_CH2
  */
#define TIMx_CH2_PIN                                    GPIO_PIN_5
#define TIMx_CH2_AF                                     GPIO_AF4_TIM2
#define TIMx_CH2_PORT                                   GPIOB

#define EXTERNAL_TRIGGER_SIGNAL_PIN                     GPIO_PIN_3          
#define EXTERNAL_TRIGGER_SIGNAL_PORT                    GPIOB

#endif /* STEVAL_IDB012V1 */

 
    /**
  * @brief Key push-button
  */
#if defined(STEVAL_IDB011V1) || defined(STEVAL_IDB012V1) 
#define USER_BUTTON_PIN                         GPIO_PIN_10
#define USER_BUTTON_GPIO_PORT                   GPIOA
#define USER_BUTTON_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE() 
#define USER_BUTTON_SYSCFG_CLK_ENABLE()         __HAL_RCC_SYSCFG_CLK_ENABLE()   
#define USER_BUTTON_EXTI_LINE                   EXTI_LINE_PA10
#define USER_BUTTON_EXTI_IRQn                   GPIOA_IRQn
#define USER_BUTTON_IRQHANDLER                  GPIOA_IRQHandler
#endif

/* Exported functions ------------------------------------------------------- */
void Example_EXTI_Callback(uint32_t Line);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


