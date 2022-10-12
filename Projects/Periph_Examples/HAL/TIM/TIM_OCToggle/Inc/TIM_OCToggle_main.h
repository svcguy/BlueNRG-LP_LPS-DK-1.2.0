/**
  ******************************************************************************
  * @file    TIM/TIM_OCToggle/Inc/TIM_OCToggle_main.h
  * @author  RF Application Team
  * @brief   Header for TIM_OCToggle_main.c module
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
/* Define Timer periode and pulse  */
/* ---------------------------------------------------------------------------
   TIM1 Configuration: Output Compare Toggle Mode:

  To get TIM1 counter clock at 1 MHz, the prescaler is computed as follows:
  Prescaler = (TIM1CLK / TIM1 counter clock) - 1
  Prescaler = (HAL_TIM_GetPeriphClock(htim1.Instance) /1000000) - 1

  CC1 update rate = TIM1 counter clock / uhCCR1_Val
                  = 1 MHz/625 = 1600 Hz
  ==> So the TIM1 Channel 1 generates a periodic signal with a frequency equal
      to 800 Hz.

  CC2 update rate = TIM1 counter clock / uhCCR2_Val
                  = 1 MHz/1250 = 800 Hz
  ==> So the TIM1 Channel 2 generates a periodic signal with a frequency equal
      to 400 Hz.

  CC3 update rate = TIM1 counter clock / uhCCR3_Val
                  = 1 MHz/2500 = 400 Hz
  ==> So the TIM1 Channel 3 generates a periodic signal with a frequency equal
      to 200 Hz.

  CC4 update rate = TIM1 counter clock / uhCCR4_Val
                  = 1 MHz/5000 = 200 Hz
  ==> So the TIM1 Channel 4 generates a periodic signal with a frequency equal
      to 100 Hz.


  --------------------------------------------------------------------------- */
#define PRESCALER_VALUE (uint32_t)(( HAL_TIM_GetPeriphClock(htim1.Instance) / 1000000) - 1)

#define PULSE1_VALUE 625
#define PULSE2_VALUE 1250
#define PULSE3_VALUE 2500
#define PULSE4_VALUE 5000

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

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

  /**TIM1 GPIO Configuration  
  PA5 / AF4   ------> TIM1_CH2
  */
#define TIMx_CH2_PIN                    GPIO_PIN_5
#define TIMx_CH2_AF                     GPIO_AF4_TIM1
#define TIMx_CH2_PORT                   GPIOA

  /**TIM1 GPIO Configuration  
  PB2/AF3     ------> TIM1_CH3
  */
#define TIMx_CH3_PIN                    GPIO_PIN_2
#define TIMx_CH3_AF                     GPIO_AF3_TIM1
#define TIMx_CH3_PORT                   GPIOB

  /**TIM1 GPIO Configuration  
  PA1/AF4     ------> TIM1_CH4 
  */
#define TIMx_CH4_PIN                    GPIO_PIN_1
#define TIMx_CH4_AF                     GPIO_AF4_TIM1
#define TIMx_CH4_PORT                   GPIOA

  /**TIM1 GPIO Configuration  
  PB14/AF4    ------> TIM1_CH5
  */
#define TIMx_CH5_PIN                    GPIO_PIN_14
#define TIMx_CH5_AF                     GPIO_AF4_TIM1
#define TIMx_CH5_PORT                   GPIOB

  /**TIM1 GPIO Configuration  
  PA11/AF4    ------> TIM1_CH6
  */
#define TIMx_CH6_PIN                    GPIO_PIN_11
#define TIMx_CH6_AF                     GPIO_AF4_TIM1
#define TIMx_CH6_PORT                   GPIOA
 

#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1

#define TIMx                            TIM2
#define TIMx_IRQn                       TIM2_IRQn
#define TIMx_IRQHandler                 TIM2_IRQHandler
#define EnableClock_TIMx()              __HAL_RCC_TIM2_CLK_ENABLE() 
#define DisableClock_TIMx()             __HAL_RCC_TIM2_CLK_DISABLE()  
  
  /**TIM2 GPIO Configuration  
  PB06/AF4   ------> TIM2_CH1 
  */
#define TIMx_CH1_PIN                    GPIO_PIN_6
#define TIMx_CH1_AF                     GPIO_AF4_TIM2
#define TIMx_CH1_PORT                   GPIOB

  /**TIM2 GPIO Configuration  
  PB05/AF4   ------> TIM2_CH2
  */
#define TIMx_CH2_PIN                    GPIO_PIN_5
#define TIMx_CH2_AF                     GPIO_AF4_TIM2
#define TIMx_CH2_PORT                   GPIOB

  /**TIM1 GPIO Configuration  
  PB02/AF3   ------> TIM2_CH3
  */
#define TIMx_CH3_PIN                    GPIO_PIN_2
#define TIMx_CH3_AF                     GPIO_AF3_TIM2
#define TIMx_CH3_PORT                   GPIOB

  /**TIM1 GPIO Configuration  
  PB03/AF3   ------> TIM2_CH4 
  */
#define TIMx_CH4_PIN                    GPIO_PIN_3
#define TIMx_CH4_AF                     GPIO_AF3_TIM2
#define TIMx_CH4_PORT                   GPIOB
 
#endif /* STEVAL_IDB012V1 */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


