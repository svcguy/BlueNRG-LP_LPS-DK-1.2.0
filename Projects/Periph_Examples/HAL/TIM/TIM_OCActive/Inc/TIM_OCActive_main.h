/**
  ******************************************************************************
  * @file    TIM/TIM_OCActive/Inc/TIM_OCActive_main.h
  * @author  RF Application Team
  * @brief   Header for TIM_OCActive_main.c module
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
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
/* Compute the prescaler value to have TIMx counter clock equal to 10 kHz */
#define  PRESCALER_VALUE  ((( HAL_TIM_GetPeriphClock(htimx.Instance) ) / 10000) - 1)

/* The TIM1 CCR1 register value is equal to 10000: 
 * TIM1_CH1 delay = CCR1_Val/TIM1 counter clock  = 1s 
 * so the TIM1 Channel 1 generates a signal with a delay equal to 1s. 
 */
#define  PULSE1_VALUE       10000        /* Capture Compare 1 Value  */

/* The TIM1 CCR2 register value is equal to 5000:
 * TIM1_CH2 delay = CCR2_Val/TIM1 counter clock = 500 ms
 * so the TIM1 Channel 2 generates a signal with a delay equal to 500 ms.
 */
#define  PULSE2_VALUE       5000         /* Capture Compare 2 Value  */
 
/* The TIM1 CCR3 register value is equal to 2500:
 * TIM1_CH3 delay = CCR3_Val/TIM1 counter clock = 250 ms
 * so the TIM1 Channel 3 generates a signal with a delay equal to 250 ms.
 */ 
#define  PULSE3_VALUE       2500         /* Capture Compare 3 Value  */

/* The TIM1 CCR4 register value is equal to 1250:
 * TIM1_CH4 delay = CCR4_Val/TIM1 counter clock = 125 ms
 * so the TIM1 Channel 4 generates a signal with a delay equal to 125 ms.
 */ 
#define  PULSE4_VALUE       1250         /* Capture Compare 4 Value  */ 

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

  /**TIM2 GPIO Configuration  
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

