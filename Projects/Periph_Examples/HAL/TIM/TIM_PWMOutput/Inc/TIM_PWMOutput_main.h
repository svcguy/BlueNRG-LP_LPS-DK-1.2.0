/**
  ******************************************************************************
  * @file    TIM/TIM_PWMOutput/Inc/TIM_PWMOutput_main.h
  * @author  RF Application Team
  * @brief   Header for TIM_PWMOutput_main.c module
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
  *******************************************************************************/

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
#ifdef STEVAL_IDB011V1
  
#define TIMx                            TIM1
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

  
#ifdef STEVAL_IDB011V1
/**TIMx GPIO Configuration  
  PA14 / AF4   ------> TIMx_BKIN 
  */
#define TIMx_BKIN_PIN                                   LL_GPIO_PIN_14
#define TIMx_BKIN_AF                                    LL_GPIO_AF_4
#define TIMx_BKIN_PORT                                  GPIOA 
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
/**TIMx GPIO Configuration  
  PB5 / AF2   ------> TIM17_BKIN 
  */
#define TIMx_BKIN_PIN                                   LL_GPIO_PIN_5
#define TIMx_BKIN_AF                                    LL_GPIO_AF_2
#define TIMx_BKIN_PORT                                  GPIOB 
#endif /* STEVAL_IDB012V1 */


#ifdef STEVAL_IDB012V1

#define TIMx                            TIM17
#define EnableClock_TIMx()              __HAL_RCC_TIM17_CLK_ENABLE() 
#define DisableClock_TIMx()             __HAL_RCC_TIM17_CLK_DISABLE()  
  
  /**TIM GPIO Configuration  
  PB3 / AF2   ------> TIM17_CH1 
  */
#define TIMx_CH1_PIN                    GPIO_PIN_3
#define TIMx_CH1_AF                     GPIO_AF2_TIM17
#define TIMx_CH1_PORT                   GPIOB

#endif /* STEVAL_IDB012V1 */


  
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
/* Compute the prescaler value to have TIM1 counter clock equal to 1000000 Hz */
#define PRESCALER_VALUE     (uint32_t)((HAL_TIM_GetPeriphClock(TIMx) / 1000000) - 1)

/* -----------------------------------------------------------------------
TIM1 Configuration: generate 6 PWM signals with 6 different duty cycles.

    In this example TIM1 input clock (TIM1CLK) is set to APB0 clock (PCLK1),
    since APB0 prescaler is equal to 1.
      TIM1CLK = PCLK1
        => TIM1CLK = HCLK = 64 MHz

    To get TIM1 counter clock at 1 MHz, the prescaler is computed as follows:
       Prescaler = (TIM1CLK / TIM1 counter clock) - 1
       Prescaler = (64 MHz /1 MHz) - 1

    To get TIM1 output clock at 24 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM1 counter clock / TIM1 output clock) - 1
           = 40

    TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR + 1)* 100 = 50%
    TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR + 1)* 100 = 37.5%
    TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR + 1)* 100 = 25%
    TIM1 Channel4 duty cycle = (TIM1_CCR4/ TIM1_ARR + 1)* 100 = 12.5%
    TIM1 Channel5 duty cycle = (TIM1_CCR3/ TIM1_ARR + 1)* 100 = 75%
    TIM1 Channel6 duty cycle = (TIM1_CCR4/ TIM1_ARR + 1)* 100 = 62.5%

  ----------------------------------------------------------------------- */

/* Initialize TIMx peripheral as follows:
   + Prescaler = (HAL_TIM_GetPeriphClock(htimx.Instance) / 1000000) - 1
   + Period = (41 - 1)
   + ClockDivision = 0
   + Counter direction = Up
*/
#define  PERIOD_VALUE       (uint32_t)(41 - 1)  /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE*0.50)        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       (uint32_t)(PERIOD_VALUE*0.375)       /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       (uint32_t)(PERIOD_VALUE*0.25)        /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       (uint32_t)(PERIOD_VALUE*0.125)       /* Capture Compare 4 Value  */
#define  PULSE5_VALUE       (uint32_t)(PERIOD_VALUE*0.75)        /* Capture Compare 3 Value  */
#define  PULSE6_VALUE       (uint32_t)(PERIOD_VALUE*0.625)       /* Capture Compare 4 Value  */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


