/**
  ******************************************************************************
  * @file    TIM/TIM_OCInactive/Inc/TIM_OCInactive_main.h
  * @author  RF Application Team
  * @brief   Header for TIM_OCInactive_main.c module
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
#define  PULSE1_VALUE       10000         /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       5000         /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       2500         /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       1250         /* Capture Compare 4 Value  */

#ifdef STEVAL_IDB011V1
#define TIMx                            TIM1
#define TIMx_IRQHandler                 TIM1_IRQHandler
#define TIMx_IRQn                       TIM1_IRQn
#define EnableClock_TIMx()              __HAL_RCC_TIM1_CLK_ENABLE()  
#define DisableClock_TIMx()             __HAL_RCC_TIM1_CLK_DISABLE()
#define EnableClock_GPIOx()             __HAL_RCC_GPIOA_CLK_ENABLE()    
#define GPIO_OUT_CH1                    GPIO_PIN_11
#define GPIO_OUT_CH2                    GPIO_PIN_12
#define GPIO_OUT_CH3                    GPIO_PIN_13
#define GPIO_OUT_CH4                    GPIO_PIN_14
#define GPIO_OUT_PORT_CH1_2_3_4         GPIOA
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
#define TIMx                            TIM2
#define TIMx_IRQHandler                 TIM2_IRQHandler
#define TIMx_IRQn                       TIM2_IRQn
#define EnableClock_TIMx()              __HAL_RCC_TIM2_CLK_ENABLE() 
#define DisableClock_TIMx()             __HAL_RCC_TIM2_CLK_DISABLE()
#define EnableClock_GPIOx()             __HAL_RCC_GPIOB_CLK_ENABLE()  
#define GPIO_OUT_CH1                    GPIO_PIN_2
#define GPIO_OUT_CH2                    GPIO_PIN_3
#define GPIO_OUT_CH3                    GPIO_PIN_6
#define GPIO_OUT_CH4                    GPIO_PIN_5
#define GPIO_OUT_PORT_CH1_2_3_4         GPIOB
#endif /* STEVAL_IDB012V1 */



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

