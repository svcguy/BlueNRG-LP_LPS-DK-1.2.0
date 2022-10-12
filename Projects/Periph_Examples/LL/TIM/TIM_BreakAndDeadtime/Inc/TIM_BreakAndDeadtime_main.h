/**
  ******************************************************************************
  * @file    LL/TIM/TIM_BreakAndDeadtime/Inc/TIM_BreakAndDeadtime_main.h
  * @author  RF Application Team
  * @brief   Header for TIM_BreakAndDeadtime_main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_bus.h"
#include "rf_driver_ll_system.h"
#include "rf_driver_ll_exti.h"
#include "rf_driver_ll_cortex.h"
#include "rf_driver_ll_utils.h"
#include "rf_driver_ll_tim.h"
#include "rf_driver_ll_gpio.h"
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "bluenrg_lpx.h"
#include "bluenrg_lp_evb_config.h"
#endif

#if defined(USE_FULL_ASSERT)
#include "rf_driver_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/ 
#ifdef STEVAL_IDB011V1
    
#define TIMx                                            TIM1
#define LL_EnableClock(LL_PERIPH_TIMx)                  LL_APB0_EnableClock(LL_APB0_PERIPH_TIM1);
#define TIMx_IRQn                                       TIM1_IRQn

  /**TIM1 GPIO Configuration  
  PA4 / AF4   ------> TIM1_CH1 
  */
#define TIMx_CH1_PIN                                    LL_GPIO_PIN_4
#define TIMx_CH1_AF                                     LL_GPIO_AF_4
#define TIMx_CH1_PORT                                   GPIOA

  /**TIM1 GPIO Configuration  
  PA5 / AF4   ------> TIM1_CH2
  */
#define TIMx_CH2_PIN                                    LL_GPIO_PIN_5
#define TIMx_CH2_AF                                     LL_GPIO_AF_4
#define TIMx_CH2_PORT                                   GPIOA

  /**TIMx GPIO Configuration  
  PB9 / AF3   ------> TIM1_CH1N
  */
#define TIMx_CH1N_PIN                                   LL_GPIO_PIN_9
#define TIMx_CH1N_AF                                    LL_GPIO_AF_3
#define TIMx_CH1N_PORT                                  GPIOB

  /**TIMx GPIO Configuration  
  PB0 / AF3   ------> TIM1_CH2N
  */
#define TIMx_CH2N_PIN                                   LL_GPIO_PIN_0
#define TIMx_CH2N_AF                                    LL_GPIO_AF_3
#define TIMx_CH2N_PORT                                  GPIOB
  
  /**TIMx GPIO Configuration  
  PA14 / AF4   ------> TIMx_BKIN 
  */
#define TIMx_BKIN_PIN                                   LL_GPIO_PIN_14
#define TIMx_BKIN_AF                                    LL_GPIO_AF_4
#define TIMx_BKIN_PORT                                  GPIOA
 
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1

#define TIMx                                            TIM17
#define LL_EnableClock(LL_PERIPH_TIMx)                  LL_APB0_EnableClock(LL_APB0_PERIPH_TIM17)
#define TIMx_IRQn                                       TIM17_IRQn

  /**TIM1 GPIO Configuration  
  PB3 / AF2   ------> TIM17_CH1 
  */
#define TIMx_CH1_PIN                                    LL_GPIO_PIN_3
#define TIMx_CH1_AF                                     LL_GPIO_AF_2
#define TIMx_CH1_PORT                                   GPIOB

  /**TIMx GPIO Configuration  
  PA10 / AF2   ------> TIM17_CH1N
  */
#define TIMx_CH1N_PIN                                   LL_GPIO_PIN_10
#define TIMx_CH1N_AF                                    LL_GPIO_AF_6
#define TIMx_CH1N_PORT                                  GPIOA
  
  /**TIMx GPIO Configuration  
  PB5 / AF2   ------> TIM17_BKIN 
  */
#define TIMx_BKIN_PIN                                   LL_GPIO_PIN_5
#define TIMx_BKIN_AF                                    LL_GPIO_AF_2
#define TIMx_BKIN_PORT                                  GPIOB 

#endif /* STEVAL_IDB012V1 */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


