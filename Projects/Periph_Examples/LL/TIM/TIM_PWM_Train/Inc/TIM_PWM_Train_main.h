/**
  ******************************************************************************
  * @file    LL/EXTI/TIM_PWM_Train/Inc/TIM_PWM_Train_main.h
  * @author  RF Application Team
  * @brief   Header for TIM_PWM_Train_main.c module
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

/* Includes ------------------------------------------------------------------*/
#include "rf_driver_ll_bus.h"
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_system.h"
#include "rf_driver_ll_utils.h"
#include "rf_driver_ll_pwr.h"
#include "rf_driver_ll_exti.h"
#include "rf_driver_ll_tim.h"
#include "rf_driver_ll_gpio.h"
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "bluenrg_lpx.h"
#include "bluenrg_lp_evb_config.h"
#endif

#if defined(USE_FULL_ASSERT)
#include "rf_driver_assert.h"
#endif /* USE_FULL_ASSERT */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ----------------------------------------------------------- */

/* Exported functions ------------------------------------------------------- */

/* Private define ------------------------------------------------------------*/
#ifdef STEVAL_IDB011V1
      
#define TIMx                                      TIM1
#define LL_EnableClock_TIMx()                     LL_APB0_EnableClock(LL_APB0_PERIPH_TIM1)
#define TIMx_IRQHandler                           TIM1_IRQHandler
#define TIMx_IRQn                                 TIM1_IRQn

  /** TIMx GPIO Configuration  
      PA1/AF4     TIMx_CH4 
   */
#define TIMx_CH4_PIN                              LL_GPIO_PIN_1
#define TIMx_CH4_AF                               LL_GPIO_AF_4
#define TIMx_CH4_PORT                             GPIOA
#define LL_EnableClock_TIMx_CH4()                 LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA)
#define LL_GPIO_SetAFPin_TIMx_CH4()               LL_GPIO_SetAFPin_0_7(TIMx_CH4_PORT, TIMx_CH4_PIN, TIMx_CH4_AF);
 
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
#define TIMx                                      TIM2
#define LL_EnableClock_TIMx()                     LL_APB0_EnableClock(LL_APB0_PERIPH_TIM2);
#define TIMx_IRQHandler                           TIM2_IRQHandler
#define TIMx_IRQn                                 TIM2_IRQn
  /** TIMx GPIO Configuration  
      PB3/AF3     TIM2_CH4 
   */
#define TIMx_CH4_PIN                              LL_GPIO_PIN_3
#define TIMx_CH4_AF                               LL_GPIO_AF_3
#define TIMx_CH4_PORT                             GPIOB
#define LL_EnableClock_TIMx_CH4()                 LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#define LL_GPIO_SetAFPin_TIMx_CH4()               LL_GPIO_SetAFPin_0_7(TIMx_CH4_PORT, TIMx_CH4_PIN, TIMx_CH4_AF);
#endif /* STEVAL_IDB012V1 */



#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


