/**
  ******************************************************************************
  * @file    LL/TIM/TIM_Freq_Div/Inc/TIM_Freq_Div_main.h
  * @author  RF Application Team
  * @brief   Header for TIM_Freq_Div_main.c module
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
#include "rf_driver_ll_pwr.h"
#include "rf_driver_ll_dma.h"
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
/**
  * @brief BSP_LED2 
  */

#ifdef STEVAL_IDB011V1
#define LED2_PIN                                LL_GPIO_PIN_8
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
#define LED2_PIN                                LL_GPIO_PIN_4
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#endif /* STEVAL_IDB012V1 */

/**
  * @brief Toggle periods for various blinking modes
  */
#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000
  
#ifdef STEVAL_IDB011V1
      
#define TIMx                                                    TIM1
#define LL_EnableClock_TIMx()                                   LL_APB0_EnableClock(LL_APB0_PERIPH_TIM1)
#define LL_EnableClock_TIMx_CH1()                               LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA)
#define LL_EnableClock_TIMx_ETR()                               LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#define TIMx_IRQHandler                                         TIM1_IRQHandler
#define TIMx_IRQn                                               TIM1_IRQn

  /* TIM1 GPIO Configuration  
  PA4/AF4     ------> TIMx_CH1  output 
  */
#define TIMx_CH1_PIN                                            LL_GPIO_PIN_4
#define TIMx_CH1_AF                                             LL_GPIO_AF_4
#define TIMx_CH1_PORT                                           GPIOA
#define LL_GPIO_SetAFPin_TIMx_CH1()                             LL_GPIO_SetAFPin_0_7(TIMx_CH1_PORT, TIMx_CH1_PIN, TIMx_CH1_AF);
 
  /* GPIO TIMx configuration 
    PB14/AF2     ------> TIMx_ETR  input
  */
#define TIMx_ETR_PIN                                            LL_GPIO_PIN_14
#define TIMx_ETR_AF                                             LL_GPIO_AF_2
#define TIMx_ETR_PORT                                           GPIOB
#define LL_GPIO_SetAFPin_TIMx_ETR()                             LL_GPIO_SetAFPin_8_15(TIMx_ETR_PORT, TIMx_ETR_PIN, TIMx_ETR_AF)
 
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1

#define TIMx                                                    TIM2
#define LL_EnableClock_TIMx()                                   LL_APB0_EnableClock(LL_APB0_PERIPH_TIM2)
#define TIMx_IRQHandler                                         TIM2_IRQHandler
#define TIMx_IRQn                                               TIM2_IRQn

  /* GPIO Configuration  
  PB4/AF4   ------> TIM2_CH1 
  */
#define TIMx_CH1_PIN                                            LL_GPIO_PIN_4
#define TIMx_CH1_AF                                             LL_GPIO_AF_4
#define TIMx_CH1_PORT                                           GPIOB
#define LL_EnableClock_TIMx_CH1()                               LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#define LL_GPIO_SetAFPin_TIMx_CH1()                             LL_GPIO_SetAFPin_0_7(TIMx_CH1_PORT, TIMx_CH1_PIN, TIMx_CH1_AF)

  /* GPIO configuration 
    PB14/AF2     ------> TIMx_ETR  input
  */
#define TIMx_ETR_PIN                                            LL_GPIO_PIN_14
#define TIMx_ETR_AF                                             LL_GPIO_AF_2
#define TIMx_ETR_PORT                                           GPIOB
#define LL_EnableClock_TIMx_ETR()                               LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#define LL_GPIO_SetAFPin_TIMx_ETR()                             LL_GPIO_SetAFPin_8_15(TIMx_ETR_PORT, TIMx_ETR_PIN, TIMx_ETR_AF)

#endif /* STEVAL_IDB012V1 */



/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* IRQ Handler treatment.*/
void UserButton_Callback(void); 

/* Private user code ---------------------------------------------------------*/
void LED_On(void);
void LED_Blinking(uint32_t Period);
void LED_Off(void);

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


