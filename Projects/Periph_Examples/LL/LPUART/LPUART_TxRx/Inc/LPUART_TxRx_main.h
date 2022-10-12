/**
  ******************************************************************************
  * @file    LL/LPUART/LPUART_TxRx/Inc/LPUART_TxRx_main.h
  * @author  RF Application Team
  * @brief   Header for LPUART_TxRx_main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "rf_driver_ll_cortex.h"
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_system.h"
#include "rf_driver_ll_utils.h"
#include "rf_driver_ll_gpio.h"
#include "rf_driver_ll_lpuart.h"
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "bluenrg_lpx.h"
#endif

#if defined(USE_FULL_ASSERT)
#include "rf_driver_assert.h"
#endif /* USE_FULL_ASSERT */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/**
  * @brief BSP_LED2 
  */

#ifdef STEVAL_IDB011V1
#define LEDx_PIN                                LL_GPIO_PIN_8
#define LEDx_GPIO_PORT                          GPIOB
#define LEDx_GPIO_CLK_ENABLE()                  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
#define LEDx_PIN                                LL_GPIO_PIN_4
#define LEDx_GPIO_PORT                          GPIOB
#define LEDx_GPIO_CLK_ENABLE()                  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#endif /* STEVAL_IDB012V1 */


#ifdef STEVAL_IDB011V1
#define LPUART_TX_PORT 						GPIOB
#define LPUART_TX_PIN						LL_GPIO_PIN_4
#define LPUART_TX_AF						LL_GPIO_AF_0
#define LL_GPIO_SetAFPin_LPUART_TX()                            LL_GPIO_SetAFPin_0_7(LPUART_TX_PORT, LPUART_TX_PIN, LPUART_TX_AF)
#define LPUART_RX_PORT 						GPIOB
#define LPUART_RX_PIN						LL_GPIO_PIN_5
#define LPUART_RX_AF						LL_GPIO_AF_0
#define LL_GPIO_SetAFPin_LPUART_RX()                            LL_GPIO_SetAFPin_0_7(LPUART_RX_PORT, LPUART_RX_PIN, LPUART_RX_AF)
#define LL_EnableClock_LPUART()                                 LL_APB1_EnableClock(LL_APB1_PERIPH_LPUART)
#define LL_EnableClock_LPUART_TX_PORT()                         LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#define LL_EnableClock_LPUART_RX_PORT()                         LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
#define LPUART_TX_PORT 						GPIOB
#define LPUART_TX_PIN						LL_GPIO_PIN_4
#define LPUART_TX_AF						LL_GPIO_AF_0
#define LL_GPIO_SetAFPin_LPUART_TX()                            LL_GPIO_SetAFPin_0_7(LPUART_TX_PORT, LPUART_TX_PIN, LPUART_TX_AF)
#define LPUART_RX_PORT 						GPIOB
#define LPUART_RX_PIN						LL_GPIO_PIN_5
#define LPUART_RX_AF						LL_GPIO_AF_0
#define LL_GPIO_SetAFPin_LPUART_RX()                            LL_GPIO_SetAFPin_0_7(LPUART_RX_PORT, LPUART_RX_PIN, LPUART_RX_AF)
#define LL_EnableClock_LPUART()                                 LL_APB1_EnableClock(LL_APB1_PERIPH_LPUART)
#define LL_EnableClock_LPUART_TX_PORT()                         LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#define LL_EnableClock_LPUART_RX_PORT()                         LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#endif /* STEVAL_IDB012V1 */
/**
  * @brief Toggle periods for various blinking modes
  */
#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* IRQ Handler treatment functions */
void LPUART_CharReception_Callback(void); 
void Error_Callback(void); 

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


