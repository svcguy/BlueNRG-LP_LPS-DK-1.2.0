/**
  ******************************************************************************
  * @file    LL/USART/USART_Comm_Tx_IT_Init/Inc/USART_Comm_Tx_IT_Init_main.h
  * @author  RF Application Team
  * @brief   Header for USART_Comm_Tx_IT_Init_main.c module
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
#include "rf_driver_ll_usart.h"
#include "rf_driver_ll_gpio.h"
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "bluenrg_lpx.h"
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
void USART_Tx_Text(void); 
void USART_TXEmpty_Callback(void); 
void USART_CharTransmitComplete_Callback(void); 
void USART_CharReception_Callback(void);
void Error_Callback(void); 

/* Private defines -----------------------------------------------------------*/

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

/* Define used to enable time-out management*/
#define USE_TIMEOUT       0

/**
  * @brief Toggle periods for various blinking modes
  */

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000




#ifdef STEVAL_IDB011V1
  /**USART1 GPIO Configuration
  PA9/AF0   ------> TX   (FTDI Yellow)
  PA8/AF0   ------> RX   (FTDI Orange)  
  PB3/AF0   ------> CTS  (FTDI Green)
  PB2/AF0   ------> RTS  (FTDI Brown)
  */
#define USART1_TX_PIN                           LL_GPIO_PIN_9
#define USART1_TX_PORT                          GPIOA
#define USART1_TX_AF                            LL_GPIO_AF_0
#define USART1_RX_PIN                           LL_GPIO_PIN_8
#define USART1_RX_PORT                          GPIOA
#define USART1_RX_AF                            LL_GPIO_AF_0 
#define LL_EnableClock_USART()                  LL_APB1_EnableClock(LL_APB1_PERIPH_USART)
#define USART1_CTS_PIN                          LL_GPIO_PIN_3
#define USART1_CTS_PORT                         GPIOB
#define USART1_CTS_AF                           LL_GPIO_AF_0
#define USART1_RTS_PIN                          LL_GPIO_PIN_2
#define USART1_RTS_PORT                         GPIOB
#define USART1_RTS_AF                           LL_GPIO_AF_0
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
  /**USART1 GPIO Configuration  
  PA1  AF2   ------> TX   (FTDI Yellow)
  PA8  AF0   ------> RX   (FTDI Orange)
  PB3  AF0   ------> CTS  (FTDI Green)
  PB2  AF0   ------> RTS  (FTDI Brown)
  */
#define USART1_TX_PIN                           LL_GPIO_PIN_1
#define USART1_TX_PORT                          GPIOA
#define USART1_TX_AF                            LL_GPIO_AF_2
#define USART1_RX_PIN                           LL_GPIO_PIN_8
#define USART1_RX_PORT                          GPIOA
#define USART1_RX_AF                            LL_GPIO_AF_0 
#define LL_EnableClock_USART()                  LL_APB1_EnableClock(LL_APB1_PERIPH_USART)
#define USART1_CTS_PIN                          LL_GPIO_PIN_3
#define USART1_CTS_PORT                         GPIOB
#define USART1_CTS_AF                           LL_GPIO_AF_0
#define USART1_RTS_PIN                          LL_GPIO_PIN_2
#define USART1_RTS_PORT                         GPIOB
#define USART1_RTS_AF                           LL_GPIO_AF_0
#endif /* STEVAL_IDB012V1 */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


