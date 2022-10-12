/**
  ******************************************************************************
  * @file           : USART_Comm_Rx_IT_Cont_Init_main.h
  * @brief          : Header for USART_Comm_Rx_IT_Cont_Init_main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "rf_driver_ll_usart.h"
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
void USART_CharReception_Callback(void);
void Error_Callback(void);

/* Private defines -----------------------------------------------------------*/

#ifdef STEVAL_IDB011V1
  /**USART1 GPIO Configuration  
  PA9  AF0   ------> USART1_TX    
  PA8  AF0   ------> USART1_RX   
  */
#define USART1_TX_PIN                      LL_GPIO_PIN_9
#define USART1_TX_PORT                     GPIOA
#define USART1_TX_AF                       LL_GPIO_AF_0
#define USART1_RX_PIN                      LL_GPIO_PIN_8
#define USART1_RX_PORT                     GPIOA
#define USART1_RX_AF                       LL_GPIO_AF_0 
#define LL_EnableClock_USART()             LL_APB1_EnableClock(LL_APB1_PERIPH_USART)
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
  /**USART1 GPIO Configuration  
  PA1  AF2   ------> USART1_TX    
  PB0  AF0   ------> USART1_RX   
  */
#define USART1_TX_PIN                      LL_GPIO_PIN_1
#define USART1_TX_PORT                     GPIOA
#define USART1_TX_AF                       LL_GPIO_AF_2
#define USART1_RX_PIN                      LL_GPIO_PIN_0
#define USART1_RX_PORT                     GPIOB
#define USART1_RX_AF                       LL_GPIO_AF_0 
#define LL_EnableClock_USART()             LL_APB1_EnableClock(LL_APB1_PERIPH_USART)
#endif /* STEVAL_IDB012V1 */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


