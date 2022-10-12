/**
  ******************************************************************************
  * @file    UART/UART_TxPolling_RxIT/Inc/UART_TxPolling_RxIT_main.h
  * @author  RF Application Team
  * @brief   Header for UART_TxPolling_RxIT_main.c module
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
#include "rf_driver_hal.h"

/* Private includes ----------------------------------------------------------*/
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "bluenrg_lp_evb_config.h"
#endif
#include "rf_driver_ll_bus.h"
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_system.h"
#include "rf_driver_ll_utils.h"
#include "rf_driver_ll_gpio.h"
#include "rf_driver_ll_exti.h"
#include "rf_driver_ll_usart.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* IRQ Handler treatment functions */
void UART_CharReception_Callback(void); 
void UART_Error_Callback(void); 

/* Private defines -----------------------------------------------------------*/
/* Size of Transmission buffer */
#define TXSTARTMESSAGESIZE                   (COUNTOF(aTxStartMessage) - 1)
#define TXENDMESSAGESIZE                     (COUNTOF(aTxEndMessage) - 1)

/* Size of Reception buffer */
#define RX_BUFFER_SIZE                     10


#ifdef STEVAL_IDB011V1
#define USARTx                             USART1
#define USARTx_IRQn                        USART1_IRQn
#define USARTx_IRQHandler                  USART1_IRQHandler
  /**USARTx GPIO Configuration  
  PA9  AF0   ------> USARTx_TX    
  PA8  AF0   ------> USARTx_RX   
  */
#define USARTx_TX_PIN                      GPIO_PIN_9
#define USARTx_TX_PORT                     GPIOA
#define USARTx_TX_AF                       GPIO_AF0_USART1
#define USARTx_RX_PIN                      GPIO_PIN_8
#define USARTx_RX_PORT                     GPIOA
#define USARTx_RX_AF                       GPIO_AF0_USART1
#define EnableClock_USART                  __HAL_RCC_USART_CLK_ENABLE
#define DisableClock_USART                 __HAL_RCC_USART_CLK_DISABLE
#define EnableClock_USART_TX_PORT          __HAL_RCC_GPIOA_CLK_ENABLE
#define EnableClock_USART_RX_PORT          __HAL_RCC_GPIOA_CLK_ENABLE
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
#define USARTx                             USART1
#define USARTx_IRQn                        USART1_IRQn
#define USARTx_IRQHandler                  USART1_IRQHandler
  /**USARTx GPIO Configuration  
  PA1  AF2   ------> USARTx_TX    
  PB0  AF0   ------> USARTx_RX   
  */
#define USARTx_TX_PIN                      GPIO_PIN_1
#define USARTx_TX_PORT                     GPIOA
#define USARTx_TX_AF                       GPIO_AF2_USART1
#define USARTx_RX_PIN                      GPIO_PIN_0
#define USARTx_RX_PORT                     GPIOB
#define USARTx_RX_AF                       GPIO_AF0_USART1 
#define EnableClock_USART                  __HAL_RCC_USART_CLK_ENABLE
#define DisableClock_USART                 __HAL_RCC_USART_CLK_DISABLE
#define EnableClock_USART_TX_PORT          __HAL_RCC_GPIOA_CLK_ENABLE
#define EnableClock_USART_RX_PORT          __HAL_RCC_GPIOB_CLK_ENABLE
#endif /* STEVAL_IDB012V1 */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


