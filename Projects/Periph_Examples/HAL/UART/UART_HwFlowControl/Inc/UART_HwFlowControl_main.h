/**
  ******************************************************************************
  * @file    UART/UART_HwFlowControl/Inc/UART_HwFlowControl_main.h
  * @author  RF Application Team
  * @brief   Header for UART_HwFlowControl_main.c module
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
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
/* Size of Transmission buffer */
#define TXSTARTMESSAGESIZE                   (COUNTOF(aTxStartMessage) - 1)
#define TXENDMESSAGESIZE                     (COUNTOF(aTxEndMessage) - 1)

/* Size of Reception buffer */
#define RXBUFFERSIZE                       10


#ifdef STEVAL_IDB011V1
#define USARTx_INSTANCE                    USART1
#define USARTx_IRQn                        USART1_IRQn
#define USARTx_IRQHandler                  USART1_IRQHandler
  /**USART1 GPIO Configuration
  PA9/AF0   ------> TX   (FTDI Yellow)
  PA8/AF0   ------> RX   (FTDI Orange)  
  PB3/AF0   ------> CTS  (FTDI Green)
  PB2/AF0   ------> RTS  (FTDI Brown)
  */
#define USARTx_TX_PIN                      GPIO_PIN_9
#define USARTx_TX_PORT                     GPIOA
#define USARTx_TX_AF                       GPIO_AF0_USART1
#define USARTx_RX_PIN                      GPIO_PIN_8
#define USARTx_RX_PORT                     GPIOA
#define USARTx_RX_AF                       GPIO_AF0_USART1
#define EnableClock_USART()                __HAL_RCC_USART_CLK_ENABLE()
#define EnableClock_USART_TX_PORT()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define EnableClock_USART_RX_PORT()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_CTS_PIN                     GPIO_PIN_3
#define USARTx_CTS_PORT                    GPIOB
#define USARTx_CTS_AF                      GPIO_AF0_USART1
#define USARTx_RTS_PIN                     GPIO_PIN_2
#define USARTx_RTS_PORT                    GPIOB
#define USARTx_RTS_AF                      GPIO_AF0_USART1
#define EnableClock_USART_CTS_PORT()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define EnableClock_USART_RTS_PORT()       __HAL_RCC_GPIOB_CLK_ENABLE()
    
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
#define USARTx_INSTANCE                    USART1
#define USARTx_IRQn                        USART1_IRQn
#define USARTx_IRQHandler                  USART1_IRQHandler
  /**USART1 GPIO Configuration  
  PA1  AF2   ------> TX   (FTDI Yellow)
  PB0  AF0   ------> RX   (FTDI Orange)
  PB3  AF0   ------> CTS  (FTDI Green)
  PB2  AF0   ------> RTS  (FTDI Brown)
  */
#define USARTx_TX_PIN                      GPIO_PIN_1
#define USARTx_TX_PORT                     GPIOA
#define USARTx_TX_AF                       GPIO_AF2_USART1 
#define USARTx_RX_PIN                      GPIO_PIN_0
#define USARTx_RX_PORT                     GPIOB
#define USARTx_RX_AF                       GPIO_AF0_USART1 
#define EnableClock_USART()                __HAL_RCC_USART_CLK_ENABLE()
#define EnableClock_USART_TX_PORT()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define EnableClock_USART_RX_PORT()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_CTS_PIN                     GPIO_PIN_3
#define USARTx_CTS_PORT                    GPIOB
#define USARTx_CTS_AF                      GPIO_AF0_USART1
#define USARTx_RTS_PIN                     GPIO_PIN_2
#define USARTx_RTS_PORT                    GPIOB
#define USARTx_RTS_AF                      GPIO_AF0_USART1
#define EnableClock_USART_CTS_PORT()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define EnableClock_USART_RTS_PORT()       __HAL_RCC_GPIOB_CLK_ENABLE()
#endif /* STEVAL_IDB012V1 */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

