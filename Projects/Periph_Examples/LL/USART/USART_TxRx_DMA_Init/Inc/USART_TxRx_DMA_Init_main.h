/**
  ******************************************************************************
  * @file    LL/USART/USART_Comm_TxRx_DMA_Init/Inc/USART_Comm_TxRx_DMA_Init_main.h
  * @author  RF Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "rf_driver_ll_dma.h"
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_bus.h"
#include "rf_driver_ll_system.h"
#include "rf_driver_ll_exti.h"
#include "rf_driver_ll_cortex.h"
#include "rf_driver_ll_utils.h"
#include "rf_driver_ll_pwr.h"
#include "rf_driver_ll_usart.h"
#include "rf_driver_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "rf_driver_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/**
  * @brief Toggle periods for various blinking modes
  */

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void DMA1_TransmitComplete_Callback(void);
void DMA1_ReceiveComplete_Callback(void);
void USART_TransferError_Callback(void);


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




#ifdef STEVAL_IDB011V1
  /**USART1 GPIO Configuration  
  PA9  AF0   ------> USART1_TX    
  PA8  AF0   ------> USART1_RX   
  */
#define USART1_TX_PIN                           LL_GPIO_PIN_9
#define USART1_TX_PORT                          GPIOA
#define USART1_TX_AF                            LL_GPIO_AF_0
#define USART1_RX_PIN                           LL_GPIO_PIN_8
#define USART1_RX_PORT                          GPIOA
#define USART1_RX_AF                            LL_GPIO_AF_0 
#define LL_EnableClock_USART()                  LL_APB1_EnableClock(LL_APB1_PERIPH_USART)
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
  /**USART1 GPIO Configuration  
  PA1  AF2   ------> USART1_TX    
  PB0  AF0   ------> USART1_RX   
  */
#define USART1_TX_PIN                           LL_GPIO_PIN_1
#define USART1_TX_PORT                          GPIOA
#define USART1_TX_AF                            LL_GPIO_AF_2
#define USART1_RX_PIN                           LL_GPIO_PIN_0
#define USART1_RX_PORT                          GPIOB
#define USART1_RX_AF                            LL_GPIO_AF_0 
#define LL_EnableClock_USART()                  LL_APB1_EnableClock(LL_APB1_PERIPH_USART)
#endif /* STEVAL_IDB012V1 */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

