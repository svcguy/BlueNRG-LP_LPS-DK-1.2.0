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
  * @brief Toggle periods for various blinking modes
  */

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

/**
  * @brief Key push-button
  */
#if defined(STEVAL_IDB011V1) || defined(STEVAL_IDB012V1) 
#define USER_BUTTON_PIN                         LL_GPIO_PIN_10
#define USER_BUTTON_GPIO_PORT                   GPIOA
#define USER_BUTTON_GPIO_CLK_ENABLE()           LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA) 
#define USER_BUTTON_SYSCFG_CLK_ENABLE()         LL_APB0_EnableClock(LL_APB0_PERIPH_SYSCFG)   
#define USER_BUTTON_EXTI_LINE                   LL_EXTI_LINE_PA10
#define USER_BUTTON_EXTI_IRQn                   GPIOA_IRQn
#define USER_BUTTON_EXTI_LINE_ENABLE()          LL_EXTI_EnableIT(USER_BUTTON_EXTI_LINE)   
#define USER_BUTTON_EXTI_RISING_TRIG_ENABLE()   LL_EXTI_SetTrigger(LL_EXTI_TRIGGER_RISING_EDGE, USER_BUTTON_EXTI_LINE)   
#endif


/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void DMA1_TransmitComplete_Callback(void);
void DMA1_ReceiveComplete_Callback(void);
void USART_TransferError_Callback(void);
void UserButton_Callback(void);
void USART_CharReception_Callback(void);

/* Private defines -----------------------------------------------------------*/




#ifdef STEVAL_IDB011V1
  /**USART1 GPIO Configuration  
  PB8  AF0   ------> USART1_CK    SPI SCK
  PA9  AF0   ------> USART1_TX    SPI MOSI 
  PA8  AF0   ------> USART1_RX    SPI MISO
  PB3  AF0   ------> USART1_NSS   SPI CS
  */
#define USART1_CK_PIN                           LL_GPIO_PIN_8
#define USART1_CK_PORT                          GPIOB
#define USART1_CK_AF                            LL_GPIO_AF_0
#define USART1_TX_PIN                           LL_GPIO_PIN_15
#define USART1_TX_PORT                          GPIOB
#define USART1_TX_AF                            LL_GPIO_AF_6
#define USART1_RX_PIN                           LL_GPIO_PIN_14
#define USART1_RX_PORT                          GPIOB
#define USART1_RX_AF                            LL_GPIO_AF_0
#define USART1_NSS_PIN                          LL_GPIO_PIN_3
#define USART1_NSS_PORT                         GPIOB
#define USART1_NSS_AF                           LL_GPIO_AF_0
#define LL_EnableClock_USART()                  LL_APB1_EnableClock(LL_APB1_PERIPH_USART)
  /**SPI_MASTER GPIO Configuration
  PA1  SPI_MASTER_CS
  */ 
#define GPIO_PORT_MASTER_CS                     GPIOA
#define GPIO_PIN_SPI_MASTER_CS                  LL_GPIO_PIN_1
#define LL_PWR_EnablePDA_USART_TX()             LL_PWR_EnablePDA(LL_PWR_PUPD_IO8);
#define LL_PWR_EnablePDA_USART_RX()             LL_PWR_EnablePDA(LL_PWR_PUPD_IO9);
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
  /**USART1 GPIO Configuration 
  PB1  AF0   ------> USART1_CK    SPI SCK 
  PA1  AF2   ------> USART1_TX    SPI MISO
  PB0  AF0   ------> USART1_RX    SPI MOSI
  PB7  AF2   ------> USART1_NSS   SPI CS
  */
#define USART1_CK_PIN                           LL_GPIO_PIN_1
#define USART1_CK_PORT                          GPIOB
#define USART1_CK_AF                            LL_GPIO_AF_0
#define USART1_TX_PIN                           LL_GPIO_PIN_1
#define USART1_TX_PORT                          GPIOA
#define USART1_TX_AF                            LL_GPIO_AF_2
#define USART1_RX_AF                            LL_GPIO_AF_0
#define USART1_RX_PIN                           LL_GPIO_PIN_0
#define USART1_RX_PORT                          GPIOB
#define USART1_NSS_PIN                          LL_GPIO_PIN_7
#define USART1_NSS_PORT                         GPIOB
#define USART1_NSS_AF                           LL_GPIO_AF_2
#define LL_EnableClock_USART()                  LL_APB1_EnableClock(LL_APB1_PERIPH_USART)

#define LL_PWR_EnablePDA_USART_TX()             LL_PWR_EnablePDA(LL_PWR_PUPD_IO1);
#define LL_PWR_EnablePDA_USART_RX()             LL_PWR_EnablePDB(LL_PWR_PUPD_IO0);
#endif /* STEVAL_IDB012V1 */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


