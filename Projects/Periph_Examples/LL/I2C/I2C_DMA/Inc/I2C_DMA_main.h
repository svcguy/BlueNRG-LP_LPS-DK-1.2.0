/**
  ******************************************************************************
  * @file    LL/I2C/I2C_DMA/Inc/I2C_DMA_main.h
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
#include "rf_driver_ll_i2c.h"
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_bus.h"
#include "rf_driver_ll_system.h"
#include "rf_driver_ll_exti.h"
#include "rf_driver_ll_cortex.h"
#include "rf_driver_ll_utils.h"
#include "rf_driver_ll_pwr.h"
#include "rf_driver_ll_dma.h"
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
/* Define used to enable time-out management*/
#define USE_TIMEOUT       0

#define I2C_SEND_TIMEOUT_STOP_MS        10000
  
/**
  * @brief Toggle periods for various blinking modes
  */
#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

/**
  * @brief Slave settings
  */
#define SLAVE_OWN_ADDRESS                       180 /* This value is a left shift of a real 7 bits of a slave address
                                                        value which can find in a Datasheet as example: b0101101
                                                        mean in uint8_t equivalent at 0x2D and this value can be
                                                        seen in the OAR1 register in bits OA1[1:7] */

/** Uncomment this line to use the board as slave, if not it is used as master */
//#define SLAVE_BOARD 

  

#if defined(STEVAL_IDB011V1)
#define I2Cx                          I2C2
#define I2Cx_SCL_PORT                 GPIOA   
#define I2Cx_SCL_PIN                  LL_GPIO_PIN_13
#define I2Cx_SCL_AF                   LL_GPIO_AF_0  
#define I2Cx_SDA_PORT                 GPIOA
#define I2Cx_SDA_PIN                  LL_GPIO_PIN_14 
#define I2Cx_SDA_AF                   LL_GPIO_AF_0
#define I2Cx_IRQn                     I2C2_IRQn
#define I2Cx_IRQHandler               I2C2_IRQHandler
#define LL_I2Cx_SCL_EnableClock()     LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA)
#define LL_I2Cx_SDA_EnableClock()     LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA)
#define LL_DMAMUX_REQ_I2Cx_RX         LL_DMAMUX_REQ_I2C2_RX
#define LL_DMAMUX_REQ_I2Cx_TX         LL_DMAMUX_REQ_I2C2_TX
#define LL_I2Cx_EnableClock()         LL_APB1_EnableClock(LL_APB1_PERIPH_I2C2)
#endif /* STEVAL_IDB011V1 */


#if defined(STEVAL_IDB012V1)
#define I2Cx                          I2C1
#define I2Cx_SCL_PORT                 GPIOB
#define I2Cx_SCL_PIN                  LL_GPIO_PIN_6
#define I2Cx_SCL_AF                   LL_GPIO_AF_0
#define I2Cx_SDA_PORT                 GPIOB
#define I2Cx_SDA_PIN                  LL_GPIO_PIN_7
#define I2Cx_SDA_AF                   LL_GPIO_AF_0
#define I2Cx_IRQn                     I2C1_IRQn
#define I2Cx_IRQHandler               I2C1_IRQHandler
#define LL_I2Cx_SCL_EnableClock()     LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#define LL_I2Cx_SDA_EnableClock()     LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#define LL_DMAMUX_REQ_I2Cx_RX	      LL_DMAMUX_REQ_I2C1_RX
#define LL_DMAMUX_REQ_I2Cx_TX         LL_DMAMUX_REQ_I2C1_TX
#define LL_I2Cx_EnableClock()         LL_APB1_EnableClock(LL_APB1_PERIPH_I2C1)
#endif /* STEVAL_IDB012V1 */ 

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* IRQ Handler treatment functions */
void UserButton_Callback(void);
#ifdef SLAVE_BOARD
void DMA1_Transfer_Complete_Callback(void);
void DMA1_Transfer_Error_Callback(void);
#else /* MASTER_BOARD */
void DMA1_Transfer_Complete_Callback(void);
void DMA1_Transfer_Error_Callback(void);
#endif /* SLAVE_BOARD */
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


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


