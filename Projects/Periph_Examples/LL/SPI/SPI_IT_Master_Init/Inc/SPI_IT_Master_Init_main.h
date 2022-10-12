/**
  ******************************************************************************
  * @file    LL/SPI/SPI_IT_Master_Init/Inc/SPI_IT_Master_Init_main.h
  * @author  RF Application Team
  * @brief   Header for SPI_IT_Master_Init_main.c module
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
#include "rf_driver_ll_spi.h"
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

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

void SPI_MASTER_Rx_Callback(void);
void SPI_MASTER_Tx_Callback(void);
void SPI_MASTER_TransferError_Callback(void);

/* Private defines -----------------------------------------------------------*/
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
#define USER_BUTTON_SETTED 0UL

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




// Mandatory to test the USART SPI Slave demo
//   #define SPI_MASTER_PULL_UP_V_2  1
//   #define CONFIG_DATASIZE_8BIT    1
//   #define USE_SPI1_PINS           1

#if !defined( CONFIG_DATASIZE_16BIT ) & !defined( CONFIG_DATASIZE_8BIT )
  // default SPI CONFIG DATASIZE is 16 BIT
  #define CONFIG_DATASIZE_16BIT 1
#endif 

#ifdef STEVAL_IDB011V1

#if !defined( USE_SPI1_PINS ) & !defined( USE_SPI2_PINS )
  // default SPI pins for this example
  #define USE_SPI1_PINS 1
#endif  
  
#if defined( USE_SPI2_PINS ) /* Master SPI2 */
    /**SPI_MASTER GPIO Configuration    
    SPI2:
    PA5/AF1    ------> SPI2_SCK
    PA7/AF1    ------> SPI2_MISO
    PA12/AF3   ------> SPI2_MOSI 
    PA6        ------> SPI2_CS
    */
#define GPIO_PORT_MASTER_SCK                    GPIOA
#define GPIO_PORT_MASTER_MISO                   GPIOA
#define GPIO_PORT_MASTER_MOSI                   GPIOA
#define GPIO_PORT_MASTER_CS                     GPIOA
#define GPIO_PIN_SPI_MASTER_SCK                 LL_GPIO_PIN_5
#define GPIO_PIN_SPI_MASTER_MISO                LL_GPIO_PIN_7
#define GPIO_PIN_SPI_MASTER_MOSI                LL_GPIO_PIN_12
#define GPIO_PIN_SPI_MASTER_CS                  LL_GPIO_PIN_6
#define GPIO_AF_SPI_MASTER_SCK                  LL_GPIO_AF_1
#define GPIO_AF_SPI_MASTER_MISO                 LL_GPIO_AF_1
#define GPIO_AF_SPI_MASTER_MOSI                 LL_GPIO_AF_3
#define SPI_MASTER                              SPI2
#define LL_SPI_Master_EnableClock()             LL_APB1_EnableClock(LL_APB1_PERIPH_SPI2)
#define SPI_MASTER_IRQn                         SPI2_IRQn
#define SPI_MASTER_IRQHandler                   SPI2_IRQHandler
#elif defined( USE_SPI1_PINS ) /* Master SPI1 */
  /**SPI_MASTER GPIO Configuration  
  PA13   ------> SPI_MASTER_SCK
  PA14   ------> SPI_MASTER_MISO
  PB14   ------> SPI_MASTER_MOSI 
  PA11   ------> SPI_MASTER_CS
  */ 
#define GPIO_PORT_MASTER_SCK                    GPIOA
#define GPIO_PORT_MASTER_MISO                   GPIOA
#define GPIO_PORT_MASTER_MOSI                   GPIOB
#define GPIO_PORT_MASTER_CS                     GPIOA
#define GPIO_PIN_SPI_MASTER_SCK                 LL_GPIO_PIN_13
#define GPIO_PIN_SPI_MASTER_MISO                LL_GPIO_PIN_14
#define GPIO_PIN_SPI_MASTER_MOSI                LL_GPIO_PIN_14
#define GPIO_PIN_SPI_MASTER_CS                  LL_GPIO_PIN_11
#define GPIO_AF_SPI_MASTER_SCK                  LL_GPIO_AF_2
#define GPIO_AF_SPI_MASTER_MISO                 LL_GPIO_AF_2
#define GPIO_AF_SPI_MASTER_MOSI                 LL_GPIO_AF_0
#define SPI_MASTER                              SPI1
#define LL_SPI_Master_EnableClock()             LL_APB1_EnableClock(LL_APB1_PERIPH_SPI1)
#define SPI_MASTER_IRQn                         SPI1_IRQn
#define SPI_MASTER_IRQHandler                   SPI1_IRQHandler
#endif

#endif /* STEVAL_IDB011V1 */


#ifdef STEVAL_IDB012V1
    /** GPIO Configuration
    PB3/AF4    ------> SPI3_SCK
    PA8/AF3    ------> SPI3_MISO
    PA11/AF3   ------> SPI3_MOSI 
    PA9        ------> SPI3_CS 
    */
#define GPIO_PORT_MASTER_SCK                    GPIOB
#define GPIO_PORT_MASTER_MISO                   GPIOA
#define GPIO_PORT_MASTER_MOSI                   GPIOA
#define GPIO_PORT_MASTER_CS                     GPIOA
#define GPIO_PIN_SPI_MASTER_SCK                 LL_GPIO_PIN_3
#define GPIO_PIN_SPI_MASTER_MISO                LL_GPIO_PIN_8
#define GPIO_PIN_SPI_MASTER_MOSI                LL_GPIO_PIN_11
#define GPIO_PIN_SPI_MASTER_CS                  LL_GPIO_PIN_9
#define GPIO_AF_SPI_MASTER_SCK                  LL_GPIO_AF_4
#define GPIO_AF_SPI_MASTER_MISO                 LL_GPIO_AF_3
#define GPIO_AF_SPI_MASTER_MOSI                 LL_GPIO_AF_3
#define SPI_MASTER                              SPI3
#define LL_SPI_Master_EnableClock()             LL_APB1_EnableClock(LL_APB1_PERIPH_SPI3)
#define SPI_MASTER_IRQn                         SPI3_IRQn
#define SPI_MASTER_IRQHandler                   SPI3_IRQHandler
#define LL_DMAMUX_REQ_SPI_MASTER_TX             LL_DMAMUX_REQ_SPI3_TX
#define LL_DMAMUX_REQ_SPI_MASTER_RX             LL_DMAMUX_REQ_SPI3_RX

#endif /* STEVAL_IDB012V1 */



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


