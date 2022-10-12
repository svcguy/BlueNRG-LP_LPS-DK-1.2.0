/**
  ******************************************************************************
  * @file    I2C/I2C_Polling/Inc/I2C_Polling_main.h 
  * @author  RF Application Team
  * @brief   Header for I2C_Polling_main.c module
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
#define I2C_ADDRESS 0x30F

/** Uncomment this line to use the board as master, if not it is used as slave */
//#define MASTER_BOARD

#if defined(STEVAL_IDB011V1) 
#define I2Cx                                            I2C2
#define I2Cx_SCL_PORT                                   GPIOA   
#define I2Cx_SCL_PIN                                    GPIO_PIN_13
#define I2Cx_SCL_AF                                     GPIO_AF0_I2C2  
#define I2Cx_SDA_PORT                                   GPIOA
#define I2Cx_SDA_PIN                                    GPIO_PIN_14 
#define I2Cx_SDA_AF                                     GPIO_AF0_I2C2
#define __HAL_RCC_I2Cx_CLK_ENABLE                       __HAL_RCC_I2C2_CLK_ENABLE
#define __HAL_RCC_I2Cx_CLK_DISABLE                      __HAL_RCC_I2C2_CLK_DISABLE
#define __HAL_RCC_I2Cx_SCL_GPIO_CLK_ENABLE              __HAL_RCC_GPIOA_CLK_ENABLE
#define __HAL_RCC_I2Cx_SDA_GPIO_CLK_ENABLE              __HAL_RCC_GPIOA_CLK_ENABLE
#define I2Cx_IRQn                                       I2C2_IRQn
#define I2Cx_IRQHandler                                 I2C2_IRQHandler
#define DMA_REQUEST_I2Cx_TX                             DMA_REQUEST_I2C2_TX
#define DMA_REQUEST_I2Cx_RX                             DMA_REQUEST_I2C2_RX
#endif /* STEVAL_IDB011V1 */

#if defined(STEVAL_IDB012V1) 
#define I2Cx                                            I2C1
#define I2Cx_SCL_PORT                                   GPIOB   
#define I2Cx_SCL_PIN                                    GPIO_PIN_6
#define I2Cx_SCL_AF                                     GPIO_AF0_I2C1  
#define I2Cx_SDA_PORT                                   GPIOB
#define I2Cx_SDA_PIN                                    GPIO_PIN_7 
#define I2Cx_SDA_AF                                     GPIO_AF0_I2C1
#define __HAL_RCC_I2Cx_CLK_ENABLE                       __HAL_RCC_I2C1_CLK_ENABLE
#define __HAL_RCC_I2Cx_CLK_DISABLE                      __HAL_RCC_I2C1_CLK_DISABLE
#define __HAL_RCC_I2Cx_SCL_GPIO_CLK_ENABLE              __HAL_RCC_GPIOB_CLK_ENABLE
#define __HAL_RCC_I2Cx_SDA_GPIO_CLK_ENABLE              __HAL_RCC_GPIOB_CLK_ENABLE
#define I2Cx_IRQn                                       I2C1_IRQn
#define I2Cx_IRQHandler                                 I2C1_IRQHandler
#define DMA_REQUEST_I2Cx_TX                             DMA_REQUEST_I2C1_TX
#define DMA_REQUEST_I2Cx_RX                             DMA_REQUEST_I2C1_RX
#endif /* STEVAL_IDB012V1 */




/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


