/**
  ******************************************************************************
  * @file    I2S/I2S_Transmitter/Inc/I2S_Transmitter_main.h
  * @author  RF Application Team
  * @brief   Header for I2S_Transmitter_main.c module
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
  
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) + 1)

#if defined(STEVAL_IDB011V1) 
  
 /* I2S peripheral configuration defines */
#define I2S_INSTANCE                 SPI2
#define SPIx_IRQHandler                         SPI2_IRQHandler
#define I2S_CLK_ENABLE()             __HAL_RCC_SPI2_CLK_ENABLE()
#define I2S_CLK_CONFIG()             __HAL_RCC_SPI2I2S_CONFIG(RCC_SPI2I2S_CLKSOURCE_16M)
#define I2S_CLK_DISABLE()            __HAL_RCC_SPI2_CLK_DISABLE()


#define I2S_SCK_AF         	        GPIO_AF1_SPI2
#define I2S_WS_AF         	        GPIO_AF1_SPI2
#define I2S_SD_AF         	        GPIO_AF3_SPI2
#define I2S_MCK_AF                      GPIO_AF2_SPI2

#define PERIPHCLOCK_I2S RCC_PERIPHCLK_SPI2_I2S

#define I2S_SD_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2S_SCK_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2S_MCK_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2S_WS_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2S_SCK_PIN                  GPIO_PIN_5   //   PA5/AF1   ------> I2S_CK  (SPI2_SCK)                 
#define I2S_WS_PIN                   GPIO_PIN_4   //   PA4/AF1   ------> I2S_WS  (SPI2_NSS)
#define I2S_SD_PIN                   GPIO_PIN_12   //   PA12/AF3   ------> I2S_SD  (SPI2_MISO)
#define I2S_MCK_PIN                  GPIO_PIN_9   //   PB9/AF2   ------> I2S_MCK (SPI2_MCK)


#define I2S_SCK_GPIO_PORT            GPIOA
#define I2S_WS_GPIO_PORT             GPIOA
#define I2S_SD_GPIO_PORT             GPIOA
#define I2S_MCK_GPIO_PORT            GPIOB   

  /* I2S DMA Stream Tx definitions */
#define I2S_DMAx_CLK_ENABLE()        __HAL_RCC_DMA_CLK_ENABLE()
#define I2S_DMAx_PERIPH_DATA_SIZE    DMA_PDATAALIGN_HALFWORD
#define I2S_DMAx_MEM_DATA_SIZE       DMA_MDATAALIGN_HALFWORD 
#define I2S_DMAx_REQUEST_SPIx_TX     DMA_REQUEST_SPI2_TX
#define I2S_DMAx_STREAM              DMA1_Channel1 
#define I2S_DMAx_IRQ                 DMA_IRQn 
#define IRQHandler                   DMA_IRQHandler


#endif  /* defined(STEVAL_IDB011V1) */
  
#if defined(STEVAL_IDB012V1) 

 /* I2S peripheral configuration defines */
#define I2S_INSTANCE                 SPI3
#define SPIx_IRQHandler                         SPI3_IRQHandler
#define I2S_CLK_ENABLE()             __HAL_RCC_SPI3_CLK_ENABLE()
#define I2S_CLK_CONFIG()             __HAL_RCC_SPI3I2S_CONFIG(RCC_SPI3I2S_CLKSOURCE_16M)
#define I2S_CLK_DISABLE()            __HAL_RCC_SPI3_CLK_DISABLE()


#define I2S_SCK_AF         	        GPIO_AF4_SPI3
#define I2S_WS_AF         	        GPIO_AF3_SPI3
#define I2S_SD_AF                    GPIO_AF3_SPI3
#define I2S_MCK_AF                   GPIO_AF3_SPI3

#define PERIPHCLOCK_I2S RCC_PERIPHCLK_SPI3_I2S

#define I2S_SD_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2S_SCK_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2S_MCK_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2S_WS_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2S_SCK_PIN                  GPIO_PIN_3   //   PB3/AF4   ------> I2S_CK  (SPI3_SCK)                 
#define I2S_WS_PIN                   GPIO_PIN_9   //   PA9/AF3   ------> I2S_WS  (SPI3_NSS)
#define I2S_SD_PIN                   GPIO_PIN_8   //   PA8/AF3   ------> I2S_SD  (SPI3_MISO)
#define I2S_MCK_PIN                  GPIO_PIN_10  //   PA10/AF3  ------> I2S_MCK (SPI3_MCK)


#define I2S_SCK_GPIO_PORT            GPIOB
#define I2S_WS_GPIO_PORT             GPIOA
#define I2S_SD_GPIO_PORT             GPIOA
#define I2S_MCK_GPIO_PORT            GPIOA   

  /* I2S DMA Stream Tx definitions */
#define I2S_DMAx_CLK_ENABLE()        __HAL_RCC_DMA_CLK_ENABLE()
#define I2S_DMAx_PERIPH_DATA_SIZE    DMA_PDATAALIGN_HALFWORD
#define I2S_DMAx_MEM_DATA_SIZE       DMA_MDATAALIGN_HALFWORD 
#define I2S_DMAx_REQUEST_SPIx_TX     DMA_REQUEST_SPI3_TX
#define I2S_DMAx_STREAM              DMA1_Channel1 
#define I2S_DMAx_IRQ                 DMA_IRQn 
#define IRQHandler                   DMA_IRQHandler

#endif  /* defined(STEVAL_IDB012V1) */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


