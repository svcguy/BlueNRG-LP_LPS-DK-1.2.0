/**
  ******************************************************************************
  * @file    SPI/SPI_Polling_Master/Inc/SPI_Polling_Master_main.h
  * @author  RF Application Team
  * @brief   Header for SPI_Polling_Master_main.c module
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
  *******************************************************************************/

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
#if !defined( CONFIG_DATASIZE_16BIT ) & !defined( CONFIG_DATASIZE_8BIT )
  // default SPI CONFIG DATASIZE is 16 BIT
  #define CONFIG_DATASIZE_16BIT 1
#endif  

#if defined(STEVAL_IDB011V1)  
  
#if !defined( USE_SPI1_PINS ) & !defined( USE_SPI2_PINS )
  // default SPI pins for this example
  #define USE_SPI2_PINS 1
#endif  

#if defined( USE_SPI2_PINS ) /* Master SPI2 */
    /**SPI_MASTER GPIO Configuration    
    SPI2:
    PA5/AF1    ------> SPI2_SCK
    PA7/AF1    ------> SPI2_MISO
    PA12/AF3   ------> SPI2_MOSI 
    */
#define SPI_MASTER                              SPI2
#define GPIO_MASTER_SCK                         GPIOA
#define GPIO_MASTER_MISO                        GPIOA
#define GPIO_MASTER_MOSI                        GPIOA
#define GPIO_PIN_SPI_MASTER_SCK                 GPIO_PIN_5
#define GPIO_PIN_SPI_MASTER_MISO                GPIO_PIN_7
#define GPIO_PIN_SPI_MASTER_MOSI                GPIO_PIN_12
#define GPIO_AF_SPI_MASTER_SCK                  GPIO_AF1_SPI2
#define GPIO_AF_SPI_MASTER_MISO                 GPIO_AF1_SPI2
#define GPIO_AF_SPI_MASTER_MOSI                 GPIO_AF3_SPI2
#define __HAL_RCC_SPI_MASTER_CLK_ENABLE         __HAL_RCC_SPI2_CLK_ENABLE
#define __HAL_RCC_SPI_MASTER_FORCE_RESET        __HAL_RCC_SPI2_FORCE_RESET
#define __HAL_RCC_SPI_MASTER_RELEASE_RESET      __HAL_RCC_SPI2_RELEASE_RESET
#define __HAL_RCC_SPI_MASTER_CLK_DISABLE        __HAL_RCC_SPI2_CLK_DISABLE
#define SPI_MASTER_IRQn                         SPI2_IRQn
#define SPI_MASTER_IRQHandler                   SPI2_IRQHandler
#define DMA_REQUEST_SPI_MASTER_TX               DMA_REQUEST_SPI2_TX
#define DMA_REQUEST_SPI_MASTER_RX               DMA_REQUEST_SPI2_RX

#elif defined( USE_SPI1_PINS ) 
/* Master SPI1 */
    /**SPI_MASTER GPIO Configuration    
    PA13/AF1     ------> SPI_MASTER_SCK
    PA14/AF1     ------> SPI_MASTER_MISO
    PB14/AF0     ------> SPI_MASTER_MOSI 
    */
#define SPI_MASTER                              SPI1
#define GPIO_MASTER_SCK                         GPIOA
#define GPIO_MASTER_MISO                        GPIOA
#define GPIO_MASTER_MOSI                        GPIOB
#define GPIO_AF_SPI_MASTER_SCK                  GPIO_AF2_SPI1
#define GPIO_AF_SPI_MASTER_MISO                 GPIO_AF2_SPI1
#define GPIO_AF_SPI_MASTER_MOSI                 GPIO_AF0_SPI1
#define GPIO_PIN_SPI_MASTER_SCK                 GPIO_PIN_13
#define GPIO_PIN_SPI_MASTER_MISO                GPIO_PIN_14
#define GPIO_PIN_SPI_MASTER_MOSI                GPIO_PIN_14
#define __HAL_RCC_SPI_MASTER_CLK_ENABLE         __HAL_RCC_SPI1_CLK_ENABLE
#define __HAL_RCC_SPI_MASTER_FORCE_RESET        __HAL_RCC_SPI1_FORCE_RESET
#define __HAL_RCC_SPI_MASTER_RELEASE_RESET      __HAL_RCC_SPI1_RELEASE_RESET
#define __HAL_RCC_SPI_MASTER_CLK_DISABLE        __HAL_RCC_SPI1_CLK_DISABLE
#define SPI_MASTER_IRQn                         SPI1_IRQn
#define SPI_MASTER_IRQHandler                   SPI1_IRQHandler
#define DMA_REQUEST_SPI_MASTER_TX               DMA_REQUEST_SPI1_TX
#define DMA_REQUEST_SPI_MASTER_RX               DMA_REQUEST_SPI1_RX
#endif

#endif  /* defined(STEVAL_IDB011V1)  */


#if defined(STEVAL_IDB012V1) 
  
/**SPI_MASTER GPIO Configuration    
  PB3/AF4    ------> SPI3_SCK
  PA8/AF3    ------> SPI3_MISO
  PA11/AF3   ------> SPI3_MOSI 
*/
#define SPI_MASTER                              SPI3
#define GPIO_MASTER_SCK                         GPIOB
#define GPIO_MASTER_MISO                        GPIOA
#define GPIO_MASTER_MOSI                        GPIOA
#define GPIO_AF_SPI_MASTER_SCK                  GPIO_AF4_SPI3
#define GPIO_AF_SPI_MASTER_MISO                 GPIO_AF3_SPI3
#define GPIO_AF_SPI_MASTER_MOSI                 GPIO_AF3_SPI3
#define GPIO_PIN_SPI_MASTER_SCK                 GPIO_PIN_3
#define GPIO_PIN_SPI_MASTER_MISO                GPIO_PIN_8
#define GPIO_PIN_SPI_MASTER_MOSI                GPIO_PIN_11
#define __HAL_RCC_SPI_MASTER_CLK_ENABLE         __HAL_RCC_SPI3_CLK_ENABLE
#define __HAL_RCC_SPI_MASTER_FORCE_RESET        __HAL_RCC_SPI3_FORCE_RESET
#define __HAL_RCC_SPI_MASTER_RELEASE_RESET      __HAL_RCC_SPI3_RELEASE_RESET
#define __HAL_RCC_SPI_MASTER_CLK_DISABLE        __HAL_RCC_SPI3_CLK_DISABLE
#define SPI_MASTER_IRQn                         SPI3_IRQn
#define SPI_MASTER_IRQHandler                   SPI3_IRQHandler
#define DMA_REQUEST_SPI_MASTER_TX               DMA_REQUEST_SPI3_TX
#define DMA_REQUEST_SPI_MASTER_RX               DMA_REQUEST_SPI3_RX

#endif  /* defined(STEVAL_IDB012V1)  */




/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


