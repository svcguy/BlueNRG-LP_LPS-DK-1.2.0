/**
  ******************************************************************************
  * @file    I2S/I2S_CCA01M1_ComIT/Inc/I2S_CCA01M1_ComIT_main.h
  * @author  RF Application Team
  * @brief   Header for I2S_CCA01M1_ComIT_main.c module
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
#include "audio_application.h"
#include "cca01m1_audio.h"
  
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
  
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void Example_EXTI_Callback(uint32_t Line);

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) + 1)
#define LED_APPLICATION                  BSP_LED1

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


