/**
  ******************************************************************************
  * @file    LL/PKA/PKA_EccScalarMul/Inc/PKA_EccScalarMul_main.h
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
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lpx.h"
#include "rf_driver_ll_pka_v7c.h"
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_bus.h"
#include "rf_driver_ll_system.h"
#include "rf_driver_ll_exti.h"
#include "rf_driver_ll_cortex.h"
#include "rf_driver_ll_utils.h"
#include "rf_driver_ll_gpio.h"

#include "bluenrg_lp_evb_config.h"

/* Private includes ----------------------------------------------------------*/
#if defined(USE_FULL_ASSERT)
#include "rf_driver_assert.h"
#endif /* USE_FULL_ASSERT */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Toggle periods for various blinking modes */
#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000


/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void PKA_ERROR_callback(void);
void PKA_PROCEND_callback(void);

/* Private defines -----------------------------------------------------------*/
/**
  * @brief BSP_LED2 
  */
#ifdef STEVAL_IDB012V1
#define LED2_PIN                                LL_GPIO_PIN_4
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#endif /* STEVAL_IDB012V1 */   

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
