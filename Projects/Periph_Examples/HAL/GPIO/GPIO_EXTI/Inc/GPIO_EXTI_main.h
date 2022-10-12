/**
  ******************************************************************************
  * @file    GPIO/GPIO_EXTI/Inc/GPIO_EXTI_main.h
  * @author  RF Application Team
  * @brief   Header for GPIO_EXTI_main.c module
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
void Example_EXTI_Callback(uint32_t Line);

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#if defined(STEVAL_IDB011V1) || defined(STEVAL_IDB012V1) 
#define USER_BUTTON_PIN                         GPIO_PIN_10
#define USER_BUTTON_GPIO_PORT                   GPIOA
#define USER_BUTTON_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE() 
#define USER_BUTTON_SYSCFG_CLK_ENABLE()         __HAL_RCC_SYSCFG_CLK_ENABLE()   
#define USER_BUTTON_EXTI_LINE                   LL_EXTI_LINE_PA10
#define USER_BUTTON_EXTI_IRQn                   GPIOA_IRQn
#define USER_BUTTON_EXTI_LINE_ENABLE()          LL_EXTI_EnableIT(USER_BUTTON_EXTI_LINE)   
#define USER_BUTTON_EXTI_RISING_TRIG_ENABLE()   LL_EXTI_SetTrigger(LL_EXTI_TRIGGER_RISING_EDGE, USER_BUTTON_EXTI_LINE)   
#define USER_BUTTON_IRQHANDLER                  GPIOA_IRQHandler
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


