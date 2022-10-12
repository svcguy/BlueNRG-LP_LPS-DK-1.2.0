/**
  ******************************************************************************
  * @file    Cortex/CORTEXM_ModeP/Inc/CORTEXM_ModeP_main.h
  * @author  RF Application Team
  * @brief   Header for CORTEXM_ModeP_main.c module
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
#define SP_PROCESS                  0x02   /* Process stack */
#define SP_MAIN                     0x00   /* Main stack */
#define THREAD_MODE_PRIVILEGED      0x00   /* Thread mode has privileged access */
#define THREAD_MODE_UNPRIVILEGED    0x01   /* Thread mode has unprivileged access */

/* Exported macro ------------------------------------------------------------*/

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


