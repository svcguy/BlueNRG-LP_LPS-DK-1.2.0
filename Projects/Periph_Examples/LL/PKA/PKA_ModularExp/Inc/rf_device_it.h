/**
  ******************************************************************************
  * @file    LL/PKA/PKA_ModularExp/Inc/rf_device_it.h
  * @author  RF Application Team
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __RF_DEVICE_IT_H
#define __RF_DEVICE_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void HardFault_IRQHandler(void);
void SysTick_IRQHandler(void);
void PKA_ERROR_callback(void);
void PKA_PROCEND_callback(void);
void PKA_IRQHandler(void);


#ifdef __cplusplus
}
#endif

#endif /* __RF_DEVICE_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
