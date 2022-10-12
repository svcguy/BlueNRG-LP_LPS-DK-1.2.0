/**
******************************************************************************
* @file    cca01m1_audio_ex.h
* @author  SRA - Central Labs
* @version v3.0.0
* @date    6-May-19
* @brief   This file contains definitions for cca01m1_audio_ex.c
*          firmware driver.
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
******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CCA01M1_AUDIO_EX_H
#define CCA01M1_AUDIO_EX_H

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/
   
#include <stdio.h>
#include <string.h>
#include "cca01m1_conf.h"
#include "audio.h"  
#include "sta350bw.h" 
  
/** @addtogroup BSP
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1 X_NUCLEO_CCA01M1
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1_AUDIO X_NUCLEO_CCA01M1_AUDIO
* @{
*/  


/** @defgroup  X_NUCLEO_CCA01M1_AUDIO_Exported_Functions Exported Functions 
* @{
*/
int32_t CCA01M1_AUDIO_OUT_SetEq(uint32_t Instance, uint8_t ramBlock, uint8_t filterNumber, uint32_t * filterValues);
int32_t CCA01M1_AUDIO_OUT_SetTone(uint32_t Instance, uint8_t toneGain);
int32_t CCA01M1_AUDIO_OUT_SetDSPOption(uint32_t Instance, uint8_t option, uint8_t state);

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

#ifdef __cplusplus
}
#endif

#endif /* CCA01M1_AUDIO_EX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


