/**
  ******************************************************************************
  * @file    audio_application.h 
  * @author  SRA - Central Labs
  * @version v3.0.0
  * @date    6-May-19
  * @brief   Header for audio_application.c module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIO_APPLICATION_H
#define __AUDIO_APPLICATION_H

/* Includes ------------------------------------------------------------------*/
#include "I2S_CCA01M1_ComIT_main.h"
#include "Fragment1.h"
#include "stdlib.h"


/** @addtogroup X_CUBE_SOUNDTER1_Applications
* @{
*/ 

/** @addtogroup Audio_Streaming
* @{
*/

/** @defgroup AUDIO_APPLICATION 
* @{
*/


/** @defgroup AUDIO_APPLICATION_Private_Types 
* @{
*/  

/**
* @}
*/ 


/** @defgroup AUDIO_APPLICATION_Exported_Defines 
* @{
*/
#define FILTER_NB 2
/**
* @}
*/


/** @defgroup AUDIO_APPLICATION_Exported_Functions_Prototypes 
* @{
*/
uint32_t Init_Biquads_Filter(void);
uint32_t Init_AudioOut_Device(void);
uint32_t Start_AudioOut_Device(void);
uint32_t Stop_AudioOut_Device(void);
uint32_t Switch_Demo(void);


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



#endif /* __AUDIO_APPLICATION_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


