/**
 ******************************************************************************
 * @file    cca01m1_conf.h
 * @author  SRA- Central Labs
 * @version v3.0.0
 * @date    6-May-19
 * @brief   This file contains definitions for the SOUNDTER1 applications
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
#ifndef CCA01M1_CONF_H__
#define CCA01M1_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/  

#define AUDIO_OUT_CHANNELS 		        1
#define AUDIO_OUT_SAMPLING_FREQUENCY 	        32000

#define AUDIO_OUT_BUFFER_SIZE                   512  
#define AUDIO_VOLUME_OUT	                0x10

#define CCA01M1_AUDIO_OUT_INSTANCE		1U 
#define CCA01M1_AUDIO_OUT_IT_PRIORITY           6U

#ifdef __cplusplus
}
#endif

#endif /* CCA01M1_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



