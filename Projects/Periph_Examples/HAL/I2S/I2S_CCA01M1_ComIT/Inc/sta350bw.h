/**
******************************************************************************
* @file    sta350bw.h
* @author  SRA - Central Labs
* @version v3.0.0
* @date    6-May-19
* @brief   This file contains definitions for sta350bw.c
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
#ifndef STA350BW_H
#define STA350BW_H

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "I2S_CCA01M1_ComIT_main.h"
#include "sta350bw_reg.h"
#include <string.h>



extern I2C_HandleTypeDef hi2c1;


 
/** @addtogroup BSP
* @{
*/ 

/** @addtogroup Components
* @{
*/ 

/** @addtogroup STA350BW
* @{
*/

/** @defgroup STA350BW_Exported_Types
* @{
*/
 
typedef HAL_StatusTypeDef (*STA350BW_Init_Func)(I2C_HandleTypeDef *);
typedef HAL_StatusTypeDef (*STA350BW_DeInit_Func)(I2C_HandleTypeDef *);
typedef uint32_t (*STA350BW_GetTick_Func)(void);
typedef HAL_StatusTypeDef (*STA350BW_WriteReg_Func)(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *pData, uint16_t Size, uint32_t Timeout);
typedef HAL_StatusTypeDef (*STA350BW_ReadReg_Func)(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);

typedef struct
{
  STA350BW_Init_Func          Init;
  STA350BW_DeInit_Func        DeInit;
  uint32_t                    BusType; /*0 means I2C, 1 means SPI-3-Wires */
  uint8_t                     Address;
  STA350BW_WriteReg_Func      WriteReg;
  STA350BW_ReadReg_Func       ReadReg;
  STA350BW_GetTick_Func       GetTick;
} STA350BW_IO_t;

typedef struct
{
  STA350BW_IO_t        IO;
  STA350BW_ctx_t       Ctx;
  uint8_t            is_initialized;
  uint8_t            audio_is_enabled;
} STA350BW_Object_t;

typedef struct
{
  int32_t (*Init)(STA350BW_Object_t *, void *);
  int32_t (*DeInit)(STA350BW_Object_t *);
  int32_t (*ReadID)(STA350BW_Object_t *, uint32_t *);
  int32_t (*Play)(STA350BW_Object_t *, uint16_t*, uint16_t);
  int32_t (*Pause)(STA350BW_Object_t *);
  int32_t (*Resume)(STA350BW_Object_t *);
  int32_t (*Stop)(STA350BW_Object_t *, uint32_t);
  int32_t (*SetFrequency)(STA350BW_Object_t *, uint32_t);
  int32_t (*GetFrequency   )(void*);  
  int32_t (*SetVolume)(STA350BW_Object_t *, uint32_t, uint8_t);
  int32_t (*GetVolume      )(void*, uint32_t, uint8_t*);
  int32_t (*SetMute)(STA350BW_Object_t *, uint32_t);  
  int32_t (*SetOutputMode)(STA350BW_Object_t *, uint8_t);
  int32_t (*SetResolution  )(void*, uint32_t);
  int32_t (*GetResolution  )(void*, uint32_t*);  
  int32_t (*SetProtocol    )(void*, uint32_t);
  int32_t (*GetProtocol    )(void*, uint32_t*);
  int32_t (*Reset)(STA350BW_Object_t *);
} STA350BW_AUDIO_Drv_t;


/**
 * @}
 */
  
/** @defgroup STA350BW_Exported_Constants
* @{
*/
  
#define STA350BW_I2C_BUS           0U

/** STA350BW error codes  **/
#define STA350BW_OK                 0
#define STA350BW_ERROR             -1

/**
 * @}
 */
  
  

/** @defgroup STA350BW_Exported_Functions
* @{
*/
  
int32_t STA350BW_RegisterBusIO(STA350BW_Object_t *pObj, STA350BW_IO_t *pIO);
int32_t STA350BW_Init(STA350BW_Object_t *pObj, void *params);
int32_t STA350BW_DeInit(STA350BW_Object_t *pObj);
int32_t STA350BW_ReadID(STA350BW_Object_t *pObj, uint32_t *Id);
int32_t STA350BW_Play(STA350BW_Object_t *pObj, uint16_t* pBuffer, uint16_t Size);
int32_t STA350BW_Pause(STA350BW_Object_t *pObj);
int32_t STA350BW_Resume(STA350BW_Object_t *pObj);
int32_t STA350BW_Stop(STA350BW_Object_t *pObj, uint32_t Cmd);
int32_t STA350BW_SetVolume(STA350BW_Object_t *pObj, uint32_t Cmd, uint8_t Volume);
int32_t STA350BW_SetMute(STA350BW_Object_t *pObj, uint32_t Cmd);
int32_t STA350BW_SetFrequency(STA350BW_Object_t *pObj, uint32_t AudioFreq);
int32_t STA350BW_Reset(STA350BW_Object_t *pObj);
int32_t STA350BW_SetOutputMode(STA350BW_Object_t *pObj, uint8_t Output);
int32_t STA350BW_SetResolution(void* a, uint32_t b);
int32_t STA350BW_GetResolution(void* a, uint32_t* b);  
int32_t STA350BW_SetProtocol(void* a, uint32_t b);
int32_t STA350BW_GetProtocol(void* a, uint32_t* b);
int32_t STA350BW_GetVolume(void* a, uint32_t b, uint8_t* c);
int32_t STA350BW_GetFrequency(void* a);  
  
int32_t STA350BW_SetEq(STA350BW_Object_t *pObj, uint8_t ramBlock, uint8_t filterNumber, uint32_t * filterValues);
int32_t STA350BW_SetTone(STA350BW_Object_t *pObj, uint8_t toneGain);
int32_t STA350BW_SetDSPOption(STA350BW_Object_t *pObj, uint8_t option, uint8_t state);
  
int32_t STA350BW_Read_Reg(STA350BW_Object_t *pObj, uint8_t Reg, uint8_t *Data);
int32_t STA350BW_Write_Reg(STA350BW_Object_t *pObj, uint8_t Reg, uint8_t Data);
  
  /**
 * @}
 */

/** @addtogroup STA350BW_Exported_Variables
* @{
*/
/* Audio processor driver structure */
extern STA350BW_AUDIO_Drv_t STA350BW_AUDIO_Driver;

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

#endif /*STA350BW_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


