/**
******************************************************************************
* @file    cca01m1_audio.c
* @author  SRA - Central Labs
* @version v3.0.0
* @date    6-May-19
* @brief   This file provides a set of functions needed to manage the 
*          sound terminal device.
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
/* Includes ------------------------------------------------------------------*/
#include "I2S_CCA01M1_ComIT_main.h"
#include "cca01m1_audio.h"
#include "audio.h"
#include "nucleo_errno.h"


I2C_HandleTypeDef   hi2c1;

/** @addtogroup BSP BSP
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1 X_NUCLEO_CCA01M1
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1_AUDIO X_NUCLEO_CCA01M1_AUDIO
* @{
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Private_Defines Private Defines
* @{
*/
#ifndef NULL
#define NULL      (void *) 0
#endif
  
    
/**
* @}
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Private_Variables Private Variables
* @{
*/
AUDIO_OUT_Ctx_t                 AudioOutCtx[AUDIO_OUT_INSTANCES_NBR] = {0};

static AUDIO_Drv_t              *AudioDrv = NULL;
void                            *CompObj = NULL;


I2S_HandleTypeDef hAudioOut[SOUNDTERMINAL_DEVICE_NBR];

/**
* @}
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Private_Function_Prototypes Private Function Prototypes
* @{
*/
/* I2S Msp config */
static void I2S_InitMXConfigStruct(I2S_HandleTypeDef* hi2s, MX_I2S_OUT_Config *MXConfig);

static int32_t STA350BW_Probe(void);

/**
* @}
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Exported_Functions Exported Functions
* @{
*/

/**
* @brief  Initialization of AUDIO Device.
* @param  handle: device handle 
* @param  Volume: initialization volume
* @param  AudioFreq: sampling frequency
* @retval COMPONENT_OK if no problem during initialization, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_Init(uint32_t Instance, CCA01M1_AUDIO_Init_t* AudioInit)
{ 
  uint8_t tmp;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM; 
  }
  else
  {  
    /* Fill AudioOutCtx structure */
    AudioOutCtx[Instance].Device         = AudioInit->Device;
    AudioOutCtx[Instance].Instance       = Instance; 
    AudioOutCtx[Instance].SampleRate     = AudioInit->SampleRate;
    AudioOutCtx[Instance].BitsPerSample  = AudioInit->BitsPerSample;
    AudioOutCtx[Instance].ChannelsNbr    = AudioInit->ChannelsNbr;
    AudioOutCtx[Instance].Volume         = AudioInit->Volume;
    AudioOutCtx[Instance].State          = AUDIO_OUT_STATE_RESET;
    

    if (Instance == 1U)
    {   
      switch(AudioOutCtx[Instance].Device)
      {
      case CODEC_SENSORS_AUTO:
      default:
        {
          tmp = STA350BW_0;  
          hAudioOut[STA350BW_0].Instance = AUDIO_OUT1_I2S_INSTANCE;   
          break;
        }
      case STA350BW_0:
        {
          tmp = STA350BW_0;
          hAudioOut[STA350BW_0].Instance = AUDIO_OUT1_I2S_INSTANCE;   
          break;
        }
      }
      /* PLL clock SETUP */ 
      if(MX_I2S_OUT_ClockConfig(&hAudioOut[tmp], AudioOutCtx[Instance].SampleRate) != HAL_OK)
      {
        return BSP_ERROR_CLOCK_FAILURE;
      }  
      
      MX_I2S_OUT_Config mx_i2s_config;
      
      /* Prepare hAudioOut handle */
      mx_i2s_config.AudioFreq    = AudioOutCtx[Instance].SampleRate;
      mx_i2s_config.CPOL         = I2S_CPOL_LOW;
      mx_i2s_config.DataFormat   = I2S_DATAFORMAT_16B;
      mx_i2s_config.MCLKOutput   = I2S_MCLKOUTPUT_ENABLE;
      mx_i2s_config.Mode         = I2S_MODE_MASTER_TX;
      mx_i2s_config.Standard     = I2S_STANDARD_PHILIPS;
      
      
      /* I2S peripheral initialization */
      if(MX_I2S_OUT_Init(&hAudioOut[tmp], &mx_i2s_config) != HAL_OK)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }
      
      if( CCA01M1_AUDIO_OUT_Reset(Instance) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_WRONG_PARAM;
      }

      // to avoid the PLL Locked error use 320 

      uint16_t dummy[320]={0};
      HAL_I2S_Transmit_DMA(&hAudioOut[tmp], dummy, 320);
            
      
      if(STA350BW_Probe()!= BSP_ERROR_NONE)
      {
        return BSP_ERROR_COMPONENT_FAILURE;
      } 
      
      HAL_I2S_DMAStop(&hAudioOut[tmp]);
      
      if(tmp == STA350BW_0)
      {    
        HAL_NVIC_EnableIRQ(AUDIO_OUT1_I2S_DMAx_IRQ);
      }    

    }
    
    /* Update BSP AUDIO OUT state */
    AudioOutCtx[Instance].State = AUDIO_OUT_STATE_STOP;
  }
  
  return BSP_ERROR_NONE; 
}

/**
* @brief  De-initialization of AUDIO Device.
* @param  handle: device handle 
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_DeInit(uint32_t Instance)
{  
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tmp = AudioOutCtx[Instance].Device;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM; 
  }
  else
  {
    if (Instance == 1U)
    { 
      HAL_I2S_DMAStop(&hAudioOut[tmp]);
      /* Call the Media layer stop function */
      if(AudioDrv->DeInit(CompObj) != 0)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else if(HAL_I2S_DeInit(&hAudioOut[tmp]) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }   
      else
      {
        /* Update BSP AUDIO OUT state */    
        AudioOutCtx[Instance].State = AUDIO_OUT_STATE_RESET;    
      }
    }
  }
  /* Return BSP status */
  return ret; 
}


/**
* @brief  Starts audio streaming to the AUDIO Device.
* @param  handle: device handle
* @param  *pBuffer: pointer to the data to be streamed
* @param  Size: data size
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_Play(uint32_t Instance, uint8_t* pData, uint32_t NbrOfBytes)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tmp = AudioOutCtx[Instance].Device;
  
  if((Instance >= AUDIO_OUT_INSTANCES_NBR) || (((NbrOfBytes / (AudioOutCtx[Instance].BitsPerSample/8U)) > 0xFFFFU)))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }  
  else if((AudioOutCtx[Instance].State == AUDIO_OUT_STATE_STOP) || (AudioOutCtx[Instance].State == AUDIO_OUT_STATE_RESET))
  {  
    if (Instance == 1U)
    {
      if (HAL_I2S_Transmit_DMA(&hAudioOut[tmp], (uint16_t *)pData, DMA_MAX(NbrOfBytes))!= HAL_OK) 
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if(ret == BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_PLAYING;
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret;  
  
}

/**
* @brief  This function Pauses the audio stream. In case
*         of using DMA, the DMA Pause feature is used.
* @param  handle: device handle
* @WARNING When calling CCA01M1_AUDIO_OUT_Pause() function for pause, only
*          CCA01M1_AUDIO_OUT_Resume() function should be called for resume (use of CCA01M1_AUDIO_OUT_Play() 
*          function for resume could lead to unexpected behavior).
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_Pause(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tmp = AudioOutCtx[Instance].Device;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  { 
    /* Call the Audio Codec Pause/Resume function */
    if(AudioDrv->Pause(CompObj) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    
    if (Instance == 1U)
    {    
      if (HAL_I2S_DMAPause(&hAudioOut[tmp])!= HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }   
    }
    
    if(ret == BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_PAUSE;   
    }
  }
  
  /* Return BSP status */
  return ret;
}

/**
* @brief  Resumes the audio stream.
* @param  handle: device handle
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_Resume(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tmp = AudioOutCtx[Instance].Device;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    ((STA350BW_Object_t *)CompObj)->audio_is_enabled = 1;
    /* Call the Audio Codec Pause/Resume function */
    if(AudioDrv->Resume(CompObj) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
        
    if (Instance == 1U)
    {  
      if (HAL_I2S_DMAResume(&hAudioOut[tmp])!= HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      } 
    }
    
    if(ret == BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_PLAYING;
    }
  }
  
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Stop the audio stream.
* @param  handle: device handle
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_Stop(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tmp = AudioOutCtx[Instance].Device;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (AudioOutCtx[Instance].State == AUDIO_OUT_STATE_PLAYING)
  { 
    /* Call the Audio Codec Pause/Resume function */
    if(AudioDrv->Stop(CompObj, (uint32_t)NULL) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    if (Instance == 1U)
    { 
      if (HAL_I2S_DMAStop(&hAudioOut[tmp])!= HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    
    if( ret==BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_STOP;  
    }
  }  
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret;
}

/**
* @brief  Set volume.
* @param  handle: device handle
* @param  channel: channel to be configured
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_SetVolume(uint32_t Instance, uint32_t Volume)
{
  int32_t ret = BSP_ERROR_NONE;
  int32_t ii;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  { 
    for (ii = 0; ii<AudioOutCtx[Instance].ChannelsNbr; ii++)
    {
      if(AudioDrv->SetVolume(CompObj, ii, Volume) != 0)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else if(Volume == 0U)
      {
        /* Update Mute State */
        AudioOutCtx[Instance].IsMute = CCA01M1_AUDIO_MUTE_ENABLED;
      }
      else
      {
        /* Update Mute State */
        AudioOutCtx[Instance].IsMute = CCA01M1_AUDIO_MUTE_DISABLED;      
      }
    }
    AudioOutCtx[Instance].Volume = Volume;
  }
  
  /* Return BSP status */
  return ret;  
}


/**
* @brief  Get the current audio volume level.
* @param AUDIO OUT Instance. It can only be 0 (SAI)
* @param  Volume    pointer to volume to be returned
* @retval BSP status
*/
int32_t CCA01M1_AUDIO_OUT_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    *Volume = AudioOutCtx[Instance].Volume;
  }
  /* Return BSP status */
  return ret;  
}


/**
* @brief  Set mute.
* @param  handle: device handle
* @param  channel: channel to be muted
* @param  state: enable or disable value
*         This parameter can be a value of @ref STA350BW_state_define 
* @param  filterValues: pointer to filter values    
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_Mute(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  { 
    ret = BSP_ERROR_WRONG_PARAM;    
  }
  else
  {
    if(AudioDrv->SetMute(CompObj, CODEC_MUTE_ON) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      /* Update Mute State */
      AudioOutCtx[Instance].IsMute = CCA01M1_AUDIO_MUTE_ENABLED;
    }
  }
  /* Return BSP status */
  return ret;    
}


/**
* @brief  Set unmute.
* @param  handle: device handle
* @param  channel: channel to be muted
* @param  state: enable or disable value
*         This parameter can be a value of @ref STA350BW_state_define 
* @param  filterValues: pointer to filter values    
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_UnMute(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  { 
    ret = BSP_ERROR_WRONG_PARAM;    
  }
  else
  {
    if(AudioDrv->SetMute(CompObj, CODEC_MUTE_OFF) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      /* Update Mute State */
      AudioOutCtx[Instance].IsMute = CCA01M1_AUDIO_MUTE_DISABLED;
    }
  }
  /* Return BSP status */
  return ret;    
}


/**
* @brief  Check whether the MUTE mode is enabled or not
* @param AUDIO OUT Instance. It can only be 0 (SAI)
* @param  IsMute    pointer to mute state
* @retval Mute status
*/
int32_t CCA01M1_AUDIO_OUT_IsMute(uint32_t Instance, uint32_t *IsMute)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    *IsMute = AudioOutCtx[Instance].IsMute; 
  }
  /* Return BSP status */
  return ret; 
}

/**
* @brief  Switch dynamically (while audio file is played) the output target 
*         (speaker or headphone).
* @param AUDIO OUT Instance. It can only be 0 (SAI)
* @param  Device  The audio output device
* @retval BSP status
*/
int32_t CCA01M1_AUDIO_OUT_SetDevice(uint32_t Instance, uint32_t Device)
{
  int32_t ret;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  { 
    /* Call the Codec output device function */
    if(AudioDrv->SetOutputMode(CompObj, Device) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    
    if (Instance == 1U)
    {
      MX_I2S_OUT_Config mx_out_config;
      /* Get SAI MX configuration */
      I2S_InitMXConfigStruct(&hAudioOut[Device], &mx_out_config);
      if(MX_I2S_OUT_Init(&hAudioOut[Device], &mx_out_config) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    /* Update AudioOutCtx structure */
    AudioOutCtx[Instance].Device = Device;
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret; 
}

/**
* @brief  Get the Output Device 
* @param AUDIO OUT Instance. It can only be 0 (SAI)
* @param  Device    The audio output device
* @retval BSP status
*/
int32_t CCA01M1_AUDIO_OUT_GetDevice(uint32_t Instance, uint32_t *Device)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  { 
    /* Get AudioOutCtx Device */
    *Device = AudioOutCtx[Instance].Device;
  } 
  /* Return BSP status */
  return ret;   
}



/**
* @brief  Set frequency of the I2S bus.
* @param  handle: device handle
* @param  AudioFreq: sampling frequency
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_SetSampleRate(uint32_t Instance, uint32_t SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tmp = AudioOutCtx[Instance].Device;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  {
    if(AudioDrv->SetFrequency(CompObj, SampleRate) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }  
    
    if (Instance == 1U)
    {
      MX_I2S_OUT_Config mx_i2s_config;
      
      /* Get I2S MX configuration */
      I2S_InitMXConfigStruct(&hAudioOut[tmp], &mx_i2s_config);
      /* PLL clock setup */ 
      
      if(MX_I2S_OUT_ClockConfig(&hAudioOut[tmp], SampleRate) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else if(MX_I2S_OUT_Init(&hAudioOut[tmp], &mx_i2s_config) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    /* Store new sample rate */
    AudioOutCtx[Instance].SampleRate = SampleRate;     
  }
  else
  {
    ret = BSP_ERROR_BUSY; 
  }
  
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Get the audio frequency.
* @param AUDIO OUT Instance. It can only be 0 (SAI)
* @param  SampleRate  Audio frequency used to play the audio stream.
* @retval BSP status
*/
int32_t CCA01M1_AUDIO_OUT_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    *SampleRate = AudioOutCtx[Instance].SampleRate;
  }   
  /* Return BSP status */
  return ret; 
}


/**
* @brief  Get the audio Resolution.
* @param AUDIO OUT Instance. It can only be 0 (SAI)
* @param  BitsPerSample  Audio Resolution used to play the audio stream.
* @retval BSP status
*/

int32_t CCA01M1_AUDIO_OUT_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  {
    AudioOutCtx[Instance].BitsPerSample = BitsPerSample;
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret; 
}    

/**
* @brief  Get the audio Resolution.
* @param AUDIO OUT Instance. It can only be 0 (SAI)
* @param  BitsPerSample  Audio Resolution used to play the audio stream.
* @retval BSP status
*/
int32_t CCA01M1_AUDIO_OUT_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Get audio Out resolution */
    *BitsPerSample = AudioOutCtx[Instance].BitsPerSample;    
  }
  
  /* Return BSP status */
  return ret; 
}

/**
* @brief  Set the audio Channels number.
* @param AUDIO OUT Instance. It can only be 0 (SAI)
* @param  ChannelNbr  Audio Channels number used to play the audio stream.
* @retval BSP status
*/
int32_t CCA01M1_AUDIO_OUT_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if((Instance >= AUDIO_OUT_INSTANCES_NBR) || (ChannelNbr > 2U))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  {
    /* Set the audio Channels number */
    if(ChannelNbr != 2U)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {    
      /* Store new Channel number */
      AudioOutCtx[Instance].ChannelsNbr = ChannelNbr;         
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret; 
}

/**
* @brief  Get the audio Channels number.
* @param AUDIO OUT Instance. It can only be 0 (SAI)
* @param  ChannelNbr     Audio Channels number used to play the audio stream.
* @retval BSP status
*/
int32_t CCA01M1_AUDIO_OUT_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Get the audio Channels number */
    *ChannelNbr = AudioOutCtx[Instance].ChannelsNbr;
  }
  
  /* Return BSP status */
  return ret; 
}

/**
* @brief  Get Audio Out state
* @param AUDIO OUT Instance. It can only be 0 (SAI)
* @param  State     Audio Out state
* @retval BSP status
*/
int32_t CCA01M1_AUDIO_OUT_GetState(uint32_t Instance, uint32_t *State)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Return audio Output State */
    *State = AudioOutCtx[Instance].State;
  }
  
  /* Return BSP status */  
  return ret;
}


/**
* @brief  Reset the CCA01M1 device.
* @param  handle: device handle
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise 
* @retval None
*/
int32_t CCA01M1_AUDIO_OUT_Reset(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {    
    if(AudioOutCtx[Instance].Device == STA350BW_0)
    {
      HAL_GPIO_WritePin(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN, GPIO_PIN_RESET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN, GPIO_PIN_SET);  
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }
  return ret; 
}

/**
* @brief  Manages the DMA full Transfer complete event.
* @param  OutputDevice: device relevant with the thrown interrupt
*         This parameter can be a value of @ref CODEC_ID_t 
* @retval None
*/
__weak void CCA01M1_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance); 
}

/**
* @brief  Manages the DMA Half Transfer complete event.
* @param  OutputDevice: device relevant with the thrown interrupt
*         This parameter can be a value of @ref CODEC_ID_t 
* @retval None
*/
__weak void CCA01M1_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance); 
}



/*******************************************************************************
Static Functions
*******************************************************************************/
/**
* @brief  Register Bus IOs if component ID is OK
* @retval error status
*/
static int32_t STA350BW_Probe(void)
{
  int32_t ret = BSP_ERROR_NONE;
  STA350BW_IO_t              IOCtx;
  static STA350BW_Object_t   STA350BWObj;

  
/* Configure the SDA setup, hold time and the SCL high, low period */
/* (uint32_t)0x10320309 = I2C TIMING in Fast Mode                  */

/* Configure the SDA setup, hold time and the SCL high, low period 
 * For Fast-mode     kHz, PRESC | 0h | SCLDEL | SDADEL | SCLH | SCLL 
 *                          1h  | 0h |    3h  |   2h   |  03h |  09h 
 * timing = __LL_I2C_CONVERT_TIMINGS(0x01, 0x03, 0x02, 0x03, 0x09);
 */ 

 /* I2C TIMING in Fast Mode */
__IO uint32_t timing = __LL_I2C_CONVERT_TIMINGS(0x01, 0x03, 0x02, 0x03, 0x09);

  hi2c1.Instance = AUDIO_OUT1_I2C_INSTANCE;
  hi2c1.Init.Timing = timing;
  hi2c1.Init.OwnAddress1 = STA350BW_ADDRESS_1;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  
  /* Configure the audio driver */
  IOCtx.Init        = HAL_I2C_Init;
  IOCtx.DeInit      = HAL_I2C_DeInit;  
  IOCtx.BusType     = STA350BW_I2C_BUS;
  IOCtx.ReadReg     = HAL_I2C_Mem_Read;
  IOCtx.WriteReg    = HAL_I2C_Mem_Write; 
  IOCtx.GetTick     = HAL_GetTick;  

  if (AudioOutCtx[0].Device == STA350BW_0 && AudioOutCtx[0].ChannelsNbr != 0)
  {
    IOCtx.Address     = STA350BW_ADDRESS_1;
  }
  else if (AudioOutCtx[0].Device == STA350BW_1 && AudioOutCtx[0].ChannelsNbr != 0)
  {
    IOCtx.Address     = STA350BW_ADDRESS_2;
  }
  else if (AudioOutCtx[1].Device == STA350BW_0 && AudioOutCtx[1].ChannelsNbr != 0)
  {
    IOCtx.Address     = STA350BW_ADDRESS_1;
  }
  else if (AudioOutCtx[1].Device == STA350BW_1 && AudioOutCtx[1].ChannelsNbr != 0)
  {
    IOCtx.Address     = STA350BW_ADDRESS_2;
  }
  else
  {
    return BSP_ERROR_WRONG_PARAM;
  }
   /* copy value from IOCtx to STA350BWObj */
  if(STA350BW_RegisterBusIO (&STA350BWObj, &IOCtx) != STA350BW_OK)
  {
    ret = BSP_ERROR_BUS_FAILURE;   
  }
  else
  {
    AudioDrv = (AUDIO_Drv_t *) (void *) &STA350BW_AUDIO_Driver;
    CompObj = &STA350BWObj;    
  }
  

    if (AudioDrv->Init(CompObj, (void *)&AudioOutCtx[1]) != STA350BW_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  return ret;
} 



/**
* @brief  hi2s clock Config.
* @param  hi2s hi2s handle
* @param  SampleRate  Audio frequency used to play the audio stream.
* @note   This API is called by CCA01M1_AUDIO_OUT_Init() and CCA01M1_AUDIO_OUT_SetFrequency()
*         Being __weak it can be overwritten by the application     
* @retval HAL status
*/
__weak HAL_StatusTypeDef MX_I2S_OUT_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t SampleRate)
{
  HAL_StatusTypeDef ret = HAL_OK;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);
  
  
  /* The STA350Bx supports sample rates of 32 kHz, 44.1 kHz, 48 kHz, 88.2 kHz, 96 kHz,176.4 kHz, and 192 kHz. Therefore the internal clock is:
   * 32.768 MHz for 32 kHz
   */
  if (SampleRate == 32000)
  {
     __HAL_RCC_SPI2I2S_CONFIG(RCC_SPI2I2S_CLKSOURCE_32M);
  }
  else
  {
    ret = HAL_ERROR;
  } 
  
  return ret;
}


/**
* @brief  Initializes the Audio Codec audio out instance (I2S).
* @param  MXConfig I2S configuration structure
* @note   Being __weak it can be overwritten by the application
* @retval HAL status
*/
__weak HAL_StatusTypeDef MX_I2S_OUT_Init(I2S_HandleTypeDef* hi2s, MX_I2S_OUT_Config *MXConfig)
{ 
  HAL_StatusTypeDef ret = HAL_OK;
  
  /* Disable I2S peripheral to allow access to I2S internal registers */
  __HAL_I2S_DISABLE(hi2s);
  
  /* Configure I2S */
  hi2s->Init.AudioFreq       = MXConfig->AudioFreq;  
  hi2s->Init.CPOL            = MXConfig->CPOL;
  hi2s->Init.DataFormat	     = MXConfig->DataFormat;
  hi2s->Init.MCLKOutput      = MXConfig->MCLKOutput;
  hi2s->Init.Mode            = MXConfig->Mode;
  hi2s->Init.Standard        = MXConfig->Standard;
  
  
  if(HAL_I2S_Init(hi2s) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  
  __HAL_I2S_ENABLE(hi2s);
  
  return ret;
}



/**
* @brief Tx Transfer completed callbacks
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Call the record update function to get the next buffer to fill and its size (size is ignored) */
  if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
  {
    CCA01M1_AUDIO_OUT_TransferComplete_CallBack(1);
  }
  
}

/**
* @brief Tx Transfer Half completed callbacks
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Manage the remaining file size and new address offset: This function 
  should be coded by user */  
  if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
  {
    CCA01M1_AUDIO_OUT_HalfTransfer_CallBack(1);
  }


}

/**
* @brief  Get I2S MX Init configuration
* @param  hi2s I2S handle
* @param  MXConfig I2S configuration structure
* @retval None
*/
static void I2S_InitMXConfigStruct(I2S_HandleTypeDef* hi2s, MX_I2S_OUT_Config *MXConfig)
{
  MXConfig->AudioFreq  = hi2s->Init.AudioFreq; 
  MXConfig->CPOL       = hi2s->Init.CPOL;      
  MXConfig->DataFormat = hi2s->Init.DataFormat;
  MXConfig->MCLKOutput = hi2s->Init.MCLKOutput;
  MXConfig->Mode       = hi2s->Init.Mode;      
  MXConfig->Standard   = hi2s->Init.Standard ; 
  
}

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


     
