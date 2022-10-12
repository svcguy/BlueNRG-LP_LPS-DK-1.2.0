/**
  ******************************************************************************
  * @file    audio_application.c 
  * @author  SRA - Central Labs
  * @version v3.0.0
  * @date    6-May-19
  * @brief   Audio output application. 
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

/* Includes ------------------------------------------------------------------*/
#include "audio_application.h"

/** @addtogroup X_CUBE_SOUNDTER1_Applications
* @{
*/ 

/** @addtogroup Audio_Streaming
* @{
*/

/** @defgroup AUDIO_APPLICATION 
* @{
*/

/** @defgroup AUDIO_APPLICATION_Exported_Variables 
* @{
*/

/**
* @}
*/

/** @defgroup AUDIO_APPLICATION_Private_Variables 
* @{
*/

static int16_t Audio_output_buffer[AUDIO_OUT_BUFFER_SIZE * 2];
static uint32_t song_position = 0;

CCA01M1_AUDIO_Init_t OutParams;
/**
* @}
*/

/** @defgroup AUDIO_APPLICATION_Exported_Function 
* @{
*/

/**
* @brief  Initializes all the required peripherals using the BSP Init function.
* @param  None
* @retval AUDIO_OK if no problem during initialization, AUDIO_ERROR otherwise
*/
uint32_t Init_AudioOut_Device(void)
{
  OutParams.BitsPerSample = 16;
  OutParams.ChannelsNbr = AUDIO_OUT_CHANNELS;
  OutParams.Device = STA350BW_0;
  OutParams.SampleRate = AUDIO_OUT_SAMPLING_FREQUENCY;
  OutParams.Volume = AUDIO_VOLUME_OUT;

  return CCA01M1_AUDIO_OUT_Init(CCA01M1_AUDIO_OUT_INSTANCE, &OutParams);
}

/**
* @brief  Starts audio output.
* @param  None
* @retval AUDIO_OK if no problem during initialization, AUDIO_ERROR otherwise
*/
uint32_t Start_AudioOut_Device(void)
{
  return CCA01M1_AUDIO_OUT_Play(CCA01M1_AUDIO_OUT_INSTANCE, (uint8_t *)Audio_output_buffer, AUDIO_OUT_BUFFER_SIZE*2);
}

/**
* @brief  Stops audio output.
* @param  None
* @retval AUDIO_OK if no problem during initialization, AUDIO_ERROR otherwise
*/
uint32_t Stop_AudioOut_Device(void)
{
  return CCA01M1_AUDIO_OUT_Stop(CCA01M1_AUDIO_OUT_INSTANCE);
}

/**
* @brief  Switch Filter configuration for demo purpose.
* @param  None
* @retval AUDIO_OK if no problem during initialization, AUDIO_ERROR otherwise
*/
uint32_t Switch_Demo(void)
{
  uint8_t ret = 0;    
  static uint8_t current_demo = 0;
  
  switch(current_demo)
  {
  case 0:
    { 
      printf("Setup Default Master Volume\n\r");
      if(CCA01M1_AUDIO_OUT_SetVolume(CCA01M1_AUDIO_OUT_INSTANCE ,AUDIO_VOLUME_OUT) != 0)
      {
          Error_Handler(); 
      }     
      break;
    }
  case 1:
    { 
      printf("mute the system\n\r");
      if(CCA01M1_AUDIO_OUT_Mute(CCA01M1_AUDIO_OUT_INSTANCE) != 0)
      {
          Error_Handler(); 
      }
      break; 
    }    
  case 2:
    {
      printf("unMute the system\n\r");
      if(CCA01M1_AUDIO_OUT_UnMute(CCA01M1_AUDIO_OUT_INSTANCE) != 0)
      {
          Error_Handler(); 
      }
      break; 
    }     
  case 3:
    {
      printf("Pause the system\n\r");
      if(CCA01M1_AUDIO_OUT_Pause(CCA01M1_AUDIO_OUT_INSTANCE) != 0)
      {
          Error_Handler(); 
      }
      break; 
    }     

  case 4:
    {
      printf("Resume audio\n\r");
      if(CCA01M1_AUDIO_OUT_Resume(CCA01M1_AUDIO_OUT_INSTANCE) != 0)
      {
          Error_Handler(); 
      }
      break; 
    }     

  }
  current_demo = (current_demo + 1) % 5;
  return ret;
}

/**
* @brief  Manages the DMA Half Transfer complete event.
* @param  OutputDevice: the sound terminal device related to the DMA 
*         channel that generates the interrupt
* @retval None
*/
void CCA01M1_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)
{ 
  uint32_t i = 0;

  /*Copy song fragment to Audio Output buffer*/
  for(i=0; i<AUDIO_OUT_BUFFER_SIZE/2; i++){    
    Audio_output_buffer[2*i]= Fragment1[song_position]; /*Left Channel*/
    Audio_output_buffer[2*i + 1]= Fragment1[song_position]; /*Right Channel*/
    song_position = (song_position +1) % Fragment1_size;
  }
  
}

/**
* @brief  Manages the DMA Transfer complete event.
* @param  OutputDevice: the sound terminal device related to the DMA 
*         channel that generates the interrupt
* @retval None
*/
void CCA01M1_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance)
{
  uint32_t i = 0;
  /*Copy song fragment to Audio Output buffer*/
  for(i=AUDIO_OUT_BUFFER_SIZE/2; i<AUDIO_OUT_BUFFER_SIZE; i++)
  {    
    Audio_output_buffer[2*i]= Fragment1[song_position]; /*Left Channel*/
    Audio_output_buffer[2*i + 1]= Fragment1[song_position]; /*Right Channel*/
    song_position = (song_position +1) % Fragment1_size;
  }  
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

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



