/**
  ******************************************************************************
  * @file    PDM_DiglMic.c
  * @author  RF Application Team
  * @brief   Digital Microphone
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  *******************************************************************************/
#include "bluenrg_lpx.h"
#include "sensor.h"
#if ENABLE_BLUEVOICE  
/* Includes ------------------------------------------------------------------*/
#include "rf_driver_ll_adc.h"
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_bus.h"
#include "rf_driver_ll_system.h"
#include "rf_driver_ll_exti.h"
#include "rf_driver_ll_cortex.h"
#include "rf_driver_ll_utils.h"
#include "rf_driver_ll_pwr.h"
#include "rf_driver_ll_dma.h"
#include "rf_driver_ll_usart.h"

#include "rf_driver_ll_gpio.h"

    
#if defined(USE_FULL_ASSERT)
#include "bluenrg_lp_assert.h"
#endif /* USE_FULL_ASSERT */

#include "bluenrg_lp_evb_config.h"
#include "sensor.h"

/** @addtogroup RF_DRIVER_LL_Examples
  * @{
  */

/** @addtogroup PDM_DiglMic
  * @{
  */


/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
extern int16_t PCM_Buffer[];
volatile static uint16_t discard_first_flow = 2, start_sending_data = 0;

/* Private function prototypes -----------------------------------------------*/
static void APP_ADC_Init(void);
static void APP_DMA_Init(void);

/* Private user code ---------------------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
void DMA_Rearm(uint32_t dma_channel, uint32_t buffer, uint32_t size) 
{

  /* Rearm the DMA transfer */
  LL_DMA_SetMemoryAddress(DMA1, dma_channel, buffer);
  
  LL_DMA_SetDataLength(DMA1, dma_channel, size);

}

/**
  * @brief  PDM_DigitalMicrophone_Init
  * @retval None
  */
void PDM_DigitalMicrophone_Init(void ) 
{
  /* Initialize the digital microphone pins */
  BSP_DIGMIC_Init();
  
  /* Initialize the ADC peripheral */
  APP_ADC_Init();
  
  /* Initialize the DMA peripheral */
  APP_DMA_Init();
  
  /* Start the DMA transfer */
  LL_ADC_DMAModeDFEnable(ADC);
  
  /* Start ADC conversion */ 
  LL_ADC_SyncADCStartWithSMPSPulseEnable(ADC);
}



/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void APP_ADC_Init(void)
{
  /* Peripheral clock enable */
  LL_APB1_EnableClock(LL_APB1_PERIPH_ADCDIG | LL_APB1_PERIPH_ADCANA);

  /* This function must not be called on QFN32 package */
  LL_ADC_LDOEnable(ADC);

  /* Enable the ADC */
  LL_ADC_Enable(ADC);
    
  /* Set digital microphone output datarate at 7.936 kHz */
  LL_ADC_SetMicrophoneOutputDatarate(ADC, LL_ADC_OUTPUT_FREQ_MIC_DIG_7936_HZ);

  /* Configure the operation mode as ADC mode (PDM mode) */
  LL_ADC_SetADCMode(ADC, LL_ADC_OP_MODE_ADC);

  /* Configure the ADC continuous mode */
  LL_ADC_ContinuousModeEnable(ADC);
  
  LL_ADC_SetOverrunDF(ADC, LL_ADC_NEW_DATA_IS_LOST);  
  
}


/**
  * @brief DMA Initialization Function
  * @param None
  * @retval None
  */
static void APP_DMA_Init(void)
{
  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB_EnableClock(LL_AHB_PERIPH_DMA);

  /* Configure DMA request MEMTOMEM_DMA1_Channel1 */

  /* Set the DMA channel 1 with the ADC Decimation Filter output */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_ADC, LL_DMAMUX_REQ_ADC_DF);

  /* Set transfer direction from ADC DS peripheral to RAM memory */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_ADC, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* Set priority level */
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_ADC, LL_DMA_PRIORITY_HIGH);

  /* Set DMA mode */
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_ADC, LL_DMA_MODE_CIRCULAR);

  /* Set peripheral increment mode to no increment */
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_ADC, LL_DMA_PERIPH_NOINCREMENT);

  /* Set memory increment mode to increment */
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_ADC, LL_DMA_MEMORY_INCREMENT);

  /* Set peripheral data width to 16-bit */
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_ADC, LL_DMA_PDATAALIGN_HALFWORD);

  /* Set memory data width to 16-bit */
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_ADC, LL_DMA_MDATAALIGN_HALFWORD);


  /* Set DMA transfer addresses of source and destination */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_ADC,
                         LL_ADC_GetOutputDataRegDF(ADC),
                         (uint32_t)&PCM_Buffer,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* Set DMA transfer size */
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_ADC, PCM_BUFFER_SIZE);

  /* Enable DMA Transfer Complete interrupts */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_ADC);

  /* Enable DMA Half Transfer interrupts */
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_ADC);
  
  /* Configure NVIC for DMA half transfer/transfer complete interrupts */
  NVIC_SetPriority(DMA_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(DMA_IRQn);
}

#endif 
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
