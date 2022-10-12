
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : ADC_AnalogMicrophone_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the ADC functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  ADC_AnalogMicrophone/ADC_AnalogMicrophone_main.c
 * @brief This example shows how to use the ADC peripheral to interface an analog
 * microphone.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\ADC\\ADC_AnalogMicrophone\\MDK-ARM\\{STEVAL-IDB011V1}\\ADC_AnalogMicrophone.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\ADC\\ADC_AnalogMicrophone\\EWARM\\{STEVAL-IDB011V1}\\ADC_AnalogMicrophone.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\ADC\\ADC_AnalogMicrophone\\WiSE-Studio\\{STEVAL-IDB011V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - Release configuration


* \section Board_supported Boards supported
- \c STEVAL-IDB011V1
- \c STEVAL-IDB011V2



* \section Power_settings Power configuration settings
@table

==========================================================================================================
|                                         STEVAL-IDB01xV1                                                |
----------------------------------------------------------------------------------------------------------
| Jumper name | Description                                                                |
| JP2         |                                                                            |
----------------------------------------------------------------------------------------------------------
| USB         | USB supply power                                                            |
| BAT         | The supply voltage must be provided through battery pins.                   |


@endtable

* \section Jumper_settings Jumper settings
@table

========================================================================================================================================================================================
|                                                                             STEVAL-IDB01xV1                                                                                          |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Jumper name |                                                                Description                                                                                             |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
| JP1         | It provides the voltage to the BlueNRG-LP circuit. It must be fitted. It can be used for current measurements of the BlueNRG-LP device.                                |          
| JP2         | It is a switch between two power domains. BAT position: to provide power from battery holder; USB position: to provide power from USB connector.                       |
| JP3         | It connects the BLE_SWCLK pin of the BlueNRG-LP with the SWCLK pin of the USB_CMSISDAP. It must be fitted.                                                             |          
| JP4         | It connects the BLE_SWDIO pin of the BlueNRG-LP with the SWDIO pin of the USB_CMSISDAP. It must be fitted.                                                             |
| JP5         | It connects the BLE_RSTN pin of the BlueNRG-LP with the rest of the board (the USB_CMSISDAP and RESET push button). It must be fitted.                                 |


@endtable 

* \section Pin_settings Pin settings
@table
|  PIN name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |
---------------------------------------------------------
|     A0     |        N.A.        |        N.A.        |
|     A1     |      Not Used      |      Not Used      |
|     A10    |        N.A.        |        N.A.        |
|     A11    |      Not Used      |      Not Used      |
|     A12    |      Not Used      |      Not Used      |
|     A13    |      Not Used      |      Not Used      |
|     A14    |      Not Used      |      Not Used      |
|     A15    |      Not Used      |      Not Used      |
|     A3     |        N.A.        |        N.A.        |
|     A4     |      Not Used      |      Not Used      |
|     A5     |      Not Used      |      Not Used      |
|     A6     |      Not Used      |      Not Used      |
|     A7     |      Not Used      |      Not Used      |
|     A8     |      Not Used      |      Not Used      |
|     A9     |      Not Used      |      Not Used      |
|     B0     |      Not Used      |      Not Used      |
|     B1     |        N.A.        |        N.A.        |
|     B12    |        N.A.        |        N.A.        |
|     B13    |        N.A.        |        N.A.        |
|     B14    |      Not Used      |      Not Used      |
|     B15    |        N.A.        |        N.A.        |
|     B2     |      Not Used      |      Not Used      |
|     B3     |      Not Used      |      Not Used      |
|     B4     |      Not Used      |      Not Used      |
|     B5     |      Not Used      |      Not Used      |
|     B6     |        N.A.        |        N.A.        |
|     B7     |      Not Used      |      Not Used      |
|     B8     |      Not Used      |      Not Used      |
|     B9     |      Not Used      |      Not Used      |
|     GND    |      Not Used      |      Not Used      |
|     RST    |      Not Used      |      Not Used      |
|    VBAT    |      Not Used      |      Not Used      |

@endtable 

@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 921600 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Start bits      | 1                | bit       |
| Stop bits       | 1                | bit       |
| HW flow control | None             | bit       |
@endtable

* \section LEDs_description LEDs description
@table
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |
---------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |
|     U5     |    Activity LED    |    Activity LED    |

@endtable


* \section LEDs_description LEDs description
@table
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |
---------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |
|     U5     |    Activity LED    |    Activity LED    |

@endtable


* \section Buttons_description Buttons description
@table
|  PIN name  | STEVAL-IDB011V{1|2} |
-----------------------------------
|     A1     |       Not Used      |
|     A11    |       Not Used      |
|     A12    |       Not Used      |
|     A13    |       Not Used      |
|     A14    |       Not Used      |
|     A15    |       Not Used      |
|     A4     |       Not Used      |
|     A5     |       Not Used      |
|     A6     |          U5         |
|     A7     |       Not Used      |
|     A8     |       USART TX      |
|     A9     |       USART RX      |
|     B0     |       Not Used      |
|     B14    |       Not Used      |
|     B2     |       Not Used      |
|     B3     |       Not Used      |
|     B4     |       PGA_VIN       |
|     B5     |       VBIAS_MIC     |
|     B7     |       Not Used      |
|     B8     |       Not Used      |
|     B9     |       Not Used      |
|     A0     |         N.A.        |
|     A10    |         N.A.        |
|     B1     |         N.A.        |
|     B6     |         N.A.        |
|     B15    |         N.A.        |
|     GND    |       Not Used      |
|     RST    |       Not Used      |
|    VBAT    |       Not Used      |
@endtable

* \section Usage Usage


This example shows how to use the ADC peripheral to interface an analog microphone. 
The DMA peripheral is used to transfer the data output from the ADC output register to a RAM buffer. 
Then, the data are printed out by using the USART peripheral.
This example is driven by interrupts from the DMA peripheral: Half Transfer and Transfer Complete events.
This example is based on the BLUENRG_LP ADC LL API.
The peripheral initialization is done using LL unitary service functions for optimization purposes (performance and size).

Example configuration:
The ADC is configured to sample the output data coming from an analog microphone connected to the PGA port of the ADC peripheral.
The user can change the PGA setting and the sample rate of the ADC.
The USART baudrate is set to 921600 bps for maintain high performance.

Example execution:
The user must connect the analog microphone according to the application circuit suggested for the specific analog microphone. 
The input port is the pin PB4. 
The BlueNRG-LP provides also a Vbias for the analog microphone from the pin PB5. 
The specific Vbias voltage can be set by user.
The user can get the output data by logging the data through a serial terminal and then process the audio stream acquired.

Connection needed:
PB4 must be connected to the output of the analog microphone, it is recommended to use a capacitor of 1 uF, see Reference Manual.
PB5 must be connected to supply the input voltage to the analog microphone.

Other peripherals used:
  1 USART for output the data acquired
  1 DMA for transfer the output data from the ADC to the RAM memory

**/
   
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
#include "bluenrg_lpx.h"
#include "rf_driver_ll_gpio.h"
#if defined(USE_FULL_ASSERT)
#include "rf_driver_assert.h"
#endif /* USE_FULL_ASSERT */

#include "bluenrg_lp_evb_config.h"

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PCM_BUFFER_SIZE (2600 * 2)

#define USER_CONFIG_PGA_BIAS    (LL_ADC_PGA_BIAS_050_BAT)
#define USER_CONFIG_PGA_GAIN    (LL_ADC_PGA_GAIN_30_DB)
#define USER_CONFIG_MIC_FREQ    (LL_ADC_OUTPUT_FREQ_MIC_ANA_7936_HZ)
#define USER_CONFIG_SAMPLE_RATE (LL_ADC_SAMPLE_RATE_16)

#define LL_DMA_CHANNEL_ADC      LL_DMA_CHANNEL_3

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
static int16_t PCM_Buffer[PCM_BUFFER_SIZE];
volatile static uint16_t discard_first_flow = 2, start_sending_data = 0;

/* Private function prototypes -----------------------------------------------*/
static void APP_ADC_Init(void);
static void APP_DMA_Init(void);
static void BSP_Init(void);
static void TC_IT_Callback(void);
static void HT_IT_Callback(void);
static void DMA_Rearm(uint32_t dma_channel, uint32_t buffer, uint32_t size);

/* Private user code ---------------------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
static void DMA_Rearm(uint32_t dma_channel, uint32_t buffer, uint32_t size)
{
  if(start_sending_data)
    while(!LL_DMA_IsActiveFlag_TC1(DMA1));
  
  /* Disable DMA Channel */
  LL_DMA_DisableChannel(DMA1, dma_channel);
  
  /* Rearm the DMA transfer */
  LL_DMA_SetMemoryAddress(DMA1, dma_channel, buffer);
  
  LL_DMA_SetDataLength(DMA1, dma_channel, size);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, RADIO_SYSCLK_NONE) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* BSP Init */
  BSP_Init();
  
  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);
  
  /* Initialize the digital microphone pins */
  BSP_ANAMIC_Init();
  
  /* Initialize the ADC peripheral */
  APP_ADC_Init();
  
  /* Initialize the DMA peripheral */
  APP_DMA_Init();
  
  /* Start the DMA transfer */
  LL_ADC_DMAModeDFEnable(ADC);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_ADC);
  
  /* Start ADC conversion */
  LL_ADC_StartConversion(ADC);
  
  /* Infinite loop */
  while (1) {
    /* Led toggle - activity led */
    LL_mDelay(250);
    BSP_LED_Toggle(BSP_LED1);
  }
  
}


static void BSP_Init(void)
{
  /* Initialization of LEDs */
  BSP_LED_Init(BSP_LED1);

#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
    
  /* USART TX DMA configuration */
  LL_DMA_InitTypeDef DMA_InitStruct;
  LL_AHB_EnableClock(LL_AHB_PERIPH_DMA);
  LL_DMA_StructInit(&DMA_InitStruct);
  
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_USART1_TX;
  DMA_InitStruct.Priority = LL_DMA_PRIORITY_VERYHIGH;
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&PCM_Buffer, (uint32_t)&(USART1->TDR), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  
  /* Enable UART_DMAReq_Tx */
  LL_USART_EnableDMAReq_TX(USART1);
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

  /* Initialize the PGA */
  LL_ADC_ConfigureMicrophonePGA(ADC, USER_CONFIG_PGA_BIAS, USER_CONFIG_PGA_GAIN);
  LL_ADC_SetCalibPoint1Gain(ADC,0xFFF);
  
  /* Set microphone output datarate */
  LL_ADC_SetMicrophoneOutputDatarate(ADC, USER_CONFIG_MIC_FREQ);
  
  /* Set the sample rate */
  LL_ADC_SetSampleRate(ADC, USER_CONFIG_SAMPLE_RATE);

  /* Configure the operation mode as AUDIO mode */
  LL_ADC_SetADCMode(ADC, LL_ADC_OP_MODE_AUDIO);

  /* Start the sampling at end of previous sample */
  LL_ADC_InputSamplingMode(ADC, LL_ADC_SAMPLING_AT_END);

  /* Configure the ADC continuous mode */
  LL_ADC_ContinuousModeEnable(ADC);
  
  LL_ADC_SetOverrunDF(ADC, LL_ADC_NEW_DATA_IS_KEPT);
  
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



typedef struct 
{
  int32_t Z; 
  int32_t oldOut; 
  int32_t oldIn; 
}HP_FilterState_TypeDef;

HP_FilterState_TypeDef HP_Filter;

#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))


/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/

/**
* @brief  DMA Transfer Complete Callback.
* @param  None
* @retval None
*/
static void TC_IT_Callback(void)
{
  uint16_t nBytes, nPCMs = 0;
  
  if(!discard_first_flow)
    for ( nBytes = 0; nBytes< PCM_BUFFER_SIZE; nBytes++)
    {
      if((nBytes%2)==0) {
        HP_Filter.Z = PCM_Buffer[nPCMs + PCM_BUFFER_SIZE/2];
        HP_Filter.oldOut = (0xFC * (HP_Filter.oldOut +  HP_Filter.Z - HP_Filter.oldIn)) >> 8;
        HP_Filter.oldIn = HP_Filter.Z;
        PCM_Buffer[nPCMs + PCM_BUFFER_SIZE/2] = SaturaLH(HP_Filter.oldOut, -32760, 32760);
        nPCMs++;
      }
    }
  
  /* Send the PDM data through UART */
  if(!discard_first_flow) {
    start_sending_data = 1;
    DMA_Rearm(LL_DMA_CHANNEL_1, (uint32_t)&(((uint8_t *)PCM_Buffer)[PCM_BUFFER_SIZE]), PCM_BUFFER_SIZE);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  }
  else
    discard_first_flow--;
}

/**
* @brief  DMA Half Transfer Callback.
* @param  None
* @retval None
*/
static void HT_IT_Callback(void)
{
  uint16_t nBytes, nPCMs = 0;
  
  if(!discard_first_flow) {
    for ( nBytes = 0; nBytes<PCM_BUFFER_SIZE; nBytes++)
    {
      if((nBytes%2)==0) {
        HP_Filter.Z = PCM_Buffer[nPCMs] ;
        HP_Filter.oldOut = (0xFC * (HP_Filter.oldOut +  HP_Filter.Z - HP_Filter.oldIn)) >> 8;
        HP_Filter.oldIn = HP_Filter.Z;
        PCM_Buffer[nPCMs] = SaturaLH(HP_Filter.oldOut, -32760, 32760);
        nPCMs++;
      }
    }
    DMA_Rearm(LL_DMA_CHANNEL_1, (uint32_t)&(((uint8_t *)PCM_Buffer)[0]), PCM_BUFFER_SIZE);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  }

}


/**
  * @brief  This function handles DMA channel 1 interrupt request.
  */
void DMA_IRQHandler(void)
{
  /* Check the channel 3 Transfer Complete interrupt */
  if(LL_DMA_IsActiveFlag_TC3(DMA1)) {
    LL_DMA_ClearFlag_TC3(DMA1);
    TC_IT_Callback();
  }
    
  /* Check the channel 3 Half Transfer interrupt */
  if(LL_DMA_IsActiveFlag_HT3(DMA1)) {
    LL_DMA_ClearFlag_HT3(DMA1);
    HT_IT_Callback();
  }
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
