
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : ADC_ConvSequ_main.c
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
 * @file  ADC_ConvSequ/ADC_ConvSequ_main.c
 * @brief This example shows how to use the sequencer of the ADC peripheral in order to
 * acquire samples from different sources. 
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\ADC\\ADC_ConvSequ\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\ADC_ConvSequ.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\ADC\\ADC_ConvSequ\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\ADC_ConvSequ.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\ADC\\ADC_ConvSequ\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
- \c STEVAL-IDB012V1



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
|  PIN name  | STEVAL-IDB011V{1|2} |   STEVAL-IDB012V1  |
--------------------------------------------------------
|     A1     |       Not Used      |      USART TX      |
|     A11    |       Not Used      |      Not Used      |
|     A12    |       Not Used      |        N.A.        |
|     A13    |       Not Used      |        N.A.        |
|     A14    |       Not Used      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |       Not Used      |        N.A.        |
|     A5     |       Not Used      |        N.A.        |
|     A6     |          U5         |      Not Used      |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       USART TX      |      Not Used      |
|     A9     |       USART RX      |      Not Used      |
|     B0     |       Not Used      |      USART RX      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       ADC_VINP0     |      ADC_VINP0     |
|     B4     |       Not Used      |        DL2         |
|     B5     |       Not Used      |      Not Used      |
|     B7     |       Not Used      |      Not Used      |
|     B8     |         DL2         |        N.A.        |
|     B9     |       Not Used      |        N.A.        |
|     A0     |         N.A.        |      Not Used      |
|     A10    |         N.A.        |      Not Used      |
|     B1     |         N.A.        |      Not Used      |
|     B6     |         N.A.        |      Not Used      |
|     B15    |         N.A.        |      Not Used      |
|     GND    |       Not Used      |      Not Used      |
|     RST    |       Not Used      |      Not Used      |
|    VBAT    |       Not Used      |      Not Used      |
@endtable 

@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Start bits      | 1                | bit       |
| Stop bits       | 1                | bit       |
| HW flow control | None             | bit       |
@endtable

* \section LEDs_description LEDs description
@table
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |
|     U5     |      Not Used      |      Not Used      |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage

This example shows how to use the sequencer of the ADC peripheral in order to
acquire samples from different sources.
In the example, a sequence of four values is selected.
The DMA peripheral is used to transfer the data from the output data register of
the ADC to the RAM memory.
This example is driven by polling the following events:
 - Transfer Complete (DMA)
 - Half Transfer (DMA)
 - End Of Sequence of conversion (ADC)
 - Overrun of Down Sampler (ADC)
 - Transfer Error (DMA).
This example is based on the BLUENRG_LP ADC HAL API.
for optimization purposes (performance and size).

Example configuration:
The ADC is configured to sample from four different sources by using
the sequencer of the ADC. The internal path of the Down Sampler is used.
The sources are:
1) VINP0 to single positive input (PB3).
4) Battery level detector.
4) Temperature sensor.
The USART baudrate is set to 115200 bps.

Example execution:
The user must just load and then run the application.
The user can get the output data through a serial terminal program.

Connection needed:
None.

Other peripherals used:
  1 USART for output the data acquired

**/
   

/* Includes ------------------------------------------------------------------*/
#include "rf_driver_hal.h"
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "bluenrg_lp_evb_config.h"
#endif
#include "ADC_DMA_main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#if defined(CONFIG_DEVICE_BLUENRG_LP)
#define USER_SAMPLERATE       (ADC_SAMPLE_RATE_28)
#endif
#if defined(CONFIG_DEVICE_BLUENRG_LPS)
#define USER_SAMPLERATE_MSB       (LL_ADC_SAMPLE_RATE_MSB_4)
#define USER_SAMPLERATE           (LL_ADC_SAMPLE_RATE_0)
#endif

#define USER_DATAWIDTH        (ADC_DS_DATA_WIDTH_16_BIT)
#define USER_RATIO            (ADC_DS_RATIO_128)

/* Private macro -------------------------------------------------------------*/
#define PRINT_INT(x)    ((int)(x))
#define PRINT_FLOAT(x)  (x>0)? ((int) (((x) - PRINT_INT(x)) * 1000)) : (-1*(((int) (((x) - PRINT_INT(x)) * 1000))))
#define ADC_DMA_BUF_LEN (16+16+16)

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef adc_handle;
ADC_ConfigChannelTypeDef xChannel;
DMA_HandleTypeDef hdma_adc;
uint16_t ADC_DMA_buffer[ADC_DMA_BUF_LEN];
int8_t offset_vinp0 = 0;

/* Private function prototypes -----------------------------------------------*/
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);

/* Private user code ---------------------------------------------------------*/


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, RADIO_SYSCLK_NONE) != SUCCESS)
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
  
  /* Initialization of LEDs */
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);
  
  /* Configure ADC and DMA */
  MX_ADC_Init();
  MX_DMA_Init();
   
  printf("ADC average value of\r\nPin Voltage\tBattery Voltage\tTemperature\r\n");
    
  /* Start ADC-DMA conversion */
   if (HAL_ADC_Start_DMA(&adc_handle, (uint32_t *)ADC_DMA_buffer, ADC_DMA_BUF_LEN) != HAL_OK) {
    Error_Handler();
   }
  
  /* Infinite loop */
  while (1) {
    /* Nothing to do */
  }
}

  /* @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{
  /* Enable the ADC peripheral */
  HAL_ADC_StructInit(&adc_handle);
  adc_handle.Init.DataRatio = USER_RATIO;
  adc_handle.Init.DataWidth = USER_DATAWIDTH;
  adc_handle.Init.SampleRate = USER_SAMPLERATE;
#if defined(CONFIG_DEVICE_BLUENRG_LPS)
  adc_handle.Init.SampleRateMsb = USER_SAMPLERATE_MSB;
#endif
  adc_handle.Init.SamplingMode = ADC_SAMPLING_AT_START;
  adc_handle.Init.SequenceLength = ADC_SEQ_LEN_03;
  adc_handle.DMA_Handle = &hdma_adc;
  
  if (HAL_ADC_Init(&adc_handle) != HAL_OK) {
    Error_Handler();
  }

  /* Start the sampling at end of previous sample */
  LL_ADC_InputSamplingMode(ADC, LL_ADC_SAMPLING_AT_END);
  
  /* Set the input channel */
  xChannel.ChannelType = ADC_CH_VINP0_TO_SINGLE_POSITIVE_INPUT;
  xChannel.SequenceNumber = ADC_SEQ_POS_01;
  xChannel.VoltRange = ADC_VIN_RANGE_3V6;
  if (HAL_ADC_ConfigChannel(&adc_handle, &xChannel)!= HAL_OK) {
    Error_Handler();
  }

  /* Set the GAIN */
  if(LL_ADC_GET_CALIB_GAIN_FOR_VINPX_3V6() != 0xFFF) {
    LL_ADC_SetCalibPoint1Gain(ADC, LL_ADC_GET_CALIB_GAIN_FOR_VINPX_3V6() );
    
    offset_vinp0 = LL_ADC_GET_CALIB_OFFSET_FOR_VINPX_3V6();
#ifdef CONFIG_DEVICE_BLUENRG_LP
    if(offset_vinp0 < -64 || offset_vinp0 > 63) {
      LL_ADC_SetCalibPoint1Offset(ADC, 0);
    }
    else
#endif
    {
      LL_ADC_SetCalibPoint1Offset(ADC, offset_vinp0);
      offset_vinp0 = 0;
    }
  }
  else {
    LL_ADC_SetCalibPoint1Gain(ADC, LL_ADC_DEFAULT_RANGE_VALUE_3V6);
  }
  LL_ADC_SetCalibPointForSinglePos3V6(ADC, LL_ADC_CALIB_POINT_1);
  
  /* Set the input channe2 */
  xChannel.ChannelType = ADC_CH_BATTERY_LEVEL_DETECTOR;
  xChannel.SequenceNumber = ADC_SEQ_POS_02;
  if (HAL_ADC_ConfigChannel(&adc_handle, &xChannel)!= HAL_OK) {
    Error_Handler();
  }

#if defined(CONFIG_DEVICE_BLUENRG_LP)
  /* Enable the temperature sensor */
  LL_PWR_EnableTempSens();
#endif
  
  /* Set the input channe2 */
  xChannel.ChannelType = ADC_CH_TEMPERATURE_SENSOR;
  xChannel.SequenceNumber = ADC_SEQ_POS_03;
  xChannel.VoltRange = ADC_VIN_RANGE_1V2;
  if (HAL_ADC_ConfigChannel(&adc_handle, &xChannel)!= HAL_OK) {
    Error_Handler();
  }
  if(LL_ADC_GET_CALIB_GAIN_FOR_VINPX_1V2() != 0xFFF) {
    LL_ADC_SetCalibPoint2Gain(ADC, LL_ADC_GET_CALIB_GAIN_FOR_VINPX_1V2() );
  }
  else {
    LL_ADC_SetCalibPoint2Gain(ADC, LL_ADC_DEFAULT_RANGE_VALUE_1V2);
  }
  LL_ADC_SetCalibPointForSinglePos1V2(ADC, LL_ADC_CALIB_POINT_2);
  
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA_CLK_ENABLE();

  /* Configure NVIC for DMA */
  HAL_NVIC_SetPriority(DMA_IRQn, IRQ_HIGH_PRIORITY);
  HAL_NVIC_EnableIRQ(DMA_IRQn);

}


/**
  * @brief  Conversion complete callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  HAL_ADC_Stop_DMA(&adc_handle);

  uint8_t j =0;
  float vpin = 0.0, vbattery = 0.0;
  float temp = 0.0;
  
  for (int i=0; i<ADC_DMA_BUF_LEN/3; i++) {
    vpin += LL_ADC_GetADCConvertedValueSingle(ADC, ADC_DMA_buffer[j], LL_ADC_VIN_RANGE_3V6, USER_DATAWIDTH, offset_vinp0);
    vbattery += LL_ADC_GetADCConvertedValueBatt(ADC, ADC_DMA_buffer[j+1], USER_DATAWIDTH, 0);
    temp += LL_ADC_GetADCConvertedValueTemp(ADC, ADC_DMA_buffer[j+2], USER_DATAWIDTH)/100.0;
    j+=3;
  }
  vpin /= ADC_DMA_BUF_LEN/3;
  vbattery /= ADC_DMA_BUF_LEN/3;
  temp /= ADC_DMA_BUF_LEN/3;
  printf("%d.%03d mV\t%d.%03d mV\t%d.%02d %cC\r\n", PRINT_INT(vpin),PRINT_FLOAT(vpin), PRINT_INT(vbattery),PRINT_FLOAT(vbattery), PRINT_INT(temp),PRINT_FLOAT(temp), 248);

#if defined(CONFIG_DEVICE_BLUENRG_LP)
  /* Enable the temperature sensor */
  LL_PWR_EnableTempSens();
#endif

  /* Restart ADC DMA conversion */
  if (HAL_ADC_Start_DMA(&adc_handle, (uint32_t *)ADC_DMA_buffer, ADC_DMA_BUF_LEN) != HAL_OK) {
   Error_Handler();
  }
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
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



