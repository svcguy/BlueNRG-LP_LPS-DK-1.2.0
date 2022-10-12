
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : ADC_ConvSequ_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 15-April-2019
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
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\ADC\\ADC_ConvSequ\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\ADC_ConvSequ.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\ADC\\ADC_ConvSequ\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\ADC_ConvSequ.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\ADC\\ADC_ConvSequ\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     B2     |       ADC_VINM0     |      ADC_VINM0     |
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

* \section Serial_IO Serial I/O
  The application will listen for keys typed and it will send back in the serial port.
  In other words everything typed in serial port will be send back.
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
|     DL2    |    Error detect    |    Error detect    |    Error detect    |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |
|     U5     |    Activity LED    |    Activity LED    |    Activity LED    |

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


This example shows how to use the sequencer of the ADC peripheral in order to acquire samples from different sources.
In the example, a sequence of four values is selected.
The data are sampled printed out by using the USART peripheral with a rate of 1 sample each 200 ms.
The DMA peripheral is used to transfer the data from the output data register of the ADC to the RAM memory.
This example is driven by polling the following events:
 - Transfer Complete (DMA)
 - Half Transfer (DMA)
 - End Of Sequence of conversion (ADC)
 - Overrun of Down Sampler (ADC)
 - Transfer Error (DMA).
This example is based on the BLUENRG_LP ADC LL API.
The peripheral initialization is done using LL unitary service functions for optimization purposes (performance and size).

Example configuration:
The ADC is configured to sample from four different sources by using the sequencer of the ADC. The internal path of the Down Sampler is used.
The sources are:
1) VINP0 to single positive input
2) VINM0 to single negative input
3) VINP0-VINM0 to differential input
4) Battery level detector.


Example execution:
The user must just load and then run the application.
The user can get the output data through a serial terminal program.

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
#include "rf_driver_ll_gpio.h"
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "bluenrg_lpx.h"
#include "bluenrg_lp_evb_config.h"
#endif
#if defined(USE_FULL_ASSERT)
#include "rf_driver_assert.h"
#endif /* USE_FULL_ASSERT */


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#if defined(CONFIG_DEVICE_BLUENRG_LP)
#define USER_SAMPLERATE       (LL_ADC_SAMPLE_RATE_28)
#endif
#if defined(CONFIG_DEVICE_BLUENRG_LPS)
#define USER_SAMPLERATE_MSB       (LL_ADC_SAMPLE_RATE_MSB_4)
#define USER_SAMPLERATE           (LL_ADC_SAMPLE_RATE_0)
#endif

#define USER_DATAWIDTH        (LL_ADC_DS_DATA_WIDTH_16_BIT)
#define USER_RATIO            (LL_ADC_DS_RATIO_128)

#define BUFFER_SIZE           (5)

/* Private macro -------------------------------------------------------------*/
#define PRINT_INT(x)    ((int)(x))
#define PRINT_FLOAT(x)  (x>0)? ((int) (((x) - PRINT_INT(x)) * 1000)) : (-1*(((int) (((x) - PRINT_INT(x)) * 1000))))

/* Private variables ---------------------------------------------------------*/
static uint16_t aDST_Buffer[BUFFER_SIZE];
int8_t offset_vinp0 = 0;
int8_t offset_vinm0 = 0;

/* Private function prototypes -----------------------------------------------*/
static void APP_ADC_Init(void);
static void APP_DMA_Init(void);
static void BSP_Init(void);

/* Private user code ---------------------------------------------------------*/


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  float adcValue = 0.0;
    
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, RADIO_SYSCLK_NONE) != SUCCESS)
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }

  /* BSP Init */
  BSP_Init();

  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);

  /* Initialize all configured peripherals */
  APP_ADC_Init();
  APP_DMA_Init();
  
  /* Set DMA transfer addresses of source and destination */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
                         LL_ADC_GetOutputDataRegDS(ADC),
                         (uint32_t)&aDST_Buffer,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* Set DMA transfer size */
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, BUFFER_SIZE);

  /* Enable DMA transfer complete/error interrupts */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

  /* Start the DMA transfer */
  LL_ADC_DMAModeDSEnable(ADC);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  
  /* Start ADC conversion */
  LL_ADC_StartConversion(ADC);

  /* Infinite loop */
  while (1) {

    /* Check the ADC flag End Of Sequence of conversion */
    if( LL_ADC_IsActiveFlag_EOS(ADC) == 1) {
      /* Clear the ADC flag End Of Sequence of conversion */
      LL_ADC_ClearFlag_EOS(ADC);
    }
    
    /* Check the DMA flag Transfer Complete on channel 1 */
    if( LL_DMA_IsActiveFlag_TC1(DMA1) == 1) {
      LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
      
      /* Printout the output value */
      adcValue = (float)LL_ADC_GetADCConvertedValueSingle(ADC, aDST_Buffer[0], LL_ADC_VIN_RANGE_3V6, USER_DATAWIDTH, offset_vinp0);
      printf("ADC pin 1 %d.%03d mV\r\n", PRINT_INT(adcValue),PRINT_FLOAT(adcValue));
      adcValue = (float)LL_ADC_GetADCConvertedValueSingle(ADC, aDST_Buffer[1], LL_ADC_VIN_RANGE_3V6, USER_DATAWIDTH, offset_vinm0);
      printf("ADC pin 2 %d.%03d mV\r\n", PRINT_INT(adcValue),PRINT_FLOAT(adcValue));
      adcValue = (float)LL_ADC_GetADCConvertedValueDiff(ADC, aDST_Buffer[2], LL_ADC_VIN_RANGE_3V6, USER_DATAWIDTH, 0);
      printf("ADC differential %d.%03d mV\r\n", PRINT_INT(adcValue),PRINT_FLOAT(adcValue));
      adcValue = (float)LL_ADC_GetADCConvertedValueBatt(ADC, aDST_Buffer[3], USER_DATAWIDTH, 0);
      printf("Battery voltage %d.%03d mV\r\n", PRINT_INT(adcValue),PRINT_FLOAT(adcValue));
      adcValue = LL_ADC_GetADCConvertedValueTemp(ADC, aDST_Buffer[4], USER_DATAWIDTH)/100.0;
      printf("Temperature %d.%02d %cC\r\n\n", PRINT_INT(adcValue),PRINT_FLOAT(adcValue), 248);

      /* Clear the DMA flag Transfer Complete on channel 1 */
      LL_DMA_ClearFlag_TC1(DMA1);
      
      /* Toggle the conversion/activity LED */
      BSP_LED_Toggle(BSP_LED1);
      
      /* Add 100 ms of delay between each sequence of measurement */
      LL_mDelay(200);
      
      /* Restart DMA channel */
      LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&aDST_Buffer);
      LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, BUFFER_SIZE);
      LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
      
      /* Restart ADC conversion */
      LL_ADC_StartConversion(ADC);
    }
   
    /* Check the ADC flag overrun of Down Sampler */
    if( LL_ADC_IsActiveFlag_OVRDS(ADC) == 1) {
      
      /* Clear the ADC flag overrun of Down Sampler */
      LL_ADC_ClearFlag_OVRDS(ADC);

      /* Turn on the LED2 if overrun occurs */
      BSP_LED_On(BSP_LED2);
    }

    /* Check the DMA flag Transfer Error on channel 1 */
    if( LL_DMA_IsActiveFlag_TE1(DMA1) == 1) {
      
      /* Clear the DMA flag Transfer Error on channel 1 */
      LL_ADC_ClearFlag_OVRDS(ADC);

      /* Turn on the LED2 if transfer error occurs */
      BSP_LED_On(BSP_LED2);
    }
  }
}


static void BSP_Init(void)
{
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  LL_GPIO_InitTypeDef  gpioinitstruct = {0};
  LL_GPIO_StructInit(&gpioinitstruct);
  
  /* Initialization of LEDs */
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);
  
  /* Initialization of the ADC pins */
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
  gpioinitstruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
  gpioinitstruct.Mode = LL_GPIO_MODE_ANALOG;
  gpioinitstruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpioinitstruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;  
  gpioinitstruct.Pull = LL_GPIO_PULL_NO;  
  LL_GPIO_Init(GPIOB, &gpioinitstruct);  
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
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
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  LL_ADC_LDOEnable(ADC);
#endif

  /* Enable the ADC */
  LL_ADC_Enable(ADC);

  /* Configure the sample rate */
#if defined(CONFIG_DEVICE_BLUENRG_LP)
  LL_ADC_SetSampleRate(ADC, USER_SAMPLERATE);
#endif
#if defined(CONFIG_DEVICE_BLUENRG_LPS)
  LL_ADC_SetSampleRate(ADC, USER_SAMPLERATE_MSB, USER_SAMPLERATE);
#endif

  /* Configure the Down Sampler data width and ratio */
  LL_ADC_ConfigureDSDataOutput(ADC, USER_DATAWIDTH, USER_RATIO);

  /* Configure the operation mode as ADC mode (static/low frequency signal) */
  LL_ADC_SetADCMode(ADC, LL_ADC_OP_MODE_ADC);

  /* Start the sampling at end of previous sample */
  LL_ADC_InputSamplingMode(ADC, LL_ADC_SAMPLING_AT_END);

  /* Set the length of the conversion sequence as 5 */
  LL_ADC_SetSequenceLength(ADC, LL_ADC_SEQ_LEN_05);
  
  /* Set the 1st entry of the input sequence as VINP0 */
  LL_ADC_SetChannelSeq0(ADC, LL_ADC_CH_VINP0_TO_SINGLE_POSITIVE_INPUT);
  LL_ADC_SetVoltageRangeSingleVinp0(ADC, LL_ADC_VIN_RANGE_3V6);
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

  /* Set the 2nd entry of the input sequence as VINM0 */
  LL_ADC_SetChannelSeq1(ADC, LL_ADC_CH_VINM0_TO_SINGLE_NEGATIVE_INPUT);
  LL_ADC_SetVoltageRangeSingleVinm0(ADC, LL_ADC_VIN_RANGE_3V6);
  if(LL_ADC_GET_CALIB_GAIN_FOR_VINMX_3V6() != 0xFFF) {
    LL_ADC_SetCalibPoint2Gain(ADC, LL_ADC_GET_CALIB_GAIN_FOR_VINMX_3V6() );

    offset_vinm0 = LL_ADC_GET_CALIB_OFFSET_FOR_VINMX_3V6();
#ifdef CONFIG_DEVICE_BLUENRG_LP
    if(offset_vinm0 < -64 || offset_vinm0 > 63) {
      LL_ADC_SetCalibPoint2Offset(ADC, 0);
    }
    else
#endif
    {
      LL_ADC_SetCalibPoint2Offset(ADC, offset_vinm0);
      offset_vinm0 = 0;
    }
  }
  else {
    LL_ADC_SetCalibPoint2Gain(ADC, LL_ADC_DEFAULT_RANGE_VALUE_3V6);
  }
  LL_ADC_SetCalibPointForSingleNeg3V6(ADC, LL_ADC_CALIB_POINT_2);
  
  /* Set the 3rd entry of the input sequence as differential (VINP0-VINM0) */
  LL_ADC_SetChannelSeq2(ADC, LL_ADC_CH_VINP0_VINM0_TO_DIFF_INPUT);
  LL_ADC_SetVoltageRangeDiffVinp0Vinm0(ADC, LL_ADC_VIN_RANGE_3V6);
  if(LL_ADC_GET_CALIB_GAIN_FOR_VINDIFF_3V6() != 0xFFF) {
    LL_ADC_SetCalibPoint3Gain(ADC, LL_ADC_GET_CALIB_GAIN_FOR_VINDIFF_3V6() );
    LL_ADC_SetCalibPoint3Offset(ADC, LL_ADC_GET_CALIB_OFFSET_FOR_VINDIFF_3V6() );
  }
  else {
    LL_ADC_SetCalibPoint3Gain(ADC, LL_ADC_DEFAULT_RANGE_VALUE_3V6);
  }
  LL_ADC_SetCalibPointForDiff3V6(ADC, LL_ADC_CALIB_POINT_3);
  
  /* Set the 4th entry of the input sequence as battery level detector */
  LL_ADC_SetChannelSeq3(ADC, LL_ADC_CH_BATTERY_LEVEL_DETECTOR);
  
  /* Set the 5th entry of the input sequence as temperature sensor */
  LL_ADC_SetChannelSeq4(ADC, LL_ADC_CH_TEMPERATURE_SENSOR);
  
#ifdef CONFIG_DEVICE_BLUENRG_LP
  /* Enable the temperature sensor */
  LL_PWR_EnableTempSens();
#endif
  
  if(LL_ADC_GET_CALIB_GAIN_FOR_VINPX_1V2() != 0xFFF) {
    LL_ADC_SetCalibPoint4Gain(ADC, LL_ADC_GET_CALIB_GAIN_FOR_VINPX_1V2() );
  }
  else {
    LL_ADC_SetCalibPoint4Gain(ADC, LL_ADC_DEFAULT_RANGE_VALUE_1V2);
  }
  LL_ADC_SetCalibPointForSinglePos1V2(ADC, LL_ADC_CALIB_POINT_4);
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

  /* Set the DMA channel 1 with the ADC Down Sampler output */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC_DS);

  /* Set transfer direction from ADC DS peripheral to RAM memory */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* Set priority level */
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);

  /* Set DMA mode */
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

  /* Set peripheral increment mode to no increment */
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  /* Set memory increment mode to increment */
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  /* Set peripheral data width to 16-bit */
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  /* Set memory data width to 16-bit */
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

}


/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/



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
