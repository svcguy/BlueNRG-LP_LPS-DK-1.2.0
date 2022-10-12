
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : TIM_Input_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the TIM functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  TIM_Input/TIM_Input_main.c
 * @brief Use of the TIM peripheral to measure a periodic signal frequency 
 * provided either by an external signal generator or by 
 * another timer instance. 
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_Input\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_Input.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_Input\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_Input.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_Input\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A4     |       TIM1 CH1      |        N.A.        |
|     A5     |       Not Used      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       USART TX      |      Not Used      |
|     A9     |       USART RX      |      Not Used      |
|     B0     |       Not Used      |      USART RX      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |      TIM17 CH1     |
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


* \section LEDs_description LEDs description
@table
|  LED name  |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |     STEVAL-IDB012V1    |
--------------------------------------------------------------------------------------------
|     DL1    |        Not Used        |        Not Used        |        Not Used        |
|     DL2    |  Slow blinking: error  |  Slow blinking: error  |  Slow blinking: error  |
|     DL3    |        Not Used        |        Not Used        |        Not Used        |
|     DL4    |        Not Used        |        Not Used        |        Not Used        |
|     U5     |        Not Used        |        Not Used        |        Not Used        |

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

Use of the TIM peripheral to measure a periodic signal frequency provided either by an external signal generator or by another timer instance. 
This example is based on the BLUENRG_LP TIM LL API. 
The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size).
  
TIM1_CH1 is configured in input capture mode. The TIM1CLK frequency is set to its maximum value (Prescaler is /2 * 2 = 1) in order to get the best possible resolution.

The "uwMeasuredFrequency" variable contains the measured signal frequency.
It is calculated within the capture/compare 1 interrupt service routine.

The minimum frequency value to measure is TIM1 counter clock / TIMx_CCR1 MAX
                                              = 64 MHz / 65535
                                              
Due to TIM1 interrupt routine processing time (around 1us), the maximum frequency value to measure is around 1 MHz.
  
BlueNRG_LP-EVB Set-up
- TIM1_CH1   PA4  the external signal must be connected to TIM1 Channel1 used as input pin.
Connect USART1 TX/RX to respectively RX and TX pins of PC UART (could be done through a USB to UART adapter) :
- Connect BlueNRG_LP board USART1 TX pin to PC COM port RX signal
- Connect BlueNRG_LP board USART1 RX pin to PC COM port TX signal
- Connect BlueNRG_LP board GND to PC COM port GND signal

Launch serial communication SW on PC (as HyperTerminal or TeraTerm) with proper configuration :
- 115200 bauds
- 8 bits data
- 1 start bit
- 1 stop bit
- no parity
- no HW flow control 
**/
   
/* Includes ------------------------------------------------------------------*/
#include "TIM_Input_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Number of frequencies */
#define TIM_FREQUENCIES_NB 10

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Measured frequency */
__IO uint32_t uwMeasuredFrequency = 0;

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
static void MX_GPIO_Init(void);
static void MX_TIMx_Init(void);
__STATIC_INLINE void LED_Blinking(uint32_t Period);

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
  LL_Init();

  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);
  
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
      
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIMx_Init();
  
  /**************************/
  /* TIMx interrupts set-up */
  /**************************/
  /* Enable the capture/compare interrupt for channel 1 */
  LL_TIM_EnableIT_CC1(TIMx);
  
  /***********************/
  /* Start input capture */
  /***********************/
  /* Enable output channel 1 */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIMx);
    
  /* Infinite loop */
  while (1)
  {
    LL_mDelay(500);
    printf("uwMeasuredFrequency = %d\n\r", uwMeasuredFrequency);
  }
}

static void LL_Init(void)
{
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
}


/**
* @brief TIMx Initialization Function
* @param None
* @retval None
*/
static void MX_TIMx_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* Peripheral clock enable */
  LL_EnableClock_TIMx(); 
  LL_EnableClock_TIMx_CH1();
  
  GPIO_InitStruct.Pin = TIMx_CH1_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = TIMx_CH1_AF;
  LL_GPIO_Init(TIMx_CH1_PORT, &GPIO_InitStruct);
  
  /* TIMx interrupt Init */
  NVIC_SetPriority(TIMx_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(TIMx_IRQn);
  
  /* Configure the TIMx time base unit */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0xFFFF;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIMx, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIMx);
  
  /* Configure the channel 1 */
  LL_TIM_IC_SetActiveInput(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
  
  /* Configure GPIO for LED */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
  GPIO_InitStruct.Pin = LED2_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
}


/**
* @brief  Set LED2 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
* @param  Period : Period of time (in ms) between each toggling of LED
*   This parameter can be user defined values. Pre-defined values used in that example are :
*     @arg LED_BLINK_FAST : Fast Blinking
*     @arg LED_BLINK_SLOW : Slow Blinking
*     @arg LED_BLINK_ERROR : Error specific Blinking
* @retval None
*/
__STATIC_INLINE void LED_Blinking(uint32_t Period)
{
  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);  
    LL_mDelay(Period*10);
  }
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/

/**
* @brief  Timer capture/compare interrupt processing
* @note TIMx input capture module is used to capture the value of the counter
*       after a transition is detected by the corresponding input channel.
* @param  None
* @retval None
*/
void TimerCaptureCompare_Callback(void)
{
  /* Capture index */
  static uint16_t uhCaptureIndex = 0;
  
  /* Captured Values */
  static uint32_t uwICValue1 = 0;
  static uint32_t uwICValue2 = 0;
  static uint32_t uwDiffCapture = 0;
  
  uint32_t TIMxCLK;
  uint32_t PSC;
  uint32_t IC1PSC;
  uint32_t IC1Polarity;
  
  if(uhCaptureIndex == 0)
  {
    /* Get the 1st Input Capture value */
    uwICValue1 = LL_TIM_IC_GetCaptureCH1(TIMx);
    uhCaptureIndex = 1;
  }
  else if(uhCaptureIndex == 1)
  {
    /* Get the 2nd Input Capture value */
    uwICValue2 = LL_TIM_IC_GetCaptureCH1(TIMx); 
    
    /* Capture computation */
    if (uwICValue2 > uwICValue1)
    {
      uwDiffCapture = (uwICValue2 - uwICValue1); 
    }
    else if (uwICValue2 < uwICValue1)
    {
      uwDiffCapture = ((TIMx_ARR_MAX - uwICValue1) + uwICValue2) + 1; 
    }
    else
    {
      /* If capture values are equal, we have reached the limit of frequency  */
      /* measures.                                                            */
      LED_Blinking(LED_BLINK_ERROR);
    }
    
    /* The signal frequency is calculated as follows:                         */      
    /* Frequency = (TIMx*IC1PSC) / (Capture*(PSC+1)*IC1Polarity)           */
    /* where:                                                                 */                                                          
    /*  Capture is the difference between two consecutive captures            */
    /*  TIMxCLK is the timer counter clock frequency                           */
    /*  PSC is the timer prescaler value                                      */
    /*  IC1PSC is the input capture prescaler value                           */
    /*  IC1Polarity value depends on the capture sensitivity:                 */
    /*    1 if the input is sensitive to rising or falling edges              */
    /*    2 if the input is sensitive to both rising and falling edges        */
    
    /* Retrieve actual TIMx counter clock frequency */
    TIMxCLK = LL_TIM_GetPeriphClock(TIMx);
    
    /* Retrieve actual TIMx prescaler value */
    PSC = LL_TIM_GetPrescaler(TIMx);
    
    /* Retrieve actual IC1 prescaler ratio */
    IC1PSC = __LL_TIM_GET_ICPSC_RATIO(LL_TIM_IC_GetPrescaler(TIMx, LL_TIM_CHANNEL_CH1));
    
    /* Retrieve actual IC1 polarity setting */
    if (LL_TIM_IC_GetPolarity(TIMx, LL_TIM_CHANNEL_CH1) == LL_TIM_IC_POLARITY_BOTHEDGE)
      IC1Polarity = 2;
    else
      IC1Polarity = 1;
    
    /* Calculate input signal frequency */
    uwMeasuredFrequency = (TIMxCLK *IC1PSC) / (uwDiffCapture*(PSC+1)*IC1Polarity);
    
    /* reset capture index */
    uhCaptureIndex = 0;    
  }
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  LED_Blinking(LED_BLINK_ERROR);
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
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



