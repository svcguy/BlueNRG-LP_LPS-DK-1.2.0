
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : TIM_Freq_Div_main.c
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
 * @file  TIM_Freq_Div/TIM_Freq_Div_main.c
 * @brief Configuration of the TIM peripheral to generate an output signal with a frequency equal at the half of the input signal.  
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_Freq_Div\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_Freq_Div.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_Freq_Div\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_Freq_Div.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_Freq_Div\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  PIN name  | STEVAL-IDB011V{1|2} | 
------------------------------------
|     A1     |       Not Used      | 
|     A11    |       Not Used      |
|     A12    |       Not Used      |
|     A13    |       Not Used      |
|     A14    |       Not Used      |
|     A15    |       Not Used      |
|     A4     |       TIM1 CH1      |
|     A5     |       Not Used      |
|     A6     |       Not Used      |
|     A7     |       Not Used      |
|     A8     |       USART TX      |
|     A9     |       USART RX      |
|     B0     |       Not Used      |
|     B14    |       TIM1 ETR      |
|     B2     |       Not Used      |
|     B3     |       Not Used      |
|     B4     |       Not Used      |
|     B5     |       Not Used      |
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

* \section Serial_IO Serial I/O
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Stop bits       | 1                | bit       |
@endtable


* \section LEDs_description LEDs description
@table
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |
---------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |
|     U5     |      Not Used      |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |
-------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage

Configuration of the TIM peripheral to generate an output signal with a frequency equal at the half of the input signal.  
This example is based on the BLUENRG_LP TIM LL API. 
The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size). 

TIM1 is configured with the External clock source mode 2 enabled.
The configuration continues setting the channel 1 (TIM peripheral) in PWM (pulse width modulation) mode.
The output signal generated has the frequency equal to the half of the input signal.


Connect the following pins to an oscilloscope to monitor the different waveforms:
- TIM1_CH1 : PA4
- TIM1_ETR : PB14  

  
BlueNRG_LP-EVB Set-up
- Connect the external signal to use as refernce for the test to PB14 pin (TIM1_ETR).
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
#include "TIM_Freq_Div_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
static void MX_GPIO_Init(void);

/* Private user code ---------------------------------------------------------*/
__IO uint8_t ubButtonPress = 0;

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

  /* Peripheral clock enable */
  LL_EnableClock_TIMx();
  LL_EnableClock_TIMx_CH1();
  LL_EnableClock_TIMx_ETR();
  
  LL_GPIO_SetPinMode(TIMx_CH1_PORT, TIMx_CH1_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(TIMx_CH1_PORT, TIMx_CH1_PIN, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(TIMx_CH1_PORT, TIMx_CH1_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_TIMx_CH1();
  
  LL_GPIO_SetPinMode(TIMx_ETR_PORT, TIMx_ETR_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(TIMx_ETR_PORT, TIMx_ETR_PIN, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(TIMx_ETR_PORT, TIMx_ETR_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_TIMx_ETR();
   
  /* As no filter is needed in this example, write ETF[3:0]=0000 in the TIMx_SMCR register. */
  /* Set the prescaler by writing ETPS[1:0]=01 in the TIMx_SMCR register */
  /* Select rising edge detection on the ETR pin by writing ETP=0 in the TIMx_SMCR register */
  LL_TIM_ConfigETR(TIMx, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
  
  /* Enable external clock mode 2 by writing ECE=1 in the TIMx_SMCR register. */
  LL_TIM_SetClockSource(TIMx, LL_TIM_CLOCKSOURCE_EXT_MODE2);
  
  /* Enable counter. Note that the counter will stop automatically at the     */
  /* next update event (UEV).   */
  
  /*********************************/
  /* Output waveform configuration */
  /*********************************/
  /* Select counter mode: counting up */
  LL_TIM_SetCounterMode(TIMx, LL_TIM_COUNTERMODE_UP);
  
  /* CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1)  */
  /*                               PSC[15:0] = 0   */ 
  /* CK_CNT is equal to fCK_PSC / (    0     + 1)  */ 
  /* CK_CNT is equal to fCK_PSC / (     1       )  */
  LL_TIM_SetPrescaler(TIMx, 0x00);
  
  /* Set the compare register */
  LL_TIM_OC_SetCompareCH1(TIMx, 0x1);
  
  /* Enable TIMx_CCR1 register preload. Read/Write operations access the      */
  /* preload register. TIMx_CCR1 preload value is loaded in the active        */
  /* at each update event.                                                    */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  
  /* Set the autoreload register to get a pulse length of 3s (3000000 us)*/
  LL_TIM_SetAutoReload(TIMx, 0x1);
  
  /* Enable auto-reload register preload */
  LL_TIM_EnableARRPreload(TIMx);
  
  /* Set output channel 1 */
  LL_TIM_OC_SetMode(TIMx,  LL_TIM_CHANNEL_CH1,  LL_TIM_OCMODE_PWM1);
  
  /* Configure output channel 1 configuration */
  LL_TIM_OC_ConfigOutput(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH | LL_TIM_OCIDLESTATE_LOW);
  
  /* Enable channel 1 */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
  
  /* Enable TIMx outputs */
  LL_TIM_EnableAllOutputs(TIMx);
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIMx);
  
  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIMx);

  printf("TIM_Freq_Div\n\r");
  
  /* Infinite loop */
  while (1)
  {
  }
}

static void LL_Init(void)
{
  /* System interrupt init */
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
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
  LED2_GPIO_CLK_ENABLE();
  
  /* Configure GPIO for LED */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
  GPIO_InitStruct.Pin = LED2_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
  
  /* Configure NVIC for SysTick_IRQn */
  NVIC_SetPriority(SysTick_IRQn, IRQ_LOW_PRIORITY);
}


/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/

/**
* @brief  Turn-on LED2.
* @param  None
* @retval None
*/
void LED_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
* @brief  Turn-off LED2.
* @param  None
* @retval None
*/
void LED_Off(void)
{
  /* Turn LED2 off */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
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
void LED_Blinking(uint32_t Period)
{
  /* Toggle LED2 in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(Period);
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



