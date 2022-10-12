
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : TIM_BreakAndDeadtime_main.c
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
 * @file  TIM_BreakAndDeadtime/TIM_BreakAndDeadtime_main.c
 * @brief Configuration of the TIM peripheral to generate three center-aligned PWM and complementary 
 * PWM signals, insert a defined deadtime value, use the break feature, and lock the break 
 * and dead-time configuration.

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_BreakAndDeadtime\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_BreakAndDeadtime.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_BreakAndDeadtime\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_BreakAndDeadtime.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_BreakAndDeadtime\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A1     |       Not Used      |      Not Used      |
|     A11    |       Not Used      |      Not Used      |
|     A12    |       Not Used      |        N.A.        |
|     A13    |       Not Used      |        N.A.        |
|     A14    |       TIM1 BKIN     |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |       TIM1 CH1      |        N.A.        |
|     A5     |       TIM1 CH2      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       USART TX      |      Not Used      |
|     A9     |       USART RX      |      Not Used      |
|     B0     |       TIM1 CH2N     |      Not Used      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |      TIM17 CH1     |
|     B4     |       Not Used      |      Not Used      |
|     B5     |       Not Used      |      TIM17 BKIN    |
|     B7     |       Not Used      |      Not Used      |
|     B8     |       Not Used      |        N.A.        |
|     B9     |       TIM1 CH1N     |        N.A.        |
|     A0     |         N.A.        |      Not Used      |
|     A10    |         N.A.        |      TIM17 CH1N    |
|     B1     |         N.A.        |      Not Used      |
|     B6     |         N.A.        |      Not Used      |
|     B15    |         N.A.        |      Not Used      |
|     GND    |       Not Used      |      Not Used      |
|     RST    |       Not Used      |      Not Used      |
|    VBAT    |       Not Used      |      Not Used      |
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
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |
--------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |       Not Used       |
|      PUSH2     |      Not Used      |      Not Used      |       Not Used       |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage


Configuration of the TIM peripheral to generate three center-aligned PWM and complementary PWM signals, insert a defined deadtime value, use the break feature, and lock the break and dead-time configuration.
This example is based on the BLUENRG_LP TIM LL API. 
The peripheral initialization uses LL initialization function to demonstrate LL Init.

The TIM1 auto-reload is set to generate PWM signal at 10 KHz.

The Three Duty cycles are computed as the following description: 
The channel 1 duty cycle is set to 50%   so channel 1N is set to 50%.
The channel 2 duty cycle is set to 25%   so channel 2N is set to 75%.

A dead time equal to 0,1 us is inserted between the different complementary signals, and the Lock level 1 is selected.
- The OCx output signal is the same as the reference signal except for the rising edge, which is delayed relative to the reference rising edge.
- The OCxN output signal is the opposite of the reference signal except for the rising edge, which is delayed relative to the reference falling edge

Note that calculated duty cycles apply to the reference signal (OCxREF) from which outputs OCx and OCxN are generated. 
As dead time insertion is enabled the duty cycle measured on OCx will be slightly lower.

The TIM1 waveforms can be displayed using an oscilloscope.

  
BlueNRG_LP-EVB Set-up
Connect the TIM1 pins to an oscilloscope to monitor the different waveforms:  
- TIM1_CH1N 
- TIM1_CH2N 
- TIM1_CH1  
- TIM1_CH2  
Connect the TIM1 break to the GND. To generate a break event, switch this pin level from 0V to 3.3V.  
- TIM1_BKIN 
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
#include "TIM_BreakAndDeadtime_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint32_t tim_prescaler = 0;
uint32_t tim_period = 0;
uint32_t pulse_value_1 = 0;
uint32_t pulse_value_2 = 0;
uint32_t tim_deadtime = 0;

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
static void MX_GPIO_Init(void);
static void MX_TIMx_Init(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, RADIO_SYSCLK_NONE) != SUCCESS)
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
  
  /* Set the TIMx prescaler to get counter clock frequency at 10 MHz */
  tim_prescaler = __LL_TIM_CALC_PSC(LL_TIM_GetPeriphClock(TIMx), 10000000);

  /* Set the TIMx auto-reload register to get a PWM frequency at 10 KHz */  
  /* Note that in macro call below, targeted PWM frequency must be multiplied */ 
  /* by 2 when the counter operates in center-aligned mode (due to the       */
  /* symmetry of the pattern).                                                */
  tim_period = __LL_TIM_CALC_ARR(LL_TIM_GetPeriphClock(TIMx), tim_prescaler, 10000*2);

  /* Set PWM output channel 1 duty cycle to 50% */
  pulse_value_1 = (tim_period * 500) / 1000;

  /* Set PWM output channel 2 duty cycle to 25% */
  pulse_value_2 = (tim_period * 250) / 1000;

  /* Set dead time to 0,1 us (100 ns) */
  tim_deadtime = __LL_TIM_CALC_DEADTIME(LL_TIM_GetPeriphClock(TIMx), 0, 100);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIMx_Init();

  /**********************************/
  /* Start output signal generation */
  /**********************************/
  /* Enable outputs OC1, OC1N, OC2 */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1  |
                                LL_TIM_CHANNEL_CH1N );
  
#ifdef LL_TIM_CHANNEL_CH2
  /* Enable outputs OC2 */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2 );
#endif /* LL_TIM_CHANNEL_CH2 */
  
#ifdef LL_TIM_CHANNEL_CH2N
  /* Enable outputs OC2N */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N );
#endif /* LL_TIM_CHANNEL_CH2N */           
  
  /* Enable TIMx outputs */
  LL_TIM_EnableAllOutputs(TIMx);
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIMx);
  
  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIMx);

  printf("Connect the TIMx pins to an oscilloscope to monitor the different waveforms.\n\r");
  printf("Connect the TIMx break to the GND. To generate a break event, switch this pin level from 0V to 3.3V.\n\r");
  printf("\n\r");

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
  * @brief TIMx Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIMx_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_EnableClock(LL_PERIPH_TIMx);
  
  GPIO_InitStruct.Pin = TIMx_CH1_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = TIMx_CH1_AF;
  LL_GPIO_Init(TIMx_CH1_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = TIMx_CH1N_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = TIMx_CH1N_AF;
  LL_GPIO_Init(TIMx_CH1N_PORT, &GPIO_InitStruct);
  
#ifdef TIMx_CH2_PIN
  GPIO_InitStruct.Pin = TIMx_CH2_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = TIMx_CH2_AF;
  LL_GPIO_Init(TIMx_CH2_PORT, &GPIO_InitStruct);
#endif /* TIMx_CH2_PIN */
  
#ifdef TIMx_CH2N_PIN
  GPIO_InitStruct.Pin = TIMx_CH2N_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = TIMx_CH2N_AF;
  LL_GPIO_Init(TIMx_CH2N_PORT, &GPIO_InitStruct);
#endif /* TIMx_CH2N_PIN */
  
  GPIO_InitStruct.Pin = TIMx_BKIN_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = TIMx_BKIN_AF;
  LL_GPIO_Init(TIMx_BKIN_PORT, &GPIO_InitStruct);

  /* Configure the TIMx time base unit */
  TIM_InitStruct.Prescaler = tim_prescaler;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP;
  TIM_InitStruct.Autoreload = tim_period;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIMx, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIMx);
  
  /* Time Base configuration structure definition */
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  
  /* Configure the output channel 1 */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.CompareValue = pulse_value_1;
  LL_TIM_OC_Init(TIMx, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIMx, LL_TIM_CHANNEL_CH1);
  
#ifdef TIMx_CH2_PIN
  /* Configure the output channel 2 */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.CompareValue = pulse_value_2;
  LL_TIM_OC_Init(TIMx, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIMx, LL_TIM_CHANNEL_CH2);
#endif /* TIMx_CH2_PIN */
  
  /* Configure the Break and Dead Time feature of the TIMx */
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_ENABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_ENABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_1;
  TIM_BDTRInitStruct.DeadTime = tim_deadtime;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_ENABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
#ifdef TIM_BDTR_BKBID
  TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
#endif /* TIM_BDTR_BKBID */
#ifdef LL_TIM_BREAK2_DISABLE
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
#endif /* LL_TIM_BREAK2_DISABLE */
#ifdef LL_TIM_BREAK2_POLARITY_HIGH
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
#endif /* LL_TIM_BREAK2_POLARITY_HIGH */  
#ifdef LL_TIM_BREAK2_FILTER_FDIV1  
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
#endif /* LL_TIM_BREAK2_FILTER_FDIV1 */
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE;
  LL_TIM_BDTR_Init(TIMx, &TIM_BDTRInitStruct);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1);
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



