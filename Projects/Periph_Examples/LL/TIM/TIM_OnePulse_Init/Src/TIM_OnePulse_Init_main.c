
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : TIM_OnePulse_Init_main.c
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
 * @file  TIM_OnePulse_Init/TIM_OnePulse_Init_main.c
 * @brief Configuration of a timer to generate a positive pulse in 
 * Output Compare mode with a length of tPULSE and after a delay of tDELAY. 
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_OnePulse_Init\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_OnePulse_Init.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_OnePulse_Init\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_OnePulse_Init.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_OnePulse_Init\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A1     |       TIM1 CH4      |      Not Used      |
|     A11    |       Not Used      |      Not Used      |
|     A12    |       Not Used      |        N.A.        |
|     A13    |       Not Used      |        N.A.        |
|     A14    |       Not Used      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |       Not Used      |        N.A.        |
|     A5     |       Not Used      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       USART TX      |      Not Used      |
|     A9     |       USART RX      |      Not Used      |
|     B0     |       Not Used      |      Not Used      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |      TIM2 CH4      |
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
|   BUTTON name  |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |    STEVAL-IDB012V1   |
------------------------------------------------------------------------------------------
|      PUSH1     |  Start TIM1 counter  |  Start TIM1 counter  |  Start TIM1 counter  |
|      PUSH2     |       Not Used       |       Not Used       |       Not Used       |
|      RESET     |   Reset BlueNRG-LP   |   Reset BlueNRG-LP   |   Reset BlueNRG-LP   |

@endtable

* \section Usage Usage



Configuration of a timer to generate a positive pulse in Output Compare mode with a length of tPULSE and after a delay of tDELAY. 
This example is based on the BLUENRG_LP TIM LL API. 
The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size).

The pulse is generated on OC1.

TIM1 generates a positive pulse of 50 us after a delay of 50 us. 
User push-button is used to start TIMx counter. 

TIMx_CHx delay and pulse length are measured every time a pulse is generated. 


TIMx is configured to generate a single pulse (timer counter stops automatically at the next update event UEV).


  
BlueNRG_LP-EVB Set-up
Connect the following pins to an oscilloscope to monitor the different waveforms: 
- TIMx_CH4  
- LED2
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
#include "TIM_OnePulse_Init_main.h"

/* Private includes -----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

__IO uint8_t updateMeasured = 0;

/* TIMx clock configuration */
static uint32_t tim_swtrigger_prescaler = 0;
static uint32_t tim_swtrigger_period = 0;
static uint32_t tim_swtrigger_pulse = 0;


/* Measured pulse delay (in us) */
__IO uint32_t uwMeasuredDelay = 0;

/* Measured pulse length (in us) */
__IO uint32_t uwMeasuredPulseLength = 0;

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);
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
  BSP_COM_Init(Process_InputData);
 
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  
  /* Reset LED_PIN */
  LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_PIN);  

  /* Set the TIMx prescaler to get counter clock frequency at 10 MHz  */
  /* In this example TIMx input clock (TIMxCLK) is set to APB0 clock. */
  /*    => TIMxCLK = 64 MHz                                           */  
  tim_swtrigger_prescaler = __LL_TIM_CALC_PSC(LL_TIM_GetPeriphClock(TIMx), 10000000);
  
  /* Set the autoreload register to get a pulse length of 50 us */
  tim_swtrigger_period = __LL_TIM_CALC_PULSE(LL_TIM_GetPeriphClock(TIMx), tim_swtrigger_prescaler, 50, 50);

  /* Set the capture/compare register to get a pulse delay of 50 us */
  tim_swtrigger_pulse = __LL_TIM_CALC_DELAY(LL_TIM_GetPeriphClock(TIMx), tim_swtrigger_prescaler, 50);

  MX_TIMx_Init();

  /**************************/
  /* TIMx interrupts set-up */
  /**************************/
  /* Enable the capture/compare interrupt for channel 4 */
  LL_TIM_EnableIT_CC4(TIMx);

  /**************************/
  /* Start pulse generation */
  /**************************/
  /* Enable channel 4 */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  /* Enable TIMx outputs */
  LL_TIM_EnableAllOutputs(TIMx);

  /* Enable auto-reload register preload */
  LL_TIM_EnableARRPreload(TIMx);

  // User push-button (PUSH1) or 'c'/'C' character is used to start TIMx counter.
  printf("Wait for User push-button (PUSH1) press or enter 'c'/'C' character.\n\r");
  
  /* Infinite loop */
  while (1)
  {
    if(updateMeasured == 1)
    {
      printf("uwMeasuredDelay = %d\n\r", uwMeasuredDelay);
      printf("uwMeasuredPulseLength = %d\n\r", uwMeasuredPulseLength);
      printf("\n\r"); 
      updateMeasured =0;
      LL_mDelay(10);
      /* Reset LED_PIN */
      LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_PIN);
    }
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

  /* GPIO Ports Clock Enable */
  LL_EnableClock_TIMx_CH4() ;

  GPIO_InitStruct.Pin = TIMx_CH4_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = TIMx_CH4_AF;
  LL_GPIO_Init(TIMx_CH4_PORT, &GPIO_InitStruct);
  
  /* Peripheral clock enable */
  LL_EnableClock_TIMx();

  /* TIMx interrupt Init */
  NVIC_SetPriority(TIMx_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(TIMx_IRQn);

  /* Configure the TIMx time base unit */
  TIM_InitStruct.Prescaler = tim_swtrigger_prescaler;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = tim_swtrigger_period;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIMx, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIMx);
  
  /* Configure the output channel 4 */
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = tim_swtrigger_pulse;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIMx, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIMx, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetOnePulseMode(TIMx, LL_TIM_ONEPULSEMODE_SINGLE);

  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_LOW;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
  #ifdef TIM_BDTR_BK2P
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_LOW;
  #endif /* TIM_BDTR_BK2P */
  #ifdef TIM_BDTR_BK2F
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  #endif /* TIM_BDTR_BK2F */
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIMx, &TIM_BDTRInitStruct);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
  LL_APB0_EnableClock(LL_APB0_PERIPH_SYSCFG);
  
  /* Configure GPIO for LED */
  LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_PIN);
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
   
  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
 
  /* Enable a rising trigger External line 10 Interrupt */
  EXTI_InitStruct.Line = LL_EXTI_LINE_PA10;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Type = LL_EXTI_TYPE_EDGE;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_EDGE;
  LL_EXTI_Init(&EXTI_InitStruct);

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);

  /* Configure NVIC for SysTick_IRQn */
  NVIC_SetPriority(SysTick_IRQn, IRQ_LOW_PRIORITY);
}


/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
  * @brief  User button interrupt processing
  * @note   TIMx counter is enabled every time the user button is presssed. 
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  /* Set LED_PIN */
  LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_PIN);
  /* Enable counter. Note that the counter will stop automatically at the     */
  /* next update event (UEV).                                                 */
  LL_TIM_EnableCounter(TIMx);
}

/**
  * @brief  Timer capture/compare interrupt processing
  * @note   Calculates the pulse delay and pulse length of the output waveform
  *         generated by TIMx.
  * @param  None
  * @retval None
  */
void TimerCaptureCompare_Callback(void)
{
  uint32_t CNT;
  uint32_t PSC; 
  uint32_t ARR; 
  
  CNT = LL_TIM_GetCounter(TIMx);
  PSC = LL_TIM_GetPrescaler(TIMx);
  ARR = LL_TIM_GetAutoReload(TIMx);
   
  uwMeasuredDelay = (CNT * 1000000)/(LL_TIM_GetPeriphClock(TIMx)/(PSC + 1));
  uwMeasuredPulseLength = ((ARR - CNT) * 1000000)/(LL_TIM_GetPeriphClock(TIMx)/(PSC + 1)); 
  updateMeasured =1;
}


void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
  if(Nb_bytes>0)
  {
    if(data_buffer[0] == 'c' || data_buffer[0] == 'C' )
    {
      UserButton_Callback();
    }
  }
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



