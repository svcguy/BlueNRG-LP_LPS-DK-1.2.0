
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : TIM_OC_main.c
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
 * @file  TIM_OC/TIM_OC_main.c
 * @brief Configuration of the TIM peripheral to generate an output 
 * waveform in different output compare modes. This example is based on the 
 * BLUENRG_LP TIM LL API. The peripheral initialization uses 
 * LL unitary service functions for optimization purposes (performance and size).
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_OC\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_OC.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_OC\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_OC.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_OC\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     B3     |       Not Used      |      Not Used      |
|     B4     |       Not Used      |      Not Used      |
|     B5     |       Not Used      |      Not Used      |
|     B7     |       Not Used      |      Not Used      |
|     B8     |       Not Used      |        N.A.        |
|     B9     |       Not Used      |        N.A.        |
|     A0     |         N.A.        |      Not Used      |
|     A10    |         N.A.        |      Not Used      |
|     B1     |         N.A.        |      Not Used      |
|     B6     |         N.A.        |      TIM2 CH1      |
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
|   BUTTON name  |        STEVAL-IDB011V1       |        STEVAL-IDB011V2       |        STEVAL-IDB012V1       |
------------------------------------------------------------------------------------------------------------------
|      PUSH1     |  Change output compare mode  |  Change output compare mode  |  Change output compare mode  |
|      PUSH2     |           Not Used           |           Not Used           |           Not Used           |
|      RESET     |       Reset BlueNRG-LP       |       Reset BlueNRG-LP       |       Reset BlueNRG-LP       |

@endtable

* \section Usage Usage

Configuration of the TIM peripheral to generate an output waveform in different output compare modes. 
This example is based on the BLUENRG_LP TIM LL API. 
The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size).

TIMx input clock (TIMxCLK) is 64MHz, since prescaler is equal to 1.
    TIMxCLK = 64 Mhz

To set the TIMx counter clock frequency to 10 KHz, the pre-scaler (PSC) is calculated as follows:
PSC = (TIMxCLK / TIMx counter clock) - 1
PSC = (64 MHz /10 KHz) - 1

SystemCoreClock is set to 64 MHz for BLUENRG_LP Devices.

Auto-reload (ARR) is calculated to get a time base period of 100 ms, meaning a time base frequency of 10 Hz.
ARR = (TIMx counter clock / time base frequency) - 1
ARR = (TIMx counter clock / 10) - 1

The capture/compare register (CCR1) of the output channel is set to half the auto-reload value. 
Therefore the timer output compare delay is 50 ms.
Generally speaking this delay is calculated as follows:
CC1_delay = TIMx counter clock / CCR1

The timer output channel must be connected to PB11 on board BlueNRG_LP-EVB.
Thus TIMx_CH1 status (on/off) mirrors the timer output level (active v.s. inactive).

User push-button (PUSH1) can be used to change the output compare mode:
  - When the output channel is configured in output compare toggle:  TIMx_CH1 
    TOGGLES when the counter (CNT) matches the capture/compare register (CCR1).
  - When the output channel is configured in output compare active:  TIMx_CH1 
    switched ON when the counter (CNT) matches the capture/compare register (CCR1).
  - When the output channel is configured in output compare inactive:  TIMx_CH1 
    switched OFF when the counter (CNT) matches the capture/compare register (CCR1).
    
Initially the output channel is configured in output compare toggle mode.


BlueNRG_LP-EVB Set-up
Connect the following pins to an oscilloscope to monitor the different waveforms:
- TIMx_CH1  
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
#include "TIM_OC_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Number of output compare modes */
#define TIM_OC_MODES_NB 3

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint8_t ubButtonPress = 0;

/* Output compare modes */
static uint32_t aOCMode[TIM_OC_MODES_NB] = {
  LL_TIM_OCMODE_TOGGLE,
  LL_TIM_OCMODE_ACTIVE,
  LL_TIM_OCMODE_INACTIVE
};

/* Output compare mode index */
static uint8_t iOCMode = 0;

/* Compare match count */
static uint32_t uwCompareMatchCount = 0;

/* TIMx Clock */
static uint32_t tim_prescaler = 0;
static uint32_t tim_arr = 0;
static uint32_t tim_pulse_value = 0;

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);
static void UserButton_Init(void);
static void MX_TIMx_Init(void);
__STATIC_INLINE void Configure_OCMode(uint32_t OCMode);

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
  BSP_COM_Init(Process_InputData);
 
  /* Initialize button in EXTI mode */
  UserButton_Init();
  
  MX_TIMx_Init();

  /**************************/
  /* TIMx interrupts set-up */
  /**************************/
  /* Enable the capture/compare interrupt for channel 1 */
  LL_TIM_EnableIT_CC1(TIMx);

  /**********************************/
  /* Start output signal generation */
  /**********************************/
  /* Enable output channel 1 */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);

  /* Enable the outputs (set the MOE bit in TIMx_BDTR register). */
  LL_TIM_EnableAllOutputs(TIMx);  
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIMx);

  printf("The output compare mode can be chenged.\n\r");
  printf("Wait for User push-button (PUSH1) press or enter 'c'/'C' character.\n\r");
         
  /* Infinite loop */
  while (1)
  {
    if(ubButtonPress == 1)
    {             
      /* Set new OC mode */
      iOCMode = (iOCMode + 1) % TIM_OC_MODES_NB;
        
      /* Switch to next OC mode */
      Configure_OCMode(aOCMode[iOCMode]);
      
      ubButtonPress = 0;
      printf("The output compare mode can be chenged.\n\r");
      printf("Wait for User push-button (PUSH1) press or enter 'c'/'C' character.\n\r");
    }
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
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  /*************************/
  /* GPIO AF configuration */
  /*************************/
  /* Enable the peripheral clock of GPIOs */
  LL_EnableClock_TIMx_CH1();
  
  /* GPIO TIMx_CH1 configuration */
  LL_GPIO_SetPinMode(TIMx_CH1_PORT, TIMx_CH1_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(TIMx_CH1_PORT, TIMx_CH1_PIN, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinSpeed(TIMx_CH1_PORT, TIMx_CH1_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_TIMx_CH1(); 
  
  /***********************************************/
  /* Configure the NVIC to handle TIMx interrupt */
  /***********************************************/
  NVIC_SetPriority(TIMx_IRQn, IRQ_LOW_PRIORITY);
  NVIC_EnableIRQ(TIMx_IRQn);
  
  /******************************/
  /* Peripheral clocks enabling */
  /******************************/
  /* Enable the timer peripheral clock */
  LL_EnableClock_TIMx(); 

  /* Set the pre-scaler value */
  tim_prescaler = __LL_TIM_CALC_PSC(LL_TIM_GetPeriphClock(TIMx), 10000);

  /* Set the auto-reload value to have a counter frequency of 10 Hz */
  /* TIMxCLK = LL_TIM_GetPeriphClock(TIMx) / (APB prescaler & multiplier)               */
  tim_arr = __LL_TIM_CALC_ARR(LL_TIM_GetPeriphClock(TIMx), tim_prescaler, 10);

  /* Set output compare active/inactive delay to half of the auto-reload value */
  tim_pulse_value = tim_arr / 2;

  TIM_InitStruct.Prescaler = tim_prescaler;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = tim_arr;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIMx, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIMx);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_TOGGLE;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = tim_pulse_value;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIMx, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIMx, LL_TIM_CHANNEL_CH1);

}

/**
  * @brief  Configures User push-button (PUSH1) in GPIO or EXTI Line Mode.
  * @param  None 
  * @retval None
  */
void UserButton_Init(void)
{
  /* Enable the BUTTON Clock */
  USER_BUTTON_GPIO_CLK_ENABLE();
  USER_BUTTON_SYSCFG_CLK_ENABLE();
  
  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);

  /* Enable a rising trigger External line 10 Interrupt */
  USER_BUTTON_EXTI_LINE_ENABLE();
  USER_BUTTON_EXTI_RISING_TRIG_ENABLE();
  
  /* Clear the event occurred on the interrupt line 10 port A. */
  if (LL_EXTI_IsInterruptPending(USER_BUTTON_EXTI_LINE) != RESET)
  {
    LL_EXTI_ClearInterrupt(USER_BUTTON_EXTI_LINE);
  }

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);

  /* Configure NVIC for SysTick_IRQn */
  NVIC_SetPriority(SysTick_IRQn, IRQ_LOW_PRIORITY);
}

/**
  * @brief  Changes the output compare mode.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void Configure_OCMode(uint32_t OCMode)
{
  /* Disable the counter */
  LL_TIM_DisableCounter(TIMx);
  
  /* Reset the counter */
  LL_TIM_SetCounter(TIMx, 0);
  
  /* Reset the compare match count */
  uwCompareMatchCount = 0;
  
  /* Set the output level (active v.s. inactive) according to the new OC mode */
  switch (OCMode)
  {
    case LL_TIM_OCMODE_TOGGLE:
       /* Set the output channel to its toggles on compare match*/
      printf("Set the output channel to its toggles on compare match.\n\r");
      LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_TOGGLE);
      break; 
      
    case LL_TIM_OCMODE_ACTIVE:
      /* Set the output channel to its inactive level (LOW)*/
      printf("Set the output channel to its inactive level (LOW).\n\r");
      LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_FORCED_INACTIVE);
      break;
      
    case LL_TIM_OCMODE_INACTIVE:
      /* Set the output channel to its active level (HIGH)*/
      printf("Set the output channel to its active level (HIGH).\n\r");
      LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_FORCED_ACTIVE);
      break;
        
    default:
      break;
  }
  
  /* Update the output channel mode */
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, OCMode);
  
  /* Re-enable the counter */
  LL_TIM_EnableCounter(TIMx);
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
  * @brief  User button interrupt processing
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  /* Update User push-button (PUSH1) variable : to be checked in waiting loop in main program */
  ubButtonPress = 1;
}

/**
  * @brief  Timer capture/compare interrupt processing
  * @note   The capture/compare interrupt is generated whatever the compare
  *         mode is (as long as the timer counter is enabled).
  * @param  None
  * @retval None
  */
void TimerCaptureCompare_Callback(void)
{
   /* Upon compare match, the counter value  should be equal to the */
   /* capture/compare register value */
  if(LL_TIM_GetCounter(TIMx) == LL_TIM_OC_GetCompareCH1(TIMx))
  {
    /* Increment the compare match count */
    uwCompareMatchCount++;
  }
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


