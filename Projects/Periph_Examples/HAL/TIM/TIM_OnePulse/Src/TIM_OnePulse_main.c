
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : TIM_OnePulse_main.c
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
 * @file  TIM_OnePulse/TIM_OnePulse_main.c
 * @brief Use of the TIM peripheral to generate a single pulse when 
an external signal falling edge is received on the timer input pin.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_OnePulse\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_OnePulse.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_OnePulse\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_OnePulse.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_OnePulse\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A14    |       Not Used      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |       TIM1 CH1      |        N.A.        |
|     A5     |       TIM1 CH2      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       Not Used      |      Not Used      |
|     A9     |       Not Used      |      Not Used      |
|     B0     |       Not Used      |      Not Used      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |      Trigger       |
|     B4     |       Not Used      |      Not Used      |
|     B5     |       Not Used      |      TIM2 CH2      |
|     B7     |       Not Used      |      Not Used      |
|     B8     |       Trigger       |        N.A.        |
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
|   BUTTON name  |       STEVAL-IDB011V1      |       STEVAL-IDB011V2      |       STEVAL-IDB012V1      |
------------------------------------------------------------------------------------------------------------
|      PUSH1     |  Trigger the input signal  |  Trigger the input signal  |  Trigger the input signal  |
|      PUSH2     |          Not Used          |          Not Used          |          Not Used          |
|      RESET     |      Reset BlueNRG-LP      |      Reset BlueNRG-LP      |      Reset BlueNRG-LPS     |

@endtable

* \section Usage Usage


Use of the TIM peripheral to generate a single pulse when an external signal falling edge is received on the timer input pin.

The external signal is connected to TIMx_CH2, and a falling edge on this input is used to trigger the Timer.
The One Pulse signal is output on TIMx_CH1.

The delay value is fixed to:
Delay =  CCR1/TIM1 counter clock 
      = 16383 / 1000000 [sec] = 16 ms 

The pulse value is fixed to : 
Pulse value = (TIM_Period - TIM_Pulse)/TIM1 counter clock  
            = (65535 - 16383) / 1000000 [sec] = 49ms

Connecting the Trigger pin to TIMx TI2 allows to trigger TIMx counter by pressing the User push-button.

Connect the following pins to an oscilloscope to monitor the different waveforms:
- Trigger pin
- TIMx_CH1 
- TIMx_CH2 

**/
   

/* Includes ------------------------------------------------------------------*/
#include "TIM_OnePulse_main.h"

/** @addtogroup RF_DRIVER_HAL_Examples
  * @{
  */

/** @addtogroup TIM_OnePulse
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef    TimHandle;
EXTI_HandleTypeDef HEXTI_InitStructure;

/* Prescaler value declartion*/
uint32_t uwPrescalerValue = 0;

/* Timer One Pulse Configuration Structure declaration */
TIM_OnePulse_InitTypeDef sConfig = {0};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void EXTI10_IRQHandler_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, RADIO_SYSCLK_NONE) != SUCCESS)
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  HAL_Init();
  
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);

  /* Initialize all configured peripherals */
  EXTI10_IRQHandler_Config();
  
  /* 2 Compute the prescaler value, to have TIM1Freq = 1000000 Hz
   * TIM1CLK = HAL_TIM_GetPeriphClock(TIMx)
   *
   * Prescaler = (TIM1CLK /TIM1 counter clock) - 1
   *
   * The prescaler value is computed in order to have TIM1 counter clock
   * set at 1000000 Hz.
   */
  uwPrescalerValue = (uint32_t)((HAL_TIM_GetPeriphClock(TIMx)) / 1000000) - 1;

  /* 3  Configure the TIM peripheral
   *
   *-The external signal is connected to TIM1_CH2 ,
   * and a falling edge on this input is used to trigger the Timer.
   *
   *-The One Pulse signal is output on TIM1_CH1.
   *
   * The delay value is fixed to:
   * - Delay =  CCR1/TIM1 counter clock
   *          = 16383 / 1000000 [sec]
   *
   * The pulse value is fixed to :
   * - Pulse value = (TIM_Period - TIM_Pulse)/TIM1 counter clock
   *               = (65535 - 16383) / 1000000 [sec]
   *
   **/

  TimHandle.Instance               = TIMx;
  TimHandle.Init.Period            = 0xFFFF;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  if (HAL_TIM_OnePulse_Init(&TimHandle, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*-2- Configure the Channel 1 in One Pulse mode */
  sConfig.OCMode       = TIM_OCMODE_PWM2;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.Pulse        = 16383;
  sConfig.ICPolarity   = TIM_ICPOLARITY_FALLING;
  sConfig.ICSelection  = TIM_ICSELECTION_DIRECTTI;
  sConfig.ICFilter     = 0;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_OnePulse_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /*-4- Start the One Pulse mode */
  if (HAL_TIM_OnePulse_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  HAL_GPIO_WritePin(EXTERNAL_TRIGGER_SIGNAL_PORT, EXTERNAL_TRIGGER_SIGNAL_PIN, GPIO_PIN_RESET);

  printf("Wait for User push-button (PUSH1) press start the application.\n\r");
  
  while (1)
  {
  }
}

/**
  * @brief  Configures EXTI line 10 (connected to PA.10 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTI10_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure = {0};
  EXTI_ConfigTypeDef EXTI_Config_InitStructure = {0};
  
  /* Enable GPIOC clock */
  USER_BUTTON_GPIO_CLK_ENABLE();
  USER_BUTTON_SYSCFG_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = USER_BUTTON_PIN;
  HAL_GPIO_Init(USER_BUTTON_GPIO_PORT, &GPIO_InitStructure);

  EXTI_Config_InitStructure.Line =    USER_BUTTON_EXTI_LINE;
  EXTI_Config_InitStructure.Trigger = EXTI_TRIGGER_RISING_EDGE;
  EXTI_Config_InitStructure.Type =    EXTI_TYPE_EDGE;
   
  HAL_EXTI_SetConfigLine(&HEXTI_InitStructure, &EXTI_Config_InitStructure);
  HAL_EXTI_RegisterCallback(&HEXTI_InitStructure, HAL_EXTI_COMMON_CB_ID, Example_EXTI_Callback);
  HAL_EXTI_Cmd(&HEXTI_InitStructure , ENABLE);
  
  HAL_EXTI_ClearPending(&HEXTI_InitStructure);
  
  /* Enable and set line Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, IRQ_HIGH_PRIORITY);
  HAL_NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);
  
  /* Configure NVIC for SysTick_IRQn */
  HAL_NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void Example_EXTI_Callback(uint32_t Line)
{
  /* Use EXTERNAL_TRIGGER_SIGNAL where is't available a pin to link LED2 to the TIMx CH2 */
  HAL_GPIO_TogglePin(EXTERNAL_TRIGGER_SIGNAL_PORT, EXTERNAL_TRIGGER_SIGNAL_PIN);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while (1)
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
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



