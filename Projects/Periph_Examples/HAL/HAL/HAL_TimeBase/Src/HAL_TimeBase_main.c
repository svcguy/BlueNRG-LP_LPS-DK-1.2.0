
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : HAL_TimeBase_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating how to use the HAL time base
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  HAL_TimeBase/HAL_TimeBase_main.c
 * @brief Example of how to customize HAL using a general-purpose timer as main source of time base, instead of Systick.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\HAL\\HAL_TimeBase\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\HAL_TimeBase.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\HAL\\HAL_TimeBase\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\HAL_TimeBase.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\HAL\\HAL_TimeBase\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       USART TX      |      Not Used      |
|     A9     |       USART RX      |      Not Used      |
|     B0     |       Not Used      |      USART RX      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |      Not Used      |
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
|  LED name  |                           STEVAL-IDB011V1                          |                           STEVAL-IDB011V2                          |                           STEVAL-IDB012V1                          |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                              Not Used                              |                              Not Used                              |                              Not Used                              |
|     DL2    |  Blinking: tick increment is on; Off: tick increment is suspended  |  Blinking: tick increment is on; Off: tick increment is suspended  |  Blinking: tick increment is on; Off: tick increment is suspended  |
|     DL3    |                              Not Used                              |                              Not Used                              |                              Not Used                              |
|     DL4    |                              Not Used                              |                              Not Used                              |                              Not Used                              |
|     U5     |                              Not Used                              |                              Not Used                              |                              Not Used                              |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |                  STEVAL-IDB011V1                 |                  STEVAL-IDB011V2                 |                  STEVAL-IDB012V1                 |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |  it is used to suspend or resume tick increment  |  it is used to suspend or resume tick increment  |  it is used to suspend or resume tick increment  |
|      PUSH2     |                     Not Used                     |                     Not Used                     |                     Not Used                     |
|      RESET     |                 Reset BlueNRG-LP                 |                 Reset BlueNRG-LP                 |                 Reset BlueNRG-LP                 |

@endtable

* \section Usage Usage

How to customize HAL using a general-purpose timer as main source of time base, instead of Systick.

In this example the used timer is TIM1.

The example brings, in user file, a new implementation of the following HAL weak functions:

HAL_InitTick() 
HAL_SuspendTick()
HAL_ResumeTick()

This implementation will overwrite native implementation from rf_driver_hal.c and so user functions will be invoked instead when called.

The following time base functions are kept as implemented natively:

HAL_IncTick()
HAL_Delay()
HAL_IncTick()

When user pushes the User push-button (PUSH1), the Tick is modified, in an infinite loop.

In order to make the program work, you must do the following:
 - Launch serial communication SW on PC
 - Flash the project in the Board
 - Press the RESET button

BlueNRG_LP-EVB Set-up
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
#include "HAL_TimeBase_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htimx;

/* Private variables ---------------------------------------------------------*/
uint32_t uwIncrementState = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

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
  
  /* Initialize all configured peripherals */
  /* Configure LED2 */
  BSP_LED_Init(BSP_LED2);
  
  /* Configure User push-button (PUSH1) */
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_EXTI);

  /* Insert a Delay of 500 ms and toggle LED2, in an infinite loop */
  printf("When user pushes the User push-button (PUSH1), the Tick is modified.\n\r");
  
  /* Infinite loop */
  while (1)
  {    
    /* Insert a 500ms delay */
    HAL_Delay(500);
    
    /* Toggle LED2 */
    BSP_LED_Toggle(BSP_LED2);
  }
}

/**
* @brief  This function configures the TIMx as a time base source. 
*         The time source is configured  to have 1ms time base with a dedicated 
*         Tick interrupt priority. 
* @note   This function is called  automatically at the beginning of program after
*         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig(). 
* @param  TickPriority: Tick interrupt priority.
* @retval HAL status
*/
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;
  
  /*Configure the TIMx IRQ priority */
  HAL_NVIC_SetPriority(TIMx_IRQn, TickPriority);
  
  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIMx_IRQn);
  
  /* Enable TIMx clock */
  EnableClock_TIMx();
  
  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  
  /* Compute the prescaler value to have TIMx counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((HAL_TIM_GetPeriphClock(TIMx) / 1000000) - 1);
  
  /* Initialize TIMx peripheral as follow:
  + Prescaler = (HAL_TIM_GetPeriphClock(TIMx)/1000000 - 1) to have a 1MHz TIMx counter clock.
  + Period = (TIMx counter clock / TIMx output clock) - 1. to have a (1/1000) s time base.
  + ClockDivision = 0
  + Counter direction = Up
  */
  htimx.Instance = TIMx;
  htimx.Init.Period = (1000000 / 1000) - 1;
  htimx.Init.Prescaler = uwPrescalerValue;
  htimx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htimx) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Start the TIM time Base generation in interrupt mode */
  if(HAL_TIM_Base_Start_IT(&htimx) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  
  /* Return function status */
  return HAL_OK;
}

/**
* @brief  Suspend Tick increment.
* @note   Disable the tick increment by disabling TIMx update interrupt.
* @param  None
* @retval None
*/
void HAL_SuspendTick(void)
{
  /* Disable TIMx update Interrupt */
  __HAL_TIM_DISABLE_IT(&htimx, TIM_IT_UPDATE);                                                  
}

/**
* @brief  Resume Tick increment.
* @note   Enable the tick increment by Enabling TIMx update interrupt.
* @param  None
* @retval None
*/
void HAL_ResumeTick(void)
{
  /* Enable TIMx Update interrupt */
  __HAL_TIM_ENABLE_IT(&htimx, TIM_IT_UPDATE);
}

/**
* @brief  Period elapsed callback in non blocking mode
* @note   This function is called  when TIMx interrupt took place, inside
* HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
* a global variable "uwTick" used as application time base.
* @param  htim : TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  HAL_IncTick();
}

/**
* @brief EXTI line detection callback.
* @param GPIOx: Specifies the port used
* @param GPIO_Pin: Specifies the pins connected EXTI line
* @retval None
*/
void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  if(GPIO_Pin == BSP_PUSH1_PIN)
  {
    if (uwIncrementState == 0)
    {
      /* Suspend tick increment */
      printf("Suspend tick increment.\n\r");
      
      /* Turn off LED2 */
      BSP_LED_Off(BSP_LED2);
      
      HAL_SuspendTick();
      
      /* Change the Push button state */
      uwIncrementState = 1;
    }
    else
    {
      /* Resume tick increment */
      printf("Resume tick increment.\n\r");
      HAL_ResumeTick();
      
      /* Change the Push button state */
      uwIncrementState = 0;
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



