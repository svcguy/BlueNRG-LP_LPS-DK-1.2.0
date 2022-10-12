
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : TIM_TimeBase_main.c
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
 * @file  TIM_TimeBase/TIM_TimeBase_main.c
 * @brief Configuration of the TIM peripheral to generate a timebase of one second with the corresponding interrupt request.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_TimeBase\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_TimeBase.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_TimeBase\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_TimeBase.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_TimeBase\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A4     |       Not Used      |        N.A.        |
|     A5     |       Not Used      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       Not Used      |      Not Used      |
|     A9     |       Not Used      |      Not Used      |
|     B0     |       Not Used      |      Not Used      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |      Not Used      |
|     B4     |       Not Used      |        DL2         |
|     B5     |       Not Used      |      Not Used      |
|     B7     |       Not Used      |      Not Used      |
|     B8     |         DL2         |        N.A.        |
|     B9     |         DL3         |        N.A.        |
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
|  LED name  |           STEVAL-IDB011V1          |           STEVAL-IDB011V2          |           STEVAL-IDB012V1          |
--------------------------------------------------------------------------------------------------------------------------------
|     DL1    |              Not Used              |              Not Used              |              Not Used              |
|     DL2    |  Blinking according to TIM period  |  Blinking according to TIM period  |  Blinking according to TIM period  |
|     DL3    |        Slow blinking: error        |        Slow blinking: error        |        Slow blinking: error        |
|     DL4    |              Not Used              |              Not Used              |              Not Used              |
|     U5     |              Not Used              |              Not Used              |              Not Used              |

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
Configuration of the TIM peripheral to generate a timebase of one second with the corresponding interrupt request.

    TIM1 input clock (TIM1CLK) is 64MHz, since prescaler is equal to  1.
    TIM1CLK = 64MHz
    To get TIM1 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM1CLK / TIM1 counter clock) - 1
    Prescaler = (64MHz /10 KHz) - 1


The TIM1 ARR register value is equal to 10000 - 1, Update rate = TIM1 counter clock / (Period + 1) = 1 Hz.
So the TIM1 generates an interrupt each 1 s.

When the counter value reaches the auto-reload register value, the TIM update interrupt is generated and, in the handler routine, connected to LED2 on board BlueNRG_LP-EVB is toggled. 
So led blinks at the following frequency: 0.5Hz.

In case of error, LED3 is turned ON.

BlueNRG_LP-EVB Set-up
- Use LED2 and connect it on an oscilloscope to show the Time Base signal.  

**/
   

/* Includes ------------------------------------------------------------------*/
#include "TIM_TimeBase_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htimx;

/* Private function prototypes -----------------------------------------------*/
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
  HAL_Init();
    
  /* Configure LED2 & LED3 */
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  
  /* Initialize all configured peripherals */
  MX_TIMx_Init();
  
  /*## Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&htimx) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  
  /* Infinite loop */
  while (1)
  {
  }
}

/**
* @brief TIMx Initialization Function
* @param None
* @retval None
*/
static void MX_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;

  htimx.Instance = TIMx;
  htimx.Init.Prescaler = PRESCALER_VALUE;
  htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx.Init.Period = PERIOD_VALUE;
  htimx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htimx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htimx) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htimx, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
}   

/**
* @brief  Period elapsed callback in non blocking mode
* @param  htim : TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  BSP_LED_Toggle(BSP_LED2);
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(BSP_LED3);
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
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


