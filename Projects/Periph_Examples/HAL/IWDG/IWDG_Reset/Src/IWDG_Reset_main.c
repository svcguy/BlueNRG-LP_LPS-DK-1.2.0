
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : IWDG_Reset_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the IWDG functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file IWDG_Reset/IWDG_Reset_main.c
 * @brief How to handle the IWDG reload counter and simulate a software fault that generates an IWDG reset after a preset laps of time.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\IWDG\\IWDG_Reset\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\IWDG_Reset.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\IWDG\\IWDG_Reset\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\IWDG_Reset.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\IWDG\\IWDG_Reset\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  LED name  |           STEVAL-IDB011V1          |           STEVAL-IDB011V2          |           STEVAL-IDB012V1          |
--------------------------------------------------------------------------------------------------------------------------------
|     DL1    |              Not Used              |              Not Used              |              Not Used              |
|     DL2    |  Blinking: application is running  |  Blinking: application is running  |  Blinking: application is running  |
|     DL3    |           Blinking: error          |           Blinking: error          |           Blinking: error          |
|     DL4    |              Not Used              |              Not Used              |              Not Used              |
|     U5     |              Not Used              |              Not Used              |              Not Used              |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |    STEVAL-IDB012V1   |
------------------------------------------------------------------------------------------
|      PUSH1     |   Activate the IWDG  |   Activate the IWDG  |   Activate the IWDG  |
|      PUSH2     |       Not Used       |       Not Used       |       Not Used       |
|      RESET     |   Reset BlueNRG-LP   |   Reset BlueNRG-LP   |   Reset BlueNRG-LP   |

@endtable

* \section Usage Usage

How to handle the IWDG reload counter and simulate a software fault that generates an IWDG reset after a preset laps of time.

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals, initialize the Flash interface and the systick.


The watchdog counter counts down from the IWDG_RELOAD value. The timeout period is a function of this value and the clock prescaler IWDG_PRESCALER. 
Refer to the datasheet for the timeout information the IWDG timeout is set to 1 second:
#define IWDG_PRESCALER IWDG_PRESCALER_8
#define IWDG_RELOAD    0xFFF // 1024ms 

The IWDG reload counter is refreshed each 700 ms in the main program infinite loop to prevent a IWDG reset.
  
LED2 is toggling each 700 ms indicating that the program is running.

An EXTI Line is connected to a GPIO pin, configured to generate an interrupt when the User push-button (PUSH1) is pressed.

The EXTI Line is used to simulate a software failure: once the EXTI Line event occurs by pressing the User push-button (PUSH1), the corresponding interrupt is served.

In the ISR, a write to invalid address generates a Hardfault exception containing an infinite loop and preventing to return to main program (the IWDG reload counter is not refreshed).
As a result, when the IWDG counter reaches 0, the IWDG reset occurs.

If the IWDG reset is generated, after the system resumes from reset, a message is showed over the UART.
If the EXTI Line event does not occur, the IWDG counter is indefinitely refreshed in the main program infinite loop, and there is no IWDG reset.

LED3 blink slowly on if any error occurs.

  
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
#include "IWDG_Reset_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define IWDG_WINDOW IWDG_WINDOW_DISABLE
#define IWDG_PRESCALER IWDG_PRESCALER_8
#define IWDG_RELOAD 0xFFF // 1024ms 

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

/* Private function prototypes -----------------------------------------------*/
static void MX_IWDG_Init(void);
void resetReasonByIWDGRST(void);

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
  
  printf("Application started\n\r");
	
  /* Configure LED */
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  
  /* Configure User push-button */
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_EXTI);
  
  /* Check if the system has resumed from IWDG reset */
  resetReasonByIWDGRST();
  
  /* Clear reset flags in any cases */
  __HAL_RCC_CLEAR_RESET_FLAGS();
  
  /* Configure & Start the IWDG peripheral */
  
  /* Initialize all configured peripherals */
  MX_IWDG_Init();
	
  printf("Wait for User push-button PUSH1 or for the push-button Reset\n\r");
	
  /* Infinite loop */
  while (1)
  {
    /* Toggle LED2 */
    BSP_LED_Toggle(BSP_LED2);
    
    /* Insert 700 ms delay */
    HAL_Delay(700);
    
    /* Refresh IWDG: reload counter */
    if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
    {
      /* Refresh Error */
      Error_Handler();
    }
  }
}

/**
* @brief IWDG Initialization Function
* @param None
* @retval None
*/
static void MX_IWDG_Init(void)
{
  /* WDG clock enable */
  __HAL_RCC_WDG_CLK_ENABLE();
  
  /* Force WDG peripheral reset */
  __HAL_RCC_WDG_FORCE_RESET();
  __HAL_RCC_WDG_RELEASE_RESET();
  
  /* Check if WDG Reset Release flag interrupt occurred or not */
  while(__HAL_RCC_GET_IT(RCC_IT_WDGRSTRELRDY) == 0)
  {
  }
  
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER;
  hiwdg.Init.Window = IWDG_WINDOW;
  hiwdg.Init.Reload = IWDG_RELOAD;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief  
* @param  
* @retval 1 if 
*/
void resetReasonByIWDGRST()
{
  printf("Check if the system has resumed from IWDG reset:\n\r");

  if( (RAM_VR.ResetReason & RCC_FLAG_WDGRST) == RCC_FLAG_WDGRST)
  {
    /* IWDGRST flag is set */
    printf("YES, IWDG reset is generated.\n\r");
  }
  else
  {
    printf("NO.\n\r");
  }
  printf("\n\r");
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  while(1) 
  {
    BSP_LED_Toggle(BSP_LED3);
    HAL_Delay(990);
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



