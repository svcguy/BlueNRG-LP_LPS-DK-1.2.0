
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RNG_MultiRNG_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 19-April-2019
* Description        : Code demonstrating the basic RNG functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RNG_MultiRNG/RNG_MultiRNG_main.c
 * @brief Configuration of the RNG using the HAL API. This example uses the RNG to generate 32-bit long random numbers.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\RNG\\RNG_MultiRNG\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RNG_MultiRNG.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\RNG\\RNG_MultiRNG\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RNG_MultiRNG.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\RNG\\RNG_MultiRNG\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     B4     |       Not Used      |      Not Used      |
|     B5     |       Not Used      |      Not Used      |
|     B7     |       Not Used      |      Not Used      |
|     B8     |       Not Used      |        N.A.        |
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
|  LED name  |                        STEVAL-IDB011V1                       |                        STEVAL-IDB011V2                       |                        STEVAL-IDB012V1                       |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                           Not Used                           |                           Not Used                           |                           Not Used                           |
|     DL2    |   Fast blinking: wait for user action; Slow blinking: error  |   Fast blinking: wait for user action; Slow blinking: error  |   Fast blinking: wait for user action; Slow blinking: error  |
|     DL3    |                           Not Used                           |                           Not Used                           |                           Not Used                           |
|     DL4    |                           Not Used                           |                           Not Used                           |                           Not Used                           |
|     U5     |                           Not Used                           |                           Not Used                           |                           Not Used                           |

@endtable

* \section Buttons_description Buttons description
@table
|   BUTTON name  |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |     STEVAL-IDB012V1    |
------------------------------------------------------------------------------------------------
|      PUSH1     |  Start RNG generation  |  Start RNG generation  |  Start RNG generation  |
|      PUSH2     |        Not Used        |        Not Used        |        Not Used        |
|      RESET     |    Reset BlueNRG-LP    |    Reset BlueNRG-LP    |    Reset BlueNRG-LP    |

@endtable

* \section Usage Usage

Configuration of the RNG using the HAL API. This example uses the RNG to generate 16-bit long random numbers.

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals, initialize the Flash interface and the systick.

The RNG peripheral configuration is ensured by the HAL_RNG_Init() function.
The latter is calling the HAL_RNG_MspInit() function which implements the configuration of the needed RNG resources according to the used hardware (CLOCK, GPIO, DMA and NVIC). 
You may update this function to change RNG configuration.

After startup, user is asked to press User push-button (PUSH1). 
The 8-entry array aRandom[] is filled up by 16-bit long random numbers at each User push-button (PUSH1) press.


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
#include "RNG_MultiRNG_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;

/* Used for storing 8 Random 32bit Numbers */
uint32_t aRandom[8];
__IO uint8_t    ubUserButtonClickEvent = RESET;  /* Event detection: Set after User Button interrupt */

/* Private function prototypes -----------------------------------------------*/
static void MX_RNG_Init(void);

/* Private user code ---------------------------------------------------------*/

/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
  uint32_t counter = 0;
  
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
  
  /* Initialize LED Error on board */
  BSP_LED_Init(BSP_LED2);
  
  /* Initialize all configured peripherals */
  MX_RNG_Init();
  
  /* Configure User push-button (PUSH1) in Interrupt mode */
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_EXTI);
  
  /* Generate eight 16-bit long random numbers */
  printf("Generate eight 16-bit long random numbers.\n\r");
  for (counter = 0; counter < 8; counter++)
  {
    /* Wait for event on push button to perform following actions */
    printf("Wait for event on push button.\n\r");
    while ((ubUserButtonClickEvent) == RESET)
    {
      BSP_LED_Toggle(BSP_LED2);
      HAL_Delay(200);
    }
    
    /* Reset variable for next loop iteration */
    ubUserButtonClickEvent = RESET;
    
    if (HAL_RNG_GenerateRandomNumber(&hrng, &aRandom[counter]) != HAL_OK)
    {
      /* Random number generation error */
      Error_Handler();      
    }
    else
    {
      printf("Value of generated random number: %#X\n\r", aRandom[counter]);
    }
  }
  
  printf("End of the example.\n\r");
  
  if (HAL_RNG_DeInit(&hrng) != HAL_OK)
  {
    Error_Handler();
  }  
  
  BSP_LED_Off(BSP_LED2);
  
  /* Infinite loop */
  while (1)
  {
  }
}

/**
* @brief RNG Initialization Function
* @param None
* @retval None
*/
static void MX_RNG_Init(void)
{
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
}   

/**
* @brief EXTI line detection callbacks
* @param GPIO_Pin: Specifies the pins connected EXTI line
* @retval None
*/
void HAL_GPIO_EXTI_Callback(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BSP_PUSH1_PIN)
  {
    /* Set variable to report push button event to main program */
    ubUserButtonClickEvent = SET;
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
    /* Toggle LED3 */
    BSP_LED_Toggle(BSP_LED2);
    HAL_Delay(1000);
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



