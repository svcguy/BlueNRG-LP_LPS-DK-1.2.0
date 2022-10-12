
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : I2S_Transmitter_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the I2S functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  I2S_Transmitter/I2S_Transmitter_main.c
 * @brief The application is designed to configure the I2S peripheral on slave board to transmit data to the master board. 
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\I2S\\I2S_Transmitter\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\I2S_Transmitter.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\I2S\\I2S_Transmitter\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\I2S_Transmitter.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\I2S\\I2S_Transmitter\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A12    |        I2S SD       |        N.A.        |
|     A13    |       Not Used      |        N.A.        |
|     A14    |       Not Used      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |        I2S WS       |        N.A.        |
|     A5     |        I2S CK       |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       USART TX      |       I2S SD       |
|     A9     |       USART RX      |       I2S WS       |
|     B0     |       Not Used      |      USART RX      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |       I2S CK       |
|     B4     |       Not Used      |      Not Used      |
|     B5     |       Not Used      |      Not Used      |
|     B7     |       Not Used      |      Not Used      |
|     B8     |       Not Used      |        N.A.        |
|     B9     |       I2S MCK       |        N.A.        |
|     A0     |         N.A.        |      Not Used      |
|     A10    |         N.A.        |      I2S MCK       |
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


Configuration of the I2S peripheral to transmit data to the master board. 
This example is based on the BLUENRG_LP I2S HAL API. 
The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size). 


In order to achieve the reeception data, the application performs the following steps:
- Initialize communication peripheral, I2S buses is configurad in Master Rx mode
- Master (receiver board) starts the communication after that the user press the User push-button (PUSH1) on the Master Board.

The data is stored into a circular buffer through the DMA peripheral of the I2S feature.
The slave (transmitter board) has to be enabled before the external master (receiver board) starts the communication.
  
In order to make the program work, you must do the following:
 - Launch serial communication SW on PC
 - Flash the project in the Board
 - Press the RESET button
   
BlueNRG_LP-EVB Set-up
- Connect BlueNRG_LP master board I2S_WS pin to slave board I2S_WS pin
- Connect BlueNRG_LP master board I2S_CK pin to slave board I2S_CK pin
- Connect BlueNRG_LP master board I2S_SD pin to slave board I2S_SD pin 
Connect USART1 TX/RX to respectively RX and TX pins of PC UART (could be done through a USB to UART adapter) :
- Connect BlueNRG_LP board USART1 TX pin to PC COM port RX signal
- Connect BlueNRG_LP board USART1 RX pin  to PC COM port TX signal
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
#include "I2S_Transmitter_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define DUMMY_BUFFER 320

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s;

/* Private function prototypes -----------------------------------------------*/
static void MX_I2S_Init(void);

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
  
  /* Initialize LED2 */
  BSP_LED_Init(BSP_LED2);
  
  /* I2S configuration */
  MX_I2S_Init();  
  
  /* Initialize Righ and Left channels to Audio Output buffer*/
  // to avoid the PLL Locked error use 320
  uint16_t dummy[DUMMY_BUFFER]={0};
  uint32_t i=0;
  
  while(i<DUMMY_BUFFER)
  {
    dummy[i++]= 0x0000; /*Left Channel*/
    dummy[i++]= 0xFFFF; /*Right Channel*/
  }
  
  /* Helpful to see on analyzer the start of the frame */
  dummy[0]= 0xAAAA; /*Left Channel*/
  dummy[1]= 0xBBBB; /*Right Channel*/

  printf("Slave Transmitter\n\r");
  
  /* Non-Blocking mode: DMA */
  if(HAL_I2S_Transmit_DMA(&hi2s, dummy, 320) != HAL_OK) 
  {
    printf("Non-Blocking mode: DMA ended with error.\n\r");
    Error_Handler();
  }

  /* Infinite loop */
  while (1)
  {
  }
}

/**
* @brief I2S Initialization Function
* @param None
* @retval None
*/
static void MX_I2S_Init(void)
{
  /* Disable I2S peripheral to allow access to I2S internal registers */
  __HAL_I2S_DISABLE(&hi2s);
  
  /* Configure I2S */
  hi2s.Instance             = I2S_INSTANCE;
  hi2s.Init.AudioFreq       = I2S_AUDIOFREQ_32K;  
  hi2s.Init.CPOL            = I2S_CPOL_LOW;
  hi2s.Init.DataFormat	    = I2S_DATAFORMAT_16B;
  // uncomment to use MCK - MCK enbable
//  hi2s.Init.MCLKOutput      = I2S_MCLKOUTPUT_ENABLE;
  // comment to use MCK - MCK enbable
  hi2s.Init.MCLKOutput      = I2S_MCLKOUTPUT_DISABLE;
  hi2s.Init.Mode            = I2S_MODE_SLAVE_TX;
  hi2s.Init.Standard        = I2S_STANDARD_PHILIPS;
  
  I2S_CLK_ENABLE();
  
  /* Configure SPIxI2S clock source */
  I2S_CLK_CONFIG();
  
  /* Initialization I2S function */
  if(HAL_I2S_Init(&hi2s) != HAL_OK)
  {
    printf("Initializes the I2S ended with error.\n\r");
    Error_Handler();
  }
  
  /* Disable I2S peripheral */
  __HAL_I2S_ENABLE(&hi2s);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  BSP_LED_On(BSP_LED2);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  BSP_LED_Off(BSP_LED2);
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



