
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : SPI_HD_P_IT_M_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the SPI functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  SPI_HD_P_IT_M/SPI_HD_P_IT_M_main.c
 * @brief Data buffer transmission/reception between two boards via SPI using Polling (LL driver) and Interrupt modes (HAL driver).
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\SPI\\SPI_HD_P_IT_M\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\SPI_HD_P_IT_M.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\SPI\\SPI_HD_P_IT_M\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\SPI_HD_P_IT_M.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\SPI\\SPI_HD_P_IT_M\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A13    |       SPI1 SCK      |
|     A14    |       Not Used      |
|     A15    |       Not Used      |
|     A4     |       Not Used      |
|     A5     |       Not Used      |
|     A6     |       Not Used      |
|     A7     |       Not Used      |
|     A8     |       USART TX      |
|     A9     |       USART RX      |
|     B0     |       Not Used      | 
|     B14    |       SPI1 MOSI     |
|     B2     |       Not Used      |
|     B3     |       Not Used      |
|     B4     |       Not Used      |
|     B5     |       Not Used      |
|     B7     |       Not Used      |
|     B8     |         DL2         |
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
|  LED name  |              STEVAL-IDB011V1             |              STEVAL-IDB011V2             |
-----------------------------------------------------------------------------------------------------
|     DL1    |                 Not Used                 |                 Not Used                 |
|     DL2    |                 Not Used                 |                 Not Used                 |
|     DL3    |                 ON: error                |                 ON: error                |
|     DL4    |                 Not Used                 |                 Not Used                 |
|     U5     |  Fast blinking: waiting for user action  |  Fast blinking: waiting for user action  |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |
---------------------------------------------------------------------
|      PUSH1     |   Start communication  |   Start communication  |
|      PUSH2     |        Not Used        |        Not Used        |
|      RESET     |    Reset BlueNRG-LP    |    Reset BlueNRG-LP    |

@endtable

* \section Usage Usage

Data buffer transmission/reception between two boards via SPI using Polling (LL driver) and Interrupt modes (HAL driver).

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals, initialize the Flash interface and the systick.

The SPI peripheral configuration is ensured by the HAL_SPI_Init() function.
This later is calling the HAL_SPI_MspInit()function which core is implementing the configuration of the needed SPI resources according to the used hardware (CLOCK, GPIO). 
You may update this function to change SPI configuration.
The Half-Duplex SPI transmission (8bit) is done using LL Driver on Master board (Tx) by using function LL_SPI_TransmitData8.
The The Half-Duplex SPI reception (8bit) is done using HAL Driver on Slave board (Rx) by using function HAL_SPI_Receive_IT.

Example execution:
First step, press the User push-button (PUSH1), this action initiates a Half-Duplex transfer between Master and Slave.
After end of transfer, aRxBuffer and aTxBuffer are compared through Buffercmp() in order to check buffers correctness.

BlueNRG_LP board's LEDs can be used to monitor the transfer status:
 - LED1 toggles quickly on master board waiting User push-button (PUSH1) to be pressed.
 - LED2 turns ON on slave board if reception is complete and OK.
 - LED3 turns ON when there is an error in reception process. 

You need to perform a reset on Slave board, then perform it on Master board to have the correct behavior of this example.

In order to make the program work, you must do the following:
 - Launch serial communication SW on PC
 - Flash the project in the Board
 - Press the RESET button

BlueNRG_LP-EVB Set-up
- Connect Master board SPI CLK pin to Slave Board SPI CLK pin
- Connect Master board SPI MOSI pin to Slave Board SPI MISO pin
- Connect Master board GND to Slave Board GND
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
#include "SPI_HD_P_IT_M_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspiMaster;

#if defined( CONFIG_DATASIZE_16BIT )
/* Buffer used for transmission */
uint16_t aTxBuffer[] = {0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF};
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer)/2;
__IO uint16_t ubNbDataToReceive = sizeof(aTxBuffer)/2;
/* Buffer used for reception */
uint16_t aRxBuffer[sizeof(aTxBuffer)/2];

#elif defined( CONFIG_DATASIZE_8BIT ) 

/* Buffer used for transmission */
uint8_t aTxBuffer[] = {0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF};
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer);
__IO uint16_t ubNbDataToReceive = sizeof(aTxBuffer);
/* Buffer used for reception */
uint8_t aRxBuffer[sizeof(aTxBuffer)];

#endif

__IO uint8_t ubNbDataTransmitted = 0;


/* transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_SPI_MASTER_Init(void);

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
  MX_GPIO_Init();
  MX_SPI_MASTER_Init();

  /* Configure LEDs */
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);

  printf("Master board\n\r");
  
#if defined( CONFIG_DATASIZE_16BIT ) 
  printf("SPI data width 16 bit\n\r");
#elif defined( CONFIG_DATASIZE_8BIT )
  printf("SPI data width 8 bit\n\r");
#endif 
  
  /* Configure User push-button (PUSH1) */
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_GPIO);

  /* Wait for User push-button (PUSH1) press before starting the Communication */
  printf("Wait for User push-button (PUSH1) press before starting the Communication.\n\r");
  
  while (BSP_PB_GetState(BSP_PUSH1) == GPIO_PIN_RESET)
  {
    BSP_LED_Toggle(BSP_LED1);
    HAL_Delay(200);
  }
  BSP_LED_Off(BSP_LED1);

  /*-1- Start the Half Duplex Communication process */
  /* Half Duplex Direction (Tx) not Done by HAL_Init. */
  LL_SPI_SetTransferDirection(hspiMaster.Instance, LL_SPI_HALF_DUPLEX_TX);
  
  /* Enable SPI before start transmission */
  LL_SPI_Enable(hspiMaster.Instance);
  
  while(ubNbDataToTransmit > 0)
  {
    /* Check TXE flag to transmit data */
    if(LL_SPI_IsActiveFlag_TXE(hspiMaster.Instance))
    {
#if defined( CONFIG_DATASIZE_16BIT ) 
      /* Transmit 16bit Data */
      LL_SPI_TransmitData16(hspiMaster.Instance, aTxBuffer[ubNbDataTransmitted++]);
#elif defined( CONFIG_DATASIZE_8BIT )
     /* Transmit 8bit Data */
      LL_SPI_TransmitData8(hspiMaster.Instance, aTxBuffer[ubNbDataTransmitted++]);
#endif
      ubNbDataToTransmit--;
    }
  }
  
  /* Wait End Of Transmission: TXE set and Tx Fifo empty */
  printf("Wait End Of Transmission: TXE set and Tx Fifo empty.\n\r");
  while((LL_SPI_IsActiveFlag_TXE(hspiMaster.Instance) != 1));
  while(LL_SPI_GetTxFIFOLevel(hspiMaster.Instance) != LL_SPI_TX_FIFO_EMPTY);
  
  /* Disable SPI after End of Transmission */
  LL_SPI_Disable(hspiMaster.Instance);

  printf("End of the example.\n\r");
  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief SPI_MASTER Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI_MASTER_Init(void)
{
  /* SPI_MASTER parameter configuration*/
  hspiMaster.Instance = SPI_MASTER;
  hspiMaster.Init.Mode = SPI_MODE_MASTER;
  hspiMaster.Init.Direction = SPI_DIRECTION_1LINE;
  hspiMaster.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspiMaster.Init.CLKPhase = SPI_PHASE_2EDGE;
#ifdef CONFIG_DATASIZE_16BIT 
  hspiMaster.Init.DataSize = SPI_DATASIZE_16BIT;
#else // CONFIG_DATASIZE_8BIT
  hspiMaster.Init.DataSize = SPI_DATASIZE_8BIT;
#endif 
  hspiMaster.Init.NSS = SPI_NSS_SOFT;
  hspiMaster.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspiMaster.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspiMaster.Init.TIMode = SPI_TIMODE_DISABLE;
  hspiMaster.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspiMaster.Init.CRCPolynomial = 7;
  hspiMaster.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspiMaster.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspiMaster) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



