
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : UART_TxPolling_RxIT_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 27-May-2019
* Description        : Code demonstrating the use of a UART to transmit data (transmit/receive) in polling/interrupt mode 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  UART_TxPolling_RxIT/UART_TxPolling_RxIT_main.c
 * @brief Use of a UART to transmit data (transmit/receive)  between a board and an HyperTerminal PC application both in Polling and Interrupt  modes.  
 *
 
* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\UART\\UART_TxPolling_RxIT\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\UART_TxPolling_RxIT.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\UART\\UART_TxPolling_RxIT\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\UART_TxPolling_RxIT.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\UART\\UART_TxPolling_RxIT\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - UART configuration


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
|  PIN name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------
|     A0     |        N.A.        |        N.A.        |      Not Used      |
|     A1     |      Not Used      |      Not Used      |      Not Used      |
|     A10    |        N.A.        |        N.A.        |      Not Used      |
|     A11    |      Not Used      |      Not Used      |      Not Used      |
|     A12    |      Not Used      |      Not Used      |        N.A.        |
|     A13    |      Not Used      |      Not Used      |        N.A.        |
|     A14    |      Not Used      |      Not Used      |        N.A.        |
|     A15    |      Not Used      |      Not Used      |        N.A.        |
|     A3     |        N.A.        |        N.A.        |      Not Used      |
|     A4     |      Not Used      |      Not Used      |        N.A.        |
|     A5     |      Not Used      |      Not Used      |        N.A.        |
|     A6     |      Not Used      |      Not Used      |        N.A.        |
|     A7     |      Not Used      |      Not Used      |        N.A.        |
|     A8     |      Not Used      |      Not Used      |      Not Used      |
|     A9     |      Not Used      |      Not Used      |        N.A.        |
|     B0     |      Not Used      |      Not Used      |      Not Used      |
|     B1     |        N.A.        |        N.A.        |      Not Used      |
|     B12    |        N.A.        |        N.A.        |      Not Used      |
|     B13    |        N.A.        |        N.A.        |      Not Used      |
|     B14    |      Not Used      |      Not Used      |      Not Used      |
|     B15    |        N.A.        |        N.A.        |      Not Used      |
|     B2     |      Not Used      |      Not Used      |      Not Used      |
|     B3     |      Not Used      |      Not Used      |      Not Used      |
|     B4     |      Not Used      |      Not Used      |      Not Used      |
|     B5     |      Not Used      |      Not Used      |      Not Used      |
|     B6     |        N.A.        |        N.A.        |      Not Used      |
|     B7     |      Not Used      |      Not Used      |      Not Used      |
|     B8     |      Not Used      |      Not Used      |        N.A.        |
|     B9     |      Not Used      |      Not Used      |        N.A.        |
|     GND    |      Not Used      |      Not Used      |      Not Used      |
|     RST    |      Not Used      |      Not Used      |      Not Used      |
|    VBAT    |      Not Used      |      Not Used      |      Not Used      |

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
|  LED name  |                                STEVAL-IDB011V1                               |                                STEVAL-IDB011V2                               |                                STEVAL-IDB012V1                               |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                                   Not Used                                   |                                   Not Used                                   |                                   Not Used                                   |
|     DL2    |   it toggles each time transmission/reception of 10 characters is completed  |   it toggles each time transmission/reception of 10 characters is completed  |   it toggles each time transmission/reception of 10 characters is completed  |
|     DL3    |          ON when there is an error in transmission/reception process         |          ON when there is an error in transmission/reception process         |          ON when there is an error in transmission/reception process         |
|     DL4    |                                   Not Used                                   |                                   Not Used                                   |                                   Not Used                                   |
|     U5     |                                   Not Used                                   |                                   Not Used                                   |                                   Not Used                                   |

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

Use of a UART to transmit data (transmit/receive) between a board and an HyperTerminal PC application both in Polling and Interrupt modes. 
This example describes how to use the USART peripheral through the BLUENRG_LP UART HAL and LL API, the LL API being used for performance improvement.               

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals, initialize the Flash interface and the systick.

The UART peripheral configuration is ensured by the HAL_UART_Init() function.
This later is calling the HAL_UART_MspInit()function which core is implementing the configuration of the needed UART resources according to the used hardware.
You may update this function to change UART configuration.

Reception under IT on UART/Hyperterminal interface is then enabled using LL APIs.
Each 0.5 sec, a predefined TX buffer is sent from MCU to PC reception of Data from Hyperterminal.
The Asynchronous communication aspect of the UART is clearly highlighted as the data buffers transmission/reception to/from Hyperterminal are done simultaneously.
Transmit function which allow transmission of a predefined data buffer in polling mode, is implemented using HAL UART API.
Receive function which allow the reception of Data from Hyperterminal in IT mode, is implemented using LL USART API.
A double buffer is available for received data, allowing continuous reception.
Data received in Buffer A could be echoed on TX, while buffer B is used for reception. 
When buffer B is full, it is then echoed, while buffer A becomes active buffer for next reception, and so on.

For this example the TX buffer (aTxStartMessage) is predefined and the RX buffers (aRXBufferA and aRXBufferB) size is limited to 10 data by the mean of the RXBUFFERSIZE define in the UART_TxPolling_RxIT_main.c file.

Received data will be stored in the RX buffer. 
When RX buffer is full, RX buffer is sent back to PC and led toggles.

The Asynchronous communication aspect of the UART is clearly highlighted as the data buffers transmission/reception to/from Hyperterminal are done simultaneously.

BlueNRG_LP Nucleo board's LEDs can be used to monitor the transfer status:
 - LED2 toggles each time transmission/reception of 10 characters is completed..
 - LED3 stays ON when there is an error in transmission/reception process.  

Launch serial communication SW on PC (as HyperTerminal or TeraTerm) with proper configuration.

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
#include "UART_TxPolling_RxIT_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* Buffer used for transmission */
uint8_t aTxStartMessage[] = "\n\r\n\r ****UART-Hyperterminal TXRX communication (TX based on HAL polling API, RX based on IT LL API) ****\n\r Enter characters using keyboard ...\n\r";
uint8_t ubSizeToSend = sizeof(aTxStartMessage);

/* Buffer used for reception */
uint8_t aRXBufferA[RX_BUFFER_SIZE];
uint8_t aRXBufferB[RX_BUFFER_SIZE];
__IO uint32_t uwNbReceivedChars = 0;
__IO uint32_t uwBufferReadyIndication = 0;
uint8_t *pBufferReadyForUser;
uint8_t *pBufferReadyForReception;

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_USART_Init(void);

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
  
  /* Configure led */
  BSP_LED_Init(BSP_LED3);
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART_Init();
  
  /*-2- Configure UART peripheral for reception process (using LL) */  
  /* Initializes Buffer swap mechanism :
  2 physical buffers aRXBufferA and aRXBufferB (RX_BUFFER_SIZE length)
  Any data received will be stored in active buffer : the number max of 
  data received is RX_BUFFER_SIZE */
  pBufferReadyForReception = aRXBufferA;
  pBufferReadyForUser      = aRXBufferB;
  uwNbReceivedChars = 0;
  uwBufferReadyIndication = 0;
  
  /* Enable RXNE and Error interrupts */  
  LL_USART_EnableIT_RXNE(USARTx);
  LL_USART_EnableIT_ERROR(USARTx);
  
  /*-3- Start the transmission process (using HAL Polling mode) */  
  /* In main loop, Tx buffer is sent every 0.5 sec. 
  As soon as RX buffer is detected as full, received bytes are echoed on TX line to PC com port */
  
  /* Infinite loop */
  while (1)
  {
    /* USART IRQ handler is not anymore routed to HAL_UART_IRQHandler() function 
    and is now based on LL API functions use. 
    Therefore, use of HAL IT based services is no more possible : use TX HAL polling services */
    if(HAL_UART_Transmit(&huart1, (uint8_t*)aTxStartMessage, ubSizeToSend, 1000)!= HAL_OK)
    {
      /* Transfer error in transmission process */
      Error_Handler();
    }
    
    /* Wait : checks if Buffer full indication has been set */
    while(uwBufferReadyIndication == 0);
    
    /* Reset indication */
    uwBufferReadyIndication = 0;
    
    /* USART IRQ handler is not anymore routed to HAL_UART_IRQHandler() function 
    and is now based on LL API functions use. 
    Therefore, use of HAL IT based services is no more possible : use TX HAL polling services */
    if(HAL_UART_Transmit(&huart1, (uint8_t*)pBufferReadyForUser, RX_BUFFER_SIZE, 1000)!= HAL_OK)
    {
      /* Transfer error in transmission process */
      Error_Handler();
    }
    
    /* Manage temporisation between TX buffer sendings */
    HAL_Delay(500);
  }
}

/**
* @brief USARTx Initialization Function
* @param None
* @retval None
*/
static void MX_USART_Init(void)
{
  huart1.Instance = USARTx;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/**
* @brief  Rx Transfer completed callback
* @note   This example shows a simple way to report end of IT Rx transfer, and 
*         you can add your own implementation.
* @retval None
*/
void UART_CharReception_Callback(void)
{
  uint8_t *ptemp;
  
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  pBufferReadyForReception[uwNbReceivedChars++] = LL_USART_ReceiveData8(USARTx);
  
  /* Checks if Buffer full indication has to be set */
  if (uwNbReceivedChars >= RX_BUFFER_SIZE)
  {
    /* Set Buffer swap indication */
    uwBufferReadyIndication = 1;
    
    /* Swap buffers for next bytes to be received */
    ptemp = pBufferReadyForUser;
    pBufferReadyForUser = pBufferReadyForReception;
    pBufferReadyForReception = ptemp;
    uwNbReceivedChars = 0;
  }
}

/**
* @brief  UART error callbacks
* @note   This example shows a simple way to report transfer error, and you can
*         add your own implementation.
* @retval None
*/
void UART_Error_Callback(void)
{
  __IO uint32_t isr_reg;
  
  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USARTx_IRQn);
  
  /* Error handling example :
  - Read USART ISR register to identify flag that leads to IT raising
  - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USARTx, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* Turn LED3 on: Transfer error in reception/transmission process */
    BSP_LED_On(BSP_LED3);
  }
  else
  {
    /* Turn LED3 off */
    BSP_LED_Off(BSP_LED3);
  }
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  /* Toggle LED3 for error */
  while(1)
  {
    BSP_LED_Toggle(BSP_LED3);
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



