
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : USART_Rx_IT_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the USART functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  USART_Rx_IT/USART_Rx_IT_main.c
 * @brief This example shows how to configure GPIO and USART peripheral for continuously receiving characters
 * from HyperTerminal (PC) in Asynchronous mode using Interrupt mode. Peripheral initialization is 
 * done using LL unitary services functions for optimization purpose (performance and size).
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\USART\\USART_Rx_IT\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\USART_Rx_IT.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\USART\\USART_Rx_IT\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\USART_Rx_IT.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\USART\\USART_Rx_IT\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
  The application will listen for keys typed and it will send back in the serial port.
  In other words everything typed in serial port will be send back.
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Start bits      | 1                | bit       |
| Stop bits       | 1                | bit       |
| HW flow control | None             | bit       |
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
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage
This example shows how to configure GPIO and USART peripheral for continuously receiving characters from HyperTerminal (PC) in Asynchronous mode using Interrupt mode. 
Peripheral initialization is done using LL unitary services functions for optimization purpose (performance and size).

Example execution:
After startup from reset and system configuration and the USART RX Not Empty interrupt is enabled.
When character is received on USART Rx line, a RXNE interrupt occurs. 
Received data is read from USART RDR register and stored in user buffer.
A double buffer is available for received data, allowing continuous reception.
User could process data received in Buffer A, while buffer B is used for reception. 
When buffer B is full, it could be handled by user, while buffer A becomes active buffer for next reception, and so on.
Each time a reception buffer is full, user data process callback is called.
Data processing consists in echoing data buffer content on PC Com port.

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
#include "USART_Rx_IT_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define RX_BUFFER_SIZE   10

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/**
* @brief Text strings printed on PC Com port for user information
*/
uint8_t aTextInfoStart[] = "\r\nUSART Example : Enter 10 characters to fill reception buffers.\r\n";
uint8_t aTextInfoSwap1[] = "\r\n- Current RX buffer is full : ";
uint8_t aTextInfoSwap2[] = "\r\n- Reception will go on in alternate buffer\r\n\r\n";
uint8_t endMessage[] = "** Test successfully. ** \n\r\n\r";


/**
* @brief RX buffers for storing received data
*/
uint8_t aRXBufferA[RX_BUFFER_SIZE];
uint8_t aRXBufferB[RX_BUFFER_SIZE];
__IO uint32_t uwNbReceivedChars;
__IO uint32_t uwBufferReadyIndication;
uint8_t *pBufferReadyForUser;
uint8_t *pBufferReadyForReception;

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
static void MX_USART_Init(void);
void HandleContinuousReception(void);
void PrintInfo(uint8_t *String, uint32_t Size);
void UserDataTreatment(uint8_t *DataBuffer, uint32_t Size);
void StartReception(void);

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
  
  /* Initialize all configured peripheral */
  MX_USART_Init();
  
  /* Initiate Continuous reception */
  StartReception();
    
  /* Infinite loop */
  while (1)
  {    
    /* Handle Continuous reception */
    HandleContinuousReception();
  }
}

static void LL_Init(void)
{
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
}


/**
* @brief USART1 Initialization Function
* @param None
* @retval None
*/
static void MX_USART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* Peripheral clock enable */
  LL_EnableClock_USART();

  GPIO_InitStruct.Pin = USART1_TX_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = USART1_TX_AF;
  LL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = USART1_RX_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = USART1_RX_AF;
  LL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);  
  
  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(USART1_IRQn);
  
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  /* USART Enable */
  LL_USART_Enable(USART1);
  
  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  { 
  }
  
  /* Clear Overrun flag, in case characters have already been sent to USART */
  LL_USART_ClearFlag_ORE(USART1);
  
  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);
}

/**
* @brief  This function prints user info on PC com port and initiates RX transfer
* @param  None
* @retval None
*/
void StartReception(void)
{
  /* Initializes Buffer swap mechanism :
  - 2 physical buffers aRXBufferA and aRXBufferB (RX_BUFFER_SIZE length)
  */
  pBufferReadyForReception = aRXBufferA;
  pBufferReadyForUser      = aRXBufferB;
  uwNbReceivedChars = 0;
  uwBufferReadyIndication = 0;
  
  /* Print user info on PC com port */
  PrintInfo(aTextInfoStart, sizeof(aTextInfoStart));
}

/**
* @brief  This function monitors buffer filling indication and calls User callbacks when a buffer is full
* @param  None
* @retval None
*/
void HandleContinuousReception(void)
{
  /* Checks if Buffer full indication has been set */
  if (uwBufferReadyIndication != 0)
  {
    /* Reset indication */
    uwBufferReadyIndication = 0;
    
    /* Call user Callback in charge of consuming data from filled buffer */
    UserDataTreatment(pBufferReadyForUser, RX_BUFFER_SIZE);
  }
}


/**
* @brief  Send Txt information message on USART Tx line (to PC Com port).
* @param  None
* @retval None
*/
void PrintInfo(uint8_t *String, uint32_t Size)
{
  uint32_t index = 0;
  uint8_t *pchar = String;
  
  /* Send characters one per one, until last char to be sent */
  for (index = 0; index < Size; index++)
  {
    /* Wait for TXE flag to be raised */
    while (!LL_USART_IsActiveFlag_TXE(USART1))
    {
    }
    
    /* Write character in Transmit Data register.
    TXE flag is cleared by writing data in TDR register */
    LL_USART_TransmitData8(USART1, *pchar++);
  }
  
  /* Wait for TC flag to be raised for last char */
  while (!LL_USART_IsActiveFlag_TC(USART1))
  {
  }
}

/**
* @brief  Example of User callback in charge of consuming received data.
* @param  None
* @retval None
*/
void UserDataTreatment(uint8_t *DataBuffer, uint32_t Size)
{
  /* Display info message + buffer content on PC com port */
  PrintInfo(aTextInfoSwap1, sizeof(aTextInfoSwap1));
  PrintInfo(DataBuffer, Size);
  PrintInfo(aTextInfoSwap2, sizeof(aTextInfoSwap2));
  
  PrintInfo(endMessage, sizeof(endMessage));
  
  /* Initiate Continuous reception */
  StartReception();
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT Functions                                     */
/******************************************************************************/


/**
* @brief  Function called from USART IRQ Handler when RXNE flag is set
*         Function is in charge of reading character received on USART RX line.
* @param  None
* @retval None
*/
void USART_CharReception_Callback(void)
{
  uint8_t *ptemp;
  
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  pBufferReadyForReception[uwNbReceivedChars++] = LL_USART_ReceiveData8(USART1);
  
  /* Checks if Buffer full indication has been set */
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
* @brief  Function called in case of error detected in USART IT Handler
* @param  None
* @retval None
*/
void Error_Callback(void)
{
  __IO uint32_t isr_reg;
  
  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USART1_IRQn);
  
  /* Error handling example :
  - Read USART ISR register to identify flag that leads to IT raising
  - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USART1, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* case Noise Error flag is raised : Clear NF Flag */
    LL_USART_ClearFlag_NE(USART1);
  }
  else
  {
    /* Unexpected IT source */
    Error_Handler();
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


