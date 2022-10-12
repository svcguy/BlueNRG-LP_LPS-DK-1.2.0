
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : USART_HFC_IT_main.c
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
 * @file  USART_HFC_IT/USART_HFC_IT_main.c
 * @brief This example shows how to configure GPIO and USART peripheral to send characters 
 *        asynchronously to/from HyperTerminal (PC) with hardware flow control mode in Interrupt mode. 
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\USART\\USART_HFC_IT\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\USART_HFC_IT.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\USART\\USART_HFC_IT\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\USART_HFC_IT.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\USART\\USART_HFC_IT\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A8     |       USART TX      |      USART RX      |
|     A9     |       USART RX      |      Not Used      |
|     B0     |       Not Used      |      Not Used      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |      USART nRTS     |      USART nRTS    |
|     B3     |      USART nCTS     |      USART nCTS    |
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
  The application will listen for keys typed and it will send back in the serial port.
  In other words everything typed in serial port will be send back.
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 9600             | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | Odd              | bit       |
| Start bits      | 1                | bit       |
| Stop bits       | 1                | bit       |
| HW flow control | CTS/RTS          | bit       |
@endtable


* \section LEDs_description LEDs description
@table
|  LED name  |                                        STEVAL-IDB011V1                                       |                                        STEVAL-IDB011V2                                       |                                        STEVAL-IDB012V1                                       |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                                           Not Used                                           |                                           Not Used                                           |                                           Not Used                                           |
|     DL2    |   ON: OK - Toggle: Specific value is received from the HyperTerminal - Slow blinking: error  |   ON: OK - Toggle: Specific value is received from the HyperTerminal - Slow blinking: error  |   ON: OK - Toggle: Specific value is received from the HyperTerminal - Slow blinking: error  |
|     DL3    |                                           Not Used                                           |                                           Not Used                                           |                                           Not Used                                           |
|     DL4    |                                           Not Used                                           |                                           Not Used                                           |                                           Not Used                                           |
|     U5     |                                           Not Used                                           |                                           Not Used                                           |                                           Not Used                                           |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |     STEVAL-IDB012V1    |
------------------------------------------------------------------------------------------------
|      PUSH1     |   Start communication  |   Start communication  |   Start communication  |
|      PUSH2     |        Not Used        |        Not Used        |        Not Used        |
|      RESET     |    Reset BlueNRG-LP    |    Reset BlueNRG-LP    |    Reset BlueNRG-LP    |

@endtable

* \section Usage Usage

This example shows how to configure GPIO and USART peripheral to send characters asynchronously to/from HyperTerminal (PC) with hardware flow control mode in Interrupt mode. 
This example is based on BLUENRG_LP USART LL API. 
Peripheral initialization is done using LL unitary services functions for optimization purpose (performance and size).

Example execution:
At the beginning of the application, USART TX Empty interrupt is enabled.
First character of buffer to be transmitted is written into USART Transmit Data Register (TDR) in order to initialize transfer procedure.
When character is sent from TDR, a TXE interrupt occurs. 
USART IRQ Handler routine is sending next character on USART Tx line. 
IT will be raised until last byte is to be transmitted : Then, Transmit Complete (TC) interrupt is enabled 
instead of TX Empty (TXE). 
When character is received on USART Rx line, a RXNE interrupt occurs. 
USART IRQ Handler routine is then checking received character value. 
On a specific value ('S' or 's'), LED2 is turned On/Off. 
Received character is echoed on Tx line.

In case of errors, LED2 is blinking (1sec period).

BlueNRG_LP-EVB Set-up. Connect:
STEVAL-IDB011V{1|2}:
  PA9--> TX   (FTDI Yellow)
  PA8--> RX   (FTDI Orange)  
  PB3--> CTS  (FTDI Green)
  PB2--> RTS  (FTDI Brown)
STEVAL-IDB012V1:
  PA1--> TX   (FTDI Yellow)
  PA8--> RX   (FTDI Orange)
  PB3--> CTS  (FTDI Green)
  PB2--> RTS  (FTDI Brown)

Launch serial communication SW on PC (as HyperTerminal or TeraTerm) with proper configuration :
- 9600 bauds
- 8 bits data
- 1 start bit
- 1 stop bit
- Odd parity
- HW flow control CTS RTS

Use the battery mode to execute this example.

**/
   
/* Includes */
#include "USART_HFC_IT_main.h"

/* Private includes */

/* Private typedef */

/* Private define */

/* Private macro */

/* Private variables */
__IO uint8_t ubButtonPress = 0;
__IO uint8_t ubSend = 0;
const uint8_t aStringToSend[] = "USART LL API Example : TX RX in IT mode\r\nConfiguration UART 115200 bps, 8 data bit/1 stop bit/No parity/No HW flow control\r\nInsert text: \r\nReceived character value is checked :\r\n- On a specific value (S or s), LED2 is toggled\r\n";
uint8_t ubSizeToSend = sizeof(aStringToSend);

/* Private function prototypes */
static void LL_Init(void);
static void MX_GPIO_Init(void);
static void MX_USART_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Blinking(uint32_t Period);
void LED_Toggle(void);

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
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART_Init();
  
  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  { 
  }
  
  /* Enable RXNE and Error interrupts */  
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);
  
  USART_Tx_Text();
  
  /* Set LED2 Off */
  LED_Off();
  
  /* Infinite loop */
  while (1)
  {
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
  LL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = USART1_CTS_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = USART1_CTS_AF;
  LL_GPIO_Init(USART1_CTS_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = USART1_RTS_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = USART1_RTS_AF;
  LL_GPIO_Init(USART1_RTS_PORT, &GPIO_InitStruct);
  
  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(USART1_IRQn);
  
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_ODD;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_RTS_CTS;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  LED2_GPIO_CLK_ENABLE();
  
  /* Configure GPIO for LED */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
  GPIO_InitStruct.Pin = LED2_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
  
}

/**
* @brief  Turn-on LED2.
* @param  None
* @retval None
*/
void LED_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
* @brief  Turn-off LED2.
* @param  None
* @retval None
*/
void LED_Off(void)
{
  /* Turn LED2 off */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
* @brief  Set LED2 to toggle once time
* @retval None
*/
void LED_Toggle(void)
{
  /* Toggle IO */
  LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
}

/**
* @brief  Set LED2 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
* @param  Period : Period of time (in ms) between each toggling of LED
*   This parameter can be user defined values. Pre-defined values used in that example are :
*     @arg LED_BLINK_FAST : Fast Blinking
*     @arg LED_BLINK_SLOW : Slow Blinking
*     @arg LED_BLINK_ERROR : Error specific Blinking
* @retval None
*/
void LED_Blinking(uint32_t Period)
{
  /* Toggle LED2 in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);  
    LL_mDelay(Period);
  }
}

/**
* @brief  Function called from USART IRQ Handler when RXNE flag is set
*         Function is in charge of reading character received on USART RX line.
* @param  None
* @retval None
*/
void USART_CharReception_Callback(void)
{
  __IO uint32_t received_char;
  
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  received_char = LL_USART_ReceiveData8(USART1);
  
  /* Check if received value is corresponding to specific one : S or s */
  if ((received_char == 'S') || (received_char == 's'))
  {
    /* Turn LED2 on/off : Expected character has been received */
    LED_Toggle();
  }
  
  /* Echo received character on TX */
  LL_USART_TransmitData8(USART1, received_char);
}

void USART_Tx_Text(void)
{
  /* Start transfer only if not already ongoing */
  if (ubSend == 0)
  {
    /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
    LL_USART_TransmitData8(USART1, aStringToSend[ubSend++]); 
    
    /* Enable TXE interrupt */
    LL_USART_EnableIT_TXE(USART1); 
  }
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT Functions                                     */
/******************************************************************************/

/**
* @brief  Function called for achieving next TX Byte sending
* @param  None
* @retval None
*/
void USART_TXEmpty_Callback(void)
{
  if(ubSend == (ubSizeToSend - 1))
  {
    /* Disable TXE interrupt */
    LL_USART_DisableIT_TXE(USART1);
    
    /* Enable TC interrupt */
    LL_USART_EnableIT_TC(USART1);
  }
  
  /* Fill TDR with a new char */
  LL_USART_TransmitData8(USART1, aStringToSend[ubSend++]);
}

/**
* @brief  Function called at completion of last byte transmission
* @param  None
* @retval None
*/
void USART_CharTransmitComplete_Callback(void)
{
  if(ubSend == sizeof(aStringToSend))
  {
    ubSend = 0;
    
    /* Disable TC interrupt */
    LL_USART_DisableIT_TC(USART1);
    
    /* Turn LED2 on at end of transfer : Tx sequence completed successfully */
    LED_On();
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
    /* case Noise Error flag is raised : ... */
    LED_Blinking(LED_BLINK_FAST);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
    LED_Blinking(LED_BLINK_ERROR);
  }
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  LED_Blinking(LED_BLINK_ERROR);
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


