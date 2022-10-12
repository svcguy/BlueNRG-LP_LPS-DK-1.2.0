
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : SPI_IT_Master_Init_main.c
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
 * @file  SPI_IT_Master_Init/SPI_IT_Master_Init_main.c
 * @brief Data buffer transmission and reception via SPI using Interrupt mode. This 
 * example is based on the BLUENRG_LP SPI LL API. The peripheral 
 * initialization uses LL unitary service functions for optimization purposes (performance and size).
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\SPI\\SPI_IT_Master_Init\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\SPI_IT_Master_Init.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\SPI\\SPI_IT_Master_Init\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\SPI_IT_Master_Init.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\SPI\\SPI_IT_Master_Init\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c 8bit - 8 bit data lenght configuration
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
|     A11    |       SPI2 CS       |      SPI3 MOSI     |
|     A12    |       SPI2 MOSI     |        N.A.        |
|     A13    |       Not Used      |        N.A.        |
|     A14    |       Not Used      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |       Not Used      |        N.A.        |
|     A5     |       SPI2 SCK      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       SPI2 MISO     |        N.A.        |
|     A8     |       USART TX      |      SPI3 MISO     |
|     A9     |       USART RX      |      SPI3 CS       |
|     B0     |       Not Used      |      USART RX      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |      SPI3 SCK      |
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
|            |                                                                                                    Release                                                                                                    |||                                                                                                     8bit                                                                                                      |||
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |                           STEVAL-IDB011V1                          |                           STEVAL-IDB011V2                          |                           STEVAL-IDB012V1                          |                           STEVAL-IDB011V1                          |                           STEVAL-IDB011V2                          |                           STEVAL-IDB012V1                          |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                              Not Used                              |                              Not Used                              |                              Not Used                              |                              Not Used                              |                              Not Used                              |                              Not Used                              |
|     DL2    |   ON: OK - Fast blinking: wait User action - Slow blinking: error  |   ON: OK - Fast blinking: wait User action - Slow blinking: error  |   ON: OK - Fast blinking: wait User action - Slow blinking: error  |   ON: OK - Fast blinking: wait User action - Slow blinking: error  |   ON: OK - Fast blinking: wait User action - Slow blinking: error  |   ON: OK - Fast blinking: wait User action - Slow blinking: error  |
|     DL3    |                              Not Used                              |                              Not Used                              |                              Not Used                              |                              Not Used                              |                              Not Used                              |                              Not Used                              |
|     DL4    |                              Not Used                              |                              Not Used                              |                              Not Used                              |                              Not Used                              |                              Not Used                              |                              Not Used                              |
|     U5     |                              Not Used                              |                              Not Used                              |                              Not Used                              |                              Not Used                              |                              Not Used                              |                              Not Used                              |

@endtable


* \section Buttons_description Buttons description
@table
|                |                                  Release                                  |||                                   8bit                                    |||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |     STEVAL-IDB012V1    |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |     STEVAL-IDB012V1    |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |   Start communication  |   Start communication  |   Start communication  |   Start communication  |   Start communication  |   Start communication  |
|      PUSH2     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|      RESET     |    Reset BlueNRG-LP    |    Reset BlueNRG-LP    |    Reset BlueNRG-LP    |    Reset BlueNRG-LP    |    Reset BlueNRG-LP    |    Reset BlueNRG-LP    |

@endtable

* \section Usage Usage

Data buffer transmission and receptionvia SPI using Interrupt mode. 
This example is based on the BLUENRG_LP SPI LL API. 
The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size).

The communication is done with 2 boards through SPI.

This example shows how to configure GPIO and SPI peripherals to use a Full-Duplex communication using IT mode through the BLUENRG_LP COM_INSTANCE1_TYPE LL API.

This example is splitted in two projects, Master board and Slave board:

- Master Board
  SPI2 Peripheral is configured in Master mode.
  SPI2_IRQn activated. 
  RXNE and TXE Interrupts SPI peripheral activated.
  
- Slave Board
  SPI2 Peripheral is configured in Slave mode.
  SPI2_IRQn activated. 
  RXNE and TXE Interrupts SPI peripheral activated.


Example execution:
On BOARD MASTER, LED2 is blinking Fast (200ms) and wait User push-button (PUSH1) action.
Press User push-button (PUSH1) on BOARD MASTER start a Full-Duplex communication through IT.
On MASTER side, Clock will be generated on SCK line, Transmission(MOSI Line) and reception (MISO Line) will be done at the same time. 
SLAVE SPI will received  the Clock (SCK Line), so Transmission(MISO Line) and reception (MOSI Line) will be done also.

In order to make the program work, you must do the following:
 - Launch serial communication SW on PC
 - Flash the project in the Board
 - Press the RESET button

The CONFIG_DATASIZE_8BIT configuration is available to test this application with the LL USART_SPI_Slave.

BlueNRG_LP-EVB Set-up
- Connect Master board SPI CLK  pin to Slave Board SPI CLK  pin
- Connect Master board SPI MISO pin to Slave Board SPI MISO pin
- Connect Master board SPI MOSI pin to Slave Board SPI MOSI pin
- Connect Master board SPI CS   pin to Slave Board SPI CS   pin
- Connect Master board RST pin to Slave Board RST pin
- Connect Master board GND pin to Slave Board GND pin
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
#include "SPI_IT_Master_Init_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

__IO uint8_t waitChar = 0;

/* Buffer used for transmission */
#if defined( CONFIG_DATASIZE_16BIT )
uint16_t aTxBuffer[] = {0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF};
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer)/2;
#elif defined( CONFIG_DATASIZE_8BIT ) 
uint8_t aTxBuffer[] = "**** SPI_IT communication **** SPI_IT communication **** SPI_IT communication ****";
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer)/2;
#endif

__IO uint8_t ubTransmitIndex = 0;

/* Buffer used for reception */
#if defined( CONFIG_DATASIZE_16BIT )
static volatile uint16_t aRxBuffer[sizeof(aTxBuffer)/2];
uint8_t ubNbDataToReceive = sizeof(aTxBuffer)/2;
#elif defined( CONFIG_DATASIZE_8BIT )
uint8_t aRxBuffer[sizeof(aTxBuffer)];
uint8_t ubNbDataToReceive = sizeof(aTxBuffer);
#endif 
__IO uint8_t ubReceiveIndex = 0;

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);
static void MX_GPIO_Init(void);
static void MX_SPI_Master_Init(void);
/* Private user code ---------------------------------------------------------*/
void Activate_SPI(void);
void LED_On(void);
void LED_Blinking(uint32_t Period);
void LED_Off(void);
void WaitForUserButtonPress(void);
void WaitAndCheckEndOfTransfer(void);
uint8_t Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength);

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

#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Initialization of COM port */
  BSP_COM_Init(Process_InputData);
  
  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI_Master_Init();
  
  printf("Master board.\n\r");
#if defined( CONFIG_DATASIZE_16BIT ) 
  printf("SPI data width 16 bit\n\r");
#elif defined( CONFIG_DATASIZE_8BIT )
  printf("SPI data width 8 bit\n\r");
#endif 
  
  /* Wait for User push-button (PUSH1) press to start transfer */
  WaitForUserButtonPress();

  /* Enable the SPI_MASTER peripheral */
  LL_GPIO_ResetOutputPin(GPIO_PORT_MASTER_CS, GPIO_PIN_SPI_MASTER_CS);
  Activate_SPI();
  printf("Enable the SPI_MASTER peripheral.\n\r");
  
  /* Wait for the end of the transfer and check received data */
  /* LED blinking FAST during waiting time */
  WaitAndCheckEndOfTransfer();

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
  * @brief SPI_MASTER Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI_Master_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_SPI_Master_EnableClock();
  
  /* SCK */
  GPIO_InitStruct.Pin = GPIO_PIN_SPI_MASTER_SCK;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = GPIO_AF_SPI_MASTER_SCK;
  LL_GPIO_Init(GPIO_PORT_MASTER_SCK, &GPIO_InitStruct);
 
#ifdef SPI_MASTER_PULL_UP_V_2
  /* Pull Up */
  /* MOSI */
  GPIO_InitStruct.Pin = GPIO_PIN_SPI_MASTER_MOSI;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = GPIO_AF_SPI_MASTER_MOSI;
  LL_GPIO_Init(GPIO_PORT_MASTER_MOSI, &GPIO_InitStruct);
#else
  /* MOSI */
  GPIO_InitStruct.Pin = GPIO_PIN_SPI_MASTER_MOSI;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = GPIO_AF_SPI_MASTER_MOSI;
  LL_GPIO_Init(GPIO_PORT_MASTER_MOSI, &GPIO_InitStruct);
#endif

  /* MISO */
  GPIO_InitStruct.Pin = GPIO_PIN_SPI_MASTER_MISO;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = GPIO_AF_SPI_MASTER_MISO;
  LL_GPIO_Init(GPIO_PORT_MASTER_MISO, &GPIO_InitStruct);

  /* CS */
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
  LL_GPIO_SetOutputPin(GPIO_PORT_MASTER_CS, GPIO_PIN_SPI_MASTER_CS);
  GPIO_InitStruct.Pin = GPIO_PIN_SPI_MASTER_CS;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIO_PORT_MASTER_CS, &GPIO_InitStruct);
  
  
  /* SPI_MASTER interrupt Init */
  NVIC_SetPriority(SPI_MASTER_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(SPI_MASTER_IRQn);

  /* SPI_MASTER parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
#if defined( CONFIG_DATASIZE_16BIT ) 
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
#elif defined( CONFIG_DATASIZE_8BIT )
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
#endif   
  LL_SPI_Init(SPI_MASTER, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI_MASTER, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI_MASTER);

  if(SPI_InitStruct.DataWidth == LL_SPI_DATAWIDTH_16BIT)
  {
    /* Configure the SPI_MASTER FIFO Threshold */
    LL_SPI_SetRxFIFOThreshold(SPI_MASTER, LL_SPI_RX_FIFO_TH_HALF);
  }
  if(SPI_InitStruct.DataWidth == LL_SPI_DATAWIDTH_8BIT)
  {
    /* Configure the SPI_MASTER FIFO Threshold */
    LL_SPI_SetRxFIFOThreshold(SPI_MASTER, LL_SPI_RX_FIFO_TH_QUARTER);
  }
  
  /* Configure SPI_MASTER transfer interrupts */
  /* Enable TXE   Interrupt */
  LL_SPI_EnableIT_TXE(SPI_MASTER);
  /* Enable RXNE  Interrupt */
  LL_SPI_EnableIT_RXNE(SPI_MASTER);
  /* Enable SPI_MASTER Error Interrupt */
  LL_SPI_EnableIT_ERR(SPI_MASTER);
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
  USER_BUTTON_GPIO_CLK_ENABLE();
  USER_BUTTON_SYSCFG_CLK_ENABLE();
  
  /* Configure GPIO for LED */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
  GPIO_InitStruct.Pin = LED2_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
   
  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
}

/**
  * @brief  This function Activate SPI_MASTER
  * @param  None
  * @retval None
  */
void Activate_SPI(void)
{
  /* Enable SPI_MASTER */
  LL_SPI_Enable(SPI_MASTER);
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
  * @brief  Wait for User push-button (PUSH1) press to start transfer.
  * @param  None
  * @retval None
  */
  /*  */
void WaitForUserButtonPress(void)
{
  printf("Wait for User push-button (PUSH1) press or enter 'c'/'C' character.\n\r");
  while ( LL_GPIO_IsInputPinSet(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN) == USER_BUTTON_SETTED && waitChar == 0 )
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(LED_BLINK_FAST);
  }
  /* Ensure that LED2 is turned Off */
  LED_Off();
}

/**
  * @brief  Wait end of transfer and check if received Data are well.
  * @param  None
  * @retval None
  */
void WaitAndCheckEndOfTransfer(void)
{
  /* 1 - Wait end of transmission */
  while (ubTransmitIndex < (ubNbDataToTransmit-1))
  {
  }

  /* 2 - Wait end of reception */
  while (ubNbDataToReceive > ubReceiveIndex)
  {
  }

  /* 3 - Compare Transmit data to receive data */
  if(Buffercmp8((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, ubNbDataToTransmit))
  {
    /* Processing Error */
    printf("Processing Error.\n\r");
    LED_Blinking(LED_BLINK_ERROR);
  }
  else
  {
    /* Turn On Led if data are well received */
    LED_On();
    printf("Data are well received.\n\r");
    printf("** Test successfully. ** \n\r\n\r");
  }
}

/**
* @brief Compares two 8-bit buffers and returns the comparison result.
* @param pBuffer1: pointer to the source buffer to be compared to.
* @param pBuffer2: pointer to the second source buffer to be compared to the first.
* @param BufferLength: buffer's length.
* @retval   0: Comparison is OK (the two Buffers are identical)
*           Value different from 0: Comparison is NOK (Buffers are different)
*/
uint8_t Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return 1;
    }

    pBuffer1++;
    pBuffer2++;
  }
  return 0;
}


/**
  * @brief  Function called from SPI_MASTER IRQ Handler when RXNE flag is set
  *         Function is in charge of retrieving received byte from SPI lines.
  * @param  None
  * @retval None
  */
void  SPI_MASTER_Rx_Callback(void)
{
  /* Read character in Data register.
  RXNE flag is cleared by reading data in DR register */
#if defined( CONFIG_DATASIZE_16BIT ) 
  aRxBuffer[ubReceiveIndex++] = LL_SPI_ReceiveData16(SPI_MASTER);
#elif defined( CONFIG_DATASIZE_8BIT )
  aRxBuffer[ubReceiveIndex++] = LL_SPI_ReceiveData8(SPI_MASTER);
#endif 

  /* Check the end of reception */
  if(ubNbDataToReceive == ubReceiveIndex)
  {
    /* Disable RXNE Interrupt */
    LL_SPI_DisableIT_RXNE(SPI_MASTER);
    while(LL_SPI_GetTxFIFOLevel(SPI_MASTER)!=LL_SPI_TX_FIFO_EMPTY);
    while(LL_SPI_IsActiveFlag_BSY(SPI_MASTER));
    LL_SPI_Disable(SPI_MASTER);
    LL_GPIO_SetOutputPin(GPIO_PORT_MASTER_CS, GPIO_PIN_SPI_MASTER_CS);
  }
}

/**
  * @brief  Function called from SPI_MASTER IRQ Handler when TXE flag is set
  *         Function is in charge  to transmit byte on SPI lines.
  * @param  None
  * @retval None
  */
void  SPI_MASTER_Tx_Callback(void)
{
  /* Write character in Data register.
  TXE flag is cleared by reading data in DR register */
#if defined( CONFIG_DATASIZE_16BIT ) 
  LL_SPI_TransmitData16(SPI_MASTER, aTxBuffer[ubTransmitIndex++]);
#elif defined( CONFIG_DATASIZE_8BIT )
  LL_SPI_TransmitData8(SPI_MASTER, aTxBuffer[ubTransmitIndex++]);
#endif
  
  /* Check the end of transmission */
  if (ubTransmitIndex == ubNbDataToTransmit)
  {
    /* Disable TXE Interrupt */
    LL_SPI_DisableIT_TXE(SPI_MASTER);
  }
}

/**
  * @brief  Function called in case of error detected in SPI IT Handler
  * @param  None
  * @retval None
  */
void SPI_MASTER_TransferError_Callback(void)
{
  /* Disable RXNE  Interrupt             */
  LL_SPI_DisableIT_RXNE(SPI_MASTER);

  /* Disable TXE   Interrupt             */
  LL_SPI_DisableIT_TXE(SPI_MASTER);

  /* Set LED2 to Blinking mode to indicate error occurs */
  LED_Blinking(LED_BLINK_ERROR);
}


void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
  if(Nb_bytes>0)
  {
    if(data_buffer[0] == 'c' || data_buffer[0] == 'C' )
    {
      waitChar = 1;
    }
  }
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the LL error return state */
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



