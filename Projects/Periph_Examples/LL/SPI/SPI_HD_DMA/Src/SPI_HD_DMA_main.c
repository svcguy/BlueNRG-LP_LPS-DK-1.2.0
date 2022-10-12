
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : SPI_HD_DMA_main.c
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
 * @file  SPI_HD_DMA/SPI_HD_DMA_main.c
 * @brief Configuration of GPIO and SPI peripherals to transmit
 * bytes from an SPI Master device to an SPI Slave device in DMA mode.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\SPI\\SPI_HD_DMA\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\SPI_HD_DMA.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\SPI\\SPI_HD_DMA\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\SPI_HD_DMA.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\SPI\\SPI_HD_DMA\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A5     |       SPI2 SCK      |
|     A6     |       Not Used      |
|     A7     |       SPI2 MISO     |
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
|  LED name  |                           STEVAL-IDB011V1                          |                           STEVAL-IDB011V2                          |
---------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                              Not Used                              |                              Not Used                              |
|     DL2    |   ON: OK - Fast blinking: wait User action - Slow blinking: error  |   ON: OK - Fast blinking: wait User action - Slow blinking: error  |
|     DL3    |                              Not Used                              |                              Not Used                              |
|     DL4    |                              Not Used                              |                              Not Used                              |
|     U5     |                              Not Used                              |                              Not Used                              |

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

Configuration of GPIO and SPI peripherals to transmit bytes from an SPI Master device to an SPI Slave device in DMA mode. 
This example is based on the BLUENRG_LP SPI LL API. 
The peripheral initialization uses the LL initialization function to demonstrate LL init usage.


SPI1 Peripheral is configured in Master mode Half-Duplex Tx.
SPI2 Peripheral is configured in Slave mode Half-Duplex Rx.
GPIO associated to User push-button is linked with EXTI. 

Example execution:
LED2 is blinking Fast (200ms) and wait User push-button (PUSH1) action.
Press User push-button (PUSH1) on BOARD start a Half-Duplex communication through DMA.
On MASTER side (SPI1), Clock will be generated on SCK line, Transmission done on MOSI Line.
On SLAVE side (SPI2) reception is done through the MISO Line.

In order to make the program work, you must do the following:
 - Launch serial communication SW on PC
 - Flash the project in the Board
 - Press the RESET button

BlueNRG_LP-EVB Set-up
- Connect Master SPI1 CLK  pin to Slave SPI2 CLK pin
- Connect Master SPI1 MOSI pin to Slave SPI2 MISO pin
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
#include "SPI_HD_DMA_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
LL_SPI_InitTypeDef spi_initstruct;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint8_t ubButtonPress = 0;

/* Buffer used for transmission */
#if defined( CONFIG_DATASIZE_16BIT )
uint16_t aTxBuffer[] = {0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0xFFFF};
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer)/2;
#elif defined( CONFIG_DATASIZE_8BIT ) 
uint8_t aTxBuffer[] = "**** SPI_IT communication **** SPI_IT communication **** SPI_IT communication ****";
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer);
#endif

/* Buffer used for reception */
#if defined( CONFIG_DATASIZE_16BIT )
uint16_t aRxBuffer[sizeof(aTxBuffer)/2];
uint8_t ubNbDataToReceive = sizeof(aTxBuffer)/2;
#elif defined( CONFIG_DATASIZE_8BIT )
uint8_t aRxBuffer[sizeof(aTxBuffer)];
uint8_t ubNbDataToReceive = sizeof(aTxBuffer);
#endif 

__IO uint8_t ubTransmissionComplete = 0;
__IO uint8_t ubReceptionComplete = 0;

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void Activate_SPI1(void);
void Activate_SPI2(void);
void LED_On(void);
void LED_Blinking(uint32_t Period);
void LED_Off(void);
void UserButton_Init(void);
void WaitForUserButtonPress(void);
void WaitAndCheckEndOfTransfer(void);
uint8_t Buffercmp8(uint8_t *pBuffer1, uint8_t *pBuffer2, uint8_t BufferLength);

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

#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Initialization of COM port */
  BSP_COM_Init(Process_InputData);
 
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_DMA_Init();

#if defined( CONFIG_DATASIZE_16BIT ) 
  printf("SPI data width 16 bit\n\r");
#elif defined( CONFIG_DATASIZE_8BIT )
  printf("SPI data width 8 bit\n\r");
#endif 
  
  /* Enable the slave SPI2 peripheral */
  Activate_SPI2();

  printf("Enable the slave SPI2 peripheral.\n\r");
  
  /* Wait for User push-button (PUSH1) press to start transfer */
  WaitForUserButtonPress();

  /* Enable the master SPI1 peripheral */
  Activate_SPI1();

  printf("Enable the master SPI1 peripheral.\n\r");
  
  /* Wait for the end of the transfer and check received data */
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_EnableClock(LL_APB1_PERIPH_SPI1);
  LL_AHB_EnableClock(LL_AHB_PERIPH_DMA);
  
  /**SPI1 GPIO Configuration  
  PA13   ------> SPI1_SCK
  PB14   ------> SPI1_MOSI 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI1 DMA Init */

  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
#if defined( CONFIG_DATASIZE_16BIT ) 
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
#elif defined( CONFIG_DATASIZE_8BIT )
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
#endif 
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
  
  /* Initialize FFIFO Threshold */
#if defined( CONFIG_DATASIZE_16BIT )
  LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_HALF); 
#elif defined( CONFIG_DATASIZE_8BIT )
  LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
#endif 
  /* Configure SPI1 DMA transfer interrupts */
  /* Enable DMA TX Interrupt */
  LL_SPI_EnableDMAReq_TX(SPI1);
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_EnableClock(LL_APB1_PERIPH_SPI2);
  LL_AHB_EnableClock(LL_AHB_PERIPH_DMA);
  
  /**SPI2 GPIO Configuration  
  PA5   ------> SPI2_SCK
  PA7   ------> SPI2_MISO 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI2_RX Init */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_HALF_DUPLEX_RX;
  SPI_InitStruct.Mode = LL_SPI_MODE_SLAVE;
#if defined( CONFIG_DATASIZE_16BIT ) 
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
#elif defined( CONFIG_DATASIZE_8BIT )
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
#endif 
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
//SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI2);
  
#if defined( CONFIG_DATASIZE_16BIT ) 
  /* Initialize FFIFO Threshold */
  LL_SPI_SetRxFIFOThreshold(SPI2, LL_SPI_RX_FIFO_TH_HALF);
#elif defined( CONFIG_DATASIZE_8BIT )
    /* Initialize FFIFO Threshold */
  LL_SPI_SetRxFIFOThreshold(SPI2, LL_SPI_RX_FIFO_TH_QUARTER);
#endif 
  /* Configure SPI1 DMA transfer interrupts */
  /* Enable DMA RX Interrupt */
  LL_SPI_EnableDMAReq_RX(SPI2);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB_EnableClock(LL_AHB_PERIPH_DMA); 

  /* SPI2 DMA Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_SPI2_RX);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
#if defined( CONFIG_DATASIZE_16BIT ) 
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);  
#elif defined( CONFIG_DATASIZE_8BIT )
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);
#endif 
  
  /* SPI1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_SPI1_TX);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
#if defined( CONFIG_DATASIZE_16BIT ) 
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_HALFWORD);
#elif defined( CONFIG_DATASIZE_8BIT )
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);
#endif 
  
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)aTxBuffer, LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, ubNbDataToTransmit);
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_SPI1_TX);

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, LL_SPI_DMA_GetRegAddr(SPI2), (uint32_t)aRxBuffer, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ubNbDataToTransmit);
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_SPI2_RX);

  /* Enable DMA interrupts complete/error */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);  
  
  /* DMA interrupt init */
  /* DMA_IRQn interrupt configuration */
  NVIC_SetPriority(DMA_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(DMA_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  
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

  /* Turn Off Led2 */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
 
  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);

  /* Enable a rising trigger External line 10 Interrupt */
  EXTI_InitStruct.Line = LL_EXTI_LINE_PA10;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Type = LL_EXTI_TYPE_EDGE;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_EDGE;
  LL_EXTI_Init(&EXTI_InitStruct);

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);

  /* Configure NVIC for SysTick_IRQn */
  NVIC_SetPriority(SysTick_IRQn, IRQ_LOW_PRIORITY);
}

/**
  * @brief  This function Activate SPI1
  * @param  None
  * @retval None
  */
void Activate_SPI1(void)
{
  /* Enable SPI1 */
  LL_SPI_Enable(SPI1);
  /* Enable DMA Channels Tx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
}

/**
  * @brief  This function Activate SPI2
  * @param  None
  * @retval None
  */
void Activate_SPI2(void)
{
  /* Enable SPI2 */
  LL_SPI_Enable(SPI2);
  /* Enable DMA Channels Rx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
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
void WaitForUserButtonPress(void)
{
  printf("Wait for User push-button (PUSH1) press or enter 'c'/'C' character.\n\r");
  while (ubButtonPress == 0)
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
  while (ubTransmissionComplete != 1)
  {
  }
  /* Disable DMA1 Tx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
  /* 2 - Wait end of reception */
  while (ubReceptionComplete != 1)
  {
  }
  /* Disable DMA Rx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  /* 3 - Compare Transmit data to receive data */
  if (Buffercmp8((uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, sizeof(aTxBuffer)))
  {
    /* Processing Error */
    printf("Processing Error.\n\r");
    LED_Blinking(LED_BLINK_ERROR);
  }
  else
  {
    /* Turn On Led if data are well received */
    printf("Data are well received.\n\r");
    LED_On();
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
uint8_t Buffercmp8(uint8_t *pBuffer1, uint8_t *pBuffer2, uint8_t BufferLength)
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


/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT Functions                                     */
/******************************************************************************/
/**
  * @brief  Function to manage User push-button (PUSH1)
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  /* Update User push-button (PUSH1) variable : to be checked in waiting loop in main program */
  ubButtonPress = 1;
}

/**
  * @brief  Function called from DMA1 IRQ Handler when Tx transfer is completed
  * @param  None
  * @retval None
  */
void DMA1_TransmitComplete_Callback(void)
{
  /* DMA Tx transfer completed */
  ubTransmissionComplete = 1;
}

/**
  * @brief  Function called from DMA IRQ Handler when Rx transfer is completed
  * @param  None
  * @retval None
  */
void DMA_ReceiveComplete_Callback(void)
{
  /* DMA Rx transfer completed */
  ubReceptionComplete = 1;
}

/**
  * @brief  Function called in case of error detected in SPI IT Handler
  * @param  None
  * @retval None
  */
void SPI_TransferError_Callback(void)
{
  /* Disable DMA1 Tx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
  /* Disable DMA Rx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);

  /* Set LED2 to Blinking mode to indicate error occurs */
  LED_Blinking(LED_BLINK_ERROR);
}



void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
  if(Nb_bytes>0)
  {
    if(data_buffer[0] == 'c' || data_buffer[0] == 'C' )
    {
      UserButton_Callback();
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
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



