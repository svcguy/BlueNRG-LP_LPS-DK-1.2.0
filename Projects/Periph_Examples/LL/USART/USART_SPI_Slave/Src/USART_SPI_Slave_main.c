
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : USART_SPI_Slave_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the USART syncronous functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  USART_SPI_Slave/USART_SPI_Slave_main.c
 * @brief How to configure USART peripheral to send characters synchronously to/from 
 * the LL SPI_IT_Master_Init example in DMA mode.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\USART\\USART_SPI_Slave\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\USART_SPI_Slave.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\USART\\USART_SPI_Slave\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\USART_SPI_Slave.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\USART\\USART_SPI_Slave\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A1     |       USART CS      |      USART TX      |
|     A11    |       Not Used      |      Not Used      |
|     A12    |       Not Used      |        N.A.        |
|     A13    |       Not Used      |        N.A.        |
|     A14    |       Not Used      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |       Not Used      |        N.A.        |
|     A5     |       Not Used      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       USART RX      |      Not Used      |
|     A9     |       USART TX      |      Not Used      |
|     B0     |       Not Used      |      USART RX      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |      Not Used      |
|     B4     |       Not Used      |        DL2         |
|     B5     |       Not Used      |      Not Used      |
|     B7     |       Not Used      |      USART NSS     |
|     B8     |       USART CK      |        N.A.        |
|     B9     |       Not Used      |        N.A.        |
|     A0     |         N.A.        |      Not Used      |
|     A10    |         N.A.        |      Not Used      |
|     B1     |         N.A.        |      USART CS      |
|     B6     |         N.A.        |      Not Used      |
|     B15    |         N.A.        |      USART TX      |
|     GND    |       Not Used      |      Not Used      |
|     RST    |       Not Used      |      Not Used      |
|    VBAT    |       Not Used      |      Not Used      |
@endtable 



* \section LEDs_description LEDs description
@table
|  LED name  |         STEVAL-IDB011V1        |         STEVAL-IDB011V2        |         STEVAL-IDB012V1        |
--------------------------------------------------------------------------------------------------------------------
|     DL1    |            Not Used            |            Not Used            |            Not Used            |
|     DL2    |            Not Used            |            Not Used            |            Not Used            |
|     DL3    |  ON: OK; Slow blinking: error  |  ON: OK; Slow blinking: error  |  ON: OK; Slow blinking: error  |
|     DL4    |            Not Used            |            Not Used            |            Not Used            |
|     U5     |            Not Used            |            Not Used            |            Not Used            |

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

This example shows how to configure GPIO and USART peripheral to send/receive characters synchronously to/from the LL SPI SPI_IT_Master_Init in DMA mode. 
This example is based on BLUENRG_LP USART LL API. 
Peripheral initialization is done using LL unitary services functions for optimization purpose (performance and size).

Example execution:
After startup from reset and system configuration, the master LED is blinking quickly and wait User push-button (PUSH1) action.
USART and DMA are configured.
On press on push button , TX DMA transfer to SPI Slave application is initailize.
RX DMA transfer is also ready to retrieve characters from MISO pin.

In case of both transfers successfully completed, LEDx is turned on.
In case of errors, LEDx is blinking (1sec period).

In order to make the program work, you must do the following:
- Launch serial communication SW on PC
- Flash the project in the Board
- Press the RESET button

To test this SPI Slave application was been used the LL/SPI/SPI_IT_Master_Init with the SPI configured as SPI1 and the data size set to 8bit.
It is mandatory add to the slave application the follows define in the main.h file:
 #define CONFIG_DATASIZE_8BIT  1
 #define USE_SPI1_PINS         1

   |                  | STEVAL_IDB012V1 |
   |    MASTER        |      SLAVE      |
   |  SPI Pin/Signal  | USART Pin/Signal|
   |     SCK          |   PB1 / CK      |
   |     MISO         |   PA1 / TX      |
   |     MOSI         |   PB0 / RX      |
   |     NSS          |   PB7 / NSS     |
   
   Note: NSS and nCTS share the same pin.
   LEDx is turned on in case of success. 
   The aRxBuffer contains the received string.


**/
   

/* Includes ------------------------------------------------------------------*/
#include "USART_SPI_Slave_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint8_t ubButtonPress = 0;
__IO uint8_t ubSend = 0;
/* Buffer used for transmission */
uint8_t aTxBuffer[] = "**** SPI_IT communication **** SPI_IT communication **** SPI_IT communication ****";
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer);
__IO uint8_t ubTransmissionComplete = 0;

/* Buffer used for reception */
uint8_t ubNbDataToReceive = sizeof(aTxBuffer);
uint8_t aRxBuffer[sizeof(aTxBuffer)];
__IO uint32_t uwNbReceivedChars = 0;
__IO uint8_t ubReceptionComplete = 0;


/* Private function prototypes -----------------------------------------------*/
static void MX_DMA_Init(void);
static void MX_USART_Init(void);
static void LL_Init(void);
void StartReceptions(void);
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
  
  /* Initialization of LEDs */
  BSP_LED_Init(BSP_LED3);
  /* Initialize User push-button (PUSH1) in EXTI mode */
  UserButton_Init();
  
  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_USART_Init();
   
  /* Initiate DMA transfers */
  StartReceptions();
  
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
* @brief USART1 Initialization Function
* @param None
* @retval None
*/
static void MX_USART_Init(void)
{
  
  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_USART_ClockInitTypeDef USART_ClockInitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* Peripheral clock enable */
  LL_EnableClock_USART();
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
  
  GPIO_InitStruct.Pin = USART1_CK_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = USART1_CK_AF;
  LL_GPIO_Init(USART1_CK_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = USART1_TX_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = USART1_TX_AF;
  LL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = USART1_RX_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = USART1_RX_AF;
  LL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);
  
  /* CS */
  GPIO_InitStruct.Pin = USART1_NSS_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = USART1_NSS_AF;
  LL_GPIO_Init(USART1_NSS_PORT, &GPIO_InitStruct);
  
  /* Disable pull-up and pull-down configuration */
  LL_PWR_DisablePUPDCfg();
  
  /* USART1 DMA Init */
  
  /* USART1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_7, LL_DMAMUX_REQ_USART1_TX);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MDATAALIGN_BYTE);
  
  /* USART1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_6, LL_DMAMUX_REQ_USART1_RX);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);
  
 
  /* Enable DMA transfer complete/error interrupts  */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_7);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_6);

  USART_ClockInitStruct.ClockOutput = LL_USART_CLOCK_ENABLE;
  USART_ClockInitStruct.ClockPhase = LL_USART_PHASE_2EDGE;
  USART_ClockInitStruct.ClockPolarity = LL_USART_POLARITY_HIGH;
  /* The LBCL (used only on SPI master mode), CPOL and CPHA bits have to be selected when the USART is disabled (UE=0) to ensure that the clock pulses function correctly */
  USART_ClockInitStruct.LastBitClockPulse = LL_USART_LASTCLKPULSE_OUTPUT; /* USART_CR2_LBCL */ 
  LL_USART_ClockInit(USART1, &USART_ClockInitStruct);
  
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 0; // not used
  /* When the peripheral is used in SPI slave mode, the peripheral clock source (fck_pres) must be greater than 3xSCLK input clock */
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX; //LL_USART_DIRECTION_RX
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_DisableSCLKOutput(USART1);
  LL_USART_DisableLIN(USART1);
  LL_USART_DisableSmartcard(USART1);
  LL_USART_DisableIrda(USART1);
  LL_USART_DisableHalfDuplex(USART1);
  LL_USART_EnableSPISlave(USART1);
  LL_USART_EnableSPISlaveSelect(USART1);
  
  LL_USART_SetTransferBitOrder(USART1, LL_USART_BITORDER_MSBFIRST);
  
  LL_USART_Enable(USART1);
  
  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  
  LL_USART_ClearFlag_UDR(USART1);
  LL_USART_ClearFlag_ORE(USART1);
  
  /* Enable RXNE and Error interrupts */
  LL_USART_DisableIT_RXNE(USART1);
  LL_USART_DisableIT_TXE_TXFNF(USART1);
  LL_USART_EnableIT_ERROR(USART1);
}

/** 
* Enable DMA controller clock
*/
static void MX_DMA_Init(void)
{
  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB_EnableClock(LL_AHB_PERIPH_DMA); 
  
  /* DMA interrupt init */
  /* DMA_IRQn interrupt configuration */
  NVIC_SetPriority(DMA_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(DMA_IRQn);  
}


/**
* @brief  This function initiates TX and RX DMA transfers by enabling DMA channels
* @param  None
* @retval None
*/
void StartReceptions(void)
{
  
  /* Configure the DMA functional parameters for transmission */
  LL_DMA_ConfigAddresses(DMA1, 
                         LL_DMA_CHANNEL_7,
                         (uint32_t)aTxBuffer,
                         LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT),
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7));
  LL_DMA_SetDataLength(DMA1, 
                       LL_DMA_CHANNEL_7, 
                       ubNbDataToTransmit);
  
  /* Configure the DMA functional parameters for reception */
  LL_DMA_ConfigAddresses(DMA1, 
                         LL_DMA_CHANNEL_6,
                         LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE),
                         (uint32_t)aRxBuffer,
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6));
  LL_DMA_SetDataLength(DMA1, 
                       LL_DMA_CHANNEL_6, 
                       ubNbDataToReceive);
  
  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(USART1);
  
  /* Enable DMA TX Interrupt */
  LL_USART_EnableDMAReq_TX(USART1);
  
  /* Enable DMA Channel Rx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);
  
  /* Enable DMA Channel Tx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);
 
}

/**
* @brief  Configures User push-button (PUSH1) in GPIO or EXTI Line Mode.
* @param  None 
* @retval None
*/
void UserButton_Init(void)
{
  /* Enable the BUTTON Clock */
  USER_BUTTON_GPIO_CLK_ENABLE();
  USER_BUTTON_SYSCFG_CLK_ENABLE();
  
  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);
  
  /* Enable a rising trigger External line 10 Interrupt */
  USER_BUTTON_EXTI_LINE_ENABLE();
  USER_BUTTON_EXTI_RISING_TRIG_ENABLE();
  
  /* Clear the event occurred on the interrupt line 10 port A. */
  if (LL_EXTI_IsInterruptPending(USER_BUTTON_EXTI_LINE) != RESET)
  {
    LL_EXTI_ClearInterrupt(USER_BUTTON_EXTI_LINE);
  }
  
  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);
  
  /* Configure NVIC for SysTick_IRQn */
  NVIC_SetPriority(SysTick_IRQn, IRQ_LOW_PRIORITY);
}

/**
* @brief  Wait for User push-button (PUSH1) press to start transfer.
* @param  None
* @retval None
*/
/*  */
void WaitForUserButtonPress(void)
{
  while (ubButtonPress == 0)
  {
    BSP_LED_Toggle(BSP_LED3);
    LL_mDelay(LED_BLINK_FAST);
  }
}

/**
* @brief  Wait end of transfer and check if received Data are well.
* @param  None
* @retval None
*/
void WaitAndCheckEndOfTransfer(void)
{
  /* Wait end of transmission */
  while (ubTransmissionComplete != 1)
  {
  }
  ubTransmissionComplete = 0;
  
  /* Wait end of reception */
  while (ubReceptionComplete != 1)
  {
  }
  /* Disable DMA1 Rx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
  
  /* Compare received string to expected one */
  if (Buffercmp8((uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, ubNbDataToReceive))
  {
    /* Processing Error */
    Error_Handler();
  }
  else
  {
    /* Turn On Led if data are well received */
    BSP_LED_On(BSP_LED3);  
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
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  aRxBuffer[uwNbReceivedChars++] = LL_USART_ReceiveData8(USART1);
  
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
* @brief  Function called from DMA1 IRQ Handler when Rx transfer is completed
* @param  None
* @retval None
*/
void DMA1_ReceiveComplete_Callback(void)
{
  /* DMA Rx transfer completed */
  ubReceptionComplete = 1;
}

/**
* @brief  Function called in case of error detected in USART IT Handler
* @param  None
* @retval None
*/
void USART_TransferError_Callback(void)
{
  /* Disable DMA1 Rx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
  
  /* Set LED2 to Blinking mode to indicate error occurs */
  Error_Handler();
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
    BSP_LED_Toggle(BSP_LED3);
    LL_mDelay(LED_BLINK_ERROR);
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


