
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : DMA_RAMToRAM_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the DMA functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file DMA_RAMToRAM/DMA_RAMToRAM_main.c
 * @brief How to use a DMA channel to transfer a word data buffer 
 * from embedded SRAM to embedded SRAM. The peripheral initialization uses 
 * LL unitary service functions for optimization purposes (performance and size).
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\DMA\\DMA_RAMToRAM\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\DMA_RAMToRAM.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\DMA\\DMA_RAMToRAM\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\DMA_RAMToRAM.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\DMA\\DMA_RAMToRAM\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  LED name  |       STEVAL-IDB011V1      |       STEVAL-IDB011V2      |       STEVAL-IDB012V1      |
--------------------------------------------------------------------------------------------------------
|     DL1    |          Not Used          |          Not Used          |          Not Used          |
|     DL2    |  ON: transfer is complete  |  ON: transfer is complete  |  ON: transfer is complete  |
|     DL3    |          ON: error         |          ON: error         |          ON: error         |
|     DL4    |          Not Used          |          Not Used          |          Not Used          |
|     U5     |     ON: transfer error     |     ON: transfer error     |     ON: transfer error     |

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

How to use a DMA to transfer a word data buffer from embedded SRAM to embedded SRAM through the HAL API.

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals and the systick.

DMA1_Channel1 is configured to transfer the contents of a 32-word data buffer stored in RAM to the reception buffer declared in RAM.

The start of transfer is triggered by software. DMA1_Channel1 memory-to-memory transfer is enabled. Source and destination addresses incrementing is also enabled.
The transfer is started by setting the channel enable bit for DMA1_Channel1.
At the end of the transfer a Transfer Complete interrupt is generated since it is enabled and the callback function (customized by user) is called.

BlueNRG_LP-EVB board's LEDs can be used to monitor the transfer status:
 - LED1 is ON when there is a transfer error 
 - LED2 is ON when the transfer is complete (into the Transfer Complete interrupt
   routine).
 - LED3 is ON when a Error_Handler is called
 
It is possible to select a different channel for the DMA transfer example by modifying defines values in the file DMA_RAMToRAM_main.h.


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
#include "DMA_RAMToRAM_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
DMA_HandleTypeDef hdma_memtomem_dma1_channel1;

static uint32_t aSRC_Buffer[BUFFER_SIZE] =
{
  0x01020304, 0x05060708, 0x090A0B0C, 0x0D0E0F10,
  0x11121314, 0x15161718, 0x191A1B1C, 0x1D1E1F20,
  0x21222324, 0x25262728, 0x292A2B2C, 0x2D2E2F30,
  0x31323334, 0x35363738, 0x393A3B3C, 0x3D3E3F40,
  0x41424344, 0x45464748, 0x494A4B4C, 0x4D4E4F50,
  0x51525354, 0x55565758, 0x595A5B5C, 0x5D5E5F60,
  0x61626364, 0x65666768, 0x696A6B6C, 0x6D6E6F70,
  0x71727374, 0x75767778, 0x797A7B7C, 0x7D7E7F80
};

static uint32_t aDST_Buffer[BUFFER_SIZE];

static __IO uint32_t transferErrorDetected;    /* Set to 1 if an error transfer is detected */
static __IO uint32_t transferCompleteDetected; /* Set to 1 if transfer is correctly completed */

/* Private function prototypes -----------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void MX_DMA_Init(void);
static void TransferComplete(DMA_HandleTypeDef *hdma_memtomem_dma1_channel1);
static void TransferError(DMA_HandleTypeDef *hdma_memtomem_dma1_channel1);
void PrintBuffer(uint32_t* pBuffer, uint32_t BufferLength);
void CompareBuffer(void);
uint8_t Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint32_t BufferLength);

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
  
  printf("Initial buffer data:\n\r");  
  PrintBuffer((uint32_t*)aSRC_Buffer, BUFFER_SIZE);
  PrintBuffer((uint32_t*)aDST_Buffer, BUFFER_SIZE);
  
  /* Initialize all configured peripherals */
  MX_DMA_Init();
  
  /* Initialize LEDs */
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED3);
  
  /* Infinite loop */
  /* Reset transferErrorDetected to 0, it will be set to 1 if a transfer error is detected */
  transferErrorDetected = 0;
  /* Reset transferCompleteDetected to 0, it will be set to 1 if a transfer is correctly completed */
  transferCompleteDetected = 0;

  /* Select Callbacks functions called after Transfer complete and Transfer error */
  HAL_DMA_RegisterCallback(&hdma_memtomem_dma1_channel1, HAL_DMA_XFER_CPLT_CB_ID, TransferComplete);
  HAL_DMA_RegisterCallback(&hdma_memtomem_dma1_channel1, HAL_DMA_XFER_ERROR_CB_ID, TransferError);
  
  /* Configure the source, destination and buffer size DMA fields and Start DMA Channel/Stream transfer */
  /* Enable All the DMA interrupts */
  if (HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel1, (uint32_t)&aSRC_Buffer, (uint32_t)&aDST_Buffer, BUFFER_SIZE) != HAL_OK)
  {
    /* Transfer Error */
    Error_Handler();
  }

  /* Infinite loop */
  while (1)
  {
    if (transferErrorDetected == 1)
    {
      /* Turn LED1 on*/
      BSP_LED_On(BSP_LED1);
      transferErrorDetected = 0;
      HAL_Delay(1000);
      BSP_LED_Off(BSP_LED3);
    }
    if (transferCompleteDetected == 1)
    {
      CompareBuffer();
      transferCompleteDetected = 0;
      HAL_Delay(1000);
      BSP_LED_Off(BSP_LED3);
    }
  }
}

void PrintBuffer(uint32_t* pBuffer, uint32_t BufferLength)
{
  printf("Data:\n\r");
  for(int i=0; i<BufferLength; i++)
  {
    printf("0x");
    printf("0x%08X ", pBuffer[i++]);
    printf("0x%08X ", pBuffer[i++]);
    printf("0x%08X ", pBuffer[i++]);
    printf("0x%08X ", pBuffer[i]);
    printf("\n\r");
  }
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0: pBuffer identical to pBuffer1
  *         1: pBuffer differs from pBuffer1
  */
uint8_t Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint32_t BufferLength)
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
  * @brief  DMA transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
void CompareBuffer()
{
  printf("DMA transfer completed\n\r");
  printf("Verify the data transfered\n\r");
  PrintBuffer((uint32_t*)aSRC_Buffer, BUFFER_SIZE);
  PrintBuffer((uint32_t*)aDST_Buffer, BUFFER_SIZE);
  
  /* DMA transfer completed */
  /* Verify the data transfered */
  if (Buffercmp((uint32_t*)aSRC_Buffer, (uint32_t*)aDST_Buffer, BUFFER_SIZE) == 1)
  {
    /* DMA data transfered not consistency */
    Error_Handler();
  }
  
  /* DMA data transfered consistency */
  printf("DMA data transfered consistency.\n\r");
  BSP_LED_On(BSP_LED2);
}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_channel1
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
  hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
  hdma_memtomem_dma1_channel1.Init.Request = DMA_REQUEST_MEM2MEM;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK)
  {
    Error_Handler();
  }

  /* DMA interrupt init */
  /* DMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA_IRQn, IRQ_HIGH_PRIORITY);
  HAL_NVIC_EnableIRQ(DMA_IRQn);
}
 
/**
  * @brief  DMA conversion complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void TransferComplete(DMA_HandleTypeDef *hdma_memtomem_dma1_channel1)
{
    transferCompleteDetected = 1;
}

/**
  * @brief  DMA conversion error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
static void TransferError(DMA_HandleTypeDef *hdma_memtomem_dma1_channel1)
{
    transferErrorDetected = 1;
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED3 on: Transfer Error */
  BSP_LED_On(BSP_LED3);
  while (1)
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
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



