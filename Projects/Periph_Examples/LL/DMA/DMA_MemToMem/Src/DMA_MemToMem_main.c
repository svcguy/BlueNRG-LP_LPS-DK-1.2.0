
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : DMA_MemToMem_main.c
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
 * @file DMA_MemToMem/DMA_MemToMem_main.c
 * @brief How to use a DMA channel to transfer a word data buffer 
 * from embedded SRAM to embedded SRAM. The peripheral initialization uses 
 * LL unitary service functions for optimization purposes (performance and size).
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\DMA\\DMA_MemToMem\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\DMA_MemToMem.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\DMA\\DMA_MemToMem\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\DMA_MemToMem.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\DMA\\DMA_MemToMem\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  LED name  |          STEVAL-IDB011V1         |          STEVAL-IDB011V2         |          STEVAL-IDB012V1         |
--------------------------------------------------------------------------------------------------------------------------
|     DL1    |             Not Used             |             Not Used             |             Not Used             |
|     DL2    |   ON: OK - Slow blinking: error  |   ON: OK - Slow blinking: error  |   ON: OK - Slow blinking: error  |
|     DL3    |             Not Used             |             Not Used             |             Not Used             |
|     DL4    |             Not Used             |             Not Used             |             Not Used             |
|     U5     |             Not Used             |             Not Used             |             Not Used             |

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

How to use a DMA channel to transfer a word data buffer from embedded SRAM to embedded SRAM. 
The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size).

Then the LED_Init() function is used to initialize the LED2.

Then the Configure_DMA() function is used to configure the DMA1 to transfer the contents of a 32-word data buffer stored in embedded SRAM to the reception buffer declared in RAM.

The start of transfer is triggered by software(LL_DMA_EnableChannel()). 
DMA1 Channel1 memory-to-memory transfer is enabled. 
Source and destination addresses incrementing is also enabled.
The transfer is started by setting the channel enable bit for DMA1_Channel1.
At the end of the transfer a Transfer Complete interrupt is generated since it is enabled and the callback function (customized by user) is called.

Finally, aSRC_Const_Buffer and aDST_Buffer are compared through Buffercmp() in order to check buffers correctness.  

BlueNRG_LP-EVB's LED2 can be used to monitor the transfer status:
- LED2 is turn on if DMA data transferred consistency.
- LED2 is blinking every 1 sec in case of error.


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
#include "DMA_MemToMem_main.h"

/** @addtogroup Peripherals_LL_Examples
  * @{
  */

/** @addtogroup DMA_MemToMem
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE              32

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
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

/* Private function prototypes -----------------------------------------------*/
void Configure_DMA(void);
uint8_t Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint32_t BufferLength);
void LED_Init(void);
void LED_On(void);
void LED_Blinking(uint32_t Period);
void PrintBuffer(uint32_t* pBuffer, uint32_t BufferLength);
static void LL_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
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
  BSP_COM_Init(NULL);

  printf("The transfer is started by setting the channel enable bit for DMA1_Channel1.\n\r");  
  PrintBuffer((uint32_t*)aSRC_Buffer, BUFFER_SIZE);
  PrintBuffer((uint32_t*)aDST_Buffer, BUFFER_SIZE);
  
  /* Initialize LED2 */
  LED_Init();

  /* Configure DMA1 Channel 1                                    */
  /* Enable DMA1 clock                                           */
  /* Configure the DMA functionnal parameters :                  */
  /*        - Transfer memory word to memory word in normal mode */
  /*        - Memory and Periph increment mode                   */
  /* Configure NVIC for DMA transfer complete/error interrupts   */
  Configure_DMA();

  /* Start the DMA transfer Flash to Memory */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  
  /* Infinite loop */
  while (1)
  {
    /* Transfer completion and comparison is done on interrupt callback directly */
  }
}

static void LL_Init(void)
{
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
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
  * @brief  This function configures the DMA1 Channel 1 to copy data from
  *         Internal SRAM(aSRC_Buffer) to Internal SRAM(aDST_Buffer).
  * @note   This function is used to :
  *         -1- Enable DMA1 clock
  *         -2- Configure the DMA functionnal parameters
  *         -3- Configure NVIC for DMA transfer complete/error interrupts
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param   None
  * @retval  None
  */
void Configure_DMA(void)
{
  /* DMA controller clock enable */
  LL_AHB_EnableClock(LL_AHB_PERIPH_DMA);

  /* Configure the DMA functionnal parameters */
  /* Configuration of the DMA parameters can be done using unitary functions or using the specific configure function */ 
  /* Unitary Functions */
  /* Set transfer direction */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_MEMORY_TO_MEMORY);

  /* Set priority level */
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);

  /* Set DMA mode */
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

  /* Set peripheral increment mode */
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_INCREMENT);

  /* Set memory increment mode */
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  /* Set peripheral data width */
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);

  /* Set memory data width */
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);
     
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, BUFFER_SIZE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&aSRC_Buffer, (uint32_t)&aDST_Buffer, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1));

  /* Enable DMA transfer complete/error interrupts */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
  NVIC_SetPriority(DMA_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(DMA_IRQn);
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
  * @brief  Initialize LED2.
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
  /* Enable the LED2 Clock */
  LED2_GPIO_CLK_ENABLE();

  /* Configure IO in output push-pull mode to drive external LED2 */
  LL_GPIO_SetPinMode(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  LL_GPIO_SetPinOutputType(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  LL_GPIO_SetPinSpeed(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  /* Reset value is LL_GPIO_PULL_NO */
  LL_GPIO_SetPinPull(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_PULL_NO);

  /* Turn LED2 off */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
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
  /* Turn LED2 on */
  LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
  
  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);  
    LL_mDelay(Period);
  }
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
  * @brief  DMA transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
void TransferComplete()
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
    LED_Blinking(LED_BLINK_ERROR);
  }
  
  /* DMA data transfered consistency */
  printf("DMA data transfered consistency.\n\r");
  LED_On();
  printf("** Test successfully. ** \n\r\n\r");

}

/**
  * @brief  DMA transfer error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
void TransferError()
{
  /* Error detected during DMA transfer */
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
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



