
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : FLASH_EraseProgram_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 27-May-2019
* Description        : Code demonstrating the basic Flash functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  FLASH_EraseProgram/FLASH_EraseProgram_main.c
 * @brief How to configure and use the FLASH HAL API to erase and program the internal Flash memory.
 *


* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\FLASH\\FLASH_EraseProgram\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\FLASH_EraseProgram.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\FLASH\\FLASH_EraseProgram\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\FLASH_EraseProgram.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
    -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\FLASH\\FLASH_EraseProgram\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  LED name  |                                STEVAL-IDB011V1                               |                                STEVAL-IDB011V2                               |                                STEVAL-IDB012V1                               |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                                   Not Used                                   |                                   Not Used                                   |                                   Not Used                                   |
|     DL2    |  ON: no errors after data programming - Fast Blinking: wait for user action  |  ON: no errors after data programming - Fast Blinking: wait for user action  |  ON: no errors after data programming - Fast Blinking: wait for user action  |
|     DL3    |                            ON: program is running                            |                            ON: program is running                            |                            ON: program is running                            |
|     DL4    |                                   Not Used                                   |                                   Not Used                                   |                                   Not Used                                   |
|     U5     |                       ON: errors after data programming                      |                       ON: errors after data programming                      |                       ON: errors after data programming                      |

@endtable

* \section Buttons_description Buttons description
@table
|   BUTTON name  |           STEVAL-IDB011V1          |           STEVAL-IDB011V2          |           STEVAL-IDB012V1          |
------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |   it is used to start application  |   it is used to start application  |   it is used to start application  |
|      PUSH2     |              Not Used              |              Not Used              |              Not Used              |
|      RESET     |          Reset BlueNRG-LP          |          Reset BlueNRG-LP          |          Reset BlueNRG-LP          |

@endtable

* \section Usage Usage

How to configure and use the FLASH HAL API to erase and program the internal Flash memory.

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals, initialize the Flash interface and the systick.

After Reset, the Flash memory Program/Erase Controller is locked. 
A dedicated function is used to enable the FLASH control register access.
Before programming the desired addresses, an erase operation is performed using the flash erase  feature. 
The erase procedure is done by filling the erase init structure giving the starting erase  and the number of s to erase.
At this stage, all these s will be erased one by one separately.

Once this operation is finished,  double-word programming operation will be performed 
in the Flash memory. The written data is then read back and checked.

The BlueNRG_LP-EVB board LEDs can be used to monitor the transfer status:
 - LED1 is ON when there are errors detected after data programming 
 - LED2 is ON when there are no errors detected after data programming 
 - LED3 is ON when there is an issue during erase or program procedure

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
#include "FLASH_EraseProgram_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define FLASH_USER_START_ADDR   (FLASH_END_ADDR - FLASH_PAGE_SIZE + 1)   
#define FLASH_USER_END_ADDR     (FLASH_END_ADDR + 1)
#define DATA_32                 ((uint32_t)0x12345678)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint32_t FirstPage = 0, NbOfPages = 0;
uint32_t Address = 0, PageError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static uint32_t GetPage(uint32_t Address);

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
  
  /* Initialize LED1, LED2 and LED3 */
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  
  /* Configure User push-button (PUSH1) button */
  BSP_PB_Init(BSP_PUSH1,BUTTON_MODE_GPIO);
  
  /* Wait for User push-button (PUSH1) press to trigger random numbers generation */
  WaitForUserButtonPress();
  
  /* Get the 1st page to erase */
  FirstPage = GetPage(FLASH_USER_START_ADDR);
  
  /* Get the number of pages to erase from 1st page */
  NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage;
  
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPE_ERASE_PAGES;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;
  
  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
  you have to make sure that these data are rewritten before they are accessed during code
  execution. If this cannot be done safely, it is recommended to flush the caches by setting the
  DCRST and ICRST bits in the FLASH_CR register. */
  printf("Perform a mass erase or erase the specified FLASH memory pages.\n\r");
  printf("FLASH_USER_START_ADDR 0x%08X \n\r ", FLASH_USER_START_ADDR);
  printf("FLASH_USER_END_ADDR   0x%08X \n\r ", FLASH_USER_END_ADDR);
  printf("FirstPage               %d \n\r ", FirstPage);
  printf("NbOfPages               %d \n\r ", NbOfPages);

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    /*
    Error occurred while erase.
    User can add here some code to deal with this error.
    PageError will contain the faulty  and then to know the code error on this ,
    user can call function 'HAL_FLASH_GetError()'
    */
    printf("Error occurred while erase.\n\r");
    /* Infinite loop */
    while (1)
    {
      BSP_LED_On(BSP_LED3);
    }
  }
  
  /* Program the user Flash area word by word
  (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
  printf("Program the user Flash area word by word.\n\r");
  Address = FLASH_USER_START_ADDR;
  
  while (Address < FLASH_USER_END_ADDR)
  {
    if (HAL_FLASH_Program(Address, DATA_32) == HAL_OK)
    {
      Address = Address + 4;
    }
    else
    {
      /* Error occurred while writing data in Flash memory.
      User can add here some code to deal with this error */
      printf("Error occurred while writing data in Flash memory.\n\r");
      while (1)
      {
        BSP_LED_On(BSP_LED3);
      }
    }
  }
  
  /* 
  Check if the programmed data is OK
  MemoryProgramStatus = 0: data programmed correctly
  MemoryProgramStatus != 0: number of words not programmed correctly 
  */
  Address = FLASH_USER_START_ADDR;
  MemoryProgramStatus = 0x0;
  printf("Check the programmed data:\n\r");
  while (Address < FLASH_USER_END_ADDR)
  {
    data32 = *(__IO uint32_t *)Address;
    
    if (data32 != DATA_32)
    {
      MemoryProgramStatus++;
    }
    Address = Address + 4;
  }
  
  /*Check if there is an issue to program data*/
  if (MemoryProgramStatus == 0)
  {
    /* No error detected. Switch on LED2*/
    printf("No error detected.\n\r");
    BSP_LED_On(BSP_LED2);
  }
  else
  {
    /* Error detected. Switch on LED1*/
    printf("Error detected.\n\r");
    BSP_LED_On(BSP_LED1);
  }
  
  /* Infinite loop */
  while (1)
  {
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
}


/**
* @brief  Wait for User push-button (PUSH1) press to start the application.
* @param  None 
* @retval None
*/
void WaitForUserButtonPress(void)
{
  printf("Wait for User push-button (PUSH1) press to start the application.\n\r");
  /* Wait for User push-button (PUSH1) press before starting the Communication */
  while (BSP_PB_GetState(BSP_PUSH1) != GPIO_PIN_RESET)
  {
    BSP_LED_Toggle(BSP_LED2);
    HAL_Delay(200);
  }
  while (BSP_PB_GetState(BSP_PUSH1) != GPIO_PIN_SET)
  {
    BSP_LED_Toggle(BSP_LED2);
    HAL_Delay(200);
  }
  printf("PUSH1 pressed.\n\r");
}

/**
* @brief  Gets the page of a given address
* @param  Addr: Address of the FLASH Memory
* @retval The page of a given address
*/
static uint32_t GetPage(uint32_t Addr)
{
  if(!IS_ADDR_ALIGNED_32BITS(Addr)){
    while(1);
  }
  uint32_t page = (Addr - FLASH_START_ADDR) / FLASH_PAGE_SIZE;
  if(!IS_FLASH_PAGE(page)){
    while(1);
  }
  return page;
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
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


