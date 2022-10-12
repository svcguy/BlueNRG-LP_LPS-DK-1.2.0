
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : CORTEX_MPU_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the CORTEX functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  CORTEX_MPU/CORTEX_MPU_main.c
 * @brief This example configures a memory area as privileged read-only, and attempts to 
 *        perform read and write operations in
 * different modes.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\CORTEX\\CORTEX_MPU\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\CORTEX_MPU.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\CORTEX\\CORTEX_MPU\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\CORTEX_MPU.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\CORTEX\\CORTEX_MPU\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  LED name  |                 STEVAL-IDB011V1                |                 STEVAL-IDB011V2                |                 STEVAL-IDB012V1                |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                    Not Used                    |                    Not Used                    |                    Not Used                    |
|     DL2    |  ON: hard fault - toggle: access is permitted  |  ON: hard fault - toggle: access is permitted  |  ON: hard fault - toggle: access is permitted  |
|     DL3    |                    Not Used                    |                    Not Used                    |                    Not Used                    |
|     DL4    |                    Not Used                    |                    Not Used                    |                    Not Used                    |
|     U5     |                    Not Used                    |                    Not Used                    |                    Not Used                    |

@endtable

* \section Buttons_description Buttons description
@table
|   BUTTON name  |        STEVAL-IDB011V1       |        STEVAL-IDB011V2       |        STEVAL-IDB012V1       |
------------------------------------------------------------------------------------------------------------------
|      PUSH1     |  generate a fault exception  |  generate a fault exception  |  generate a fault exception  |
|      PUSH2     |           Not Used           |           Not Used           |           Not Used           |
|      RESET     |       Reset BlueNRG-LP       |       Reset BlueNRG-LP       |       Reset BlueNRG-LP       |

@endtable

* \section Usage Usage

Presentation of the MPU feature. This example configures a memory area as privileged read-only, and attempts to perform read and write operations in different modes.

If the access is permitted LED2 is toggling. If the access is not permitted, a Hard fault is generated and LED2 is ON.

To generate an MPU memory fault exception due to an access right error, press user button.


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
#include "CORTEX_MPU_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PRIVILEGED_READ_ONLY_SIZE                256U
#define PRIVILEGED_READ_ONLY_ADDRESS_START       (0x20002000UL)
#define REGION_SIZE                              LL_MPU_REGION_SIZE_256B
#define REGION_NUMBER                            LL_MPU_REGION_NUMBER3
#define EXAMPLE_RAM_ADDRESS_START                (0x20000000UL)
#define EXAMPLE_RAM_SIZE                         LL_MPU_REGION_SIZE_256KB
#define EXAMPLE_PERIPH_ADDRESS_START             (0x40000000)
#define EXAMPLE_PERIPH_SIZE                      LL_MPU_REGION_SIZE_512MB
#define EXAMPLE_FLASH_ADDRESS_START              (0x08000000)
#define EXAMPLE_FLASH_SIZE                       LL_MPU_REGION_SIZE_1MB
#define EXAMPLE_RAM_REGION_NUMBER                LL_MPU_REGION_NUMBER0
#define EXAMPLE_FLASH_REGION_NUMBER              LL_MPU_REGION_NUMBER1
#define EXAMPLE_PERIPH_REGION_NUMBER             LL_MPU_REGION_NUMBER2
#define MPU_REGION_READ_WRITE                    LL_MPU_REGION_FULL_ACCESS
#define MPU_REGION_PRIVILEGED_READ_ONLY          LL_MPU_REGION_PRIV_RO
#define MPU_REGION_READ_ONLY                     LL_MPU_REGION_PRIV_RO_URO
#define MPU_REGION_PRIVILEGED_READ_WRITE         LL_MPU_REGION_PRIV_RW


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint32_t accessPermitted = 0;
__IO uint32_t memManageCallback = 0;
__IO uint32_t callback = 0;
__IO uint8_t  ubButtonPress = 0;


/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);
static void MX_GPIO_Init(void);
void Configure_MPU(void);
void MPU_AccessPermConfig(void);
void LED_Blinking(uint32_t Period);

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
  
  /* Set MPU regions */
  Configure_MPU();
  MPU_AccessPermConfig();
  
  printf("To generate an MPU memory fault exception due to an access right error, press PUSH1.\n\r");
  printf("If access is not permitted, a Hard fault is generated and LED2 is ON.\n\r");
  printf("Wait for User push-button (PUSH1) press or enter 'c'/'C' character.\n\r");
  
  while (ubButtonPress == 0)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(LED_BLINK_FAST);
  }
  
  /* This will generate error */
  int * ptr = (int*) (PRIVILEGED_READ_ONLY_ADDRESS_START + (PRIVILEGED_READ_ONLY_SIZE - 1));
  *ptr = 1;
  accessPermitted = 1;

  while(1);
}

static void LL_Init(void)
{
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
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
* @brief  Configures the main MPU regions.
* @param  None
* @retval None
*/
void Configure_MPU(void)
{
  /* Disable MPU */
  LL_MPU_Disable();
  
  /* Configure RAM region as Region N°0, 256KB of size and R/W region */
  LL_MPU_ConfigRegion(EXAMPLE_RAM_REGION_NUMBER, 0x00, EXAMPLE_RAM_ADDRESS_START, 
                      EXAMPLE_RAM_SIZE | MPU_REGION_READ_WRITE | LL_MPU_ACCESS_NOT_BUFFERABLE |
                        LL_MPU_ACCESS_NOT_CACHEABLE | LL_MPU_ACCESS_NOT_SHAREABLE | LL_MPU_TEX_LEVEL0 |
                          LL_MPU_INSTRUCTION_ACCESS_ENABLE);
  
  /* Configure FLASH region as REGION N°1, 1MB of size and R/W region */
  LL_MPU_ConfigRegion(EXAMPLE_FLASH_REGION_NUMBER, 0x00, EXAMPLE_FLASH_ADDRESS_START, 
                      EXAMPLE_FLASH_SIZE | MPU_REGION_READ_WRITE | LL_MPU_ACCESS_NOT_BUFFERABLE |
                        LL_MPU_ACCESS_NOT_CACHEABLE | LL_MPU_ACCESS_NOT_SHAREABLE | LL_MPU_TEX_LEVEL0 |
                          LL_MPU_INSTRUCTION_ACCESS_ENABLE);
  
  /* Configure Peripheral region as REGION N°2, 512MB of size, R/W and Execute
  Never region */
  LL_MPU_ConfigRegion(EXAMPLE_PERIPH_REGION_NUMBER, 0x00, EXAMPLE_PERIPH_ADDRESS_START, 
                      EXAMPLE_PERIPH_SIZE | MPU_REGION_READ_WRITE | LL_MPU_ACCESS_NOT_BUFFERABLE |
                        LL_MPU_ACCESS_NOT_CACHEABLE | LL_MPU_ACCESS_NOT_SHAREABLE | LL_MPU_TEX_LEVEL0 |
                          LL_MPU_INSTRUCTION_ACCESS_DISABLE);
  
  /* Enable MPU (any access not covered by any enabled region will cause a fault) */
  LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);
}

/**
* @brief  This function configures the access right using Cortex-M0+ MPU regions.
* @param  None
* @retval None
*/
void MPU_AccessPermConfig(void)
{
  /* Configure region for PrivilegedReadOnlyArray as REGION N°3, 32byte and R
  only in privileged mode */
  /* Disable MPU */
  LL_MPU_Disable();
  
  LL_MPU_ConfigRegion(REGION_NUMBER, 0x00, PRIVILEGED_READ_ONLY_ADDRESS_START, 
                      REGION_SIZE | MPU_REGION_PRIVILEGED_READ_ONLY | LL_MPU_ACCESS_NOT_BUFFERABLE |
                        LL_MPU_ACCESS_NOT_CACHEABLE | LL_MPU_ACCESS_NOT_SHAREABLE | LL_MPU_TEX_LEVEL0 |
                          LL_MPU_INSTRUCTION_ACCESS_ENABLE);
  
  /* Enable MPU (any access not covered by any enabled region will cause a fault) */
  LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);
  
  
  /* Read from 0x200020FF. This will not generate error */
  int * ptr = (int*) PRIVILEGED_READ_ONLY_ADDRESS_START + (PRIVILEGED_READ_ONLY_SIZE - 1);
  (void)*ptr;
}


/**
* @brief  Set LED to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
* @param  Period : Period of time (in ms) between each toggling of LED
*   This parameter can be user defined values. Pre-defined values used in that example are :
*     @arg LED_BLINK_FAST : Fast Blinking
*     @arg LED_BLINK_SLOW : Slow Blinking
*     @arg LED_BLINK_ERROR : Error specific Blinking
* @retval None
*/
void LED_Blinking(uint32_t Period)
{
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
* @brief  Function to manage memory exception
* @param  None
* @retval None
*/
void MemManage_Callback(void) 
{
  /* Turn on LED */
  LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
  
  printf("** Test successfully. ** \n\r\n\r");
  
  memManageCallback = 1;
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
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
  LED_Blinking(LED_BLINK_SLOW);
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



