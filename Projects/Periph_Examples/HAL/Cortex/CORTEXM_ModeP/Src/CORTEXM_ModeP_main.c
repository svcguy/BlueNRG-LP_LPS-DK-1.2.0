
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : CORTEXM_ModeP_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the CORTEX Mode Privilege functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  CORTEXM_ModeP/CORTEXM_ModeP_main.c
 * @brief  This example shows how to modify the Thread mode privilege access and stack. 
 * Thread mode is entered on reset or when returning from an exception.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\CORTEX\\CORTEXM_ModeP\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\CORTEXM_ModeP.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\CORTEX\\CORTEXM_ModeP\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\CORTEXM_ModeP.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\CORTEX\\CORTEXM_ModeP\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  LED name  |        STEVAL-IDB011V1       |        STEVAL-IDB011V2       |        STEVAL-IDB012V1       |
--------------------------------------------------------------------------------------------------------------
|     DL1    |           Not Used           |           Not Used           |           Not Used           |
|     DL2    |   ON: test end with success  |   ON: test end with success  |   ON: test end with success  |
|     DL3    |           Not Used           |           Not Used           |           Not Used           |
|     DL4    |           Not Used           |           Not Used           |           Not Used           |
|     U5     |           Not Used           |           Not Used           |           Not Used           |

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

How to modify the Thread mode privilege access and stack. Thread mode is entered on reset or when returning from an exception.

The associated program is used to:

1. Switch the Thread mode stack from Main stack to Process stack

2. Switch the Thread mode from Privileged to Unprivileged

3. Switch the Thread mode from Unprivileged back to Privileged

To monitor the stack used and the privileged or unprivileged access level of code in Thread mode, a set of variables is available within the program. 
It is also possible to use the 'Cortex register' window of the debugger.

LED2 Turns ON when the test is finished successfully.


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
#include "CORTEXM_ModeP_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SP_PROCESS_SIZE             0x200  /* Process stack size */

/* Private function prototypes -----------------------------------------------*/
static __INLINE void __SVC(void);

/* Private macro -------------------------------------------------------------*/
#if defined ( __CC_ARM   )
__ASM void __SVC(void)
{
  SVC 0x01
    BX R14
}
#elif defined ( __ICCARM__ )
static __INLINE  void __SVC()
{
  __ASM("svc 0x01");
}
#elif defined   (  __GNUC__  )
static __INLINE void __SVC()
{
  __ASM volatile("svc 0x01");
}
#endif

/* Private variables ---------------------------------------------------------*/
__IO uint8_t PSPMemAlloc[SP_PROCESS_SIZE];
__IO uint32_t Index = 0, PSPValue = 0, CurrentStack = 0, ThreadMode = 0, OperationComplete = 0;


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
  BSP_LED_Init(BSP_LED2);
  
  /* Switch Thread mode Stack from Main to Process -----------------------------*/
  printf("Switch Thread mode Stack from Main to Process.\n\r");
  
  /* Initialize memory reserved for Process Stack */
  for(Index = 0; Index < SP_PROCESS_SIZE; Index++)
  {
    PSPMemAlloc[Index] = 0x00;
  }
  
  /* Set Process stack value */
  __set_PSP((uint32_t)PSPMemAlloc + SP_PROCESS_SIZE);
  
  /* Select Process Stack as Thread mode Stack */
  __set_CONTROL(SP_PROCESS);
  
  /* Execute ISB instruction to flush pipeline as recommended by Arm */
  __ISB(); 
  
  /* Get the Thread mode stack used */
  if((__get_CONTROL() & 0x02) == SP_MAIN)
  {
    /* Main stack is used as the current stack */
    printf("Main stack is used as the current stack.\n\r");
    CurrentStack = SP_MAIN;
  }
  else
  {
    /* Process stack is used as the current stack */
    printf("Process stack is used as the current stack.\n\r");
    CurrentStack = SP_PROCESS;
    
    /* Get process stack pointer value */
    PSPValue = __get_PSP();
  }
  
  /* Switch Thread mode from privileged to unprivileged ------------------------*/
  /* Thread mode has unprivileged access */
  __set_CONTROL(THREAD_MODE_UNPRIVILEGED | SP_PROCESS);
  
  /* Execute ISB instruction to flush pipeline as recommended by Arm */
  /* This ensures that instructions after the ISB execute using the new stack pointer */
  __ISB(); 
  
  /* Unprivileged access mainly affect ability to:
  - Use or not use certain instructions such as MSR fields
  - Access System Control Space (SCS) registers such as NVIC and SysTick */
  
  /* Check Thread mode privilege status */
  if((__get_CONTROL() & 0x01) == THREAD_MODE_PRIVILEGED)
  {
    /* Thread mode has privileged access  */
    printf("Thread mode has privileged access.\n\r");
    ThreadMode = THREAD_MODE_PRIVILEGED;
  }
  else
  {
    /* Thread mode has unprivileged access*/
    printf("Thread mode has unprivileged access.\n\r");
    ThreadMode = THREAD_MODE_UNPRIVILEGED;
  }
  
  /* Switch back Thread mode from unprivileged to privileged -------------------*/
  /* Try to switch back Thread mode to privileged (Not possible, this can be
  done only in Handler mode) */
  /* Only privileged software can write to the CONTROL register to change the privilege level for
  software execution in Thread mode. */
  __set_CONTROL(THREAD_MODE_PRIVILEGED | SP_PROCESS);
  
  /* Execute ISB instruction to flush pipeline as recommended by Arm */
  __ISB(); 
  
  /* Generate a system call exception, and in the ISR switch back Thread mode
  to privileged */
  __SVC();
  
  /* Check Thread mode privilege status */
  if((__get_CONTROL() & 0x01) == THREAD_MODE_PRIVILEGED)
  {
    /* Thread mode has privileged access  */
    printf("Thread mode has privileged access.\n\r");
    ThreadMode = THREAD_MODE_PRIVILEGED;
    
    /* Turn ON LED once test finished */
    BSP_LED_On(BSP_LED2);
    printf("test finished.\n\r");
  }
  else
  {
    /* Thread mode has unprivileged access*/
    printf("Thread mode has unprivileged access.\n\r");
    ThreadMode = THREAD_MODE_UNPRIVILEGED;
  }
  OperationComplete = 1;
  /* Infinite loop */
  while (1)
  {
  }
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  OperationComplete = 2;
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



