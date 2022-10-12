
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : PWR_PVD_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the PWR functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  PWR_PVD/PWR_PVD_main.c
 * @brief How to configure and use the PWR to generata a PDV interrupt. 
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\PWR\\PWR_PVD\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\PWR_PVD.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\PWR\\PWR_PVD\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\PWR_PVD.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\PWR\\PWR_PVD\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
{SERIAL_IO_TABLE}
@endtable

* \section LEDs_description LEDs description
@table
|  LED name  |                              STEVAL-IDB011V1                             |                              STEVAL-IDB011V2                             |                              STEVAL-IDB012V1                             |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                                 Not Used                                 |                                 Not Used                                 |                                 Not Used                                 |
|     DL2    |        Off for 2sec indicate that a PVD interrupt was been rised.        |        Off for 2sec indicate that a PVD interrupt was been rised.        |        Off for 2sec indicate that a PVD interrupt was been rised.        |
|     DL3    |  On: the threshold is set to 2.05V - Off: the threshold is set to 2.91V  |  On: the threshold is set to 2.05V - Off: the threshold is set to 2.91V  |  On: the threshold is set to 2.05V - Off: the threshold is set to 2.91V  |
|     DL4    |                                 Not Used                                 |                                 Not Used                                 |                                 Not Used                                 |
|     U5     |                                 Not Used                                 |                                 Not Used                                 |                                 Not Used                                 |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |        STEVAL-IDB011V1       |        STEVAL-IDB011V2       |        STEVAL-IDB012V1       |
------------------------------------------------------------------------------------------------------------------
|      PUSH1     |   Change the PVD threshold.  |   Change the PVD threshold.  |   Change the PVD threshold.  |
|      PUSH2     |           Not Used           |           Not Used           |           Not Used           |
|      RESET     |       Reset BlueNRG-LP       |       Reset BlueNRG-LP       |       Reset BlueNRG-LP       |

@endtable

* \section Usage Usage

This example shows how the PVD measure a voltage below the comparator and how is raised an interrupt to the CPU in the SYSCFG block. 
This example is based on the BLUENRG_LP LL API. 

When user pushes the User push-button (PUSH1), the PVD threshold is modified, in an infinite loop. LED3 On indicates that the threshold is set to 2.05V, LED3 Off indicates the threshold is set to 2.91V.

As long as the voltage is above the target threshold, LED2 is On (or appears to be turned off if the voltage is getting really low); when the voltage drops below the threshold, LED2 remains off for 2 seconds.

To test the example, connect the board with an external voltage generator and decrease the voltage under the threshold. The interrupt will be raised when the VDD will be under the threshold.

  
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
#include "PWR_PVD_main.h"

/** @addtogroup Peripherals_LL_Examples
  * @{
  */

/** @addtogroup PWR_PVD
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static __IO uint8_t ubKeyPressed = 0;
static __IO uint8_t switchThreshold = 0;
static __IO uint8_t ubPVD = 0;

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
static void MX_GPIO_Init(void);

/* Private functions ---------------------------------------------------------*/
/*

Power Voltage Detection (PVD)
The PVD can be used to monitor:
� the VDDIO is compared to a programmed threshold (between 2.05V and 2.91V),
� the threshold programming is done through PWRC_CR2.PVDLS[2:0] bit field, an external analog input signal:
� an external analog signal is compared to an internal VBGP (at 1.0V) voltage.
� the feature is selected through PWRC_CR2.PVDLS[2:0] bit field,
The PVD can be enabled or disabled through PWRC_CR2.PVDE bit.
When the feature is enabled and the PVD measure a voltage below the comparator, a status
flag is raised in the SYSCFG block that can generate an interrupt to the


*/
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
  
  /* Initialization of LEDs */
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  
  printf("Power Voltage Detection.\n\r"); 
  
  /* programmed threshold (between 2.05V) */
  LL_PWR_SetPVDLevel(LL_PWR_PVDLEVEL_0);
  BSP_LED_On(BSP_LED3);
      
  /* Enable PWR PVD Interrupt */
  LL_SYSCFG_PWRC_EnableIT(LL_SYSCFG_PWRC_PVD);
  
  /* Configure NVIC for PVD_IRQn */
  NVIC_SetPriority(PVD_IRQn, IRQ_MED_PRIORITY);
  NVIC_EnableIRQ(PVD_IRQn);

  /* Enable the PVD */
  LL_PWR_EnablePVD();
  
  /* Configure NVIC for SysTick_IRQn */
  NVIC_SetPriority(SysTick_IRQn, IRQ_LOW_PRIORITY);
  
  BSP_LED_On(BSP_LED2);
  printf("Wait for User push-button (PUSH1): ");
  
  /* Infinite loop */
  while (1)
  {
    if (1 == ubKeyPressed)
    {
      if(switchThreshold==1)
      {
        printf("Set the programmed threshold to 2.05V \n\r"); 
        /* programmed threshold (between 2.05V) */
        LL_PWR_SetPVDLevel(LL_PWR_PVDLEVEL_0);
        BSP_LED_On(BSP_LED3);
        switchThreshold = 0;
      }
      else
      {
        printf("Set the programmed threshold to 2.91V \n\r"); 
        /* programmed threshold (between 2.91V) */
        LL_PWR_SetPVDLevel(LL_PWR_PVDLEVEL_6);
        BSP_LED_Off(BSP_LED3);
        switchThreshold = 1;
      }
      ubKeyPressed = 0;
      printf("Wait for User push-button (PUSH1): ");
    }
    
    if(ubPVD == 1)
    {
      printf("PVD Interrupt was been generated. \n\r");
      ubPVD = 0;
      BSP_LED_Off(BSP_LED2);
      /* Add 1s of delay between each interrupt rised. */
      LL_mDelay(2000);
      BSP_LED_On(BSP_LED2);
    }
  }
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
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
  LL_APB0_EnableClock(LL_APB0_PERIPH_SYSCFG);
   
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

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
* @brief  Function to manage IRQ Handler
* @param  None
* @retval None
*/
void UserButton_Callback(void)
{
  ubKeyPressed = 1;
}

/**
* @brief  Function to manage IRQ Handler
* @param  None
* @retval None
*/
void UserPVD_Callback(void)
{
  ubPVD = 1;
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
