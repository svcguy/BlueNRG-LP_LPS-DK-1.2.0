
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RCC_OutputMCO_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the RCC functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RCC_OutputMCO/RCC_OutputMCO_main.c
 * @brief How to configure of MCO pin  to output the system clock.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\RCC\\RCC_OutputMCO\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RCC_OutputMCO.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\RCC\\RCC_OutputMCO\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RCC_OutputMCO.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\RCC\\RCC_OutputMCO\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A1     |       Not Used      |      Not Used      |
|     A11    |         MCO         |        MCO         |
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
|     B0     |       Not Used      |      Not Used      |
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
|  LED name  |                            STEVAL-IDB011V1                           |                            STEVAL-IDB011V2                           |                            STEVAL-IDB012V1                           |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                               Not Used                               |                               Not Used                               |                               Not Used                               |
|     DL2    |  Toggle: it indicates a change in MCO config - Slow blinking: error  |  Toggle: it indicates a change in MCO config - Slow blinking: error  |  Toggle: it indicates a change in MCO config - Slow blinking: error  |
|     DL3    |                               Not Used                               |                               Not Used                               |                               Not Used                               |
|     DL4    |                               Not Used                               |                               Not Used                               |                               Not Used                               |
|     U5     |                               Not Used                               |                               Not Used                               |                               Not Used                               |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |      STEVAL-IDB011V1     |      STEVAL-IDB011V2     |      STEVAL-IDB012V1     |
------------------------------------------------------------------------------------------------------
|      PUSH1     |   Set MCO configuration  |   Set MCO configuration  |   Set MCO configuration  |
|      PUSH2     |         Not Used         |         Not Used         |         Not Used         |
|      RESET     |     Reset BlueNRG-LP     |     Reset BlueNRG-LP     |     Reset BlueNRG-LP     |

@endtable

* \section Usage Usage

Configuration of MCO pin to output the system clock.

At start-up, User push-button (PUSH1) and MCO pin are configured. 

The signal on PA11 can be monitored with an oscilloscope to check the different MCO configuration set at each User push-button (PUSH1) press.
Different configuration will be observed
 - SYSCLK frequency with frequency value divided by 1.
 - SYSCLK frequency with frequency value divided by 2.
 - MCO output disabled. No clock on MCO pad.

When user press User push-button, a LED2 toggle is done to indicate a change in MCO config.

In case of errors, LED2 is blinking Slowly (1s).

BlueNRG_LP-EVB Set-up
- Connect the MCO pin PA.11 to an oscilloscope to monitor the different waveforms
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
#include "RCC_OutputMCO_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Structure based on parameters used for MCO config */
typedef struct
{
  uint32_t Source;          /*!< Source of the MCO output */
  uint32_t Prescaler;       /*!< Prescaler applied to MCO output */
} RCC_MCO_ConfigTypeDef;

__IO uint8_t ubButtonPress = 0;

/* Private define ------------------------------------------------------------*/

/* Number of MCO Config */
#define RCC_MCO_CONFIG_NB   3

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Variable to set different MCO config */
static RCC_MCO_ConfigTypeDef aMCO_Config[RCC_MCO_CONFIG_NB] = {
  {LL_RCC_MCOSOURCE_SYSCLK, LL_RCC_MCO_DIV_1},
  {LL_RCC_MCOSOURCE_SYSCLK, LL_RCC_MCO_DIV_2},
  {LL_RCC_MCOSOURCE_NOCLOCK, LL_RCC_MCO_DIV_1}
  };

/* MCO Config index */
__IO uint8_t bMCOIndex = 0;

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
static void MX_GPIO_Init(void);
static void MX_MCO_Init(void);

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
  BSP_COM_Init(NULL);
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_MCO_Init();

  printf("Each time, user press user-button, a new MCO config is set\n\r");

  /* Infinite loop */
  while (1)
  {
    /* All the process are managed in UserButton_Callback function */
    /* Each time, user press user-button, a new MCO config is set */
    if(ubButtonPress==1)
    {
      ubButtonPress = 0;
      printf("new MCO config is set\n\r");
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
static void MX_MCO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  MCO_GPIO_CLK_ENABLE();
 
  /* MCO */
  GPIO_InitStruct.Pin = MCO_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = MCO_GPIO_AF;
  LL_GPIO_Init(MCO_GPIO_PORT, &GPIO_InitStruct);
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

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
  * @brief  Function to manage User button press
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  register uint32_t source = 0, prescaler = 0;

  /* Get the MCO config to apply */
  source = aMCO_Config[bMCOIndex].Source;
  prescaler = aMCO_Config[bMCOIndex].Prescaler;

  /* Toggle LED2 to indicate a button press*/
  LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
  
  /* Update User push-button (PUSH1) variable : to be checked in waiting loop in main program */
  ubButtonPress = 1;
  
  /* Select MCO clock source and prescaler */
  LL_RCC_ConfigMCO(source, prescaler);

  /* Set new MCO config Index */
  bMCOIndex = (bMCOIndex + 1) % RCC_MCO_CONFIG_NB;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(1000);
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
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



