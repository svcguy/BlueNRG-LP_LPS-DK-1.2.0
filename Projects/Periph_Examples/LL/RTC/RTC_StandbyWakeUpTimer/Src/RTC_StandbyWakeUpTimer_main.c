
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RTC_StandbyWakeUpTimer_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the RTC functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RTC_StandbyWakeUpTimer/RTC_StandbyWakeUpTimer_main.c
 * @brief Configuration of the RTC to wake up from Standby mode using the RTC Wakeup timer.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\RTC\\RTC_StandbyWakeUpTimer\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RTC_StandbyWakeUpTimer.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\RTC\\RTC_StandbyWakeUpTimer\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RTC_StandbyWakeUpTimer.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\RTC\\RTC_StandbyWakeUpTimer\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  LED name  |                                               STEVAL-IDB011V1                                              |                                               STEVAL-IDB011V2                                              |                                               STEVAL-IDB012V1                                              |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                                                  Not Used                                                  |                                                  Not Used                                                  |                                                  Not Used                                                  |
|     DL2    |   On: system is resumed from standby mode;  fast blinking: wait fpr User push-buton; slow blinking: error  |   On: system is resumed from standby mode;  fast blinking: wait fpr User push-buton; slow blinking: error  |   On: system is resumed from standby mode;  fast blinking: wait fpr User push-buton; slow blinking: error  |
|     DL3    |                                                  Not Used                                                  |                                                  Not Used                                                  |                                                  Not Used                                                  |
|     DL4    |                                                  Not Used                                                  |                                                  Not Used                                                  |                                                  Not Used                                                  |
|     U5     |                                                  Not Used                                                  |                                                  Not Used                                                  |                                                  Not Used                                                  |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |       STEVAL-IDB011V1      |       STEVAL-IDB011V2      |       STEVAL-IDB012V1      |
------------------------------------------------------------------------------------------------------------
|      PUSH1     |   Enter in low power mode  |   Enter in low power mode  |   Enter in low power mode  |
|      PUSH2     |          Not Used          |          Not Used          |          Not Used          |
|      RESET     |      Reset BlueNRG-LP      |      Reset BlueNRG-LP      |      Reset BlueNRG-LP      |

@endtable

* \section Usage Usage

Configuration of the RTC to wake up from Standby mode using the RTC Wakeup timer. The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size).
  
Example execution:
- 1st execution of the system, LED2 is quickly blinking.
- RTC wakup timer is configured to 5 seconds
- Press the User push-button (PUSH1): System enters in standby mode (LED2 is switched off)
- After 5 seconds, system resumes from standby mode, then LED2 is turned on.
- LED2 is toggling every 1 second: This indicates that the system generates an error.

This example is applicable only for BlueNRG_LP cut 2.0 or above.

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
#include "RTC_StandbyWakeUpTimer_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Value defined for WUT */
#define RTC_WUT_TIME               ((uint32_t)4)     /* 5 s */
#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7F)
#define RTC_SYNCH_PREDIV           ((uint32_t)0x00FF)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint8_t ubButtonPress = 0;

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
static void MX_GPIO_Init(void);
void Configure_RTC(void);
void EnableStandbyMode(void);
void LED_On(void);
void LED_Off(void);
void LED_Blinking(uint32_t Period);
void LED_Toggle(void);
void WaitForUserButtonPress(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);

/* Private user code ---------------------------------------------------------*/

/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
  uint8_t static volatile ret_val = 0;
  uint32_t static volatile wakeupSources = 0;
  PowerSaveLevels static stopLevel;
  WakeupSourceConfig_TypeDef wakeupIO = {0,0,0,0};
  wakeupIO.RTC_enable = 1;
  
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
  LED_Off();
  
  /* Run after normal reset */
  /* Fast Toggle LED in waiting for user-button press */
  printf("Wait for User push-button (PUSH1) press or enter 'c'/'C' character.\n\r");
  WaitForUserButtonPress();
  
  /* Configure RTC to use WUT */
  Configure_RTC();
  
  /* Enable wake-up timer and enter in standby mode */
  printf("Enable wake-up timer and enter in standby mode.\n\r");
  LL_mDelay(500);
  
  EnableStandbyMode();
  
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  ret_val = HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_WITH_TIMER, wakeupIO, &stopLevel);
  if (ret_val != SUCCESS){
    /* Error during clock config */
    Error_Handler();
  }
  if (stopLevel >= POWER_SAVE_LEVEL_STOP_WITH_TIMER) {
    if( HAL_PWR_MNGR_WakeupSource() != WAKEUP_RTC)
    {       
      /* WAKEUP_RTC Wakeup on Internal event (RTC) */
      Error_Handler();
    }
  }
#endif
  
  /* Clear RTC Wake Up timer Flag */
  LL_RTC_ClearFlag_WUT(RTC);
  
  LED_On();
  
  printf("System resumes from standby mode, then LED2 is turned on\n\r");
  printf("** Test successfully. ** \n\r\n\r");
  
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
* Brief   This function configures RTC.
* Param   None
* Retval  None
*/
void Configure_RTC(void)
{
  /* Enable Peripheral Clock */
  LL_APB0_EnableClock(LL_APB0_PERIPH_RTC);
  
  /* RTC peripheral reset */
  LL_RCC_ClearFlag_RTCRSTREL();
  LL_APB0_ForceReset(LL_APB0_PERIPH_RTC);
  LL_APB0_ReleaseReset(LL_APB0_PERIPH_RTC);
  while(LL_RCC_IsActiveFlag_RTCRSTREL() == 0);
  LL_RCC_ClearFlag_RTCRSTREL();
  
  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection(RTC);
  
  /* Set Initialization mode */
  LL_RTC_EnableInitMode(RTC);
  /* Check if the Initialization mode is set */
  while (LL_RTC_IsActiveFlag_INIT(RTC) != 1)
  {
  }
  
  /* Set prescaler according to source clock */
  LL_RTC_SetAsynchPrescaler(RTC, RTC_ASYNCH_PREDIV);
  LL_RTC_SetSynchPrescaler(RTC, RTC_SYNCH_PREDIV);
  
  /* Disable wake up timer to modify it */
  LL_RTC_WAKEUP_Disable(RTC);
  
  /* Wait until it is allow to modify wake up reload value */
  while (LL_RTC_IsActiveFlag_WUTW(RTC) != 1)
  {
  }
  
  /* Exit Initialization mode */
  LL_RTC_DisableInitMode(RTC);
  /* Clear RSF flag */
  LL_RTC_ClearFlag_RS(RTC);
  /* Wait the registers to be synchronised */
  while (LL_RTC_IsActiveFlag_RS(RTC) != 1)
  {
  }
  
  /* Enable RTC registers write protection */
  LL_RTC_EnableWriteProtection(RTC);
}

/**
* @brief  Function to configure and enable in STANDBY Mode.
* @param  None
* @retval None
*/
void EnableStandbyMode(void)
{
  /* Disable write protection */
  LL_RTC_DisableWriteProtection(RTC);
  
  /* Disable Wake-up Timer */
  LL_RTC_WAKEUP_Disable(RTC);
  
  /* Wait till RTC WUTWF flag is set  */
  while(LL_RTC_IsActiveFlag_WUTW(RTC) == 0);
  
  /* Clear RTC Wake Up timer Flag */
  LL_RTC_ClearFlag_WUT(RTC);
  
  /* Configure the Wake-up Timer counter */
  LL_RTC_WAKEUP_SetAutoReload(RTC,  RTC_WUT_TIME);
  
  /* Configure the clock source */
  LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
  
  /* Enable the Wake-up Timer */
  LL_RTC_WAKEUP_Enable(RTC);
  
  /* Enable write protection */
  LL_RTC_EnableWriteProtection(RTC);
  
  /* Clear RTC Wake Up timer Flag */
  LL_RTC_ClearFlag_WUT(RTC);
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
  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(Period);
  }
}

/**
* @brief  Set LED2 to toggle once time
* @retval None
*/
void LED_Toggle(void)
{
  /* Toggle IO */
  LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
}

/**
* @brief  Wait for User push-button
* @param  None
* @retval None
*/
void WaitForUserButtonPress(void)
{
  while (ubButtonPress == 0)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(LED_BLINK_FAST);
  }
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
  ubButtonPress = 0;
}

/******************************************************************************/
/*  USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
* @brief  Function to manage User button
* @param  None
* @retval None
*/
void UserButton_Callback(void)
{
  /* Update User push-button (PUSH1) variable : to be checked in waiting loop in main function */
  ubButtonPress = 1;
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
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



