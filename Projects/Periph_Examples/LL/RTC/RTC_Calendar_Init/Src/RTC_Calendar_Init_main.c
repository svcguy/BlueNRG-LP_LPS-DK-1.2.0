
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RTC_Calendar_Init_main.c
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
 * @file  RTC_Calendar_Init/RTC_Calendar_Init_main.c
 * @brief Configuration of the LL API to set the RTC calendar.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\RTC\\RTC_Calendar_Init\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RTC_Calendar_Init.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\RTC\\RTC_Calendar_Init\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RTC_Calendar_Init.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\RTC\\RTC_Calendar_Init\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  LED name  |         STEVAL-IDB011V1        |         STEVAL-IDB011V2        |         STEVAL-IDB012V1        |
--------------------------------------------------------------------------------------------------------------------
|     DL1    |            Not Used            |            Not Used            |            Not Used            |
|     DL2    |   on: RTC ok; blinking: error  |   on: RTC ok; blinking: error  |   on: RTC ok; blinking: error  |
|     DL3    |            Not Used            |            Not Used            |            Not Used            |
|     DL4    |            Not Used            |            Not Used            |            Not Used            |
|     U5     |            Not Used            |            Not Used            |            Not Used            |

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

Configuration of the LL API to set the RTC calendar. The peripheral initialization uses LL unitary service 
functions for optimization purposes (performance and size).

The RTC peripheral configuration is ensured by the Configure_RTC() function (configure of the needed RTC resources according to the used hardware CLOCK, PWR, RTC clock source and BackUp). You may update this function to change RTC configuration.

Configure_RTC_Calendar function is then called to initialize the time and the date.

A key value RTC_BKP_DATE_TIME_UPDTATED is written in backup data register 1 to indicate 
if the RTC is already configured.
To debug the intialize procedure when the backup data register 1 is been already written, is mandatory change the value of the #define RTC_BKP_DATE_TIME_UPDTATED.
 
When a reset (except power on reset) occurs the BKP domain is not reset and the RTC configuration is not lost.

When power on reset occurs and there are no battery connected to the VBAT pin: the BKP domain is reset and the RTC configuration is lost.

The current time and date are updated and displayed on the USART.

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
#include "RTC_Calendar_Init_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* ck_apre = Freq/(ASYNC prediv + 1) with 32 kHz  */
#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7F)
/* ck_spre = ck_apre/(SYNC prediv + 1) = 1 Hz */
#define RTC_SYNCH_PREDIV           ((uint32_t)0x00FF)

/* Define used to indicate date/time updated */
#define RTC_BKP_DATE_TIME_UPDTATED ((uint32_t)0x32F1)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Buffers used for displaying Time and Date */
uint8_t aShowTime[] = "hh:ms:ss";
uint8_t aShowDate[] = "mm:dd:aaaa";
uint8_t aShowTimeCheckSuccess[] = "00:00:00";

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
uint32_t Enter_RTC_InitMode(void);
uint32_t Exit_RTC_InitMode(void);
uint32_t WaitForSynchro_RTC(void);
void Show_RTC_Calendar(void);
void LED_On(void);
void LED_Blinking(uint32_t Period);
uint8_t Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength);

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
  MX_RTC_Init();
  
  /* Turn-on LED2 to indicate that calendar has been well configured */
  LED_On();
  
  /* Infinite loop */
  while (1)
  {
    /* Display the updated Time and Date */
    Show_RTC_Calendar();
    
    printf("%s\n\r", aShowTime);
    printf("%s\n\r", aShowDate);
    printf("\n\r");
    LL_mDelay(1000);
    
    if(Buffercmp8((uint8_t*)aShowTime, (uint8_t*)aShowTimeCheckSuccess, sizeof(aShowTime)))
    {
      printf("Wait for User reset-button (RST) press.\n\r");
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
* @brief RTC Initialization Function
* @param None
* @retval None
*/
static void MX_RTC_Init(void)
{
  LL_RTC_InitTypeDef RTC_InitStruct = {0};
  LL_RTC_TimeTypeDef RTC_TimeStruct = {0};
  LL_RTC_DateTypeDef RTC_DateStruct = {0};
  
  /* Check if Data stored in BackUp register1: No Need to reconfigure RTC */
  /* Read the Back Up Register 1 Data */
  if (LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR1) != RTC_BKP_DATE_TIME_UPDTATED)
  {
    printf("Check Data stored in BackUp register1: configuration RTC started\n\r");
    /* Force RTC peripheral reset */
    /* This action delete the calendar configuration in case of hardware reset */
    
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
    
    /* Init mode setup */
    LL_RTC_EnableInitMode(RTC);
    
    /* Wait till the Init mode is active */
    while(LL_RTC_IsActiveFlag_INIT(RTC) == RESET);
    
    /* Enter in initialization mode */
    if (Enter_RTC_InitMode() != RTC_ERROR_NONE)
    {
      /* Initialization Error */
      LED_Blinking(LED_BLINK_ERROR);
    }
    
    /* Configure RTC */
    RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
    RTC_InitStruct.AsynchPrescaler = RTC_ASYNCH_PREDIV;
    RTC_InitStruct.SynchPrescaler = RTC_SYNCH_PREDIV;
    LL_RTC_Init(RTC, &RTC_InitStruct);
    
    /* Configure the Time */
    RTC_TimeStruct.TimeFormat = LL_RTC_TIME_FORMAT_AM_OR_24;
    RTC_TimeStruct.Hours = 0x23;
    RTC_TimeStruct.Minutes = 0x59;
    RTC_TimeStruct.Seconds = 0x50;
    LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_TimeStruct);
    /* Configure the Date */
    RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_FRIDAY;
    RTC_DateStruct.Month = LL_RTC_MONTH_DECEMBER;
    RTC_DateStruct.Day = 0x31;
    RTC_DateStruct.Year = 0x19;
    LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_DateStruct);
    
    /* Exit of initialization mode */
    if (Exit_RTC_InitMode() != RTC_ERROR_NONE)
    {
      /* Initialization Error */
      LED_Blinking(LED_BLINK_ERROR);
    }
    
    /* Enable RTC registers write protection */
    LL_RTC_EnableWriteProtection(RTC);
    
    /* Writes a data in a RTC Backup data Register1 */
    LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR1, RTC_BKP_DATE_TIME_UPDTATED);
  }
  else
  {
    printf("Check Data stored in BackUp register1: No Need to reconfigure RTC\n\r");
    printf("** Test successfully. ** \n\r\n\r");
  }
}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  LED2_GPIO_CLK_ENABLE();
  
  /* Configure GPIO for LED */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
  GPIO_InitStruct.Pin = LED2_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
}

/**
* @brief  Enter in initialization mode
* @note In this mode, the calendar counter is stopped and its value can be updated
* @param  None
* @retval RTC_ERROR_NONE if no error
*/
uint32_t Enter_RTC_InitMode(void)
{
  /* Set Initialization mode */
  LL_RTC_EnableInitMode(RTC);
  
  /* Check if the Initialization mode is set */
  while (LL_RTC_IsActiveFlag_INIT(RTC) != 1)
  {
  }
  
  return RTC_ERROR_NONE;
}

/**
* @brief  Exit Initialization mode
* @param  None
* @retval RTC_ERROR_NONE if no error
*/
uint32_t Exit_RTC_InitMode(void)
{
  LL_RTC_DisableInitMode(RTC);
  
  /* Wait for synchro */
  /* Note: Needed only if Shadow registers is enabled           */
  /*       LL_RTC_IsShadowRegBypassEnabled function can be used */
  return (WaitForSynchro_RTC());
}

/**
* @brief  Wait until the RTC Time and Date registers (RTC_TR and RTC_DR) are
*         synchronized with RTC APB clock.
* @param  None
* @retval RTC_ERROR_NONE if no error
*/
uint32_t WaitForSynchro_RTC(void)
{
  /* Clear RSF flag */
  LL_RTC_ClearFlag_RS(RTC);
  
  /* Wait the registers to be synchronised */
  while (LL_RTC_IsActiveFlag_RS(RTC) != 1)
  {
  }
  return RTC_ERROR_NONE;
}

/**
* @brief  Display the current time and date.
* @param  None
* @retval None
*/
void Show_RTC_Calendar(void)
{
  /* Note: need to convert in decimal value in using __LL_RTC_CONVERT_BCD2BIN helper macro */
  /* Display time Format : hh:mm:ss */
  sprintf((char *)aShowTime, "%.2d:%.2d:%.2d", __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC)),
          __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC)),
          __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC)));
  /* Display date Format : mm-dd-yy */
  sprintf((char *)aShowDate, "%.2d-%.2d-%.2d", __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC)),
          __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC)),
          2000 + __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC)));
}


/**
* @brief Compares two 8-bit buffers and returns the comparison result.
* @param pBuffer1: pointer to the source buffer to be compared to.
* @param pBuffer2: pointer to the second source buffer to be compared to the first.
* @param BufferLength: buffer's length.
* @retval   0: Comparison is OK (the two Buffers are identical)
*           Value different from 0: Comparison is NOK (Buffers are different)
*/
uint8_t Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return 0;
    }
    pBuffer1++;
    pBuffer2++;
  }  
  return 1;
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
  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(Period);
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

