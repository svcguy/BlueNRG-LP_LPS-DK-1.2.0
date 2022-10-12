
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RTC_Calendar_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 19-April-2019
* Description        : Code demonstrating the basic RTC functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RTC_Calendar/RTC_Calendar_main.c
 * @brief Configuration of the calendar using the RTC HAL API.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\RTC\\RTC_Calendar\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RTC_Calendar.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\RTC\\RTC_Calendar\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RTC_Calendar.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\RTC\\RTC_Calendar\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  LED name  |      STEVAL-IDB011V1     |      STEVAL-IDB011V2     |      STEVAL-IDB012V1     |
--------------------------------------------------------------------------------------------------
|     DL1    |         Not Used         |         Not Used         |         Not Used         |
|     DL2    |   On: RTC is configured  |   On: RTC is configured  |   On: RTC is configured  |
|     DL3    |      blinking: error     |      blinking: error     |      blinking: error     |
|     DL4    |         Not Used         |         Not Used         |         Not Used         |
|     U5     |         Not Used         |         Not Used         |         Not Used         |

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

Configuration of the calendar using the RTC HAL API.

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals, initialize the Flash interface and the systick.

The RTC peripheral configuration is ensured by the HAL_RTC_Init() function.
This later is calling the HAL_RTC_MspInit() function which core is implementing the configuration of the needed RTC resources according to the used hardware (CLOCK, PWR, RTC clock source and BackUp). 
You may update this function to change RTC configuration.

LSE oscillator clock is used as RTC clock source. 

HAL_RTC_SetTime() and HAL_RTC_SetDate() functions are then called to initialize the time and the date.

A key value is written in backup data register 1 to indicate if the RTC is already configured.  
The RTC is in the backup (BKP) domain, still powered by VBAT when VDD is switched off, so the RTC configuration is not lost if a battery is connected to the VBAT pin. 
The program behaves as follows:

After startup the program checks the backup data register 0 value:
- BKP_DR0 value not correct: (RTC_BKP_DR0 value is not correct or has not yet been programmed when the program is executed for the first time) the RTC is configured and internal time stamp is enabled. LED3 is turned on.
- BKP_DR0 value correct: this means that the RTC is configured and the time and date are displayed on Debugger. 

The current time and date are updated and displayed on the USART1.


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
#include "RTC_Calendar_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define RTC_BKP_DATA_VALUE 0x32F1

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

/* Buffers used for displaying Time and Date */
uint8_t aShowTime[16] = "hh:ms:ss";
uint8_t aShowDate[16] = "dd-mm-yyyy";
__IO uint32_t RTCStatus = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_RTC_Init(void);
static void RTC_CalendarShow(uint8_t *showtime, uint8_t *showdate);

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
  
  /* Configure LED2 and LED3 */
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  
  /* Initialize all configured peripherals */
  MX_RTC_Init();
  BSP_LED_On(BSP_LED2);
  
  /* Infinite loop */
  RTCStatus = 1;
  while (1)
  {
    /* Display the updated Time and Date */
    RTC_CalendarShow(aShowTime, aShowDate);
    printf("%s\t%s\n\r",aShowTime, aShowDate);
    HAL_Delay(1000);
  }
}

/**
* @brief RTC Initialization Function
* @param None
* @retval None
*/
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* Set Date and Time (if not already done before)*/
  /* Read the Back Up Register 0 Data */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != RTC_BKP_DATA_VALUE)
  {
    /* RTC clock enable */
    __HAL_RCC_RTC_CLK_ENABLE();
    
    __HAL_RCC_CLEAR_IT(RCC_IT_RTCRSTRELRDY);
    /* Force RTC peripheral reset */
    __HAL_RCC_RTC_FORCE_RESET();
    __HAL_RCC_RTC_RELEASE_RESET();
    /* Check if RTC Reset Release flag interrupt occurred or not */
    while(__HAL_RCC_GET_IT(RCC_IT_RTCRSTRELRDY) == 0)
    {
    }
    __HAL_RCC_CLEAR_IT(RCC_IT_RTCRSTRELRDY);
    
    /** Initialize RTC Only */
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
    hrtc.Init.SynchPrediv = RTC_SYNCH_PREDIV;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
      Error_Handler();
    }
    /* Configure RTC Calendar */
    printf("Configure RTC Calendar\n\r");
    
    /** Initialize RTC and set the Time and Date */
    sTime.Hours = 0x0;
    sTime.Minutes = 0x59;
    sTime.Seconds = 0x50;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 0x01;
    sDate.Year = 0x20;
    
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }
    
    /* Writes a data in a RTC Backup data Register0 */
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, RTC_BKP_DATA_VALUE);
  }
  else
  {
    printf("Read the Back Up Register: Set Date and Time is already done before\n\r");
  }
}

/**
* @brief  Display the current time and date.
* @param  showtime : pointer to buffer
* @param  showdate : pointer to buffer
* @retval None
*/
static void RTC_CalendarShow(uint8_t *showtime, uint8_t *showdate)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  sprintf((char *)showtime, "%2d:%2d:%2d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
  /* Display date Format : mm-dd-yy */
  sprintf((char *)showdate, "%2d-%2d-%2d", sdatestructureget.Month, sdatestructureget.Date, 2000 + sdatestructureget.Year);
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  RTCStatus = 0xE;
  while (1)
  {
    HAL_Delay(1000);
    BSP_LED_Toggle(BSP_LED3);
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
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



