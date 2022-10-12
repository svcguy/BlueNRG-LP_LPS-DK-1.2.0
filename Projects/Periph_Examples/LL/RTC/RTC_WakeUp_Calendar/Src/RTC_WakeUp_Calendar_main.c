
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RTC_WakeUp_Calendar_main.c
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
 * @file  RTC_WakeUp_Calendar/RTC_WakeUp_Calendar_main.c
 * @brief This example shows how to configure RTC peripheral to set the power save level stop with timer.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\RTC\\RTC_WakeUp_Calendar\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RTC_WakeUp_Calendar.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\RTC\\RTC_WakeUp_Calendar\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RTC_WakeUp_Calendar.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\RTC\\RTC_WakeUp_Calendar\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A11    |        Output       |      Not Used      |
|     A12    |       Not Used      |        N.A.        |
|     A13    |       Not Used      |        N.A.        |
|     A14    |       Not Used      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |        Output       |        N.A.        |
|     A5     |       Not Used      |        N.A.        |
|     A6     |        Output       |        N.A.        |
|     A7     |        Output       |        N.A.        |
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
  The application will listen for keys typed and it will send back in the serial port.
  In other words everything typed in serial port will be send back.
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200           | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Start bits      | 1                | bit       |
| Stop bits       | 1                | bit       |
| HW flow control | None             | bit       |
@endtable


* \section LEDs_description LEDs description
@table
|  LED name  |                                 STEVAL-IDB011V1                                |                                 STEVAL-IDB011V2                                |                                 STEVAL-IDB012V1                                |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                                    Not Used                                    |                                    Not Used                                    |                                    Not Used                                    |
|     DL2    |  ON; RTC configuration is done correctly; toggling: system generates an error  |  ON; RTC configuration is done correctly; toggling: system generates an error  |  ON; RTC configuration is done correctly; toggling: system generates an error  |
|     DL3    |                                    Not Used                                    |                                    Not Used                                    |                                    Not Used                                    |
|     DL4    |                                    Not Used                                    |                                    Not Used                                    |                                    Not Used                                    |
|     U5     |                                    Not Used                                    |                                    Not Used                                    |                                    Not Used                                    |

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


Configuration of the LL API to set the RTC calendar. The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size).

The RTC peripheral configuration is ensured by the Configure_RTC() function (configure of the needed RTC resources according to the used hardware CLOCK, PWR, RTC clock source and BackUp). You may update this function to change RTC configuration.

Configure_RTC_Calendar function is then called to initialize the time and the date.

After startup from reset and system configuration, the  RTC Wakeup peripheral is initialized.
The application shows a state massage on USART peripheral and it is put in stop mode with timer enabled.
On first  character reception from USART Com port (ex: using HyperTerminal) the device is waked up. 
The device can be waked up also pressing the User push-button (PUSH1).
After 5 sec, if User doesn't interact with the application the device is waked up due to the RTC wake up timer is elapsed.

When the device is waked up the output value on port PA4, PA6, PA7 and PA11 is changed from high to low and vice versa.

A key value RTC_BKP_DATE_TIME_UPDTATED is written in backup data register 1 to indicate if the RTC is already configured.
To debug the intialize procedure when the backup data register 1 is been already written, is mandatory change the value of the #define RTC_BKP_DATE_TIME_UPDTATED.
 
When a reset (except power on reset) occurs the BKP domain is not reset and the RTC configuration is not lost.

When power on reset occurs and there are no battery connected to the VBAT pin: the BKP domain is reset and the RTC configuration is lost.
 
LED2 is turned ON when the RTC configuration is done correctly.
LED2 is toggling : This indicates that the system generates an error.
The current time and date are updated and displayed on the debugger in aShowTime and aShowDate variables (watch or live watch).

This example is applicable only for BlueNRG_LP cut 2.0 or above.

In order to make the program work, you must do the following:
 - Launch serial communication SW on PC
 - Flash the project in the Board
 - Press the RESET button

BlueNRG_LP-EVB Set-up
Connect the following pins to an oscilloscope to monitor the output waveforms:
- PA4
- PA6
- PA7
- PA11
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
#include <stdio.h>
#include <string.h>
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_rtc.h"
#include "rf_driver_ll_utils.h"
#include "hal_miscutil.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_device_it.h"
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "bluenrg_lpx.h"
#include "bluenrg_lp_evb_config.h"
#include "bluenrg_lp_evb_com.h"
#include "bluenrg_lp_evb_io.h"
#include "bluenrg_lp_evb_led.h"
#endif


#if defined(USE_FULL_ASSERT)
#include "rf_driver_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* the WUTF flag is set every (WUT[15:0] + 1) ck_wut cycles */
#define WAKEUP_TIMEOUT       4         /* 5 s */ 

/* ck_apre = Freq/(ASYNC prediv + 1) with 32 kHz  */
#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7F)
/* ck_spre = ck_apre/(SYNC prediv + 1) = 1 Hz */
#define RTC_SYNCH_PREDIV           ((uint32_t)0x00FF)

#define RTC_ERROR_NONE    0
#define RTC_ERROR_TIMEOUT 1

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

/**
  * @brief BSP_LED2 
  */

#ifdef STEVAL_IDB011V1
#define LED2_PIN                                LL_GPIO_PIN_8
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
#define LED2_PIN                                LL_GPIO_PIN_4
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#endif /* STEVAL_IDB012V1 */

/* Define used to indicate date/time updated */
#define RTC_BKP_DATE_TIME_UPDTATED ((uint32_t)0x32F2)

/** @addtogroup BlueNRGLP_StdPeriph_Examples Peripheral Examples
* @{
*/


/** @addtogroup RTC Examples
* @{
*/

/** @addtogroup RTC_WakeUp_Calendar Example
* @{
*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/  
/* Buffers used for displaying Time and Date */
uint8_t aShowTime[] = "hh:ms:ss";
uint8_t aShowDate[] = "mm:dd:aaaa";

/* Private function prototypes -----------------------------------------------*/
void PrintNegitatedLevel(uint8_t stopLevel);
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
void PrintWakeupSource(uint32_t wakeupSources);
#if defined(STEVAL_IDB011V1)
void TogglePinsOutputDeepstop(void);
#endif /* STEVAL_IDB011V1 */
#endif


uint32_t Enter_RTC_InitMode(void);
uint32_t Exit_RTC_InitMode(void);
uint32_t WaitForSynchro_RTC(void);
void Show_RTC_Calendar(void);

void LED_On(void);
void LED_Blinking(uint32_t Period);

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Display the Stop Level negotiated.
* @param  stopLevel negotiated Stop level
* @retval None
*/
void PrintNegitatedLevel(uint8_t stopLevel)
{
  printf("Power save level negotiated: ");
  switch (stopLevel)
  { 
  case POWER_SAVE_LEVEL_RUNNING:
    printf ("RUNNING\r\n");
    break;
  case POWER_SAVE_LEVEL_CPU_HALT:
    printf ("CPU_HALT\r\n");
    break;
  case POWER_SAVE_LEVEL_STOP_WITH_TIMER:
    printf ("STOP_WITH_TIMER\r\n");
    break;
  case POWER_SAVE_LEVEL_STOP_NOTIMER:
    printf ("STOP_NOTIMER\r\n");
    break;
  }
}

#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
/**
* @brief  Display the Wakeup Source.
* @param  wakeupSource Wakeup Sources
* @retval None
*/
void PrintWakeupSource(uint32_t wakeupSources)
{
  printf("Wakeup Source : ");
  switch (wakeupSources)
  {
  case WAKEUP_RTC:
    printf("WAKEUP_RTC ");
    break;
  case WAKEUP_PA10:
    printf("WAKEUP_PA10 ");
    break;
  case WAKEUP_PA8:
    printf("WAKEUP_PA8 ");
    break;
  case WAKEUP_PB0:
    printf("WAKEUP_PB0 ");
    break;
  default:
    printf("(default) WAKEUP source 0x%08x ", wakeupSources);
  }
  printf("\r\n");
}
#endif


void RTC_WakeupInit(void)
{
  LL_RTC_InitTypeDef RTC_InitStruct = {0};
  LL_RTC_TimeTypeDef RTC_TimeStruct = {0};
  LL_RTC_DateTypeDef RTC_DateStruct = {0};

  /* Enable Peripheral Clock */
  if (LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR1) != RTC_BKP_DATE_TIME_UPDTATED)
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
  
    /* Enter in initialization mode */
    if (Enter_RTC_InitMode() != RTC_ERROR_NONE)
    {
      /* Initialization Error */
      LED_Blinking(LED_BLINK_ERROR);
    }
    RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
    RTC_InitStruct.AsynchPrescaler = RTC_ASYNCH_PREDIV;
    RTC_InitStruct.SynchPrescaler = RTC_SYNCH_PREDIV;
    LL_RTC_Init(RTC, &RTC_InitStruct);
    
    RTC_TimeStruct.TimeFormat = LL_RTC_TIME_FORMAT_AM_OR_24;
    RTC_TimeStruct.Hours = 0x23;
    RTC_TimeStruct.Minutes = 0x59;
    RTC_TimeStruct.Seconds = 0x50;
    LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_TimeStruct);
    RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_FRIDAY;
    RTC_DateStruct.Month = LL_RTC_MONTH_DECEMBER;
    RTC_DateStruct.Day = 0x31;
    RTC_DateStruct.Year = 0x19;
    
    LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_DateStruct);
    
    /* Output disabled */
    LL_RTC_SetAlarmOutEvent(RTC, LL_RTC_ALARMOUT_DISABLE);
    
    /* Output polarity */
    LL_RTC_SetOutputPolarity(RTC, LL_RTC_OUTPUTPOLARITY_PIN_HIGH);
    
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
}

void SetRTC_WakeupTimeout(uint32_t time)
{
  /* Disable write protection */
  LL_RTC_DisableWriteProtection(RTC);
  
  /* Disable Wake-up Timer */
  LL_RTC_WAKEUP_Disable(RTC);
  
  /* In case of interrupt mode is used, the interrupt source must disabled */
  LL_RTC_DisableIT_WUT(RTC);
  
  /* Wait till RTC WUTWF flag is set  */
  while(LL_RTC_IsActiveFlag_WUTW(RTC) == 0);

#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* Clear PWR wake up Flag */
  LL_PWR_ClearWakeupSource(LL_PWR_EWS_INT);
#endif

  /* Clear RTC Wake Up timer Flag */
  LL_RTC_ClearFlag_WUT(RTC);
  
  /* Configure the Wake-up Timer counter */
  LL_RTC_WAKEUP_SetAutoReload(RTC, time);
  
  /* Configure the clock source */
  LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
  
  /* Configure the Interrupt in the RTC_CR register */
  LL_RTC_EnableIT_WUT(RTC);
  
  /* Enable the Wake-up Timer */
  LL_RTC_WAKEUP_Enable(RTC);
  
  /* Enable write protection */
  LL_RTC_EnableWriteProtection(RTC);
  
  /* Configure NVIC for RTC */
  NVIC_SetPriority(RTC_IRQn, IRQ_LOW_PRIORITY);
  NVIC_EnableIRQ(RTC_IRQn);    
  
  /* Clear RTC Wake Up timer Flag */
  LL_RTC_ClearFlag_WUT(RTC);
}

void DisableRTC_WakeupTimeout(void)
{
  /* Disable write protection */
  LL_RTC_DisableWriteProtection(RTC);
  
  /* Enable the Wake-up Timer */
  LL_RTC_WAKEUP_Disable(RTC);
  
  /* Enable write protection */
  LL_RTC_EnableWriteProtection(RTC);
}

/**
* @brief  Main program.
* @param  None
* @retval None
*/
int main(void)
{
  uint8_t ret_val;
  WakeupSourceConfig_TypeDef wakeupIO;
  PowerSaveLevels stopLevel;
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  uint32_t wakeupSources;
#endif

  
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, RADIO_SYSCLK_NONE) != SUCCESS)
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
  
  /* Init LED2 */
  BSP_LED_Init(BSP_LED2);
  
  /* Init BUTTON 1 */
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_EXTI);
  
  /* RTC Wakeup Peripheral Init */
  RTC_WakeupInit();
  
  while(1) {
    /* Display the updated Time and Date */
    Show_RTC_Calendar();
    
#if defined(STEVAL_IDB011V1) 
    TogglePinsOutputDeepstop();
#endif /* STEVAL_IDB011V1 */
    
    /* POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on UART (PA8)/timeout 5 sec, (RTC)/button PUSH1 (PA10) */
    printf("Enable Power Save Request : STOP_WITH_TIMER (RTC)\r\n");
    /* Display time Format : hh:ms:ss */
    printf(" Time : ");
    printf("%.2d:%.2d:%.2d", __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC)),
           __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC)),
           __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC)));
    printf("\n\r");
    /* Display date Format : mm-dd-yy */
    printf(" Date : ");
    printf("%.2d-%.2d-%.2d", __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC)),
           __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC)),
           2000 + __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC)));
    printf("\n\r");
    
    while(BSP_COM_UARTBusy());
    wakeupIO.RTC_enable = 1;
    wakeupIO.LPU_enable = 0;
#if defined(STEVAL_IDB011V1)
    wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
    wakeupIO.IO_Mask_Low_polarity = WAKEUP_PA8;
#endif
#if defined(STEVAL_IDB012V1)
    wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
    wakeupIO.IO_Mask_Low_polarity = WAKEUP_PB0;
#endif
    SetRTC_WakeupTimeout(WAKEUP_TIMEOUT);
    ret_val = HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_WITH_TIMER, wakeupIO, &stopLevel);
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
    if (ret_val != SUCCESS)
      printf("Error during clock config 0x%2x\r\n", ret_val);
    PrintNegitatedLevel(stopLevel);
    if (stopLevel >= POWER_SAVE_LEVEL_STOP_WITH_TIMER) {
      wakeupSources = HAL_PWR_MNGR_WakeupSource();
      PrintWakeupSource(wakeupSources);
    }
#endif

  }
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
* @retval RTC_ERROR_NONE if no error (RTC_ERROR_TIMEOUT will occur if RTC is
*         not synchronized)
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


#if defined(STEVAL_IDB011V1)
/**
* @brief  Toggle output signa on PA11 every time that wake on.
* @param  None
* @retval None
*/
void TogglePinsOutputDeepstop(void)
{
  uint32_t tmp = LL_PWR_GetPA11OutputinDEEPSTOP();
  if(tmp==LL_PWR_IOCFG_HIGH)
    LL_PWR_SetPA11OutputinDEEPSTOP(LL_PWR_IOCFG_LOW);
  else
    LL_PWR_SetPA11OutputinDEEPSTOP(LL_PWR_IOCFG_HIGH);
  
  tmp = LL_PWR_GetPA4OutputinDEEPSTOP();
  if(tmp==LL_PWR_IOCFG_HIGH)
    LL_PWR_SetPA4OutputinDEEPSTOP(LL_PWR_IOCFG_LOW);
  else
    LL_PWR_SetPA4OutputinDEEPSTOP(LL_PWR_IOCFG_HIGH);
  
  tmp = LL_PWR_GetPA6OutputinDEEPSTOP();
  if(tmp==LL_PWR_IOCFG_HIGH)
    LL_PWR_SetPA6OutputinDEEPSTOP(LL_PWR_IOCFG_LOW);
  else
    LL_PWR_SetPA6OutputinDEEPSTOP(LL_PWR_IOCFG_HIGH);
  
  tmp = LL_PWR_GetPA7OutputinDEEPSTOP();
  if(tmp==LL_PWR_IOCFG_HIGH)
    LL_PWR_SetPA7OutputinDEEPSTOP(LL_PWR_IOCFG_LOW);
  else
    LL_PWR_SetPA7OutputinDEEPSTOP(LL_PWR_IOCFG_HIGH);
}
#endif /* STEVAL_IDB011V1 */

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

#endif

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/



