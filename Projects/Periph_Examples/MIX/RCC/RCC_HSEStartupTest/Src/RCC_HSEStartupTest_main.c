
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RCC_HSEStartupTest_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 05-June-2020
* Description        : HSE Start-up Test example
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file RCC_HSEStartupTest/RCC_HSEStartupTest_main.c
 * @brief This is a demo that shows how to measure the HSE Start-up time for a BlueNRG-LP device 
 * 


* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RCC\\RCC_HSEStartupTest\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RCC_HSEStartupTest.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RCC\\RCC_HSEStartupTest\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RCC_HSEStartupTest.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RCC\\RCC_HSEStartupTest\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |
---------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |
|     U5     |      Not Used      |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |
-------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage

 - This project provides a reference example in order to measure the HSE startup time on the application board.A pulse on a configurable GPIO (PA4 is the default) is generated when HSE is ready. The startup time has to be measured from the time when the voltage begins to raise on VCAP pin and the time the voltage on the test GPIO is high.
 - The example also measures the start-up time using the internal clock as a reference. In this case no instrument is needed to measure the time.

@note The measurement must be taken at the minimum operating voltage, since start-up time increases at low voltage.

@note This example has been tested with STMicroelectronics BlueNRG_LP-EVB board and can be easily tailored to any other supported device and development board.

**/
   
/* Includes ------------------------------------------------------------------*/
#include "RCC_HSEStartupTest_main.h"
#include "rf_driver_hal_vtimer.h"
#include "rf_driver_ll_gpio.h"

/** @addtogroup BlueNRGLP_StdPeriph_Examples MIX Peripheral Examples
  * @{
  */


/** @addtogroup RCC_Examples Micro Examples
  * @{
  */

/** @addtogroup RCC_HSEStartupTest HSE Startup Test Example
  * @{
  */

uint32_t old_wakeup_time_mach, wakeup_time_mach, hse_ready_time_mach;

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define MAX(a,b) ((a) < (b) )? (b) : (a)

/* Private variables ---------------------------------------------------------*/
static VTIMER_HandleType timerHandle;

/* Private function prototypes -----------------------------------------------*/
void GPIO_Init();
void ModulesInit(void);
void ModulesTick(void);
void timeoutCB(void *param);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  WakeupSourceConfig_TypeDef wakeupIO = {0,0,0,0};  /* No Wakeup Source needed */
  PowerSaveLevels stopLevel;
  uint32_t max_hse_startup_time_sys = 0;

  /* MCU Configuration--------------------------------------------------------*/

  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
  
  GPIO_Init();
  
  ModulesInit();
  
  /* Enable HSE Ready interrupt */
  LL_RCC_ClearFlag_RC64MPLLRDY();
  LL_RCC_EnableIT_RC64MPLLRDY();
  NVIC_EnableIRQ(RCC_IRQn);
  
  timerHandle.callback = timeoutCB;
  HAL_VTIMER_StartTimerMs(&timerHandle, WAKEUP_INTERVAL_MS);
  
  printf("\nHSE test app started\n");

  /* Infinite loop */
  while (1)
  {
    ModulesTick();
    
    if(old_wakeup_time_mach != wakeup_time_mach)
    {
      uint32_t hse_startup_time_sys;
      
      old_wakeup_time_mach = wakeup_time_mach;
      hse_startup_time_sys = TIMER_MachineTimeToSysTime(hse_ready_time_mach - wakeup_time_mach);
      max_hse_startup_time_sys = MAX(hse_startup_time_sys,max_hse_startup_time_sys);
      //printf("%X %X %d\n", wakeup_time_mach, hse_ready_time_mach, hse_ready_time_mach - wakeup_time_mach);
      printf("Last: %u. Worst: %u (%u us)\n", hse_startup_time_sys, max_hse_startup_time_sys, max_hse_startup_time_sys*625/256);
    }
    
    /* Power Save Request */
    HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_WITH_TIMER, wakeupIO, &stopLevel);
  }
}

void GPIO_Init()
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {
    .Pin = LL_GPIO_PIN_X,
    .Mode = LL_GPIO_MODE_OUTPUT,
    .Speed = LL_GPIO_SPEED_FREQ_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    .Pull = LL_GPIO_PULL_NO,
  };
  
  BSP_LED_Init(BSP_LED1); //Activity led
  BSP_LED_On(BSP_LED1);
  
  /* GPIO for test signal */
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOX);  
  LL_GPIO_Init(GPIOX, &GPIO_InitStruct);
}

void ModulesInit(void)
{  
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  
  /* VTimer module Init */  
  HAL_VTIMER_Init(&VTIMER_InitStruct);
}

void ModulesTick(void)
{
  /* Timer tick */
  HAL_VTIMER_Tick();
}

void timeoutCB(void *param)
{
  HAL_VTIMER_StartTimerMs(&timerHandle, WAKEUP_INTERVAL_MS);  
}

PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
    return POWER_SAVE_LEVEL_RUNNING;
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  /* Turn LED3 on: Transfer Error */
  BSP_LED_On(BSP_LED3);
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{ 

  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}
#endif /* USE_FULL_ASSERT */



/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



