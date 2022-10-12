
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : BLE_Beacon_Flash_main.c
* Author             : RF Application Team
* Version            : 2.0.0
* Date               : 04-January-2021
* Description        : Code demonstrating the Bluetooth LE Beacon application and coexistence with Flash operations
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file BLE_Beacon_Flash_main.c
 * @brief This is a Bluetooth LE beacon demo that shows how to configure a BlueNRG-LP device 
 * in order to advertise specific manufacturing data and performs Flash write operations
 * by synchronizing them with the planned Bluetooth LE radio operations
 * 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon_FlashManagement\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_Beacon_FlashManagement.uvprojx</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Debug->Start/Stop Debug Session to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon_FlashManagement\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_Beacon_FlashManagement.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon_FlashManagement\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  PIN name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------
|     A0     |        N.A.        |        N.A.        |      Not Used      |
|     A1     |      Not Used      |      Not Used      |      Not Used      |
|     A10    |        N.A.        |        N.A.        |      Not Used      |
|     A11    |      Not Used      |      Not Used      |      Not Used      |
|     A12    |      Not Used      |      Not Used      |        N.A.        |
|     A13    |      Not Used      |      Not Used      |        N.A.        |
|     A14    |      Not Used      |      Not Used      |        N.A.        |
|     A15    |      Not Used      |      Not Used      |        N.A.        |
|     A3     |        N.A.        |        N.A.        |      Not Used      |
|     A4     |      Not Used      |      Not Used      |        N.A.        |
|     A5     |      Not Used      |      Not Used      |        N.A.        |
|     A6     |      Not Used      |      Not Used      |        N.A.        |
|     A7     |      Not Used      |      Not Used      |        N.A.        |
|     A8     |      Not Used      |      Not Used      |      Not Used      |
|     A9     |      Not Used      |      Not Used      |        N.A.        |
|     B0     |      Not Used      |      Not Used      |      Not Used      |
|     B1     |        N.A.        |        N.A.        |      Not Used      |
|     B12    |        N.A.        |        N.A.        |      Not Used      |
|     B13    |        N.A.        |        N.A.        |      Not Used      |
|     B14    |      Not Used      |      Not Used      |      Not Used      |
|     B15    |        N.A.        |        N.A.        |      Not Used      |
|     B2     |      Not Used      |      Not Used      |      Not Used      |
|     B3     |      Not Used      |      Not Used      |      Not Used      |
|     B4     |      Not Used      |      Not Used      |      Not Used      |
|     B5     |      Not Used      |      Not Used      |      Not Used      |
|     B6     |        N.A.        |        N.A.        |      Not Used      |
|     B7     |      Not Used      |      Not Used      |      Not Used      |
|     B8     |      Not Used      |      Not Used      |        N.A.        |
|     B9     |      Not Used      |      Not Used      |        N.A.        |
|     GND    |      Not Used      |      Not Used      |      Not Used      |
|     RST    |      Not Used      |      Not Used      |      Not Used      |
|    VBAT    |      Not Used      |      Not Used      |      Not Used      |

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
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |
|     U5     |      Not Used      |      Not Used      |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |
--------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |       Not Used       |
|      PUSH2     |      Not Used      |      Not Used      |       Not Used       |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage

The Beacon demo configures a BlueNRG-LP device in advertising mode (non-connectable mode) with specific manufacturing data.
It transmits advertisement packets at regular intervals which contain the following manufacturing data:
@table   
------------------------------------------------------------------------------------------------------------------------
| Data field              | Description                       | Notes                                                  |
------------------------------------------------------------------------------------------------------------------------
| Company identifier code | SIG company identifier (1)        | Default is 0x0030 (STMicroelectronics)                 |
| ID                      | Beacon ID                         | Fixed value                                            |
| Length                  | Length of the remaining payload   | NA                                                     |
| Location UUID           | Beacons UUID                      | It is used to distinguish specific beacons from others |
| Major number            | Identifier for a group of beacons | It is used to group a related set of beacons           |                                              
| Minor number            | Identifier for a single beacon    | It is used to identify a single beacon                 |                                       
| Tx Power                | 2's complement of the Tx power    | It is used to establish how far you are from device    |                                       
@endtable

 - (1): SIG company identifiers are available on https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers/
 - NA : Not Applicable;

Further this application shows how to properly handling Flash operations (Erase and Write) and preserving the Bluetooth LE radio activities.
This is achieved by synchronizing with the scheduled Bluetooth LE radio activities through the aci_hal_end_of_radio_activity_event() event callback timing information.
It is possible to check the Flash activity by reading the Flash memory at the address 0x10066800 (for example using the BlueNRGx Flasher - Utility GUI).

**/
   
/** @addtogroup BlueNRGLP_demonstrations_applications
 *  BlueNRG-LP Beacon demo \see BLE_Beacon_Flash_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "Beacon_config.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "rf_driver_ll_flash.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_BEACON_VERSION_STRING "1.0.0"

/* Set to 1 for enabling Flags AD Type position at the beginning 
   of the advertising packet */
#define ENABLE_FLAGS_AD_TYPE_AT_BEGINNING 1

#define FLASH_ERASE_GUARD_TIME   25  // 25 ms
#define FLASH_WRITE_GUARD_TIME   1  // 1 ms

#define BLE_BEACON_TEST_PAGE_ADDRESS          0x10066800

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t Next_Advertising_SysTime = 0;

/* Set AD Type Flags at beginning on Advertising packet  */
static uint8_t adv_data[] = {
  /* Advertising data: Flags AD Type: not support if boradcaster only */
  //0x02, 
  //0x01, 
  //0x06, 
  /* Advertising data: manufacturer specific data */
  26, //len
  AD_TYPE_MANUFACTURER_SPECIFIC_DATA,  //manufacturer type
  0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
  0x02,       // ID
  0x15,       //Length of the remaining payload
  0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
  0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
  0x00, 0x00, // Major number 
  0x00, 0x09, // Minor number 
  0xC8        //2's complement of the Tx power (-56dB)};      
};
NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

static Advertising_Set_Parameters_t Advertising_Set_Parameters[1]; 

/* Private function prototypes -----------------------------------------------*/
void FlashRoutine(void);

/* Private functions ---------------------------------------------------------*/

void ModulesInit(void)
{
  uint8_t ret;
  BLE_STACK_InitTypeDef BLE_STACK_InitParams = BLE_STACK_INIT_PARAMETERS;
  
  LL_AHB_EnableClock(LL_AHB_PERIPH_PKA|LL_AHB_PERIPH_RNG);

  BLECNTR_InitGlobal();
  
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
  BLEPLAT_Init();  
  if (PKAMGR_Init() == PKAMGR_ERROR)
  {
      while(1);
  }
  if (RNGMGR_Init() != RNGMGR_SUCCESS)
  {
      while(1);
  }
  
  /* Init the AES block */
  AESMGR_Init();
  
  /* BlueNRG-LP stack init */
  ret = BLE_STACK_Init(&BLE_STACK_InitParams);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BLE_STACK_Init() 0x%02x\r\n", ret);
    while(1);
  }

}

void ModulesTick(void)
{
  /* Timer tick */
  HAL_VTIMER_Tick();
  
  /* Bluetooth stack tick */
  BLE_STACK_Tick();
  
  /* NVM manager tick */
  NVMDB_Tick();
}

void Device_Init(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  
  /* Set the TX Power to 0 dBm */
  ret = aci_hal_set_tx_power_level(0, 24);
  if(ret != 0) {
    printf ("Error in aci_hal_set_tx_power_level() 0x%04xr\n", ret);
    while(1);
  }
  
  /* Init the GAP with broadcaster role */
  ret = aci_gap_init(GAP_BROADCASTER_ROLE, 0x00, 0x08, 0, &service_handle,
                     &dev_name_char_handle, &appearance_char_handle);
  if (ret != 0)
    printf ("Error in aci_gap_init() 0x%04x\r\n", ret);
  else
    printf ("aci_gap_init() --> SUCCESS\r\n");
}


/**
* @brief  Start beaconing
* @param  None 
* @retval None
*/
static void Start_Beaconing(void)
{  
  uint8_t ret = BLE_STATUS_SUCCESS;
  
  /* broadcaster mode */
  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_BROADCAST,
                                              ADV_PROP_LEGACY,
                                              160, 160,
                                              ADV_CH_ALL,
                                              0,NULL,
                                              ADV_NO_WHITE_LIST_USE,
                                               0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf ("Error in aci_gap_set_advertising_configuration() 0x%04x\r\n", ret);
    return;
  }
  
  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf ("Error in aci_gap_set_advertising_data() 0x%04x\r\n", ret);
    return;
  }
  
   /* Enable advertising */
  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;
  
  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf ("Error in aci_gap_set_advertising_enable() 0x%04x\r\n", ret);
    return;
  }
  else
    printf ("aci_gap_set_advertising_enable() --> SUCCESS\r\n");
}

int main(void) 
{
  WakeupSourceConfig_TypeDef wakeupIO;
  PowerSaveLevels stopLevel;
  
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) 
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  /* Configure IOs for pwer save modes */
  BSP_IO_Init();
  
  /* Init the UART peripheral */
  BSP_COM_Init(NULL);

  /* Erase a page of the Flash memory */
  
  LL_FLASH_Erase(FLASH, LL_FLASH_TYPE_ERASE_PAGES, (BLE_BEACON_TEST_PAGE_ADDRESS-_MEMORY_FLASH_BEGIN_)/_MEMORY_BYTES_PER_PAGE_, 1);
  
  ModulesInit(); 

  /* Init the BlueNRG-LP device */
  Device_Init();
  
  /* Enable the callback end_of_radio_activity_event for advertising */
  aci_hal_set_radio_activity_mask(0x0002);
  
  /* Start Beacon Non Connectable Mode*/
  Start_Beaconing();
  
  printf("BlueNRG-LP BLE Beacon Application (version: %s)\r\n", BLE_BEACON_VERSION_STRING); 
 
  /* No Wakeup Source needed */
  wakeupIO.IO_Mask_High_polarity = 0;
  wakeupIO.IO_Mask_Low_polarity = 0;
  wakeupIO.RTC_enable = 0;          
  wakeupIO.LPU_enable = 0;
  
  while(1) 
  {
    ModulesTick();
    
    /* Flash operation */
    FlashRoutine();
 
    /* Power Save Request */
    HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, wakeupIO, &stopLevel);
  }
}


void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)
{
  if (Next_State == 0x01) /* 0x01: Advertising */
  {  /* Check if data for flash write are ready: do write only if there is enough time before next radio activity */
    Next_Advertising_SysTime = Next_State_SysTime;
  }
}


void FlashRoutine(void)
{
  static uint32_t flash_counter = 0;
  static uint32_t flash_pattern = 0xAAAAAAAA;
  
  if(flash_counter == 512) {
    if (HAL_VTIMER_DiffSysTimeMs(Next_Advertising_SysTime, HAL_VTIMER_GetCurrentSysTime()) > FLASH_ERASE_GUARD_TIME) {
     
      if (LL_FLASH_GetFlag(FLASH, LL_FLASH_FLAG_CMDDONE) == SET) {
        LL_FLASH_Erase(FLASH, LL_FLASH_TYPE_ERASE_PAGES, (BLE_BEACON_TEST_PAGE_ADDRESS-_MEMORY_FLASH_BEGIN_)/_MEMORY_BYTES_PER_PAGE_, 1);
        flash_pattern = ~flash_pattern;
        flash_counter = 0;
      }
    }
  }
  else {
    if (HAL_VTIMER_DiffSysTimeMs(Next_Advertising_SysTime, HAL_VTIMER_GetCurrentSysTime()) > FLASH_WRITE_GUARD_TIME) {
     
      if (LL_FLASH_GetFlag(FLASH, LL_FLASH_FLAG_CMDDONE) == SET) {
        LL_FLASH_Program(FLASH, (BLE_BEACON_TEST_PAGE_ADDRESS + flash_counter*4), flash_pattern);
        flash_counter++;
      }
    }
  }  
}


/****************** BlueNRG-LP Power Management Callback ********************************/

PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
    return POWER_SAVE_LEVEL_RUNNING;
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}

/* Event used to notify the Host that a hardware failure has occurred in the Controller. 
   See bluenrg_lp_events.h. */
void hci_hardware_error_event(uint8_t Hardware_Code)
{
  if (Hardware_Code <= 0x03)
  {
    NVIC_SystemReset();
  }
}

/***************************************************************************************/

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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
