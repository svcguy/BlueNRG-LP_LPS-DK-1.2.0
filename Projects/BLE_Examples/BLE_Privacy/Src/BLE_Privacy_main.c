
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : BLE_Privacy_main.c
* Author             : RF Application Team
* Version            : 2.0.0
* Date               : 04-January-2021
* Description        : Code demonstrating Bluetooth LE Controller Privacy feature.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file BLE_Privacy_main.c
 * @brief This application contains an example which shows how implementing a Bluetooth LE Controller Privacy on a Central/Master and Peripheral/Slave device.
 * Controller privacy requires 32MHz High Speed Crystal on selected platform (BlueNRG-LP STEVAL Kits).
 * 
 * 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
      <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Privacy\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_Privacy.uvprojx</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Debug->Start/Stop Debug Session  to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Privacy\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_Privacy.eww</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Privacy\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Master - Controller Privacy: master configuration
- \c Slave - Controller Privacy: slave configuration


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
|            |                             Slave                             |||                            Master                             |||
--------------------------------------------------------------------------------------------------------------------------------------------------
|  PIN name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------------------------------------------------------------------------
|     A0     |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     A1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     A10    |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     A11    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     A12    |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A13    |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A14    |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A15    |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A3     |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     A4     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A5     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A6     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A7     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A8     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     A9     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     B0     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B1     |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B12    |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B13    |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B14    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B15    |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B3     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B4     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B5     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B6     |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B7     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B8     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     B9     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     GND    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     RST    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    VBAT    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |

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
|            |                             Slave                             |||                            Master                             |||
--------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     U5     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|                |                                Slave                                |||                               Master                                |||
------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |    STEVAL-IDB012V1   |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |    STEVAL-IDB012V1   |
------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |   BLE disconnection  |   BLE disconnection  |   BLE disconnection  |   BLE disconnection  |   BLE disconnection  |   BLE disconnection  |
|      PUSH2     |       Not Used       |       Not Used       |       Not Used       |       Not Used       |       Not Used       |       Not Used       |
|      RESET     |   Reset BlueNRG-LP   |   Reset BlueNRG-LP   |   Reset BlueNRG-LPS  |   Reset BlueNRG-LP   |   Reset BlueNRG-LP   |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage

This demo implements a Controller Privacy scenario using Privacy 1.2 available with BlueNRG-LP Bluetooth LE stack v3.x. 
Two devices Master, Slave, both configured with aci_gap_init(privacy flag = 0x02), which should perform these macro steps: 

 - Initially, master and slave devices have no info on their security database: the two devices should connect and make a paring and bonding (fixed key: 123456).
 - Once the bonding is completed, the slave calls the aci_gap_configure_white_and_resolving_list() API to add its bonded device address to the controller's white list and to the list of resolvable addresses
 - The master device enables the slave characteristic notification. After the first connection and the pairing/bonding phase, devices disconnect.
 - The slave enters undirected connectable mode with its own address type = resolvable address and white list = 0x03 as advertising filter policy.
 - The master device performs a direct connection to the detected slave device, which accepts the connection since the master address is on its white list: the two devices reconnect and the slave starts a notification cycle to the master.


- NOTES: 
  - When  connected, if user presses the Bluetooth LE platform  button  PUSH1 on one of the two devices, device disconnect and slave enters in undirected connectable mode with filtering enabled (WHITE_LIST_FOR_ALL: Process scan and connection requests only from devices in the White List). This implies that slave device accepts connection requests only from devices on its  white list: master device is still be able to connect to the slave device; any other device connection requests are not accepted from the slave device. 
 

**/
   
/** @addtogroup BlueNRGLP_demonstrations_applications
 * BlueNRG-LP Controller Privacy demo  \see BLE_Privacy_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes-----------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "bleplat.h"
#include "nvm_db.h"

#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"

#if SLAVE
#include "slave.h"
#elif MASTER
#include "master.h"  
#endif 
   
#include "rf_driver_hal_power_manager.h"
#include "osal.h"

#include "user_config.h"

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

static WakeupSourceConfig_TypeDef wakeupIO;

static void user_set_wakeup_source(Button_TypeDef Button)
{
  if (Button == BSP_PUSH1)
  {
    wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
    wakeupIO.IO_Mask_Low_polarity = 0;
    wakeupIO.RTC_enable = 0;
    wakeupIO.LPU_enable = 0;
  }
}

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

int main(void)
{
  PowerSaveLevels stopLevel;

  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) 
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  /* Configure IOs for pwer save modes */
  BSP_IO_Init();
  
  /* Configure I/O communication channel */
  BSP_COM_Init(BSP_COM_RxDataUserCb);

  ModulesInit();

  /* Application demo Led Init */
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED3);
  BSP_LED_On(BSP_LED1);
  BSP_LED_On(BSP_LED3);

  device_initialization();
  
#if SLAVE
  //set_database
  set_database();
  
  // Set timer for notification frequency
  set_slave_update_timer();

  //set_device_discoverable
  set_device_discoverable();
#elif MASTER
  // start slave discovery procedure
  device_scanning(); 
#else
#error "MASTER or SLAVE macro must be selected"
#endif

  /* Initialize the button */
  BSP_PB_Init(USER_BUTTON, BUTTON_MODE_EXTI); 
  
  user_set_wakeup_source(USER_BUTTON);

  while(1) {
    
    ModulesTick();
    
    /* Application Tick */
    APP_Tick();

    /* Power Save management */
    HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, wakeupIO, &stopLevel);
  }
}

/****************** BlueNRG-LP Power Management Callback ********************************/

PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(APP_FLAG(DO_TERMINATE) || BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
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


/**
  * This event is generated to report firmware error informations.
  * FW_Error_Type possible values: 
  * Values:
  - 0x01: L2CAP recombination failure
  - 0x02: GATT unexpected response
  - 0x03: GATT unexpected request
    After this event with error type (0x01, 0x02, 0x3) it is recommended to disconnect. 
*/
void aci_hal_fw_error_event(uint8_t FW_Error_Type,
                            uint8_t Data_Length,
                            uint8_t Data[])
{
  if (FW_Error_Type <= 0x03)
  {
    uint16_t connHandle;
    
    /* Data field is the connection handle where error has occurred */
    connHandle = LE_TO_HOST_16(Data);
    
    aci_gap_terminate(connHandle, BLE_ERROR_TERMINATED_REMOTE_USER); 
  }
}

/* User callback if an interrupt is associated to the wakeup source */
void HAL_PWR_MNGR_WakeupIOCallback(uint32_t source)
{
  if (source & WAKEUP_PA10) {    
    /* When connected BUTTON_1 allows to disconnects */
    if (APP_FLAG(CONNECTED))
    { 
      APP_FLAG_SET(DO_TERMINATE);
    }
  }
}


/***************************************************************************************/


#ifdef USE_FULL_ASSERT


/**
 * @brief  Reports the name of the source file and the source line number where the assert_param error has occurred
 * @param  file	pointer to the source file name.
 * @param  line	assert_param error line source number.
 * @retval None.
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/** \endcond 
*/
