
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : Power_Consumption_Test.c
* Author             : RF Application Team
* Version            : 2.0.0
* Date               : 04-January-2021
* Description        : Code demonstrating the BlueNRG-LP current consumption in Bluetooth LE normal operating mode
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file Power_Consumption_Test.c
 * @brief This demonstration application allows to properly configure the BlueNRG-LP devices and show the related current consumption  during two Bluetooth LE operating modes: advertising and connection.
 * 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Power_Consumption\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_Power_Consumption.uvprojx</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Power_Consumption\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_Power_Consumption.eww</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Power_Consumption\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
  The application will listen for keys typed and it will execute the associated command.
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
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |
|      RESET     |      Not Used      |      Not Used      |      Not Used      |

@endtable

* \section Usage Usage

The application allows to put the selected Bluetooth LE device in discovery mode: user can select from a test menu which advertising interval to use (100 ms or 1000 ms). To take the BlueNRG-LP current consumption is necessary to connect a DC power analyzer to the JP1 connector of the STEVAL-IDB011V1 kit platforms.
Further, user can setup a connection with another device configured as a master and take the related power consumption measurements. 
The master role can be covered from another BlueNRG-LP platform configured with the DTM FW application (DTM_UART.hex), and running a specific script through BlueNRG GUI or Script launcher PC applications. 
On BLE_Power_Consumption demo application project folder, two scripts are provided for configuring the master device and create a connection with the BlueNRG-Lp kit platform under test.
The two scripts allow to establish a connection with, respectively, 100 ms and 1000 ms as connection interval.

The Power_Consumption_Test_parameters.h header file highlighs some  configurations parameters and 
the possible options in order to improve power consumption performances: 
 - System clock and Bluetooth LE clocks configuration
 - High Speed Cystal (HSE) start up time
 - LSE oscillator drive capability 

System clock (SYSCLK_DIRECT_HSE) and Bluetooth LE (BLE_SYSCLK_16M) clocks configuration is the lowest power consumption.
High Speed Cystal (HSE) start up time has been reduced to 512 us (instead of default one: 780 us): 
please refer to the related "WARNING" on  Power_Consumption_Test_parameters.h header file. 
LSE oscillator drive capability is still set to HIGH value (LL_RCC_LSEDRIVE_HIGH). User could try to reduce this value on 
system_BlueNRG_LP.c, LSConfigLSE() function (LL_RCC_LSEDRIVE_MEDIUMHIGH, LL_RCC_LSEDRIVE_MEDIUMLOW, LL_RCC_LSEDRIVE_LOW),
but this change is not recommended since it could severely impacts the proper device functionality.
  
 
The power consumption demo supports some test commands:
- <b>f</b>: Device in Discoverable mode with fast interval 100 ms.
- <b>s</b>: Device in discoverable mode with slow interval 1000 ms.
- <b>r</b>: Reset the BlueNRG-LP.
- <b>?</b>: Display this help menu.


**/
   

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
*/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "Power_Consumption_Test_parameters.h"
#include "Power_Consumption_Test_config.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"
#include "gap_profile.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define ADV_INTERVAL_FAST_MS  100
#define ADV_INTERVAL_SLOW_MS  1000

#define DEVICE_IDLE_STATE         0
#define DEVICE_ADV_STATE          1
#define DEVICE_CONN_STATE         2
#define DEVICE_SLEEP_STATE        3
#define DEVICE_DISCONNECTED_STATE 4

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

static uint8_t device_state;
static uint16_t connection_interval;

static uint8_t adv_data[] = {0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
                               21, AD_TYPE_COMPLETE_LOCAL_NAME, 'P', 'o', 'w', 'e', 'r', 'C', 'o', 'n', 's', 'u', 'm', 'p', 't', 'i', 'o', 'n', 'T', 'e', 's', 't'};

static Advertising_Set_Parameters_t Advertising_Set_Parameters[1]; 

/*******************************************************************************
 * Function Name  : help.
 * Description    : Prints the test option.
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void help(void)
{
  printf("\r\n");
  printf("f:   Device in Discoverable mode with fast interval 100 ms \r\n");
  printf("s:   Device in discoverable mode with slow interval 1000 ms \r\n");
  printf("r:   Reset the BlueNRG-LP\r\n");
  printf("?:   Display this help menu\r\n");
  printf("\r\n> ");
}

/*******************************************************************************
 * Function Name  : DeviceInit.
 * Description    : Init the device.
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void DeviceInit(void)
{
  uint8_t bdaddr[] = {0xBC, 0xEC, 0x00, 0xE1, 0x80, 0x02};
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t device_name[] = {'P', 'o', 'w', 'e', 'r','T', 'e', 's', 't'}; //'C', 'o', 'n', 's', 'u', 'm', 'p', 't', 'i', 'o', 'n',

  /* Set the device public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);  
  if(ret != BLE_STATUS_SUCCESS) {
    printf("aci_hal_write_config_data() failed: 0x%02x\r\n", ret);
  }
  
  /* Set the TX power 0 dBm */
  aci_hal_set_tx_power_level(0, 24);
  
  /* GATT Init */
  ret = aci_gatt_srv_init();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("aci_gatt_srv_init() failed: 0x%02x\r\n", ret);
  }
 
  /* GAP Init */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, sizeof(device_name), 0, &service_handle,
                     &dev_name_char_handle, &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("aci_gap_init() failed: 0x%02x\r\n", ret);
  }
   /* Set the device name */
  ret = Gap_profile_set_dev_name(0, sizeof(device_name), device_name);
  if (ret != BLE_STATUS_SUCCESS) {
    printf ("Error in Gap_profile_set_dev_name 0x%02x\r\n", ret);
    return;
  } else {
    printf ("Gap_profile_set_dev_name() --> SUCCESS\r\n");
  }
  
}

/*******************************************************************************
 * Function Name  : Configure_DeviceAdvertising.
 * Description    : Configure the device advertising interval
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Configure_DeviceAdvertising(uint16_t adv_interval)
{  
  uint8_t ret;
  
  if (adv_interval == ADV_INTERVAL_SLOW_MS)
    printf("Set Discoverable Mode Slow Interval 1000 ms\r\n");
  else
    printf("Set Discoverable Mode Fast Interval 100 ms\r\n");
   
  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                              ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                              (adv_interval*1000)/625,(adv_interval*1000)/625,
                                              ADV_CH_ALL,
                                              PUBLIC_ADDR,NULL,
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
  printf("Advertising configuration (Advertising Inteval: %d): %02X\n", adv_interval, ret);
  
  //ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, 0, NULL);//TBR
  
  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  
  printf("Set advertising data %02X\n", ret);  
  
}

/*******************************************************************************
 * Function Name  : Set_DeviceConnectable.
 * Description    : Puts the device in connectable mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Set_DeviceConnectable(void)
{  
  uint8_t ret;

  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;

  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 
  if(ret != BLE_STATUS_SUCCESS) {
    printf("aci_gap_set_advertising_enable --> FAILED: 0x%02x\r\n",ret);
  } else {
    device_state = DEVICE_ADV_STATE;
    printf("aci_gap_set_advertising_enable --> SUCCESS\r\n");
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
  uint8_t charRead;
  WakeupSourceConfig_TypeDef wakeupIO;
  PowerSaveLevels stopLevel;
  
  /* System initialization function.
     Set the clocks combination with the lowest power consumption.
     Refer to Power_Consumption_Test_parameters.h header file 
  */
  if (SystemInit(POWER_CONSUMPTION_DEMO_SYSTEM_CLOCK, POWER_CONSUMPTION_DEMO_BLE_CLOCK) != SUCCESS) 
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  /* Configure IOs for pwer save modes */
  BSP_IO_Init();
  
  /* Init the UART peripheral */
  BSP_COM_Init(NULL);

  ModulesInit(); 
  
  /* Setup the BLE GAP and GATT layer*/
  DeviceInit();

  printf("Power Consumption Test\r\n");
  printf("Enter ? for list of commands\r\n");

  /* No Wakeup Source needed */
  wakeupIO.IO_Mask_High_polarity = 0;
  wakeupIO.IO_Mask_Low_polarity = 0;
  wakeupIO.RTC_enable = 0;
  wakeupIO.LPU_enable = 0;
  
  device_state = DEVICE_IDLE_STATE;
  while(1) {
    
    ModulesTick();

    if (BSP_COM_Read(&charRead)) {
      switch (charRead) {
      case 'f':
        if (device_state == DEVICE_IDLE_STATE) {
          /* Device in discoverable mode with fast adv interval 100 ms */
          Configure_DeviceAdvertising(ADV_INTERVAL_FAST_MS);
          Set_DeviceConnectable();
          device_state = DEVICE_ADV_STATE;
        } else {
          printf ("Device already in discoverable or connected state. Please reset the device.\r\n");
        }
        break;
      case 's':
        if (device_state == DEVICE_IDLE_STATE) {
          /* Device in discoverable mode with slow adv interval 1000 ms */
          Configure_DeviceAdvertising(ADV_INTERVAL_SLOW_MS);
          Set_DeviceConnectable();
          device_state = DEVICE_ADV_STATE;
        } else {
          printf ("Device already in discoverable or connected state. Please reset the device.\r\n");
        }
        break;
      case '?':
        help();
        break;
      case 'r':
        NVIC_SystemReset();
        break;
      case '\r':
      case '\n':
        // ignored
        break;
      default:
        printf("Unknown Command\r\n");
      }
    }
  
    switch(device_state) {
    case DEVICE_DISCONNECTED_STATE:
      /* Enable the UART */
      BSP_COM_Init(NULL);
      printf ("Device Disconnected!!!\r\n");
      device_state = DEVICE_IDLE_STATE;
      break;
    case DEVICE_CONN_STATE:
      /* Enable the UART */
      BSP_COM_Init(NULL);
      printf("Device Connected with Interval %d * 1.25 ms\r\n", connection_interval);
      device_state = DEVICE_SLEEP_STATE;
      break;
    case DEVICE_IDLE_STATE:
      /* Nothing to Do */
      break;
    default:
      /* Disable the UART to save power */
      while(LL_USART_IsEnabled(BSP_UART) && (BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy()));
      BSP_COM_DeInit();

      /* Power Save Request */
      HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, wakeupIO, &stopLevel);
    }
  }
}

PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if (LL_USART_IsEnabled(BSP_UART) && (BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy()))
  {
    return POWER_SAVE_LEVEL_RUNNING;
  }
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{ 
  if (Status == BLE_STATUS_SUCCESS) {
    device_state = DEVICE_CONN_STATE;
    connection_interval = Conn_Interval;
  }
}/* end hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_le_enhanced_connection_complete_event.
 * Description    : This event indicates that a new connection has been created
 * Input          : See file bluenrg_lp_events.h
 * Output         : See file bluenrg_lp_events.h
 * Return         : See file bluenrg_lp_events.h
 *******************************************************************************/
void hci_le_enhanced_connection_complete_event(uint8_t Status,
                                               uint16_t Connection_Handle,
                                               uint8_t Role,
                                               uint8_t Peer_Address_Type,
                                               uint8_t Peer_Address[6],
                                               uint8_t Local_Resolvable_Private_Address[6],
                                               uint8_t Peer_Resolvable_Private_Address[6],
                                               uint16_t Conn_Interval,
                                               uint16_t Conn_Latency,
                                               uint16_t Supervision_Timeout,
                                               uint8_t Master_Clock_Accuracy)
{
  
  hci_le_connection_complete_event(Status,
                                   Connection_Handle,
                                   Role,
                                   Peer_Address_Type,
                                   Peer_Address,
                                   Conn_Interval,
                                   Conn_Latency,
                                   Supervision_Timeout,
                                   Master_Clock_Accuracy);
}

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  device_state = DEVICE_DISCONNECTED_STATE;
}/* end hci_disconnection_complete_event() */


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


/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/
/** \endcond
*/
