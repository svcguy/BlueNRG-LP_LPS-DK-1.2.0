
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : BLE_Throughput_main.c
* Author             : RF Application Team
* Version            : 2.0.0
* Date               : 26-August-2019
* Description        : Code demonstrating the BlueNRG-LP/BlueNRG-LPS Bluetooth LE Throughput with ATT_MTU, DLE, 1/2 Mbps PHY, L2CAP COS
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file BLE_Throughput_main.c
 * @brief This is a Throughput demo that shows how to implement some throughput tests  between two BlueNRG-LP,BlueNRG-LPS devices using the ATT MTU size up to 247 bytes, data length extension, 1Mbps, 2Mbps PHY, L2CAP COS features.
 * Several Throughput options are available: 
 * Server notification to client side with data length extension: 27, 100, 251 bytes
 * Server performs att mtu exchange to modify att mtu size
 * Bidirectional communication (server: notification and client: write without response) can be turned on, off.
 * User can set the PHY to be used for the communication (1Mbps, 2Mbps, ...).
 * 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Throughput\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_Throughput.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Debug->Start/Stop Debug Session to download the related binary image.
  -# Alternatively, open the Flasher utility, put the board in bootloader mode and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Throughput\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_Throughput.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility, put the board in bootloader mode and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Throughput\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Client - Client role configuration for throughput test
- \c Server - Server role configuration for throughput test


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
|            |                            Client                             |||                            Server                             |||
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
| Parameter name  | Value               | Unit      |
------------------------------------------------------
| Baudrate        | 921600              |  bit/sec  |
| Data bits       | 8                   | bit       |
| Parity          | None                | bit       |
| Stop bits       | 1                   | bit       |
@endtable

* \section LEDs_description LEDs description
@table
|            |                            Client                             |||                            Server                             |||
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
|                |                            Client                             |||                            Server                             |||
------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage

This Throughput demo has 2 roles:
 - The server that expose the Throughput service. It is the slave.
 - The client that uses the Throughput service. It is the master.

The Throughput Service contains 2 Characteristics:
 -# The TX Characteristic: the client can enable notifications on this characteristic. When the server has data to be sent, it sends notifications which contains the value of the TX Characteristic
 -# The RX Characteristic: it is a writable characteristic. When the client has data to be sent to the server, it writes a value into this characteristic.

The following interactive options are available on Server side (open hyperteminal with 921600, 8,None,1, None):   
        Press u to Send data len update request for 27 bytes
        Press m to Send data len update request for 100 bytes
        Press U to Send data len update request for 251 bytes
        Press z to Enable/disable slow throughput
        Press 1 to Change L2C COS MTU value
        Press 2 to Change L2C COS MPS value
        Press c to Send connection parameter update request
        Press f to Enable/disable flushable PDUs
        Press e to Toggle notify
        Press p to Print APP flags
        Press s to Read LE PHY (TX, RX)
        Press d to Set LE RX PHY to Coded
        Press D to Set LE TX PHY to Coded
        Press t to Set LE TX PHY to 1 Mbps
        Press r to Set LE RX PHY to 1 Mpbs
        Press T to Set LE TX PHY to 2 Mbps
        Press R to Set LE RX PHY to 2 Mpbs
        Press x to System reset
        Press ? to Print help

The following interactive options are available on Client side (open hyperteminal with 921600, 8,None,1, None):
        Press u to Send data len update request for 27 bytes
        Press m to Send data len update request for 100 bytes
        Press U to Send data len update request for 251 bytes
        Press a to Send ATT_MTU exchange
        Press z to Enable/disable slow throughput
        Press 1 to Change L2C COS MTU value
        Press 2 to Change L2C COS MPS value
        Press b to Switch bidirectional test on-off
        Press n to Send notifications
        Press i to Send indication
        Press p to Print APP flags
        Press s to Read LE PHY (TX, RX)
        Press d to Set LE RX PHY to Coded
        Press D to Set LE TX PHY to Coded
        Press t to Set LE TX PHY to 1 Mbps
        Press r to Set LE RX PHY to 1 Mpbs
        Press T to Set LE TX PHY to 2 Mbps
        Press R to Set LE RX PHY to 2 Mpbs
        Press x to System reset
        Press ? to Print help

NOTES:
 - The <b>Client</b> and <b>Server</b> workspaces (by default) allow to target a unidirectional throughput test: server device sends characteristic notifications (20 bytes) to the client device. The required serial port baudrate is 921600.

Test setup: 

 - Program the client side on one BlueNRG-LP platform and reset it. The platform is seen on the PC as a virtual COM port. Open the port in a serial terminal emulator. The required serial port baudrate is 921600
 - Program the server side on a second BlueNRG-LP platform and reset it. The platform is seen on the PC as a virtual COM port. Open the port in a serial terminal emulator. The required serial port baudrate is 921600.
-  The two platforms try to establish a connection. As soon as they get connected, the slave continuously sends notification of a characteristic to the client. 
 - User can play with the provided commands in order to select the following link layer packet length: 27 (default),100 and 251 (maximum allowed) bytes. 
 - User can also performs an ATT MTU exchange command on server side in order to allow server to increase the ATT_MTU size (247 bytes)
 - He can also enable the bidirectional throughput on client side (client writes on RX characteristic).
 - User can set the PHY to be used for the communication (1Mbps-default, 2Mbps, ...) 
 

**/
    
/** @addtogroup BlueNRGLP_demonstrations_applications
 * BlueNRG-LP extended packet length throughput demo \see BLE_Throughput_main.c for documentation.
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
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "clock.h"
#include "gp_timer.h" 
#include "app_state.h"
#include "throughput.h"
#include "Throughput_config.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"

#define BLE_THROUGHPUT_VERSION_STRING "1.0.0"

NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);

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
  
  NVIC_DisableIRQ(BSP_UART_IRQn); 
   
  /* Bluetooth stack tick */
  BLE_STACK_Tick();
  
  NVIC_EnableIRQ(BSP_UART_IRQn); 
  
  /* NVM manager tick */
  NVMDB_Tick();
}


int main(void)
{
    uint8_t ret;

    /* System initialization function */
    if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) 
    {
      /* Error during system clock configuration take appropriate action */
      while(1);
    }
    /* Configure IOs for pwer save modes */
    BSP_IO_Init();
    
    /* Init Clock */
    Clock_Init(); 

    /* Configure I/O communication channel */
    BSP_COM_Init(Process_InputData);
  
    ModulesInit(); 

#if SERVER
    printf("BlueNRG-LP BLE Throughput Server Application (version: %s)\r\n", BLE_THROUGHPUT_VERSION_STRING);
#else
    printf("BlueNRG-LP BLE Throughput Client Application (version: %s)\r\n", BLE_THROUGHPUT_VERSION_STRING);
#endif

    /* Init Throughput test */
    ret = DeviceInit();
    if (ret != BLE_STATUS_SUCCESS)
    {
        printf("DeviceInit()--> Failed 0x%02x\r\n", ret);
        while(1)
        {
        }
    }
    printf("BLE Stack Initialized \n");
    print_help();
    while(1)
    {
        ModulesTick();

        /* Application tick */
        APP_Tick();
    }
} /* end main() */


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



#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     * ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
