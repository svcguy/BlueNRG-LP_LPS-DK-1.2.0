
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : Reset_Manager.c
* Author             : RF Application Team
* Version            : 2.0.0
* Date               : 04-January-2021
* Description        : Bluetooth LE Static stack
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file Reset_Manager.c
 * @brief This application provides an example of the Bluetooth LE Static Stack building. 
 * It must be loaded on BlueNRG-LP device, as preliminary step for using the Bluetooth LE Static Stack approach.


* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_StaticStack\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_StaticStack.uvprojx</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Debug->Start/Stop Debug Session  to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_StaticStack\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_StaticStack.eww</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_StaticStack\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Basic - Basic configuration
- \c OTA_BTL_ResetManager_Basic - Basic Configuration with OTA support


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
|            |                             Basic                             |||                                 OTA_BTL_ResetManager_Basic                                  |||
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  PIN name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |        STEVAL-IDB011V1       |        STEVAL-IDB011V2       |        STEVAL-IDB012V1       |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     A0     |        N.A.        |        N.A.        |      Not Used      |             N.A.             |             N.A.             |           Not Used           |
|     A1     |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     A10    |        N.A.        |        N.A.        |      Not Used      |             N.A.             |             N.A.             |           Not Used           |
|     A11    |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     A12    |      Not Used      |      Not Used      |        N.A.        |           Not Used           |           Not Used           |             N.A.             |
|     A13    |      Not Used      |      Not Used      |        N.A.        |           Not Used           |           Not Used           |             N.A.             |
|     A14    |      Not Used      |      Not Used      |        N.A.        |           Not Used           |           Not Used           |             N.A.             |
|     A15    |      Not Used      |      Not Used      |        N.A.        |           Not Used           |           Not Used           |             N.A.             |
|     A3     |        N.A.        |        N.A.        |      Not Used      |             N.A.             |             N.A.             |           Not Used           |
|     A4     |      Not Used      |      Not Used      |        N.A.        |           Not Used           |           Not Used           |             N.A.             |
|     A5     |      Not Used      |      Not Used      |        N.A.        |           Not Used           |           Not Used           |             N.A.             |
|     A6     |      Not Used      |      Not Used      |        N.A.        |           Not Used           |           Not Used           |             N.A.             |
|     A7     |      Not Used      |      Not Used      |        N.A.        |           Not Used           |           Not Used           |             N.A.             |
|     A8     |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     A9     |      Not Used      |      Not Used      |        N.A.        |           Not Used           |           Not Used           |             N.A.             |
|     B0     |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     B1     |        N.A.        |        N.A.        |      Not Used      |             N.A.             |             N.A.             |           Not Used           |
|     B12    |        N.A.        |        N.A.        |      Not Used      |             N.A.             |             N.A.             |           Not Used           |
|     B13    |        N.A.        |        N.A.        |      Not Used      |             N.A.             |             N.A.             |           Not Used           |
|     B14    |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     B15    |        N.A.        |        N.A.        |      Not Used      |             N.A.             |             N.A.             |           Not Used           |
|     B2     |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     B3     |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     B4     |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     B5     |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     B6     |        N.A.        |        N.A.        |      Not Used      |             N.A.             |             N.A.             |           Not Used           |
|     B7     |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     B8     |      Not Used      |      Not Used      |        N.A.        |           Not Used           |           Not Used           |             N.A.             |
|     B9     |      Not Used      |      Not Used      |        N.A.        |           Not Used           |           Not Used           |             N.A.             |
|     GND    |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     RST    |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|    VBAT    |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |

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
|            |                             Basic                             |||                                 OTA_BTL_ResetManager_Basic                                  |||
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |        STEVAL-IDB011V1       |        STEVAL-IDB011V2       |        STEVAL-IDB012V1       |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |
|     U5     |      Not Used      |      Not Used      |      Not Used      |           Not Used           |           Not Used           |           Not Used           |

@endtable


* \section Buttons_description Buttons description
@table
|                |                              Basic                              |||                                 OTA_BTL_ResetManager_Basic                                  |||
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |        STEVAL-IDB011V1       |        STEVAL-IDB011V2       |        STEVAL-IDB012V1       |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |       Not Used       |           Not Used           |           Not Used           |           Not Used           |
|      PUSH2     |      Not Used      |      Not Used      |       Not Used       |           Not Used           |           Not Used           |           Not Used           |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |       Reset BlueNRG-LP       |       Reset BlueNRG-LP       |       Reset BlueNRG-LPS      |

@endtable

* \section Usage Usage
   -  Bluetooth Low Energy Static Stack example: it allows to build a project containing the Bluetooth LE stack library with all (or part) of its APIs and build a second project that does not contains the stack and nevertheless it can use it. 
   - A second project, containing only the application, can be programmed into the device without reprogramming the Bluetooth LE stack.  

   - NOTES: This process assumes that Bluetooth Low Energy static stack application has been loaded as first step. The related demo application built for running with Bluetooth Low Energy static stack must be loaded without removing the previoulsy loaded Bluetooth LE static stack. BLE_SensorDemo_StaticStack project provides an example of Sensor Demo variant which uses the Bluetooth LE static stack approach.

**/
   
/** @addtogroup BlueNRGLP_demonstrations_applications
 * BlueNRG-LP BLE Static Stack \see Reset_Manager.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
#include "Reset_Manager_Config.h"
#include "system_BlueNRG_LP.h"

int main(void)
{
  if(*(uint32_t *)BLUE_FLAG_FLASH_BASE_ADDRESS == BLUE_FLAG_TAG){
    
    EntryPoint entryPoint = (EntryPoint)(*(volatile uint32_t *)(APP_ADDR + 4));
    __set_MSP(*(volatile uint32_t*) APP_ADDR);
    entryPoint();
  }  
  while(1);
}

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
