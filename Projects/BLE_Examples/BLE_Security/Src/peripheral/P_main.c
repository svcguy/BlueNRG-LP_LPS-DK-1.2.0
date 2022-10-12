
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : P_main.c
* Author             : RF Application Team
* Version            : 2.0.0
* Date               : 04-January-2021
* Description        : Code demonstrating BlueNRG-LP Bluetooth LE security example on Peripheral device.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file P_main.c
 * @brief This application contains an example which shows how implementing a Bluetooth LE security scenario on a peripheral device.
 * 
 * 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
      <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Security\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_Security.uvprojx</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Debug->Start/Stop Debug Sessionto download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Security\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_Security.eww</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_Security\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Slave_JustWorks - Peripheral Security Just Works configuration
- \c Slave_NumericComp - Peripheral Security Numeric Comparison configuration
- \c Slave_PassKey_Fixed - Peripheral Security Pass Key entry (fixed pin) configuration
- \c Slave_PassKey_Random - Peripheral Security Pass Key entry (no fixed pin) configuration


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
|            |                           Slave_PassKey_Random                            |||                            Slave_PassKey_Fixed                            |||                        Slave_JustWorks                        |||                          Slave_NumericComp                          |||
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  PIN name  |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |     STEVAL-IDB012V1    |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |     STEVAL-IDB012V1    |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |    STEVAL-IDB012V1   |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     A0     |          N.A.          |          N.A.          |        Not Used        |          N.A.          |          N.A.          |        Not Used        |        N.A.        |        N.A.        |      Not Used      |         N.A.         |         N.A.         |       Not Used       |
|     A1     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     A10    |          N.A.          |          N.A.          |        Not Used        |          N.A.          |          N.A.          |        Not Used        |        N.A.        |        N.A.        |      Not Used      |         N.A.         |         N.A.         |       Not Used       |
|     A11    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     A12    |        Not Used        |        Not Used        |          N.A.          |        Not Used        |        Not Used        |          N.A.          |      Not Used      |      Not Used      |        N.A.        |       Not Used       |       Not Used       |         N.A.         |
|     A13    |        Not Used        |        Not Used        |          N.A.          |        Not Used        |        Not Used        |          N.A.          |      Not Used      |      Not Used      |        N.A.        |       Not Used       |       Not Used       |         N.A.         |
|     A14    |        Not Used        |        Not Used        |          N.A.          |        Not Used        |        Not Used        |          N.A.          |      Not Used      |      Not Used      |        N.A.        |       Not Used       |       Not Used       |         N.A.         |
|     A15    |        Not Used        |        Not Used        |          N.A.          |        Not Used        |        Not Used        |          N.A.          |      Not Used      |      Not Used      |        N.A.        |       Not Used       |       Not Used       |         N.A.         |
|     A3     |          N.A.          |          N.A.          |        Not Used        |          N.A.          |          N.A.          |        Not Used        |        N.A.        |        N.A.        |      Not Used      |         N.A.         |         N.A.         |       Not Used       |
|     A4     |        Not Used        |        Not Used        |          N.A.          |        Not Used        |        Not Used        |          N.A.          |      Not Used      |      Not Used      |        N.A.        |       Not Used       |       Not Used       |         N.A.         |
|     A5     |        Not Used        |        Not Used        |          N.A.          |        Not Used        |        Not Used        |          N.A.          |      Not Used      |      Not Used      |        N.A.        |       Not Used       |       Not Used       |         N.A.         |
|     A6     |        Not Used        |        Not Used        |          N.A.          |        Not Used        |        Not Used        |          N.A.          |      Not Used      |      Not Used      |        N.A.        |       Not Used       |       Not Used       |         N.A.         |
|     A7     |        Not Used        |        Not Used        |          N.A.          |        Not Used        |        Not Used        |          N.A.          |      Not Used      |      Not Used      |        N.A.        |       Not Used       |       Not Used       |         N.A.         |
|     A8     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     A9     |        Not Used        |        Not Used        |          N.A.          |        Not Used        |        Not Used        |          N.A.          |      Not Used      |      Not Used      |        N.A.        |       Not Used       |       Not Used       |         N.A.         |
|     B0     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     B1     |          N.A.          |          N.A.          |        Not Used        |          N.A.          |          N.A.          |        Not Used        |        N.A.        |        N.A.        |      Not Used      |         N.A.         |         N.A.         |       Not Used       |
|     B12    |          N.A.          |          N.A.          |        Not Used        |          N.A.          |          N.A.          |        Not Used        |        N.A.        |        N.A.        |      Not Used      |         N.A.         |         N.A.         |       Not Used       |
|     B13    |          N.A.          |          N.A.          |        Not Used        |          N.A.          |          N.A.          |        Not Used        |        N.A.        |        N.A.        |      Not Used      |         N.A.         |         N.A.         |       Not Used       |
|     B14    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     B15    |          N.A.          |          N.A.          |        Not Used        |          N.A.          |          N.A.          |        Not Used        |        N.A.        |        N.A.        |      Not Used      |         N.A.         |         N.A.         |       Not Used       |
|     B2     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     B3     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     B4     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     B5     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     B6     |          N.A.          |          N.A.          |        Not Used        |          N.A.          |          N.A.          |        Not Used        |        N.A.        |        N.A.        |      Not Used      |         N.A.         |         N.A.         |       Not Used       |
|     B7     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     B8     |        Not Used        |        Not Used        |          N.A.          |        Not Used        |        Not Used        |          N.A.          |      Not Used      |      Not Used      |        N.A.        |       Not Used       |       Not Used       |         N.A.         |
|     B9     |        Not Used        |        Not Used        |          N.A.          |        Not Used        |        Not Used        |          N.A.          |      Not Used      |      Not Used      |        N.A.        |       Not Used       |       Not Used       |         N.A.         |
|     GND    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     RST    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|    VBAT    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |

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
|            |                           Slave_PassKey_Random                            |||                            Slave_PassKey_Fixed                            |||                        Slave_JustWorks                        |||                          Slave_NumericComp                          |||
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |     STEVAL-IDB012V1    |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |     STEVAL-IDB012V1    |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |    STEVAL-IDB012V1   |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     DL2    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     DL3    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     DL4    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |
|     U5     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |      Not Used      |      Not Used      |      Not Used      |       Not Used       |       Not Used       |       Not Used       |

@endtable


* \section Buttons_description Buttons description
@table
|                |                           Slave_PassKey_Random                            |||                            Slave_PassKey_Fixed                            |||                           Slave_JustWorks                           |||                          Slave_NumericComp                          |||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |     STEVAL-IDB012V1    |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |     STEVAL-IDB012V1    |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |    STEVAL-IDB012V1   |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |    STEVAL-IDB012V1   |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |    BLE disconnection   |    BLE disconnection   |    BLE disconnection   |    BLE disconnection   |    BLE disconnection   |    BLE disconnection   |   BLE disconnection  |   BLE disconnection  |   BLE disconnection  |   BLE disconnection  |   BLE disconnection  |   BLE disconnection  |
|      PUSH2     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |       Not Used       |       Not Used       |       Not Used       |       Not Used       |       Not Used       |       Not Used       |
|      RESET     |    Reset BlueNRG-LP    |    Reset BlueNRG-LP    |    Reset BlueNRG-LPS   |    Reset BlueNRG-LP    |    Reset BlueNRG-LP    |    Reset BlueNRG-LPS   |   Reset BlueNRG-LP   |   Reset BlueNRG-LP   |   Reset BlueNRG-LPS  |   Reset BlueNRG-LP   |   Reset BlueNRG-LP   |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage

This demo addresses some  examples of Bluetooth Low Energy Security scenarios:

 - Pairing with pass key entry method (no fixed pin) and bonding (select: Peripheral_PassKey_Random and Master_PassKey_Random projects configurations)
 - Pairing with pass key entry method (with fixed pin) and bonding  (select: Peripheral_PassKey_Fixed and Master_PassKey_Fixed projects configurations)
 - Pairing with just works  method and bonding (select: Peripheral_JustWorks and Master_JustWorks projects configurations)
 - Pairing with Numeric Comparison(Bluetooth LE V4.2 security feature) and bondig (select: Peripheral_NumericComp and Master_NumericComp projects configurations)

In order to exercise each specific security scenario, each Peripheral project configuration must be used with the related Master project configuration as follows: 

@table Peripheral (Slave) and Central (Master) devices allowed project configurations combinations

|                                |  Slave_PassKey_Random  |  Slave_PassKey_Fixed  |  Slave_JustWorks  | Slave_NumericComp  |
----------------------------------------------------------------------------------------------------------------------------
|  Master_PassKey_Random         |           X            |                       |                   |                    |
|  Master_PassKey_Fixed          |                        |           X           |                   |                    |
|  Master_JustWorks              |                        |                       |        X          |                    |
|  Master_NumericComp            |                        |                       |                   |         X          |
@endtable

The following tables describes the projects main security settings, which combination lead to the selection of the specific Bluetooth Low Energy Security scenario
(PassKey entry with random pin, PassKey entry with fixed pin, Just works, Numeric Value Comparison).

- Peripheral (Slave) device security settings

@table 
|                                |   Slave_PassKey_Random    |  Slave_PassKey_Fixed      |   Slave_JustWorks       |  Slave_NumericComp            |
----------------------------------------------------------------------------------------------------------------------------------------------------
|  IO capability                 |  Display Only             |  Display Only             |  Display Only           |  Keyboard Display             |
|  Use Fixed Pin                 |  NO                       |  YES                      |  YES                    |  YES                          |
|  MITM mode                     |  Required                 |  Required                 |  Not Required           |  Required                     |
|  LE Secure support             |  Supported but optional   |  Supported but optional   |  Supported but optional |  Supported but optional       |
|  Key Press Notification        |  Not supported            |  Not supported            |  Not supported          |  Not supported                |
|  Identity  Address             |  Public                   |  Public                   |  Public                 |  Public                       |
@endtable

- Central (Master) device security settings

@table 
|                                |  Master_PassKey_Random    |  Master_PassKey_Fixed      |  Master_JustWorks        | Master_NumericComp          |
----------------------------------------------------------------------------------------------------------------------------------------------------
|  IO capability                 |  Keyboard Only            |  Keyboard Only             |  Display Only            |                             |
|  Use Fixed Pin                 |  NO                       |  YES                       |  NO                      |                             |
|  MITM mode                     |  Required                 |  Required                  |  Not Required            |                             |
|  LE Secure support (1)         |  Supported and mandatory  |  Supported and mandatory   |  Supported and mandatory | Supported and mandatory (2) |
|  Key Press Notification        |  Not supported            |  Not supported             |  Not supported           | Not supported               |
|  Identity  Address             |  Public                   |  Public                    |  Public                  | Public                      |
@endtable

- NOTES: 
  - (1) Set to Seupported and mandatory in order to force the Bluetooth LE V4.2 LE Secure connections when Central device connects to Peripheral device.
  - (2) Set to mandatory in order to trigger Numeric Comparison (it is supported on new Bluetooth LE V 4.2 LE Secure Connections mode) 

Central device is using the Central/Master library (refer to the related html documentation for more information). 

***  Pairing with pass key entry method (no fixed pin) and bonding:

On reset, after initialization, Peripheral device sets security IO capability as display only (IO_CAP_DISPLAY_ONLY) and set authentication requirements as follow:

- MITM authentication (MITM_PROTECTION_REQUIRED)
- don't use fixed pin (DONOT_USE_FIXED_PIN_FOR_PAIRING)
- Secure connection supported but optional  (SC_IS_SUPPORTED)
- Key press notification not supported (KEYPRESS_IS_NOT_SUPPORTED)
- Identity address public (0x00)
- bonding required (BONDING)
                                              
- NOTE: the security pin is randomly generated using the hci_le_rand() API. 
  The Pass Key value displayed on Peripheral hyper terminal must be inserted on Central device hyper terminal (Central device has IO capability keyboard only: IO_CAP_KEYBOARD_ONLY).

After initialization phase, Peripheral device defines a custom service with 2 proprietary characteristics (UUID 128 bits):

- TX characteristic: notification (CHAR_PROP_NOTIFY), no security (ATTR_PERMISSION_NONE); 

- RX characteristic with properties: read (CHAR_PROP_READ),  Link must be encrypted to read (ATTR_PERMISSION_ENCRY_READ), Need authentication to read (ATTR_PERMISSION_AUTHEN_READ),  
GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RES (application is notified when a read request of any type is received for this attribute). 


The Peripheral device enters in discovery mode with local name SlaveSec_A2. When a Central device starts the discovery procedure and detects the Peripheral device, the two devices connects.
After connection, Peripheral device starts a slave security request to the Central device (aci_gap_slave_security_req()) and , as consequence, Central devices starts pairing procedure.
After devices pairs and get bonded, Peripheral device displays the list of its bonded devices and configures its white list in order to add the bonded Central device to its white list.
Central devices starts the service discovery procedure to identify the Peripheral service and characteristics and, then, enabling the TX characteristic notification. 
Peripheral device starts  TX characteristic notification to the Central device at periodic interval,  and it provides the RX characteristic value to the Central device each time it reads it.
When  connected, if user presses the Bluetooth LE platform  button  PUSH1,  Peripheral device disconnects and enters in undirected connectable mode with filtering enabled (WHITE_LIST_FOR_ALL: Process scan and connection requests only from devices in the White List). This implies that Peripheral device accepts connection requests only from devices on its  white list: Central device is still be able to connect to the Peripheral device; any other device connection requests are not accepted from the Peripheral device. 

***  Pairing with pass key entry method (with fixed pin) and bonding:

On reset, after initialization, Peripheral device sets security IO capability as display only (IO_CAP_DISPLAY_ONLY) and set authentication requirements as follow:

- MITM authentication (MITM_PROTECTION_REQUIRED)
- use fixed pin (USE_FIXED_PIN_FOR_PAIRING)
- Secure connection supported but optional  (SC_IS_SUPPORTED)
- Key press notification not supported (KEYPRESS_IS_NOT_SUPPORTED)
- Identity address public (0x00)
- bonding required (BONDING)
                                              
- NOTE: the security pin is fixed (PERIPHERAL_SECURITY_KEY on file BLE_Security_Peripheral.h).

After initialization phase, Peripheral device defines a custom service with 2 proprietary characteristics (UUID 128 bits):

- TX characteristic: notification (CHAR_PROP_NOTIFY), no security (ATTR_PERMISSION_NONE); 

- RX characteristic with properties: read (CHAR_PROP_READ),  Link must be encrypted to read (ATTR_PERMISSION_ENCRY_READ), Need authentication to read (ATTR_PERMISSION_AUTHEN_READ),  
GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RES (application is notified when a read request of any type is received for this attribute). 


The Peripheral device enters in discovery mode with local name SlaveSec_A1. When a Central device starts the discovery procedure and detects the Peripheral device, the two devices connects.
After connection, Peripheral device starts a slave security request to the Central device (aci_gap_slave_security_req()) and , as consequence, Central devices starts pairing procedure.
After devices pairs and get bonded, Peripheral device displays the list of its bonded devices and configures its white list in order to add the bonded Central device to its white list.
Central devices starts the service discovery procedure to identify the Peripheral service and characteristics and, then, enabling the TX characteristic notification. 
Peripheral device starts  TX characteristic notification to the Central device at periodic interval,  and it provides the RX characteristic value to the Central device each time it reads it.
When connected, if user presses the Bluetooth LE platform  button  PUSH1,  Peripheral device disconnects and enters in undirected connectable mode mode with filtering enabled (WHITE_LIST_FOR_ALL: Process scan and connection requests only from devices in the White List). This implies that Peripheral device accepts connection requests only from devices on its  white list: Central device is still be able to connect to the Peripheral device; any other device connection requests are not accepted from the Peripheral device. 

***  Pairing with just works  method and bonding.

On reset, after initialization, Peripheral device sets security IO capability as display only (IO_CAP_DISPLAY_ONLY) and set authentication requirements as follow:

- MITM authentication (MITM_PROTECTION_NOT_REQUIRED)
- use fixed pin (USE_FIXED_PIN_FOR_PAIRING)
- Secure connection supported but optional  (SC_IS_SUPPORTED)
- Key press notification not supported (KEYPRESS_IS_NOT_SUPPORTED)
- Identity address public (0x00)
- bonding required (BONDING)
                                              
After initialization phase, Peripheral device defines a custom service with 2 proprietary characteristics (UUID 128 bits):

- TX characteristic: notification (CHAR_PROP_NOTIFY), no security (ATTR_PERMISSION_NONE); 

- RX characteristic with properties: read (CHAR_PROP_READ),  Link must be encrypted to read (ATTR_PERMISSION_ENCRY_READ)
GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RES (application is notified when a read request of any type is received for this attribute). 


The Peripheral device enters in discovery mode with local name SlaveSec_A0. When a Central device starts the discovery procedure and detects the Peripheral device, the two devices connects.
After connection, Peripheral device starts a slave security request to the Central device (aci_gap_slave_security_req()) and , as consequence, Central devices starts pairing procedure.
After devices pairs and get bonded, Peripheral device displays the list of its bonded devices and configures its white list in order to add the bonded Central device to its white list.
Central devices starts the service discovery procedure to identify the Peripheral service and characteristics and, then, enabling the TX characteristic notification. 
Peripheral device starts  TX characteristic notification to the Central device at periodic interval,  and it provides the RX characteristic value to the Central device each time it reads it.
When connected, if user presses the Bluetooth LE platform  button  PUSH1,  Peripheral device disconnects and enters in undirected connectable mode mode with filtering enabled (WHITE_LIST_FOR_ALL: Process scan and connection requests only from devices in the White List). This implies that Peripheral device accepts connection requests only from devices on its  white list: Central device is still be able to connect to the Peripheral device; any other device connection requests are not accepted from the Peripheral device. 

***  Pairing with Numeric Comparison (Bluetooth LE V4.2 only) and bonding:

On reset, after initialization, Peripheral device sets security IO capability as Keyboard, display  (IO_CAP_KEYBOARD_DISPLAY) and set authentication requirements as follow:

- MITM authentication (MITM_PROTECTION_REQUIRED)
- use fixed pin (DONOT_USE_FIXED_PIN_FOR_PAIRING)
- Secure connection supported but optional  (SC_IS_SUPPORTED)
- Key press notification not supported (KEYPRESS_IS_NOT_SUPPORTED)
- Identity address public (0x00)
- bonding required (BONDING)
                                              
After initialization phase, Peripheral device defines a custom service with 2 proprietary characteristics (UUID 128 bits):

- TX characteristic: notification (CHAR_PROP_NOTIFY), no security (ATTR_PERMISSION_NONE); 

- RX characteristic with properties: read (CHAR_PROP_READ),  Link must be encrypted to read (ATTR_PERMISSION_ENCRY_READ), Need authentication to read (ATTR_PERMISSION_AUTHEN_READ),  
GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RES (application is notified when a read request of any type is received for this attribute). 


The Peripheral device enters in discovery mode with local name SlaveSec_A3. When a Central device starts the discovery procedure and detects the Peripheral device, the two devices connects.
After connection, Peripheral device starts a slave security request to the Central device (aci_gap_slave_security_req()) and , as consequence, Central devices starts pairing procedure.

On both devices the confirm numeric value event is triggered and the related numeric value is displayed. User is requested to press key y in order to confirm the numeric value on both devices (any other key for not confirming it). 
After devices pairs and get bonded, Peripheral device displays the list of its bonded devices and configures its white list in order to add the bonded Central device to its white list.
Central devices starts the service discovery procedure to identify the Peripheral service and characteristics and, then, enabling the TX characteristic notification. 
Peripheral device starts  TX characteristic notification to the Central device at periodic interval,  and it provides the RX characteristic value to the Central device each time it reads it.
When  connected, if user presses the Bluetooth LE platform  button  PUSH1,  Peripheral device disconnects and enters in undirected connectable mode mode with filtering enabled (WHITE_LIST_FOR_ALL: Process scan and connection requests only from devices in the White List). This implies that Peripheral device accepts connection requests only from devices on its  white list: Central device is still be able to connect to the Peripheral device; any other device connection requests are not accepted from the Peripheral device. 

GENERAL NOTE: TX and RX characteristics length is 20 bytes and related values are defined as follow:

  - TX characteristic value: {'S','L','A','V','E','_','S','E','C','U','R','I','T','Y','_','T','X',' ',x1,x2};  where x1, x2 are counter values
  - RX characteristic value: {'S','L','A','V','E','_','S','E','C','U','R','I','T','Y','_','R','X',' ',x1,x2};  where x1, x2 are counter values
  - When using a Smarthphone as Central device, please notice that if this device uses a random resolvable address, Periheral device is able to accept connection or scan requests coming from  it on reconnection phase.
    This is due to the fact, that when disconnecting, Peripheral device enters in undirected connectable mode with filtering enabled (WHITE_LIST_FOR_ALL: Process scan and connection requests only from devices in the White List). As consequence, it is able to accept Smarthphone's scan request or connection requests, only if the Controller Privacy is enabled on Peripheral device (please refer to the BlueNRG GUI SW package (STSW-BNRGUI) script: BlueNRG GUI x.x.x\Application\scripts\Privacy_1_2_Whitelist\Privacy_1_2_Slave_WhiteList.py for a a complete reference example).
    A possible simple option, on Peripheral device, is to replace the WHITE_LIST_FOR_ALL advertising filter policy with NO_WHITE_LIST_USE: Peripheral device doesn't enable device filtering after reconnection,  and it is able to accept connection or scan requests coming from a Smartphone using resolvable random addresses. 


**/
   
/** @addtogroup BlueNRGLP_demonstrations_applications
 * BlueNRG-LP Security peripheral \see P_main.c for documentation.
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

#include "app_state.h"
#include "BLE_Security_Peripheral.h"

#include "BLE_Security_Peripheral_config.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_APPLICATION_VERSION_STRING "1.0.0" 

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static char Security_Configuration[] = SECURITY_MODE_NAME; 

/* Uart RX */
static WakeupSourceConfig_TypeDef wakeupIO = {0, 0, WAKEUP_PA8, 0};

NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

/* Private function prototypes -----------------------------------------------*/

static void user_set_wakeup_source(Button_TypeDef Button)
{
  if (Button == BSP_PUSH1) 
  {
    wakeupIO.IO_Mask_High_polarity |= WAKEUP_PA10;
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

/* Private functions ---------------------------------------------------------*/

int main(void) 
{
  uint8_t ret;
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
  
  /* Init Device */
  ret = DeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("DeviceInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }
  
  /* Initialize the button */
  BSP_PB_Init(USER_BUTTON, BUTTON_MODE_EXTI); 

  user_set_wakeup_source(USER_BUTTON); 
  
  PRINTF("BlueNRG-LP BLE Security Peripheral Application (version: %s, security mode: %s; button: %d)\r\n", BLE_APPLICATION_VERSION_STRING,Security_Configuration,USER_BUTTON);
  
  while(1) {
    
    ModulesTick();
    
    /* Application tick */
    APP_Tick();
    
    /* Power Save management: no timer and wakeup on UART RX, PUSH button */
    HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, wakeupIO, &stopLevel);
    
  }
  
} /* end main() */

/* User callback if an interrupt is associated to the wakeup source */
void HAL_PWR_MNGR_WakeupIOCallback(uint32_t source)
{
  if (source & WAKEUP_PA10) {    
    /* When connected BUTTON allows to disconnects */
    if (APP_FLAG(CONNECTED))
    {
      APP_FLAG_SET(START_TERMINATE_LINK_FLAG);
    }
  }
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
