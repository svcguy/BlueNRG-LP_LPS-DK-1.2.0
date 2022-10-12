
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : BLE_DirectionFinding_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 27-May-2021
* Description        : Code demonstrating Bluetooth LE Direction Finding feature (tag, locator, connection mode)
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file BLE_DirectionFinding_main.c
 * @brief This application contains an example which shows how to implement Bluetooth LE Direction Finding tag and locator roles using connection CTE mode.
 * It is supported only on BlueNRG-LPS device. 
 * 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
      <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_DirectionFinding\\MDK-ARM\\{STEVAL-IDB012V1}\\BLE_DirectionFinding.uvprojx</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Debug->Start/Stop Debug Session  to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_DirectionFinding\\EWARM\\{STEVAL-IDB012V1}\\BLE_DirectionFinding.eww</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_DirectionFinding\\WiSE-Studio\\{STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Locator - Direction Finding: locator configuration
- \c Tag - Direction Finding: tag configuration


* \section Board_supported Boards supported
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
|            |         Tag        |       Locator      |
--------------------------------------------------------
|  PIN name  |   STEVAL-IDB012V1  |   STEVAL-IDB012V1  |
--------------------------------------------------------
|     A0     |      Not Used      |      Not Used      |
|     A1     |      Not Used      |      Not Used      |
|     A10    |      Not Used      |      Not Used      |
|     A11    |      Not Used      |      Not Used      |
|     A12    |        N.A.        |        N.A.        |
|     A13    |        N.A.        |        N.A.        |
|     A14    |        N.A.        |        N.A.        |
|     A15    |        N.A.        |        N.A.        |
|     A3     |      Not Used      |      Not Used      |
|     A4     |        N.A.        |        N.A.        |
|     A5     |        N.A.        |        N.A.        |
|     A6     |        N.A.        |        N.A.        |
|     A7     |        N.A.        |        N.A.        |
|     A8     |      Not Used      |      Not Used      |
|     A9     |        N.A.        |        N.A.        |
|     B0     |      Not Used      |      Not Used      |
|     B1     |      Not Used      |      Not Used      |
|     B12    |      Not Used      |      Not Used      |
|     B13    |      Not Used      |      Not Used      |
|     B14    |      Not Used      |      Not Used      |
|     B15    |      Not Used      |      Not Used      |
|     B2     |      Not Used      |      Not Used      |
|     B3     |      Not Used      |      Not Used      |
|     B4     |      Not Used      |      Not Used      |
|     B5     |      Not Used      |      Not Used      |
|     B6     |      Not Used      |      Not Used      |
|     B7     |      Not Used      |      Not Used      |
|     B8     |        N.A.        |        N.A.        |
|     B9     |        N.A.        |        N.A.        |
|     GND    |      Not Used      |      Not Used      |
|     RST    |      Not Used      |      Not Used      |
|    VBAT    |      Not Used      |      Not Used      |

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
|            |         Tag        |       Locator      |
--------------------------------------------------------
|  LED name  |   STEVAL-IDB012V1  |   STEVAL-IDB012V1  |
--------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |
|     U5     |      Not Used      |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|                |          Tag         |        Locator       |
----------------------------------------------------------------
|   BUTTON name  |    STEVAL-IDB012V1   |    STEVAL-IDB012V1   |
----------------------------------------------------------------
|      PUSH1     |       Not Used       |       Not Used       |
|      PUSH2     |       Not Used       |       Not Used       |
|      RESET     |   Reset BlueNRG-LPS  |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage

This demo implements a basic Direction Finding scenario with tag and locator roles (connection CTE mode), in order to show how to use the related Bluetooth LE stack capabilities. 
When connected, the locator device sends CTE (Constant Tone Extension) requests to the peer to receive packets with CTE field, which is used to collect IQ samples for direction finding. The actual calculation of the direction from received IQ samples is beyond the scope of this demonstration application and it requires specific algorithms. 
 

**/
   
/** @addtogroup BlueNRGLP_demonstrations_applications
 * BlueNRG-LPS Direction Finding demo \see BLE_DirectionFinding_main.c for documentation.
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
#include "app_state.h"
#include "profile.h"
#include "DirectionFinding_config.h"
#include "OTA_btl.h" 
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "clock.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DEBUG

/* Private macro -------------------------------------------------------------*/
#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/
NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);
   
/* Private function prototypes -----------------------------------------------*/
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
    PRINTF("Error in BLE_STACK_Init() 0x%02x\r\n", ret);
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
  BSP_COM_Init(NULL);

  ModulesInit(); 

#if CTE_TAG
  PRINTF("BlueNRG-LPS Tag\n");
#else
  PRINTF("BlueNRG-LPS Locator\n"); 
#endif

  /* Init BLE profile */
  ret = Profile_Init();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Profile_Init()--> Failed 0x%02x\n", ret);
    while(1);
  }
  
  PRINTF("BLE Stack Initialized.\n");
   
  while(1) {
    
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
