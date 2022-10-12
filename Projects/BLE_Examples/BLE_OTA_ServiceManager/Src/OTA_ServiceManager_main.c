
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : OTA_ServiceManager_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 18-March-2019
* Description        : Code demonstrating the Bluetooth LE OTA Service Manager application
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  OTA_ServiceManager_main.c
 * @brief This application implements a basic standalone Bluetooth LE Over The Air (OTA) firmware updgrade. 
 *        It provides the Bluetooth LE Over-The-Air Service management for handling the OTA firmware upgrade
 *        of a Bluetooth LE application which doesn't have any Bluetooth LE OTA service. 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_OTA_ServiceManager\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_OTA_ServiceManager.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Debug->Start/Stop Debug Session to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_OTA_ServiceManager\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\OTA_ServiceManager.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_OTA_ServiceManager\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  LED name  |          STEVAL-IDB011V1         |          STEVAL-IDB011V2         |          STEVAL-IDB012V1         |
--------------------------------------------------------------------------------------------------------------------------
|     DL1    |             Not Used             |             Not Used             |             Not Used             |
|     DL2    |             Not Used             |             Not Used             |             Not Used             |
|     DL3    |             Not Used             |             Not Used             |             Not Used             |
|     DL4    |             Not Used             |             Not Used             |             Not Used             |
|     U5     |  ON when OTA upgrade is ongoing  |  ON when OTA upgrade is ongoing  |  ON when OTA upgrade is ongoing  |

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

 - The OTA Service Manager is a basic application which only supports the OTA Bootloader service.
 - It provides the Bluetooth LE OTA bootloader service to any Bluetooth LE application stored at fixed base address 
   on user Flash  which doesn't include any OTA service.
   It also includes the OTA Reset Manager functionalities in order to
   transfer the control to the proper valid application, after a Bluetooth LE OTA session.
 - User is only requested to load the OTA_ServiceManager application and then build 
   any application using it with the CONFIG_OTA_SERVICE_MANAGER
   as preprocessor option. 
 - Further, for jumping to the OTA Service Manager application, the application can call the
   OTA_Jump_To_Service_Manager_Application function (i.e. just using a platform button to 
   activate such call). 

  NOTEs: 
     - Refer to Use_OTA_ServiceManager workspaces on BLE_Beacon, BLE_SerialPort and BLE_SensorDemo IAR projects for all related examples.
     

**/
   
/** @addtogroup BlueNRGLP_demonstrations_applications
* BlueNRG-LP OTA Service manager \see OTA_ServiceManager_main.c for documentation.
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
#include "OTA_ServiceManager.h"
#include "OTA_ServiceManager_config.h"
#include "ble_const.h"
#include "OTA_btl.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "rf_driver_ll_rcc.h"
#include "bluenrg_lp_evb_com.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"


/* Private typedef -----------------------------------------------------------*/
typedef  void (*pFunction)(void);

typedef struct secureBootS {
  uint32_t RESERVED[10];
  int32_t (*generate_SHA256_digest)(const uint8_t*, int32_t, uint8_t*,int32_t*);
  int32_t (*verify_RSA_precomputedR2)(const uint8_t*, uint32_t, const uint8_t*, uint32_t, const uint8_t*, const uint8_t*, const uint8_t*,uint8_t,int32_t*);
} secureBoot_type;

/* Private define ------------------------------------------------------------*/

#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define RESET_WAKE_DEEPSLEEP_REASONS 0x05

#define BLE_OTA_SERVICE_MANAGER_VERSION_STRING "1.0.0" 

#define SECURE_BOOT_MAGIC_WORD                 0xEC1CC10B
#define SECURE_BOOT_OTP_ACTIVATION_ADDR        0x10001800
#define SECURE_BOOT_OTP_R2_ADDR                0x10001808
#define SECURE_BOOT_OTP_MODULUS_2048_ADDR      0x10001908
#define SECURE_BOOT_OTP_E_65537_ADDR           0x10001A08

#define CRL_SHA256_SIZE         32
#define E_SHA256                3
#define HASH_SUCCESS            0
#define SIGNATURE_VALID         1003
#define SIGNATURE_INVALID       1004
#define FAULT_RESISTANT_PATTERN 0x3A3A3A

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/  

NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

/* Private function prototypes -----------------------------------------------*/

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

/**
* @brief  It erases destination flash erase before starting OTA upgrade session. 
* @param  None.
* @retval None.
*
* @note The API code could be subject to change in future releases.
*/
static void OTA_Erase_Flash(uint16_t startNumber, uint16_t endNumber)
{
  LL_FLASH_Erase(FLASH, LL_FLASH_TYPE_ERASE_PAGES, startNumber, endNumber-startNumber+1);
}

/**
* @brief  It checks the runtime operation type and set the related OTA tags 
*         for handling the proper jumping to the valid application. 
* @param  None
* @retval None
*
* @note The API code could be subject to change in future releases.
*/
static void OTA_Check_ServiceManager_Operation(void) 
{
  if (RAM_VR.OTAActivation  == OTA_APP_SWITCH_OP_CODE_GO_TO_OTA_SERVICE_MANAGER) //Go to OTA Service manager
  {
    /* Set Invalid valid tag x OTA Application with OTA Service Manager  for allowing jumping to OTA Service manager */
    LL_FLASH_Program(FLASH, APP_WITH_OTA_SERVICE_ADDRESS + OTA_TAG_VECTOR_TABLE_ENTRY_OFFSET, OTA_INVALID_OLD_TAG);

    /* Reset Service Manager ram location */ 
    RAM_VR.OTAActivation  = OTA_INVALID_OLD_TAG; 
  }
}

/**
* @brief  It defines the valid application address where to jump
*         by checking the OTA application validity tags for the lower and
*         higher applications
* @param  None.
* @retval appaddress: the application base address where to jump
*
* @note The API code could be subject to change in future releases.
*/
static uint32_t OTA_Check_Application_Tags_Value(void)
{
  uint32_t appAddress = 0;
  
  if (((TAG_VALUE(APP_OTA_SERVICE_ADDRESS) == OTA_SERVICE_MANAGER_TAG) && (TAG_VALUE(APP_WITH_OTA_SERVICE_ADDRESS) == OTA_IN_PROGRESS_TAG))|| /* 10 */
      ((TAG_VALUE(APP_OTA_SERVICE_ADDRESS) == OTA_SERVICE_MANAGER_TAG) && (TAG_VALUE(APP_WITH_OTA_SERVICE_ADDRESS) == OTA_INVALID_OLD_TAG))) /* 11 */ 
  {
    /* Jump to OTA Service Manager Application */
    appAddress = APP_OTA_SERVICE_ADDRESS;
  }
  else if ((TAG_VALUE(APP_OTA_SERVICE_ADDRESS) == OTA_SERVICE_MANAGER_TAG) && (TAG_VALUE(APP_WITH_OTA_SERVICE_ADDRESS) == OTA_VALID_TAG)) /* 12 */
  {
    /* Jump to Application using OTA Service Manager */
    appAddress = APP_WITH_OTA_SERVICE_ADDRESS;
  }  
  
  return appAddress;
}

/**
* @brief  It executes a secure boot validation of the application. If the device  contains a not
*         valid image, the code execution will be stopped in an infinite loop.
* @param  appAddress Application address to validate
* @retval None.
*
* @note If this function is used out of this project, the application needs to reserve two words
*       - @ address 0x200000C4 and 0x200000C8 for BLueNRG-LP device
*       - The latest two words of the RAM for BlueNRG-LPS device 
*/
static void verifyFW(uint32_t appAddress)
{
  uint32_t FW_size;
  uint32_t *FW_input;
  uint32_t *FW_sign_2048;
  uint32_t *R2;
  uint32_t *Modulus_2048;
  uint32_t *e_65537;
  uint32_t Modulus_2048_size, e_65537_size;
  uint8_t FW_input_digest[CRL_SHA256_SIZE]; /* Buffer to store the computed
                                               digest: MUST be allocated
                                               before calling the SHA256
                                               functions */
  int32_t outSize,     /* Used to store the digest's length after its computation */
  doubleCheck;        /* Used to prevent fault attacks on verification step */
  int32_t retval;
    
  secureBoot_type *secureBoot = (secureBoot_type*)SYSTEM_MEMORY_BASE;
  
  /* Disable Peripheral Radio */
  LL_APB2_DisableClock(LL_APB2_PERIPH_MRBLE);
  /* Configure 1 FLASH Wait State */
  LL_FLASH_SetWaitStates(FLASH, LL_FLASH_WAIT_STATES_1);
  /* Setup the HSI frequency to 64 MHz */
  LL_RCC_SetRC64MPLLPrescaler(LL_RCC_RC64MPLL_DIV_1);
    
  FW_size = *(uint32_t*)(appAddress + 0x18);
  FW_input = (uint32_t*)appAddress;
  FW_sign_2048 = (uint32_t*)((uint8_t*)FW_input+FW_size);
  R2 = (uint32_t*)SECURE_BOOT_OTP_R2_ADDR;
  Modulus_2048 = (uint32_t*)SECURE_BOOT_OTP_MODULUS_2048_ADDR;
  Modulus_2048_size = 256;
  e_65537 = (uint32_t*)SECURE_BOOT_OTP_E_65537_ADDR;
  e_65537_size = 4;
  
  /* -----------------------------------------------------------------------*
  * Generate digest of the customer's public key (SHA-256) -----------------*
  * ------------------------------------------------------------------------*/
  retval = secureBoot->generate_SHA256_digest((uint8_t*)FW_input,
                                              FW_size,
                                              FW_input_digest,
                                              &outSize);
  if (retval != HASH_SUCCESS) {
    while(1);
  }
  
  /* -----------------------------------------------------------------------*
  * RSA PKCS#1 v1.5 Signature verification ---------------------------------*
  * ------------------------------------------------------------------------*/
  retval = secureBoot->verify_RSA_precomputedR2((uint8_t*)Modulus_2048, Modulus_2048_size,
                                                (uint8_t*)e_65537, e_65537_size,
                                                (uint8_t*)R2,
                                                (uint8_t*)FW_sign_2048,
                                                FW_input_digest,
                                                E_SHA256,
                                                &doubleCheck);
  /* Check the return code: the following double check is provided to be
  *  resistant to fault attacks 
  */
  if ((retval != SIGNATURE_VALID) || ((retval ^ doubleCheck) != FAULT_RESISTANT_PATTERN)) {
    while(1);
  }   
}

int main(void)
{
  pFunction Jump_To_Application;
  uint32_t JumpAddress, appAddress;
  uint8_t ret;
  WakeupSourceConfig_TypeDef wakeupIO;
  PowerSaveLevels stopLevel;  
    
  /* Check Service manager RAM Location to verify if a jump to Service Manager has been set from the Application */
  OTA_Check_ServiceManager_Operation();
  
  /* Identifies the valid application where to jump based on the OTA application validity tags values placed on
  reserved vector table entry: OTA_TAG_VECTOR_TABLE_ENTRY_INDEX */
  appAddress = OTA_Check_Application_Tags_Value();
  
  /* Check if there is a valid application where to jump */
  if (appAddress == APP_WITH_OTA_SERVICE_ADDRESS)
  {
    /* Secure boot activation*/
    if (*(volatile uint32_t*)SECURE_BOOT_OTP_ACTIVATION_ADDR == SECURE_BOOT_MAGIC_WORD)
      verifyFW(appAddress);
    
    /* Jump to user application */
    JumpAddress = *(__IO uint32_t*) (appAddress + 4);
    Jump_To_Application = (pFunction) JumpAddress;
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t*) appAddress);
    Jump_To_Application();
    
    /* Infinite loop */
    while (1)
    {
    }
  }
  
  /* Here Ota Service Manager Application is started */
  
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
  
  ModulesInit(); 

  PRINTF("\r\nBlueNRG-LP BLE OTA Service Manager (version: %s)\r\n", BLE_OTA_SERVICE_MANAGER_VERSION_STRING); 
  
  /* Init OTA Service Manager Device */
  ret = OTA_ServiceManager_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("OTA_ServiceManager_DeviceInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }

  /* No Wakeup Source needed */
  wakeupIO.IO_Mask_High_polarity = 0;
  wakeupIO.IO_Mask_Low_polarity = 0;
  wakeupIO.RTC_enable = 0;
  wakeupIO.LPU_enable = 0;
  
  /* Erase the storage area from start page to end page */
  OTA_Erase_Flash(APP_WITH_OTA_SERVICE_PAGE_NUMBER_START,APP_WITH_OTA_SERVICE_PAGE_NUMBER_END); 
  
  
  while(1)
  {
    ModulesTick();
     
    /* Application tick */
    APP_Tick();
    
    /* Check if the OTA firmware upgrade session has been completed */
    if (OTA_Tick() == 1)
    {
      /* Jump to the new application */
      OTA_Jump_To_New_Application();
    }
    /* Power Save Request */
    HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, wakeupIO, &stopLevel);
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



#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
*/
