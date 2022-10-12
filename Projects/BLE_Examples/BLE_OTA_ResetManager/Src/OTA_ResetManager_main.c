
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : OTA_ResetManager_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 18-March-2019
* Description        : Code demonstrating the Bluetooth LE OTA Reset Manager application
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  OTA_ResetManager_main.c
 * @brief This application implements the OTA Reset Manager which, at reset,  
 *        passes control to the latest valid Bluetooth LE application updated through the 
 *        Bluetooth LE Over-The-Air (OTA) Service.
 * 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_OTA_ResetManager\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_OTA_ResetManager.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Debug->Start/Stop Debug Session  to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_OTA_ResetManager\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\BLE_OTA_ResetManager.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Project\\BLE_Examples\\BLE_OTA_ResetManager\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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

  - The OTA Reset Manager is a basic application which is is stored at BlueNRG-LP FLASH base address (0x10040000) and 
    it allows to transfer of control towards the new upgraded application every time we reset.
  - The new application has to add the OTA service and related characteristics defined on 
    files OTA_btl.c.
  - At device reset, the reset manager will take care of jumping to the location of the last image 
    that was successfully loaded by the OTA bootloader.

NOTEs
  - Before downloading the OTA Reset Manager performs a device Mass Erase of the selected BlueNRG-LP device (use IAR, Project, Download, Erase Memory). Then, open the IAR project related to a Lower Application with OTA Service and download it on the selected device. At this stage, the BlueNRG-LP device is ready for performing OTA upgrades.
  - Refer to BLE_SerialPort and BLE_SensorDemo projects for related OTA update examples (Lower and Higher Applications with OTA service configurations).
  - On BlueNRG-LP, Bluetooth LE stack v3.0 or later, OTA FW upgrade supports the data length extended capability. User is requested to add the OTA_EXTENDED_PACKET_LEN=1 option and to use a Bluetooth LE stack configuration ioption supporting this capability (BLE_STACK_CONFIGURATION=BLE_OTA_BASIC_CONFIGURATION or BLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION).

**/
   
/** @addtogroup BlueNRGLP_demonstrations_applications
* BlueNRG-LP OTA Reset manager \see OTA_ResetManager_main.c for documentation.
*
*@{
*/

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
*/
/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lpx.h"
#include "system_BlueNRG_LP.h"
#include "bluenrg_lp_api.h"
#include "OTA_btl.h"
#include "rf_driver_ll_flash.h"
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_bus.h"


/* Defines -------------------------------------------------------------------*/
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

#define RESET_WAKE_DEEPSLEEP_REASONS 0x05

/** @brief Get specific application tag value stored at vector table index 
* OTA_TAG_VECTOR_TABLE_ENTRY_OFFSET
*/
#define TAG_VALUE(x)     (* ((volatile uint32_t*) ((x) + OTA_TAG_VECTOR_TABLE_ENTRY_OFFSET)))

/* Typedef -------------------------------------------------------------------*/
typedef struct secureBootS {
  uint32_t RESERVED[10];
  int32_t (*generate_SHA256_digest)(const uint8_t*, int32_t, uint8_t*,int32_t*);
  int32_t (*verify_RSA_precomputedR2)(const uint8_t*, uint32_t, const uint8_t*, uint32_t, const uint8_t*, const uint8_t*, const uint8_t*,uint8_t,int32_t*);
} secureBoot_type;

typedef  void (*pFunction)(void);

/* Private Functions ---------------------------------------------------------*/
/**
* @brief  It check if  flash storage area has to be erased or not
* @param  None.
* @retval Status: 1 (erase flash); 0 (don't erase flash).
*
* @note The API code could be subject to change in future releases.
*/
static uint8_t OTA_Check_Storage_Area(uint32_t start_address, uint32_t end_address)
{
  volatile uint32_t *address; 
  uint32_t i; 
  
  for(i=start_address;i<end_address; i = i +4)
  { 
    address = (volatile uint32_t *) i;
    if (*address != 0xFFFFFFFF)
      return 1; /* do flash erase */
  }
  
  return 0; /* no flash erase is required */
}

/**
* @brief  It erases the new flash storage area. 
* @param  None.
* @retval None.
*
* @note The API code could be subject to change in future releases.
*/
static void OTA_Erase_Storage_Area(uint16_t startPageNumber, uint16_t endPageNumber)
{
  LL_FLASH_Erase(FLASH, LL_FLASH_TYPE_ERASE_PAGES, (startPageNumber), (endPageNumber-startPageNumber+1));
}

/**
* @brief  It defines the valid application address where to jump
*         by checking the OTA application validity tags for the lower and
*         higher applications
* @param  None.
* @retval None.
*
* @note The API code could be subject to change in future releases.
*/
static uint32_t OTA_Check_Application_Tags_Value(void)
{
  uint32_t appAddress = 0;
  if ( ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_INVALID_OLD_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_INVALID_OLD_TAG))   || /* 1 */
      ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_VALID_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_INVALID_OLD_TAG))         || /* 2 */
        ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_VALID_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_VALID_TAG))               || /* 4 */
          ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_INVALID_OLD_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_IN_PROGRESS_TAG))   || /* 8 */
            ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_VALID_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_IN_PROGRESS_TAG)))           /* 9 */  
  {
    /* Jump to Lower Application */
    appAddress = APP_LOWER_ADDRESS;
    
    if (OTA_Check_Storage_Area(APP_HIGHER_ADDRESS,APP_HIGHER_ADDRESS_END))
    {
      /* Erase OLD Higher application storage area */
      OTA_Erase_Storage_Area(OTA_HIGHER_APPLICATION_PAGE_NUMBER_START, OTA_HIGHER_APPLICATION_PAGE_NUMBER_END); 
     
    }
  }
  else if ( ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_INVALID_OLD_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_VALID_TAG))       || /* 3 */
           ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_IN_PROGRESS_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_INVALID_OLD_TAG)) || /* 6 */
             ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_IN_PROGRESS_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_VALID_TAG)))         /* 7 */     
  {
    /* Jump to Higher Application */
    appAddress = APP_HIGHER_ADDRESS;
    
    if (OTA_Check_Storage_Area(APP_LOWER_ADDRESS,APP_LOWER_ADDRESS_END))
    { 
      /* Erase OLD Lower application storage area */
      OTA_Erase_Storage_Area(OTA_LOWER_APPLICATION_PAGE_NUMBER_START, OTA_LOWER_APPLICATION_PAGE_NUMBER_END); 
    }
  }
  else if ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_IN_PROGRESS_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_IN_PROGRESS_TAG))   /* 5 */
  {
    /* 5: Is it possible? No. What to do?*/
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

/**
* @brief  OTA Reset Manager main function
* @param  None.
* @retval None.
*
* @note The code could be subject to change in future releases.
*/
int main(void) 
{
  pFunction Jump_To_Application;
  uint32_t JumpAddress, appAddress;
  /* Identifies the valid application where to jump based on the OTA application validity tags values placed on
  reserved vector table entry: OTA_TAG_VECTOR_TABLE_ENTRY_INDEX */
  appAddress = OTA_Check_Application_Tags_Value();
  
  if (appAddress == 0) {
    /* This case indicates that no valid application is present and this normally should not happen */
    while (1);
  }
  
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
/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
*/
