/**
  ******************************************************************************
  * @file    OTA_btl.h
  * @author  AMS - RF Application team
  * @version V1.1.0
  * @date    14-March-2022
  * @brief   Bluetooth LE Over The Air (OTA) FW upgrade header file
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2022 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BTL_H
#define __BTL_H

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lpx.h"
#include "rf_driver_ll_flash.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define PAGE_SIZE_ROUND(size) (FLASH_PAGE_SIZE*(((size)+FLASH_PAGE_SIZE-1)/FLASH_PAGE_SIZE))
#define PAGE_SIZE_TRUNC(size) (FLASH_PAGE_SIZE*((size)/FLASH_PAGE_SIZE))

/* Exported define -----------------------------------------------------------*/

/* BlueNRG-x Flash memory layout for OTA */

/** @brief Flash erase basic unit */
#define FLASH_PAGE_SIZE LL_FLASH_PAGE_SIZE 
/** @brief Reset manager size: it defines the lower application base address (and
  * the OTA Service Manger base address) 
  */
#ifndef RESET_MANAGER_SIZE
#define RESET_MANAGER_SIZE         PAGE_SIZE_ROUND(LL_FLASH_PAGE_SIZE) 
#endif

/**
 * @brief Set the MAX ATT_MTU size used for  OTA FW data packet transfer
 */
#if defined(CONFIG_SW_OTA_DATA_LENGTH_EXT) 
  /** @brief MAX ATT_MTU size (< 251; max LL data packet length) allowed from both OTA Client (i.e. DTM application) and Server sides.
    * It is  used for OTA transfer with data length extension feature on BlueNRG-LP, BLE stack v3.x or later.
    */ 
  #define OTA_ATT_MTU_SIZE_CONF     (BLE_STACK_DEFAULT_MAX_ATT_MTU)              //i.e. OTA Client, DTM framework MAX_ATT_MTU value (DTM_config.h)              
#else 
  /**  @brief MAX ATT_MTU size with no data length extension support */   
  #define OTA_ATT_MTU_SIZE_CONF     (BLE_STACK_DEFAULT_ATT_MTU)  // DEFAULT_ATT_MTU size = 23 bytes   
#endif

#define NVM_SIZE                   PAGE_SIZE_ROUND(4 * 1024)

#define RESET_MANAGER_OFFSET       (0x0)
#define LOWER_APP_OFFSET           (RESET_MANAGER_OFFSET + RESET_MANAGER_SIZE)
#define LOWER_APP_SIZE             PAGE_SIZE_TRUNC((_MEMORY_FLASH_SIZE_-RESET_MANAGER_SIZE-NVM_SIZE)/2)
#define HIGHER_APP_OFFSET          (LOWER_APP_OFFSET + LOWER_APP_SIZE)
#define HIGHER_APP_SIZE            PAGE_SIZE_TRUNC((_MEMORY_FLASH_SIZE_-RESET_MANAGER_SIZE-NVM_SIZE)/2)

#define SERVICE_MANAGER_OFFSET     (0x0)

#define SERVICE_MANAGER_SIZE       PAGE_SIZE_ROUND(94* 1024) /* BlueNRG-LP,BlueNRG-LPS BLE stack v3.1x with modular approach */

#define SM_APP_OFFSET              (SERVICE_MANAGER_OFFSET + SERVICE_MANAGER_SIZE)
#define SM_APP_SIZE                PAGE_SIZE_TRUNC((_MEMORY_FLASH_SIZE_-SERVICE_MANAGER_SIZE-NVM_SIZE))


/** @brief OTA Lower application  page numbers start, end, start address and end boundary: don't change them */
#define APP_LOWER_ADDRESS                        (_MEMORY_FLASH_BEGIN_ + LOWER_APP_OFFSET)
#define APP_LOWER_ADDRESS_END                    (APP_LOWER_ADDRESS + LOWER_APP_SIZE - 1)
#define OTA_LOWER_APPLICATION_PAGE_NUMBER_START  (LOWER_APP_OFFSET/FLASH_PAGE_SIZE) //1
#define OTA_LOWER_APPLICATION_PAGE_NUMBER_END    (OTA_LOWER_APPLICATION_PAGE_NUMBER_START + (LOWER_APP_SIZE/FLASH_PAGE_SIZE) - 1)//38

/** @brief OTA Higher application  page numbers start, end, start address and end boundary: don't change them */
#define APP_HIGHER_ADDRESS                       (_MEMORY_FLASH_BEGIN_ + HIGHER_APP_OFFSET) 
#define APP_HIGHER_ADDRESS_END                   (APP_HIGHER_ADDRESS + HIGHER_APP_SIZE - 1) 
#define OTA_HIGHER_APPLICATION_PAGE_NUMBER_START (HIGHER_APP_OFFSET/FLASH_PAGE_SIZE) //39: 0x13800 Higher application starts at sector 39 and ends at sector 76 
#define OTA_HIGHER_APPLICATION_PAGE_NUMBER_END   (OTA_HIGHER_APPLICATION_PAGE_NUMBER_START + (HIGHER_APP_SIZE/FLASH_PAGE_SIZE) - 1) //76  

/** @brief OTA Service Manager application address,page numbers start, end: don't change them */
#define APP_OTA_SERVICE_ADDRESS                  (_MEMORY_FLASH_BEGIN_ + SERVICE_MANAGER_OFFSET) 
#define APP_WITH_OTA_SERVICE_PAGE_NUMBER_START   (SM_APP_OFFSET/FLASH_PAGE_SIZE) //29
#define APP_WITH_OTA_SERVICE_PAGE_NUMBER_END     (APP_WITH_OTA_SERVICE_PAGE_NUMBER_START + (SM_APP_SIZE/FLASH_PAGE_SIZE) - 1) //77


/** @brief OTA application with OTA Service manager address: don't change them */
#define APP_WITH_OTA_SERVICE_ADDRESS             (_MEMORY_FLASH_BEGIN_ + SM_APP_OFFSET)

#define APP_WITH_OTA_SERVICE_ADDRESS_END         (APP_WITH_OTA_SERVICE_ADDRESS + SM_APP_SIZE - 1)

/** @brief  OTA application validity tags: don't change them.
  */
#define OTA_NO_OPERATION                                 (0x11)
#define OTA_APP_SWITCH_OP_CODE_NO_OPERATION              (0xb0014211)
#define OTA_APP_SWITCH_OP_CODE_GO_TO_LOWER_APP           (OTA_APP_SWITCH_OP_CODE_NO_OPERATION + (OTA_NO_OPERATION*2)) //0xb0014233
#define OTA_APP_SWITCH_OP_CODE_GO_TO_HIGHER_APP          (OTA_APP_SWITCH_OP_CODE_NO_OPERATION + (OTA_NO_OPERATION*3)) //0xb0014244
#define OTA_APP_SWITCH_OP_CODE_GO_TO_OTA_SERVICE_MANAGER (OTA_APP_SWITCH_OP_CODE_NO_OPERATION + (OTA_NO_OPERATION*4)) 
#define OTA_APP_SWITCH_OP_CODE_GO_TO_NEW_APP             (OTA_APP_SWITCH_OP_CODE_NO_OPERATION + (OTA_NO_OPERATION*5)) 

/** @brief  Compiler/Linker options to be used on project preprocessor options based on 
            specific application scenario: don't change them */

/** @brief Options for Higher Application with OTA:
  *        Compiler option: CONFIG_OTA_HIGHER
  *        Linker Option:   CONFIG_OTA_HIGHER=1
  */
#if defined (CONFIG_OTA_HIGHER) /* linker option */
    /* BLE Application with OTA Service, based at higher FLASH address */

    #define ST_OTA_FIRMWARE_UPGRADE_SUPPORT 1 

    /** @brief  Free space x lower application starts from  Reset Manager size */
    #define OTA_FREE_SPACE_RANGE_START      __REV(APP_LOWER_ADDRESS)
    #define OTA_FREE_SPACE_RANGE_END        __REV(APP_LOWER_ADDRESS_END)

    #define OTA_OP_CODE OTA_APP_SWITCH_OP_CODE_GO_TO_LOWER_APP // Go to Lower App when OTA completes with success

/** @brief Options for Lower Application with OTA:
  *        Compiler option: CONFIG_OTA_LOWER
  *        Linker Option:   CONFIG_OTA_LOWER=1
  */
#elif defined (CONFIG_OTA_LOWER)  /* linker option */
    /* BLE Application with OTA Service, based at lower FLASH address */

    #define ST_OTA_FIRMWARE_UPGRADE_SUPPORT 1

    /** @brief  Free space for higher application starts from  Higher Application default base */
    #define OTA_FREE_SPACE_RANGE_START      __REV(APP_HIGHER_ADDRESS)
    #define OTA_FREE_SPACE_RANGE_END        __REV(APP_HIGHER_ADDRESS_END)

    #define OTA_OP_CODE OTA_APP_SWITCH_OP_CODE_GO_TO_HIGHER_APP// Go to Higher App when OTA completes with success
    
/** @brief Options for OTA Service Manager:
  *        Compiler option: CONFIG_OTA_SERVICE_MANAGER
*        Linker option: MEMORY_FLASH_APP_SIZE=0xF800
  */
#elif defined (CONFIG_OTA_SERVICE_MANAGER)
    /* OTA Service manager, based at  FLASH base address */

    /** @brief  Free space x application starts from  OTA_ServiceManager_Size */  
    #define OTA_FREE_SPACE_RANGE_START      __REV(APP_WITH_OTA_SERVICE_ADDRESS)
    #define OTA_FREE_SPACE_RANGE_END        __REV(APP_WITH_OTA_SERVICE_ADDRESS_END)
    #define OTA_OP_CODE OTA_APP_SWITCH_OP_CODE_GO_TO_NEW_APP 

/** @brief Options for Application which can use OTA Service Manager:
  *        Compiler option: CONFIG_OTA_USE_SERVICE_MANAGER
  *        Linker Option:   CONFIG_OTA_USE_SERVICE_MANAGER=1
  */
#elif defined (CONFIG_OTA_USE_SERVICE_MANAGER) /* linker option */
    /* BLE Application (with no OTA service) which uses the OTA Service Manager */
    /* BLE application is based at new higher FLASH address */
    
    #define OTA_FREE_SPACE_RANGE_START      0 //Not used in this context TBR
    #define OTA_FREE_SPACE_RANGE_END        0 //Not used in this context TBR

    #define OTA_OP_CODE OTA_APP_SWITCH_OP_CODE_GO_TO_OTA_SERVICE_MANAGER //Not used in this context TBR

#else 
/* Nothing to do: no OTA Service is supported; No OTA Service Manager is used */
#endif 

/* ************************************************************************************************ */

/** @brief OTA Service Manager defines values magic location in RAM: don't change it */
#define OTA_SERVICE_MANAGER_RAM_LOCATION (0x20000004)   
/** @brief OTA Service Manager utility to set the magic location value: don't change it */
#define OTA_SET_SERVICE_MANAGER_RAM_LOCATION (*(uint32_t *)(OTA_SERVICE_MANAGER_RAM_LOCATION))

/** @brief  Vector table entry used to register OTA application validity tag*/
#define OTA_TAG_VECTOR_TABLE_ENTRY_INDEX  (4)
/** @brief  Address offset for vector table entry used to register OTA application validity tag */
#define OTA_TAG_VECTOR_TABLE_ENTRY_OFFSET (OTA_TAG_VECTOR_TABLE_ENTRY_INDEX * 4)

/** @brief Application flag values to register application
  * validity on OTA_TAG_VECTOR_TABLE_ENTRY_INDEX as consequence of an OTA upgrade:
  */

/** @brief OTA in progress tag: it is sets on vector table during OTA upgrade */
#define OTA_IN_PROGRESS_TAG      (0xFFFFFFFF)

/** @brief  OTA  invalid, old tag: it tags old application as invalid/old 
  * (OTA upgrade done and jump to new application) 
  */
#define OTA_INVALID_OLD_TAG      (0x00000000)

/** @brief  OTA valid tag: it tags new application as valid 
  * (It is done after a OTA upgrade process is completed with success:  
  * as consequence of a SW reset to OTA Reset Manager) */
#define OTA_VALID_TAG            (0xAA5555AA)

/** @brief  OTA Service Manager valid tag: It tags OTA Service manager as valid
  */
#define OTA_SERVICE_MANAGER_TAG  (0xAABBCCDD)

/* Exported Variables  --------------------------------------------------------*/
/** @brief OTA Service UUID */
extern uint8_t BTLServiceUUID4Scan[];

/* Exported Functions  --------------------------------------------------------*/

/** 
 * @brief This function handles the OTA bootloader updgrade. 
 * It is called on the aci_gatt_srv_attribute_modified_event() callback context for handling the
 * the specific characteristic wirte coming from the OTA Client.
 * 
 * @param connection_handle Handle of the connection.
 * @param attr_handle Handle of the OTA attribute that was modified.
 * @param data_length Length of att_data in octets
 * @param att_data    The modified value
 *
 * @retval None
 */
void OTA_Write_Request_CB(uint16_t connection_handle, 
                          uint16_t attr_handle,
                          uint8_t data_length,
                          uint8_t *att_data);

/**
 * @brief  Add the 'OTABootloader' service.
 * @retval Value indicating success or error code.
 *
 * @note The API code could be subject to change in future releases.
 */
uint8_t OTA_Add_Btl_Service(void);

/**
 * @brief  It returns the OTA upgrade fw status
 * @retval 1 if OTA upgrade session has been completed; 0 otherwise
 *
 * @note The API code could be subject to change in future releases.
 */
uint8_t OTA_Tick(void);

/**
 * @brief  It jumps to the new upgraded application
 * @retval None
 *
 * @note The API code could be subject to change in future releases.
 */
void OTA_Jump_To_New_Application(void);

/**
 * @brief  It jumps to the OTA Service Manager application
 * @retval None
 *
 * @note The API code could be subject to change in future releases.
 */
void OTA_Jump_To_Service_Manager_Application(void);

/**
 * @brief  It tracks the timing for next radio activity slot when connected as slave
 * @param  Next_State_SysTime time of next radio activity slot 
 * @retval None
 *
 * @note The API code could be subject to change in future releases.
 */
void OTA_Radio_Activity(uint32_t Next_State_SysTime); 

/**
 * @brief  It just informs OTA manager of disconnection complete event in order to
 *         jump to new application
 * @retval None
 *
 * @note The API code could be subject to change in future releases.
 */
void OTA_terminate_connection(void); 

/**
 * @brief  Function to be called when an aci_att_exchange_mtu_resp_event is
 *         received.
 * @param Connection_Handle Connection handle related to the response
 * @param Server_RX_MTU ATT_MTU value agreed between server and client
 *
 * @note The API code could be subject to change in future releases.
 */
void OTA_att_exchange_mtu_resp_CB(uint16_t Connection_Handle,
                                  uint16_t Att_MTU);

/**
 * @brief  Function to be called when an hci_le_data_length_change_event is
 *         received.
 * @param Connection_Handle Connection_Handle that identifies the
 *        connection.
 *
 * @note The API code could be subject to change in future releases.
 */
void OTA_data_length_change_CB(uint16_t Connection_Handle);

/**
 * @brief  Function to be called when an aci_gatt_srv_read_event is
 *         received.
 * @param Connection_Handle Handle identifying the connection where the read
 *        operation has been received.
 * @param Attribute_Handle Handle of the attribute to read.
 * @param Data_Offset Offset from which the peer is requesting the attribute
 *        value.
 *
 * @note The API code could be subject to change in future releases.
 */
void OTA_Read_Char(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Data_Offset); 
#endif /* __BTL_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
