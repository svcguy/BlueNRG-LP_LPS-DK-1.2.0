
/**
  ******************************************************************************
  * @file    DTM_cmds.c
  * @author  AMS - RF Application team
  * @version V1.3.0
  * @date    08-May-2020
  * @brief   DTM specific commands to be implemented on DTM context 
  *          (not present on BLE stack) 
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
  * <h2><center>&copy; COPYRIGHT 2018 STMicroelectronics</center></h2>
  ******************************************************************************
  */
    
#include "bluenrg_lpx.h"
#include "rf_device_it.h"
#include "bluenrg_lp_api.h"
#include "DTM_cmd_db.h"
#include "ble_status.h"
#include "cmd.h"
#include "transport_layer.h"
#include "stack_user_cfg.h"
#include "adv_buff_alloc.h"
#include "adv_buff_alloc_tiny.h"
#include "link_layer.h"
#include "system_BlueNRG_LP.h"
#include "bluenrg_lp_stack.h"
#include "rf_driver_ll_gpio.h"

#include "DTM_burst.h"

#define MIN(a,b)                        (((a) < (b))? (a) : (b))

#ifdef WATCHDOG
#include "rf_driver_ll_iwdg.h"
#endif

/** @name BLE stack v3.1 stack modular configurations bitmap
  * @{
  */
#define CONTROLLER_PRIVACY_BIT                 ((uint16_t)0x0001)  /*!< Bit 0 selected */
#define SECURE_CONNECTIONS_BIT                 ((uint16_t)0x0002)  /*!< Bit 1 selected */
/* Observer (scanner) or Central role support:
   -  Observer disabled (CONTROLLER_MASTER_BIT = 0) or enabled (CONTROLLER_MASTER_BIT = 1) if CONNECTION_BIT = 0;
   -  Observer and Central disabled (CONTROLLER_MASTER_BIT = 0) or enabled (CONTROLLER_MASTER_BIT = 1) if CONNECTION_BIT = 1.
*/
#define CONTROLLER_MASTER_BIT                  ((uint16_t)0x0004)  /*!< Bit 2 selected */
#define CONTROLLER_DATA_LENGTH_EXTENSION_BIT   ((uint16_t)0x0008)  /*!< Bit 3 selected */
#define LINK_LAYER_ONLY_BIT                    ((uint16_t)0x0010)  /*!< Bit 4 selected */
#define CONTROLLER_2M_CODED_PHY_BIT            ((uint16_t)0x0020)  /*!< Bit 5 selected */
#define CONTROLLER_EXT_ADV_SCAN_BIT            ((uint16_t)0x0040)  /*!< Bit 6 selected */
#define L2CAP_COS_BIT                          ((uint16_t)0x0080)  /*!< Bit 7 selected */
#define CONTROLLER_PERIODIC_ADV_BIT            ((uint16_t)0x0100)  /*!< Bit 8 selected */
#define CONTROLLER_CTE_BIT                     ((uint16_t)0x0200)  /*!< Bit 9 selected */
#define CONTROLLER_POWER_CONTROL_BIT           ((uint16_t)0x0400)  /*!< Bit 10 selected */
/* Support to connections:
   - If CONNECTION_BIT = 0, connections are not supported;
     device is a broadcaster-only if CONTROLLER_MASTER_BIT = 0,
     or a broadcaster and observer if CONTROLLER_MASTER_BIT = 1.
   - If CONNECTION_BIT = 1, connections are supported;
     device can only act as broadcaster or peripheral if CONTROLLER_MASTER_BIT = 0,
     or any role (broadcaster, observer, peripheral, and central) if CONTROLLER_MASTER_BIT = 1.
*/
#define CONNECTION_BIT                         ((uint32_t)0x0800)  /*!< Bit 11 selected */
/**
  * @}
  */

/** @brief Link Layer Enabled or not based on LL_ONLY and BLESTACK_CONTROLLER_ONLY preprocessor options */
#ifndef BLESTACK_CONTROLLER_ONLY
#define BLESTACK_CONTROLLER_ONLY (0U)
#endif
#if defined(LL_ONLY) || (BLESTACK_CONTROLLER_ONLY == 1U)
#define LINK_LAYER_ONLY_ENABLED (1U)
#else
#define LINK_LAYER_ONLY_ENABLED (0U)
#endif

/** @brief Define BLE stack configurations variant bitmap value based on enabled BLE stack options and associated bits (LSB 5 bits) */
#define BLE_STACK_CONFIGURATIONS_VARIANT (                                                          \
    ((uint16_t)(CONTROLLER_PRIVACY_ENABLED * CONTROLLER_PRIVACY_BIT))                             | \
    ((uint16_t)(SECURE_CONNECTIONS_ENABLED * SECURE_CONNECTIONS_BIT))                             | \
    ((uint16_t)(CONTROLLER_MASTER_ENABLED * CONTROLLER_MASTER_BIT))                               | \
    ((uint16_t)(CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED * CONTROLLER_DATA_LENGTH_EXTENSION_BIT)) | \
    ((uint16_t)(LINK_LAYER_ONLY_ENABLED * LINK_LAYER_ONLY_BIT))                                   | \
    ((uint16_t)(CONTROLLER_2M_CODED_PHY_ENABLED * CONTROLLER_2M_CODED_PHY_BIT))                   | \
    ((uint16_t)(CONTROLLER_EXT_ADV_SCAN_ENABLED * CONTROLLER_EXT_ADV_SCAN_BIT))                   | \
    ((uint16_t)(L2CAP_COS_ENABLED * L2CAP_COS_BIT))                                               | \
    ((uint16_t)(CONTROLLER_PERIODIC_ADV_ENABLED * CONTROLLER_PERIODIC_ADV_BIT))                   | \
    ((uint16_t)(CONTROLLER_CTE_ENABLED * CONTROLLER_CTE_BIT))                                     | \
    ((uint16_t)(CONTROLLER_POWER_CONTROL_ENABLED * CONTROLLER_POWER_CONTROL_BIT))                 | \
    ((uint16_t)(CONNECTION_ENABLED * CONNECTION_BIT))                               \
)

tBleStatus aci_hal_updater_start(void)
{
   // For ACI_HAL_UPDATER_START, set flag to issue a updater start
   RAM_VR.BlueFlag = BLUE_FLAG_RAM_RESET;
   reset_pending = 1;
   return BLE_STATUS_SUCCESS;
}

tBleStatus aci_hal_get_firmware_details(uint8_t *DTM_version_major,
                                        uint8_t *DTM_version_minor,
                                        uint8_t *DTM_version_patch,
                                        uint8_t *DTM_variant,
                                        uint16_t *DTM_Build_Number,
                                        uint8_t *BTLE_Stack_version_major,
                                        uint8_t *BTLE_Stack_version_minor,
                                        uint8_t *BTLE_Stack_version_patch,
                                        uint8_t *BTLE_Stack_development,
                                        uint16_t *BTLE_Stack_variant,
                                        uint16_t *BTLE_Stack_Build_Number)
{

    aci_hal_get_fw_build_number(BTLE_Stack_Build_Number);
    uint8_t HCI_Version = 0;
    uint16_t HCI_Revision = 0;
    uint8_t LMP_PAL_Version = 0;
    uint16_t Manufacturer_Name = 0;
    uint16_t LMP_PAL_Subversion = 0;
    
    hci_read_local_version_information(&HCI_Version, &HCI_Revision, &LMP_PAL_Version,
                                       &Manufacturer_Name, &LMP_PAL_Subversion);
        
    *DTM_version_major  = DTM_FW_VERSION_MAJOR;
    *DTM_version_minor  = DTM_FW_VERSION_MINOR;
    *DTM_version_patch  = DTM_FW_VERSION_PATCH;
    *DTM_variant = DTM_VARIANT;
    *DTM_Build_Number = 0;
    *BTLE_Stack_version_major = HCI_Revision&0x0F;
    *BTLE_Stack_version_minor = (LMP_PAL_Subversion>>4)&0x0F;
    *BTLE_Stack_version_patch = LMP_PAL_Subversion&0x0F;
    *BTLE_Stack_development = (LMP_PAL_Subversion>>15)&0x01;
     
    /* Set the stack configurations variant bitmap value:
       first LSB 7 bits are reserved for BLE stack modular options + Link Layer only*/
    *BTLE_Stack_variant |= BLE_STACK_CONFIGURATIONS_VARIANT;
    
    return (BLE_STATUS_SUCCESS);
    
}

/**
 * @brief  This API implements the hci le transmitter test with 
 *         the capability to set the number of packets to be sent. 
 * @param  TX_Frequency: TX frequency 
 * @param  Length_Of_Test_Data: lenght of test data
 * @param  Packet_Payload: packet payload 
 * @param  Number_Of_Packets: number pf packets to be sent on test
 * @param  PHY: PHY to be used by the transmitter
 * @retval status
*/
tBleStatus aci_hal_transmitter_test_packets(uint8_t TX_Frequency,
                                            uint8_t Length_Of_Test_Data,
                                            uint8_t Packet_Payload,
                                            uint16_t Number_Of_Packets,
                                            uint8_t PHY)
{
  extern uint16_t num_packets;
  tBleStatus status; 
  
  if(Number_Of_Packets == 0)
  {
    return BLE_ERROR_INVALID_HCI_CMD_PARAMS;
  }

#ifdef CONTROLLER_2M_CODED_PHY_ENABLED
  
  status =  hci_le_enhanced_transmitter_test(TX_Frequency,
                                            Length_Of_Test_Data,
                                            Packet_Payload,
                                            PHY);
  
#else
    
  status = hci_le_transmitter_test(TX_Frequency /* 1 */,
                                   Length_Of_Test_Data /* 1 */,
                                   Packet_Payload /* 1 */);
#endif 
  
  if(status == 0x00)
  {    
    num_packets = Number_Of_Packets;
  }
  
  return status;
}

/**
 * @brief  This API implements the hci le transmitter test v2 with 
 *         the capability to set the number of packets to be sent. 
 * @param  TX_Channel: TX channel 
 * @param  Test_Data_Length: lenght of test data
 * @param  Packet_Payload: packet payload 
 * @param  Number_Of_Packets: number of packets to be sent on test
 * @param  PHY: PHY to be used by the transmitter
 * @param  CTE_Length: CTE length
 * @param  CTE_Type: CTE type
 * @param  Switching_Pattern_Length: switching pattern length
 * @param  Antenna_IDs: antenna ids
 * @param  Transmit_Power_Level: tx power level
 * @retval status
*/
tBleStatus aci_hal_transmitter_test_packets_v2(uint8_t TX_Channel,
                                               uint8_t Test_Data_Length,
                                               uint8_t Packet_Payload,
                                               uint16_t Number_Of_Packets,
                                               uint8_t PHY,
                                               uint8_t CTE_Length,
                                               uint8_t CTE_Type,
                                               uint8_t Switching_Pattern_Length,
                                               uint8_t Antenna_IDs[])
{
  extern uint16_t num_packets;
  tBleStatus status; 
  
  if(Number_Of_Packets == 0)
  {
    return BLE_ERROR_INVALID_HCI_CMD_PARAMS;
  }

  status =  hci_le_transmitter_test_v3(TX_Channel,
                                       Test_Data_Length,
                                       Packet_Payload,
                                       PHY,
                                       CTE_Length,
                                       CTE_Type,
                                       Switching_Pattern_Length,
                                       Antenna_IDs);
  
  if(status == 0x00)
  {    
    num_packets = Number_Of_Packets;
  }
  
  return status;
}


tBleStatus aci_test_tx_notification_start(uint16_t Connection_Handle, uint16_t Service_Handle, uint16_t Char_Handle, uint16_t Value_Length)
{
  return BURST_TXNotificationStart(Connection_Handle, Service_Handle, Char_Handle, Value_Length);
}

tBleStatus aci_test_tx_write_command_start(uint16_t Connection_Handle, uint16_t Attr_Handle, uint16_t Value_Length)
{
  return BURST_TXWriteCommandStart(Connection_Handle, Attr_Handle, Value_Length);
}

tBleStatus aci_test_rx_start(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint8_t Notifications_WriteCmds)
{   
  return BURST_RXStart(Connection_Handle, Attribute_Handle, Notifications_WriteCmds);
}

tBleStatus aci_test_stop(uint8_t TX_RX)
{
  switch(TX_RX){
  case 0:
    BURST_TXStop();
    break;
  case 1:
    BURST_RXStop();
    break;
  default:
    return BLE_ERROR_INVALID_HCI_CMD_PARAMS;
  }
  
  return BLE_STATUS_SUCCESS;
}

tBleStatus aci_test_report(uint32_t *TX_Packets, uint32_t *RX_Packets, uint16_t *RX_Data_Length, uint32_t *RX_Sequence_Errors)
{
  *TX_Packets = BURST_TXReport();
  *RX_Packets = BURST_RXReport(RX_Data_Length, RX_Sequence_Errors);
  
  return BLE_STATUS_SUCCESS;
}
