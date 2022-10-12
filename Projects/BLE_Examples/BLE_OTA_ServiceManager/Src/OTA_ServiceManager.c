/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : OTA_ServiceManager.c
* Author             : AMS - RF Application
* Version            : V1.0.0
* Date               : 04-April-2019
* Description        : BlueNRG-LP OTA Service Manager APIs.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "clock.h" 
#include "OTA_ServiceManager.h"
#include "ble_const.h"

#include "OTA_btl.h" 
#include <string.h>
#include "gap_profile.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint8_t set_connectable = 1;

static Advertising_Set_Parameters_t Advertising_Set_Parameters[1]; 

#define ADV_INTERVAL_MIN_MS  200
#define ADV_INTERVAL_MAX_MS  200

/* Private function prototypes -----------------------------------------------*/
/*******************************************************************************
* Function Name  : setConnectable
* Description    : Enter in connectable mode.
* Input          : None.
* Return         : None.
*******************************************************************************/
static void setConnectable(void)
{  
 
  
  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;

  aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 
}


/*******************************************************************************
* Function Name  : OTA_ServiceManager_DeviceInit
* Description    : Init the OTA Service Manager device.
* Input          : none.
* Return         : Status.
*******************************************************************************/
uint8_t OTA_ServiceManager_DeviceInit(void)
{
  uint8_t ret;
  //uint8_t name[] = {'O','T','A','S','e','r','v','i','c','e','M','g','r'};
  
  static uint8_t adv_data[] = {0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
                               14, AD_TYPE_COMPLETE_LOCAL_NAME,'O','T','A','S','e','r','v','i','c','e','M','g','r'};
  
#if 0 //TBR: use static random address for OTA Service manager
  {
    uint8_t bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};
    
    aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN,
                              bdaddr);
  }
#endif
  
  ret = aci_gatt_srv_init();    
  
  {
    uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
    
    ret  = aci_gap_init(GAP_PERIPHERAL_ROLE, 0,0x07, STATIC_RANDOM_ADDR, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }

#if 0 //TBR
 /* Set the device name */
  ret = Gap_profile_set_dev_name(0, sizeof(name), name);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF ("Error in Gap_profile_set_dev_name 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF ("Gap_profile_set_dev_name() --> SUCCESS\r\n");
  }
#endif 
  
  /* Add OTA bootloader service */
  ret = OTA_Add_Btl_Service();
  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("\r\nOTA service added successfully.\n");
  else
    PRINTF("\r\nError while adding OTA service.\n");
  
  /* 0 dBm output power */
  aci_hal_set_tx_power_level(0, 24);
  
  
  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                              ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                              (ADV_INTERVAL_MIN_MS*1000)/625, (ADV_INTERVAL_MAX_MS*1000)/625, //32, 32,
                                              ADV_CH_ALL,
                                              0,NULL,
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
  
  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  
  /* Add OTA service UUID to scan response */
  aci_gap_set_scan_response_data(0,18,BTLServiceUUID4Scan);
  
  return ret;
  
}

/*******************************************************************************
* Function Name  : APP_Tick.
* Description    : Tick to run the application state machine.
* Input          : none.
* Return         : none.
*******************************************************************************/
void APP_Tick(void)
{
  if(set_connectable) {
    setConnectable();
    set_connectable = 0;
  }
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/* This function is called when there is a LE Connection Complete event.
*/
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

/* This function is called when the peer device get disconnected.
*/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  /* Make the device connectable again. */
  set_connectable = TRUE;
  
  OTA_terminate_connection();
  
}/* end hci_disconnection_complete_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_srv_attribute_modified_event.
 * Description    : This event occurs when an attribute is modified.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_srv_attribute_modified_event(uint16_t Connection_Handle,
                                           uint16_t Attr_Handle,
                                           uint16_t Attr_Data_Length,
                                           uint8_t Attr_Data[])
{
  OTA_Write_Request_CB(Connection_Handle, Attr_Handle, Attr_Data_Length, Attr_Data);     
}

void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)
{
  if (Next_State == 0x02) /* 0x02: Connection event slave */
  {
    OTA_Radio_Activity(Next_State_SysTime);  
  }
}

void aci_gatt_srv_read_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Data_Offset)
{
  /* Handle Read operations on OTA characteristics with READ property */ 
  OTA_Read_Char(Connection_Handle,  Attribute_Handle, Data_Offset); 	
}
  
void aci_gatt_srv_write_event(uint16_t Connection_Handle,
                              uint8_t Resp_Needed,
                              uint16_t Attribute_Handle,
                              uint16_t Data_Length,
                              uint8_t Data[])
{
  uint8_t att_error = BLE_ATT_ERR_NONE;
  OTA_Write_Request_CB(Connection_Handle, Attribute_Handle, Data_Length, Data); 

  if (Resp_Needed == 1U)
  {
    aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, att_error, 0,  NULL);
  }
}

void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t Server_RX_MTU)
{
  OTA_att_exchange_mtu_resp_CB(Connection_Handle, Server_RX_MTU);
}

void hci_le_data_length_change_event(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
  OTA_data_length_change_CB(Connection_Handle);  
}


/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
*/
