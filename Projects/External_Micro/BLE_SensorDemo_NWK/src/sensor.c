/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
 * File Name          : sensor.c
 * @author            : RF Application Team - AMG
 * Version            : V1.1.0
 * Date               : 12-November-2018
 * Description        : Sensor init and sensor state machines 
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "bluenrg_lp_aci.h"
#include "bluenrg_lp_hci_le.h"
#include "gatt_db.h"
#include "hal_types.h"
#include "SDK_EVAL_Led.h"
#include "user_timer.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define UPDATE_CONN_PARAM 0 // Can be set to 1 only when no low power mode is used 
#define  ADV_INTERVAL_MIN_MS  1000
#define  ADV_INTERVAL_MAX_MS  1200

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t set_connectable = 1;
uint16_t connection_handle = 0;

BOOL sensor_board = FALSE; // It is True if sensor boad has been detected

int connected = FALSE;
#if UPDATE_CONN_PARAM
int l2cap_request_sent = FALSE;
struct timer l2cap_req_timer;
#endif

volatile uint8_t request_free_fall_notify = FALSE; 
static Advertising_Set_Parameters_t Advertising_Set_Parameters[1];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/*******************************************************************************
 * Function Name  : Sensor_DeviceInit.
 * Description    : Init the device sensors.
 * Input          : None.
 * Return         : Status.
 *******************************************************************************/
uint8_t Sensor_DeviceInit(void)
{
  uint8_t bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t device_name[] = {'B', 'l', 'u', 'e', 'N', 'R', 'G'};
  static uint8_t adv_data[] = {0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
                              8, AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G'};

  /* Set the device public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);  
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_hal_write_config_data() failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
    PRINTF("aci_hal_write_config_data() --> SUCCESS\r\n");
  
  /* Set the TX power 0 dBm */
  ret = aci_hal_set_tx_power_level(0, 24);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_hal_set_tx_power_level() 0x%04x\r\n", ret);
    return ret;
  }
  else
    PRINTF("aci_hal_set_tx_power_level() --> SUCCESS\r\n");
  
  /* GATT Init */
  ret = aci_gatt_srv_init();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gatt_srv_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
    PRINTF("aci_gatt_srv_init() --> SUCCESS\r\n");
  
  /* GAP Init */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, STATIC_RANDOM_ADDR, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
    PRINTF("aci_gap_init() --> SUCCESS\r\n");
 
  /* Update device name */
  aci_gatt_srv_write_handle_value_nwk(dev_name_char_handle + 1, 0, sizeof(device_name), device_name);
  
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("Update name failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
  {
    PRINTF("aci_gatt_srv_write_handle_value_nwk() --> SUCCESS\r\n");
  }
  
  ret = aci_gap_set_authentication_requirement(BONDING,
                                               MITM_PROTECTION_REQUIRED,
                                               SC_IS_NOT_SUPPORTED,
                                               KEYPRESS_IS_NOT_SUPPORTED,
                                               7, 
                                               16,
                                               USE_FIXED_PIN_FOR_PAIRING,
                                               123456);
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_set_authentication_requirement()failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
    PRINTF("aci_gap_set_authentication_requirement() --> SUCCESS\r\n");
  
   ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                              ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                              (ADV_INTERVAL_MIN_MS*1000)/625, (ADV_INTERVAL_MAX_MS*1000)/625,
                                              ADV_CH_ALL,
                                              0,NULL,
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);  
  printf("Advertising configuration 0x%02X\n", ret);
  
  ret = aci_gap_set_advertising_data_nwk(0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  
  printf("Set advertising data 0x%02X\n", ret);  
  
  PRINTF("BLE Stack Initialized with SUCCESS\n");

  /* Add ACC service and Characteristics */
  ret = Add_Acc_Service();
  if(ret == BLE_STATUS_SUCCESS) {
    PRINTF("Acceleration service added successfully.\n");
  }
  else {
    PRINTF("Error while adding Acceleration service: 0x%02x\r\n", ret);
    return ret;
  }

  /* Add Environmental Sensor Service */
  ret = Add_Environmental_Sensor_Service();
  if(ret == BLE_STATUS_SUCCESS) {
    PRINTF("Environmental service added successfully.\n");
  }
  else {
    PRINTF("Error while adding Environmental service: 0x%04x\r\n", ret);
    return ret;
  }

  /* Start the Sensor Timer */
  Init_User_Timer();
  Start_User_Timer();

  return BLE_STATUS_SUCCESS;
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

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  hci_le_set_scan_response_data(18,BTLServiceUUID4Scan); 
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 
  PRINTF("Set General Discoverable Mode.\n");
  
  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;
  
  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 
  
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("aci_gap_set_advertising_enable() failed: 0x%02x\r\n",ret);
    SdkEvalLedOn();
  }
  else
    PRINTF("aci_gap_set_advertising_enable() --> SUCCESS\r\n");
}

/*******************************************************************************
 * Function Name  : APP_Tick.
 * Description    : Sensor Demo state machine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void APP_Tick(void)
{
  /* Make the device discoverable */
  if(set_connectable) {
    Set_DeviceConnectable();
    set_connectable = FALSE;
  }

#if UPDATE_CONN_PARAM      
  /* Connection parameter update request */
  if(connected && !l2cap_request_sent && Timer_Expired(&l2cap_req_timer)){
    ret = aci_l2cap_connection_parameter_update_req(connection_handle, 9, 20, 0, 600); //24, 24
    l2cap_request_sent = TRUE;
  }
#endif
    
  /*  Update sensor value */
  if (user_timer_expired) {
    user_timer_expired = FALSE;
    if(connected) {
      AxesRaw_t acc_data;
      
      /* Activity Led */
      SdkEvalLedToggle();

      /* Get Acceleration data */
      if (GetAccAxesRaw(&acc_data) == 0) {
        Acc_Update(&acc_data);
      }
        
      /* Get free fall status */
      GetFreeFallStatus();
    }
  }

  /* Free fall notification */
  if(request_free_fall_notify == TRUE) {
    request_free_fall_notify = FALSE;
    Free_Fall_Notify();
  }
}

/* ***************** BlueNRG-LP Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg_lp_events.h
 * Output         : See file bluenrg_lp_events.h
 * Return         : See file bluenrg_lp_events.h
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
  
  connected = TRUE;
  connection_handle = Connection_Handle;
  
#if UPDATE_CONN_PARAM    
  l2cap_request_sent = FALSE;
  Timer_Set(&l2cap_req_timer, CLOCK_SECOND*2);
#endif
    
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
 * Input          : See file bluenrg_lp_events.h
 * Output         : See file bluenrg_lp_events.h
 * Return         : See file bluenrg_lp_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  connected = FALSE;
  user_timer_expired = 0;
  /* Make the device connectable again. */
  set_connectable = TRUE;
  connection_handle =0;
  PRINTF("Disconnected\n");
  
  SdkEvalLedOn();//activity led 
}/* end hci_disconnection_complete_event() */

