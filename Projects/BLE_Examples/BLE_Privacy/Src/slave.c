/*
  ******************************************************************************
  * @file    slave.c 
  * @author  AMS - RF Application Team
  * @version V1.1.0
  * @date    16 - September - 2019
  * @brief   Application functions
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
  * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
  
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */ 
 
/* Includes-----------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "slave.h"
#include "osal.h"
#include "user_config.h"
#include "gap_profile.h"

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define MAX_NUM_BONDED_DEVICES 1 //TBR

 /*
  UUIDs:
  57f03e20-cc5c-11e8-b568-0800200c9a66
  57f03e21-cc5c-11e8-b568-0800200c9a66
  */
#define SRVC_UUID 0x66,0x9a,0x0c,0x20,0x00,0x08,0x68,0xb5,0xe8,0x11,0x5c,0xcc,0x20,0x3e,0xf0,0x57
#define CHAR_UUID 0x66,0x9a,0x0c,0x20,0x00,0x08,0x68,0xb5,0xe8,0x11,0x5c,0xcc,0x21,0x3e,0xf0,0x57

#define ADVERTISING_HANDLE 0x00


BLE_GATT_SRV_CCCD_DECLARE(Privacy_desc, 
                          NUM_LINKS, 
                          BLE_GATT_SRV_PERM_AUTHEN_WRITE | BLE_GATT_SRV_PERM_ENCRY_WRITE,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);


/* characteristic definition */
static const ble_gatt_chr_def_t privacy_chars[] = {
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_WRITE | BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_AUTHEN_READ | BLE_GATT_SRV_PERM_ENCRY_READ | BLE_GATT_SRV_PERM_AUTHEN_WRITE | BLE_GATT_SRV_PERM_ENCRY_WRITE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(Privacy_desc),
            .descr_count = 1U,
        },
    },
};

/* Privacy Service definition */
static const ble_gatt_srv_def_t privacy_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(SRVC_UUID),
   .chrs = {
       .chrs_p = (ble_gatt_chr_def_t *)privacy_chars,
       .chr_count = 1U,
   },
};

static uint16_t conn_handle; 

static Bonded_Device_Entry_t bonded_device_entry_53[MAX_NUM_BONDED_DEVICES];
static uint8_t num_of_addresses; 

static uint16_t char_handle;
static uint16_t counter = 0; 
static uint8_t slave_notification[2]; 
  
#define FIXED_PIN 123456
#define DEVICE_ID_LEN  8

//'pslave' (0x70,0x73,0x6c,0x61,0x76,0x65) is the code used at application level in order to allow slave selection from master
static uint8_t device_id[DEVICE_ID_LEN]  = {0x7,0x09,0x70,0x73,0x6c,0x61,0x76,0x65}; 

static uint8_t LE_Event_Mask[8] = {0x1F,0x02,0x00,0x00,0x00,0x00,0x00,0x00};

static volatile uint8_t update_char_value = 0; 
volatile int app_flags;


#define SLAVE_TIMER 1
static uint16_t slave_update_rate = 200;
static uint8_t slaveTimer_expired = FALSE;
static VTIMER_HandleType slaveTimerHandle;

static Advertising_Set_Parameters_t Advertising_Set_Parameters[1]; 

void SlaveUpdateTimeoutCB(void *param);

/**
 * @brief  Init a BlueNRG device
 * @param  None.
 * @retval None.
*/
void device_initialization(void)
{
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t status;

  uint8_t value_6[] = {0x99,0x39,0x22,0xE1,0x80,0x02};
  
  
  PRINTF("***************** Controller Privacy: Slave device with Fixed pin (123456)\r\n");
  
  //aci_hal_write_config_data
  //status = aci_hal_write_config_data(offset,length,value);
  status = aci_hal_write_config_data(0x00,0x06,value_6);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_hal_write_config_data() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_hal_write_config_data --> SUCCESS\r\n");
  }

  //aci_hal_set_tx_power_level
  //status = aci_hal_set_tx_power_level(en_high_power,pa_level);
  status = aci_hal_set_tx_power_level(0, 24);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_hal_set_tx_power_level() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_hal_set_tx_power_level --> SUCCESS\r\n");
  }

  status = hci_le_set_event_mask(LE_Event_Mask); //It allows to enable HCI_LE_ENHANCED_CONNECTION_COMPLETE_EVENT
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("hci_le_set_event_mask() failed:0x%02x\r\n", status);
  }else{
    PRINTF("hci_le_set_event_mask --> SUCCESS\r\n");
  }
  //aci_gatt_srv_init
  //status = aci_gatt_srv_init();
  status = aci_gatt_srv_init();
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gatt_srv_init() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gatt_srv_init --> SUCCESS\r\n");
  }

  //aci_gap_init 0x00
  status = aci_gap_init(GAP_PERIPHERAL_ROLE,0x02,DEVICE_ID_LEN,0, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_init() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gap_init --> SUCCESS\r\n");
  }
  
  /* Set the device name */
  status = Gap_profile_set_dev_name(0, DEVICE_ID_LEN, device_id);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("Error in Gap_profile_set_dev_name 0x%02x\r\n", status);
  } else {
    PRINTF("Gap_profile_set_dev_name() --> SUCCESS\r\n");
  }

  //aci_gap_clear_security_db
  //status = aci_gap_clear_security_db();
  status = aci_gap_clear_security_db();
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_clear_security_db() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gap_clear_security_db --> SUCCESS\r\n");
  }  

  //aci_gap_set_io_capability
  //status = aci_gap_set_io_capability(io_capability);
  status = aci_gap_set_io_capability(0x00);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_set_io_capability() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gap_set_io_capability --> SUCCESS\r\n");
  }

  //aci_gap_set_authentication_requirement  
  //status = aci_gap_set_authentication_requirement(bonding_mode,mitm_mode,sc_support,keypress_notification_support,min_encryption_key_size,max_encryption_key_size,use_fixed_pin,use_fixed_pin);
  status = aci_gap_set_authentication_requirement(BONDING,MITM_PROTECTION_REQUIRED,SECURITY_SUPPORT,KEYPRESS_IS_NOT_SUPPORTED,0x07,0x10,USE_FIXED_PIN_FOR_PAIRING,FIXED_PIN); 
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_set_authentication_requirement() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gap_set_authentication_requirement --> SUCCESS\r\n");
  }
  
  status = aci_gap_set_advertising_configuration(ADVERTISING_HANDLE, GAP_MODE_NON_DISCOVERABLE, 
                                                 ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                                 0x100,0x200,
                                                 ADV_CH_ALL,
                                                 0,NULL,
                                                 ADV_NO_WHITE_LIST_USE,
                                                 0, /* 0 dBm */
                                                 LE_1M_PHY, /* Primary advertising PHY */
                                                 0, /* 0 skips */
                                                 LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                                 0, /* SID */
                                                 0 /* No scan request notifications */);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_gap_set_advertising_configuration 0x%02x\r\n", status);
  } else {
    PRINTF("aci_gap_set_advertising_configuration() --> SUCCESS\r\n");
  }
  
  status = aci_gap_set_advertising_data(ADVERTISING_HANDLE, ADV_COMPLETE_DATA, 0, NULL);
  
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_gap_set_advertising_data 0x%02x\r\n", status);
  } else {
    PRINTF("aci_gap_set_advertising_data() --> SUCCESS\r\n");
  }
}


/**
 * @brief  Create a GATT DATABASE
 * @param  None.
 * @retval None.
*/
void set_database(void)
{
  uint8_t status;

  status = aci_gatt_srv_add_service((ble_gatt_srv_def_t *)&privacy_service);
  if (status != BLE_STATUS_SUCCESS)
  {
    printf("Error while adding device service: 0x%02x\r\n", status);
    return;
  }
  char_handle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&privacy_chars[0]);
  PRINTF("char_handle() :0x%02x\r\n", char_handle);
}

void set_slave_update_timer(void)
{
  uint8_t ret; 
   /* Start the slave Timer */
  
  slaveTimerHandle.callback = SlaveUpdateTimeoutCB;  
  ret = HAL_VTIMER_StartTimerMs(&slaveTimerHandle, slave_update_rate);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("HAL_VTIMER_StartTimerMs() failed; 0x%02x\r\n", ret);
  } else {
    slaveTimer_expired = FALSE;
  }
}

/**
 * @brief  Puts the device in connectable mode
 * @param  None.
 * @retval None.
*/
void set_device_discoverable(void)
{
  uint8_t status;

  status = aci_gap_set_scan_response_data(ADVERTISING_HANDLE,0x08,device_id);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_set_scan_response_data() failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gap_set_scan_response_data --> SUCCESS\r\n");
  }
  
  Advertising_Set_Parameters[0].Advertising_Handle = ADVERTISING_HANDLE;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;
  
  //enable advertising
  status = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_set_advertising_enable(NO_WHITE_LIST_USE) failed:0x%02x\r\n", status);
  }else{
    PRINTF("aci_gap_set_advertising_enable(NO_WHITE_LIST_USE) --> SUCCESS\r\n");
  }
}


/**
 * @brief  Device Demo state machine
 * @param  None.
 * @retval None.
*/
void APP_Tick(void)
{
  uint8_t status; 
  
  if APP_FLAG(DO_SLAVE_SECURITY_REQUEST)
  {
    APP_FLAG_CLEAR(DO_SLAVE_SECURITY_REQUEST);
    //aci_gap_slave_security_req
    //status = aci_gap_slave_security_req(connection_handle);
    status = aci_gap_slave_security_req(conn_handle);
    if (status != BLE_STATUS_SUCCESS) {
      PRINTF("aci_gap_slave_security_req() failed:0x%02x\r\n", status);
    }else{
      PRINTF("aci_gap_slave_security_req --> SUCCESS\r\n");
    }
  }
  else if APP_FLAG(GET_BONDED_DEVICES)
  {
    APP_FLAG_CLEAR(GET_BONDED_DEVICES);
    
    //aci_gap_get_bonded_devices
    //status = aci_gap_get_bonded_devices(&num_of_addresses, bonded_device_entry_53);
    status = aci_gap_get_bonded_devices(0, MAX_NUM_BONDED_DEVICES, &num_of_addresses, bonded_device_entry_53);
    if (status != BLE_STATUS_SUCCESS) {
      PRINTF("aci_gap_get_bonded_devices() failed:0x%02x\r\n", status);
    }
    else
    {
      PRINTF("aci_gap_get_bonded_devices --> SUCCESS; N: %d, update_char_value = %d\r\n", num_of_addresses, update_char_value);
      if (num_of_addresses>=1)
      {
        if (!update_char_value) 
        {
          APP_FLAG_SET(WAIT_SERVICES_DISCOVERY); 
        }
        else
        {
          APP_FLAG_SET(DO_NOTIFICATIONS); 
        }
      }
    }
  } 
  else if APP_FLAG(DO_TERMINATE)
  {
      APP_FLAG_CLEAR(DO_TERMINATE);
     //aci_gap_terminate
      //status = aci_gap_terminate(connection_handle,reason);
      status = aci_gap_terminate(conn_handle,0x13);
      if (status != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_terminate() failed:0x%02x\r\n", status);
      }else{
        PRINTF("aci_gap_terminate --> SUCCESS\r\n");
      }
  }                
  else if APP_FLAG(DO_CONFIGURE_WHITELIST)
  {
    APP_FLAG_CLEAR(DO_CONFIGURE_WHITELIST);
   
    status = aci_gap_configure_white_and_resolving_list(0x01|0x02);
    if (status != BLE_STATUS_SUCCESS) {
      PRINTF("aci_gap_configure_white_and_resolving_list() failed:0x%02x\r\n", status);
    }else
    {
      Bonded_Device_Entry_t bonded_device;
      uint8_t num_addresses = 1;
      uint8_t i;
        
      PRINTF("aci_gap_configure_white_and_resolving_list --> SUCCESS\r\n");
      
      /* Set device privacy mode instead of network privacy mode: we accept also
         identity addresses. */      
      for(i = 0; ; i++){
        status = aci_gap_get_bonded_devices(i, 1, &num_addresses, &bonded_device);
        if(status != BLE_STATUS_SUCCESS){
          PRINTF("aci_gap_get_bonded_devices() failed:0x%02x\r\n", status);
          break;
        }
        if(num_addresses==0){
          break;
        }
        status = hci_le_set_privacy_mode(bonded_device.Address_Type, bonded_device.Address, 1);
        if(status != BLE_STATUS_SUCCESS){
          PRINTF("hci_le_set_privacy_mode() failed:0x%02x\r\n", status);
          break;
        }
      }
      
      /* configure advertising for filtering */
      status = aci_gap_set_advertising_configuration(ADVERTISING_HANDLE, GAP_MODE_NON_DISCOVERABLE, 
                                              ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                              32,100,
                                              ADV_CH_ALL,
                                              0,NULL,
                                              ADV_WHITE_LIST_FOR_ALL,
                                              0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
      if (status != BLE_STATUS_SUCCESS) {
        PRINTF("Error in aci_gap_set_advertising_configuration 0x%02x\r\n", status);
      } else {
        PRINTF("aci_gap_set_advertising_configuration(ADV_WHITE_LIST_FOR_ALL) --> SUCCESS\r\n");
      }
      status = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 
      if (status != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_set_advertising_enable(ADV_WHITE_LIST_FOR_ALL) failed:0x%02x\r\n", status);
      }else{
        PRINTF("aci_gap_set_advertising_enable(ADV_WHITE_LIST_FOR_ALL) --> SUCCESS\r\n");
        APP_FLAG_SET(WAIT_RECONNECTION);
      }
    }
  }
  else if (APP_FLAG(DO_NOTIFICATIONS) && !APP_FLAG(TX_BUFFER_FULL))
  {

    if (slaveTimer_expired) 
    {
      slaveTimer_expired = FALSE;

      if (HAL_VTIMER_StartTimerMs(&slaveTimerHandle, slave_update_rate)!= BLE_STATUS_SUCCESS)
      {
        slaveTimer_expired = TRUE;
      }
        //aci_gatt_update_char_value_ext
      //status = aci_gatt_update_char_value_ext(conn_handle_to_notify,service_handle,char_handle,update_type,char_length,value_offset,value_length,value);
      slave_notification[1] = (uint8_t) ((counter)  >> 8);
      slave_notification[0] = (uint8_t) (counter & 0xFF); 
      status = aci_gatt_srv_notify(conn_handle, char_handle + 1, 0, 0x2, slave_notification); 
      if (status == BLE_STATUS_INSUFFICIENT_RESOURCES)
      {
        // Radio is busy (buffer full).
        APP_FLAG_SET(TX_BUFFER_FULL);
      }
      else if (status != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gatt_srv_notify() failed:0x%02x\r\n", status);
      }else{
        //PRINTF("aci_gatt_srv_notify() OK: S0: 0x%02x, S1: 0x%02x, counter: 0x%04x\r\n", slave_notification[0], slave_notification[1], counter);
        PRINTF("Send Notification. Counter: %d\r\n", counter);
        counter+=1;
      }
    }    
  }
}

/* *************** BlueNRG-LP Stack Callbacks****************/

/**
 * @brief  The LE Enhanced Connection Complete event indicates to both of the Hosts
forming the connection that a new connection has been created. Upon the
creation of the connection a Connection_Handle shall be assigned by the
Controller, and passed to the Host in this event. If the connection establishment
fails, this event shall be provided to the Host that had issued the
LE_Create_Connection command.
If this event is unmasked and LE Connection Complete event is unmasked,
only the LE Enhanced Connection Complete event is sent when a new
connection has been completed.
This event indicates to the Host that issued a LE_Create_Connection
command and received a Command Status event if the connection
establishment failed or was successful.
The Master_Clock_Accuracy parameter is only valid for a slave. On a master,
this parameter shall be set to 0x00.
 * @param  param See file bluenrg1_events.h.
 * @retval See file bluenrg1_events.h.
*/
void hci_le_enhanced_connection_complete_event(uint8_t status,uint16_t connection_handle,uint8_t role,uint8_t peer_address_type,uint8_t peer_address[6],uint8_t local_resolvable_private_address[6],uint8_t peer_resolvable_private_address[6],uint16_t conn_interval,uint16_t conn_latency,uint16_t supervision_timeout,uint8_t master_clock_accuracy)
{
  if(status != BLE_STATUS_SUCCESS)
    return;
  
  //USER ACTION IS NEEDED
  PRINTF("hci_le_enhanced_connection_complete_event --> EVENT\r\n");
  
  conn_handle = connection_handle;
  APP_FLAG_SET(DO_SLAVE_SECURITY_REQUEST); 
  APP_FLAG_SET(CONNECTED);
  
  if APP_FLAG(WAIT_RECONNECTION)
  {
    APP_FLAG_CLEAR(WAIT_RECONNECTION); 
    APP_FLAG_SET(DO_SLAVE_SECURITY_REQUEST); 
    
  }
}


/**
 * @brief  This event is generated when the slave security request is successfully sent to the master.
 * @param  param See file bluenrg1_events.h.
 * @retval retVal See file bluenrg1_events.h.
*/
void aci_gap_slave_security_initiated_event()
{
  //USER ACTION IS NEEDED
  PRINTF("aci_gap_slave_security_initiated_event --> EVENT\r\n");
}


/**
 * @brief  This event is generated by the Security manager to the application when a passkey is
required for pairing. When this event is received, the application has to respond with the
@ref ACI_GAP_PASS_KEY_RESP command.
 * @param  param See file bluenrg1_events.h.
 * @retval retVal See file bluenrg1_events.h.
*/
void aci_gap_pass_key_req_event(uint16_t connection_handle)
{
  //USER ACTION IS NEEDED
  PRINTF("aci_gap_pass_key_req_event --> EVENT\r\n");
}


/**
 * @brief  The Encryption Change event is used to indicate that the change of the encryption
mode has been completed. The Connection_Handle will be a Connection_Handle
for an ACL connection. The Encryption_Enabled event parameter
specifies the new Encryption_Enabled parameter for the Connection_Handle
specified by the Connection_Handle event parameter. This event will occur on
both devices to notify the Hosts when Encryption has changed for the specified
Connection_Handle between two devices. Note: This event shall not be generated
if encryption is paused or resumed; during a role switch, for example.
The meaning of the Encryption_Enabled parameter depends on whether the
Host has indicated support for Secure Connections in the Secure_Connections_Host_Support
parameter. When Secure_Connections_Host_Support is
'disabled' or the Connection_Handle refers to an LE link, the Controller shall
only use Encryption_Enabled values 0x00 (OFF) and 0x01 (ON).
(See Bluetooth Specification v.4.1, Vol. 2, Part E, 7.7.8)
 * @param  param See file bluenrg1_events.h.
 * @retval See file bluenrg1_events.h.
*/
void hci_encryption_change_event(uint8_t status,uint16_t connection_handle,uint8_t encryption_enabled)
{
  //USER ACTION IS NEEDED
  PRINTF("hci_encryption_change_event --> EVENT\r\n");
}


/**
 * @brief  This event is generated when the pairing process has completed successfully or a pairing
procedure timeout has occurred or the pairing has failed. This is to notify the application that
we have paired with a remote device so that it can take further actions or to notify that a
timeout has occurred so that the upper layer can decide to disconnect the link.
 * @param  param See file bluenrg1_events.h.
 * @retval retVal See file bluenrg1_events.h.
*/
void aci_gap_pairing_complete_event(uint16_t connection_handle,uint8_t status,uint8_t reason)
{
  //USER ACTION IS NEEDED
  if (status == 0)
  {
    PRINTF("aci_gap_pairing_complete_event --> SUCCESS\r\n");
  }
  else
  {
    PRINTF("aci_gap_pairing_complete_event --> Failure: status = 0x%02x, reason = 0x%02x\r\n", status, reason);
  }
  
  APP_FLAG_SET(GET_BONDED_DEVICES); 
}


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
  //USER ACTION IS NEEDED
  //PRINTF("aci_gatt_srv_attribute_modified_event --> EVENT; attr_handle = 0x%02x, attr_data_length: %d, attr_data[0] : %d, attr_data[1] : %d,\r\n",attr_handle,attr_data_length,attr_data[0],attr_data[1]);
  
  /* Characteristic notification has been enabled */
   if APP_FLAG(WAIT_SERVICES_DISCOVERY)
  {
    APP_FLAG_CLEAR(WAIT_SERVICES_DISCOVERY);
    APP_FLAG_SET(DO_TERMINATE);
    update_char_value = 1; 
  }
  
}

void aci_gatt_srv_write_event(uint16_t Connection_Handle, uint8_t Resp_Needed, uint16_t Attribute_Handle, uint16_t Data_Length, uint8_t Data[])
{
  uint8_t att_error = BLE_ATT_ERR_NONE;
  
  if (Resp_Needed == 1U)
  {
    aci_gatt_srv_resp(Connection_Handle, 0, att_error, 0, NULL);
  }
}

void aci_gatt_srv_read_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Data_Offset)
{
  uint8_t att_err;
  uint8_t read_buff[2];
  
  read_buff[1] = (uint8_t) ((counter)  >> 8);
  read_buff[0] = (uint8_t) (counter & 0xFF); 
  att_err = BLE_ATT_ERR_NONE;
  if(Attribute_Handle == char_handle + 1)
  {
    aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, att_err, 2, read_buff);
  }

}

/**
 * @brief  The Disconnection Complete event occurs when a connection is terminated.
The status parameter indicates if the disconnection was successful or not. The
reason parameter indicates the reason for the disconnection if the disconnection
was successful. If the disconnection was not successful, the value of the
reason parameter can be ignored by the Host. For example, this can be the
case if the Host has issued the Disconnect command and there was a parameter
error, or the command was not presently allowed, or a Connection_Handle
that didn't correspond to a connection was given.
 * @param  param See file bluenrg1_events.h.
 * @retval See file bluenrg1_events.h.
*/
void hci_disconnection_complete_event(uint8_t status,uint16_t connection_handle,uint8_t reason)
{
  //USER ACTION IS NEEDED
  PRINTF("hci_disconnection_complete_event --> EVENT, reason = 0x%02x\r\n",reason);
  APP_FLAG_CLEAR(CONNECTED);
  APP_FLAG_CLEAR(DO_NOTIFICATIONS); 
  if (num_of_addresses>=1)
  {
    APP_FLAG_SET(DO_CONFIGURE_WHITELIST);
  }
}

void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{       
  /* It allows to notify when at least 2 GATT TX buffers are available */
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
} 

void SlaveUpdateTimeoutCB(void *param)
{
  slaveTimer_expired = TRUE;
}

/** \endcond 
*/
