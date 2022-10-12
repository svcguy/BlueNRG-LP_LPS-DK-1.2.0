/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : BLE_Security_Peripheral.c
* Author             : AMS RF Application Team
* Version            : V1.0.0
* Date               : 12-March-2019
* Description        : This file handles the Peripheral device functionalities
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"

#include "app_state.h"
#include "osal.h"
#include "gatt_db.h"
#include "BLE_Security_Peripheral.h"
#include "gap_profile.h"


/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define MAX_NUM_BONDED_DEVICES 3

/* Private variables ---------------------------------------------------------*/

uint8_t Connected_Address_Type;
uint8_t Connected_Address[6];
uint8_t connInfo[20];
volatile int app_flags = SET_CONNECTABLE;
volatile uint16_t connection_handle = 0;


/* UUIDs */
UUID_t UUID_Tx;
UUID_t UUID_Rx;

/* Peripheral security key (default value is PERIPHERAL_SECURITY_KEY if not randomly generated) */
static uint32_t Peripheral_Pass_Key = PERIPHERAL_SECURITY_KEY;  
static uint16_t update_rate = 2000; //2 seconds

static uint8_t Timer_expired = FALSE;
static uint8_t advertising_filter = ADV_NO_WHITE_LIST_USE;

static VTIMER_HandleType peripheralTimerHandle;

Bonded_Device_Entry_t Bonded_Device_Entry[MAX_NUM_BONDED_DEVICES]; 

static void PeripheralUpdateTimeoutCB(void *param);

static Advertising_Set_Parameters_t Advertising_Set_Parameters[1]; 

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : App_PowerSaveLevel_Check
* Description    : It sets the Power Save Level 
* Input          : None
* Return         : Current Power Save Level
*******************************************************************************/
PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  /* BLE Security v4.2 is supported: BLE stack FW version >= 2.x */
  if (APP_FLAG(START_CONFIRM_NUMERIC_VALUE) || APP_FLAG(START_TERMINATE_LINK_FLAG) || BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
  {
    return POWER_SAVE_LEVEL_RUNNING;
  }
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}


#if GENERATE_RANDOM_KEY 
/*******************************************************************************
* Function Name  : Setup_DeviceAddress.
* Description    : Setup the device address.
* Input          : None.
* Return         : None.
*******************************************************************************/
static uint32_t Setup_RandomKey(void)
{
  tBleStatus ret;
  uint8_t random_number[8];
  uint32_t random_key = 99999; 
  
  /* get a random number from BlueNRG-1 */ 
  ret = hci_le_rand(random_number);
  if(ret != BLE_STATUS_SUCCESS)
     PRINTF("hci_le_rand() call failed: 0x%02x\n", ret);
  
  /* setup random_key with random number */
  for (uint8_t i=0; i<8; i++) {
    random_key += (435*random_number[i]); //2 TBR
  }
     
  return random_key;
}
#endif 
/*******************************************************************************
* Function Name  : Clear_Security_Database
* Description    : Clear the security database
* Input          : none.
* Return         : Status.
*******************************************************************************/
void Clear_Security_Database(void)
{
  uint8_t ret; 
  
  /* ACI_GAP_CLEAR_SECURITY_DB*/
  ret = aci_gap_clear_security_db();
  if (ret != BLE_STATUS_SUCCESS) 
  {
    PRINTF("aci_gap_clear_security_db() failed:0x%02x\r\n", ret);
  }
  else
  {
    PRINTF("aci_gap_clear_security_db() --> SUCCESS\r\n");
  }
  
}

/*******************************************************************************
* Function Name  : DeviceInit.
* Description    : Init the device.
* Input          : none.
* Return         : Status.
*******************************************************************************/
uint8_t DeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t name[] = {PERIPHERAL_DEVICE_NAME};
  uint8_t role = GAP_PERIPHERAL_ROLE;
  uint8_t bdaddr[] = PERIPHERAL_PUBLIC_ADDRESS; 
  
  static uint8_t adv_data[] = {0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
                               12, AD_TYPE_COMPLETE_LOCAL_NAME,PERIPHERAL_DEVICE_NAME};
 
#if GENERATE_RANDOM_KEY 
  /* init peripheral random pass key */
  Peripheral_Pass_Key = Setup_RandomKey(); 
#endif 
  
  /* Configure Public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != BLE_STATUS_SUCCESS){
    PRINTF("Setting BD_ADDR failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
  {
    PRINTF("Public Address: "); 
      
    for (uint8_t j = 0; j< 6; j++)
    {
      PRINTF("0x%02x ", bdaddr[j]);
    }
    PRINTF("\r\n");
  }    

  /* Set the TX power to 0 dBm */
  aci_hal_set_tx_power_level(0, 24);

  /* GATT Init */
  ret = aci_gatt_srv_init();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF ("Error in aci_gatt_srv_init(): 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF ("aci_gatt_srv_init() --> SUCCESS\r\n");
  }
  
  /* GAP Init */
  ret = aci_gap_init(role, 0x00,sizeof(name),IDENTITY_ADDRESS, &service_handle,  
                     &dev_name_char_handle, &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF ("Error in aci_gap_init() 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF ("aci_gap_init(PERIPHERAL, device name lenght: %d) --> SUCCESS\r\n",sizeof(name));
  }

  /* Set the device name */
  /* Set the device name */
  ret = Gap_profile_set_dev_name(0, sizeof(name), name);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in Gap_profile_set_dev_name 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("Gap_profile_set_dev_name() --> SUCCESS\r\n");
  }

  Clear_Security_Database();

  /* ACI_GAP_SET_IO_CAPABILITY */
  ret = aci_gap_set_io_capability(IO_CAPABILITY);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_set_io_capability(%d) failed:0x%02x\r\n", IO_CAPABILITY, ret);
  }
  else
  {
    PRINTF("aci_gap_set_io_capability(%d) --> SUCCESS\r\n",IO_CAPABILITY);
  }
 
  /* BLE Security v4.2 is supported: BLE stack FW version >= 2.x */  
  ret = aci_gap_set_authentication_requirement(SLAVE_BONDING_USAGE,
                                               MITM_MODE,
                                               SECURE_CONNECTION_SUPPORT,
                                               KEYPRESS_NOTIFICATION,
                                               MIN_KEY_SIZE, 
                                               MAX_KEY_SIZE,
                                               FIXED_PIN_POLICY,
                                               PERIPHERAL_SECURITY_KEY);
  
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_set_authentication_requirement() failed: 0x%02x\r\n", ret);
    return ret;
  }  
  else
  {
    PRINTF("aci_gap_set_authentication_requirement() --> SUCCESS\r\n");
  }
  
  ret = Add_Service();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in Add_Service 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("Add_Service() --> SUCCESS\r\n");
  }
  
  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_gap_set_advertising_data 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("aci_gap_set_advertising_data() --> SUCCESS\r\n");
  }
  
  /* Start the Peripheral Timer */
  peripheralTimerHandle.callback = PeripheralUpdateTimeoutCB;  
  ret = HAL_VTIMER_StartTimerMs(&peripheralTimerHandle, update_rate);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("HAL_VTIMER_StartTimerMs() failed; 0x%02x\r\n", ret);
    return ret;
  } else {
    Timer_expired = FALSE;
  }
  
  return BLE_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name  : Process_InputData.
* Description    : Process a command. It should be called when data are received.
* Input          : data_buffer: data address.
*	           Nb_bytes: number of received bytes.
* Return         : none.
*******************************************************************************/
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
    
}


/*******************************************************************************
* Function Name  : Make_Connection.
* Description    : If the device is a Client create the connection. Otherwise puts
*                  the device in discoverable mode.
* Input          : none.
* Return         : none.
*******************************************************************************/
void Make_Connection(uint8_t filter)
{  
  tBleStatus ret;
  
  if (filter == ADV_NO_WHITE_LIST_USE)
  {
      /* no advertising filter is used: enters in discoverable mode for first detection from master device based on local_name on advertising packet */
     
      ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                                  ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                                  0x100,0x200,
                                                  ADV_CH_ALL,
                                                  0,NULL,
                                                  filter,
                                                  0, /* 0 dBm */
                                                  LE_1M_PHY, /* Primary advertising PHY */
                                                  0, /* 0 skips */
                                                  LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                                  0, /* SID */
                                                  0 /* No scan request notifications */); 
      if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("Error in aci_gap_set_advertising_configuration 0x%02x\r\n", ret);
      } else {
        PRINTF ("aci_gap_set_advertising_configuration() OK; Filter =  0x%02x)\r\n", filter);
      }
  }
  else if (filter == ADV_WHITE_LIST_FOR_ALL)
  {
      static uint8_t adv_data1[] = {12, AD_TYPE_COMPLETE_LOCAL_NAME,PERIPHERAL_DEVICE_NAME};
      
      ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data1), adv_data1);
      
      if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("Error in aci_gap_set_advertising_data (no ADType flags) 0x%02x\r\n", ret);
      } else {
        PRINTF("aci_gap_set_advertising_data(no ADType flags) --> SUCCESS\r\n");
      }
      
      /* Advertising filter is enabled: enter in undirected connectable mode in order to use the advertising filter on bonded device */
      ret = aci_gap_set_advertising_configuration(0, 0x0,
                                                  ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                                  0x100,0x200,
                                                  ADV_CH_ALL,
                                                  0,NULL,
                                                  filter,
                                                  0, /* 0 dBm */
                                                  LE_1M_PHY, /* Primary advertising PHY */
                                                  0, /* 0 skips */
                                                  LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                                  0, /* SID */
                                                  0 /* No scan request notifications */); 
      if (ret != BLE_STATUS_SUCCESS)
      {
        PRINTF ("Error in aci_gap_set_advertising_configuration: 0x%02x, (Filter =  0x%02x)\r\n", ret, filter);
      }
      else
      {
        PRINTF ("aci_gap_set_advertising_configuration() OK; Filter =  0x%02x)\r\n", filter);
      } 
  }
  else 
  {
    PRINTF ("Make_Connection() wrong filter type: 0x%02x\r\n",filter);
  }

  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;
  
  //enable advertising
  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF ("Error in aci_gap_set_advertising_enable: 0x%02x\r\n", ret);
  }
  else
  {
    PRINTF ("aci_gap_set_advertising_enable() OK\r\n");
  }
}

/*******************************************************************************
* Function Name  : APP_Tick.
* Description    : Tick to run the application state machine.
* Input          : none.
* Return         : none.
*******************************************************************************/
void APP_Tick(void)
{

  tBleStatus ret;
  uint8_t Num_of_Addresses; 
  
  if(APP_FLAG(SET_CONNECTABLE))
  {
    Make_Connection(advertising_filter);
    APP_FLAG_CLEAR(SET_CONNECTABLE);
  }
  
  if(APP_FLAG(START_GAP_SLAVE_SECURITY_REQUEST))
  {
     
    /* BLE Security v4.2 is supported: BLE stack FW version >= 2.x */
    ret = aci_gap_slave_security_req(connection_handle);
    if (ret == BLE_STATUS_SUCCESS)
    {
      APP_FLAG_CLEAR(START_GAP_SLAVE_SECURITY_REQUEST);
      PRINTF("Device Connected: do aci_gap_slave_security_req() --> SUCCESS\r\n");
      PRINTF("\r\n");
      
#if GENERATE_RANDOM_KEY 
    if (Peripheral_Pass_Key <99999)
    {
      Peripheral_Pass_Key = Peripheral_Pass_Key + 100000;
    }
#endif
    }
    
  }
  if (APP_FLAG(ACI_GAP_PASS_KEY_REQ_EVENT_FLAG))
  {
    PRINTF("** Peripheral Security Key (IO DISPLAY): %d. INSERT IT on CENTRAL Device !\r\n", (int)Peripheral_Pass_Key);
    PRINTF("\r\n");
    APP_FLAG_CLEAR(ACI_GAP_PASS_KEY_REQ_EVENT_FLAG);
    
  }
      
  //if ((APP_FLAG(HCI_ENCRYPTION_CHANGE_EVENT_FLAG)) && (APP_FLAG(ACI_GAP_PAIRING_COMPLETE_EVENT_FLAG))) 
  if ((APP_FLAG(ACI_GAP_PAIRING_COMPLETE_EVENT_FLAG)) || (APP_FLAG(HCI_ENCRYPTION_CHANGE_EVENT_FLAG))) 
  {
    
    ret =  aci_gap_get_bonded_devices(0, MAX_NUM_BONDED_DEVICES, &Num_of_Addresses,
                                       Bonded_Device_Entry);
    
    if (ret == BLE_STATUS_SUCCESS)
    {
      if (Num_of_Addresses > 0)
      {
        PRINTF("Bonded with %d device(s): \r\n", Num_of_Addresses); 
      }
        
      for (uint8_t i = 0; i< Num_of_Addresses; i++)
      {
        PRINTF("Address_Type: ");
        if (Bonded_Device_Entry[i].Address_Type == PUBLIC_ADDR) 
          PRINTF("PUBLIC ");
        else 
          PRINTF("RANDOM ");
        PRINTF("Address: "); 
          
        for (uint8_t j = 0; j< 6; j++)
        {
          PRINTF("0x%02x ", Bonded_Device_Entry[i].Address[j]);
        }
        PRINTF("\r\n");
      }
    }
    if (Num_of_Addresses > 0)
    {
      ret = aci_gap_configure_white_and_resolving_list(0x01);
      if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_configure_white_and_resolving_list() failed:0x%02x\r\n", ret);
      }
      else
      {
        PRINTF("aci_gap_configure_white_and_resolving_list() --> SUCCESS\r\n");
      }
    }
    
    APP_FLAG_CLEAR(HCI_ENCRYPTION_CHANGE_EVENT_FLAG);
    APP_FLAG_CLEAR(ACI_GAP_PAIRING_COMPLETE_EVENT_FLAG);
  }
  
  if (APP_FLAG(START_TERMINATE_LINK_FLAG))
  {
    advertising_filter = ADV_WHITE_LIST_FOR_ALL; /* filter on whitelist */
    
    ret = aci_gap_terminate(connection_handle,0x13);
    if (ret != BLE_STATUS_SUCCESS) {
      PRINTF("aci_gap_terminate() failed:0x%02x\r\n", ret);
    }
    else
    {
      PRINTF("aci_gap_terminate --> SUCCESS\r\n");
    }
    APP_FLAG_CLEAR(START_TERMINATE_LINK_FLAG); 
    
  }
  if (APP_FLAG(PRINT_CONNECTED_DATA))
  {
    PRINTF("Connected with: Address_Type: ");
      if (Connected_Address_Type == PUBLIC_ADDR) 
        PRINTF("PUBLIC ");
      else 
        PRINTF("RANDOM ");
      PRINTF("Address: "); 
        
      for (uint8_t j = 0; j< 6; j++)
      {
        PRINTF("0x%02x ", Connected_Address[j]);
      }
      PRINTF("\r\n");
    
    APP_FLAG_CLEAR(PRINT_CONNECTED_DATA);
  }

  
  /*  Update TX value */
  if (Timer_expired) 
  {    
    Timer_expired = FALSE;
    if (HAL_VTIMER_StartTimerMs(&peripheralTimerHandle, update_rate) != BLE_STATUS_SUCCESS)
        Timer_expired = TRUE;
    if (APP_FLAG(CONNECTED) && APP_FLAG(DO_NOTIFICATIONS_FLAG))
    {
        notification_data_handler();
    }
    
  }
  
  /* BLE Security v4.2 is supported: BLE stack FW version >= 2.x */
  if (APP_FLAG(START_CONFIRM_NUMERIC_VALUE))
  {
    Confirm_Numeric_Value_SM(); 
  }
  
}/* end APP_Tick() */


/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
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
  if(Status != BLE_STATUS_SUCCESS)
    return;
  
  connection_handle = Connection_Handle;
  
  APP_FLAG_SET(CONNECTED);
  APP_FLAG_SET(START_GAP_SLAVE_SECURITY_REQUEST);
  
  APP_FLAG_SET(PRINT_CONNECTED_DATA); 
  
  Connected_Address_Type = Peer_Address_Type;
  
  memcpy(&(Connected_Address[0]), &(Peer_Address[0]),6);
  
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
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  APP_FLAG_CLEAR(CONNECTED);
  APP_FLAG_CLEAR(DO_NOTIFICATIONS_FLAG);
  /* Make the device connectable again. */
  APP_FLAG_SET(SET_CONNECTABLE);
  APP_FLAG_CLEAR(NOTIFICATIONS_ENABLED);
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
  PRINTF("Disconnection, Reason: 0x%02x\r\n",Reason); 

  advertising_filter = ADV_WHITE_LIST_FOR_ALL; /* filter on whitelist */

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
  PRINTF("------ aci_gatt_srv_attribute_modified_event, Attr_Handle: 0x%02x, Value: %d%d\r\n", Attr_Handle, Attr_Data[1], Attr_Data[0]);
  if (Attr_Handle == TXCharHandle+2)
  {
    PRINTF("Start notification cycle of TX characteristic value!\r\n");
  } 
}


/*******************************************************************************
 * Function Name  : aci_gatt_tx_pool_available_event.
 * Description    : This event occurs when a TX pool available is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{       
  /* It allows to notify when at least 2 GATT TX buffers are available */
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
}  


void aci_gatt_srv_read_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Data_Offset)
{
  read_data_handler();
}

/* BLE Security v4.2 is supported: BLE stack FW version >= 2.x */
void aci_gap_pairing_complete_event(uint16_t Connection_Handle,
                                    uint8_t Status,
                                    uint8_t Reason)
{
  
  APP_FLAG_SET(ACI_GAP_PAIRING_COMPLETE_EVENT_FLAG); 
  if (Status == BLE_STATUS_SUCCESS) 
  {
    APP_FLAG_SET(DO_NOTIFICATIONS_FLAG);
  }
  PRINTF("aci_gap_pairing_complete_event(), connection handle: 0x%02x, status: 0x%02x, reason: 0x%02x\r\n", Connection_Handle, Status, Reason);
 }

void aci_gap_pass_key_req_event(uint16_t Connection_Handle)
{
  uint8_t status; 
  
  status = aci_gap_pass_key_resp(connection_handle, Peripheral_Pass_Key);
  if (status != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_pass_key_resp(%d) failed:0x%02x\r\n", (int)Peripheral_Pass_Key, status);
  }
  else
  {
    PRINTF("aci_gap_pass_key_resp(%d) OK\r\n", (int)Peripheral_Pass_Key);
    APP_FLAG_SET(ACI_GAP_PASS_KEY_REQ_EVENT_FLAG);
  }
  
}

void hci_encryption_change_event(uint8_t Status,
                                 uint16_t Connection_Handle,
                                 uint8_t Encryption_Enabled)
{
  APP_FLAG_SET(HCI_ENCRYPTION_CHANGE_EVENT_FLAG);
  PRINTF("hci_encryption_change_event(), connection handle:0x%02x, status: 0x%02x, encryption enable: %d\r\n", Connection_Handle, Status, Encryption_Enabled);
  
}

void aci_gap_bond_lost_event(void)
{
  uint8_t ret; 
  
  ret = aci_gap_allow_rebond(connection_handle);
  PRINTF("aci_gap_allow_rebond(), status: 0x%02x\r\n", ret);
  
}

void aci_gap_slave_security_initiated_event(void)
{
  
  PRINTF("aci_gap_slave_security_initiated_event()\r\n"); 
}

/* BLE Security v4.2 is supported: BLE stack FW version >= 2.x */
static void Confirm_Numeric_Value(uint16_t Connection_Handle, uint8_t yes_no)
{
  uint8_t ret = 0;
  
  /* First simple case: we confirm the value assiming it is correct */
  ret = aci_gap_numeric_comparison_value_confirm_yesno(Connection_Handle, yes_no); 
        
  if (ret != BLE_STATUS_SUCCESS) 
  {
     PRINTF("\r\n****aci_gap_numeric_comparison_value_confirm_yesno(%d) procedure fails with status 0x%02x\r\n", yes_no, ret);
  }
  else
  {
     PRINTF("\r\n****aci_gap_numeric_comparison_value_confirm_yesno(%d) OK \r\n", yes_no);
  }
}

void Confirm_Numeric_Value_SM(void)
{
  uint8_t input = 200;
    
  if (BSP_COM_Read(&input))
  {
    if (input == 'y')
    {
      /* Input digit is OK */
      PRINTF("\r\n Numeric Value is CONFIRMED!\r\n"); 
      Confirm_Numeric_Value(connection_handle, 0x01);    
      APP_FLAG_CLEAR(START_CONFIRM_NUMERIC_VALUE); 

    }
    else if (input == 'n')
    {
      /* Input digit is NO */
      PRINTF("\r\n Numeric Value is NOT CONFIRMED!\r\n"); 
      Confirm_Numeric_Value(connection_handle, 0x00);    
      APP_FLAG_CLEAR(START_CONFIRM_NUMERIC_VALUE); 
    }
    else
    {
      /* for MISRA compliance only */
    }  
  }
}

void aci_gap_numeric_comparison_value_event(uint16_t Connection_Handle, uint32_t Numeric_Value)
{
  PRINTF("\r\n Numeric value = %d. Do you confirm? (y/n)\r\n", (int)Numeric_Value);
  /* Add logic  (State machine) for confirming the value by calling the 
   
    aci_gap_numeric_comparison_confirm_yesno(uint16_t Connection_Handle, uint8_t confirm_yes_no); 
   */
  APP_FLAG_SET(START_CONFIRM_NUMERIC_VALUE); 
   
}
/*******************************************************************************
 * Function Name  : PeripheralUpdateTimeoutCB.
 * Description    : This function will be called on the expiry of 
 *                  a one-shot virtual timer.
 * Input          : 
 * Output         : 
 * Return         : 
 *******************************************************************************/      
static  void PeripheralUpdateTimeoutCB(void *param)
{
  Timer_expired = TRUE;
}
