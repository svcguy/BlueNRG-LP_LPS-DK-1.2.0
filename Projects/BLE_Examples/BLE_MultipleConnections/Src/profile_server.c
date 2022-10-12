/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : profile.c
* Author             : AMS - RF  Application team
* Description        : This file define the procedure to connect and send data.
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
#include "gp_timer.h"
#include "ble_const.h" 
#include "bluenrg_lp_stack.h"
#include "app_state.h"
#include "osal.h"
#include "gatt_db.h"
#include "profile.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "gap_profile.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

#define ADV_INTERVAL_MIN    ((uint16_t)(60/0.625))      // 60 ms
#define ADV_INTERVAL_MAX    ((uint16_t)(60/0.625))      // 60 ms

#define DEBOUNCE_TIMEOUT_MS          300
#define NOTIFICATION_INTERVAL_MS     1000

#define DEBUG         2

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#if DEBUG > 1
#include <stdio.h>
#define PRINTF_DBG2(...) printf(__VA_ARGS__)
#else
#define PRINTF_DBG2(...)
#endif

#define PRINT_ADDRESS(a)   PRINTF("0x%02X%02X%02X%02X%02X%02X", a[5], a[4], a[3], a[2], a[1], a[0])

uint8_t button_timer_expired = TRUE;


/* Private variables ---------------------------------------------------------*/

volatile int app_flags = SET_CONNECTABLE;

static uint8_t debounce_timeout_occurred = TRUE;
static VTIMER_HandleType debounce_timer;
static VTIMER_HandleType notification_timer;

uint16_t connection_handle;

static char name[] = LOCAL_NAME;
#define NAME_LENGTH (sizeof(name)-1)
    
static uint8_t adv_data[5+NAME_LENGTH+19] = {
  0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
  NAME_LENGTH+1, AD_TYPE_COMPLETE_LOCAL_NAME};

#define NON_DISCOVERABLE_ADV_DATA_LEN   3               // Do not send name and UUID if not discoverable
#define DISCOVERABLE_ADV_DATA_LEN       sizeof(adv_data)

/* Private function prototypes -----------------------------------------------*/
void StopRadioActivity(void);
void DebounceTimeoutCB(void *param);
void SendDataCB(void *param);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : DeviceInit.
* Description    : Init the Serial Port device adding the service.
* Input          : none.
* Return         : Status.
*******************************************************************************/
uint8_t DeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  
  uint8_t role = GAP_PERIPHERAL_ROLE;
  
  uint8_t addr_len;
  uint8_t address[6];

  /* Set the TX power to 0 dBm */
  aci_hal_set_tx_power_level(0, 24);
  
  /* Since we need to transfer notifications of 244 bytes in a single packet, the LL payload must be
   244 bytes for application data + 3 bytes for ATT header + 4 bytes for L2CAP header. */
   ret = hci_le_write_suggested_default_data_length(251, 2120);
   PRINTF("hci_le_write_suggested_default_data_length(): 0x%02x\r\n", ret);

  /* GATT Init */
  ret = aci_gatt_srv_init();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_gatt_srv_init(): 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("aci_gatt_srv_init() --> SUCCESS\r\n");
  }
  
  /* GAP Init */
  ret = aci_gap_init(role, 0x00, strlen(name), STATIC_RANDOM_ADDR, &service_handle,  
                     &dev_name_char_handle, &appearance_char_handle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_gap_init() 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("aci_gap_init() --> SUCCESS\r\n");
  }
  
  aci_hal_read_config_data(0x80, &addr_len, address);
  PRINTF("Static random address: ");
  PRINT_ADDRESS(address);
  PRINTF("\r\n");

  /* Set the device name */
  ret = Gap_profile_set_dev_name(0, sizeof(name), (uint8_t *)name);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF ("Error in Gap_profile_set_dev_name 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF ("Gap_profile_set_dev_name() --> SUCCESS\r\n");
  }
  
  aci_gap_set_io_capability(IO_CAP_NO_INPUT_NO_OUTPUT);
  
  ret = Add_SerialPort_Service();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in Add_SerialPort_Service 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("Add_SerialPort_Service() --> SUCCESS\r\n");
  }
    
  // Add name to advertising data
  Osal_MemCpy(adv_data+5,name,NAME_LENGTH);
  // Add service UUID to advertising data
  adv_data[5+NAME_LENGTH] = 18;
  adv_data[5+NAME_LENGTH+1] = AD_TYPE_128_BIT_UUID_SERVICE_DATA;
  Osal_MemCpy(adv_data+5+NAME_LENGTH+2,SerialPort_service_uuid,sizeof(SerialPort_service_uuid));
  adv_data[5+NAME_LENGTH+2+16] = PROFILE_DATA_NODE; // Service data, to identify different kind of devices (0: slave-only device, 1: master slave)
  
  debounce_timer.callback = DebounceTimeoutCB;
  notification_timer.callback = SendDataCB;
  
  return BLE_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name  : App_SleepMode_Check.
* Description    : Check if the device can go to sleep. See sleep.h
* Input          : Requested sleep mdoe.
* Return         : Allowed sleep mode
*******************************************************************************/
PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
    return POWER_SAVE_LEVEL_RUNNING;
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}


/*******************************************************************************
* Function Name  : Make_Connection.
* Description    : If the device is a Client create the connection. Otherwise puts
*                  the device in discoverable mode.
* Input          : none.
* Return         : none.
*******************************************************************************/
void Connect(void)
{  
  tBleStatus ret;
  
  if(APP_FLAG(PAIRING_MODE)){
    
    adv_data[2] = FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED;
    
    // Forget any bonded devices
    aci_gap_clear_security_db();
    
    aci_gap_set_authentication_requirement(BONDING, MITM_PROTECTION_NOT_REQUIRED, 0, 0, 7, 16, DONOT_USE_FIXED_PIN_FOR_PAIRING, 0);
    
    ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                                ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                                ADV_INTERVAL_MIN, ADV_INTERVAL_MAX,
                                                ADV_CH_ALL,
                                                0,NULL,
                                                ADV_NO_WHITE_LIST_USE,
                                                0, /* 0 dBm */
                                                LE_1M_PHY, /* Primary advertising PHY */
                                                0, /* 0 skips */
                                                LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                                0, /* SID */
                                                0 /* No scan request notifications */);
    
    PRINTF("aci_gap_set_advertising_configuration %02X\n", ret);
    
    ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, DISCOVERABLE_ADV_DATA_LEN, adv_data);
    PRINTF("Set advertising data %02X\n", ret);
  }
  else {
    
    adv_data[2] = FLAG_BIT_BR_EDR_NOT_SUPPORTED;
    
    // Setting MITM_PROTECTION_REQUIRED does not allow to pair, since IO capabilities are set to "No input / no output"
    aci_gap_set_authentication_requirement(NO_BONDING, MITM_PROTECTION_REQUIRED, 0, 0, 7, 16, DONOT_USE_FIXED_PIN_FOR_PAIRING, 0);
    
    // Add bonded devices to the whitelist
    aci_gap_configure_white_and_resolving_list(1);
    
    ret = aci_gap_set_advertising_configuration(0, GAP_MODE_NON_DISCOVERABLE,
                                                ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                                ADV_INTERVAL_MIN, ADV_INTERVAL_MAX,
                                                ADV_CH_ALL,
                                                0,NULL,
                                                ADV_WHITE_LIST_FOR_ALL,
                                                0, /* 0 dBm */
                                                LE_1M_PHY, /* Primary advertising PHY */
                                                0, /* 0 skips */
                                                LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                                0, /* SID */
                                                0 /* No scan request notifications */);
    
    PRINTF("aci_gap_set_advertising_configuration %02X\n", ret);
    
    ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, NON_DISCOVERABLE_ADV_DATA_LEN, adv_data);    
    PRINTF("Set advertising data %02X\n", ret);
  }
  
  Advertising_Set_Parameters_t Advertising_Set_Parameters[1];    
  
  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;
  
  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 
  if (ret != BLE_STATUS_SUCCESS)
    PRINTF ("Error in aci_gap_set_advertising_enable(): 0x%02x\r\n", ret);
  else
    PRINTF ("aci_gap_set_advertising_enable() --> SUCCESS\r\n");
  
  APP_FLAG_CLEAR(SET_CONNECTABLE);
}

/*******************************************************************************
* Function Name  : StopRadioActivity.
* Description    : Stop any radio activity, i.e. scanning, advertising or connection
* Input          : None
* Return         : None
*******************************************************************************/
void StopRadioActivity(void)
{
  PRINTF_DBG2("StopRadioActivity\r\n");
  
  if(connection_handle){
    // If connected, terminate connection
    aci_gap_terminate(connection_handle, BLE_ERROR_TERMINATED_REMOTE_USER);
  }
  else {
    // If advertising, stop advertising
    aci_gap_set_advertising_enable(0,0,NULL);
    APP_FLAG_SET(SET_CONNECTABLE);
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
  if(!APP_FLAG(PAIRING_MODE) && debounce_timeout_occurred && BSP_PB_GetState(BSP_PUSH1)==1){
    // Put device in pairing mode.
    StopRadioActivity();
    
    APP_FLAG_SET(PAIRING_MODE);
    BSP_LED_On(BSP_LED3);
    
    debounce_timeout_occurred = FALSE;
    HAL_VTIMER_StartTimerMs(&debounce_timer, DEBOUNCE_TIMEOUT_MS);
  }

  
  if(APP_FLAG(SET_CONNECTABLE))
  {
    Connect();    
  }
  
}/* end APP_Tick() */

void DebounceTimeoutCB(void *param)
{
  debounce_timeout_occurred = TRUE;
}

void SendDataCB(void *param)
{
  tBleStatus ret;  
  static uint32_t counter = 0;  
  uint8_t data[CHARACTERISTIC_LEN] = {0};
  
  HOST_TO_LE_32(data,counter);
  
  ret =  aci_gatt_srv_notify(connection_handle, TXCharHandle + 1, 0, CHARACTERISTIC_LEN, data);
  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("Data sent (%d)\n",counter);
  else
    PRINTF("Error sending data: 0x%02X\n",ret);
  
  counter++;
  
  HAL_VTIMER_StartTimerMs(&notification_timer, NOTIFICATION_INTERVAL_MS);
}

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
   PRINTF_DBG2("hci_le_connection_complete_event %d\r\n", Status);
   
   if(Status == 0){
     
     connection_handle = Connection_Handle;
     
     APP_FLAG_SET(CONNECTED);
     
     PRINTF("Connected\r\n");
     
     APP_FLAG_CLEAR(PAIRING_MODE);
     BSP_LED_Off(BSP_LED3);
     
   }   
   else if(Status == BLE_ERROR_UNKNOWN_CONNECTION_ID){
     PRINTF_DBG2("Connection canceled.\r\n");
   }

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
  if(Status != 0)
    return;
  
  PRINTF("Disconnected, status 0x%02X, reason 0x%02X\r\n", Status, Reason);
  
  connection_handle = 0;
  
  APP_FLAG_CLEAR(CONNECTED);
  /* Make the device connectable again. */
  APP_FLAG_SET(SET_CONNECTABLE);
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
  
  HAL_VTIMER_StopTimer(&notification_timer);
  
}/* end hci_disconnection_complete_event() */

void aci_gatt_srv_write_event(uint16_t Connection_Handle,
                                 uint8_t Resp_Needed,
                                 uint16_t Attribute_Handle,
                                 uint16_t Data_Length,
                                 uint8_t Data[])
{
    uint8_t att_error = BLE_ATT_ERR_NONE;

    if(Attribute_Handle == RXCharHandle + 1)
    {
      uint32_t counter;
    
      counter = LE_TO_HOST_32(Data);
    
      PRINTF("Write from client: %d (%d bytes)\n", counter, Data_Length);
    }
    
    if (Resp_Needed == 1U)
    {
        aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, att_error, 0,  NULL);
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

void aci_gap_bond_lost_event(void)
{  
  PRINTF("aci_gap_bond_lost_event\r\n");
}

/*******************************************************************************
 * Function Name  : hci_le_data_length_change_event.
 * Description    : function called when LL data length changes.
 *                  See file bluenrg_lp_events.h.
 * Input          : See file bluenrg_lp_events.h
 * Output         : See file bluenrg_lp_events.h
 * Return         : See file bluenrg_lp_events.h
 *******************************************************************************/
void hci_le_data_length_change_event(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
  PRINTF("hci_le_data_length_change_event handle: 0x%04X, MaxTxOctets: %d, MaxTxTime: %d, MaxRxOctets: %d, MaxRxTime: %d.\r\n", Connection_Handle, MaxTxOctets, MaxTxTime, MaxRxOctets, MaxRxTime);
}

/*******************************************************************************
 * Function Name  : aci_att_exchange_mtu_resp_event.
 * Description    : function called when an update of the ATT MTU is received.
 *                  See file bluenrg_lp_events.h.
 * Input          : Connection_Handle handle of the connection
 *                  RX_MTU New ATT MTU on this connection handle
 * Output         : None
 * Return         : None
 *******************************************************************************/
void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t RX_MTU)
{
  PRINTF("aci_att_exchange_mtu_resp_event, handle: 0x%04X, RX_MTU: %d \r\n", Connection_Handle, RX_MTU);
}

/*******************************************************************************
 * Function Name  : aci_gap_pairing_complete_event.
 * Description    : Function called when pairing is complete or link encrypted
 *                  See file bluenrg_lp_events.h.
 * Input          : See file bluenrg_lp_events.h
 * Output         : See file bluenrg_lp_events.h
 * Return         : See file bluenrg_lp_events.h
 *******************************************************************************/
void aci_gap_pairing_complete_event(uint16_t Connection_Handle,
                                    uint8_t Status,
                                    uint8_t Reason)
{
  PRINTF("Paired device\n");
  HAL_VTIMER_StartTimerMs(&notification_timer, NOTIFICATION_INTERVAL_MS);
}
