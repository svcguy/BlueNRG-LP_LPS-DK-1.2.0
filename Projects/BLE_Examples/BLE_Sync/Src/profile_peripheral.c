/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : profile_peripheral.c
* Author             : AMS - RF  Application team
* Description        : This file implements the profile on the peripheral
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
#include "procedures.h"
#include "profile.h"
#include "gap_profile.h"
#include "netclock.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

#define WRITE_INTERVAL_MS           2000

#define DEBUG         1

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


/* Private variables ---------------------------------------------------------*/

volatile int app_flags = 0;

struct {
  uint16_t conn_handle;
  uint16_t event_counter;
  uint64_t event_systime;
  uint8_t first_event;
}connection_info;
 
static uint64_t next_vtime_vtimer = 0;

/* States of the state machine used to discover services, enable notifications and
  write the offset into the slaves.
*/
typedef enum{
  IDLE = 0,
  START_SERVICE_DISCOVERY,
  DISCOVERING_SERVICES,
  START_SYNC_CHAR_DISCOVERY,
  DISCOVERING_SYNC_CHAR,
  READ_SYNC_CLOCK_CHAR,
  READING_SYNC_CLOCK_CHAR,
  DONE,
} client_state_t;

// Type of the structure used to store the state related to each server
typedef struct central_info_s {
  uint8_t  address_type;
  uint8_t  address[6];
  uint16_t conn_handle;
  client_state_t client_state;
  uint16_t sync_serv_start_handle;
  uint16_t sync_serv_end_handle;
  uint16_t sync_clock_handle;
}central_info_t;

central_info_t central_info;

void CentralInfoInit(void);

/* Private function prototypes -----------------------------------------------*/
void SendDataCB(void *param);
void testCB(void *param);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : address_is_set.
* Description    : Check if address is set, i.e. if it is different
*                  from 0x000000000000
* Input          : the address.
* Return         : TRUE if addres is set, FALSE otherwise.
*******************************************************************************/
uint8_t address_is_set(uint8_t address[6])
{
  int i;
  
  for(i = 0; i < 6; i++){
    if(address[i] != 0)
      break;
  }
  if(i == 6)
    return FALSE;
  else  
    return TRUE;
}


/*******************************************************************************
* Function Name  : Chat_DeviceInit.
* Description    : Init the CHAT device.
* Input          : none.
* Return         : Status.
*******************************************************************************/
uint8_t DeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t addr_len;
  uint8_t address[6];
  
  /* Set the TX power to 0 dBm */
  aci_hal_set_tx_power_level(0, 24);  
  
  aci_hal_set_radio_activity_mask(0xFF); //(0x04|0x20|0x10); // slave, master
  
  uint8_t val = 1;
  aci_hal_write_config_data(0xD3, 1, &val);
  
  NETCLOCK_Init();

  /* GATT Init */
  ret = aci_gatt_srv_init();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_gatt_srv_init(): 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("aci_gatt_srv_init() --> SUCCESS\r\n");
  }
  
  /* GAP Init */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0x00, strlen(name), STATIC_RANDOM_ADDR, &service_handle,  
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
  ret = Gap_profile_set_dev_name(0, strlen(name), (uint8_t *) name);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error with Gap_profile_set_dev_name 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("Gap_profile_set_dev_name() --> SUCCESS\r\n");
  }
  
  aci_gap_set_io_capability(IO_CAP_NO_INPUT_NO_OUTPUT);
  
  aci_gap_set_authentication_requirement(NO_BONDING, MITM_PROTECTION_NOT_REQUIRED, 0, 0, 7, 16, DONOT_USE_FIXED_PIN_FOR_PAIRING, 0); 
  
  // Initialize info related to central device.
  CentralInfoInit();
  
  ret = ConfigureAdvertising();
    
  PRINTF("Advertising configuration %02X\n", ret); 
  
  return BLE_STATUS_SUCCESS;
}

void testCB(void *param)
{
  LL_GPIO_SetOutputPin(TEST_PULSE_GPIO_PORT, TEST_PULSE_GPIO_PIN);

  printf("0x%08X%08X\n", (uint32_t)(next_vtime_vtimer>>32),(uint32_t)next_vtime_vtimer);
  
  next_vtime_vtimer += NETCLOCK_SECONDS/4;
  
  NETCLOCK_StartTimer(next_vtime_vtimer, testCB);  
  
  LL_GPIO_ResetOutputPin(TEST_PULSE_GPIO_PORT, TEST_PULSE_GPIO_PIN);
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
* Function Name  : SlaveInit.
* Description    : Init the slave state
* Input          : Index of the slave
* Return         : none.
*******************************************************************************/
void CentralInfoInit(void)
{  
  Osal_MemSet(&central_info, 0, sizeof(central_info));
}

/*******************************************************************************
* Function Name  : StartDiscovery.
* Description    : Begin discovery of the services for the selected slave
* Input          : Index of the slave
* Return         : none.
*******************************************************************************/
void StartDiscovery(void)
{
  central_info.client_state = START_SERVICE_DISCOVERY;
}

/*******************************************************************************
* Function Name  : PerSlaveStateMachine.
* Description    : State machine handling the discovery of the services, setting
*                  of the client characteristic configuratino descriptors and
*                  writing into the characteristics.
* Input          : none
* Return         : none.
*******************************************************************************/
void ClientStateMachine(void)
{
  tBleStatus ret;
  
  switch(central_info.client_state){
    
  case START_SERVICE_DISCOVERY:
    {
      /* Start discovery of all primary services */
      
      ret = aci_gatt_clt_disc_all_primary_services(central_info.conn_handle);
      PRINTF_DBG2("aci_gatt_clt_disc_all_primary_services(): 0x%02X\r\n", ret);
      if(ret == 0){
        central_info.client_state = DISCOVERING_SERVICES;
      }
      else {
        central_info.client_state = IDLE;
      }      
    }
    break;
  case START_SYNC_CHAR_DISCOVERY:
    {
      /* Start characteristic discovery for Chat Service */
      ret = aci_gatt_clt_disc_all_char_of_service(central_info.conn_handle, central_info.sync_serv_start_handle, central_info.sync_serv_end_handle);
      PRINTF_DBG2("aci_gatt_clt_disc_all_char_of_service() for Sync service: 0x%02X\r\n", ret);
      if(ret == 0){
        central_info.client_state = DISCOVERING_SYNC_CHAR;
      }
      else {
        central_info.client_state = IDLE;
      }      
    }
    break;
  case READ_SYNC_CLOCK_CHAR:
    {        
      ret = aci_gatt_clt_read(central_info.conn_handle, central_info.sync_clock_handle + 1);
      PRINTF_DBG2("aci_gatt_clt_read() to read sync clock characteristic: 0x%02X\r\n", ret);
      
      if(ret == 0){
        central_info.client_state = READING_SYNC_CLOCK_CHAR;
      }
      else if(ret == BLE_STATUS_INSUFFICIENT_RESOURCES){ // TODO: check
        // Retry later
      }
      else {
        central_info.client_state = IDLE;
      }
    }
    break;
  default:
    break;      
  }  
}

static void DeviceStateMachine(void)
{
  
  if(!APP_FLAG(CONNECTED) && !APP_FLAG(ADVERTISING)){
    
    if(StartAdvertising() == BLE_STATUS_SUCCESS){
      APP_FLAG_SET(ADVERTISING);
    }    
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
  
  DeviceStateMachine();
  ClientStateMachine();
  
}/* end APP_Tick() */

/* ***************** BlueNRG-LP Stack Callbacks ********************************/

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
  PRINTF_DBG2("hci_le_connection_complete_event, handle: 0x%04X, Status %d\r\n", Connection_Handle, Status);
   
   if(Status == 0){
     
     if(Role == 0x01) { // Slave role
       
       APP_FLAG_CLEAR(ADVERTISING);
       
       connection_info.event_counter = 0;
         
       central_info.address_type = Peer_Address_Type;
       memcpy(central_info.address, Peer_Address, 6);         
       central_info.conn_handle = Connection_Handle;
       
       PRINTF("Connected with central ");
       PRINT_ADDRESS(Peer_Address);
       PRINTF("\r\n");
       
       APP_FLAG_SET(CONNECTED);
       
       NETCLOCK_SetSyncInterval(((uint32_t)(Conn_Interval))<<9); // Converting Conn_Interval to units of 625/256 us (1250/625*256 = 2*256 = 512 )
       
       StartDiscovery();       
     }
     
   }   
   else if(Status == BLE_ERROR_UNKNOWN_CONNECTION_ID){
     PRINTF_DBG2("Connection canceled.\r\n");
   }

}/* end hci_le_connection_complete_event() */


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

void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  
  PRINTF("hci_disconnection_complete_event, Status 0x%02X, Handle 0x%04X, Reason 0x%02X\n", Status, Connection_Handle, Reason);
  
  if(Status != 0){
    return;
  }  
    
  PRINTF("Disconnected from master ");
  PRINT_ADDRESS(central_info.address);
  PRINTF(", status 0x%02X, reason 0x%02X\r\n", Status, Reason);
  
  CentralInfoInit();
  
  APP_FLAG_CLEAR(CONNECTED);
  
}/* end hci_disconnection_complete_event() */

#define CONN_EVENT_SLAVE  0x02
#define CONN_EVENT_MASTER 0x05


void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)
{  
  uint16_t event_counter;
  uint32_t anchor_point32;
  uint64_t anchor_point64;
  
  if(Last_State == CONN_EVENT_SLAVE)
  {    
    aci_hal_get_anchor_point(central_info.conn_handle, &event_counter, &anchor_point32);
    
    anchor_point64 = NETCLOCK_convertPastSysTime32ToSysTime64(anchor_point32);
    
    NETCLOCK_SaveSyncEventOnSlave(anchor_point64, event_counter);
  }
}


void aci_gap_pairing_complete_event(uint16_t Connection_Handle,
                                    uint8_t Status,
                                    uint8_t Reason)
{
  PRINTF("Paired device\n");  
}

void aci_att_clt_read_resp_event(uint16_t Connection_Handle,
                                 uint16_t Event_Data_Length,
                                 uint8_t Attribute_Value[])
{
  switch(central_info.client_state){
  case READING_SYNC_CLOCK_CHAR:
    {    
      if(Event_Data_Length != 10)
      {
        PRINTF("Error while reading sync clock characteristic\n");
      }
      
      uint16_t event_counter = LE_TO_HOST_16(Attribute_Value);
      uint64_t vtime = LE_TO_HOST_64(Attribute_Value+2);
      
      printf("VClock set: event counter: %u, vtime: 0x%08X%08X\n", event_counter, (uint32_t)(vtime>>32),(uint32_t)vtime);
      
      uint8_t ret = NETCLOCK_SetNetTimeFromMaster(event_counter, vtime);
      
      printf("NETCLOCK_SetNetTimeFromMaster %d\r\n", ret);
      
      ret = NETCLOCK_GetCurrentNetTime(&vtime);  
      
      next_vtime_vtimer = (vtime/NETCLOCK_SECONDS + 2)*NETCLOCK_SECONDS;
      
      printf("next_vtime_vtimer 0x%08X%08X\n", (uint32_t)(next_vtime_vtimer>>32),(uint32_t)next_vtime_vtimer);
      
      NETCLOCK_StartTimer(next_vtime_vtimer, testCB);
    }
    
    break;    
	default:
		break;
  }  
}

void aci_att_clt_read_by_group_type_resp_event(uint16_t Connection_Handle,
                                           uint8_t Attribute_Data_Length,
                                           uint16_t Data_Length,
                                           uint8_t Attribute_Data_List[])
{  
  PRINTF_DBG2("aci_att_clt_read_by_group_type_resp_event, Connection Handle: 0x%04X\r\n", Connection_Handle);
  
  switch(central_info.client_state){
    
  case DISCOVERING_SERVICES:
    if(Attribute_Data_Length == 20){ // Only 128bit UUIDs
      for(int i = 0; i < Data_Length; i += Attribute_Data_Length){
        if(memcmp(&Attribute_Data_List[i+4],sync_service_uuid,16) == 0){
          memcpy(&central_info.sync_serv_start_handle, &Attribute_Data_List[i], 2);
          memcpy(&central_info.sync_serv_end_handle, &Attribute_Data_List[i+2], 2);
          PRINTF("Sync service handles: 0x%04X 0x%04X\r\n", central_info.sync_serv_start_handle, central_info.sync_serv_end_handle);
        }
      }
    }
    break;
  default:
	break;
  }
  
}

void print_uuid(uint8_t *uuid)
{
  for(int i = 0; i < 16; i++)
    PRINTF("%02X",uuid[i]);
}

void aci_att_clt_read_by_type_resp_event(uint16_t Connection_Handle,
                                     uint8_t Handle_Value_Pair_Length,
                                     uint16_t Data_Length,
                                     uint8_t Handle_Value_Pair_Data[])
{
  uint16_t handle;
  
  PRINTF_DBG2("aci_att_clt_read_by_type_resp_event, Connection Handle: 0x%04X\r\n", Connection_Handle);
  
  switch(central_info.client_state)
  {
  case DISCOVERING_SYNC_CHAR:
    for(int i = 0; i < Data_Length; i += Handle_Value_Pair_Length)
    {
      if(Handle_Value_Pair_Length == 21)
      { // 128-bit UUID
        handle = LE_TO_HOST_16(&Handle_Value_Pair_Data[i]);
        //print_uuid(&Handle_Value_Pair_Data[i+5]);
        if(memcmp(&Handle_Value_Pair_Data[i+5], vclock_char_uuid, 16) == 0)
        {
          central_info.sync_clock_handle = handle;
          PRINTF("Sync clock handle: 0x%04X\r\n", handle);
        }
      }
    }
    break;
	default:
		break;
  }
}

void aci_gatt_clt_error_resp_event(uint16_t Connection_Handle,
                               uint8_t Req_Opcode,
                               uint16_t Attribute_Handle,
                               uint8_t Error_Code)
{  
  PRINTF_DBG2("aci_gatt_clt_error_resp_event %04X %02X %04X %02X\n", Connection_Handle, Req_Opcode, Attribute_Handle, Error_Code);
}

void aci_gatt_clt_proc_complete_event(uint16_t Connection_Handle,
                                      uint8_t Error_Code)
{
  
  if(Error_Code != BLE_STATUS_SUCCESS){
    PRINTF_DBG2("Procedure terminated with error 0x%02X (0x%04X).\r\n", Error_Code, central_info.conn_handle);
    central_info.client_state = DONE;
    return;
  }
  
  switch(central_info.client_state){        
  case DISCOVERING_SERVICES:
    PRINTF_DBG2("Discovering services ended (0x%04X).\r\n", central_info.conn_handle);
    if(central_info.sync_serv_start_handle != 0)
      central_info.client_state = START_SYNC_CHAR_DISCOVERY;
    else
      central_info.client_state = DONE;
    break;    
  case DISCOVERING_SYNC_CHAR:
    PRINTF_DBG2("Discovering Chat Service characteristics ended (0x%04X).\r\n", central_info.conn_handle);
    if(central_info.sync_clock_handle != 0)
      central_info.client_state = READ_SYNC_CLOCK_CHAR;
    else 
      central_info.client_state = DONE;
    break;
  case READING_SYNC_CLOCK_CHAR:
    central_info.client_state = DONE;    
    break;
  default:
	break;
  }
}

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

void hci_le_data_length_change_event(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
  PRINTF("hci_le_data_length_change_event handle: 0x%04X, MaxTxOctets: %d, MaxTxTime: %d, MaxRxOctets: %d, MaxRxTime: %d.\r\n", Connection_Handle, MaxTxOctets, MaxTxTime, MaxRxOctets, MaxRxTime);
}

void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t RX_MTU)
{
  PRINTF("aci_att_exchange_mtu_resp_event, handle: 0x%04X, RX_MTU: %d \r\n", Connection_Handle, RX_MTU);
}
