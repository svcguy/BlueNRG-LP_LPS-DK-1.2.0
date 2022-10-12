/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : profile_central.c
* Author             : AMS - RF  Application team
* Description        : This file define the procedures for a Central
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

#define SCAN_INTERVAL       ((uint16_t)(100/0.625))     // 100 ms
#define SCAN_WINDOW         ((uint16_t)(100/0.625))     // 100 ms
#define CONN_INTERVAL_MIN   ((uint16_t)(350/1.25))      // 350 ms
#define CONN_INTERVAL_MAX   ((uint16_t)(350/1.25))      // 350 ms
#define SUPERVISION_TIMEOUT ((uint16_t)(2000/10))       // 2000 ms
#define CE_LENGTH           ((uint16_t)(0/0.625))       // 0 ms

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
}connection_info;
 
static uint64_t next_vtime_vtimer = 0;

uint8_t num_connected_slaves = 0;

void SlaveInit(uint8_t slave_index);


/* Private function prototypes -----------------------------------------------*/
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
  uint8_t role = 0;
  
  role |= GAP_CENTRAL_ROLE;
  
  /* Set the TX power to 0 dBm */
  aci_hal_set_tx_power_level(0, 24);
  
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
  ret = Gap_profile_set_dev_name(0, strlen(name), (uint8_t *) name);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error with Gap_profile_set_dev_name 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("Gap_profile_set_dev_name() --> SUCCESS\r\n");
  }
  
  aci_gap_set_io_capability(IO_CAP_NO_INPUT_NO_OUTPUT);
 
  ret = Add_Sync_Service();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in Add_Sync_Service 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("Add_Sync_Service() --> SUCCESS\r\n");
  }
  
  aci_gap_set_authentication_requirement(NO_BONDING, MITM_PROTECTION_NOT_REQUIRED, 0, 0, 7, 16, DONOT_USE_FIXED_PIN_FOR_PAIRING, 0);
  
  ret = aci_gap_set_scan_configuration(DUPLICATE_FILTER_ENABLED, SCAN_ACCEPT_ALL, LE_1M_PHY_BIT, ACTIVE_SCAN, SCAN_INTERVAL, SCAN_WINDOW);
  
  PRINTF("Scan configuration %02X\n", ret);
    
  ret = aci_gap_set_connection_configuration(LE_1M_PHY_BIT, CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, 0, SUPERVISION_TIMEOUT, CE_LENGTH, CE_LENGTH);
  
  PRINTF("Connection configuration %02X\n", ret);
  
  uint64_t vtime;
  
  ret = NETCLOCK_GetCurrentNetTime(&vtime);  

  next_vtime_vtimer = (vtime/NETCLOCK_SECONDS + 2)*NETCLOCK_SECONDS;
  
  NETCLOCK_StartTimer(next_vtime_vtimer, testCB);
  
  
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

static void DeviceStateMachine(void)
{
  if(!APP_FLAG(CONNECTED) && !APP_FLAG(SCANNING)){
    if(StartGeneralConnectionEstablishment() == BLE_STATUS_SUCCESS){
      APP_FLAG_SET(SCANNING);
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
     
     if(Role == 0x00) { // Master role 
       
       APP_FLAG_CLEAR(SCANNING);
       
       connection_info.conn_handle = Connection_Handle;       
       
       PRINTF("Connected with slave ");
       PRINT_ADDRESS(Peer_Address);
       PRINTF("\r\n");
       
       APP_FLAG_SET(CONNECTED);
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
  
  PRINTF("Disconnected from slave\n");
  
  APP_FLAG_CLEAR(CONNECTED);
  
}/* end hci_disconnection_complete_event() */

void hci_le_advertising_report_event(uint8_t Num_Reports,
                                     Advertising_Report_t Advertising_Report[])
{
  uint8_t AD_len, AD_type;
  uint8_t i = 0;
  tBleStatus ret;
  
  while(i < Advertising_Report[0].Data_Length){
    AD_len = Advertising_Report[0].Data[i];
    AD_type = Advertising_Report[0].Data[i+1];    
    if(AD_type == AD_TYPE_SERV_SOLICIT_128_BIT_UUID_LIST){
      // Search for Chat service UUID
      if(memcmp(&Advertising_Report[0].Data[i+2], sync_service_uuid, sizeof(sync_service_uuid))==0){
        // Device found!
        ret = aci_gap_create_connection(LE_1M_PHY_BIT, Advertising_Report[0].Address_Type, Advertising_Report[0].Address);        
        PRINTF("aci_gap_create_connection %02X\r\n", ret);
        return;
      }
    }
    i += AD_len+1;
  }
  
}

void hci_le_extended_advertising_report_event(uint8_t Num_Reports,
                                              Extended_Advertising_Report_t Extended_Advertising_Report[])
{
  Advertising_Report_t Advertising_Report;
  Advertising_Report.Address_Type = Extended_Advertising_Report[0].Address_Type;
  memcpy(Advertising_Report.Address, Extended_Advertising_Report[0].Address, 6);
  Advertising_Report.Data_Length = Extended_Advertising_Report[0].Data_Length;
  Advertising_Report.Data = Extended_Advertising_Report[0].Data;
  Advertising_Report.RSSI = Extended_Advertising_Report[0].RSSI;
  hci_le_advertising_report_event(1, &Advertising_Report);
}

void aci_gatt_srv_read_event(uint16_t Connection_Handle,
                             uint16_t Attribute_Handle,
                             uint16_t Data_Offset)
{
  uint16_t event_counter;
  uint8_t  charac_val[10];
  uint32_t anchor_point32;
  uint64_t anchor_point64;
  
  PRINTF_DBG2("aci_gatt_srv_read_event, connection: 0x%04X, att: 0x%04X\r\n", Connection_Handle, Attribute_Handle);
  
  if(Attribute_Handle == vclockCharHandle + 1)
  {    
    aci_hal_get_anchor_point(Connection_Handle, &event_counter, &anchor_point32);
    anchor_point64 = NETCLOCK_convertPastSysTime32ToSysTime64(anchor_point32);
    
    printf("aci_hal_get_anchor_point 0x%08X%08X %u\n",(uint32_t)(anchor_point64>>32),(uint32_t)anchor_point64,event_counter);
    
    HOST_TO_LE_16(charac_val, event_counter);
    HOST_TO_LE_64(charac_val+2, anchor_point64);    
    
    aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, 0,sizeof(charac_val), charac_val);
  }
  else {  
    aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, 0x02, 0, NULL);
  }
  
}
