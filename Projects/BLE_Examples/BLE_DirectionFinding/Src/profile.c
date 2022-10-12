/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : profile.c
* Author             : AMS - RF  Application team
* Date               : 27-May-2021
* Description        : This file implements a Bluetooth profile
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
#include "profile.h"
#include "gap_profile.h"
#include "bluenrg_lp_evb_com.h"
#include "miscutil.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define DEBUG

#define ADV_INTERVAL_MIN    ((uint32_t)(100/0.625))     // 100 ms
#define ADV_INTERVAL_MAX    ((uint32_t)(100/0.625))     // 100 ms
#define SCAN_INTERVAL       ((uint16_t)(400/0.625))     // 400 ms
#define SCAN_WINDOW         ((uint16_t)(100/0.625))     // 100 ms
#define CONN_INTERVAL_MIN   ((uint16_t)(20/1.25))       // 20 ms
#define CONN_INTERVAL_MAX   ((uint16_t)(20/1.25))       // 20 ms
#define SUPERVISION_TIMEOUT ((uint16_t)(1000/10))       // 1000 ms
#define CE_LENGTH           ((uint16_t)(20/0.625))      // 20 ms

#define BD_ADDR_MASTER      0xb1, 0xaa, 0x00, 0xE1, 0x80, 0x02
#define BD_ADDR_SLAVE       0xb2, 0xaa, 0x00, 0xE1, 0x80, 0x02

/* Parameters for Direction Finding */
#if CTE_TAG

#define CTE_TYPE_SUPPORT            (CTE_AOA_BIT|CTE_AOD_1us_BIT|CTE_AOD_2us_BIT) /* Support for any CTE type */
#define TX_SWITCHING_PATTERN_LENGTH sizeof(antenna_ids) /* Only used for AoD. */
#define ANTENNA_IDS                 {0,1,2,3}           /* Only used for AoD. */
#define ANDENNA_IDS_BITS            (0x03)              /* Bitmask for the pins for antenna switching to be configured */

#else /* CTE_LOCATOR */

#define CTE_SLOT_DURATION           CTE_SLOT_1us        /* Only used for AoA */
#define RX_SWITCHING_PATTERN_LENGTH sizeof(antenna_ids) /* Only used for AoA. */
#define ANTENNA_IDS                 {0,1,2,3}           /* Only used for AoA. */

#define CTE_REQ_INTERVAL            10                  /* Interval for CTE requests, in number of connection intervals */
#define MIN_CTE_LENGTH              10                   /* Minimum CTE length, in units of 8 us */
#define CTE_TYPE                    CTE_AOD_1us         /* Requested CTE type */

#endif

/* Private macros ------------------------------------------------------------*/
#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/
volatile int app_flags = SET_CONNECTABLE;
volatile uint16_t connection_handle = 0;
uint8_t antenna_ids[] = ANTENNA_IDS;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint8_t Profile_Init(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  
#if CTE_TAG
  uint8_t role = GAP_PERIPHERAL_ROLE;
  uint8_t bdaddr[] = {BD_ADDR_SLAVE};
  uint8_t name[] = "Tag";
  static uint8_t adv_data[] = {0x02,AD_TYPE_FLAGS,FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
                               0x04, AD_TYPE_COMPLETE_LOCAL_NAME, 'T','a','g'};
  
  /* Enable the output that controls the antenna switch. Enable also RF_ACTIVITY signal. */
  aci_hal_set_antenna_switch_parameters(ANDENNA_IDS_BITS, 0, 0, 1);
  
  
#else
  uint8_t LE_event_mask[8]={
    0x00,
    HCI_LE_EVT_MASK_BYTE1_ENHANCED_CONNECTION_COMPLETE,
    HCI_LE_EVT_MASK_BYTE2_CONNECTION_IQ_REPORT,
    0x00,
    0x00,
    0x00};
  uint8_t role = GAP_CENTRAL_ROLE;
  uint8_t bdaddr[] = {BD_ADDR_MASTER};
  uint8_t name[] = "Locator";
#endif 
  
  /* Configure Public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != BLE_STATUS_SUCCESS){
    PRINTF("Setting BD_ADDR failed: 0x%02x\r\n", ret);
    return ret;
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
  ret = aci_gap_init(role, 0x00, 0x09, PUBLIC_ADDR, &service_handle,  
                     &dev_name_char_handle, &appearance_char_handle);
  
#if CTE_LOCATOR  
  hci_le_set_event_mask(LE_event_mask);
#endif
  
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF ("Error in aci_gap_init() 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF ("aci_gap_init() --> SUCCESS\r\n");
  }

  /* Set the device name */
  ret = Gap_profile_set_dev_name(0, sizeof(name)-1, name);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF ("Error in Gap_profile_set_dev_name 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF ("Gap_profile_set_dev_name() --> SUCCESS\r\n");
  }
  
#if CTE_TAG
  
  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                              ADV_PROP_CONNECTABLE,
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
  PRINTF("Advertising configuration %02X\n", ret);

  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  
  PRINTF("Set advertising data %02X\n", ret);  
  
#else
  
  ret = aci_gap_set_scan_configuration(DUPLICATE_FILTER_ENABLED, SCAN_ACCEPT_ALL, LE_1M_PHY_BIT, PASSIVE_SCAN, SCAN_INTERVAL, SCAN_WINDOW);
  
  PRINTF("Scan configuration %02X\n", ret);
  
  ret = aci_gap_set_connection_configuration(LE_1M_PHY_BIT, CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, 0, SUPERVISION_TIMEOUT, CE_LENGTH, CE_LENGTH);
  
  PRINTF("Connection configuration %02X\n", ret);
  
#endif
  
  return BLE_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name  : Make_Connection.
* Description    : If the device is a Client create the connection. Otherwise puts
*                  the device in discoverable mode.
* Input          : none.
* Return         : none.
*******************************************************************************/
void Make_Connection(void)
{  
  tBleStatus ret;
  
#if CTE_LOCATOR
  
  tBDAddr bdaddr = {BD_ADDR_SLAVE}; 
  
  PRINTF("Connecting...\n");
  
  ret = aci_gap_create_connection(LE_1M_PHY_BIT, PUBLIC_ADDR, bdaddr);
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error while starting connection: 0x%04x\r\n", ret);
    Clock_Wait(100);        
  }
  
#else
  
  Advertising_Set_Parameters_t Advertising_Set_Parameters;
  
  Advertising_Set_Parameters.Advertising_Handle = 0;
  Advertising_Set_Parameters.Duration = 0;
  Advertising_Set_Parameters.Max_Extended_Advertising_Events = 0;
  
  ret = aci_gap_set_advertising_enable(ENABLE, 1, &Advertising_Set_Parameters); 
  if (ret != BLE_STATUS_SUCCESS)
    PRINTF("Error in aci_gap_set_advertising_enable(): 0x%02x\r\n", ret);
  else
    PRINTF("aci_gap_set_advertising_enable() --> SUCCESS\r\n");
  
#endif
}

/*******************************************************************************
* Function Name  : APP_Tick.
* Description    : Tick to run the application state machine.
* Input          : none.
* Return         : none.
*******************************************************************************/
void APP_Tick(void)
{
  if(APP_FLAG(SET_CONNECTABLE))
  {
    Make_Connection();
    APP_FLAG_CLEAR(SET_CONNECTABLE);
  }
  
}/* end APP_Tick() */


/* ***************** BlueNRG Stack Callbacks ********************************/

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
  tBleStatus ret;
  
  if(Status != BLE_STATUS_SUCCESS)
    return;
  
  connection_handle = Connection_Handle;
  
  APP_FLAG_SET(CONNECTED);
  
  printf("Connection, Status: 0x%02X\n", Status); 
  
#if CTE_TAG  
  ret = hci_le_set_connection_cte_transmit_parameters(Connection_Handle,
                                                      CTE_TYPE_SUPPORT,
                                                      TX_SWITCHING_PATTERN_LENGTH,
                                                      antenna_ids);
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Failed to set CTE parameters (0x%02X)\n", ret);
    return;
  }
  
  ret = hci_le_connection_cte_response_enable(Connection_Handle, ENABLE);
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Failed to enable CTE response (0x%02X)\n", ret);
  }
#else /* LOCATOR */
  
  ret = hci_le_set_connection_cte_receive_parameters(Connection_Handle, ENABLE,
                                                     CTE_SLOT_DURATION,
                                                     RX_SWITCHING_PATTERN_LENGTH,
                                                     antenna_ids);
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Failed to set CTE parameters (0x%02X)\n", ret);
    return;
  }
  
  ret = hci_le_connection_cte_request_enable(Connection_Handle, ENABLE,
                                             CTE_REQ_INTERVAL, MIN_CTE_LENGTH,
                                             CTE_TYPE);
  
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Failed to enable CTE requests (0x%02X)\n", ret);
  }
  
#endif
  
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
  APP_FLAG_CLEAR(CONNECTED);
  /* Make the device connectable again. */
  APP_FLAG_SET(SET_CONNECTABLE);
  
  printf("Disconnection, Reason: 0x%04X\n", Reason); 
  
}/* end hci_disconnection_complete_event() */


void hci_le_connection_iq_report_event(uint16_t Connection_Handle,
                                       uint8_t RX_PHY,
                                       uint8_t Data_Channel_Index,
                                       int16_t RSSI,
                                       uint8_t RSSI_Antenna_ID,
                                       uint8_t CTE_Type,
                                       uint8_t Slot_Durations,
                                       uint8_t Packet_Status,
                                       uint16_t Connection_Event_Counter,
                                       uint8_t Sample_Count,
                                       Samples_t Samples[])
{
  printf("IQ samples: ");
  for(int i = 0; i < Sample_Count; i++)
  {
    printf("(%d,%d)",Samples[i].I_Sample,Samples[i].Q_Sample);
  }
  printf("\n");
}

void hci_le_cte_request_failed_event(uint8_t Status,
                                     uint16_t Connection_Handle)
{
  printf("CTE Request failed 0x%02X.\n", Status);
  
}

