/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : rc.c
* Author             : AMS - RF application team
* Version            : V1.0.0
* Date               : 11-March-2019
* Description        : Remote Control configuration function and state machines
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
#include <stdlib.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "bluenrg_lp_evb_button.h"
#include "rc.h"
#include "gatt_db.h"
#include "app_state.h"
#include "gap_profile.h"
#include "gatt_profile.h"

uint8_t button1_pressed = FALSE, button2_pressed = FALSE;

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#define DEBOUNCE_TIMEOUT_MS     300
#define STATS_INTERVAL_MS       10000


/* Enable debug printf */
#ifndef DEBUG
#define DEBUG 1
#endif

/* Private macros ------------------------------------------------------------*/
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define BLE_RC_VERSION_STRING "1.0.0" 

/* Private variables ---------------------------------------------------------*/
volatile int app_flags = SET_CONNECTABLE;
volatile uint16_t connection_handle = 0;
static uint8_t debounce_timeout_occurred = TRUE;
static VTIMER_HandleType debounce_timer;
static VTIMER_HandleType scanningLEDTimerHandle;
static VTIMER_HandleType writeTimerHandle;
static VTIMER_HandleType statsTimerHandle;
static uint8_t phy = LE_1M_PHY;

static llc_conn_per_statistic_st per_statistic_ptr;
static uint16_t CRC_errs_perc;
static uint16_t missed_evts_perc;
static uint8_t packet_err_rate;

  
#if DO_NOT_USE_VTIMER_CB
  static uint32_t last_time;  
#endif
  
/* Private function prototypes -----------------------------------------------*/
void ScanningLEDTimeoutCB(void *param);
void writeTimeoutCB(void *param);
void statsTimeoutCB(void *param);
void toggle_LED(void);
/* Private functions ---------------------------------------------------------*/
void DebounceTimeoutCB(void *param);

/* Init remote control device */
uint8_t RC_DeviceInit(void)
{
  
  uint8_t role = GAP_CENTRAL_ROLE;
  uint8_t bdaddr[] = {BD_ADDR_MASTER};
  uint8_t device_name[]={'N', 'o', 'd', 'e'};
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

  /* Configure Public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_hal_write_config_data() failed: 0x%02x\r\n", ret);
    return ret;
  }
  
  /* Set the TX power */
  aci_hal_set_tx_power_level(0, OUTPUT_POWER_LEVEL);
  
  /* GATT Init */
  ret = aci_gatt_srv_init();    
  if(ret){
    PRINTF("aci_gatt_srv_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
      
  /* GAP Init */
  ret = aci_gap_init(role, 0, 0x07,  0x00, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if(ret){
    PRINTF("aci_gap_Init() failed: 0x%02x\r\n", ret);
    return ret;
  }
      
  /* Set the device name */
  ret = Gap_profile_set_dev_name(0, sizeof(device_name), device_name);
  if(ret){
    PRINTF("Gap_profile_set_dev_name() failed: 0x%02x\r\n", ret);
    return ret;
  }
  
  /* Set the IO capability */
  ret = aci_gap_set_io_capability(IO_CAP_DISPLAY_ONLY);
  if(ret){
    PRINTF("aci_gap_set_io_capability() failed: 0x%02x\r\n", ret);
    return ret;
  }

  ret = aci_gap_set_authentication_requirement(BONDING,
                                               MITM_PROTECTION_REQUIRED,
                                               SC_IS_NOT_SUPPORTED,
                                               KEYPRESS_IS_NOT_SUPPORTED,
                                               7, 
                                               16,
                                               USE_FIXED_PIN_FOR_PAIRING,
                                               123456);
   if(ret){
    PRINTF("aci_gap_set_authentication_requirement failed: 0x%02x\r\n", ret);
    return ret;
  }
  
  ret = aci_gap_set_scan_configuration(DUPLICATE_FILTER_ENABLED, SCAN_ACCEPT_ALL, LE_1M_PHY_BIT, PASSIVE_SCAN, SCAN_INTERVAL, SCAN_WINDOW);
  printf("Scan configuration for LE_1M_PHY: 0x%02X\n", ret);
  
  ret = aci_gap_set_scan_configuration(DUPLICATE_FILTER_ENABLED, SCAN_ACCEPT_ALL, LE_CODED_PHY_BIT, PASSIVE_SCAN, SCAN_INTERVAL, SCAN_WINDOW);  
  printf("Scan configuration for LE_CODED_PHY: 0x%02X\n", ret);
  
  ret = aci_gap_set_connection_configuration(LE_1M_PHY_BIT, CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, 0, SUPERVISION_TIMEOUT, CE_LENGTH, CE_LENGTH);
  printf("Connection configuration for LE_1M_PHY:  0x%02X\n", ret);
  
  ret = aci_gap_set_connection_configuration(LE_CODED_PHY_BIT, CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, 0, SUPERVISION_TIMEOUT, CE_LENGTH, CE_LENGTH);  
  printf("Connection configuration for LE_CODED_PHY: 0x%02X\n", ret);
  
  
  debounce_timer.callback = DebounceTimeoutCB;
  scanningLEDTimerHandle.callback = ScanningLEDTimeoutCB;
  writeTimerHandle.callback = writeTimeoutCB;
  statsTimerHandle.callback = statsTimeoutCB;
    
  return BLE_STATUS_SUCCESS;
}

void Connect(void)
{  
  tBleStatus ret;
  uint8_t phy_bit;
  
  tBDAddr bdaddr = {BD_ADDR_SLAVE}; 
  
  if(phy == LE_1M_PHY){
    phy_bit = LE_1M_PHY_BIT;
    BSP_LED_Off(LONG_RANGE_LED);    
  }
  else {
    phy_bit = LE_CODED_PHY_BIT;
    BSP_LED_On(LONG_RANGE_LED);
  }    
  
  ret = aci_gap_create_connection(phy_bit, PUBLIC_ADDR, bdaddr);
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf("Error while starting connection: 0x%02x\r\n", ret);
    return;
  }
  
  printf("Connecting on phy %d...\n", phy);
  
  HAL_VTIMER_StartTimerMs(&scanningLEDTimerHandle, ADVSCAN_LED_INTERVAL_MS);  
}

void CancelConnect(void)
{
  aci_gap_terminate_proc(GAP_DIRECT_CONNECTION_ESTABLISHMENT_PROC);
  HAL_VTIMER_StopTimer(&scanningLEDTimerHandle);
  BSP_LED_Off(ADVSCAN_CONN_LED);
}

/* Remote Control State machine */
void APP_Tick(void)
{  
  if(APP_FLAG(SET_CONNECTABLE))
  {
    Connect();
    APP_FLAG_CLEAR(SET_CONNECTABLE);
  }
  
  if(button1_pressed && debounce_timeout_occurred){
    
    button1_pressed = FALSE;
    debounce_timeout_occurred = FALSE;
    HAL_VTIMER_StartTimerMs(&debounce_timer, DEBOUNCE_TIMEOUT_MS);
    
    if(APP_FLAG(CONNECTED)){
      if(phy == LE_1M_PHY){
        hci_le_set_phy(connection_handle, 0, LE_CODED_PHY_BIT, LE_CODED_PHY_BIT, 2); // S = 8 
        PRINTF("Switch to LE CODED PHY\n");
      }
      else {
        hci_le_set_phy(connection_handle, 0, LE_1M_PHY_BIT, LE_1M_PHY_BIT, 2); // S = 8 
        PRINTF("Switch to LE 1M PHY\n");
      }
    }
    else{
      CancelConnect();
      if(phy == LE_1M_PHY){
        phy = LE_CODED_PHY;
      }
      else {
        phy = LE_1M_PHY;
      }
    }
  }
  
#if AUTO_TOGGLE_LED == 0
  
  if(APP_FLAG(CONNECTED) && button2_pressed && debounce_timeout_occurred){
    
    button2_pressed = FALSE;
    debounce_timeout_occurred = FALSE;
    HAL_VTIMER_StartTimerMs(&debounce_timer, DEBOUNCE_TIMEOUT_MS);    

    toggle_LED();

  }
#else
  
#if DO_NOT_USE_VTIMER_CB
  if(APP_FLAG(CONNECTED) && HAL_VTIMER_DiffSysTimeMs(HAL_VTIMER_GetCurrentSysTime(), last_time) > DEBOUNCE_TIMEOUT_MS){
    toggle_LED();
    last_time = HAL_VTIMER_GetCurrentSysTime();
  }
#endif
  
#endif
  
}

void toggle_LED(void)
{
  tBleStatus ret;
  static uint8_t val = 1<<CONTROL_LED;
  
  if(val==0){
    BSP_LED_On(CONTROL_LED);
    val = 1<<CONTROL_LED;
  }
  else {
    BSP_LED_Off(CONTROL_LED);
    val = 0;
  }
  
  ret = aci_gatt_clt_write(connection_handle, 0x0012, 1, &val);
  
  PRINTF("Write 0x%02X to handle 0x0012, 0x%02X\n", val, ret);
}

void statsTimeoutCB(void *param)
{
  CRC_errs_perc = (uint32_t)per_statistic_ptr.num_crc_err*100/per_statistic_ptr.num_pkts;
  missed_evts_perc = (uint32_t)per_statistic_ptr.num_miss_evts*100/per_statistic_ptr.num_evts;
  packet_err_rate = (uint32_t)(per_statistic_ptr.num_miss_evts+per_statistic_ptr.num_crc_err)*100/(per_statistic_ptr.num_evts+per_statistic_ptr.num_miss_evts);
  
  PRINTF("- CRC errs = %d%%\n- Missed evts = %d%%\n", CRC_errs_perc, missed_evts_perc);

  PRINTF("- PER = %d%%\n", packet_err_rate);
  
  llc_conn_per_statistic(connection_handle, &per_statistic_ptr);

  HAL_VTIMER_StartTimerMs(&statsTimerHandle, STATS_INTERVAL_MS);
}

void DebounceTimeoutCB(void *param)
{
  debounce_timeout_occurred = TRUE;
  button1_pressed = FALSE;
  button2_pressed = FALSE;
}


void ScanningLEDTimeoutCB(void *param)
{
  BSP_LED_Toggle(ADVSCAN_CONN_LED);
  HAL_VTIMER_StartTimerMs(&scanningLEDTimerHandle, ADVSCAN_LED_INTERVAL_MS);
}

void writeTimeoutCB(void *param)
{
  toggle_LED();
  HAL_VTIMER_StartTimerMs(&writeTimerHandle, WRITE_INTERVAL_MS);
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/* This function is called when there is a LE Connection Complete event. */
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
  if(Status == BLE_ERROR_UNKNOWN_CONNECTION_ID)
  {
    /* A connection has been cancelled. Restart */
    APP_FLAG_SET(SET_CONNECTABLE);
    return;
  }
  else if(Status != 0)
    return;
  
  APP_FLAG_SET(CONNECTED); 
  connection_handle = Connection_Handle;
  
  printf("Connected\n");
  
  BSP_LED_On(ADVSCAN_CONN_LED);
  
  HAL_VTIMER_StopTimer(&scanningLEDTimerHandle);
  
  llc_conn_per_statistic(connection_handle, &per_statistic_ptr);

  HAL_VTIMER_StartTimerMs(&statsTimerHandle, STATS_INTERVAL_MS);

#if AUTO_TOGGLE_LED  
    
#if DO_NOT_USE_VTIMER_CB
    last_time = HAL_VTIMER_GetCurrentSysTime();
#else
    HAL_VTIMER_StartTimerMs(&writeTimerHandle, WRITE_INTERVAL_MS);
#endif
    
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

/* This function is called when the peer device get disconnected. */
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  APP_FLAG_CLEAR(CONNECTED);
  /* Make the device connectable again. */
  APP_FLAG_SET(SET_CONNECTABLE);
  
  printf("Disconnected\n");
  
  BSP_LED_Off(ADVSCAN_CONN_LED);
  BSP_LED_Off(CONTROL_LED);
  
  HAL_VTIMER_StopTimer(&statsTimerHandle);
  
#if AUTO_TOGGLE_LED  
#if !DO_NOT_USE_VTIMER_CB
    HAL_VTIMER_StopTimer(&writeTimerHandle);
#endif
#endif
    
}/* end hci_disconnection_complete_event() */


/* This function is called when there is a L2CAP_CONN_UPDATE_RESP_Event vendor specific event. */ 
void aci_l2cap_connection_update_resp_event(uint16_t Connection_Handle,
                                            uint16_t Result)
{
  if(Result) {
    PRINTF("> Connection parameters rejected.\n");
  } else  {
    PRINTF("> Connection parameters accepted.\n");
  }
}

void hci_le_phy_update_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t TX_PHY,
                                      uint8_t RX_PHY)
{
  PRINTF("PHY changed: %d %d\n", TX_PHY, RX_PHY);
  if(TX_PHY == LE_CODED_PHY && RX_PHY == LE_CODED_PHY){
    BSP_LED_On(LONG_RANGE_LED);
    phy = LE_CODED_PHY;
  }
  else if(TX_PHY == LE_1M_PHY && RX_PHY == LE_1M_PHY){
    BSP_LED_Off(LONG_RANGE_LED);
    phy = LE_1M_PHY;
  }
  else {
    PRINTF("Unexpected\n");
    BSP_LED_Off(LONG_RANGE_LED);
  }

  HAL_VTIMER_StopTimer(&statsTimerHandle);
  CRC_errs_perc = 0.0;
  missed_evts_perc = 0.0;
  packet_err_rate = 0.0;
  
  llc_conn_per_statistic(connection_handle, &per_statistic_ptr);
  HAL_VTIMER_StartTimerMs(&statsTimerHandle, STATS_INTERVAL_MS);  

}

