/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : profile.c
* Author             : AMS - RF application team
* Description        : Simple profile that just connects two boards and uses LE
*                      Power Control features
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
#include "main.h"
#include "gap_profile.h"


/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#define DEBOUNCE_TIMEOUT_MS     300

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

#define INT(x)    ((int)(x))
#define FRACTIONAL(x)  (x>0)? ((int) (((x) - INT(x)) * 10)) : ((int) ((INT(x) - (x)) * 10))

#define BLE_RC_VERSION_STRING "1.0.0"

/* Private variables ---------------------------------------------------------*/
volatile uint16_t connection_handle = 0;
static VTIMER_HandleType advscanLEDTimerHandle;
static VTIMER_HandleType pathLossLEDTimerHandle;

enum{
  IDLE = 0,
  CONNECTING,
  CONNECTED,
} device_state;

/* Discoverable */
uint8_t adv_data[] = {0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED};

uint8_t path_loss_zone = 0;
const uint16_t path_loss_interval[3] = {PATHLOSS_LOW_LED_INTERVAL_MS, PATHLOSS_MID_LED_INTERVAL_MS, PATHLOSS_HIGH_LED_INTERVAL_MS};

/* The used PHY. It can be one between LE_1M_PHY and LE_CODED_PHY*/
#define USED_PHY     LE_1M_PHY
//#define USED_PHY     LE_CODED_PHY


/* Private function prototypes -----------------------------------------------*/
void SensorUpdateTimeoutCB(void *);
void DisconnectTimeoutCB(void *);
void AdvScanLEDTimeoutCB(void *param);
void PathLossLEDTimeoutCB(void *param);
void Configure_Advertising(void);
void Configure_Scanning(void);
/* Private functions ---------------------------------------------------------*/

/* Init remote control device */
uint8_t ProfileInit(void)
{
#if PERIPHERAL
  uint8_t role = GAP_PERIPHERAL_ROLE;
  uint8_t bdaddr[] = {BD_ADDR_SLAVE};
  uint8_t device_name[] = "Server";
#else
  uint8_t role = GAP_CENTRAL_ROLE;
  uint8_t bdaddr[] = {BD_ADDR_MASTER};
  uint8_t device_name[] = "Client";
#endif

  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t LE_Event_Mask[8]={
    HCI_LE_EVT_MASK_BYTE0_CONNECTION_COMPLETE,
    HCI_LE_EVT_MASK_BYTE1_ENHANCED_CONNECTION_COMPLETE,
    0x00,
    HCI_LE_EVT_MASK_BYTE3_PATH_LOSS_THRESHOLD,
    HCI_LE_EVT_MASK_BYTE4_TRANSMIT_POWER_REPORTING,
    0x00};

  // Only for debug
  //uint8_t val = 1;
  //ret = aci_hal_write_config_data(0xD3, 1, &val);

  /* Configure Public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_hal_write_config_data() failed: 0x%02x\r\n", ret);
    return ret;
  }

  /* Set the TX power to 6 dBm */
  aci_hal_set_tx_power_level(0, 31);

  ret = aci_hal_set_le_power_control(ENABLE, LE_1M_PHY, RSSI_TARGET_1M, RSSI_HYSTERESIS, 6, 3);
  ret |= aci_hal_set_le_power_control(ENABLE, LE_2M_PHY, RSSI_TARGET_2M, RSSI_HYSTERESIS, 6, 3);
  ret |=aci_hal_set_le_power_control(ENABLE, LE_CODED_PHY_S8, RSSI_TARGET_CODED_S8, RSSI_HYSTERESIS, 6, 3);
  ret |=aci_hal_set_le_power_control(ENABLE, LE_CODED_PHY_S2, RSSI_TARGET_CODED_S2, RSSI_HYSTERESIS, 6, 3);
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_hal_set_le_power_control() failed\r\n");
    return ret;
  }

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

  ret = hci_le_set_event_mask(LE_Event_Mask);
  if(ret)
  {
    PRINTF("hci_le_set_event_mask 0x%02X\n",ret);
  }

  /* Set the device name */
  ret = Gap_profile_set_dev_name(0, sizeof(device_name)-1, device_name);
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
                                               SC_IS_SUPPORTED,
                                               KEYPRESS_IS_NOT_SUPPORTED,
                                               7,
                                               16,
                                               USE_FIXED_PIN_FOR_PAIRING,
                                               123456);
   if(ret){
    PRINTF("aci_gap_set_authentication_requirement failed: 0x%02x\r\n", ret);
    return ret;
  }

  advscanLEDTimerHandle.callback = AdvScanLEDTimeoutCB;
  pathLossLEDTimerHandle.callback = PathLossLEDTimeoutCB;

#if PERIPHERAL
  Configure_Advertising();
#else
  Configure_Scanning();
#endif

  return BLE_STATUS_SUCCESS;
}

void Configure_Advertising(void)
{
  uint8_t ret;
  uint16_t adv_properties;

#if (USED_PHY == LE_1M_PHY)
  adv_properties = ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY;
#else
  adv_properties = ADV_PROP_CONNECTABLE;
#endif

  uint8_t peer_address[6] = {BD_ADDR_MASTER};

  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                              adv_properties,
                                              ADV_INTERVAL_MIN,
                                              ADV_INTERVAL_MAX,
                                              ADV_CH_ALL,
                                              PUBLIC_ADDR,peer_address,
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              USED_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              USED_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
  printf("Advertising configuration %02X\n", ret);

  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_set_advertising_data() failed: 0x%02x\r\n", ret);
  }
}

void Connect(void)
{
  uint8_t ret;

#if PERIPHERAL
  Advertising_Set_Parameters_t Advertising_Set_Parameters[1];

  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;

  //enable advertising
  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters);

  if (ret != BLE_STATUS_SUCCESS){
    printf ("Error in aci_gap_set_advertising_enable(): 0x%02x\r\n", ret);
    return;
  }
  else
    printf ("aci_gap_set_advertising_enable() --> SUCCESS\r\n");

  PRINTF("Start Advertising phy %d\r\n", USED_PHY);

  HAL_VTIMER_StartTimerMs(&advscanLEDTimerHandle, ADVSCAN_LED_INTERVAL_MS);

#else
  uint8_t phy_bit;

  tBDAddr bdaddr = {BD_ADDR_SLAVE};

#if USED_PHY == LE_1M_PHY
  phy_bit = LE_1M_PHY_BIT;
#else
  phy_bit = LE_CODED_PHY_BIT;
#endif

  ret = aci_gap_create_connection(phy_bit, PUBLIC_ADDR, bdaddr);

  if (ret != BLE_STATUS_SUCCESS)
  {
    printf("Error while starting connection: 0x%02x\r\n", ret);
    return;
  }

  printf("Connecting on phy %d...\n", USED_PHY);

  HAL_VTIMER_StartTimerMs(&advscanLEDTimerHandle, ADVSCAN_LED_INTERVAL_MS);
#endif
}

void Configure_Scanning(void)
{
  tBleStatus ret;

  ret = aci_gap_set_scan_configuration(DUPLICATE_FILTER_ENABLED, SCAN_ACCEPT_ALL, LE_1M_PHY_BIT, PASSIVE_SCAN, SCAN_INTERVAL, SCAN_WINDOW);
  printf("Scan configuration for LE_1M_PHY: 0x%02X\n", ret);

  ret = aci_gap_set_scan_configuration(DUPLICATE_FILTER_ENABLED, SCAN_ACCEPT_ALL, LE_CODED_PHY_BIT, PASSIVE_SCAN, SCAN_INTERVAL, SCAN_WINDOW);
  printf("Scan configuration for LE_CODED_PHY: 0x%02X\n", ret);

  ret = aci_gap_set_connection_configuration(LE_1M_PHY_BIT, CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, 0, SUPERVISION_TIMEOUT, CE_LENGTH, CE_LENGTH);
  printf("Connection configuration for LE_1M_PHY:  0x%02X\n", ret);

  ret = aci_gap_set_connection_configuration(LE_CODED_PHY_BIT, CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, 0, SUPERVISION_TIMEOUT, CE_LENGTH, CE_LENGTH);
  printf("Connection configuration for LE_CODED_PHY: 0x%02X\n", ret);
}

/* Remote Control State machine */
void APP_Tick(void)
{
  if(device_state == IDLE)
  {
    Connect();
    device_state = CONNECTING;
  }
}

void AdvScanLEDTimeoutCB(void *param)
{
  BSP_LED_Toggle(ADVSCAN_CONN_LED);
  HAL_VTIMER_StartTimerMs(&advscanLEDTimerHandle, ADVSCAN_LED_INTERVAL_MS);
}

void PathLossLEDTimeoutCB(void *param)
{
#if ENABLE_LOW_POWER_MODE
  uint32_t tmp = LL_PWR_GetPA6OutputinDEEPSTOP();
  if(tmp==LL_PWR_IOCFG_HIGH)
    LL_PWR_SetPA6OutputinDEEPSTOP(LL_PWR_IOCFG_LOW);
  else
    LL_PWR_SetPA6OutputinDEEPSTOP(LL_PWR_IOCFG_HIGH);
#else
  BSP_LED_Toggle(PATH_LOSS_LED);
#endif
  HAL_VTIMER_StartTimerMs(&pathLossLEDTimerHandle, path_loss_interval[path_loss_zone]);
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
  tBleStatus ret;

  if(Status != BLE_STATUS_SUCCESS)
    return;

  // Now connected.
  device_state = CONNECTED;

  connection_handle = Connection_Handle;

  printf("Connected\n");

  BSP_LED_On(ADVSCAN_CONN_LED);

  HAL_VTIMER_StopTimer(&advscanLEDTimerHandle);

  HAL_VTIMER_StartTimerMs(&pathLossLEDTimerHandle, PATHLOSS_HIGH_LED_INTERVAL_MS);

  /* Enable TX power and path loss reporting */

  ret = hci_le_set_transmit_power_reporting_enable(connection_handle, ENABLE, ENABLE);
  PRINTF("hci_le_set_transmit_power_reporting_enable 0x%02X\n",ret);

  ret = hci_le_set_path_loss_reporting_parameters(connection_handle, HIGH_THRESHOLD, HIGH_HYSTERESIS, LOW_THRESHOLD, LOW_HYSTERESIS, MIN_TIME);
  PRINTF("hci_le_set_path_loss_reporting_parameters 0x%02X\n",ret);

  ret = hci_le_set_path_loss_reporting_enable(connection_handle, ENABLE);
  PRINTF("hci_le_set_path_loss_reporting_enable 0x%02X\n",ret);

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
  printf("Disconnected\n");

  device_state = IDLE;

  BSP_LED_Off(ADVSCAN_CONN_LED);

  HAL_VTIMER_StopTimer(&pathLossLEDTimerHandle);

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
}

void hci_le_transmit_power_reporting_event(uint8_t Status,
                                           uint16_t Connection_Handle,
                                           uint8_t Reason,
                                           uint8_t PHY,
                                           int8_t Transmit_Power_Level,
                                           uint8_t Transmit_Power_Level_Flag,
                                           int8_t Delta)
{
  if(Reason==LOCAL_TX_POWER_CHANGE)
  {
    PRINTF("Local:\n%d dBm (%d dB)\n", Transmit_Power_Level, Delta);
  }
  else
  {
    PRINTF("\t\t\tRemote:\n\t\t\t%d dBm (%d dB)\n", Transmit_Power_Level, Delta);
  }
}

void hci_le_path_loss_threshold_event(uint16_t Connection_Handle,
                                      uint8_t Current_Path_Loss,
                                      uint8_t Zone_Entered)
{
  PRINTF("\t\t\t\t\t\tPATH LOSS:\n\t\t\t\t\t\t%d dB (zone %d)\n",
         Current_Path_Loss,
         Zone_Entered);

  path_loss_zone = Zone_Entered;

}

