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
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "rc.h"
#include "gatt_db.h"
#include "app_state.h"
#include "gap_profile.h"

#ifndef SENSOR_EMULATION /* User Real sensor: lps22hh (pressure and temperature) */
#include "bluenrg_lp_evb_config.h"
#endif 

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#define ADV_INTERVAL_MIN_MS     1000    
#define ADV_INTERVAL_MAX_MS     1000

#define TEMPERATURE_UPDATE_RATE  1000 /* 1000ms */
#define DISCONNECTION_TIMEOUT    60000 /* 60 sec */

#define PUBLIC_ADDRESS 0x01, 0x00, 0x80, 0xE1, 0x80, 0x02

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
static uint8_t sensor_update_timer_expired = FALSE;
volatile int app_flags = START_ADVERTISING;
volatile uint16_t connection_handle = 0;

static VTIMER_HandleType sensorTimerHandle;
static VTIMER_HandleType disconnectTimerHandle;

static uint8_t scan_resp_data[] = {0x05,AD_TYPE_COMPLETE_LOCAL_NAME,'N','o','d','e'}; 

/* Discoverable */
#define TEMP_OFFSET 8
uint8_t adv_data[2][10] =
{
  {0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,0x06,AD_TYPE_MANUFACTURER_SPECIFIC_DATA,0x30,0x00,0x05,0xFF,0xFF},
  {0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,0x06,AD_TYPE_MANUFACTURER_SPECIFIC_DATA,0x30,0x00,0x05,0xFF,0xFF}
};
uint8_t adv_data_index = 0;

#ifndef SENSOR_EMULATION /* User Real sensor: LPS22HH (pressure and temperature) */
 /* LPS22HH initialization */
lps22hh_ctx_t pressureHandle;

#endif
  
static Advertising_Set_Parameters_t Advertising_Set_Parameters[1]; 

/* Private function prototypes -----------------------------------------------*/
void SensorUpdateTimeoutCB(void *);
void DisconnectTimeoutCB(void *);
/* Private functions ---------------------------------------------------------*/

/* Init Tempereture sensor */
void Init_Temperature_Sensor(void)
{
#ifndef SENSOR_EMULATION /* User Real sensor: LPS22HH (it has pressure and temperature sensors) */
  /* LPS22HH initialization */
  uint8_t rst;
  
   /* Initialize the handle of the LPS22HH driver */
  pressureHandle.write_reg = BSP_I2C_Write;
  pressureHandle.read_reg = BSP_I2C_Read;
  
  /* Inizialize the SPI */
  BSP_I2C_Init();
  
  /* Restore default configuration */
  lps22hh_reset_set(&pressureHandle, PROPERTY_ENABLE);
  do {
    lps22hh_reset_get(&pressureHandle, &rst);
  } while (rst);
  
  /*  Enable Block Data Update */
  lps22hh_block_data_update_set(&pressureHandle, PROPERTY_ENABLE);
  
  /* Set Output Data Rate */
  lps22hh_data_rate_set(&pressureHandle, LPS22HH_1_Hz_LOW_NOISE);
  
#endif 
}

/* Update temperature data in advertising packets */
void Update_Temperature(void)
{
  float temperature_degC;
  uint8_t status = 1;
  
#ifdef SENSOR_EMULATION /* User Emulated Data */
   temperature_degC = 26 + ((uint64_t)rand()*15)/RAND_MAX;
#else
    axis1bit16_t data_raw_temperature;
    lps22hh_reg_t reg;
  
    /* Read output only if new value is available */
    lps22hh_read_reg(&pressureHandle, LPS22HH_STATUS, (uint8_t *)&reg, 1);
    status = reg.status.t_da;
    
    if (status) {
      lps22hh_temperature_raw_get(&pressureHandle, data_raw_temperature.u8bit);
      temperature_degC = lps22hh_from_lsb_to_celsius(data_raw_temperature.i16bit);
      
      //printf("temperature [degC]:%6.2f\r\n", temperature_degC);
    }
#endif 
   if (status)
   {
      HOST_TO_LE_16(adv_data[adv_data_index]+TEMP_OFFSET, (int16_t)temperature_degC);
      status = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data[0]), adv_data[adv_data_index]);
      if(status != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_set_advertising_data() failed: 0x%02x\r\n", status);
      }
      if(++adv_data_index == 2)
        adv_data_index=0;
      PRINTF("Updated temperature: %d.%d C deg\n", INT(temperature_degC), FRACTIONAL(temperature_degC));
   }
}

/* Init remote control device */
uint8_t RC_DeviceInit(void)
{
  uint8_t bdaddr[] = {PUBLIC_ADDRESS};
  uint8_t device_name[]={'N', 'o', 'd', 'e'};
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  
  /* Init temperature sensor */
  Init_Temperature_Sensor();  

  /* Configure Public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_hal_write_config_data() failed: 0x%02x\r\n", ret);
    return ret;
  }
  
  /* Set the TX power to 0 dBm */
  aci_hal_set_tx_power_level(0, 24);
  
  /* GATT Init */
  ret = aci_gatt_srv_init();    
  if(ret){
    PRINTF("aci_gatt_srv_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
      
  /* GAP Init */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07,  0x00, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if(ret){
    PRINTF("aci_gap_Init() failed: 0x%02x\r\n", ret);
    return ret;
  }
      
  /* Update device name */
  Gap_profile_set_dev_name(0, sizeof(device_name), device_name);
 
#if ENABLE_SECURITY
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
#endif 
  
  /* Add the remote control service and characteristics */
  ret = Add_RC_Service();
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("ADD_RC_Service() failed: 0x%02x\r\n", ret);
    return ret;
  }
  
  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                              ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                              (ADV_INTERVAL_MIN_MS*1000)/625, 
                                              (ADV_INTERVAL_MAX_MS*1000)/625,
                                              ADV_CH_ALL,
                                              PUBLIC_ADDR,NULL,
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
  
  printf("Advertising configuration 0x%02X\n", ret);
  
     
  /*  Set the scan response data */
  ret = aci_gap_set_scan_response_data(0,sizeof(scan_resp_data),scan_resp_data);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_set_scan_response_data() failed: 0x%02x\r\n", ret);
  }
  
  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data[0]), adv_data[adv_data_index]);
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_set_advertising_data() failed: 0x%02x\r\n", ret);
  }
  
  sensorTimerHandle.callback = SensorUpdateTimeoutCB; 
  disconnectTimerHandle.callback = DisconnectTimeoutCB;
    
  return BLE_STATUS_SUCCESS;
}

/* Make the remote control device discoverable */
void Start_Advertising(void)
{  
  uint8_t ret;  
 
  /* Update temperature */
  Update_Temperature();

  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;
  
  //enable advertising
  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 

  if (ret != BLE_STATUS_SUCCESS)
    printf ("Error in aci_gap_set_advertising_enable(): 0x%02x\r\n", ret);
  else
    printf ("aci_gap_set_advertising_enable() --> SUCCESS\r\n");
  PRINTF("Start Advertising \r\n");
}


/* Remote Control State machine */
void APP_Tick(void)
{
  if(APP_FLAG(START_ADVERTISING)){
    sensor_update_timer_expired = TRUE;
    Start_Advertising();
    APP_FLAG_CLEAR(START_ADVERTISING);
  }
    
  if(sensor_update_timer_expired){
    
    sensor_update_timer_expired = FALSE;
    HAL_VTIMER_StartTimerMs(&sensorTimerHandle, TEMPERATURE_UPDATE_RATE);
    
    Update_Temperature();
  }    
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
  if(Status != BLE_STATUS_SUCCESS)
    return;
  
  APP_FLAG_SET(CONNECTED); 
  connection_handle = Connection_Handle;
    
  /* Start a timer to disconnect the link after a while. */    
  sensor_update_timer_expired = FALSE;
  HAL_VTIMER_StopTimer(&sensorTimerHandle);
  HAL_VTIMER_StartTimerMs(&disconnectTimerHandle, DISCONNECTION_TIMEOUT); 
    
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
  APP_FLAG_SET(START_ADVERTISING);

  /* Stop the timer when a disconnection event occurs. */
  HAL_VTIMER_StopTimer(&disconnectTimerHandle);
}/* end hci_disconnection_complete_event() */


/* This function is called when a DB attribute is modified */
void aci_gatt_srv_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  
}

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

void SensorUpdateTimeoutCB(void *param)
{
  sensor_update_timer_expired = TRUE;
}

void DisconnectTimeoutCB(void *param)
{
  aci_gap_terminate(connection_handle,0x13); /* 0x13: Remote User Terminated Connection */  
}
