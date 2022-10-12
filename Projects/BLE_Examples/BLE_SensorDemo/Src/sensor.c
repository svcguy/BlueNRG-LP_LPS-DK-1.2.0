/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
 * File Name          : sensor.c
 * Author             : AMS - RF Application team
 * Version            : V2.0.0
 * Date               : 26-February-2019
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
#include "ble_const.h" 
#include "bluenrg_lp_stack.h"
#include "OTA_btl.h" 

#include "gatt_db.h"
#include "rf_driver_hal_vtimer.h"
#include "gap_profile.h"
#include "clock.h"

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

#define UPDATE_CONN_PARAM 1 //0
#define ADV_INTERVAL_MIN_MS  200
#define ADV_INTERVAL_MAX_MS  200

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t set_connectable = 1;
uint16_t connection_handle = 0;
uint8_t connInfo[20];
int connected = FALSE;

static VTIMER_HandleType sensorTimerHandle;

volatile uint8_t request_free_fall_notify = FALSE; 

BOOL sensor_board = FALSE; // It is True if sensor board has been detected

#if UPDATE_CONN_PARAM
#define UPDATE_TIMER 2 //TBR
static VTIMER_HandleType l2cap_TimerHandle;
int l2cap_request_sent = FALSE;
static uint8_t l2cap_req_timer_expired = FALSE; 
#endif

#define SENSOR_TIMER 1
#define ACC_UPDATE_INTERVAL_MS 200
static uint8_t sensorTimer_expired = FALSE;

#ifndef SENSOR_ACCELEROMETER_EMULATION

/* We use the real accelerometer on BlueNRG-LP Kit */
lsm6dsox_ctx_t inertialHandle;

#endif 

#ifndef SENSOR_PRESSURE_TEMPERATURE_EMULATION

/* We use the real pressure-temperature sensor on BlueNRG-LP Kit */

/* LPS22HH initialization */
lps22hh_ctx_t pressureHandle;

#endif 
  
/* Private function prototypes -----------------------------------------------*/
void SensorUpdateTimeoutCB(void *);

static Advertising_Set_Parameters_t Advertising_Set_Parameters[1];

/* Private functions ---------------------------------------------------------*/

#ifndef SENSOR_ACCELEROMETER_EMULATION
/*******************************************************************************
 * Function Name  : Init_Accelerometer.
 * Description    : Init LIS331DLH accelerometer.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Init_Accelerometer(void)
{
  uint8_t rst;
  
  lsm6dsox_pin_int1_route_t int_1_reg;
  /*
   * Uncomment if interrupt generation on Free Fall INT2 pin.
   */
  //lsm6dsox_int2_route_t int_2_reg;
  
  
  /* Initialize the handle of the LSM6DSO driver */
  inertialHandle.write_reg = BSP_SPI_Write;
  inertialHandle.read_reg = BSP_SPI_Read;
  
  /* Inizialize the SPI */
  BSP_SPI_Init();
  
  /* Restore default configuration */
  lsm6dsox_reset_set(&inertialHandle, PROPERTY_ENABLE);
  do {
    lsm6dsox_reset_get(&inertialHandle, &rst);
  } while (rst);
  
  /* Disable I3C interface. */
  lsm6dsox_i3c_disable_set(&inertialHandle, LSM6DSOX_I3C_DISABLE);
  
  /* Enable Block Data Update */
  lsm6dsox_block_data_update_set(&inertialHandle, PROPERTY_ENABLE);
  
  /* Set full scale */  
  lsm6dsox_xl_full_scale_set(&inertialHandle, LSM6DSOX_2g);
  lsm6dsox_gy_full_scale_set(&inertialHandle, LSM6DSOX_2000dps);
  
  /* Set Output Data Rate for Acc and Gyro */
  lsm6dsox_xl_data_rate_set(&inertialHandle, LSM6DSOX_XL_ODR_12Hz5);
  lsm6dsox_gy_data_rate_set(&inertialHandle, LSM6DSOX_GY_ODR_12Hz5);
  
  /* Enable interrupt generation on Free Fall INT1 pin. */
  lsm6dsox_int_notification_set(&inertialHandle, LSM6DSOX_ALL_INT_LATCHED);
  
  /* Set Free Fall duration to 3 and 6 samples event duration. */
  lsm6dsox_ff_dur_set(&inertialHandle, 0);
  lsm6dsox_ff_threshold_set(&inertialHandle, LSM6DSOX_FF_TSH_156mg);

  
  lsm6dsox_pin_int1_route_get(&inertialHandle, &int_1_reg);
  int_1_reg.md1_cfg.int1_ff = PROPERTY_ENABLE;
  lsm6dsox_pin_int1_route_set(&inertialHandle, &int_1_reg);
  
  /* IRQ pin setting */
  BSP_SPI_GpioInt_Init();
  
}
#endif 

#ifndef SENSOR_PRESSURE_TEMPERATURE_EMULATION

/*******************************************************************************
 * Function Name  : Init_Pressure_Temperature_Sensor.
 * Description    : Init LPS22HH pressure and temperature sensor.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Init_Pressure_Temperature_Sensor(void)
{  
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

}

#endif

/*******************************************************************************
 * Function Name  : Sensor_DeviceInit.
 * Description    : Init the device sensors.
 * Input          : None.
 * Return         : Status.
 *******************************************************************************/
uint8_t  Sensor_DeviceInit()
{
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t device_name[] = {'B', 'l', 'u', 'e', 'N', 'R', 'G'};
  static uint8_t adv_data[] = {0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
                              8, AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G'};

  /* Set the TX power 0 dBm */
  aci_hal_set_tx_power_level(0, 24);
  
  /* GATT Init */
  ret = aci_gatt_srv_init();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gatt_srv_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
  
  /* GAP Init */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, STATIC_RANDOM_ADDR, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("aci_gap_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
 
  /* Update device name */
  Gap_profile_set_dev_name(0, sizeof(device_name), device_name);
  
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
  
  ret = aci_gap_set_advertising_configuration( 0, GAP_MODE_GENERAL_DISCOVERABLE,
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
  PRINTF("Advertising configuration 0x%02X\n", ret);
  
  ret = aci_gap_set_advertising_data( 0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  
  PRINTF("Set advertising data 0x%02X\n", ret);  
  
  PRINTF("BLE Stack Initialized with SUCCESS\n");

#ifndef SENSOR_ACCELEROMETER_EMULATION /* User Real accelerometer sensors */
  Init_Accelerometer();
#endif
  
#ifndef SENSOR_PRESSURE_TEMPERATURE_EMULATION
  Init_Pressure_Temperature_Sensor(); /* User Real temperature/pressure sensor */
#endif   


  /* Add ACC service and Characteristics */
  ret = Add_Acc_Service();
  if(ret == BLE_STATUS_SUCCESS) {
    PRINTF("Acceleration service added successfully.\n");
  } else {
    PRINTF("Error while adding Acceleration service: 0x%02x\r\n", ret);
    return ret;
  }

  /* Add Environmental Sensor Service */
  ret = Add_Environmental_Sensor_Service();
  if(ret == BLE_STATUS_SUCCESS) {
    PRINTF("Environmental service added successfully.\n");
  } else {
    PRINTF("Error while adding Environmental service: 0x%04x\r\n", ret);
    return ret;
  }

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT     
  ret = OTA_Add_Btl_Service();
  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("OTA service added successfully.\n");
  else
    PRINTF("Error while adding OTA service.\n");
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 
  
  /* Start the Sensor Timer */
  sensorTimerHandle.callback = SensorUpdateTimeoutCB;  
  ret = HAL_VTIMER_StartTimerMs(&sensorTimerHandle, ACC_UPDATE_INTERVAL_MS);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("HAL_VTIMER_StartTimerMs() failed; 0x%02x\r\n", ret);
    return ret;
  } else {
    sensorTimer_expired = FALSE;
  }

  return BLE_STATUS_SUCCESS;
}

void SensorUpdateTimeoutCB(void *param)
{
  sensorTimer_expired = TRUE;
}

#if UPDATE_CONN_PARAM
void l2cap_UpdateTimeoutCB(void *param)
{
  l2cap_req_timer_expired = TRUE;
}
#endif

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
  ret = aci_gap_set_scan_response_data(0,18,BTLServiceUUID4Scan);
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("aci_gap_set_scan_response_data() failed: 0x%02x\r\n",ret);
    BSP_LED_On(BSP_LED3);  
  }
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 
  PRINTF("Set General Discoverable Mode.\n");
  
  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;
  
  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 
  
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("aci_gap_set_advertising_enable() failed: 0x%02x\r\n",ret);
    BSP_LED_On(BSP_LED3);  
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
#if UPDATE_CONN_PARAM    
  uint8_t ret = 0; 
#endif 
  /* Make the device discoverable */
  if(set_connectable) {
    Set_DeviceConnectable();
    set_connectable = FALSE;
  }

#if UPDATE_CONN_PARAM      
  /* Connection parameter update request */
  if(connected && !l2cap_request_sent && l2cap_req_timer_expired){
    ret = aci_l2cap_connection_parameter_update_req(connection_handle, 9, 20, 0, 600); //24, 24
    PRINTF("aci_l2cap_connection_parameter_update_req(): 0x%02x\r\n", ret); (void)ret;
    l2cap_request_sent = TRUE;
  }
#endif
    
  /*  Update sensor value */
  if (sensorTimer_expired) {
    sensorTimer_expired = FALSE;
    if(HAL_VTIMER_StartTimerMs(&sensorTimerHandle, ACC_UPDATE_INTERVAL_MS) != BLE_STATUS_SUCCESS)
      sensorTimer_expired = TRUE;
    if(connected) {
      AxesRaw_t acc_data;
      
      /* Activity Led */
      BSP_LED_Toggle(BSP_LED1);  

      /* Get Acceleration data */
      if (GetAccAxesRaw(&acc_data) == 1) {
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
#if UPDATE_CONN_PARAM  
  uint8_t ret; 
#endif
  
  if(Status != BLE_STATUS_SUCCESS)
    return;
  
  connected = TRUE;
  connection_handle = Connection_Handle;
  
#if UPDATE_CONN_PARAM    
  l2cap_request_sent = FALSE;
  
   /* Start the l2cap Timer */
  l2cap_TimerHandle.callback = l2cap_UpdateTimeoutCB;  
  ret = HAL_VTIMER_StartTimerMs(&l2cap_TimerHandle, CLOCK_SECOND*2);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("HAL_VTIMER_StartTimerMs(l2cap) failed; 0x%02x\r\n", ret);
  } else {
    l2cap_req_timer_expired = FALSE;
  }
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
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  connected = FALSE;
  /* Make the device connectable again. */
  set_connectable = TRUE;
  connection_handle =0;
  
  BSP_LED_On(BSP_LED1);//activity led   
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  OTA_terminate_connection();
#endif 
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
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  OTA_Write_Request_CB(Connection_Handle, Attr_Handle, Attr_Data_Length, Attr_Data); 
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 
}


void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)
{
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  if (Next_State == 0x02) /* 0x02: Connection event slave */
  {
    OTA_Radio_Activity(Next_State_SysTime);  
  }
#endif 
}

#if UPDATE_CONN_PARAM

void aci_l2cap_connection_update_resp_event(uint16_t Connection_Handle,
                                            uint16_t Result)
{
   PRINTF("aci_l2cap_connection_update_resp_event; 0x%02x\r\n", Result);
}

void aci_l2cap_command_reject_event(uint16_t Connection_Handle,
                                    uint8_t Identifier,
                                    uint16_t Reason,
                                    uint8_t Data_Length,
                                    uint8_t Data[])
{
  PRINTF("aci_l2cap_command_reject_event; 0x%02x\r\n", Reason);
}
#endif 

void hci_le_connection_update_complete_event(uint8_t Status,
                                             uint16_t Connection_Handle,
                                             uint16_t Conn_Interval,
                                             uint16_t Conn_Latency,
                                             uint16_t Supervision_Timeout)
{
  PRINTF("hci_le_connection_update_complete_event; %d\r\n", Conn_Interval);
}

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t Server_RX_MTU)
{
  OTA_att_exchange_mtu_resp_CB(Connection_Handle, Server_RX_MTU);
}

void hci_le_data_length_change_event(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
  OTA_data_length_change_CB(Connection_Handle);  
}
#endif
