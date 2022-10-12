/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : keyboard.c
* Author             : AMS - RF Application Team
* Version            : V1.0.0
* Date               : 03-April-2019
* Description        : BlueNRG-LP keyboard init and process data functions
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
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "hid_peripheral.h"
#include "hid_peripheral_config.h"
#include "keyboard.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct hidValueS {
  uint8_t key;
  uint8_t hid[2];
} hidValueType;

/* Private define ------------------------------------------------------------*/
#define NUM_0      0x30
#define NUM_9      0x39
#define CHAR_A     0x41
#define CHAR_Z     0x5A
#define CHAR_a     0x61
#define CHAR_z     0x7A
#define RETURN     0x0D
#define BACKSPACE  0x08
#define TAB        0x09
#define SPACE      0x20

#define DEBUG 1

#define APP_TIMER 1
#define IDLE_CONNECTION_TIMEOUT (120*1000) // 2 min

#define KEY_TABLE_LEN 33

#define NUM_REPORTS 2

/* Private macro -------------------------------------------------------------*/
#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/

hidService_Type hid_param;
report_Type reportReferenceDesc[NUM_REPORTS];
const char *manufacName = "ST";
const char *modelNumber = "0001";
const char *fwRevision = "0630";
const char *swRevision = "0001";


uint8_t dev_name[]={'S', 'T', 'K', 'e', 'y', 'b', 'o', 'a', 'r', 'd'}; 

// Keyboard report descriptor
uint8_t reportDesc[] = {
        0x05, 0x01,                 // Usage Page (Generic Desktop)        
	0x09, 0x06,                 // Usage (Keyboard)        
	0xA1, 0x01,                 // Collection (Application)        
	0x05, 0x07,                 //     Usage Page (Key Codes)        
	0x19, 0xe0,                 //     Usage Minimum (224)        
	0x29, 0xe7,                 //     Usage Maximum (231)        
	0x15, 0x00,                 //     Logical Minimum (0)        
	0x25, 0x01,                 //     Logical Maximum (1)        
	0x75, 0x01,                 //     Report Size (1)        
	0x95, 0x08,                 //     Report Count (8)        
	0x81, 0x02,                 //     Input (Data, Variable, Absolute)            
	0x95, 0x01,                 //     Report Count (1)        
	0x75, 0x08,                 //     Report Size (8)        
	0x81, 0x01,                 //     Input (Constant) reserved byte(1)            
	0x95, 0x05,                 //     Report Count (5)        
	0x75, 0x01,                 //     Report Size (1)        
	0x05, 0x08,                 //     Usage Page (Page# for LEDs)        
	0x19, 0x01,                 //     Usage Minimum (1)        
	0x29, 0x05,                 //     Usage Maximum (5)        
	0x91, 0x02,                 //     Output (Data, Variable, Absolute), Led report        
	0x95, 0x01,                 //     Report Count (1)        
	0x75, 0x03,                 //     Report Size (3)        
	0x91, 0x01,                 //     Output (Data, Variable, Absolute), Led report padding
	0x95, 0x06,                 //     Report Count (6)        
	0x75, 0x08,                 //     Report Size (8)        
	0x15, 0x00,                 //     Logical Minimum (0)        
	0x25, 0x65,                 //     Logical Maximum (101)        
	0x05, 0x07,                 //     Usage Page (Key codes)        
	0x19, 0x00,                 //     Usage Minimum (0)        
	0x29, 0x65,                 //     Usage Maximum (101)        
	0x81, 0x00,                 //     Input (Data, Array) Key array(6 bytes)            
	0x09, 0x05,                 //     Usage (Vendor Defined)        
	0x15, 0x00,                 //     Logical Minimum (0)        
	0x26, 0xFF, 0x00,           //     Logical Maximum (255)        
	0x75, 0x08,                 //     Report Count (2)        
	0x95, 0x02,                 //     Report Size (8 bit)        
	0xB1, 0x02,                 //     Feature (Data, Variable, Absolute)        
	0xC0                        // End Collection (Application)    
};

hidValueType lookupTable[KEY_TABLE_LEN] = {
  {0x21, {TRUE,  0x1E}},
  {0x22, {TRUE,  0x34}},
  {0x23, {TRUE,  0x20}},
  {0x24, {TRUE,  0x21}},
  {0x25, {TRUE,  0x22}},
  {0x26, {TRUE,  0x24}},
  {0x27, {FALSE, 0x34}},
  {0x28, {TRUE,  0x26}},
  {0x29, {TRUE,  0x27}},
  {0x2A, {TRUE,  0x25}},
  {0x2B, {TRUE,  0x2E}},
  {0x2C, {FALSE, 0x36}},
  {0x2D, {FALSE, 0x2D}},
  {0x2E, {FALSE, 0x37}},
  {0x2F, {FALSE, 0x38}},
  {0x3A, {TRUE,  0x33}},
  {0x3B, {FALSE, 0x33}},
  {0x3C, {TRUE,  0x36}},
  {0x3D, {FALSE, 0x2E}},
  {0x3E, {TRUE,  0x37}},
  {0x3F, {TRUE,  0x38}},
  {0x40, {TRUE,  0x1F}},
  {0x5B, {FALSE, 0x2F}},
  {0x5C, {FALSE, 0x31}},
  {0x5D, {FALSE, 0x30}},
  {0x5E, {TRUE,  0x23}},
  {0x5F, {TRUE,  0x2D}},
  {0x60, {FALSE, 0x35}},
  {0x7B, {TRUE,  0x2F}},
  {0x7C, {TRUE,  0x31}},
  {0x7D, {TRUE,  0x30}},
  {0x7E, {TRUE,  0x35}},
  {0x7F, {FALSE, 0x4C}},
};

/* Extern function prototypes ------------------------------------------------*/

/* Private Functions ---------------------------------------------------------*/
static uint8_t hid_keyboard_map(uint8_t charac, uint8_t *upperCase)
{
  uint8_t hidValue, i;

  hidValue = 0;
  *upperCase = FALSE;

  if ((charac >= NUM_0) && (charac <= NUM_9)) {
    hidValue = charac - 0x30;
    if (hidValue == 0)
      hidValue = 0x27;
    else
      hidValue += 0x1D;
  }
  
  if ((charac >= CHAR_A)  && (charac <= CHAR_Z)) {
    hidValue = charac - 0x41 + 0x04;
    *upperCase = TRUE;
  }

  if ((charac >= CHAR_a)  && (charac <= CHAR_z)) {
    hidValue = charac - 0x61 + 0x04;
  } else {
    for (i=0; i<KEY_TABLE_LEN; i++) {
      if (lookupTable[i].key == charac) {
        *upperCase = lookupTable[i].hid[0];
        hidValue = lookupTable[i].hid[1];
        break;
      }
    }
  }

  switch(charac) {
  case RETURN:
    hidValue = 0x28;
    break;
  case BACKSPACE:
    hidValue = 0x02A;
    break;
  case SPACE:
    hidValue = 0x2C;
    break;
  case TAB:
    hidValue = 0x2B;
    break;
  }

  return hidValue;
}

/* Public Functions ----------------------------------------------------------*/
void processInputData(uint8_t* data_buffer, uint8_t Nb_bytes)
{
  uint8_t ret, i, upperCase, nmbTimes, keys[8]={0,0,0,0,0,0,0,0};
  
  if (hidDeviceStatus() & HID_DEVICE_READY_TO_NOTIFY) {

    BSP_LED_On(BSP_LED3);
    
    for (i=0; i<Nb_bytes; i++) {
      PRINTF("%c", data_buffer[i]);
      
      keys[2] = hid_keyboard_map(data_buffer[i], &upperCase);
      if (upperCase)
	keys[0] = 0x02;
      else
	keys[0] = 0x00;
      ret = hidSendReport(0, INPUT_REPORT, sizeof(keys), keys);
      if (ret != BLE_STATUS_SUCCESS)
        PRINTF("Error while sending the report 0x%02x\n", ret);
      keys[0] = 0;
      keys[2] = 0;
      nmbTimes = 0;
      do {
	ret = hidSendReport(0, INPUT_REPORT, sizeof(keys), keys);
        nmbTimes++;
      } while ((ret != BLE_STATUS_SUCCESS) && (nmbTimes < 200));
    }
    hidSetNotificationPending(TRUE);
  } else {
    BSP_LED_Off(BSP_LED3);
  }
}

void setDefaultHidParams(void)
{
  hid_param.bootSupport = FALSE;
  hid_param.reportSupport = TRUE;
  hid_param.num_reports = NUM_REPORTS;
  hid_param.reportReferenceDesc = reportReferenceDesc;
  hid_param.reportReferenceDesc[0].ID = REPORT_ID;
  hid_param.reportReferenceDesc[0].type = INPUT_REPORT;
  hid_param.reportReferenceDesc[1].ID = REPORT_ID;
  hid_param.reportReferenceDesc[1].type = OUTPUT_REPORT;
  hid_param.isBootDevKeyboard = FALSE;
  hid_param.isBootDevMouse = FALSE;
  hid_param.externalReportEnabled = 0;
  hid_param.includedServiceEnabled = FALSE;
  hid_param.informationCharac[0] = 0x01;
  hid_param.informationCharac[1] = 0x01;
  hid_param.informationCharac[2] = 0;
  hid_param.informationCharac[3] = 0x01;
}

void setTestHidParams(void)
{
  hid_param.bootSupport = TRUE;
  hid_param.reportSupport = TRUE;  
  hid_param.reportReferenceDesc[0].ID = REPORT_ID;
  hid_param.reportReferenceDesc[0].type = INPUT_REPORT;
  hid_param.reportReferenceDesc[1].ID = REPORT_ID;
  hid_param.reportReferenceDesc[1].type = OUTPUT_REPORT;
  hid_param.isBootDevKeyboard = TRUE;
  hid_param.isBootDevMouse = FALSE;
  hid_param.externalReportEnabled = 1;
  hid_param.includedServiceEnabled = FALSE;
  hid_param.informationCharac[0] = 0x01;
  hid_param.informationCharac[1] = 0x01;
  hid_param.informationCharac[2] = 0;
  hid_param.informationCharac[3] = 0x01;
}

void setHidParams_ReportType(uint8_t reportType)
{
  hid_param.reportReferenceDesc[0].type = reportType;
}  

uint8_t Configure_HidPeripheral(void) 
{
  uint8_t ret;
  batteryService_Type battery;
  devInfService_Type devInf;
  hidService_Type hid;
  connParam_Type connParam;

  /* HID Peripheral Init */
  connParam.interval_min = 9;
  connParam.interval_max = 10;
  connParam.slave_latency = 0;
  connParam.timeout_multiplier = 300;
  ret = hidDevice_Init(IO_CAP_DISPLAY_ONLY, connParam, sizeof(dev_name), dev_name, STATIC_RANDOM_ADDR);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in hidDevice_Init() 0x%02x\n", ret);
    return ret;
  }

  /* Set the HID Peripheral Security */
  ret = hidSetDeviceSecurty(TRUE, TRUE, 123456);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in hidSetDeviceSecurty() 0x%02x\n", ret);
    return ret;
  }

  /* Set the HID Idle Timeout */
  hidSetIdleTimeout(IDLE_CONNECTION_TIMEOUT);
  
  /* Set the TX Power */
  ret = aci_hal_set_tx_power_level(0, 24);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error with aci_hal_set_tx_power_level() 0x%02x\n", ret);
    return ret;
  }

  /**** Setup the GATT Database ****/

  /* Battery Service */
  battery.inReportMap = FALSE;

  /* Device Information Service */
  devInf.manufacName = manufacName;
  devInf.modelNumber = modelNumber;
  devInf.fwRevision = fwRevision;
  devInf.swRevision = swRevision;
  devInf.pnpID[0] = 0x01;
  devInf.pnpID[1] = 0x30;
  devInf.pnpID[2] = 0x00;
  devInf.pnpID[3] = 0xfc;
  devInf.pnpID[4] = 0x00;
  devInf.pnpID[5] = 0xec;
  devInf.pnpID[6] = 0x00;

  /* HID Service */
  hid = hid_param;  
  hid.reportDescLen = sizeof(reportDesc);
  hid.reportDesc = reportDesc;
  
  ret = hidAddServices(&battery, &devInf, &hid);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in hidAddServices() 0x%02x\n", ret);
    return ret;
  }

  /* Set the HID Peripheral device discoverable */
  ret = hidSetDeviceDiscoverable(LIMITED_DISCOVERABLE_MODE, sizeof(dev_name), dev_name);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in hidSetDeviceDiscoverable() 0x%02x\n", ret);
    return ret;
  }
  
  PRINTF("HID Keyboard Configured\n");

  BSP_LED_Off(BSP_LED1);
  
  /* Button Init */
  BSP_PB_Init(USER_BUTTON, BUTTON_MODE_EXTI);
  
  return BLE_STATUS_SUCCESS;
}

void DevicePowerSaveProcedure(void)
{
  Button_TypeDef Button = USER_BUTTON;
  WakeupSourceConfig_TypeDef wakeupIO;
  PowerSaveLevels stopLevel;
  PowerSaveLevels Level = POWER_SAVE_LEVEL_STOP_NOTIMER;  
  uint8_t hid_status = hidDeviceStatus();

  /* Allow sleep mode only if not connected because we cannot receive characters from UART while sleeping. */
  if ( hid_status & HID_DEVICE_READY_TO_SLEEP && !(hid_status & HID_DEVICE_CONNECTED)) {
    wakeupIO.IO_Mask_High_polarity = 0;
    wakeupIO.IO_Mask_Low_polarity = WAKEUP_PA8;
    wakeupIO.RTC_enable = 0;
    wakeupIO.LPU_enable = 0;
    if (Button == BSP_PUSH1)
    {
      wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
    }
    /* Power Save Request */
    HAL_PWR_MNGR_Request(Level, wakeupIO, &stopLevel);
    
    if(stopLevel >=POWER_SAVE_LEVEL_STOP_WITH_TIMER && HAL_PWR_MNGR_WakeupSource() == WAKEUP_PA8){
      hidSetNotificationPending(TRUE);
    }
  }
}

void HAL_PWR_MNGR_WakeupIOCallback(uint32_t source)
{
  hidSetNotificationPending(TRUE);
}

void DeviceInputData(void)
{
  uint8_t char_read;

  if (BSP_COM_Read(&char_read)) {
    processInputData(&char_read, 1);
  }
}

void APP_Tick(void)
{
  /* HID library Tick */
  hidDevice_Tick();
  
#ifndef PTS_AUTOMATING   
  /* Process Device Input Data */
  DeviceInputData();  
#endif    
}

/**** HID/HOGP Callback functions ****/
void hidSetReport_CB(uint8_t ID, uint8_t type, uint8_t len, uint8_t *data)
{
  uint8_t i;

  PRINTF("Set Report ID = 0x%x, Type = 0x%x, len = 0x%x data:", ID, type, len);
  for (i=0; i<len; i++){
    PRINTF("%x - ", data[i]);
  }
  PRINTF("\n");
}

void hidGetReport_CB(uint8_t ID, uint8_t type)
{
  uint8_t ret, len;
  uint8_t data[8]={0,0,0,0,0,0,0,0};
  
  PRINTF("Get Report Callback ID = %d\n", ID);

  if ((type == BOOT_KEYBOARD_INPUT_REPORT) || ((ID == REPORT_ID) && (type == INPUT_REPORT)))
    len = 8;
  if ((type == BOOT_KEYBOARD_OUTPUT_REPORT) || ((ID == REPORT_ID) && (type == OUTPUT_REPORT)))
    len = 1;
  
  ret = hidUpdateReportValue(ID, type, len, data);
  if (ret != BLE_STATUS_SUCCESS)
    PRINTF("Update report value Error during a hidGetReport_CB() procedure: 0x%02x\n", ret);
  else 
    PRINTF("hidGetReport_CB() procedure OK\n");
}

void hidChangeProtocolMode_CB(uint8_t mode)
{
  PRINTF("Protocol mode changed to %x\n", mode);
}

void hidControlPoint_CB(uint8_t value)
{
   if (value == 1)
#ifdef PTS_AUTOMATING 
    PRINTF("(%c%c1) HID Control Point Command: 1 (Exit Suspend)\n", PTS_CODE, HIDS_PTS_CODE);
#else
    PRINTF("Control Point value changed to 1. Host Exit Suspend state\n");
#endif
  else if (value == 0)
#ifdef PTS_AUTOMATING 
    PRINTF("(%c%c2) HID Control Point Command: 0 (Suspend)\n", PTS_CODE, HIDS_PTS_CODE);
#else
    PRINTF("Control Point value changed to 0. Host in Suspend state\n");
#endif   
}

/***************** BlueNRG-1 stack events **********************************/

void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  HID_Lib_hci_disconnection_complete_event(Status, Connection_Handle, Reason);
  PRINTF("Disconnection Complete Event\n");
}

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
  HID_Lib_hci_le_connection_complete_event(Status, Connection_Handle,
                                           Role, Peer_Address_Type,
                                           Peer_Address, Conn_Interval,
                                           Conn_Latency, Supervision_Timeout,
                                           Master_Clock_Accuracy);
  if(Status == 0)
    PRINTF("Connected.\n");
}

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

void aci_gap_limited_discoverable_event()
{
  HID_Lib_aci_gap_limited_discoverable_event();
}

void aci_gap_pass_key_req_event(uint16_t Connection_Handle)
{
  HID_Lib_aci_gap_pass_key_req_event(Connection_Handle);
}

void aci_gap_pairing_complete_event(uint16_t Connection_Handle,
                                    uint8_t Status,
                                    uint8_t Reason)
{
  HID_Lib_aci_gap_pairing_complete_event(Connection_Handle, Status);
}

void aci_gap_bond_lost_event()
{
  HID_Lib_aci_gap_bond_lost_event();
}

void aci_gatt_srv_attribute_modified_event(uint16_t Connection_Handle,
                                           uint16_t Attr_Handle,
                                           uint16_t Attr_Data_Length,
                                           uint8_t Attr_Data[])
{
  
  PRINTF("Modified event on handle;  0x%02x, value:  0x%0x,  0x%0x\n\n", Attr_Handle,Attr_Data[1],Attr_Data[0]); 
#ifdef PTS_AUTOMATING   
  if ((Attr_Data_Length == 2) && (Attr_Data[1] == 0x00 && Attr_Data[0] == 0x01))
  {
    PRINTF("(%c%c3) Enabled Char to Notify\n\n", PTS_CODE, HIDS_PTS_CODE);
  }  
#endif 
}
void aci_gatt_srv_write_event(uint16_t Connection_Handle, uint8_t Resp_Needed, uint16_t Attribute_Handle, uint16_t Data_Length, uint8_t Data[]) 
{
    uint8_t att_error = BLE_ATT_ERR_NONE;
    
    HID_Lib_aci_gatt_srv_attribute_modified_event(Connection_Handle,
                                              Attribute_Handle,
                                              Data_Length,
                                              Data);
    if (Resp_Needed == 1U)
    {
        aci_gatt_srv_resp(Connection_Handle, 0, att_error, 0, NULL);
    }
}


void aci_gatt_srv_read_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Data_Offset)
{
  PRINTF("aci_gatt_srv_read_event() handle:  0x%02x, Data_Offset:  0x%02x\n\n", Attribute_Handle,Data_Offset); 
  HID_Lib_aci_gatt_read_permit_req_event(Connection_Handle, Attribute_Handle,Data_Offset);
}


void aci_l2cap_connection_update_resp_event(uint16_t Connection_Handle,
                                            uint16_t Result)
{
  HID_Lib_aci_l2cap_connection_update_resp_event(Connection_Handle, Result);
}

void aci_l2cap_proc_timeout_event(uint16_t Connection_Handle,
                                  uint8_t Data_Length,
                                  uint8_t Data[])
{
  HID_Lib_aci_l2cap_proc_timeout_event(Connection_Handle, Data_Length,
                                       Data);
}

