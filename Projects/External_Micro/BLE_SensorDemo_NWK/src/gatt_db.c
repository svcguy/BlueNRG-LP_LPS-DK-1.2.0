/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
* File Name          : gatt_db.c
* Author             : RF Application Team - AMS
* Version            : V1.1.0
* Date               : 12-November-2018
* Description        : Functions to build GATT DB and handle GATT events.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


#include "hal_types.h"
#include "gatt_db.h"
#include "bluenrg_lp_gatt_server.h"
#include "bluenrg_lp_gap.h"
#include "bluenrg_lp_gatt_aci.h"
#include <string.h>
#include "osal.h"
#include "hci_const.h"
#include <stdio.h>
#include <hal.h>
#include <stdlib.h>
#include <gp_timer.h>

#include "SDK_EVAL_Config.h"

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)


#define COPY_ACC_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x02,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_FREE_FALL_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0xe2,0x3e,0x78,0xa0, 0xcf,0x4a, 0x11,0xe1, 0x8f,0xfc, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x34,0x0a,0x1b,0x80, 0xcf,0x4b, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#define COPY_ENV_SENS_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x42,0x82,0x1a,0x40, 0xe4,0x77, 0x11,0xe2, 0x82,0xd0, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TEMP_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0xa3,0x2e,0x55,0x20, 0xe4,0x77, 0x11,0xe2, 0xa9,0xe3, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_PRESS_CHAR_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0xcd,0x20,0xc4,0x80, 0xe4,0x8b, 0x11,0xe2, 0x84,0x0b, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_HUMIDITY_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x01,0xc5,0x0b,0x60, 0xe4,0x8c, 0x11,0xe2, 0xa0,0x73, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)


uint16_t accServHandle, freeFallCharHandle, accCharHandle;
uint16_t envSensServHandle, tempCharHandle, pressCharHandle;

/* UUIDS */
Service_UUID_t service_uuid;
Char_UUID_t char_uuid;
Char_Desc_Uuid_t char_desc_uuid;

extern uint16_t connection_handle;
extern BOOL sensor_board;

uint8_t GetAccAxesRaw(AxesRaw_t * emulated_acceleration_data)
{  
  emulated_acceleration_data->AXIS_X = ((uint64_t)rand()) % X_OFFSET;
  emulated_acceleration_data->AXIS_Y = ((uint64_t)rand()) % Y_OFFSET; 
  emulated_acceleration_data->AXIS_Z = ((uint64_t)rand()) % Z_OFFSET;
  return 0;
}

void GetFreeFallStatus(void)
{
}

/*******************************************************************************
* Function Name  : Add_Chat_Service
* Description    : Add the 'Accelerometer' service.
* Input          : None
* Return         : Status.
*******************************************************************************/
tBleStatus Add_Acc_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];
  
  COPY_ACC_SERVICE_UUID(uuid);

  Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_srv_add_service_nwk(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 7, &accServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
  COPY_FREE_FALL_UUID(uuid);
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);

  ret =  aci_gatt_srv_add_char_nwk(accServHandle, UUID_TYPE_128, &char_uuid, 1, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                           16, 0, &freeFallCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  COPY_ACC_UUID(uuid);  
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_srv_add_char_nwk(accServHandle, UUID_TYPE_128, &char_uuid, 6, CHAR_PROP_NOTIFY|CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &accCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  PRINTF("Service ACC added. Handle 0x%04X, Free fall Charac handle: 0x%04X, Acc Charac handle: 0x%04X\n",accServHandle, freeFallCharHandle, accCharHandle);	
  return BLE_STATUS_SUCCESS; 
  
fail:
  PRINTF("Error while adding ACC service.\n");
  return BLE_STATUS_ERROR ;
  
}

/*******************************************************************************
* Function Name  : Free_Fall_Notify
* Description    : Send a notification for a Free Fall detection.
* Input          : None
* Return         : Status.
*******************************************************************************/
tBleStatus Free_Fall_Notify(void)
{  
  uint8_t val;
  tBleStatus ret;
  
  val = 0x01;	
  
  ret = aci_gatt_srv_notify(connection_handle, freeFallCharHandle + 1, GATT_NOTIFICATION, 1, &val);
  
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while notifying ACC characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;
  
}

  
/*******************************************************************************
* Function Name  : Acc_Update
* Description    : Update acceleration characteristic value
* Input          : AxesRaw_t structure containing acceleration value in mg.
* Return         : Status.
*******************************************************************************/
tBleStatus Acc_Update(AxesRaw_t *data)
{
  uint8_t buff[6];
  tBleStatus ret;    
  
  HOST_TO_LE_16(buff,data->AXIS_X);
  HOST_TO_LE_16(buff+2,data->AXIS_Y);
  HOST_TO_LE_16(buff+4,data->AXIS_Z);
  
  ret = aci_gatt_srv_notify(connection_handle, accCharHandle + 1, GATT_NOTIFICATION, 6, buff);
  
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating ACC characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  
  return BLE_STATUS_SUCCESS;
  
}

/*******************************************************************************
* Function Name  : Add_Environmental_Sensor_Service
* Description    : Add the Environmental Sensor service.
* Input          : None
* Return         : Status.
*******************************************************************************/
tBleStatus Add_Environmental_Sensor_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];
  uint16_t uuid16;
  charactFormat charFormat;
  uint16_t descHandle;
  
  COPY_ENV_SENS_SERVICE_UUID(uuid);
  
  Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_srv_add_service_nwk(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 10, &envSensServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
   
  /* Temperature Characteristic */
  
  COPY_TEMP_CHAR_UUID(uuid);  
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_srv_add_char_nwk(envSensServHandle, UUID_TYPE_128, &char_uuid, 2, CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &tempCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  charFormat.format = FORMAT_SINT16;
  charFormat.exp = -1;
  charFormat.unit = UNIT_TEMP_CELSIUS;
  charFormat.name_space = 0;
  charFormat.desc = 0;
  
  uuid16 = CHAR_FORMAT_DESC_UUID;
  
  Osal_MemCpy(&char_desc_uuid.Char_UUID_16, &uuid16, 2);
  
  ret = aci_gatt_srv_add_char_desc_nwk(tempCharHandle,
                               UUID_TYPE_16,
                               &char_desc_uuid, 
                               7,
                               7,
                               (void *)&charFormat, 
                               ATTR_PERMISSION_NONE,
                               ATTR_ACCESS_READ_ONLY,
                               0,
                               16,
                               FALSE,
                               &descHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  /* Pressure Characteristic */
    COPY_PRESS_CHAR_UUID(uuid); 
  Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16); 
    ret =  aci_gatt_srv_add_char_nwk(envSensServHandle, UUID_TYPE_128, &char_uuid, 3, CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                             16, 0, &pressCharHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;
    
    charFormat.format = FORMAT_SINT24;
    charFormat.exp = -5;
    charFormat.unit = UNIT_PRESSURE_BAR;
    charFormat.name_space = 0;
    charFormat.desc = 0;
    
    uuid16 = CHAR_FORMAT_DESC_UUID;
    
  Osal_MemCpy(&char_desc_uuid.Char_UUID_16, &uuid16, 2);
    ret = aci_gatt_srv_add_char_desc_nwk(pressCharHandle,
                                 UUID_TYPE_16,
                                 &char_desc_uuid, 
                                 7,
                                 7,
                                 (void *)&charFormat, 
                                 ATTR_PERMISSION_NONE,
                                 ATTR_ACCESS_READ_ONLY,
                                 0,
                                 16,
                                 FALSE,
                                 &descHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;

  PRINTF("Service ENV_SENS added. Handle 0x%04X, TEMP Charac handle: 0x%04X, PRESS Charac handle: 0x%04X.\n",envSensServHandle, tempCharHandle, pressCharHandle);	
  return BLE_STATUS_SUCCESS; 
  
fail:
  PRINTF("Error while adding ENV_SENS service.\n");
  return BLE_STATUS_ERROR ;
  
}

/*******************************************************************************
* Function Name  : Temp_Update
* Description    : Update temperature characteristic value
* Input          : temperature in tenths of degree
* Return         : Status.
*******************************************************************************/
tBleStatus Temp_Update(int16_t temp)
{  
  tBleStatus ret;
  
  ret = aci_gatt_srv_notify(connection_handle, tempCharHandle + 1, 0, 2, (uint8_t *)&temp);
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating TEMP characteristic: 0x%02X\n",ret);
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;
  
}

/*******************************************************************************
* Function Name  : Press_Update
* Description    : Update pressure characteristic value
* Input          : Pressure in mbar
* Return         : Status.
*******************************************************************************/
tBleStatus Press_Update(int32_t press)
{  
  tBleStatus ret;
  
  ret = aci_gatt_srv_notify(connection_handle, pressCharHandle + 1, 0, 3, (uint8_t *)&press);
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating Pressure characteristic: 0x%02X\n",ret);
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
  
}


/*******************************************************************************
 * Function Name  : aci_gatt_srv_authorize_event.
 * Description    : This event is given when an authorization for a read or write
 *                  request is received
 *******************************************************************************/
void aci_gatt_srv_authorize_nwk_event(uint16_t Conn_Handle,
                                  uint16_t Attr_Handle,
                                  uint8_t Operation_Type,
                                  uint16_t Attr_Val_Offset,
                                  uint8_t Data_Length,
                                  uint8_t Data[])
{
  if(Operation_Type == 0){ // Read
    
    tBleStatus ret;
    uint8_t buff[6];
    uint16_t data_length;
    
    if(Attr_Handle == accCharHandle + 1)
    {
      AxesRaw_t acc_data;
      if (GetAccAxesRaw(&acc_data) == 0)
      {        
        HOST_TO_LE_16(buff,acc_data.AXIS_X);
        HOST_TO_LE_16(buff+2,acc_data.AXIS_Y);
        HOST_TO_LE_16(buff+4,acc_data.AXIS_Z);        
        data_length = 6;
      }
    }
    else if(Attr_Handle == tempCharHandle + 1)
    {
      float data;
      int16_t tenth_deg;
      data = 27 + ((uint64_t)rand()*15)/RAND_MAX;
      tenth_deg = (int16_t)data * 10;
      HOST_TO_LE_16(buff, tenth_deg);
      data_length = 2;
      
    }
    else if(Attr_Handle == pressCharHandle + 1)
    {
      float data;
      uint32_t hundredth_pa;
      data = 1000 + ((uint64_t)rand()*1000)/RAND_MAX;
      hundredth_pa = (int32_t)(data *100);      
      HOST_TO_LE_32(buff, hundredth_pa);
      data_length = 3;
    }
    else {      
      /* Unknown handle. */
      return;      
    }
    
    ret = aci_gatt_srv_write_handle_value_nwk(Attr_Handle, 0, data_length, buff);
    if (ret != BLE_STATUS_SUCCESS){
      PRINTF("Error updating characteristic value: 0x%02X\n",ret);
    }

    ret = aci_gatt_srv_authorize_resp_nwk(Conn_Handle, Attr_Handle, Operation_Type, 0, Attr_Val_Offset, Data_Length, Data);
    
    if (ret != BLE_STATUS_SUCCESS)
    {
      SdkEvalLedOn();
    }
    PRINTF("--- Updated characterisic value, data_length: %d, ret: 0x%02X\n", data_length, ret);
    
  }
  
  /* No characteristics that require authorization to write, so no need to handle
     other operation types. */
  
}
