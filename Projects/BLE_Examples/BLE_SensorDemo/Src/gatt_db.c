/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : gatt_db.c
* Author             : BlueNRG-LP database file for Sensor demo
* Version            : V1.0.0
* Date               : 02-April-2019
* Description        : Functions to build GATT DB and handle GATT events.
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
#include <stdlib.h>
#include "ble_const.h" 
#include "bluenrg_lp_stack.h"
#include "clock.h" 
#include "gp_timer.h" 
#include "gatt_db.h"
#include "osal.h"
#include "SensorDemo_config.h"
#include "gatt_profile.h"
#include "gap_profile.h"
#include "OTA_btl.h" 


#include "bluenrg_lp_evb_config.h" 

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define ACC_SERVICE_UUID        0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xb4,0x9a,0xe1,0x11,0x3a,0xcf,0x80,0x6e,0x36,0x02
#define FREE_FALL_UUID          0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xfc,0x8f,0xe1,0x11,0x4a,0xcf,0xa0,0x78,0x3e,0xe2
#define ACC_UUID                0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x4b,0xcf,0x80,0x1b,0x0a,0x34
#define ENV_SENS_SERVICE_UUID   0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xd0,0x82,0xe2,0x11,0x77,0xe4,0x40,0x1a,0x82,0x42
#define TEMP_CHAR_UUID          0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xe3,0xa9,0xe2,0x11,0x77,0xe4,0x20,0x55,0x2e,0xa3
#define PRESS_CHAR_UUID         0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x0b,0x84,0xe2,0x11,0x8b,0xe4,0x80,0xc4,0x20,0xcd

uint16_t accServHandle, freeFallCharHandle, accCharHandle;
uint16_t envSensServHandle, tempCharHandle, pressCharHandle;

extern uint16_t connection_handle;
extern BOOL sensor_board;

#ifndef SENSOR_ACCELEROMETER_EMULATION
extern lsm6dsox_ctx_t inertialHandle;
#endif

#ifndef SENSOR_PRESSURE_TEMPERATURE_EMULATION
extern lps22hh_ctx_t pressureHandle;
#endif

/* Client Configuration Characteristics Descriptor Definition: free fall characteristic  */
BLE_GATT_SRV_CCCD_DECLARE(free_fall,
                     NUM_LINKS,
                     BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                     BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);

/* Client Configuration Characteristics Descriptor Definition: acceleration characteristic*/
BLE_GATT_SRV_CCCD_DECLARE(accell,
                     NUM_LINKS,
                     BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                     BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);

/* Acceleration Service Characteristics Definition */
static ble_gatt_chr_def_t acc_chars[] = {
    /* Free Fall characteristic */
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(FREE_FALL_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(free_fall),
            .descr_count = 1U,
        },
    },
    /* Acceleration characteristic */
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_NOTIFY | BLE_GATT_SRV_CHAR_PROP_READ,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(ACC_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(accell),
            .descr_count = 1U,
        },
    }
};

/* Acceleration service definition */
static ble_gatt_srv_def_t acc_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(ACC_SERVICE_UUID),
   .chrs = {
       .chrs_p = acc_chars,
       .chr_count = 2U,
   },
};

/* Temperature characteristic format descriptor definition */
static charactFormat temp_char_format = {
    .format = FORMAT_SINT16,
    .exp = -1,
    .unit = UNIT_TEMP_CELSIUS,
    .name_space = 0,
    .desc = 0,
};

/* Pressure characteristic format descriptor definition */
static charactFormat press_char_format = {
    .format = FORMAT_SINT24,
    .exp = -5,
    .unit = UNIT_PRESSURE_BAR,
    .name_space = 0,
    .desc = 0,
};

/* Environmental temperature and pressure characteristic descriptor format definition */ 
static ble_gatt_val_buffer_def_t env_descr_val_buffers[] =
{
    {
        .buffer_len = 7,
        .buffer_p = (uint8_t *)&temp_char_format,
    },
    {
        .buffer_len = 7,
        .buffer_p = (uint8_t *)&press_char_format,
    }
};

/* Environmental temperature and pressure characteristic descriptor definition */ 
static ble_gatt_descr_def_t env_descrs[] =
{
    {
        .uuid = BLE_UUID_INIT_16(CHAR_FORMAT_DESC_UUID),
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .properties = BLE_GATT_SRV_DESCR_PROP_READ,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .val_buffer_p = &env_descr_val_buffers[0], /* temperature char format */
    },
    {
        .uuid = BLE_UUID_INIT_16(CHAR_FORMAT_DESC_UUID),
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .properties = BLE_GATT_SRV_DESCR_PROP_READ,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .val_buffer_p = &env_descr_val_buffers[1], /* pressure  char format */
    },
};

/* Environmental temperature and pressure characteristic  definition */ 
static ble_gatt_chr_def_t env_chars[] = {
    {/* Temperature Characteristic */
        .properties = BLE_GATT_SRV_CHAR_PROP_READ,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(TEMP_CHAR_UUID),
        .descrs = {
            .descrs_p = &env_descrs[0],
            .descr_count = 1,
        },
    },
    {/* Pressure Characteristic */
        .properties = BLE_GATT_SRV_CHAR_PROP_READ,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(PRESS_CHAR_UUID),
        .descrs = {
            .descrs_p = &env_descrs[1],
            .descr_count = 1,
        },
    },
};

/* Environmental service definition */ 
static ble_gatt_srv_def_t env_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(ENV_SENS_SERVICE_UUID),
   .chrs = {
       .chrs_p = env_chars,
       .chr_count = 2U,
   },
};

uint8_t GetAccAxesRaw(AxesRaw_t * acceleration_data)
{
  
#ifdef SENSOR_ACCELEROMETER_EMULATION
  uint8_t tmp = 1;
  acceleration_data->AXIS_X = ((uint64_t)rand()) % X_OFFSET;
  acceleration_data->AXIS_Y = ((uint64_t)rand()) % Y_OFFSET; 
  acceleration_data->AXIS_Z = ((uint64_t)rand()) % Z_OFFSET;
#else
  uint8_t tmp = 0;
  (void)tmp;
  
  axis3bit16_t data_raw_acceleration;
    
   /* Read output only if new value is available */
  lsm6dsox_xl_flag_data_ready_get(&inertialHandle, &tmp);
  if(tmp)
  {
    memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
    /* Read acceleration field data */
    lsm6dsox_acceleration_raw_get(&inertialHandle, data_raw_acceleration.u8bit);
    acceleration_data->AXIS_X = (int32_t)lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[0]); 
    acceleration_data->AXIS_Y = (int32_t)lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
    acceleration_data->AXIS_Z = (int32_t)lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);
  }
      
#endif 
  return tmp;
}


void GetFreeFallStatus(void) 
{
#ifndef SENSOR_ACCELEROMETER_EMULATION
  
  lsm6dsox_all_sources_t all_source;

  /* Check if Free Fall events
  */
  lsm6dsox_all_sources_get(&inertialHandle, &all_source);
  if (all_source.wake_up_src.ff_ia)
  {
    request_free_fall_notify = TRUE;
  }
#endif
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

  ret = aci_gatt_srv_add_service(&acc_service);
  if (ret != BLE_STATUS_SUCCESS)
  {
    goto fail;
  }
  
  accServHandle = aci_gatt_srv_get_service_handle(&acc_service);
  freeFallCharHandle = aci_gatt_srv_get_char_decl_handle(&acc_chars[0]);
  accCharHandle = aci_gatt_srv_get_char_decl_handle(&acc_chars[1]);
  PRINTF("Service ACC added. Handle 0x%04X, Free fall Charac handle: 0x%04X, Acc Charac handle: 0x%04X\n",
         accServHandle, freeFallCharHandle, accCharHandle);

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

  ret = aci_gatt_srv_notify(connection_handle, freeFallCharHandle + 1, 0, 1, &val);
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating Free Fall characteristic: 0x%02X\n",ret) ;
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

  HOST_TO_LE_16(buff,-data->AXIS_Y);
  HOST_TO_LE_16(buff+2,data->AXIS_X);
  HOST_TO_LE_16(buff+4,-data->AXIS_Z);
  ret = aci_gatt_srv_notify(connection_handle, accCharHandle + 1, 0, 6, buff);
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating Acceleration characteristic: 0x%02X\n",ret) ;
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
  
  ret = aci_gatt_srv_add_service(&env_service);
  if (ret != BLE_STATUS_SUCCESS)
  {
    goto fail;
  }
  
  envSensServHandle = aci_gatt_srv_get_service_handle(&env_service);
  tempCharHandle = aci_gatt_srv_get_char_decl_handle(&env_chars[0]);
  pressCharHandle = aci_gatt_srv_get_char_decl_handle(&env_chars[1]);
  
  PRINTF("Service ENV_SENS added. Handle 0x%04X, TEMP Charac handle: 0x%04X, PRESS Charac handle: 0x%04X\n",envSensServHandle, tempCharHandle, pressCharHandle);	
  return BLE_STATUS_SUCCESS; 
  
fail:
  PRINTF("Error while adding ENV_SENS service.\n");
  return BLE_STATUS_ERROR;
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

#ifndef SENSOR_PRESSURE_TEMPERATURE_EMULATION

uint8_t GetPressure(float * pressure_hPa)
{
   axis1bit32_t data_raw_pressure;
   lps22hh_reg_t reg;
   
   data_raw_pressure.i32bit = 0;
   
   lps22hh_read_reg(&pressureHandle, LPS22HH_STATUS, (uint8_t *)&reg, 1);
   if (reg.status.p_da) 
   {
      lps22hh_pressure_raw_get(&pressureHandle, data_raw_pressure.u8bit);
      *pressure_hPa = lps22hh_from_lsb_to_hpa(data_raw_pressure.i32bit);
   }
   return (reg.status.p_da);
}

/* Update temperature data in advertising packets */
uint8_t GetTemperature(float * temperature_degC)
{
    axis1bit16_t data_raw_temperature;
    lps22hh_reg_t reg;

    /* Read output only if new value is available */
    lps22hh_read_reg(&pressureHandle, LPS22HH_STATUS, (uint8_t *)&reg, 1);
    if (reg.status.t_da) 
    {
      lps22hh_temperature_raw_get(&pressureHandle, data_raw_temperature.u8bit);
      *temperature_degC = lps22hh_from_lsb_to_celsius(data_raw_temperature.i16bit);
    }
    return (reg.status.t_da);
}

#endif 

void aci_gatt_srv_write_event(uint16_t Connection_Handle, uint8_t Resp_Needed, uint16_t Attribute_Handle, uint16_t Data_Length, uint8_t Data[])
{
    uint8_t att_error = BLE_ATT_ERR_NONE;
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  OTA_Write_Request_CB(Connection_Handle, Attribute_Handle, Data_Length, Data); 
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 
  
    if (Resp_Needed == 1U)
    {
        aci_gatt_srv_resp(Connection_Handle, 0, att_error, 0, NULL);
    }
}

void aci_gatt_srv_read_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Data_Offset)
{
    uint8_t att_err;
    uint8_t buff[6], *data_p;
    uint16_t data_len;
    int16_t temp_val;
    int32_t press_val;
    float fdata;
#ifdef SENSOR_PRESSURE_TEMPERATURE_EMULATION
    uint16_t udata;
#endif

    att_err = BLE_ATT_ERR_NONE;
    if(Attribute_Handle == accCharHandle + 1)
    {
        AxesRaw_t acc_data;
        if (GetAccAxesRaw(&acc_data) == 1)
        {
            HOST_TO_LE_16(buff,     -acc_data.AXIS_Y);
            HOST_TO_LE_16(buff + 2,  acc_data.AXIS_X);
            HOST_TO_LE_16(buff + 4, -acc_data.AXIS_Z);
            data_len = 6;
            data_p = buff;
        }
        else
        {
            att_err = BLE_ATT_ERR_APPL_MIN;
        }
    }
    else if(Attribute_Handle == tempCharHandle + 1)
    {
#ifdef SENSOR_PRESSURE_TEMPERATURE_EMULATION
        fdata = 27 + ((uint64_t)rand()*15)/RAND_MAX;
#else
        if (GetTemperature(&fdata) != 1)
        {
            att_err = BLE_ATT_ERR_APPL_MIN;
        }
#endif
        data_len = 2;
        temp_val = (int16_t)(fdata * 10);
        data_p = (uint8_t *)&temp_val;
    }
    else if(Attribute_Handle == pressCharHandle + 1)
    {
#ifdef SENSOR_PRESSURE_TEMPERATURE_EMULATION
        fdata = 1000 + ((uint64_t)rand()*1000)/RAND_MAX;
#else
        if (GetPressure(&fdata) != 1)
        {
            att_err = BLE_ATT_ERR_APPL_MIN;
        }
#endif
        data_len = 3;
        press_val = (int32_t)(fdata * 100);
        data_p = (uint8_t *)&press_val;
    }
    
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT 
    OTA_Read_Char(Connection_Handle, Attribute_Handle,Data_Offset);
#endif 

    aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, att_err, data_len, data_p);
}

