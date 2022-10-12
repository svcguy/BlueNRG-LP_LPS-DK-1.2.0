/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
* File Name          : gatt_db.c
* Author             : AMS - RF Application team
* Version            : V1.0.0
* Date               : 22-September-2020
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
#include "gatt_db.h"
#include "osal.h"
#include "rf_driver_hal_vtimer.h"

#include "bluenrg_lp_evb_config.h"
#include "SensorDemo_config.h"
#include "gatt_profile.h"
#include "gap_profile.h"
#include "sensor.h"
#if ENABLE_BLUEVOICE
  #include "bluevoice_adpcm_3_x.h"	// BlueVoice for BLE stack v3.x
#endif

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Hardware Characteristics Service */ 
#define HW_SENS_W2ST_SERVICE_UUID      0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xb4,0x9a,0xe1,0x11,0x01,0x00,0x00,0x00,0x00,0x00

#define ENVIRONMENTAL_W2ST_CHAR_UUID   0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0x14,0x00 // 0x14: (0x04|0x10) [0x04: One Temperature value; 0x10: Pressure value ]

/* 0x00 c0 00 00  = 00 80 00 00 (acc) + 00 40 00 00 (gyro); [no + 00 20 00 00 (mag),  since magnetometer is not supported on BlueNRG-1/2/LP eval kits] */
#define ACC_GYRO_MAG_W2ST_CHAR_UUID    0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0xC0,0x00


/* Client Configuration Characteristics Descriptor Definition: env characteristic*/
BLE_GATT_SRV_CCCD_DECLARE(env,
                          NUM_LINKS,
                          BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);


/* Client Configuration Characteristics Descriptor Definition: acc_gyro characteristic*/
BLE_GATT_SRV_CCCD_DECLARE(accell,
                          NUM_LINKS,
                          BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);



/* HW Sensor Service Characteristics Definition */
static ble_gatt_chr_def_t hw_chars[] = {
    /* env. characteristic */
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_NOTIFY | BLE_GATT_SRV_CHAR_PROP_READ,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(ENVIRONMENTAL_W2ST_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(env),
            .descr_count = 1U,
        },
    },
    /* Acceleration/gyro characteristic */
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_NOTIFY ,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(ACC_GYRO_MAG_W2ST_CHAR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(accell),
            .descr_count = 1U,
        },
    },

};

/* hw service definition */
static ble_gatt_srv_def_t hw_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(HW_SENS_W2ST_SERVICE_UUID),
   .chrs = {
       .chrs_p = hw_chars,
       .chr_count = 2U,
   },
};

uint16_t HWServiceHandle, EnvironmentalCharHandle, AccGyroMagCharHandle;


extern uint16_t connection_handle;
extern lsm6dsox_ctx_t inertialHandle;
/* LPS22HB initialization */
extern lps22hh_ctx_t pressureHandle;
extern uint32_t start_time;

uint8_t GetPressure(float * pressure_hPa);
uint8_t GetTemperature(float * temperature_degC);

#if ENABLE_BLUEVOICE
  BV_ADPCM_3_x_ProfileHandle_t TX_handle_BV;

  BV_ADPCM_3_x_Config_t BV_ADPCM_3_x_Config;
#endif
        
/*******************************************************************************
* Function Name  : Add_HWServW2ST_Service
* Description    : Add the 'Accelerometer' service.
* Input          : None
* Return         : Status.
*******************************************************************************/
tBleStatus Add_HWServW2ST_Service(void)
{
  tBleStatus ret;
#if ENABLE_BLUEVOICE
  BV_3_x_Status BV_ret; 
#endif

  /* Add service + characteristics */
  ret = aci_gatt_srv_add_service(&hw_service);
  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }
  /* GEt service handle */
  HWServiceHandle =  aci_gatt_srv_get_service_handle(&hw_service);
  
  /* Fill the Environmental BLE Characteristc */
  EnvironmentalCharHandle = aci_gatt_srv_get_char_decl_handle(&hw_chars[0]);
  /* Fill the Acceleration BLE Characteristc */
  AccGyroMagCharHandle = aci_gatt_srv_get_char_decl_handle(&hw_chars[1]);
  
#if ENABLE_BLUEVOICE
  
  /* Set configuration: to be done before add char in order to have bluevoice configured  */
  BV_ADPCM_3_x_Config.sampling_frequency = FR_8000; 
  BV_ADPCM_3_x_Config.channel_in = 1;
  BV_ADPCM_3_x_Config.channel_tot = 1;
  BluevoiceADPCM_3_x_SetConfig(&BV_ADPCM_3_x_Config);
  
  /* Store the local service HWServiceHandle */
  TX_handle_BV.ServiceHandle = HWServiceHandle; 
  
  /* Add the Audio and Audio Dync char to the HWServiceHandle service */
  BV_ret = BluevoiceADPCM_3_x_AddChar(TX_handle_BV.ServiceHandle, &TX_handle_BV);
  if (BV_ret != BV_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }
  
  BluevoiceADPCM_3_x_SetTxHandle(&TX_handle_BV);
#endif 
  return BLE_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name  : Acc_Update
* Description    : Update acceleration characteristic value
* Input          : AxesRaw_t structure containing acceleration value in mg.
* Return         : Status.
*******************************************************************************/
tBleStatus Acc_Update(AxesRaw_t *x_axes, AxesRaw_t *g_axes)
{  
  uint8_t buff[2+2*6];
  tBleStatus ret;
  uint32_t time = HAL_VTIMER_DiffSysTimeMs(HAL_VTIMER_GetCurrentSysTime(), start_time);
  
  HOST_TO_LE_16(buff,time);
  
  HOST_TO_LE_16(buff+2,-x_axes->AXIS_Y);
  HOST_TO_LE_16(buff+4, x_axes->AXIS_X);
  HOST_TO_LE_16(buff+6,-x_axes->AXIS_Z);
    
  HOST_TO_LE_16(buff+8,g_axes->AXIS_Y);
  HOST_TO_LE_16(buff+10,g_axes->AXIS_X);
  HOST_TO_LE_16(buff+12,g_axes->AXIS_Z);
  
  ret = aci_gatt_srv_notify(connection_handle, AccGyroMagCharHandle + 1, 0,  2+2*6, buff);
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating Acceleration characteristic: 0x%02X\n",ret) ;
    return BLE_STATUS_ERROR ;
  }

  return BLE_STATUS_SUCCESS;    
}


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
    uint8_t buff[2+2*6],*data_p;
    uint16_t data_len;

    att_err = BLE_ATT_ERR_NONE;
    if(Attribute_Handle == AccGyroMagCharHandle + 1)
    {
        AxesRaw_t x_axes, g_axes;
        if (GetAccAxesRaw(&x_axes, &g_axes)) 
        { 
          uint32_t time = HAL_VTIMER_DiffSysTimeMs(HAL_VTIMER_GetCurrentSysTime(), start_time);
          
          HOST_TO_LE_16(buff,time);
          
          HOST_TO_LE_16(buff+2,-x_axes.AXIS_Y);
          HOST_TO_LE_16(buff+4, x_axes.AXIS_X);
          HOST_TO_LE_16(buff+6,-x_axes.AXIS_Z);
            
          HOST_TO_LE_16(buff+8,g_axes.AXIS_Y);
          HOST_TO_LE_16(buff+10,g_axes.AXIS_X);
          HOST_TO_LE_16(buff+12,g_axes.AXIS_Z);
          data_p = buff;
          data_len = 2+2*6;
        }
        else
        {
            att_err = BLE_ATT_ERR_APPL_MIN;
        }
    }
    else if(Attribute_Handle == EnvironmentalCharHandle + 1)
    {
      float temp_val, press_val;
      GetPressure(&press_val);
      GetTemperature(&temp_val);
      
      HOST_TO_LE_32(buff+2,(int32_t)(press_val *100));
      HOST_TO_LE_16(buff+6,(int16_t)(temp_val * 10));
      data_len = 8;
      data_p = buff;
    }
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT 
    OTA_Read_Char(Connection_Handle, Attribute_Handle,Data_Offset); 
#endif 

    aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, att_err, data_len, data_p);
}

tBleStatus BlueMS_Environmental_Update(int32_t press, int16_t temp)
{  
  tBleStatus ret;
  uint8_t buff[8];
  uint32_t time = HAL_VTIMER_DiffSysTimeMs(HAL_VTIMER_GetCurrentSysTime(), start_time);
  HOST_TO_LE_16(buff, time);
  
  HOST_TO_LE_32(buff+2,press);
  HOST_TO_LE_16(buff+6,temp);
  
  ret =  aci_gatt_srv_notify(connection_handle, EnvironmentalCharHandle + 1, 0,8, buff);
  if (ret != BLE_STATUS_SUCCESS){
          PRINTF("Error while updating TEMP characteristic: 0x%04X\n",ret) ;
          return BLE_STATUS_ERROR ;
  }
  
  return BLE_STATUS_SUCCESS;	
}

