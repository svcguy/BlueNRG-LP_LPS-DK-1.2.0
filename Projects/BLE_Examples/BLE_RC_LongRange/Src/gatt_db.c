#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "app_state.h"
#include "gatt_db.h"
#include "osal.h"
#include "bluenrg_lp_evb_config.h"
#include "rc.h"

#ifndef DEBUG
#define DEBUG 1
#endif

#define ENABLE_SECURITY         0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define LED_NUMBER 3

/*
  UUIDs:
  ed0ef62e-9b0d-11e4-89d3-123b93f75cba
  ed0efb1a-9b0d-11e4-89d3-123b93f75cba
  */

#define RC_SRVC_UUID  0xba,0x5c,0xf7,0x93,0x3b,0x12,0xd3,0x89,0xe4,0x11,0x0d,0x9b,0x2e,0xf6,0x0e,0xed
#define RC_CONTROL_POINT_UUID 0xba,0x5c,0xf7,0x93,0x3b,0x12,0xd3,0x89,0xe4,0x11,0x0d,0x9b,0x1a,0xfb,0x0e,0xed  

/* RC service, control point characteristics definition */
static const ble_gatt_chr_def_t rc_chars[] = {
    /* Control point characteristic */ 
 #if ENABLE_SECURITY
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_WRITE | BLE_GATT_SRV_CHAR_PROP_WRITE_NO_RESP | BLE_GATT_SRV_CHAR_PROP_AUTH_SIGN_WRITE,
        .permissions = BLE_GATT_SRV_PERM_AUTHEN_READ|BLE_GATT_SRV_PERM_AUTHEN_WRITE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(RC_CONTROL_POINT_UUID), 
    },   
#else
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_WRITE_NO_RESP| BLE_GATT_SRV_CHAR_PROP_WRITE,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(RC_CONTROL_POINT_UUID),
    },   
#endif 
};

/* RC Service definition */
static const ble_gatt_srv_def_t rc_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(RC_SRVC_UUID),
   .chrs = {
       .chrs_p = (ble_gatt_chr_def_t *)rc_chars,
       .chr_count = 1U,
   },
};    

uint16_t controlPointHandle;
  
/*******************************************************************************
* Function Name  : Add_Chat_Service
* Description    : Add the 'Accelerometer' service.
* Input          : None
* Return         : Status.
*******************************************************************************/
tBleStatus Add_RC_Service(void)
{
  tBleStatus ret;

  ret = aci_gatt_srv_add_service((ble_gatt_srv_def_t *)&rc_service);
  if (ret != BLE_STATUS_SUCCESS)
  {
    goto fail;
  }
  controlPointHandle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&rc_chars[0]);

  PRINTF("RC Service added.\nControl Point Char Handle %04X\n", controlPointHandle);
  return BLE_STATUS_SUCCESS; 
      
fail:
  PRINTF("Error while adding RC service.\n");
  return BLE_STATUS_ERROR ;
}

static uint8_t leds_value[2] = {0,0};

void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
  if(handle == controlPointHandle + 1){
    
    leds_value[0] = att_data[0];
    leds_value[1] = att_data[1];
    
    if(att_data[0]&(1<<CONTROL_LED))
      BSP_LED_On(CONTROL_LED);
    else
      BSP_LED_Off(CONTROL_LED);
      
    /*
    uint8_t led_num = LED_NUMBER;
    
    PRINTF("Command received.\n");
    
    for(int i = 0; i < led_num; i++){ 
        if((att_data[0]>>i)&0x01){
          PRINTF("LED %d ON\n",i+1);
          BSP_LED_On((Led_TypeDef)(i));
        }
        else {
          PRINTF("LED %d OFF\n",i+1);
          BSP_LED_Off((Led_TypeDef)(i)); 
        }
    }*/
  }
}

void aci_gatt_srv_write_event(uint16_t Connection_Handle, uint8_t Resp_Needed, uint16_t Attribute_Handle, uint16_t Data_Length, uint8_t Data[])
{
    uint8_t att_error = BLE_ATT_ERR_NONE;
    
    Attribute_Modified_CB(Attribute_Handle, Data_Length, Data); 
  
    if (Resp_Needed == 1U)
    {
        aci_gatt_srv_resp(Connection_Handle, 0, att_error, 0, NULL);
    }
}

void aci_gatt_srv_read_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Data_Offset)
{
  uint8_t att_err;

  att_err = BLE_ATT_ERR_NONE;
  if(Attribute_Handle == controlPointHandle + 1)
  {
    aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, att_err, 2, leds_value);
  }
}

