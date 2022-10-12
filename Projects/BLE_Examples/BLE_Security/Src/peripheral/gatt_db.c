
#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "osal.h"
#include "app_state.h"
#include "gatt_db.h"
#include "BLE_Security_Peripheral.h"
#include "BLE_Security_Peripheral_config.h"

/*
  UUIDs:
  D973F2E0-B19E-11E2-9E96-0800200C9A66
  D973F2E1-B19E-11E2-9E96-0800200C9A66
  D973F2E2-B19E-11E2-9E96-0800200C9A66
  */

#define SRVC_UUID   0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9
#define TX_CHR_UUID 0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9
#define RX_CHR_UUID 0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9
  
#if (MITM_MODE == MITM_PROTECTION_REQUIRED)
BLE_GATT_SRV_CCCD_DECLARE(tx, NUM_LINKS, BLE_GATT_SRV_PERM_AUTHEN_WRITE | BLE_GATT_SRV_PERM_ENCRY_WRITE,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);
#else

BLE_GATT_SRV_CCCD_DECLARE(tx, NUM_LINKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);
#endif


/* Security TX (notification), RX(read)  characteristics definition */
static const ble_gatt_chr_def_t security_chars[] = {
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(TX_CHR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(tx),
            .descr_count = 1U,
        },
    },
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_READ,
        .permissions = RX_CHAR_SECURITY_PERMISSIONS,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(RX_CHR_UUID),
    },
};

/* Security Service definition */
static const ble_gatt_srv_def_t security_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(SRVC_UUID),
   .chrs = {
       .chrs_p = (ble_gatt_chr_def_t *)security_chars,
       .chr_count = 2U,
   },
};

uint16_t TXCharHandle, RXCharHandle;

static uint16_t rx_counter = 0; 
static uint16_t tx_counter = 0; 
extern volatile uint16_t connection_handle;

/*******************************************************************************
* Function Name  : Add_Service
* Description    : Add the Slave Security service.
* Input          : None
* Return         : Status.
*******************************************************************************/
uint8_t Add_Service(void)
{
  uint8_t ret;

  ret = aci_gatt_srv_add_service((ble_gatt_srv_def_t *)&security_service);
    if (ret != BLE_STATUS_SUCCESS)
    {
        goto fail;
    }
  TXCharHandle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&security_chars[0]);
  RXCharHandle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&security_chars[1]);
    
  printf("Device Service added.\nTX Char Handle %04X, RX Char Handle %04X\n", TXCharHandle, RXCharHandle);
  
   return BLE_STATUS_SUCCESS; 
fail:
  printf("Error while adding device ervice.\n");
  return BLE_STATUS_ERROR ;
}

/*******************************************************************************
* Function Name  : read_data_handler
* Description    : Updated RX char value for the Client read
* Input          : None.
* Return         : None.
*******************************************************************************/
uint8_t read_data_handler(void)
{
  /* read data handler */
  uint8_t ret= BLE_STATUS_SUCCESS; 
  uint8_t buff_rx[20] = {'S','L','A','V','E','_','S','E','C','U','R','I','T','Y','_','R','X',' ',0,0};
  
  STORE_VALUE(&buff_rx[18],rx_counter);
  rx_counter +=1;
  
  ret = aci_gatt_srv_resp(connection_handle, RXCharHandle, 0, 20, buff_rx); 
  if (ret != BLE_STATUS_SUCCESS){
    printf("Error while aci_gatt_srv_resp on  RXCharHandle characteristic: 0x%02X\n",ret) ;
    return BLE_STATUS_ERROR ;
  }  
  
  return ret;
}

/*******************************************************************************
* Function Name  : notification_data_handler
* Description    : Updated TX char value x notification to Client
* Input          : None.
* Return         : None.
*******************************************************************************/
uint8_t notification_data_handler(void)
{
  uint8_t ret= BLE_STATUS_SUCCESS; 
  
  uint8_t buff_tx[20] = {'S','L','A','V','E','_','S','E','C','U','R','I','T','Y','_', 'T','X',' ',0,0};
  
  /* notification */
  STORE_VALUE(&buff_tx[18],tx_counter);
  tx_counter = tx_counter +1;

  ret = aci_gatt_srv_notify(connection_handle, TXCharHandle + 1, 0, 20, buff_tx);
  if (ret != BLE_STATUS_SUCCESS){
    printf("Error while updating TXCharHandle characteristic: 0x%02X\r\n",ret) ;
    return BLE_STATUS_ERROR ;
  }
  return ret;
}
  

