
#include <stdio.h>
#include <string.h>
#include "ble_const.h" 
#include "bluenrg_lp_stack.h"
#include "osal.h"
#include "app_state.h"
#include "serial_port.h"
#include "SerialPort_config.h"

BLE_GATT_SRV_CCCD_DECLARE(tx, NUM_LINKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                     BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);


/* Serial port TX (notification), RX(write without response)  characteristics definition */
static const ble_gatt_chr_def_t serial_port_chars[] = {
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
        .properties = BLE_GATT_SRV_CHAR_PROP_WRITE | BLE_GATT_SRV_CHAR_PROP_WRITE_NO_RESP,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(RX_CHR_UUID),
    },
};

/* Serial port Service definition */
static const ble_gatt_srv_def_t serial_port_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(SRVC_UUID),
   .chrs = {
       .chrs_p = (ble_gatt_chr_def_t *)serial_port_chars,
       .chr_count = 2U,
   },
};

uint16_t TXCharHandle, RXCharHandle;


/*******************************************************************************
* Function Name  : Add_Serial_port_Service
* Description    : Add the serial port service.
* Input          : None
* Return         : Status.
*******************************************************************************/
uint8_t Add_Serial_port_Service(void)
{
  uint8_t ret;

    ret = aci_gatt_srv_add_service((ble_gatt_srv_def_t *)&serial_port_service);
    if (ret != BLE_STATUS_SUCCESS)
    {
        goto fail;
    }
    TXCharHandle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&serial_port_chars[0]);
    RXCharHandle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&serial_port_chars[1]);

    printf("Serial Port Service added.\nTX Char Handle %04X, RX Char Handle %04X\n",
            TXCharHandle, RXCharHandle);

  return BLE_STATUS_SUCCESS; 

fail:
  printf("Error while adding Serial port service.\n");
  return BLE_STATUS_ERROR ;
}



