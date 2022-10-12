
#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "osal.h"
#include "app_state.h"
#include "throughput.h"
#include "bluenrg_lp_api.h"
#include "Throughput_config.h"

uint16_t TXCharHandle, RXCharHandle;

#if SERVER
BLE_GATT_SRV_CCCD_DECLARE(tx, NUM_LINKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT, BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);

static const ble_gatt_chr_def_t throughput_chars[] = {
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_NOTIFY | CHAR_PROP_INDICATE,
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

static const ble_gatt_srv_def_t throughput_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(SRVC_UUID),
   .chrs = {
       .chrs_p = (ble_gatt_chr_def_t *)throughput_chars,
       .chr_count = 2U,
   },
};

/*******************************************************************************
 * Function Name  : Add_Throughput_Service
 * Description    : Add the 'Throughput' service.
 * Input          : None
 * Return         : Status.
 *******************************************************************************/
uint8_t Add_Throughput_Service(void)
{
    uint8_t ret;

    ret = aci_gatt_srv_add_service((ble_gatt_srv_def_t *)&throughput_service);
    if (ret != BLE_STATUS_SUCCESS)
    {
        printf("Error while adding Throughput service.\n");

        return BLE_STATUS_ERROR;
    }

    TXCharHandle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&throughput_chars[0]);
    RXCharHandle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&throughput_chars[1]);

    printf("Throughput Service added.\nTX Char Handle %04X, RX Char Handle %04X\n",
           TXCharHandle, RXCharHandle);

    return BLE_STATUS_SUCCESS;
}
#endif
