
#include <stdio.h>
#include <string.h>
#include "ble_const.h" 
#include "bluenrg_lp_stack.h"
#include "osal.h"
#include "app_state.h"
#include "profile.h"
#include "gatt_db.h"
#include "Sync_config.h"

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

uint16_t vclockCharHandle;

/*
  Sync service UUIDs:
  3295a048-e6ed-11e7-80c1-9a214cf093ae
  c641900b-846a-4aa6-9b55-984729b4b7da
  */
#define SYNC_SRVC_UUID           0xae,0x93,0xf0,0x4c,0x21,0x9a,0xc1,0x80,0xe7,0x11,0xed,0xe6,0x48,0xa0,0x95,0x32
#define SYNC_CLOCK_CHR_UUID     0xda,0xb7,0xb4,0x29,0x47,0x98,0x55,0x9b,0xa6,0x4a,0x6a,0x84,0x0b,0x90,0x41,0xc6

const uint8_t sync_service_uuid[16] = {SYNC_SRVC_UUID};
const uint8_t vclock_char_uuid[16] =  {SYNC_CLOCK_CHR_UUID};


static const ble_gatt_chr_def_t sync_chars[] = {
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_READ,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .uuid = BLE_UUID_INIT_128(SYNC_CLOCK_CHR_UUID),
    },
};

/* Sync Service definition */
static const ble_gatt_srv_def_t sync_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(SYNC_SRVC_UUID),
   .chrs = {
       .chrs_p = (ble_gatt_chr_def_t *)sync_chars,
       .chr_count = 1U,
   },
};

/*******************************************************************************
* Function Name  : Add_Sync_Service
* Description    : Add the Sync service. This service has two writable
*                  characteristics.
* Input          : None
* Return         : Status.
*******************************************************************************/
uint8_t Add_Sync_Service(void)
{
  uint8_t ret;
  
  ret = aci_gatt_srv_add_service((ble_gatt_srv_def_t *)&sync_service);
  if (ret != BLE_STATUS_SUCCESS)
  {
    goto fail;
  }
  
  vclockCharHandle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&sync_chars[0]);
  
  printf("Sync Service added.\nVClock handle 0x%04X\n",vclockCharHandle);
  
  return BLE_STATUS_SUCCESS; 

fail:
  printf("Error while adding Sync service.\n");
  return BLE_STATUS_ERROR ;
}
