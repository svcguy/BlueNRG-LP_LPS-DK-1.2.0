
#include <stdio.h>
#include <string.h>
#include "gp_timer.h"
#include "ble_const.h" 
#include "bluenrg_lp_stack.h"
#include "app_state.h"
#include "osal.h"
#include "gatt_db.h"
#include "profile.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "procedures.h"

#define ADV_INTERVAL_MIN    ((uint16_t)(100/0.625))     // 100 ms
#define ADV_INTERVAL_MAX    ((uint16_t)(100/0.625))     // 100 ms

#define DEBUG 2

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#if DEBUG > 1
#include <stdio.h>
#define PRINTF_DBG2(...) printf(__VA_ARGS__)
#else
#define PRINTF_DBG2(...)
#endif



const char name[] = LOCAL_NAME;
#define NAME_LENGTH (sizeof(name)-1)

#if PROFILE_ROLE == ROLE_PERIPHERAL

static uint8_t scan_resp_data[18];

static uint8_t adv_data[5+NAME_LENGTH] = {
  0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
  NAME_LENGTH+1, AD_TYPE_COMPLETE_LOCAL_NAME};

#define DISCOVERABLE_ADV_DATA_LEN       sizeof(adv_data)

#endif /* PROFILE_ROLE == ROLE_PERIPHERAL */

#if PROFILE_ROLE == ROLE_CENTRAL
tBleStatus StartGeneralConnectionEstablishment(void)
{
  tBleStatus ret;
  
  ret = aci_gap_start_procedure(GAP_GENERAL_CONNECTION_ESTABLISHMENT_PROC, LE_1M_PHY_BIT, 0, 0);
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error while starting scanning: 0x%02X\r\n", ret);
    return ret;
  }
  else {
    PRINTF("Scanning...\r\n");
  }
  
  return BLE_STATUS_SUCCESS;
}
#endif 

#if PROFILE_ROLE == ROLE_PERIPHERAL
tBleStatus ConfigureAdvertising(void)
{
  tBleStatus ret;
  
  adv_data[2] = FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED;    
  // Add name to advertising data
  Osal_MemCpy(adv_data+5,name,NAME_LENGTH);
  scan_resp_data[0] = 17;
  scan_resp_data[1] = AD_TYPE_SERV_SOLICIT_128_BIT_UUID_LIST;
  Osal_MemCpy(scan_resp_data+2,sync_service_uuid,sizeof(sync_service_uuid));
  
  ret = aci_gap_set_advertising_configuration(0x00, // Advertising handle
                                              0x02, // General discoverable mode
                                              0x0013, // Connectable, Scannable, Legacy
                                              ADV_INTERVAL_MIN,
                                              ADV_INTERVAL_MAX,
                                              ADV_CH_ALL,
                                              0, NULL, // No peer address
                                              ADV_NO_WHITE_LIST_USE,
                                              127, // No preference for TX power
                                              LE_1M_PHY, // Primary_Advertising_PHY (not used for legacy adv)
                                              0, // Secondary_Advertising_Max_Skip (not used for legacy adv)
                                              LE_1M_PHY, //  Secondary_Advertising_PHY (not used for legacy adv)
                                              0, // Advertising_SID (not used for legacy adv)
                                              0); // No scan request notification
  PRINTF("Advertising configuration (discoverable) 0x%02X\n", ret);
  if(ret)
    return ret;
  
  ret = aci_gap_set_scan_response_data(0x00, sizeof(scan_resp_data), scan_resp_data);
  if(ret)
    return ret;
  
  ret = aci_gap_set_advertising_data(0x00, // Advertising handle
                                     0x03, // Complete data
                                     DISCOVERABLE_ADV_DATA_LEN, adv_data);
  PRINTF("aci_gap_set_advertising_data 0x%02X\n", ret);
  
  return ret;
  
}

tBleStatus StartAdvertising()
{
  tBleStatus ret;
  Advertising_Set_Parameters_t Advertising_Set_Parameters = {
    .Advertising_Handle = 0,
    .Duration = 0,
    .Max_Extended_Advertising_Events = 0,
  };
  
  ret = aci_gap_set_advertising_enable(ENABLE,1,&Advertising_Set_Parameters);
  
  PRINTF("Enable advertising 0x%02X\n", ret);
  return ret;
}

#endif

