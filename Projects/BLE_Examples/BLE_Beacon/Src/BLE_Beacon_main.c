/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
* File Name          : BLE_Beacon_main.c
* Author             : AMS - RF Application Team
* Version            : V1.0.0
* Date               : 8-June-2020
* Description        : Main file for beacon device
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/** @addtogroup BlueNRGLP_demonstrations_applications
 *  BlueNRG-LP Beacon demo \see BLE_Beacon_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "Beacon_config.h"
#include "OTA_btl.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_BEACON_VERSION_STRING "2.1"

/* Advertising interval for legacy advertising (0.625 ms units) 
  For iBeacon this should be set to 100 ms. */
#define LEGACY_ADV_INTERVAL     160  /* 100 ms */
/* Advertising interval for extended advertising (0.625 ms units) */
#if PERIODIC_ADV
/* If periodic advertising is used. Reduce advertising rate to save power. */
#define EXT_ADV_INTERVAL        1600 /* 1 s */
#else
#define EXT_ADV_INTERVAL        160 /* 100 ms */
#endif
/* Advertising interval for periodic advertising (1.25 ms units) */
#define PERIODIC_ADV_INTERVAL   240 /* 300 ms*/

/* Set to 1 to enable the name AD data in extended advertising events (if
  extended advertising events are used).  */
#define DEVICE_NAME_IN_ADV 0

/* PHY used in extended advertising events. One between: LE_1M_PHY,
  LE_2M_PHY and LE_CODED_PHY (LE_CODED_PHY not possible for direction finding).  */
#define EXT_ADV_PHY LE_1M_PHY


/* Parameters for Direction Finding */
#define CTE_LENGTH                  20
#define CTE_TYPE                    0   /* 0: AoA, 1: AoD with 1 us slots, 2: AoD with 2 us slots */
#define CTE_COUNT                   4
#define SWITCHING_PATTERN_LENGTH    0 /* Ignored for AoA */
#define ANTENNA_IDS                 NULL


/* Private macro -------------------------------------------------------------*/
#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/* Private variables ---------------------------------------------------------*/

/* This is the length of advertising data to be set for legacy advertising and
   periodic advertising.  It does not include the device name, which is sent
   only in extended advertising events. */
#define SHORT_ADV_DATA_LENGTH    27

static uint8_t adv_data[] = {
  /* Advertising data: Flags AD Type not supported if broadcaster only */
  /* 0x02, 0x01, 0x06, */
  /* Advertising data: manufacturer specific data */
  26, //len
  AD_TYPE_MANUFACTURER_SPECIFIC_DATA,  //manufacturer type
  0x4C, 0x00, //Company identifier code
  0x02,       // ID
  0x15,       //Length of the remaining payload
  0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
  0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
  0x00, 0x05, // Major number 
  0x00, 0x07, // Minor number 
  (uint8_t)-56,         // Tx power measured at 1 m of distance (in dBm)
#if DEVICE_NAME_IN_ADV
  15,       // Length of following AD data
  0x09,'E','x','t','e','n','d','e','d','B','e','a','c','o','n'
#endif
};

NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void ModulesInit(void)
{
  uint8_t ret;
  BLE_STACK_InitTypeDef BLE_STACK_InitParams = BLE_STACK_INIT_PARAMETERS;
  
  LL_AHB_EnableClock(LL_AHB_PERIPH_PKA|LL_AHB_PERIPH_RNG);

  
  BLECNTR_InitGlobal();
  
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
  BLEPLAT_Init();  
  if (PKAMGR_Init() == PKAMGR_ERROR)
  {
      while(1);
  }
  if (RNGMGR_Init() != RNGMGR_SUCCESS)
  {
      while(1);
  }
  
  /* Init the AES block */
  AESMGR_Init();
  
  /* BlueNRG-LP stack init */
  ret = BLE_STACK_Init(&BLE_STACK_InitParams);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BLE_STACK_Init() 0x%02x\r\n", ret);
    while(1);
  }
  
}

void ModulesTick(void)
{
  /* Timer tick */
  HAL_VTIMER_Tick();
  
  /* Bluetooth stack tick */
  BLE_STACK_Tick();
  
  /* NVM manager tick */
  NVMDB_Tick();
}

void Device_Init(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t address[CONFIG_DATA_PUBADDR_LEN] = {0x66,0x77,0x88,0xE1,0x80,0x02};
  
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, address);
  
  /* Set the TX Power to 0 dBm */
  ret = aci_hal_set_tx_power_level(0, 24);
  if(ret != 0) {
    PRINTF ("Error in aci_hal_set_tx_power_level() 0x%04xr\n", ret);
    while(1);
  }

  
  /* Init the GAP: broadcaster role */
  ret = aci_gap_init(GAP_BROADCASTER_ROLE, 0x00, 0x08, PUBLIC_ADDR, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if (ret != 0)
  {
    PRINTF ("Error in aci_gap_init() 0x%04x\r\n", ret);
  }
  else
  {
    PRINTF ("aci_gap_init() --> SUCCESS\r\n");
  }
}

/**
* @brief  Start beaconing
* @param  None 
* @retval None
*/
static void Start_Beaconing(void)
{  
  uint8_t ret;
  Advertising_Set_Parameters_t Advertising_Set_Parameters[2];
  uint8_t adv_sets = 0;
  
#ifdef LEGACY_ADV
   
  /* Set advertising configuration for legacy advertising in broadcast mode */  
  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_BROADCAST,
                                              ADV_PROP_LEGACY,
                                              LEGACY_ADV_INTERVAL, LEGACY_ADV_INTERVAL,
                                              ADV_CH_ALL,
                                              0,NULL, /* No peer address */
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error in aci_gap_set_advertising_configuration() 0x%02x\r\n", ret);
    return;
  }
  
  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, SHORT_ADV_DATA_LENGTH, adv_data);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error in aci_gap_set_advertising_data() 0x%02x\r\n", ret);
    return;
  }
  
  printf("Legacy advertising configured\n");
  
  Advertising_Set_Parameters[adv_sets].Advertising_Handle = 0;
  Advertising_Set_Parameters[adv_sets].Duration = 0;
  Advertising_Set_Parameters[adv_sets].Max_Extended_Advertising_Events = 0;
  
  adv_sets++;
  
#endif
  
#ifdef EXTENDED_ADV
  /* Set advertising configuration for extended advertising in broadcast mode */  
  ret = aci_gap_set_advertising_configuration(1, GAP_MODE_BROADCAST,
                                              ADV_PROP_NONE,
                                              EXT_ADV_INTERVAL, EXT_ADV_INTERVAL,
                                              ADV_CH_ALL,
                                              0,NULL, /* No peer address */
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              (EXT_ADV_PHY==LE_2M_PHY)?LE_1M_PHY:EXT_ADV_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              EXT_ADV_PHY, /* Secondary advertising PHY */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error in aci_gap_set_advertising_configuration() 0x%02x\r\n", ret);
    return;
  }
  
  ret = aci_gap_set_advertising_data(1, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error in aci_gap_set_advertising_data() 0x%02x\r\n", ret);
    return;
  }
  
  printf("Extended advertising configured\n");
  
  Advertising_Set_Parameters[adv_sets].Advertising_Handle = 1;
  Advertising_Set_Parameters[adv_sets].Duration = 0;
  Advertising_Set_Parameters[adv_sets].Max_Extended_Advertising_Events = 0;
  
  adv_sets++;
  
#endif /* EXTENDED_ADV */
  
#ifdef PERIODIC_ADV
  /* Configure periodic advertising */
  ret = aci_gap_set_periodic_advertising_configuration(1, PERIODIC_ADV_INTERVAL, PERIODIC_ADV_INTERVAL,  0 );                              
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error in aci_gap_set_periodic_advertising_configuration() 0x%02x\r\n", ret);
    return;
  }    
      
  /* Set periodic advertising data: beacon payload */
  ret = aci_gap_set_periodic_advertising_data(1, SHORT_ADV_DATA_LENGTH, adv_data);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error in aci_gap_set_periodic_advertising_data() 0x%02x\r\n", ret);
    return;
  }
    
#if CTE_TAG  
  ret = hci_le_set_connectionless_cte_transmit_parameters(1, CTE_LENGTH, CTE_TYPE, CTE_COUNT, SWITCHING_PATTERN_LENGTH, ANTENNA_IDS);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error in hci_le_set_connectionless_cte_transmit_parameters() 0x%02x\r\n", ret);
    return;
  }
  
  ret = hci_le_set_connectionless_cte_transmit_enable(1, 1);
  
  printf("CTE configured\n");
#endif
  
  /* Enable periodic advertising */
  ret = aci_gap_set_periodic_advertising_enable(ENABLE, 1);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error in aci_gap_set_periodic_advertising_enable() 0x%02x\r\n", ret);
    return;
  }
            
  printf("Periodic advertising configured\n");

#endif /* PERIODIC_ADV */  
  
  /* Enable advertising */
  ret = aci_gap_set_advertising_enable(ENABLE, adv_sets, Advertising_Set_Parameters);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF ("Error in aci_gap_set_advertising_enable() 0x%02x\r\n", ret);
    return;
  }
  
  printf("Advertising started\n");

}

int main(void) 
{
  WakeupSourceConfig_TypeDef wakeupIO;
  PowerSaveLevels stopLevel;

  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) 
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  /* Configure IOs for power save modes */
  BSP_IO_Init();
  /* Init the UART peripheral */
  BSP_COM_Init(NULL);
  /* Init BLE stack, HAL virtual timer and NVM modules */
  ModulesInit(); 
  /* Init the Bluetooth LE stack layesrs */
  Device_Init();
#if CONFIG_OTA_USE_SERVICE_MANAGER
  /* Initialize the button */
  BSP_PB_Init(USER_BUTTON, BUTTON_MODE_GPIO); 
#endif /* CONFIG_OTA_USE_SERVICE_MANAGER */
  
  printf("BlueNRG-LP BLE Beacon Application (version: %s)\r\n", BLE_BEACON_VERSION_STRING); 
 
  /* Start beaconing */
  Start_Beaconing();
  
  /* No Wakeup Source needed */
  wakeupIO.IO_Mask_High_polarity = 0;
  wakeupIO.IO_Mask_Low_polarity = 0;
  wakeupIO.RTC_enable = 0;          
  wakeupIO.LPU_enable = 0;
 
  while(1) 
  {
    ModulesTick();
    
    /* Power Save Request */
    HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, wakeupIO, &stopLevel); 
     
#if CONFIG_OTA_USE_SERVICE_MANAGER
    if (BSP_PB_GetState(USER_BUTTON) == SET)
    {
      OTA_Jump_To_Service_Manager_Application();
    }
#endif /* CONFIG_OTA_USE_SERVICE_MANAGER */
  }
}

/****************** BlueNRG-LP Power Management Callback ********************************/

PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
    return POWER_SAVE_LEVEL_RUNNING;
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}

/* Event used to notify the Host that a hardware failure has occurred in the Controller. 
   See bluenrg_lp_events.h. */
void hci_hardware_error_event(uint8_t Hardware_Code)
{
  if (Hardware_Code <= 0x03)
  {
    NVIC_SystemReset();
  }
}


#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
