/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : C_main.c
* Author             : AMS RF Application Team
* Version            : V1.0.0
* Date               : 04-April-2019
* Description        : BlueNRG-LP Security central role main file
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/** @addtogroup BlueNRGLP_demonstrations_applications
 * BlueNRG-LP Security Central \see C_main.c for documentation.
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
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "gatt_db.h"
#include "master_basic_profile.h"
#include "master_config.h" 
#include "BLE_Security_Central_config.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"

#define BLE_APPLICATION_VERSION_STRING "1.0.0" 

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/
static char Security_Configuration[] = SECURITY_MODE_NAME; 

/* Uart RX */
static WakeupSourceConfig_TypeDef wakeupIO = {0, 0, WAKEUP_PA8, 0};

NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

/* Private function prototypes -----------------------------------------------*/

static void user_set_wakeup_source(Button_TypeDef Button)
{
  if (Button == BSP_PUSH1)
  {
     /* IRQ_ON_FALLING_EDGE is set since wakeup level is low (mandatory setup for wakeup sources);  
       IRQ_ON_RISING_EDGE is the specific application level */    
    wakeupIO.IO_Mask_High_polarity |= WAKEUP_PA10;
  }
}
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

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{    
  uint8_t ret;
  uint32_t powersave_state = 0; 
  PowerSaveLevels stopMode = POWER_SAVE_LEVEL_STOP_NOTIMER;
  PowerSaveLevels stopLevel;
	
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) 
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }  
  /* Configure IOs for pwer save modes */
  BSP_IO_Init();
  
  /* Configure I/O communication channel */
  BSP_COM_Init(BSP_COM_RxDataUserCb);
  
  ModulesInit(); 
  
  /* Master Init procedure */
  if (deviceInit() != BLE_STATUS_SUCCESS) {
    PRINTF("Error during the device init procedure\r\n");
  }
  
  /* Master set security */
  ret = Device_Security();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in Device_Security() 0x%02x\r\n", ret);
    while(1);
  }
    
  /* Initialize the button */
  BSP_PB_Init(USER_BUTTON, BUTTON_MODE_EXTI); 
  user_set_wakeup_source(USER_BUTTON);
  
  PRINTF("BlueNRG-LP BLE Security Central Application (version: %s, security mode: %s, button: %d)\r\n", BLE_APPLICATION_VERSION_STRING,Security_Configuration, USER_BUTTON);
   
  
  /* Start Master Configure Scanning procedure */
  if (deviceScanConfiguration() != BLE_STATUS_SUCCESS) {
    PRINTF("Error during the Scanning procedure configuration\r\n");
  }  
  
  /* Start Master Configure Connection procedure */
  if (deviceConnectionConfiguration() != BLE_STATUS_SUCCESS) {
    PRINTF("Error during the Connection procedure configuration\r\n");
  } 
  
  /* Start Master Device Discovery procedure */
  if (deviceDiscovery() != BLE_STATUS_SUCCESS) {
    PRINTF("Error during the device discovery procedure\r\n");
  }
  
  /* Main loop */
  while(1) {
    
    ModulesTick();
    
    /* Application Tick */
    APP_Tick();
		
    /* Master Tick */
    powersave_state = Master_Process(); 

    /* Check master library power state LSB bit: if 1 micro can enter in low powe mode */
    (powersave_state & 0x1) ? (stopMode = POWER_SAVE_LEVEL_STOP_NOTIMER) : (stopMode = POWER_SAVE_LEVEL_RUNNING);   
    
    /* Power Save management: no timer and wakeup on UART RX, PUSH button */
    HAL_PWR_MNGR_Request(stopMode, wakeupIO, &stopLevel);
    
  }
}

/* User callback if an interrupt is associated to the wakeup source */
void HAL_PWR_MNGR_WakeupIOCallback(uint32_t source)
{
  if (source & WAKEUP_PA10) {    
     /* When connected BUTTON_1 allows to disconnect */
    if  (masterContext.isconnected)
    {
      masterContext.do_terminate_connection = 1;
    }
  }
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


/**
  * This event is generated to report firmware error informations.
  * FW_Error_Type possible values: 
  * Values:
  - 0x01: L2CAP recombination failure
  - 0x02: GATT unexpected response
  - 0x03: GATT unexpected request
    After this event with error type (0x01, 0x02, 0x3) it is recommended to disconnect. 
*/
void aci_hal_fw_error_event(uint8_t FW_Error_Type,
                            uint8_t Data_Length,
                            uint8_t Data[])
{
  if (FW_Error_Type <= 0x03)
  {
    uint16_t connHandle;
    
    /* Data field is the connection handle where error has occurred */
    connHandle = LE_TO_HOST_16(Data);
    
    aci_gap_terminate(connHandle, BLE_ERROR_TERMINATED_REMOTE_USER); 
  }
}


#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
