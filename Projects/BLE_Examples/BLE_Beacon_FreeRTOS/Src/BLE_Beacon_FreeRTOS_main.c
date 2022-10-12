/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : BLE_Beacon_FreeRTOS_main.c
* Author             : AMS - RF Application Team
* Description        : FreeRTOS example that show concurrent operations of user 
*                      tasks and BLE stack.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/** @addtogroup BlueNRGLP_demonstrations_applications
 *  BlueNRG-LP Beacon FreeRTOS demo \see BLE_Beacon_FreeRTOS_main.c for documentation.
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


/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "freertos_ble.h"

/* Binary semaphore used to synchronize Stack Tick and radio ISR. */
SemaphoreHandle_t radioActivitySemaphoreHandle;
/* Mutex used to avoid that the BLE Stack Tick can be interrupted by an ACI
   function in another thread. */
SemaphoreHandle_t BLETickSemaphoreHandle;
/* Mutex used to access UART resource */
SemaphoreHandle_t UARTSemaphoreHandle;

#define BLE_BEACON_VERSION_STRING "1.1"

/* Advertising interval for legacy advertising (0.625 ms units) 
  For iBeacon this should be set to 100 ms. */
#define LEGACY_ADV_INTERVAL     160  /* 100 ms */
#define EXT_ADV_INTERVAL        160 /* 100 ms */

/* Set to 1 to enable the name AD data in extended advertising events (if
  extended advertising events are used).  */
#define DEVICE_NAME_IN_ADV 0

/* PHY used in extended advertising events. One between: LE_1M_PHY,
  LE_2M_PHY and LE_CODED_PHY.  */
#define EXT_ADV_PHY LE_CODED_PHY

/*-----------------------------------------------------------*/
/* Priorities at which the tasks are created.
   Assigning an high priority to BLE Task can give better latency, especially
   if other tasks are CPU resource hungry.
*/
#define TEST_TASK_PRIORITY		            ( tskIDLE_PRIORITY + 2 )
#define	BLE_TASK_PRIORITY		            ( tskIDLE_PRIORITY + 1 )

/*-----------------------------------------------------------*/
/* Wait time of the test task (numbe rof ticks) */
#define TEST_PERIOD         			    ( 200 / portTICK_PERIOD_MS )
#define ADV_CHANGE_PERIOD         			( 500 / portTICK_PERIOD_MS )

/* Private macro -------------------------------------------------------------*/
#define DEBUG 1 

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) do{ xSemaphoreTake(UARTSemaphoreHandle, portMAX_DELAY);\
                      printf(__VA_ARGS__);                              \
                      xSemaphoreGive(UARTSemaphoreHandle); }while(0)
                        
#else
#define PRINTF(...)
#endif
                        
/* Private variables ---------------------------------------------------------*/

/* This is the length of advertising data to be set for legacy advertising.
   It does not include the device name, which is sent only in extended
  advertising events. */
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

void createTasks( void );

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
  
  UARTSemaphoreHandle = xSemaphoreCreateMutex();
  
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED3);
  
  BSP_LED_On(BSP_LED1);
  
  
   /* Create a binary semaphore to sync with radio interrupts */
  radioActivitySemaphoreHandle = xSemaphoreCreateBinary();
  /* Create a mutex semaphore to avoid calling aci functions while
    BTLE_StackTick() is running.*/
  BLETickSemaphoreHandle = xSemaphoreCreateMutex();
  if(radioActivitySemaphoreHandle==NULL || BLETickSemaphoreHandle == NULL){
    while(1);
  }
    
  ModulesInit(); 
  
  PRINTF("BlueNRG-LP BLE Beacon with FreeRTOS Application (version: %s)\r\n", BLE_BEACON_VERSION_STRING); 

  createTasks();

}


static void BLETask( void *pvParameters )
{
  /* To make sure no other BLE functions are called from other tasks. */
  xSemaphoreTake(BLETickSemaphoreHandle, portMAX_DELAY);
  
  /* Init the BlueNRG-LP device */
  Device_Init();
  
  /* Start Beacon Non Connectable Mode*/
  Start_Beaconing();
  
  /* BLE is initialized. Let other tasks call BLE functions. */
  xSemaphoreGive(BLETickSemaphoreHandle);
  
  while(1)
  {
    /* Take the semaphore to avoid that other ACI functions can interrupt the
       execution of ModulesTick().   */
    xSemaphoreTake(BLETickSemaphoreHandle, portMAX_DELAY);
    ModulesTick();    
    xSemaphoreGive(BLETickSemaphoreHandle);
    /* Preliminary check if there is something to process at the moment. If
      something else to process will arrive later, before chip goes to sleep,
      this will be handled inside HAL_PWR_MNGR_Request(). */
    if(BLE_STACK_SleepCheck() != POWER_SAVE_LEVEL_RUNNING && HAL_VTIMER_SleepCheck() == TRUE)
    {
      xSemaphoreTake(radioActivitySemaphoreHandle, portMAX_DELAY);
    }
  }
}

/*-----------------------------------------------------------*/
/* Just a test task which makes a very short pulse on a GPIO. */
static void testTask( void *pvParameters )
{
  TickType_t xNextWakeTime;
  
  /* Initialise xNextWakeTime - this only needs to be done once. */
  xNextWakeTime = xTaskGetTickCount();
  
  for( ;; )
  {
    /* Place this task in the blocked state until it is time to run again.
    The block time is specified in ticks, the constant used converts ticks
    to ms.  While in the Blocked state this task will not consume any CPU
    time. */
    vTaskDelayUntil( &xNextWakeTime, TEST_PERIOD );
    
    /* Only do a pulse. */
    BSP_LED_On(BSP_LED3);
    __NOP();__NOP();__NOP();__NOP();
    BSP_LED_Off(BSP_LED3);
    
    PRINTF("Test Task\r\n");
  }  
}



/***************************************************************************************/

/*-----------------------------------------------------------*/
/* Another task that changes the advertising data */
static void changeADVDataTask( void *pvParameters )
{
  TickType_t xNextWakeTime;
  uint8_t Random_Number[8];
  
  /* Initialise xNextWakeTime - this only needs to be done once. */
  xNextWakeTime = xTaskGetTickCount();
  
  for( ;; )
  {
    /* Place this task in the blocked state until it is time to run again.
    The block time is specified in ticks, the constant used converts ticks
    to ms.  While in the Blocked state this task will not consume any CPU
    time. */
    vTaskDelayUntil( &xNextWakeTime, ADV_CHANGE_PERIOD );
    
    BLE_ACI_PROTECTED(hci_le_rand(Random_Number));
    
    adv_data[25] = Random_Number[0];
    
    /* In this case there is no need to disable advertising before updating buffer
       content, since only one byte is changed (atomic operation) and there is no
       risk to have inconsistent data. */
    
    BLE_ACI_PROTECTED(aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, SHORT_ADV_DATA_LENGTH, adv_data));
    
    
#if EXTENDED_ADV
    BLE_ACI_PROTECTED(aci_gap_set_advertising_data(1, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data));
#endif
    
    PRINTF("ADV change %d\r\n", adv_data[25]);
    
  }  
}
/*-----------------------------------------------------------*/

void createTasks( void )
{
  
  xTaskCreate(BLETask,"BLEStack", 650, NULL, BLE_TASK_PRIORITY, NULL);
  
  xTaskCreate( testTask, "Test", 80, NULL, TEST_TASK_PRIORITY, NULL );
  
  xTaskCreate( changeADVDataTask, "ADV", 150, NULL, TEST_TASK_PRIORITY, NULL );
  
  /* Start the tasks and timer running. */
  vTaskStartScheduler();
  
  /* If all is well, the scheduler will now be running, and the following
  line will never be reached.  If the following line does execute, then
  there was insufficient FreeRTOS heap memory available for the idle and/or
  timer tasks	to be created.  See the memory management section on the
  FreeRTOS web site for more details. */
  for( ;; );
}
/*-----------------------------------------------------------*/


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

/***************************************************************************************/


void vApplicationMallocFailedHook( void )
{
  /* vApplicationMallocFailedHook() will only be called if
  configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
  function that will get called if a call to pvPortMalloc() fails.
  pvPortMalloc() is called internally by the kernel whenever a task, queue,
  timer or semaphore is created.  It is also called by various parts of the
  demo application.  If heap_1.c or heap_2.c are used, then the size of the
  heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
  FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
  to query the size of free heap space that remains (although it does not
  provide information on how the remaining heap might be fragmented). */
  taskDISABLE_INTERRUPTS();
  for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
  /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
  to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
  task.  It is essential that code added to this hook function never attempts
  to block in any way (for example, call xQueueReceive() with a block time
  specified, or call vTaskDelay()).  If the application makes use of the
  vTaskDelete() API function (as this demo application does) then it is also
  important that vApplicationIdleHook() is permitted to return to its calling
  function, because it is the responsibility of the idle task to clean up
  memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;
  
  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
  function is called if a stack overflow is detected. */
  taskDISABLE_INTERRUPTS();
  for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
  /* This function will be called by each tick interrupt if
  configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
  added here, but the tick hook is called from an interrupt context, so
  code must not attempt to block, and only the interrupt safe FreeRTOS API
  functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/


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
  ex: PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
