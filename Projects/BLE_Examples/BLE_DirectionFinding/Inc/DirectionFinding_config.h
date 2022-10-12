/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : DirectionFinding.h
* Author             : AMS - RF Application Team
* Date               : 27-May-2021
* Description        : Header file for Bluetooth stack configuration.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef _DIRECTIONFINDING_CONFIG_H_
#define _DIRECTIONFINDING_CONFIG_H_

#include "bluenrg_lp_stack.h"
#include "stack_user_cfg.h"

/* This file contains all the information needed to init the BlueNRG stack.
 * These constants and variables are used from the BlueNRG stack to reserve RAM and FLASH 
 * according the application requests
 */

/* Number of radio state machines */
#if CTE_TAG
#define MIN_NUM_LINK_CONF                2   /* 2 slots used for extended advertising (used in this demo, even if not strictly needed). */
#else /* CTE_LOCATOR */
#define MIN_NUM_LINK_CONF                1 + NUM_AUX_SCAN_SLOTS_CONF /* For extended scan (used in this demo). */
#endif

/* Number of services */
#define NUM_APP_GATT_SERVICES_CONF  (0)

/* Number of characteristics attributes */
#define NUM_APP_GATT_CHAR_ATTRIBUTES_CONF (0)

/* Set supported max value for ATT_MTU enabled by the application */
#define MAX_ATT_MTU_CONF    (BLE_STACK_DEFAULT_ATT_MTU)             /* DEFAULT_ATT_MTU size = 23 bytes */  

/* Number of links needed for the demo: 1
 * Only 1 the default
 */
#define NUM_LINKS                                    (MIN_NUM_LINK_CONF)
#define NUM_ADV_SETS_CONF                            (1U) 
#define NUM_AUX_SCAN_SLOTS_CONF                      (2U)
#define NUM_SYNC_SLOTS_CONF                          (0U)
#define WHITE_LIST_SIZE_LOG2_CONF                    (0U)
#define L2CAP_MPS_CONF                               (247U)
#define NUM_L2CAP_COCS_CONF                          (0U)
#define MAX_NUM_CTE_ANTENNA_IDS                      (8U)
#define MAX_NUM_CTE_IQ_SAMPLES                       (82U)

#define ISR0_FIFO_SIZE   (256)
#define ISR1_FIFO_SIZE   (768)
#define USER_FIFO_SIZE   (1024)

/* Number of GATT attributes needed for the demo. */
#define NUM_GATT_ATTRIBUTES     (NUM_APP_GATT_SERVICES_CONF + NUM_APP_GATT_CHAR_ATTRIBUTES_CONF)

/* Additional number of memory blocks  to be added to the minimum */
#define OPT_MBLOCKS_CONF		(0) 

/* Set the number of memory block for packet allocation */
#define MBLOCKS_COUNT           (BLE_STACK_MBLOCKS_CALC(MAX_ATT_MTU_CONF, NUM_LINKS) + OPT_MBLOCKS_CONF)

#define NUM_OF_CONCURRENT_GATT_CLIENT_PROC_CONF              (1)

/* RAM reserved to manage all the data stack according the number of links,
 * number of services, number of attributes and attribute value length
 */
#define DYNAMIC_MEMORY_SIZE (BLE_STACK_TOTAL_BUFFER_SIZE(NUM_LINKS,NUM_GATT_ATTRIBUTES,NUM_OF_CONCURRENT_GATT_CLIENT_PROC_CONF,MBLOCKS_COUNT,\
                                                         NUM_ADV_SETS_CONF,NUM_AUX_SCAN_SLOTS_CONF,WHITE_LIST_SIZE_LOG2_CONF,NUM_L2CAP_COCS_CONF,\
                                                         NUM_SYNC_SLOTS_CONF, MAX_NUM_CTE_ANTENNA_IDS, MAX_NUM_CTE_IQ_SAMPLES, \
                                                         ISR0_FIFO_SIZE, ISR1_FIFO_SIZE, USER_FIFO_SIZE)) 


/* Maximum duration of the connection event */
#define MAX_CONN_EVENT_LENGTH_CONF 0xFFFFFFFF

#define CALIBRATION_INTERVAL_CONF   10000

#if defined CONFIG_HW_LS_RO  

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        500

/* Calibration must be done */
#define INITIAL_CALIBRATION         TRUE
#define CALIBRATION_INTERVAL        CALIBRATION_INTERVAL_CONF

#elif defined CONFIG_HW_LS_XTAL

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        100

/* No Calibration */
#define INITIAL_CALIBRATION         FALSE
#define CALIBRATION_INTERVAL        0

#endif

/* High Speed start up time */
#define HS_STARTUP_TIME 320 // 780 us


/* Radio Config Hot Table */
extern uint8_t hot_table_radio_config[];

/* This structure contains memory and low level hardware configuration data for the device */
#define BLE_STACK_INIT_PARAMETERS {                                             \
    .BLEStartRamAddress = (uint8_t*)dyn_alloc_a,                                \
    .TotalBufferSize = DYNAMIC_MEMORY_SIZE,                                     \
    .NumAttrRecords = NUM_GATT_ATTRIBUTES,                                      \
    .MaxNumOfClientProcs = NUM_OF_CONCURRENT_GATT_CLIENT_PROC_CONF,             \
    .NumOfLinks = NUM_LINKS,                                                    \
    .NumBlockCount = MBLOCKS_COUNT,                                             \
    .ATT_MTU = MAX_ATT_MTU_CONF,                                                \
    .MaxConnEventLength = MAX_CONN_EVENT_LENGTH_CONF,                           \
    .SleepClockAccuracy = SLEEP_CLOCK_ACCURACY,                                 \
    .NumOfAdvDataSet = NUM_ADV_SETS_CONF,                                       \
    .NumOfAuxScanSlots = NUM_AUX_SCAN_SLOTS_CONF,                               \
    .WhiteListSizeLog2 = WHITE_LIST_SIZE_LOG2_CONF,                             \
    .L2CAP_MPS = L2CAP_MPS_CONF,                                                \
    .L2CAP_NumChannels = NUM_L2CAP_COCS_CONF,                                   \
    .NumOfSyncSlots = NUM_SYNC_SLOTS_CONF,                                      \
    .CTE_MaxNumAntennaIDs = MAX_NUM_CTE_ANTENNA_IDS,                            \
    .CTE_MaxNumIQSamples = MAX_NUM_CTE_IQ_SAMPLES, 		         	\
    .isr0_fifo_size = ISR0_FIFO_SIZE,                                           \
    .isr1_fifo_size = ISR1_FIFO_SIZE,                                           \
    .user_fifo_size = USER_FIFO_SIZE                                            \
}
#endif // _DIRECTIONFINDING_CONFIG_H_
