
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RADIO_BeepMultiState_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 25-March-2019
* Description        : Transmission only example with multi state functionality. A certain number of state machine are configured to transmit in different configurations and are executed consecutively.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RADIO_BeepMultiState/RADIO_BeepMultiState_main.c
 * @brief Transmission only example with multi state functionality. A certain number of state machine are configured to transmit in different configurations and are executed consecutively.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt> C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_BeepMultiState\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_BeepMultiState.uvprojx </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt> C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\BeepMultiState\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_BeepMultiState.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\BeepMultiState\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - Release configuration


* \section Board_supported Boards supported
- \c STEVAL-IDB011V1
- \c STEVAL-IDB011V2
- \c STEVAL-IDB012V1


* \section Power_settings Power configuration settings
@table

==========================================================================================================
|                                         STEVAL-IDB01xV1                                                |
----------------------------------------------------------------------------------------------------------
| Jumper name | Description                                                                |
| JP2         |                                                                            |
----------------------------------------------------------------------------------------------------------
| USB         | USB supply power                                                            |
| BAT         | The supply voltage must be provided through battery pins.                   |


@endtable

* \section Jumper_settings Jumper settings
@table

========================================================================================================================================================================================
|                                                                             STEVAL-IDB01xV1                                                                                          |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Jumper name |                                                                Description                                                                                             |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
| JP1         | It provides the voltage to the BlueNRG-LP circuit. It must be fitted. It can be used for current measurements of the BlueNRG-LP device.                                |          
| JP2         | It is a switch between two power domains. BAT position: to provide power from battery holder; USB position: to provide power from USB connector.                       |
| JP3         | It connects the BLE_SWCLK pin of the BlueNRG-LP with the SWCLK pin of the USB_CMSISDAP. It must be fitted.                                                             |          
| JP4         | It connects the BLE_SWDIO pin of the BlueNRG-LP with the SWDIO pin of the USB_CMSISDAP. It must be fitted.                                                             |
| JP5         | It connects the BLE_RSTN pin of the BlueNRG-LP with the rest of the board (the USB_CMSISDAP and RESET push button). It must be fitted.                                 |


@endtable 

* \section Pin_settings Pin settings
@table
|  PIN name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------
|     A0     |        N.A.        |        N.A.        |      Not Used      |
|     A1     |      Not Used      |      Not Used      |      Not Used      |
|     A10    |        N.A.        |        N.A.        |      Not Used      |
|     A11    |      Not Used      |      Not Used      |      Not Used      |
|     A12    |      Not Used      |      Not Used      |        N.A.        |
|     A13    |      Not Used      |      Not Used      |        N.A.        |
|     A14    |      Not Used      |      Not Used      |        N.A.        |
|     A15    |      Not Used      |      Not Used      |        N.A.        |
|     A3     |        N.A.        |        N.A.        |      Not Used      |
|     A4     |      Not Used      |      Not Used      |        N.A.        |
|     A5     |      Not Used      |      Not Used      |        N.A.        |
|     A6     |      Not Used      |      Not Used      |        N.A.        |
|     A7     |      Not Used      |      Not Used      |        N.A.        |
|     A8     |      Not Used      |      Not Used      |      Not Used      |
|     A9     |      Not Used      |      Not Used      |        N.A.        |
|     B0     |      Not Used      |      Not Used      |      Not Used      |
|     B1     |        N.A.        |        N.A.        |      Not Used      |
|     B12    |        N.A.        |        N.A.        |      Not Used      |
|     B13    |        N.A.        |        N.A.        |      Not Used      |
|     B14    |      Not Used      |      Not Used      |      Not Used      |
|     B15    |        N.A.        |        N.A.        |      Not Used      |
|     B2     |      Not Used      |      Not Used      |      Not Used      |
|     B3     |      Not Used      |      Not Used      |      Not Used      |
|     B4     |      Not Used      |      Not Used      |      Not Used      |
|     B5     |      Not Used      |      Not Used      |      Not Used      |
|     B6     |        N.A.        |        N.A.        |      Not Used      |
|     B7     |      Not Used      |      Not Used      |      Not Used      |
|     B8     |      Not Used      |      Not Used      |        N.A.        |
|     B9     |      Not Used      |      Not Used      |        N.A.        |
|     GND    |      Not Used      |      Not Used      |      Not Used      |
|     RST    |      Not Used      |      Not Used      |      Not Used      |
|    VBAT    |      Not Used      |      Not Used      |      Not Used      |

@endtable 

* \section LEDs_description LEDs description
@table
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |
|     U5     |      Not Used      |      Not Used      |      Not Used      |

@endtable

* \section Buttons_description Buttons description
@table
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |
--------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |       Not Used       |
|      PUSH2     |      Not Used      |      Not Used      |       Not Used       |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage
Transmission only example as for the Beep, but with multi state functionality. A certain number of state machines are configured to transmit in different configurations and are executed consecutively. It is possible to enable/disable the encryption by using the array encryption_values[], 1 to enable the encription, while 0 to disable the encryption. Using the SnifferMultiState example is possible to receive the communication coming from the BeepMultiState by using the default configuration for both the examples.


**/
   

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdint.h>
#include "rf_driver_hal_vtimer.h"
#include "rf_driver_hal_radio_2g4.h"

#include "bluenrg_lp_evb_config.h"

/** @addtogroup BlueNRG1_StdPeriph_Examples
  * @{
  */

/** @addtogroup RADIO_Examples RADIO Examples
  * @{
  */

/** @addtogroup RADIO_Beep_MultiState RADIO Beep MultiState Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint8_t activity_flag;
  uint32_t wakeup_time[STATEMACHINE_COUNT];
}multi_state_t;

/* Private define ------------------------------------------------------------*/
#define HS_STARTUP_TIME         (uint16_t)(1)  /* High Speed start up time min value */
#define DATA_LEN                (uint8_t)(26)
#define N_STATE_MACHINES        STATEMACHINE_COUNT /* The number of state machines */
#define CALIBRATION_INTERVAL_CONF   10000

#if defined CONFIG_HW_LS_RO  

/* Calibration must be done */
#define INITIAL_CALIBRATION TRUE
#define CALIBRATION_INTERVAL        CALIBRATION_INTERVAL_CONF

#elif defined CONFIG_HW_LS_XTAL

/* No Calibration */
#define INITIAL_CALIBRATION FALSE
#define CALIBRATION_INTERVAL        0

#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
multi_state_t multi_state = {{0,}, {0,}};

uint8_t channel_map[5] = {0xFF,0xFF,0xFF,0xFF,0x0F};
uint8_t channel_values[STATEMACHINE_COUNT] = {21,22,23,25,26,27,28,29};
uint32_t period_usec_values[STATEMACHINE_COUNT] = {100000,100000,100000,100000,100000,100000,100000,100000};
uint32_t network_id_values[STATEMACHINE_COUNT] = {0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6};
uint8_t encryption_values[STATEMACHINE_COUNT] = {0,0,0,1,1,1,1,0};

uint8_t channel[N_STATE_MACHINES];
uint32_t period_usec[N_STATE_MACHINES];
uint32_t network_id[N_STATE_MACHINES];
uint8_t encryption[N_STATE_MACHINES];

uint8_t sendData[N_STATE_MACHINES][DATA_LEN+3];
ActionPacket actPacket[N_STATE_MACHINES];

uint8_t count_tx[5] = {0x00,0x00,0x00,0x00,0x00};
uint8_t count_rcv[5] = {0x00,0x00,0x00,0x00,0x00};
uint8_t enc_key[16] = {0xBF,0x01,0xFB,0x9D,0x4E,0xF3,0xBC,0x36,0xD8,0x74,0xF5,0x39,0x41,0x38,0x68,0x4C};
uint8_t enc_iv[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  multi_state_schedular routine.
  * @param  multi_state
  * @retval void
  */
void multi_state_schedular(multi_state_t * multi_state)
{
  uint32_t next_wakeup_time_min = multi_state->wakeup_time[0];
  uint8_t i = 0;
  uint8_t index = 0;
  
  /* Find the minimum wakeup time among the active state machine */
  for(i = 1;i<STATEMACHINE_COUNT;i++) {
    if((next_wakeup_time_min > multi_state->wakeup_time[i]) && ((multi_state->activity_flag >> i)&0x01)) {
      next_wakeup_time_min = multi_state->wakeup_time[i];
      index  = i;
    }
  }
  RADIO_SetChannel(actPacket[index].StateMachineNo, channel[index], 0);
  RADIO_MakeActionPacketPending(&actPacket[index]);
  multi_state->wakeup_time[index] +=(period_usec[index] >> 10);
}

/**
  * @brief  Condition routine.
  * @param  ActionPacket
  * @retval TRUE
  */
uint8_t conditionRoutine(ActionPacket* p)
{
  return TRUE;   
} 

/**
  * @brief  Data routine.
  * @param  ActionPacket: current
  * @param  ActionPacket: next
  * @retval TRUE
  */
uint8_t dataRoutine(ActionPacket* p,  ActionPacket* next)
{
  if(next == NULL) {
    BSP_LED_Toggle(BSP_LED1);
    multi_state_schedular(&multi_state);
  }
  return TRUE;
}


/**
  * @brief  Beep Initilize.
  * @param  State Machine No
  * @retval void
  */
void beep_init(uint8_t StateMachineNo)
{
  sendData[StateMachineNo][0] = 0x02; 
  sendData[StateMachineNo][1] = DATA_LEN;       /* Length position is fixed */
  sendData[StateMachineNo][2] = StateMachineNo;
  
  for(uint8_t i = 1; i < DATA_LEN; i++) {
    sendData[StateMachineNo][2+i] = i;
  }
  
  if(encryption[StateMachineNo]) {
    sendData[StateMachineNo][1] += MIC_FIELD_LENGTH;
  }
  
  actPacket[StateMachineNo].StateMachineNo = StateMachineNo;
  actPacket[StateMachineNo].ActionTag = RELATIVE | TIMER_WAKEUP | TXRX | PLL_TRIG ;
  actPacket[StateMachineNo].WakeupTime = period_usec[StateMachineNo];
  actPacket[StateMachineNo].MaxReceiveLength = 0;/* not apllied for Tx*/
  actPacket[StateMachineNo].data = (uint8_t *)&sendData[StateMachineNo][0]; 
  actPacket[StateMachineNo].next_true = NULL;   /* -> points to AP[0]  */
  actPacket[StateMachineNo].next_false = NULL;  /* -> points to AP[0]  */   
  actPacket[StateMachineNo].condRoutine = conditionRoutine;
  actPacket[StateMachineNo].dataRoutine = dataRoutine;  
  multi_state.activity_flag = multi_state.activity_flag | 1<<StateMachineNo;
  multi_state.wakeup_time[StateMachineNo] = actPacket[StateMachineNo].WakeupTime;
  RADIO_SetChannelMap(StateMachineNo, &channel_map[0]);
  RADIO_SetChannel(StateMachineNo, channel[StateMachineNo],0);
  RADIO_SetTxAttributes(StateMachineNo, network_id[StateMachineNo] , 0x555555);
  RADIO_SetTxPower(MAX_OUTPUT_RF_POWER);
  
  if(encryption[StateMachineNo]) {
    RADIO_SetEncryptionCount(StateMachineNo, &count_tx[0], &count_rcv[0]); 
    RADIO_SetEncryptionAttributes(StateMachineNo, enc_iv, enc_key);
    RADIO_SetEncryptFlags(StateMachineNo,(FunctionalState)encryption[StateMachineNo],(FunctionalState)encryption[StateMachineNo]);    
  }
  
  RADIO_SetReservedArea(&actPacket[StateMachineNo]); 
}  

/**
  * @brief  Main program code.
  * @param  None
  * @retval None
  */
int main(void)
{
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }


  BSP_LED_Init(BSP_LED1);
  
  RADIO_Init();
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
  /* Set the parameters and configure the state machiness */
  for(uint8_t i = 0; i < N_STATE_MACHINES; i++) {
    network_id[i] = network_id_values[i];
    channel[i] = channel_values[i];
    encryption[i] = encryption_values[i];
    period_usec[i] = period_usec_values[i];
    
    beep_init(i);
  }
  
  /* Call this function for the first action packet to be executed */
  RADIO_MakeActionPacketPending(&actPacket[0]);
 
  /* Infinite loop */
  while(1) {
    HAL_VTIMER_Tick();
  }   
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
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

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
