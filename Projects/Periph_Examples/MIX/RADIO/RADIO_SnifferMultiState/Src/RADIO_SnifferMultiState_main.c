
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RADIO_SnifferMultiState_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 25-March-2019
* Description        : Sniffer application with multi state functionality.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RADIO_SnifferMultiState/RADIO_SnifferMultiState_main.c
 * @brief Sniffer application with multi state functionality.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_SnifferMultiState\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_SnifferMultiState.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_SnifferMultiState\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_SnifferMultiState.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\SnifferMultiState\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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

* \section Serial_IO Serial I/O
  The application will listen for keys typed in one node and, on return press, it will send them to the remote node.
  The remote node will listen for RF messages and it will output them in the serial port.
  In other words everything typed in one node will be visible to the other node and vice versa.
@table
| Parameter name  | Value               | Unit      |
------------------------------------------------------
| Baudrate        | 115200 [default]    | bit/sec   |
| Data bits       | 8                   | bit       |
| Parity          | None                | bit       |
| Stop bits       | 1                   | bit       |

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
This demo shows a Sniffer application with multi state functionality that allows to configure a certain number of state machine in order to sniff in different configuration as for example sniff in different channels (not at the same time).
The application prints out on a serial terminal the packet received on a specific channel and with a specific NetworkID.
Once the packet is received, it is printed the RSSI associated to the packet, the timestamp and the entire frame.


**/
   

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "rf_driver_hal_vtimer.h"
#include "rf_driver_ll_radio_2g4.h"
#include "fifo.h"

#include "bluenrg_lp_evb_config.h"

/** @addtogroup BlueNRG1_StdPeriph_Examples
* @{
*/

/** @addtogroup RADIO_Examples RADIO Examples
* @{
*/

/** @addtogroup RADIO_Sniffer_MultiState RADIO Sniffer MultiState Example
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define HS_STARTUP_TIME         (uint16_t)(1)  /* start up time min value */
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

#define PRINT_INT(x)    ((int)(x))
#define PRINT_FLOAT(x)  (x>0)? ((int) (((x) - PRINT_INT(x)) * 1000)) : (-1*(((int) (((x) - PRINT_INT(x)) * 1000))))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Blue FIFO */
circular_fifo_t blueRec_fifo;
uint8_t blueRec_buffer[MAX_PACKET_LENGTH*2+MAX_PACKET_LENGTH];

uint8_t receivedData[MAX_PACKET_LENGTH];

uint32_t delay = 1000;
uint32_t rx_timeout = 500000;
int32_t rssi_val = 0;
uint32_t timestamp = 0;
uint32_t actual_ch = 0;
uint8_t channel_map[5] = {0xFF,0xFF,0xFF,0xFF,0xFF};

uint32_t network_id_values[STATEMACHINE_COUNT] = {0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6};
uint8_t channel_values[STATEMACHINE_COUNT] = {21,22,23,24,25,26,27,28};
uint8_t encryption_values[STATEMACHINE_COUNT] = {0,0,0,1,1,1,1,0};

uint32_t network_id[N_STATE_MACHINES];
uint8_t channel[N_STATE_MACHINES];
uint8_t encryption[N_STATE_MACHINES];
uint8_t No_Packet_Per_StateMachineNo = 5;

static ActionPacket aPacket[N_STATE_MACHINES]; 

uint8_t count_tx[5]     = {0x00,0x00,0x00,0x00,0x00};
uint8_t count_rcv[5]    = {0x00,0x00,0x00,0x00,0x00};
uint8_t enc_key[16]     = {0xBF,0x01,0xFB,0x9D,0x4E,0xF3,0xBC,0x36,0xD8,0x74,0xF5,0x39,0x41,0x38,0x68,0x4C};
uint8_t enc_iv[8]       = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint8_t nullfunction(ActionPacket* p)
{
  return TRUE;
}


uint8_t RxCallback(ActionPacket* p, ActionPacket* next)
{
  static uint8_t count_t = 0;
  static uint8_t StateMachine_index = 0;

  /* received a packet */
  if((p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0){

  /* Number of packet capture per State Machine */
  if(count_t < No_Packet_Per_StateMachineNo) {
    count_t++;
    
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
      fifo_put_var_len_item(&blueRec_fifo, p->data[1]+2, p->data,0,NULL);
      fifo_put(&blueRec_fifo, 4, (uint8_t*)(&p->rssi) );
      fifo_put(&blueRec_fifo, 4, (uint8_t*)(&p->timestamp_receive));
      fifo_put(&blueRec_fifo, 1, (uint8_t*)(&channel[p->StateMachineNo]));
      RADIO_SetChannel(StateMachine_index,(uint8_t)channel[StateMachine_index], 0);
      RADIO_MakeActionPacketPending(&aPacket[StateMachine_index]);       
    }
    else if((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) {
//              printf("Timeout:%dus\r\n",rx_timeout);
      printf("%d",count_t);
      RADIO_SetChannel(StateMachine_index,(uint8_t)channel[StateMachine_index], 0);
      RADIO_MakeActionPacketPending(&aPacket[StateMachine_index]);
    }
    else if((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0) {
            printf("CRC error\r\n");
      printf("%d",count_t);
      RADIO_SetChannel(StateMachine_index,(uint8_t)channel[StateMachine_index], 0);
      RADIO_MakeActionPacketPending(&aPacket[StateMachine_index]);
    }              
  }
  else {
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
      fifo_put_var_len_item(&blueRec_fifo, p->data[1]+2, p->data,0,NULL);
      fifo_put(&blueRec_fifo, 4, (uint8_t*)(&p->rssi) );
      fifo_put(&blueRec_fifo, 4, (uint8_t*)(&p->timestamp_receive));
      fifo_put(&blueRec_fifo, 1, (uint8_t*)(&channel[p->StateMachineNo])); 
    }
    count_t = 0;
    StateMachine_index++;
    StateMachine_index = StateMachine_index%N_STATE_MACHINES; /* Update the State Machine */
    
    RADIO_SetChannel(StateMachine_index,(uint8_t)channel[StateMachine_index], 0);
    RADIO_MakeActionPacketPending(&aPacket[StateMachine_index]);
    //      printf("\r\n >> Sniffing channel..%d StateMachineNo:%d Encryption:%d Network_ID:0x%X\r\n",(uint8_t)channel[StateMachine_index],StateMachine_index,encryption[StateMachine_index],network_id[StateMachine_index]);
    printf("\r\n >> Sniffing channel..%d\r\n", (uint8_t)channel[StateMachine_index]);
  }
  }
  return TRUE;   
}  


int sniffer_init(uint8_t StateMachineNo)
{
  RADIO_SetChannelMap(StateMachineNo, channel_map);
  RADIO_SetChannel(StateMachineNo, (uint8_t)channel[StateMachineNo], 0);
  RADIO_SetTxAttributes(StateMachineNo, network_id[StateMachineNo], 0x555555);  
  
  aPacket[StateMachineNo].StateMachineNo = StateMachineNo;   
  aPacket[StateMachineNo].ActionTag = PLL_TRIG | RELATIVE | TIMER_WAKEUP;
  aPacket[StateMachineNo].WakeupTime = delay;
  aPacket[StateMachineNo].MaxReceiveLength = MAX_LL_PACKET_LENGTH; 
  aPacket[StateMachineNo].data = receivedData; 
  aPacket[StateMachineNo].next_true = NULL_0;
  aPacket[StateMachineNo].next_false = NULL_0;    
  aPacket[StateMachineNo].condRoutine = nullfunction;
  aPacket[StateMachineNo].dataRoutine = RxCallback;
  
  RADIO_SetEncryptionAttributes(StateMachineNo, enc_iv, enc_key);
  RADIO_SetEncryptionCount(StateMachineNo, &count_tx[0], &count_rcv[0]); 
  RADIO_SetEncryptFlags(StateMachineNo,(FunctionalState)encryption[StateMachineNo],(FunctionalState)encryption[StateMachineNo]);    
  RADIO_SetReservedArea(&aPacket[StateMachineNo]);
  
  printf(" >> Sniffing channel..%d StateMachineNo:%d Encryption:%d Network_ID:0x%X\r\n", (uint8_t)channel[StateMachineNo], StateMachineNo, encryption[StateMachineNo], network_id[StateMachineNo]);
  return 0;
}

int main(void)
{
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  uint8_t packet[MAX_PACKET_LENGTH];
  uint16_t length;
  
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }


  
  /* Set the parameters */
  for(uint8_t i = 0; i < N_STATE_MACHINES; i++) {
    network_id[i] = network_id_values[i];
    channel[i] = channel_values[i];
    encryption[i] = encryption_values[i];
  }

  /* Configure I/O communication channel */
  BSP_COM_Init(NULL);
  
  /* Create the blueRec FIFO */
  fifo_init(&blueRec_fifo, MAX_PACKET_LENGTH*2, blueRec_buffer, 1);
  
  /* Radio configuration - HS_STARTUP_TIME 642 us, external LS clock, NULL, whitening enabled */
  RADIO_Init();
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
  /* Initilize the sniffer State Machines */
  for(uint8_t i = 0; i<N_STATE_MACHINES; i++) {
    sniffer_init(i);
  }
  RADIO_SetGlobalReceiveTimeout(rx_timeout);
  /* Call this function for the first action packet to be executed */
  printf("\r\n >> Sniffing channel..%d\r\n", (uint8_t)channel[0]);
  RADIO_MakeActionPacketPending(&aPacket[0]);
  
  /* Infinite loop */
  while(1) {
    HAL_VTIMER_Tick();
    
    /* Print the packets sniffed */
    if(fifo_size(&blueRec_fifo) !=0) {
      
      /* Get the length of the packet and the packet itself */
      fifo_get_var_len_item(&blueRec_fifo, &length, packet);

      /* Get the RSSI information */
      fifo_get(&blueRec_fifo, 4, (uint8_t*)&rssi_val);
      
      /* Get the timestamp */
      fifo_get(&blueRec_fifo, 4, (uint8_t*)&timestamp);
      
      /* Get the channel */
      fifo_get(&blueRec_fifo, 1, (uint8_t*)&actual_ch);
      
      printf("\r\nchannel: %d, RSSI: %d dBm\r\n", actual_ch, rssi_val);
      //printf("Timestamp: %.3f ms\r\n", ((float)timestamp)/512.0);
      printf("Timestamp: %d.%03d ms\r\n", PRINT_INT(timestamp/512.0), PRINT_FLOAT(timestamp/512.0));
      
      printf("Frame: ");
      for(uint16_t i= 0; i<(length/*-MIC_FIELD_LENGTH*/); i++) {
        printf("%02x:", packet[i]);
      }
      printf("\r\n");
      //      printf("\r\nMIC: ");
      //      for(uint16_t i= (length+2-MIC_FIELD_LENGTH); i<(length+2); i++) {
      //        printf("%02x:", temp[i]);
      //      }
    }
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
