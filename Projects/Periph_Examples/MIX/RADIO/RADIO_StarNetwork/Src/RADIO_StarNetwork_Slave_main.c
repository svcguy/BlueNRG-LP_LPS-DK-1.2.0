
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RADIO_StarNetwork_Slave_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 25-March-2019
* Description        : The code defines a Slave device in a star network. The Slave waits for a request of data from the Master. Once the request is received, then the Slave sends the data packet and waits for an ACK packet from the Master. The confirm the data reception of the Master. If the ACK packet is not received, then the Slave sends again the data packet to the Master. The Slave can send up to three packets of data to the Master if the ACK packet is not received.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RADIO_StarNetwork/RADIO_StarNetwork_Slave_main.c
 * @brief Code demonstrating a Slave device in a star network.
 * Two devices are necessary to run fully this demo.
 *


* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIXRADIO\\RADIO_StarNetwork\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_StarNetwork.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
      <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_StarNetwork\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_StarNetwork.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RxStarNetwork\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Slave_1 - Slave configuration
- \c Slave_2 - Slave configuration


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
|            |                            Slave_1                            |||                            Slave_2                            |||
--------------------------------------------------------------------------------------------------------------------------------------------------
|  PIN name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------------------------------------------------------------------------
|     A0     |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     A1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     A10    |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     A11    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     A12    |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A13    |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A14    |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A15    |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A3     |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     A4     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A5     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A6     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A7     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A8     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     A9     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     B0     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B1     |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B12    |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B13    |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B14    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B15    |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B3     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B4     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B5     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B6     |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B7     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B8     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     B9     |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     GND    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     RST    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    VBAT    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |

@endtable 

* \section LEDs_description LEDs description
@table
|            |                            Slave_1                            |||                            Slave_2                            |||
--------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     U5     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |

@endtable

* \section Buttons_description Buttons description
@table
|                |                             Slave_1                             |||                             Slave_2                             |||
----------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |
----------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |      Not Used      |       Not Used       |
|      PUSH2     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |      Not Used      |       Not Used       |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage
The code defines a Slave device in a star network. The Slave waits for a request of data from the Master. Once the request is received, then the Slave sends the data packet and waits for an ACK packet from the Master. If the ACK packet is not received, then the Slave sends again the data packet to the Master. The Slave can send up to three packets of data to the Master if the ACK packet is not received. The project allows to define new Slave in the network, just changing the BLE_NETWORK_ID value with a new Slave address.


**/
   

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rf_driver_hal_vtimer.h"
#include "rf_driver_hal_radio_2g4.h"
#include "rf_driver_ll_radio_2g4.h"

#include "bluenrg_lp_evb_config.h"
    
#include "main_common.h"


/** @addtogroup BlueNRG1_StdPeriph_Examples
  * @{
  */

/** @addtogroup RADIO_Examples RADIO Examples
  * @{
  */

/** @addtogroup RADIO_StarNetwork RADIO StarNetwork Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_NETWORK_ID          SLAVE_ID
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
uint8_t txDataBuffer[62];
uint8_t ackBuffer[2];
uint8_t receiveBuffer[MAX_PACKET_LENGTH];

ActionPacket waitRequestAction;
ActionPacket sendDataAction;
ActionPacket waitAckAction;

uint16_t pkt_counter_test = 0;
uint16_t retry_counter_test = 0;
uint8_t retry = 0;
uint16_t ack_counter_test = 0;


/* Private function prototypes -----------------------------------------------*/
void InitializationActionPackets(void);
uint8_t waitRequestCB(ActionPacket* p);
uint8_t sendDataCB(ActionPacket* p);
uint8_t waitAckCB(ActionPacket* p);
uint8_t dataRoutine(ActionPacket* p,  ActionPacket* next);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Condition routine.
  * @param  ActionPacket
  * @retval TRUE
  */
uint8_t waitRequestCB(ActionPacket* p)
{
  /* received a packet */
  if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
    return TRUE;
  }
  else if(((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) || ((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0)) {
    return FALSE;
  }
  return FALSE;
} 

uint8_t sendDataCB(ActionPacket* p)
{
  return TRUE;
} 

uint8_t waitAckCB(ActionPacket* p)
{
  /* received a packet */

  if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
    return TRUE;
  }
  else if(((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) || ((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0)) {
    if(retry < MAX_RETRY_TEST) {
      retry++;
      return FALSE;
    }
    else {
      retry = 0;
      return TRUE;
    }
  }
  
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
  if( (p->status & BLUE_INTERRUPT1REG_DONE) != 0) {
    
    /* received a packet */
    if((p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0){
      
      if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {    
      }
      else if((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) {
      }
      else if((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0) {
      }
    }
  }
  return TRUE;  
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


    
  BSP_COM_Init(NULL);
  BSP_LED_Init(BSP_LED1);
  
  RADIO_Init();
  HAL_VTIMER_Init(&VTIMER_InitStruct);

  /* Configures the transmit power level */
  RADIO_SetTxPower(MAX_OUTPUT_RF_POWER);
  
  /* Build data packet */
  txDataBuffer[0] = 0x0;  /* First byte */
  txDataBuffer[1] = 60;   /* Length position is fixed */
  for(uint8_t i=0; i != 60; i++) {
    txDataBuffer[i+2] = i;
  }
  
  /* Build ACK packet */
  ackBuffer[0] = 0xAE;
  ackBuffer[1] = 0;

  /* Initialize the routine for sending data and wait for the ACK */
  InitializationActionPackets();
  
  /* Call this function for the first action packet to be executed */
  RADIO_MakeActionPacketPending(&waitRequestAction);
  
  /* Infinite loop */
  while(1) {
    HAL_VTIMER_Tick();
  }   
}



void InitializationActionPackets(void)
{
  /* Build Action Packets */
  waitRequestAction.StateMachineNo = STATE_MACHINE_0;
  waitRequestAction.ActionTag = RELATIVE | TIMER_WAKEUP | PLL_TRIG;
  waitRequestAction.WakeupTime = 30+WKP_OFFSET;           /* 30 us minimum of internal delay before start action */
  waitRequestAction.MaxReceiveLength = MAX_LL_PACKET_LENGTH;
  waitRequestAction.data = receiveBuffer;                 /* Data to send */
  waitRequestAction.next_true = &sendDataAction;           /* If condRoutine returns TRUE => Go to next TX the 2nd part of the payload */
  waitRequestAction.next_false = &waitRequestAction;       /* Not Used */   
  waitRequestAction.condRoutine = waitRequestCB;          /* Condition routine */
  waitRequestAction.dataRoutine = dataRoutine;            /* Data routine */

  sendDataAction.StateMachineNo = STATE_MACHINE_0;
  sendDataAction.ActionTag = RELATIVE | TIMER_WAKEUP | TXRX;
  sendDataAction.WakeupTime = 40+WKP_OFFSET;              /* 40 us minimum of internal delay before start action */
  sendDataAction.MaxReceiveLength = 0;                    
  sendDataAction.data = txDataBuffer;                     /* Buffer for ACK packet reception */
  sendDataAction.next_true = &waitAckAction;               /* Reschedule the RX */   
  sendDataAction.next_false = &waitAckAction;              /* Reschedule the RX */   
  sendDataAction.condRoutine = sendDataCB;                /* Condition routine */
  sendDataAction.dataRoutine = dataRoutine;               /* Data routine */
  
  waitAckAction.StateMachineNo = STATE_MACHINE_0;
  waitAckAction.ActionTag = RELATIVE | TIMER_WAKEUP;
  waitAckAction.WakeupTime = 30+WKP_OFFSET;              /* 30 us minimum of internal delay before start action */
  waitAckAction.MaxReceiveLength = MAX_LL_PACKET_LENGTH;  
  waitAckAction.data = receiveBuffer;                    /* Buffer for ACK packet reception */
  waitAckAction.next_true = &waitRequestAction;           /* Reschedule the RX */   
  waitAckAction.next_false = &sendDataAction;             /* Reschedule the RX */   
  waitAckAction.condRoutine = waitAckCB;                 /* Condition routine */
  waitAckAction.dataRoutine = dataRoutine;               /* Data routine */
  
  RADIO_SetGlobalReceiveTimeout(RX_TIMEOUT_DATA);
  
  /* Channel map configuration */
  uint8_t map[5]= {0xFF,0xFF,0xFF,0xFF,0xFF};
  RADIO_SetChannelMap(STATE_MACHINE_0, &map[0]);
  
  /* Set the channel */
  RADIO_SetChannel(STATE_MACHINE_0, APP_CHANNEL, 0);

  /* Sets of the NetworkID and the CRC */
  RADIO_SetTxAttributes(STATE_MACHINE_0, BLE_NETWORK_ID, 0x555555);
    
  /* Call these functions before execute the action packets */
  RADIO_SetReservedArea(&waitRequestAction);
  RADIO_SetReservedArea(&sendDataAction);
  RADIO_SetReservedArea(&waitAckAction);
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
