
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RADIO_SerialPort_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 25-March-2019
* Description        : This code generate a point to point communication generating a two ways chat.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RADIO_SerialPort/RADIO_SerialPort_main.c 
 * @brief This code implements a point to point two way communication.
 * Two devices are necessary to run fully this demo.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_SerialPort.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_SerialPort\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_SerialPort.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\SerialPort\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Encryption - Packet encryption configuration
- \c Release - Release configuration (no encryption)


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
|            |                 Release                 ||                          Encryption                           |||
---------------------------------------------------------------------------------------------------------------------------
|  PIN name  |   STEVAL-IDB011V1  |   STEVAL-IDB012V1  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
---------------------------------------------------------------------------------------------------------------------------
|     A0     |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     A1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     A10    |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     A11    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     A12    |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A13    |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A14    |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A15    |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A3     |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     A4     |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A5     |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A6     |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A7     |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     A8     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     A9     |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     B0     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B1     |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B12    |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B13    |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B14    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B15    |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B3     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B4     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B5     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B6     |        N.A.        |      Not Used      |        N.A.        |        N.A.        |      Not Used      |
|     B7     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     B8     |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     B9     |      Not Used      |        N.A.        |      Not Used      |      Not Used      |        N.A.        |
|     GND    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     RST    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    VBAT    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |

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
|            |                 Release                 ||                          Encryption                           |||
---------------------------------------------------------------------------------------------------------------------------
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB012V1  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
---------------------------------------------------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     U5     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |

@endtable

* \section Buttons_description Buttons description
@table
|                |                 Release                 ||                           Encryption                            |||
---------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB012V1  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |
---------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |       Not Used       |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |       Not Used       |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage
This code generate a point to point communication generating a two ways chat.
Program two devices with the same firmware, then open a serial terminal on the two COM ports and interact with the serial terminal by sending message. Be sure to end the message with the carriage return character (usually the enter button of the keyboard). Each device is always programmed in reception with a defined listening window (CHAT_RECEIVE_TIMEOUT). If the user write a message in a serial terminal and then send a "return carriage" character, a transmission is scheduled with that message as payload. Maximum payload 31 bytes. The communication requires ACK to confirm the packet reception. The ACK packet is a packet of 1 byte of payload defined as 0xAE and with a length field of 0 (see the user manual for much information). A message received is printed out on a serial terminal. Up to three retries are used by the transmitter if the acknowledge is lost.In this example the hal_radio layer is used in order to schedule the transmissions and the receptions. The callback RXCallback provided to the API HAL_RADIO_SendPacketWithAck define the behavior of the radio according to the IRQ flags. The same is for the TXCallback provided to the HAL_RADIO_ReceivePacketWithAck API. The project contains also a configuration where the encryption feature is enabled.

**/
   

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rf_driver_hal_vtimer.h"
#include "rf_driver_hal_radio_2g4.h"
#include "rf_driver_ll_radio_2g4.h"
#include "osal.h"
#include "fifo.h"

#include "bluenrg_lp_evb_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define BLE_ADV_ACCESS_ADDRESS  (uint32_t)(0x8E89BED6)
#define STARTING_CHANNEL        (uint8_t)(24)    // RF channel 22
#define HS_STARTUP_TIME         (uint16_t)(1)  /* High Speed start up time min value */

#define CHAT_RECEIVE_RELATIVETIME       5000
#define CHAT_TRANSMIT_RELATIVETIME      5000
#define CHAT_RECEIVE_TIMEOUT            100000  /* 100 ms */

#define MAX_CHAT_PACKET_LEN 30
#define MAX_RETRIES 3

#define CHAT_FREQUENCYCHANNEL           STARTING_CHANNEL

#ifdef PKT_ENCRYPTION
#define	MIC_FIELD_LEN	MIC_FIELD_LENGTH
#else
#define	MIC_FIELD_LEN	0
#endif

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

/* Blue FIFO */
circular_fifo_t blueRec_fifo;
uint8_t blueRec_buffer[MAX_PACKET_LENGTH*2+MAX_PACKET_LENGTH];

uint32_t interval;
uint8_t uart_buffer[MAX_PACKET_LENGTH];
uint8_t sendData[MAX_PACKET_LENGTH];
uint8_t sendAckData[2] = {0xAE, 0}; /* 0xAE ACK value, length = 0 */
uint8_t receivedData[MAX_PACKET_LENGTH];
uint8_t receivedAckData[MAX_PACKET_LENGTH];
static uint8_t retries = 0;

uint8_t flag_SendingPacket = FALSE;
uint8_t SendingPacketFailed = FALSE;

/* Private function prototypes -----------------------------------------------*/
uint8_t TxCallback(ActionPacket* p, ActionPacket* next);
uint8_t RxCallback(ActionPacket* p, ActionPacket* next);

/* Private functions ---------------------------------------------------------*/

/**
* @brief  This routine is called when a receive event is complete. 
* @param  p: Current action packet which its transaction has been completed.
* @param  next: Next action packet which is going to be scheduled.
* @retval return value: TRUE
*/
uint8_t RxCallback(ActionPacket* p, ActionPacket* next)
{    
  /* received a packet */
  
  if((p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0){
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
      BSP_LED_Toggle(BSP_LED2);
      fifo_put_var_len_item(&blueRec_fifo, p->data[1]+2, &p->data[0],0,NULL);
      if( (p->status & BLUE_INTERRUPT1REG_ENCERROR) != 0) {
        BSP_LED_On(BSP_LED2);
      }
      else {
        BSP_LED_Off(BSP_LED2);
      }
    }
    else if((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) {
      BSP_LED_Toggle(BSP_LED1);
      if(flag_SendingPacket == FALSE) {    
        HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback);
      }
      else {
        HAL_RADIO_SendPacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_TRANSMIT_RELATIVETIME, sendData, receivedAckData, CHAT_RECEIVE_TIMEOUT, MAX_LL_PACKET_LENGTH, TxCallback);
      }        
    }
    else if((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0) {
      if(flag_SendingPacket == FALSE) {    
        HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback);
      }
      else {
        HAL_RADIO_SendPacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_TRANSMIT_RELATIVETIME, sendData, receivedAckData, CHAT_RECEIVE_TIMEOUT, MAX_LL_PACKET_LENGTH, TxCallback);
      }   
    }
  }
  else if((p->status & BLUE_INTERRUPT1REG_DONE) != 0){
    BSP_LED_Toggle(BSP_LED3);
    if(flag_SendingPacket == FALSE) { 
      HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback);
    }
    else {
      HAL_RADIO_SendPacketWithAck(CHAT_FREQUENCYCHANNEL,CHAT_TRANSMIT_RELATIVETIME, sendData, receivedAckData, CHAT_RECEIVE_TIMEOUT, MAX_LL_PACKET_LENGTH, TxCallback);
    }
  }
  return TRUE;
}

/**
* @brief  This routine is called when a transmit event is complete. 
* @param  p: Current action packet which its transaction has been completed.
* @param  next: Next action packet which is going to be scheduled.
* @retval return value: TRUE
*/
uint8_t TxCallback(ActionPacket* p, ActionPacket* next)
{   
  /* received a packet */
  if((p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0){
    
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
      flag_SendingPacket = FALSE;
      HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback);
    }
    else if((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) {
      if (retries > 0) {
        HAL_RADIO_SendPacketWithAck(CHAT_FREQUENCYCHANNEL,CHAT_TRANSMIT_RELATIVETIME, sendData, receivedAckData, CHAT_RECEIVE_TIMEOUT, MAX_LL_PACKET_LENGTH, TxCallback);
        retries--;
      }
      else {
        flag_SendingPacket = FALSE;
        SendingPacketFailed = TRUE;
        HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback);
      }
    }
    else if((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0) {
      if (retries > 0) {
        HAL_RADIO_SendPacketWithAck(CHAT_FREQUENCYCHANNEL,CHAT_TRANSMIT_RELATIVETIME, sendData, receivedAckData, CHAT_RECEIVE_TIMEOUT, MAX_LL_PACKET_LENGTH, TxCallback);
        retries--;
      }
      else {
        flag_SendingPacket = FALSE;
        SendingPacketFailed = TRUE;
        HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback);        
      }
    }
  }
  /* Transmit complete */
  else {
  }
  
  return TRUE;
}

static FlagStatus xUartDataReady = RESET;
void IOProcessInputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
  static uint8_t cUartDataSize = 0;
  uint8_t i;
  
  for (i = 0; i < Nb_bytes; i++) {
    uart_buffer[cUartDataSize] = data_buffer[i];
    BSP_COM_Write(&data_buffer[i],1);
    cUartDataSize++;

    if((uart_buffer[cUartDataSize-1] == '\r') || (cUartDataSize == (MAX_CHAT_PACKET_LEN - MIC_FIELD_LEN))) {
      xUartDataReady = SET;
      Osal_MemCpy(&sendData[2], uart_buffer, cUartDataSize);
			sendData[1] = cUartDataSize + MIC_FIELD_LEN;
      cUartDataSize = 0;
    }
  }
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



  /* Configure I/O communication channel */
  BSP_COM_Init(IOProcessInputData);
  
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);

  RADIO_Init();
  HAL_VTIMER_Init(&VTIMER_InitStruct);

  /* Create the blueRec FIFO */
  fifo_init(&blueRec_fifo, MAX_PACKET_LENGTH*2, blueRec_buffer, 1);
  
  /* Set the Network ID */
  HAL_RADIO_SetNetworkID(BLE_ADV_ACCESS_ADDRESS);
  
  /* Configures the transmit power level */
  RADIO_SetTxPower(MAX_OUTPUT_RF_POWER);

#ifdef PKT_ENCRYPTION
  /* Set the encryption parameter */
  uint8_t RxCounter[5]    = {0,0,0,0,0};
  uint8_t TxCounter[5]    = {0,0,0,0,0};
  uint8_t encKey[16]      = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xFF,0xFF};
  uint8_t encInitVector[8]= {0,0,0,0,0,0,0,0};
  
  RADIO_SetEncryptionCount(STATE_MACHINE_0, TxCounter, RxCounter);
  RADIO_SetEncryptionAttributes(STATE_MACHINE_0, encInitVector, encKey);
  RADIO_SetEncryptFlags(STATE_MACHINE_0, ENABLE, ENABLE);
#endif
  
  /* Receives a packet. Then sends a packet as an acknowledgment. */
  HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback); 
  
  /* Infinite loop */
  while(1) {
    HAL_VTIMER_Tick();
    
    /* SendingPacket == FALSE */
    if(flag_SendingPacket == FALSE) {

      if(xUartDataReady == SET) {
        /* enable sending */
        sendData[0] = 0x02;
        xUartDataReady = RESET;
        retries = MAX_RETRIES;        
        flag_SendingPacket = TRUE; 
      }
    }
    
    if(fifo_size(&blueRec_fifo) !=0) {
      /* Get the length of the packet and the packet itself */
      fifo_get_var_len_item(&blueRec_fifo, &length, packet);        
      
      /* -4 because the MIC field */
      for(uint8_t i= 2; i<(length-MIC_FIELD_LEN); i++) {
        printf("%c", packet[i]);
      }
    }
    else if(SendingPacketFailed == TRUE) {
      SendingPacketFailed = FALSE;
      printf("\n\r NACK \r\n");
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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
