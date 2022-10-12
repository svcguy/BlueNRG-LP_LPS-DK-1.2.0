
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RADIO_OTAServer_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 25-March-2019
* Description        : Proprietary Over-The-Air Firmware Updated application, Server role.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RADIO_OTA_RM/RADIO_OTAServer_main.c
 * @brief Proprietary Over-The-Air Firmware Updated application, Server role.
 * Two devices are necessary to run fully this demo.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_OTA_RM\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_OTA_RM.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_OTA_RM\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_OTA_RM.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\OTAServer\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Server_FixedImage - OTA Server Fixed Image configuration. The fixed image is a .c file where is present an array holding a binary image.
- \c Server_Ymodem - OTA Server Ymodem configuration


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
|            |                          Server_FixedImage                          |||                         Server_Ymodem                         |||
--------------------------------------------------------------------------------------------------------------------------------------------------------
|  PIN name  |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |    STEVAL-IDB012V1   |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------------------------------------------------------------------------------
|     A0     |         N.A.         |         N.A.         |       Not Used       |        N.A.        |        N.A.        |      Not Used      |
|     A1     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     A10    |         N.A.         |         N.A.         |       Not Used       |        N.A.        |        N.A.        |      Not Used      |
|     A11    |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     A12    |       Not Used       |       Not Used       |         N.A.         |      Not Used      |      Not Used      |        N.A.        |
|     A13    |       Not Used       |       Not Used       |         N.A.         |      Not Used      |      Not Used      |        N.A.        |
|     A14    |       Not Used       |       Not Used       |         N.A.         |      Not Used      |      Not Used      |        N.A.        |
|     A15    |       Not Used       |       Not Used       |         N.A.         |      Not Used      |      Not Used      |        N.A.        |
|     A3     |         N.A.         |         N.A.         |       Not Used       |        N.A.        |        N.A.        |      Not Used      |
|     A4     |       Not Used       |       Not Used       |         N.A.         |      Not Used      |      Not Used      |        N.A.        |
|     A5     |       Not Used       |       Not Used       |         N.A.         |      Not Used      |      Not Used      |        N.A.        |
|     A6     |       Not Used       |       Not Used       |         N.A.         |      Not Used      |      Not Used      |        N.A.        |
|     A7     |       Not Used       |       Not Used       |         N.A.         |      Not Used      |      Not Used      |        N.A.        |
|     A8     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     A9     |       Not Used       |       Not Used       |         N.A.         |      Not Used      |      Not Used      |        N.A.        |
|     B0     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     B1     |         N.A.         |         N.A.         |       Not Used       |        N.A.        |        N.A.        |      Not Used      |
|     B12    |         N.A.         |         N.A.         |       Not Used       |        N.A.        |        N.A.        |      Not Used      |
|     B13    |         N.A.         |         N.A.         |       Not Used       |        N.A.        |        N.A.        |      Not Used      |
|     B14    |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     B15    |         N.A.         |         N.A.         |       Not Used       |        N.A.        |        N.A.        |      Not Used      |
|     B2     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     B3     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     B4     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     B5     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     B6     |         N.A.         |         N.A.         |       Not Used       |        N.A.        |        N.A.        |      Not Used      |
|     B7     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     B8     |       Not Used       |       Not Used       |         N.A.         |      Not Used      |      Not Used      |        N.A.        |
|     B9     |       Not Used       |       Not Used       |         N.A.         |      Not Used      |      Not Used      |        N.A.        |
|     GND    |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     RST    |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|    VBAT    |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |

@endtable 

* \section LEDs_description LEDs description
@table
|            |                          Server_FixedImage                          |||                         Server_Ymodem                         |||
--------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |    STEVAL-IDB012V1   |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     DL2    |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     DL3    |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     DL4    |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |
|     U5     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |      Not Used      |

@endtable

* \section Buttons_description Buttons description
@table
|                |                          Server_FixedImage                          |||                          Server_Ymodem                          |||
--------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |    STEVAL-IDB012V1   |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |
--------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |       Not Used       |
|      PUSH2     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |       Not Used       |
|      RESET     |   Reset BlueNRG-LP   |   Reset BlueNRG-LP   |   Reset BlueNRG-LPS  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage
Proprietary Over-The-Air Firmware Updated application. This code implements the Server side.
Program two devices: one with the Client configuration and the other with the Server configuration. On the Server configuration, open a serial terminal to transfer the binary image with the standard YMODEM.
As example, TxRX project provides the OTA configurations (RX_Use_OTA_ResetManager and TX_Use_OTA_ResetManager) allowing to build the required images to be transferred on the OTA Client device. On TxRx OTA configurations, PUSH2 button allows to activate the OTA Client and get new image from an OTA Server.

The OTA Server sends the binary image to the Client that load it in its Flash memory.
A proprietary protocol implemented on the Radio low level driver is used.


**/
   
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rf_driver_hal_radio_2g4.h"
#include "rf_driver_hal_vtimer.h"
#include "rf_driver_ll_flash.h"
#include "main_common.h"

#include "bluenrg_lp_evb_config.h"

#include "ymodem.h"
#include "clock.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define OTA_MANAGER_VERSION_STRING "1.0.0" 

#define TOKEN_RELEASE 0
#define TOKEN_TAKEN_YMODEM   1
#define TOKEN_TAKEN_OTA      2

#define YMODEM_START 0xFF

#define YMODEM_SIZE  0
#define YMODEM_LOAD  1
#define YMODEM_WAIT  2
#define YMODEM_CLOSE 4
#define YMODEM_COMPLETE 5
#define YMODEM_ABORT 9

#define MAX_RETRY     (10)


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static volatile uint16_t seq_num = 0;
static volatile uint16_t seq_num_max = 0;
static volatile uint8_t last_frame = 0;
static volatile uint8_t app_token = TOKEN_RELEASE;
static volatile uint8_t ymodem_state_machine_g = YMODEM_SIZE;
static volatile ota_state_machine_t ota_state_machine_g = OTA_CONNECTION;
static volatile uint32_t data_len = 0;
static volatile uint8_t bootloadingCompleted = 0;
static volatile uint8_t bootloadingCompleted_end = 0; 
static uint32_t image_size;

uint8_t image[1024+8];
uint8_t image_tmp[BYTE_IN_FRAME+2+2+2];
uint8_t rx_ota_buffer[MAX_PACKET_LENGTH], tx_ota_buffer[MAX_PACKET_LENGTH];

/* Private function prototypes -----------------------------------------------*/
uint8_t OTA_ConnectionCallback(ActionPacket* p, ActionPacket* next);
uint8_t OTA_SizeCallback(ActionPacket* p, ActionPacket* next);
uint8_t OTA_StartCallback(ActionPacket* p, ActionPacket* next);
uint8_t OTA_DataRequestCallback(ActionPacket* p, ActionPacket* next);
uint8_t OTA_SendDataCallback(ActionPacket* p, ActionPacket* next);

uint8_t OTA_Init(void);
uint8_t OTA_Tick(void);
int32_t OTA_ymodem_tick(void);


extern uint8_t rx_buffer[];
extern uint32_t rx_buffer_size;
extern uint8_t *rx_buffer_ptr;
extern uint8_t *rx_buffer_tail_ptr;

/**
* @brief  Process input routine used for handle the YMODEM data.
* @param  data_buffer The array with the data received
* @param  Nb_bytes The number of bytes received in the data_buffer array
* @retval None.
*/
void processInputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
  volatile int i;
  if (rx_buffer_size == 0) {
    rx_buffer_ptr = rx_buffer;
    rx_buffer_tail_ptr = rx_buffer_ptr;
  }
  
  for (i = 0; i < Nb_bytes; i++) {
    if( (rx_buffer_tail_ptr-rx_buffer) >= RX_BUFFER_SIZE){
      // Buffer is full
      return;
    }
    *rx_buffer_tail_ptr++ = data_buffer[i];
  }
  rx_buffer_size += Nb_bytes;
}



/**
  * @brief  Main program code.
  * @param  None
  * @retval None
  */
int main(void)
{
  /* System Init */
    if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) 
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }

  BSP_LED_Init(BSP_LED2);
  BSP_LED_On(BSP_LED2);
  
  Clock_Init();
  
  BSP_COM_Init(processInputData);
  
  OTA_Init();
  Ymodem_Init();
  PRINTF("\r\nBlueNRG-LP OTA Manager SERVER (version: %s)\r\n", OTA_MANAGER_VERSION_STRING);
  
  while(1) {
    
    /* Timer tick */
    HAL_VTIMER_Tick();
    
    OTA_ymodem_tick();
    
    OTA_Tick();
    
  }  
}


/**
* @brief  Callback used during CONNECTION state.
* @param  None.
* @retval None.
*/
uint8_t OTA_ConnectionCallback(ActionPacket* p, ActionPacket* next)
{
  /* received a packet */
  if( (p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0) {
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
      if(p->data[0] == HEADER_CONNECTION) {
        ota_state_machine_g = OTA_SIZE;
      }
      else {
        PRINTF(">> OTA_ConnectionCallback != %02x\r\n",p->data[0]);
        ota_state_machine_g = OTA_CONNECTION;
      }
    }
    else if(((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) || ((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0)) {
      ota_state_machine_g = OTA_CONNECTION;
    }
  }
  return TRUE;   
}


/**
* @brief  Callback used during SIZE state.
* @param  None.
* @retval None.
*/
uint8_t OTA_SizeCallback(ActionPacket* p, ActionPacket* next)
{
  static volatile uint8_t retry = 0;
  
  /* received a packet */
  if( (p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0) {
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
      if(p->data[0] == HEADER_SIZE) {
        ota_state_machine_g = OTA_START;
        retry = 0;
      }
      else {
        PRINTF(">> OTA_SizeCallback != %02x\r\n",p->data[0]);
        ota_state_machine_g = OTA_SIZE;
      }
    }
    else if(((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) || ((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0)) {
      ota_state_machine_g = OTA_SIZE;
      retry++;      
    }
  }
  /* Max number of Re-Try is handled raising a failure */
  if(retry > MAX_RETRY) {
    PRINTF(">> OTA_SizeCallback MAX RETRY\r\n");
    ota_state_machine_g = OTA_CONNECTION;
    retry = 0;
  }
  return TRUE;   
}


/**
* @brief  Callback used during START state.
* @param  None.
* @retval None.
*/
uint8_t OTA_StartCallback(ActionPacket* p, ActionPacket* next)
{
  static volatile uint8_t rx_valid = 0, retry = 0;
  
  /* received a packet */
  if( (p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0) {
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
      if(p->data[0] == HEADER_START) {
        rx_valid = 1;
      }
      else {
        PRINTF(">> OTA_StartCallback != %02x\r\n",p->data[0]);
      }
    }
    else if(((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) || ((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0)) {
      ota_state_machine_g = OTA_START;
      retry++;
    }
  }
  else { /* ACK transmission completed */
    if(rx_valid) {
      rx_valid = 0;
      retry = 0;
      ota_state_machine_g = OTA_DATAREQ;
    }
    else {
      ota_state_machine_g = OTA_START;
    }
  }
  /* Max number of Re-Try is handled raising a failure */
  if(retry > MAX_RETRY) {
    PRINTF(">> OTA_StartCallback MAX RETRY\r\n");
    ota_state_machine_g = OTA_CONNECTION;
    retry = 0;
  }
  return TRUE;   
}


/**
* @brief  Callback used during DATA REQUEST state.
* @param  None.
* @retval None.
*/
uint8_t OTA_DataRequestCallback(ActionPacket* p, ActionPacket* next)
{
  static volatile uint8_t rx_valid = 0, retry = 0;
  
  /* received a packet */
  if( (p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0) {
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
      if(p->data[0] == HEADER_DATAREQ) {
        if(p->data[1] == 2) {
          rx_valid = 1;
          seq_num = (((uint16_t)p->data[2])<<8) | ((uint16_t)p->data[3]);
        }
        else {
          PRINTF(">> OTA_DataRequestCallback != %d LENGTH WRONG\r\n", p->data[1]);
          BSP_LED_Off(BSP_LED2);
          while(1);
        }
      }
      /* It have received the ACK packet from GETDATA state */
      else if(p->data[0] == HEADER_SENDATA) {
        PRINTF(">> OTA_DataRequestCallback get HEADER_SENDATA\r\n");
      }
      else {
        PRINTF(">> OTA_DataRequestCallback != %02x\r\n", p->data[0]);
        while(1);
      }
    }
    else if(((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) || ((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0)) {
      ota_state_machine_g = OTA_DATAREQ;
      retry++;
    }
  }
  else { /* ACK transmission completed */
    if(rx_valid) {
      retry = 0;
      rx_valid = 0;
      ota_state_machine_g = OTA_SENDATA;
    }
    else { /* Received a HEADER_SENDATA */
      ota_state_machine_g = OTA_DATAREQ;
    }
  }
  if(retry > MAX_RETRY) { /* create a reset function */
    retry = 0;
    // TODO: handle the crash of the connection, once the Ymodem is opened (send abort on ymodem channel */
    PRINTF(">> OTA_DataRequestCallback MAX RETRY\r\n");
    //    ota_state_machine_g = OTA_CONNECTION;
    //    seq_num = 0;
  }
  return TRUE;   
}


/**
* @brief  Callback used during SEND DATA state.
* @param  None.
* @retval None.
*/
uint8_t OTA_SendDataCallback(ActionPacket* p, ActionPacket* next)
{
  static volatile uint8_t retry = 0;
  
  /* received a packet */
  if( (p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0) {
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
      retry = 0;
      if(p->data[0] == HEADER_SENDATA) {
        if(last_frame) {
          BSP_LED_Off(BSP_LED2);
          ota_state_machine_g = OTA_COMPLETE;
        }
        else {
          ota_state_machine_g = OTA_DATAREQ;
        }
      }
      else if(p->data[0] == HEADER_DATAREQ) {
        if(p->data[1] == 2) {
          seq_num = (((uint16_t)p->data[2])<<8) | ((uint16_t)p->data[3]);
          ota_state_machine_g = OTA_SENDATA;
        }
        else {
          PRINTF(">> OTA_DataRequestCallback != %d LENGTH WRONG\r\n", p->data[1]);
          BSP_LED_Off(BSP_LED2);
          while(1);
        }
      }
      else {
        PRINTF(">> OTA_SendDataCallback != %02x\r\n", p->data[0]);
        ota_state_machine_g = OTA_CONNECTION;
      }
    }
    else if(((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) || ((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0)) {
      ota_state_machine_g = OTA_SENDATA;
      retry++;
    }
  }
  if(retry > MAX_RETRY) {
    PRINTF(">> OTA_SendDataCallback MAX RETRY\r\n");
    retry = 0;
    ota_state_machine_g = OTA_DATAREQ;
  }
  return TRUE;   
}


/**
* @brief  Initialize the OTA state machine
* @param  None.
* @retval uint8_t return value.
*/
uint8_t OTA_Init(void)
{
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  
  RADIO_Init();
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
  /* Set the Network ID */
  HAL_RADIO_SetNetworkID(OTA_ACCESS_ADDRESS);
  
  /* Configures the transmit power level */
  RADIO_SetTxPower(MAX_OUTPUT_RF_POWER);
  
  tx_ota_buffer[0] = HEADER_CONNECTION;
  tx_ota_buffer[1] = 0;
  
  ota_state_machine_g = OTA_CONNECTION;
  
  return 0;
}

/**
* @brief  State machine routine for OTA
* @param  None.
* @retval uint8_t return value.
*/
static volatile uint32_t debugger[3+1];
uint8_t OTA_Tick()
{
  static uint32_t ota_pkt_idx = 0;
  static uint32_t ota_pkt_size = 0;
  uint8_t ret;
  
  /* Manage the token between the two state machines */
  if(app_token == TOKEN_TAKEN_YMODEM)
    return 0;
  else
    app_token = TOKEN_TAKEN_OTA;
  
  if(ota_state_machine_g == OTA_CONNECTION) {
    PRINTF("OTA_CONNECTION\r\n");
    
    /* state setting */
    last_frame = 0;
    seq_num = 0;
    seq_num_max = image_size / BYTE_IN_FRAME;
    if(image_size%BYTE_IN_FRAME) {
      seq_num_max++;
    }
    
    /* frame setting */
    tx_ota_buffer[0] = HEADER_CONNECTION;
    tx_ota_buffer[1] = 0;
    
    debugger[3] = ota_state_machine_g;
    ota_state_machine_g = OTA_ONGOING;
    RADIO_SetBackToBackTime(150);
    
    BSP_LED_On(BSP_LED2);
    ret = HAL_RADIO_SendPacketWithAck(OTA_CHANNEL, TX_WAKEUP_TIME, tx_ota_buffer, rx_ota_buffer, RX_TIMEOUT_ACK, MAX_LL_PACKET_LENGTH, OTA_ConnectionCallback);
    if(ret != SUCCESS_0) {
      PRINTF("ERROR %d\r\n",ret);
    }
  }
  
  else if(ota_state_machine_g == OTA_SIZE) {
    PRINTF("OTA_SIZE\r\n");
    
    /* frame setting */
    tx_ota_buffer[0] = HEADER_SIZE;
    tx_ota_buffer[1] = 4;
    tx_ota_buffer[2] = (uint8_t)(image_size>>24);
    tx_ota_buffer[3] = (uint8_t)(image_size>>16);
    tx_ota_buffer[4] = (uint8_t)(image_size>>8);
    tx_ota_buffer[5] = (uint8_t)image_size;
    
    debugger[3] = ota_state_machine_g;
    ota_state_machine_g = OTA_ONGOING;
    RADIO_SetBackToBackTime(150);
    ret = HAL_RADIO_SendPacketWithAck(OTA_CHANNEL, TX_WAKEUP_TIME, tx_ota_buffer, rx_ota_buffer, RX_TIMEOUT_ACK, MAX_LL_PACKET_LENGTH, OTA_SizeCallback);
    if(ret != SUCCESS_0) {
      PRINTF("ERROR %d\r\n",ret);
    }
  }
  
  else if(ota_state_machine_g == OTA_START) {
    PRINTF("OTA_START\r\n");
    
    /* frame setting */
    tx_ota_buffer[0] = HEADER_START;
    tx_ota_buffer[1] = 0;
    
    debugger[3] = ota_state_machine_g;
    ota_state_machine_g = OTA_ONGOING;
    RADIO_SetBackToBackTime(300);
    ret = HAL_RADIO_ReceivePacketWithAck(OTA_CHANNEL, RX_WAKEUP_TIME, rx_ota_buffer, tx_ota_buffer, RX_TIMEOUT_DATA, MAX_LL_PACKET_LENGTH, OTA_StartCallback);
    if(ret != SUCCESS_0) {
      PRINTF("ERROR %d\r\n",ret);
    }
  }
  
  else if(ota_state_machine_g == OTA_DATAREQ) {
    PRINTF("OTA_DATAREQ %d\r\n", seq_num);
    
    /* Manage the token between the two state machines */
    if(ymodem_state_machine_g == YMODEM_LOAD) {
      app_token = TOKEN_RELEASE;
      return 0;
    }
    
    /* frame setting */
    tx_ota_buffer[0] = HEADER_DATAREQ;
    tx_ota_buffer[1] = 0;
    debugger[3] = ota_state_machine_g;
    ota_state_machine_g = OTA_ONGOING;
    RADIO_SetBackToBackTime(300);
    ret = HAL_RADIO_ReceivePacketWithAck(OTA_CHANNEL, RX_WAKEUP_TIME, rx_ota_buffer, tx_ota_buffer, RX_TIMEOUT_DATA, MAX_LL_PACKET_LENGTH, OTA_DataRequestCallback);
    if(ret != SUCCESS_0) {
      PRINTF("ERROR %d\r\n",ret);
    }
  }
  
  else if(ota_state_machine_g == OTA_SENDATA) {
    PRINTF("OTA_SENDATA %d\r\n", seq_num);
    
    /* pointer  of buffer */
    ota_pkt_idx = (seq_num * BYTE_IN_FRAME);
    
    /* size of buffer */
    if((ota_pkt_idx + BYTE_IN_FRAME) <= data_len) {
      debugger[0]++;
      ota_pkt_size = BYTE_IN_FRAME + 2; /* +2 because the sequence number is added */
    }
    else if((ota_pkt_idx + 1) > data_len) {
      debugger[1]++;
      /* last frame already sent */
      ota_state_machine_g = OTA_DATAREQ;
      if(ymodem_state_machine_g == YMODEM_CLOSE) {
        while(1);
      }
      app_token = TOKEN_RELEASE;
      return 0;
    }
    /* still a few byte to send, it is the last frame */
    else if((ota_pkt_idx + 1) <= data_len) {
      debugger[2]++;
      ota_pkt_size = (data_len - ota_pkt_idx) + 2; /* +2 because the sequence number is added */
      if(ymodem_state_machine_g == YMODEM_CLOSE)
        last_frame = 1;
    }
    else {
      while(1); /* not possible ? */
    }
    
    /* copy the buffer */
    for(uint8_t i = 0; i < ota_pkt_size; i++) {
      image_tmp[4+i] = image[ota_pkt_idx%1024 + i];
    }
    
    image_tmp[0] = HEADER_SENDATA;
    image_tmp[1] = ota_pkt_size;
    image_tmp[2] = (uint8_t)(seq_num>>8);
    image_tmp[3] = (uint8_t)seq_num;
    
    debugger[3] = ota_state_machine_g;
    ota_state_machine_g = OTA_ONGOING;
    RADIO_SetBackToBackTime(150);
    ret = HAL_RADIO_SendPacketWithAck(OTA_CHANNEL, TX_WAKEUP_TIME, (uint8_t*)image_tmp, rx_ota_buffer, RX_TIMEOUT_ACK, MAX_LL_PACKET_LENGTH, OTA_SendDataCallback);
    if(ret != SUCCESS_0) {
      PRINTF("ERROR %d\r\n",ret);
    }
  }
  
  else if(ota_state_machine_g == OTA_COMPLETE) {
    PRINTF("OTA_COMPLETE\r\n");
    
    PRINTF("image %d\r\n",image_size);    
    ota_state_machine_g = OTA_CONNECTION;
    last_frame = 0;
    seq_num = 0;
    app_token = TOKEN_RELEASE;
  }
  
  if (bootloadingCompleted) { 
    bootloadingCompleted = 0; 
  }
  
  return (bootloadingCompleted_end);
}



/**
* @brief  State machine routine for YMODEM
* @param  None.
* @retval uint8_t return value.
*/
int32_t OTA_ymodem_tick(void)
{
  int32_t ret = 0;
  static volatile uint32_t packet = 0;
  uint32_t ymodem_data_len = 0;
  volatile uint32_t tmp_size;
  uint8_t tmp_buff[1024+8];  
  
  if(app_token == TOKEN_TAKEN_OTA)
    return 0;
  app_token = TOKEN_TAKEN_YMODEM;
  
  /* Get file name and size */
  if(ymodem_state_machine_g == YMODEM_SIZE) {
    ret = Ymodem_Receive ((uint8_t *)tmp_buff, 0, (uint32_t *) &image_size, packet++);    
    if (ret != YMODEM_CONTINUE) {
      ymodem_state_machine_g = YMODEM_ABORT;
    }
    else {
      ymodem_state_machine_g = YMODEM_LOAD;
      app_token = TOKEN_RELEASE;
    }
  }
  
  else if(ymodem_state_machine_g == YMODEM_LOAD) {
    
    /* Answer to the previous Receive */
    
    Ymodem_SendAck();
    
    ret = Ymodem_Receive (image, 0, (uint32_t *) &ymodem_data_len, packet++);      
    if (ret != YMODEM_CONTINUE) {
      ymodem_state_machine_g = YMODEM_ABORT;
      return ret;
    }
    else {
      app_token = TOKEN_RELEASE;
    }
    
    if((ymodem_data_len+data_len) > image_size) {
      ymodem_state_machine_g = YMODEM_CLOSE;
      
      /* Answer to the previous Receive */
      Ymodem_SendAck();
      data_len = image_size;
    }
    else {
      data_len += ymodem_data_len;
      ymodem_state_machine_g = YMODEM_WAIT;
    }
  }
  
  else if(ymodem_state_machine_g == YMODEM_WAIT) {
    /* Empty state waiting for OTA action. Still data with ymodem. */
    ymodem_state_machine_g = YMODEM_LOAD;
  }
  
  else if(ymodem_state_machine_g == YMODEM_CLOSE) {
    ymodem_state_machine_g = YMODEM_COMPLETE;
    
    /* Close Ymodem session */
    ret = Ymodem_Receive (tmp_buff, 0, (uint32_t *) &tmp_size, packet++);
    if (ret != YMODEM_DONE) {
      Ymodem_Abort();
    }
    Ymodem_SendAck();
  }
  
  else if(ymodem_state_machine_g == YMODEM_COMPLETE) {
    
  }
  
  else if(ymodem_state_machine_g == YMODEM_ABORT) {
    Ymodem_Abort();
    /* Reset all the states and variables or reset the system */
  }
  return 0;  
}


/***************************************************************************************/

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

