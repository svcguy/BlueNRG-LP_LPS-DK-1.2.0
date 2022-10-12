
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RADIO_Sniffer_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 25-March-2019
* Description        : Sniffer application
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RADIO_Sniffer/RADIO_Sniffer_main.c
 * @brief Sniffer application.
 * It is also possible to enable the encryption capability.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\Sniffer\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_Sniffer.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\Sniffer\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_Sniffer.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\sniffer\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Encryption - Encryption configuration
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
|            |                            Release                            |||                          Encryption                           |||
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
|            |                            Release                            |||                          Encryption                           |||
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
|                |                             Release                             |||                           Encryption                            |||
----------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |
----------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |      Not Used      |       Not Used       |
|      PUSH2     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |      Not Used      |       Not Used       |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage
This demo shows a sniffer application on a specific frequency channel.
The application prints out on a serial terminal the packet received on a specific channel and with a specific BLE_ADV_ACCESS_ADDRESS.
Once the packet is received, it is printed the RSSI associated to the packet, the timestamp and the entire frame.
It is also possible to enable the encryption capability (Encryption configuration). 


**/
   

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdio.h"
#include "rf_driver_hal_radio_2g4.h"
#include "rf_driver_ll_radio_2g4.h"
#include "rf_driver_hal_vtimer.h"
#include "fifo.h"

#include "bluenrg_lp_evb_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_ADV_ACCESS_ADDRESS  (uint32_t)(0x8E89BED6)
#define HS_STARTUP_TIME         (uint16_t)(0x0107)  /* High Speed start up time min value */
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

uint32_t delay =1000;
uint8_t channel = 24;                    
uint32_t timeOut = 1000000;
int32_t rssi_val = 0;
uint32_t timestamp = 0;

#ifdef PACKET_ENCRYPTION
#define MIC_FIELD_LEN                   MIC_FIELD_LENGTH
uint8_t count_tx[5]     = {0x00,0x00,0x00,0x00,0x00};
uint8_t count_rcv[5]    = {0x00,0x00,0x00,0x00,0x00};
uint8_t enc_key[16]     = {0xBF,0x01,0xFB,0x9D,0x4E,0xF3,0xBC,0x36,0xD8,0x74,0xF5,0x39,0x41,0x38,0x68,0x4C};
uint8_t enc_iv[8]       = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#else

#define MIC_FIELD_LEN                   0
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint8_t nullfunction(ActionPacket* p,  ActionPacket* next)
{
  return TRUE;
}

uint8_t RxCallback(ActionPacket* p, ActionPacket* next)
{
  if( (p->status & BLUE_INTERRUPT1REG_DONE) != 0) {
    
    if((p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0){
     
      if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
        /* Check if there is enough space in the FIFO to store the incoming data */
        if(MAX_PACKET_LENGTH*2 - fifo_size(&blueRec_fifo) > (p->data[1]+ 4 + 4 + 4)){
          fifo_put_var_len_item(&blueRec_fifo, p->data[1]+2, p->data,0,NULL);
          fifo_put(&blueRec_fifo, 4, (uint8_t*)(&p->rssi));
          fifo_put(&blueRec_fifo, 4, (uint8_t*)(&p->timestamp_receive));
        }
        HAL_RADIO_ReceivePacket(channel, delay, receivedData, timeOut, MAX_LL_PACKET_LENGTH, RxCallback);
      }
      else if((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) {
        HAL_RADIO_ReceivePacket(channel,delay, receivedData, timeOut, MAX_LL_PACKET_LENGTH, RxCallback);
      }
      else if((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0) {
        HAL_RADIO_ReceivePacket(channel, delay, receivedData, timeOut, MAX_LL_PACKET_LENGTH, RxCallback);
      }
    }
  }
  return TRUE;
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

  /* Enable UART: 115200-8bit-No Parity-1 Stop bit */
  BSP_COM_Init(NULL);
  
  /* Create the blueRec FIFO */
  fifo_init(&blueRec_fifo,MAX_PACKET_LENGTH*2, blueRec_buffer, 1);
  
  RADIO_Init();
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
   /* Set the Network ID */
  HAL_RADIO_SetNetworkID(BLE_ADV_ACCESS_ADDRESS); /* 0x8E89BED6 */
    
  printf("\r\nSniffing channel: %d, Network_ID: 0x%08X\r\n", channel, BLE_ADV_ACCESS_ADDRESS);
  
#ifdef PACKET_ENCRYPTION
  /* Set Encryption key and Initial vector */
  RADIO_SetEncryptionAttributes(STATE_MACHINE_0, enc_iv, enc_key);
  
  RADIO_SetEncryptionCount(STATE_MACHINE_0, &count_tx[0], &count_rcv[0]); 
  
  /* Enable encryption:1 for Enable, 0 for disable */
  RADIO_SetEncryptFlags(STATE_MACHINE_0, ENABLE, ENABLE);
#endif
  
  /* Receives a packet on the desired channel. */
  HAL_RADIO_ReceivePacket(channel, delay, receivedData, timeOut, MAX_LL_PACKET_LENGTH, RxCallback);
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
      
      printf("\r\nchannel: %d, RSSI: %d dBm\r\n", channel, rssi_val);
      //printf("Timestamp: %.3f ms\r\n", ((float)timestamp)*625/256000.0);
      printf("Timestamp: %d.%03d ms\r\n", PRINT_INT((timestamp*625/256000.0)), PRINT_FLOAT((timestamp*625/256000.0)));
      
      
      printf("Frame: ");
      for(uint16_t i= 0; i<(length-MIC_FIELD_LEN); i++) {
        printf("%02x:", packet[i]);
      }
#ifdef PACKET_ENCRYPTION
      printf("\r\nMIC: ");
      for(uint16_t i= (length-MIC_FIELD_LEN); i<(length); i++) {
        printf("%02x:", packet[i]);
      }
#endif
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
