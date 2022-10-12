
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RADIO_Sleep_TX_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 01-July-2019
* Description        : This example shows the usage of the sleep functionality with the radio driver: TX transmission
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RADIO_Sleep/RADIO_Sleep_TX_main.c
 * @brief Code demonstrating a simple TX/RX scenario with sleep management. This code implements the transmitter side.
 * Two devices are necessary to run fully this demo.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_Sleep\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_Sleep.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_Sleep\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_Sleep.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\SleepTx\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c TX - TX sleep configuration


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
Code demonstrating a simple TX/RX scenario. This code implements the transmitter side.
Program two devices: one with the TX configuration and the other with the RX configuration. On the TX configuration, open a serial terminal to get the result of the communication.
The transmitter sends a number of packets equals to MAX_NUM_PACKET.The communication requires ACK to confirm the packet reception.
In this example the hal_radio layer is used in order to schedule the transmissions. The callback TXCallback provided to the API HAL_RADIO_SendPacketWithAck define the behavior of the radio according to the IRQ flags. If the IRQ_TIMEOUT occurs, the counter timeout_error_counter is incremented.If the IRQ_CRC_ERR occurs, the counter crc_error_counter is incremented.In the last byte of the packet is sent a counter that is increased with the number of packets sent. Once the maximum number of packets to send is reached, the summary of the communication is printed out with the information on the timeout errors, CRC errors, and the Packet Error Rate (PER). Then after a small delay, the sequence is repeated. The callback App_SleepMode_Check is used to avoid the sleep during the UART communication.

**/

 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rf_driver_hal_vtimer.h"
#include "rf_driver_hal_radio_2g4.h"
#include "rf_driver_hal_power_manager.h"
#include "main_common.h"
 
#include "bluenrg_lp_evb_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_NUM_PACKET           100    /* Number of packets used for the test */

#define PRINT_INT(x)    ((int)(x))
#define PRINT_FLOAT(x)  (x>0)? ((int) (((x) - PRINT_INT(x)) * 1000)) : (-1*(((int) (((x) - PRINT_INT(x)) * 1000))))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/  
uint16_t packet_counter = 0;
uint16_t crc_error_counter = 0;
uint16_t timeout_error_counter = 0;

uint16_t packet_counter_res = 0;
uint16_t crc_error_counter_res = 0;
uint16_t timeout_error_counter_res = 0;

uint8_t print_res = 0;

uint8_t channel = FREQUENCY_CHANNEL;
uint8_t sendData[MAX_PACKET_LENGTH];
uint8_t receivedData[MAX_PACKET_LENGTH];

uint8_t sendNewPacket = TRUE;

/* Private function prototypes -----------------------------------------------*/
uint8_t TxCallback(ActionPacket* p, ActionPacket* next);
/* Private functions ---------------------------------------------------------*/

PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
    return POWER_SAVE_LEVEL_RUNNING;
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}

/**
* @brief  This routine is called when a transmit event is complete. 
* @param  p: Current action packet which its transaction has been completed.
* @param  next: Next action packet which is going to be scheduled.
* @retval return value: TRUE
*/
uint8_t TxCallback(ActionPacket* p, ActionPacket* next)
{ 
  if((p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0){
    
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
    }
    else if((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) {
      timeout_error_counter++;
    }
    else if((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0) {
      crc_error_counter++;
    }
    sendData[6] = (uint8_t)(++packet_counter);
    
    if(packet_counter == MAX_NUM_PACKET) {
      print_res = 1;
      packet_counter_res = packet_counter;
      crc_error_counter_res = crc_error_counter;
      timeout_error_counter_res = timeout_error_counter;
      packet_counter = 0;
      timeout_error_counter = 0;
      crc_error_counter = 0;
      sendData[6] = 0;
    }
    sendNewPacket = TRUE;
  }
  return TRUE;
}

int main(void)
{
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  uint8_t ret;
  WakeupSourceConfig_TypeDef wakeupIO;
  PowerSaveLevels stopLevel;

  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }

  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  
  BSP_LED_On(BSP_LED1);
  
  /* Enable UART: 115200-8bit-No Parity-1 Stop bit */
  BSP_COM_Init(NULL);

  RADIO_Init();
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  /* No Wakeup Source needed */
  wakeupIO.IO_Mask_High_polarity = 0;
  wakeupIO.IO_Mask_Low_polarity = 0;
  wakeupIO.RTC_enable = 0;
  wakeupIO.LPU_enable = 0;
  
  /* Build packet */
  sendData[0] = 0x02;
  sendData[1] = 5;   /* Length position is fixed */
  sendData[2] = 0x01;
  sendData[3] = 0x02;
  sendData[4] = 0x03;
  sendData[5] = 0x04;
  sendData[6] = (uint8_t)packet_counter;

  /* Set the Network ID */
  HAL_RADIO_SetNetworkID(BLE_ADV_ACCESS_ADDRESS);
  
  /* Configures the transmit power level */
  RADIO_SetTxPower(MAX_OUTPUT_RF_POWER);
  
  while(1) {
    HAL_VTIMER_Tick();
    if(sendNewPacket){
      sendNewPacket = FALSE;
      ret =  HAL_RADIO_SendPacketWithAck(FREQUENCY_CHANNEL,TX_WAKEUP_TIME, sendData, receivedData, RX_TIMEOUT_ACK, MAX_LL_PACKET_LENGTH, TxCallback);
      if(ret != SUCCESS_0) {
        printf("ERROR %d (%d)\r\n",ret, packet_counter);
      }
    }
    if(print_res) {
      //printf("Channel = %d,Timeout errors = %d,CRC errors = %d,PER = %.2f %%\r\n",channel, timeout_error_counter_res, crc_error_counter_res, ((float)(timeout_error_counter_res + crc_error_counter_res))/packet_counter_res*100.0);
      
      printf("Channel = %d,Timeout errors = %d,CRC errors = %d,PER = %d.%02d  %%\r\n",channel, timeout_error_counter_res, crc_error_counter_res, PRINT_INT((timeout_error_counter_res + crc_error_counter_res)*100/packet_counter_res),PRINT_FLOAT((timeout_error_counter_res + crc_error_counter_res)*100/packet_counter_res));
      
      print_res = 0;
    }
    ret = HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_WITH_TIMER, wakeupIO, &stopLevel);
    if (ret != SUCCESS)
      printf("Error during clock config 0x%2x\r\n", ret);
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
