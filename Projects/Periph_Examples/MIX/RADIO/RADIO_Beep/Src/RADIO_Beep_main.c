
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RADIO_Beep_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 25-March-2019
* Description        : Transmission only example. The device sends a packet continuously in three different channels: 24, 25, 26 (with or without encryption)
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RADIO_Beep/RADIO_Beep_main.c
 * @brief Transmission only example. The device sends a packet continuously in three different channels: 24, 25, 26.
 * It also allows to enable the encryption capability.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_Beep\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_Beep.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_Beep\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_Beep.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\Beep\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Beep_Encryption - Beep with encryption configuration
- \c Beep_NO_Encryption - Beep with No encryption configuration


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
|            |                         Beep_NO_Encryption                          |||                        Beep_Encryption                        |||
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
|            |                         Beep_NO_Encryption                          |||                        Beep_Encryption                        |||
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
|                |                         Beep_NO_Encryption                          |||                         Beep_Encryption                         |||
--------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |    STEVAL-IDB012V1   |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |
--------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |       Not Used       |
|      PUSH2     |       Not Used       |       Not Used       |       Not Used       |      Not Used      |      Not Used      |       Not Used       |
|      RESET     |   Reset BlueNRG-LP   |   Reset BlueNRG-LP   |   Reset BlueNRG-LPS  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage
Transmission only example. The device sends a packet continuously in three different channels: 24, 25, 26. The device sends a packet of 16 bytes of payload and after each transmission the channel is changed.
The next channel setting is not done automatically as in the AutomaticChMgm example, but inside the callback conditionRoutine used by the mechanism of ActionPAcket. In the transmission part of the conditionRoutine, the new channel is programmed. In this example the ActionPacket mechanism exported by the BlueNRG_radio driver is used to schedule the transmission.
It is also possible to enable the encryption capability (Beep_Encryption configuration).


**/
   

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdio.h"
#include "rf_driver_hal_vtimer.h"
#include "rf_driver_ll_radio_2g4.h"

#include "bluenrg_lp_evb_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_ADV_ACCESS_ADDRESS  (uint32_t)(0x8E89BED6)
#define STARTING_CHANNEL        (uint8_t)(24)   // RF channel 22
#define END_CHANNEL             (uint8_t)(26)
#define HS_STARTUP_TIME         (uint16_t)(1)  /* High Speed start up time min value */
#define WAKEUP_TIME             100000         /* Wake up time 100 ms */
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
uint8_t channel = STARTING_CHANNEL;   /* Start Channel */
uint8_t sendData[MAX_PACKET_LENGTH];
uint8_t data_val = 0;
uint8_t DataLen;
ActionPacket actPacket; 

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

/**
  * @brief  Condition routine.
  * @param  ActionPacket
  * @retval TRUE
  */
uint8_t conditionRoutine(ActionPacket* p)
{
  if( (p->status & BLUE_INTERRUPT1REG_DONE) != 0) {
    if(p->status & BLUE_STATUSREG_PREVTRANSMIT){
      BSP_LED_Toggle(BSP_LED1);
      channel ++;
      if(channel == (END_CHANNEL+1)) {
        channel = STARTING_CHANNEL;
      }
      RADIO_SetChannel(STATE_MACHINE_0, channel, 0);
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
  for(uint8_t i = 0; i < (DataLen-15); i++) {
    sendData[i+17] = i + data_val;
  }
  data_val++;
  return TRUE;
}

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

  DataLen = 20;
  
  /* Build packet */
  sendData[0] = 0x02; 
  sendData[1] = DataLen + MIC_FIELD_LEN;   /* Length position is fixed */
  
  sendData[2] = 0x01; /* Advertising address */
  sendData[3] = 0x02;
  sendData[4] = 0x03;
  sendData[5] = 0x04;
  sendData[6] = 0x05;
  sendData[7] = 0x06;
  
  sendData[8] = 0x08; /* Length name */
  sendData[9] = 0x08; /* Shortened local name */
  
  sendData[10] = 'B'; /* Local name */
  sendData[11] = 'l';
  sendData[12] = 'u';
  sendData[13] = 'e';
  sendData[14] = 'N';
  sendData[15] = 'R';
  sendData[16] = 'G';
  

  for(uint8_t i = 0; i < (DataLen-15); i++) {
    sendData[i+17] = i + data_val;
  }
  data_val++;
  
  /* Build Action Packet */
  actPacket.StateMachineNo = STATE_MACHINE_0;
  actPacket.ActionTag = RELATIVE | TIMER_WAKEUP | TXRX | PLL_TRIG;
  actPacket.WakeupTime = WAKEUP_TIME;                
  actPacket.MaxReceiveLength = 0;                    /* Not applied for TX */
  actPacket.data = sendData;                         /* Data to send */
  actPacket.next_true = &actPacket;                  /* Pointer to the next Action Packet*/
  actPacket.next_false = NULL_0;                     /* Null */   
  actPacket.condRoutine = conditionRoutine;          /* Condition routine */
  actPacket.dataRoutine = dataRoutine;               /* Data routine */
  
  /* Channel map configuration */
  uint8_t map[5]= {0xFF,0xFF,0xFF,0xFF,0xFF};
  RADIO_SetChannelMap(STATE_MACHINE_0, &map[0]);
  
  /* Setting of channel and the channel increment*/
  RADIO_SetChannel(STATE_MACHINE_0, STARTING_CHANNEL, 0);

  /* Sets of the NetworkID and the CRC.*/
  RADIO_SetTxAttributes(STATE_MACHINE_0, BLE_ADV_ACCESS_ADDRESS, 0x555555);
  
  /* Configures the transmit power level */
  RADIO_SetTxPower(MAX_OUTPUT_RF_POWER);
  
#ifdef PACKET_ENCRYPTION  
  /* Configures the encryption count */
  RADIO_SetEncryptionCount(STATE_MACHINE_0, count_tx, count_rcv);
  
  /* Encryption Key and Initial Vector */
  RADIO_SetEncryptionAttributes( STATE_MACHINE_0, enc_iv, enc_key);
  
  /* Configures the packets encryption. Not used in this application */
  RADIO_SetEncryptFlags(STATE_MACHINE_0, ENABLE, ENABLE);
#endif
  
  /* Call this function before execute the action packet */
  RADIO_SetReservedArea(&actPacket);
  
  /* Call this function for the first action packet to be executed */
  RADIO_MakeActionPacketPending(&actPacket);
 
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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
