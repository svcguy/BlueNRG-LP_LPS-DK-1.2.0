
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : RADIO_Throughput_TX_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 25-March-2019
* Description        : Application code for a throughput test, transmitter configuration.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RADIO_Throughput/RADIO_Throughput_TX_main.c
 * @brief Application code for a throughput test, transmitter configuration.
 * Two configurations: unidirectional (TX only), bidirectional (two devices are needed, one with the RX configuration).
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_Throughput\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_Throughput.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_Throughput\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_Throughput.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\TxThroughput\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Bidirectional_TX - TX bidirectional configuration
- \c Unidirectional - TX unidirectional configuration


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
|            |                        Unidirectional                         |||                       Bidirectional_TX                        |||
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

* \section Buttons_description Buttons description
@table
|                |                         Unidirectional                          |||                        Bidirectional_TX                         |||
----------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |    STEVAL-IDB012V1   |
----------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |      Not Used      |       Not Used       |
|      PUSH2     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |      Not Used      |       Not Used       |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage
Application code for a throughput test, transmitter configuration. 
Two configurations: unidirectional (TX only), bidirectional (two devices are needed, one with the RX configuration).
The unidirectional configuration show the time needed to send a packet, from the transmission command to the TX callback. The time is measured with the TIM peripheral. The result is shown after the MAX_NUM_PACKET (1000) has been sent. The number of bytes of payload can be changed using the DATA_PACKET_LEN symbol. It is not required an RX device for this configuration, but could be used the sniffer application if it is necessary to get the packets.
In the bidirectional configuration, the TX device will wait for the ACK packet. The usage is the same of the unidirectional configuration, but it is necessary to have also another device configured with the RxThroughput Bidirectional.

**/
   

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdio.h"
#include "rf_driver_hal_vtimer.h"
#include "rf_driver_hal_radio_2g4.h"
#include "rf_driver_ll_radio_2g4.h"
#include "rf_driver_ll_tim.h"
#include "main_common.h"

#include "bluenrg_lp_evb_config.h"


/** @addtogroup BlueNRG1_StdPeriph_Examples
* @{
*/

/** @addtogroup RADIO_Examples RADIO Examples
* @{
*/

/** @addtogroup RADIO_TxRx RADIO TxRx Example
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_NUM_PACKET          1000    /* Number of packets used for the test */
#define DATA_PACKET_LEN         20 //20 //255 //31    /* PDU length in bytes  */
#define CALIBRATION_INTERVAL_CONF   10000

#ifdef STEVAL_IDB011V1
#define TIMx                                                    TIM1
#define LL_EnableClock_TIMx()                                   LL_APB0_EnableClock(LL_APB0_PERIPH_TIM1)
#define TIMx_IRQHandler                                         TIM1_IRQHandler
#define TIMx_IRQn                                               TIM1_IRQn
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
#define TIMx                                                    TIM2
#define LL_EnableClock_TIMx()                                   LL_APB0_EnableClock(LL_APB0_PERIPH_TIM2);
#define TIMx_IRQHandler                                         TIM2_IRQHandler
#define TIMx_IRQn                                               TIM2_IRQn
#endif /* STEVAL_IDB012V1 */

#if defined CONFIG_HW_LS_RO  

/* Calibration must be done */
#define INITIAL_CALIBRATION TRUE
#define CALIBRATION_INTERVAL        CALIBRATION_INTERVAL_CONF

#elif defined CONFIG_HW_LS_XTAL

/* No Calibration */
#define INITIAL_CALIBRATION FALSE
#define CALIBRATION_INTERVAL        0

#endif

#ifdef STEVAL_IDB011V1
#define TIMx                                                    TIM1
#define LL_EnableClock_TIMx()                                   LL_APB0_EnableClock(LL_APB0_PERIPH_TIM1)
#define TIMx_IRQHandler                                         TIM1_IRQHandler
#define TIMx_IRQn                                               TIM1_IRQn
#endif /* STEVAL_IDB011V1 */

#ifdef STEVAL_IDB012V1
#define TIMx                                                    TIM2
#define LL_EnableClock_TIMx()                                   LL_APB0_EnableClock(LL_APB0_PERIPH_TIM2);
#define TIMx_IRQHandler                                         TIM2_IRQHandler
#define TIMx_IRQn                                               TIM2_IRQn
#endif /* STEVAL_IDB012V1 */

#define PRINT_INT(x)    ((int)(x))
#define PRINT_FLOAT(x)  (x>0)? ((int) (((x) - PRINT_INT(x)) * 1000)) : (-1*(((int) (((x) - PRINT_INT(x)) * 1000))))
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t tim_prescaler = 0;
static uint32_t tim_period = 0;
uint16_t packet_counter = 0;
uint16_t crc_error_counter = 0;
uint16_t timeout_error_counter = 0;
uint8_t channel = FREQUENCY_CHANNEL;
uint8_t button_flag = 0;

static uint32_t time2 = 0, diff = 0, time_cumulate = 0;
extern uint32_t timer_reload;
static uint32_t timer_reload_store = 0;

static ActionPacket aPacket[2]; 

uint8_t sendData[MAX_PACKET_LENGTH];
uint8_t receivedData[MAX_PACKET_LENGTH];
uint8_t sendNewPacket = FALSE;

/* Private function prototypes -----------------------------------------------*/
uint8_t TxCallback(ActionPacket* p, ActionPacket* next);
uint8_t UNIDIRECTIONAL_Sequence(uint8_t channel, 
                                uint8_t* txBuffer, 
                                uint8_t (*Callback)(ActionPacket*, ActionPacket*));

uint8_t BIDIRECTIONAL_TX_Sequence(uint8_t channel, 
                                   uint8_t* txBuffer, 
                                   uint8_t* rxBuffer,
                                   uint32_t receive_timeout, 
                                   uint8_t receive_length,
                                   uint8_t (*Callback)(ActionPacket*, ActionPacket*));
/* Private functions ---------------------------------------------------------*/
static uint8_t CondRoutineTrue(ActionPacket* p)
{
  return TRUE;
}

static uint8_t dataRoutineNull(ActionPacket* current_action_packet, ActionPacket* next)
{
  return TRUE;
}

/**
  * @brief  Timer Configuration.
  * @param  None
  * @retval None
  */
static void MX_TIMx_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_EnableClock_TIMx();

  /* TIMx interrupt Init */
  NVIC_SetPriority(TIMx_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(TIMx_IRQn);

  TIM_InitStruct.Prescaler = tim_prescaler;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_DOWN;
  TIM_InitStruct.Autoreload = tim_period;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIMx, &TIM_InitStruct);
  LL_TIM_SetClockSource(TIMx, LL_TIM_CLOCKSOURCE_INTERNAL);

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
    time2 = LL_TIM_GetCounter(TIMx);
    timer_reload_store = timer_reload;
    packet_counter++;
  }
  else if((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) {
  }
  else if((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0) {
  }

  if(packet_counter != MAX_NUM_PACKET) {
    sendNewPacket = TRUE;
  }
  else
  {
    diff = (0xFFFF - time2) + timer_reload_store*0xFFFF;
    time_cumulate += diff;
  }
}
  /* Transmit complete */
  else if((p->status & BLUE_INTERRUPT1REG_DONE) != 0){
    if(packet_counter != MAX_NUM_PACKET) {
      sendNewPacket = TRUE;
    }
  }
  return TRUE;
}


uint8_t UniTxCallback(ActionPacket* p, ActionPacket* next)
{ 
  /* Transmit complete */
  if((p->status & BLUE_INTERRUPT1REG_DONE) != 0){
    time2 = LL_TIM_GetCounter(TIMx);
    timer_reload_store = timer_reload;
    packet_counter++;
    if(packet_counter != MAX_NUM_PACKET) {
      sendNewPacket = TRUE;
    }
  }
  return TRUE;
}

uint8_t UNIDIRECTIONAL_Sequence(uint8_t channel, 
                                uint8_t* txBuffer, 
                                uint8_t (*Callback)(ActionPacket*, ActionPacket*))
{
  uint8_t returnValue = SUCCESS_0;
  uint32_t dummy;
  
  
  if(channel > 39) {
    returnValue = INVALID_PARAMETER_C0;
  }
  
  if(RADIO_GetStatus(&dummy) != BLUE_IDLE_0) {
    returnValue = RADIO_BUSY_C4;
  }
  
  if(returnValue == SUCCESS_0) {
    uint8_t map[5]= {0xFF,0xFF,0xFF,0xFF,0xFF};
    RADIO_SetChannelMap(0, &map[0]);
    RADIO_SetChannel(0, channel, 0);
    RADIO_SetTxAttributes(0, BLE_ADV_ACCESS_ADDRESS, 0x555555);

    aPacket[0].StateMachineNo = STATE_MACHINE_0;
    aPacket[0].ActionTag =  TXRX;
    aPacket[0].WakeupTime = 0;
    aPacket[0].MaxReceiveLength = 0; /* does not affect for Tx */
    aPacket[0].data = txBuffer;
    aPacket[0].next_true = NULL_0;
    aPacket[0].next_false = NULL_0;
    aPacket[0].condRoutine = CondRoutineTrue;
    aPacket[0].dataRoutine = Callback;
    
    RADIO_SetReservedArea(&aPacket[0]); 
    RADIO_MakeActionPacketPending(&aPacket[0]);
    blueglob->TIMER12INITDELAYCAL = 4;
    TIMER_SetRadioCloseTimeout();
  }
  
  return returnValue; 
}


uint8_t BIDIRECTIONAL_TX_Sequence(uint8_t channel, 
                                    uint8_t* txBuffer, 
                                    uint8_t* rxBuffer,
                                    uint32_t receive_timeout, 
                                    uint8_t receive_length,
                                    uint8_t (*Callback)(ActionPacket*, ActionPacket*))
{
  uint8_t returnValue = SUCCESS_0;
  uint32_t dummy;
  
 
  if(channel > 39) {
    returnValue = INVALID_PARAMETER_C0;      
  }

  if(RADIO_GetStatus(&dummy) != BLUE_IDLE_0) {
    returnValue = RADIO_BUSY_C4;
  }
      
  uint8_t map[5]= {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
  RADIO_SetChannelMap(0, &map[0]);
  RADIO_SetChannel(0, channel, 0);
  RADIO_SetTxAttributes(0, BLE_ADV_ACCESS_ADDRESS, 0x555555);
  RADIO_SetGlobalReceiveTimeout(receive_timeout);
  
  if(returnValue == SUCCESS_0) {
    aPacket[0].StateMachineNo = 0;
    aPacket[0].ActionTag = TXRX;
    aPacket[0].WakeupTime = 0;
    aPacket[0].MaxReceiveLength = 0; /* does not affect for Tx */
    aPacket[0].data = txBuffer;
    aPacket[0].next_true = &aPacket[1];
    aPacket[0].next_false = &aPacket[1];
    aPacket[0].condRoutine = CondRoutineTrue;
    aPacket[0].dataRoutine = dataRoutineNull;
    
    aPacket[1].StateMachineNo = STATE_MACHINE_0;   
    aPacket[1].ActionTag = 0;   
    aPacket[1].WakeupTime = 0;
    aPacket[1].MaxReceiveLength = receive_length; 
    aPacket[1].data = rxBuffer; 
    aPacket[1].next_true = NULL_0;
    aPacket[1].next_false = NULL_0;    
    aPacket[1].condRoutine = CondRoutineTrue;
    aPacket[1].dataRoutine = Callback;
    
    RADIO_SetReservedArea(&aPacket[0]);
    RADIO_SetReservedArea(&aPacket[1]);
    RADIO_MakeActionPacketPending(&aPacket[0]); 
    blueglob->TIMER12INITDELAYCAL = 4;
    TIMER_SetRadioCloseTimeout();
  }  
    
  return returnValue; 
}


/**
* @brief  This main routine.
*
*/
int main(void)
{  
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  uint8_t ret;
  
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }

  tim_prescaler = __LL_TIM_CALC_PSC(LL_TIM_GetPeriphClock(TIMx), 1000000);
  tim_period = 0xFFFF;

  /* Init Clock */
  MX_TIMx_Init();
  
  /* Configure I/O communication channel */
  BSP_COM_Init(NULL);
  RADIO_Init();
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
  if(DATA_PACKET_LEN > (MAX_PACKET_LENGTH-HEADER_LENGTH)) {
    printf("DATA_PACKET_LEN too big %d\r\n", DATA_PACKET_LEN);
    while(1);
  }
     
  /* Build packet */
  sendData[0] = 0x02;
  sendData[1] = DATA_PACKET_LEN;   /* Length position is fixed */
  for(volatile uint16_t j=0; j<DATA_PACKET_LEN; j++) {
    sendData[2+j] = 0xAE;
  }
  
  /* Configures the transmit power level */
  RADIO_SetTxPower(MAX_OUTPUT_RF_POWER);
  
  printf("BlueNRG-LP Radio Driver Throughput Application\r\n");
  
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Enable the update interrupt */
  LL_TIM_EnableIT_UPDATE(TIMx);

  /* Enable counter */
  LL_TIM_EnableCounter(TIMx);
  
  ret = HAL_RADIO_SendPacket(channel, TX_WAKEUP_TIME, sendData, TxCallback);
  if(ret != SUCCESS_0) {
    printf("ERROR %d (%d)\r\n",ret, packet_counter);
  }
  
  RADIO_SetBackToBackTime(100);
  /* Infinite loop */
  while(1) {
    HAL_VTIMER_Tick();
    
    if(packet_counter == MAX_NUM_PACKET) {
      if(time_cumulate!=0) {
        //printf("%d TX packets. %d PCKT LEN. Average time: %.2f ms. Data throughput: %.1f kbps.\r\n", packet_counter, DATA_PACKET_LEN, (((float)time_cumulate)/1000.0/packet_counter), (((float)packet_counter*DATA_PACKET_LEN*8)*1000.0)/time_cumulate);
        printf("%d TX packets. %d PCKT LEN. Average time: %d.%02d ms. Data throughput: %d.%01d kbps.\r\n", packet_counter, DATA_PACKET_LEN,PRINT_INT(time_cumulate/1000.0/packet_counter),PRINT_FLOAT(time_cumulate/1000.0/packet_counter),PRINT_INT((packet_counter*DATA_PACKET_LEN*8)*1000.0/time_cumulate),PRINT_FLOAT((packet_counter*DATA_PACKET_LEN*8)*1000.0/time_cumulate));
      }
      packet_counter = 0;
      timeout_error_counter = 0;
      crc_error_counter = 0;
      sendData[6] = 0;
      sendNewPacket = TRUE;
      time_cumulate = 0;
      for(volatile uint32_t i = 0; i<0xFFFFF; i++);
    }
    
    if(sendNewPacket == TRUE) {
      sendNewPacket = FALSE;
      if(packet_counter != 0){
        diff = (0xFFFF - time2) + timer_reload_store*0xFFFF;
      }
      time_cumulate += diff;
      timer_reload = 0;
      LL_TIM_SetCounter(TIMx, 0xFFFF);
#ifdef UNIDIRECTIONAL_TEST
      ret = UNIDIRECTIONAL_Sequence(channel, sendData, UniTxCallback);
#else
      ret = BIDIRECTIONAL_TX_Sequence(channel, sendData, receivedData, RX_TIMEOUT_ACK, MAX_LL_PACKET_LENGTH, TxCallback);
#endif
      if(ret != SUCCESS_0) {
        printf("ERROR %d (%d)\r\n",ret, packet_counter);
      }
      
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
