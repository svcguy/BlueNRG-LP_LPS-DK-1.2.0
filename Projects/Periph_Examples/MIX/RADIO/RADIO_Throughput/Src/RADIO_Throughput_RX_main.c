/**
  ******************************************************************************
  * @file    RADIO/RADIO_Throughput/Src/RADIO_Throughput_RX_main.c
  * @author  RF Application Team
  * @brief   Code demonstrating the throughput in a Tx/Rx or unidirectional
  *          scenario.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdio.h"
#include "rf_driver_hal_vtimer.h"
#include "rf_driver_hal_radio_2g4.h"
#include "rf_driver_ll_radio_2g4.h"
#include "main_common.h"

#include "bluenrg_lp_evb_config.h"

#if ST_USE_OTA_RESET_MANAGER
#include "radio_ota.h"
#endif

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
uint8_t channel = FREQUENCY_CHANNEL;
uint8_t sendAckData[2] = {0xAE, 0}; /* 0xAE ACK value, length = 0 */
uint8_t receivedData[MAX_PACKET_LENGTH];
static ActionPacket aPacket[2]; 

/* Private function prototypes -----------------------------------------------*/
uint8_t RxCallback(ActionPacket* p, ActionPacket* next);
uint8_t BIDIRECTIONAL_RX_Sequence(uint8_t channel, 
                                  uint8_t* rxBuffer, 
                                  uint8_t* txBuffer,
                                  uint32_t receive_timeout,
                                  uint8_t receive_length, 
                                  uint8_t (*Callback)(ActionPacket*, ActionPacket*));

/* Private functions ---------------------------------------------------------*/
static uint8_t CondRoutineTrue(ActionPacket* p)
{
  return TRUE;
}

static uint8_t CondRoutineRxTrue(ActionPacket* p)
{
  /* received a packet */
  if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0){
    /* packet received without CRC error */ 
    return TRUE;
  }
  return FALSE; 
}

/**
* @brief  This routine is called when a receive event is complete. 
* @param  p: Current action packet which its transaction has been completed.
* @param  next: Next action packet which is going to be scheduled.
* @retval return value: TRUE
*/
uint8_t RxCallback(ActionPacket* p, ActionPacket* next)
{
  uint8_t ret;
  
  /* received a packet */
 if((p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0){

  if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
    BSP_LED_Toggle(BSP_LED1);
  }
  else if( ((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) || ((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0) ){
    BSP_LED_Toggle(BSP_LED2);
    ret = BIDIRECTIONAL_RX_Sequence(channel, receivedData, sendAckData, RX_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback);
    if(ret != SUCCESS_0) {
      BSP_LED_On(BSP_LED1);
      BSP_LED_On(BSP_LED2);
      while(1);
    }
  }
}
  /* Transmit complete */ 
  else if((p->status & BLUE_INTERRUPT1REG_DONE) != 0) {
    ret = BIDIRECTIONAL_RX_Sequence(channel, receivedData, sendAckData, RX_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback);
    if(ret != SUCCESS_0) {
      BSP_LED_On(BSP_LED1);
      BSP_LED_On(BSP_LED2);
      while(1);
    }
  }
  return TRUE;   
}

uint8_t BIDIRECTIONAL_RX_Sequence(uint8_t channel, 
                                  uint8_t* rxBuffer, 
                                  uint8_t* txBuffer,
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
    
  if(returnValue == SUCCESS_0) {
    uint8_t map[5]= {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
  
    RADIO_SetChannelMap(0, &map[0]);
    RADIO_SetChannel(0, channel, 0);
    RADIO_SetTxAttributes(0, BLE_ADV_ACCESS_ADDRESS, 0x555555);
    RADIO_SetGlobalReceiveTimeout(receive_timeout);
    
    aPacket[0].StateMachineNo = STATE_MACHINE_0;
    aPacket[0].ActionTag =  0;
    aPacket[0].WakeupTime = 0;
    aPacket[0].MaxReceiveLength = receive_length;
    aPacket[0].data = rxBuffer;
    aPacket[0].next_true = &aPacket[1];
    aPacket[0].next_false = NULL_0;
    aPacket[0].condRoutine = CondRoutineRxTrue;
    aPacket[0].dataRoutine = Callback;
        
    aPacket[1].StateMachineNo = STATE_MACHINE_0;
    aPacket[1].ActionTag =  TXRX;
    aPacket[1].WakeupTime = 0;
    aPacket[1].MaxReceiveLength = 0; /* does not affect for Tx */
    aPacket[1].data = txBuffer;
    aPacket[1].next_true = NULL;
    aPacket[1].next_false = NULL;
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

  /* Configure the LEDs */
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);
    
  BSP_COM_Init(NULL);

  RADIO_Init();
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
  /* Set the Network ID */
  HAL_RADIO_SetNetworkID(BLE_ADV_ACCESS_ADDRESS);
  
  /* Configures the transmit power level */
  RADIO_SetTxPower(MAX_OUTPUT_RF_POWER);
  RADIO_SetBackToBackTime(100);
  ret = HAL_RADIO_ReceivePacketWithAck(channel, RX_WAKEUP_TIME, receivedData, sendAckData, RX_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback);
  if(ret != SUCCESS_0) {
    BSP_LED_On(BSP_LED1);
    BSP_LED_On(BSP_LED2);
    while(1);
  }   
  
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
