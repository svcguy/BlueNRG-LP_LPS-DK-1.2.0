/**
  ******************************************************************************
  * @file    RADIO/RADIO_StarNetwork/Src/RADIO_StarNetwork_Master_main.c
  * @author  RF Application Team
  * @brief   The code defines a Master device of a start network. 
*            The Master knows how many slaves are in the network and their slave
*            address. Then, periodically, asks for data the slaves and waits for
*            their data. If data are received, then the Master answers with an
*            ACK packet to the selected Slave.
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
uint8_t comm_token = 1;         /* Communication token */
uint8_t sendmeBuffer[2];
uint8_t receiveBuffer[MAX_PACKET_LENGTH];
uint8_t ackBuffer[2];

uint32_t networkID[NSLAVES] = {SLAVE_DEV1, SLAVE_DEV2};

ActionPacket waitDataAction;
ActionPacket sendmeAction;
ActionPacket sendAckAction;

uint32_t request_counter_test[NSLAVES] = {0,};
uint32_t 
pkt_counter_test[NSLAVES] = {0,};
uint32_t pkt_lost_counter_test[NSLAVES] = {0,};
uint8_t retry = 0;
uint8_t printer = 0;

/* Private function prototypes -----------------------------------------------*/
void InitializationActionPackets(void);
uint8_t sendmeCB(ActionPacket* p);
uint8_t waitDataCB(ActionPacket* p);
uint8_t sendAckCB(ActionPacket* p);
uint8_t dataRoutine(ActionPacket* p,  ActionPacket* next);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Condition routine.
  * @param  ActionPacket
  * @retval TRUE
  */
uint8_t sendmeCB(ActionPacket* p)
{
  BSP_LED_Toggle(BSP_LED1);
  request_counter_test[comm_token-1]++;
  printer++;
  return TRUE;
} 


uint8_t waitDataCB(ActionPacket* p)
{
  /* received a packet */
  if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
    printer++;
    pkt_counter_test[comm_token-1]++;
    return TRUE;
  }
  else if(((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) || ((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0)) {
    pkt_lost_counter_test[comm_token-1]++;
    printer = 0;
    
    comm_token++;
    if(comm_token > NSLAVES) {
      comm_token = 1;
    }
    RADIO_SetTxAttributes(STATE_MACHINE_0, networkID[comm_token-1], 0x555555);
    
    return FALSE;      
  }
  return TRUE;
}

/**
  * @brief  Data routine.
  * @param  ActionPacket: current
  * @param  ActionPacket: next
  * @retval TRUE
  */
uint8_t sendAckCB(ActionPacket* p)
{
  retry = 0;
  
  comm_token++;
  if(comm_token > NSLAVES) {
    comm_token = 1;
  }
  RADIO_SetTxAttributes(STATE_MACHINE_0, networkID[comm_token-1], 0x555555);
  
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
    
    if((p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0){
      
      /* received a packet */
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
    
  /* Build send me packet */
  sendmeBuffer[0] = 0xB4;
  sendmeBuffer[1] = 0;
  
  /* Build ACK packet */
  sendmeBuffer[0] = 0xAE;
  sendmeBuffer[1] = 0;
  
  /* Initialize the routine for sending data and wait for the ACK */
  InitializationActionPackets();
  
  
  /* Call this function for the first action packet to be executed */
  RADIO_MakeActionPacketPending(&sendmeAction);
  
  printf("\r\n\nRequest Sent,Packet received, Request lost\r\n");
  
  /* Infinite loop */
  while(1) {
    HAL_VTIMER_Tick();
    
    /* Printout a summary about the communication in the network */
    if(printer==0) {
      for(uint8_t i = 0; i< NSLAVES; i++) {
        printf("%8d,%8d,%8d,",(int)request_counter_test[i], (int)pkt_counter_test[i], (int)(request_counter_test[i]-pkt_counter_test[i]));
      }      
      printf("\r");
    }
  }   
}


void InitializationActionPackets(void)
{
  /* Build Action Packets */
  sendmeAction.StateMachineNo = STATE_MACHINE_0;
  sendmeAction.ActionTag = RELATIVE | TIMER_WAKEUP | TXRX | PLL_TRIG;
  sendmeAction.WakeupTime = 30+WKP_OFFSET;           /* Internal delay before start action */
  sendmeAction.MaxReceiveLength = 0;                 /* Not applied for TX */
  sendmeAction.data = sendmeBuffer;                  /* Data to send */
  sendmeAction.next_true = &waitDataAction;          /* Wait for data */
  sendmeAction.next_false = &waitDataAction;         /* Wait for data */
  sendmeAction.condRoutine = sendmeCB;               /* Condition callback */
  sendmeAction.dataRoutine = dataRoutine;            /* Data callback */

  waitDataAction.StateMachineNo = STATE_MACHINE_0;
  waitDataAction.ActionTag = RELATIVE | TIMER_WAKEUP;
  waitDataAction.WakeupTime = 30+WKP_OFFSET;               /* Internal delay before start action */
  waitDataAction.MaxReceiveLength = MAX_LL_PACKET_LENGTH;  
  waitDataAction.data = receiveBuffer;                     /* Buffer for ACK packet reception */
  waitDataAction.next_true = &sendAckAction;               /* If TRUE: send the ACK */   
  waitDataAction.next_false = &sendmeAction;               /* If FALSE: send a new request */
  waitDataAction.condRoutine = waitDataCB;                 /* Condition callback */
  waitDataAction.dataRoutine = dataRoutine;                /* Data callback */
  
  sendAckAction.StateMachineNo = STATE_MACHINE_0;
  sendAckAction.ActionTag = RELATIVE | TIMER_WAKEUP | TXRX;
  sendAckAction.WakeupTime = 40+WKP_OFFSET;           /* Internal delay before start action */
  sendAckAction.MaxReceiveLength = 0;                 
  sendAckAction.data = ackBuffer;                     /* Buffer for ACK packet reception */
  sendAckAction.next_true = &sendmeAction;            /* send a new request */   
  sendAckAction.next_false = &sendmeAction;           /* send a new request */   
  sendAckAction.condRoutine = sendAckCB;              /* Condition callback */
  sendAckAction.dataRoutine = dataRoutine;            /* Data callback */
  
  RADIO_SetGlobalReceiveTimeout(RX_TIMEOUT_DATA);
  
  /* Channel map configuration */
  uint8_t map[5]= {0xFF,0xFF,0xFF,0xFF,0xFF};
  RADIO_SetChannelMap(STATE_MACHINE_0, &map[0]);
  
  /* Set the channel */
  RADIO_SetChannel(STATE_MACHINE_0, APP_CHANNEL, 0);

  /* Sets of the NetworkID and the CRC */
  RADIO_SetTxAttributes(STATE_MACHINE_0, networkID[0], 0x555555);
    
  /* Call these functions before execute the action packets */
  RADIO_SetReservedArea(&sendmeAction);
  RADIO_SetReservedArea(&waitDataAction);
  RADIO_SetReservedArea(&sendAckAction);
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
