/**
  ******************************************************************************
  * @file    RADIO/RADIO_Tx/Src/RADIO_TxRx_RX_main.c
  * @author  RF Application Team
  * @brief   Code demonstrating a simple TX/RX scenario.
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
#include "main_common.h"

#include "bluenrg_lp_evb_config.h"

#if ST_USE_OTA_RESET_MANAGER
#include "radio_ota.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/  
uint8_t packet_counter = 0;
uint8_t channel = FREQUENCY_CHANNEL;
uint8_t sendAckData[2] = {0xAE, 0}; /* 0xAE ACK value, length = 0 */
uint8_t receivedData[MAX_PACKET_LENGTH];
uint8_t rx_done = FALSE, button_flag = 0;

/* Private function prototypes -----------------------------------------------*/
uint8_t RxCallback(ActionPacket* p, ActionPacket* next);
/* Private functions ---------------------------------------------------------*/

uint8_t RxCallback(ActionPacket* p, ActionPacket* next)
{
  uint8_t ret;
  /* received a packet */
  if((p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0){
    
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
      BSP_LED_Toggle(BSP_LED1);
      rx_done = TRUE;
    }
    else if( ((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) || ((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0) ){
      BSP_LED_Toggle(BSP_LED2);
      ret = HAL_RADIO_ReceivePacketWithAck(channel, RX_WAKEUP_TIME, receivedData, sendAckData, RX_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback);
      if(ret != SUCCESS_0) {
        printf("ERROR %d (%d)\r\n",ret, packet_counter);
      }
    }
  }
  else if((p->status & BLUE_INTERRUPT1REG_DONE) != 0) {
    ret = HAL_RADIO_ReceivePacketWithAck(channel, RX_WAKEUP_TIME, receivedData, sendAckData, RX_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback);
    if(ret != SUCCESS_0) {
      printf("ERROR %d (%d)\r\n",ret, packet_counter);
    }
  }
  return TRUE;   
}

int main(void)
{
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  uint8_t ret;

  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }

  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  BSP_LED_Off(BSP_LED3);
  
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_EXTI);

#if ST_USE_OTA_RESET_MANAGER
  BSP_PB_Init(BSP_PUSH2, BUTTON_MODE_GPIO);
  if (BSP_PB_GetState(BSP_PUSH2) == SET)
  {
    while(BSP_PB_GetState(BSP_PUSH2) == SET);
  }
#endif  
  
  /* Enable UART: 115200-8bit-No Parity-1 Stop bit */
  BSP_COM_Init(NULL);

  RADIO_Init();
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  /* Set the Network ID */
  HAL_RADIO_SetNetworkID(BLE_ADV_ACCESS_ADDRESS);

#ifdef TXRX_PHY_2M
  RADIO_SetPhy(STATE_MACHINE_0,PHY_2M);
#endif
  
 ret = HAL_RADIO_ReceivePacketWithAck(channel, RX_WAKEUP_TIME, receivedData, sendAckData, RX_TIMEOUT, MAX_LL_PACKET_LENGTH, RxCallback);
  if(ret != SUCCESS_0) {
    printf("ERROR %d (%d)\r\n",ret, packet_counter);
  }   
  
  /* Infinite loop */
  while(1) {
    /* Perform calibration procedure */    
    HAL_VTIMER_Tick();
    if(rx_done == TRUE) {
      printf("Packet Received: ");
      for(volatile uint16_t i = 0; i < (receivedData[1] + 2); i++) {
        printf("%02X ", receivedData[i]);
      }
      printf("\r\n");
      rx_done = FALSE;
    }
    if(button_flag == 1) {
      button_flag = 0;
      printf("Channel %d\r\n",channel);
    }
#if ST_USE_OTA_RESET_MANAGER
    if (BSP_PB_GetState(BSP_PUSH2) == SET)
    {
      OTA_Jump_To_Reset_Manager();
    }
#endif  
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
