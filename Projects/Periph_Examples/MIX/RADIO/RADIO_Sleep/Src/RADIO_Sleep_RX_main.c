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
#include "rf_driver_hal_power_manager.h"
#include "main_common.h"

#include "bluenrg_lp_evb_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/  
uint8_t packet_counter = 0;
uint8_t sendData[MAX_PACKET_LENGTH];
uint8_t sendAckData[2] = {0xAE, 0}; /* 0xAE ACK value, length = 0 */
uint8_t receivedData[MAX_PACKET_LENGTH];
uint8_t receivedAckData[MAX_PACKET_LENGTH];

uint8_t rx_done = FALSE;
uint8_t schedule_rx = TRUE;
uint32_t rx_timeout = RX_TIMEOUT_NOTOK;

/* Private function prototypes -----------------------------------------------*/
uint8_t RxCallback(ActionPacket* p, ActionPacket* next);
/* Private functions ---------------------------------------------------------*/

PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
    return POWER_SAVE_LEVEL_RUNNING;
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}

/**
* @brief  This routine is called when a receive event is complete. 
* @param  p: Current action packet which its transaction has been completed.
* @param  next: Next action packet which is going to be scheduled.
* @retval return value: TRUE
*/
uint8_t RxCallback(ActionPacket* p, ActionPacket* next)
{  
  /* received a packet */
  if( (p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0) {
    
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
      rx_done = TRUE;
      rx_timeout = RX_TIMEOUT_OK;
    }
    else if( ((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) || ((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0) ){
      rx_timeout = RX_TIMEOUT_NOTOK;
      schedule_rx = TRUE;
    }
  }
  /* Transmit complete */ 
  else {
    schedule_rx = TRUE;
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
  
  /* Set the Network ID */
  HAL_RADIO_SetNetworkID(BLE_ADV_ACCESS_ADDRESS);
  
  /* Infinite loop */
  while(1) {
    /* Perform calibration procedure */    
    HAL_VTIMER_Tick();
    if(schedule_rx == TRUE) {
      schedule_rx = FALSE;
      
      ret = HAL_RADIO_ReceivePacketWithAck(FREQUENCY_CHANNEL, RX_WAKEUP_TIME, receivedData, sendAckData, rx_timeout, MAX_LL_PACKET_LENGTH, RxCallback);
      if(ret != SUCCESS_0) {
        printf("ERROR %d (%d)\r\n",ret, packet_counter);
      }
      
      if(rx_done == TRUE) {
        rx_done = FALSE;
        printf("Packet Received: ");
        for(volatile uint16_t i = 0; i < (receivedData[1] + HEADER_LENGTH); i++) {
          printf("%02X ", receivedData[i]);
        }
        printf("\r\n");
      }
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
