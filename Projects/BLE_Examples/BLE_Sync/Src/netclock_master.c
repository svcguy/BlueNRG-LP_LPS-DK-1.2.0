/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : netclock_master.c
* Author             : AMS - RF  Application team
* Description        : This library implements a netowrk clock that can be
*                      synchronized with another clock through sync events. To
*                      be used on master side.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include <system_util.h>
#include "rf_driver_hal_vtimer.h"
#include "netclock.h"

#define DEBUG   0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

void NETCLOCK_Init(void)
{
}

uint8_t NETCLOCK_GetCurrentNetTime(uint64_t *net_time)
{
  *net_time = HAL_VTIMER_GetCurrentSysTime();
  
  return TRUE;    
}

VTIMER_HandleType timerHandle;

int NETCLOCK_StartTimer(uint64_t net_time, VTIMER_CallbackType cb)
{  
  PRINTF("vtime %llu (0x%016llx)\n", net_time, net_time);
  
  timerHandle.callback = cb;
    
  return HAL_VTIMER_StartTimerSysTime(&timerHandle, net_time);
}
