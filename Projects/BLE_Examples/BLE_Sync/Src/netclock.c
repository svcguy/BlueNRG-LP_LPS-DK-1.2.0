/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : netclock.c
* Author             : AMS - RF  Application team
* Description        : Common functions for master/slave for network clock library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include "netclock.h"

uint64_t NETCLOCK_convertSysTime32ToSysTime64(uint32_t sysTime)
{
  uint64_t current_time = HAL_VTIMER_GetCurrentSysTime();
  uint32_t sysTime_ms32b = current_time >> 32; /* Most significant 32 bits of sysTime64 */
  
  if((int32_t)(sysTime - (uint32_t)current_time) > 0)
  {
    /* sysTime is in the future */    
    if(sysTime < (uint32_t)current_time){    
      /* Need to get most signicant 32 bits of current time increased by one */
      sysTime_ms32b++;
    }
  }
  else
  {
    /* sysTime is in the past */
    if(sysTime > (uint32_t)current_time){    
      /* Need to get most signicant 32 bits of current time decreased by one */
      sysTime_ms32b--;
    }    
  }
  
  return sysTime | (((uint64_t)sysTime_ms32b) << 32);
  
}

uint64_t NETCLOCK_convertPastSysTime32ToSysTime64(uint32_t sysTime)
{
  uint64_t current_time = HAL_VTIMER_GetCurrentSysTime();
  uint32_t sysTime_ms32b = current_time >> 32; /* Most significant 32 bits of sysTime64 */
  
  if(sysTime > (uint32_t)current_time)
  {
    /* Overflow of the 32-bit version of sysTime  */
    sysTime_ms32b--;
  }
  
  return sysTime | (((uint64_t)sysTime_ms32b) << 32);  
}
