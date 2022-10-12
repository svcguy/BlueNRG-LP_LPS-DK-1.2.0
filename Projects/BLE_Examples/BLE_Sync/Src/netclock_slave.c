/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : netclock_slave.c
* Author             : AMS - RF  Application team
* Description        : This library implements a virtual clock that can be
*                      synchronized with another clock through sync events. To
*                      be used on slave side.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include <system_util.h>
#include <string.h>
#include "rf_driver_hal_vtimer.h"
#include "netclock.h"

#define DEBUG   0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

// Master will send a packet containing the value of the virtual timer and the value of the connection event counter at that time.
// This connection event counter may not be the latest one. So, at least last 3 events are stored.
#define NUM_SYNC_EVENTS 3

#define SYSTIME_MARGIN_FOR_TIMER    400

/* Set to 1 if the network timer must be updated each time a new sync event is
 received. Otherwise the timer expires using an old estimation of the network time.
 Setting to 1 is especially needed when the timer is programmed to expire in a distant
 future.  */
#define ADJUST_NETWORK_TIMER    1

struct sync_evt_slave {  
  uint64_t sys_time;      // Value of the system timer when last sync has been received (STU)
  uint16_t event_counter; // event counter when last sync has been received
  uint64_t net_time;      // network time when last sync has been received (STU)
  uint8_t  valid;         // If this is a valid sync event
};

static struct sync_evt_slave sync_evts[NUM_SYNC_EVENTS];
static int8_t last_saved_event_index = 0;

static uint8_t  time_offset_set = FALSE; // If vime_offset has been set or not
static uint32_t sync_interval = 0; // In systime

VTIMER_HandleType timer_handle;
uint64_t net_expiry_time;

void NETCLOCK_Init(void)
{
  memset(sync_evts, 0, sizeof(sync_evts));
  time_offset_set = FALSE;
  last_saved_event_index = 0;
  sync_evts[last_saved_event_index].sys_time = HAL_VTIMER_GetCurrentSysTime();
  sync_evts[last_saved_event_index].net_time = 0;
}

void NETCLOCK_SetSyncInterval(uint32_t interval)
{
  sync_interval = interval;
}

void NETCLOCK_SaveSyncEventOnSlave(uint64_t sys_time, uint16_t event_counter)
{  
  uint64_t old_vtime = sync_evts[last_saved_event_index].net_time;
  uint64_t old_systime = sync_evts[last_saved_event_index].sys_time;
  uint32_t net_time_from_last_sync = sys_time - old_systime;
  uint32_t num_sync = 1;
  
  if(net_time_from_last_sync > sync_interval+sync_interval/2){
    // Some sync events have been missed, estimate how many
    num_sync = net_time_from_last_sync/sync_interval;
    if(num_sync > 20){
      /* Too many missed sync events.
         TODO: what to do? Local virtual clock may have been diverged too much.
         Device should request an update of the network clock.
         For the moment set netclock as unreliable. It will be set to reliable
         after a call to set_vclock_from_master. */
      time_offset_set = FALSE;
    }
    
    /* Evaluate if the rest the devision net_time_from_last_sync/sync_interval is greater
      than sync_interval/2  */
    if((net_time_from_last_sync - (num_sync * sync_interval)) > sync_interval/2){
      // Round to the next multiple of sync_interval
      num_sync++;
    }
    
  }
  
  last_saved_event_index = (last_saved_event_index+1)%NUM_SYNC_EVENTS;
  
  sync_evts[last_saved_event_index].sys_time = sys_time;
  sync_evts[last_saved_event_index].event_counter = event_counter;
  sync_evts[last_saved_event_index].net_time = old_vtime + num_sync*sync_interval;
  sync_evts[last_saved_event_index].valid = TRUE;
  
#if ADJUST_NETWORK_TIMER
  /* Checking if there are running timers whose expyring time needs to be updated. */
  if(timer_handle.active && HAL_VTIMER_GetCurrentSysTime() + SYSTIME_MARGIN_FOR_TIMER <  timer_handle.expiryTime)
  {    
    /* Try to stop timer */
    HAL_VTIMER_StopTimer(&timer_handle);
    NETCLOCK_StartTimer(net_expiry_time, timer_handle.callback);
  }
#endif
}

uint8_t NETCLOCK_SetNetTimeFromMaster(uint16_t event_counter, uint64_t net_time)
{
  // Search for the sync event with the given event counter
  for(uint8_t i = 0; i < NUM_SYNC_EVENTS; i++){
    if(sync_evts[i].valid && sync_evts[i].event_counter == event_counter){
      // Sync event found!
      // Offset between the local clock and the master clock
      int64_t time_offset = net_time - sync_evts[i].net_time;
      
      // Change value of latest network time in sync event
      sync_evts[last_saved_event_index].net_time += time_offset;
      
      time_offset_set = TRUE;
      return 0;
    }
  }
  
  return 1;    
}

uint8_t NETCLOCK_GetCurrentNetTime(uint64_t *net_time)
{
  uint32_t from_last_sync_evt; // uint32_t should be enough, since it is only a difference
  uint8_t synchronized = TRUE;

  if(!time_offset_set){
    // Clock is not yet synchronized
    synchronized = FALSE;
  }
  
  //PRINTF("Saved: %d %d\n", sync_evts[last_saved_event_index].sys_time, sync_evts[last_saved_event_index].virtual_time);
  from_last_sync_evt = HAL_VTIMER_GetCurrentSysTime()-sync_evts[last_saved_event_index].sys_time;
  *net_time = from_last_sync_evt + sync_evts[last_saved_event_index].net_time;
  
  return synchronized;    
}

uint8_t NETCLOCK_GetTimeFromLastSync(uint64_t *net_time)
{
  if(!time_offset_set){
    // Clock is not yet synchronized
    return FALSE;
  }
  
  *net_time = HAL_VTIMER_GetCurrentSysTime()-sync_evts[last_saved_event_index].sys_time;
  
  return TRUE;  
}

int NETCLOCK_StartTimer(uint64_t net_time, VTIMER_CallbackType cb)
{
  uint32_t net_time_from_last_sync;
  uint64_t systime;
  
  net_expiry_time = net_time;
  
  net_time_from_last_sync = net_time - sync_evts[last_saved_event_index].net_time; // 32 bits should be enough, since last sync events should be quite near in time.
  
  systime = sync_evts[last_saved_event_index].sys_time + net_time_from_last_sync;
  
  PRINTF("net_time %llu, last net_time %llu, last systime %llu, systime %llu (0x%016llX)\n", net_time, sync_evts[last_saved_event_index].net_time, sync_evts[last_saved_event_index].sys_time, systime, systime);
  
  timer_handle.callback = cb;
    
  return HAL_VTIMER_StartTimerSysTime(&timer_handle, systime);
}



