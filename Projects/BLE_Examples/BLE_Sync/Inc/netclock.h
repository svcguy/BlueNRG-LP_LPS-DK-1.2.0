/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
* File Name          : netclock.h
* Author             : AMS - RF  Application team
* Description        : This library implements a network clock that can be
*                      synchronized with another clock through sync events.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include <stdint.h>
#include <rf_driver_hal_vtimer.h>

#define NETCLOCK_SECONDS (409600) // (1000000/625*256)

/**
* @brief  Initialize the Network Clock.
*
*         Call this function to initialize the Netowrk Clock library.
*
* @param  None
*
* @retval None
*/
void NETCLOCK_Init(void);

/**
* @brief  Set the connection interval on slave.
*
*         Call this function when the connection has been established.
*         NOTE: Only available on slave.
*
* @param  connection_interval The connection interval of the current connection, in system time units.
*
* @retval None
*/
void NETCLOCK_SetSyncInterval(uint32_t interval);

/**
* @brief  Store a sync event (on master side)
*
*         Function to be called when a sync event is received, in order to update the
*         value of the virtual clock. To be used on the master. The sync events has to be 
*         received at a time interval equal to the connection interval.
*
* @param  sys_time Curent value of the system timer (in system time units)
* @param  slave_event_counter The connection event counter
*
* @retval None
*/
void NETCLOCK_SaveSyncEventOnMaster(uint64_t sys_time, uint16_t slave_event_counter);

/**
* @brief  Store the first sync event (on master side)
*
*         Function to be called when the first sync event is received, in order to link
*         the value of the virtual clock to the first sync event.
*         To be used on the master.
*
* @param  sys_time Current value of the system timer (in system time units)
*
* @retval None
*/
void NETCLOCK_SaveFirstSyncEventOnMaster(uint64_t sys_time);

/**
* @brief  Store a sync event (on slave side)
*
*         Function to be called when a sync event is received, in order to update the
*         value of the virtual clock. To be used on the slave.
*
* @param  sys_time Curent value of the system timer (in system time units)
* @param  event_counter The connection event counter for the current connection
* @retval None
*/
void NETCLOCK_SaveSyncEventOnSlave(uint64_t sys_time, uint16_t event_counter);

/**
* @brief  Pass the information about the master network clock to the library (on slave side)
*
*         Function to be called as soon as the value of the network clock on the master and the
*         related connection event counter has been received. It adjusts the value of the network clock
*         in order ot be synchronized with the master clock without offset.
*         WARNING: this function cannot be called while SaveSyncEventOnMaster()
*         or SaveSyncEventOnSlave() are called, therefore interrupts from which these
*         functions are executed must be temporarily disabled.
*
* @param  event_counter Event counter on the master when the value of the network clock has been captured.
* @param  vtime Value of the network clock on the master at the given connection event counter
*
* @retval 0: the network clock has been adjusted with the correct offset. If the sync event with the
*         passed event counter has not been found among the stored records, 1 is returned.
*/
uint8_t NETCLOCK_SetNetTimeFromMaster(uint16_t event_counter, uint64_t vtime);

/**
* @brief  Get the value of the virtual clock at the last sync event (master side).
*
*         Function to retrieve the value of the virtual clock (only on the master)
*         and the related value of the connection event counter for the given slave.
*         compared to another one in input.
*         WARNING: this function cannot be called while SaveSyncEventOnMaster()
*         or SaveSyncEventOnSlave() are called, therefore interrupts from which these
*         functions are executed must be temporarily disabled.
*
* @param[out]  vtime The value of the virtual clock at the last sync event
* @param[out]  event_counter The value of the event counter for the salve specified by 'slave' parameter.
* @retval None
*/
void NETCLOCK_GetNetTimeOnLastSync(uint64_t *vtime, uint16_t *event_counter);

/**
* @brief  Get the time passed from last sync events.
*
*         This function may be useful to determine if the slave virtual clock may have deviated too much from
*         the virtual clock on the master. The less the value is, the more accurate the virtual clock is.
*         Deviation depends on the accuracy of the sleep clocks on both master and slave.
*         WARNING: this function cannot be called while SaveSyncEventOnMaster()
*         or SaveSyncEventOnSlave() are called, therefore interrupts from which these
*         functions are executed must be temporarily disabled.
*
* @param[out]  vtime The virtual clock time passed from last sync event
*
* @retval None
*/
uint8_t GetTimeFromLastSync(uint64_t *vtime);

/**
* @brief  Get the current value of the network clock
*
*         The function returns the current value of the network clock. This
*         clock is synchonized using the sync events as a reference. 
*         WARNING: this function cannot be called while SaveSyncEventOnMaster()
*         or SaveSyncEventOnSlave() are called, therefore interrupts from which these
*         functions are executed must be temporarily disabled.
*
* @param[out]  vtime The current value of the network clock (STU)
*                    The less time has passed from last sync event, the more accurate the
*                    value of the network clock is (compared to the master's one).
*
* @retval Returns if the value of the network clock is synchronized (with offset 0) with master network clock
*         (1: TRUE, 0: FALSE).
*/
uint8_t NETCLOCK_GetCurrentNetTime(uint64_t *vtime);

/**
* @brief  Start a timer synchronized with the network clock
*
*         The functions starts a timer which will expire at the given network time.
*         A callback is called when timer expires.
*         On master side the network clock is equal to the master's own system clock, so
*         HAL_VTIMER_StartTimerSysTime can be used instead.
*
* @param  net_time The value of the network clock when the timer will expire.
* @param  cb    The function that will ba called when the timer expires.
*
* @retval Returns 0 if thevtimer has been started succesfully, 1 otherwise.
*/
int NETCLOCK_StartTimer(uint64_t net_time, VTIMER_CallbackType cb);

/**
* @brief  Convert a 32-bit sysTime in a 64-bit sysTime.
*
*         The conversion assumes that the 32-bit sysTime is no more than around 87
*         minutes in the future or in the past compared to current time.
*         Use NETCLOCK_convertPastSysTime32ToSysTime64() if it is sure that sysTime
*         is in the past.
*
* @param  sysTime Curent value of the system timer, in system time units (625/256 microseconds).
*
* @retval 64-bit system time (units of 625/256 microseconds)
*/
uint64_t NETCLOCK_convertSysTime32ToSysTime64(uint32_t sysTime);

/**
* @brief  Convert a 32-bit past sysTime in a 64-bit sysTime.
*
*         The conversion assumes that the 32-bit sysTime is no more than around 174
*         minutes in the past compared to current time. If sysTime is in the future,
*         use NETCLOCK_convertSysTime32ToSysTime64().
*
* @param  sysTime Curent value of the system timer, in system time units (625/256 microseconds).
*
* @retval 64-bit system time (units of 625/256 microseconds)
*/
uint64_t NETCLOCK_convertPastSysTime32ToSysTime64(uint32_t sysTime);
