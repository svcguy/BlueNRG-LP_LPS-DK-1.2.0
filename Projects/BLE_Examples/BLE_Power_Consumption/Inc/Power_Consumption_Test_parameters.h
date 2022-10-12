/**
  ******************************************************************************
  * @file    Power_Consumption_Test_parameters.h
  * @author  AMS - RF Application team
  * @date    28 August 2020
  * @brief   Header file for low power configuration of power consumption demo 
  *          application. It allows to define the best configuration options
  *          in order to improve the power consumption performances.
  *          WARNING: Please review all the highlighted comments and possible 
  *          impact on setting some specific values. 
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT STMicroelectronics</center></h2>
  ******************************************************************************
  */
#ifndef _POWER_CONSUMPTION_TEST_PARAMETERS_H_
#define _POWER_CONSUMPTION_TEST_PARAMETERS_H_

#include "system_BlueNRG_LP.h" 

/* ********************* System and Bluetooth LE clocks configuration ***************/ 

/* System initialization function. Set the clocks combination with the lowest 
   power consumption:
   Sys_CLK = SYSCLK_DIRECT_HSE (system clock frequency is 32 MHz DIRECT HSE with
             HSI OFF);
   BLE_CLK = BLE_SYSCLK_16M (Bluetooth LE system clock frequency is 16 MHz).

   NOTE: This configuration is the lowest power consumption, since the HSI clock 
         is disabled during the ACTIVE mode  and the PLL is off. 
*/
    
/* System clock: direct HSE */
#define POWER_CONSUMPTION_DEMO_SYSTEM_CLOCK (SYSCLK_DIRECT_HSE)
/* Bluetoooth LE clock: 16MHz */
#define POWER_CONSUMPTION_DEMO_BLE_CLOCK    (BLE_SYSCLK_16M)

/* ********************* HSE Startup time *******************************************/
    
/* The High Speed Cystal start up time depends on temperature and voltage levels.
   The recommended value 780 us (320) is covering all the possible conditions 
   in terms of temperatures variations and voltage variations [1.7V - 3.3V]. 

   WARNING: User can potentially reduce this value in order to improve 
            power consumption performances for this specific demo case.

*/
#define HS_STARTUP_TIME 210 // [320: 780us];   [210: 512 us]

/* ********************* LSE oscillator drive capability ****************************/    
    
/* LSE oscillator drive capability is set on system_BlueNRG_LP.c.
   LSConfigLSE() function using the LL_RCC_LSE_SetDriveCapability API.

   The recommended value is high drive value (LL_RCC_LSEDRIVE_HIGH):
	LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_HIGH).

   WARNING: User can potentially reduce this value from high to low 
            (LL_RCC_LSEDRIVE_MEDIUMHIGH, LL_RCC_LSEDRIVE_MEDIUMLOW, LL_RCC_LSEDRIVE_LOW) 
            in order to improve power consumption values, but this change is not recommended
            since it could severely impact the proper device functionality.
*/
    
#endif // _POWER_CONSUMPTION_TEST_PARAMETERS_H_
