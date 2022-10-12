/**
  ******************************************************************************
  * @file    Examples/RADIO/RADIO_Sniffer/Src/rf_device_it.c
  * @author  RF Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "rf_driver_ll_tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
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
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/

void NMI_IRQHandler(void)
{
  /* Go to infinite loop when NMI exception occurs */
  while (1)
  {}
}

void HardFault_IRQHandler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

void SVC_IRQHandler(void)
{
}

void SysTick_IRQHandler(void)
{ 
}

void USART1_Handler(void)
{
  BSP_COM_IRQHandler();
}
/******************************************************************************/
/*                 BlueNRG-LP Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (system_bluenrgLP.c).                                               */
/******************************************************************************/

uint32_t timer_reload = 0;
/**
  * @brief This function handles TIMx update interrupt and TIMx6 global interrupt.
  */
void TIMx_IRQHandler(void)
{
  /* Check whether update interrupt is pending */
  if(LL_TIM_IsActiveFlag_UPDATE(TIMx) == 1)
  {
    /* Clear the update interrupt flag */
    LL_TIM_ClearFlag_UPDATE(TIMx);
    
    timer_reload++;
  }
}

void BLE_WKUP_IRQHandler(void)
{
  HAL_VTIMER_WakeUpCallback();
}

void CPU_WKUP_IRQHandler(void) 
{
  HAL_VTIMER_TimeoutCallback();
}

void BLE_ERROR_IRQHandler(void)
{
  volatile uint32_t debug_cmd;
  
  BLUE->DEBUGCMDREG |= 1;

  /* If the device is configured with 
     System clock = 64 MHz and BLE clock = 16 MHz
     a register read is necessary to end fine  
     the clear interrupt register operation,
     due the AHB down converter latency */ 
  debug_cmd = BLUE->DEBUGCMDREG;
}

void BLE_TX_RX_IRQHandler(void)
{
  RADIO_IRQHandler();
  HAL_VTIMER_RadioTimerIsr();
}
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
