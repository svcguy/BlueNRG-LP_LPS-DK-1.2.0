/**
******************************************************************************
* @file    rf_device_it.c
* @author  AMS RF Application Team
* @version V1.0.0
* @date    27-March-2019
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and
*          peripherals interrupt service routine.
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
* <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "rf_driver_hal_vtimer.h"
#include "hal_miscutil.h"
#include "crash_handler.h"
#include "FreeRTOS.h"
#include "semphr.h"


/** @addtogroup BlueNRG1_StdPeriph_Examples
* @{
*/

/** @addtogroup GPIO_Examples
* @{
*/ 

/** @addtogroup GPIO_IOToggle
* @{
*/ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/
extern SemaphoreHandle_t radioActivitySemaphoreHandle;


/**
  * @brief  This function handles NMI exception.
  */
NOSTACK_FUNCTION(NORETURN_FUNCTION(void NMI_IRQHandler(void)))
{
  HAL_CrashHandler(__get_MSP(), NMI_SIGNATURE);  
  /* Go to infinite loop when NMI exception occurs */
  while (1)
  {}
}

/**
* @brief  This function handles Hard Fault exception.
*/
NOSTACK_FUNCTION(NORETURN_FUNCTION(void HardFault_IRQHandler(void)))
{
  HAL_CrashHandler(__get_MSP(), HARD_FAULT_SIGNATURE);  
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}


/******************************************************************************/
/*                 BLUENRG_LP Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (EXTI), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_bluenrg_lp.s).                                               */
/******************************************************************************/
/**
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/

void USART1_IRQHandler(void)
{  

}


/**
* @brief  This function handles DMA Handler.
*/
void DMA_IRQHandler(void)
{
  
}


/**
* @brief  This function handles GPIO interrupt request.
* @param  None
* @retval None
*/

void GPIOA_IRQHandler(void)
{

  
}

void BLE_WKUP_IRQHandler(void)
{
  HAL_VTIMER_WakeUpCallback();
}

void CPU_WKUP_IRQHandler(void) 
{
  BaseType_t xHigherPriorityTaskWoken;
  
  HAL_VTIMER_TimeoutCallback();
  /* Unblock BTLE_StackTick(). */
  xSemaphoreGiveFromISR( radioActivitySemaphoreHandle, &xHigherPriorityTaskWoken );
  /* A context switch should be performed if xHigherPriorityTaskWoken is equal to pdTRUE. */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
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
  BaseType_t xHigherPriorityTaskWoken;
   
  uint32_t blue_status = BLUE->STATUSREG;
  uint32_t blue_interrupt = BLUE->INTERRUPT1REG;

  /** clear all pending interrupts */
  BLUE->INTERRUPT1REG = blue_interrupt;

  HAL_VTIMER_EndOfRadioActivityIsr();
  BLE_STACK_RadioHandler(blue_status|blue_interrupt);
  HAL_VTIMER_RadioTimerIsr();
    
  /* Unblock BTLE_StackTick(). */
  xSemaphoreGiveFromISR( radioActivitySemaphoreHandle, &xHigherPriorityTaskWoken );
  /* A context switch should be performed if xHigherPriorityTaskWoken is equal to pdTRUE. */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

  /* If the device is configured with 
     System clock = 64 MHz and BLE clock = 16 MHz
     a register read is necessary to end fine  
     the clear interrupt register operation,
     due the AHB down converter latency */ 
  blue_interrupt = BLUE->INTERRUPT1REG;
}

void BLE_RXTX_SEQ_IRQHandler(void)
{
  HAL_RXTX_SEQ_IRQHandler();
}

#ifdef JUST_AN_EXAMPLE_ISR

void Dummy_IRQHandler(void)
{
long lHigherPriorityTaskWoken = pdFALSE;

	/* Clear the interrupt if necessary. */
	Dummy_ClearITPendingBit();

	/* This interrupt does nothing more than demonstrate how to synchronise a
	task with an interrupt.  A semaphore is used for this purpose.  Note
	lHigherPriorityTaskWoken is initialised to zero. Only FreeRTOS API functions
	that end in "FromISR" can be called from an ISR. */
	xSemaphoreGiveFromISR( xTestSemaphore, &lHigherPriorityTaskWoken );

	/* If there was a task that was blocked on the semaphore, and giving the
	semaphore caused the task to unblock, and the unblocked task has a priority
	higher than the current Running state task (the task that this interrupt
	interrupted), then lHigherPriorityTaskWoken will have been set to pdTRUE
	internally within xSemaphoreGiveFromISR().  Passing pdTRUE into the
	portEND_SWITCHING_ISR() macro will result in a context switch being pended to
	ensure this interrupt returns directly to the unblocked, higher priority,
	task.  Passing pdFALSE into portEND_SWITCHING_ISR() has no effect. */
	portEND_SWITCHING_ISR( lHigherPriorityTaskWoken );
}

#endif /* JUST_AN_EXAMPLE_ISR */

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
